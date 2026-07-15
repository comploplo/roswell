# roscmp-dds

M4 transport: a thin bridge from roscmp's generated bindings to **real RTPS**.

Decision (M4): **borrow [RustDDS](https://crates.io/crates/rustdds)** (pure-Rust
RTPS — discovery, QoS, reliability) for transport, but keep **our** generated
CDR serializer (M2) for the payload via a custom `SerializerAdapter`. This keeps
the single-serializer thesis intact while not reimplementing the RTPS protocol.
The transport trait will be *extracted* once a second backend (own-RTPS) exists.

## What works (all verified against vanilla ROS2)

`src/codec.rs` plugs roscmp's CDR into RustDDS via `Set`/`De` adapters
(`to_cdr()[4..]` out, header rebuilt on the way in) plus ROS2 naming
(`/chatter` → `rt/chatter`, type `std_msgs::msg::dds_::String_`). Four bins:

| bin | does | vanilla counterpart |
|-----|------|---------------------|
| `talker` | publish `std_msgs/String` on `/chatter` | `ros2 topic echo` |
| `listener` | subscribe `std_msgs/String` on `/chatter` | `ros2 topic pub` |
| `teleop` | publish `geometry_msgs/Twist` on `/cmd_vel` | `ros2 topic echo` |
| `add_server` | serve `example_interfaces/AddTwoInts` | `ros2 service call` |
| `add_client` | call a vanilla `AddTwoInts` service | ROS2 service server |
| `runtime_contracts` | publish `/clock`, `/rosout`, `/tf`, `/tf_static`; serve lifecycle/type-description services | ROS2 topic/service tools |
| `fibonacci_action_server` | serve `example_interfaces/Fibonacci` action goals | `ros2 action send_goal` |
| `fibonacci_action_client` | send a `Fibonacci` goal, stream feedback, print result | ROS2 action server |
| `bag_play` | replay ROS2 CDR samples from MCAP type-blind, with optional `/clock` | ROS2 topic subscribers |
| `bag_record` | record topics (explicit or `--all` discovered) to MCAP, lz4-chunked | ROS2 topic publishers |
| `turtle_teleop` | TUI: drive turtlesim (`Twist`→cmd_vel, read `Pose`) | `turtlesim_node` |

The service correlates replies to requests via the RTPS **sample identity**
(`SampleInfo::sample_identity` → `WriteOptions::related_sample_identity`), so a
stock `ros2 service call` gets its `AddTwoInts_Response(sum=7)`.

## Node runtime

`node.rs` is a single-threaded ROS-like node: callback subscriptions, in-loop
services, timers, and a unified `spin`. Nodes participate in the ROS graph
(`ros_discovery_info`, so they appear in `ros2 node list`), parse standard
`--ros-args` (`-r` remaps incl. `__node`/`__ns`, `-p`, `--params-file` YAML),
serve the full parameter services, and honor `use_sim_time` (timers driven by
`/clock`). `parameters.rs` also has a `ParameterClient` for reading/setting a
*remote* node's parameters. Known gap: our discovery entries carry empty
reader/writer GID lists (RustDDS doesn't expose endpoint GIDs), which is enough
for `ros2 node list` but not per-node endpoint attribution in `ros2 node info`.

## Run the interop proof

Needs Docker (`colima start` on macOS). Builds all bins and runs each against a
vanilla ROS2 node inside one `ros:jazzy` container:

```sh
CONTAINER_ENGINE=podman ./interop_test.sh
# ... ===== SUMMARY: 15 passed, 0 failed =====
```

## MCAP record & replay

`raw.rs` contains a small ROS2 CDR MCAP reader/writer. The reader scans top-level
records and chunks (uncompressed, `zstd`, or `lz4`), preserves schemas/channels,
and yields `RawSample` values so playback does not need generated message types.
The writer batches into chunks (lz4 or uncompressed, 1 MiB threshold) or writes
flat.

```sh
cargo run -p roscmp-dds --bin bag_record -- --all --output run.mcap --duration 30
cargo run -p roscmp-dds --bin bag_record -- --topic /chatter:std_msgs/msg/String --output run.mcap
cargo run -p roscmp-dds --bin bag_play -- run.mcap --clock --speed 1.0
cargo run -p roscmp-dds --bin bag_play -- run.mcap --topic /scan --no-clock
```

This is intentionally **MCAP-native replay**, not full `rosbag2` parity. It does
not yet handle SQLite `.db3` bags, rosbag2
`metadata.yaml`, QoS profile restoration, or bag splitting.

## Graph-Aware Tunnel Foundation

`tunnel.rs` defines the ROS-facing behavior for a future encrypted graph tunnel:
per-channel reliability, priority, deadlines, watchdogs, bounded queues, and a
small length-prefixed frame format for raw CDR topic samples. The robot default
policy keeps control channels high-priority and latest-reliable while allowing
camera/pointcloud-style channels to drop stale samples under congestion.

This is deliberately above the packet layer. WireGuard, TCP, QUIC, or another
encrypted pipe can carry these frames, but ROS semantics stay explicit here.

The first concrete carrier is a plain TCP topic bridge:

```sh
# Robot side: receive commands, send diagnostics.
cargo run -p roscmp-dds --bin tcp_topic_bridge -- \
  serve 0.0.0.0:7447 \
  --rx /cmd_vel:geometry_msgs/msg/Twist \
  --tx /diagnostics:diagnostic_msgs/msg/DiagnosticArray

# Operator side: send commands, receive diagnostics.
cargo run -p roscmp-dds --bin tcp_topic_bridge -- \
  connect robot.local:7447 \
  --tx /cmd_vel:geometry_msgs/msg/Twist \
  --rx /diagnostics:diagnostic_msgs/msg/DiagnosticArray
```

Routes are explicit and directional to avoid echo loops: `--tx` means local ROS
DDS to tunnel, `--rx` means tunnel to local ROS DDS.

The same frame codec is intended to work over USB CDC/serial links for embedded
Rust devices. A microcontroller does not need DDS; it can implement the tunnel
frames for selected topics, let the host bridge republish them into ROS, and use
`LatestReliable`/watchdog policies for command-like channels while allowing
bulk telemetry to drop stale samples under congestion.

## Drive turtlesim (M5)

Interactive — run vanilla `turtlesim_node`, then in another terminal:

```sh
cargo run -p roscmp-dds --bin turtle_teleop      # W/S/A/D to drive, Q to quit
```

Headless proof (Docker) — drives turtlesim under xvfb and checks the pose moved:

```sh
./turtle_sim_test.sh
# INIT  x=5.544 ...  /  FINAL x=9.608 ... theta=-2.741  /  MOVED=true
```

## Next

Bags now capture each channel's QoS (`offered_qos_profiles` channel metadata,
compact `key=value` form) and `bag_play` restores it (`--no-qos-restore` to
opt out). Remaining gaps: rosbag2 `.db3`/`metadata.yaml` replay, advertising
RIHS01 type hashes in discovery and per-node endpoint GIDs (both blocked on
RustDDS), XCDR2, and wheel packaging for the Python client (`python/`).

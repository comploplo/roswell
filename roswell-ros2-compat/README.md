# roswell-ros2-compat

`roswell-ros2-compat` is the ROS 2 compatibility runtime for Roswell.

It lets Roswell-generated and runtime-loaded messages participate in a normal
ROS 2 graph: publish/subscribe, services, actions, parameters, `/clock`, `/tf`,
bags, and bridge tools. Under the hood it uses RustDDS for RTPS/DDS interop and
Roswell's own CDR serializer for message payloads.

This crate is named for what it provides to users: compatibility with existing
ROS 2 systems. DDS is an implementation detail unless you are working on the
transport boundary.

## What It Provides

- ROS 2 topic naming and type naming (`/chatter` -> `rt/chatter`)
- CDR adapter glue between Roswell messages and RustDDS readers/writers
- QoS presets for common ROS 2 profiles: default, sensor data, and latched
- typed publishers and subscribers
- service clients and servers with ROS 2 request/reply correlation
- action clients and servers
- a single-threaded node runtime with subscriptions, services, timers, remaps,
  parameters, and `use_sim_time`
- graph discovery helpers
- `/rosout`, diagnostics, lifecycle, type-description, `/tf`, and `/tf_static`
- MCAP record/replay for ROS 2 CDR samples
- TCP and USB topic bridges for selected raw CDR topics

## Tools

| tool | purpose |
|---|---|
| `talker` | publish `std_msgs/String` on `/chatter` |
| `listener` | subscribe to `std_msgs/String` on `/chatter` |
| `teleop` | publish `geometry_msgs/Twist` on `/cmd_vel` |
| `add_server` | serve `example_interfaces/AddTwoInts` |
| `add_client` | call an `AddTwoInts` service |
| `runtime_contracts` | exercise `/clock`, `/rosout`, `/tf`, lifecycle, parameters, and type-description services |
| `fibonacci_action_server` | serve `example_interfaces/Fibonacci` action goals |
| `fibonacci_action_client` | send a `Fibonacci` goal and stream feedback |
| `bag_record` | record explicit or discovered topics to MCAP |
| `bag_play` | replay MCAP samples, optionally publishing `/clock` |
| `tcp_topic_bridge` | forward selected topics over a TCP link |
| `usb_topic_bridge` | forward selected topics over USB CDC / serial |
| `turtle_teleop` | drive `turtlesim` from a terminal |

## Interop Test

The interop script builds the tools and runs them against ordinary ROS 2 Jazzy
nodes inside a container.

```sh
CONTAINER_ENGINE=podman ./interop_test.sh
# ... ===== SUMMARY: 15 passed, 0 failed =====
```

On macOS, start your container runtime first, for example `colima start`.

## MCAP Record And Replay

The MCAP tools are type-blind: schemas and channels are stored in the file, so
playback does not need generated message types.

```sh
cargo run -p roswell-ros2-compat --bin bag_record -- --all --output run.mcap --duration 30
cargo run -p roswell-ros2-compat --bin bag_record -- --topic /chatter:std_msgs/msg/String --output run.mcap
cargo run -p roswell-ros2-compat --bin bag_play -- run.mcap --clock --speed 1.0
cargo run -p roswell-ros2-compat --bin bag_play -- run.mcap --topic /scan --no-clock
```

Bags preserve schemas, channels, compression (`zstd`/`lz4`/none), and compact
QoS metadata. This is MCAP-native tooling, not full rosbag2 parity: SQLite
`.db3` bags, rosbag2 `metadata.yaml`, and bag splitting are outside the current
scope.

## Topic Bridges

The tunnel code carries raw ROS CDR samples over an explicit route list. It is
useful for constrained links where you want selected topics, not a full graph.

```sh
# Robot side: receive commands, send diagnostics.
cargo run -p roswell-ros2-compat --bin tcp_topic_bridge -- \
  serve 0.0.0.0:7447 \
  --rx /cmd_vel:geometry_msgs/msg/Twist \
  --tx /diagnostics:diagnostic_msgs/msg/DiagnosticArray

# Operator side: send commands, receive diagnostics.
cargo run -p roswell-ros2-compat --bin tcp_topic_bridge -- \
  connect robot.local:7447 \
  --tx /cmd_vel:geometry_msgs/msg/Twist \
  --rx /diagnostics:diagnostic_msgs/msg/DiagnosticArray
```

Routes are directional: `--tx` sends local ROS traffic into the bridge, and
`--rx` publishes bridge traffic into local ROS.

The same frame format is shared with `roswell-tunnel-core`, so no_std firmware
can send selected topics over UART or USB CDC to a host bridge.

## Turtlesim Demo

Run `turtlesim_node`, then:

```sh
cargo run -p roswell-ros2-compat --bin turtle_teleop
```

There is also a headless container proof:

```sh
./turtle_sim_test.sh
# INIT  x=5.544 ...  /  FINAL x=9.608 ... theta=-2.741  /  MOVED=true
```

## Current Limits

- Per-node endpoint attribution in `ros2 node info` is limited because RustDDS
  does not expose every endpoint GID Roswell would need to advertise.
- RIHS01 type hashes are generated, but not yet advertised through discovery.
- XCDR2 support is present in parts of the codec/codegen surface, but ROS 2
  compatibility is still centered on XCDR1.
- DDS remains the compatibility path for ordinary ROS 2 peers and remote hosts.

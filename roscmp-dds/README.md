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
| `turtle_teleop` | TUI: drive turtlesim (`Twist`→cmd_vel, read `Pose`) | `turtlesim_node` |

The service correlates replies to requests via the RTPS **sample identity**
(`SampleInfo::sample_identity` → `WriteOptions::related_sample_identity`), so a
stock `ros2 service call` gets its `AddTwoInts_Response(sum=7)`.

## Run the interop proof

Needs Docker (`colima start` on macOS). Builds all bins and runs each against a
vanilla ROS2 node inside one `ros:jazzy` container:

```sh
./interop_test.sh
# ... ===== SUMMARY: 5 passed, 0 failed =====
```

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

## Next (M4.4)

Extract a transport trait and generalize the per-type `impl_cdr!` glue so an
own-RTPS backend can slot in behind the same interface.

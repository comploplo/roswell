# roswell-build

Generate roswell Rust message bindings from `build.rs`, the way `prost-build` /
`tonic-build` do for protobuf: point it at ROS interface search roots, name the
interfaces you need, and `include!` the generated file.

The output is a single `roswell_msgs.rs` containing `#[repr(C)]` structs, the
embedded CDR runtime, `to_cdr`/`from_cdr` per message, and
`roswell_ros2_compat::codec::CdrMsg` (+ action-trait) impls so the types ride RTPS
directly via `roswell-ros2-compat` publishers, subscribers, services, and actions.

## Usage

`Cargo.toml`:

```toml
[dependencies]
roswell-ros2-compat = { path = "../roswell/roswell-ros2-compat" }

[build-dependencies]
roswell-build = { path = "../roswell/roswell-build" }
```

`build.rs`:

```rust
fn main() {
    roswell_build::Config::new()
        // Plain package trees and ament install prefixes both work:
        .type_paths(["msgs", "/opt/ros/jazzy"])
        .compile([
            "geometry_msgs/msg/Twist",
            "example_interfaces/srv/AddTwoInts",
            "example_interfaces/action/Fibonacci",
        ])
        .unwrap();
}
```

`src/main.rs`:

```rust
#[allow(non_camel_case_types, non_upper_case_globals, dead_code, clippy::all, clippy::pedantic)]
mod msgs {
    include!(concat!(env!("OUT_DIR"), "/roswell_msgs.rs"));
}

use msgs::{geometry_msgs__Twist, geometry_msgs__Vector3};
use roswell_ros2_compat::transport::{Dds, MsgPublisher, Qos, Transport};

fn main() {
    let dds = Dds::new(0);
    let cmd_vel = dds.publisher::<geometry_msgs__Twist>("/cmd_vel", Qos::Default);
    cmd_vel.publish(geometry_msgs__Twist {
        linear: geometry_msgs__Vector3 { x: 0.5, y: 0.0, z: 0.0 },
        angular: geometry_msgs__Vector3 { x: 0.0, y: 0.0, z: 0.2 },
    });
}
```

Notes:

- References take `pkg/msg/Name`, `pkg/srv/Name`, or `pkg/action/Name` (a
  trailing `.msg`/`.srv`/`.action` is stripped); `srv`/`action` expand into
  their request/response/feedback wire messages, and every transitively
  referenced `.msg` is resolved from the same roots.
- Search roots are probed both as `<root>/<pkg>/msg/...` (colcon `src/`
  checkout) and `<root>/share/<pkg>/msg/...` (ament prefix such as
  `/opt/ros/jazzy` — pass entries of `$AMENT_PREFIX_PATH` if you want the
  installed interfaces).
- `cargo:rerun-if-changed` is emitted per search root, so edits to interface
  files re-run codegen.
- The `#[allow(...)]` on the wrapping module replaces the generated file's
  former inner attribute (inner attributes cannot cross `include!`).

The end-to-end test for this crate (`tests/build_integration.rs`) compiles and
runs `tests/fixture`, a real out-of-workspace crate using exactly this flow.

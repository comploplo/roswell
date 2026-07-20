# Embedded Python And No-Std Rust On ROS 2 Topics

This demo shows Roswell's shared message model reaching a small Python edge
node and a `no_std` Rust firmware node. They exchange a ROS 2 message without a
ROS installation in either process and without duplicating the wire protocol.

- **Python edge node:** the shipped `roswell` wheel, using `ctypes` over the
  Roswell runtime. It publishes `geometry_msgs/msg/Twist` on `/cmd_vel` over
  RTPS/DDS.
- **Embedded Rust node:** the `roswell-hil-fw` `--features uart` firmware:
  `no_std`, cross-compiled to `thumbv6m-none-eabi`, running on a simulated
  Cortex-M0 + PL011 UART inside [Renode](https://renode.io) (driven by `hilt`).
- **Host bridge:** the Roswell runtime's DDS-to-tunnel path. The same
  `roswell-ros2-compat` crate that builds `tcp_topic_bridge` and
  `usb_topic_bridge` subscribes to `/cmd_vel` and forwards each sample to the
  firmware over the `hilt`-bridged UART.

```text
wheel_node.py
  -> publishes geometry_msgs/msg/Twist over RTPS/DDS
  -> roswell-ros2-compat subscribes and creates a TopicSample frame
  -> the tunnel carries the frame over UART
  -> the Cortex-M0 firmware decodes it and returns an acknowledgement
```

## Run it

```sh
embedded/demo/run_demo.sh
# or:
cd hil-renode
cargo test -p roswell-hil --test wheel_firmware_demo -- --ignored --nocapture
```

This requires the `roswell` wheel in `python/.venv-test` (see
[`python/README.md`](../python/README.md)), the
Renode image (`hilt` auto-builds `localhost/hilt-renode:arm64` on Apple Silicon),
and `cargo-binutils`. A typical run takes about one minute.

## Expected Result

```text
wheel_node: publishing geometry_msgs/msg/Twist on /cmd_vel
wheel_node: published 10 Twist sample(s)
firmware peer="roswell-mcu" acknowledged /cmd_vel seq=1
DEMO OK: Python -> DDS -> UART tunnel -> Cortex-M firmware
test result: ok. 1 passed
```

The Renode TCP port can become visible before the in-container socket terminal
is listening, so an early connection may close. The harness reconnects until
the firmware answers; the Stage-B HIL test covers this startup race.

## Files

- `demo/wheel_node.py`: the Python edge node using the shipped `roswell` wheel.
- `demo/run_demo.sh`: the demo runner.
- `../hil-renode/tests/wheel_firmware_demo.rs`: the demo harness. It boots the
  firmware, runs the wheel node, runs the DDS-to-UART bridge through the Roswell
  runtime, and asserts the firmware's acknowledgement. The harness runs the
  bridge in process because the standalone bridge handles acknowledgements
  internally.
- `../hil-renode/tests/raw_dds_probe.rs`: a diagnostic showing that the raw DDS
  endpoints the bridge uses (`RawDdsPublisher` and `RawDdsSubscriber`) receive
  both raw-to-raw and typed-to-raw traffic.

## Optional ROS 2 Check

On a native Linux host, a ROS 2 participant sharing the DDS domain can observe
the message with:

```sh
ros2 topic echo /cmd_vel geometry_msgs/msg/Twist
```

Containerized ROS 2 on macOS commonly runs on a separate virtual network where
DDS multicast does not reach host participants. The interop test avoids that
network boundary by running the Roswell binaries inside the ROS container.

## MCU To ROS 2 Topic Without Micro-ROS

The following process puts a custom message on a ROS 2 topic from a bare-metal
MCU using components in this repository. Each step names its corresponding
test.

### 1. Generate heapless (`no_std`) message bindings

```sh
cargo run -- --lang rust --no-std --out gen/ \
    samples/geometry_msgs/msg/Vector3.msg samples/geometry_msgs/msg/Twist.msg
# writes gen/roswell_msgs_nostd.rs
```

The output is one self-contained, `core`-only Rust file: no `std`, no allocator,
and no heap. Strings and sequences are fixed-capacity (`BoundedString<N>` and
`BoundedVec<T, N>`); declared `.msg` bounds (`string<=N`, `T[<=N]`) are used
directly, unbounded fields get defaults of 64 bytes / 16 elements, overridable
with `--string-cap N` / `--seq-cap N`. Encode/decode work over caller-provided
`&mut [u8]` buffers and return `Result`. A value that exceeds its configured
capacity or an output buffer that is too small returns an error instead of
silently truncating data.

`tests/nostd_codegen_tests.rs` verifies that values within the configured
capacity produce the same bytes as the standard generated encoder across
XCDR1 LE/BE and XCDR2, for `std_msgs/String`, `geometry_msgs/Twist`,
`sensor_msgs/Imu`, and a sequence-bearing type; overflow paths return errors.

### 2. Encode on the MCU

Drop the generated file into the firmware crate and encode into a stack buffer
(this repo's firmware does exactly this in `hil-renode/fw/src/uart.rs`,
with the generated `hil-renode/fw/src/msgs_nostd.rs`):

```rust
mod msgs_nostd;
use msgs_nostd::{geometry_msgs__Twist, geometry_msgs__Vector3, Endian};

let twist = geometry_msgs__Twist {
    linear: geometry_msgs__Vector3 { x: 1.25, y: -2.5, z: 0.5 },
    angular: geometry_msgs__Vector3 { x: 0.0, y: 0.125, z: -3.75 },
};
let mut cdr = [0u8; 64];
let n = twist.to_cdr(&mut cdr, Endian::Little)?; // real ROS 2 CDR bytes
```

### 3. Frame it as a tunnel `TopicSample` and ship it over the wire

```rust
use roswell_tunnel_core as wire; // no_std, alloc-free
let mut frame = [0u8; 512];
let len = wire::encode_topic_sample(
    &mut frame, seq, "/mcu_twist", "geometry_msgs/msg/Twist", stamp_nanos, &cdr[..n],
)?;
// write frame[..len] to your UART / USB CDC
```

`hil-renode/tests/tunnel_hil.rs::mcu_encodes_real_twist_cdr_over_uart` verifies
steps 2 and 3 on simulated silicon. The Cortex-M0 firmware in Renode encodes
the Twist above with the no_std codegen
output, frames it, and sends it over the PL011 UART; the host decodes the CDR
with the standard generated decoder and asserts every field value.

### 4. Bridge to DDS on the host

The shipped bridge bins consume tunnel `TopicSample` frames from a serial/TCP
carrier and republish them as raw DDS samples (and the reverse direction):

```sh
cargo run -p roswell-ros2-compat --bin usb_topic_bridge -- /dev/ttyACM0 \
    --rx /mcu_twist:geometry_msgs/msg/Twist     # MCU → DDS
```

At that point any DDS participant on the same bus, including `ros2 topic echo`,
can receive the MCU's message. `raw_dds_probe.rs` and the wheel demo exercise
the DDS hop in both directions.

The MCU uses no micro-ROS, RMW, or ROS message headers. Its firmware carries
only the generated `core`-only Rust bindings and the
`roswell-tunnel-core` framing codec.

### Embassy And Async Runtimes

`roswell-tunnel-core` is `#![no_std]`, allocation-free, and contains no blocking
calls, timers, or I/O. Encoders write into caller buffers, and
`parse_payload` borrows from a caller slice, so it is directly usable from
Embassy tasks (or any executor): do your `read_exact`/`write_all` with
whatever async HAL you have and hand the bytes to the codec. The same holds
for the `--no-std` generated message bindings. This repository verifies the
codec with polling PL011 firmware; it does not include an Embassy application.

# `embedded/` — the MicroPython/Pi-class ⇄ Rust-on-silicon ROS2 demo

The killer demo: an **embedded Python node** and an **embedded Rust node**
exchange a real ROS2 message with **zero ROS and zero DDS daemon installed**, and
without a line of duplicated protocol code.

- **Embedded Python node** — the shipped `roscmp` **FFI wheel** (pure `ctypes`
  over the roscmp Rust runtime; the Raspberry-Pi-class story). It publishes a
  `geometry_msgs/msg/Twist` on `/cmd_vel` over real RTPS/DDS. No new Python
  protocol code — this is the client from `python/`, used as-is.
- **Embedded Rust node** — the `roscmp-hil-fw` `--features uart` firmware:
  `no_std`, cross-compiled to `thumbv6m-none-eabi`, running on a simulated
  Cortex-M0 + PL011 UART inside [Renode](https://renode.io) (driven by `hilt`).
- **The bridge** — the roscmp runtime's DDS⇄tunnel machinery (the same
  `roscmp-dds` crate the `tcp_topic_bridge` / `usb_topic_bridge` bins are built
  on) subscribes to `/cmd_vel` and forwards each sample to the firmware over the
  `hilt`-bridged UART.

```
 wheel_node.py  (roscmp FFI wheel, pure ctypes)
      │  publish geometry_msgs/msg/Twist on /cmd_vel
      ▼
   real RTPS/DDS  ──────────────────────────────────────────┐
      │                                                       │  (also visible to
      ▼                                                       │   any ROS2 node —
 roscmp-dds subscriber  (bridge role, roscmp runtime)         │   see the cherry)
      │  encode tunnel TopicSample
      ▼
   tunnel over UART  (hilt UART⇄TCP bridge)
      │
      ▼
 Rust firmware  (no_std, Cortex-M0, Renode)  ── Ack(seq) ──►  asserted ✓
      │  ── Hello("roscmp-mcu") ──►  asserted ✓  (firmware → host direction)
```

## Run it

```sh
embedded/demo/run_demo.sh
# or:
cd hil-renode && cargo test -p roscmp-hil --test wheel_firmware_demo -- --ignored --nocapture
```

Needs the `roscmp` wheel in `python/.venv-test` (see `python/README.md`), the
Renode image (`hilt` auto-builds `localhost/hilt-renode:arm64` on Apple Silicon),
and `cargo-binutils`. ~1 min.

## Verified live (this host)

```
wheel_node: publishing geometry_msgs/msg/Twist on /cmd_vel
wheel_node: published 10 Twist sample(s)
 ...
attempt 8: firmware peer="roscmp-mcu" acked /cmd_vel seq=1 (wheel-published Twist, delivered over real DDS)
DEMO OK: embedded Python wheel node -> real DDS -> tunnel/UART -> embedded Rust firmware (Cortex-M) acked the message. Zero ROS installed.
test result: ok. 1 passed  (105.96s)
```

The first attempts fail-and-retry: the Renode TCP port is published by gvproxy
before the in-container socket terminal is listening, so an early connect EOFs —
the harness reconnects until the firmware answers (the same startup race the
Stage-B HIL test documents).

## Files

- `demo/wheel_node.py` — the embedded Python node (shipped `roscmp` wheel).
- `demo/run_demo.sh` — one-command runner.
- `../hil-renode/tests/wheel_firmware_demo.rs` — the demo harness: boots the
  firmware, runs the wheel node, plays the DDS⇄UART bridge via the roscmp
  runtime, and **asserts** the firmware's ack. In-process (rather than the
  shipped bridge bin) precisely so the ack is observable — the bins consume acks
  internally for reliability.
- `../hil-renode/tests/raw_dds_probe.rs` — a fast diagnostic proving the raw DDS
  endpoints the bridge uses (`RawDdsPublisher`/`RawDdsSubscriber`) receive on
  this host, both `raw→raw` and `typed→raw` (the wheel publishes typed).

## The vanilla-ROS2 cherry (documented skip)

Because the message travels on real RTPS/DDS, a vanilla `ros2 topic echo
/cmd_vel geometry_msgs/msg/Twist` witnesses it wherever it shares the DDS bus.
On a native Linux host that is one extra command. It is **not shown live here**:
the only ROS2 available is the `ros:jazzy` **podman** image, and podman-machine
runs the container in a Linux VM on a separate L2 segment, so SPDP multicast from
the mac host never reaches it (the repo's `interop_test.sh` runs the roscmp bins
*inside* that container for exactly this reason). The reality of the DDS hop is
instead proven directly by `raw_dds_probe.rs` (roscmp typed-pub → raw-sub over
real UDP RTPS) and by the wheel node's own publish landing on the bus in the
demo.

## MCU to ROS2 topic, no micro-ROS

The recipe for putting **your own message** on a ROS2 topic from a bare-metal
MCU, using only pieces that exist in this repo. Every step below is proven by a
test named next to it.

### 1. Generate heapless (`no_std`) message bindings

```sh
cargo run -- --lang rust --no-std --out gen/ \
    samples/geometry_msgs/msg/Vector3.msg samples/geometry_msgs/msg/Twist.msg
# writes gen/roscmp_msgs_nostd.rs
```

The output is one self-contained, `core`-only Rust file: no std, no alloc, no
heap. Strings and sequences are fixed-capacity (`BoundedString<N>` /
`BoundedVec<T, N>`); declared `.msg` bounds (`string<=N`, `T[<=N]`) are used
directly, unbounded fields get defaults of 64 bytes / 16 elements, overridable
with `--string-cap N` / `--seq-cap N`. Encode/decode work over caller-provided
`&mut [u8]` buffers and return `Result` — no panics, and overflow (a value that
does not fit its capacity, or an output buffer that is too small) is an error,
never silent truncation.

**Proven:** `tests/nostd_codegen_tests.rs` — for values that fit, the no_std
encoder's bytes are **byte-identical** to the std generated encoder's, across
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
let n = twist.to_cdr(&mut cdr, Endian::Little)?; // real ROS2 CDR bytes
```

### 3. Frame it as a tunnel `TopicSample` and ship it over the wire

```rust
use roscmp_tunnel_core as wire; // no_std, alloc-free
let mut frame = [0u8; 512];
let len = wire::encode_topic_sample(
    &mut frame, seq, "/mcu_twist", "geometry_msgs/msg/Twist", stamp_nanos, &cdr[..n],
)?;
// write frame[..len] to your UART / USB CDC
```

**Proven (steps 2+3 together, on simulated silicon):**
`hil-renode/tests/tunnel_hil.rs::mcu_encodes_real_twist_cdr_over_uart` — the
Cortex-M0 firmware in Renode encodes the Twist above with the no_std codegen
output, frames it, and sends it over the PL011 UART; the host decodes the CDR
with the **std** generated decoder and asserts every field value. That test is
the "MCU produces genuine ROS2 CDR" pin.

### 4. Bridge to DDS on the host

The shipped bridge bins consume tunnel `TopicSample` frames from a serial/TCP
carrier and republish them as raw DDS samples (and the reverse direction):

```sh
cargo run -p roscmp-dds --bin usb_topic_bridge -- /dev/ttyACM0 \
    --rx /mcu_twist:geometry_msgs/msg/Twist     # MCU → DDS
```

At that point any DDS participant — including a vanilla ROS2 `ros2 topic echo`
on the same bus — sees the MCU's message. The DDS hop itself is proven by
`raw_dds_probe.rs` and by the wheel demo above (which runs the same tunnel⇄DDS
machinery in the opposite direction); the podman-multicast caveat from the
cherry section applies to demonstrating `ros2 topic echo` on this Mac.

No micro-ROS, no rmw, no ROS message headers on the MCU — the firmware carries
only the generated bindings (~ a few hundred lines of `core`-only Rust) and the
`roscmp-tunnel-core` framing codec.

### Embassy / async-runtime compatibility

`roscmp-tunnel-core` is `#![no_std]`, alloc-free, and contains **no blocking
calls, no timers, and no I/O** — encoders write into caller buffers and
`parse_payload` borrows from a caller slice, so it is directly usable from
Embassy tasks (or any executor): do your `read_exact`/`write_all` with
whatever async HAL you have and hand the bytes to the codec. The same holds
for the `--no-std` generated message bindings. (Verified by inspection of the
crate — it has zero dependencies — and by its use in the polling PL011
firmware here; no Embassy project is built in this repo.)

## Future work — MicroPython on the MCU (not built)

To put a *MicroPython* interpreter on a microcontroller speaking this exact
protocol, the doctrine-correct shape is to compile the existing
`roscmp-tunnel-core` Rust crate to a MicroPython **native module** (`.mpy`
natmod) — the same Rust codec, exposed to MicroPython via its C natmod ABI, with
**no** Python reimplementation of the frame format. That keeps the single-source
invariant (`roscmp_dds::tunnel ≡ roscmp-tunnel-core`) intact. Documented here as
the intended path; deliberately not implemented.

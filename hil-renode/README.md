# Renode HIL: Roswell Tunnel Over UART

This workspace shows that an MCU can speak selected ROS 2 topics through the Roswell tunnel
bridge, without micro-ROS and without physical hardware: a no_std Cortex-M
firmware runs the same Roswell tunnel frame codec as the host bridge inside a
[Renode](https://renode.io) simulation driven by the `hilt` crate.

This is a standalone workspace (not part of the root roswell `--workspace` gate,
like `fuzz/`): the firmware cross-compiles to `thumbv6m-none-eabi`, the harness
runs on the host.

## Layout

- `fw/` — the firmware crate (`roswell-hil-fw`). `src/main.rs` self-checks the
  codec and calls `hil_marker` (hooked by hilt to log `HIL OK`);
  `src/uart.rs` (feature `uart`) drives a PL011 UART and answers tunnel frames.
  Both use the shared no_std [`roswell-tunnel-core`](../roswell-tunnel-core) crate.
  `src/msgs_nostd.rs` is the `roswell --lang rust --no-std` generated
  heapless CDR bindings for `geometry_msgs` Vector3/Twist (freshness pinned by
  the root `tests/nostd_codegen_tests.rs`).
- `tests/tunnel_hil.rs` — the host HIL tests:
  - `tunnel_codec_runs_on_cortex_m` (Stage A): boot the firmware, assert `HIL OK`.
  - `mcu_acks_topic_sample_over_uart` (Stage B): bridge the UART to a host TCP
    port, send a `TopicSample`, assert the firmware replies with an `Ack`.
  - `mcu_encodes_real_twist_cdr_over_uart` (Stage C): send a `Heartbeat`; the
    firmware encodes a real `geometry_msgs/msg/Twist` with the `--no-std`
    generated codec and replies with a `TopicSample`; the host decodes the CDR
    with the std generated decoder and asserts every field.

## Running

```sh
cargo test -p roswell-hil -- --ignored     # runs the Renode-backed HIL tests
```

The firmware alone builds and its symbols/vector table verify without Renode:

```sh
cargo build -p roswell-hil-fw --target thumbv6m-none-eabi --release
cargo build -p roswell-hil-fw --target thumbv6m-none-eabi --release --features uart
```

## Why The HIL Tests Are Ignored

They need a Renode container image and each takes ~2 min (the simulation runs
its full `RunFor` window), so they're too slow for the default `cargo test` and
are gated behind Rust's `#[ignore]` attribute and the `--ignored` test flag.

`hilt` selects the image by host architecture: the amd64 `antmicro/renode` on
x86-64, or a **native `linux-arm64`** image it builds on demand on Apple-Silicon.
The amd64 image is avoided on arm64 because emulation can make Renode platform
loading prohibitively slow. The native image avoids that delay.

Running them needs `cargo install cargo-binutils` (provides `rust-objdump` /
`rust-nm`, which `hilt` uses to read the firmware vector table).

The protocol is also verified without Renode by:

- `cargo test -p roswell-tunnel-core` — the codec's own round-trip tests,
- `cargo test -p roswell-ros2-compat --test tunnel_core_equivalence` — byte-for-byte
  equivalence between the host `tunnel` codec and this no_std core.

# hil-renode — roscmp tunnel-over-UART, on simulated silicon

Proves the strategic claim *"an MCU speaks ROS 2 via the roscmp tunnel bridge,
no micro-ROS"* without physical hardware: a no_std Cortex-M firmware runs the
**same** `roscmp` tunnel frame codec the host bridge uses, inside a
[Renode](https://renode.io) simulation driven by [`hilt`](../../../hardware/hilt).

This is a standalone workspace (not part of the root roscmp `--workspace` gate,
like `fuzz/`): the firmware cross-compiles to `thumbv6m-none-eabi`, the harness
runs on the host.

## Layout

- `fw/` — the firmware crate (`roscmp-hil-fw`). `src/main.rs` self-checks the
  codec and calls `hil_marker` (hooked by hilt to log `HIL OK`);
  `src/uart.rs` (feature `uart`) drives a PL011 UART and answers tunnel frames.
  Both use the shared no_std [`roscmp-tunnel-core`](../roscmp-tunnel-core) crate.
  `src/msgs_nostd.rs` is the `roscmp --lang rust --no-std` generated
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
cargo test -p roscmp-hil -- --ignored     # runs the Renode-backed HIL tests
```

The firmware alone builds and its symbols/vector table verify without Renode:

```sh
cargo build -p roscmp-hil-fw --target thumbv6m-none-eabi --release
cargo build -p roscmp-hil-fw --target thumbv6m-none-eabi --release --features uart
```

## Why the HIL tests are `#[ignore]`d here

They need a Renode container image and each takes ~2 min (the simulation runs
its full `RunFor` window), so they're too slow for the default `cargo test` and
are gated behind `--ignored`.

`hilt` selects the image by host architecture: the amd64 `antmicro/renode` on
x86-64, or a **native `linux-arm64`** image it builds on demand on Apple-Silicon.
The amd64 image is avoided on arm64 because, under `qemu-user` in podman's Linux
VM (Rosetta is deliberately disabled there), `machine LoadPlatformDescription`
never completes (>400 s, measured) — the native image loads it in ~3 s. Both
Stage A and Stage B pass live on this Apple-Silicon host with the native image.

Running them needs `cargo install cargo-binutils` (provides `rust-objdump` /
`rust-nm`, which `hilt` uses to read the firmware vector table).

The protocol itself is also verified on this host without Renode by:

- `cargo test -p roscmp-tunnel-core` — the codec's own round-trip tests,
- `cargo test -p roscmp-dds --test tunnel_core_equivalence` — byte-for-byte
  equivalence between the host `tunnel` codec and this no_std core.

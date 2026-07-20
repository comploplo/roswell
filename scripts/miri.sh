#!/usr/bin/env bash
# Run the PURE, unsafe-codec test surface under Miri (UB + leak detection).
#
# Requirements:
#   rustup +nightly component add miri
#
# Miri interprets Rust with no real OS/network, so RustDDS / loopback / network
# tests CANNOT run under it. We therefore restrict to the byte-level codec:
#   - roswell lib unit tests            → the CDR runtime (src/cdr.rs::tests)
#   - roswell dynamic_decode_regression → decode error-path leak-freedom
#   - roswell-ros2-compat dynamic_byte_equality → the main unsafe-walker exerciser
#     (encode/decode/fini/dealloc over C-ABI memory, byte-equality vs codegen)
#
# `-Zmiri-disable-isolation` lets the one sample-file-loading test in
# dynamic_byte_equality read from disk; nothing here touches the network.
set -euo pipefail
cd "$(dirname "$0")/.."

export MIRIFLAGS="${MIRIFLAGS:-} -Zmiri-disable-isolation"

echo "==> miri: roswell CDR runtime unit tests"
cargo +nightly miri test -p roswell --lib

echo "==> miri: roswell decode error-path regressions"
cargo +nightly miri test -p roswell --test dynamic_decode_regression

echo "==> miri: roswell-ros2-compat dynamic byte-equality (unsafe walkers)"
cargo +nightly miri test -p roswell-ros2-compat --test dynamic_byte_equality

echo "Miri: all pure codec tests passed."

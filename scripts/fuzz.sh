#!/usr/bin/env bash
# Run every cargo-fuzz target for a configurable duration.
#
# Requirements:
#   rustup toolchain install nightly
#   cargo install cargo-fuzz
#   (macOS note: LeakSanitizer is unsupported on Darwin, so leak bugs in the
#    unsafe codec are caught by `scripts/miri.sh`, not by these runs.)
#
# Usage:
#   scripts/fuzz.sh [SECONDS_PER_TARGET]   # default 60
#
# The `fuzz/` crate is detached from the workspace (see fuzz/Cargo.toml); these
# targets build only under `cargo +nightly fuzz` and never via scripts/check.sh.
set -euo pipefail
cd "$(dirname "$0")/.."

DURATION="${1:-60}"
TARGETS=(dynamic_decode msg_parser idl_parser mcap_reader)

for t in "${TARGETS[@]}"; do
    echo "==> fuzzing ${t} for ${DURATION}s"
    cargo +nightly fuzz run "${t}" -- \
        -max_total_time="${DURATION}" -rss_limit_mb=4096
done

echo "Fuzzing complete: ${#TARGETS[@]} targets, ${DURATION}s each."

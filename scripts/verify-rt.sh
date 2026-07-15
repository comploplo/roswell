#!/usr/bin/env bash
# Re-discharge the machine-checked real-time proofs (Creusot + Why3).
#
# Verifies the pure hot-path cores in the `roscmp-verify` crate: panic-freedom
# and functional contracts for the CDR alignment arithmetic (`pad_to`), layout
# offset rounding (`round_up`), timer catch-up scheduling (`next_fire_after`),
# and the tunnel bounded-queue policy (`plan_enqueue`). See docs/RT.md for the
# exact properties proven and, importantly, what is NOT proven (timing/WCET,
# allocator latency, RustDDS internals, OS scheduling).
#
# ── Requirements (opt-in; NONE of this touches a normal `cargo build`) ────────
#   • Creusot + Why3 installed. This repo was verified with:
#       cargo-creusot 0.13.0-dev, toolchain nightly-2026-06-22,
#       Why3 1.8.2, why3find 1.3.0, provers alt-ergo 2.6.2 / z3 4.15.3 /
#       cvc5 1.3.1 / cvc4 1.8.
#   • Why3 + provers on PATH. The standard install puts them in the Creusot opam
#     switch; this script prepends "$CREUSOT_OPAM/bin" (default ~/.creusot/_opam).
#   • `~/.cargo/config.toml` must carry the Creusot patch so the optional,
#     verification-only `creusot-std` dependency resolves:
#       [patch.crates-io]
#       creusot-std = { path = "<creusot checkout>/creusot-std" }
#     (`cargo creusot config` writes this.) `creusot-std` is LGPL and pulled in
#     ONLY under the `verify` feature, so it never enters shipped artifacts.
#
# Usage:
#   scripts/verify-rt.sh           # translate + discharge all proofs
#   scripts/verify-rt.sh --replay  # replay checked-in proof sessions only
set -euo pipefail
cd "$(dirname "$0")/.."

CREUSOT_OPAM="${CREUSOT_OPAM:-$HOME/.creusot/_opam}"
export PATH="$CREUSOT_OPAM/bin:$PATH"

if ! command -v cargo-creusot >/dev/null 2>&1; then
  echo "error: cargo-creusot not found on PATH. Install Creusot first." >&2
  echo "       See https://creusot.gitlabpages.inria.fr/creusot/ and rerun." >&2
  exit 127
fi
if ! command -v why3 >/dev/null 2>&1; then
  echo "error: why3 not found (expected under $CREUSOT_OPAM/bin)." >&2
  echo "       Set CREUSOT_OPAM to your Creusot opam switch and rerun." >&2
  exit 127
fi

# `creusot-std` is an optional dependency, so cargo-creusot's version probe
# (which reads plain `cargo metadata`) cannot see it; `--no-check-version`
# skips that probe. The build step itself force-enables creusot-std's features.
ARGS=(--no-check-version)
[ "${1:-}" = "--replay" ] && ARGS+=(--replay)

echo "==> Discharging real-time proofs (roscmp-verify) with Creusot + Why3"
cargo creusot "${ARGS[@]}"
echo "All real-time proofs discharged."

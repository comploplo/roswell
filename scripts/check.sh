#!/usr/bin/env bash
# Full local quality gate.
# Usage: ./scripts/check.sh
set -euo pipefail
cd "$(dirname "$0")/.."

echo "==> rustfmt"
cargo fmt --all -- --check

echo "==> clippy (pedantic, warnings = errors)"
cargo clippy --workspace --all-targets -- -D warnings

echo "==> cargo-deny"
cargo deny check

echo "==> tests"
cargo test --workspace

echo "==> python tests"
uv run --project python --extra test pytest python/tests -q

echo "All checks passed."

#!/usr/bin/env bash
# Full local quality gate — same checks CI and pre-commit run.
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

echo "All checks passed."

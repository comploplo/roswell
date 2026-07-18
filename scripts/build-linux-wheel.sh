#!/usr/bin/env bash
#
# Build a manylinux Python wheel for roscmp inside a container.
#
# roscmp is pure ctypes over a C ABI, so the wheel is py3/none but carries a
# platform tag for the bundled roscmp_c cdylib. This script builds that cdylib
# against an old glibc (manylinux_2_28) so the resulting wheel installs on any
# reasonably modern Linux, then confirms the tag with `auditwheel show`.
#
# Usage:
#   scripts/build-linux-wheel.sh [aarch64|x86_64]
#
# ARCH defaults to the host arch (aarch64 on Apple Silicon). x86_64 on an arm
# host works only if podman/qemu emulation is configured — expect a slow build.
#
# Requires: podman (a running machine). No Rust/Python needed on the host.
# Output: python/dist/roscmp-<ver>-py3-none-manylinux_2_28_<arch>.whl
set -euo pipefail

ARCH="${1:-$(uname -m)}"
case "$ARCH" in
  arm64 | aarch64) ARCH=aarch64 ;;
  x86_64 | amd64) ARCH=x86_64 ;;
  *) echo "unsupported arch: $ARCH (use aarch64 or x86_64)" >&2; exit 2 ;;
esac

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="quay.io/pypa/manylinux_2_28_${ARCH}:latest"
STAGE="$(mktemp -d)"
trap 'rm -rf "$STAGE"' EXIT

echo ">> building manylinux_2_28_${ARCH} wheel via ${IMAGE}"
if [ "$ARCH" != "$(uname -m | sed s/arm64/aarch64/)" ]; then
  echo ">> NOTE: ${ARCH} != host arch — this runs under qemu emulation and will be slow"
fi

# Stage a clean source copy so the container's Linux build never pollutes the
# host's target/ (different object format) or picks up host build artifacts.
echo ">> staging source into ${STAGE}"
rsync -a \
  --exclude='target' --exclude='.git' --exclude='.venv*' --exclude='fuzz' \
  --exclude='python/dist' --exclude='.why3find' --exclude='.pytest_cache' \
  --exclude='python/roscmp.egg-info' --exclude='__pycache__' \
  "$REPO_ROOT/" "$STAGE/"

podman pull -q "$IMAGE" >/dev/null

podman run --rm -v "$STAGE":/build:rw,Z "$IMAGE" bash -euo pipefail -c '
  export CARGO_HOME=/build/.cargo-home RUSTUP_HOME=/build/.rustup
  echo ">> installing rust toolchain"
  curl --proto "=https" --tlsv1.2 -sSf https://sh.rustup.rs \
    | sh -s -- -y --profile minimal --default-toolchain stable >/dev/null 2>&1
  export PATH="$CARGO_HOME/bin:$PATH"
  rustc --version
  PY=/opt/python/cp311-cp311/bin/python
  "$PY" -m pip install -q --upgrade build >/dev/null
  echo ">> building wheel (cargo build -p roscmp-c --release + bundle)"
  cd /build
  "$PY" -m build --wheel python/
  echo ">> auditwheel show (confirm manylinux tag is legitimate)"
  auditwheel show python/dist/*.whl
'

mkdir -p "$REPO_ROOT/python/dist"
cp "$STAGE"/python/dist/*.whl "$REPO_ROOT/python/dist/"
echo ">> done. wheels in python/dist/:"
ls -1 "$REPO_ROOT"/python/dist/*manylinux*"${ARCH}"*.whl

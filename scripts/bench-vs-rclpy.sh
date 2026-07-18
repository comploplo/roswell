#!/usr/bin/env bash
#
# Honest head-to-head bench: roscmp wheel vs rclpy, both inside ONE ros:jazzy
# container (same kernel, same CPU, both native Linux; rclpy exactly as ROS
# ships it — default rmw + transports). Runs the two benchmark scripts in
# python/benchmarks/ sequentially and prints their JSON lines.
#
# Caveat for the report: on macOS this runs in podman's Linux VM, so absolute
# numbers are "Linux VM on Apple Silicon"; the RELATIVE comparison is the claim.
#
# Usage: scripts/bench-vs-rclpy.sh [wheel-path]
# Requires: podman (running machine), the manylinux wheel in python/dist/.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ARCH="$(uname -m | sed s/arm64/aarch64/)"
WHEEL="${1:-$(ls "$REPO_ROOT"/python/dist/roscmp-*-manylinux_2_28_"${ARCH}".whl | head -1)}"
IMAGE="docker.io/library/ros:jazzy"

echo ">> wheel: $WHEEL"
echo ">> image: $IMAGE"

podman run --rm \
  -v "$WHEEL":/bench/"$(basename "$WHEEL")":ro,Z \
  -v "$REPO_ROOT"/python/benchmarks:/bench/benchmarks:ro,Z \
  "$IMAGE" bash -euo pipefail -c '
    set +u; source /opt/ros/jazzy/setup.bash; set -u
    apt-get update -qq >/dev/null && apt-get install -y -qq python3-pip >/dev/null
    pip install -q --break-system-packages /bench/roscmp-*.whl
    export ROS_DOMAIN_ID=17
    echo "== rclpy =="
    python3 /bench/benchmarks/bench_rclpy.py
    echo "== roscmp =="
    python3 /bench/benchmarks/bench_roscmp.py
  '

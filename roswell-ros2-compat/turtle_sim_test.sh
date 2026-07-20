#!/bin/bash
# M5 proof: drive a real ROS2 sim (turtlesim) from our side over RTPS.
# Runs vanilla turtlesim_node (headless via xvfb) and our `turtle_teleop auto`,
# then checks the turtle's pose actually changed. Requires Docker.
set -e
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
ENGINE="${CONTAINER_ENGINE:-docker}"
"$ENGINE" run --rm -v "$ROOT":/work:ro ros:jazzy bash -lc '
set -e
cp -r /work /build && cd /build
echo "=== installing toolchain + turtlesim + xvfb ==="
apt-get update -qq >/dev/null 2>&1
apt-get install -y -qq curl build-essential xvfb ros-jazzy-turtlesim ros-jazzy-geometry-msgs >/dev/null 2>&1
curl --proto "=https" --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y --profile minimal >/dev/null 2>&1
. "$HOME/.cargo/env"
echo "=== building turtle_teleop (release) ==="
cargo build --release -p roswell-ros2-compat --bin turtle_teleop 2>&1 | tail -2
. /opt/ros/jazzy/setup.bash

echo "=== starting vanilla turtlesim (headless) ==="
xvfb-run -a ros2 run turtlesim turtlesim_node > /tmp/sim.txt 2>&1 &
sleep 8

echo "=== driving it with our auto teleop ==="
./target/release/turtle_teleop auto | tee /tmp/drive.txt
'

#!/bin/bash
# M4 interop proof: every roscmp-dds endpoint exercised against a *vanilla* ROS2
# node (ros:jazzy), all in one container so RTPS discovery just works.
#   1. talker   -> `ros2 topic echo`        (we publish, ROS2 receives)
#   2. listener <- `ros2 topic pub`         (ROS2 publishes, we receive)
#   3. teleop   -> `ros2 topic echo`        (we publish Twist on /cmd_vel)
#   4. add_server <- `ros2 service call`    (ROS2 calls our service, gets sum)
# Requires Docker or Podman (CONTAINER_ENGINE=podman). Mounts the workspace root.
set -e
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
ENGINE="${CONTAINER_ENGINE:-docker}"
"$ENGINE" run --rm -v "$ROOT":/work:ro ros:jazzy bash -lc '
set -e
cp -r /work /build && cd /build
echo "=== installing toolchain + ROS interface packages ==="
apt-get update -qq >/dev/null 2>&1
apt-get install -y -qq curl build-essential ros-jazzy-geometry-msgs ros-jazzy-example-interfaces >/dev/null 2>&1
curl --proto "=https" --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y --profile minimal >/dev/null 2>&1
. "$HOME/.cargo/env"
echo "=== building all bins (release) ==="
cargo build --release -p roscmp-dds --bins 2>&1 | tail -2
. /opt/ros/jazzy/setup.bash
B=./target/release

pass=0; fail=0
check() { if grep -q "$2" "$3"; then echo "TEST $1: PASS"; pass=$((pass+1)); else echo "TEST $1: FAIL"; fail=$((fail+1)); tail -8 "$3"; fi; }

echo "===== 1. talker -> ros2 topic echo ====="
( timeout 18 ros2 topic echo /chatter std_msgs/msg/String > /tmp/t1.txt 2>&1 ) &
sleep 6; timeout 10 $B/talker > /dev/null 2>&1 || true; sleep 1
check 1 "hello from roscmp" /tmp/t1.txt

echo "===== 2. listener <- ros2 topic pub ====="
( timeout 20 $B/listener > /tmp/t2.txt 2>&1 ) &
sleep 6; timeout 8 ros2 topic pub -r 5 /chatter std_msgs/msg/String "{data: from vanilla ros2}" > /dev/null 2>&1 || true; sleep 2
check 2 "received: from vanilla ros2" /tmp/t2.txt

echo "===== 3. teleop -> ros2 topic echo (/cmd_vel) ====="
( timeout 18 ros2 topic echo /cmd_vel geometry_msgs/msg/Twist > /tmp/t3.txt 2>&1 ) &
sleep 6; timeout 10 $B/teleop > /dev/null 2>&1 || true; sleep 1
check 3 "x: 0.2" /tmp/t3.txt

echo "===== 4. add_server <- ros2 service call ====="
( timeout 25 $B/add_server > /tmp/t4s.txt 2>&1 ) &
sleep 7; timeout 12 ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 3, b: 4}" > /tmp/t4.txt 2>&1 || true; sleep 1
check 4 "sum=7" /tmp/t4s.txt

echo "===== SUMMARY: $pass passed, $fail failed ====="
'

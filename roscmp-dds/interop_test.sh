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
# Copy sources only: host target/ dirs are gigabytes of foreign-arch artifacts.
mkdir /build
tar -C /work --exclude=target --exclude=./.git --exclude="python/.venv*" \
    --exclude=python/dist --exclude="*/target" -cf - . | tar -C /build -xf -
cd /build
echo "=== installing toolchain + ROS interface packages ==="
apt-get update -qq >/dev/null 2>&1
apt-get install -y -qq curl build-essential ros-jazzy-geometry-msgs ros-jazzy-example-interfaces ros-jazzy-demo-nodes-cpp ros-jazzy-rosgraph-msgs ros-jazzy-rcl-interfaces ros-jazzy-tf2-msgs ros-jazzy-lifecycle-msgs ros-jazzy-type-description-interfaces >/dev/null 2>&1
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

echo "===== 5. QoS sensor_data (best-effort) -> ros2 topic echo ====="
( timeout 18 ros2 topic echo --qos-reliability best_effort /chatter std_msgs/msg/String > /tmp/t5.txt 2>&1 ) &
sleep 6; timeout 10 $B/talker sensor_data > /dev/null 2>&1 || true; sleep 1
check 5 "hello from roscmp" /tmp/t5.txt

echo "===== 6. QoS latched (transient_local) -> late ros2 subscriber ====="
# Publish latched, hold the writer, THEN start a late transient_local subscriber:
# it must still receive the retained sample (volatile would get nothing).
( timeout 30 $B/talker latched > /tmp/t6pub.txt 2>&1 ) &
sleep 16
timeout 8 ros2 topic echo --qos-durability transient_local --qos-reliability reliable --once /chatter std_msgs/msg/String > /tmp/t6.txt 2>&1 || true
check 6 "hello from roscmp" /tmp/t6.txt

echo "===== 7. add_client -> vanilla ROS2 service (we call them) ====="
( timeout 25 ros2 run demo_nodes_cpp add_two_ints_server > /tmp/t7s.txt 2>&1 ) &
sleep 6; timeout 15 $B/add_client > /tmp/t7.txt 2>&1 || true; sleep 1
check 7 "sum=11" /tmp/t7.txt

echo "===== 8. graph introspection sees a vanilla publisher ====="
( timeout 20 ros2 topic pub -r 5 /chatter std_msgs/msg/String "{data: hi}" > /dev/null 2>&1 ) &
sleep 4; timeout 10 $B/graph > /tmp/t8.txt 2>&1 || true; sleep 1
check 8 "/chatter  \[std_msgs/msg/String\]" /tmp/t8.txt

echo "===== 9. runtime contracts: /clock ====="
( timeout 55 $B/runtime_contracts > /tmp/t9node.txt 2>&1 ) &
sleep 8
timeout 10 ros2 topic echo --once /clock rosgraph_msgs/msg/Clock > /tmp/t9.txt 2>&1 || true
check 9 "clock:" /tmp/t9.txt

echo "===== 10. runtime contracts: /rosout ====="
timeout 10 ros2 topic echo --once /rosout rcl_interfaces/msg/Log > /tmp/t10.txt 2>&1 || true
check 10 "runtime contracts online" /tmp/t10.txt

echo "===== 11. runtime contracts: /tf ====="
timeout 10 ros2 topic echo --once /tf tf2_msgs/msg/TFMessage > /tmp/t11.txt 2>&1 || true
check 11 "child_frame_id: base_link" /tmp/t11.txt

echo "===== 12. runtime contracts: lifecycle services ====="
timeout 10 ros2 service call /runtime_contracts/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 1, label: configure}}" > /tmp/t12a.txt 2>&1 || true
timeout 10 ros2 service call /runtime_contracts/get_state lifecycle_msgs/srv/GetState "{}" > /tmp/t12b.txt 2>&1 || true
check 12 "inactive" /tmp/t12b.txt

echo "===== 13. runtime contracts: get_type_description ====="
timeout 10 ros2 service call /runtime_contracts/get_type_description type_description_interfaces/srv/GetTypeDescription "{type_name: std_msgs/msg/String, type_hash: '"'"''"'"', include_type_sources: false}" > /tmp/t13.txt 2>&1 || true
check 13 "std_msgs/msg/String" /tmp/t13.txt

echo "===== 14. action server: ros2 action send_goal ====="
( timeout 55 $B/fibonacci_action_server > /tmp/t14server.txt 2>&1 ) &
sleep 8
timeout 20 ros2 action send_goal /fibonacci example_interfaces/action/Fibonacci "{order: 6}" > /tmp/t14.txt 2>&1 || true
check 14 "sequence:" /tmp/t14.txt

echo "===== 15. ros_discovery_info: ros2 node list sees our node ====="
( timeout 28 $B/graph --advertise-node roscmp_discovery_node > /tmp/t15node.txt 2>&1 ) &
sleep 6; timeout 10 ros2 node list > /tmp/t15.txt 2>&1 || true; sleep 1
check 15 "roscmp_discovery_node" /tmp/t15.txt

echo "===== 16. ros2 node info sees our node publisher/subscriber ====="
# Same advertised node (still alive from check 15): its /chatter publisher and
# /commands subscriber GIDs are registered on ros_discovery_info.
timeout 10 ros2 node info /roscmp_discovery_node > /tmp/t16.txt 2>&1 || true; sleep 1
check 16 "/chatter: std_msgs/msg/String" /tmp/t16.txt

echo "===== SUMMARY: $pass passed, $fail failed ====="
'

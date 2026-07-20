#!/usr/bin/env bash
# The killer demo: the embedded **Python** node (the shipped `roswell` FFI wheel,
# pure ctypes over the Rust runtime — the Raspberry-Pi-class node) publishes a
# ROS topic over real RTPS/DDS, and the embedded **Rust** firmware (no_std, on
# simulated Cortex-M silicon in Renode) receives it over the tunnel/UART and acks
# it. Zero ROS installed, zero DDS daemon, no parallel protocol code.
#
#   wheel_node.py  --geometry_msgs/msg/Twist on /cmd_vel-->  real DDS
#        --> demo harness (roswell-ros2-compat subscriber, same runtime as the bridge bins)
#        --> tunnel/UART (hilt bridge) --> Renode firmware --Ack--> harness (asserts)
#
# Requirements (all present on this host):
#   * roswell wheel installed in python/.venv-test  (see python/README.md)
#   * Renode image  — auto-built by hilt on arm64 (localhost/hilt-renode)
#   * cargo-binutils — `cargo install cargo-binutils`
#
# ~1 min (full Renode RunFor window + DDS discovery).
set -euo pipefail
cd "$(dirname "$0")/../../hil-renode"

exec cargo test -p roswell-hil --test wheel_firmware_demo \
    -- --ignored --nocapture

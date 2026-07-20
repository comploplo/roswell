"""rclpy cross-host bench: driver and echo halves, two separate netns.

Mirror of ``bench_cross_roswell.py`` for rclpy, run exactly as ROS ships it
(default rmw_fastrtps, default transports). Same types / QoS / sizes, same
RTT/2 echo protocol (see that file for why RTT/2). The publisher and subscriber
live in different containers, so FastDDS cannot use shared memory and falls back
to UDP — the same real-network fight roswell gets.

Roles:
  driver  publishes seq on /ping_<size>, reads the echo on /pong_<size>.
  echo    reflects every /ping_<size> sample back on /pong_<size>.
"""

import argparse
import json
import struct
import sys
import time

import rclpy
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
import numpy as np
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

from _harness import measure, settle, stats  # noqa: E402

# (size_bytes, iters) — reduced iters vs same-host for wall-clock sanity.
SIZES = [(64, 100), (64 * 1024, 100), (1024 * 1024, 50), (10 * 1024 * 1024, 20)]
RTT_TIMEOUT_S = 10.0
SETTLE_S = 25.0

QOS = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
)


def make_endpoints(node, size):
    inbox = []
    if size == 64:
        pub = node.create_publisher(String, f"/ping_{size}", QOS)
        node.create_subscription(String, f"/pong_{size}", lambda m: inbox.append(m), QOS)
        msg = String()

        def send(seq):
            msg.data = f"{seq:016d}" + "x" * 48
            pub.publish(msg)

        def recv_seq(deadline):
            while time.perf_counter() < deadline:
                rclpy.spin_once(node, timeout_sec=0.001)
                if inbox:
                    return int(inbox.pop().data[:16])
            return None

        return send, recv_seq

    pub = node.create_publisher(PointCloud2, f"/ping_{size}", QOS)
    node.create_subscription(PointCloud2, f"/pong_{size}", lambda m: inbox.append(m), QOS)
    msg = PointCloud2()
    msg.header.frame_id = "map"
    msg.height = 1
    msg.width = size
    msg.point_step = 1
    msg.row_step = size
    payload = np.zeros(size, dtype=np.uint8)
    msg.data = payload.tobytes()

    def send(seq):
        payload[:8] = np.frombuffer(struct.pack("<Q", seq), dtype=np.uint8)
        msg.data = payload.tobytes()
        pub.publish(msg)

    def recv_seq(deadline):
        while time.perf_counter() < deadline:
            rclpy.spin_once(node, timeout_sec=0.001)
            if inbox:
                return struct.unpack("<Q", bytes(inbox.pop().data[:8]))[0]
        return None

    return send, recv_seq


def make_echo(node, size):
    if size == 64:
        pub = node.create_publisher(String, f"/pong_{size}", QOS)

        def cb(m):
            out = String()
            out.data = m.data
            pub.publish(out)

        node.create_subscription(String, f"/ping_{size}", cb, QOS)
        return

    pub = node.create_publisher(PointCloud2, f"/pong_{size}", QOS)

    def cb(m):
        pub.publish(m)  # reflect verbatim

    node.create_subscription(PointCloud2, f"/ping_{size}", cb, QOS)


def run_driver(node, size, iters):
    send, recv_seq = make_endpoints(node, size)

    settle(send, recv_seq, SETTLE_S, f"size {size}")
    rtt, lost = measure(send, recv_seq, iters, RTT_TIMEOUT_S)
    return {
        "case": f"{'String' if size == 64 else 'PointCloud2'} {size}B",
        "e2e_rtt2": stats([dt / 2.0 for dt in rtt]) if rtt else None,  # RTT/2 -> one-way
        "lost": lost,
    }


def driver():
    rclpy.init()
    t = time.perf_counter()
    node = rclpy.create_node("bench_cross_rclpy_driver")
    node_ms = (time.perf_counter() - t) * 1000.0
    try:
        results = []
        for size, iters in SIZES:
            try:
                results.append(run_driver(node, size, iters))
            except RuntimeError as e:
                results.append({"case": f"{size}B", "error": str(e)})
        print(json.dumps({"lib": "rclpy", "node_ms": round(node_ms, 2), "results": results}))
    finally:
        node.destroy_node()
        rclpy.shutdown()


def echo(seconds):
    rclpy.init()
    node = rclpy.create_node("bench_cross_rclpy_echo")
    try:
        for size, _ in SIZES:
            make_echo(node, size)
        deadline = time.perf_counter() + seconds
        while time.perf_counter() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("role", choices=["driver", "echo"])
    ap.add_argument("--seconds", type=float, default=240.0)
    a = ap.parse_args()
    if a.role == "driver":
        driver()
    else:
        echo(a.seconds)


if __name__ == "__main__":
    sys.exit(main())

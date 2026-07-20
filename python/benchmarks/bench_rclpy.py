"""rclpy side of the honest head-to-head vs roswell.

Identical measurement logic to bench_roswell.py — both import the protocol from
``_harness``: same message types, same QoS, same one-message-in-flight
end-to-end latency protocol, same iteration counts. rclpy runs exactly as ROS
ships it (default rmw, default transports) — no handicapping.
"""

import struct
import sys
import time

t0 = time.perf_counter()
import rclpy  # noqa: E402
from rclpy.qos import (  # noqa: E402
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

IMPORT_MS = (time.perf_counter() - t0) * 1000.0

import numpy as np  # noqa: E402
from sensor_msgs.msg import PointCloud2  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from _harness import SIZES, emit, run_e2e  # noqa: E402

QOS = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
)


def bench_string(node):
    inbox = []
    pub = node.create_publisher(String, "/bench_str", QOS)
    node.create_subscription(String, "/bench_str", lambda m: inbox.append(m), QOS)
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

    return run_e2e("std_msgs/String 64B", send, recv_seq, lambda: pub.publish(msg))


def bench_cloud(node, size):
    inbox = []
    topic = f"/bench_pc_{size}"
    pub = node.create_publisher(PointCloud2, topic, QOS)
    node.create_subscription(PointCloud2, topic, lambda m: inbox.append(m), QOS)
    msg = PointCloud2()
    msg.header.frame_id = "map"
    msg.height = 1
    msg.width = size
    msg.point_step = 1
    msg.row_step = size
    payload = np.zeros(size, dtype=np.uint8)

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

    msg.data = payload.tobytes()
    return run_e2e(f"PointCloud2 {size}B", send, recv_seq, lambda: pub.publish(msg))


def main():
    rclpy.init()
    t = time.perf_counter()
    node = rclpy.create_node("bench_rclpy")
    node_ms = (time.perf_counter() - t) * 1000.0
    try:
        emit(
            "rclpy",
            IMPORT_MS,
            node_ms,
            [lambda: bench_string(node)]
            + [(lambda s=size: bench_cloud(node, s)) for size, _ in SIZES[1:]],
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())

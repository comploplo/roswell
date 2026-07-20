"""roswell side of the honest head-to-head vs rclpy.

Identical measurement logic to bench_rclpy.py — both import the protocol from
``_harness``: same message types (std_msgs/String, sensor_msgs/PointCloud2),
same QoS (reliable, keep-last 10, volatile), same one-message-in-flight
end-to-end latency protocol (seq stamped in the payload, same-process clock),
same iteration counts.

Prints one line of JSON: {"lib": ..., "import_ms": ..., "node_ms": ...,
"rss_mb": ..., "results": [{...}]}.
"""

import struct
import sys
import time

t0 = time.perf_counter()
import roswell  # noqa: E402

IMPORT_MS = (time.perf_counter() - t0) * 1000.0

import numpy as np  # noqa: E402

from _harness import SIZES, emit, run_e2e  # noqa: E402


def bench_string(node):
    T = node.load_type("std_msgs/msg/String")
    pub = node.publisher("/bench_str", T)
    sub = node.subscribe("/bench_str", T)
    msg = pub.new()

    def send(seq):
        msg.data = f"{seq:016d}" + "x" * 48  # 64-char payload
        pub.publish(msg)

    def recv_seq(deadline):
        while time.perf_counter() < deadline:
            m = sub.take()
            if m is not None:
                return int(str(m.data)[:16])
        return None

    return run_e2e("std_msgs/String 64B", send, recv_seq, lambda: pub.publish(msg))


def bench_cloud(node, size):
    T = node.load_type("sensor_msgs/msg/PointCloud2")
    topic = f"/bench_pc_{size}"
    pub = node.publisher(topic, T)
    sub = node.subscribe(topic, T)
    msg = pub.new()
    msg.header.frame_id = "map"
    msg.height = 1
    msg.width = size
    msg.point_step = 1
    msg.row_step = size
    payload = np.zeros(size, dtype=np.uint8)

    def send(seq):
        payload[:8] = np.frombuffer(struct.pack("<Q", seq), dtype=np.uint8)
        msg.data = payload
        pub.publish(msg)

    def recv_seq(deadline):
        while time.perf_counter() < deadline:
            m = sub.take()
            if m is not None:
                return struct.unpack("<Q", bytes(m.data[:8]))[0]
        return None

    msg.data = payload
    return run_e2e(f"PointCloud2 {size}B", send, recv_seq, lambda: pub.publish(msg))


def main():
    t = time.perf_counter()
    node = roswell.Node("bench_roswell", domain=0)
    node_ms = (time.perf_counter() - t) * 1000.0
    try:
        emit(
            "roswell",
            IMPORT_MS,
            node_ms,
            [lambda: bench_string(node)]
            + [(lambda s=size: bench_cloud(node, s)) for size, _ in SIZES[1:]],
        )
    finally:
        node.close()


if __name__ == "__main__":
    sys.exit(main())

"""rclpy side of the honest head-to-head vs roscmp.

Identical measurement logic to bench_roscmp.py: same message types
(std_msgs/String, sensor_msgs/PointCloud2), same QoS (reliable, keep-last 10,
volatile), same one-message-in-flight end-to-end latency protocol (seq stamped
in the payload, same-process clock), same iteration counts. rclpy runs exactly
as ROS ships it (default rmw, default transports) — no handicapping.
"""

import json
import resource
import statistics
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

SIZES = [(64, 200), (64 * 1024, 200), (1024 * 1024, 100), (10 * 1024 * 1024, 30)]
WARMUP = 10
E2E_TIMEOUT_S = 5.0

QOS = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
)


def stats(samples):
    s = sorted(samples)
    return {
        "median_ms": round(statistics.median(s), 4),
        "p95_ms": round(s[int(0.95 * (len(s) - 1))], 4),
        "mean_ms": round(statistics.mean(s), 4),
        "n": len(s),
    }


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


def run_e2e(label, send, recv_seq, pub_only):
    size = int(label.rsplit(" ", 1)[1].rstrip("B")) if label[-1] == "B" else 64
    iters = next(n for s, n in SIZES if s == size)

    # Discovery settle: volatile writers drop pre-match samples, so retry-publish
    # until the first sample comes back, then drain stragglers.
    settle_deadline = time.perf_counter() + 20.0
    while time.perf_counter() < settle_deadline:
        send(999_999)
        if recv_seq(time.perf_counter() + 0.2) is not None:
            break
    else:
        raise RuntimeError(f"{label}: discovery never settled")
    while recv_seq(time.perf_counter() + 0.3) is not None:
        pass

    for i in range(WARMUP):
        send(1_000_000 + i)
        recv_seq(time.perf_counter() + E2E_TIMEOUT_S)

    e2e, lost = [], 0
    for seq in range(iters):
        t = time.perf_counter()
        send(seq)
        deadline = t + E2E_TIMEOUT_S
        got = recv_seq(deadline)
        while got is not None and got != seq:  # discard stale stragglers
            got = recv_seq(deadline)
        dt = (time.perf_counter() - t) * 1000.0
        if got == seq:
            e2e.append(dt)
        else:
            lost += 1
            if lost > max(5, iters // 5):  # hopeless case: stop burning timeouts
                break

    pub_lat = []
    for _ in range(iters):
        t = time.perf_counter()
        pub_only()
        pub_lat.append((time.perf_counter() - t) * 1000.0)
        recv_seq(time.perf_counter() + 0.2)  # drain

    return {
        "case": label,
        "e2e": stats(e2e) if e2e else None,
        "publish": stats(pub_lat),
        "lost": lost,
    }


def main():
    rclpy.init()
    t = time.perf_counter()
    node = rclpy.create_node("bench_rclpy")
    node_ms = (time.perf_counter() - t) * 1000.0
    try:
        results = []
        for case in [lambda: bench_string(node)] + [
            (lambda s=size: bench_cloud(node, s)) for size, _ in SIZES[1:]
        ]:
            try:
                results.append(case())
            except RuntimeError as e:
                results.append({"case": str(e).split(":")[0], "error": str(e)})
        # ru_maxrss is bytes on macOS, kilobytes on Linux.
        raw = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
        rss_mb = raw / (1024 * 1024) if sys.platform == "darwin" else raw / 1024
        print(
            json.dumps(
                {
                    "lib": "rclpy",
                    "import_ms": round(IMPORT_MS, 2),
                    "node_ms": round(node_ms, 2),
                    "rss_mb": round(rss_mb, 1),
                    "results": results,
                }
            )
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())

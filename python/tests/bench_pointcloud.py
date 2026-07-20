"""Acceptance benchmark: end-to-end publish() latency for a large PointCloud2-like
message (~10 MB uint8[] payload), measured from Python.

rclpy's cited figure for publishing a comparable message is ~92 ms (see
ros2/rclpy#763). We print our measured median; this is a report, not a CI gate.

Run directly:  python tests/bench_pointcloud.py
"""

import pathlib
import statistics
import time

import numpy as np

import roswell

FIXTURES = pathlib.Path(__file__).resolve().parent / "fixtures"
PAYLOAD_BYTES = 10 * 1024 * 1024
WARMUP = 5
ITERS = 30


def main() -> None:
    node = roswell.Node("bench_pointcloud", domain=0)
    try:
        cloud_t = node.load_type(FIXTURES / "test_msgs" / "msg" / "BigCloud.msg")
        pub = node.publisher("/bench_points", cloud_t)

        msg = pub.new()
        msg.header.frame_id = "map"  # nested-message string field
        msg.height = 1
        msg.width = PAYLOAD_BYTES
        msg.point_step = 1
        msg.row_step = PAYLOAD_BYTES
        msg.is_dense = True
        # ~10 MB uint8[] copied into a Rust-owned buffer once.
        msg.data = np.zeros(PAYLOAD_BYTES, dtype=np.uint8)

        for _ in range(WARMUP):
            pub.publish(msg)

        samples = []
        for _ in range(ITERS):
            start = time.perf_counter()
            pub.publish(msg)
            samples.append((time.perf_counter() - start) * 1000.0)

        median = statistics.median(samples)
        p95 = sorted(samples)[int(0.95 * (len(samples) - 1))]
        mb = PAYLOAD_BYTES / (1024 * 1024)
        print(
            f"roswell publish() for ~{mb:.0f} MB uint8[]: "
            f"median {median:.3f} ms, p95 {p95:.3f} ms "
            f"(over {ITERS} iters after {WARMUP} warmup)"
        )
        print("rclpy cited baseline for a comparable publish: ~92 ms (ros2/rclpy#763)")
    finally:
        node.close()


if __name__ == "__main__":
    main()

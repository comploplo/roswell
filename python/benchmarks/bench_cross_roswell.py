"""roswell cross-host bench: driver and echo halves, two separate netns.

Same types / QoS / sizes as the same-host bench (``bench_roswell.py``), but the
publisher and subscriber live in *different containers* (different net/PID/IPC
namespaces) so no shared memory is possible and both hops ride real UDP over
the podman veth+bridge.

Because the two halves run in different processes we cannot compare a one-way
timestamp against a same-process clock, so we measure a full round trip with an
echo server in the far container and report **RTT/2** as the one-way estimate.
The driver times everything on its own clock, so cross-container clock skew is
irrelevant to the number. The same RTT/2 protocol is applied to rclpy, so the
comparison stays apples-to-apples.

Roles:
  driver  publishes seq on /ping_<size>, reads the echo on /pong_<size>.
  echo    reflects every /ping_<size> sample back on /pong_<size>.
"""

import argparse
import json
import struct
import sys
import time

import roswell
import numpy as np

from _harness import measure, settle, stats  # noqa: E402

# (size_bytes, iters) — reduced iters vs same-host for wall-clock sanity.
SIZES = [(64, 100), (64 * 1024, 100), (1024 * 1024, 50), (10 * 1024 * 1024, 20)]
RTT_TIMEOUT_S = 10.0
SETTLE_S = 25.0


def make_endpoints(node, size):
    """(send, recv_seq) for one size. Payload seq is stamped in the first 8 B."""
    if size == 64:
        T = node.load_type("std_msgs/msg/String")
        ping = node.publisher(f"/ping_{size}", T)
        pong = node.subscribe(f"/pong_{size}", T)
        msg = ping.new()

        def send(seq):
            msg.data = f"{seq:016d}" + "x" * 48
            ping.publish(msg)

        def recv_seq(deadline):
            while time.perf_counter() < deadline:
                m = pong.take()
                if m is not None:
                    return int(str(m.data)[:16])
            return None

        return send, recv_seq

    T = node.load_type("sensor_msgs/msg/PointCloud2")
    ping = node.publisher(f"/ping_{size}", T)
    pong = node.subscribe(f"/pong_{size}", T)
    msg = ping.new()
    msg.header.frame_id = "map"
    msg.height = 1
    msg.width = size
    msg.point_step = 1
    msg.row_step = size
    payload = np.zeros(size, dtype=np.uint8)
    msg.data = payload

    def send(seq):
        payload[:8] = np.frombuffer(struct.pack("<Q", seq), dtype=np.uint8)
        msg.data = payload
        ping.publish(msg)

    def recv_seq(deadline):
        while time.perf_counter() < deadline:
            m = pong.take()
            if m is not None:
                return struct.unpack("<Q", bytes(m.data[:8]))[0]
        return None

    return send, recv_seq


def make_echo(node, size):
    """(recv, reflect): take a /ping sample, re-emit it verbatim on /pong."""
    if size == 64:
        T = node.load_type("std_msgs/msg/String")
        ping = node.subscribe(f"/ping_{size}", T)
        pong = node.publisher(f"/pong_{size}", T)
        out = pong.new()

        def pump():
            m = ping.take()
            if m is None:
                return False
            out.data = str(m.data)
            pong.publish(out)
            return True

        return pump

    T = node.load_type("sensor_msgs/msg/PointCloud2")
    ping = node.subscribe(f"/ping_{size}", T)
    pong = node.publisher(f"/pong_{size}", T)
    out = pong.new()
    out.header.frame_id = "map"
    out.height = 1
    out.width = size
    out.point_step = 1
    out.row_step = size

    def pump():
        m = ping.take()
        if m is None:
            return False
        out.data = np.frombuffer(bytes(m.data), dtype=np.uint8)
        pong.publish(out)
        return True

    return pump


def run_driver(node, size, iters):
    send, recv_seq = make_endpoints(node, size)

    settle(send, recv_seq, SETTLE_S, f"size {size}")
    rtt, lost = measure(send, recv_seq, iters, RTT_TIMEOUT_S)
    return {
        "case": f"{'String' if size == 64 else 'PointCloud2'} {size}B",
        "e2e_rtt2": stats([dt / 2.0 for dt in rtt]) if rtt else None,  # RTT/2 -> one-way
        "lost": lost,
    }


def driver(domain):
    t = time.perf_counter()
    node = roswell.Node("bench_cross_roswell_driver", domain=domain)
    node_ms = (time.perf_counter() - t) * 1000.0
    try:
        results = []
        for size, iters in SIZES:
            try:
                results.append(run_driver(node, size, iters))
            except RuntimeError as e:
                results.append({"case": f"{size}B", "error": str(e)})
        print(json.dumps({"lib": "roswell", "node_ms": round(node_ms, 2), "results": results}))
    finally:
        node.close()


def echo(domain, seconds):
    node = roswell.Node("bench_cross_roswell_echo", domain=domain)
    try:
        pumps = [make_echo(node, size) for size, _ in SIZES]
        deadline = time.perf_counter() + seconds
        while time.perf_counter() < deadline:
            busy = False
            for pump in pumps:
                while pump():
                    busy = True
            if not busy:
                time.sleep(0.0002)
    finally:
        node.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("role", choices=["driver", "echo"])
    ap.add_argument("--domain", type=int, default=41)
    ap.add_argument("--seconds", type=float, default=240.0)
    a = ap.parse_args()
    if a.role == "driver":
        driver(a.domain)
    else:
        echo(a.domain, a.seconds)


if __name__ == "__main__":
    sys.exit(main())

"""Shared measurement harness for the roswell-vs-rclpy benchmarks.

Both sides of every head-to-head import this module, so the protocol —
discovery settle, warmup, one-message-in-flight seq/echo timing, stats shape,
JSON emit — is identical by construction, not by copy-paste discipline.
"""

import json
import resource
import statistics
import sys
import time

# Same-host head-to-head: (size_bytes, iters).
SIZES = [(64, 200), (64 * 1024, 200), (1024 * 1024, 100), (10 * 1024 * 1024, 30)]
WARMUP = 10
E2E_TIMEOUT_S = 5.0


def stats(samples):
    s = sorted(samples)
    return {
        "median_ms": round(statistics.median(s), 4),
        "p95_ms": round(s[int(0.95 * (len(s) - 1))], 4),
        "mean_ms": round(statistics.mean(s), 4),
        "n": len(s),
    }


def settle(send, recv_seq, settle_s, label):
    """Discovery settle: volatile writers drop pre-match samples, so
    retry-publish until the first sample comes back, then drain stragglers."""
    deadline = time.perf_counter() + settle_s
    while time.perf_counter() < deadline:
        send(999_999)
        if recv_seq(time.perf_counter() + 0.2) is not None:
            break
    else:
        raise RuntimeError(f"{label}: discovery never settled")
    while recv_seq(time.perf_counter() + 0.3) is not None:
        pass


def measure(send, recv_seq, iters, timeout_s):
    """Warmed-up one-in-flight seq/echo loop -> (latencies_ms, lost)."""
    for i in range(WARMUP):
        send(1_000_000 + i)
        recv_seq(time.perf_counter() + timeout_s)

    out, lost = [], 0
    for seq in range(iters):
        t = time.perf_counter()
        send(seq)
        deadline = t + timeout_s
        got = recv_seq(deadline)
        while got is not None and got != seq:  # discard stale stragglers
            got = recv_seq(deadline)
        dt = (time.perf_counter() - t) * 1000.0
        if got == seq:
            out.append(dt)
        else:
            lost += 1
            if lost > max(5, iters // 5):  # hopeless case: stop burning timeouts
                break
    return out, lost


def run_e2e(label, send, recv_seq, pub_only):
    """Same-host case: end-to-end latency plus publish-only latency."""
    size = int(label.rsplit(" ", 1)[1].rstrip("B")) if label[-1] == "B" else 64
    iters = next(n for s, n in SIZES if s == size)

    settle(send, recv_seq, 20.0, label)
    e2e, lost = measure(send, recv_seq, iters, E2E_TIMEOUT_S)

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


def emit(lib, import_ms, node_ms, cases):
    """Run each zero-arg case, collect results, print the one-line JSON report."""
    results = []
    for case in cases:
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
                "lib": lib,
                "import_ms": round(import_ms, 2),
                "node_ms": round(node_ms, 2),
                "rss_mb": round(rss_mb, 1),
                "results": results,
            }
        )
    )

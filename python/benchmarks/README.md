# roscmp vs rclpy — honest head-to-head

Both stacks run inside **one `ros:jazzy` container** (same kernel, same CPU,
both native Linux; rclpy exactly as ROS ships it — default rmw_fastrtps with
its shared-memory transport, no handicapping). The two scripts here use
identical measurement logic: same types (`std_msgs/String`,
`sensor_msgs/PointCloud2`), same QoS (reliable / keep-last 10 / volatile), same
one-message-in-flight end-to-end protocol (seq stamped in the payload,
same-process clock), same iteration counts.

```sh
scripts/bench-vs-rclpy.sh          # needs podman + python/dist manylinux wheel
```

Caveat: on macOS this runs in podman's Linux VM, so absolute numbers are
"Linux VM on Apple Silicon" — the *relative* comparison is the claim.

## Results (2026-07-17, M-series host, jazzy container, rustdds 0.13 + 16 MiB SO_RCVBUF)

Requires `net.core.rmem_max >= 16 MiB` on the host/VM (standard ROS large-data
tuning; the kernel clamps `SO_RCVBUF` to it, default is ~208 KB).

| case | rclpy e2e med/p95 | roscmp e2e med/p95 |
|---|---|---|
| String 64B | 0.078 / 0.090 ms | **0.053 / 0.111 ms** |
| PointCloud2 64KB | **0.123 / 0.136 ms** | 0.867 / 3.615 ms |
| PointCloud2 1MB | **0.376 / 0.394 ms** | 14.0 / 18.9 ms |
| PointCloud2 10MB | **2.86 / 3.29 ms** | 131 / 147 ms |
| import | 68 ms | 68 ms |
| node startup | 105 ms | **1.7 ms** |
| peak RSS | 119 MB | 1.4 GB (large-payload cases; needs investigation) |

All cases deliver with zero loss on both stacks.

Honest reading:
- roscmp wins **small-message latency** (~1.5x) and **node startup** (~60x);
  import cost is a wash.
- rclpy wins **≥64KB same-host** structurally: FastDDS uses shared memory on
  loopback; roscmp rides UDP + fragmentation. Cross-host (Wi-Fi/Ethernet) both
  stacks use UDP — that comparison has not been run yet.
- History: with rustdds 0.11 (never set `SO_RCVBUF`) roscmp could not deliver
  ≥1MB samples at all on default Linux; the 0.13 upgrade + 16 MiB receive
  buffer took 10MB from "not delivered" to 131 ms / zero loss.
- roscmp peak RSS on the large-payload cases (~1.4 GB) is a known follow-up.
- The previously cited "rclpy ~92 ms for a 10MB publish" (ros2/rclpy#763) does
  **not** describe modern jazzy on loopback; do not use it in comparisons.

# Roswell And rclpy Benchmark Notes

These notes compare roswell with `rclpy` in a controlled ROS 2 Jazzy
environment. The goal is to understand where roswell is already useful and
where the standard ROS 2 stack is still clearly ahead.

Both stacks run inside **one `ros:jazzy` container** (same kernel, same CPU,
both native Linux; rclpy exactly as ROS ships it, using default rmw_fastrtps
with its shared-memory transport). The two scripts here use identical
measurement logic: same types (`std_msgs/String`,
`sensor_msgs/PointCloud2`), same QoS (reliable / keep-last 10 / volatile), same
one-message-in-flight end-to-end protocol (seq stamped in the payload,
same-process clock), same iteration counts.

```sh
scripts/bench-vs-rclpy.sh          # needs podman + python/dist manylinux wheel
```

Caveat: on macOS this runs in podman's Linux VM, so absolute numbers are
"Linux VM on Apple Silicon" — the *relative* comparison is the claim.

## Same-Host Results

Measured July 17, 2026 on an M-series host with a Jazzy container, RustDDS
0.13, and a 16 MiB `SO_RCVBUF`.

Requires `net.core.rmem_max >= 16 MiB` on the host/VM (standard ROS large-data
tuning; the kernel clamps `SO_RCVBUF` to it, default is ~208 KB).

| Case | rclpy median / p95 | Roswell median / p95 |
|---|---|---|
| String 64B | 0.078 / 0.090 ms | **0.053 / 0.111 ms** |
| PointCloud2 64KB | **0.123 / 0.136 ms** | 0.867 / 3.615 ms |
| PointCloud2 1MB | **0.376 / 0.394 ms** | 14.0 / 18.9 ms |
| PointCloud2 10MB | **2.86 / 3.29 ms** | 131 / 147 ms |
| import | 68 ms | 68 ms |
| node startup | 105 ms | **1.7 ms** |
| peak RSS | 119 MB | 1.4 GB (large-payload cases; needs investigation) |

All cases deliver with zero loss on both stacks.

### Interpretation
- roswell wins **small-message latency** (~1.5x) and **node startup** (~60x);
  import cost is a wash.
- rclpy wins **≥64KB same-host** structurally: FastDDS uses shared memory on
  loopback; roswell rides UDP + fragmentation. Cross-host (Wi-Fi/Ethernet) both
  stacks use UDP — that comparison has not been run yet.
- History: with rustdds 0.11 (never set `SO_RCVBUF`) roswell could not deliver
  ≥1MB samples at all on default Linux; the 0.13 upgrade + 16 MiB receive
  buffer took 10MB from "not delivered" to 131 ms / zero loss.
- roswell peak RSS on the large-payload cases (~1.4 GB) is a known follow-up.
- The previously cited "rclpy ~92 ms for a 10MB publish" (ros2/rclpy#763) does
  **not** describe modern jazzy on loopback; do not use it in comparisons.

## Cross-Host Results

The same-host table above let FastDDS use **shared memory** on loopback, which
roswell's UDP path cannot match ≥64KB. This section removes SHM from the board:
the publisher and subscriber run in **two separate containers** on a podman
bridge network — different net/PID/IPC namespaces, a real veth+bridge path, no
shared `/dev/shm` — so FastDDS is forced onto UDP, the same transport roswell
always uses.

```sh
scripts/bench-cross-host.sh      # needs podman + python/dist manylinux wheel
```

Because the two halves are separate processes, we can't diff a one-way stamp
against a same-process clock. Each stack instead measures a full **round trip**
through an echo server in the far container and reports **RTT/2**. The driver
times everything on its own clock, so cross-container clock skew is irrelevant
(no need to trust a shared clock). The identical RTT/2 echo protocol runs for
both stacks, same types / QoS / sizes; iters reduced to 100/100/50/20.

**Note:** these are RTT/2 numbers (two DDS hops), so they are *not* directly
comparable to the one-way same-host table above — only rclpy-vs-roswell within
this section is the claim. Discovery is stock multicast SPDP for both stacks
(verified to work across the bridge); no unicast peers, same setting for both.

### Network Isolation Check

1. **Separate network namespaces:** the two containers get distinct bridge IPs
   (e.g. `10.89.0.31` / `10.89.0.32`) on a veth path.
2. **No shared memory:** a marker file made in the echo container's
   `/dev/shm` is **ABSENT** in the driver container (separate IPC/mount ns).
3. **Shared memory cross-check:** rclpy is rerun with SHM explicitly disabled
   via a FastDDS UDP-only profile (`fastdds_no_shm.xml`); the numbers match the
   default run (below), proving the namespace split — not our profile — is what
   removed shared memory. If SHM were somehow active, disabling it would move
   the numbers; it doesn't.

### Measurements

Measured July 18, 2026 on an M-series host with Jazzy containers. Each
configuration was run twice.

e2e = **RTT/2 median / p95 (ms)**; zero loss on every case, both stacks, both runs.

| case | rclpy run A | rclpy run B | roswell run A | roswell run B |
|---|---|---|---|---|
| String 64B | **0.156 / 0.212** | **0.157 / 0.256** | 0.102 / 1.48 | 0.715 / 7.29 |
| PointCloud2 64KB | **0.224 / 0.317** | **0.176 / 0.282** | 1.31 / 3.12 | 2.64 / 7.62 |
| PointCloud2 1MB | **0.713 / 0.749** | **0.715 / 1.11** | 16.6 / 17.6 | 20.3 / 25.2 |
| PointCloud2 10MB | **5.91 / 6.24** | **6.03 / 9.82** | 149 / 155 | 162 / 369 |

rclpy SHM-explicitly-off cross-check (median ms), to compare against rclpy above:

| case | run A | run B |
|---|---|---|
| 64B | 0.158 | 0.152 |
| 64KB | 0.207 | 0.205 |
| 1MB | 0.719 | 0.740 |
| 10MB | 6.01 | 6.91 |

### Interpretation
- **SHM is not the whole story.** Even with shared memory provably off the
  table (cross-check numbers ≈ default numbers), FastDDS still wins **≥64KB
  decisively** on pure UDP: ~0.7 ms vs ~18 ms at 1MB, ~6 ms vs ~155 ms at 10MB.
  RustDDS 0.13's large-data UDP path, including fragmentation and flow control, is
  slower than FastDDS's; the same-host loss was never only about loopback SHM.
- roswell still competes at the **64B** end — 0.10 ms on a quiet run, matching or
  beating rclpy — but its small-message latency is **noisy** in the two-netns
  setup (0.10 → 0.72 ms median across runs, p95 spikes to 7 ms), likely VM/veth
  scheduling jitter; rclpy's small-message latency is steadier (~0.16 ms).
- Both stacks deliver **10MB with zero loss** cross-host; the 0.13 + 16 MiB
  `SO_RCVBUF` work that fixed same-host large data holds over real UDP too.
- The one-message-in-flight RTT/2 protocol removes clock-skew risk entirely at
  the cost of doubling the wire path — read these as relative, not absolute.

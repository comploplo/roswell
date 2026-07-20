#!/usr/bin/env bash
#
# Cross-host bench: roswell vs rclpy across TWO separate network namespaces.
#
# The same-host bench (scripts/bench-vs-rclpy.sh) let FastDDS use shared memory
# on loopback, which roswell's UDP path cannot match >=64KB. This script runs the
# publisher and subscriber in two SEPARATE containers on a shared podman bridge
# network: different net/PID/IPC namespaces, a real veth+bridge path, and no
# shared /dev/shm -- so FastDDS is forced onto UDP, the fair fight.
#
# Because the two halves are separate processes we can't diff a one-way stamp
# against a same-process clock, so each stack measures a full round trip through
# an echo server in the far container and reports RTT/2 (the driver times it all
# on its own clock, so cross-container clock skew is irrelevant). The identical
# RTT/2 protocol runs for both stacks.
#
# Discovery: multicast SPDP works across the podman bridge (verified below), so
# both stacks use their stock multicast discovery -- the same setting for both,
# no unicast peers needed.
#
# Usage: scripts/bench-cross-host.sh [wheel-path]
# Requires: podman (running machine) + the manylinux wheel in python/dist/.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ARCH="$(uname -m | sed s/arm64/aarch64/)"
WHEEL="${1:-$(ls "$REPO_ROOT"/python/dist/roswell-*-manylinux_2_28_"${ARCH}".whl | head -1)}"
BASE_IMAGE="docker.io/library/ros:jazzy"
BENCH_IMAGE="localhost/roswell-cross-bench:latest"
NET="roswell-bench"
BENCH="$REPO_ROOT/python/benchmarks"

echo ">> wheel: $WHEEL"
echo ">> base : $BASE_IMAGE"

# --- network: dedicated bridge (separate netns per container) ---------------
podman network exists "$NET" || podman network create "$NET" >/dev/null
echo ">> network: $NET"

# --- roswell image with the wheel baked in (cached across reruns) ------------
build_bench_image() {
  local ctx
  ctx="$(mktemp -d)"
  cp "$WHEEL" "$ctx/"
  cat >"$ctx/Containerfile" <<EOF
FROM $BASE_IMAGE
RUN apt-get update -qq >/dev/null && apt-get install -y -qq python3-pip >/dev/null
COPY $(basename "$WHEEL") /tmp/
RUN pip install -q --break-system-packages /tmp/$(basename "$WHEEL")
EOF
  podman build -q -t "$BENCH_IMAGE" "$ctx" >/dev/null
  rm -rf "$ctx"
}
if ! podman image exists "$BENCH_IMAGE"; then
  echo ">> building $BENCH_IMAGE (one-time)"; build_bench_image
fi
echo ">> bench image: $BENCH_IMAGE"

MOUNT=(-v "$BENCH":/bench:ro,Z)
cleanup() { podman rm -f xecho xmc >/dev/null 2>&1 || true; }
trap cleanup EXIT

# --- isolation proof --------------------------------------------------------
prove_isolation() {
  echo
  echo "================ ISOLATION PROOF ================"
  # Two containers on the bridge => two netns with distinct IPs on a veth path.
  podman run -d --rm --name xecho --network "$NET" "$BASE_IMAGE" sleep 60 >/dev/null
  local echo_ip drv_ip
  echo_ip=$(podman exec xecho sh -c "hostname -i" 2>/dev/null)
  drv_ip=$(podman run --rm --network "$NET" "$BASE_IMAGE" sh -c "hostname -i" 2>/dev/null)
  echo "netns: echo container IP=$echo_ip  driver container IP=$drv_ip (distinct => separate netns, veth+bridge path)"

  # Separate IPC/mount namespaces => /dev/shm is NOT shared. FastDDS SHM writes
  # segment files into /dev/shm; a peer in another container can never see them.
  podman exec xecho sh -c "touch /dev/shm/echo_only_marker"
  local seen
  seen=$(podman run --rm --network "$NET" "$BASE_IMAGE" sh -c "ls /dev/shm/echo_only_marker 2>/dev/null || echo ABSENT")
  echo "shm : marker made in echo's /dev/shm; driver container sees: $seen (ABSENT => no shared memory possible)"

  podman rm -f xecho >/dev/null 2>&1 || true

  # Multicast SPDP reachability across the bridge (both stacks rely on it). The
  # receiver is its own container's main process so `podman logs` captures it.
  podman run -d --name xmc --network "$NET" "$BASE_IMAGE" python3 -c '
import socket,struct
s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM); s.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
s.bind(("",15098)); s.setsockopt(socket.IPPROTO_IP,socket.IP_ADD_MEMBERSHIP,struct.pack("4sl",socket.inet_aton("239.255.0.98"),socket.INADDR_ANY))
s.settimeout(8)
try: print("mcast: RECEIVED from", s.recvfrom(64)[1], "(multicast SPDP works across the bridge)", flush=True)
except socket.timeout: print("mcast: TIMEOUT (multicast blocked)", flush=True)' >/dev/null
  sleep 1
  podman run --rm --network "$NET" "$BASE_IMAGE" python3 -c '
import socket,time
s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM); s.setsockopt(socket.IPPROTO_IP,socket.IP_MULTICAST_TTL,4)
[s.sendto(b"spdp",("239.255.0.98",15098)) or time.sleep(0.2) for _ in range(20)]' >/dev/null 2>&1 || true
  podman wait --condition stopped xmc >/dev/null 2>&1 || true
  podman logs xmc 2>&1 | grep -E "mcast:" || echo "mcast: (no line captured)"
  podman rm -f xmc >/dev/null 2>&1 || true
  echo "================================================"
  echo
}

# --- one stack: start echo (far container), run driver (near container) -----
run_stack() {  # $1=lib  $2=image  $3=driver-cmd  $4=echo-cmd  $5..=extra env
  local lib="$1" image="$2" drv="$3" ech="$4"; shift 4
  local env=("$@")
  echo "== $lib =="
  podman run -d --rm --name xecho --network "$NET" "${env[@]+"${env[@]}"}" "${MOUNT[@]}" \
    "$image" bash -lc "set +u; source /opt/ros/jazzy/setup.bash; set -u; $ech" >/dev/null
  sleep 3  # let the echo node come up before the driver probes discovery
  podman run --rm --network "$NET" "${env[@]+"${env[@]}"}" "${MOUNT[@]}" \
    "$image" bash -lc "set +u; source /opt/ros/jazzy/setup.bash; set -u; $drv"
  podman rm -f xecho >/dev/null 2>&1 || true
}

prove_isolation

# rclpy: stock FastDDS, forced onto UDP by the namespace split.
run_stack rclpy "$BASE_IMAGE" \
  "python3 /bench/bench_cross_rclpy.py driver" \
  "python3 /bench/bench_cross_rclpy.py echo" \
  -e ROS_DOMAIN_ID=31

# Cross-check: rerun rclpy with SHM explicitly disabled via a FastDDS profile.
# If the numbers match the run above, SHM was already out of play (proof the
# namespace split -- not our profile -- is what removed it).
XML=/bench/fastdds_no_shm.xml
run_stack "rclpy (SHM explicitly off, cross-check)" "$BASE_IMAGE" \
  "python3 /bench/bench_cross_rclpy.py driver" \
  "python3 /bench/bench_cross_rclpy.py echo" \
  -e ROS_DOMAIN_ID=32 \
  -e FASTRTPS_DEFAULT_PROFILES_FILE="$XML" \
  -e RMW_FASTRTPS_USE_QOS_FROM_XML=0

# roswell: rustdds 0.13 + 16 MiB SO_RCVBUF, UDP always.
run_stack roswell "$BENCH_IMAGE" \
  "python3 /bench/bench_cross_roswell.py driver --domain 33" \
  "python3 /bench/bench_cross_roswell.py echo --domain 33"

echo ">> done"

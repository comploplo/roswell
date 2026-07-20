"""Embedded Python ROS node — the existing `roswell` FFI wheel (pure ctypes over
the roswell Rust runtime), the Raspberry-Pi-class node in the demo.

No new protocol code: this is the shipped client. It publishes a
`geometry_msgs/msg/Twist` on `/cmd_vel` over real RTPS/DDS. The demo harness
subscribes to that topic (also via the roswell runtime), bridges each sample to
the embedded Rust firmware over the tunnel/UART, and asserts the firmware's ack.

Usage:  python wheel_node.py [duration_seconds] [rate_hz]
"""

import asyncio
import sys

import roswell

DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 90.0
RATE_HZ = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0


async def main():
    node = roswell.Node("wheel_cmd_vel_node", domain=0)
    Twist = node.load_type("geometry_msgs/msg/Twist")  # bundled — no ROS install
    pub = node.publisher("/cmd_vel", Twist)
    print("wheel_node: publishing geometry_msgs/msg/Twist on /cmd_vel", flush=True)

    period = 1.0 / RATE_HZ
    loop = asyncio.get_event_loop()
    deadline = loop.time() + DURATION
    seq = 0
    while loop.time() < deadline:
        msg = pub.new()
        msg.linear.x = 0.25
        msg.angular.z = 0.1
        pub.publish(msg)
        seq += 1
        if seq % int(RATE_HZ) == 0:
            print("wheel_node: published %d Twist sample(s)" % seq, flush=True)
        await asyncio.sleep(period)
    node.close()


asyncio.run(main())

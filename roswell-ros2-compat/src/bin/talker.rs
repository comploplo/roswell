//! Publish `std_msgs/String` on `/chatter`. Interop: `ros2 topic echo /chatter`.
//!
//!   talker                # reliable/volatile (ROS2 default)
//!   talker sensor_data    # best-effort
//!   talker latched        # transient-local; keeps the writer alive so a
//!                           late transient_local subscriber still gets a sample

use std::ffi::c_char;
use std::time::Duration;

use roswell_ros2_compat::msgs::{std_msgs__String, RosString};
use roswell_ros2_compat::transport::{Dds, MsgPublisher, Qos, Transport};

fn main() {
    let (qos, latched) = match std::env::args().nth(1).as_deref() {
        Some("sensor_data") => (Qos::SensorData, false),
        Some("latched") => (Qos::Latched, true),
        _ => (Qos::Default, false),
    };
    let dds = Dds::new(0);
    let publisher = dds.publisher::<std_msgs__String>("/chatter", qos);

    println!("talker: /chatter as std_msgs/msg/String ({qos:?})");
    for i in 0..40 {
        // Non-owning RosString backed by a stack buffer (no leak on move).
        let mut buf = format!("hello from roswell #{i}").into_bytes();
        buf.push(0);
        // SAFETY: `buf` is valid UTF-8 + trailing NUL and outlives `msg`, which is
        // published before `buf` is dropped; `capacity == 0` marks it borrowed.
        let data = unsafe {
            RosString::from_raw_parts(buf.as_mut_ptr().cast::<c_char>(), buf.len() - 1, 0)
        };
        let msg = std_msgs__String { data };
        publisher.publish(msg);
        drop(buf);
        println!("published #{i}");
        std::thread::sleep(Duration::from_millis(300));
    }

    if latched {
        // Keep the writer (and its transient_local history) alive so a
        // late-joining subscriber can still receive the last sample.
        println!("talker: holding latched sample...");
        std::thread::sleep(Duration::from_secs(25));
    }
}

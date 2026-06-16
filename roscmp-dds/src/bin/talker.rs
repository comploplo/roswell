//! Publish `std_msgs/String` on `/chatter`. Interop: `ros2 topic echo /chatter`.

use std::ffi::c_char;
use std::time::Duration;

use roscmp_dds::msgs::{std_msgs__String, RosString};
use roscmp_dds::transport::{Dds, MsgPublisher, Transport};

fn main() {
    let dds = Dds::new(0);
    let publisher = dds.publisher::<std_msgs__String>("/chatter");

    println!("talker: /chatter as std_msgs/msg/String");
    for i in 0..40 {
        // Non-owning RosString backed by a stack buffer (no leak on move).
        let mut buf = format!("hello from roscmp #{i}").into_bytes();
        buf.push(0);
        let msg = std_msgs__String {
            data: RosString {
                data: buf.as_mut_ptr().cast::<c_char>(),
                size: buf.len() - 1,
                capacity: 0,
            },
        };
        publisher.publish(msg);
        drop(buf);
        println!("published #{i}");
        std::thread::sleep(Duration::from_millis(300));
    }
}

//! Subscribe to `std_msgs/String` on `/chatter` and print what arrives.
//! Interop: `ros2 topic pub /chatter std_msgs/msg/String "{data: hi}"`.

use std::time::Duration;

use roswell_ros2_compat::msgs::std_msgs__String;
use roswell_ros2_compat::transport::{Dds, MsgSubscriber, Qos, Transport};

fn main() {
    let dds = Dds::new(0);
    let mut sub = dds.subscriber::<std_msgs__String>("/chatter", Qos::Default);

    println!("listener: waiting on /chatter (std_msgs/msg/String)");
    let mut count = 0;
    for _ in 0..300 {
        while let Some(mut msg) = sub.take() {
            println!("received: {}", msg.data.as_str());
            // SAFETY: `msg` owns `data` (from our from_cdr) and is finalized once here.
            unsafe {
                msg.fini();
            }
            count += 1;
        }
        if count >= 3 {
            break;
        }
        std::thread::sleep(Duration::from_millis(100));
    }

    if count == 0 {
        eprintln!("listener: no messages received");
        std::process::exit(1);
    }
    println!("listener: received {count} message(s)");
}

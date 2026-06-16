//! Publish `geometry_msgs/Twist` on `/cmd_vel` (teleop).
//! Interop: `ros2 topic echo /cmd_vel geometry_msgs/msg/Twist`.

use std::time::Duration;

use roscmp_dds::msgs::{geometry_msgs__Twist, geometry_msgs__Vector3};
use roscmp_dds::transport::{Dds, MsgPublisher, Transport};

fn main() {
    let dds = Dds::new(0);
    let publisher = dds.publisher::<geometry_msgs__Twist>("/cmd_vel");

    println!("teleop: driving forward + turning on /cmd_vel");
    for i in 0..40 {
        let msg = geometry_msgs__Twist {
            linear: geometry_msgs__Vector3 {
                x: 0.2,
                y: 0.0,
                z: 0.0,
            },
            angular: geometry_msgs__Vector3 {
                x: 0.0,
                y: 0.0,
                z: 0.5,
            },
        };
        publisher.publish(msg);
        println!("cmd #{i}: linear.x=0.2 angular.z=0.5");
        std::thread::sleep(Duration::from_millis(300));
    }
}

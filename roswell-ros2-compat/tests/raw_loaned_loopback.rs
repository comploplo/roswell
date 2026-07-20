//! Loopback for the loaned-buffer publish path: `publish_loaned` must deliver
//! the same bytes on the wire as `publish`, and report the filled length so
//! callers can right-size the next buffer.

use std::time::{Duration, Instant};

use roswell_ros2_compat::raw::{RawDdsPublisher, RawDdsSubscriber, RawQos};
use roswell_ros2_compat::transport::Dds;

#[test]
fn publish_loaned_round_trips_byte_exact_and_reports_len() {
    let dds = Dds::new(0);
    let topic = "/raw_loaned_loopback";
    let ros_type = "std_msgs/msg/String";
    let publisher = RawDdsPublisher::new(dds.participant(), topic, ros_type, RawQos::Default);
    let mut subscriber = RawDdsSubscriber::new(dds.participant(), topic, ros_type, RawQos::Default);

    // Same padding-free CDR_LE shape as raw_record_loopback.
    let cdr: Vec<u8> = vec![0x00, 0x01, 0x00, 0x00, 4, 0, 0, 0, b'h', b'i', b'!', 0];

    let deadline = Instant::now() + Duration::from_secs(10);
    let mut got = None;
    while got.is_none() && Instant::now() < deadline {
        let len = publisher.publish_loaned(cdr.len(), |buf| buf.extend_from_slice(&cdr));
        assert_eq!(len, cdr.len(), "publish_loaned must report bytes written");
        got = subscriber.take();
        if got.is_none() {
            std::thread::sleep(Duration::from_millis(50));
        }
    }
    let msg = got.expect("no loaned sample received on loopback within timeout");
    assert_eq!(msg.ros_type(), ros_type);
    assert_eq!(
        msg.cdr(),
        cdr.as_slice(),
        "loaned publish must be byte-exact"
    );
}

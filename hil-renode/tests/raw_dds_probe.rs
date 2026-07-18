//! Diagnostic: does the `RawDdsSubscriber` used by `tcp_topic_bridge` /
//! `usb_topic_bridge` actually receive on THIS host? The FFI-wheel demo node
//! publishes typed; the bridge subscribes raw, so the live demo depends on
//! typed-pub -> raw-sub working over real RTPS/UDP.
//!
//! `#[ignore]`d (opens real DDS sockets, ~seconds). Run:
//!   cargo test -p roscmp-hil --test raw_dds_probe -- --ignored --nocapture

use std::ffi::c_char;
use std::time::{Duration, Instant};

use roscmp_dds::msgs::{std_msgs__String, RosString};
use roscmp_dds::raw::{raw_qos_for_topic, RawDdsPublisher, RawDdsSubscriber, RawMsg};
use roscmp_dds::transport::{Dds, MsgPublisher, Qos, Transport};

fn wait_for_sample(
    sub: &mut RawDdsSubscriber,
    publish: impl Fn(),
    budget: Duration,
) -> Option<RawMsg> {
    let deadline = Instant::now() + budget;
    while Instant::now() < deadline {
        publish();
        if let Some(msg) = sub.take() {
            return Some(msg);
        }
        std::thread::sleep(Duration::from_millis(100));
    }
    None
}

#[test]
#[ignore = "opens real DDS sockets; run with --ignored"]
fn raw_subscriber_receives_from_raw_publisher() {
    let dds_pub = Dds::new(0);
    let dds_sub = Dds::new(0);
    let pubr = RawDdsPublisher::new(
        dds_pub.participant(),
        "/raw_probe",
        "std_msgs/msg/String",
        raw_qos_for_topic("/raw_probe"),
    );
    let mut sub = RawDdsSubscriber::new(
        dds_sub.participant(),
        "/raw_probe",
        "std_msgs/msg/String",
        raw_qos_for_topic("/raw_probe"),
    );
    let cdr = b"\x00\x01\x00\x00\x04\x00\x00\x00hi\x00\x00".to_vec();
    let got = wait_for_sample(
        &mut sub,
        || pubr.publish(&RawMsg::new("std_msgs/msg/String", cdr.clone())),
        Duration::from_secs(20),
    );
    println!("raw->raw: {got:?}");
    assert!(got.is_some(), "raw sub never received from raw pub");
}

#[test]
#[ignore = "opens real DDS sockets; run with --ignored"]
fn raw_subscriber_receives_from_typed_publisher() {
    let dds_pub = Dds::new(0);
    let dds_sub = Dds::new(0);
    let publisher = dds_pub.publisher::<std_msgs__String>("/typed_probe", Qos::Default);
    let mut sub = RawDdsSubscriber::new(
        dds_sub.participant(),
        "/typed_probe",
        "std_msgs/msg/String",
        raw_qos_for_topic("/typed_probe"),
    );

    let publish = || {
        let mut buf = b"hi from typed\x00".to_vec();
        let data = unsafe {
            RosString::from_raw_parts(buf.as_mut_ptr().cast::<c_char>(), buf.len() - 1, 0)
        };
        publisher.publish(std_msgs__String { data });
        drop(buf);
    };
    let got = wait_for_sample(&mut sub, publish, Duration::from_secs(20));
    println!("typed->raw: {got:?}");
    assert!(got.is_some(), "raw sub never received from typed pub");
}

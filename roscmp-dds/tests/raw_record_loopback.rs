//! Live loopback for the type-blind record path: a raw DDS publisher feeds a
//! raw subscriber, whose samples are written through the chunked/compressed
//! `McapWriter` and read back with `RawSampleReader`.

use std::time::{Duration, Instant};

use roscmp_dds::raw::{
    Compression, McapWriter, RawDdsPublisher, RawDdsSubscriber, RawMsg, RawQos, RawSampleReader,
    RawSink,
};
use roscmp_dds::transport::Dds;

#[test]
fn records_live_raw_samples_to_compressed_mcap() {
    let dds = Dds::new(0);
    let topic = "/raw_record_loopback";
    let ros_type = "std_msgs/msg/String";
    let publisher = RawDdsPublisher::new(dds.participant(), topic, ros_type, RawQos::Default);
    let mut subscriber = RawDdsSubscriber::new(dds.participant(), topic, ros_type, RawQos::Default);

    // CDR_LE payload: 4-byte encapsulation header + a body whose length is a
    // multiple of 4, so DDS adds no alignment padding and the raw round-trip
    // reproduces the exact bytes we publish.
    let cdr: Vec<u8> = vec![0x00, 0x01, 0x00, 0x00, 4, 0, 0, 0, b'h', b'i', b'!', 0];
    let msg = RawMsg::new(ros_type, cdr.clone());

    let mut writer = McapWriter::with_compression(Vec::new(), Compression::Lz4).unwrap();
    let deadline = Instant::now() + Duration::from_secs(10);
    let mut recorded = 0;
    while recorded < 3 && Instant::now() < deadline {
        publisher.publish(&msg);
        while let Some(got) = subscriber.take() {
            writer
                .write(topic, 1_000 + i64::from(recorded), &got)
                .unwrap();
            recorded += 1;
        }
        std::thread::sleep(Duration::from_millis(50));
    }
    assert!(
        recorded > 0,
        "no raw samples received on loopback within timeout"
    );

    let bytes = writer.finish().unwrap();
    let samples: Vec<_> = RawSampleReader::from_bytes(bytes)
        .unwrap()
        .map(Result::unwrap)
        .collect();
    assert_eq!(samples.len(), recorded as usize);
    for sample in &samples {
        assert_eq!(sample.topic, topic);
        assert_eq!(sample.msg.ros_type(), ros_type);
        assert_eq!(sample.msg.cdr(), cdr.as_slice());
    }
}

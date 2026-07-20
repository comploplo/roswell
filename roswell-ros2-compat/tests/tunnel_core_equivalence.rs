//! Byte-for-byte equivalence between the std `roswell_ros2_compat::tunnel` codec and the
//! `no_std` `roswell_tunnel_core` codec that a no-DDS MCU uses.
//!
//! This is the twin-implementation pin (in the spirit of the `roswell_verify`
//! equivalence tests): frames the host writes must parse in the MCU core, and
//! frames the MCU core writes must parse in the host — so firmware built on the
//! extracted core and the host bridge cannot silently drift apart.

use std::io::Cursor;

use roswell_ros2_compat::raw::RawMsg;
use roswell_ros2_compat::tunnel::{read_frame, write_frame, TunnelFrame};
use roswell_tunnel_core as wire;

/// Frames covering every wire variant and a range of field values.
fn sample_frames() -> Vec<TunnelFrame> {
    vec![
        TunnelFrame::Hello {
            peer: "roswell-mcu".into(),
        },
        TunnelFrame::TopicSample {
            sequence: 0x0102_0304_0506_0708,
            topic: "/cmd_vel".into(),
            stamp_nanos: -7,
            msg: RawMsg::new("geometry_msgs/msg/Twist", vec![9, 8, 7, 6]),
        },
        TunnelFrame::TopicSample {
            sequence: 1,
            topic: "/tf".into(),
            stamp_nanos: i64::MAX,
            msg: RawMsg::new("tf2_msgs/msg/TFMessage", Vec::new()),
        },
        TunnelFrame::Ack {
            sequence: 0xDEAD_BEEF_0000_0001,
        },
        TunnelFrame::Heartbeat {
            stamp_nanos: 1_234_567_890,
        },
    ]
}

/// Host `write_frame` output must parse identically through the MCU core.
#[test]
fn host_encoded_frames_parse_in_the_mcu_core() {
    for frame in sample_frames() {
        let mut bytes = Vec::new();
        write_frame(&mut bytes, &frame).unwrap();

        let header: &[u8; wire::HEADER_LEN] = bytes[..wire::HEADER_LEN].try_into().unwrap();
        let len = wire::frame_len(header).unwrap();
        assert_eq!(
            len,
            bytes.len() - wire::HEADER_LEN,
            "core frame length disagrees with host framing"
        );
        let parsed = wire::parse_payload(&bytes[wire::HEADER_LEN..]).unwrap();
        assert_matches_host(&frame, &parsed);
    }
}

/// MCU-core `encode_*` output must parse identically through the host codec.
#[test]
fn mcu_core_encoded_frames_parse_in_the_host() {
    let mut buf = [0u8; 512];

    let n = wire::encode_hello(&mut buf, "roswell-mcu").unwrap();
    assert_eq!(
        read_frame(&mut Cursor::new(&buf[..n])).unwrap(),
        TunnelFrame::Hello {
            peer: "roswell-mcu".into(),
        }
    );

    let n = wire::encode_ack(&mut buf, 0xDEAD_BEEF_0000_0001).unwrap();
    assert_eq!(
        read_frame(&mut Cursor::new(&buf[..n])).unwrap(),
        TunnelFrame::Ack {
            sequence: 0xDEAD_BEEF_0000_0001,
        }
    );

    let n = wire::encode_heartbeat(&mut buf, -99).unwrap();
    assert_eq!(
        read_frame(&mut Cursor::new(&buf[..n])).unwrap(),
        TunnelFrame::Heartbeat { stamp_nanos: -99 }
    );

    // TopicSample (borrows the buffer as it encodes).
    let n = wire::encode_topic_sample(
        &mut buf,
        42,
        "/diagnostics",
        "diagnostic_msgs/msg/DiagnosticArray",
        7,
        &[0, 1, 2, 3],
    )
    .unwrap();
    let back = read_frame(&mut Cursor::new(&buf[..n])).unwrap();
    assert_eq!(
        back,
        TunnelFrame::TopicSample {
            sequence: 42,
            topic: "/diagnostics".into(),
            stamp_nanos: 7,
            msg: RawMsg::new("diagnostic_msgs/msg/DiagnosticArray", vec![0, 1, 2, 3]),
        }
    );
}

/// Encoding the same logical frame with both codecs yields identical bytes.
#[test]
fn both_codecs_emit_identical_bytes() {
    let mut host = Vec::new();
    write_frame(
        &mut host,
        &TunnelFrame::Ack {
            sequence: 0x1122_3344_5566_7788,
        },
    )
    .unwrap();

    let mut core = [0u8; 64];
    let n = wire::encode_ack(&mut core, 0x1122_3344_5566_7788).unwrap();

    assert_eq!(host, &core[..n], "host and MCU core emit different bytes");
}

fn assert_matches_host(host: &TunnelFrame, core: &wire::FrameRef<'_>) {
    match (host, core) {
        (TunnelFrame::Hello { peer }, wire::FrameRef::Hello { peer: p }) => assert_eq!(peer, p),
        (
            TunnelFrame::TopicSample {
                sequence,
                topic,
                stamp_nanos,
                msg,
            },
            wire::FrameRef::TopicSample {
                sequence: s,
                topic: t,
                ros_type,
                stamp_nanos: st,
                cdr,
            },
        ) => {
            assert_eq!(sequence, s);
            assert_eq!(topic, t);
            assert_eq!(stamp_nanos, st);
            assert_eq!(msg.ros_type(), *ros_type);
            assert_eq!(msg.cdr(), *cdr);
        }
        (TunnelFrame::Ack { sequence }, wire::FrameRef::Ack { sequence: s }) => {
            assert_eq!(sequence, s);
        }
        (TunnelFrame::Heartbeat { stamp_nanos }, wire::FrameRef::Heartbeat { stamp_nanos: s }) => {
            assert_eq!(stamp_nanos, s);
        }
        (h, c) => panic!("variant mismatch: host {h:?} vs core {c:?}"),
    }
}

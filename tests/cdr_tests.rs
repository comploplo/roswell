//! CDR codegen conformance: compile the generated Rust bindings, then check
//! that serialized bytes match hand-computed CDR (XCDR1) vectors and that
//! serialize→deserialize round-trips.
//!
//! These vectors are derived from the OMG CDR spec; the byte-for-byte match
//! against a live `ros:jazzy` node is M2.4's final step (needs Docker).

use std::process::Command;

use roscmp::codegen;
use roscmp::ir::MsgId;
use roscmp::{parse_message, resolve};

/// Generate bindings for `defs`, append `main_body`, compile, run, return stdout.
fn run_generated(defs: &[(&str, &str, &str)], main_body: &str, tag: &str) -> String {
    let inputs = defs
        .iter()
        .map(|(pkg, name, src)| (MsgId::new(*pkg, *name), parse_message(src).expect("parse")))
        .collect();
    let program = resolve(inputs).expect("resolve");
    let mut code = codegen::rust::generate(&program);
    code.push_str("\nfn main() {\n");
    code.push_str(main_body);
    code.push_str("\n}\n");

    let dir = std::env::temp_dir().join(format!("roscmp_cdr_{tag}"));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    let src = dir.join("gen.rs");
    std::fs::write(&src, &code).unwrap();
    let bin = dir.join("bin");

    let out = Command::new("rustc")
        .args(["--edition", "2021", "-O"])
        .arg(&src)
        .arg("-o")
        .arg(&bin)
        .output()
        .expect("run rustc");
    assert!(
        out.status.success(),
        "generated Rust failed to compile:\n{}",
        String::from_utf8_lossy(&out.stderr)
    );

    let run = Command::new(&bin).output().expect("run generated bin");
    assert!(
        run.status.success(),
        "generated bin failed:\n{}",
        String::from_utf8_lossy(&run.stderr)
    );
    String::from_utf8(run.stdout).unwrap()
}

const POINT: (&str, &str, &str) = (
    "geometry_msgs",
    "Point",
    "float64 x\nfloat64 y\nfloat64 z\n",
);

#[test]
fn point_serializes_to_expected_bytes() {
    let body = r#"
    let p = geometry_msgs__Point { x: 1.0, y: 2.0, z: 3.0 };
    let b = p.to_cdr(Endian::Little);
    println!("{:02x?}", b);
    "#;
    let out = run_generated(&[POINT], body, "point");
    // 4-byte CDR_LE header + 3×f64 (no leading pad).
    let expected = "[00, 01, 00, 00, \
        00, 00, 00, 00, 00, 00, f0, 3f, \
        00, 00, 00, 00, 00, 00, 00, 40, \
        00, 00, 00, 00, 00, 00, 08, 40]";
    assert_eq!(out.trim(), expected);
}

#[test]
fn header_serializes_to_expected_bytes() {
    let body = r#"
    let h = std_msgs__Header {
        stamp: builtin_interfaces__Time { sec: 1, nanosec: 2 },
        frame_id: RosString::alloc("map"),
    };
    let b = h.to_cdr(Endian::Little);
    println!("{:02x?}", &b[4..]);
    let mut h = h;
    unsafe { h.frame_id.free(); }
    "#;
    let out = run_generated(
        &[(
            "std_msgs",
            "Header",
            "builtin_interfaces/Time stamp\nstring frame_id\n",
        )],
        body,
        "header",
    );
    // sec=1, nanosec=2, frame_id len=4 "map\0".
    let expected = "[01, 00, 00, 00, 02, 00, 00, 00, 04, 00, 00, 00, 6d, 61, 70, 00]";
    assert_eq!(out.trim(), expected);
}

#[test]
fn uint8_sequence_serializes_with_count_prefix() {
    let body = r#"
    let m = demo_msgs__Bytes { data: RosSequence::alloc(vec![1u8, 2, 3]) };
    let b = m.to_cdr(Endian::Little);
    println!("{:02x?}", &b[4..]);
    let mut m = m;
    unsafe { m.fini(); }
    "#;
    let out = run_generated(&[("demo_msgs", "Bytes", "uint8[] data\n")], body, "byteseq");
    // count=3 then the three bytes.
    assert_eq!(out.trim(), "[03, 00, 00, 00, 01, 02, 03]");
}

#[test]
fn jazzy_verified_wire_bytes() {
    // These exact bytes were captured from a real ROS2 `ros:jazzy` node via
    // `rclpy.serialization.serialize_message` on 2026-06-14 and matched ours
    // byte-for-byte (encapsulation header + body). Locking them in as a
    // regression independent of Docker availability.
    let body = r#"
    let t = builtin_interfaces__Time { sec: 1, nanosec: 2 };
    println!("{}", t.to_cdr(Endian::Little).iter().map(|b| format!("{:02x}", b)).collect::<String>());
    let mut h = std_msgs__Header {
        stamp: builtin_interfaces__Time { sec: 1, nanosec: 2 },
        frame_id: RosString::alloc("map"),
    };
    println!("{}", h.to_cdr(Endian::Little).iter().map(|b| format!("{:02x}", b)).collect::<String>());
    unsafe { h.fini(); }
    "#;
    let out = run_generated(
        &[(
            "std_msgs",
            "Header",
            "builtin_interfaces/Time stamp\nstring frame_id\n",
        )],
        body,
        "jazzy",
    );
    let mut lines = out.lines();
    assert_eq!(lines.next().unwrap(), "000100000100000002000000");
    assert_eq!(
        lines.next().unwrap(),
        "000100000100000002000000040000006d617000"
    );
}

// ---- XCDR2 (PLAIN_CDR2) wire vectors ------------------------------------
//
// For a @final struct (every ROS2 type) PLAIN_CDR2 differs from classic CDR
// only by the `00 07`/`00 06` encapsulation id and by capping 8-byte-primitive
// alignment at 4. These vectors are hand-derived from OMG DDS-XTypes 1.3
// §7.4.3.4 and pin the alignment delta byte-for-byte.

#[test]
fn xcdr2_time_matches_hand_vector() {
    // No 8-byte fields, so the body equals classic CDR; only the header changes.
    let body = r#"
    let t = builtin_interfaces__Time { sec: 1, nanosec: 2 };
    println!("{}", t.to_cdr_xcdr2(Endian::Little).iter().map(|b| format!("{:02x}", b)).collect::<String>());
    "#;
    let out = run_generated(
        &[("builtin_interfaces", "Time", "int32 sec\nuint32 nanosec\n")],
        body,
        "xcdr2_time",
    );
    // header 00 07 00 00, then sec=1, nanosec=2.
    assert_eq!(out.trim(), "000700000100000002000000");
}

#[test]
fn xcdr2_string_matches_hand_vector() {
    let body = r#"
    let mut m = std_msgs__String { data: RosString::alloc("hi") };
    println!("{}", m.to_cdr_xcdr2(Endian::Little).iter().map(|b| format!("{:02x}", b)).collect::<String>());
    unsafe { m.fini(); }
    "#;
    let out = run_generated(
        &[("std_msgs", "String", "string data\n")],
        body,
        "xcdr2_str",
    );
    // header, len=3 (incl NUL), 'h' 'i' NUL.
    assert_eq!(out.trim(), "0007000003000000686900");
}

#[test]
fn xcdr2_eight_byte_primitive_aligns_to_four() {
    // {uint8 flag; float64 value; int64 count}: under XCDR2 `value` lands at
    // body offset 4 (not 8) and `count` at 12 (not 16) — a 20-byte body vs the
    // 24-byte XCDR1 body. This is the entire XCDR2 wire delta, proven exactly.
    let src = "uint8 flag\nfloat64 value\nint64 count\n";
    let body = r#"
    let m = demo_msgs__Wide { flag: 1, value: 2.0, count: 3 };
    let x2 = m.to_cdr_xcdr2(Endian::Little);
    let x1 = m.to_cdr(Endian::Little);
    println!("{}", x2.iter().map(|b| format!("{:02x}", b)).collect::<String>());
    println!("{} {}", x1.len(), x2.len());
    "#;
    let out = run_generated(&[("demo_msgs", "Wide", src)], body, "xcdr2_wide");
    let mut lines = out.lines();
    // header 00 07 00 00 | flag=1 +3pad | value=2.0 @off4 | count=3 @off12.
    assert_eq!(
        lines.next().unwrap(),
        "00070000\
         01000000\
         0000000000000040\
         0300000000000000"
            .replace(' ', "")
    );
    // XCDR1 body is 24 bytes (+4 header), XCDR2 is 20 (+4 header).
    assert_eq!(lines.next().unwrap(), "28 24");
}

#[test]
fn xcdr2_nested_sequence_aligns_to_four() {
    // A sequence of Point (3×f64): under XCDR2 the first f64 sits right after the
    // 4-byte length (offset 4), with no 8-byte pad. Proves nested + sequence
    // element alignment follows the max-align-4 rule.
    let body = r#"
    let m = demo_msgs__Cloud {
        pts: RosSequence::alloc(vec![geometry_msgs__Point { x: 1.0, y: 2.0, z: 3.0 }]),
    };
    let x2 = m.to_cdr_xcdr2(Endian::Little);
    println!("{}", x2.iter().map(|b| format!("{:02x}", b)).collect::<String>());
    println!("{} {}", m.to_cdr(Endian::Little).len(), x2.len());
    let mut m = m; unsafe { m.fini(); }
    "#;
    let out = run_generated(
        &[POINT, ("demo_msgs", "Cloud", "geometry_msgs/Point[] pts\n")],
        body,
        "xcdr2_cloud",
    );
    let mut lines = out.lines();
    assert_eq!(
        lines.next().unwrap(),
        "00070000\
         01000000\
         000000000000f03f\
         0000000000000040\
         0000000000000840"
    );
    // XCDR1 pads 4 bytes after the length before the first f64: 32 vs 28 body.
    assert_eq!(lines.next().unwrap(), "36 32");
}

#[test]
fn xcdr2_round_trips_through_from_cdr() {
    // `from_cdr` auto-detects the encapsulation id, so an XCDR2 payload decodes
    // with no extra entry point and re-encodes identically.
    let body = r#"
    let m = demo_msgs__Wide { flag: 5, value: -1.5, count: -42 };
    let bytes = m.to_cdr_xcdr2(Endian::Little);
    let back = demo_msgs__Wide::from_cdr(&bytes).unwrap();
    let ok = back.flag == 5 && back.value == -1.5 && back.count == -42;
    let reser = back.to_cdr_xcdr2(Endian::Little);
    println!("{} {}", ok, reser == bytes);
    "#;
    let out = run_generated(
        &[(
            "demo_msgs",
            "Wide",
            "uint8 flag\nfloat64 value\nint64 count\n",
        )],
        body,
        "xcdr2_roundtrip",
    );
    assert_eq!(out.trim(), "true true");
}

#[test]
fn complex_message_round_trips() {
    // Exercises scalars, strings, fixed arrays, sequences, and nested messages.
    let src = "\
std_msgs/Header header
uint8 state
string name
float64[3] xyz
int32[] readings
geometry_msgs/Point[] waypoints
";
    let body = r#"
    let m = demo_msgs__Telemetry {
        header: std_msgs__Header {
            stamp: builtin_interfaces__Time { sec: 7, nanosec: 8 },
            frame_id: RosString::alloc("odom"),
        },
        state: 2,
        name: RosString::alloc("turtle"),
        xyz: [0.5, 1.5, 2.5],
        readings: RosSequence::alloc(vec![-1i32, 0, 1, 100]),
        waypoints: RosSequence::alloc(vec![
            geometry_msgs__Point { x: 1.0, y: 2.0, z: 3.0 },
            geometry_msgs__Point { x: 4.0, y: 5.0, z: 6.0 },
        ]),
    };
    let bytes = m.to_cdr(Endian::Little);
    let mut back = demo_msgs__Telemetry::from_cdr(&bytes).unwrap();
    unsafe {
        let ok = back.header.stamp.sec == 7
            && back.header.stamp.nanosec == 8
            && back.header.frame_id.as_str() == "odom"
            && back.state == 2
            && back.name.as_str() == "turtle"
            && back.xyz == [0.5, 1.5, 2.5]
            && back.readings.as_slice() == [-1, 0, 1, 100]
            && back.waypoints.size == 2
            && back.waypoints.as_slice()[1].y == 5.0;
        // Re-serializing the decoded message must reproduce the bytes exactly.
        let reser = back.to_cdr(Endian::Little);
        println!("{} {}", ok, reser == bytes);
        let mut m = m;
        m.fini();
        back.fini();
    }
    "#;
    let out = run_generated(&[POINT, ("demo_msgs", "Telemetry", src)], body, "complex");
    assert_eq!(out.trim(), "true true");
}

#[test]
fn sensor_msgs_imu_round_trips() {
    // A real sensor_msgs type: nested Header, two nested Vector3/Quaternion, and
    // three float64[9] covariance arrays.
    let imu = "\
std_msgs/Header header
geometry_msgs/Quaternion orientation
float64[9] orientation_covariance
geometry_msgs/Vector3 angular_velocity
float64[9] angular_velocity_covariance
geometry_msgs/Vector3 linear_acceleration
float64[9] linear_acceleration_covariance
";
    let body = r#"
    let cov: [f64; 9] = [0.1, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 0.3];
    let m = sensor_msgs__Imu {
        header: std_msgs__Header {
            stamp: builtin_interfaces__Time { sec: 10, nanosec: 11 },
            frame_id: RosString::alloc("imu_link"),
        },
        orientation: geometry_msgs__Quaternion { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
        orientation_covariance: cov,
        angular_velocity: geometry_msgs__Vector3 { x: 0.1, y: 0.2, z: 0.3 },
        angular_velocity_covariance: cov,
        linear_acceleration: geometry_msgs__Vector3 { x: 9.8, y: 0.0, z: 0.0 },
        linear_acceleration_covariance: cov,
    };
    let bytes = m.to_cdr(Endian::Little);
    let mut back = sensor_msgs__Imu::from_cdr(&bytes).unwrap();
    unsafe {
        let ok = back.header.stamp.sec == 10
            && back.header.frame_id.as_str() == "imu_link"
            && back.orientation.w == 1.0
            && back.orientation_covariance == cov
            && back.angular_velocity.z == 0.3
            && back.linear_acceleration.x == 9.8
            && back.linear_acceleration_covariance[8] == 0.3;
        let reser = back.to_cdr(Endian::Little);
        println!("{} {}", ok, reser == bytes);
        let mut m = m;
        m.fini();
        back.fini();
    }
    "#;
    let out = run_generated(
        &[
            (
                "geometry_msgs",
                "Quaternion",
                "float64 x\nfloat64 y\nfloat64 z\nfloat64 w\n",
            ),
            (
                "geometry_msgs",
                "Vector3",
                "float64 x\nfloat64 y\nfloat64 z\n",
            ),
            ("sensor_msgs", "Imu", imu),
        ],
        body,
        "imu",
    );
    assert_eq!(out.trim(), "true true");
}

#[test]
fn hostile_sequence_length_errors_without_aborting() {
    // A tiny packet claiming a 0xFFFFFFFF-element sequence must fail cleanly,
    // not pre-allocate a multi-GB Vec and abort. If decoding aborted, the
    // generated binary would exit non-zero and `run_generated` would panic.
    let body = r#"
    // LE encapsulation header, then seq_len = 0xFFFFFFFF, then no payload.
    let bytes: [u8; 8] = [0x00, 0x01, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF];
    let got = matches!(demo_msgs__Readings::from_cdr(&bytes), Err(CdrError::Truncated));
    println!("{got}");
    "#;
    let out = run_generated(
        &[("demo_msgs", "Readings", "int32[] values\n")],
        body,
        "hostile_seq",
    );
    assert_eq!(out.trim(), "true");
}

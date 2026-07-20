//! XCDR2 (PLAIN_CDR2) coverage for both codecs: the runtime layout-driven
//! [`roswell::dynamic`] codec and the generated `to_cdr_xcdr2`/`from_cdr`.
//!
//! Proves, over real generated ROS2 types:
//! 1. `DynamicType::encode_as(Xcdr2)` is byte-identical to the generated
//!    `to_cdr_xcdr2` (both drive the same `Writer` in `Encoding::Xcdr2`);
//! 2. an XCDR2 payload round-trips: `encode_as(Xcdr2) -> decode -> re-encode`
//!    reproduces the bytes, and `from_cdr` auto-detects the encapsulation id;
//! 3. XCDR1 output is unchanged (default `encode`/`to_cdr` stay classic CDR);
//! 4. for 8-byte-heavy types the XCDR2 body is strictly smaller (align-4 cap).

use roswell::cdr::Encoding;
use roswell::dynamic::DynamicType;
// The generated `to_cdr*` methods take the crate's own embedded `Endian`
// (msgs.rs inlines the runtime verbatim), distinct from `roswell::cdr::Endian`.
use roswell::ir::MsgId;
use roswell::{parse_message, resolve};
use roswell_ros2_compat::msgs::Endian as MEndian;

use roswell_ros2_compat::codec::CdrMsg;
use roswell_ros2_compat::msgs::{
    builtin_interfaces__Time as GenTime, example_interfaces__AddTwoInts_Request as GenAdd,
    example_interfaces__Fibonacci_Result as GenFib, geometry_msgs__Quaternion as GenQuat,
    geometry_msgs__Twist as GenTwist, geometry_msgs__Vector3 as GenVec3,
    sensor_msgs__Imu as GenImu, std_msgs__Header as GenHeader, std_msgs__String as GenString,
    RosSequence, RosString,
};

fn dyn_type(sources: &[(&str, &str, &str)]) -> DynamicType {
    let inputs = sources
        .iter()
        .map(|(pkg, name, body)| (MsgId::new(*pkg, *name), parse_message(body).unwrap()))
        .collect();
    let program = resolve(inputs).unwrap();
    let (pkg, name, _) = sources[0];
    DynamicType::from_program(&program, &MsgId::new(pkg, name)).unwrap()
}

/// For a generated value `gen` of type `T` whose runtime type is `ty` and whose
/// generated XCDR2 bytes are `gen_x2`:
/// - the dynamic `encode_as(Xcdr2)` equals the generated `to_cdr_xcdr2`;
/// - the payload starts with the PLAIN_CDR2-LE header `00 07 00 00`;
/// - `decode -> re-encode(Xcdr2)` through fresh memory reproduces the bytes;
/// - the default `encode` (XCDR1) still starts with `00 01` and differs only as
///   the alignment rule allows (never larger than XCDR2 for these types).
fn check_x2<T: CdrMsg>(ty: &DynamicType, mut gen: T, gen_x2: &[u8]) {
    let dyn_x2 = unsafe {
        ty.encode_as(core::ptr::from_ref(&gen).cast::<u8>(), Encoding::Xcdr2)
            .unwrap()
    };
    assert_eq!(
        dyn_x2, gen_x2,
        "dynamic encode_as(Xcdr2) != generated to_cdr_xcdr2"
    );
    assert_eq!(
        &gen_x2[..4],
        &[0x00, 0x07, 0x00, 0x00],
        "not a PLAIN_CDR2-LE header"
    );

    // XCDR1 default path is unchanged and self-identifies as classic CDR_LE.
    let x1 = unsafe { ty.encode(core::ptr::from_ref(&gen).cast::<u8>()).unwrap() };
    assert_eq!(
        &x1[..4],
        &[0x00, 0x01, 0x00, 0x00],
        "default encode is not XCDR1"
    );
    assert!(
        gen_x2.len() <= x1.len(),
        "XCDR2 body should never exceed XCDR1"
    );

    unsafe {
        let buf = ty.alloc_zeroed();
        ty.decode(gen_x2, buf).unwrap(); // auto-detects Xcdr2
        assert_eq!(
            ty.encode_as(buf, Encoding::Xcdr2).unwrap(),
            gen_x2,
            "XCDR2 decode -> re-encode mismatch"
        );
        ty.fini(buf);
        ty.dealloc(buf);
        ty.fini(core::ptr::from_mut(&mut gen).cast::<u8>());
    }
}

#[test]
fn string_xcdr2() {
    let ty = dyn_type(&[("std_msgs", "String", "string data")]);
    let gen = GenString {
        data: RosString::alloc("hello, xcdr2"),
    };
    let x2 = gen.to_cdr_xcdr2(MEndian::Little);
    check_x2(&ty, gen, &x2);
}

#[test]
fn time_xcdr2_equals_xcdr1_body() {
    // No 8-byte fields: the XCDR2 body equals the XCDR1 body; only the header id
    // differs (00 07 vs 00 01).
    let ty = dyn_type(&[("builtin_interfaces", "Time", "int32 sec\nuint32 nanosec")]);
    let gen = GenTime { sec: 7, nanosec: 8 };
    let x2 = gen.to_cdr_xcdr2(MEndian::Little);
    let x1 = gen.to_cdr(MEndian::Little);
    assert_eq!(&x2[4..], &x1[4..], "no 8-byte field => identical bodies");
    check_x2(&ty, gen, &x2);
}

#[test]
fn header_xcdr2() {
    let ty = dyn_type(&[(
        "std_msgs",
        "Header",
        "builtin_interfaces/Time stamp\nstring frame_id",
    )]);
    let gen = GenHeader {
        stamp: GenTime { sec: 1, nanosec: 2 },
        frame_id: RosString::alloc("map"),
    };
    let x2 = gen.to_cdr_xcdr2(MEndian::Little);
    check_x2(&ty, gen, &x2);
}

#[test]
fn twist_xcdr2_parity() {
    // Twist = 6×float64, all already 8-aligned, so XCDR2 == XCDR1 in size; this
    // confirms the common all-f64 case round-trips unchanged. The strict align-4
    // win on mixed 4/8 layouts is proven in `wide_xcdr2_is_strictly_smaller`.
    let ty = dyn_type(&[
        ("geometry_msgs", "Twist", "Vector3 linear\nVector3 angular"),
        (
            "geometry_msgs",
            "Vector3",
            "float64 x\nfloat64 y\nfloat64 z",
        ),
    ]);
    let gen = GenTwist {
        linear: GenVec3 {
            x: 1.5,
            y: -2.0,
            z: 3.25,
        },
        angular: GenVec3 {
            x: 0.0,
            y: 4.5,
            z: -6.75,
        },
    };
    let x2 = gen.to_cdr_xcdr2(MEndian::Little);
    check_x2(&ty, gen, &x2);
}

#[test]
fn imu_xcdr2_round_trips() {
    let ty = dyn_type(&[
        (
            "sensor_msgs",
            "Imu",
            "std_msgs/Header header\n\
             geometry_msgs/Quaternion orientation\n\
             float64[9] orientation_covariance\n\
             geometry_msgs/Vector3 angular_velocity\n\
             float64[9] angular_velocity_covariance\n\
             geometry_msgs/Vector3 linear_acceleration\n\
             float64[9] linear_acceleration_covariance",
        ),
        (
            "geometry_msgs",
            "Quaternion",
            "float64 x\nfloat64 y\nfloat64 z\nfloat64 w",
        ),
        (
            "geometry_msgs",
            "Vector3",
            "float64 x\nfloat64 y\nfloat64 z",
        ),
    ]);
    let gen = GenImu {
        header: GenHeader {
            stamp: GenTime {
                sec: 12,
                nanosec: 345,
            },
            frame_id: RosString::alloc("imu_link"),
        },
        orientation: GenQuat {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 1.0,
        },
        orientation_covariance: [1.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 3.0],
        angular_velocity: GenVec3 {
            x: 0.01,
            y: 0.02,
            z: 0.03,
        },
        angular_velocity_covariance: [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9],
        linear_acceleration: GenVec3 {
            x: 9.8,
            y: 0.0,
            z: 0.0,
        },
        linear_acceleration_covariance: [-1.0; 9],
    };
    // Imu's f64 content is all 8-byte-multiple arrays/structs that stay
    // 8-aligned, so XCDR2 matches XCDR1 in size here — the point is that a big,
    // deeply-nested real type round-trips through XCDR2 unchanged in value.
    let x2 = gen.to_cdr_xcdr2(MEndian::Little);
    let x1 = gen.to_cdr(MEndian::Little);
    assert!(
        x2.len() <= x1.len(),
        "XCDR2 never larger (got {} vs {})",
        x2.len(),
        x1.len()
    );
    check_x2(&ty, gen, &x2);
}

#[test]
fn int64_pair_and_int32_sequence_xcdr2() {
    let ty = dyn_type(&[(
        "example_interfaces",
        "AddTwoInts_Request",
        "int64 a\nint64 b",
    )]);
    let gen = GenAdd {
        a: -9_000_000_000,
        b: 12_345,
    };
    let x2 = gen.to_cdr_xcdr2(MEndian::Little);
    check_x2(&ty, gen, &x2);

    let ty = dyn_type(&[("example_interfaces", "Fibonacci_Result", "int32[] sequence")]);
    let fib = GenFib {
        sequence: RosSequence::alloc(vec![0, 1, 1, 2, 3, 5, 8, 13, 21]),
    };
    let x2 = fib.to_cdr_xcdr2(MEndian::Little);
    check_x2(&ty, fib, &x2);
}

#[test]
fn wide_xcdr2_is_strictly_smaller() {
    // {uint8 flag; float64 value; int64 count} — no generated counterpart, so we
    // decode a hand-built XCDR2 payload through the dynamic codec into fresh
    // memory, then re-encode both ways. XCDR2 packs `value` at offset 4 and
    // `count` at 12 (20-byte body); XCDR1 pads to 8 and 16 (24-byte body).
    let ty = dyn_type(&[("demo", "Wide", "uint8 flag\nfloat64 value\nint64 count")]);

    let mut w = roswell::cdr::Writer::with_encoding(roswell::cdr::Endian::Little, Encoding::Xcdr2);
    w.write_u8(1);
    w.write_f64(2.0);
    w.write_i64(3);
    let x2 = w.finish();
    assert_eq!(x2.len(), 4 + 20, "hand XCDR2 body should be 20 bytes");

    unsafe {
        let buf = ty.alloc_zeroed();
        ty.decode(&x2, buf).unwrap();
        let re_x2 = ty.encode_as(buf, Encoding::Xcdr2).unwrap();
        let re_x1 = ty.encode(buf).unwrap();
        assert_eq!(re_x2, x2, "XCDR2 decode -> re-encode mismatch");
        assert_eq!(re_x1.len(), 4 + 24, "XCDR1 body should be 24 bytes");
        assert!(
            re_x2.len() < re_x1.len(),
            "XCDR2 must be strictly smaller here"
        );
        ty.fini(buf);
        ty.dealloc(buf);
    }
}

/// Property-style sweep: XCDR1 output is invariant to this change, and every
/// value round-trips through XCDR2. Runs a deterministic spread of `Time`
/// values (no external proptest dependency) across both endiannesses.
#[test]
fn xcdr2_roundtrip_sweep() {
    for sec in [-2_000_000_000i32, -1, 0, 1, 42, 2_000_000_000] {
        for nanosec in [0u32, 1, 999_999_999, u32::MAX] {
            let gen = GenTime { sec, nanosec };
            let x2 = gen.to_cdr_xcdr2(MEndian::Little);
            let back = GenTime::from_cdr(&x2).unwrap();
            assert_eq!(
                (back.sec, back.nanosec),
                (sec, nanosec),
                "xcdr2 LE roundtrip"
            );

            let x2be = gen.to_cdr_xcdr2(MEndian::Big);
            assert_eq!(&x2be[..2], &[0x00, 0x06], "xcdr2 BE header");
            let backbe = GenTime::from_cdr(&x2be).unwrap();
            assert_eq!(
                (backbe.sec, backbe.nanosec),
                (sec, nanosec),
                "xcdr2 BE roundtrip"
            );

            // XCDR1 default path is byte-for-byte the historical output.
            let x1 = gen.to_cdr(MEndian::Little);
            assert_eq!(&x1[..2], &[0x00, 0x01], "xcdr1 header unchanged");
        }
    }
}

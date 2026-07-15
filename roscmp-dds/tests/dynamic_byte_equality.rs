//! Proof that the runtime, layout-driven [`roscmp::dynamic`] codec is
//! byte-for-byte identical to the *generated* `to_cdr`/`from_cdr` in
//! `roscmp_dds::msgs`, operating over the **same C-ABI struct memory**.
//!
//! The generated `#[repr(C)]` structs ARE the C layout, so we pass
//! `&generated_struct as *const u8` straight into `DynamicType::encode` and
//! compare bytes with the struct's own `.encode()`. We also assert the runtime
//! layout engine (size/align/offsets) matches `#[repr(C)]`, round-trip
//! `decode -> re-encode` through freshly allocated memory, and check `fini`
//! double-frees cleanly. String sequences / bounded types (no generated
//! counterpart) round-trip against hand-driven `roscmp::cdr::Writer` bytes.

use core::mem::{align_of, offset_of, size_of};

use roscmp::cdr::{Endian, Writer};
use roscmp::dynamic::DynamicType;
use roscmp::ir::MsgId;
use roscmp::{parse_message, resolve};

use roscmp_dds::codec::CdrMsg;
use roscmp_dds::msgs::{
    builtin_interfaces__Time as GenTime, example_interfaces__AddTwoInts_Request as GenAdd,
    example_interfaces__Fibonacci_Result as GenFib, geometry_msgs__Quaternion as GenQuat,
    geometry_msgs__Twist as GenTwist, geometry_msgs__Vector3 as GenVec3,
    sensor_msgs__Imu as GenImu, std_msgs__Header as GenHeader, std_msgs__String as GenString,
    RosSequence, RosString,
};

/// Build a [`DynamicType`] from inline `(package, Name, body)` message sources.
/// The first entry is the root; builtins are injected by [`resolve`].
fn dyn_type(sources: &[(&str, &str, &str)]) -> DynamicType {
    let inputs = sources
        .iter()
        .map(|(pkg, name, body)| (MsgId::new(*pkg, *name), parse_message(body).unwrap()))
        .collect();
    let program = resolve(inputs).unwrap();
    let (pkg, name, _) = sources[0];
    DynamicType::from_program(&program, &MsgId::new(pkg, name)).unwrap()
}

/// Verify, for a generated message value `gen` of type `T`:
/// 1. the runtime layout's total size/align matches `#[repr(C)]`;
/// 2. `DynamicType::encode` over `&gen` bytes equals `gen.encode()` exactly;
/// 3. decoding those bytes into fresh memory then re-encoding reproduces them;
/// 4. `fini` frees the decoded memory and is safe to call twice.
///
/// The fixture `gen` is consumed and its own heap buffers are freed via the
/// codec's `fini` after use (the generated `Ros{String,Sequence}` allocations
/// share the codec's global allocator), so the whole exercise leaves nothing
/// allocated — keeping the run strictly leak-clean under Miri while still
/// leak-checking every codec path.
fn check<T: CdrMsg>(ty: &DynamicType, mut gen: T) {
    assert_eq!(ty.size(), size_of::<T>(), "size mismatch");
    assert_eq!(ty.align(), align_of::<T>(), "align mismatch");

    let expected = gen.encode();
    let got = unsafe { ty.encode(core::ptr::from_ref(&gen).cast::<u8>()).unwrap() };
    assert_eq!(got, expected, "runtime encode != generated to_cdr");

    unsafe {
        let buf = ty.alloc_zeroed();
        ty.decode(&expected, buf).unwrap();
        assert_eq!(
            ty.encode(buf).unwrap(),
            expected,
            "decode -> re-encode mismatch"
        );
        ty.fini(buf);
        ty.fini(buf); // idempotent: freed triples were reset.
        ty.dealloc(buf);

        // Free the fixture's own buffers through the same layout. `T` is a
        // C-ABI struct with no `Drop`, so the ensuing drop of `gen` is a no-op
        // (its triples are now null/empty).
        ty.fini(core::ptr::from_mut(&mut gen).cast::<u8>());
    }
}

#[test]
fn std_msgs_string() {
    let ty = dyn_type(&[("std_msgs", "String", "string data")]);
    let gen = GenString {
        data: RosString::alloc("hello, dynamic world"),
    };
    check(&ty, gen);
}

#[test]
fn geometry_msgs_twist_nested_messages() {
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
    check(&ty, gen);
}

#[test]
fn geometry_msgs_quaternion_and_defaults() {
    let ty = dyn_type(&[(
        "geometry_msgs",
        "Quaternion",
        "float64 x 0.0\nfloat64 y 0.0\nfloat64 z 0.0\nfloat64 w 1.0",
    )]);
    check(
        &ty,
        GenQuat {
            x: 0.1,
            y: 0.2,
            z: 0.3,
            w: 0.9,
        },
    );

    // init_default honors the `.msg` identity-quaternion default (w = 1.0).
    unsafe {
        let buf = ty.alloc_zeroed();
        ty.init_default(buf);
        let bytes = ty.encode(buf).unwrap();
        let q = GenQuat::from_cdr(&bytes).unwrap();
        assert_eq!((q.x, q.y, q.z, q.w), (0.0, 0.0, 0.0, 1.0));
        ty.fini(buf);
        ty.dealloc(buf);
    }
}

#[test]
fn sensor_msgs_imu_nested_header_and_fixed_arrays() {
    let ty = imu_type();
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
    check(&ty, gen);
}

#[test]
fn imu_layout_matches_repr_c() {
    // Directly assert the runtime layout engine reproduces `#[repr(C)]` offsets
    // for a message exercising nesting, strings, and fixed float arrays.
    let ty = imu_type();
    let layout = ty.layout();
    let off = |name: &str| {
        layout
            .fields
            .iter()
            .find(|f| f.name == name)
            .unwrap()
            .offset
    };
    assert_eq!(ty.size(), size_of::<GenImu>());
    assert_eq!(ty.align(), align_of::<GenImu>());
    assert_eq!(off("header"), offset_of!(GenImu, header));
    assert_eq!(off("orientation"), offset_of!(GenImu, orientation));
    assert_eq!(
        off("orientation_covariance"),
        offset_of!(GenImu, orientation_covariance)
    );
    assert_eq!(
        off("angular_velocity"),
        offset_of!(GenImu, angular_velocity)
    );
    assert_eq!(
        off("linear_acceleration_covariance"),
        offset_of!(GenImu, linear_acceleration_covariance)
    );
}

fn imu_type() -> DynamicType {
    dyn_type(&[
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
    ])
}

#[test]
fn primitive_sequence_matches_generated() {
    // int32[] sequence — same field layout as the generated Fibonacci_Result.
    let ty = dyn_type(&[("example_interfaces", "Fibonacci_Result", "int32[] sequence")]);
    let gen = GenFib {
        sequence: RosSequence::alloc(vec![0, 1, 1, 2, 3, 5, 8, 13, 21]),
    };
    check(&ty, gen);
}

#[test]
fn scalar_ints_match_generated() {
    let ty = dyn_type(&[(
        "example_interfaces",
        "AddTwoInts_Request",
        "int64 a\nint64 b",
    )]);
    check(
        &ty,
        GenAdd {
            a: -9_000_000_000,
            b: 12_345,
        },
    );
}

#[test]
fn string_sequences_and_bounded_types_roundtrip() {
    // No generated counterpart, so decode hand-built CDR into fresh memory and
    // re-encode, proving the codec allocates/reads string+primitive sequence
    // and bounded buffers correctly (and `fini` frees them).
    let ty = dyn_type(&[(
        "demo",
        "Mixed",
        "string[] tags\nstring<=8 label\nint32[<=4] codes\nbool[3] flags",
    )]);

    let tags = ["alpha", "beta"];
    let codes = [7i32, 8, 9];
    let flags = [true, false, true];

    let mut w = Writer::new(Endian::Little);
    w.write_seq_len(tags.len());
    for t in tags {
        w.write_string(t);
    }
    w.write_string("ok");
    w.write_seq_len(codes.len());
    for c in codes {
        w.write_i32(c);
    }
    for f in flags {
        w.write_bool(f);
    }
    let expected = w.finish();

    unsafe {
        let buf = ty.alloc_zeroed();
        ty.decode(&expected, buf).unwrap();
        assert_eq!(
            ty.encode(buf).unwrap(),
            expected,
            "string-seq/bounded round-trip"
        );
        ty.fini(buf);
        ty.fini(buf);
        ty.dealloc(buf);
    }
}

#[test]
fn message_sequence_with_owning_elements_roundtrip() {
    // A sequence of nested messages, each owning a string, exercises the
    // recursive decode/encode/fini path (no generated counterpart exists).
    let ty = dyn_type(&[
        ("demo", "Names", "std_msgs/String[] items"),
        ("std_msgs", "String", "string data"),
    ]);

    let names = ["alpha", "beta", "gamma-longer"];
    let mut w = Writer::new(Endian::Little);
    w.write_seq_len(names.len());
    for n in names {
        w.write_string(n); // each std_msgs/String serializes as its lone string
    }
    let expected = w.finish();

    unsafe {
        let buf = ty.alloc_zeroed();
        ty.decode(&expected, buf).unwrap();
        assert_eq!(ty.encode(buf).unwrap(), expected, "message-seq round-trip");
        ty.fini(buf); // frees each element's string, then the element buffer
        ty.fini(buf);
        ty.dealloc(buf);
    }
}

#[test]
fn truncated_cdr_is_an_error() {
    let ty = dyn_type(&[("demo", "Two", "int32 a\nint32 b")]);
    let gen_bytes = GenAdd { a: 1, b: 2 }.encode(); // two i64 — plenty of bytes
                                                    // But decode our two-i32 type from a deliberately short buffer.
    let short = &gen_bytes[..gen_bytes.len().min(6)];
    unsafe {
        let buf = ty.alloc_zeroed();
        assert!(ty.decode(short, buf).is_err());
        ty.fini(buf); // partial decode is still safe to fini.
        ty.dealloc(buf);
    }
}

#[test]
fn service_loader_from_sample_srv() {
    // Item 3: the CLI pipeline as a library call. The sample .srv yields request
    // and response DynamicTypes; the request encodes byte-for-byte like the
    // generated AddTwoInts_Request.
    let path = roscmp::dynamic::sample_path("example_interfaces/srv/AddTwoInts.srv");
    let (req, resp) = roscmp::dynamic::load_service(&path, &[] as &[&std::path::Path]).unwrap();
    assert_eq!(req.root().name, "AddTwoInts_Request");
    assert_eq!(resp.root().name, "AddTwoInts_Response");
    assert_eq!(
        req.dds_type_name(),
        "example_interfaces::srv::dds_::AddTwoInts_Request_"
    );
    check(&req, GenAdd { a: 3, b: 4 });
}

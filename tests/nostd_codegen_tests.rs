//! Twin-codec equivalence for the `--no-std` profile: for values that fit
//! their fixed capacities, the heapless (`core`-only, caller-buffer) encoder
//! must emit bytes **identical** to the std generated encoder — across XCDR1
//! LE/BE and XCDR2 — and decode the std encoder's bytes back to equal values.
//! Overflow (string/sequence beyond capacity, output buffer too small) must be
//! a `Result` error, never a panic and never silent truncation.
//!
//! Pattern follows `tests/cdr_tests.rs`: generate both backends, compile the
//! pair with `rustc` (std bindings at top level, no_std bindings in a module),
//! run assertions inside the generated program.

use std::process::Command;

use roswell::codegen;
use roswell::codegen::rust_nostd::Caps;
use roswell::ir::MsgId;
use roswell::{parse_message, resolve};

/// Generate std + no_std bindings for `defs`, append `main_body`, compile, run.
fn run_twin(defs: &[(&str, &str, &str)], main_body: &str, tag: &str) -> String {
    let inputs: Vec<_> = defs
        .iter()
        .map(|(pkg, name, src)| (MsgId::new(*pkg, *name), parse_message(src).expect("parse")))
        .collect();
    let program = resolve(inputs).expect("resolve");
    let mut code = codegen::rust::generate(&program);
    code.push_str("\nmod nostd {\n");
    code.push_str(&codegen::rust_nostd::generate(&program, Caps::default()));
    code.push_str("\n}\n");
    code.push_str("\nfn main() {\n");
    code.push_str(main_body);
    code.push_str("\n}\n");

    let dir = std::env::temp_dir().join(format!("roswell_nostd_{tag}"));
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

const VECTOR3: (&str, &str, &str) = (
    "geometry_msgs",
    "Vector3",
    "float64 x\nfloat64 y\nfloat64 z\n",
);
const TWIST: (&str, &str, &str) = (
    "geometry_msgs",
    "Twist",
    "Vector3 linear\nVector3 angular\n",
);
const QUATERNION: (&str, &str, &str) = (
    "geometry_msgs",
    "Quaternion",
    "float64 x\nfloat64 y\nfloat64 z\nfloat64 w\n",
);
const TIME: (&str, &str, &str) = ("builtin_interfaces", "Time", "int32 sec\nuint32 nanosec\n");
const HEADER: (&str, &str, &str) = (
    "std_msgs",
    "Header",
    "builtin_interfaces/Time stamp\nstring frame_id\n",
);
const STRING: (&str, &str, &str) = ("std_msgs", "String", "string data\n");
const IMU: (&str, &str, &str) = (
    "sensor_msgs",
    "Imu",
    "std_msgs/Header header\n\
     geometry_msgs/Quaternion orientation\n\
     float64[9] orientation_covariance\n\
     geometry_msgs/Vector3 angular_velocity\n\
     float64[9] angular_velocity_covariance\n\
     geometry_msgs/Vector3 linear_acceleration\n\
     float64[9] linear_acceleration_covariance\n",
);
/// Sequence-bearing type: unbounded prim/string sequences (defaults: 16) and a
/// bounded sequence (`<=4`, declared bound wins).
const SEQMSG: (&str, &str, &str) = (
    "test_msgs",
    "SeqMsg",
    "uint8[] raw\nint32[] xs\nstring[] names\nfloat64[<=4] vals\n",
);

#[test]
#[allow(clippy::too_many_lines)] // the length is the embedded generated-program body
fn string_twist_imu_and_sequences_are_byte_identical() {
    let body = r#"
    // ---- std_msgs/String: XCDR1 LE/BE + XCDR2 ----
    let s_std = std_msgs__String { data: RosString::alloc("hello no_std") };
    let s_no = nostd::std_msgs__String {
        data: nostd::BoundedString::try_from_str("hello no_std").unwrap(),
    };
    let mut buf = [0u8; 512];
    for (std_bytes, no_n) in [
        (s_std.to_cdr(Endian::Little), s_no.to_cdr(&mut buf, nostd::Endian::Little).unwrap()),
    ] {
        assert_eq!(std_bytes.as_slice(), &buf[..no_n], "String XCDR1 LE");
    }
    let std_bytes = s_std.to_cdr(Endian::Big);
    let n = s_no.to_cdr(&mut buf, nostd::Endian::Big).unwrap();
    assert_eq!(std_bytes.as_slice(), &buf[..n], "String XCDR1 BE");
    let std_bytes = s_std.to_cdr_xcdr2(Endian::Little);
    let n = s_no.to_cdr_xcdr2(&mut buf, nostd::Endian::Little).unwrap();
    assert_eq!(std_bytes.as_slice(), &buf[..n], "String XCDR2 LE");

    // ---- geometry_msgs/Twist: XCDR1 LE/BE ----
    let t_std = geometry_msgs__Twist {
        linear: geometry_msgs__Vector3 { x: 1.25, y: -2.5, z: 0.5 },
        angular: geometry_msgs__Vector3 { x: 0.0, y: 0.125, z: -3.75 },
    };
    let t_no = nostd::geometry_msgs__Twist {
        linear: nostd::geometry_msgs__Vector3 { x: 1.25, y: -2.5, z: 0.5 },
        angular: nostd::geometry_msgs__Vector3 { x: 0.0, y: 0.125, z: -3.75 },
    };
    let std_bytes = t_std.to_cdr(Endian::Little);
    let n = t_no.to_cdr(&mut buf, nostd::Endian::Little).unwrap();
    assert_eq!(std_bytes.as_slice(), &buf[..n], "Twist XCDR1 LE");
    let std_bytes = t_std.to_cdr(Endian::Big);
    let n = t_no.to_cdr(&mut buf, nostd::Endian::Big).unwrap();
    assert_eq!(std_bytes.as_slice(), &buf[..n], "Twist XCDR1 BE");

    // ---- sensor_msgs/Imu: nested Header/string + f64 arrays; the u32 stamp
    // then f64 fields make XCDR1 (8-align) and XCDR2 (4-align) genuinely
    // different bodies, so both are pinned. ----
    let mut cov = [0.0f64; 9];
    for (i, c) in cov.iter_mut().enumerate() { *c = i as f64 * 0.5 - 1.0; }
    let i_std = sensor_msgs__Imu {
        header: std_msgs__Header {
            stamp: builtin_interfaces__Time { sec: 7, nanosec: 3 },
            frame_id: RosString::alloc("imu_link"),
        },
        orientation: geometry_msgs__Quaternion { x: 0.1, y: 0.2, z: 0.3, w: 0.9 },
        orientation_covariance: cov,
        angular_velocity: geometry_msgs__Vector3 { x: 4.0, y: 5.0, z: 6.0 },
        angular_velocity_covariance: cov,
        linear_acceleration: geometry_msgs__Vector3 { x: -1.0, y: -2.0, z: 9.81 },
        linear_acceleration_covariance: cov,
    };
    let i_no = nostd::sensor_msgs__Imu {
        header: nostd::std_msgs__Header {
            stamp: nostd::builtin_interfaces__Time { sec: 7, nanosec: 3 },
            frame_id: nostd::BoundedString::try_from_str("imu_link").unwrap(),
        },
        orientation: nostd::geometry_msgs__Quaternion { x: 0.1, y: 0.2, z: 0.3, w: 0.9 },
        orientation_covariance: cov,
        angular_velocity: nostd::geometry_msgs__Vector3 { x: 4.0, y: 5.0, z: 6.0 },
        angular_velocity_covariance: cov,
        linear_acceleration: nostd::geometry_msgs__Vector3 { x: -1.0, y: -2.0, z: 9.81 },
        linear_acceleration_covariance: cov,
    };
    let mut big = [0u8; 1024];
    for endian in [Endian::Little, Endian::Big] {
        let ne = if matches!(endian, Endian::Little) { nostd::Endian::Little } else { nostd::Endian::Big };
        let std_bytes = i_std.to_cdr(endian);
        let n = i_no.to_cdr(&mut big, ne).unwrap();
        assert_eq!(std_bytes.as_slice(), &big[..n], "Imu XCDR1");
        let std_bytes2 = i_std.to_cdr_xcdr2(endian);
        let n2 = i_no.to_cdr_xcdr2(&mut big, ne).unwrap();
        assert_eq!(std_bytes2.as_slice(), &big[..n2], "Imu XCDR2");
        assert_ne!(std_bytes, std_bytes2, "XCDR1 vs XCDR2 must differ for Imu");
    }

    // ---- test_msgs/SeqMsg: unbounded + bounded sequences, strings in seq ----
    let q_std = test_msgs__SeqMsg {
        raw: RosSequence::alloc(vec![1u8, 2, 3]),
        xs: RosSequence::alloc(vec![-4i32, 5]),
        names: RosSequence::alloc(vec![RosString::alloc("a"), RosString::alloc("bc")]),
        vals: RosSequence::alloc(vec![1.5f64, -2.5, 3.5]),
    };
    let mut q_no = nostd::test_msgs__SeqMsg::default();
    for v in [1u8, 2, 3] { q_no.raw.push(v).unwrap(); }
    for v in [-4i32, 5] { q_no.xs.push(v).unwrap(); }
    for s in ["a", "bc"] {
        q_no.names.push(nostd::BoundedString::try_from_str(s).unwrap()).unwrap();
    }
    for v in [1.5f64, -2.5, 3.5] { q_no.vals.push(v).unwrap(); }
    for endian in [Endian::Little, Endian::Big] {
        let ne = if matches!(endian, Endian::Little) { nostd::Endian::Little } else { nostd::Endian::Big };
        let std_bytes = q_std.to_cdr(endian);
        let n = q_no.to_cdr(&mut big, ne).unwrap();
        assert_eq!(std_bytes.as_slice(), &big[..n], "SeqMsg XCDR1");
        let std_bytes2 = q_std.to_cdr_xcdr2(endian);
        let n2 = q_no.to_cdr_xcdr2(&mut big, ne).unwrap();
        assert_eq!(std_bytes2.as_slice(), &big[..n2], "SeqMsg XCDR2");
    }

    // ---- decode: std-encoded bytes -> no_std values (and back) ----
    let bytes = i_std.to_cdr(Endian::Little);
    let back = nostd::sensor_msgs__Imu::from_cdr(&bytes).unwrap();
    assert_eq!(back.header.stamp.sec, 7);
    assert_eq!(back.header.frame_id.as_str(), "imu_link");
    assert_eq!(back.orientation.w, 0.9);
    assert_eq!(back.linear_acceleration_covariance, cov);
    let bytes = q_std.to_cdr(Endian::Big);
    let back = nostd::test_msgs__SeqMsg::from_cdr(&bytes).unwrap();
    assert_eq!(back.raw.as_slice(), &[1u8, 2, 3]);
    assert_eq!(back.xs.as_slice(), &[-4i32, 5]);
    assert_eq!(back.names.as_slice()[1].as_str(), "bc");
    assert_eq!(back.vals.as_slice(), &[1.5f64, -2.5, 3.5]);
    // no_std-encoded bytes decode through the std decoder too.
    let n = q_no.to_cdr(&mut big, nostd::Endian::Little).unwrap();
    let back_std = test_msgs__SeqMsg::from_cdr(&big[..n]).unwrap();
    assert_eq!(back_std.xs.as_slice(), &[-4i32, 5]);
    assert_eq!(back_std.names.as_slice()[0].as_str(), "a");

    println!("ALL OK");
    "#;
    let out = run_twin(
        &[
            VECTOR3, TWIST, QUATERNION, TIME, HEADER, STRING, IMU, SEQMSG,
        ],
        body,
        "equiv",
    );
    assert_eq!(out.trim(), "ALL OK");
}

#[test]
fn overflow_is_an_error_never_truncation_or_panic() {
    let body = r#"
    // A 65-byte string does not fit the default 64-byte capacity.
    let long = "x".repeat(65);
    assert!(matches!(
        nostd::BoundedString::<64>::try_from_str(&long),
        Err(nostd::CdrError::CapacityExceeded)
    ));

    // Decoding a std-encoded oversized string fails whole — no truncation.
    let s_std = std_msgs__String { data: RosString::alloc(&long) };
    let bytes = s_std.to_cdr(Endian::Little);
    assert!(matches!(
        nostd::std_msgs__String::from_cdr(&bytes),
        Err(nostd::CdrError::CapacityExceeded)
    ));
    // At exactly capacity (64) it decodes intact.
    let s64 = "y".repeat(64);
    let bytes = std_msgs__String { data: RosString::alloc(&s64) }.to_cdr(Endian::Little);
    assert_eq!(nostd::std_msgs__String::from_cdr(&bytes).unwrap().data.as_str(), s64);

    // A 17-element sequence overflows the default 16-element capacity.
    let q_std = test_msgs__SeqMsg {
        raw: RosSequence::alloc((0..17u8).collect()),
        xs: RosSequence::alloc(vec![]),
        names: RosSequence::alloc(vec![]),
        vals: RosSequence::alloc(vec![]),
    };
    let bytes = q_std.to_cdr(Endian::Little);
    assert!(matches!(
        nostd::test_msgs__SeqMsg::from_cdr(&bytes),
        Err(nostd::CdrError::CapacityExceeded)
    ));

    // A 5-element bounded (`<=4`) sequence overflows its declared bound.
    let q_std = test_msgs__SeqMsg {
        raw: RosSequence::alloc(vec![]),
        xs: RosSequence::alloc(vec![]),
        names: RosSequence::alloc(vec![]),
        vals: RosSequence::alloc(vec![0.0; 5]),
    };
    let bytes = q_std.to_cdr(Endian::Little);
    assert!(matches!(
        nostd::test_msgs__SeqMsg::from_cdr(&bytes),
        Err(nostd::CdrError::CapacityExceeded)
    ));
    let mut q_no = nostd::test_msgs__SeqMsg::default();
    for v in [0.0f64; 4] { q_no.vals.push(v).unwrap(); }
    assert!(matches!(q_no.vals.push(0.0), Err(nostd::CdrError::CapacityExceeded)));

    // Encoding into a too-small output buffer is BufferFull, not a panic.
    let t = nostd::geometry_msgs__Twist::default();
    let mut tiny = [0u8; 16];
    assert!(matches!(
        t.to_cdr(&mut tiny, nostd::Endian::Little),
        Err(nostd::CdrError::BufferFull)
    ));
    let mut nothing = [0u8; 2];
    assert!(matches!(
        t.to_cdr(&mut nothing, nostd::Endian::Little),
        Err(nostd::CdrError::BufferFull)
    ));

    // Truncated input is Truncated, not a panic.
    let mut buf = [0u8; 128];
    let n = t.to_cdr(&mut buf, nostd::Endian::Little).unwrap();
    assert!(matches!(
        nostd::geometry_msgs__Twist::from_cdr(&buf[..n - 1]),
        Err(nostd::CdrError::Truncated)
    ));

    println!("ALL OK");
    "#;
    let out = run_twin(
        &[
            VECTOR3, TWIST, QUATERNION, TIME, HEADER, STRING, IMU, SEQMSG,
        ],
        body,
        "overflow",
    );
    assert_eq!(out.trim(), "ALL OK");
}

/// The committed firmware bindings (`hil-renode/fw/src/msgs_nostd.rs`) must be
/// exactly what `roswell --lang rust --no-std` emits for Vector3 + Twist today.
#[test]
fn firmware_twist_bindings_are_fresh() {
    let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR"));
    let inputs = ["Vector3", "Twist"]
        .iter()
        .map(|name| {
            let src =
                std::fs::read_to_string(root.join(format!("samples/geometry_msgs/msg/{name}.msg")))
                    .expect("read sample msg");
            (
                MsgId::new("geometry_msgs", *name),
                parse_message(&src).expect("parse"),
            )
        })
        .collect();
    let program = resolve(inputs).expect("resolve");
    let generated = codegen::rust_nostd::generate(&program, Caps::default());

    // The committed file is `cargo fmt`ed (the fw workspace fmt gate covers
    // it), so normalize the generated text the same way before comparing.
    let dir = std::env::temp_dir().join("roswell_nostd_fw_fresh");
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join("gen.rs");
    std::fs::write(&path, &generated).unwrap();
    let status = Command::new("rustfmt")
        .args(["--edition", "2021"])
        .arg(&path)
        .status()
        .expect("run rustfmt");
    assert!(status.success(), "rustfmt failed on generated bindings");
    let expected = std::fs::read_to_string(&path).unwrap();

    let committed = std::fs::read_to_string(root.join("hil-renode/fw/src/msgs_nostd.rs"))
        .expect("read committed firmware bindings");
    assert_eq!(
        committed, expected,
        "hil-renode/fw/src/msgs_nostd.rs is stale; regenerate with \
         `cargo run -- --lang rust --no-std --out <dir> samples/geometry_msgs/msg/Vector3.msg \
         samples/geometry_msgs/msg/Twist.msg`, copy roswell_msgs_nostd.rs over it, \
         then `cd hil-renode && cargo fmt`"
    );
}

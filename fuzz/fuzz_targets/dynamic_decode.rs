#![no_main]
//! Fuzz the runtime, layout-driven CDR codec over C-ABI memory
//! (`roscmp::dynamic`) — the unsafe walker surface.
//!
//! A fixed set of `DynamicType`s (built once) covers scalars, strings, fixed
//! arrays, primitive/string/message sequences and nesting. For each, we decode
//! the fuzz bytes into freshly `alloc_zeroed`ed memory, and on success re-encode
//! and decode again asserting round-trip idempotence. Every iteration ends with
//! `fini` (twice — must be idempotent) then `dealloc`, so LeakSanitizer flags
//! any buffer the codec allocates but fails to link back to `out` (e.g. a
//! partial sequence on a truncated decode). Any panic, leak, or UB is a finding.

use std::sync::OnceLock;

use libfuzzer_sys::fuzz_target;

use roscmp::dynamic::DynamicType;
use roscmp::ir::MsgId;
use roscmp::{parse_message, resolve};

/// Build a `DynamicType` from inline `(package, Name, body)` sources; the first
/// entry is the root. Mirrors `roscmp-dds/tests/dynamic_byte_equality.rs`.
fn dyn_type(sources: &[(&str, &str, &str)]) -> DynamicType {
    let inputs = sources
        .iter()
        .map(|(pkg, name, body)| (MsgId::new(*pkg, *name), parse_message(body).unwrap()))
        .collect();
    let program = resolve(inputs).unwrap();
    let (pkg, name, _) = sources[0];
    DynamicType::from_program(&program, &MsgId::new(pkg, name)).unwrap()
}

/// The corpus of message shapes exercised every iteration. Built once.
fn types() -> &'static [DynamicType] {
    static TYPES: OnceLock<Vec<DynamicType>> = OnceLock::new();
    TYPES.get_or_init(|| {
        vec![
            // Lone string.
            dyn_type(&[("std_msgs", "String", "string data")]),
            // Nested Header + Pose (Point + Quaternion): strings + f64 + nesting.
            dyn_type(&[
                (
                    "geometry_msgs",
                    "PoseStamped",
                    "std_msgs/Header header\ngeometry_msgs/Pose pose",
                ),
                (
                    "geometry_msgs",
                    "Pose",
                    "geometry_msgs/Point position\ngeometry_msgs/Quaternion orientation",
                ),
                ("geometry_msgs", "Point", "float64 x\nfloat64 y\nfloat64 z"),
                (
                    "geometry_msgs",
                    "Quaternion",
                    "float64 x\nfloat64 y\nfloat64 z\nfloat64 w",
                ),
            ]),
            // Imu: nested header/string, fixed float[9] arrays, more nesting.
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
            ]),
            // Primitive sequence (int32[]): the truncation/leak-prone path.
            dyn_type(&[("example_interfaces", "Fibonacci_Result", "int32[] sequence")]),
            // Mixed sequences: string[] (owning elements), bounded string, a
            // bounded primitive sequence and a fixed bool array.
            dyn_type(&[(
                "demo",
                "Mixed",
                "string[] tags\nstring<=8 label\nint32[<=4] codes\nbool[3] flags",
            )]),
            // Sequence of nested messages that each own a string: recursive
            // decode/fini and the string-inside-partial-sequence leak path.
            dyn_type(&[
                ("demo", "Names", "std_msgs/String[] items"),
                ("std_msgs", "String", "string data"),
            ]),
        ]
    })
}

fuzz_target!(|data: &[u8]| {
    for ty in types() {
        unsafe {
            let buf = ty.alloc_zeroed();
            if ty.decode(data, buf).is_ok() {
                // Round-trip: re-encode, decode into fresh memory, re-encode
                // again; the two encodings must match (idempotence).
                let once = ty.encode(buf).unwrap();
                let buf2 = ty.alloc_zeroed();
                ty.decode(&once, buf2)
                    .expect("re-decode of our own encoding must succeed");
                let twice = ty.encode(buf2).unwrap();
                assert_eq!(once, twice, "encode/decode not idempotent");
                ty.fini(buf2);
                ty.dealloc(buf2);
            }
            // On both the ok and err paths the message must be safe to fini
            // (idempotently) and free — no leaked buffers, no double free.
            ty.fini(buf);
            ty.fini(buf);
            ty.dealloc(buf);
        }
    }
});

//! Regression tests for the runtime codec's decode error paths, distilled from
//! fuzzing + Miri. These exercise malformed/truncated CDR that fails *mid
//! decode*; the contract (see `DynamicType::decode`) is that on error `out` still
//! owns every buffer allocated so far, so `fini` frees it — no leaks. Run under
//! Miri (`scripts/miri.sh`) to actually observe the leak-freedom; a plain
//! `cargo test` run just asserts no panic/UB.

use roscmp::dynamic::DynamicType;
use roscmp::ir::MsgId;
use roscmp::{parse_message, resolve};

fn dyn_type(sources: &[(&str, &str, &str)]) -> DynamicType {
    let inputs = sources
        .iter()
        .map(|(pkg, name, body)| (MsgId::new(*pkg, *name), parse_message(body).unwrap()))
        .collect();
    let program = resolve(inputs).unwrap();
    let (pkg, name, _) = sources[0];
    DynamicType::from_program(&program, &MsgId::new(pkg, name)).unwrap()
}

/// A primitive sequence whose element decode is truncated *after* the buffer is
/// allocated: `int32[]` with count 3 but only ~1.5 elements of payload. Before
/// the fix, `decode_message` allocated the element buffer and returned `Err`
/// without linking it to `out`, leaking it (Miri: "memory leaked").
#[test]
fn truncated_primitive_sequence_frees_cleanly() {
    let ty = dyn_type(&[("example_interfaces", "Fibonacci_Result", "int32[] sequence")]);
    // header + seq_len=3 + one full i32 + a partial (2 bytes) i32.
    let cdr = [
        0x00, 0x01, 0x00, 0x00, // CDR_LE encapsulation
        0x03, 0x00, 0x00, 0x00, // seq len = 3 (<= remaining bytes, passes the bound)
        0x01, 0x00, 0x00, 0x00, // element 0
        0x02, 0x00, // element 1: truncated
    ];
    unsafe {
        let buf = ty.alloc_zeroed();
        assert!(ty.decode(&cdr, buf).is_err(), "expected truncation error");
        ty.fini(buf); // must free the partially-decoded element buffer.
        ty.fini(buf); // idempotent.
        ty.dealloc(buf);
    }
}

/// A sequence of owning-string messages truncated after the first element is
/// fully decoded (heap string allocated) but before the second. Both the
/// element buffer *and* the first element's string must be freed by `fini`.
#[test]
fn truncated_message_sequence_frees_owned_strings() {
    let ty = dyn_type(&[
        ("demo", "Names", "std_msgs/String[] items"),
        ("std_msgs", "String", "string data"),
    ]);
    let cdr = [
        0x00, 0x01, 0x00, 0x00, // CDR_LE
        0x02, 0x00, 0x00, 0x00, // seq len = 2
        0x02, 0x00, 0x00, 0x00, b'a', 0x00, // element 0 string "a" (len incl NUL = 2)
        0x03, 0x00, // element 1: truncated string length prefix
    ];
    unsafe {
        let buf = ty.alloc_zeroed();
        assert!(ty.decode(&cdr, buf).is_err(), "expected truncation error");
        ty.fini(buf);
        ty.fini(buf);
        ty.dealloc(buf);
    }
}

/// A bounded primitive sequence (`int32[<=4]`) truncated mid-element — same
/// allocate-then-fail path via a different field shape.
#[test]
fn truncated_bounded_sequence_frees_cleanly() {
    let ty = dyn_type(&[("demo", "Codes", "int32[<=4] codes")]);
    let cdr = [
        0x00, 0x01, 0x00, 0x00, // CDR_LE
        0x02, 0x00, 0x00, 0x00, // seq len = 2
        0x07, 0x00, 0x00, 0x00, // element 0
        0x08, // element 1: truncated
    ];
    unsafe {
        let buf = ty.alloc_zeroed();
        assert!(ty.decode(&cdr, buf).is_err());
        ty.fini(buf);
        ty.dealloc(buf);
    }
}

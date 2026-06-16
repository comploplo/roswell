//! CDR (Common Data Representation) runtime — the DDS/ROS2 wire format.
//!
//! This implements **Classic CDR / XCDR1** as used by ROS2's `rmw_fastrtps` and
//! `rmw_cyclonedds`:
//!
//! - A 4-byte **encapsulation header** prefixes every message:
//!   `repr_id` (2 bytes, big-endian) + `options` (2 bytes). `CDR_LE = 0x0001`,
//!   `CDR_BE = 0x0000`.
//! - Primitives are **self-aligned** relative to the start of the body (the
//!   byte right after the encapsulation header — the alignment origin is reset
//!   there, matching Fast-CDR/Cyclone). Padding bytes are zero.
//! - `string`: `uint32` length *including* the trailing NUL, then the bytes,
//!   then the NUL.
//! - sequence: `uint32` element count, then the elements (each self-aligned).
//! - fixed array: just the elements, no length prefix.
//! - nested struct: inlined; alignment continues across the whole stream.
//!
//! The alignment-origin-reset assumption is the one detail worth confirming
//! against real ROS2 output (see M2.1); it is isolated to `Cursor` so a flip is
//! a one-line change.
//!
//! The implementation lives in `cdr_runtime.rs`, which is also embedded verbatim
//! into generated bindings (see `codegen::rust`) so they are self-contained.

include!("cdr_runtime.rs");

#[cfg(test)]
mod tests {
    use super::*;

    /// Body bytes only (drop the 4-byte encapsulation header).
    fn body(v: &[u8]) -> &[u8] {
        &v[4..]
    }

    #[test]
    fn encapsulation_header_is_cdr_le() {
        let w = Writer::new(Endian::Little);
        assert_eq!(&w.finish()[..4], &[0x00, 0x01, 0x00, 0x00]);
    }

    #[test]
    fn u32_le() {
        let mut w = Writer::new(Endian::Little);
        w.write_u32(5);
        assert_eq!(body(&w.finish()), &[0x05, 0x00, 0x00, 0x00]);
    }

    #[test]
    fn point_three_f64_no_leading_pad() {
        // geometry_msgs/Point {1.0, 2.0, 3.0}: origin reset means the leading
        // f64 needs no padding after the header.
        let mut w = Writer::new(Endian::Little);
        w.write_f64(1.0);
        w.write_f64(2.0);
        w.write_f64(3.0);
        let out = w.finish();
        assert_eq!(out.len(), 4 + 24);
        assert_eq!(
            body(&out),
            &[
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0x3F, // 1.0
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, // 2.0
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x40, // 3.0
            ]
        );
    }

    #[test]
    fn mixed_alignment_pads() {
        // {u8 a=1; u32 b=2}: a at body0, 3 pad bytes, then b at body4.
        let mut w = Writer::new(Endian::Little);
        w.write_u8(1);
        w.write_u32(2);
        assert_eq!(
            body(&w.finish()),
            &[0x01, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00]
        );
    }

    #[test]
    fn string_includes_nul_in_length() {
        let mut w = Writer::new(Endian::Little);
        w.write_string("abc");
        assert_eq!(
            body(&w.finish()),
            &[0x04, 0x00, 0x00, 0x00, b'a', b'b', b'c', 0x00]
        );
    }

    #[test]
    fn sequence_u8_count_prefix() {
        let mut w = Writer::new(Endian::Little);
        w.write_seq_len(3);
        for b in [1u8, 2, 3] {
            w.write_u8(b);
        }
        assert_eq!(
            body(&w.finish()),
            &[0x03, 0x00, 0x00, 0x00, 0x01, 0x02, 0x03]
        );
    }

    #[test]
    fn header_like_layout() {
        // {int32 sec=1; uint32 nanosec=2; string frame_id="map"}
        let mut w = Writer::new(Endian::Little);
        w.write_i32(1);
        w.write_u32(2);
        w.write_string("map");
        assert_eq!(
            body(&w.finish()),
            &[
                0x01, 0x00, 0x00, 0x00, // sec
                0x02, 0x00, 0x00, 0x00, // nanosec
                0x04, 0x00, 0x00, 0x00, b'm', b'a', b'p', 0x00, // frame_id
            ]
        );
    }

    #[test]
    fn big_endian_roundtrip() {
        let mut w = Writer::new(Endian::Big);
        w.write_u32(5);
        let out = w.finish();
        assert_eq!(&out[..2], &REPR_CDR_BE);
        assert_eq!(body(&out), &[0x00, 0x00, 0x00, 0x05]);

        let mut r = Reader::new(&out).unwrap();
        assert_eq!(r.endian(), Endian::Big);
        assert_eq!(r.read_u32().unwrap(), 5);
    }

    #[test]
    fn reader_mirrors_writer() {
        let mut w = Writer::new(Endian::Little);
        w.write_u8(7);
        w.write_f64(2.5);
        w.write_string("hello");
        w.write_seq_len(2);
        w.write_i32(-1);
        w.write_i32(42);
        let out = w.finish();

        let mut r = Reader::new(&out).unwrap();
        assert_eq!(r.read_u8().unwrap(), 7);
        assert_eq!(r.read_f64().unwrap(), 2.5);
        assert_eq!(r.read_string().unwrap(), "hello");
        assert_eq!(r.read_seq_len().unwrap(), 2);
        assert_eq!(r.read_i32().unwrap(), -1);
        assert_eq!(r.read_i32().unwrap(), 42);
    }

    #[test]
    fn truncated_is_error() {
        let buf = [0x00, 0x01, 0x00, 0x00, 0x01, 0x02]; // only 2 body bytes
        let mut r = Reader::new(&buf).unwrap();
        assert_eq!(r.read_u32(), Err(CdrError::Truncated));
    }

    #[test]
    fn bad_encapsulation_is_error() {
        let buf = [0xFF, 0xFF, 0x00, 0x00];
        assert_eq!(Reader::new(&buf).err(), Some(CdrError::BadEncapsulation));
    }
}

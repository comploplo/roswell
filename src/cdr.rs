//! CDR (Common Data Representation) runtime — the DDS/ROS2 wire format.
//!
//! This implements **Classic CDR / XCDR1** (the ROS2 wire default) and
//! **PLAIN_CDR2 / XCDR2** (the forward-looking Iron+ / rmw_zenoh format),
//! selected by [`Encoding`]. A [`Reader`] auto-detects the encoding from the
//! encapsulation identifier; a [`Writer`] emits whichever is requested (default
//! XCDR1). For a `@final` struct — which every ROS2 interface type is — the only
//! body difference is that XCDR2 caps the maximum primitive alignment at 4, so
//! 8-byte primitives align to 4 rather than 8. As used by ROS2's `rmw_fastrtps`
//! and `rmw_cyclonedds`:
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

    #[test]
    fn from_vec_reuses_dirty_buffer_byte_identically() {
        // The loaned-buffer hot path: a dirty, over-capacity buffer must yield
        // the exact bytes a fresh `with_encoding` writer produces.
        let write = |mut w: Writer| {
            w.write_u8(1);
            w.write_f64(2.0);
            w.write_string("abc");
            w.finish()
        };
        let fresh = write(Writer::with_encoding(Endian::Little, Encoding::Xcdr1));
        let dirty = vec![0xAA; fresh.len() + 32];
        let reused = write(Writer::from_vec(dirty, Endian::Little, Encoding::Xcdr1));
        assert_eq!(reused, fresh);
    }

    // ---- XCDR2 (PLAIN_CDR2) ------------------------------------------------

    #[test]
    fn xcdr2_encapsulation_header_le_is_cdr2() {
        let w = Writer::with_encoding(Endian::Little, Encoding::Xcdr2);
        // PLAIN_CDR2 little-endian: repr id `00 07`, options `00 00`.
        assert_eq!(&w.finish()[..4], &[0x00, 0x07, 0x00, 0x00]);
    }

    #[test]
    fn xcdr2_encapsulation_header_be_is_cdr2() {
        let w = Writer::with_encoding(Endian::Big, Encoding::Xcdr2);
        assert_eq!(&w.finish()[..4], &[0x00, 0x06, 0x00, 0x00]);
    }

    #[test]
    fn xcdr2_f64_aligns_to_four_not_eight() {
        // {uint8 a=1; float64 b=2.0}: in XCDR1 `b` pads to offset 8; in XCDR2 the
        // max-align-4 rule lands it at offset 4. This is the whole XCDR2 delta.
        let mut w = Writer::with_encoding(Endian::Little, Encoding::Xcdr2);
        w.write_u8(1);
        w.write_f64(2.0);
        assert_eq!(
            body(&w.finish()),
            &[
                0x01, 0x00, 0x00, 0x00, // a=1 then 3 pad bytes to align(4)
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, // b=2.0 at offset 4
            ]
        );
    }

    #[test]
    fn xcdr1_f64_still_aligns_to_eight() {
        // Same message under XCDR1: `b` pads all the way to offset 8 (unchanged).
        let mut w = Writer::new(Endian::Little);
        w.write_u8(1);
        w.write_f64(2.0);
        assert_eq!(body(&w.finish()).len(), 16);
    }

    #[test]
    fn xcdr2_is_smaller_for_eight_byte_heavy_types() {
        // {int32 a; int64 b}: XCDR1 = 4 + 4 pad + 8 = 16; XCDR2 = 4 + 8 = 12.
        let enc = |e: Encoding| {
            let mut w = Writer::with_encoding(Endian::Little, e);
            w.write_i32(7);
            w.write_i64(9);
            w.finish().len()
        };
        assert_eq!(enc(Encoding::Xcdr1), 4 + 16);
        assert_eq!(enc(Encoding::Xcdr2), 4 + 12);
        assert!(enc(Encoding::Xcdr2) < enc(Encoding::Xcdr1));
    }

    #[test]
    fn xcdr2_string_framing_unchanged() {
        // Strings keep the classic uint32 length (incl. NUL) + bytes + NUL.
        let mut w = Writer::with_encoding(Endian::Little, Encoding::Xcdr2);
        w.write_string("abc");
        assert_eq!(
            body(&w.finish()),
            &[0x04, 0x00, 0x00, 0x00, b'a', b'b', b'c', 0x00]
        );
    }

    #[test]
    fn reader_autodetects_xcdr2_and_roundtrips() {
        let mut w = Writer::with_encoding(Endian::Little, Encoding::Xcdr2);
        w.write_u8(1);
        w.write_f64(2.0);
        w.write_i32(3);
        w.write_i64(4);
        w.write_string("hello");
        let out = w.finish();

        let mut r = Reader::new(&out).unwrap();
        assert_eq!(r.encoding(), Encoding::Xcdr2);
        assert_eq!(r.endian(), Endian::Little);
        assert_eq!(r.read_u8().unwrap(), 1);
        assert_eq!(r.read_f64().unwrap(), 2.0);
        assert_eq!(r.read_i32().unwrap(), 3);
        assert_eq!(r.read_i64().unwrap(), 4);
        assert_eq!(r.read_string().unwrap(), "hello");
    }

    #[test]
    fn reader_reports_xcdr1_encoding() {
        let out = Writer::new(Endian::Little).finish();
        assert_eq!(Reader::new(&out).unwrap().encoding(), Encoding::Xcdr1);
    }

    /// The CDR alignment formula is embedded verbatim into generated bindings,
    /// so it cannot call an external crate. This pins the embedded `pad_to` to
    /// its Creusot-verified twin `roswell_verify::pad_to` across every CDR
    /// alignment and a wide offset range, so the machine-checked panic-freedom,
    /// `result < a`, and alignment guarantees transfer to this copy. See
    /// `docs/RT.md`.
    #[test]
    fn pad_to_matches_verified_core() {
        for a in [1usize, 2, 4, 8] {
            for off in 0usize..1024 {
                let p = pad_to(off, a);
                assert_eq!(p, roswell_verify::pad_to(off, a));
                assert!(p < a);
                assert_eq!((off + p) % a, 0);
            }
        }
    }
}

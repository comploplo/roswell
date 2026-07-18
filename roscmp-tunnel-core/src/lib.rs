//! `no_std`, alloc-free wire codec for roscmp tunnel frames.
//!
//! This is the byte-for-byte core of the framing implemented in
//! `roscmp_dds::tunnel`. It is split out so a no-DDS microcontroller (which has
//! no `std`, no allocator, and no `RawMsg`) can speak the exact same tunnel
//! wire protocol as the host-side bridge, and so the two implementations can be
//! pinned together by an equivalence test.
//!
//! A frame on the wire is:
//!
//! ```text
//! MAGIC (8 bytes) | payload_len: u32 (LE) | payload[payload_len]
//! ```
//!
//! and the payload is a one-byte [`kind`] tag followed by little-endian fields.
//! Encoders write a whole framed message (header + payload) into a caller
//! buffer and return its length; [`parse_payload`] borrows straight out of a
//! received payload slice with no copies.

#![no_std]

/// Frame magic prefixing every tunnel frame on the wire.
pub const MAGIC: [u8; 8] = *b"RCPTUN1\0";

/// Length of the fixed frame header: [`MAGIC`] plus the `u32` payload length.
pub const HEADER_LEN: usize = MAGIC.len() + 4;

/// Upper bound on a payload length accepted by [`frame_len`], matching the
/// host tunnel's `MAX_FRAME_LEN` (64 MiB).
pub const MAX_PAYLOAD_LEN: usize = 64 * 1024 * 1024;

/// One-byte payload tags identifying each [`FrameRef`] variant on the wire.
pub mod kind {
    /// [`super::FrameRef::Hello`].
    pub const HELLO: u8 = 1;
    /// [`super::FrameRef::TopicSample`].
    pub const TOPIC_SAMPLE: u8 = 2;
    /// [`super::FrameRef::Ack`].
    pub const ACK: u8 = 3;
    /// [`super::FrameRef::Heartbeat`].
    pub const HEARTBEAT: u8 = 4;
}

/// Codec failures. No allocation, no `std::io`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    /// The output buffer was too small to hold the encoded frame.
    ShortBuffer,
    /// Input ended in the middle of a field.
    Truncated,
    /// Frame header did not start with [`MAGIC`].
    BadMagic,
    /// Unknown payload [`kind`] tag.
    BadKind,
    /// A length field exceeded [`MAX_PAYLOAD_LEN`] or `u32`.
    TooLong,
    /// A string field was not valid UTF-8.
    BadUtf8,
}

/// A borrowed view of a decoded frame payload. Field slices point into the
/// buffer passed to [`parse_payload`]; nothing is copied.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FrameRef<'a> {
    /// Peer announcement.
    Hello {
        /// Human-readable peer name.
        peer: &'a str,
    },
    /// A raw ROS topic sample.
    TopicSample {
        /// Monotonic sender sequence number.
        sequence: u64,
        /// ROS topic name, e.g. `/cmd_vel`.
        topic: &'a str,
        /// ROS type name, e.g. `geometry_msgs/msg/Twist`.
        ros_type: &'a str,
        /// Source timestamp in nanoseconds since the Unix epoch.
        stamp_nanos: i64,
        /// CDR-encoded message body.
        cdr: &'a [u8],
    },
    /// Acknowledgement of a reliable sample.
    Ack {
        /// Sequence number being acknowledged.
        sequence: u64,
    },
    /// Liveliness heartbeat.
    Heartbeat {
        /// Sender timestamp in nanoseconds since the Unix epoch.
        stamp_nanos: i64,
    },
}

/// Validates a 12-byte frame header and returns its payload length.
///
/// Read [`HEADER_LEN`] bytes off the wire, hand them here, then read exactly
/// the returned number of payload bytes and pass them to [`parse_payload`].
///
/// # Errors
///
/// [`Error::BadMagic`] if the prefix is not [`MAGIC`], [`Error::TooLong`] if
/// the declared payload exceeds [`MAX_PAYLOAD_LEN`].
pub fn frame_len(header: &[u8; HEADER_LEN]) -> Result<usize, Error> {
    if header[..MAGIC.len()] != MAGIC {
        return Err(Error::BadMagic);
    }
    let mut len = [0u8; 4];
    len.copy_from_slice(&header[MAGIC.len()..HEADER_LEN]);
    let len = u32::from_le_bytes(len) as usize;
    if len > MAX_PAYLOAD_LEN {
        return Err(Error::TooLong);
    }
    Ok(len)
}

// ---------------------------------------------------------------------------
// Encoding
// ---------------------------------------------------------------------------

/// Encodes a [`FrameRef::Hello`] into `out`, returning the framed byte count.
///
/// # Errors
///
/// [`Error::ShortBuffer`] if `out` cannot hold the frame.
pub fn encode_hello(out: &mut [u8], peer: &str) -> Result<usize, Error> {
    frame(out, |c| {
        c.put_u8(kind::HELLO)?;
        c.put_lp(peer.as_bytes())
    })
}

/// Encodes a [`FrameRef::Ack`] into `out`, returning the framed byte count.
///
/// # Errors
///
/// [`Error::ShortBuffer`] if `out` cannot hold the frame.
pub fn encode_ack(out: &mut [u8], sequence: u64) -> Result<usize, Error> {
    frame(out, |c| {
        c.put_u8(kind::ACK)?;
        c.put(&sequence.to_le_bytes())
    })
}

/// Encodes a [`FrameRef::Heartbeat`] into `out`, returning the framed byte count.
///
/// # Errors
///
/// [`Error::ShortBuffer`] if `out` cannot hold the frame.
pub fn encode_heartbeat(out: &mut [u8], stamp_nanos: i64) -> Result<usize, Error> {
    frame(out, |c| {
        c.put_u8(kind::HEARTBEAT)?;
        c.put(&stamp_nanos.to_le_bytes())
    })
}

/// Encodes a [`FrameRef::TopicSample`] into `out`, returning the framed byte count.
///
/// # Errors
///
/// [`Error::ShortBuffer`] if `out` cannot hold the frame.
pub fn encode_topic_sample(
    out: &mut [u8],
    sequence: u64,
    topic: &str,
    ros_type: &str,
    stamp_nanos: i64,
    cdr: &[u8],
) -> Result<usize, Error> {
    frame(out, |c| {
        c.put_u8(kind::TOPIC_SAMPLE)?;
        c.put(&sequence.to_le_bytes())?;
        c.put_lp(topic.as_bytes())?;
        c.put_lp(ros_type.as_bytes())?;
        c.put(&stamp_nanos.to_le_bytes())?;
        c.put_lp(cdr)
    })
}

// ---------------------------------------------------------------------------
// Decoding
// ---------------------------------------------------------------------------

/// Decodes a frame payload (the bytes after the [`HEADER_LEN`] header).
///
/// The returned [`FrameRef`] borrows from `payload`.
///
/// # Errors
///
/// [`Error::Truncated`] on a short payload, [`Error::BadKind`] on an unknown
/// tag, [`Error::BadUtf8`] on an invalid string field.
pub fn parse_payload(payload: &[u8]) -> Result<FrameRef<'_>, Error> {
    let mut r = Reader::new(payload);
    let kind = r.u8()?;
    match kind {
        kind::HELLO => Ok(FrameRef::Hello { peer: r.lp_str()? }),
        kind::TOPIC_SAMPLE => {
            let sequence = r.u64()?;
            let topic = r.lp_str()?;
            let ros_type = r.lp_str()?;
            let stamp_nanos = r.i64()?;
            let cdr = r.lp_bytes()?;
            Ok(FrameRef::TopicSample {
                sequence,
                topic,
                ros_type,
                stamp_nanos,
                cdr,
            })
        }
        kind::ACK => Ok(FrameRef::Ack { sequence: r.u64()? }),
        kind::HEARTBEAT => Ok(FrameRef::Heartbeat {
            stamp_nanos: r.i64()?,
        }),
        _ => Err(Error::BadKind),
    }
}

// ---------------------------------------------------------------------------
// Cursors
// ---------------------------------------------------------------------------

/// Runs `build` between a reserved length header and back-patches the length.
fn frame<F>(out: &mut [u8], build: F) -> Result<usize, Error>
where
    F: FnOnce(&mut Writer) -> Result<(), Error>,
{
    let mut c = Writer { buf: out, pos: 0 };
    c.put(&MAGIC)?;
    c.put(&[0u8; 4])?; // length placeholder
    build(&mut c)?;
    let payload_len = c.pos - HEADER_LEN;
    let n = u32::try_from(payload_len).map_err(|_| Error::TooLong)?;
    c.buf[MAGIC.len()..HEADER_LEN].copy_from_slice(&n.to_le_bytes());
    Ok(c.pos)
}

struct Writer<'a> {
    buf: &'a mut [u8],
    pos: usize,
}

impl Writer<'_> {
    fn put(&mut self, bytes: &[u8]) -> Result<(), Error> {
        let end = self.pos.checked_add(bytes.len()).ok_or(Error::TooLong)?;
        let dst = self.buf.get_mut(self.pos..end).ok_or(Error::ShortBuffer)?;
        dst.copy_from_slice(bytes);
        self.pos = end;
        Ok(())
    }

    fn put_u8(&mut self, value: u8) -> Result<(), Error> {
        self.put(&[value])
    }

    /// Length-prefixed bytes: `u32` LE length then the bytes.
    fn put_lp(&mut self, bytes: &[u8]) -> Result<(), Error> {
        let n = u32::try_from(bytes.len()).map_err(|_| Error::TooLong)?;
        self.put(&n.to_le_bytes())?;
        self.put(bytes)
    }
}

struct Reader<'a> {
    buf: &'a [u8],
    pos: usize,
}

impl<'a> Reader<'a> {
    fn new(buf: &'a [u8]) -> Self {
        Self { buf, pos: 0 }
    }

    fn take(&mut self, n: usize) -> Result<&'a [u8], Error> {
        let end = self.pos.checked_add(n).ok_or(Error::Truncated)?;
        let slice = self.buf.get(self.pos..end).ok_or(Error::Truncated)?;
        self.pos = end;
        Ok(slice)
    }

    fn u8(&mut self) -> Result<u8, Error> {
        Ok(self.take(1)?[0])
    }

    fn u32(&mut self) -> Result<u32, Error> {
        let mut b = [0u8; 4];
        b.copy_from_slice(self.take(4)?);
        Ok(u32::from_le_bytes(b))
    }

    fn u64(&mut self) -> Result<u64, Error> {
        let mut b = [0u8; 8];
        b.copy_from_slice(self.take(8)?);
        Ok(u64::from_le_bytes(b))
    }

    fn i64(&mut self) -> Result<i64, Error> {
        Ok(self.u64()? as i64)
    }

    fn lp_bytes(&mut self) -> Result<&'a [u8], Error> {
        let len = self.u32()? as usize;
        self.take(len)
    }

    fn lp_str(&mut self) -> Result<&'a str, Error> {
        core::str::from_utf8(self.lp_bytes()?).map_err(|_| Error::BadUtf8)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ack_round_trips_through_header_and_payload() {
        let mut buf = [0u8; 32];
        let n = encode_ack(&mut buf, 0x1122_3344_5566_7788).unwrap();
        assert!(n >= HEADER_LEN);
        let header: &[u8; HEADER_LEN] = buf[..HEADER_LEN].try_into().unwrap();
        let len = frame_len(header).unwrap();
        assert_eq!(len, n - HEADER_LEN);
        match parse_payload(&buf[HEADER_LEN..n]).unwrap() {
            FrameRef::Ack { sequence } => assert_eq!(sequence, 0x1122_3344_5566_7788),
            other => panic!("expected ack, got {other:?}"),
        }
    }

    #[test]
    fn topic_sample_round_trips_all_fields() {
        let mut buf = [0u8; 128];
        let n = encode_topic_sample(
            &mut buf,
            7,
            "/cmd_vel",
            "geometry_msgs/msg/Twist",
            -42,
            &[1, 2, 3, 4],
        )
        .unwrap();
        let header: &[u8; HEADER_LEN] = buf[..HEADER_LEN].try_into().unwrap();
        let len = frame_len(header).unwrap();
        assert_eq!(n, HEADER_LEN + len);
        match parse_payload(&buf[HEADER_LEN..HEADER_LEN + len]).unwrap() {
            FrameRef::TopicSample {
                sequence,
                topic,
                ros_type,
                stamp_nanos,
                cdr,
            } => {
                assert_eq!(sequence, 7);
                assert_eq!(topic, "/cmd_vel");
                assert_eq!(ros_type, "geometry_msgs/msg/Twist");
                assert_eq!(stamp_nanos, -42);
                assert_eq!(cdr, &[1, 2, 3, 4]);
            }
            other => panic!("expected topic sample, got {other:?}"),
        }
    }

    #[test]
    fn hello_and_heartbeat_round_trip() {
        let mut buf = [0u8; 64];
        let n = encode_hello(&mut buf, "mcu").unwrap();
        assert_eq!(
            parse_payload(&buf[HEADER_LEN..n]).unwrap(),
            FrameRef::Hello { peer: "mcu" }
        );
        let n = encode_heartbeat(&mut buf, 999).unwrap();
        assert_eq!(
            parse_payload(&buf[HEADER_LEN..n]).unwrap(),
            FrameRef::Heartbeat { stamp_nanos: 999 }
        );
    }

    #[test]
    fn short_buffer_is_reported() {
        let mut buf = [0u8; 4];
        assert_eq!(encode_ack(&mut buf, 1), Err(Error::ShortBuffer));
    }

    #[test]
    fn bad_magic_and_truncation_are_reported() {
        assert_eq!(frame_len(&[0u8; HEADER_LEN]), Err(Error::BadMagic));
        assert_eq!(parse_payload(&[kind::ACK, 0, 0]), Err(Error::Truncated));
        assert_eq!(parse_payload(&[0xFF]), Err(Error::BadKind));
    }
}

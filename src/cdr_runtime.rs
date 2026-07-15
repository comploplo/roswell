// CDR (Classic CDR / XCDR1) runtime. This file is `include!`d by `src/cdr.rs`
// for in-crate unit tests, and embedded verbatim into generated bindings so
// they are self-contained. Keep it free of inner attributes (`//!`) and of
// anything that depends on the rest of the crate.

/// CDR representation identifier (first 2 bytes of the encapsulation header).
const REPR_CDR_BE: [u8; 2] = [0x00, 0x00];
const REPR_CDR_LE: [u8; 2] = [0x00, 0x01];

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Endian {
    Little,
    Big,
}

impl Endian {
    /// The native endianness of the host.
    pub const NATIVE: Endian = if cfg!(target_endian = "little") {
        Endian::Little
    } else {
        Endian::Big
    };
}

/// Tracks the alignment origin so padding is computed relative to the body, not
/// the raw buffer (the encapsulation header sits before the origin).
#[derive(Debug, Clone, Copy)]
struct Cursor {
    origin: usize,
}

impl Cursor {
    /// Bytes needed to align `pos` to `a` relative to the body origin.
    fn padding(self, pos: usize, a: usize) -> usize {
        pad_to(pos - self.origin, a)
    }
}

/// Padding bytes to advance an offset `off` (measured from the alignment
/// origin) up to the next multiple of `a`. `a` is always a CDR primitive
/// alignment (1, 2, 4, or 8) — a nonzero power of two — at every call site.
///
/// This is the exact formula machine-checked (Creusot) as
/// `roscmp_verify::pad_to`: given `a > 0` it never panics (no
/// division-by-zero, no underflow in `a - off % a`), the result is strictly
/// less than `a`, and `off + result` is an exact multiple of `a`. The embedded
/// copy here is held identical by `cdr::tests::pad_to_matches_verified_core`.
/// See `docs/RT.md`.
fn pad_to(off: usize, a: usize) -> usize {
    (a - (off % a)) % a
}

/// Serializes values into a CDR byte buffer.
#[derive(Debug)]
pub struct Writer {
    buf: Vec<u8>,
    endian: Endian,
    cursor: Cursor,
}

impl Writer {
    /// Start a new message, emitting the encapsulation header.
    pub fn new(endian: Endian) -> Self {
        let mut buf = Vec::with_capacity(64);
        buf.extend_from_slice(match endian {
            Endian::Little => &REPR_CDR_LE,
            Endian::Big => &REPR_CDR_BE,
        });
        buf.extend_from_slice(&[0x00, 0x00]); // options
        let origin = buf.len();
        Writer {
            buf,
            endian,
            cursor: Cursor { origin },
        }
    }

    /// Consume the writer and return the full message (header + body).
    pub fn finish(self) -> Vec<u8> {
        self.buf
    }

    fn align(&mut self, a: usize) {
        let pad = self.cursor.padding(self.buf.len(), a);
        self.buf.resize(self.buf.len() + pad, 0);
    }

    fn put(&mut self, le: &[u8], be: &[u8]) {
        match self.endian {
            Endian::Little => self.buf.extend_from_slice(le),
            Endian::Big => self.buf.extend_from_slice(be),
        }
    }

    pub fn write_u8(&mut self, v: u8) {
        self.buf.push(v);
    }
    pub fn write_i8(&mut self, v: i8) {
        self.buf.push(v as u8);
    }
    pub fn write_bool(&mut self, v: bool) {
        self.buf.push(u8::from(v));
    }

    pub fn write_u16(&mut self, v: u16) {
        self.align(2);
        self.put(&v.to_le_bytes(), &v.to_be_bytes());
    }
    pub fn write_i16(&mut self, v: i16) {
        self.write_u16(v as u16);
    }

    pub fn write_u32(&mut self, v: u32) {
        self.align(4);
        self.put(&v.to_le_bytes(), &v.to_be_bytes());
    }
    pub fn write_i32(&mut self, v: i32) {
        self.write_u32(v as u32);
    }
    pub fn write_f32(&mut self, v: f32) {
        self.write_u32(v.to_bits());
    }

    pub fn write_u64(&mut self, v: u64) {
        self.align(8);
        self.put(&v.to_le_bytes(), &v.to_be_bytes());
    }
    pub fn write_i64(&mut self, v: i64) {
        self.write_u64(v as u64);
    }
    pub fn write_f64(&mut self, v: f64) {
        self.write_u64(v.to_bits());
    }

    /// Write a `string`: `uint32` length incl. NUL, bytes, then NUL.
    pub fn write_string(&mut self, s: &str) {
        let bytes = s.as_bytes();
        self.write_u32((bytes.len() + 1) as u32);
        self.buf.extend_from_slice(bytes);
        self.buf.push(0);
    }

    /// Write a sequence length prefix (`uint32` element count).
    pub fn write_seq_len(&mut self, n: usize) {
        self.write_u32(n as u32);
    }
}

/// Error reading a CDR buffer.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum CdrError {
    Truncated,
    BadEncapsulation,
    BadUtf8,
    BadString,
}

impl std::fmt::Display for CdrError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let s = match self {
            CdrError::Truncated => "unexpected end of CDR buffer",
            CdrError::BadEncapsulation => "invalid CDR encapsulation header",
            CdrError::BadUtf8 => "string is not valid UTF-8",
            CdrError::BadString => "string is missing its NUL terminator",
        };
        f.write_str(s)
    }
}

impl std::error::Error for CdrError {}

/// Deserializes values from a CDR byte buffer.
#[derive(Debug)]
pub struct Reader<'a> {
    buf: &'a [u8],
    pos: usize,
    endian: Endian,
    cursor: Cursor,
}

impl<'a> Reader<'a> {
    /// Parse the encapsulation header and position at the body start.
    pub fn new(buf: &'a [u8]) -> Result<Self, CdrError> {
        if buf.len() < 4 {
            return Err(CdrError::BadEncapsulation);
        }
        let endian = match [buf[0], buf[1]] {
            REPR_CDR_LE => Endian::Little,
            REPR_CDR_BE => Endian::Big,
            _ => return Err(CdrError::BadEncapsulation),
        };
        Ok(Reader {
            buf,
            pos: 4,
            endian,
            cursor: Cursor { origin: 4 },
        })
    }

    pub fn endian(&self) -> Endian {
        self.endian
    }

    fn align(&mut self, a: usize) {
        self.pos += self.cursor.padding(self.pos, a);
    }

    fn take(&mut self, n: usize) -> Result<&'a [u8], CdrError> {
        let end = self.pos.checked_add(n).ok_or(CdrError::Truncated)?;
        let slice = self.buf.get(self.pos..end).ok_or(CdrError::Truncated)?;
        self.pos = end;
        Ok(slice)
    }

    fn get<const N: usize>(&mut self, a: usize) -> Result<[u8; N], CdrError> {
        self.align(a);
        let slice = self.take(N)?;
        let mut arr = [0u8; N];
        arr.copy_from_slice(slice);
        Ok(arr)
    }

    fn decode_u16(&self, b: [u8; 2]) -> u16 {
        match self.endian {
            Endian::Little => u16::from_le_bytes(b),
            Endian::Big => u16::from_be_bytes(b),
        }
    }
    fn decode_u32(&self, b: [u8; 4]) -> u32 {
        match self.endian {
            Endian::Little => u32::from_le_bytes(b),
            Endian::Big => u32::from_be_bytes(b),
        }
    }
    fn decode_u64(&self, b: [u8; 8]) -> u64 {
        match self.endian {
            Endian::Little => u64::from_le_bytes(b),
            Endian::Big => u64::from_be_bytes(b),
        }
    }

    pub fn read_u8(&mut self) -> Result<u8, CdrError> {
        Ok(self.take(1)?[0])
    }
    pub fn read_i8(&mut self) -> Result<i8, CdrError> {
        Ok(self.read_u8()? as i8)
    }
    pub fn read_bool(&mut self) -> Result<bool, CdrError> {
        Ok(self.read_u8()? != 0)
    }

    pub fn read_u16(&mut self) -> Result<u16, CdrError> {
        let b = self.get::<2>(2)?;
        Ok(self.decode_u16(b))
    }
    pub fn read_i16(&mut self) -> Result<i16, CdrError> {
        Ok(self.read_u16()? as i16)
    }

    pub fn read_u32(&mut self) -> Result<u32, CdrError> {
        let b = self.get::<4>(4)?;
        Ok(self.decode_u32(b))
    }
    pub fn read_i32(&mut self) -> Result<i32, CdrError> {
        Ok(self.read_u32()? as i32)
    }
    pub fn read_f32(&mut self) -> Result<f32, CdrError> {
        Ok(f32::from_bits(self.read_u32()?))
    }

    pub fn read_u64(&mut self) -> Result<u64, CdrError> {
        let b = self.get::<8>(8)?;
        Ok(self.decode_u64(b))
    }
    pub fn read_i64(&mut self) -> Result<i64, CdrError> {
        Ok(self.read_u64()? as i64)
    }
    pub fn read_f64(&mut self) -> Result<f64, CdrError> {
        Ok(f64::from_bits(self.read_u64()?))
    }

    /// Read a `string` (length incl. NUL, bytes, NUL).
    pub fn read_string(&mut self) -> Result<String, CdrError> {
        let len = self.read_u32()? as usize;
        if len == 0 {
            return Err(CdrError::BadString);
        }
        let bytes = self.take(len)?;
        if bytes[len - 1] != 0 {
            return Err(CdrError::BadString);
        }
        std::str::from_utf8(&bytes[..len - 1])
            .map(str::to_string)
            .map_err(|_| CdrError::BadUtf8)
    }

    /// Read a sequence length prefix (`uint32` element count).
    ///
    /// The count is validated against the bytes actually left in the buffer
    /// (every element occupies at least one wire byte, even an empty nested
    /// message, which CDR encodes as a dummy octet) — so a malformed prefix
    /// cannot drive an attacker-sized allocation in any decode path.
    pub fn read_seq_len(&mut self) -> Result<usize, CdrError> {
        let n = self.read_u32()? as usize;
        if n > self.buf.len() - self.pos {
            return Err(CdrError::Truncated);
        }
        Ok(n)
    }
}

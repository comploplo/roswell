//! Runtime, layout-driven CDR codec over **C-ABI struct memory**.
//!
//! Where [`codegen`](crate::codegen) emits `#[repr(C)]` structs and their
//! `to_cdr`/`from_cdr` at compile time, this module interprets the same
//! layout-aware [`ir`](crate::ir) at *runtime* to (de)serialize a message that
//! is laid out exactly like those generated structs — scalars at their C
//! offsets, strings/sequences as `{ data, size, capacity }` triples, fixed
//! arrays inline, nested messages embedded by value. No codegen, no per-type
//! Rust code.
//!
//! It is the foundation for plain C-FFI + `ctypes` Python bindings: the Python
//! shim builds `ctypes.Structure` classes from the [`TypeLayout`] this module
//! exposes, allocates that memory, and calls [`DynamicType::encode`] /
//! [`decode`] / [`fini`] / [`init_default`] over it.
//!
//! The wire bytes are identical to the generated `to_cdr` because both read the
//! same field values and drive the same [`crate::cdr`] `Writer` primitives in
//! the same order; the memory layout is computed with the same `#[repr(C)]`
//! rules the codegen backends rely on (verified in `tests/layout_tests.rs`).
//! Both facts are proven against the generated types in `roscmp-dds/tests`.
#![allow(clippy::cast_ptr_alignment)]
// every raw access here is read/write_unaligned
// This module is one cohesive unsafe layer over C-ABI memory; each `unsafe fn`
// documents its invariants and its body is treated as one unsafe context.
#![allow(unsafe_op_in_unsafe_fn)]

use std::alloc::{Layout, alloc_zeroed, dealloc};
use std::collections::BTreeMap;
use std::mem::ManuallyDrop;
use std::path::{Path, PathBuf};

use crate::cdr::{CdrError, Encoding, Endian, Reader, Writer};
use crate::ir::{Element, Message, MsgId, Prim, Program, ResolvedType};

/// Size (and alignment) of a machine word — the ABI uses `usize`/pointer-sized
/// fields for the string/sequence triples, matching the generated structs.
const WORD: usize = std::mem::size_of::<usize>();

/// The C-ABI layout of one message: total `size`/`align` and per-field offsets.
/// Everything a Python shim needs to reconstruct the `ctypes.Structure`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TypeLayout {
    pub size: usize,
    pub align: usize,
    pub fields: Vec<FieldLayout>,
}

/// One field's placement and shape within its message.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FieldLayout {
    pub name: String,
    /// Byte offset of the field within the enclosing struct.
    pub offset: usize,
    pub multiplicity: Multiplicity,
    pub element: ElementLayout,
}

/// Whether a field holds one element, a fixed-size inline array, or a sequence
/// (a `{ data, size, capacity }` triple pointing at a heap buffer).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Multiplicity {
    Scalar,
    Array(usize),
    Sequence,
}

/// The element type of a field, with its C size/align (the buffer stride for
/// arrays and sequences).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ElementLayout {
    pub kind: ElemKind,
    /// Size in bytes of one element (a string/nested-message element is the
    /// triple / nested struct, not the pointed-to data).
    pub size: usize,
    pub align: usize,
}

/// What an element is.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ElemKind {
    Prim(Prim),
    /// `string`/`wstring`, laid out as the `{ data, size, capacity }` triple.
    /// `bound` is the declared upper length (`string<=N`), for the Python shim's
    /// validation; it does not affect the wire format or memory layout.
    String {
        wide: bool,
        bound: Option<usize>,
    },
    /// A nested message, identified for layout lookup.
    Message(MsgId),
}

/// An error from the runtime codec.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum DynError {
    /// A referenced nested message was not part of this type's closure.
    UnknownMessage(String),
    /// The CDR buffer was malformed or truncated while decoding.
    Cdr(CdrError),
}

impl std::fmt::Display for DynError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            DynError::UnknownMessage(id) => write!(f, "unknown nested message `{id}`"),
            DynError::Cdr(e) => write!(f, "cdr error: {e}"),
        }
    }
}

impl std::error::Error for DynError {}

impl From<CdrError> for DynError {
    fn from(e: CdrError) -> Self {
        DynError::Cdr(e)
    }
}

/// A runtime message type: a root message plus its dependency closure, with the
/// C-ABI layout of each precomputed. Drives encode/decode/fini/init over raw
/// struct memory laid out per that ABI.
#[derive(Debug, Clone)]
pub struct DynamicType {
    root: MsgId,
    messages: BTreeMap<MsgId, Message>,
    layouts: BTreeMap<MsgId, TypeLayout>,
}

impl DynamicType {
    /// Build a type for `root` from a resolved [`Program`] (which carries the
    /// dependency closure). Errors if `root` is absent.
    pub fn from_program(program: &Program, root: &MsgId) -> Result<Self, DynError> {
        let messages: BTreeMap<MsgId, Message> = program
            .messages
            .iter()
            .map(|m| (m.id.clone(), m.clone()))
            .collect();
        if !messages.contains_key(root) {
            return Err(DynError::UnknownMessage(id_str(root)));
        }
        // Program is topologically ordered by *by-value* deps, so nested-by-value
        // and fixed-array element messages have a computed layout already — but a
        // *sequence* element message (referenced by pointer) may be ordered later.
        // Its size doesn't affect the containing struct (a sequence field is just
        // the triple), so we compute every layout first, then patch each
        // message-element stride/align from the now-complete map.
        let mut layouts: BTreeMap<MsgId, TypeLayout> = BTreeMap::new();
        for msg in &program.messages {
            let layout = compute_layout(msg, &layouts);
            layouts.insert(msg.id.clone(), layout);
        }
        let sizes: BTreeMap<MsgId, (usize, usize)> = layouts
            .iter()
            .map(|(id, l)| (id.clone(), (l.size, l.align)))
            .collect();
        for layout in layouts.values_mut() {
            for field in &mut layout.fields {
                if let ElemKind::Message(id) = &field.element.kind
                    && let Some(&(size, align)) = sizes.get(id)
                {
                    field.element.size = size;
                    field.element.align = align;
                }
            }
        }
        Ok(Self {
            root: root.clone(),
            messages,
            layouts,
        })
    }

    /// The root message identity.
    #[must_use]
    pub fn root(&self) -> &MsgId {
        &self.root
    }

    /// The root message's C-ABI layout.
    #[must_use]
    pub fn layout(&self) -> &TypeLayout {
        &self.layouts[&self.root]
    }

    /// Total size in bytes of the root message struct.
    #[must_use]
    pub fn size(&self) -> usize {
        self.layout().size
    }

    /// Alignment in bytes of the root message struct.
    #[must_use]
    pub fn align(&self) -> usize {
        self.layout().align
    }

    /// The layout of any message in this type's closure (for nested types the
    /// Python shim also needs to model). `None` if not part of the closure.
    #[must_use]
    pub fn message_layout(&self, id: &MsgId) -> Option<&TypeLayout> {
        self.layouts.get(id)
    }

    /// Every message id in the closure (root plus dependencies).
    #[must_use]
    pub fn message_ids(&self) -> Vec<&MsgId> {
        self.layouts.keys().collect()
    }

    /// The ROS2 DDS type name, e.g. `sensor_msgs::msg::dds_::Imu_`.
    #[must_use]
    pub fn dds_type_name(&self) -> String {
        dds_type_name(&self.root)
    }

    fn layout_of(&self, id: &MsgId) -> Result<&TypeLayout, DynError> {
        self.layouts
            .get(id)
            .ok_or_else(|| DynError::UnknownMessage(id_str(id)))
    }

    // ---- allocation helpers --------------------------------------------

    /// Allocate zeroed, correctly-aligned memory for one root message and return
    /// a pointer to it. Pair with [`DynamicType::dealloc`] (call [`fini`] first
    /// if it was decoded/initialized). A zeroed buffer is a valid "empty"
    /// message: all triples read as null/0, safe to [`fini`].
    ///
    /// [`fini`]: DynamicType::fini
    #[must_use]
    pub fn alloc_zeroed(&self) -> *mut u8 {
        let (size, align) = (self.size().max(1), self.align());
        // Safe: size >= 1, align is a non-zero power of two from the ABI.
        unsafe { alloc_zeroed(Layout::from_size_align(size, align).unwrap()) }
    }

    /// Free memory from [`alloc_zeroed`](DynamicType::alloc_zeroed). Does **not**
    /// run [`fini_raw`](DynamicType::fini_raw); call that first if the message
    /// owns any string/sequence buffers.
    ///
    /// # Safety
    /// `ptr` must have come from this type's [`alloc_zeroed`](DynamicType::alloc_zeroed).
    pub unsafe fn dealloc(&self, ptr: *mut u8) {
        let (size, align) = (self.size().max(1), self.align());
        dealloc(ptr, Layout::from_size_align(size, align).unwrap());
    }

    // ---- encode ---------------------------------------------------------

    /// Encode the C-ABI message struct at `ptr` as a full CDR message (4-byte
    /// encapsulation header + body, XCDR1, little-endian). Byte-identical to the
    /// generated `to_cdr` for the same data. Currently infallible (the `Result`
    /// mirrors [`decode_raw`](DynamicType::decode_raw) for FFI symmetry).
    ///
    /// # Safety
    /// `ptr` must point to a valid, initialized instance of this message type
    /// laid out per the C ABI ([`layout`](DynamicType::layout)): every
    /// string/sequence triple must be a valid `{ data, size, capacity }` with
    /// `size` readable elements, and every string must be valid UTF-8.
    #[allow(clippy::unnecessary_wraps)] // Result reserved for FFI symmetry/validation
    pub unsafe fn encode(&self, ptr: *const u8) -> Result<Vec<u8>, DynError> {
        self.encode_as(ptr, Encoding::Xcdr1)
    }

    /// Like [`encode`](DynamicType::encode) but selecting the wire [`Encoding`]
    /// (little-endian). `Xcdr1` reproduces [`encode`] byte-for-byte; `Xcdr2`
    /// emits PLAIN_CDR2. [`decode`](DynamicType::decode) auto-detects either from
    /// the encapsulation header, so no matching decode entry point is needed.
    ///
    /// # Safety
    /// Same as [`encode`](DynamicType::encode).
    #[allow(clippy::unnecessary_wraps)] // Result reserved for FFI symmetry/validation
    pub unsafe fn encode_as(
        &self,
        ptr: *const u8,
        encoding: Encoding,
    ) -> Result<Vec<u8>, DynError> {
        let mut w = Writer::with_encoding(Endian::Little, encoding);
        self.encode_message(&self.root, ptr, &mut w);
        Ok(w.finish())
    }

    unsafe fn encode_message(&self, id: &MsgId, base: *const u8, w: &mut Writer) {
        // `layout_of` cannot fail: the closure is complete after construction.
        let layout = &self.layouts[id];
        for field in &layout.fields {
            let ptr = base.add(field.offset);
            match field.multiplicity {
                Multiplicity::Scalar => self.encode_element(&field.element.kind, ptr, w),
                Multiplicity::Array(len) => {
                    for i in 0..len {
                        self.encode_element(
                            &field.element.kind,
                            ptr.add(i * field.element.size),
                            w,
                        );
                    }
                }
                Multiplicity::Sequence => {
                    let data = load_ptr(ptr, 0);
                    let size = load_usize(ptr, WORD);
                    w.write_seq_len(size);
                    for i in 0..size {
                        self.encode_element(
                            &field.element.kind,
                            data.add(i * field.element.size),
                            w,
                        );
                    }
                }
            }
        }
    }

    unsafe fn encode_element(&self, kind: &ElemKind, ptr: *const u8, w: &mut Writer) {
        match kind {
            ElemKind::Prim(p) => encode_prim(*p, ptr, w),
            ElemKind::String { .. } => {
                let data = load_ptr(ptr, 0);
                let size = load_usize(ptr, WORD);
                w.write_string(read_ros_str(data, size));
            }
            ElemKind::Message(id) => self.encode_message(id, ptr, w),
        }
    }

    // ---- decode ---------------------------------------------------------

    /// Decode a full CDR message into caller-provided struct memory at `out`,
    /// allocating owned buffers for strings/sequences. The encoding (XCDR1 or
    /// XCDR2) and endianness are auto-detected from the encapsulation header.
    ///
    /// # Safety
    /// `out` must point to `self.size()` bytes, aligned to `self.align()`, and
    /// **zero-initialized** (see [`alloc_zeroed`](DynamicType::alloc_zeroed)).
    /// On success the message owns heap buffers; free with [`fini`] then the
    /// backing allocation. On error, `out` holds a partially-decoded message
    /// (unwritten fields remain zero) and is still safe to [`fini`].
    ///
    /// [`fini`]: DynamicType::fini
    pub unsafe fn decode(&self, cdr: &[u8], out: *mut u8) -> Result<(), DynError> {
        let mut r = Reader::new(cdr)?;
        self.decode_message(&self.root, &mut r, out)?;
        Ok(())
    }

    unsafe fn decode_message(
        &self,
        id: &MsgId,
        r: &mut Reader,
        base: *mut u8,
    ) -> Result<(), DynError> {
        let layout = self.layout_of(id)?;
        for field in &layout.fields {
            let ptr = base.add(field.offset);
            match field.multiplicity {
                Multiplicity::Scalar => self.decode_element(&field.element, r, ptr)?,
                Multiplicity::Array(len) => {
                    for i in 0..len {
                        self.decode_element(&field.element, r, ptr.add(i * field.element.size))?;
                    }
                }
                Multiplicity::Sequence => {
                    let n = r.read_seq_len()?;
                    let buf = alloc_buf(n, field.element.size, field.element.align);
                    // Link the (zeroed) buffer into `ptr` *before* decoding any
                    // element, so a mid-sequence decode error still leaves it
                    // reachable from `out` and thus freed by `fini` (the
                    // undecoded tail stays zeroed — safe to fini). Storing only
                    // after the loop leaked `buf` on the error path.
                    store_seq(ptr, buf, n);
                    for i in 0..n {
                        self.decode_element(&field.element, r, buf.add(i * field.element.size))?;
                    }
                }
            }
        }
        Ok(())
    }

    unsafe fn decode_element(
        &self,
        elem: &ElementLayout,
        r: &mut Reader,
        ptr: *mut u8,
    ) -> Result<(), DynError> {
        match &elem.kind {
            ElemKind::Prim(p) => decode_prim(*p, r, ptr)?,
            ElemKind::String { .. } => store_ros_string(ptr, &r.read_string()?),
            ElemKind::Message(id) => self.decode_message(id, r, ptr)?,
        }
        Ok(())
    }

    // ---- fini -----------------------------------------------------------

    /// Free every string/sequence buffer owned by the message at `ptr`,
    /// recursively, mirroring the generated `fini`. Leaves scalar fields
    /// untouched and resets freed triples so a second call is a no-op.
    ///
    /// # Safety
    /// `ptr` must point to a valid instance of this type whose owned buffers
    /// were produced by this codec's [`decode`](DynamicType::decode) /
    /// [`init_default`](DynamicType::init_default) (or the equivalent
    /// generated allocation, which uses the same global allocator).
    pub unsafe fn fini(&self, ptr: *mut u8) {
        self.fini_message(&self.root, ptr);
    }

    unsafe fn fini_message(&self, id: &MsgId, base: *mut u8) {
        let Some(layout) = self.layouts.get(id) else {
            return;
        };
        let layout = layout.clone();
        for field in &layout.fields {
            let ptr = base.add(field.offset);
            match field.multiplicity {
                Multiplicity::Scalar => self.fini_element(&field.element, ptr),
                Multiplicity::Array(len) => {
                    for i in 0..len {
                        self.fini_element(&field.element, ptr.add(i * field.element.size));
                    }
                }
                Multiplicity::Sequence => {
                    let data = load_ptr(ptr, 0);
                    let size = load_usize(ptr, WORD);
                    let cap = load_usize(ptr, 2 * WORD);
                    for i in 0..size {
                        self.fini_element(&field.element, data.add(i * field.element.size));
                    }
                    free_buf(data, cap, field.element.size, field.element.align);
                    store_seq_empty(ptr, field.element.align);
                }
            }
        }
    }

    unsafe fn fini_element(&self, elem: &ElementLayout, ptr: *mut u8) {
        match &elem.kind {
            ElemKind::Prim(_) => {}
            ElemKind::String { .. } => free_ros_string(ptr),
            ElemKind::Message(id) => self.fini_message(id, ptr),
        }
    }

    // ---- init_default ---------------------------------------------------

    /// Initialize the message at `out` to its `.msg` defaults: primitives take
    /// their declared default (else zero), strings default to `""` (or the
    /// declared literal), fixed arrays to zeroed/declared buffers, sequences to
    /// empty, nested messages recurse.
    ///
    /// # Safety
    /// `out` must point to `self.size()` bytes, aligned to `self.align()`, and
    /// **zero-initialized**. The message then owns heap buffers; free with
    /// [`fini`](DynamicType::fini) then the backing allocation.
    pub unsafe fn init_default(&self, out: *mut u8) {
        self.init_message(&self.root, out);
    }

    unsafe fn init_message(&self, id: &MsgId, base: *mut u8) {
        let Some(layout) = self.layouts.get(id) else {
            return;
        };
        let msg = self.messages[id].clone();
        let layout = layout.clone();
        for (field, resolved) in layout.fields.iter().zip(&msg.fields) {
            let ptr = base.add(field.offset);
            let default = resolved.default.as_ref();
            match field.multiplicity {
                Multiplicity::Scalar => self.init_scalar(&field.element, default, ptr),
                Multiplicity::Array(len) => self.init_array(&field.element, default, len, ptr),
                // Sequences default to empty (or an array literal, if declared).
                Multiplicity::Sequence => {
                    if let Some(crate::ast::Value::Array(items)) = default {
                        let n = items.len();
                        let buf = alloc_buf(n, field.element.size, field.element.align);
                        for (i, item) in items.iter().enumerate() {
                            self.init_from_ast(
                                &field.element,
                                Some(item),
                                buf.add(i * field.element.size),
                            );
                        }
                        store_seq(ptr, buf, n);
                    } else {
                        store_seq_empty(ptr, field.element.align);
                    }
                }
            }
        }
    }

    unsafe fn init_scalar(
        &self,
        elem: &ElementLayout,
        default: Option<&crate::ast::Value>,
        ptr: *mut u8,
    ) {
        self.init_from_ast(elem, default, ptr);
    }

    unsafe fn init_array(
        &self,
        elem: &ElementLayout,
        default: Option<&crate::ast::Value>,
        len: usize,
        ptr: *mut u8,
    ) {
        let items = match default {
            Some(crate::ast::Value::Array(items)) => items.as_slice(),
            _ => &[],
        };
        for i in 0..len {
            self.init_from_ast(elem, items.get(i), ptr.add(i * elem.size));
        }
    }

    /// Initialize one element from an optional `.msg` literal.
    unsafe fn init_from_ast(
        &self,
        elem: &ElementLayout,
        default: Option<&crate::ast::Value>,
        ptr: *mut u8,
    ) {
        match &elem.kind {
            ElemKind::Prim(p) => store_prim_default(*p, default, ptr),
            ElemKind::String { .. } => {
                let s = match default {
                    Some(crate::ast::Value::String(s)) => s.as_str(),
                    _ => "",
                };
                store_ros_string(ptr, s);
            }
            ElemKind::Message(id) => self.init_message(id, ptr),
        }
    }
}

// ---- layout computation -----------------------------------------------------

fn compute_layout(msg: &Message, layouts: &BTreeMap<MsgId, TypeLayout>) -> TypeLayout {
    let mut offset = 0usize;
    let mut align = 1usize;
    let mut fields = Vec::with_capacity(msg.fields.len());
    for f in &msg.fields {
        let element = elem_layout(field_element(&f.ty), layouts);
        let (fsize, falign, multiplicity) = match &f.ty {
            ResolvedType::Scalar(_) => (element.size, element.align, Multiplicity::Scalar),
            ResolvedType::Array { len, .. } => (
                element.size * *len,
                element.align,
                Multiplicity::Array(*len),
            ),
            // A sequence field is the {data,size,capacity} triple.
            ResolvedType::Sequence { .. } => (3 * WORD, WORD, Multiplicity::Sequence),
        };
        align = align.max(falign);
        offset = round_up(offset, falign);
        fields.push(FieldLayout {
            name: f.name.clone(),
            offset,
            multiplicity,
            element,
        });
        offset += fsize;
    }
    TypeLayout {
        size: round_up(offset, align),
        align,
        fields,
    }
}

/// The element behind any multiplicity.
fn field_element(ty: &ResolvedType) -> &Element {
    match ty {
        ResolvedType::Scalar(e)
        | ResolvedType::Array { elem: e, .. }
        | ResolvedType::Sequence { elem: e, .. } => e,
    }
}

fn elem_layout(e: &Element, layouts: &BTreeMap<MsgId, TypeLayout>) -> ElementLayout {
    match e {
        Element::Prim(p) => {
            let (size, align) = p.size_align();
            ElementLayout {
                kind: ElemKind::Prim(*p),
                size,
                align,
            }
        }
        Element::String { wide, bound } => ElementLayout {
            kind: ElemKind::String {
                wide: *wide,
                bound: *bound,
            },
            size: 3 * WORD,
            align: WORD,
        },
        Element::Message(id) => {
            // Present for by-value nesting (topo order); sequence/array element
            // messages are resolved for encode via the full map. Fall back to a
            // zero placeholder only if somehow absent (cannot happen post-build).
            let (size, align) = layouts.get(id).map_or((0, 1), |l| (l.size, l.align));
            ElementLayout {
                kind: ElemKind::Message(id.clone()),
                size,
                align,
            }
        }
    }
}

fn round_up(off: usize, align: usize) -> usize {
    debug_assert!(align.is_power_of_two());
    // Machine-checked (Creusot): `off <= result < off + align`, `result` is a
    // multiple of `align`, and the arithmetic never overflows for `align > 0`
    // with `off + align <= usize::MAX`. See `roscmp_verify::round_up`.
    roscmp_verify::round_up(off, align)
}

// ---- raw memory access ------------------------------------------------------

unsafe fn load_ptr(base: *const u8, off: usize) -> *mut u8 {
    base.add(off).cast::<*mut u8>().read_unaligned()
}

unsafe fn load_usize(base: *const u8, off: usize) -> usize {
    base.add(off).cast::<usize>().read_unaligned()
}

unsafe fn store_ptr(base: *mut u8, off: usize, v: *mut u8) {
    base.add(off).cast::<*mut u8>().write_unaligned(v);
}

unsafe fn store_usize(base: *mut u8, off: usize, v: usize) {
    base.add(off).cast::<usize>().write_unaligned(v);
}

/// Write a `{ data, size, capacity }` triple with `size == capacity == n`.
unsafe fn store_seq(base: *mut u8, data: *mut u8, n: usize) {
    store_ptr(base, 0, data);
    store_usize(base, WORD, n);
    store_usize(base, 2 * WORD, n);
}

/// Write an empty triple with a dangling but aligned data pointer (matching
/// `RosSequence::alloc(Vec::new())`), so `fini` treats it as owning nothing.
unsafe fn store_seq_empty(base: *mut u8, elem_align: usize) {
    store_ptr(base, 0, elem_align as *mut u8);
    store_usize(base, WORD, 0);
    store_usize(base, 2 * WORD, 0);
}

/// Borrow a `RosString`'s bytes as `&str`, matching `RosString::as_str`.
unsafe fn read_ros_str<'a>(data: *mut u8, size: usize) -> &'a str {
    if data.is_null() || size == 0 {
        return "";
    }
    std::str::from_utf8_unchecked(std::slice::from_raw_parts(data, size))
}

/// Allocate an owning `RosString` for `s` and write its triple at `ptr`.
/// Mirrors `RosString::alloc` exactly so `fini`/generated `free` agree.
unsafe fn store_ros_string(ptr: *mut u8, s: &str) {
    let mut bytes = s.as_bytes().to_vec();
    bytes.push(0);
    let size = bytes.len() - 1;
    let mut bytes = ManuallyDrop::new(bytes);
    store_ptr(ptr, 0, bytes.as_mut_ptr());
    store_usize(ptr, WORD, size);
    store_usize(ptr, 2 * WORD, bytes.capacity());
}

/// Free a `RosString` triple at `ptr` and reset it. Mirrors `RosString::free`
/// (reconstruct a full-capacity `Vec<u8>` to drop the whole allocation).
#[allow(clippy::same_length_and_capacity)]
unsafe fn free_ros_string(ptr: *mut u8) {
    let data = load_ptr(ptr, 0);
    let cap = load_usize(ptr, 2 * WORD);
    if !data.is_null() && cap > 0 {
        drop(Vec::from_raw_parts(data, cap, cap));
    }
    store_ptr(ptr, 0, core::ptr::null_mut());
    store_usize(ptr, WORD, 0);
    store_usize(ptr, 2 * WORD, 0);
}

/// Allocate a zeroed, `elem_align`-aligned buffer for `n` elements of `stride`
/// bytes. `n == 0` yields a dangling aligned pointer (no allocation), matching
/// an empty `Vec<T>`. Compatible with `Vec<T>`'s allocation, so the generated
/// `fini` (which reconstitutes a `Vec<T>`) can also free it.
unsafe fn alloc_buf(n: usize, stride: usize, elem_align: usize) -> *mut u8 {
    if n == 0 || stride == 0 {
        return elem_align as *mut u8;
    }
    let layout = Layout::from_size_align(n * stride, elem_align).unwrap();
    let p = alloc_zeroed(layout);
    assert!(!p.is_null(), "allocation failed for sequence buffer");
    p
}

/// Free a sequence element buffer of `cap` elements. `Layout::from_size_align`
/// here equals `Layout::array::<T>(cap)`, so this pairs with both `alloc_buf`
/// and a generated `Vec<T>` allocation.
unsafe fn free_buf(data: *mut u8, cap: usize, stride: usize, elem_align: usize) {
    if cap > 0 && stride > 0 {
        dealloc(
            data,
            Layout::from_size_align(cap * stride, elem_align).unwrap(),
        );
    }
}

/// Overwrite the primitive `{ data, size, capacity }` sequence triple at
/// `triple` with `count` elements copied from `src`, each `elem_size` bytes and
/// `elem_align`-aligned. Any buffer the triple previously owned is freed first.
///
/// This is the write path for the C-FFI / Python `numpy`-array assignment: the
/// buffer is allocated (and the old one freed) here, in Rust, using the same
/// allocator contract as [`DynamicType::decode`], so the codec's own
/// [`fini`](DynamicType::fini) can later free it. **Primitive elements only** —
/// the freed buffer is released without running element finalizers, so it must
/// not be used for string- or message-element sequences.
///
/// # Safety
/// `triple` must point at a valid sequence triple whose element type is a
/// primitive of `(elem_size, elem_align)`. `src` must be readable for
/// `count * elem_size` bytes (ignored when `count == 0`).
pub unsafe fn assign_prim_sequence(
    triple: *mut u8,
    elem_size: usize,
    elem_align: usize,
    src: *const u8,
    count: usize,
) {
    let old_data = load_ptr(triple, 0);
    let old_cap = load_usize(triple, 2 * WORD);
    free_buf(old_data, old_cap, elem_size, elem_align);
    if count == 0 {
        store_seq_empty(triple, elem_align);
        return;
    }
    let buf = alloc_buf(count, elem_size, elem_align);
    core::ptr::copy_nonoverlapping(src, buf, count * elem_size);
    store_seq(triple, buf, count);
}

/// Overwrite the ROS string `{ data, size, capacity }` triple at `triple` with
/// the UTF-8 bytes `src[..len]` (invalid UTF-8 is replaced with the empty
/// string), freeing any buffer the triple previously owned. Allocation stays in
/// Rust so the codec's [`fini`](DynamicType::fini) can free it. The write path
/// for setting a string field from the C-FFI / Python boundary.
///
/// # Safety
/// `triple` must point at a valid ROS string triple. `src` must be readable for
/// `len` bytes (ignored when `len == 0`).
pub unsafe fn assign_string(triple: *mut u8, src: *const u8, len: usize) {
    free_ros_string(triple);
    let s = if src.is_null() || len == 0 {
        ""
    } else {
        std::str::from_utf8(std::slice::from_raw_parts(src, len)).unwrap_or("")
    };
    store_ros_string(triple, s);
}

// ---- primitive read/write ---------------------------------------------------

/// Read the primitive at `ptr` (host-native) and write it to the CDR stream.
unsafe fn encode_prim(p: Prim, ptr: *const u8, w: &mut Writer) {
    match p {
        Prim::Bool => w.write_bool(ptr.read_unaligned() != 0),
        Prim::Byte | Prim::Char | Prim::Uint8 => w.write_u8(ptr.read_unaligned()),
        Prim::Int8 => w.write_i8(ptr.cast::<i8>().read_unaligned()),
        Prim::Uint16 => w.write_u16(ptr.cast::<u16>().read_unaligned()),
        Prim::Int16 => w.write_i16(ptr.cast::<i16>().read_unaligned()),
        Prim::Uint32 => w.write_u32(ptr.cast::<u32>().read_unaligned()),
        Prim::Int32 => w.write_i32(ptr.cast::<i32>().read_unaligned()),
        Prim::Uint64 => w.write_u64(ptr.cast::<u64>().read_unaligned()),
        Prim::Int64 => w.write_i64(ptr.cast::<i64>().read_unaligned()),
        Prim::Float32 => w.write_f32(ptr.cast::<f32>().read_unaligned()),
        Prim::Float64 => w.write_f64(ptr.cast::<f64>().read_unaligned()),
    }
}

/// Read the next primitive from the CDR stream and store it at `ptr`.
unsafe fn decode_prim(p: Prim, r: &mut Reader, ptr: *mut u8) -> Result<(), CdrError> {
    match p {
        Prim::Bool => ptr.write_unaligned(u8::from(r.read_bool()?)),
        Prim::Byte | Prim::Char | Prim::Uint8 => ptr.write_unaligned(r.read_u8()?),
        Prim::Int8 => ptr.cast::<i8>().write_unaligned(r.read_i8()?),
        Prim::Uint16 => ptr.cast::<u16>().write_unaligned(r.read_u16()?),
        Prim::Int16 => ptr.cast::<i16>().write_unaligned(r.read_i16()?),
        Prim::Uint32 => ptr.cast::<u32>().write_unaligned(r.read_u32()?),
        Prim::Int32 => ptr.cast::<i32>().write_unaligned(r.read_i32()?),
        Prim::Uint64 => ptr.cast::<u64>().write_unaligned(r.read_u64()?),
        Prim::Int64 => ptr.cast::<i64>().write_unaligned(r.read_i64()?),
        Prim::Float32 => ptr.cast::<f32>().write_unaligned(r.read_f32()?),
        Prim::Float64 => ptr.cast::<f64>().write_unaligned(r.read_f64()?),
    }
    Ok(())
}

/// Store a primitive default (from a `.msg` literal, else zero) at `ptr`.
#[allow(clippy::cast_precision_loss)]
unsafe fn store_prim_default(p: Prim, default: Option<&crate::ast::Value>, ptr: *mut u8) {
    use crate::ast::Value as A;
    let as_i = |x: Option<&A>| match x {
        Some(A::Integer(i)) => *i,
        Some(A::Bool(b)) => i128::from(*b),
        _ => 0,
    };
    let as_f = |x: Option<&A>| match x {
        Some(A::Float(f)) => *f,
        Some(A::Integer(i)) => *i as f64,
        _ => 0.0,
    };
    match p {
        Prim::Bool => {
            let b = matches!(default, Some(A::Bool(true))) || as_i(default) != 0;
            ptr.write_unaligned(u8::from(b));
        }
        Prim::Byte | Prim::Char | Prim::Uint8 => ptr.write_unaligned(as_i(default) as u8),
        Prim::Int8 => ptr.cast::<i8>().write_unaligned(as_i(default) as i8),
        Prim::Uint16 => ptr.cast::<u16>().write_unaligned(as_i(default) as u16),
        Prim::Int16 => ptr.cast::<i16>().write_unaligned(as_i(default) as i16),
        Prim::Uint32 => ptr.cast::<u32>().write_unaligned(as_i(default) as u32),
        Prim::Int32 => ptr.cast::<i32>().write_unaligned(as_i(default) as i32),
        Prim::Uint64 => ptr.cast::<u64>().write_unaligned(as_i(default) as u64),
        Prim::Int64 => ptr.cast::<i64>().write_unaligned(as_i(default) as i64),
        Prim::Float32 => ptr.cast::<f32>().write_unaligned(as_f(default) as f32),
        Prim::Float64 => ptr.cast::<f64>().write_unaligned(as_f(default)),
    }
}

// ---- shared -----------------------------------------------------------------

fn id_str(id: &MsgId) -> String {
    format!("{}/{}", id.package, id.name)
}

/// The ROS2 DDS type string for a message, e.g. `std_msgs::msg::dds_::String_`.
/// Mirrors `codegen::rust`'s naming so runtime and generated endpoints agree.
fn dds_type_name(id: &MsgId) -> String {
    let ns = if id.name.contains("_SendGoal_")
        || id.name.contains("_GetResult_")
        || id.name.ends_with("_Goal")
        || id.name.ends_with("_Result")
        || id.name.ends_with("_Feedback")
        || id.name.ends_with("_FeedbackMessage")
    {
        "action"
    } else if id.name.ends_with("_Request") || id.name.ends_with("_Response") {
        "srv"
    } else {
        "msg"
    };
    format!("{}::{ns}::dds_::{}_", id.package, id.name)
}

// ---- file loader ------------------------------------------------------------

/// An error from the [`load_message`] / [`load_service`] file loaders.
#[derive(Debug)]
pub enum LoadError {
    Io(std::io::Error),
    Parse(String),
    Resolve(crate::ResolveError),
    Build(DynError),
}

impl std::fmt::Display for LoadError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            LoadError::Io(e) => write!(f, "io error: {e}"),
            LoadError::Parse(e) => write!(f, "parse error: {e}"),
            LoadError::Resolve(e) => write!(f, "{e}"),
            LoadError::Build(e) => write!(f, "{e}"),
        }
    }
}

impl std::error::Error for LoadError {}

impl From<std::io::Error> for LoadError {
    fn from(e: std::io::Error) -> Self {
        LoadError::Io(e)
    }
}
impl From<crate::ResolveError> for LoadError {
    fn from(e: crate::ResolveError) -> Self {
        LoadError::Resolve(e)
    }
}
impl From<DynError> for LoadError {
    fn from(e: DynError) -> Self {
        LoadError::Build(e)
    }
}

/// Load a `.msg` file (plus any dependency `.msg`/`.srv`/`.action`/`.idl` files
/// needed to resolve its non-builtin references) into a [`DynamicType`].
/// Builtins (`Time`, `Duration`, `Header`, `UUID`) are injected automatically.
pub fn load_message(
    path: impl AsRef<Path>,
    deps: &[impl AsRef<Path>],
) -> Result<DynamicType, LoadError> {
    let path = path.as_ref();
    let root = infer_id(path);
    let mut inputs = parse_file(path)?;
    for dep in deps {
        inputs.extend(parse_file(dep.as_ref())?);
    }
    let program = crate::resolve(inputs)?;
    Ok(DynamicType::from_program(&program, &root)?)
}

/// Load a `.srv` file into its request and response [`DynamicType`]s.
pub fn load_service(
    path: impl AsRef<Path>,
    deps: &[impl AsRef<Path>],
) -> Result<(DynamicType, DynamicType), LoadError> {
    let path = path.as_ref();
    let id = infer_id(path);
    let src = std::fs::read_to_string(path)?;
    let svc = crate::parse_service(&src).map_err(|e| LoadError::Parse(e.to_string()))?;
    let mut inputs = crate::service_messages(&id.package, &id.name, &svc);
    for dep in deps {
        inputs.extend(parse_file(dep.as_ref())?);
    }
    let program = crate::resolve(inputs)?;
    let req = MsgId::new(&id.package, format!("{}_Request", id.name));
    let resp = MsgId::new(&id.package, format!("{}_Response", id.name));
    Ok((
        DynamicType::from_program(&program, &req)?,
        DynamicType::from_program(&program, &resp)?,
    ))
}

/// Parse one interface file into `(id, spec)` inputs, honoring its extension.
fn parse_file(path: &Path) -> Result<Vec<(MsgId, crate::ast::MessageSpec)>, LoadError> {
    let src = std::fs::read_to_string(path)?;
    let id = infer_id(path);
    let parse = |e: crate::ParseError| LoadError::Parse(e.to_string());
    match path.extension().and_then(|s| s.to_str()) {
        Some("srv") => {
            let svc = crate::parse_service(&src).map_err(parse)?;
            Ok(crate::service_messages(&id.package, &id.name, &svc))
        }
        Some("action") => {
            let act = crate::parse_action(&src).map_err(parse)?;
            Ok(crate::action_messages(&id.package, &id.name, &act))
        }
        Some("idl") => crate::parse_idl(&src).map_err(parse),
        _ => Ok(vec![(id, crate::parse_message(&src).map_err(parse)?)]),
    }
}

/// Infer `(package, Name)` from an interface path using the ROS layout
/// `<package>/{msg,srv,action}/<Name>.<ext>`.
fn infer_id(path: &Path) -> MsgId {
    let name = path
        .file_stem()
        .and_then(|s| s.to_str())
        .unwrap_or("Unnamed")
        .to_string();
    let parent = path.parent();
    let parent_name = parent.and_then(|p| p.file_name()).and_then(|s| s.to_str());
    let package = match parent_name {
        Some("msg" | "srv" | "action") => parent
            .and_then(Path::parent)
            .and_then(|p| p.file_name())
            .and_then(|s| s.to_str()),
        other => other,
    }
    .unwrap_or("pkg")
    .to_string();
    MsgId { package, name }
}

/// Convenience: a path under the repo's `samples/` directory, resolved relative
/// to the crate manifest (used by tests and the Python bindings' fixtures).
#[must_use]
pub fn sample_path(rel: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("samples")
        .join(rel)
}

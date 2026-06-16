//! Resolved, layout-aware intermediate representation.
//!
//! The AST mirrors `.msg` syntax; the IR is what the codegen backends consume.
//! Every type reference has been resolved to a concrete element, messages are
//! ordered so dependencies precede dependents, and each primitive knows its
//! C-ABI size/alignment and its spelling in each target language.

use crate::ast::Value;

/// A fully-resolved set of messages, in dependency (definition) order.
#[derive(Debug, Clone)]
pub struct Program {
    pub messages: Vec<Message>,
}

/// Fully-qualified message identity: `package` + CamelCase `name`.
#[derive(Debug, Clone, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct MsgId {
    pub package: String,
    pub name: String,
}

impl MsgId {
    pub fn new(package: impl Into<String>, name: impl Into<String>) -> Self {
        MsgId {
            package: package.into(),
            name: name.into(),
        }
    }
}

#[derive(Debug, Clone)]
pub struct Message {
    pub id: MsgId,
    pub constants: Vec<ResolvedConstant>,
    pub fields: Vec<ResolvedField>,
}

#[derive(Debug, Clone)]
pub struct ResolvedConstant {
    pub name: String,
    pub prim: ConstType,
    pub value: Value,
}

/// The type of a constant — primitives or (unbounded) string only.
#[derive(Debug, Clone, PartialEq)]
pub enum ConstType {
    Prim(Prim),
    String,
}

#[derive(Debug, Clone)]
pub struct ResolvedField {
    pub name: String,
    pub ty: ResolvedType,
    pub default: Option<Value>,
}

#[derive(Debug, Clone)]
pub enum ResolvedType {
    /// A single value.
    Scalar(Element),
    /// `type[N]` — fixed-size inline array.
    Array { elem: Element, len: usize },
    /// `type[]` or `type[<=N]` — a dynamic sequence. Both share one ABI: a
    /// `{ data, size, capacity }` triple. `bound` is kept for validation only.
    Sequence { elem: Element, bound: Option<usize> },
}

#[derive(Debug, Clone)]
pub enum Element {
    Prim(Prim),
    /// `string`/`wstring`; laid out as the runtime string triple regardless of
    /// `bound`. `wide` distinguishes `wstring` for future UTF-16 handling.
    String {
        bound: Option<usize>,
        wide: bool,
    },
    /// A nested message, embedded by value (scalar/array) or by pointer
    /// (sequence).
    Message(MsgId),
}

/// A fixed-layout primitive type.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Prim {
    Bool,
    Byte,
    Char,
    Int8,
    Uint8,
    Int16,
    Uint16,
    Int32,
    Uint32,
    Int64,
    Uint64,
    Float32,
    Float64,
}

impl Prim {
    /// (size, alignment) in bytes — identical across the C/Rust/Python targets,
    /// which is what makes the bindings ABI-compatible.
    pub fn size_align(self) -> (usize, usize) {
        use Prim::*;
        match self {
            Bool | Byte | Char | Int8 | Uint8 => (1, 1),
            Int16 | Uint16 => (2, 2),
            Int32 | Uint32 | Float32 => (4, 4),
            Int64 | Uint64 | Float64 => (8, 8),
        }
    }

    /// Spelling as a Rust type.
    pub fn rust(self) -> &'static str {
        use Prim::*;
        match self {
            Bool => "bool",
            Byte | Uint8 | Char => "u8",
            Int8 => "i8",
            Int16 => "i16",
            Uint16 => "u16",
            Int32 => "i32",
            Uint32 => "u32",
            Int64 => "i64",
            Uint64 => "u64",
            Float32 => "f32",
            Float64 => "f64",
        }
    }

    /// Spelling as a C99 (`<stdint.h>`/`<stdbool.h>`) type.
    pub fn c(self) -> &'static str {
        use Prim::*;
        match self {
            Bool => "bool",
            Byte | Uint8 | Char => "uint8_t",
            Int8 => "int8_t",
            Int16 => "int16_t",
            Uint16 => "uint16_t",
            Int32 => "int32_t",
            Uint32 => "uint32_t",
            Int64 => "int64_t",
            Uint64 => "uint64_t",
            Float32 => "float",
            Float64 => "double",
        }
    }

    /// CDR method stem: pairs with `write_`/`read_` in the CDR runtime
    /// (e.g. `Int32` -> `"i32"` -> `write_i32`/`read_i32`).
    pub fn cdr_fn(self) -> &'static str {
        use Prim::*;
        match self {
            Bool => "bool",
            Byte | Uint8 | Char => "u8",
            Int8 => "i8",
            Int16 => "i16",
            Uint16 => "u16",
            Int32 => "i32",
            Uint32 => "u32",
            Int64 => "i64",
            Uint64 => "u64",
            Float32 => "f32",
            Float64 => "f64",
        }
    }

    /// Spelling as a Python `ctypes` type.
    pub fn ctypes(self) -> &'static str {
        use Prim::*;
        match self {
            Bool => "ctypes.c_bool",
            Byte | Uint8 | Char => "ctypes.c_uint8",
            Int8 => "ctypes.c_int8",
            Int16 => "ctypes.c_int16",
            Uint16 => "ctypes.c_uint16",
            Int32 => "ctypes.c_int32",
            Uint32 => "ctypes.c_uint32",
            Int64 => "ctypes.c_int64",
            Uint64 => "ctypes.c_uint64",
            Float32 => "ctypes.c_float",
            Float64 => "ctypes.c_double",
        }
    }
}

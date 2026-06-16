//! Abstract syntax tree for a single ROS2 `.msg` definition.
//!
//! One `.msg` file parses into one [`MessageSpec`]: an ordered list of
//! constants followed by an ordered list of fields (ROS keeps field order
//! because it is significant for the wire layout).

/// A parsed message definition.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct MessageSpec {
    pub constants: Vec<Constant>,
    pub fields: Vec<Field>,
}

/// A parsed `.srv` definition: request and response, split by a `---` line.
#[derive(Debug, Clone, PartialEq)]
pub struct ServiceSpec {
    pub request: MessageSpec,
    pub response: MessageSpec,
}

/// A parsed `.action` definition: goal, result, and feedback, split by `---`.
#[derive(Debug, Clone, PartialEq)]
pub struct ActionSpec {
    pub goal: MessageSpec,
    pub result: MessageSpec,
    pub feedback: MessageSpec,
}

/// A named, typed field: `type name [default]`.
#[derive(Debug, Clone, PartialEq)]
pub struct Field {
    pub ty: FieldType,
    pub name: String,
    pub default: Option<Value>,
}

/// A compile-time constant: `type NAME=value`.
///
/// Constants are restricted to primitive scalar types (incl. strings) by the
/// ROS interface grammar — no arrays, no nested messages.
#[derive(Debug, Clone, PartialEq)]
pub struct Constant {
    pub ty: BaseType,
    pub name: String,
    pub value: Value,
}

/// A field's full type: a base type with an optional array modifier.
#[derive(Debug, Clone, PartialEq)]
pub struct FieldType {
    pub base: BaseType,
    pub array: Option<ArrayKind>,
}

impl FieldType {
    pub fn scalar(base: BaseType) -> Self {
        FieldType { base, array: None }
    }
}

/// The element type, before any array modifier is applied.
#[derive(Debug, Clone, PartialEq)]
pub enum BaseType {
    Bool,
    /// ROS2 `byte` — an octet, laid out as `u8`.
    Byte,
    /// ROS2 `char` — an unsigned 8-bit character, laid out as `u8`.
    Char,
    Float32,
    Float64,
    Int8,
    Uint8,
    Int16,
    Uint16,
    Int32,
    Uint32,
    Int64,
    Uint64,
    /// `string` or bounded `string<=N`.
    String {
        bound: Option<usize>,
    },
    /// `wstring` or bounded `wstring<=N`.
    WString {
        bound: Option<usize>,
    },
    /// A reference to another message type, e.g. `geometry_msgs/Point` or a
    /// bare `Point` (same-package / relative).
    Named(TypeName),
    /// ROS1 `time` builtin (mapped to `builtin_interfaces/Time`).
    Time,
    /// ROS1 `duration` builtin (mapped to `builtin_interfaces/Duration`).
    Duration,
}

impl BaseType {
    /// True for the numeric/bool primitives that have a fixed scalar layout.
    pub fn is_numeric(&self) -> bool {
        use BaseType::*;
        matches!(
            self,
            Bool | Byte
                | Char
                | Float32
                | Float64
                | Int8
                | Uint8
                | Int16
                | Uint16
                | Int32
                | Uint32
                | Int64
                | Uint64
        )
    }
}

/// A possibly-namespaced type reference.
///
/// `package` is `None` for bare relative references (`Point`), and `Some` for
/// fully-qualified ones (`geometry_msgs/Point`, `geometry_msgs/msg/Point`).
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct TypeName {
    pub package: Option<String>,
    pub name: String,
}

/// How an array modifier `[...]` bounds the collection.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ArrayKind {
    /// `type[N]` — fixed-length.
    Fixed(usize),
    /// `type[<=N]` — dynamic, upper-bounded.
    Bounded(usize),
    /// `type[]` — dynamic, unbounded.
    Unbounded,
}

/// A literal value used for a constant or a field default.
///
/// Integers are widened to `i128` so the single variant can hold every ROS
/// integer type including `uint64`.
#[derive(Debug, Clone, PartialEq)]
pub enum Value {
    Bool(bool),
    Integer(i128),
    Float(f64),
    String(String),
    Array(Vec<Value>),
}

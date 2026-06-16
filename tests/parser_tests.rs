use roscmp::ast::*;
use roscmp::parse_message;

fn field(spec: &MessageSpec, name: &str) -> Field {
    spec.fields
        .iter()
        .find(|f| f.name == name)
        .unwrap_or_else(|| panic!("no field `{name}`"))
        .clone()
}

#[test]
fn geometry_msgs_point() {
    // geometry_msgs/msg/Point
    let src = "float64 x\nfloat64 y\nfloat64 z\n";
    let spec = parse_message(src).unwrap();
    assert_eq!(spec.fields.len(), 3);
    assert!(spec.constants.is_empty());
    for n in ["x", "y", "z"] {
        assert_eq!(field(&spec, n).ty, FieldType::scalar(BaseType::Float64));
    }
}

#[test]
fn comments_and_blank_lines_ignored() {
    let src = "\
# This is a point in free space.
float64 x  # the x coordinate
float64 y

float64 z
";
    let spec = parse_message(src).unwrap();
    assert_eq!(spec.fields.len(), 3);
}

#[test]
fn hash_inside_string_is_not_a_comment() {
    let src = r#"string greeting "hello #world""#;
    let spec = parse_message(src).unwrap();
    assert_eq!(
        field(&spec, "greeting").default,
        Some(Value::String("hello #world".into()))
    );
}

#[test]
fn std_msgs_header_uses_namespaced_and_bare_types() {
    // ROS2 std_msgs/msg/Header
    let src = "builtin_interfaces/Time stamp\nstring frame_id\n";
    let spec = parse_message(src).unwrap();
    let stamp = field(&spec, "stamp");
    assert_eq!(
        stamp.ty.base,
        BaseType::Named(TypeName {
            package: Some("builtin_interfaces".into()),
            name: "Time".into()
        })
    );
    assert_eq!(
        field(&spec, "frame_id").ty.base,
        BaseType::String { bound: None }
    );
}

#[test]
fn pkg_msg_type_form_drops_into_package() {
    let src = "geometry_msgs/msg/Point position\n";
    let spec = parse_message(src).unwrap();
    assert_eq!(
        field(&spec, "position").ty.base,
        BaseType::Named(TypeName {
            package: Some("geometry_msgs/msg".into()),
            name: "Point".into()
        })
    );
}

#[test]
fn bare_relative_type() {
    let src = "Point position\n";
    let spec = parse_message(src).unwrap();
    assert_eq!(
        field(&spec, "position").ty.base,
        BaseType::Named(TypeName {
            package: None,
            name: "Point".into()
        })
    );
}

#[test]
fn array_kinds() {
    let src = "\
float64[] dynamic
float64[36] fixed
float64[<=10] bounded
";
    let spec = parse_message(src).unwrap();
    assert_eq!(field(&spec, "dynamic").ty.array, Some(ArrayKind::Unbounded));
    assert_eq!(field(&spec, "fixed").ty.array, Some(ArrayKind::Fixed(36)));
    assert_eq!(
        field(&spec, "bounded").ty.array,
        Some(ArrayKind::Bounded(10))
    );
}

#[test]
fn bounded_strings_and_string_arrays() {
    let src = "\
string<=10 short
string[5] names
string<=8[<=3] matrix
";
    let spec = parse_message(src).unwrap();
    assert_eq!(
        field(&spec, "short").ty.base,
        BaseType::String { bound: Some(10) }
    );
    assert_eq!(field(&spec, "short").ty.array, None);

    let names = field(&spec, "names");
    assert_eq!(names.ty.base, BaseType::String { bound: None });
    assert_eq!(names.ty.array, Some(ArrayKind::Fixed(5)));

    let matrix = field(&spec, "matrix");
    assert_eq!(matrix.ty.base, BaseType::String { bound: Some(8) });
    assert_eq!(matrix.ty.array, Some(ArrayKind::Bounded(3)));
}

#[test]
fn constants() {
    let src = "\
int32 X=123
int32 Y=-7
string GREETING=hi there
float64 RATE=2.5
bool ENABLED=true
";
    let spec = parse_message(src).unwrap();
    assert_eq!(spec.constants.len(), 5);
    assert!(spec.fields.is_empty());

    let c = |n: &str| spec.constants.iter().find(|c| c.name == n).unwrap().clone();
    assert_eq!(c("X").value, Value::Integer(123));
    assert_eq!(c("Y").value, Value::Integer(-7));
    // Unquoted string constant keeps trailing content verbatim.
    assert_eq!(c("GREETING").value, Value::String("hi there".into()));
    assert_eq!(c("RATE").value, Value::Float(2.5));
    assert_eq!(c("ENABLED").value, Value::Bool(true));
}

#[test]
fn field_defaults() {
    let src = "\
int32 count 5
string name \"robot\"
bool active true
float64 ratio 0.5
";
    let spec = parse_message(src).unwrap();
    assert_eq!(field(&spec, "count").default, Some(Value::Integer(5)));
    assert_eq!(
        field(&spec, "name").default,
        Some(Value::String("robot".into()))
    );
    assert_eq!(field(&spec, "active").default, Some(Value::Bool(true)));
    assert_eq!(field(&spec, "ratio").default, Some(Value::Float(0.5)));
}

#[test]
fn array_default() {
    let src = "int32[] data [1, 2, 3]\n";
    let spec = parse_message(src).unwrap();
    assert_eq!(
        field(&spec, "data").default,
        Some(Value::Array(vec![
            Value::Integer(1),
            Value::Integer(2),
            Value::Integer(3),
        ]))
    );
}

#[test]
fn byte_and_char_aliases() {
    let src = "byte b\nchar c\n";
    let spec = parse_message(src).unwrap();
    assert_eq!(field(&spec, "b").ty.base, BaseType::Byte);
    assert_eq!(field(&spec, "c").ty.base, BaseType::Char);
}

#[test]
fn rejects_identifier_starting_with_keyword() {
    // `int32x` is a (relative) type name, not the primitive `int32`.
    let src = "int32x value\n";
    let spec = parse_message(src).unwrap();
    assert_eq!(
        field(&spec, "value").ty.base,
        BaseType::Named(TypeName {
            package: None,
            name: "int32x".into()
        })
    );
}

#[test]
fn error_reports_line_number() {
    let src = "float64 x\nthis is not valid\nfloat64 z\n";
    let err = parse_message(src).unwrap_err();
    assert_eq!(err.line, 2);
}

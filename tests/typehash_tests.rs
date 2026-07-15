//! RIHS01 type hashes verified byte-exact against `ros:jazzy` (captured from the
//! installed type-description JSON / `ros2 topic info -v` on 2026-06-15).

use roscmp::ir::MsgId;
use roscmp::typehash::{type_description_json, type_hash};
use roscmp::{
    action_messages, parse_action, parse_message, parse_service, resolve, service_messages,
};

fn program(defs: &[(&str, &str, &str)]) -> Vec<roscmp::ir::Message> {
    let inputs = defs
        .iter()
        .map(|(p, n, s)| (MsgId::new(*p, *n), parse_message(s).unwrap()))
        .collect();
    resolve(inputs).unwrap().messages
}

fn hash_of(defs: &[(&str, &str, &str)], pkg: &str, name: &str) -> String {
    let msgs = program(defs);
    type_hash(&msgs, &MsgId::new(pkg, name)).unwrap()
}

#[test]
fn string_hash() {
    let h = hash_of(
        &[("std_msgs", "String", "string data\n")],
        "std_msgs",
        "String",
    );
    assert_eq!(
        h,
        "RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18"
    );
}

#[test]
fn point_hash() {
    let h = hash_of(
        &[(
            "geometry_msgs",
            "Point",
            "float64 x\nfloat64 y\nfloat64 z\n",
        )],
        "geometry_msgs",
        "Point",
    );
    assert_eq!(
        h,
        "RIHS01_6963084842a9b04494d6b2941d11444708d892da2f4b09843b9c43f42a7f6881"
    );
}

#[test]
fn time_hash() {
    // builtin_interfaces/Time is auto-injected by the resolver.
    let h = hash_of(
        &[(
            "std_msgs",
            "Header",
            "builtin_interfaces/Time stamp\nstring frame_id\n",
        )],
        "builtin_interfaces",
        "Time",
    );
    assert_eq!(
        h,
        "RIHS01_b106235e25a4c5ed35098aa0a61a3ee9c9b18d197f398b0e4206cea9acf9c197"
    );
}

#[test]
fn header_hash_with_referenced_type() {
    // Exercises a nested field (type_id 1) + a referenced_type_descriptions entry.
    let h = hash_of(
        &[(
            "std_msgs",
            "Header",
            "builtin_interfaces/Time stamp\nstring frame_id\n",
        )],
        "std_msgs",
        "Header",
    );
    assert_eq!(
        h,
        "RIHS01_f49fb3ae2cf070f793645ff749683ac6b06203e41c891e17701b1cb597ce6a01"
    );
}

#[test]
fn multiarray_hash_sequence_and_sorted_refs() {
    // std_msgs/Float32MultiArray: a sequence field (float32[] -> type_id 154),
    // a nested type, and TWO referenced types that must be sorted by name.
    let defs = &[
        (
            "std_msgs",
            "MultiArrayDimension",
            "string label\nuint32 size\nuint32 stride\n",
        ),
        (
            "std_msgs",
            "MultiArrayLayout",
            "MultiArrayDimension[] dim\nuint32 data_offset\n",
        ),
        (
            "std_msgs",
            "Float32MultiArray",
            "MultiArrayLayout layout\nfloat32[] data\n",
        ),
    ];
    let h = hash_of(defs, "std_msgs", "Float32MultiArray");
    assert_eq!(
        h,
        "RIHS01_0599f6f85b4bfca379873a0b4375a0aca022156bd2d7021275d116ed1fa8bfe0"
    );
}

#[test]
fn generated_service_type_description_uses_srv_namespace() {
    let svc = parse_service("int64 a\n---\nint64 b\n").unwrap();
    let msgs = resolve(service_messages("demo", "Add", &svc))
        .unwrap()
        .messages;
    let json = type_description_json(&msgs, &MsgId::new("demo", "Add_Request")).unwrap();
    assert!(json.contains("\"type_name\": \"demo/srv/Add_Request\""));
}

#[test]
fn generated_action_type_description_uses_action_namespace() {
    let act = parse_action("bool goal\n---\nbool result\n---\nbool feedback\n").unwrap();
    let msgs = resolve(action_messages("demo", "Wave", &act))
        .unwrap()
        .messages;
    let json = type_description_json(&msgs, &MsgId::new("demo", "Wave_SendGoal_Request")).unwrap();
    assert!(json.contains("\"type_name\": \"demo/action/Wave_SendGoal_Request\""));
    assert!(json.contains("\"nested_type_name\": \"demo/action/Wave_Goal\""));
}

#[test]
fn dds_codegen_emits_type_description_registry_helper() {
    let msgs = program(&[
        (
            "geometry_msgs",
            "Point",
            "float64 x\nfloat64 y\nfloat64 z\n",
        ),
        (
            "demo",
            "StampedPoint",
            "std_msgs/Header header\ngeometry_msgs/Point point\n",
        ),
    ]);
    let code = roscmp::codegen::rust::generate_dds(&roscmp::ir::Program { messages: msgs });
    assert!(code.contains("pub fn register_type_descriptions"));
    assert!(code.contains("demo/msg/StampedPoint"));
    assert!(code.contains("std_msgs/msg/Header"));
    assert!(code.contains("geometry_msgs/msg/Point"));
}

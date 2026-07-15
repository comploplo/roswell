//! `.srv` / `.action` parsing and ROS1 `time`/`duration` support, through to
//! resolution and (for the service) compiling the generated Rust.

use std::process::Command;

use roscmp::ast::*;
use roscmp::codegen;
use roscmp::ir::Element;
use roscmp::{action_messages, parse_action, parse_service, resolve, service_messages};

#[test]
fn service_splits_request_and_response() {
    let src = "\
int64 a
int64 b
---
int64 sum
";
    let svc = parse_service(src).unwrap();
    assert_eq!(svc.request.fields.len(), 2);
    assert_eq!(svc.response.fields.len(), 1);
    assert_eq!(svc.response.fields[0].name, "sum");
}

#[test]
fn service_requires_one_separator() {
    // No `---` => only one section.
    let err = parse_service("int64 a\n").unwrap_err();
    assert!(err.message.contains("2 sections"));
}

#[test]
fn action_splits_three_sections() {
    let src = "\
# goal
geometry_msgs/Point target
---
# result
bool reached
---
# feedback
float64 distance
";
    let act = parse_action(src).unwrap();
    assert_eq!(act.goal.fields[0].name, "target");
    assert_eq!(act.result.fields[0].name, "reached");
    assert_eq!(act.feedback.fields[0].name, "distance");
}

#[test]
fn ros1_time_and_duration_parse_and_resolve() {
    let src = "time stamp\nduration timeout\n";
    let spec = roscmp::parse_message(src).unwrap();
    assert_eq!(spec.fields[0].ty.base, BaseType::Time);
    assert_eq!(spec.fields[1].ty.base, BaseType::Duration);

    // They resolve to the builtin_interfaces messages.
    let program = resolve(vec![(roscmp::ir::MsgId::new("demo", "Stamped"), spec)]).unwrap();
    let stamped = program
        .messages
        .iter()
        .find(|m| m.id.name == "Stamped")
        .unwrap();
    match &stamped.fields[0].ty {
        roscmp::ir::ResolvedType::Scalar(Element::Message(id)) => {
            assert_eq!(id.package, "builtin_interfaces");
            assert_eq!(id.name, "Time");
        }
        other => panic!("expected Time message, got {other:?}"),
    }
}

#[test]
fn time_is_not_a_keyword_in_field_position() {
    // `float64 time` => field NAMED time, not the `time` builtin.
    let spec = roscmp::parse_message("float64 time\n").unwrap();
    assert_eq!(spec.fields[0].name, "time");
    assert_eq!(spec.fields[0].ty.base, BaseType::Float64);
}

#[test]
fn service_messages_generate_and_compile() {
    let src = "\
int64 a
int64 b
---
int64 sum
";
    let svc = parse_service(src).unwrap();
    let inputs = service_messages("example_interfaces", "AddTwoInts", &svc);
    let program = resolve(inputs).unwrap();

    // The two expected messages exist.
    let names: Vec<_> = program.messages.iter().map(|m| m.id.name.clone()).collect();
    assert!(names.contains(&"AddTwoInts_Request".to_string()));
    assert!(names.contains(&"AddTwoInts_Response".to_string()));

    // Generated Rust (with CDR) compiles.
    let code = codegen::rust::generate(&program);
    let dir = std::env::temp_dir().join("roscmp_srv");
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    let rs = dir.join("gen.rs");
    std::fs::write(&rs, &code).unwrap();
    let out = Command::new("rustc")
        .args(["--edition", "2021", "--crate-type", "lib"])
        .arg(&rs)
        .arg("-o")
        .arg(dir.join("libgen.rlib"))
        .output()
        .expect("run rustc");
    assert!(
        out.status.success(),
        "generated service Rust failed to compile:\n{}",
        String::from_utf8_lossy(&out.stderr)
    );
}

#[test]
fn action_messages_expand_to_ros2_wrappers() {
    let act = parse_action("bool g\n---\nbool r\n---\nbool f\n").unwrap();
    let inputs = action_messages("demo", "Wave", &act);
    let names: Vec<_> = inputs.iter().map(|(id, _)| id.name.clone()).collect();
    assert_eq!(
        names,
        [
            "Wave_Goal",
            "Wave_Result",
            "Wave_Feedback",
            "Wave_SendGoal_Request",
            "Wave_SendGoal_Response",
            "Wave_GetResult_Request",
            "Wave_GetResult_Response",
            "Wave_FeedbackMessage",
        ]
    );

    let program = resolve(inputs).unwrap();
    let code = codegen::rust::generate(&program);
    let dir = std::env::temp_dir().join("roscmp_action");
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    let rs = dir.join("gen.rs");
    std::fs::write(&rs, &code).unwrap();
    let out = Command::new("rustc")
        .args(["--edition", "2021", "--crate-type", "lib"])
        .arg(&rs)
        .arg("-o")
        .arg(dir.join("libgen.rlib"))
        .output()
        .expect("run rustc");
    assert!(
        out.status.success(),
        "generated action Rust failed to compile:\n{}",
        String::from_utf8_lossy(&out.stderr)
    );
}

#[test]
fn dds_action_wrappers_compile_with_runtime_traits() {
    let act = parse_action("bool g\n---\nbool r\n---\nbool f\n").unwrap();
    let program = resolve(action_messages("demo", "Wave", &act)).unwrap();
    let mut code = codegen::rust::generate_dds(&program);
    code.push_str(
        r"
pub mod codec {
    #[derive(Debug)]
    pub struct CodecError(pub &'static str);
    impl core::fmt::Display for CodecError {
        fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result { f.write_str(self.0) }
    }
    impl std::error::Error for CodecError {}
    pub trait CdrMsg: Sized {
        const TYPE_NAME: &'static str;
        fn encode(&self) -> Vec<u8>;
        fn decode(buf: &[u8]) -> Result<Self, CodecError>;
    }
}
pub mod time {
    pub struct Time;
    impl Time {
        pub fn to_msg(self) -> crate::builtin_interfaces__Time {
            crate::builtin_interfaces__Time { sec: 0, nanosec: 0 }
        }
    }
}
pub mod action {
    #[derive(Clone, Copy)]
    pub struct GoalId(pub [u8; 16]);
    #[repr(i8)]
    pub enum GoalStatus { Accepted = 1, Executing = 2, Succeeded = 4 }
    pub trait SendGoalRequest { type Goal; fn goal_id(&self) -> GoalId; fn goal(&self) -> &Self::Goal; }
    pub trait SendGoalResponse { fn new(accepted: bool, stamp: crate::time::Time) -> Self; }
    pub trait GetResultRequest { fn goal_id(&self) -> GoalId; }
    pub trait GetResultResponse { type Result; fn new(status: GoalStatus, result: Self::Result) -> Self; }
    pub trait FeedbackMessage { type Feedback; fn new(goal_id: GoalId, feedback: Self::Feedback) -> Self; }
}
pub mod type_description {
    #[derive(Default)]
    pub struct TypeDescriptionRegistry;
    impl TypeDescriptionRegistry { pub fn insert(&mut self, _data: TypeDescriptionData) {} }
    pub struct TypeDescriptionData {
        pub type_hash: String,
        pub type_description: IndividualTypeDescriptionData,
        pub referenced_type_descriptions: Vec<IndividualTypeDescriptionData>,
        pub type_sources: Vec<TypeSourceData>,
        pub extra_information: Vec<KeyValueData>,
    }
    pub struct IndividualTypeDescriptionData { pub type_name: String, pub fields: Vec<FieldData> }
    pub struct FieldData {
        pub name: String,
        pub type_id: u8,
        pub capacity: u64,
        pub string_capacity: u64,
        pub nested_type_name: String,
        pub default_value: String,
    }
    pub struct TypeSourceData { pub type_name: String, pub encoding: String, pub raw_file_contents: String }
    pub struct KeyValueData { pub key: String, pub value: String }
}
pub fn _uses_action_traits(
    req: &demo__Wave_SendGoal_Request,
) -> crate::action::GoalId {
    <demo__Wave_SendGoal_Request as crate::action::SendGoalRequest>::goal_id(req)
}
",
    );
    let dir = std::env::temp_dir().join("roscmp_dds_action");
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    let rs = dir.join("gen.rs");
    std::fs::write(&rs, &code).unwrap();
    let out = Command::new("rustc")
        .args(["--edition", "2021", "--crate-type", "lib"])
        .arg(&rs)
        .arg("-o")
        .arg(dir.join("libgen.rlib"))
        .output()
        .expect("run rustc");
    assert!(
        out.status.success(),
        "generated DDS action Rust failed to compile:\n{}",
        String::from_utf8_lossy(&out.stderr)
    );
}

use roswell::ast::*;
use roswell::ir::MsgId;
use roswell::{codegen, parse_idl, parse_message, resolve};
use std::process::Command;

fn field<'a>(spec: &'a MessageSpec, name: &str) -> &'a Field {
    spec.fields.iter().find(|f| f.name == name).unwrap()
}

#[test]
fn point_idl_matches_msg() {
    let idl = "
module geometry_msgs {
  module msg {
    struct Point {
      double x;
      double y;
      double z;
    };
  };
};";
    let msgs = parse_idl(idl).unwrap();
    assert_eq!(msgs.len(), 1);
    let (id, spec) = &msgs[0];
    assert_eq!(*id, MsgId::new("geometry_msgs", "Point"));

    // Identical to the .msg frontend.
    let from_msg = parse_message("float64 x\nfloat64 y\nfloat64 z\n").unwrap();
    assert_eq!(spec.fields, from_msg.fields);
}

#[test]
fn idl_features() {
    let idl = r#"
module demo_msgs {
  module msg {
    module Telemetry_Constants {
      const uint8 STATE_IDLE = 0;
      const string ROBOT = "turtlebot";
    };
    @verbatim (language="comment", text="telemetry")
    struct Telemetry {
      std_msgs::msg::Header header;
      uint8 state;
      string name;
      string<10> short_name;
      double covariance[36];
      sequence<int32> readings;
      sequence<int32, 5> bounded;
      sequence<geometry_msgs::msg::Point> waypoints;
    };
  };
};"#;
    let msgs = parse_idl(idl).unwrap();
    let (_, spec) = msgs.iter().find(|(id, _)| id.name == "Telemetry").unwrap();

    // Constants from the `_Constants` module are attached.
    assert_eq!(spec.constants.len(), 2);
    let idle = spec
        .constants
        .iter()
        .find(|c| c.name == "STATE_IDLE")
        .unwrap();
    assert_eq!(idle.value, Value::Integer(0));
    let robot = spec.constants.iter().find(|c| c.name == "ROBOT").unwrap();
    assert_eq!(robot.value, Value::String("turtlebot".into()));

    assert_eq!(
        field(spec, "header").ty.base,
        BaseType::Named(TypeName {
            package: Some("std_msgs".into()),
            name: "Header".into()
        })
    );
    assert_eq!(field(spec, "state").ty.base, BaseType::Uint8);
    assert_eq!(
        field(spec, "name").ty.base,
        BaseType::String { bound: None }
    );
    assert_eq!(
        field(spec, "short_name").ty.base,
        BaseType::String { bound: Some(10) }
    );
    let cov = field(spec, "covariance");
    assert_eq!(cov.ty.base, BaseType::Float64);
    assert_eq!(cov.ty.array, Some(ArrayKind::Fixed(36)));
    assert_eq!(field(spec, "readings").ty.array, Some(ArrayKind::Unbounded));
    assert_eq!(field(spec, "bounded").ty.array, Some(ArrayKind::Bounded(5)));
    let wp = field(spec, "waypoints");
    assert_eq!(
        wp.ty.base,
        BaseType::Named(TypeName {
            package: Some("geometry_msgs".into()),
            name: "Point".into()
        })
    );
    assert_eq!(wp.ty.array, Some(ArrayKind::Unbounded));
}

#[test]
fn traditional_idl_integer_spellings() {
    let idl = "
module p {
  module msg {
    struct T {
      unsigned long long a;
      long long b;
      unsigned short c;
      short d;
      unsigned long e;
      long f;
      boolean g;
      octet h;
    };
  };
};";
    let (_, spec) = &parse_idl(idl).unwrap()[0];
    let b = |n: &str| field(spec, n).ty.base.clone();
    assert_eq!(b("a"), BaseType::Uint64);
    assert_eq!(b("b"), BaseType::Int64);
    assert_eq!(b("c"), BaseType::Uint16);
    assert_eq!(b("d"), BaseType::Int16);
    assert_eq!(b("e"), BaseType::Uint32);
    assert_eq!(b("f"), BaseType::Int32);
    assert_eq!(b("g"), BaseType::Bool);
    assert_eq!(b("h"), BaseType::Byte);
}

#[test]
fn idl_resolves_and_codegens() {
    let idl = "
module geometry_msgs {
  module msg {
    struct Point { double x; double y; double z; };
    struct PoseStamped {
      std_msgs::msg::Header header;
      sequence<geometry_msgs::msg::Point> path;
    };
  };
};";
    let inputs = parse_idl(idl).unwrap();
    let program = resolve(inputs).unwrap();
    let code = codegen::rust::generate(&program);

    let dir = std::env::temp_dir().join("roswell_idl");
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
        .expect("rustc");
    assert!(
        out.status.success(),
        "generated IDL Rust failed to compile:\n{}",
        String::from_utf8_lossy(&out.stderr)
    );
}

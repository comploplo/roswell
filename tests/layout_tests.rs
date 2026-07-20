//! End-to-end layout verification.
//!
//! The whole point of the C-ABI core is that the Rust, C, and Python bindings
//! for a message share one memory layout. These tests prove it empirically:
//! generate all three, compile/run each to report `sizeof` and per-field
//! offsets, and assert the three agree.
//!
//! Requires a C compiler (`cc`) and `python3` on PATH.

use std::process::Command;

use roswell::codegen;
use roswell::ir::{MsgId, Program};
use roswell::{parse_message, resolve};

/// Build a program from inline `(package, Name, source)` triples.
fn program_from(defs: &[(&str, &str, &str)]) -> Program {
    let inputs = defs
        .iter()
        .map(|(pkg, name, src)| {
            let spec = parse_message(src).expect("parse");
            (MsgId::new(*pkg, *name), spec)
        })
        .collect();
    resolve(inputs).expect("resolve")
}

fn tmp_dir(tag: &str) -> std::path::PathBuf {
    let dir = std::env::temp_dir().join(format!("roswell_layout_{tag}"));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

/// `(field_name, offset)` pairs plus the total size for one struct.
#[derive(Debug, PartialEq, Eq)]
struct Layout {
    size: usize,
    fields: Vec<(String, usize)>,
}

/// Compile the generated Rust and report the layout of `symbol`'s fields.
fn rust_layout(program: &Program, symbol: &str, fields: &[&str], dir: &std::path::Path) -> Layout {
    let code = codegen::rust::generate(program);
    let src = dir.join("gen.rs");
    std::fs::write(&src, &code).unwrap();

    let mut main = code;
    main.push_str("\nfn main() {\n");
    main.push_str(&format!(
        "    println!(\"{{}}\", core::mem::size_of::<{symbol}>());\n"
    ));
    for f in fields {
        main.push_str(&format!(
            "    println!(\"{f} {{}}\", core::mem::offset_of!({symbol}, {f}));\n"
        ));
    }
    main.push_str("}\n");
    let main_path = dir.join("main.rs");
    std::fs::write(&main_path, main).unwrap();

    let bin = dir.join("rust_bin");
    let status = Command::new("rustc")
        .args(["--edition", "2021", "-O"])
        .arg(&main_path)
        .arg("-o")
        .arg(&bin)
        .status()
        .expect("run rustc");
    assert!(status.success(), "generated Rust failed to compile");

    let out = Command::new(&bin).output().expect("run rust bin");
    assert!(out.status.success());
    parse_layout(&String::from_utf8_lossy(&out.stdout))
}

/// Compile the generated C header and report the layout via `offsetof`.
fn c_layout(program: &Program, symbol: &str, fields: &[&str], dir: &std::path::Path) -> Layout {
    let header = codegen::c::generate(program);
    std::fs::write(dir.join("roswell_msgs.h"), &header).unwrap();

    let mut main =
        String::from("#include <stdio.h>\n#include <stddef.h>\n#include \"roswell_msgs.h\"\n");
    main.push_str("int main(void) {\n");
    main.push_str(&format!("  printf(\"%zu\\n\", sizeof({symbol}));\n"));
    for f in fields {
        main.push_str(&format!(
            "  printf(\"{f} %zu\\n\", offsetof({symbol}, {f}));\n"
        ));
    }
    main.push_str("  return 0;\n}\n");
    let main_path = dir.join("main.c");
    std::fs::write(&main_path, main).unwrap();

    let bin = dir.join("c_bin");
    let status = Command::new("cc")
        .arg(&main_path)
        .arg("-I")
        .arg(dir)
        .arg("-o")
        .arg(&bin)
        .status()
        .expect("run cc");
    assert!(status.success(), "generated C failed to compile");

    let out = Command::new(&bin).output().expect("run c bin");
    assert!(out.status.success());
    parse_layout(&String::from_utf8_lossy(&out.stdout))
}

/// Import the generated Python module and report the layout via `ctypes`.
fn python_layout(
    program: &Program,
    symbol: &str,
    fields: &[&str],
    dir: &std::path::Path,
) -> Layout {
    let module = codegen::python::generate(program);
    std::fs::write(dir.join("roswell_msgs.py"), &module).unwrap();

    let mut script = String::from("import ctypes, roswell_msgs as m\n");
    script.push_str(&format!("print(ctypes.sizeof(m.{symbol}))\n"));
    for f in fields {
        script.push_str(&format!("print('{f}', m.{symbol}.{f}.offset)\n"));
    }
    let script_path = dir.join("check.py");
    std::fs::write(&script_path, script).unwrap();

    let out = Command::new("python3")
        .arg(&script_path)
        .env("PYTHONPATH", dir)
        .output()
        .expect("run python3");
    assert!(
        out.status.success(),
        "generated Python failed: {}",
        String::from_utf8_lossy(&out.stderr)
    );
    parse_layout(&String::from_utf8_lossy(&out.stdout))
}

fn parse_layout(stdout: &str) -> Layout {
    let mut lines = stdout.lines();
    let size = lines.next().unwrap().trim().parse().unwrap();
    let fields = lines
        .map(|l| {
            let (name, off) = l.trim().rsplit_once(' ').unwrap();
            (name.to_string(), off.parse().unwrap())
        })
        .collect();
    Layout { size, fields }
}

/// Assert all three backends produce the same layout for `symbol`.
fn assert_layouts_agree(program: &Program, symbol: &str, fields: &[&str], tag: &str) {
    let dir = tmp_dir(tag);
    let r = rust_layout(program, symbol, fields, &dir);
    let c = c_layout(program, symbol, fields, &dir);
    let p = python_layout(program, symbol, fields, &dir);
    assert_eq!(r, c, "Rust vs C layout mismatch for {symbol}");
    assert_eq!(c, p, "C vs Python layout mismatch for {symbol}");
    assert!(r.size > 0);
}

#[test]
fn point_layout_agrees() {
    let program = program_from(&[(
        "geometry_msgs",
        "Point",
        "float64 x\nfloat64 y\nfloat64 z\n",
    )]);
    assert_layouts_agree(&program, "geometry_msgs__Point", &["x", "y", "z"], "point");
}

#[test]
fn mixed_scalar_alignment_agrees() {
    // Deliberately misordered widths to exercise padding/alignment.
    let src = "uint8 a\nfloat64 b\nuint16 c\nint32 d\nuint8 e\n";
    let program = program_from(&[("demo_msgs", "Mixed", src)]);
    assert_layouts_agree(
        &program,
        "demo_msgs__Mixed",
        &["a", "b", "c", "d", "e"],
        "mixed",
    );
}

#[test]
fn nested_and_string_layout_agrees() {
    // std_msgs/Header pulls in builtin_interfaces/Time and a string.
    let src = "std_msgs/Header header\nstring name\n";
    let program = program_from(&[("demo_msgs", "Named", src)]);
    assert_layouts_agree(&program, "demo_msgs__Named", &["header", "name"], "named");
}

#[test]
fn arrays_and_sequences_layout_agrees() {
    let src = "\
float64[36] covariance
int32[] readings
geometry_msgs/Point[] waypoints
geometry_msgs/Point single
";
    let program = program_from(&[
        (
            "geometry_msgs",
            "Point",
            "float64 x\nfloat64 y\nfloat64 z\n",
        ),
        ("demo_msgs", "Bundle", src),
    ]);
    assert_layouts_agree(
        &program,
        "demo_msgs__Bundle",
        &["covariance", "readings", "waypoints", "single"],
        "bundle",
    );
}

//! FFI conformance: the generated Rust library is the single CDR serializer;
//! C and Python call into it. This compiles the generated Rust as a `cdylib`,
//! then serializes the same message from C and from Python and asserts both
//! produce the exact bytes the Rust serializer does.

use std::process::Command;

use roscmp::codegen;
use roscmp::ir::MsgId;
use roscmp::{parse_message, resolve};

const POINT: (&str, &str, &str) = (
    "geometry_msgs",
    "Point",
    "float64 x\nfloat64 y\nfloat64 z\n",
);

fn dylib_ext() -> &'static str {
    if cfg!(target_os = "macos") {
        "dylib"
    } else if cfg!(target_os = "windows") {
        "dll"
    } else {
        "so"
    }
}

/// Generate all three backends into `dir` and compile the Rust as a cdylib.
/// Returns the path to the shared library.
fn build_lib(defs: &[(&str, &str, &str)], dir: &std::path::Path) -> std::path::PathBuf {
    let inputs = defs
        .iter()
        .map(|(pkg, name, src)| (MsgId::new(*pkg, *name), parse_message(src).expect("parse")))
        .collect();
    let program = resolve(inputs).expect("resolve");

    std::fs::create_dir_all(dir).unwrap();
    let rs = dir.join("gen.rs");
    std::fs::write(&rs, codegen::rust::generate(&program)).unwrap();
    std::fs::write(dir.join("roscmp_msgs.h"), codegen::c::generate(&program)).unwrap();
    std::fs::write(
        dir.join("roscmp_msgs.py"),
        codegen::python::generate(&program),
    )
    .unwrap();

    let lib = dir.join(format!("libroscmp_gen.{}", dylib_ext()));
    let out = Command::new("rustc")
        .args(["--edition", "2021", "-O", "--crate-type", "cdylib"])
        .arg(&rs)
        .arg("-o")
        .arg(&lib)
        .output()
        .expect("run rustc");
    assert!(
        out.status.success(),
        "generated Rust cdylib failed to compile:\n{}",
        String::from_utf8_lossy(&out.stderr)
    );
    lib
}

#[test]
fn c_and_python_serialize_through_rust_lib() {
    let dir = std::env::temp_dir().join("roscmp_ffi_point");
    let _ = std::fs::remove_dir_all(&dir);
    let lib = build_lib(&[POINT], &dir);

    // --- C side ---
    let c_main = r#"
#include <stdio.h>
#include "roscmp_msgs.h"
int main(void) {
    geometry_msgs__Point p = {1.0, 2.0, 3.0};
    RoscmpBuf b = roscmp_geometry_msgs__Point_serialize(&p, 0);
    for (size_t i = 0; i < b.len; i++) printf("%02x", b.ptr[i]);
    printf("\n");
    roscmp_buf_free(b);
    return 0;
}
"#;
    let c_src = dir.join("main.c");
    std::fs::write(&c_src, c_main).unwrap();
    let c_bin = dir.join("c_bin");
    let status = Command::new("cc")
        .arg(&c_src)
        .arg(&lib)
        .arg("-I")
        .arg(&dir)
        .arg("-o")
        .arg(&c_bin)
        .status()
        .expect("run cc");
    assert!(status.success(), "C program failed to compile");
    let c_out = Command::new(&c_bin).output().expect("run c bin");
    assert!(c_out.status.success());
    let c_hex = String::from_utf8(c_out.stdout).unwrap().trim().to_string();

    // --- Python side ---
    let py = format!(
        "import roscmp_msgs as m\n\
         m.load({:?})\n\
         p = m.geometry_msgs__Point(1.0, 2.0, 3.0)\n\
         print(m.geometry_msgs__Point_serialize(p).hex())\n",
        lib.to_str().unwrap()
    );
    let py_src = dir.join("check.py");
    std::fs::write(&py_src, py).unwrap();
    let py_out = Command::new("python3")
        .arg(&py_src)
        .env("PYTHONPATH", &dir)
        .output()
        .expect("run python3");
    assert!(
        py_out.status.success(),
        "python failed:\n{}",
        String::from_utf8_lossy(&py_out.stderr)
    );
    let py_hex = String::from_utf8(py_out.stdout).unwrap().trim().to_string();

    // Ground truth: Point {1,2,3} CDR LE.
    let expected = "00010000000000000000f03f00000000000000400000000000000840";
    assert_eq!(c_hex, expected, "C bytes wrong");
    assert_eq!(py_hex, expected, "Python bytes wrong");
    assert_eq!(c_hex, py_hex, "C and Python disagree");
}

#[test]
fn python_round_trips_through_rust_lib() {
    let dir = std::env::temp_dir().join("roscmp_ffi_rt");
    let _ = std::fs::remove_dir_all(&dir);
    let lib = build_lib(&[POINT], &dir);

    let py = format!(
        "import roscmp_msgs as m\n\
         m.load({:?})\n\
         p = m.geometry_msgs__Point(1.5, -2.5, 3.25)\n\
         data = m.geometry_msgs__Point_serialize(p)\n\
         q = m.geometry_msgs__Point_deserialize(data)\n\
         assert (q.x, q.y, q.z) == (1.5, -2.5, 3.25), (q.x, q.y, q.z)\n\
         print('ok')\n",
        lib.to_str().unwrap()
    );
    let py_src = dir.join("rt.py");
    std::fs::write(&py_src, py).unwrap();
    let out = Command::new("python3")
        .arg(&py_src)
        .env("PYTHONPATH", &dir)
        .output()
        .expect("run python3");
    assert!(
        out.status.success(),
        "python round-trip failed:\n{}",
        String::from_utf8_lossy(&out.stderr)
    );
    assert_eq!(String::from_utf8(out.stdout).unwrap().trim(), "ok");
}

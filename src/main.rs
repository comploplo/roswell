//! `roscmp` CLI: compile `.msg` files into Rust / C / Python bindings.
//!
//! Usage:
//!   roscmp [--lang rust|c|python|all] [--out DIR] FILE.msg [FILE.msg ...]
//!
//! Package/name are inferred from the path using the ROS layout
//! `<package>/msg/<Name>.msg`; if there is no `msg` directory, the parent
//! directory name is used as the package.

use std::path::{Path, PathBuf};
use std::process::ExitCode;

use roscmp::codegen;
use roscmp::ir::MsgId;
use roscmp::{
    action_messages, parse_action, parse_idl, parse_message, parse_service, resolve,
    service_messages,
};

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    match run(&args) {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("error: {e}");
            ExitCode::FAILURE
        }
    }
}

struct Options {
    lang: String,
    out: Option<PathBuf>,
    files: Vec<PathBuf>,
}

fn run(args: &[String]) -> Result<(), String> {
    let opts = parse_args(args)?;
    if opts.files.is_empty() {
        return Err("no input files (try `roscmp file.msg`)".into());
    }

    let mut inputs = Vec::new();
    for path in &opts.files {
        let src = std::fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))?;
        let id = infer_id(path);
        let err = |e: roscmp::ParseError| format!("{}: {e}", path.display());
        match path.extension().and_then(|s| s.to_str()) {
            Some("srv") => {
                let svc = parse_service(&src).map_err(err)?;
                inputs.extend(service_messages(&id.package, &id.name, &svc));
            }
            Some("action") => {
                let act = parse_action(&src).map_err(err)?;
                inputs.extend(action_messages(&id.package, &id.name, &act));
            }
            // `.idl` carries its own package/name in module/struct declarations.
            Some("idl") => inputs.extend(parse_idl(&src).map_err(err)?),
            // Default to `.msg` semantics for `.msg` or unknown extensions.
            _ => inputs.push((id, parse_message(&src).map_err(err)?)),
        }
    }

    let program = resolve(inputs).map_err(|e| e.to_string())?;

    let targets: Vec<&str> = match opts.lang.as_str() {
        "all" => vec!["rust", "c", "python"],
        l => vec![l],
    };

    for lang in targets {
        let (code, filename) = match lang {
            "rust" => (codegen::rust::generate(&program), "roscmp_msgs.rs"),
            "c" => (codegen::c::generate(&program), "roscmp_msgs.h"),
            "python" => (codegen::python::generate(&program), "roscmp_msgs.py"),
            other => return Err(format!("unknown language `{other}`")),
        };
        match &opts.out {
            Some(dir) => {
                std::fs::create_dir_all(dir).map_err(|e| e.to_string())?;
                let path = dir.join(filename);
                std::fs::write(&path, code).map_err(|e| e.to_string())?;
                eprintln!("wrote {}", path.display());
            }
            None => print!("{code}"),
        }
    }
    Ok(())
}

fn parse_args(args: &[String]) -> Result<Options, String> {
    let mut lang = "all".to_string();
    let mut out = None;
    let mut files = Vec::new();
    let mut i = 0;
    while i < args.len() {
        match args[i].as_str() {
            "--lang" => {
                i += 1;
                lang.clone_from(args.get(i).ok_or("--lang requires a value")?);
            }
            "--out" => {
                i += 1;
                out = Some(PathBuf::from(args.get(i).ok_or("--out requires a value")?));
            }
            other if other.starts_with("--") => return Err(format!("unknown flag `{other}`")),
            other => files.push(PathBuf::from(other)),
        }
        i += 1;
    }
    Ok(Options { lang, out, files })
}

/// Infer `(package, Name)` from a `.msg` path.
fn infer_id(path: &Path) -> MsgId {
    let name = path
        .file_stem()
        .and_then(|s| s.to_str())
        .unwrap_or("Unnamed")
        .to_string();

    let parent = path.parent();
    let parent_name = parent.and_then(|p| p.file_name()).and_then(|s| s.to_str());
    // In ROS layout, interfaces live under `<pkg>/{msg,srv,action}/`; the
    // package is the grandparent in that case.
    let package = match parent_name {
        Some("msg" | "srv" | "action") => parent
            .and_then(|p| p.parent())
            .and_then(|p| p.file_name())
            .and_then(|s| s.to_str()),
        other => other,
    }
    .unwrap_or("pkg")
    .to_string();

    MsgId { package, name }
}

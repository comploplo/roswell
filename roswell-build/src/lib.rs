//! `roswell-build`: generate roswell Rust message bindings from a `build.rs`,
//! in the `prost-build`/`tonic-build` mold.
//!
//! ```no_run
//! // build.rs (doctest wraps this in `fn main` for us)
//! roswell_build::Config::new()
//!     .type_paths(["msgs", "/opt/ros/jazzy"])
//!     .compile(["geometry_msgs/msg/Twist", "example_interfaces/srv/AddTwoInts"])
//!     .unwrap();
//! ```
//!
//! Then in the consuming crate (which must depend on `roswell-ros2-compat` — the
//! generated `CdrMsg`/action-trait impls reference it):
//!
//! ```ignore
//! #[allow(non_camel_case_types, non_upper_case_globals, dead_code, clippy::all, clippy::pedantic)]
//! mod msgs {
//!     include!(concat!(env!("OUT_DIR"), "/roswell_msgs.rs"));
//! }
//! ```

use std::collections::BTreeSet;
use std::path::{Path, PathBuf};

use roswell::ir::{MsgId, Program};
use roswell::workspace::{Kind, resolve_ref};

/// A build-time codegen error, carrying a human-readable message.
#[derive(Debug)]
pub struct Error(String);

impl std::fmt::Display for Error {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::error::Error for Error {}

/// Codegen configuration, built up fluently and consumed by [`Config::compile`].
#[derive(Debug, Default)]
pub struct Config {
    type_paths: Vec<PathBuf>,
    out_dir: Option<PathBuf>,
}

impl Config {
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Add interface search roots. Each root may be a plain package tree
    /// (`<root>/<pkg>/msg/<Name>.msg`) or an ament install prefix
    /// (`<root>/share/<pkg>/msg/<Name>.msg`, e.g. `/opt/ros/jazzy`).
    #[must_use]
    pub fn type_paths(mut self, paths: impl IntoIterator<Item = impl Into<PathBuf>>) -> Self {
        self.type_paths.extend(paths.into_iter().map(Into::into));
        self
    }

    /// Override the output directory (defaults to `$OUT_DIR`). Mainly for tests.
    #[must_use]
    pub fn out_dir(mut self, dir: impl Into<PathBuf>) -> Self {
        self.out_dir = Some(dir.into());
        self
    }

    /// Resolve `references` (`pkg/msg/Name`, `pkg/srv/Name`, `pkg/action/Name`)
    /// against the configured type paths and write `roswell_msgs.rs` — repr(C)
    /// structs, CDR (de)serializers, and `roswell_ros2_compat` `CdrMsg`/action-trait
    /// impls — into the output directory. Returns the written file's path.
    pub fn compile(
        self,
        references: impl IntoIterator<Item = impl AsRef<str>>,
    ) -> Result<PathBuf, Error> {
        let out_dir = match self.out_dir {
            Some(dir) => dir,
            None => std::env::var_os("OUT_DIR")
                .map(PathBuf::from)
                .ok_or_else(|| Error("OUT_DIR not set and no out_dir() configured".into()))?,
        };
        let mut program = Program {
            messages: Vec::new(),
        };
        let mut seen: BTreeSet<MsgId> = BTreeSet::new();
        for reference in references {
            let reference = reference.as_ref();
            let (_, resolved) = resolve_ref(reference, &self.type_paths, Kind::Msg)
                .map_err(|e| Error(format!("{reference}: {e}")))?;
            // Merge, keeping each message's first occurrence. Per-program order
            // is topological and duplicates already appeared earlier, so the
            // merged list stays dependency-ordered.
            for msg in resolved.messages {
                if seen.insert(msg.id.clone()) {
                    program.messages.push(msg);
                }
            }
        }
        let code = externalize(&roswell::codegen::rust::generate_dds_external(&program));
        std::fs::create_dir_all(&out_dir)
            .map_err(|e| Error(format!("creating {}: {e}", out_dir.display())))?;
        let path = out_dir.join("roswell_msgs.rs");
        std::fs::write(&path, code)
            .map_err(|e| Error(format!("writing {}: {e}", path.display())))?;
        // Re-run when any interface file under a search root changes (cargo
        // scans directories recursively).
        for root in &self.type_paths {
            println!("cargo:rerun-if-changed={}", root.display());
        }
        Ok(path)
    }
}

/// The compiler emits trait impls addressed to `crate::…` because its dds
/// output normally lives *inside* roswell-ros2-compat (`msgs.rs`). Retarget them at the
/// external `roswell_ros2_compat` crate, and route the one cross-crate value —
/// `Time::to_msg()` returns roswell-ros2-compat's `builtin_interfaces__Time`, not the
/// generated file's — through a field-by-field rebuild.
fn externalize(code: &str) -> String {
    // The FFI exports are already omitted upstream by `generate_dds_external`;
    // here we only apply deterministic token substitutions to retarget the
    // runtime-crate paths.
    code.lines()
        // Drop the file-level `#![allow(...)]`: an inner attribute is rejected
        // when the file is `include!`d, so the wrapping module carries the
        // allows instead (see the crate docs / README).
        .filter(|line| !line.starts_with("#!["))
        .map(|line| line.replace("crate::", "::roswell_ros2_compat::"))
        .fold(String::new(), |mut out, line| {
            out.push_str(&line);
            out.push('\n');
            out
        })
        .replace(
            "Self { accepted, stamp: stamp.to_msg() }",
            "Self { accepted, stamp: { let t = stamp.to_msg(); \
             builtin_interfaces__Time { sec: t.sec, nanosec: t.nanosec } } }",
        )
}

/// The repo's `samples/` tree, as a convenience search root for tests.
#[must_use]
pub fn samples_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("workspace root")
        .join("samples")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn generates_externalized_bindings() {
        let dir = std::env::temp_dir().join(format!("roswell_build_{}", std::process::id()));
        let path = Config::new()
            .type_paths([samples_root()])
            .out_dir(&dir)
            .compile([
                "geometry_msgs/msg/Twist",
                "example_interfaces/action/Fibonacci",
            ])
            .unwrap();
        let code = std::fs::read_to_string(&path).unwrap();
        assert!(
            code.contains("impl ::roswell_ros2_compat::codec::CdrMsg for geometry_msgs__Twist")
        );
        assert!(code.contains(
            "impl ::roswell_ros2_compat::action::SendGoalRequest for \
                 example_interfaces__Fibonacci_SendGoal_Request"
        ));
        assert!(!code.contains("crate::"), "unrewritten crate:: path left");
        std::fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn missing_reference_is_a_clean_error() {
        let dir = std::env::temp_dir().join(format!("roswell_build_err_{}", std::process::id()));
        let err = Config::new()
            .type_paths([samples_root()])
            .out_dir(&dir)
            .compile(["no_such/msg/Nope"])
            .unwrap_err();
        assert!(err.to_string().contains("could not find"));
    }
}

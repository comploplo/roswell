//! Freshness guard for the committed `src/msgs.rs` (13k generated lines).
//!
//! A from-scratch regen is not reproducible here (the ROS2 interface tree that
//! produced it is not tracked), but the part that actually drifts — the CDR
//! runtime embedded verbatim by codegen — is checkable: it must byte-match the
//! workspace's `src/cdr_runtime.rs` today. `scripts/regen-msgs.sh` re-embeds it.

#[test]
fn embedded_cdr_runtime_is_fresh() {
    let manifest = std::path::Path::new(env!("CARGO_MANIFEST_DIR"));
    let msgs = std::fs::read_to_string(manifest.join("src/msgs.rs")).expect("read msgs.rs");
    let runtime = std::fs::read_to_string(manifest.join("../src/cdr_runtime.rs"))
        .expect("read cdr_runtime.rs");
    let runtime = format!("{}\n", runtime.trim_end_matches('\n'));
    assert!(
        msgs.contains(&runtime),
        "roswell-ros2-compat/src/msgs.rs embeds a stale copy of src/cdr_runtime.rs; \
         run scripts/regen-msgs.sh"
    );
}

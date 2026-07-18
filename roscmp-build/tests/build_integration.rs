//! End-to-end proof of the build.rs flow: compile `tests/fixture` — an
//! out-of-workspace crate whose `build.rs` calls `roscmp_build::Config` — with
//! a real nested cargo invocation, run it, and check its CDR round-trips.

use std::path::Path;
use std::process::Command;

#[test]
fn fixture_crate_compiles_and_round_trips() {
    let manifest_dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let fixture = manifest_dir.join("tests/fixture");
    // A dedicated target dir: sharing the workspace's would deadlock on cargo's
    // build lock while this test itself is being run under `cargo test`.
    let target = manifest_dir
        .parent()
        .expect("workspace root")
        .join("target/roscmp-build-fixture");

    let output = Command::new(env!("CARGO"))
        .arg("run")
        .arg("--quiet")
        .current_dir(&fixture)
        .env("CARGO_TARGET_DIR", &target)
        .output()
        .expect("spawn cargo");
    assert!(
        output.status.success(),
        "fixture build/run failed:\n--- stdout ---\n{}\n--- stderr ---\n{}",
        String::from_utf8_lossy(&output.stdout),
        String::from_utf8_lossy(&output.stderr),
    );
    assert!(
        String::from_utf8_lossy(&output.stdout).contains("roundtrip ok"),
        "fixture did not report a successful round-trip"
    );
}

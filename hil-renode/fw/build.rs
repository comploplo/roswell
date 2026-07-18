use std::env;
use std::fs;
use std::path::PathBuf;

fn main() {
    // Put `memory.x` (consumed by cortex-m-rt's `link.x`) on the linker search
    // path — the standard cortex-m-rt build-script dance.
    let out = PathBuf::from(env::var("OUT_DIR").unwrap());
    fs::write(out.join("memory.x"), include_bytes!("memory.x")).unwrap();
    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rerun-if-changed=build.rs");
}

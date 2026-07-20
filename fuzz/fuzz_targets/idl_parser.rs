#![no_main]
//! Fuzz the IDL (`.idl`) frontend. Arbitrary bytes as lossy-UTF-8 source into
//! `parse_idl`; it must return `Ok`/`Err`, never panic.

use libfuzzer_sys::fuzz_target;

use roswell::parse_idl;

fuzz_target!(|data: &[u8]| {
    let src = String::from_utf8_lossy(data);
    let _ = parse_idl(&src);
});

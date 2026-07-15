#![no_main]
//! Fuzz the `.msg`/`.srv`/`.action` text frontends. The input bytes are treated
//! as (lossy) UTF-8 source and fed through every parse entry point. Parsers must
//! only ever return `Ok`/`Err` — never panic — on arbitrary input.

use libfuzzer_sys::fuzz_target;

use roscmp::{parse_action, parse_message, parse_service};

fuzz_target!(|data: &[u8]| {
    let src = String::from_utf8_lossy(data);
    let _ = parse_message(&src);
    let _ = parse_service(&src);
    let _ = parse_action(&src);
});

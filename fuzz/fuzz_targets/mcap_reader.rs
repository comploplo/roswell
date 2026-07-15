#![no_main]
//! Fuzz the MCAP bag readers (`roscmp_dds::raw`) — the file-facing record
//! parser plus the zstd/lz4 chunk-decompression paths.
//!
//! Both the streaming `RawSampleReader` (drained fully) and the eager
//! `McapLog::read` consume the fuzz bytes. Neither may panic, over-allocate, or
//! exhibit UB on arbitrary input; malformed data must surface as an `Err`.

use libfuzzer_sys::fuzz_target;

use roscmp_dds::raw::{McapLog, RawSampleReader};

fuzz_target!(|data: &[u8]| {
    if let Ok(reader) = RawSampleReader::from_bytes(data.to_vec()) {
        for sample in reader {
            if sample.is_err() {
                break;
            }
        }
    }
    let _ = McapLog::read(data);
});

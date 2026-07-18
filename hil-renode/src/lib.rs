//! Host-side helpers for the roscmp Renode HIL tests.
//!
//! The actual tests live in `tests/`; this crate exists so the firmware member
//! and the host harness share one standalone workspace, kept out of the root
//! roscmp `--workspace` gate (like `fuzz/`).

/// Cargo package name of the firmware crate built by the HIL tests.
pub const FIRMWARE_PACKAGE: &str = "roscmp-hil-fw";

/// Cross-compilation target triple for the firmware.
pub const FIRMWARE_TARGET: &str = "thumbv6m-none-eabi";

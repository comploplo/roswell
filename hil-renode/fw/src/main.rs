//! no_std Cortex-M firmware proving a no-DDS MCU can speak the roswell tunnel
//! wire protocol using the shared [`roswell_tunnel_core`] codec.
//!
//! Two behaviours, selected at build time:
//!
//! * default: run [`self_check`] (encode/parse roswell frames entirely in the
//!   MCU) and, on success, call [`hil_marker`] — hilt hooks that symbol to log
//!   `HIL OK`, so the host asserts the codec runs on simulated silicon.
//! * `--features uart`: additionally serve a PL011 UART — parse inbound tunnel
//!   frames and reply with roswell `Ack`/`Hello` frames, so a host bridge can
//!   round-trip the protocol over a wire.

#![no_std]
#![no_main]

use cortex_m_rt::entry;
use roswell_tunnel_core as wire;

#[cfg(feature = "uart")]
mod msgs_nostd;
#[cfg(feature = "uart")]
mod uart;

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {
        core::hint::spin_loop();
    }
}

/// Symbol hooked by hilt to emit `HIL OK`. Kept out-of-line and exported so the
/// linker preserves it and the host can resolve its address.
#[inline(never)]
#[no_mangle]
pub extern "C" fn hil_marker() {
    // The mere fact that PC reaches this address is the signal.
    unsafe { core::arch::asm!("nop", options(nomem, nostack)) }
}

/// Exercises the shared codec: encode roswell frames, then parse them back and
/// check every field. Returns `true` iff the round-trips are exact.
fn self_check() -> bool {
    let mut buf = [0u8; 256];

    // TopicSample: the frame a no-DDS MCU would emit for a real ROS topic.
    let Ok(n) = wire::encode_topic_sample(
        &mut buf,
        42,
        "/cmd_vel",
        "geometry_msgs/msg/Twist",
        -7,
        &[9, 8, 7],
    ) else {
        return false;
    };
    let Ok(header) = <&[u8; wire::HEADER_LEN]>::try_from(&buf[..wire::HEADER_LEN]) else {
        return false;
    };
    let Ok(len) = wire::frame_len(header) else {
        return false;
    };
    if len != n - wire::HEADER_LEN {
        return false;
    }
    match wire::parse_payload(&buf[wire::HEADER_LEN..n]) {
        Ok(wire::FrameRef::TopicSample {
            sequence,
            topic,
            ros_type,
            stamp_nanos,
            cdr,
        }) => {
            if sequence != 42
                || topic != "/cmd_vel"
                || ros_type != "geometry_msgs/msg/Twist"
                || stamp_nanos != -7
                || cdr != [9, 8, 7]
            {
                return false;
            }
        }
        _ => return false,
    }

    // Ack: the frame the MCU emits to acknowledge a reliable sample.
    let Ok(n) = wire::encode_ack(&mut buf, 1234) else {
        return false;
    };
    matches!(
        wire::parse_payload(&buf[wire::HEADER_LEN..n]),
        Ok(wire::FrameRef::Ack { sequence: 1234 })
    )
}

#[entry]
fn main() -> ! {
    // Bring the UART up first so it is ready to receive before the host bridge
    // starts sending — the emulated PL011 drops input while `UARTEN` is clear.
    #[cfg(feature = "uart")]
    uart::init();

    if self_check() {
        hil_marker();
    }

    #[cfg(feature = "uart")]
    uart::serve();

    #[cfg(not(feature = "uart"))]
    loop {
        core::hint::spin_loop();
    }
}

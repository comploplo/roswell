//! Minimal PL011 UART driver plus a roscmp tunnel serve loop.
//!
//! Drives an ARM PrimeCell PL011 (modelled by Renode as `UART.PL011`) at
//! [`UART_BASE`] with nothing but volatile MMIO — no HAL. The serve loop reads
//! tunnel frames off the wire with the shared [`roscmp_tunnel_core`] codec and
//! replies with roscmp frames, so a host bridge can round-trip the protocol.

use core::ptr::{read_volatile, write_volatile};

use roscmp_tunnel_core as wire;

use crate::msgs_nostd as msgs;

/// PL011 base address; matches the `pl011_m0.repl` platform.
const UART_BASE: usize = 0x4000_4000;
const DR: *mut u32 = UART_BASE as *mut u32; // data register
const FR: *const u32 = (UART_BASE + 0x18) as *const u32; // flag register
const LCRH: *mut u32 = (UART_BASE + 0x2C) as *mut u32; // line control
const CR: *mut u32 = (UART_BASE + 0x30) as *mut u32; // control register
const FR_RXFE: u32 = 1 << 4; // receive FIFO empty
const FR_TXFF: u32 = 1 << 5; // transmit FIFO full
const LCRH_FEN: u32 = 1 << 4; // enable FIFOs
const LCRH_WLEN8: u32 = 0b11 << 5; // 8-bit words
const CR_UARTEN: u32 = 1 << 0; // UART enable
const CR_TXE: u32 = 1 << 8; // transmit enable
const CR_RXE: u32 = 1 << 9; // receive enable

/// Brings the PL011 up: 8-bit words, FIFOs on, transmit + receive enabled.
///
/// Renode's `UART.PL011` drops inbound characters while `UARTEN` is clear
/// (`"Character cannot be received by UART; UARTEN is disabled!"`), so without
/// this the receive FIFO never fills and the serve loop blocks forever.
pub fn init() {
    unsafe {
        write_volatile(CR, 0); // disable while (re)configuring
        write_volatile(LCRH, LCRH_FEN | LCRH_WLEN8);
        write_volatile(CR, CR_UARTEN | CR_TXE | CR_RXE);
    }
}

fn getc() -> u8 {
    loop {
        unsafe {
            if read_volatile(FR) & FR_RXFE == 0 {
                return (read_volatile(DR) & 0xFF) as u8;
            }
        }
    }
}

fn putc(byte: u8) {
    unsafe {
        while read_volatile(FR) & FR_TXFF != 0 {}
        write_volatile(DR, u32::from(byte));
    }
}

fn put(bytes: &[u8]) {
    for &b in bytes {
        putc(b);
    }
}

/// Encodes a real `geometry_msgs/msg/Twist` with the `--no-std` generated CDR
/// codec ([`msgs`]) and frames it as a tunnel `TopicSample` into `out`.
/// Returns the framed length, or `None` if anything does not fit.
///
/// `stamp_nanos` echoes the requesting heartbeat's stamp, proving the value
/// travelled MCU-ward and back rather than being baked into the frame.
fn encode_twist_sample(out: &mut [u8], stamp_nanos: i64) -> Option<usize> {
    let twist = msgs::geometry_msgs__Twist {
        linear: msgs::geometry_msgs__Vector3 {
            x: 1.25,
            y: -2.5,
            z: 0.5,
        },
        angular: msgs::geometry_msgs__Vector3 {
            x: 0.0,
            y: 0.125,
            z: -3.75,
        },
    };
    let mut cdr = [0u8; 64];
    let n = twist.to_cdr(&mut cdr, msgs::Endian::Little).ok()?;
    wire::encode_topic_sample(
        out,
        1,
        "/mcu_twist",
        "geometry_msgs/msg/Twist",
        stamp_nanos,
        &cdr[..n],
    )
    .ok()
}

/// Reads tunnel frames from the UART forever, replying per frame kind:
/// `Hello` → `Hello`, `TopicSample` → `Ack{sequence}`, `Heartbeat` → a
/// `TopicSample` carrying a genuine no_std-CDR-encoded `Twist`. Resynchronises
/// to the frame magic, so it tolerates mid-stream connection.
pub fn serve() -> ! {
    let mut window = [0u8; 8];
    let mut payload = [0u8; 512];
    let mut out = [0u8; 512];

    // Prime the magic window.
    for slot in &mut window {
        *slot = getc();
    }

    loop {
        // Resync: slide the window until it holds the frame magic.
        while window != wire::MAGIC {
            window.copy_within(1.., 0);
            window[7] = getc();
        }

        let mut header = [0u8; wire::HEADER_LEN];
        header[..8].copy_from_slice(&wire::MAGIC);
        for slot in &mut header[8..] {
            *slot = getc();
        }

        let Ok(len) = wire::frame_len(&header) else {
            // Bad header; drop the window and hunt for the next magic.
            for slot in &mut window {
                *slot = getc();
            }
            continue;
        };

        if len > payload.len() {
            for _ in 0..len {
                let _ = getc();
            }
        } else {
            for slot in &mut payload[..len] {
                *slot = getc();
            }
            match wire::parse_payload(&payload[..len]) {
                Ok(wire::FrameRef::Hello { .. }) => {
                    if let Ok(n) = wire::encode_hello(&mut out, "roscmp-mcu") {
                        put(&out[..n]);
                    }
                }
                Ok(wire::FrameRef::TopicSample { sequence, .. }) => {
                    if let Ok(n) = wire::encode_ack(&mut out, sequence) {
                        put(&out[..n]);
                    }
                }
                Ok(wire::FrameRef::Heartbeat { stamp_nanos }) => {
                    if let Some(n) = encode_twist_sample(&mut out, stamp_nanos) {
                        put(&out[..n]);
                    }
                }
                _ => {}
            }
        }

        // Refill the window for the next frame.
        for slot in &mut window {
            *slot = getc();
        }
    }
}

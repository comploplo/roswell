//! End-to-end HIL tests: a no_std Cortex-M firmware speaking the roscmp tunnel
//! protocol, run inside Renode via `hilt`.
//!
//! The tests are `#[ignore]`d because they need a Renode image and each takes
//! ~2 min (the container simulates the full `RunFor` window). `hilt` selects the
//! image by host architecture: the amd64 `antmicro/renode` on x86-64, or a
//! native `linux-arm64` image it builds on demand on Apple-Silicon (where the
//! amd64 image under `qemu-user` never finishes `LoadPlatformDescription`). Run:
//!
//! ```text
//! cargo test -p roscmp-hil -- --ignored
//! ```

use std::io::{Read, Write};
use std::net::TcpStream;
use std::path::PathBuf;
use std::process::Command;
use std::sync::Mutex;
use std::time::{Duration, Instant};

use hilt::{HilConfig, MachineSpec, Platform, ReplSource};
use roscmp_tunnel_core as wire;

const UART_PORT: u16 = 13456;
/// Separate port for the Twist-encoding test so the two UART tests never share
/// a socket even across orphaned containers.
const TWIST_UART_PORT: u16 = 13460;

/// Serializes the two UART-bridge tests: starting two Renode containers with
/// port publishes simultaneously races in the gvproxy/socket-terminal startup
/// path (each test is green solo, flaky in parallel). Distinct ports are not
/// enough; run them one at a time until hilt hardens parallel startup.
static UART_TEST_LOCK: Mutex<()> = Mutex::new(());

/// Builds the firmware for `thumbv6m` and returns the ELF path.
fn build_firmware(features: &[&str]) -> PathBuf {
    let manifest = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let mut cmd = Command::new(env!("CARGO"));
    cmd.current_dir(&manifest).args([
        "build",
        "-p",
        "roscmp-hil-fw",
        "--target",
        "thumbv6m-none-eabi",
        "--release",
    ]);
    if !features.is_empty() {
        cmd.args(["--features", &features.join(",")]);
    }

    // Stage A (no features) and Stage B (`uart`) compile the *same* `[[bin]]`,
    // so cargo writes both to `…/release/tunnel_selfcheck`. Run under a lock and
    // copy the artifact to a feature-specific name, so the two tests — which run
    // on parallel threads — never load each other's (differently-featured) ELF.
    static BUILD_LOCK: Mutex<()> = Mutex::new(());
    let _lock = BUILD_LOCK.lock().unwrap_or_else(|e| e.into_inner());

    let status = cmd.status().expect("failed to invoke cargo for firmware");
    assert!(status.success(), "firmware build failed");
    let built = manifest.join("target/thumbv6m-none-eabi/release/tunnel_selfcheck");
    let variant = if features.is_empty() {
        "plain".to_string()
    } else {
        features.join("-")
    };
    let dst = built.with_file_name(format!("tunnel_selfcheck-{variant}"));
    std::fs::copy(&built, &dst).expect("failed to stage firmware ELF variant");
    dst
}

/// Stage A: the codec runs on simulated silicon. The firmware self-checks the
/// roscmp frame codec and, on success, hits `hil_marker` → Renode logs `HIL OK`.
#[test]
#[ignore = "requires a Renode image (auto-built on arm64); slow (~2 min) — run with --ignored, see README"]
fn tunnel_codec_runs_on_cortex_m() {
    let elf = build_firmware(&[]);
    let output =
        hilt::RenodeRunner::new(HilConfig::single(Platform::rp2040(), elf).timeout(20)).run();
    assert!(output.passed(), "expected HIL OK marker.\n{output}");
}

/// Stage B: an MCU speaks the roscmp tunnel over UART. The host sends a
/// `TopicSample` frame down the bridged UART; the firmware parses it and replies
/// with an `Ack`, proving a no-DDS MCU round-trips the protocol over a wire.
#[test]
#[ignore = "requires a Renode image (auto-built on arm64); slow (~2 min) — run with --ignored, see README"]
fn mcu_acks_topic_sample_over_uart() {
    let _uart = UART_TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let elf = build_firmware(&["uart"]);

    // A Cortex-M0 with a PL011 UART, exposed to the host on UART_PORT.
    let platform = Platform::custom(
        "pl011m0",
        ReplSource::Embedded(include_str!("../fw/pl011_m0.repl")),
        hilt::CpuInit::VectorTable,
        None,
        None,
        30,
    );
    let machine = MachineSpec::new("hil", elf, platform).with_uart_bridge(UART_PORT, "sysbus.uart");
    let runner = hilt::RenodeRunner::new(HilConfig::multi(vec![machine]).timeout(30));

    let sim = std::thread::spawn(move || runner.run());

    // Send a reliable TopicSample; expect an Ack for the same sequence.
    let mut buf = [0u8; 256];
    let n = wire::encode_topic_sample(
        &mut buf,
        777,
        "/cmd_vel",
        "geometry_msgs/msg/Twist",
        123,
        &[1, 2, 3, 4],
    )
    .unwrap();

    let frame = request_reply(UART_PORT, &buf[..n], Duration::from_secs(90))
        .expect("no reply frame from firmware within budget");
    match wire::parse_payload(&frame).unwrap() {
        wire::FrameRef::Ack { sequence } => assert_eq!(sequence, 777),
        other => panic!("expected Ack, got {other:?}"),
    }

    let output = sim.join().unwrap();
    assert!(output.passed(), "expected HIL OK marker.\n{output}");
}

/// Stage C: the MCU produces genuine ROS2 CDR. The host sends a `Heartbeat`;
/// the firmware encodes a real `geometry_msgs/msg/Twist` with the `--no-std`
/// generated codec (`fw/src/msgs_nostd.rs`) and ships it as a tunnel
/// `TopicSample`; this test decodes the CDR bytes with the **std** generated
/// decoder (`roscmp_dds::msgs`) and asserts every field value.
#[test]
#[ignore = "requires a Renode image (auto-built on arm64); slow (~2 min) — run with --ignored, see README"]
fn mcu_encodes_real_twist_cdr_over_uart() {
    let _uart = UART_TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let elf = build_firmware(&["uart"]);

    let platform = Platform::custom(
        "pl011m0",
        ReplSource::Embedded(include_str!("../fw/pl011_m0.repl")),
        hilt::CpuInit::VectorTable,
        None,
        None,
        30,
    );
    let machine =
        MachineSpec::new("hil", elf, platform).with_uart_bridge(TWIST_UART_PORT, "sysbus.uart");
    let runner = hilt::RenodeRunner::new(HilConfig::multi(vec![machine]).timeout(30));

    let sim = std::thread::spawn(move || runner.run());

    // A Heartbeat draws the firmware's Twist TopicSample; its stamp must be
    // echoed back, proving the reply is built from the parsed request.
    let mut buf = [0u8; 64];
    let n = wire::encode_heartbeat(&mut buf, 424_242).unwrap();

    let frame = request_reply(TWIST_UART_PORT, &buf[..n], Duration::from_secs(90))
        .expect("no reply frame from firmware within budget");
    let (ros_type, stamp_nanos, cdr) = match wire::parse_payload(&frame).unwrap() {
        wire::FrameRef::TopicSample {
            topic,
            ros_type,
            stamp_nanos,
            cdr,
            ..
        } => {
            assert_eq!(topic, "/mcu_twist");
            (ros_type.to_string(), stamp_nanos, cdr.to_vec())
        }
        other => panic!("expected TopicSample, got {other:?}"),
    };
    assert_eq!(ros_type, "geometry_msgs/msg/Twist");
    assert_eq!(
        stamp_nanos, 424_242,
        "firmware must echo the heartbeat stamp"
    );

    // Decode the MCU-encoded CDR with the std generated decoder.
    let twist = roscmp_dds::msgs::geometry_msgs__Twist::from_cdr(&cdr)
        .expect("MCU CDR must decode with the std decoder");
    assert_eq!(twist.linear.x, 1.25);
    assert_eq!(twist.linear.y, -2.5);
    assert_eq!(twist.linear.z, 0.5);
    assert_eq!(twist.angular.x, 0.0);
    assert_eq!(twist.angular.y, 0.125);
    assert_eq!(twist.angular.z, -3.75);
    println!(
        "STAGE C OK: MCU-encoded Twist ({} CDR bytes) decoded by the std decoder: \
         linear=({}, {}, {}) angular=({}, {}, {})",
        cdr.len(),
        twist.linear.x,
        twist.linear.y,
        twist.linear.z,
        twist.angular.x,
        twist.angular.y,
        twist.angular.z,
    );

    let output = sim.join().unwrap();
    assert!(output.passed(), "expected HIL OK marker.\n{output}");
}

/// Sends `frame` over the bridged UART and returns the firmware's reply payload.
///
/// Renode's port is published by gvproxy before the in-container socket terminal
/// is listening, so an early connect can succeed yet immediately EOF; and the
/// firmware may not have booted (enabled its UART) when the first bytes arrive.
/// Both are startup races, so this reconnects and retransmits — the firmware's
/// serve loop resyncs on the frame magic — until a full reply frame arrives or
/// the budget is exhausted.
fn request_reply(port: u16, frame: &[u8], budget: Duration) -> Option<Vec<u8>> {
    let deadline = Instant::now() + budget;
    while Instant::now() < deadline {
        let Ok(mut stream) = TcpStream::connect(("127.0.0.1", port)) else {
            std::thread::sleep(Duration::from_millis(400));
            continue;
        };
        stream
            .set_read_timeout(Some(Duration::from_secs(2)))
            .unwrap();
        if stream
            .write_all(frame)
            .and_then(|()| stream.flush())
            .is_err()
        {
            std::thread::sleep(Duration::from_millis(400));
            continue;
        }
        match read_frame(&mut stream) {
            Ok(payload) => return Some(payload),
            Err(_) => std::thread::sleep(Duration::from_millis(400)),
        }
    }
    None
}

/// Reads one length-prefixed tunnel frame (magic + u32 len + payload), returning
/// the payload bytes.
fn read_frame(stream: &mut TcpStream) -> std::io::Result<Vec<u8>> {
    let mut header = [0u8; wire::HEADER_LEN];
    stream.read_exact(&mut header)?;
    let len = wire::frame_len(&header)
        .map_err(|_| std::io::Error::new(std::io::ErrorKind::InvalidData, "bad frame header"))?;
    let mut payload = vec![0u8; len];
    stream.read_exact(&mut payload)?;
    Ok(payload)
}

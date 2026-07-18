//! The killer demo, as a HIL test: the embedded **Python** node (the shipped
//! `roscmp` FFI wheel — pure ctypes over the Rust runtime) publishes a ROS topic
//! over real RTPS/DDS, and the embedded **Rust** firmware (no_std, on simulated
//! Cortex-M silicon in Renode) receives it over the tunnel/UART and acks it.
//!
//! Chain: `wheel_node.py` --DDS--> this harness (roscmp-dds subscriber, the same
//! runtime the `tcp_topic_bridge`/`usb_topic_bridge` bins use) --tunnel/UART-->
//! Renode firmware --Ack--> harness. No new Python protocol code; no parallel
//! codec. The harness plays the DDS<->UART bridge in-process specifically so it
//! can *assert* the firmware's ack of the wheel-published message (the shipped
//! bridge bins consume acks internally).
//!
//! `#[ignore]`d (needs the Renode image + the built wheel in `python/.venv-test`;
//! ~1 min). Run:
//!   cargo test -p roscmp-hil --test wheel_firmware_demo -- --ignored --nocapture

use std::io::{Read, Write};
use std::net::TcpStream;
use std::path::PathBuf;
use std::process::{Child, Command};
use std::sync::Mutex;
use std::time::{Duration, Instant};

use hilt::{HilConfig, MachineSpec, Platform, ReplSource};
use roscmp_dds::raw::{raw_qos_for_topic, RawDdsSubscriber};
use roscmp_dds::transport::Dds;
use roscmp_tunnel_core as wire;

const UART_PORT: u16 = 13458;

fn repo_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("..")
}

fn build_uart_firmware() -> PathBuf {
    let manifest = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    static BUILD_LOCK: Mutex<()> = Mutex::new(());
    let _lock = BUILD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
    let status = Command::new(env!("CARGO"))
        .current_dir(&manifest)
        .args([
            "build",
            "-p",
            "roscmp-hil-fw",
            "--target",
            "thumbv6m-none-eabi",
            "--release",
            "--features",
            "uart",
        ])
        .status()
        .expect("failed to invoke cargo for firmware");
    assert!(status.success(), "firmware build failed");
    let built = manifest.join("target/thumbv6m-none-eabi/release/tunnel_selfcheck");
    let dst = built.with_file_name("tunnel_selfcheck-wheel-demo");
    std::fs::copy(&built, &dst).expect("failed to stage firmware ELF");
    dst
}

/// Launches the shipped wheel node (from `python/.venv-test`) publishing
/// `/cmd_vel` for the demo window.
fn spawn_wheel_node() -> Child {
    let root = repo_root();
    let python = root.join("python/.venv-test/bin/python");
    assert!(
        python.exists(),
        "wheel venv missing at {}; build the wheel and `pip install` it into python/.venv-test",
        python.display()
    );
    Command::new(python)
        .arg(root.join("embedded/demo/wheel_node.py"))
        .arg("90") // seconds
        .arg("10") // hz
        .spawn()
        .expect("failed to launch wheel node")
}

/// Reads one length-prefixed tunnel frame, returning the payload.
fn read_frame(stream: &mut TcpStream) -> std::io::Result<Vec<u8>> {
    let mut header = [0u8; wire::HEADER_LEN];
    stream.read_exact(&mut header)?;
    let len = wire::frame_len(&header)
        .map_err(|_| std::io::Error::new(std::io::ErrorKind::InvalidData, "bad frame header"))?;
    let mut payload = vec![0u8; len];
    stream.read_exact(&mut payload)?;
    Ok(payload)
}

/// One connect+handshake+forward attempt. Returns `(firmware_peer, acked_seq)`.
fn bridge_once(
    port: u16,
    sub: &mut RawDdsSubscriber,
    seq: u64,
    deadline: Instant,
) -> Option<(String, u64)> {
    let mut stream = TcpStream::connect(("127.0.0.1", port)).ok()?;
    stream.set_read_timeout(Some(Duration::from_secs(3))).ok()?;

    // Direction: firmware -> host. Our Hello draws the firmware's Hello reply.
    let mut buf = [0u8; 512];
    let n = wire::encode_hello(&mut buf, "roscmp-demo-bridge").ok()?;
    stream.write_all(&buf[..n]).ok()?;
    stream.flush().ok()?;
    let hello = read_frame(&mut stream).ok()?;
    let peer = match wire::parse_payload(&hello).ok()? {
        wire::FrameRef::Hello { peer } => peer.to_string(),
        _ => return None,
    };

    // Wait for a real DDS sample from the wheel node, then forward it as a
    // tunnel TopicSample and read the firmware's ack.
    let msg = loop {
        if let Some(m) = sub.take() {
            break m;
        }
        if Instant::now() >= deadline {
            return None;
        }
        std::thread::sleep(Duration::from_millis(50));
    };
    let n = wire::encode_topic_sample(&mut buf, seq, "/cmd_vel", msg.ros_type(), 1234, msg.cdr())
        .ok()?;
    stream.write_all(&buf[..n]).ok()?;
    stream.flush().ok()?;

    match wire::parse_payload(&read_frame(&mut stream).ok()?).ok()? {
        wire::FrameRef::Ack { sequence } if sequence == seq => Some((peer, sequence)),
        _ => None,
    }
}

#[test]
#[ignore = "requires the Renode image (auto-built on arm64) + the roscmp wheel in python/.venv-test; slow (~1 min)"]
fn wheel_node_message_reaches_cortex_m_firmware_over_dds() {
    let elf = build_uart_firmware();

    // Embedded Rust node: Cortex-M0 + PL011 UART exposed on UART_PORT.
    let platform = Platform::custom(
        "pl011m0",
        ReplSource::Embedded(include_str!("../fw/pl011_m0.repl")),
        hilt::CpuInit::VectorTable,
        None,
        None,
        90,
    );
    let machine = MachineSpec::new("hil", elf, platform).with_uart_bridge(UART_PORT, "sysbus.uart");
    let runner = hilt::RenodeRunner::new(HilConfig::multi(vec![machine]).timeout(90));
    let sim = std::thread::spawn(move || runner.run());

    // Embedded Python node: the shipped wheel, publishing /cmd_vel over real DDS.
    let mut wheel = spawn_wheel_node();

    // Bridge role via the roscmp runtime: subscribe /cmd_vel, forward to the MCU.
    let dds = Dds::new(0);
    let mut sub = RawDdsSubscriber::new(
        dds.participant(),
        "/cmd_vel",
        "geometry_msgs/msg/Twist",
        raw_qos_for_topic("/cmd_vel"),
    );

    let deadline = Instant::now() + Duration::from_secs(85);
    let mut result = None;
    let mut attempt = 0;
    while Instant::now() < deadline {
        attempt += 1;
        if let Some((peer, seq)) = bridge_once(UART_PORT, &mut sub, 1, deadline) {
            println!(
                "attempt {attempt}: firmware peer={peer:?} acked /cmd_vel seq={seq} \
                 (wheel-published Twist, delivered over real DDS)"
            );
            result = Some((peer, seq));
            break;
        }
        std::thread::sleep(Duration::from_millis(400));
    }

    let _ = wheel.kill();
    let _ = wheel.wait();

    let (peer, seq) = result.expect("wheel message never reached the firmware within budget");
    assert_eq!(peer, "roscmp-mcu", "unexpected firmware peer name");
    assert_eq!(seq, 1);
    println!(
        "DEMO OK: embedded Python wheel node -> real DDS -> tunnel/UART -> \
         embedded Rust firmware (Cortex-M) acked the message. Zero ROS installed."
    );

    let sim_out = sim.join().unwrap();
    assert!(sim_out.passed(), "expected HIL OK marker.\n{sim_out}");
}

//! USB CDC / serial carrier for the graph-aware topic tunnel.
//!
//! Examples:
//!   usb_topic_bridge /dev/ttyACM0 --rx /cmd_vel:geometry_msgs/msg/Twist --tx /diagnostics:diagnostic_msgs/msg/DiagnosticArray
//!   usb_topic_bridge /dev/tty.usbmodem1101 --baud 921600 --tx /cmd_vel:geometry_msgs/msg/Twist --rx /diagnostics:diagnostic_msgs/msg/DiagnosticArray

use std::io;
use std::thread;
use std::time::Duration;

use roscmp_dds::raw::{raw_qos_for_topic, RawDdsPublisher, RawDdsSubscriber};
use roscmp_dds::transport::Dds;
use roscmp_dds::tunnel::{
    run_topic_rx_loop, run_topic_tx_loop, ResyncFramedIo, RoutedTopicSink, RoutedTopicSource,
    TopicBridgeRxConfig, TopicBridgeTxConfig, TopicRoute, TunnelReliabilityHandle,
};
use serialport::SerialPort;

#[path = "shared/argcursor.rs"]
mod argcursor;
use argcursor::ArgCursor;

const USAGE: &str = "usage: usb_topic_bridge PORT [--baud N] [--domain N] [--reconnect-ms N] [--tx /topic:pkg/msg/Type] [--rx /topic:pkg/msg/Type]";

fn main() -> io::Result<()> {
    let args = Args::parse()?;
    loop {
        match run_link(&args) {
            Ok(()) => return Ok(()),
            Err(err) => {
                eprintln!(
                    "usb_topic_bridge: link failed: {err}; reconnecting in {:?}",
                    args.reconnect_delay
                );
                thread::sleep(args.reconnect_delay);
            }
        }
    }
}

fn run_link(args: &Args) -> io::Result<()> {
    println!(
        "usb_topic_bridge: opening {} at {} baud",
        args.path, args.baud
    );
    let port = serialport::new(&args.path, args.baud)
        .timeout(Duration::from_millis(100))
        .open()
        .map_err(|err| io::Error::new(io::ErrorKind::NotConnected, err))?;
    let reader_port = port
        .try_clone()
        .map_err(|err| io::Error::new(io::ErrorKind::NotConnected, err))?;

    let reliability = TunnelReliabilityHandle::new();
    let reader_reliability = reliability.clone();
    let reader_routes = args.rx.clone();
    let reader_domain = args.domain;
    let reader = thread::spawn(move || {
        read_loop(
            reader_port,
            Dds::new(reader_domain),
            reader_routes,
            reader_reliability,
        )
    });

    let writer_routes = args.tx.clone();
    let writer_domain = args.domain;
    let writer = thread::spawn(move || {
        write_loop(port, Dds::new(writer_domain), writer_routes, reliability)
    });

    let reader_result = reader
        .join()
        .unwrap_or_else(|_| Err(io::Error::other("bridge reader thread panicked")));
    let writer_result = writer
        .join()
        .unwrap_or_else(|_| Err(io::Error::other("bridge writer thread panicked")));
    reader_result.and(writer_result)
}

fn write_loop(
    port: Box<dyn SerialPort>,
    dds: Dds,
    routes: Vec<TopicRoute>,
    reliability: TunnelReliabilityHandle,
) -> io::Result<()> {
    let mut carrier = ResyncFramedIo::new(port);
    let subscribers: Vec<_> = routes
        .into_iter()
        .map(|route| {
            let subscriber = RawDdsSubscriber::new(
                dds.participant(),
                &route.topic,
                &route.ros_type,
                raw_qos_for_topic(&route.topic),
            );
            RoutedTopicSource::new(route.topic, subscriber)
        })
        .collect();
    drop(dds);

    run_topic_tx_loop(
        &mut carrier,
        subscribers,
        TopicBridgeTxConfig {
            peer: "roscmp-dds usb_topic_bridge".into(),
            reliability: Some(reliability),
            ..TopicBridgeTxConfig::default()
        },
    )
}

fn read_loop(
    port: Box<dyn SerialPort>,
    dds: Dds,
    routes: Vec<TopicRoute>,
    reliability: TunnelReliabilityHandle,
) -> io::Result<()> {
    let mut carrier = ResyncFramedIo::new(port);
    let publishers: Vec<_> = routes
        .into_iter()
        .map(|route| {
            let publisher = RawDdsPublisher::new(
                dds.participant(),
                &route.topic,
                &route.ros_type,
                raw_qos_for_topic(&route.topic),
            );
            RoutedTopicSink::new(route.topic, publisher)
        })
        .collect();
    drop(dds);

    run_topic_rx_loop(
        &mut carrier,
        publishers,
        |peer| {
            println!("usb_topic_bridge: peer hello: {peer}");
        },
        TopicBridgeRxConfig {
            reliability: Some(reliability),
            ..TopicBridgeRxConfig::default()
        },
    )
}

#[derive(Clone)]
struct Args {
    path: String,
    baud: u32,
    domain: u16,
    reconnect_delay: Duration,
    tx: Vec<TopicRoute>,
    rx: Vec<TopicRoute>,
}

impl Args {
    fn parse() -> io::Result<Self> {
        let mut c = ArgCursor::new(USAGE);
        let mut args = Self {
            path: c.value()?,
            baud: 115_200,
            domain: 0,
            reconnect_delay: Duration::from_secs(1),
            tx: Vec::new(),
            rx: Vec::new(),
        };
        while let Some(arg) = c.next_arg() {
            match arg.as_str() {
                "--baud" => args.baud = c.parse()?,
                "--domain" => args.domain = c.parse()?,
                "--reconnect-ms" => args.reconnect_delay = Duration::from_millis(c.parse()?),
                "--tx" => args.tx.push(TopicRoute::parse(&c.value()?)?),
                "--rx" => args.rx.push(TopicRoute::parse(&c.value()?)?),
                _ => return Err(c.usage()),
            }
        }
        if args.tx.is_empty() && args.rx.is_empty() {
            return Err(c.usage());
        }
        Ok(args)
    }
}

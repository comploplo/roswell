//! TCP carrier for the graph-aware topic tunnel.
//!
//! Examples:
//!   tcp_topic_bridge serve 0.0.0.0:7447 --rx /cmd_vel:geometry_msgs/msg/Twist --tx /diagnostics:diagnostic_msgs/msg/DiagnosticArray
//!   tcp_topic_bridge connect robot.local:7447 --tx /cmd_vel:geometry_msgs/msg/Twist --rx /diagnostics:diagnostic_msgs/msg/DiagnosticArray

use std::io;
use std::net::{TcpListener, TcpStream};
use std::thread;

use roswell_ros2_compat::raw::{raw_qos_for_topic, RawDdsPublisher, RawDdsSubscriber};
use roswell_ros2_compat::transport::Dds;
use roswell_ros2_compat::tunnel::{
    run_topic_rx_loop, run_topic_tx_loop, FramedIo, RoutedTopicSink, RoutedTopicSource,
    TopicBridgeRxConfig, TopicBridgeTxConfig, TopicRoute, TunnelReliabilityHandle,
};

#[path = "shared/argcursor.rs"]
mod argcursor;
use argcursor::ArgCursor;

const USAGE: &str = "usage: tcp_topic_bridge <serve|connect> ADDR [--domain N] [--tx /topic:pkg/msg/Type] [--rx /topic:pkg/msg/Type]";

fn main() -> io::Result<()> {
    let args = Args::parse()?;
    let stream = match args.mode {
        Mode::Serve => {
            let listener = TcpListener::bind(&args.addr)?;
            println!("tcp_topic_bridge: listening on {}", args.addr);
            let (stream, peer) = listener.accept()?;
            println!("tcp_topic_bridge: accepted {peer}");
            stream
        }
        Mode::Connect => {
            println!("tcp_topic_bridge: connecting to {}", args.addr);
            TcpStream::connect(&args.addr)?
        }
    };
    stream.set_nodelay(true)?;

    let dds = Dds::new(args.domain);
    let reliability = TunnelReliabilityHandle::new();
    let reader_stream = stream.try_clone()?;
    let rx_routes = args.rx.clone();
    let reader_dds = Dds::new(args.domain);
    let reader_reliability = reliability.clone();
    let reader =
        thread::spawn(move || read_loop(reader_stream, reader_dds, rx_routes, reader_reliability));

    let writer = thread::spawn(move || write_loop(stream, dds, args.tx, reliability));

    let reader_result = reader
        .join()
        .unwrap_or_else(|_| Err(io::Error::other("bridge reader thread panicked")));
    let writer_result = writer
        .join()
        .unwrap_or_else(|_| Err(io::Error::other("bridge writer thread panicked")));
    reader_result.and(writer_result)
}

fn write_loop(
    stream: TcpStream,
    dds: Dds,
    routes: Vec<TopicRoute>,
    reliability: TunnelReliabilityHandle,
) -> io::Result<()> {
    let mut carrier = FramedIo::new(stream);
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
            peer: "roswell-ros2-compat tcp_topic_bridge".into(),
            reliability: Some(reliability),
            ..TopicBridgeTxConfig::default()
        },
    )
}

fn read_loop(
    stream: TcpStream,
    dds: Dds,
    routes: Vec<TopicRoute>,
    reliability: TunnelReliabilityHandle,
) -> io::Result<()> {
    let mut carrier = FramedIo::new(stream);
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
            println!("tcp_topic_bridge: peer hello: {peer}");
        },
        TopicBridgeRxConfig {
            reliability: Some(reliability),
            ..TopicBridgeRxConfig::default()
        },
    )
}

#[derive(Clone, Copy)]
enum Mode {
    Serve,
    Connect,
}

struct Args {
    mode: Mode,
    addr: String,
    domain: u16,
    tx: Vec<TopicRoute>,
    rx: Vec<TopicRoute>,
}

impl Args {
    fn parse() -> io::Result<Self> {
        let mut c = ArgCursor::new(USAGE);
        let mode = match c.next_arg().as_deref() {
            Some("serve") => Mode::Serve,
            Some("connect") => Mode::Connect,
            _ => return Err(c.usage()),
        };
        let mut args = Self {
            mode,
            addr: c.value()?,
            domain: 0,
            tx: Vec::new(),
            rx: Vec::new(),
        };
        while let Some(arg) = c.next_arg() {
            match arg.as_str() {
                "--domain" => args.domain = c.parse()?,
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

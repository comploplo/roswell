//! TCP carrier for the graph-aware topic tunnel.
//!
//! Examples:
//!   tcp_topic_bridge serve 0.0.0.0:7447 --rx /cmd_vel:geometry_msgs/msg/Twist --tx /diagnostics:diagnostic_msgs/msg/DiagnosticArray
//!   tcp_topic_bridge connect robot.local:7447 --tx /cmd_vel:geometry_msgs/msg/Twist --rx /diagnostics:diagnostic_msgs/msg/DiagnosticArray

use std::io;
use std::net::{TcpListener, TcpStream};
use std::thread;

use roscmp_dds::raw::{raw_qos_for_topic, RawDdsPublisher, RawDdsSubscriber};
use roscmp_dds::transport::Dds;
use roscmp_dds::tunnel::{
    run_topic_rx_loop, run_topic_tx_loop, FramedIo, RoutedTopicSink, RoutedTopicSource,
    TopicBridgeRxConfig, TopicBridgeTxConfig, TopicRoute, TunnelReliabilityHandle,
};

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
            peer: "roscmp-dds tcp_topic_bridge".into(),
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
        let mut iter = std::env::args().skip(1);
        let mode = match iter.next().as_deref() {
            Some("serve") => Mode::Serve,
            Some("connect") => Mode::Connect,
            _ => return Err(usage()),
        };
        let Some(addr) = iter.next() else {
            return Err(usage());
        };
        let mut args = Self {
            mode,
            addr,
            domain: 0,
            tx: Vec::new(),
            rx: Vec::new(),
        };
        while let Some(arg) = iter.next() {
            match arg.as_str() {
                "--domain" => {
                    args.domain = iter
                        .next()
                        .ok_or_else(usage)?
                        .parse()
                        .map_err(|_| usage())?;
                }
                "--tx" => args
                    .tx
                    .push(TopicRoute::parse(&iter.next().ok_or_else(usage)?)?),
                "--rx" => args
                    .rx
                    .push(TopicRoute::parse(&iter.next().ok_or_else(usage)?)?),
                _ => return Err(usage()),
            }
        }
        if args.tx.is_empty() && args.rx.is_empty() {
            return Err(usage());
        }
        Ok(args)
    }
}

fn usage() -> io::Error {
    io::Error::new(
        io::ErrorKind::InvalidInput,
        "usage: tcp_topic_bridge <serve|connect> ADDR [--domain N] [--tx /topic:pkg/msg/Type] [--rx /topic:pkg/msg/Type]",
    )
}

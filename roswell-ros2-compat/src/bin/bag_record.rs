//! Record ROS2 CDR samples to an MCAP file type-blind.
//!
//! Usage:
//!   bag_record --output out.mcap --topic /chatter:std_msgs/msg/String [...]
//!   bag_record --output out.mcap --all [--domain N] [--duration SECS] [--compression lz4|none]
//!
//! Recording stops when `--duration` elapses, or (on a terminal) when you press
//! Enter / Ctrl-D. Ctrl-C also stops the process; completed chunks are flushed
//! to disk as they fill, so a Ctrl-C'd file is still readable up to the last
//! flushed chunk. A per-topic message-count summary is printed on a clean stop.

use std::collections::{HashMap, HashSet};
use std::fs::File;
use std::io::{self, BufWriter, IsTerminal, Read};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};

use roswell_ros2_compat::graph::Graph;
use roswell_ros2_compat::raw::{
    raw_qos_for_topic, Compression, McapWriter, RawDdsSubscriber, RawSink,
};
use roswell_ros2_compat::time::Time;
use roswell_ros2_compat::transport::Dds;
use roswell_ros2_compat::tunnel::TopicRoute;

#[path = "shared/argcursor.rs"]
mod argcursor;
use argcursor::ArgCursor;

const USAGE: &str = "usage: bag_record --output out.mcap (--all | --topic /name:pkg/msg/Type ...) \
     [--domain N] [--duration SECS] [--compression lz4|none]";

/// How often `--all` re-scans the graph to pick up late-appearing topics.
const RESCAN_INTERVAL: Duration = Duration::from_secs(1);
/// Idle sleep between poll passes so the loop does not busy-spin.
const POLL_IDLE: Duration = Duration::from_millis(2);

fn main() -> io::Result<()> {
    let args = Args::parse()?;

    let dds = Dds::new(args.domain);
    let file = BufWriter::new(File::create(&args.output)?);
    let mut writer = match args.compression {
        Compression::None => McapWriter::new(file)?,
        Compression::Lz4 => McapWriter::with_compression(file, args.compression)?,
    };

    // topic -> its subscriber; `subscribed` also tracks topics so `--all`
    // re-scans only add newcomers.
    let mut subscribers: Vec<(String, RawDdsSubscriber)> = Vec::new();
    let mut subscribed: HashSet<String> = HashSet::new();
    let mut counts: HashMap<String, u64> = HashMap::new();

    for route in &args.topics {
        subscribe(
            &dds,
            &mut writer,
            &mut subscribers,
            &mut subscribed,
            &route.topic,
            &route.ros_type,
        );
    }

    let stop = install_stop_signal(args.duration);
    println!(
        "bag_record: writing {} (compression={}, {})",
        args.output,
        args.compression_label(),
        args.mode_label(),
    );

    let mut last_scan = Instant::now();
    // Force an immediate first scan in `--all` mode.
    if args.all {
        discover_into(&dds, &mut writer, &mut subscribers, &mut subscribed);
    }

    while !stop.load(Ordering::SeqCst) {
        if args.all && last_scan.elapsed() >= RESCAN_INTERVAL {
            discover_into(&dds, &mut writer, &mut subscribers, &mut subscribed);
            last_scan = Instant::now();
        }

        let mut got_any = false;
        for (topic, sub) in &mut subscribers {
            while let Some(msg) = sub.take() {
                got_any = true;
                let ts = Time::now_system().as_nanos().max(0);
                writer.write(topic, ts, &msg)?;
                *counts.entry(topic.clone()).or_insert(0) += 1;
            }
        }
        if !got_any {
            thread::sleep(POLL_IDLE);
        }
    }

    let file = writer.finish()?;
    file.into_inner().map_err(io::IntoInnerError::into_error)?;
    print_summary(&args.output, &counts);
    Ok(())
}

/// Subscribe to `topic`/`ros_type` unless already subscribed. The QoS chosen
/// for the subscription is also recorded into the writer's channel metadata so
/// replay can restore it. Discovered topic data does not expose the publisher's
/// offered QoS (RustDDS surfaces only name/type), so this records the QoS our
/// subscription requested via [`raw_qos_for_topic`].
fn subscribe(
    dds: &Dds,
    writer: &mut McapWriter<BufWriter<File>>,
    subscribers: &mut Vec<(String, RawDdsSubscriber)>,
    subscribed: &mut HashSet<String>,
    topic: &str,
    ros_type: &str,
) {
    if ros_type.is_empty() || !subscribed.insert(topic.to_string()) {
        return;
    }
    let qos = raw_qos_for_topic(topic);
    writer.set_channel_qos(topic, &qos.profile());
    let sub = RawDdsSubscriber::new(dds.participant(), topic, ros_type, qos);
    subscribers.push((topic.to_string(), sub));
}

/// Discover the current graph and subscribe to any topic not seen yet.
fn discover_into(
    dds: &Dds,
    writer: &mut McapWriter<BufWriter<File>>,
    subscribers: &mut Vec<(String, RawDdsSubscriber)>,
    subscribed: &mut HashSet<String>,
) {
    for topic in Graph::discover(dds).topics {
        if !subscribed.contains(&topic.name) {
            println!("bag_record: + {} [{}]", topic.name, topic.ros_type);
            subscribe(
                dds,
                writer,
                subscribers,
                subscribed,
                &topic.name,
                &topic.ros_type,
            );
        }
    }
}

/// A flag flipped to `true` when recording should stop: after `duration` (if
/// set) and, on a terminal, when stdin sees a line or EOF (Enter / Ctrl-D).
fn install_stop_signal(duration: Option<Duration>) -> Arc<AtomicBool> {
    let stop = Arc::new(AtomicBool::new(false));
    if let Some(duration) = duration {
        let stop = stop.clone();
        thread::spawn(move || {
            thread::sleep(duration);
            stop.store(true, Ordering::SeqCst);
        });
    }
    if io::stdin().is_terminal() {
        let stop = stop.clone();
        thread::spawn(move || {
            let mut byte = [0u8; 1];
            // Blocks until a byte (Enter) or EOF (Ctrl-D); either stops us.
            let _ = io::stdin().read(&mut byte);
            stop.store(true, Ordering::SeqCst);
        });
    }
    stop
}

fn print_summary(output: &str, counts: &HashMap<String, u64>) {
    let total: u64 = counts.values().sum();
    println!("bag_record: wrote {total} messages to {output}");
    let mut rows: Vec<_> = counts.iter().collect();
    rows.sort_by(|a, b| b.1.cmp(a.1).then_with(|| a.0.cmp(b.0)));
    for (topic, count) in rows {
        println!("  {count:>8}  {topic}");
    }
}

struct Args {
    output: String,
    domain: u16,
    duration: Option<Duration>,
    compression: Compression,
    all: bool,
    topics: Vec<TopicRoute>,
}

impl Args {
    fn parse() -> io::Result<Self> {
        let mut c = ArgCursor::new(USAGE);
        let mut output = None;
        let mut domain = 0u16;
        let mut duration = None;
        let mut compression = Compression::default();
        let mut all = false;
        let mut topics = Vec::new();
        while let Some(arg) = c.next_arg() {
            match arg.as_str() {
                "--output" | "-o" => output = Some(c.value()?),
                "--domain" => domain = c.parse()?,
                "--duration" => {
                    let secs: f64 = c.parse()?;
                    if !secs.is_finite() || secs <= 0.0 {
                        return Err(c.usage());
                    }
                    duration = Some(Duration::from_secs_f64(secs));
                }
                "--compression" => {
                    compression = c
                        .value()?
                        .parse()
                        .map_err(|e| io::Error::new(io::ErrorKind::InvalidInput, e))?;
                }
                "--all" => all = true,
                "--topic" => topics.push(TopicRoute::parse(&c.value()?)?),
                _ => return Err(c.usage()),
            }
        }
        let output = output.ok_or_else(|| c.usage())?;
        if !all && topics.is_empty() {
            return Err(c.usage());
        }
        Ok(Self {
            output,
            domain,
            duration,
            compression,
            all,
            topics,
        })
    }

    fn compression_label(&self) -> &'static str {
        match self.compression {
            Compression::None => "none",
            Compression::Lz4 => "lz4",
        }
    }

    fn mode_label(&self) -> String {
        if self.all {
            "all discovered topics".to_string()
        } else {
            format!("{} topic(s)", self.topics.len())
        }
    }
}

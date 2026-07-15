//! Replay ROS2 CDR samples from an MCAP file type-blind.
//!
//! Usage:
//!   bag_play <file.mcap> [--domain N] [--speed F] [--clock|--no-clock]
//!            [--no-qos-restore] [--topic /name]
//!
//! By default each replay publisher uses the QoS recorded in the bag's channel
//! metadata (falling back to the Default preset when absent); `--no-qos-restore`
//! forces the Default preset for every topic.

use std::collections::{HashMap, HashSet};
use std::time::Instant;

use roscmp_dds::raw::{RawDdsPublisher, RawPlayback, RawQos, RawSampleReader};
use roscmp_dds::time::{ClockMsg, Time};
use roscmp_dds::transport::{Dds, MsgPublisher, Qos, Transport};

fn main() {
    let args = Args::parse();
    let mut reader = match RawSampleReader::open(&args.path) {
        Ok(reader) => reader,
        Err(err) => {
            eprintln!("bag_play: failed to read {}: {err}", args.path);
            std::process::exit(2);
        }
    };

    let dds = Dds::new(args.domain);
    let clock = args
        .clock
        .then(|| dds.publisher::<ClockMsg>("/clock", Qos::Default));
    let playback = RawPlayback {
        publish_clock: args.clock,
        speed: args.speed,
    };
    println!(
        "bag_play: streaming {}, clock={}, speed={}",
        args.path, args.clock, args.speed
    );

    // One publisher per topic (a ROS topic carries exactly one type), keyed by
    // topic so the hot path looks up by &str without per-sample allocation.
    let mut publishers: HashMap<String, RawDdsPublisher> = HashMap::new();
    let mut first_log_time = None;
    let mut count = 0u64;
    let wall_start = Instant::now();

    while let Some(item) = reader.next() {
        let sample = match item {
            Ok(sample) => sample,
            Err(err) => {
                eprintln!("bag_play: corrupt MCAP in {}: {err}", args.path);
                std::process::exit(2);
            }
        };
        if !(args.topics.is_empty() || args.topics.contains(&sample.topic)) {
            continue;
        }
        let first = *first_log_time.get_or_insert(sample.log_time);
        playback.sleep_until(first, sample.log_time, wall_start);
        if let Some(clock) = &clock {
            clock.publish(ClockMsg {
                clock: Time::from_nanos(sample.log_time.min(i64::MAX as u64) as i64).to_msg(),
            });
        }
        if !publishers.contains_key(sample.topic.as_str()) {
            // Restore the recorded QoS from channel metadata when present and
            // not disabled; otherwise fall back to the Default preset.
            let restored = args
                .qos_restore
                .then(|| reader.topic_qos(&sample.topic))
                .flatten();
            let publisher = match restored {
                Some(profile) => RawDdsPublisher::with_policies(
                    dds.participant(),
                    &sample.topic,
                    sample.msg.ros_type(),
                    &profile.policies(),
                ),
                None => RawDdsPublisher::new(
                    dds.participant(),
                    &sample.topic,
                    sample.msg.ros_type(),
                    RawQos::Default,
                ),
            };
            publishers.insert(sample.topic.clone(), publisher);
        }
        publishers[sample.topic.as_str()].publish(&sample.msg);
        count += 1;
    }

    if count == 0 {
        eprintln!("bag_play: no matching CDR samples in {}", args.path);
        return;
    }
    println!(
        "bag_play: published {count} samples across {} topics",
        publishers.len()
    );
}

struct Args {
    path: String,
    domain: u16,
    speed: f64,
    clock: bool,
    qos_restore: bool,
    topics: HashSet<String>,
}

impl Args {
    fn parse() -> Self {
        let mut iter = std::env::args().skip(1);
        let path = iter.next().unwrap_or_else(|| {
            eprintln!(
                "usage: bag_play <file.mcap> [--domain N] [--speed F] [--no-clock] \
                 [--no-qos-restore] [--topic /name]"
            );
            std::process::exit(2);
        });
        let mut args = Self {
            path,
            domain: 0,
            speed: 1.0,
            clock: true,
            qos_restore: true,
            topics: HashSet::new(),
        };
        while let Some(arg) = iter.next() {
            match arg.as_str() {
                "--domain" => {
                    args.domain = iter
                        .next()
                        .expect("--domain requires a value")
                        .parse()
                        .expect("domain must be u16");
                }
                "--speed" => {
                    args.speed = iter
                        .next()
                        .expect("--speed requires a value")
                        .parse()
                        .expect("speed must be numeric");
                }
                "--clock" => args.clock = true,
                "--no-clock" => args.clock = false,
                "--no-qos-restore" => args.qos_restore = false,
                "--topic" => {
                    args.topics
                        .insert(iter.next().expect("--topic requires a ROS topic name"));
                }
                other => panic!("unknown argument: {other}"),
            }
        }
        args
    }
}

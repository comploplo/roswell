//! Graph-aware tunnel policies and framing.
//!
//! This is not a VPN. It is the ROS-facing layer we want above any encrypted
//! pipe: per-channel reliability, priority, deadline, and backpressure.

use std::collections::{BTreeMap, BTreeSet, HashMap, VecDeque};
use std::io::{self, Read, Write};
use std::sync::mpsc::{self, Receiver, SyncSender};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

use bytes::Bytes;

use crate::raw::RawMsg;

// Frame magic and kind tags live in the no_std `roscmp-tunnel-core` crate so a
// no-DDS MCU speaks the exact same wire protocol; the codec below and that core
// are pinned together by `tests/tunnel_core_equivalence.rs`.
use roscmp_tunnel_core::kind;
const MAGIC: &[u8; 8] = &roscmp_tunnel_core::MAGIC;
const MAX_FRAME_LEN: usize = roscmp_tunnel_core::MAX_PAYLOAD_LEN;

// Not encoded on the wire (frames carry their own kind byte in `encode_frame`);
// this is only an in-memory routing key, so variants can be added freely.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum ChannelKind {
    Topic,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Priority {
    Low = 0,
    Medium = 1,
    High = 2,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum ReliabilityMode {
    BestEffort,
    Reliable,
    LatestReliable,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum DropPolicy {
    DropOldest,
    DropNewest,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ChannelPolicy {
    pub kind: ChannelKind,
    pub pattern: String,
    pub reliability: ReliabilityMode,
    pub priority: Priority,
    pub drop_policy: DropPolicy,
    pub max_pending: usize,
    pub deadline: Option<Duration>,
    pub watchdog: Option<Duration>,
}

impl ChannelPolicy {
    #[must_use]
    pub fn reliable(kind: ChannelKind, pattern: impl Into<String>) -> Self {
        Self {
            kind,
            pattern: pattern.into(),
            reliability: ReliabilityMode::Reliable,
            priority: Priority::High,
            drop_policy: DropPolicy::DropNewest,
            max_pending: 1024,
            deadline: None,
            watchdog: None,
        }
    }

    #[must_use]
    pub fn best_effort(kind: ChannelKind, pattern: impl Into<String>) -> Self {
        Self {
            kind,
            pattern: pattern.into(),
            reliability: ReliabilityMode::BestEffort,
            priority: Priority::Low,
            drop_policy: DropPolicy::DropOldest,
            max_pending: 8,
            deadline: None,
            watchdog: None,
        }
    }

    #[must_use]
    pub fn latest_reliable(kind: ChannelKind, pattern: impl Into<String>) -> Self {
        Self {
            kind,
            pattern: pattern.into(),
            reliability: ReliabilityMode::LatestReliable,
            priority: Priority::High,
            drop_policy: DropPolicy::DropOldest,
            max_pending: 1,
            deadline: None,
            watchdog: None,
        }
    }

    #[must_use]
    pub fn matches(&self, kind: ChannelKind, name: &str) -> bool {
        self.kind == kind && pattern_matches(&self.pattern, name)
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct TunnelPolicy {
    pub default_topic: ChannelPolicy,
    rules: Vec<ChannelPolicy>,
}

impl Default for TunnelPolicy {
    fn default() -> Self {
        let mut default_topic = ChannelPolicy::best_effort(ChannelKind::Topic, "*");
        default_topic.priority = Priority::Medium;
        Self {
            default_topic,
            rules: Vec::new(),
        }
    }
}

impl TunnelPolicy {
    #[must_use]
    pub fn robot_defaults() -> Self {
        let mut policy = Self::default();
        policy.add(ChannelPolicy {
            priority: Priority::Low,
            max_pending: 2,
            ..ChannelPolicy::best_effort(ChannelKind::Topic, "/camera/*")
        });
        policy.add(ChannelPolicy {
            priority: Priority::Low,
            max_pending: 2,
            ..ChannelPolicy::best_effort(ChannelKind::Topic, "/points*")
        });
        policy.add(ChannelPolicy {
            priority: Priority::High,
            max_pending: 1,
            deadline: Some(Duration::from_millis(100)),
            watchdog: Some(Duration::from_millis(250)),
            ..ChannelPolicy::latest_reliable(ChannelKind::Topic, "/cmd_vel")
        });
        policy.add(ChannelPolicy {
            priority: Priority::High,
            ..ChannelPolicy::reliable(ChannelKind::Topic, "/tf_static")
        });
        policy.add(ChannelPolicy {
            priority: Priority::Medium,
            ..ChannelPolicy::best_effort(ChannelKind::Topic, "/tf")
        });
        policy
    }

    pub fn add(&mut self, rule: ChannelPolicy) {
        self.rules.push(rule);
    }

    #[must_use]
    pub fn for_channel(&self, kind: ChannelKind, name: &str) -> ChannelPolicy {
        self.rules
            .iter()
            .rev()
            .find(|rule| rule.matches(kind, name))
            .cloned()
            .unwrap_or_else(|| self.default_topic.clone())
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum TunnelFrame {
    Hello {
        peer: String,
    },
    TopicSample {
        sequence: u64,
        topic: String,
        stamp_nanos: i64,
        msg: RawMsg,
    },
    Ack {
        sequence: u64,
    },
    Heartbeat {
        stamp_nanos: i64,
    },
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct TopicRoute {
    pub topic: String,
    pub ros_type: String,
}

impl TopicRoute {
    pub fn parse(value: &str) -> io::Result<Self> {
        let Some((topic, ros_type)) = value.split_once(':') else {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "topic route must be /topic:pkg/msg/Type",
            ));
        };
        if !topic.starts_with('/') || ros_type.matches('/').count() != 2 {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "topic route must be /topic:pkg/msg/Type",
            ));
        }
        Ok(Self {
            topic: topic.to_string(),
            ros_type: ros_type.to_string(),
        })
    }
}

impl TunnelFrame {
    #[must_use]
    pub fn sequence(&self) -> Option<u64> {
        match self {
            Self::TopicSample { sequence, .. } | Self::Ack { sequence } => Some(*sequence),
            Self::Hello { .. } | Self::Heartbeat { .. } => None,
        }
    }
}

/// Source of raw ROS topic samples for a tunnel TX loop.
pub trait TopicSampleSource {
    fn take(&mut self) -> Option<RawMsg>;
}

/// Sink for raw ROS topic samples received from a tunnel RX loop.
pub trait TopicSampleSink {
    fn publish(&mut self, msg: &RawMsg);
}

pub struct RoutedTopicSource<S> {
    pub topic: String,
    pub source: S,
}

impl<S> RoutedTopicSource<S> {
    #[must_use]
    pub fn new(topic: impl Into<String>, source: S) -> Self {
        Self {
            topic: topic.into(),
            source,
        }
    }
}

pub struct RoutedTopicSink<S> {
    pub topic: String,
    pub sink: S,
}

impl<S> RoutedTopicSink<S> {
    #[must_use]
    pub fn new(topic: impl Into<String>, sink: S) -> Self {
        Self {
            topic: topic.into(),
            sink,
        }
    }
}

pub struct TopicBridgeTxConfig {
    pub peer: String,
    pub policy: TunnelPolicy,
    pub poll_interval: Duration,
    pub heartbeat_interval: Option<Duration>,
    pub resend_interval: Duration,
    pub resend_window: usize,
    pub reliability: Option<TunnelReliabilityHandle>,
}

impl Default for TopicBridgeTxConfig {
    fn default() -> Self {
        Self {
            peer: "roscmp-dds topic bridge".into(),
            policy: TunnelPolicy::robot_defaults(),
            poll_interval: Duration::from_millis(2),
            heartbeat_interval: Some(Duration::from_secs(1)),
            resend_interval: Duration::from_millis(100),
            resend_window: 128,
            reliability: None,
        }
    }
}

pub struct TopicBridgeRxConfig {
    pub policy: TunnelPolicy,
    pub reliability: Option<TunnelReliabilityHandle>,
}

impl Default for TopicBridgeRxConfig {
    fn default() -> Self {
        Self {
            policy: TunnelPolicy::robot_defaults(),
            reliability: None,
        }
    }
}

#[derive(Clone, Debug, Default)]
pub struct TunnelReliabilityHandle {
    inner: Arc<Mutex<TunnelReliabilityState>>,
}

impl TunnelReliabilityHandle {
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    fn note_ack(&self, sequence: u64) {
        self.inner
            .lock()
            .expect("reliability state poisoned")
            .ack(sequence);
    }

    fn note_sent(&self, queued: &QueuedFrame, resend_window: usize) {
        self.inner
            .lock()
            .expect("reliability state poisoned")
            .sent(queued, resend_window);
    }

    fn due_resends(&self, now: Instant, resend_interval: Duration) -> Vec<TunnelFrame> {
        self.inner
            .lock()
            .expect("reliability state poisoned")
            .due_resends(now, resend_interval)
    }

    #[must_use]
    pub fn pending(&self) -> usize {
        self.inner
            .lock()
            .expect("reliability state poisoned")
            .pending
            .len()
    }

    #[must_use]
    pub fn stats(&self) -> TunnelReliabilityStats {
        self.inner
            .lock()
            .expect("reliability state poisoned")
            .stats
            .clone()
    }
}

#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct TunnelReliabilityStats {
    pub acked: usize,
    pub ack_sent: usize,
    pub resent: usize,
    pub resend_dropped: usize,
}

#[derive(Clone, Debug)]
struct PendingReliableFrame {
    frame: TunnelFrame,
    last_sent: Instant,
}

#[derive(Debug, Default)]
struct TunnelReliabilityState {
    pending: BTreeMap<u64, PendingReliableFrame>,
    stats: TunnelReliabilityStats,
}

impl TunnelReliabilityState {
    fn ack(&mut self, sequence: u64) {
        if self.pending.remove(&sequence).is_some() {
            self.stats.acked += 1;
        }
    }

    fn sent(&mut self, queued: &QueuedFrame, resend_window: usize) {
        if !queued.policy.requires_ack() {
            return;
        }
        let Some(sequence) = queued.frame.sequence() else {
            return;
        };
        self.pending.insert(
            sequence,
            PendingReliableFrame {
                frame: queued.frame.clone(),
                last_sent: Instant::now(),
            },
        );
        while self.pending.len() > resend_window {
            if let Some(sequence) = self.pending.keys().next().copied() {
                self.pending.remove(&sequence);
                self.stats.resend_dropped += 1;
            }
        }
    }

    fn due_resends(&mut self, now: Instant, resend_interval: Duration) -> Vec<TunnelFrame> {
        let mut frames = Vec::new();
        for pending in self.pending.values_mut() {
            if now.duration_since(pending.last_sent) >= resend_interval {
                pending.last_sent = now;
                frames.push(pending.frame.clone());
            }
        }
        self.stats.resent += frames.len();
        frames
    }
}

impl ChannelPolicy {
    #[must_use]
    pub fn requires_ack(&self) -> bool {
        matches!(
            self.reliability,
            ReliabilityMode::Reliable | ReliabilityMode::LatestReliable
        )
    }
}

#[derive(Debug, Default)]
pub struct TopicBridgeStats {
    pub samples_sent: usize,
    pub samples_received: usize,
    pub dropped_outbound: usize,
    pub dropped_unknown_topic: usize,
    pub heartbeats_sent: usize,
}

pub struct TopicBridgeTx<S> {
    sources: Vec<RoutedTopicSource<S>>,
    config: TopicBridgeTxConfig,
    queue: OutboundQueue,
    last_heartbeat: Instant,
    pub stats: TopicBridgeStats,
}

impl<S: TopicSampleSource> TopicBridgeTx<S> {
    #[must_use]
    pub fn new(sources: Vec<RoutedTopicSource<S>>, config: TopicBridgeTxConfig) -> Self {
        Self {
            sources,
            config,
            queue: OutboundQueue::default(),
            last_heartbeat: Instant::now(),
            stats: TopicBridgeStats::default(),
        }
    }

    pub fn send_hello(&self, carrier: &mut impl FrameSink) -> io::Result<()> {
        carrier.send_frame(&TunnelFrame::Hello {
            peer: self.config.peer.clone(),
        })
    }

    pub fn poll_once(&mut self, carrier: &mut impl FrameSink) -> io::Result<()> {
        if let Some(reliability) = &self.config.reliability {
            for frame in reliability.due_resends(Instant::now(), self.config.resend_interval) {
                carrier.send_frame(&frame)?;
            }
        }
        for routed in &mut self.sources {
            while let Some(msg) = routed.source.take() {
                self.queue.enqueue_topic(
                    &self.config.policy,
                    &routed.topic,
                    now_system_nanos(),
                    msg,
                );
            }
        }
        while let Some(queued) = self.queue.pop_next() {
            carrier.send_frame(&queued.frame)?;
            self.stats.samples_sent += 1;
            if let Some(reliability) = &self.config.reliability {
                reliability.note_sent(&queued, self.config.resend_window);
            }
        }
        if self.should_send_heartbeat() {
            carrier.send_frame(&TunnelFrame::Heartbeat {
                stamp_nanos: now_system_nanos(),
            })?;
            self.stats.heartbeats_sent += 1;
            self.last_heartbeat = Instant::now();
        }
        self.stats.dropped_outbound = self.queue.dropped();
        Ok(())
    }

    fn should_send_heartbeat(&self) -> bool {
        self.config
            .heartbeat_interval
            .is_some_and(|interval| self.last_heartbeat.elapsed() >= interval)
    }
}

/// Max retained sequence numbers for duplicate suppression in
/// [`TopicBridgeRx`]; bounds memory over long-lived reliable sessions.
const DEDUP_WINDOW: usize = 4096;

pub struct TopicBridgeRx<S> {
    sinks: Vec<RoutedTopicSink<S>>,
    sink_by_topic: HashMap<String, usize>,
    // Bounded dedup window over the sender's monotonic sequences; a duplicate
    // older than the window would re-publish, but reliability retries are
    // near-term by design. See `DEDUP_WINDOW`.
    received_sequences: BTreeSet<u64>,
    config: TopicBridgeRxConfig,
    pub stats: TopicBridgeStats,
}

impl<S: TopicSampleSink> TopicBridgeRx<S> {
    #[must_use]
    pub fn new(sinks: Vec<RoutedTopicSink<S>>, config: TopicBridgeRxConfig) -> Self {
        let sink_by_topic = sinks
            .iter()
            .enumerate()
            .map(|(index, routed)| (routed.topic.clone(), index))
            .collect();
        Self {
            sinks,
            sink_by_topic,
            received_sequences: BTreeSet::new(),
            config,
            stats: TopicBridgeStats::default(),
        }
    }

    pub fn recv_once<C>(
        &mut self,
        carrier: &mut C,
        mut on_hello: impl FnMut(&str),
    ) -> io::Result<()>
    where
        C: FrameSource + FrameSink,
    {
        match carrier.recv_frame()? {
            TunnelFrame::TopicSample {
                sequence,
                topic,
                msg,
                ..
            } => {
                let requires_ack = self
                    .config
                    .policy
                    .for_channel(ChannelKind::Topic, &topic)
                    .requires_ack();
                if requires_ack {
                    carrier.send_frame(&TunnelFrame::Ack { sequence })?;
                    if let Some(reliability) = &self.config.reliability {
                        reliability
                            .inner
                            .lock()
                            .expect("reliability state poisoned")
                            .stats
                            .ack_sent += 1;
                    }
                }
                let is_duplicate = requires_ack && !self.received_sequences.insert(sequence);
                // ponytail: fixed-size sliding window (oldest evicted); enough
                // for near-term retry dedup, revisit if retry horizons grow.
                while self.received_sequences.len() > DEDUP_WINDOW {
                    self.received_sequences.pop_first();
                }
                if let Some(index) = self.sink_by_topic.get(&topic) {
                    if !is_duplicate {
                        self.sinks[*index].sink.publish(&msg);
                        self.stats.samples_received += 1;
                    }
                } else {
                    self.stats.dropped_unknown_topic += 1;
                }
            }
            TunnelFrame::Hello { peer } => on_hello(&peer),
            TunnelFrame::Ack { sequence } => {
                if let Some(reliability) = &self.config.reliability {
                    reliability.note_ack(sequence);
                }
            }
            TunnelFrame::Heartbeat { .. } => {}
        }
        Ok(())
    }
}

pub fn run_topic_tx_loop<C, S>(
    carrier: &mut C,
    sources: Vec<RoutedTopicSource<S>>,
    config: TopicBridgeTxConfig,
) -> io::Result<()>
where
    C: FrameSink,
    S: TopicSampleSource,
{
    let mut bridge = TopicBridgeTx::new(sources, config);
    bridge.send_hello(carrier)?;

    loop {
        bridge.poll_once(carrier)?;
        thread::sleep(bridge.config.poll_interval);
    }
}

pub fn run_topic_rx_loop<C, S>(
    carrier: &mut C,
    sinks: Vec<RoutedTopicSink<S>>,
    mut on_hello: impl FnMut(&str),
    config: TopicBridgeRxConfig,
) -> io::Result<()>
where
    C: FrameSource + FrameSink,
    S: TopicSampleSink,
{
    let mut bridge = TopicBridgeRx::new(sinks, config);

    loop {
        bridge.recv_once(carrier, &mut on_hello)?;
    }
}

fn now_system_nanos() -> i64 {
    let now = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default();
    i64::try_from(now.as_nanos()).unwrap_or(i64::MAX)
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct QueuedFrame {
    pub policy: ChannelPolicy,
    pub frame: TunnelFrame,
}

#[derive(Debug, Default)]
pub struct OutboundQueue {
    high: VecDeque<QueuedFrame>,
    medium: VecDeque<QueuedFrame>,
    low: VecDeque<QueuedFrame>,
    latest_by_channel: HashMap<(ChannelKind, String), (Priority, u64)>,
    next_sequence: u64,
    dropped: usize,
}

impl OutboundQueue {
    #[must_use]
    pub fn dropped(&self) -> usize {
        self.dropped
    }

    pub fn enqueue_topic(
        &mut self,
        policy: &TunnelPolicy,
        topic: &str,
        stamp_nanos: i64,
        msg: RawMsg,
    ) -> u64 {
        let channel_policy = policy.for_channel(ChannelKind::Topic, topic);
        self.next_sequence = self.next_sequence.wrapping_add(1);
        let sequence = self.next_sequence;
        let frame = QueuedFrame {
            policy: channel_policy.clone(),
            frame: TunnelFrame::TopicSample {
                sequence,
                topic: topic.to_string(),
                stamp_nanos,
                msg,
            },
        };
        self.enqueue(ChannelKind::Topic, topic, frame);
        sequence
    }

    pub fn pop_next(&mut self) -> Option<QueuedFrame> {
        self.high
            .pop_front()
            .or_else(|| self.medium.pop_front())
            .or_else(|| self.low.pop_front())
    }

    fn enqueue(&mut self, kind: ChannelKind, name: &str, frame: QueuedFrame) {
        if frame.policy.reliability == ReliabilityMode::LatestReliable {
            self.remove_latest(kind, name);
            self.latest_by_channel.insert(
                (kind, name.to_string()),
                (frame.policy.priority, frame.frame.sequence().unwrap_or(0)),
            );
        }
        let drop_oldest = frame.policy.drop_policy == DropPolicy::DropOldest;
        let max_pending = frame.policy.max_pending;
        {
            let queue = self.queue_mut(frame.policy.priority);
            // Machine-checked (Creusot): an enqueue never grows this channel's
            // backlog past `max_pending`, and never worsens an over-full shared
            // lane; `drop_front` is never requested on an empty queue. See
            // `roscmp_verify::plan_enqueue`.
            let plan = roscmp_verify::plan_enqueue(queue.len(), max_pending, drop_oldest);
            if plan.drop_front {
                queue.pop_front();
            }
            if plan.push {
                queue.push_back(frame);
            }
            // Exactly one of {evicted an old frame, rejected the new frame}
            // happens when the lane was full; neither when it had room.
            self.dropped += usize::from(plan.drop_front) + usize::from(!plan.push);
        }
    }

    fn remove_latest(&mut self, kind: ChannelKind, name: &str) {
        let key = (kind, name.to_string());
        let Some((priority, sequence)) = self.latest_by_channel.remove(&key) else {
            return;
        };
        let queue = self.queue_mut(priority);
        if let Some(pos) = queue
            .iter()
            .position(|queued| queued.frame.sequence() == Some(sequence))
        {
            queue.remove(pos);
            self.dropped += 1;
        }
    }

    fn queue_mut(&mut self, priority: Priority) -> &mut VecDeque<QueuedFrame> {
        match priority {
            Priority::High => &mut self.high,
            Priority::Medium => &mut self.medium,
            Priority::Low => &mut self.low,
        }
    }
}

/// Sends tunnel frames over a concrete carrier.
///
/// TCP streams, USB CDC serial ports, shared-memory queue writers, and test
/// buffers can all implement this same small surface.
pub trait FrameSink {
    fn send_frame(&mut self, frame: &TunnelFrame) -> io::Result<()>;
}

/// Receives tunnel frames from a concrete carrier.
///
/// This is split from [`FrameSink`] so asymmetric transports, such as one
/// shared-memory ring per direction, do not need to pretend they are a single
/// bidirectional stream.
pub trait FrameSource {
    fn recv_frame(&mut self) -> io::Result<TunnelFrame>;
}

/// Bidirectional tunnel carrier.
pub trait TunnelCarrier: FrameSink + FrameSource {}

impl<T: FrameSink + FrameSource> TunnelCarrier for T {}

/// Bounded in-process frame queue pair.
///
/// This is the queue-carrier counterpart to [`FramedIo`]. It is useful for
/// tests, in-process bridges, and as the API shape for a future OS shared
/// memory ring-buffer implementation.
#[must_use]
pub fn bounded_frame_queue(capacity: usize) -> (BoundedFrameSink, BoundedFrameSource) {
    let (tx, rx) = mpsc::sync_channel(capacity);
    (BoundedFrameSink { tx }, BoundedFrameSource { rx })
}

#[derive(Clone, Debug)]
pub struct BoundedFrameSink {
    tx: SyncSender<TunnelFrame>,
}

impl FrameSink for BoundedFrameSink {
    fn send_frame(&mut self, frame: &TunnelFrame) -> io::Result<()> {
        self.tx
            .send(frame.clone())
            .map_err(|_| io::Error::new(io::ErrorKind::BrokenPipe, "frame queue closed"))
    }
}

#[derive(Debug)]
pub struct BoundedFrameSource {
    rx: Receiver<TunnelFrame>,
}

impl FrameSource for BoundedFrameSource {
    fn recv_frame(&mut self) -> io::Result<TunnelFrame> {
        self.rx
            .recv()
            .map_err(|_| io::Error::new(io::ErrorKind::UnexpectedEof, "frame queue closed"))
    }
}

/// Length-prefixed tunnel framing over any `Read`/`Write` object.
///
/// This is the adapter for stream-like carriers: `TcpStream`, Unix sockets,
/// USB serial handles, in-memory cursors, and similar transports.
#[derive(Clone, Debug)]
pub struct FramedIo<T> {
    inner: T,
}

impl<T> FramedIo<T> {
    #[must_use]
    pub fn new(inner: T) -> Self {
        Self { inner }
    }

    #[must_use]
    pub fn into_inner(self) -> T {
        self.inner
    }

    pub fn get_mut(&mut self) -> &mut T {
        &mut self.inner
    }
}

impl<T: Write> FrameSink for FramedIo<T> {
    fn send_frame(&mut self, frame: &TunnelFrame) -> io::Result<()> {
        write_frame(&mut self.inner, frame)
    }
}

impl<T: Read> FrameSource for FramedIo<T> {
    fn recv_frame(&mut self) -> io::Result<TunnelFrame> {
        read_frame(&mut self.inner)
    }
}

/// Length-prefixed tunnel framing that scans forward to the next frame magic.
///
/// This is useful for byte-stream carriers that can start mid-frame or receive
/// noise during reconnect, such as USB CDC serial links.
#[derive(Clone, Debug)]
pub struct ResyncFramedIo<T> {
    inner: T,
}

impl<T> ResyncFramedIo<T> {
    #[must_use]
    pub fn new(inner: T) -> Self {
        Self { inner }
    }

    #[must_use]
    pub fn into_inner(self) -> T {
        self.inner
    }

    pub fn get_mut(&mut self) -> &mut T {
        &mut self.inner
    }
}

impl<T: Write> FrameSink for ResyncFramedIo<T> {
    fn send_frame(&mut self, frame: &TunnelFrame) -> io::Result<()> {
        write_frame(&mut self.inner, frame)
    }
}

impl<T: Read> FrameSource for ResyncFramedIo<T> {
    fn recv_frame(&mut self) -> io::Result<TunnelFrame> {
        read_frame_resync(&mut self.inner)
    }
}

pub fn write_frame(writer: &mut impl Write, frame: &TunnelFrame) -> io::Result<()> {
    let payload = encode_frame(frame)?;
    writer.write_all(MAGIC)?;
    writer.write_all(
        &u32::try_from(payload.len())
            .map_err(|_| io::Error::new(io::ErrorKind::InvalidInput, "tunnel frame too large"))?
            .to_le_bytes(),
    )?;
    writer.write_all(&payload)
}

pub fn read_frame(reader: &mut impl Read) -> io::Result<TunnelFrame> {
    let mut magic = [0; 8];
    reader.read_exact(&mut magic)?;
    if &magic != MAGIC {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "bad tunnel frame magic",
        ));
    }
    let mut len_buf = [0; 4];
    reader.read_exact(&mut len_buf)?;
    let len = u32::from_le_bytes(len_buf) as usize;
    if len > MAX_FRAME_LEN {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "tunnel frame exceeds size limit",
        ));
    }
    let mut payload = vec![0; len];
    reader.read_exact(&mut payload)?;
    decode_frame(&payload)
}

pub fn read_frame_resync(reader: &mut impl Read) -> io::Result<TunnelFrame> {
    let mut window = [0; MAGIC.len()];
    reader.read_exact(&mut window)?;
    while &window != MAGIC {
        window.copy_within(1.., 0);
        reader.read_exact(&mut window[MAGIC.len() - 1..])?;
    }
    let mut len_buf = [0; 4];
    reader.read_exact(&mut len_buf)?;
    let len = u32::from_le_bytes(len_buf) as usize;
    if len > MAX_FRAME_LEN {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "tunnel frame exceeds size limit",
        ));
    }
    let mut payload = vec![0; len];
    reader.read_exact(&mut payload)?;
    decode_frame(&payload)
}

fn encode_frame(frame: &TunnelFrame) -> io::Result<Vec<u8>> {
    let mut w = Vec::new();
    match frame {
        TunnelFrame::Hello { peer } => {
            w.push(kind::HELLO);
            write_string(&mut w, peer)?;
        }
        TunnelFrame::TopicSample {
            sequence,
            topic,
            stamp_nanos,
            msg,
        } => {
            w.push(kind::TOPIC_SAMPLE);
            w.write_all(&sequence.to_le_bytes())?;
            write_string(&mut w, topic)?;
            write_string(&mut w, msg.ros_type())?;
            w.write_all(&stamp_nanos.to_le_bytes())?;
            write_bytes(&mut w, msg.cdr())?;
        }
        TunnelFrame::Ack { sequence } => {
            w.push(kind::ACK);
            w.write_all(&sequence.to_le_bytes())?;
        }
        TunnelFrame::Heartbeat { stamp_nanos } => {
            w.push(kind::HEARTBEAT);
            w.write_all(&stamp_nanos.to_le_bytes())?;
        }
    }
    Ok(w)
}

fn decode_frame(payload: &[u8]) -> io::Result<TunnelFrame> {
    let mut r = payload;
    let tag = read_u8(&mut r)?;
    match tag {
        kind::HELLO => Ok(TunnelFrame::Hello {
            peer: read_string(&mut r)?,
        }),
        kind::TOPIC_SAMPLE => {
            let sequence = read_u64(&mut r)?;
            let topic = read_string(&mut r)?;
            let ros_type = read_string(&mut r)?;
            let stamp_nanos = read_i64(&mut r)?;
            let cdr = read_bytes(&mut r)?;
            Ok(TunnelFrame::TopicSample {
                sequence,
                topic,
                stamp_nanos,
                msg: RawMsg::new(ros_type, cdr),
            })
        }
        kind::ACK => Ok(TunnelFrame::Ack {
            sequence: read_u64(&mut r)?,
        }),
        kind::HEARTBEAT => Ok(TunnelFrame::Heartbeat {
            stamp_nanos: read_i64(&mut r)?,
        }),
        _ => Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "unknown tunnel frame kind",
        )),
    }
}

fn pattern_matches(pattern: &str, name: &str) -> bool {
    pattern == "*"
        || pattern == name
        || pattern
            .strip_suffix('*')
            .is_some_and(|prefix| name.starts_with(prefix))
}

fn write_string(writer: &mut impl Write, value: &str) -> io::Result<()> {
    write_bytes(writer, value.as_bytes())
}

fn read_string(reader: &mut &[u8]) -> io::Result<String> {
    String::from_utf8(read_bytes(reader)?.to_vec())
        .map_err(|_| io::Error::new(io::ErrorKind::InvalidData, "bad tunnel utf8 string"))
}

fn write_bytes(writer: &mut impl Write, value: &[u8]) -> io::Result<()> {
    writer.write_all(
        &u32::try_from(value.len())
            .map_err(|_| io::Error::new(io::ErrorKind::InvalidInput, "tunnel field too large"))?
            .to_le_bytes(),
    )?;
    writer.write_all(value)
}

fn read_bytes(reader: &mut &[u8]) -> io::Result<Bytes> {
    let len = read_u32(reader)? as usize;
    if reader.len() < len {
        return Err(io::Error::new(
            io::ErrorKind::UnexpectedEof,
            "truncated tunnel bytes",
        ));
    }
    let (data, rest) = reader.split_at(len);
    *reader = rest;
    Ok(Bytes::copy_from_slice(data))
}

fn read_u8(reader: &mut &[u8]) -> io::Result<u8> {
    if reader.is_empty() {
        return Err(io::Error::new(
            io::ErrorKind::UnexpectedEof,
            "truncated tunnel u8",
        ));
    }
    let (value, rest) = reader.split_at(1);
    *reader = rest;
    Ok(value[0])
}

fn read_u32(reader: &mut &[u8]) -> io::Result<u32> {
    let bytes = read_array::<4>(reader)?;
    Ok(u32::from_le_bytes(bytes))
}

fn read_u64(reader: &mut &[u8]) -> io::Result<u64> {
    let bytes = read_array::<8>(reader)?;
    Ok(u64::from_le_bytes(bytes))
}

fn read_i64(reader: &mut &[u8]) -> io::Result<i64> {
    let bytes = read_array::<8>(reader)?;
    Ok(i64::from_le_bytes(bytes))
}

fn read_array<const N: usize>(reader: &mut &[u8]) -> io::Result<[u8; N]> {
    if reader.len() < N {
        return Err(io::Error::new(
            io::ErrorKind::UnexpectedEof,
            "truncated tunnel integer",
        ));
    }
    let (data, rest) = reader.split_at(N);
    *reader = rest;
    let mut bytes = [0; N];
    bytes.copy_from_slice(data);
    Ok(bytes)
}

#[cfg(test)]
mod tests {
    use std::collections::VecDeque;
    use std::io::{self, Cursor};
    use std::time::Duration;

    use super::{
        bounded_frame_queue, read_frame, read_frame_resync, write_frame, ChannelKind, FrameSink,
        FrameSource, FramedIo, OutboundQueue, Priority, ReliabilityMode, RoutedTopicSink,
        RoutedTopicSource, TopicBridgeRx, TopicBridgeRxConfig, TopicBridgeTx, TopicBridgeTxConfig,
        TopicSampleSink, TopicSampleSource, TunnelFrame, TunnelPolicy, TunnelReliabilityHandle,
    };
    use crate::raw::RawMsg;

    #[derive(Default)]
    struct FakeCarrier {
        incoming: VecDeque<TunnelFrame>,
        sent: Vec<TunnelFrame>,
    }

    impl FakeCarrier {
        fn with_incoming(frames: impl IntoIterator<Item = TunnelFrame>) -> Self {
            Self {
                incoming: frames.into_iter().collect(),
                sent: Vec::new(),
            }
        }
    }

    impl FrameSink for FakeCarrier {
        fn send_frame(&mut self, frame: &TunnelFrame) -> io::Result<()> {
            self.sent.push(frame.clone());
            Ok(())
        }
    }

    impl FrameSource for FakeCarrier {
        fn recv_frame(&mut self) -> io::Result<TunnelFrame> {
            self.incoming
                .pop_front()
                .ok_or_else(|| io::Error::new(io::ErrorKind::UnexpectedEof, "no test frame"))
        }
    }

    struct FakeSource {
        samples: VecDeque<RawMsg>,
    }

    impl FakeSource {
        fn new(samples: impl IntoIterator<Item = RawMsg>) -> Self {
            Self {
                samples: samples.into_iter().collect(),
            }
        }
    }

    impl TopicSampleSource for FakeSource {
        fn take(&mut self) -> Option<RawMsg> {
            self.samples.pop_front()
        }
    }

    #[derive(Default)]
    struct FakeSink {
        samples: Vec<RawMsg>,
    }

    impl TopicSampleSink for FakeSink {
        fn publish(&mut self, msg: &RawMsg) {
            self.samples.push(msg.clone());
        }
    }

    #[test]
    fn robot_defaults_make_cmd_vel_latest_reliable_high_priority() {
        let policy = TunnelPolicy::robot_defaults();
        let cmd = policy.for_channel(ChannelKind::Topic, "/cmd_vel");
        assert_eq!(cmd.reliability, ReliabilityMode::LatestReliable);
        assert_eq!(cmd.priority, Priority::High);
        assert_eq!(cmd.max_pending, 1);
    }

    #[test]
    fn outbound_queue_prioritizes_control_over_camera() {
        let policy = TunnelPolicy::robot_defaults();
        let mut queue = OutboundQueue::default();
        queue.enqueue_topic(
            &policy,
            "/camera/front/image_raw",
            1,
            RawMsg::new("sensor_msgs/msg/Image", vec![1]),
        );
        queue.enqueue_topic(
            &policy,
            "/cmd_vel",
            2,
            RawMsg::new("geometry_msgs/msg/Twist", vec![2]),
        );
        let first = queue.pop_next().unwrap();
        match first.frame {
            TunnelFrame::TopicSample { topic, .. } => assert_eq!(topic, "/cmd_vel"),
            _ => panic!("expected topic sample"),
        }
    }

    #[test]
    fn latest_reliable_replaces_stale_pending_command() {
        let policy = TunnelPolicy::robot_defaults();
        let mut queue = OutboundQueue::default();
        queue.enqueue_topic(
            &policy,
            "/cmd_vel",
            1,
            RawMsg::new("geometry_msgs/msg/Twist", vec![1]),
        );
        queue.enqueue_topic(
            &policy,
            "/cmd_vel",
            2,
            RawMsg::new("geometry_msgs/msg/Twist", vec![2]),
        );
        assert_eq!(queue.dropped(), 1);
        let next = queue.pop_next().unwrap();
        match next.frame {
            TunnelFrame::TopicSample {
                stamp_nanos, msg, ..
            } => {
                assert_eq!(stamp_nanos, 2);
                assert_eq!(msg.cdr(), &[2]);
            }
            _ => panic!("expected topic sample"),
        }
        assert!(queue.pop_next().is_none());
    }

    #[test]
    fn tunnel_frame_round_trips_raw_topic_sample() {
        let frame = TunnelFrame::TopicSample {
            sequence: 7,
            topic: "/diagnostics".into(),
            stamp_nanos: 42,
            msg: RawMsg::new("diagnostic_msgs/msg/DiagnosticArray", vec![0, 1, 0, 0]),
        };
        let mut bytes = Vec::new();
        write_frame(&mut bytes, &frame).unwrap();
        let back = read_frame(&mut Cursor::new(bytes)).unwrap();
        assert_eq!(back, frame);
    }

    #[test]
    fn framed_io_carrier_round_trips_raw_topic_sample() {
        let frame = TunnelFrame::TopicSample {
            sequence: 8,
            topic: "/tf".into(),
            stamp_nanos: 99,
            msg: RawMsg::new("tf2_msgs/msg/TFMessage", vec![0, 1, 0, 0]),
        };
        let mut carrier = FramedIo::new(Cursor::new(Vec::new()));
        carrier.send_frame(&frame).unwrap();
        carrier.get_mut().set_position(0);
        let back = carrier.recv_frame().unwrap();
        assert_eq!(back, frame);
    }

    #[test]
    fn resync_reader_skips_noise_before_next_frame_magic() {
        let frame = TunnelFrame::Heartbeat { stamp_nanos: 1234 };
        let mut bytes = b"serial noise".to_vec();
        write_frame(&mut bytes, &frame).unwrap();

        let back = read_frame_resync(&mut Cursor::new(bytes)).unwrap();

        assert_eq!(back, frame);
    }

    #[test]
    fn topic_tx_bridge_polls_sources_and_tracks_reliable_pending() {
        let reliability = TunnelReliabilityHandle::new();
        let mut bridge = TopicBridgeTx::new(
            vec![RoutedTopicSource::new(
                "/cmd_vel",
                FakeSource::new([RawMsg::new("geometry_msgs/msg/Twist", vec![1, 2, 3])]),
            )],
            TopicBridgeTxConfig {
                heartbeat_interval: None,
                reliability: Some(reliability.clone()),
                ..TopicBridgeTxConfig::default()
            },
        );
        let mut carrier = FakeCarrier::default();

        bridge.send_hello(&mut carrier).unwrap();
        bridge.poll_once(&mut carrier).unwrap();

        assert!(matches!(carrier.sent[0], TunnelFrame::Hello { .. }));
        match &carrier.sent[1] {
            TunnelFrame::TopicSample {
                sequence,
                topic,
                msg,
                ..
            } => {
                assert_eq!(*sequence, 1);
                assert_eq!(topic, "/cmd_vel");
                assert_eq!(msg.cdr(), &[1, 2, 3]);
            }
            other => panic!("expected topic sample, got {other:?}"),
        }
        assert_eq!(reliability.pending(), 1);
        assert_eq!(bridge.stats.samples_sent, 1);
    }

    #[test]
    fn topic_rx_bridge_acks_reliable_samples_and_suppresses_duplicate_resends() {
        let reliability = TunnelReliabilityHandle::new();
        let sample = TunnelFrame::TopicSample {
            sequence: 42,
            topic: "/cmd_vel".into(),
            stamp_nanos: 1,
            msg: RawMsg::new("geometry_msgs/msg/Twist", vec![9]),
        };
        let mut carrier = FakeCarrier::with_incoming([sample.clone(), sample]);
        let mut bridge = TopicBridgeRx::new(
            vec![RoutedTopicSink::new("/cmd_vel", FakeSink::default())],
            TopicBridgeRxConfig {
                reliability: Some(reliability.clone()),
                ..TopicBridgeRxConfig::default()
            },
        );

        bridge.recv_once(&mut carrier, |_| {}).unwrap();
        bridge.recv_once(&mut carrier, |_| {}).unwrap();

        assert_eq!(bridge.sinks[0].sink.samples.len(), 1);
        assert_eq!(
            carrier.sent,
            vec![
                TunnelFrame::Ack { sequence: 42 },
                TunnelFrame::Ack { sequence: 42 },
            ]
        );
        assert_eq!(reliability.stats().ack_sent, 2);
    }

    #[test]
    fn topic_rx_bridge_does_not_ack_best_effort_samples() {
        let sample = TunnelFrame::TopicSample {
            sequence: 7,
            topic: "/camera/front/image_raw".into(),
            stamp_nanos: 1,
            msg: RawMsg::new("sensor_msgs/msg/Image", vec![5]),
        };
        let mut carrier = FakeCarrier::with_incoming([sample]);
        let mut bridge = TopicBridgeRx::new(
            vec![RoutedTopicSink::new(
                "/camera/front/image_raw",
                FakeSink::default(),
            )],
            TopicBridgeRxConfig::default(),
        );

        bridge.recv_once(&mut carrier, |_| {}).unwrap();

        assert_eq!(bridge.sinks[0].sink.samples.len(), 1);
        assert!(carrier.sent.is_empty());
    }

    #[test]
    fn ack_frames_clear_reliable_resend_state() {
        let reliability = TunnelReliabilityHandle::new();
        let mut tx = TopicBridgeTx::new(
            vec![RoutedTopicSource::new(
                "/cmd_vel",
                FakeSource::new([RawMsg::new("geometry_msgs/msg/Twist", vec![1])]),
            )],
            TopicBridgeTxConfig {
                heartbeat_interval: None,
                reliability: Some(reliability.clone()),
                ..TopicBridgeTxConfig::default()
            },
        );
        let mut tx_carrier = FakeCarrier::default();
        tx.poll_once(&mut tx_carrier).unwrap();
        assert_eq!(reliability.pending(), 1);

        let mut rx_carrier = FakeCarrier::with_incoming([TunnelFrame::Ack { sequence: 1 }]);
        let mut rx = TopicBridgeRx::<FakeSink>::new(
            Vec::new(),
            TopicBridgeRxConfig {
                reliability: Some(reliability.clone()),
                ..TopicBridgeRxConfig::default()
            },
        );
        rx.recv_once(&mut rx_carrier, |_| {}).unwrap();

        assert_eq!(reliability.pending(), 0);
        assert_eq!(reliability.stats().acked, 1);
    }

    #[test]
    fn reliable_pending_frames_are_resent_until_acked() {
        let reliability = TunnelReliabilityHandle::new();
        let mut bridge = TopicBridgeTx::new(
            vec![RoutedTopicSource::new(
                "/cmd_vel",
                FakeSource::new([RawMsg::new("geometry_msgs/msg/Twist", vec![1])]),
            )],
            TopicBridgeTxConfig {
                heartbeat_interval: None,
                resend_interval: Duration::from_millis(0),
                reliability: Some(reliability.clone()),
                ..TopicBridgeTxConfig::default()
            },
        );
        let mut carrier = FakeCarrier::default();

        bridge.poll_once(&mut carrier).unwrap();
        bridge.poll_once(&mut carrier).unwrap();

        let sample_count = carrier
            .sent
            .iter()
            .filter(|frame| matches!(frame, TunnelFrame::TopicSample { .. }))
            .count();
        assert_eq!(sample_count, 2);
        assert_eq!(reliability.stats().resent, 1);
    }

    #[test]
    fn bounded_frame_queue_moves_frames_between_queue_halves() {
        let (mut sink, mut source) = bounded_frame_queue(1);
        let frame = TunnelFrame::Heartbeat { stamp_nanos: 123 };

        sink.send_frame(&frame).unwrap();

        assert_eq!(source.recv_frame().unwrap(), frame);
    }
}

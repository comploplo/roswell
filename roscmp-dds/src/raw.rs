//! Type-blind message payloads for CLI and bagging work.

use std::collections::HashMap;
use std::io::{self, Cursor, Read, Write};
use std::marker::PhantomData;
use std::path::Path;
use std::time::{Duration, Instant};

use bytes::Bytes;
use rustdds::{
    no_key::{
        DataReader, DataWriter, Decode, DefaultDecoder, DeserializerAdapter, SerializerAdapter,
    },
    policy::{Durability, History, Reliability},
    rpc::SampleIdentity,
    DomainParticipant, QosPolicies, QosPolicyBuilder, RepresentationIdentifier, StatusEvented,
    TopicKind, WriteOptionsBuilder,
};

use crate::codec::service_topics;
use crate::qos::{DurabilityKind, QosEvent, QosProfile, ReliabilityKind};
use crate::transport::{Dds, Qos};
use crate::tunnel::{TopicSampleSink, TopicSampleSource};

/// MCAP channel-metadata key under which we store a topic's recorded QoS.
/// The key follows rosbag2's convention, but the value is roscmp's own compact
/// format (see [`encode_qos`]), not rosbag2 YAML.
pub const QOS_METADATA_KEY: &str = "offered_qos_profiles";

/// Encode the QoS subset we round-trip through bags as a compact string:
/// `reliability=<r>,durability=<d>,depth=<n>` plus `,history=keep_all` when the
/// profile keeps all samples. Deadline/lifespan/liveliness are not persisted.
#[must_use]
pub fn encode_qos(qos: &QosProfile) -> String {
    let reliability = match qos.reliability {
        ReliabilityKind::Reliable => "reliable",
        ReliabilityKind::BestEffort => "best_effort",
    };
    let durability = match qos.durability {
        DurabilityKind::Volatile => "volatile",
        DurabilityKind::TransientLocal => "transient_local",
    };
    let mut s = format!(
        "reliability={reliability},durability={durability},depth={}",
        qos.depth
    );
    if qos.keep_all {
        s.push_str(",history=keep_all");
    }
    s
}

/// Parse a QoS string produced by [`encode_qos`] back into a [`QosProfile`].
/// Missing fields default to the `Qos::Default` preset; unknown keys are
/// ignored for forward compatibility. Returns `None` on a malformed field.
#[must_use]
pub fn parse_qos(s: &str) -> Option<QosProfile> {
    let mut profile = QosProfile::from_preset(Qos::Default);
    for field in s.split(',').filter(|f| !f.trim().is_empty()) {
        let (key, value) = field.split_once('=')?;
        match key.trim() {
            "reliability" => {
                profile.reliability = match value.trim() {
                    "reliable" => ReliabilityKind::Reliable,
                    "best_effort" => ReliabilityKind::BestEffort,
                    _ => return None,
                }
            }
            "durability" => {
                profile.durability = match value.trim() {
                    "volatile" => DurabilityKind::Volatile,
                    "transient_local" => DurabilityKind::TransientLocal,
                    _ => return None,
                }
            }
            "depth" => profile.depth = value.trim().parse().ok()?,
            "history" => {
                profile.keep_all = match value.trim() {
                    "keep_all" => true,
                    "keep_last" => false,
                    _ => return None,
                }
            }
            _ => {}
        }
    }
    Some(profile)
}

/// A ROS topic sample carried as CDR bytes plus its ROS type name.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RawMsg {
    ros_type: String,
    cdr: Bytes,
}

impl RawMsg {
    #[must_use]
    pub fn new(ros_type: impl Into<String>, cdr: impl Into<Bytes>) -> Self {
        Self {
            ros_type: ros_type.into(),
            cdr: cdr.into(),
        }
    }

    #[must_use]
    pub fn ros_type(&self) -> &str {
        &self.ros_type
    }

    #[must_use]
    pub fn cdr(&self) -> &[u8] {
        &self.cdr
    }

    #[must_use]
    pub fn into_cdr(self) -> Bytes {
        self.cdr
    }
}

/// Append-only sink for raw topic samples. MCAP support can implement this
/// trait without coupling recorders to one file format.
pub trait RawSink {
    type Error;

    fn write(&mut self, topic: &str, timestamp_nanos: i64, msg: &RawMsg)
        -> Result<(), Self::Error>;
}

/// Chunk-body compression for [`McapWriter`]. Only codecs the reader can
/// decode are offered: `lz4_flex` encodes lz4, and `ruzstd` is decode-only so
/// there is deliberately no zstd encoder here.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Compression {
    None,
    #[default]
    Lz4,
}

impl Compression {
    /// The MCAP `compression` string written into the Chunk record.
    fn mcap_name(self) -> &'static str {
        match self {
            Compression::None => "",
            Compression::Lz4 => "lz4",
        }
    }
}

impl std::str::FromStr for Compression {
    type Err = String;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s {
            "none" => Ok(Compression::None),
            "lz4" => Ok(Compression::Lz4),
            other => Err(format!("unknown compression '{other}' (want lz4|none)")),
        }
    }
}

/// Default target for the uncompressed bytes accumulated in a chunk before it
/// is flushed; the final chunk may be smaller and a single oversized message
/// still forms its own chunk.
const DEFAULT_CHUNK_SIZE: usize = 1024 * 1024;

/// Upper bound on how many bytes we will *pre*-allocate for a decompressed chunk
/// based on its self-declared `uncompressed_size`. That field is attacker
/// controlled, so an unbounded `Vec::with_capacity` on it is an
/// allocate-the-universe DoS. This only caps the initial capacity *hint* — the
/// decompressor's `read_to_end` still grows the buffer to whatever the stream
/// actually produces, so honest large chunks decode correctly, just with a few
/// extra reallocations.
const MAX_DECOMPRESS_PREALLOC: usize = 64 * 1024 * 1024;

/// In-progress chunk state, present only for the chunked writer path.
struct ChunkState {
    compression: Compression,
    target_size: usize,
    buf: Vec<u8>,
    start_time: Option<u64>,
    end_time: u64,
}

/// Minimal MCAP writer for ROS2 CDR payloads.
///
/// [`McapWriter::new`] writes a flat data section (header, schema/channel
/// records, message records, `DataEnd`, `Footer`, trailing magic) with no
/// chunking. [`McapWriter::with_compression`] instead batches messages into
/// `Chunk` records with optional lz4 compression; schema and channel records
/// stay at the top level in both modes. Summary indexes are omitted; readers
/// scan the data section either way.
pub struct McapWriter<W: Write> {
    writer: W,
    channels: HashMap<(String, String), u16>,
    schemas: HashMap<String, u16>,
    /// Per-topic QoS string (see [`encode_qos`]) written into the channel's
    /// metadata map the first time that topic's channel record is emitted.
    channel_qos: HashMap<String, String>,
    next_channel_id: u16,
    next_schema_id: u16,
    sequence: u32,
    finished: bool,
    chunk: Option<ChunkState>,
}

impl<W: Write> McapWriter<W> {
    pub fn new(writer: W) -> io::Result<Self> {
        Self::build(writer, None)
    }

    /// Create a writer that batches messages into `Chunk` records using the
    /// given compression and the default chunk size.
    pub fn with_compression(writer: W, compression: Compression) -> io::Result<Self> {
        Self::build(
            writer,
            Some(ChunkState {
                compression,
                target_size: DEFAULT_CHUNK_SIZE,
                buf: Vec::new(),
                start_time: None,
                end_time: 0,
            }),
        )
    }

    fn build(mut writer: W, chunk: Option<ChunkState>) -> io::Result<Self> {
        writer.write_all(MCAP_MAGIC)?;
        write_record(&mut writer, Op::Header, |payload| {
            write_string(payload, "ros2")?;
            write_string(payload, "roscmp-dds")?;
            Ok(())
        })?;
        Ok(Self {
            writer,
            channels: HashMap::new(),
            schemas: HashMap::new(),
            channel_qos: HashMap::new(),
            next_channel_id: 1,
            next_schema_id: 1,
            sequence: 0,
            finished: false,
            chunk,
        })
    }

    /// Record the QoS to store in `topic`'s channel metadata. Must be called
    /// before the first sample for that topic is written (the channel record is
    /// emitted lazily on first write); later calls have no effect on an
    /// already-emitted channel.
    pub fn set_channel_qos(&mut self, topic: &str, qos: &QosProfile) {
        self.channel_qos.insert(topic.to_string(), encode_qos(qos));
    }

    pub fn finish(mut self) -> io::Result<W> {
        self.finish_inner()?;
        Ok(self.writer)
    }

    fn channel_id(&mut self, topic: &str, ros_type: &str) -> io::Result<u16> {
        let key = (topic.to_string(), ros_type.to_string());
        if let Some(id) = self.channels.get(&key) {
            return Ok(*id);
        }
        let schema_id = self.schema_id(ros_type)?;
        let id = self.next_channel_id;
        self.next_channel_id = self
            .next_channel_id
            .checked_add(1)
            .ok_or_else(|| io::Error::other("too many MCAP channels"))?;
        let qos = self.channel_qos.get(topic).cloned();
        write_record(&mut self.writer, Op::Channel, |payload| {
            payload.write_all(&id.to_le_bytes())?;
            payload.write_all(&schema_id.to_le_bytes())?;
            write_string(payload, topic)?;
            write_string(payload, "cdr")?;
            match &qos {
                Some(qos) => write_string_map(payload, &[(QOS_METADATA_KEY, qos.as_str())])?,
                None => payload.write_all(&0u32.to_le_bytes())?, // empty metadata map.
            }
            Ok(())
        })?;
        self.channels.insert(key, id);
        Ok(id)
    }

    fn schema_id(&mut self, ros_type: &str) -> io::Result<u16> {
        if let Some(id) = self.schemas.get(ros_type) {
            return Ok(*id);
        }
        let id = self.next_schema_id;
        self.next_schema_id = self
            .next_schema_id
            .checked_add(1)
            .ok_or_else(|| io::Error::other("too many MCAP schemas"))?;
        write_record(&mut self.writer, Op::Schema, |payload| {
            payload.write_all(&id.to_le_bytes())?;
            write_string(payload, ros_type)?;
            write_string(payload, "ros2msg")?;
            payload.write_all(&0u32.to_le_bytes())
        })?;
        self.schemas.insert(ros_type.to_string(), id);
        Ok(id)
    }

    /// Compress (if configured) and emit the buffered chunk, then reset it.
    /// A no-op when chunking is disabled or the buffer is empty.
    fn flush_chunk(&mut self) -> io::Result<()> {
        let Some(chunk) = self.chunk.as_mut() else {
            return Ok(());
        };
        if chunk.buf.is_empty() {
            return Ok(());
        }
        let uncompressed_size = chunk.buf.len() as u64;
        let start_time = chunk.start_time.unwrap_or(0);
        let end_time = chunk.end_time;
        let compression = chunk.compression;
        let records = match compression {
            Compression::None => std::mem::take(&mut chunk.buf),
            Compression::Lz4 => compress_lz4(&chunk.buf)?,
        };
        chunk.buf.clear();
        chunk.start_time = None;
        chunk.end_time = 0;
        write_record(&mut self.writer, Op::Chunk, |payload| {
            payload.write_all(&start_time.to_le_bytes())?;
            payload.write_all(&end_time.to_le_bytes())?;
            payload.write_all(&uncompressed_size.to_le_bytes())?;
            payload.write_all(&0u32.to_le_bytes())?; // uncompressed_crc (0 = absent)
            write_string(payload, compression.mcap_name())?;
            payload.write_all(&(records.len() as u64).to_le_bytes())?;
            payload.write_all(&records)
        })?;
        // Flush so completed chunks reach disk even if the process is killed
        // (e.g. Ctrl-C) before a clean `finish`.
        self.writer.flush()
    }

    fn finish_inner(&mut self) -> io::Result<()> {
        if self.finished {
            return Ok(());
        }
        self.flush_chunk()?;
        write_record(&mut self.writer, Op::DataEnd, |payload| {
            payload.write_all(&0u32.to_le_bytes())
        })?;
        write_record(&mut self.writer, Op::Footer, |payload| {
            payload.write_all(&0u64.to_le_bytes())?;
            payload.write_all(&0u64.to_le_bytes())?;
            payload.write_all(&0u32.to_le_bytes())
        })?;
        self.writer.write_all(MCAP_MAGIC)?;
        self.finished = true;
        Ok(())
    }
}

impl<W: Write> RawSink for McapWriter<W> {
    type Error = io::Error;

    fn write(
        &mut self,
        topic: &str,
        timestamp_nanos: i64,
        msg: &RawMsg,
    ) -> Result<(), Self::Error> {
        let channel_id = self.channel_id(topic, msg.ros_type())?;
        self.sequence = self.sequence.wrapping_add(1);
        let sequence = self.sequence;
        let timestamp = u64::try_from(timestamp_nanos)
            .map_err(|_| io::Error::new(io::ErrorKind::InvalidInput, "negative MCAP timestamp"))?;
        let build = |payload: &mut Vec<u8>| {
            payload.write_all(&channel_id.to_le_bytes())?;
            payload.write_all(&sequence.to_le_bytes())?;
            payload.write_all(&timestamp.to_le_bytes())?;
            payload.write_all(&timestamp.to_le_bytes())?;
            payload.write_all(msg.cdr())
        };
        match self.chunk.as_mut() {
            None => write_record(&mut self.writer, Op::Message, build)?,
            Some(chunk) => {
                write_record(&mut chunk.buf, Op::Message, build)?;
                chunk.start_time.get_or_insert(timestamp);
                chunk.end_time = timestamp;
                if chunk.buf.len() >= chunk.target_size {
                    self.flush_chunk()?;
                }
            }
        }
        Ok(())
    }
}

const MCAP_MAGIC: &[u8; 8] = b"\x89MCAP0\r\n";

#[derive(Clone, Copy)]
#[repr(u8)]
enum Op {
    Header = 0x01,
    Footer = 0x02,
    Schema = 0x03,
    Channel = 0x04,
    Message = 0x05,
    Chunk = 0x06,
    DataEnd = 0x0f,
}

fn write_record(
    writer: &mut impl Write,
    op: Op,
    f: impl FnOnce(&mut Vec<u8>) -> io::Result<()>,
) -> io::Result<()> {
    let mut payload = Vec::new();
    f(&mut payload)?;
    writer.write_all(&[op as u8])?;
    writer.write_all(&(payload.len() as u64).to_le_bytes())?;
    writer.write_all(&payload)
}

fn write_string(writer: &mut impl Write, s: &str) -> io::Result<()> {
    let len = u32::try_from(s.len())
        .map_err(|_| io::Error::new(io::ErrorKind::InvalidInput, "MCAP string too large"))?;
    writer.write_all(&len.to_le_bytes())?;
    writer.write_all(s.as_bytes())
}

/// Write an MCAP `Map<string, string>`: a uint32 byte-length prefix followed by
/// the concatenated key/value string records (matching [`read_string_map`]).
fn write_string_map(payload: &mut Vec<u8>, entries: &[(&str, &str)]) -> io::Result<()> {
    let mut buf = Vec::new();
    for (key, value) in entries {
        write_string(&mut buf, key)?;
        write_string(&mut buf, value)?;
    }
    let len = u32::try_from(buf.len())
        .map_err(|_| io::Error::new(io::ErrorKind::InvalidInput, "MCAP metadata too large"))?;
    payload.write_all(&len.to_le_bytes())?;
    payload.write_all(&buf)
}

/// One raw sample read from an MCAP log.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RawSample {
    pub topic: String,
    pub sequence: u32,
    pub log_time: u64,
    pub publish_time: u64,
    pub msg: RawMsg,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct McapChannel {
    pub id: u16,
    pub schema_id: u16,
    pub topic: String,
    pub message_encoding: String,
    pub metadata: HashMap<String, String>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct McapSchema {
    pub id: u16,
    pub name: String,
    pub encoding: String,
    pub data: Bytes,
}

/// Scanable MCAP reader for ROS2 CDR logs.
///
/// Supports top-level records and `Chunk` records, decompressing `zstd` and
/// `lz4` chunks. Unknown compression is reported as an error, not skipped.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct McapLog {
    pub schemas: HashMap<u16, McapSchema>,
    pub channels: HashMap<u16, McapChannel>,
    pub samples: Vec<RawSample>,
}

impl McapLog {
    pub fn read_path(path: impl AsRef<Path>) -> io::Result<Self> {
        let bytes = std::fs::read(path)?;
        Self::read(Cursor::new(bytes))
    }

    pub fn read(mut reader: impl Read) -> io::Result<Self> {
        let mut bytes = Vec::new();
        reader.read_to_end(&mut bytes)?;
        read_mcap_bytes(&bytes)
    }

    #[must_use]
    pub fn topics(&self) -> Vec<(&str, &str)> {
        let mut topics: Vec<_> = self
            .channels
            .values()
            .filter_map(|channel| {
                self.schemas
                    .get(&channel.schema_id)
                    .map(|schema| (channel.topic.as_str(), schema.name.as_str()))
            })
            .collect();
        topics.sort_unstable();
        topics.dedup();
        topics
    }
}

/// Streaming reader that yields [`RawSample`]s as it parses an MCAP log,
/// without collecting them all into memory first like [`McapLog`].
///
/// Samples are produced in file order (bags are written in log-time order);
/// unlike [`McapLog`] this reader does not re-sort by `log_time`.
pub struct RawSampleReader {
    frames: Vec<Frame>,
    schemas: HashMap<u16, McapSchema>,
    channels: HashMap<u16, McapChannel>,
}

/// One level of MCAP record stream: the top-level data section, or the
/// records nested inside a (possibly compressed) `Chunk`.
struct Frame {
    bytes: Bytes,
    pos: usize,
}

impl RawSampleReader {
    pub fn open(path: impl AsRef<Path>) -> io::Result<Self> {
        Self::from_bytes(std::fs::read(path)?)
    }

    pub fn from_bytes(bytes: impl Into<Bytes>) -> io::Result<Self> {
        let bytes = bytes.into();
        let body = bytes.slice(mcap_body_range(&bytes)?);
        Ok(Self {
            frames: vec![Frame {
                bytes: body,
                pos: 0,
            }],
            schemas: HashMap::new(),
            channels: HashMap::new(),
        })
    }

    /// The QoS recorded for `topic`, decoded from its channel metadata, if a
    /// channel record for that topic has been parsed and carries a valid
    /// [`QOS_METADATA_KEY`] entry. Channels are parsed before their messages,
    /// so this is populated by the time a topic's first sample is yielded.
    #[must_use]
    pub fn topic_qos(&self, topic: &str) -> Option<QosProfile> {
        self.channels
            .values()
            .find(|channel| channel.topic == topic)
            .and_then(|channel| channel.metadata.get(QOS_METADATA_KEY))
            .and_then(|qos| parse_qos(qos))
    }
}

impl Iterator for RawSampleReader {
    type Item = io::Result<RawSample>;

    fn next(&mut self) -> Option<Self::Item> {
        loop {
            let frame = self.frames.last()?;
            if frame.pos >= frame.bytes.len() {
                self.frames.pop();
                continue;
            }
            let pos = frame.pos;
            let bytes = frame.bytes.clone();
            let (op, start, end) = match read_record_header(&bytes, pos) {
                Ok(header) => header,
                Err(err) => {
                    self.frames.clear();
                    return Some(Err(err));
                }
            };
            self.frames.last_mut().unwrap().pos = end;
            let payload = &bytes[start..end];
            match op {
                x if x == Op::Schema as u8 => match parse_schema(payload) {
                    Ok((id, schema)) => {
                        self.schemas.insert(id, schema);
                    }
                    Err(err) => {
                        self.frames.clear();
                        return Some(Err(err));
                    }
                },
                x if x == Op::Channel as u8 => match parse_channel(payload) {
                    Ok((id, channel)) => {
                        self.channels.insert(id, channel);
                    }
                    Err(err) => {
                        self.frames.clear();
                        return Some(Err(err));
                    }
                },
                x if x == Op::Message as u8 => {
                    match parse_message(payload, &self.channels, &self.schemas) {
                        Ok(Some(sample)) => return Some(Ok(sample)),
                        Ok(None) => {}
                        Err(err) => {
                            self.frames.clear();
                            return Some(Err(err));
                        }
                    }
                }
                x if x == Op::Chunk as u8 => match parse_chunk_records(payload) {
                    Ok(records) => self.frames.push(Frame {
                        bytes: records,
                        pos: 0,
                    }),
                    Err(err) => {
                        self.frames.clear();
                        return Some(Err(err));
                    }
                },
                _ => {}
            }
        }
    }
}

/// A DDS writer that republishes raw ROS2 CDR bytes without generated types.
pub struct RawDdsPublisher {
    writer: DataWriter<RawPayload, RawSer>,
}

impl RawDdsPublisher {
    pub fn new(
        participant: &DomainParticipant,
        ros_topic: &str,
        ros_type: &str,
        qos: RawQos,
    ) -> Self {
        Self::with_policies(participant, ros_topic, ros_type, &qos.policies())
    }

    /// Create a publisher with explicit RustDDS policies, e.g. a QoS restored
    /// from bag channel metadata rather than one of the [`RawQos`] presets.
    pub fn with_policies(
        participant: &DomainParticipant,
        ros_topic: &str,
        ros_type: &str,
        policies: &rustdds::QosPolicies,
    ) -> Self {
        let topic = participant
            .create_topic(
                crate::codec::topic(ros_topic),
                ros_type_to_dds_type(ros_type),
                policies,
                TopicKind::NoKey,
            )
            .expect("create raw topic");
        let writer = participant
            .create_publisher(policies)
            .expect("create raw publisher")
            .create_datawriter_no_key::<RawPayload, RawSer>(&topic, None)
            .expect("create raw datawriter");
        Self { writer }
    }

    pub fn publish(&self, msg: &RawMsg) {
        let _ = self.writer.write(RawPayload::from_msg(msg), None);
    }

    /// Drain pending writer-side QoS status events (incompatible QoS, deadline
    /// missed, liveliness lost, publication matched). Non-blocking; returns `[]`
    /// when nothing is queued. Mirrors [`crate::transport::DdsPub::poll_events`].
    pub fn poll_events(&mut self) -> Vec<QosEvent> {
        let mut out = Vec::new();
        while let Some(status) = self.writer.try_recv_status() {
            out.push(status.into());
        }
        out
    }
}

impl TopicSampleSink for RawDdsPublisher {
    fn publish(&mut self, msg: &RawMsg) {
        Self::publish(self, msg);
    }
}

/// A DDS reader that receives raw ROS2 CDR bytes without generated types.
pub struct RawDdsSubscriber {
    reader: DataReader<RawPayload, RawDe>,
    ros_type: String,
}

impl RawDdsSubscriber {
    pub fn new(
        participant: &DomainParticipant,
        ros_topic: &str,
        ros_type: &str,
        qos: RawQos,
    ) -> Self {
        Self::with_policies(participant, ros_topic, ros_type, &qos.policies())
    }

    /// Create a subscriber with explicit RustDDS policies, e.g. one lowered from
    /// an arbitrary [`QosProfile`] rather than a [`RawQos`] preset.
    pub fn with_policies(
        participant: &DomainParticipant,
        ros_topic: &str,
        ros_type: &str,
        policies: &QosPolicies,
    ) -> Self {
        let topic = participant
            .create_topic(
                crate::codec::topic(ros_topic),
                ros_type_to_dds_type(ros_type),
                policies,
                TopicKind::NoKey,
            )
            .expect("create raw topic");
        let reader = participant
            .create_subscriber(policies)
            .expect("create raw subscriber")
            .create_datareader_no_key::<RawPayload, RawDe>(&topic, None)
            .expect("create raw datareader");
        Self {
            reader,
            ros_type: ros_type.to_string(),
        }
    }

    pub fn take(&mut self) -> Option<RawMsg> {
        self.reader
            .take_next_sample()
            .ok()
            .flatten()
            .map(|sample| sample.into_value().into_msg(&self.ros_type))
    }

    /// Drain pending reader-side QoS status events (incompatible QoS, deadline
    /// missed, liveliness changed, sample lost/rejected, subscription matched).
    /// Non-blocking; returns `[]` when nothing is queued.
    pub fn poll_events(&mut self) -> Vec<QosEvent> {
        let mut out = Vec::new();
        while let Some(status) = self.reader.try_recv_status() {
            out.push(status.into());
        }
        out
    }
}

impl TopicSampleSource for RawDdsSubscriber {
    fn take(&mut self) -> Option<RawMsg> {
        Self::take(self)
    }
}

#[derive(Clone, Copy, Debug)]
pub enum RawQos {
    Default,
    SensorData,
    Latched,
}

impl RawQos {
    /// The full [`QosProfile`] this preset corresponds to, for recording into
    /// bag channel metadata.
    #[must_use]
    pub fn profile(self) -> QosProfile {
        QosProfile::from_preset(match self {
            RawQos::Default => Qos::Default,
            RawQos::SensorData => Qos::SensorData,
            RawQos::Latched => Qos::Latched,
        })
    }

    #[must_use]
    pub fn policies(self) -> rustdds::QosPolicies {
        let b = QosPolicyBuilder::new();
        let reliable = Reliability::Reliable {
            max_blocking_time: rustdds::Duration::from_millis(100),
        };
        match self {
            RawQos::Default => b
                .reliability(reliable)
                .durability(Durability::Volatile)
                .history(History::KeepLast { depth: 10 }),
            RawQos::SensorData => b
                .reliability(Reliability::BestEffort)
                .durability(Durability::Volatile)
                .history(History::KeepLast { depth: 5 }),
            RawQos::Latched => b
                .reliability(reliable)
                .durability(Durability::TransientLocal)
                .history(History::KeepLast { depth: 1 }),
        }
        .build()
    }
}

/// Pick a QoS profile from a topic name using ROS2 sensor/latched conventions.
#[must_use]
pub fn raw_qos_for_topic(topic: &str) -> RawQos {
    if topic == "/tf_static" {
        RawQos::Latched
    } else if topic == "/tf"
        || topic.contains("camera")
        || topic.contains("image")
        || topic.contains("scan")
        || topic.contains("points")
        || topic.contains("imu")
    {
        RawQos::SensorData
    } else {
        RawQos::Default
    }
}

pub struct RawPlayback {
    pub publish_clock: bool,
    pub speed: f64,
}

impl Default for RawPlayback {
    fn default() -> Self {
        Self {
            publish_clock: true,
            speed: 1.0,
        }
    }
}

impl RawPlayback {
    pub fn sleep_until(&self, first_log_time: u64, sample_log_time: u64, wall_start: Instant) {
        if self.speed <= 0.0 {
            return;
        }
        let bag_delta = sample_log_time.saturating_sub(first_log_time);
        let secs = u32::try_from(bag_delta / 1_000_000_000).unwrap_or(u32::MAX);
        let nanos = u32::try_from(bag_delta % 1_000_000_000).unwrap_or(0);
        let bag_secs = f64::from(secs) + f64::from(nanos) / 1_000_000_000.0;
        let target = Duration::from_secs_f64(bag_secs / self.speed);
        if let Some(remaining) = target.checked_sub(wall_start.elapsed()) {
            std::thread::sleep(remaining);
        }
    }
}

#[derive(Clone, Debug)]
pub struct RawPayload {
    cdr: Bytes,
}

impl RawPayload {
    fn from_msg(msg: &RawMsg) -> Self {
        Self {
            cdr: msg.cdr.clone(),
        }
    }

    fn body(&self) -> &[u8] {
        let cdr = &self.cdr;
        if cdr.len() >= 4 && matches!(&cdr[..2], [0x00, 0x00 | 0x01]) {
            &cdr[4..]
        } else {
            cdr
        }
    }

    fn from_body(input: &[u8], encoding: RepresentationIdentifier) -> Self {
        let header: [u8; 4] = match encoding {
            RepresentationIdentifier::CDR_BE | RepresentationIdentifier::PL_CDR_BE => {
                [0x00, 0x00, 0x00, 0x00]
            }
            _ => [0x00, 0x01, 0x00, 0x00],
        };
        let mut cdr = Vec::with_capacity(input.len() + 4);
        cdr.extend_from_slice(&header);
        cdr.extend_from_slice(input);
        Self { cdr: cdr.into() }
    }

    fn into_msg(self, ros_type: &str) -> RawMsg {
        RawMsg::new(ros_type, self.cdr)
    }
}

pub struct RawSer(PhantomData<RawPayload>);

impl SerializerAdapter<RawPayload> for RawSer {
    type Error = io::Error;

    fn output_encoding() -> RepresentationIdentifier {
        RepresentationIdentifier::CDR_LE
    }

    fn to_bytes(value: &RawPayload) -> Result<Bytes, Self::Error> {
        Ok(Bytes::copy_from_slice(value.body()))
    }
}

#[derive(Clone, Copy)]
pub struct RawDec;

impl Decode<RawPayload> for RawDec {
    type Error = io::Error;

    fn decode_bytes(
        self,
        input_bytes: &[u8],
        encoding: RepresentationIdentifier,
    ) -> Result<RawPayload, Self::Error> {
        Ok(RawPayload::from_body(input_bytes, encoding))
    }
}

pub struct RawDe(PhantomData<RawPayload>);

const SUPPORTED_RAW: [RepresentationIdentifier; 2] = [
    RepresentationIdentifier::CDR_LE,
    RepresentationIdentifier::CDR_BE,
];

impl DeserializerAdapter<RawPayload> for RawDe {
    type Error = io::Error;
    type Decoded = RawPayload;

    fn supported_encodings() -> &'static [RepresentationIdentifier] {
        &SUPPORTED_RAW
    }

    fn transform_decoded(decoded: Self::Decoded) -> RawPayload {
        decoded
    }
}

impl DefaultDecoder<RawPayload> for RawDe {
    type Decoder = RawDec;
    const DECODER: RawDec = RawDec;
}

/// A runtime-typed ROS2 service **server** carrying raw CDR bytes.
///
/// The typed [`crate::service::Service`] is generic over `CdrMsg`; this one is
/// not — the request/reply DDS type names are plain strings supplied at
/// construction (e.g. from [`crate::raw::ros_type_to_dds_type`] or a
/// `DynamicType`), so a message loaded at runtime can serve without codegen.
/// Reply correlation uses the same RTPS sample-identity echo as the typed path.
pub struct RawService {
    reader: DataReader<RawPayload, RawDe>,
    writer: DataWriter<RawPayload, RawSer>,
    /// Outstanding request correlations for the split
    /// [`take_request`](RawService::take_request) /
    /// [`send_reply`](RawService::send_reply) path, keyed by an opaque token.
    pending: HashMap<u64, SampleIdentity>,
    next_token: u64,
}

impl RawService {
    /// Bind a raw server to `/<service>` on `dds`, with the given request/reply
    /// DDS type names (e.g. `example_interfaces::srv::dds_::AddTwoInts_Request_`).
    #[must_use]
    pub fn new(dds: &Dds, service: &str, request_type: &str, reply_type: &str) -> Self {
        let (rq, rr) = service_topics(service);
        let dp = dds.participant();
        let qos = dds.qos();
        let req_topic = dp
            .create_topic(rq, request_type.to_string(), qos, TopicKind::NoKey)
            .expect("request topic");
        let reply_topic = dp
            .create_topic(rr, reply_type.to_string(), qos, TopicKind::NoKey)
            .expect("reply topic");
        let reader = dp
            .create_subscriber(qos)
            .expect("subscriber")
            .create_datareader_no_key(&req_topic, None)
            .expect("request reader");
        let writer = dp
            .create_publisher(qos)
            .expect("publisher")
            .create_datawriter_no_key(&reply_topic, None)
            .expect("reply writer");
        Self {
            reader,
            writer,
            pending: HashMap::new(),
            next_token: 1,
        }
    }

    /// Serve every pending request with `handler`, which maps the request's full
    /// CDR bytes (encapsulation header + body) to the reply's full CDR bytes.
    /// Replies are correlated to their request by sample identity. Returns the
    /// number of requests served this call.
    pub fn serve_pending(&mut self, mut handler: impl FnMut(&[u8]) -> Vec<u8>) -> usize {
        let mut served = 0;
        while let Ok(Some(sample)) = self.reader.take_next_sample() {
            let id = sample.sample_info().sample_identity();
            let reply = handler(&sample.value().cdr);
            let opts = WriteOptionsBuilder::new()
                .related_sample_identity(id)
                .build();
            let _ = self
                .writer
                .write_with_options(RawPayload { cdr: reply.into() }, opts);
            served += 1;
        }
        served
    }

    /// Take the next pending request as full CDR bytes (encapsulation header +
    /// body) paired with an opaque correlation `token`. `None` when no request
    /// is queued. Pass the token back to [`RawService::send_reply`] to answer
    /// it. This is the split, non-callback counterpart to
    /// [`RawService::serve_pending`], for FFI callers that produce the reply out
    /// of band. The reply correlation is retained until `send_reply` consumes
    /// it.
    pub fn take_request(&mut self) -> Option<(Vec<u8>, u64)> {
        let sample = self.reader.take_next_sample().ok().flatten()?;
        let id = sample.sample_info().sample_identity();
        let token = self.next_token;
        self.next_token = self.next_token.wrapping_add(1);
        self.pending.insert(token, id);
        Some((sample.value().cdr.to_vec(), token))
    }

    /// Send `reply` (full CDR bytes) correlated to the request identified by
    /// `token` (from [`RawService::take_request`]). Returns `false` if the token
    /// is unknown (already answered or never issued).
    pub fn send_reply(&mut self, token: u64, reply: Vec<u8>) -> bool {
        let Some(id) = self.pending.remove(&token) else {
            return false;
        };
        let opts = WriteOptionsBuilder::new()
            .related_sample_identity(id)
            .build();
        let _ = self
            .writer
            .write_with_options(RawPayload { cdr: reply.into() }, opts);
        true
    }
}

/// A runtime-typed ROS2 service **client** carrying raw CDR bytes — the
/// counterpart to [`RawService`], mirroring [`crate::service::Client`] but over
/// raw payloads with runtime topic/type names.
pub struct RawClient {
    reader: DataReader<RawPayload, RawDe>,
    writer: DataWriter<RawPayload, RawSer>,
}

impl RawClient {
    /// Bind a raw client to `/<service>` on `dds`, with the given request/reply
    /// DDS type names.
    #[must_use]
    pub fn new(dds: &Dds, service: &str, request_type: &str, reply_type: &str) -> Self {
        let (rq, rr) = service_topics(service);
        let dp = dds.participant();
        let qos = dds.qos();
        let req_topic = dp
            .create_topic(rq, request_type.to_string(), qos, TopicKind::NoKey)
            .expect("request topic");
        let reply_topic = dp
            .create_topic(rr, reply_type.to_string(), qos, TopicKind::NoKey)
            .expect("reply topic");
        let writer = dp
            .create_publisher(qos)
            .expect("publisher")
            .create_datawriter_no_key(&req_topic, None)
            .expect("request writer");
        let reader = dp
            .create_subscriber(qos)
            .expect("subscriber")
            .create_datareader_no_key(&reply_topic, None)
            .expect("reply reader");
        Self { reader, writer }
    }

    /// Send `request` (full CDR bytes) and block up to `timeout` for the reply
    /// correlated to it, returned as full CDR bytes. `None` on timeout.
    pub fn call(&mut self, request: &[u8], timeout: Duration) -> Option<Vec<u8>> {
        let req_id: SampleIdentity = self
            .writer
            .write_with_options(
                RawPayload {
                    cdr: Bytes::copy_from_slice(request),
                },
                WriteOptionsBuilder::new().build(),
            )
            .ok()?;
        let deadline = Instant::now() + timeout;
        while Instant::now() < deadline {
            while let Ok(Some(sample)) = self.reader.take_next_sample() {
                if sample.sample_info().related_sample_identity() == Some(req_id) {
                    return Some(sample.into_value().cdr.to_vec());
                }
            }
            std::thread::sleep(Duration::from_millis(10));
        }
        None
    }
}

#[must_use]
pub fn ros_type_to_dds_type(ros_type: &str) -> String {
    let mut parts = ros_type.split('/');
    match (parts.next(), parts.next(), parts.next(), parts.next()) {
        (Some(pkg), Some("msg"), Some(name), None) => format!("{pkg}::msg::dds_::{name}_"),
        (Some(pkg), Some("srv"), Some(name), None) => format!("{pkg}::srv::dds_::{name}_"),
        (Some(pkg), Some("action"), Some(name), None) => {
            format!("{pkg}::action::dds_::{name}_")
        }
        _ => ros_type.to_string(),
    }
}

/// The byte range of the record section, between the leading MCAP magic and the
/// optional trailing magic. Errors if the leading magic is absent.
///
/// The trailing magic is only stripped when the file is at least two magics long
/// — otherwise (e.g. an input that is *just* the 8-byte magic) the leading and
/// trailing magic are the same bytes, and blindly subtracting both yielded a
/// reversed range (`8..0`) that panicked the slice. Both readers route through
/// here so neither can be crashed by a magic-only or near-empty input.
fn mcap_body_range(bytes: &[u8]) -> io::Result<std::ops::Range<usize>> {
    if !bytes.starts_with(MCAP_MAGIC) {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "missing MCAP magic",
        ));
    }
    let end = if bytes.len() >= 2 * MCAP_MAGIC.len() && bytes.ends_with(MCAP_MAGIC) {
        bytes.len() - MCAP_MAGIC.len()
    } else {
        bytes.len()
    };
    Ok(MCAP_MAGIC.len()..end)
}

fn read_mcap_bytes(bytes: &[u8]) -> io::Result<McapLog> {
    let body = mcap_body_range(bytes)?;
    let mut log = McapLog::default();
    read_records(&bytes[body], &mut log)?;
    log.samples.sort_by_key(|sample| sample.log_time);
    Ok(log)
}

fn read_records(bytes: &[u8], log: &mut McapLog) -> io::Result<()> {
    let mut pos = 0;
    while pos < bytes.len() {
        let (op, start, end) = read_record_header(bytes, pos)?;
        read_record_payload(op, &bytes[start..end], log)?;
        pos = end;
    }
    Ok(())
}

/// Read one record's `op` and payload range starting at `pos`, bounds-checked.
fn read_record_header(bytes: &[u8], pos: usize) -> io::Result<(u8, usize, usize)> {
    let mut r = Cursor::new(bytes);
    r.set_position(pos as u64);
    let op = read_u8(&mut r)?;
    let len = read_u64(&mut r)?;
    let len = usize::try_from(len)
        .map_err(|_| io::Error::new(io::ErrorKind::InvalidData, "MCAP record too large"))?;
    let start = r.position() as usize;
    let end = start
        .checked_add(len)
        .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidData, "MCAP record overflow"))?;
    if end > bytes.len() {
        return Err(io::Error::new(
            io::ErrorKind::UnexpectedEof,
            "truncated MCAP record",
        ));
    }
    Ok((op, start, end))
}

fn read_record_payload(op: u8, payload: &[u8], log: &mut McapLog) -> io::Result<()> {
    match op {
        x if x == Op::Schema as u8 => {
            let (id, schema) = parse_schema(payload)?;
            log.schemas.insert(id, schema);
        }
        x if x == Op::Channel as u8 => {
            let (id, channel) = parse_channel(payload)?;
            log.channels.insert(id, channel);
        }
        x if x == Op::Message as u8 => {
            if let Some(sample) = parse_message(payload, &log.channels, &log.schemas)? {
                log.samples.push(sample);
            }
        }
        x if x == Op::Chunk as u8 => read_records(&parse_chunk_records(payload)?, log)?,
        _ => {}
    }
    Ok(())
}

fn parse_schema(payload: &[u8]) -> io::Result<(u16, McapSchema)> {
    let mut r = Cursor::new(payload);
    let id = read_u16(&mut r)?;
    let name = read_string(&mut r)?;
    let encoding = read_string(&mut r)?;
    let data = read_bytes_u32(&mut r)?;
    Ok((
        id,
        McapSchema {
            id,
            name,
            encoding,
            data,
        },
    ))
}

fn parse_channel(payload: &[u8]) -> io::Result<(u16, McapChannel)> {
    let mut r = Cursor::new(payload);
    let id = read_u16(&mut r)?;
    let schema_id = read_u16(&mut r)?;
    let topic = read_string(&mut r)?;
    let message_encoding = read_string(&mut r)?;
    let metadata = read_string_map(&mut r)?;
    Ok((
        id,
        McapChannel {
            id,
            schema_id,
            topic,
            message_encoding,
            metadata,
        },
    ))
}

fn parse_message(
    payload: &[u8],
    channels: &HashMap<u16, McapChannel>,
    schemas: &HashMap<u16, McapSchema>,
) -> io::Result<Option<RawSample>> {
    let mut r = Cursor::new(payload);
    let channel_id = read_u16(&mut r)?;
    let sequence = read_u32(&mut r)?;
    let log_time = read_u64(&mut r)?;
    let publish_time = read_u64(&mut r)?;
    let channel = channels
        .get(&channel_id)
        .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidData, "message before channel"))?;
    if channel.message_encoding != "cdr" && channel.message_encoding != "cdr_le" {
        return Ok(None);
    }
    let ros_type = schemas
        .get(&channel.schema_id)
        .map(|schema| schema.name.clone())
        .or_else(|| channel.metadata.get("ros_type").cloned())
        .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidData, "channel without schema"))?;
    let mut cdr = Vec::new();
    r.read_to_end(&mut cdr)?;
    Ok(Some(RawSample {
        topic: channel.topic.clone(),
        sequence,
        log_time,
        publish_time,
        msg: RawMsg::new(ros_type, cdr),
    }))
}

/// Parse a `Chunk` record header and return its nested records, decompressing
/// `zstd` and `lz4` chunks. Unknown compression is a hard error, not a skip.
fn parse_chunk_records(payload: &[u8]) -> io::Result<Bytes> {
    let mut r = Cursor::new(payload);
    let _message_start_time = read_u64(&mut r)?;
    let _message_end_time = read_u64(&mut r)?;
    let uncompressed_size = read_u64(&mut r)?;
    let _uncompressed_crc = read_u32(&mut r)?;
    let compression = read_string(&mut r)?;
    // `records` is the final field of the Chunk record (uint64-length-prefixed);
    // nothing follows it per the MCAP spec.
    let records = read_bytes_u64(&mut r)?;
    // Cap the preallocation hint: `uncompressed_size` is untrusted (see
    // `MAX_DECOMPRESS_PREALLOC`).
    let hint = usize::try_from(uncompressed_size)
        .unwrap_or(usize::MAX)
        .min(MAX_DECOMPRESS_PREALLOC);
    match compression.as_str() {
        "" => Ok(records),
        "zstd" => decompress_zstd(&records, hint),
        "lz4" => decompress_lz4(&records, hint),
        other => Err(io::Error::new(
            io::ErrorKind::Unsupported,
            format!("unsupported MCAP chunk compression: {other}"),
        )),
    }
}

fn decompress_zstd(data: &[u8], hint: usize) -> io::Result<Bytes> {
    let mut decoder = ruzstd::decoding::StreamingDecoder::new(data)
        .map_err(|e| io::Error::new(io::ErrorKind::InvalidData, format!("zstd chunk: {e}")))?;
    let mut out = Vec::with_capacity(hint);
    decoder.read_to_end(&mut out)?;
    Ok(out.into())
}

fn compress_lz4(data: &[u8]) -> io::Result<Vec<u8>> {
    let mut encoder = lz4_flex::frame::FrameEncoder::new(Vec::new());
    encoder.write_all(data)?;
    encoder
        .finish()
        .map_err(|e| io::Error::other(format!("lz4 chunk: {e}")))
}

fn decompress_lz4(data: &[u8], hint: usize) -> io::Result<Bytes> {
    let mut decoder = lz4_flex::frame::FrameDecoder::new(data);
    let mut out = Vec::with_capacity(hint);
    decoder.read_to_end(&mut out)?;
    Ok(out.into())
}

fn read_u8(r: &mut Cursor<&[u8]>) -> io::Result<u8> {
    let mut buf = [0; 1];
    r.read_exact(&mut buf)?;
    Ok(buf[0])
}

fn read_u16(r: &mut Cursor<&[u8]>) -> io::Result<u16> {
    let mut buf = [0; 2];
    r.read_exact(&mut buf)?;
    Ok(u16::from_le_bytes(buf))
}

fn read_u32(r: &mut Cursor<&[u8]>) -> io::Result<u32> {
    let mut buf = [0; 4];
    r.read_exact(&mut buf)?;
    Ok(u32::from_le_bytes(buf))
}

fn read_u64(r: &mut Cursor<&[u8]>) -> io::Result<u64> {
    let mut buf = [0; 8];
    r.read_exact(&mut buf)?;
    Ok(u64::from_le_bytes(buf))
}

fn read_string(r: &mut Cursor<&[u8]>) -> io::Result<String> {
    let bytes = read_bytes_u32(r)?;
    String::from_utf8(bytes.to_vec())
        .map_err(|_| io::Error::new(io::ErrorKind::InvalidData, "invalid MCAP utf8 string"))
}

fn read_bytes_u32(r: &mut Cursor<&[u8]>) -> io::Result<Bytes> {
    let len = read_u32(r)?;
    read_bytes_len(r, len as usize)
}

fn read_bytes_u64(r: &mut Cursor<&[u8]>) -> io::Result<Bytes> {
    let len = read_u64(r)?;
    let len = usize::try_from(len)
        .map_err(|_| io::Error::new(io::ErrorKind::InvalidData, "MCAP byte field too large"))?;
    read_bytes_len(r, len)
}

fn read_bytes_len(r: &mut Cursor<&[u8]>, len: usize) -> io::Result<Bytes> {
    let pos = r.position() as usize;
    let end = pos
        .checked_add(len)
        .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidData, "MCAP byte field overflow"))?;
    if end > r.get_ref().len() {
        return Err(io::Error::new(
            io::ErrorKind::UnexpectedEof,
            "truncated MCAP byte field",
        ));
    }
    let bytes = Bytes::copy_from_slice(&r.get_ref()[pos..end]);
    r.set_position(end as u64);
    Ok(bytes)
}

/// Read an MCAP `Map<string, string>`: a uint32 byte-length prefix followed by
/// the concatenated key/value string records (matching [`write_string_map`]).
fn read_string_map(r: &mut Cursor<&[u8]>) -> io::Result<HashMap<String, String>> {
    let bytes = read_bytes_u32(r)?;
    let mut inner = Cursor::new(&bytes[..]);
    let mut map = HashMap::new();
    while (inner.position() as usize) < bytes.len() {
        let key = read_string(&mut inner)?;
        let value = read_string(&mut inner)?;
        map.insert(key, value);
    }
    Ok(map)
}

#[cfg(test)]
mod tests {
    use std::io::Write;

    use super::{
        ros_type_to_dds_type, write_record, write_string, Compression, McapLog, McapWriter, Op,
        RawMsg, RawSample, RawSampleReader, RawSink, MCAP_MAGIC,
    };

    /// Build the uncompressed nested records for a single-message chunk:
    /// one Schema (id 1), one Channel (id 1 → schema 1), one Message.
    fn inner_records(topic: &str, ros_type: &str, cdr: &[u8]) -> Vec<u8> {
        let mut recs = Vec::new();
        write_record(&mut recs, Op::Schema, |p| {
            p.write_all(&1u16.to_le_bytes())?;
            write_string(p, ros_type)?;
            write_string(p, "ros2msg")?;
            p.write_all(&0u32.to_le_bytes())
        })
        .unwrap();
        write_record(&mut recs, Op::Channel, |p| {
            p.write_all(&1u16.to_le_bytes())?;
            p.write_all(&1u16.to_le_bytes())?;
            write_string(p, topic)?;
            write_string(p, "cdr")?;
            p.write_all(&0u32.to_le_bytes())
        })
        .unwrap();
        write_record(&mut recs, Op::Message, |p| {
            p.write_all(&1u16.to_le_bytes())?; // channel_id
            p.write_all(&7u32.to_le_bytes())?; // sequence
            p.write_all(&99u64.to_le_bytes())?; // log_time
            p.write_all(&99u64.to_le_bytes())?; // publish_time
            p.write_all(cdr)
        })
        .unwrap();
        recs
    }

    /// Wrap already-compressed `records` bytes in a Chunk record and full MCAP
    /// framing (leading + trailing magic).
    fn chunk_mcap(compression: &str, records: &[u8], uncompressed_size: u64) -> Vec<u8> {
        let mut out = Vec::new();
        out.extend_from_slice(MCAP_MAGIC);
        write_record(&mut out, Op::Chunk, |p| {
            p.write_all(&0u64.to_le_bytes())?; // message_start_time
            p.write_all(&0u64.to_le_bytes())?; // message_end_time
            p.write_all(&uncompressed_size.to_le_bytes())?;
            p.write_all(&0u32.to_le_bytes())?; // uncompressed_crc (0 = absent)
            write_string(p, compression)?;
            p.write_all(&(records.len() as u64).to_le_bytes())?; // records length
            p.write_all(records)
        })
        .unwrap();
        out.extend_from_slice(MCAP_MAGIC);
        out
    }

    fn lz4_frame(data: &[u8]) -> Vec<u8> {
        let mut enc = lz4_flex::frame::FrameEncoder::new(Vec::new());
        enc.write_all(data).unwrap();
        enc.finish().unwrap()
    }

    /// A zstd frame carrying `data` in a single raw (stored) block, hand-built
    /// so the crate needs no zstd encoder (ruzstd is decode-only).
    fn zstd_raw_frame(data: &[u8]) -> Vec<u8> {
        let mut f = vec![0x28, 0xb5, 0x2f, 0xfd]; // magic
        f.push(0xa0); // descriptor: single-segment, 4-byte frame content size
        f.extend_from_slice(&(data.len() as u32).to_le_bytes());
        let header = (u32::try_from(data.len()).unwrap() << 3) | 1; // raw block, last
        f.extend_from_slice(&header.to_le_bytes()[..3]);
        f.extend_from_slice(data);
        f
    }

    #[test]
    fn raw_msg_keeps_type_and_payload() {
        let msg = RawMsg::new("std_msgs/msg/String", vec![0, 1, 2, 3]);
        assert_eq!(msg.ros_type(), "std_msgs/msg/String");
        assert_eq!(msg.cdr(), &[0, 1, 2, 3]);
    }

    #[test]
    fn mcap_writer_emits_scanable_records() {
        let mut writer = McapWriter::new(Vec::new()).unwrap();
        writer
            .write(
                "/chatter",
                42,
                &RawMsg::new("std_msgs/msg/String", vec![0, 1, 0, 0, 4, 0, 0, 0]),
            )
            .unwrap();
        let bytes = writer.finish().unwrap();
        assert!(bytes.starts_with(MCAP_MAGIC));
        assert!(bytes.ends_with(MCAP_MAGIC));
        assert!(bytes.windows(8).any(|w| w == b"/chatter"));
        assert!(bytes.windows(3).any(|w| w == b"cdr"));
        assert!(bytes.windows(7).any(|w| w == b"ros2msg"));
    }

    #[test]
    fn mcap_reader_recovers_raw_samples() {
        let mut writer = McapWriter::new(Vec::new()).unwrap();
        writer
            .write(
                "/chatter",
                42,
                &RawMsg::new("std_msgs/msg/String", vec![0, 1, 0, 0, 4, 0, 0, 0]),
            )
            .unwrap();
        let bytes = writer.finish().unwrap();

        let log = McapLog::read(std::io::Cursor::new(bytes)).unwrap();
        assert_eq!(log.topics(), vec![("/chatter", "std_msgs/msg/String")]);
        assert_eq!(log.samples.len(), 1);
        assert_eq!(log.samples[0].topic, "/chatter");
        assert_eq!(log.samples[0].log_time, 42);
        assert_eq!(log.samples[0].msg.ros_type(), "std_msgs/msg/String");
        assert_eq!(log.samples[0].msg.cdr(), &[0, 1, 0, 0, 4, 0, 0, 0]);
    }

    #[test]
    fn raw_sample_reader_streams_same_samples() {
        let mut writer = McapWriter::new(Vec::new()).unwrap();
        for i in 0..3u8 {
            writer
                .write(
                    "/chatter",
                    i64::from(i),
                    &RawMsg::new("std_msgs/msg/String", vec![0, 1, 0, 0, i, 0, 0, 0]),
                )
                .unwrap();
        }
        let bytes = writer.finish().unwrap();

        let samples: Vec<_> = RawSampleReader::from_bytes(bytes)
            .unwrap()
            .map(Result::unwrap)
            .collect();
        assert_eq!(samples.len(), 3);
        assert_eq!(samples[0].topic, "/chatter");
        assert_eq!(samples[2].msg.cdr(), &[0, 1, 0, 0, 2, 0, 0, 0]);
    }

    #[test]
    fn raw_sample_reader_rejects_bad_magic() {
        let Err(err) = RawSampleReader::from_bytes(vec![0u8; 4]) else {
            panic!("expected bad-magic error");
        };
        assert_eq!(err.kind(), std::io::ErrorKind::InvalidData);
    }

    #[test]
    fn magic_only_and_empty_bodies_do_not_panic() {
        // Regression: an input that is *just* the MCAP magic both starts and
        // ends with it, so subtracting a magic from each end produced a reversed
        // `8..0` range that panicked the slice. Both readers must treat it as an
        // empty (but valid) log. Also cover magic+magic (empty body, trailing
        // magic present) and a leading-magic-only stream.
        // Empty-body inputs: valid logs with no samples.
        for bytes in [
            MCAP_MAGIC.to_vec(),                          // just the magic
            [MCAP_MAGIC.as_slice(), MCAP_MAGIC].concat(), // magic + magic
        ] {
            let log = McapLog::read(std::io::Cursor::new(bytes.clone())).unwrap();
            assert!(log.samples.is_empty());
            let samples: Vec<_> = RawSampleReader::from_bytes(bytes)
                .unwrap()
                .collect::<Result<_, _>>()
                .unwrap();
            assert!(samples.is_empty());
        }

        // Leading magic + a truncated record: must surface as an `Err`, never a
        // panic. (Both readers are exercised for the no-panic guarantee.)
        let junk = [MCAP_MAGIC.as_slice(), &[0u8; 3]].concat();
        assert!(McapLog::read(std::io::Cursor::new(junk.clone())).is_err());
        let streamed: std::io::Result<Vec<_>> =
            RawSampleReader::from_bytes(junk).unwrap().collect();
        assert!(streamed.is_err());
    }

    #[test]
    fn chunk_with_lying_uncompressed_size_does_not_oom() {
        // A Chunk whose self-declared `uncompressed_size` is preposterous must
        // not drive an unbounded `Vec::with_capacity` (allocate-the-universe
        // DoS). The preallocation hint is capped (MAX_DECOMPRESS_PREALLOC) while
        // the small frame still decompresses correctly. Before the cap, the
        // `u64::MAX` hint aborted the process on the preallocation.
        let cdr = vec![0, 1, 0, 0, 4, 0, 0, 0];
        let inner = inner_records("/chatter", "std_msgs/msg/String", &cdr);
        let compressed = lz4_frame(&inner);
        let bytes = chunk_mcap("lz4", &compressed, u64::MAX);

        let samples: Vec<_> = RawSampleReader::from_bytes(bytes.clone())
            .unwrap()
            .map(Result::unwrap)
            .collect();
        assert_eq!(samples.len(), 1);
        assert_eq!(samples[0].topic, "/chatter");

        let log = McapLog::read(std::io::Cursor::new(bytes)).unwrap();
        assert_eq!(log.samples.len(), 1);
    }

    #[test]
    fn reads_zstd_compressed_chunk() {
        let cdr = vec![0, 1, 0, 0, 4, 0, 0, 0];
        let inner = inner_records("/chatter", "std_msgs/msg/String", &cdr);
        let bytes = chunk_mcap("zstd", &zstd_raw_frame(&inner), inner.len() as u64);

        let log = McapLog::read(std::io::Cursor::new(bytes)).unwrap();
        assert_eq!(log.topics(), vec![("/chatter", "std_msgs/msg/String")]);
        assert_eq!(log.samples.len(), 1);
        assert_eq!(log.samples[0].topic, "/chatter");
        assert_eq!(log.samples[0].sequence, 7);
        assert_eq!(log.samples[0].log_time, 99);
        assert_eq!(log.samples[0].msg.cdr(), cdr.as_slice());
    }

    #[test]
    fn reads_lz4_compressed_chunk() {
        let cdr = vec![0, 1, 0, 0, 9, 9, 9, 9];
        let inner = inner_records("/scan", "sensor_msgs/msg/LaserScan", &cdr);
        let bytes = chunk_mcap("lz4", &lz4_frame(&inner), inner.len() as u64);

        // Streaming reader exercises the same chunk-decompression path.
        let samples: Vec<_> = RawSampleReader::from_bytes(bytes)
            .unwrap()
            .map(Result::unwrap)
            .collect();
        assert_eq!(samples.len(), 1);
        assert_eq!(samples[0].topic, "/scan");
        assert_eq!(samples[0].msg.ros_type(), "sensor_msgs/msg/LaserScan");
        assert_eq!(samples[0].msg.cdr(), cdr.as_slice());
    }

    #[test]
    fn rejects_unknown_chunk_compression() {
        let inner = inner_records("/chatter", "std_msgs/msg/String", &[0, 1, 0, 0]);
        let bytes = chunk_mcap("brotli", &inner, inner.len() as u64);

        let Err(err) = McapLog::read(std::io::Cursor::new(bytes.clone())) else {
            panic!("expected unknown-compression error");
        };
        assert_eq!(err.kind(), std::io::ErrorKind::Unsupported);

        let mut reader = RawSampleReader::from_bytes(bytes).unwrap();
        assert_eq!(
            reader.next().unwrap().unwrap_err().kind(),
            std::io::ErrorKind::Unsupported
        );
    }

    /// Write `samples` through a chunked writer with the given compression and
    /// read them back with the streaming reader, returning the round-tripped
    /// samples for comparison.
    fn roundtrip_chunked(
        compression: Compression,
        samples: &[(&str, i64, &str, Vec<u8>)],
    ) -> Vec<RawSample> {
        let mut writer = McapWriter::with_compression(Vec::new(), compression).unwrap();
        for (topic, ts, ros_type, cdr) in samples {
            writer
                .write(topic, *ts, &RawMsg::new(*ros_type, cdr.clone()))
                .unwrap();
        }
        let bytes = writer.finish().unwrap();
        assert!(bytes.starts_with(MCAP_MAGIC));
        assert!(bytes.ends_with(MCAP_MAGIC));
        RawSampleReader::from_bytes(bytes)
            .unwrap()
            .map(Result::unwrap)
            .collect()
    }

    #[test]
    fn chunked_lz4_roundtrips_through_reader() {
        let samples = vec![
            (
                "/chatter",
                1,
                "std_msgs/msg/String",
                vec![0, 1, 0, 0, 1, 0, 0, 0],
            ),
            (
                "/scan",
                2,
                "sensor_msgs/msg/LaserScan",
                vec![0, 1, 0, 0, 9, 9, 9, 9],
            ),
            (
                "/chatter",
                3,
                "std_msgs/msg/String",
                vec![0, 1, 0, 0, 2, 0, 0, 0],
            ),
        ];
        let got = roundtrip_chunked(Compression::Lz4, &samples);
        assert_eq!(got.len(), 3);
        for (sample, (topic, ts, ros_type, cdr)) in got.iter().zip(&samples) {
            assert_eq!(sample.topic, *topic);
            assert_eq!(sample.log_time, *ts as u64);
            assert_eq!(sample.msg.ros_type(), *ros_type);
            assert_eq!(sample.msg.cdr(), cdr.as_slice());
        }
    }

    #[test]
    fn chunked_uncompressed_roundtrips_through_reader() {
        let samples = vec![
            ("/a", 10, "std_msgs/msg/Int32", vec![0, 1, 0, 0, 7, 0, 0, 0]),
            ("/b", 20, "std_msgs/msg/Int32", vec![0, 1, 0, 0, 8, 0, 0, 0]),
        ];
        let got = roundtrip_chunked(Compression::None, &samples);
        assert_eq!(got.len(), 2);
        assert_eq!(got[0].topic, "/a");
        assert_eq!(got[1].msg.cdr(), &[0, 1, 0, 0, 8, 0, 0, 0]);
    }

    #[test]
    fn many_messages_span_multiple_chunks() {
        // Payloads large enough that the 1 MiB threshold forces several chunks.
        let big = vec![0xabu8; 200 * 1024];
        let samples: Vec<_> = (0..16)
            .map(|i| {
                (
                    "/big",
                    i64::from(i),
                    "std_msgs/msg/ByteMultiArray",
                    big.clone(),
                )
            })
            .collect();
        let got = roundtrip_chunked(Compression::Lz4, &samples);
        assert_eq!(got.len(), 16);
        assert!(got.iter().all(|s| s.msg.cdr() == big.as_slice()));
        assert_eq!(got[15].log_time, 15);
    }

    #[test]
    fn chunked_lz4_output_is_smaller_for_compressible_data() {
        // Highly compressible payloads: the lz4 chunk file should beat the flat
        // uncompressed writer, confirming compression is actually applied.
        let cdr = vec![0u8; 4096];
        let mut flat = McapWriter::new(Vec::new()).unwrap();
        let mut lz4 = McapWriter::with_compression(Vec::new(), Compression::Lz4).unwrap();
        for i in 0..32 {
            let msg = RawMsg::new("std_msgs/msg/ByteMultiArray", cdr.clone());
            flat.write("/z", i, &msg).unwrap();
            lz4.write("/z", i, &msg).unwrap();
        }
        assert!(lz4.finish().unwrap().len() < flat.finish().unwrap().len());
    }

    #[test]
    fn ros_type_to_dds_type_matches_ros2_topic_type_names() {
        assert_eq!(
            ros_type_to_dds_type("std_msgs/msg/String"),
            "std_msgs::msg::dds_::String_"
        );
    }

    #[test]
    fn qos_string_roundtrips_and_has_known_wire_format() {
        use super::{encode_qos, parse_qos};
        use crate::qos::QosProfile;
        use crate::transport::Qos;

        for profile in [
            QosProfile::from_preset(Qos::Default),
            QosProfile::from_preset(Qos::SensorData),
            QosProfile::from_preset(Qos::Latched),
            QosProfile::from_preset(Qos::Default).with_keep_all(),
        ] {
            assert_eq!(parse_qos(&encode_qos(&profile)), Some(profile));
        }
        assert_eq!(
            encode_qos(&QosProfile::from_preset(Qos::Latched)),
            "reliability=reliable,durability=transient_local,depth=1"
        );
        assert_eq!(
            encode_qos(&QosProfile::from_preset(Qos::Default).with_keep_all()),
            "reliability=reliable,durability=volatile,depth=10,history=keep_all"
        );
        // Unknown keys are ignored; a malformed value is rejected.
        assert_eq!(parse_qos("depth=3,foo=bar").map(|p| p.depth), Some(3));
        assert!(parse_qos("reliability=bogus").is_none());
    }

    #[test]
    fn channel_qos_metadata_roundtrips_flat_and_chunked() {
        use super::{parse_qos, QOS_METADATA_KEY};
        use crate::qos::QosProfile;
        use crate::transport::Qos;

        let profile = QosProfile::from_preset(Qos::Latched);
        // None = flat writer; Some = chunked writer with that compression.
        for compression in [None, Some(Compression::None), Some(Compression::Lz4)] {
            let mut writer = match compression {
                None => McapWriter::new(Vec::new()).unwrap(),
                Some(c) => McapWriter::with_compression(Vec::new(), c).unwrap(),
            };
            writer.set_channel_qos("/tf_static", &profile);
            writer
                .write(
                    "/tf_static",
                    1,
                    &RawMsg::new("tf2_msgs/msg/TFMessage", vec![0, 1, 0, 0]),
                )
                .unwrap();
            let bytes = writer.finish().unwrap();

            let log = McapLog::read(std::io::Cursor::new(bytes)).unwrap();
            let channel = log
                .channels
                .values()
                .find(|c| c.topic == "/tf_static")
                .expect("channel present");
            let stored = channel
                .metadata
                .get(QOS_METADATA_KEY)
                .expect("qos metadata present");
            assert_eq!(parse_qos(stored), Some(profile));
        }
    }

    #[test]
    fn reader_topic_qos_selects_restored_profile() {
        use crate::qos::QosProfile;
        use crate::transport::Qos;

        let profile = QosProfile::from_preset(Qos::SensorData);
        let mut writer = McapWriter::with_compression(Vec::new(), Compression::Lz4).unwrap();
        writer.set_channel_qos("/scan", &profile);
        writer
            .write(
                "/scan",
                1,
                &RawMsg::new("sensor_msgs/msg/LaserScan", vec![0, 1, 0, 0]),
            )
            .unwrap();
        // A second topic with no recorded QoS exercises the fallback path.
        writer
            .write(
                "/plain",
                2,
                &RawMsg::new("std_msgs/msg/String", vec![0, 1, 0, 0]),
            )
            .unwrap();
        let bytes = writer.finish().unwrap();

        let mut reader = RawSampleReader::from_bytes(bytes).unwrap();
        // Drain samples so every channel record is parsed (this is the same
        // reader bag_play consumes; QoS is queried per topic on first sample).
        let samples: Vec<_> = reader.by_ref().map(Result::unwrap).collect();
        assert_eq!(samples.len(), 2);
        assert_eq!(reader.topic_qos("/scan"), Some(profile));
        assert_eq!(reader.topic_qos("/plain"), None);
        assert_eq!(reader.topic_qos("/missing"), None);
    }
}

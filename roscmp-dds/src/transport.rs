//! Transport abstraction over a pub/sub middleware.
//!
//! Extracted from the concrete RustDDS usage so a second backend (e.g. a
//! from-scratch RTPS) can slot in behind the same interface. The DDS backend
//! ([`Dds`]) plugs our CDR codec in via the [`crate::codec`] adapters.

use rustdds::{
    no_key::{DataReader, DataWriter},
    DomainParticipant, DomainParticipantBuilder, QosPolicies, RTPSEntity, StatusEvented, TopicKind,
};

use crate::codec::{topic, CdrMsg, De, Ser};
use crate::qos::{QosEvent, QosProfile};

/// QoS preset matching the common ROS2 profiles. Pick one per endpoint so we
/// connect to publishers/subscribers that aren't on the default profile.
#[derive(Clone, Copy, Debug)]
pub enum Qos {
    /// Reliable · volatile · keep-last(10) — the ROS2 default for most topics.
    Default,
    /// Best-effort · volatile · keep-last(5) — the ROS2 `sensor_data` profile.
    SensorData,
    /// Reliable · transient-local · keep-last(1) — latched (e.g. `/tf_static`).
    Latched,
}

impl Qos {
    /// Concrete RustDDS policies for this preset.
    #[must_use]
    pub fn policies(self) -> QosPolicies {
        QosProfile::from_preset(self).policies()
    }
}

/// Publishes messages of type `M` on one topic.
pub trait MsgPublisher<M> {
    fn publish(&self, msg: M);
}

/// Receives messages of type `M` from one topic (non-blocking).
pub trait MsgSubscriber<M> {
    /// Take the next unread message, if any.
    fn take(&mut self) -> Option<M>;
}

/// A pub/sub transport. Backends create typed publishers/subscribers from a
/// ROS topic name; ROS naming + serialization are the backend's concern.
pub trait Transport {
    type Pub<M: CdrMsg>: MsgPublisher<M>;
    type Sub<M: CdrMsg>: MsgSubscriber<M>;

    fn publisher<M: CdrMsg>(&self, ros_topic: &str, qos: Qos) -> Self::Pub<M>;
    fn subscriber<M: CdrMsg>(&self, ros_topic: &str, qos: Qos) -> Self::Sub<M>;
}

/// RustDDS-backed transport using our CDR codec and ROS2 naming/QoS.
pub struct Dds {
    participant: DomainParticipant,
    qos: QosPolicies,
}

impl Dds {
    /// Create a participant on `domain`. The stored QoS is the ROS2 default
    /// profile (reliable/volatile), used for services; per-topic pub/sub QoS is
    /// chosen at endpoint creation via [`Qos`].
    pub fn new(domain: u16) -> Self {
        // 16 MiB SO_RCVBUF so multi-MB fragment bursts (pointclouds, images)
        // survive; kernel clamps to net.core.rmem_max on Linux — raise it there.
        let participant = DomainParticipantBuilder::new(domain)
            .socket_receive_buffer_size(16 * 1024 * 1024)
            .build()
            .expect("create DomainParticipant");
        Dds {
            participant,
            qos: Qos::Default.policies(),
        }
    }

    /// Underlying participant — for backend-specific needs (e.g. services).
    pub fn participant(&self) -> &DomainParticipant {
        &self.participant
    }

    /// The default (service) QoS profile.
    pub fn qos(&self) -> &QosPolicies {
        &self.qos
    }
}

pub struct DdsPub<M: CdrMsg> {
    writer: DataWriter<M, Ser<M>>,
}
impl<M: CdrMsg> MsgPublisher<M> for DdsPub<M> {
    fn publish(&self, msg: M) {
        let _ = self.writer.write(msg, None);
    }
}
impl<M: CdrMsg> DdsPub<M> {
    /// The writer's 16-byte ROS GID (its DDS GUID: 12-byte prefix + 4-byte
    /// entity id), for advertising this endpoint on `ros_discovery_info`.
    #[must_use]
    pub fn gid(&self) -> [u8; 16] {
        self.writer.guid().to_bytes()
    }

    /// Drain pending QoS status events (deadline missed, incompatible QoS,
    /// liveliness lost, publication matched). Non-blocking; returns `[]` when
    /// nothing is queued.
    pub fn poll_events(&mut self) -> Vec<QosEvent> {
        let mut out = Vec::new();
        while let Some(status) = self.writer.try_recv_status() {
            out.push(status.into());
        }
        out
    }
}

pub struct DdsSub<M: CdrMsg> {
    reader: DataReader<M, De<M>>,
}
impl<M: CdrMsg> MsgSubscriber<M> for DdsSub<M> {
    fn take(&mut self) -> Option<M> {
        self.reader
            .take_next_sample()
            .ok()
            .flatten()
            .map(rustdds::no_key::DataSample::into_value)
    }
}
impl<M: CdrMsg> DdsSub<M> {
    /// The reader's 16-byte ROS GID (its DDS GUID: 12-byte prefix + 4-byte
    /// entity id), for advertising this endpoint on `ros_discovery_info`.
    #[must_use]
    pub fn gid(&self) -> [u8; 16] {
        self.reader.guid().to_bytes()
    }

    /// Drain pending QoS status events (deadline missed, incompatible QoS,
    /// liveliness changed, sample lost/rejected, subscription matched).
    /// Non-blocking; returns `[]` when nothing is queued.
    pub fn poll_events(&mut self) -> Vec<QosEvent> {
        let mut out = Vec::new();
        while let Some(status) = self.reader.try_recv_status() {
            out.push(status.into());
        }
        out
    }

    /// The reader's data-availability event source, for registering with an
    /// executor's mio waitset so it can block until a sample is ready.
    pub fn event_source(&mut self) -> &mut dyn mio::event::Source {
        &mut self.reader
    }
}

impl Transport for Dds {
    type Pub<M: CdrMsg> = DdsPub<M>;
    type Sub<M: CdrMsg> = DdsSub<M>;

    fn publisher<M: CdrMsg>(&self, ros_topic: &str, qos: Qos) -> DdsPub<M> {
        let q = qos.policies();
        let t = self
            .participant
            .create_topic(
                topic(ros_topic),
                M::TYPE_NAME.to_string(),
                &q,
                TopicKind::NoKey,
            )
            .expect("create topic");
        let writer = self
            .participant
            .create_publisher(&q)
            .expect("create publisher")
            .create_datawriter_no_key(&t, None)
            .expect("create datawriter");
        DdsPub { writer }
    }

    fn subscriber<M: CdrMsg>(&self, ros_topic: &str, qos: Qos) -> DdsSub<M> {
        let q = qos.policies();
        let t = self
            .participant
            .create_topic(
                topic(ros_topic),
                M::TYPE_NAME.to_string(),
                &q,
                TopicKind::NoKey,
            )
            .expect("create topic");
        let reader = self
            .participant
            .create_subscriber(&q)
            .expect("create subscriber")
            .create_datareader_no_key(&t, None)
            .expect("create datareader");
        DdsSub { reader }
    }
}

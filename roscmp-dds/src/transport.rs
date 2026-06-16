//! Transport abstraction over a pub/sub middleware.
//!
//! Extracted from the concrete RustDDS usage so a second backend (e.g. a
//! from-scratch RTPS) can slot in behind the same interface. The DDS backend
//! ([`Dds`]) plugs our CDR codec in via the [`crate::codec`] adapters.

use rustdds::{
    no_key::{DataReader, DataWriter},
    policy::Reliability,
    DomainParticipant, DomainParticipantBuilder, QosPolicies, QosPolicyBuilder, TopicKind,
};

use crate::codec::{topic, CdrMsg, De, Ser};

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

    fn publisher<M: CdrMsg>(&self, ros_topic: &str) -> Self::Pub<M>;
    fn subscriber<M: CdrMsg>(&self, ros_topic: &str) -> Self::Sub<M>;
}

/// RustDDS-backed transport using our CDR codec and ROS2 naming/QoS.
pub struct Dds {
    participant: DomainParticipant,
    qos: QosPolicies,
}

impl Dds {
    /// Create a participant on `domain` with reliable/volatile QoS (matches the
    /// ROS2 defaults used by `ros2 topic`/services).
    pub fn new(domain: u16) -> Self {
        let participant = DomainParticipantBuilder::new(domain)
            .build()
            .expect("create DomainParticipant");
        let qos = QosPolicyBuilder::new()
            .reliability(Reliability::Reliable {
                max_blocking_time: rustdds::Duration::from_millis(100),
            })
            .build();
        Dds { participant, qos }
    }

    /// Underlying participant — for backend-specific needs (e.g. services).
    pub fn participant(&self) -> &DomainParticipant {
        &self.participant
    }

    /// The shared QoS profile.
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

impl Transport for Dds {
    type Pub<M: CdrMsg> = DdsPub<M>;
    type Sub<M: CdrMsg> = DdsSub<M>;

    fn publisher<M: CdrMsg>(&self, ros_topic: &str) -> DdsPub<M> {
        let t = self
            .participant
            .create_topic(
                topic(ros_topic),
                M::TYPE_NAME.to_string(),
                &self.qos,
                TopicKind::NoKey,
            )
            .expect("create topic");
        let writer = self
            .participant
            .create_publisher(&self.qos)
            .expect("create publisher")
            .create_datawriter_no_key(&t, None)
            .expect("create datawriter");
        DdsPub { writer }
    }

    fn subscriber<M: CdrMsg>(&self, ros_topic: &str) -> DdsSub<M> {
        let t = self
            .participant
            .create_topic(
                topic(ros_topic),
                M::TYPE_NAME.to_string(),
                &self.qos,
                TopicKind::NoKey,
            )
            .expect("create topic");
        let reader = self
            .participant
            .create_subscriber(&self.qos)
            .expect("create subscriber")
            .create_datareader_no_key(&t, None)
            .expect("create datareader");
        DdsSub { reader }
    }
}

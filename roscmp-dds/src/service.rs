//! Reusable ROS2 service client/server over RTPS.
//!
//! ROS2 maps a service to two topics (`rq/...Request`, `rr/...Reply`) and
//! correlates a reply to its request via the RTPS **sample identity**: the
//! server echoes the request's identity as the reply's `related_sample_identity`,
//! and the client matches replies against the identity it received when it wrote.
//! This module wraps that plumbing so a service is a few lines, not bespoke code.

use std::time::{Duration, Instant};

use rustdds::{
    no_key::{DataReader, DataWriter},
    rpc::SampleIdentity,
    TopicKind, WriteOptionsBuilder,
};

use crate::codec::{service_topics, CdrMsg, De, Ser};
use crate::transport::Dds;

/// Service server: receives `Req`, replies `Resp`.
pub struct Service<Req: CdrMsg, Resp: CdrMsg> {
    reader: DataReader<Req, De<Req>>,
    writer: DataWriter<Resp, Ser<Resp>>,
}

impl<Req: CdrMsg, Resp: CdrMsg> Service<Req, Resp> {
    /// Bind a server to `/<service>` on `dds`.
    #[must_use]
    pub fn new(dds: &Dds, service: &str) -> Self {
        let (rq, rr) = service_topics(service);
        let dp = dds.participant();
        let qos = dds.qos();
        let req_topic = dp
            .create_topic(rq, Req::TYPE_NAME.to_string(), qos, TopicKind::NoKey)
            .expect("request topic");
        let reply_topic = dp
            .create_topic(rr, Resp::TYPE_NAME.to_string(), qos, TopicKind::NoKey)
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
        Service { reader, writer }
    }

    /// Serve every pending request with `handler`, replying correlated by sample
    /// identity. Returns the number of requests served this call.
    pub fn serve_pending(&mut self, mut handler: impl FnMut(&Req) -> Resp) -> usize {
        let mut served = 0;
        while let Ok(Some(sample)) = self.reader.take_next_sample() {
            let id = sample.sample_info().sample_identity();
            let resp = handler(sample.value());
            let opts = WriteOptionsBuilder::new()
                .related_sample_identity(id)
                .build();
            let _ = self.writer.write_with_options(resp, opts);
            served += 1;
        }
        served
    }

    /// The request reader's event source, for registering with an executor's
    /// mio waitset so it can block until a request is ready.
    pub fn event_source(&mut self) -> &mut dyn mio::event::Source {
        &mut self.reader
    }
}

/// Service client: sends `Req`, awaits the correlated `Resp`.
pub struct Client<Req: CdrMsg, Resp: CdrMsg> {
    reader: DataReader<Resp, De<Resp>>,
    writer: DataWriter<Req, Ser<Req>>,
}

impl<Req: CdrMsg, Resp: CdrMsg> Client<Req, Resp> {
    /// Bind a client to `/<service>` on `dds`.
    #[must_use]
    pub fn new(dds: &Dds, service: &str) -> Self {
        let (rq, rr) = service_topics(service);
        let dp = dds.participant();
        let qos = dds.qos();
        let req_topic = dp
            .create_topic(rq, Req::TYPE_NAME.to_string(), qos, TopicKind::NoKey)
            .expect("request topic");
        let reply_topic = dp
            .create_topic(rr, Resp::TYPE_NAME.to_string(), qos, TopicKind::NoKey)
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
        Client { reader, writer }
    }

    /// Send `req` and block up to `timeout` for the reply correlated to it.
    /// Returns `None` on timeout.
    pub fn call(&mut self, req: Req, timeout: Duration) -> Option<Resp> {
        let req_id: SampleIdentity = self
            .writer
            .write_with_options(req, WriteOptionsBuilder::new().build())
            .ok()?;
        let deadline = Instant::now() + timeout;
        while Instant::now() < deadline {
            while let Ok(Some(sample)) = self.reader.take_next_sample() {
                if sample.sample_info().related_sample_identity() == Some(req_id) {
                    return Some(sample.into_value());
                }
            }
            std::thread::sleep(Duration::from_millis(10));
        }
        None
    }
}

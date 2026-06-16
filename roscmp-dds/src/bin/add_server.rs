//! `example_interfaces/AddTwoInts` service server over RTPS.
//! Interop: `ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 3, b: 4}"`.
//!
//! ROS2 maps a service to two topics (`rq/...Request`, `rr/...Reply`) and
//! correlates a reply to its request via the RTPS **sample identity**: read the
//! request's identity from `SampleInfo`, echo it back as the reply's
//! `related_sample_identity`. RustDDS exposes both, so a vanilla client matches.

use std::time::Duration;

use roscmp_dds::codec::{service_topics, CdrMsg, De, Ser};
use roscmp_dds::msgs::{
    example_interfaces__AddTwoInts_Request as Req, example_interfaces__AddTwoInts_Response as Resp,
};
use roscmp_dds::transport::Dds;
use rustdds::{TopicKind, WriteOptionsBuilder};

fn main() {
    // Services need lower-level RTPS (sample-identity correlation), so we use the
    // transport's participant + QoS directly rather than the pub/sub trait.
    let dds = Dds::new(0);
    let dp = dds.participant();
    let qos = dds.qos();

    let (rq, rr) = service_topics("/add_two_ints");
    let req_topic = dp
        .create_topic(rq, Req::TYPE_NAME.to_string(), qos, TopicKind::NoKey)
        .expect("request topic");
    let reply_topic = dp
        .create_topic(rr, Resp::TYPE_NAME.to_string(), qos, TopicKind::NoKey)
        .expect("reply topic");

    let subscriber = dp.create_subscriber(qos).expect("subscriber");
    let publisher = dp.create_publisher(qos).expect("publisher");
    let mut req_reader = subscriber
        .create_datareader_no_key::<Req, De<Req>>(&req_topic, None)
        .expect("request reader");
    let reply_writer = publisher
        .create_datawriter_no_key::<Resp, Ser<Resp>>(&reply_topic, None)
        .expect("reply writer");

    println!("add_server: serving /add_two_ints (example_interfaces/srv/AddTwoInts)");
    let mut served = 0;
    for _ in 0..1200 {
        while let Ok(Some(sample)) = req_reader.take_next_sample() {
            // Correlate the reply to this request via the RTPS sample identity.
            let request_id = sample.sample_info().sample_identity();
            let req = sample.value();
            let sum = req.a + req.b;
            println!("request: a={} b={} -> sum={}", req.a, req.b, sum);

            let resp = Resp { sum };
            let opts = WriteOptionsBuilder::new()
                .related_sample_identity(request_id)
                .build();
            if reply_writer.write_with_options(resp, opts).is_err() {
                eprintln!("reply write failed");
            }
            served += 1;
        }
        if served >= 1 {
            // Serve a little longer in case the client retries, then exit.
            std::thread::sleep(Duration::from_millis(500));
            break;
        }
        std::thread::sleep(Duration::from_millis(50));
    }

    if served == 0 {
        eprintln!("add_server: no requests received");
        std::process::exit(1);
    }
    println!("add_server: served {served} request(s)");
}

//! Executor integration tests: callback subscriptions, timers, and services
//! serviced from the single-threaded spin loop over a domain-0 loopback.

use std::rc::Rc;
use std::sync::mpsc;
use std::time::{Duration, Instant};

use roscmp_dds::msgs::{
    example_interfaces__AddTwoInts_Request as Req, example_interfaces__AddTwoInts_Response as Resp,
};
use roscmp_dds::node::Node;
use roscmp_dds::service::Client;
use roscmp_dds::transport::{Dds, MsgPublisher, Qos};

/// A publisher and a callback subscription on one node receive the node's own
/// message when drained by `spin_once`.
#[test]
fn callback_subscription_receives_own_message() {
    let mut node = Node::new("sub_exec_test");
    let publisher = node.publisher::<Resp>("/node_exec_sub", Qos::Default);

    let received = Rc::new(std::cell::Cell::new(None));
    let sink = Rc::clone(&received);
    node.subscribe::<Resp>("/node_exec_sub", Qos::Default, move |msg| {
        sink.set(Some(msg.sum));
    });

    let deadline = Instant::now() + Duration::from_secs(15);
    while Instant::now() < deadline && received.get().is_none() {
        publisher.publish(Resp { sum: 42 });
        node.spin_once(Duration::from_millis(50));
    }
    assert_eq!(received.get(), Some(42), "callback never fired");
}

/// A timer created on the node fires when spun.
#[test]
fn timer_fires_under_spin() {
    let mut node = Node::new("timer_exec_test");
    let ticks = Rc::new(std::cell::Cell::new(0u32));
    let counter = Rc::clone(&ticks);
    node.create_timer(Duration::from_millis(10), move |_dds| {
        counter.set(counter.get() + 1);
    });
    node.spin_for(Duration::from_millis(200));
    assert!(ticks.get() >= 1, "timer never fired: {}", ticks.get());
}

/// A blocking `spin_once` wakes promptly when a message arrives — proving the
/// waitset actually blocks on reader readiness instead of busy-polling. The
/// message is published ~100ms into a 10s-timeout spin; the call must return
/// shortly after (well before the timeout) and only once the message exists.
#[test]
fn spin_once_wakes_promptly_on_message() {
    let mut node = Node::new("spin_wake_test");
    let publisher = node.publisher::<Resp>("/spin_wake", Qos::Default);

    let count = Rc::new(std::cell::Cell::new(0u32));
    let sink = Rc::clone(&count);
    node.subscribe::<Resp>("/spin_wake", Qos::Default, move |_| {
        sink.set(sink.get() + 1);
    });

    // Warm up so the intra-participant reader/writer are matched.
    let warmup = Instant::now() + Duration::from_secs(10);
    while Instant::now() < warmup && count.get() == 0 {
        publisher.publish(Resp { sum: 1 });
        node.spin_once(Duration::from_millis(50));
    }
    assert!(count.get() >= 1, "warmup: reader never matched");
    count.set(0);

    // Publish from a helper thread 100ms after we begin blocking.
    let handle = std::thread::spawn(move || {
        std::thread::sleep(Duration::from_millis(100));
        publisher.publish(Resp { sum: 2 });
    });

    let start = Instant::now();
    let handled = node.spin_once(Duration::from_secs(10));
    let elapsed = start.elapsed();
    handle.join().expect("publisher thread panicked");

    assert_eq!(handled, 1, "spin_once handled {handled} items, expected 1");
    assert!(
        elapsed >= Duration::from_millis(50),
        "spin_once returned in {elapsed:?}, before the message was published",
    );
    assert!(
        elapsed < Duration::from_secs(2),
        "spin_once blocked {elapsed:?}, far past the message — not waking on readiness",
    );
}

/// A service registered on the node is serviced from its spin loop; a remote
/// client (its own participant, on a helper thread) gets the correlated reply.
#[test]
fn service_serviced_from_spin_loop() {
    let (tx, rx) = mpsc::channel();
    let client = std::thread::spawn(move || {
        let dds = Dds::new(0);
        let mut client = Client::<Req, Resp>::new(&dds, "/node_exec_add");
        // Give discovery time to match the node's service, then retry the call
        // a few times: a volatile writer drops a request sent before matching.
        std::thread::sleep(Duration::from_secs(2));
        let mut sum = None;
        for _ in 0..5 {
            if let Some(resp) = client.call(Req { a: 3, b: 4 }, Duration::from_secs(2)) {
                sum = Some(resp.sum);
                break;
            }
        }
        let _ = tx.send(sum);
    });

    let mut node = Node::new("svc_exec_test");
    node.create_service::<Req, Resp>("/node_exec_add", |req| Resp { sum: req.a + req.b });

    let deadline = Instant::now() + Duration::from_secs(20);
    let mut got = None;
    while Instant::now() < deadline {
        node.spin_once(Duration::from_millis(20));
        if let Ok(sum) = rx.try_recv() {
            got = Some(sum);
            break;
        }
    }
    client.join().expect("client thread panicked");
    assert_eq!(got, Some(Some(7)), "service reply not received");
}

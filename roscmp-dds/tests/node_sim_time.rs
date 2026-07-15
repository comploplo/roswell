//! Sim-time integration: a node with `use_sim_time:=true` drives its timers off
//! `/clock` received from a loopback publisher, not the wall clock.

use std::rc::Rc;
use std::time::{Duration, Instant};

use roscmp_dds::node::Node;
use roscmp_dds::time::{ClockMsg, Time};
use roscmp_dds::transport::{Dds, MsgPublisher, Qos, Transport};

/// With sim time on, a 1s-period timer must not fire until `/clock` crosses the
/// 1s mark, and `node.now()` inside the callback reflects the sim instant — not
/// wall time (which never advances a full second during this test).
#[test]
fn sim_timer_fires_on_clock_not_wall() {
    let args = ["--ros-args", "-p", "use_sim_time:=true"].map(String::from);
    let mut node = Node::from_args("sim_time_test", &args);
    assert!(node.use_sim_time(), "node did not enter sim time");

    // Independent publisher for /clock (best-effort, matching the subscriber).
    let clock_dds = Dds::new(0);
    let clock_pub = clock_dds.publisher::<ClockMsg>("/clock", Qos::SensorData);

    // A 1s sim-period timer; capture the sim time observed when it fires.
    let fired_at = Rc::new(std::cell::Cell::new(None::<i64>));
    let sink = Rc::clone(&fired_at);
    node.create_timer(Duration::from_secs(1), move |_dds| {
        // `now()` is not reachable from here (no &Node); the outer loop records
        // sim time instead — see the assertion below. This just marks a fire.
        sink.set(Some(sink.get().unwrap_or(0) + 1));
    });

    // Phase 1: publish clock samples below 1s — the timer must stay silent.
    let deadline = Instant::now() + Duration::from_secs(10);
    while Instant::now() < deadline {
        clock_pub.publish(ClockMsg {
            clock: Time::from_millis(500).to_msg(),
        });
        node.spin_once(Duration::from_millis(20));
        if node.now() == Time::from_millis(500) {
            break;
        }
    }
    assert_eq!(
        node.now(),
        Time::from_millis(500),
        "sim clock never took hold"
    );
    assert!(
        fired_at.get().is_none(),
        "timer fired before sim clock reached its period"
    );

    // Phase 2: advance clock past 1s — the timer must now fire.
    let deadline = Instant::now() + Duration::from_secs(10);
    while Instant::now() < deadline && fired_at.get().is_none() {
        clock_pub.publish(ClockMsg {
            clock: Time::from_millis(1_500).to_msg(),
        });
        node.spin_once(Duration::from_millis(20));
    }
    assert!(
        fired_at.get().is_some(),
        "sim timer never fired after clock crossed its period"
    );
    assert_eq!(
        node.now(),
        Time::from_millis(1_500),
        "node.now() should track the latest sim clock sample"
    );
}

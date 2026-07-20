//! `example_interfaces/AddTwoInts` service server over RTPS, driven by the
//! node executor.
//! Interop: `ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 3, b: 4}"`.
//!
//! All the RTPS plumbing (request/reply topics, sample-identity correlation)
//! lives in [`roswell_ros2_compat::service::Service`]; the [`Node`] executor services it
//! from its spin loop.

use std::cell::Cell;
use std::rc::Rc;
use std::time::Duration;

use roswell_ros2_compat::msgs::{
    example_interfaces__AddTwoInts_Request as Req, example_interfaces__AddTwoInts_Response as Resp,
};
use roswell_ros2_compat::node::Node;

fn main() {
    let mut node = Node::new("add_server");

    let served = Rc::new(Cell::new(0u32));
    let counter = Rc::clone(&served);
    node.create_service::<Req, Resp>("/add_two_ints", move |req| {
        let sum = req.a + req.b;
        println!("request: a={} b={} -> sum={}", req.a, req.b, sum);
        counter.set(counter.get() + 1);
        Resp { sum }
    });

    println!("add_server: serving /add_two_ints (example_interfaces/srv/AddTwoInts)");
    for _ in 0..1200 {
        node.spin_once(Duration::from_millis(50));
        if served.get() >= 1 {
            // Serve a little longer in case the client retries, then exit.
            node.spin_for(Duration::from_millis(500));
            break;
        }
    }

    if served.get() == 0 {
        eprintln!("add_server: no requests received");
        std::process::exit(1);
    }
    println!("add_server: served {} request(s)", served.get());
}

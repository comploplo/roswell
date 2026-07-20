//! `example_interfaces/AddTwoInts` service *client* over RTPS.
//! Interop: run a vanilla server (`ros2 run demo_nodes_cpp add_two_ints_server`)
//! then this client calls it and prints the sum.

use std::time::Duration;

use roswell_ros2_compat::msgs::{
    example_interfaces__AddTwoInts_Request as Req, example_interfaces__AddTwoInts_Response as Resp,
};
use roswell_ros2_compat::service::Client;
use roswell_ros2_compat::transport::Dds;

fn main() {
    let dds = Dds::new(0);
    let mut client = Client::<Req, Resp>::new(&dds, "/add_two_ints");

    // Give discovery time to match the remote service before the first call.
    std::thread::sleep(Duration::from_secs(3));

    let (a, b) = (5, 6);
    println!("add_client: calling /add_two_ints with a={a} b={b}");
    if let Some(resp) = client.call(Req { a, b }, Duration::from_secs(8)) {
        println!("add_client: sum={}", resp.sum);
    } else {
        eprintln!("add_client: no reply (timeout)");
        std::process::exit(1);
    }
}

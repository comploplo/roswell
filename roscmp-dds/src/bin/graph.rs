//! Read-only graph introspection over visible DDS topics.
//!
//!   graph                        # list once after discovery settles
//!   graph --advertise-node NAME  # publish NAME on ros_discovery_info, then idle
//!
//! Note: RustDDS exposes discovered *topics* (and participant status events) but
//! not matched-endpoint queries, so this lists topics/types, not per-node wiring.

use std::time::Duration;

use roscmp_dds::discovery::DiscoveryInfo;
use roscmp_dds::graph::{ActionChannel, Graph};
use roscmp_dds::transport::Dds;

fn main() {
    let args: Vec<String> = std::env::args().collect();
    if let Some(pos) = args.iter().position(|a| a == "--advertise-node") {
        let name = args.get(pos + 1).map_or("roscmp_node", String::as_str);
        advertise_node(name);
        return;
    }

    let dds = Dds::new(0);

    // Let SPDP/SEDP discovery settle before snapshotting the graph.
    std::thread::sleep(Duration::from_secs(3));

    let graph = Graph::discover(&dds);

    println!("discovered {} topic(s):", graph.topics.len());
    for topic in &graph.topics {
        println!("  {}  [{}]", topic.name, topic.ros_type);
    }
    println!("discovered {} service(s):", graph.services.len());
    for service in &graph.services {
        println!(
            "  {}  request={} reply={}",
            service.name, service.has_request, service.has_reply
        );
    }
    println!("discovered {} action(s):", graph.actions.len());
    for action in &graph.actions {
        println!(
            "  {}  send_goal={} get_result={} cancel_goal={} feedback={} status={}",
            action.name,
            action.channels.contains(&ActionChannel::SendGoal),
            action.channels.contains(&ActionChannel::GetResult),
            action.channels.contains(&ActionChannel::CancelGoal),
            action.channels.contains(&ActionChannel::Feedback),
            action.channels.contains(&ActionChannel::Status)
        );
    }
}

/// Announce `name` on `ros_discovery_info` (latched) so it shows in
/// `ros2 node list`, then idle to keep the participant alive.
fn advertise_node(name: &str) {
    let dds = Dds::new(0);
    let mut discovery = DiscoveryInfo::new(&dds);
    discovery.add_node("/", name);
    println!("advertising node /{name} on ros_discovery_info");
    std::thread::sleep(Duration::from_secs(30));
}

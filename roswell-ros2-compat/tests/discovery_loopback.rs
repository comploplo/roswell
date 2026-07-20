use std::time::Duration;

use roswell_ros2_compat::discovery::{DiscoveryInfo, NodeName};
use roswell_ros2_compat::graph::Graph;
use roswell_ros2_compat::node::Node;
use roswell_ros2_compat::transport::Dds;

#[test]
fn advertised_node_shows_up_via_discovery() {
    let publisher_dds = Dds::new(0);
    let mut discovery = DiscoveryInfo::new(&publisher_dds);
    discovery.add_node("/", "loopback_talker");

    let consumer_dds = Dds::new(0);
    let graph = Graph::discover_with_nodes(&consumer_dds, Duration::from_secs(8));

    let expected = NodeName {
        namespace: "/".into(),
        name: "loopback_talker".into(),
    };
    assert!(
        graph.nodes.contains(&expected),
        "advertised node missing from discovery; nodes={:?}",
        graph.nodes
    );
    assert_eq!(expected.full_name(), "/loopback_talker");
}

#[test]
fn node_self_advertises_on_construction() {
    let _node = Node::with_namespace("self_advertised", "demo", 0);

    let consumer_dds = Dds::new(0);
    let graph = Graph::discover_with_nodes(&consumer_dds, Duration::from_secs(8));

    let expected = NodeName {
        namespace: "/demo".into(),
        name: "self_advertised".into(),
    };
    assert!(
        graph.nodes.contains(&expected),
        "node missing from discovery; nodes={:?}",
        graph.nodes
    );
}

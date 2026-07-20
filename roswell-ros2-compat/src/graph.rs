//! ROS graph snapshot helpers over discovered DDS topics.

use std::collections::{BTreeMap, BTreeSet};
use std::time::{Duration, Instant};

use crate::discovery::{fold_nodes, DiscoveryListener, NodeName};
use crate::transport::Dds;

#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Graph {
    pub topics: Vec<TopicInfo>,
    pub services: Vec<ServiceInfo>,
    pub actions: Vec<ActionInfo>,
    /// ROS nodes learned from `ros_discovery_info` (empty unless populated via
    /// [`Graph::discover_with_nodes`]).
    pub nodes: Vec<NodeName>,
}

impl Graph {
    #[must_use]
    pub fn discover(dds: &Dds) -> Self {
        let entries = dds
            .participant()
            .discovered_topics()
            .iter()
            .map(|topic| (topic.topic_name().clone(), topic.type_name().clone()))
            .collect::<Vec<_>>();
        Self::from_dds_topics(entries)
    }

    /// Like [`Graph::discover`], but also listens on `ros_discovery_info` for up
    /// to `listen_for` to populate [`Graph::nodes`] with ROS node names.
    #[must_use]
    pub fn discover_with_nodes(dds: &Dds, listen_for: Duration) -> Self {
        let mut listener = DiscoveryListener::new(dds);
        let mut infos = Vec::new();
        let deadline = Instant::now() + listen_for;
        while Instant::now() < deadline {
            while let Some(info) = listener.take() {
                infos.push(info);
            }
            std::thread::sleep(Duration::from_millis(50));
        }
        while let Some(info) = listener.take() {
            infos.push(info);
        }
        let mut graph = Self::discover(dds);
        graph.nodes = fold_nodes(&infos);
        graph
    }

    #[must_use]
    pub fn from_dds_topics(entries: impl IntoIterator<Item = (String, String)>) -> Self {
        let mut topics = BTreeSet::new();
        let mut service_parts: BTreeMap<String, ServiceInfo> = BTreeMap::new();
        let mut action_parts: BTreeMap<String, ActionInfo> = BTreeMap::new();
        for (dds_name, dds_type) in entries {
            let ros_type = ros_type(&dds_type);
            if let Some((service, endpoint)) = service_name(&dds_name) {
                let entry = service_parts.entry(service.clone()).or_insert(ServiceInfo {
                    name: service,
                    request_type: String::new(),
                    response_type: String::new(),
                    has_request: false,
                    has_reply: false,
                });
                match endpoint {
                    ServiceEndpoint::Request => {
                        entry.request_type = ros_type;
                        entry.has_request = true;
                    }
                    ServiceEndpoint::Reply => {
                        entry.response_type = ros_type;
                        entry.has_reply = true;
                    }
                }
                continue;
            }
            if let Some((action, channel)) = action_name(&dds_name) {
                let entry = action_parts.entry(action.clone()).or_insert(ActionInfo {
                    name: action,
                    channels: BTreeSet::new(),
                });
                entry.channels.insert(channel);
                continue;
            }
            if let Some(ros_name) = topic_name(&dds_name) {
                topics.insert(TopicInfo {
                    name: ros_name,
                    ros_type,
                });
            }
        }
        Self {
            topics: topics.into_iter().collect(),
            services: service_parts.into_values().collect(),
            actions: action_parts.into_values().collect(),
            nodes: Vec::new(),
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct TopicInfo {
    pub name: String,
    pub ros_type: String,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ServiceInfo {
    pub name: String,
    pub request_type: String,
    pub response_type: String,
    pub has_request: bool,
    pub has_reply: bool,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ActionInfo {
    pub name: String,
    pub channels: BTreeSet<ActionChannel>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ServiceEndpoint {
    Request,
    Reply,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum ActionChannel {
    SendGoal,
    GetResult,
    CancelGoal,
    Feedback,
    Status,
}

#[must_use]
pub fn ros_type(dds: &str) -> String {
    let parts: Vec<&str> = dds.split("::").filter(|p| *p != "dds_").collect();
    let mut s = parts.join("/");
    if s.ends_with('_') {
        s.pop();
    }
    s
}

fn topic_name(dds: &str) -> Option<String> {
    dds.strip_prefix("rt/").map(|name| format!("/{name}"))
}

fn service_name(dds: &str) -> Option<(String, ServiceEndpoint)> {
    dds.strip_prefix("rq/")
        .and_then(|name| name.strip_suffix("Request"))
        .map(|name| (format!("/{name}"), ServiceEndpoint::Request))
        .or_else(|| {
            dds.strip_prefix("rr/")
                .and_then(|name| name.strip_suffix("Reply"))
                .map(|name| (format!("/{name}"), ServiceEndpoint::Reply))
        })
}

fn action_name(dds: &str) -> Option<(String, ActionChannel)> {
    let topic = topic_name(dds)?;
    let (base, suffix) = topic.split_once("/_action/")?;
    let channel = match suffix {
        "send_goal" => ActionChannel::SendGoal,
        "get_result" => ActionChannel::GetResult,
        "cancel_goal" => ActionChannel::CancelGoal,
        "feedback" => ActionChannel::Feedback,
        "status" => ActionChannel::Status,
        _ => return None,
    };
    Some((base.to_string(), channel))
}

#[cfg(test)]
mod tests {
    use super::{ros_type, Graph};

    #[test]
    fn graph_classifies_topics_services_and_actions() {
        let graph = Graph::from_dds_topics([
            (
                "rt/chatter".to_string(),
                "std_msgs::msg::dds_::String_".to_string(),
            ),
            (
                "rq/add_two_intsRequest".to_string(),
                "example_interfaces::srv::dds_::AddTwoInts_Request_".to_string(),
            ),
            (
                "rr/add_two_intsReply".to_string(),
                "example_interfaces::srv::dds_::AddTwoInts_Response_".to_string(),
            ),
            (
                "rt/fibonacci/_action/status".to_string(),
                "action_msgs::msg::dds_::GoalStatusArray_".to_string(),
            ),
        ]);
        assert_eq!(graph.topics[0].name, "/chatter");
        assert_eq!(graph.services[0].name, "/add_two_ints");
        assert!(graph.services[0].has_request);
        assert!(graph.services[0].has_reply);
        assert_eq!(graph.actions[0].name, "/fibonacci");
        assert!(graph.actions[0]
            .channels
            .contains(&super::ActionChannel::Status));
    }

    #[test]
    fn ros_type_demangles_dds_type_names() {
        assert_eq!(ros_type("pkg::msg::dds_::Name_"), "pkg/msg/Name");
    }
}

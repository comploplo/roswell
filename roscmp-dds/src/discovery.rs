//! ROS2 node-graph participation via the `ros_discovery_info` topic.
//!
//! ROS2 nodes announce themselves to `ros2 node list` (and the rest of the graph
//! API) by publishing `rmw_dds_common/msg/ParticipantEntitiesInfo` on the DDS
//! topic `ros_discovery_info` (note: a plain DDS name, *not* `rt/`-mangled).
//! The message carries the participant's GID plus one [`NodeEntitiesInfo`] per
//! node hosted on that participant. Publishing it is what makes our participant
//! show up as ROS nodes.
//!
//! rmw implementations offer this topic with reliable + transient-local +
//! keep-last(1) QoS (our [`Qos::Latched`] preset), so late-joining tools still
//! receive the current node set.
//!
//! Endpoint wiring: a node's publishers/subscribers are advertised by pushing
//! each endpoint's 16-byte GID into the node's `writer_gid_seq`/`reader_gid_seq`
//! (see [`DiscoveryInfo::add_writer_gid`]/[`DiscoveryInfo::add_reader_gid`]).
//! `ros2 node info` cross-references those GIDs against DDS endpoint discovery
//! (SEDP) to list each endpoint's topic and type, so the GIDs registered here
//! must be the real DDS GUIDs of endpoints we actually create.
#![deny(unsafe_code)]

use rustdds::{
    no_key::{DataReader, DataWriter},
    RTPSEntity, TopicKind,
};

use crate::codec::{CdrMsg, CodecError, De, Ser};
use crate::msgs::{
    rmw_dds_common__Gid as GidGen, rmw_dds_common__NodeEntitiesInfo as NodeGen,
    rmw_dds_common__ParticipantEntitiesInfo as ParticipantGen, RosSequence, RosString,
};
use crate::qos::QosProfile;
use crate::transport::{Dds, Qos};

/// DDS topic name that carries `ParticipantEntitiesInfo` (not `rt/`-mangled).
const DISCOVERY_TOPIC: &str = "ros_discovery_info";

/// `rmw_dds_common/msg/Gid`: a 16-byte DDS GUID identifying a participant.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Gid {
    pub data: [u8; 16],
}

/// `rmw_dds_common/msg/NodeEntitiesInfo`: one node's identity plus the GIDs of
/// its subscriber (`reader_gid_seq`) and publisher (`writer_gid_seq`) endpoints.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct NodeEntitiesInfo {
    pub node_namespace: String,
    pub node_name: String,
    pub reader_gid_seq: Vec<Gid>,
    pub writer_gid_seq: Vec<Gid>,
}

/// `rmw_dds_common/msg/ParticipantEntitiesInfo`: the participant GID plus the
/// nodes it hosts. This is the payload on `ros_discovery_info`.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct ParticipantEntitiesInfo {
    pub gid: Gid,
    pub node_entities_info_seq: Vec<NodeEntitiesInfo>,
}

// The wire (de)serialization is the generated `rmw_dds_common` codec; these
// ergonomic (`String`/`Vec`) types convert to/from the C-ABI generated structs
// at the DDS boundary. Owned generated values are freed with `fini` after use.
fn gid_seq_gen(gids: &[Gid]) -> RosSequence<GidGen> {
    RosSequence::alloc(gids.iter().map(|g| GidGen { data: g.data }).collect())
}

fn node_to_gen(n: &NodeEntitiesInfo) -> NodeGen {
    NodeGen {
        node_namespace: RosString::alloc(&n.node_namespace),
        node_name: RosString::alloc(&n.node_name),
        reader_gid_seq: gid_seq_gen(&n.reader_gid_seq),
        writer_gid_seq: gid_seq_gen(&n.writer_gid_seq),
    }
}

fn gid_seq_from(seq: &RosSequence<GidGen>) -> Vec<Gid> {
    seq.as_slice()
        .iter()
        .map(|g| Gid { data: g.data })
        .collect()
}

fn node_from_gen(n: &NodeGen) -> NodeEntitiesInfo {
    NodeEntitiesInfo {
        node_namespace: n.node_namespace.as_str().to_string(),
        node_name: n.node_name.as_str().to_string(),
        reader_gid_seq: gid_seq_from(&n.reader_gid_seq),
        writer_gid_seq: gid_seq_from(&n.writer_gid_seq),
    }
}

impl ParticipantEntitiesInfo {
    fn to_gen(&self) -> ParticipantGen {
        ParticipantGen {
            gid: GidGen {
                data: self.gid.data,
            },
            node_entities_info: RosSequence::alloc(
                self.node_entities_info_seq
                    .iter()
                    .map(node_to_gen)
                    .collect(),
            ),
        }
    }

    fn from_gen(g: &ParticipantGen) -> Self {
        Self {
            gid: Gid { data: g.gid.data },
            node_entities_info_seq: g
                .node_entities_info
                .as_slice()
                .iter()
                .map(node_from_gen)
                .collect(),
        }
    }
}

// The only `unsafe` in this module: freeing the C-ABI generated codec buffers
// (`fini`) after they cross the wire. The ergonomic types above stay pure-safe.
#[allow(unsafe_code)]
impl CdrMsg for ParticipantEntitiesInfo {
    const TYPE_NAME: &'static str = "rmw_dds_common::msg::dds_::ParticipantEntitiesInfo_";

    fn encode(&self) -> Vec<u8> {
        let mut g = self.to_gen();
        let bytes = g.encode();
        // SAFETY: `g` is a freshly-built owned value, finalized exactly once.
        unsafe { g.fini() };
        bytes
    }

    fn decode(buf: &[u8]) -> Result<Self, CodecError> {
        let mut g = ParticipantGen::decode(buf)
            .map_err(|_| CodecError("participant-entities-info decode failed"))?;
        let out = Self::from_gen(&g);
        // SAFETY: `g` was decoded (owned) and is finalized exactly once.
        unsafe { g.fini() };
        Ok(out)
    }
}

/// Push `gid` onto the matching node's reader or writer sequence, returning
/// whether anything changed (node found and GID not already present).
fn push_gid(
    nodes: &mut [NodeEntitiesInfo],
    namespace: &str,
    name: &str,
    gid: [u8; 16],
    reader: bool,
) -> bool {
    let Some(node) = nodes
        .iter_mut()
        .find(|n| n.node_namespace == namespace && n.node_name == name)
    else {
        return false;
    };
    let seq = if reader {
        &mut node.reader_gid_seq
    } else {
        &mut node.writer_gid_seq
    };
    if seq.iter().any(|g| g.data == gid) {
        return false;
    }
    seq.push(Gid { data: gid });
    true
}

/// A discovered ROS node: its namespace and name.
#[derive(Clone, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
pub struct NodeName {
    pub namespace: String,
    pub name: String,
}

impl NodeName {
    /// The fully-qualified name, e.g. `/ns/node` (or `/node` when the namespace
    /// is the root `/`).
    #[must_use]
    pub fn full_name(&self) -> String {
        if self.namespace == "/" {
            format!("/{}", self.name)
        } else {
            format!("{}/{}", self.namespace.trim_end_matches('/'), self.name)
        }
    }
}

/// Fold received `ParticipantEntitiesInfo` messages into a sorted, de-duplicated
/// list of node names.
#[must_use]
pub fn fold_nodes<'a>(
    infos: impl IntoIterator<Item = &'a ParticipantEntitiesInfo>,
) -> Vec<NodeName> {
    let mut nodes: Vec<NodeName> = infos
        .into_iter()
        .flat_map(|info| &info.node_entities_info_seq)
        .map(|node| NodeName {
            namespace: node.node_namespace.clone(),
            name: node.node_name.clone(),
        })
        .collect();
    nodes.sort();
    nodes.dedup();
    nodes
}

/// Publisher of our participant's node set on `ros_discovery_info`.
///
/// Add/remove nodes and the full [`ParticipantEntitiesInfo`] is (re)published on
/// every change under the latched QoS, so joining ROS tools always see the
/// current set.
pub struct DiscoveryInfo {
    gid: Gid,
    nodes: Vec<NodeEntitiesInfo>,
    writer: DataWriter<ParticipantEntitiesInfo, Ser<ParticipantEntitiesInfo>>,
}

impl DiscoveryInfo {
    /// Create the discovery publisher for `dds`, deriving the participant GID
    /// from the RustDDS participant GUID (12-byte prefix + 4-byte entity id).
    #[must_use]
    pub fn new(dds: &Dds) -> Self {
        let gid = Gid {
            data: dds.participant().guid().to_bytes(),
        };
        let q = QosProfile::from_preset(Qos::Latched).policies();
        let dp = dds.participant();
        let topic = dp
            .create_topic(
                DISCOVERY_TOPIC.to_string(),
                ParticipantEntitiesInfo::TYPE_NAME.to_string(),
                &q,
                TopicKind::NoKey,
            )
            .expect("create ros_discovery_info topic");
        let writer = dp
            .create_publisher(&q)
            .expect("create discovery publisher")
            .create_datawriter_no_key(&topic, None)
            .expect("create discovery writer");
        Self {
            gid,
            nodes: Vec::new(),
            writer,
        }
    }

    /// The participant GID advertised in every message.
    #[must_use]
    pub const fn gid(&self) -> Gid {
        self.gid
    }

    /// Announce a node under `namespace`/`name` and republish. Duplicate
    /// (namespace, name) pairs are ignored.
    pub fn add_node(&mut self, namespace: &str, name: &str) {
        if self
            .nodes
            .iter()
            .any(|n| n.node_namespace == namespace && n.node_name == name)
        {
            return;
        }
        self.nodes.push(NodeEntitiesInfo {
            node_namespace: namespace.to_string(),
            node_name: name.to_string(),
            reader_gid_seq: Vec::new(),
            writer_gid_seq: Vec::new(),
        });
        self.publish();
    }

    /// Remove a previously announced node and republish. No-op if absent.
    pub fn remove_node(&mut self, namespace: &str, name: &str) {
        let before = self.nodes.len();
        self.nodes
            .retain(|n| !(n.node_namespace == namespace && n.node_name == name));
        if self.nodes.len() != before {
            self.publish();
        }
    }

    /// Advertise a publisher endpoint under `namespace`/`name` by its 16-byte
    /// GID (from [`crate::transport::DdsPub::gid`]) and republish. No-op if the
    /// node is unknown or the GID is already listed.
    pub fn add_writer_gid(&mut self, namespace: &str, name: &str, gid: [u8; 16]) {
        if push_gid(&mut self.nodes, namespace, name, gid, false) {
            self.publish();
        }
    }

    /// Advertise a subscriber endpoint under `namespace`/`name` by its 16-byte
    /// GID (from [`crate::transport::DdsSub::gid`]) and republish. No-op if the
    /// node is unknown or the GID is already listed.
    pub fn add_reader_gid(&mut self, namespace: &str, name: &str, gid: [u8; 16]) {
        if push_gid(&mut self.nodes, namespace, name, gid, true) {
            self.publish();
        }
    }

    fn publish(&self) {
        let _ = self.writer.write(self.snapshot(), None);
    }

    fn snapshot(&self) -> ParticipantEntitiesInfo {
        ParticipantEntitiesInfo {
            gid: self.gid,
            node_entities_info_seq: self.nodes.clone(),
        }
    }
}

/// Subscriber for `ros_discovery_info` that surfaces other participants' node
/// sets.
pub struct DiscoveryListener {
    reader: DataReader<ParticipantEntitiesInfo, De<ParticipantEntitiesInfo>>,
}

impl DiscoveryListener {
    /// Bind a listener on `dds` with the latched QoS rmw publishers offer.
    /// The reader keeps a deep history: with keep-last(1) on this NoKey topic,
    /// one participant's announcement would evict another's.
    #[must_use]
    pub fn new(dds: &Dds) -> Self {
        let mut profile = QosProfile::from_preset(Qos::Latched);
        profile.depth = 100;
        let q = profile.policies();
        let dp = dds.participant();
        let topic = dp
            .create_topic(
                DISCOVERY_TOPIC.to_string(),
                ParticipantEntitiesInfo::TYPE_NAME.to_string(),
                &q,
                TopicKind::NoKey,
            )
            .expect("create ros_discovery_info topic");
        let reader = dp
            .create_subscriber(&q)
            .expect("create discovery subscriber")
            .create_datareader_no_key(&topic, None)
            .expect("create discovery reader");
        Self { reader }
    }

    /// Take the next pending `ParticipantEntitiesInfo`, if any (non-blocking).
    pub fn take(&mut self) -> Option<ParticipantEntitiesInfo> {
        self.reader
            .take_next_sample()
            .ok()
            .flatten()
            .map(rustdds::no_key::DataSample::into_value)
    }
}

#[cfg(test)]
mod tests {
    use super::{fold_nodes, CdrMsg, Gid, NodeEntitiesInfo, NodeName, ParticipantEntitiesInfo};

    fn gid(seed: u8) -> Gid {
        let mut data = [0u8; 16];
        for (i, byte) in data.iter_mut().enumerate() {
            *byte = seed.wrapping_add(i as u8);
        }
        Gid { data }
    }

    #[test]
    fn gid_and_node_round_trip_inside_participant_info() {
        let info = ParticipantEntitiesInfo {
            gid: gid(1),
            node_entities_info_seq: vec![
                NodeEntitiesInfo {
                    node_namespace: "/".into(),
                    node_name: "talker".into(),
                    reader_gid_seq: vec![gid(2)],
                    writer_gid_seq: vec![gid(3), gid(4)],
                },
                NodeEntitiesInfo {
                    node_namespace: "/robot".into(),
                    node_name: "driver".into(),
                    ..NodeEntitiesInfo::default()
                },
            ],
        };
        let back = ParticipantEntitiesInfo::decode(&info.encode()).unwrap();
        assert_eq!(back, info);
    }

    #[test]
    fn push_gid_appends_to_matching_node_and_dedups() {
        let mut nodes = vec![NodeEntitiesInfo {
            node_namespace: "/".into(),
            node_name: "talker".into(),
            ..NodeEntitiesInfo::default()
        }];
        assert!(super::push_gid(
            &mut nodes,
            "/",
            "talker",
            gid(5).data,
            false
        ));
        assert!(!super::push_gid(
            &mut nodes,
            "/",
            "talker",
            gid(5).data,
            false
        )); // dedup
        assert!(super::push_gid(
            &mut nodes,
            "/",
            "talker",
            gid(6).data,
            true
        ));
        assert!(!super::push_gid(
            &mut nodes,
            "/",
            "missing",
            gid(7).data,
            true
        )); // unknown node
        assert_eq!(nodes[0].writer_gid_seq, vec![gid(5)]);
        assert_eq!(nodes[0].reader_gid_seq, vec![gid(6)]);
    }

    #[test]
    fn empty_participant_info_round_trips() {
        let info = ParticipantEntitiesInfo::default();
        let back = ParticipantEntitiesInfo::decode(&info.encode()).unwrap();
        assert_eq!(back, info);
    }

    #[test]
    fn fold_nodes_dedups_and_sorts() {
        let a = ParticipantEntitiesInfo {
            node_entities_info_seq: vec![
                NodeEntitiesInfo {
                    node_namespace: "/".into(),
                    node_name: "talker".into(),
                    ..NodeEntitiesInfo::default()
                },
                NodeEntitiesInfo {
                    node_namespace: "/".into(),
                    node_name: "talker".into(),
                    ..NodeEntitiesInfo::default()
                },
            ],
            ..ParticipantEntitiesInfo::default()
        };
        let b = ParticipantEntitiesInfo {
            node_entities_info_seq: vec![NodeEntitiesInfo {
                node_namespace: "/robot".into(),
                node_name: "driver".into(),
                ..NodeEntitiesInfo::default()
            }],
            ..ParticipantEntitiesInfo::default()
        };
        let nodes = fold_nodes([&a, &b]);
        assert_eq!(
            nodes,
            vec![
                NodeName {
                    namespace: "/".into(),
                    name: "talker".into()
                },
                NodeName {
                    namespace: "/robot".into(),
                    name: "driver".into()
                },
            ]
        );
    }

    #[test]
    fn node_full_name_handles_root_and_nested_namespaces() {
        assert_eq!(
            NodeName {
                namespace: "/".into(),
                name: "talker".into()
            }
            .full_name(),
            "/talker"
        );
        assert_eq!(
            NodeName {
                namespace: "/robot".into(),
                name: "driver".into()
            }
            .full_name(),
            "/robot/driver"
        );
    }
}

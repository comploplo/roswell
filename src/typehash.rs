//! RIHS01 type hashes — ROS2's type-description hash used in discovery/type
//! negotiation (Jazzy+).
//!
//! Replicates `rosidl_generator_type_description.calculate_type_hash`: build the
//! type description (the message + all transitively nested types as
//! `referenced_type_descriptions`, sorted by name), serialize to JSON with
//! `(", ", ": ")` separators and `default_value` removed, SHA-256, prefix
//! `RIHS01_`. Verified byte-exact against `ros:jazzy` (see `tests/typehash_tests.rs`).

use std::collections::{BTreeMap, BTreeSet};
use std::fmt::Write;

use sha2::{Digest, Sha256};

use crate::ir::{Element, Message, MsgId, Prim, ResolvedType};

/// Compute the `RIHS01_…` type hash for `root`, given all reachable messages
/// (as produced by [`crate::resolve`]). Returns `None` if `root` is absent.
pub fn type_hash(messages: &[Message], root: &MsgId) -> Option<String> {
    let json = type_description_json(messages, root)?;
    let mut hasher = Sha256::new();
    hasher.update(json.as_bytes());
    let digest = hasher.finalize();
    let mut hex = String::with_capacity(64);
    for b in digest {
        let _ = write!(hex, "{b:02x}");
    }
    Some(format!("RIHS01_{hex}"))
}

/// Build the JSON object ROS hashes for RIHS01 and serves through
/// `~/get_type_description`.
pub fn type_description_json(messages: &[Message], root: &MsgId) -> Option<String> {
    let map: BTreeMap<&MsgId, &Message> = messages.iter().map(|m| (&m.id, m)).collect();
    let root_msg = map.get(root)?;

    // Referenced type descriptions: every transitively nested message, keyed by
    // full name so the BTreeMap yields them sorted (as ROS does).
    let mut refs: BTreeMap<String, String> = BTreeMap::new();
    let mut seen: BTreeSet<MsgId> = BTreeSet::new();
    seen.insert(root.clone());
    collect_refs(root_msg, &map, &mut seen, &mut refs);

    let refs_json: Vec<String> = refs.into_values().collect();
    Some(format!(
        "{{\"type_description\": {}, \"referenced_type_descriptions\": [{}]}}",
        itd_json(root_msg),
        refs_json.join(", ")
    ))
}

fn collect_refs(
    msg: &Message,
    map: &BTreeMap<&MsgId, &Message>,
    seen: &mut BTreeSet<MsgId>,
    refs: &mut BTreeMap<String, String>,
) {
    for f in &msg.fields {
        if let Some(id) = nested_id(&f.ty)
            && seen.insert(id.clone())
            && let Some(dep) = map.get(&id)
        {
            refs.insert(full_name(&id), itd_json(dep));
            collect_refs(dep, map, seen, refs);
        }
    }
}

fn nested_id(ty: &ResolvedType) -> Option<MsgId> {
    let elem = match ty {
        ResolvedType::Scalar(e)
        | ResolvedType::Array { elem: e, .. }
        | ResolvedType::Sequence { elem: e, .. } => e,
    };
    match elem {
        Element::Message(id) => Some(id.clone()),
        _ => None,
    }
}

/// Full ROS type name, including generated service/action namespaces.
fn full_name(id: &MsgId) -> String {
    format!("{}/{}/{}", id.package, subnamespace(&id.name), id.name)
}

fn subnamespace(name: &str) -> &'static str {
    if name.contains("_SendGoal_")
        || name.contains("_GetResult_")
        || name.ends_with("_Goal")
        || name.ends_with("_Result")
        || name.ends_with("_Feedback")
        || name.ends_with("_FeedbackMessage")
    {
        "action"
    } else if name.ends_with("_Request") || name.ends_with("_Response") {
        "srv"
    } else {
        "msg"
    }
}

fn itd_json(msg: &Message) -> String {
    let fields: Vec<String> = msg.fields.iter().map(field_json).collect();
    format!(
        "{{\"type_name\": \"{}\", \"fields\": [{}]}}",
        full_name(&msg.id),
        fields.join(", ")
    )
}

fn field_json(f: &crate::ir::ResolvedField) -> String {
    let (type_id, capacity, string_capacity, nested) = field_type_props(&f.ty);
    format!(
        "{{\"name\": \"{}\", \"type\": {{\"type_id\": {type_id}, \"capacity\": {capacity}, \
         \"string_capacity\": {string_capacity}, \"nested_type_name\": \"{nested}\"}}}}",
        f.name
    )
}

/// `(type_id, capacity, string_capacity, nested_type_name)` for a field type.
fn field_type_props(ty: &ResolvedType) -> (u16, u64, u64, String) {
    // Array/sequence shift the element's base id: +48 fixed array, +96 bounded
    // sequence, +144 unbounded sequence; `capacity` holds the count/bound.
    let (elem, offset, capacity) = match ty {
        ResolvedType::Scalar(e) => (e, 0, 0),
        ResolvedType::Array { elem, len } => (elem, 48, *len as u64),
        ResolvedType::Sequence {
            elem,
            bound: Some(n),
        } => (elem, 96, *n as u64),
        ResolvedType::Sequence { elem, bound: None } => (elem, 144, 0),
    };
    let (base, string_capacity, nested) = base_props(elem);
    (u16::from(base) + offset, capacity, string_capacity, nested)
}

/// `(base_type_id, string_capacity, nested_type_name)` for an element.
fn base_props(elem: &Element) -> (u8, u64, String) {
    match elem {
        Element::Prim(p) => (prim_id(*p), 0, String::new()),
        Element::String { bound: None, wide } => (if *wide { 18 } else { 17 }, 0, String::new()),
        Element::String {
            bound: Some(n),
            wide,
        } => (if *wide { 22 } else { 21 }, *n as u64, String::new()),
        Element::Message(id) => (1, 0, full_name(id)),
    }
}

fn prim_id(p: Prim) -> u8 {
    use Prim::*;
    match p {
        Int8 => 2,
        Uint8 => 3,
        Int16 => 4,
        Uint16 => 5,
        Int32 => 6,
        Uint32 => 7,
        Int64 => 8,
        Uint64 => 9,
        Float32 => 10,
        Float64 => 11,
        Char => 13,
        Bool => 15,
        Byte => 16,
    }
}

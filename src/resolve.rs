//! Resolve parsed [`MessageSpec`]s into a layout-aware [`Program`].
//!
//! Responsibilities:
//! - normalize type references (`pkg/msg/Type` -> `pkg/Type`, bare `Header` ->
//!   `std_msgs/Header`, bare `Name` -> same package as the referrer);
//! - inject the well-known builtin messages (`builtin_interfaces/Time`,
//!   `builtin_interfaces/Duration`, `std_msgs/Header`) so output is
//!   self-contained;
//! - validate constants and report unknown types / by-value cycles;
//! - emit only the messages reachable from the user's inputs, ordered so each
//!   message's by-value dependencies come first.

use std::collections::{BTreeMap, BTreeSet};

use crate::ast::{ActionSpec, BaseType, MessageSpec, ServiceSpec, TypeName};
use crate::ir::*;

/// Expand a service into its `_Request`/`_Response` messages, ready for
/// [`resolve`]. ROS2 generates these as ordinary messages.
pub fn service_messages(package: &str, name: &str, svc: &ServiceSpec) -> Vec<(MsgId, MessageSpec)> {
    vec![
        (
            MsgId::new(package, format!("{name}_Request")),
            svc.request.clone(),
        ),
        (
            MsgId::new(package, format!("{name}_Response")),
            svc.response.clone(),
        ),
    ]
}

/// Expand an action into its `_Goal`/`_Result`/`_Feedback` messages.
pub fn action_messages(package: &str, name: &str, act: &ActionSpec) -> Vec<(MsgId, MessageSpec)> {
    let goal = format!("{name}_Goal");
    let result = format!("{name}_Result");
    let feedback = format!("{name}_Feedback");
    vec![
        (MsgId::new(package, goal.clone()), act.goal.clone()),
        (MsgId::new(package, result.clone()), act.result.clone()),
        (MsgId::new(package, feedback.clone()), act.feedback.clone()),
        (
            MsgId::new(package, format!("{name}_SendGoal_Request")),
            crate::parse_message(&format!(
                "unique_identifier_msgs/UUID goal_id\n{package}/{goal} goal\n"
            ))
            .expect("internal action SendGoal request parses"),
        ),
        (
            MsgId::new(package, format!("{name}_SendGoal_Response")),
            crate::parse_message("bool accepted\nbuiltin_interfaces/Time stamp\n")
                .expect("internal action SendGoal response parses"),
        ),
        (
            MsgId::new(package, format!("{name}_GetResult_Request")),
            crate::parse_message("unique_identifier_msgs/UUID goal_id\n")
                .expect("internal action GetResult request parses"),
        ),
        (
            MsgId::new(package, format!("{name}_GetResult_Response")),
            crate::parse_message(&format!("int8 status\n{package}/{result} result\n"))
                .expect("internal action GetResult response parses"),
        ),
        (
            MsgId::new(package, format!("{name}_FeedbackMessage")),
            crate::parse_message(&format!(
                "unique_identifier_msgs/UUID goal_id\n{package}/{feedback} feedback\n"
            ))
            .expect("internal action feedback message parses"),
        ),
    ]
}

#[derive(Debug, Clone, PartialEq)]
pub struct ResolveError(pub String);

impl std::fmt::Display for ResolveError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "resolve error: {}", self.0)
    }
}

impl std::error::Error for ResolveError {}

/// Source for the builtin messages, injected when not supplied by the caller.
const BUILTINS: &[(&str, &str, &str)] = &[
    ("builtin_interfaces", "Time", "int32 sec\nuint32 nanosec\n"),
    (
        "builtin_interfaces",
        "Duration",
        "int32 sec\nuint32 nanosec\n",
    ),
    ("unique_identifier_msgs", "UUID", "uint8[16] uuid\n"),
    (
        "std_msgs",
        "Header",
        "builtin_interfaces/Time stamp\nstring frame_id\n",
    ),
];

/// Resolve a set of `(id, spec)` inputs into an ordered program.
///
/// `roots` are emitted along with every message reachable from them by value or
/// by sequence; builtins are pulled in automatically when referenced.
pub fn resolve(inputs: Vec<(MsgId, MessageSpec)>) -> Result<Program, ResolveError> {
    // Pool of all candidate definitions: user inputs plus builtins (user inputs
    // win on conflict so a project can override a builtin).
    let mut pool: BTreeMap<MsgId, MessageSpec> = BTreeMap::new();
    let roots: Vec<MsgId> = inputs.iter().map(|(id, _)| id.clone()).collect();

    for (name_pkg, name, src) in BUILTINS {
        let id = MsgId::new(*name_pkg, *name);
        let spec = crate::parse_message(src)
            .map_err(|e| ResolveError(format!("builtin {id:?} failed to parse: {e}")))?;
        pool.insert(id, spec);
    }
    for (id, spec) in inputs {
        pool.insert(id, spec);
    }

    let mut resolved: BTreeMap<MsgId, Message> = BTreeMap::new();
    let mut queue: Vec<MsgId> = roots.clone();
    let mut seen: BTreeSet<MsgId> = BTreeSet::new();

    // Resolve reachable messages breadth-first, discovering dependencies.
    while let Some(id) = queue.pop() {
        if !seen.insert(id.clone()) {
            continue;
        }
        let spec = pool
            .get(&id)
            .ok_or_else(|| ResolveError(format!("unknown message `{}/{}`", id.package, id.name)))?;
        let msg = resolve_message(&id, spec, &pool)?;
        for dep in message_deps(&msg) {
            if !seen.contains(&dep) {
                queue.push(dep);
            }
        }
        resolved.insert(id, msg);
    }

    let ordered = topo_order(&resolved)?;
    Ok(Program { messages: ordered })
}

fn resolve_message(
    id: &MsgId,
    spec: &MessageSpec,
    pool: &BTreeMap<MsgId, MessageSpec>,
) -> Result<Message, ResolveError> {
    let mut constants = Vec::new();
    for c in &spec.constants {
        let prim = match &c.ty {
            BaseType::String { .. } | BaseType::WString { .. } => ConstType::String,
            other => ConstType::Prim(prim_of(other).ok_or_else(|| {
                ResolveError(format!(
                    "constant `{}` in `{}/{}` has non-primitive type",
                    c.name, id.package, id.name
                ))
            })?),
        };
        constants.push(ResolvedConstant {
            name: c.name.clone(),
            prim,
            value: c.value.clone(),
        });
    }

    let mut fields = Vec::new();
    for f in &spec.fields {
        let elem = resolve_element(&f.ty.base, id, pool)?;
        let ty = match f.ty.array {
            None => ResolvedType::Scalar(elem),
            Some(crate::ast::ArrayKind::Fixed(n)) => ResolvedType::Array { elem, len: n },
            Some(crate::ast::ArrayKind::Bounded(n)) => ResolvedType::Sequence {
                elem,
                bound: Some(n),
            },
            Some(crate::ast::ArrayKind::Unbounded) => ResolvedType::Sequence { elem, bound: None },
        };
        fields.push(ResolvedField {
            name: f.name.clone(),
            ty,
            default: f.default.clone(),
        });
    }

    Ok(Message {
        id: id.clone(),
        constants,
        fields,
    })
}

fn resolve_element(
    base: &BaseType,
    referrer: &MsgId,
    pool: &BTreeMap<MsgId, MessageSpec>,
) -> Result<Element, ResolveError> {
    if let Some(p) = prim_of(base) {
        return Ok(Element::Prim(p));
    }
    match base {
        BaseType::String { bound } => Ok(Element::String {
            bound: *bound,
            wide: false,
        }),
        BaseType::WString { bound } => Ok(Element::String {
            bound: *bound,
            wide: true,
        }),
        // ROS1 temporal builtins map onto the builtin_interfaces messages.
        BaseType::Time => Ok(Element::Message(MsgId::new("builtin_interfaces", "Time"))),
        BaseType::Duration => Ok(Element::Message(MsgId::new(
            "builtin_interfaces",
            "Duration",
        ))),
        BaseType::Named(tn) => {
            let id = normalize_ref(tn, referrer);
            if !pool.contains_key(&id) {
                return Err(ResolveError(format!(
                    "`{}/{}` references unknown type `{}/{}`",
                    referrer.package, referrer.name, id.package, id.name
                )));
            }
            Ok(Element::Message(id))
        }
        _ => unreachable!("primitive handled above"),
    }
}

/// Map a type reference to a fully-qualified [`MsgId`], applying ROS2's naming
/// conventions.
fn normalize_ref(tn: &TypeName, referrer: &MsgId) -> MsgId {
    match &tn.package {
        Some(pkg) => {
            // Drop a conventional trailing `/msg` subnamespace.
            let pkg = pkg.strip_suffix("/msg").unwrap_or(pkg);
            MsgId::new(pkg, &tn.name)
        }
        None if tn.name == "Header" => MsgId::new("std_msgs", "Header"),
        None => MsgId::new(&referrer.package, &tn.name),
    }
}

/// Primitive types that have a fixed scalar layout (excludes strings).
fn prim_of(base: &BaseType) -> Option<Prim> {
    Some(match base {
        BaseType::Bool => Prim::Bool,
        BaseType::Byte => Prim::Byte,
        BaseType::Char => Prim::Char,
        BaseType::Int8 => Prim::Int8,
        BaseType::Uint8 => Prim::Uint8,
        BaseType::Int16 => Prim::Int16,
        BaseType::Uint16 => Prim::Uint16,
        BaseType::Int32 => Prim::Int32,
        BaseType::Uint32 => Prim::Uint32,
        BaseType::Int64 => Prim::Int64,
        BaseType::Uint64 => Prim::Uint64,
        BaseType::Float32 => Prim::Float32,
        BaseType::Float64 => Prim::Float64,
        _ => return None,
    })
}

/// All message dependencies (by value or by sequence) of a message.
fn message_deps(msg: &Message) -> Vec<MsgId> {
    msg.fields
        .iter()
        .filter_map(|f| match &f.ty {
            ResolvedType::Scalar(e) | ResolvedType::Array { elem: e, .. } => msg_of(e),
            ResolvedType::Sequence { elem, .. } => msg_of(elem),
        })
        .collect()
}

/// By-value dependencies only — these constrain definition order.
fn by_value_deps(msg: &Message) -> Vec<MsgId> {
    msg.fields
        .iter()
        .filter_map(|f| match &f.ty {
            ResolvedType::Scalar(e) | ResolvedType::Array { elem: e, .. } => msg_of(e),
            // Sequences hold a pointer; the element need not be defined first.
            ResolvedType::Sequence { .. } => None,
        })
        .collect()
}

fn msg_of(e: &Element) -> Option<MsgId> {
    match e {
        Element::Message(id) => Some(id.clone()),
        _ => None,
    }
}

/// Depth-first topological visit, detecting by-value cycles.
fn topo_visit(
    id: &MsgId,
    resolved: &BTreeMap<MsgId, Message>,
    state: &mut BTreeMap<MsgId, Mark>,
    out: &mut Vec<Message>,
) -> Result<(), ResolveError> {
    match state.get(id) {
        Some(Mark::Done) => return Ok(()),
        Some(Mark::Active) => {
            return Err(ResolveError(format!(
                "by-value type cycle through `{}/{}`",
                id.package, id.name
            )));
        }
        None => {}
    }
    state.insert(id.clone(), Mark::Active);
    let msg = &resolved[id];
    for dep in by_value_deps(msg) {
        topo_visit(&dep, resolved, state, out)?;
    }
    state.insert(id.clone(), Mark::Done);
    out.push(msg.clone());
    Ok(())
}

/// Order messages so every by-value dependency precedes its dependents.
fn topo_order(resolved: &BTreeMap<MsgId, Message>) -> Result<Vec<Message>, ResolveError> {
    let mut ordered = Vec::new();
    let mut state: BTreeMap<MsgId, Mark> = BTreeMap::new();
    let ids: Vec<MsgId> = resolved.keys().cloned().collect();
    for id in &ids {
        topo_visit(id, resolved, &mut state, &mut ordered)?;
    }
    Ok(ordered)
}

#[derive(Clone, Copy)]
enum Mark {
    Active,
    Done,
}

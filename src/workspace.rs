//! Resolve ROS interface *references* (`pkg/msg/Name`) against workspace search
//! roots, discovering the full cross-package dependency closure by walking the
//! roots — the ROS-workspace analogue of the file-path [`load_message`] loader.
//!
//! A search root is a directory laid out one of two ways:
//! - a plain package tree (`<root>/<pkg>/{msg,srv,action}/<Name>.<ext>`), as in a
//!   colcon `src/` checkout or the bundled sample tree; or
//! - an ament install prefix (`<root>/share/<pkg>/{msg,srv,action}/<Name>.<ext>`),
//!   as pointed at by `AMENT_PREFIX_PATH`.
//!
//! Both layouts are probed for every lookup, so a caller can hand us `type_paths`,
//! `ROSWELL_TYPE_PATH`, and `AMENT_PREFIX_PATH` entries interchangeably. Parsing
//! and dependency resolution stay here in Rust; the Python/C caller only passes
//! the reference string and the list of root directories.
//!
//! [`load_message`]: crate::dynamic::load_message

use std::collections::BTreeSet;
use std::path::{Path, PathBuf};

use crate::ast::{BaseType, MessageSpec, TypeName};
use crate::dynamic::DynamicType;
use crate::ir::{MsgId, Program};

/// Which interface kind a reference names, selecting its subdirectory/extension.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Kind {
    Msg,
    Srv,
    Action,
}

impl Kind {
    fn dir_ext(self) -> &'static str {
        match self {
            Kind::Msg => "msg",
            Kind::Srv => "srv",
            Kind::Action => "action",
        }
    }
}

/// A parsed interface reference: `package`, `kind`, and `name`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Ref {
    pub package: String,
    pub kind: Kind,
    pub name: String,
}

/// A resolution failure, carrying a human-readable message for the FFI boundary.
#[derive(Debug)]
pub struct ResolveRefError(pub String);

impl std::fmt::Display for ResolveRefError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::error::Error for ResolveRefError {}

fn err(msg: impl Into<String>) -> ResolveRefError {
    ResolveRefError(msg.into())
}

/// Parse `pkg/msg/Name`, `pkg/srv/Name`, `pkg/action/Name`, or the two-segment
/// `pkg/Name` (taking `default_kind`). A trailing extension is stripped.
fn parse_ref(reference: &str, default_kind: Kind) -> Result<Ref, ResolveRefError> {
    let cleaned = reference
        .strip_suffix(".msg")
        .or_else(|| reference.strip_suffix(".srv"))
        .or_else(|| reference.strip_suffix(".action"))
        .unwrap_or(reference);
    let parts: Vec<&str> = cleaned.split('/').filter(|s| !s.is_empty()).collect();
    match parts.as_slice() {
        [pkg, kind, name] => {
            let kind = match *kind {
                "msg" => Kind::Msg,
                "srv" => Kind::Srv,
                "action" => Kind::Action,
                other => {
                    return Err(err(format!(
                        "unknown interface kind `{other}` in `{reference}`"
                    )));
                }
            };
            Ok(Ref {
                package: (*pkg).to_string(),
                kind,
                name: (*name).to_string(),
            })
        }
        [pkg, name] => Ok(Ref {
            package: (*pkg).to_string(),
            kind: default_kind,
            name: (*name).to_string(),
        }),
        _ => Err(err(format!(
            "malformed type reference `{reference}` (expected `pkg/{}/Name`)",
            default_kind.dir_ext()
        ))),
    }
}

/// Locate `<pkg>/<dir>/<name>.<ext>` under any root, probing both the plain and
/// ament `share/` layouts.
fn find_file(package: &str, name: &str, kind: Kind, roots: &[PathBuf]) -> Option<PathBuf> {
    // ROS uses the same token for the subdirectory and the extension (msg/.msg).
    let rel = format!("{dir}/{name}.{dir}", dir = kind.dir_ext());
    for root in roots {
        for base in [root.join(package), root.join("share").join(package)] {
            let candidate = base.join(&rel);
            if candidate.is_file() {
                return Some(candidate);
            }
        }
    }
    None
}

/// The builtins [`crate::resolve`] injects automatically; we never need to find
/// files for them (though a root that ships them is still honored if present).
fn is_builtin(id: &MsgId) -> bool {
    matches!(
        (id.package.as_str(), id.name.as_str()),
        ("builtin_interfaces", "Time" | "Duration")
            | ("std_msgs", "Header")
            | ("unique_identifier_msgs", "UUID")
    )
}

/// Normalize a field's type reference to a fully-qualified [`MsgId`], matching
/// [`crate::resolve`]'s naming rules (drop a `/msg` subnamespace, `Header` ->
/// `std_msgs/Header`, bare name -> the referrer's package).
fn normalize(tn: &TypeName, referrer: &MsgId) -> MsgId {
    match &tn.package {
        Some(pkg) => {
            let pkg = pkg.strip_suffix("/msg").unwrap_or(pkg);
            MsgId::new(pkg, &tn.name)
        }
        None if tn.name == "Header" => MsgId::new("std_msgs", "Header"),
        None => MsgId::new(&referrer.package, &tn.name),
    }
}

/// Every message-typed reference a spec makes (normalized against `referrer`).
fn referenced_ids(spec: &MessageSpec, referrer: &MsgId) -> Vec<MsgId> {
    spec.fields
        .iter()
        .filter_map(|f| match &f.ty.base {
            BaseType::Named(tn) => Some(normalize(tn, referrer)),
            _ => None,
        })
        .collect()
}

/// Parse the root reference's file into its `(id, spec)` inputs, expanding a
/// `.srv`/`.action` into the messages ROS generates for it.
fn base_inputs(r: &Ref, roots: &[PathBuf]) -> Result<Vec<(MsgId, MessageSpec)>, ResolveRefError> {
    let path = find_file(&r.package, &r.name, r.kind, roots).ok_or_else(|| {
        err(format!(
            "could not find `{}/{}/{}.{}` under any search root ({} root(s))",
            r.package,
            r.kind.dir_ext(),
            r.name,
            r.kind.dir_ext(),
            roots.len()
        ))
    })?;
    let src = std::fs::read_to_string(&path)
        .map_err(|e| err(format!("reading {}: {e}", path.display())))?;
    let parse = |e: crate::ParseError| err(format!("parsing {}: {e}", path.display()));
    match r.kind {
        Kind::Msg => {
            let spec = crate::parse_message(&src).map_err(parse)?;
            Ok(vec![(MsgId::new(&r.package, &r.name), spec)])
        }
        Kind::Srv => {
            let svc = crate::parse_service(&src).map_err(parse)?;
            Ok(crate::service_messages(&r.package, &r.name, &svc))
        }
        Kind::Action => {
            let act = crate::parse_action(&src).map_err(parse)?;
            Ok(crate::action_messages(&r.package, &r.name, &act))
        }
    }
}

/// Load a `.msg` dependency file into its single `(id, spec)`.
fn load_dep(
    id: &MsgId,
    roots: &[PathBuf],
) -> Result<Option<(MsgId, MessageSpec)>, ResolveRefError> {
    let Some(path) = find_file(&id.package, &id.name, Kind::Msg, roots) else {
        return Ok(None);
    };
    let src = std::fs::read_to_string(&path)
        .map_err(|e| err(format!("reading {}: {e}", path.display())))?;
    let spec =
        crate::parse_message(&src).map_err(|e| err(format!("parsing {}: {e}", path.display())))?;
    Ok(Some((id.clone(), spec)))
}

/// Resolve `reference` into a topo-ordered [`Program`] plus the parsed [`Ref`],
/// pulling every transitively-referenced message file out of `roots`.
///
/// Nested references that are neither found in a root nor a well-known builtin
/// are left for [`crate::resolve`] to report as `unknown message` — the same
/// error the file-path loader gives.
pub fn resolve_ref(
    reference: &str,
    roots: &[PathBuf],
    default_kind: Kind,
) -> Result<(Ref, Program), ResolveRefError> {
    let r = parse_ref(reference, default_kind)?;
    let mut inputs = base_inputs(&r, roots)?;
    let mut have: BTreeSet<MsgId> = inputs.iter().map(|(id, _)| id.clone()).collect();

    // Breadth-first over referenced ids, loading each dependency file once.
    let mut queue: Vec<MsgId> = inputs
        .iter()
        .flat_map(|(id, spec)| referenced_ids(spec, id))
        .collect();
    while let Some(id) = queue.pop() {
        if have.contains(&id) || is_builtin(&id) {
            continue;
        }
        have.insert(id.clone());
        if let Some((dep_id, spec)) = load_dep(&id, roots)? {
            for next in referenced_ids(&spec, &dep_id) {
                if !have.contains(&next) {
                    queue.push(next);
                }
            }
            inputs.push((dep_id, spec));
        }
        // If not found and not builtin, resolve() will produce the precise error.
    }

    let program = crate::resolve(inputs).map_err(|e| err(e.to_string()))?;
    Ok((r, program))
}

/// Resolve and build a message [`DynamicType`] from a `pkg/msg/Name` reference.
pub fn load_message_ref(
    reference: &str,
    roots: &[PathBuf],
) -> Result<DynamicType, ResolveRefError> {
    let (r, program) = resolve_ref(reference, roots, Kind::Msg)?;
    DynamicType::from_program(&program, &MsgId::new(&r.package, &r.name))
        .map_err(|e| err(e.to_string()))
}

/// Resolve and build `(request, response)` [`DynamicType`]s from a `pkg/srv/Name`
/// reference.
pub fn load_service_ref(
    reference: &str,
    roots: &[PathBuf],
) -> Result<(DynamicType, DynamicType), ResolveRefError> {
    let (r, program) = resolve_ref(reference, roots, Kind::Srv)?;
    let req = MsgId::new(&r.package, format!("{}_Request", r.name));
    let resp = MsgId::new(&r.package, format!("{}_Response", r.name));
    Ok((
        DynamicType::from_program(&program, &req).map_err(|e| err(e.to_string()))?,
        DynamicType::from_program(&program, &resp).map_err(|e| err(e.to_string()))?,
    ))
}

/// The five wire types plus the three user-facing payload types an action client
/// needs, all built from one resolved `pkg/action/Name` reference.
pub struct ActionTypes {
    pub package: String,
    pub name: String,
    pub goal: DynamicType,
    pub result: DynamicType,
    pub feedback: DynamicType,
    pub send_goal_request: DynamicType,
    pub send_goal_response: DynamicType,
    pub get_result_request: DynamicType,
    pub get_result_response: DynamicType,
    pub feedback_message: DynamicType,
}

/// Resolve and build every [`DynamicType`] for a `pkg/action/Name` reference.
pub fn load_action_ref(reference: &str, roots: &[PathBuf]) -> Result<ActionTypes, ResolveRefError> {
    let (r, program) = resolve_ref(reference, roots, Kind::Action)?;
    let build = |suffix: &str| -> Result<DynamicType, ResolveRefError> {
        let id = MsgId::new(&r.package, format!("{}{suffix}", r.name));
        DynamicType::from_program(&program, &id).map_err(|e| err(e.to_string()))
    };
    Ok(ActionTypes {
        package: r.package.clone(),
        name: r.name.clone(),
        goal: build("_Goal")?,
        result: build("_Result")?,
        feedback: build("_Feedback")?,
        send_goal_request: build("_SendGoal_Request")?,
        send_goal_response: build("_SendGoal_Response")?,
        get_result_request: build("_GetResult_Request")?,
        get_result_response: build("_GetResult_Response")?,
        feedback_message: build("_FeedbackMessage")?,
    })
}

/// Convenience for tests/tools: the repo's `samples/` directory as a search root.
#[must_use]
pub fn samples_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("samples")
}

#[cfg(test)]
mod tests {
    use super::*;

    fn roots() -> Vec<PathBuf> {
        vec![samples_root()]
    }

    #[test]
    fn parse_ref_forms() {
        assert_eq!(
            parse_ref("geometry_msgs/msg/Twist", Kind::Msg).unwrap(),
            Ref {
                package: "geometry_msgs".into(),
                kind: Kind::Msg,
                name: "Twist".into(),
            }
        );
        assert_eq!(
            parse_ref("example_interfaces/srv/AddTwoInts.srv", Kind::Msg).unwrap(),
            Ref {
                package: "example_interfaces".into(),
                kind: Kind::Srv,
                name: "AddTwoInts".into(),
            }
        );
        // two-segment falls back to the default kind
        assert_eq!(
            parse_ref("std_msgs/String", Kind::Msg).unwrap().kind,
            Kind::Msg
        );
    }

    #[test]
    fn resolves_bundled_message_with_nested_deps() {
        // Twist -> Vector3 (same package), a real nested dependency.
        let ty = load_message_ref("geometry_msgs/msg/Twist", &roots()).unwrap();
        assert_eq!(ty.dds_type_name(), "geometry_msgs::msg::dds_::Twist_");
        // The closure must include the nested Vector3.
        assert!(
            ty.message_ids()
                .iter()
                .any(|id| id.package == "geometry_msgs" && id.name == "Vector3")
        );
    }

    #[test]
    fn resolves_service_reference() {
        let (req, resp) = load_service_ref("example_interfaces/srv/AddTwoInts", &roots()).unwrap();
        assert!(req.dds_type_name().contains("AddTwoInts_Request"));
        assert!(resp.dds_type_name().contains("AddTwoInts_Response"));
    }

    #[test]
    fn resolves_action_reference() {
        let a = load_action_ref("example_interfaces/action/Fibonacci", &roots()).unwrap();
        assert_eq!(a.package, "example_interfaces");
        assert_eq!(a.name, "Fibonacci");
        assert!(
            a.send_goal_request
                .dds_type_name()
                .contains("Fibonacci_SendGoal_Request")
        );
    }

    #[test]
    fn missing_reference_is_a_clean_error() {
        let e = load_message_ref("no_such_pkg/msg/Nope", &roots()).unwrap_err();
        assert!(e.to_string().contains("could not find"));
    }

    #[test]
    fn ament_share_layout_is_probed() {
        // Build a temp install prefix: <root>/share/foo_msgs/msg/Ping.msg
        let dir = std::env::temp_dir().join(format!("roswell_ws_{}", std::process::id()));
        let msg_dir = dir.join("share").join("foo_msgs").join("msg");
        std::fs::create_dir_all(&msg_dir).unwrap();
        std::fs::write(msg_dir.join("Ping.msg"), "int32 seq\nstring note\n").unwrap();
        let ty = load_message_ref("foo_msgs/msg/Ping", std::slice::from_ref(&dir)).unwrap();
        assert_eq!(ty.dds_type_name(), "foo_msgs::msg::dds_::Ping_");
        std::fs::remove_dir_all(&dir).ok();
    }
}

//! Plain C-ABI FFI for the roswell runtime: load ROS2 message/service types at
//! runtime, allocate their C-ABI struct memory, and publish/subscribe/serve/call
//! over real RTPS — all without a ROS installation, PyO3, or codegen.
//!
//! The surface is a small set of `rcm_`-prefixed `extern "C"` functions over
//! generation-counted [`handle`]s (a `u64` slot+generation, not a raw pointer).
//! Every entry point validates its handles — live slot, matching generation,
//! matching kind — so use-after-free, double-free, use-after-shutdown, and type
//! confusion become distinct `RCM_ERR_*` codes rather than undefined behaviour.
//! All message logic (parsing, layout, CDR, QoS, transport, correlation) lives
//! in [`roswell`] / [`roswell_ros2_compat`]; this crate is the thin, hardened C boundary.
//! The hand-written header in `include/roswell.h` is the contract.
//!
//! # Errors
//! Every fallible function returns an `int` status (`0` = success, negative =
//! one of the `RCM_ERR_*` codes) or a `0`/null sentinel; on failure a
//! human-readable message is available from [`rcm_last_error`] (thread-local,
//! valid until the next failing call on the same thread).
//!
//! # Thread-safety
//! See the "Threading" section of `include/roswell.h`. In short: the handle
//! table is internally synchronized, so any handle may be validated from any
//! thread; the intended pattern is one background thread driving
//! [`rcm_wait`]/[`rcm_take`] over a context's subscribers while another thread
//! publishes/calls on the same context.
#![allow(clippy::missing_safety_doc)] // safety contracts documented in roswell.h
#![allow(unsafe_op_in_unsafe_fn)] // each extern fn is one cohesive unsafe boundary
#![allow(clippy::too_many_lines)]

mod handle;

use std::cell::RefCell;
use std::ffi::{c_char, c_int, CStr, CString};
use std::fs::File;
use std::io::BufWriter;
use std::path::PathBuf;
use std::ptr;
use std::slice;
use std::sync::{Arc, Mutex, PoisonError};
use std::time::{Duration, Instant};

use roswell::dynamic::{
    assign_prim_sequence, assign_string, load_message, load_service, DynamicType, ElemKind,
    Multiplicity, TypeLayout,
};
use roswell::ir::MsgId;
use roswell::workspace::{load_action_ref, load_message_ref, load_service_ref, ActionTypes};

use roswell_ros2_compat::action::{
    ActionNames, CancelGoalRequest, CancelGoalResponse, GoalId, GoalInfoMsg, GoalStatus,
    GoalStatusArrayMsg, GoalStatusMsg,
};
use roswell_ros2_compat::codec::CdrMsg;
use roswell_ros2_compat::discovery::DiscoveryInfo;
use roswell_ros2_compat::graph::{ActionChannel, Graph};
use roswell_ros2_compat::msgs::{
    geometry_msgs__TransformStamped, std_msgs__Header, tf2_msgs__TFMessage, Endian, RosSequence,
    RosString,
};
use roswell_ros2_compat::parameters::{
    ParameterEvent, ParameterServer, ParameterType, ParameterValue,
};
use roswell_ros2_compat::qos::{
    DurabilityKind, IncompatiblePolicy, QosEvent, QosProfile, ReliabilityKind,
};
use roswell_ros2_compat::raw::{
    Compression, McapWriter, RawClient, RawDdsPublisher, RawDdsSubscriber, RawMsg, RawSample,
    RawSampleReader, RawService, RawSink,
};
use roswell_ros2_compat::tf::{TfBuffer, Transform as TfTransform};
use roswell_ros2_compat::time::Time;
use roswell_ros2_compat::transport::{Dds, DdsPub, Qos};

use handle::{HandleError, Kind, Payload, RcmHandle};

/// The ABI version. Bumped whenever the C signatures, struct layouts, or handle
/// encoding change incompatibly. `2` is the generation-counted-handle ABI;
/// version `1` was the original raw-opaque-pointer ABI (never shipped).
/// Mirrored as `RCM_ABI_VERSION` in the header; the loader checks the two match.
const RCM_ABI_VERSION: u32 = 2;

// ---- status codes (mirror RCM_ERR_* in roswell.h) ----------------------------

const RCM_ERR: c_int = -1; // generic / unspecified failure
const RCM_ERR_NULL_HANDLE: c_int = -2;
const RCM_ERR_STALE_HANDLE: c_int = -3;
const RCM_ERR_WRONG_KIND: c_int = -4;
const RCM_ERR_TYPE_MISMATCH: c_int = -5;
const RCM_ERR_ENCODE: c_int = -6;
const RCM_ERR_DECODE: c_int = -7;
const RCM_ERR_UNKNOWN_TOKEN: c_int = -8;

// ---- version / ABI ----------------------------------------------------------

/// The ABI version this library implements. A loader must refuse to bind if this
/// disagrees with the `RCM_ABI_VERSION` it was compiled against.
#[no_mangle]
pub extern "C" fn rcm_abi_version() -> u32 {
    RCM_ABI_VERSION
}

/// The crate version string (`CARGO_PKG_VERSION`), for diagnostics. Static and
/// NUL-terminated; the caller must **not** free it.
#[no_mangle]
pub extern "C" fn rcm_version_string() -> *const c_char {
    concat!(env!("CARGO_PKG_VERSION"), "\0")
        .as_ptr()
        .cast::<c_char>()
}

// ---- error reporting --------------------------------------------------------

thread_local! {
    static LAST_ERROR: RefCell<CString> = RefCell::new(CString::default());
}

fn set_error(msg: impl Into<Vec<u8>>) {
    let c = CString::new(msg).unwrap_or_else(|_| CString::new("error").unwrap());
    LAST_ERROR.with(|e| *e.borrow_mut() = c);
}

/// The most recent error message on the calling thread (empty if none). The
/// pointer is valid until the next failing `rcm_` call on this thread.
#[no_mangle]
pub extern "C" fn rcm_last_error() -> *const c_char {
    LAST_ERROR.with(|e| e.borrow().as_ptr())
}

/// Free a heap string returned by an `rcm_*_json` / `rcm_*_name` function.
/// Passing null is a no-op. Do **not** pass the pointer from [`rcm_last_error`]
/// or [`rcm_version_string`].
#[no_mangle]
pub unsafe extern "C" fn rcm_string_free(s: *mut c_char) {
    if !s.is_null() {
        drop(CString::from_raw(s));
    }
}

/// Turn a handle-validation failure into a status code, recording a precise
/// message for [`rcm_last_error`].
fn handle_err(e: HandleError, what: &str) -> c_int {
    let (code, desc) = match e {
        HandleError::Null => (RCM_ERR_NULL_HANDLE, "null handle"),
        HandleError::Stale => (
            RCM_ERR_STALE_HANDLE,
            "stale handle (freed, double-freed, or invalidated by rcm_shutdown)",
        ),
        HandleError::WrongKind => (RCM_ERR_WRONG_KIND, "handle is of the wrong kind"),
    };
    set_error(format!("{what}: {desc}"));
    code
}

// ---- handle payloads --------------------------------------------------------

/// Owns the DDS participant and (optionally) this participant's node-graph
/// advertisement. Everything else is created against its participant.
pub struct RcmCtx {
    dds: Dds,
    discovery: Option<DiscoveryInfo>,
}

/// A runtime-loaded message type plus its dependency-closure layout.
pub struct RcmType {
    ty: DynamicType,
}

/// A raw DDS publisher bound to one topic + type.
pub struct RcmPublisher {
    pubr: RawDdsPublisher,
    ty: DynamicType,
    dds_type: String,
    /// CDR byte length of the previous publish, used to right-size the next
    /// serialization buffer so a steady stream of same-typed samples reallocates
    /// zero times (see [`RawDdsPublisher::publish_loaned`]). `0` before the first
    /// publish.
    last_cdr_len: usize,
}

/// A raw DDS subscriber bound to one topic + type, with a one-slot readiness
/// buffer so [`rcm_wait`] can report a ready reader without discarding data.
pub struct RcmSubscriber {
    sub: RawDdsSubscriber,
    ty: DynamicType,
    dds_type: String,
    pending: Option<Vec<u8>>,
}

/// A runtime-typed service server.
pub struct RcmService {
    svc: RawService,
    req_ty: DynamicType,
    resp_ty: DynamicType,
    req_dds: String,
    resp_dds: String,
}

/// A runtime-typed service client.
pub struct RcmClient {
    client: RawClient,
    req_ty: DynamicType,
    resp_ty: DynamicType,
    req_dds: String,
    resp_dds: String,
}

/// A runtime-typed action client: the three action services (`send_goal`,
/// `get_result`, `cancel_goal`) as [`RawClient`]s plus the `feedback` topic as a
/// [`RawDdsSubscriber`], with every wire type resolved at runtime into a
/// [`DynamicType`]. Mirrors the generic `roswell_ros2_compat::action::ActionClient`, but
/// type-blind: goal/result/feedback layouts come from the loaded `.action`.
///
/// The `status` topic is intentionally not subscribed — `get_result` already
/// returns the final goal status, so the array is redundant for this client.
pub struct RcmActionClient {
    goal_ty: DynamicType,
    result_ty: DynamicType,
    feedback_ty: DynamicType,
    gr_resp_ty: DynamicType,
    fb_msg_ty: DynamicType,
    goal_dds: String,
    result_dds: String,
    feedback_dds: String,
    send_goal: RawClient,
    get_result: RawClient,
    cancel: RawClient,
    feedback: RawDdsSubscriber,
}

/// One message's C-ABI struct memory plus the type it was allocated from. The
/// `ptr` is owned by this handle: valid only while the handle is live, freed by
/// [`rcm_msg_free`]. The stored `dds_type` is checked against an endpoint's type
/// on every publish/take/call/reply so type confusion is a clean error.
pub struct RcmMsg {
    ty: DynamicType,
    dds_type: String,
    ptr: *mut u8,
}

// The `*mut u8` is owned exclusively by this message and only ever touched under
// the object's own mutex (or by the single thread the caller drives it from, per
// the header's threading contract), so the raw pointer is safe to move between
// threads inside the handle table.
unsafe impl Send for RcmMsg {}

/// A parameter server plus the background thread that pumps its request queues,
/// so `ros2 param get/set/list` clients are answered without the caller having
/// to spin. `declare`/`set`/`get`/`list` reach the server under its inner mutex.
pub struct RcmParamServer {
    server: Arc<Mutex<ParameterServer<DdsPub<ParameterEvent>>>>,
    stop: Arc<std::sync::atomic::AtomicBool>,
    thread: Option<std::thread::JoinHandle<()>>,
}

impl Drop for RcmParamServer {
    fn drop(&mut self) {
        self.stop.store(true, std::sync::atomic::Ordering::Relaxed);
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
    }
}

// ---- QoS --------------------------------------------------------------------

/// A C-ABI QoS descriptor. `reliability`: 0 = best-effort, 1 = reliable.
/// `durability`: 0 = volatile, 1 = transient-local. `keep_all`: nonzero keeps
/// every sample (ignoring `depth`). `deadline_ms`/`lifespan_ms`: negative means
/// unset.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct RcmQos {
    pub reliability: u8,
    pub durability: u8,
    pub keep_all: u8,
    pub depth: u32,
    pub deadline_ms: i64,
    pub lifespan_ms: i64,
}

fn qos_from_profile(p: QosProfile) -> RcmQos {
    RcmQos {
        reliability: u8::from(p.reliability == ReliabilityKind::Reliable),
        durability: u8::from(p.durability == DurabilityKind::TransientLocal),
        keep_all: u8::from(p.keep_all),
        depth: u32::try_from(p.depth).unwrap_or(u32::MAX),
        deadline_ms: p
            .deadline
            .map_or(-1, |d| i64::try_from(d.as_millis()).unwrap_or(i64::MAX)),
        lifespan_ms: p
            .lifespan
            .map_or(-1, |d| i64::try_from(d.as_millis()).unwrap_or(i64::MAX)),
    }
}

fn qos_to_profile(q: &RcmQos) -> QosProfile {
    QosProfile {
        reliability: if q.reliability == 0 {
            ReliabilityKind::BestEffort
        } else {
            ReliabilityKind::Reliable
        },
        durability: if q.durability == 0 {
            DurabilityKind::Volatile
        } else {
            DurabilityKind::TransientLocal
        },
        depth: q.depth as usize,
        keep_all: q.keep_all != 0,
        deadline: (q.deadline_ms >= 0).then(|| Duration::from_millis(q.deadline_ms as u64)),
        lifespan: (q.lifespan_ms >= 0).then(|| Duration::from_millis(q.lifespan_ms as u64)),
        liveliness: None,
    }
}

/// A QoS descriptor for one of the named ROS presets: `"default"`,
/// `"sensor_data"`, or `"latched"` (anything else falls back to `"default"`).
#[no_mangle]
pub unsafe extern "C" fn rcm_qos_preset(name: *const c_char) -> RcmQos {
    let preset = match cstr(name) {
        Some("sensor_data") => Qos::SensorData,
        Some("latched") => Qos::Latched,
        _ => Qos::Default,
    };
    qos_from_profile(QosProfile::from_preset(preset))
}

/// The effective [`QosProfile`] for an optional (nullable) descriptor pointer.
unsafe fn profile_or_default(qos: *const RcmQos) -> QosProfile {
    if qos.is_null() {
        QosProfile::from_preset(Qos::Default)
    } else {
        qos_to_profile(&*qos)
    }
}

// ---- lookup helpers ---------------------------------------------------------

/// Validate `h` as `kind` and lock its payload for the duration of a closure.
/// On a handle error, records the message and returns `err`.
fn with<T>(h: RcmHandle, kind: Kind, what: &str, err: T, f: impl FnOnce(&mut Payload) -> T) -> T {
    match handle::lookup(h, kind) {
        Ok(obj) => f(&mut handle::lock(&obj)),
        Err(e) => {
            handle_err(e, what);
            err
        }
    }
}

// ---- context ----------------------------------------------------------------

/// Create a DDS context (participant) on `domain`. Returns `0` on failure.
#[no_mangle]
pub extern "C" fn rcm_init(domain: c_int) -> RcmHandle {
    let domain = u16::try_from(domain).unwrap_or(0);
    let ctx = RcmCtx {
        dds: Dds::new(domain),
        discovery: None,
    };
    handle::insert(None, Payload::Ctx(ctx))
}

/// Destroy a context and everything it owns, and invalidate every handle
/// created against it (its publishers/subscribers/services/clients): each such
/// handle becomes `RCM_ERR_STALE_HANDLE` on next use rather than dangling. No-op
/// on an already-stale/null handle.
#[no_mangle]
pub extern "C" fn rcm_shutdown(ctx: RcmHandle) {
    handle::invalidate_children(ctx);
    let _ = handle::remove(ctx, Kind::Ctx);
}

// ---- types ------------------------------------------------------------------

/// Load a `.msg` (or `.srv`/`.action`/`.idl`) file plus optional dependency
/// files into a message type. `deps`/`n_deps` may be null/0. Returns `0` on
/// failure (see [`rcm_last_error`]).
#[no_mangle]
pub unsafe extern "C" fn rcm_type_load(
    msg_path: *const c_char,
    deps: *const *const c_char,
    n_deps: usize,
) -> RcmHandle {
    let Some(path) = cstr(msg_path) else {
        set_error("rcm_type_load: null message path");
        return 0;
    };
    let dep_paths = collect_paths(deps, n_deps);
    match load_message(path, &dep_paths) {
        Ok(ty) => handle::insert(None, Payload::Type(RcmType { ty })),
        Err(e) => {
            set_error(format!("rcm_type_load: {e}"));
            0
        }
    }
}

/// Load a `.srv` file into its request and response types, written to
/// `out_req`/`out_resp`. Returns 0 on success, negative on failure.
#[no_mangle]
pub unsafe extern "C" fn rcm_type_load_srv(
    srv_path: *const c_char,
    deps: *const *const c_char,
    n_deps: usize,
    out_req: *mut RcmHandle,
    out_resp: *mut RcmHandle,
) -> c_int {
    let Some(path) = cstr(srv_path) else {
        set_error("rcm_type_load_srv: null service path");
        return RCM_ERR;
    };
    if out_req.is_null() || out_resp.is_null() {
        set_error("rcm_type_load_srv: null out pointer");
        return RCM_ERR;
    }
    let dep_paths = collect_paths(deps, n_deps);
    match load_service(path, &dep_paths) {
        Ok((req, resp)) => {
            *out_req = handle::insert(None, Payload::Type(RcmType { ty: req }));
            *out_resp = handle::insert(None, Payload::Type(RcmType { ty: resp }));
            0
        }
        Err(e) => {
            set_error(format!("rcm_type_load_srv: {e}"));
            RCM_ERR
        }
    }
}

/// Resolve a message type by *reference* (`pkg/msg/Name`) against the workspace
/// `roots` (colcon package trees and/or ament `share/` install prefixes),
/// discovering nested cross-package dependencies. `roots`/`n_roots` may be
/// null/0. Returns `0` on failure (see [`rcm_last_error`]).
#[no_mangle]
pub unsafe extern "C" fn rcm_type_resolve(
    reference: *const c_char,
    roots: *const *const c_char,
    n_roots: usize,
) -> RcmHandle {
    let Some(reference) = cstr(reference) else {
        set_error("rcm_type_resolve: null reference");
        return 0;
    };
    let roots = collect_paths(roots, n_roots);
    match load_message_ref(reference, &roots) {
        Ok(ty) => handle::insert(None, Payload::Type(RcmType { ty })),
        Err(e) => {
            set_error(format!("rcm_type_resolve: {e}"));
            0
        }
    }
}

/// Resolve a service type by reference (`pkg/srv/Name`) against `roots` into its
/// request and response types (written to `out_req`/`out_resp`). Returns 0 on
/// success, negative on failure.
#[no_mangle]
pub unsafe extern "C" fn rcm_type_resolve_srv(
    reference: *const c_char,
    roots: *const *const c_char,
    n_roots: usize,
    out_req: *mut RcmHandle,
    out_resp: *mut RcmHandle,
) -> c_int {
    let Some(reference) = cstr(reference) else {
        set_error("rcm_type_resolve_srv: null reference");
        return RCM_ERR;
    };
    if out_req.is_null() || out_resp.is_null() {
        set_error("rcm_type_resolve_srv: null out pointer");
        return RCM_ERR;
    }
    let roots = collect_paths(roots, n_roots);
    match load_service_ref(reference, &roots) {
        Ok((req, resp)) => {
            *out_req = handle::insert(None, Payload::Type(RcmType { ty: req }));
            *out_resp = handle::insert(None, Payload::Type(RcmType { ty: resp }));
            0
        }
        Err(e) => {
            set_error(format!("rcm_type_resolve_srv: {e}"));
            RCM_ERR
        }
    }
}

/// JSON describing the whole dependency closure's C-ABI layout (see the header
/// for the schema). Owned string; free with [`rcm_string_free`]. Null on error.
#[no_mangle]
pub extern "C" fn rcm_type_layout_json(ty: RcmHandle) -> *mut c_char {
    with(
        ty,
        Kind::Type,
        "rcm_type_layout_json",
        ptr::null_mut(),
        |p| {
            let Payload::Type(t) = p else {
                return ptr::null_mut();
            };
            into_c_string(layout_json(&t.ty))
        },
    )
}

/// The ROS2 DDS type name (e.g. `std_msgs::msg::dds_::String_`). Owned string;
/// free with [`rcm_string_free`]. Null on error.
#[no_mangle]
pub extern "C" fn rcm_type_dds_name(ty: RcmHandle) -> *mut c_char {
    with(ty, Kind::Type, "rcm_type_dds_name", ptr::null_mut(), |p| {
        let Payload::Type(t) = p else {
            return ptr::null_mut();
        };
        into_c_string(t.ty.dds_type_name())
    })
}

/// Free a type handle. Returns 0 on success, negative if the handle is already
/// stale or of the wrong kind.
#[no_mangle]
pub extern "C" fn rcm_type_free(ty: RcmHandle) -> c_int {
    free_handle(ty, Kind::Type, "rcm_type_free")
}

// ---- messages ---------------------------------------------------------------

/// Allocate zeroed, correctly-aligned memory for one message of type `ty` and
/// fill it with its `.msg` defaults, returning a message handle. The struct
/// memory pointer for ctypes/read access comes from [`rcm_msg_data`]. Free with
/// [`rcm_msg_free`]. Returns `0` on error.
#[no_mangle]
pub extern "C" fn rcm_msg_alloc(ty: RcmHandle) -> RcmHandle {
    let obj = match handle::lookup(ty, Kind::Type) {
        Ok(o) => o,
        Err(e) => {
            handle_err(e, "rcm_msg_alloc");
            return 0;
        }
    };
    let Payload::Type(t) = &*handle::lock(&obj) else {
        return 0;
    };
    let ty = t.ty.clone();
    let dds_type = ty.dds_type_name();
    // SAFETY: alloc_zeroed returns a fresh allocation sized/aligned for this
    // type; init_default writes only within it.
    let ptr = ty.alloc_zeroed();
    unsafe { ty.init_default(ptr) };
    handle::insert(None, Payload::Msg(RcmMsg { ty, dds_type, ptr }))
}

/// The base pointer of a message's C-ABI struct memory, for ctypes field access.
/// Valid only while the message handle is live (until [`rcm_msg_free`]). Null on
/// error.
#[no_mangle]
pub extern "C" fn rcm_msg_data(msg: RcmHandle) -> *mut u8 {
    with(msg, Kind::Msg, "rcm_msg_data", ptr::null_mut(), |p| {
        let Payload::Msg(m) = p else {
            return ptr::null_mut();
        };
        m.ptr
    })
}

/// Free every string/sequence buffer the message owns (recursively), leaving a
/// re-usable empty message. Idempotent. Returns 0 on success, negative on a
/// handle error.
#[no_mangle]
pub extern "C" fn rcm_msg_fini(msg: RcmHandle) -> c_int {
    with(msg, Kind::Msg, "rcm_msg_fini", RCM_ERR, |p| {
        let Payload::Msg(m) = p else { return RCM_ERR };
        // SAFETY: `ptr` was allocated for `ty` and is live while the handle is.
        unsafe { m.ty.fini(m.ptr) };
        0
    })
}

/// Finalize (if needed) and free a message's backing allocation, invalidating
/// its handle. Returns 0 on success, negative if already freed or wrong kind.
#[no_mangle]
pub extern "C" fn rcm_msg_free(msg: RcmHandle) -> c_int {
    let obj = match handle::remove(msg, Kind::Msg) {
        Ok(o) => o,
        Err(e) => return handle_err(e, "rcm_msg_free"),
    };
    let Payload::Msg(m) = &mut *handle::lock(&obj) else {
        return RCM_ERR;
    };
    if !m.ptr.is_null() {
        // SAFETY: `ptr` was produced by alloc_zeroed for `ty`; fini then dealloc
        // releases every owned buffer and the backing allocation exactly once.
        unsafe {
            m.ty.fini(m.ptr);
            m.ty.dealloc(m.ptr);
        }
        m.ptr = ptr::null_mut();
    }
    0
}

/// Overwrite a **primitive** `{data,size,capacity}` sequence triple located at
/// byte `offset` within message `msg` with `count` elements of `elem_size` bytes
/// copied from `src`, freeing any buffer it previously owned. Allocation stays
/// in Rust so [`rcm_msg_fini`] can free it. Returns 0. Primitive elements only.
#[no_mangle]
pub unsafe extern "C" fn rcm_seq_assign(
    msg: RcmHandle,
    offset: usize,
    elem_size: usize,
    elem_align: usize,
    src: *const u8,
    count: usize,
) -> c_int {
    with(msg, Kind::Msg, "rcm_seq_assign", RCM_ERR, |p| {
        let Payload::Msg(m) = p else { return RCM_ERR };
        let Some(triple) = m.field_ptr(offset) else {
            set_error("rcm_seq_assign: offset out of range for message");
            return RCM_ERR;
        };
        assign_prim_sequence(triple, elem_size, elem_align, src, count);
        0
    })
}

/// Overwrite a ROS string `{data,size,capacity}` triple located at byte `offset`
/// within message `msg` with the UTF-8 bytes `src[..len]`, freeing any buffer it
/// previously owned. Allocation stays in Rust so [`rcm_msg_fini`] can free it.
/// Returns 0 on success.
#[no_mangle]
pub unsafe extern "C" fn rcm_str_assign(
    msg: RcmHandle,
    offset: usize,
    src: *const u8,
    len: usize,
) -> c_int {
    with(msg, Kind::Msg, "rcm_str_assign", RCM_ERR, |p| {
        let Payload::Msg(m) = p else { return RCM_ERR };
        let Some(triple) = m.field_ptr(offset) else {
            set_error("rcm_str_assign: offset out of range for message");
            return RCM_ERR;
        };
        assign_string(triple, src, len);
        0
    })
}

impl RcmMsg {
    /// The address of a `{data,size,capacity}` triple at `offset`, bounds-checked
    /// against the message size (a triple is three pointer-words).
    fn field_ptr(&self, offset: usize) -> Option<*mut u8> {
        let triple_len = 3 * std::mem::size_of::<usize>();
        if self.ptr.is_null() || offset.checked_add(triple_len)? > self.ty.size() {
            return None;
        }
        // SAFETY: offset + triple_len <= size, so the triple lies within the
        // allocation.
        Some(unsafe { self.ptr.add(offset) })
    }
}

// ---- publish / subscribe ----------------------------------------------------

/// Create a publisher on `topic` for messages of type `ty`, with `qos` (null =
/// default preset). Returns `0` on error.
#[no_mangle]
pub unsafe extern "C" fn rcm_publisher(
    ctx: RcmHandle,
    topic: *const c_char,
    ty: RcmHandle,
    qos: *const RcmQos,
) -> RcmHandle {
    let Some(topic) = cstr(topic) else {
        set_error("rcm_publisher: null topic");
        return 0;
    };
    let Ok(ty) = clone_type(ty, "rcm_publisher") else {
        return 0;
    };
    let policies = profile_or_default(qos).policies();
    let dds_type = ty.dds_type_name();
    with(ctx, Kind::Ctx, "rcm_publisher", 0, |p| {
        let Payload::Ctx(c) = p else { return 0 };
        let pubr = RawDdsPublisher::with_policies(c.dds.participant(), topic, &dds_type, &policies);
        handle::insert(
            Some(ctx),
            Payload::Publisher(RcmPublisher {
                pubr,
                ty: ty.clone(),
                dds_type: dds_type.clone(),
                last_cdr_len: 0,
            }),
        )
    })
}

/// Encode message `msg` and publish it. The message's type must match the
/// publisher's (else `RCM_ERR_TYPE_MISMATCH`). Returns 0 on success, negative on
/// error.
#[no_mangle]
pub extern "C" fn rcm_publish(pubh: RcmHandle, msg: RcmHandle) -> c_int {
    let (pub_obj, msg_obj) = match dual(pubh, Kind::Publisher, msg, "rcm_publish") {
        Ok(v) => v,
        Err(code) => return code,
    };
    let mut pub_g = handle::lock(&pub_obj);
    let mut msg_g = handle::lock(&msg_obj);
    let (Payload::Publisher(p), Payload::Msg(m)) = (&mut *pub_g, &mut *msg_g) else {
        return RCM_ERR;
    };
    if m.dds_type != p.dds_type {
        set_error(format!(
            "rcm_publish: message type {} does not match publisher type {}",
            m.dds_type, p.dds_type
        ));
        return RCM_ERR_TYPE_MISMATCH;
    }
    // SAFETY: `m.ptr` is a live message of `p.ty` (checked by dds_type match).
    // Serialize straight into the transport-bound buffer, right-sized from the
    // previous sample so a steady stream reallocates zero times, and hand rustdds
    // the very same allocation — no intermediate `RawMsg`, no header-strip copy
    // (see `RawDdsPublisher::publish_loaned`).
    let (ty, ptr) = (&p.ty, m.ptr);
    let len = p.pubr.publish_loaned(p.last_cdr_len, |buf| {
        // SAFETY: as above — `ptr` is a live message matching `ty`.
        unsafe { ty.encode_into(ptr, buf) }
    });
    p.last_cdr_len = len;
    0
}

/// Pending publisher-side QoS events as a JSON array (see header). Owned string;
/// free with [`rcm_string_free`]. Null on error.
#[no_mangle]
pub extern "C" fn rcm_publisher_events(pubh: RcmHandle) -> *mut c_char {
    with(
        pubh,
        Kind::Publisher,
        "rcm_publisher_events",
        ptr::null_mut(),
        |p| {
            let Payload::Publisher(pb) = p else {
                return ptr::null_mut();
            };
            into_c_string(events_json(&pb.pubr.poll_events()))
        },
    )
}

/// Free a publisher handle. Returns 0 on success, negative if already stale.
#[no_mangle]
pub extern "C" fn rcm_publisher_free(pubh: RcmHandle) -> c_int {
    free_handle(pubh, Kind::Publisher, "rcm_publisher_free")
}

/// Create a subscriber on `topic` for messages of type `ty`, with `qos` (null =
/// default preset). Returns `0` on error.
#[no_mangle]
pub unsafe extern "C" fn rcm_subscriber(
    ctx: RcmHandle,
    topic: *const c_char,
    ty: RcmHandle,
    qos: *const RcmQos,
) -> RcmHandle {
    let Some(topic) = cstr(topic) else {
        set_error("rcm_subscriber: null topic");
        return 0;
    };
    let Ok(ty) = clone_type(ty, "rcm_subscriber") else {
        return 0;
    };
    let policies = profile_or_default(qos).policies();
    let dds_type = ty.dds_type_name();
    with(ctx, Kind::Ctx, "rcm_subscriber", 0, |p| {
        let Payload::Ctx(c) = p else { return 0 };
        let sub = RawDdsSubscriber::with_policies(c.dds.participant(), topic, &dds_type, &policies);
        handle::insert(
            Some(ctx),
            Payload::Subscriber(RcmSubscriber {
                sub,
                ty: ty.clone(),
                dds_type: dds_type.clone(),
                pending: None,
            }),
        )
    })
}

/// Take the next message into message `out` (reusable; its previous contents are
/// finalized first). The message's type must match the subscriber's (else
/// `RCM_ERR_TYPE_MISMATCH`). Returns 1 if a message was decoded, 0 if none was
/// available, negative on error.
#[no_mangle]
pub extern "C" fn rcm_take(sub: RcmHandle, out: RcmHandle) -> c_int {
    let (sub_obj, msg_obj) = match dual(sub, Kind::Subscriber, out, "rcm_take") {
        Ok(v) => v,
        Err(code) => return code,
    };
    let mut sub_g = handle::lock(&sub_obj);
    let mut msg_g = handle::lock(&msg_obj);
    let (Payload::Subscriber(s), Payload::Msg(m)) = (&mut *sub_g, &mut *msg_g) else {
        return RCM_ERR;
    };
    if m.dds_type != s.dds_type {
        set_error(format!(
            "rcm_take: message type {} does not match subscriber type {}",
            m.dds_type, s.dds_type
        ));
        return RCM_ERR_TYPE_MISMATCH;
    }
    let Some(cdr) = s
        .pending
        .take()
        .or_else(|| s.sub.take().map(|msg| msg.into_cdr().to_vec()))
    else {
        return 0;
    };
    // SAFETY: `m.ptr` is a live message of `s.ty` (checked by dds_type match).
    unsafe {
        s.ty.fini(m.ptr);
        match s.ty.decode(&cdr, m.ptr) {
            Ok(()) => 1,
            Err(e) => {
                set_error(format!("rcm_take: decode failed: {e}"));
                RCM_ERR_DECODE
            }
        }
    }
}

/// Block up to `timeout_ms` until one of `subs[0..n]` has a message ready,
/// returning its index, or -1 on timeout/error. The readied message is buffered
/// and delivered by the next [`rcm_take`] on that subscriber. Efficiently
/// multiplexes many subscribers on one thread with a short-sleep poll; the
/// handle table is locked only in brief non-blocking bursts (never across the
/// sleep), so a concurrent publisher on another thread is not stalled.
#[no_mangle]
pub unsafe extern "C" fn rcm_wait(subs: *const RcmHandle, n: usize, timeout_ms: c_int) -> c_int {
    if subs.is_null() || n == 0 {
        return -1;
    }
    let list = slice::from_raw_parts(subs, n);
    let timeout = Duration::from_millis(u64::try_from(timeout_ms).unwrap_or(0));
    let deadline = Instant::now() + timeout;
    loop {
        for (i, &h) in list.iter().enumerate() {
            let Ok(obj) = handle::lookup(h, Kind::Subscriber) else {
                continue;
            };
            let mut g = handle::lock(&obj);
            let Payload::Subscriber(s) = &mut *g else {
                continue;
            };
            if s.pending.is_some() {
                return i as c_int;
            }
            if let Some(msg) = s.sub.take() {
                s.pending = Some(msg.into_cdr().to_vec());
                return i as c_int;
            }
        }
        if Instant::now() >= deadline {
            return -1;
        }
        std::thread::sleep(Duration::from_millis(1));
    }
}

/// Pending subscriber-side QoS events as a JSON array (see header). Owned
/// string; free with [`rcm_string_free`]. Null on error.
#[no_mangle]
pub extern "C" fn rcm_subscriber_events(sub: RcmHandle) -> *mut c_char {
    with(
        sub,
        Kind::Subscriber,
        "rcm_subscriber_events",
        ptr::null_mut(),
        |p| {
            let Payload::Subscriber(s) = p else {
                return ptr::null_mut();
            };
            into_c_string(events_json(&s.sub.poll_events()))
        },
    )
}

/// Free a subscriber handle. Returns 0 on success, negative if already stale.
#[no_mangle]
pub extern "C" fn rcm_subscriber_free(sub: RcmHandle) -> c_int {
    free_handle(sub, Kind::Subscriber, "rcm_subscriber_free")
}

// ---- services ---------------------------------------------------------------

/// Create a service server named `name` with request/response types. Returns
/// `0` on error.
#[no_mangle]
pub unsafe extern "C" fn rcm_service(
    ctx: RcmHandle,
    name: *const c_char,
    req_ty: RcmHandle,
    resp_ty: RcmHandle,
) -> RcmHandle {
    let Some(name) = cstr(name) else {
        set_error("rcm_service: null name");
        return 0;
    };
    let (Ok(req_ty), Ok(resp_ty)) = (
        clone_type(req_ty, "rcm_service"),
        clone_type(resp_ty, "rcm_service"),
    ) else {
        return 0;
    };
    let (req_dds, resp_dds) = (req_ty.dds_type_name(), resp_ty.dds_type_name());
    with(ctx, Kind::Ctx, "rcm_service", 0, |p| {
        let Payload::Ctx(c) = p else { return 0 };
        let svc = RawService::new(&c.dds, name, &req_dds, &resp_dds);
        handle::insert(
            Some(ctx),
            Payload::Service(RcmService {
                svc,
                req_ty: req_ty.clone(),
                resp_ty: resp_ty.clone(),
                req_dds: req_dds.clone(),
                resp_dds: resp_dds.clone(),
            }),
        )
    })
}

/// Take the next pending request into message `out_req`, writing a correlation
/// token to `out_token`. The message's type must match the service's request
/// type (else `RCM_ERR_TYPE_MISMATCH`). Returns 1 if a request was decoded, 0 if
/// none, negative on error. Answer with [`rcm_service_send_reply`].
#[no_mangle]
pub unsafe extern "C" fn rcm_service_take_request(
    svc: RcmHandle,
    out_req: RcmHandle,
    out_token: *mut u64,
) -> c_int {
    if out_token.is_null() {
        set_error("rcm_service_take_request: null token pointer");
        return RCM_ERR;
    }
    let (svc_obj, msg_obj) = match dual(svc, Kind::Service, out_req, "rcm_service_take_request") {
        Ok(v) => v,
        Err(code) => return code,
    };
    let mut svc_g = handle::lock(&svc_obj);
    let mut msg_g = handle::lock(&msg_obj);
    let (Payload::Service(s), Payload::Msg(m)) = (&mut *svc_g, &mut *msg_g) else {
        return RCM_ERR;
    };
    if m.dds_type != s.req_dds {
        set_error("rcm_service_take_request: message type does not match request type");
        return RCM_ERR_TYPE_MISMATCH;
    }
    let Some((cdr, token)) = s.svc.take_request() else {
        return 0;
    };
    s.req_ty.fini(m.ptr);
    match s.req_ty.decode(&cdr, m.ptr) {
        Ok(()) => {
            *out_token = token;
            1
        }
        Err(e) => {
            set_error(format!("rcm_service_take_request: decode failed: {e}"));
            RCM_ERR_DECODE
        }
    }
}

/// Encode message `resp` and send it as the reply to `token`. The message's type
/// must match the service's response type (else `RCM_ERR_TYPE_MISMATCH`). Returns
/// 0 on success, negative if the token is unknown or on error.
#[no_mangle]
pub extern "C" fn rcm_service_send_reply(svc: RcmHandle, token: u64, resp: RcmHandle) -> c_int {
    let (svc_obj, msg_obj) = match dual(svc, Kind::Service, resp, "rcm_service_send_reply") {
        Ok(v) => v,
        Err(code) => return code,
    };
    let mut svc_g = handle::lock(&svc_obj);
    let mut msg_g = handle::lock(&msg_obj);
    let (Payload::Service(s), Payload::Msg(m)) = (&mut *svc_g, &mut *msg_g) else {
        return RCM_ERR;
    };
    if m.dds_type != s.resp_dds {
        set_error("rcm_service_send_reply: message type does not match response type");
        return RCM_ERR_TYPE_MISMATCH;
    }
    // SAFETY: `m.ptr` is a live message of `s.resp_ty` (checked by dds_type).
    let reply = match unsafe { s.resp_ty.encode(m.ptr) } {
        Ok(reply) => reply,
        Err(e) => {
            set_error(format!("rcm_service_send_reply: encode failed: {e}"));
            return RCM_ERR_ENCODE;
        }
    };
    if s.svc.send_reply(token, reply) {
        0
    } else {
        set_error("rcm_service_send_reply: unknown reply token");
        RCM_ERR_UNKNOWN_TOKEN
    }
}

/// Free a service handle. Returns 0 on success, negative if already stale.
#[no_mangle]
pub extern "C" fn rcm_service_free(svc: RcmHandle) -> c_int {
    free_handle(svc, Kind::Service, "rcm_service_free")
}

/// Create a service client for `name` with request/response types. Returns `0`
/// on error.
#[no_mangle]
pub unsafe extern "C" fn rcm_client(
    ctx: RcmHandle,
    name: *const c_char,
    req_ty: RcmHandle,
    resp_ty: RcmHandle,
) -> RcmHandle {
    let Some(name) = cstr(name) else {
        set_error("rcm_client: null name");
        return 0;
    };
    let (Ok(req_ty), Ok(resp_ty)) = (
        clone_type(req_ty, "rcm_client"),
        clone_type(resp_ty, "rcm_client"),
    ) else {
        return 0;
    };
    let (req_dds, resp_dds) = (req_ty.dds_type_name(), resp_ty.dds_type_name());
    with(ctx, Kind::Ctx, "rcm_client", 0, |p| {
        let Payload::Ctx(c) = p else { return 0 };
        let client = RawClient::new(&c.dds, name, &req_dds, &resp_dds);
        handle::insert(
            Some(ctx),
            Payload::Client(RcmClient {
                client,
                req_ty: req_ty.clone(),
                resp_ty: resp_ty.clone(),
                req_dds: req_dds.clone(),
                resp_dds: resp_dds.clone(),
            }),
        )
    })
}

/// Encode message `req`, send it, and block up to `timeout_ms` for the
/// correlated reply, decoded into message `resp_out` (finalized first). Both
/// messages' types must match the client's request/response types (else
/// `RCM_ERR_TYPE_MISMATCH`). Returns 1 on reply, 0 on timeout, negative on error.
#[no_mangle]
pub extern "C" fn rcm_call(
    client: RcmHandle,
    req: RcmHandle,
    resp_out: RcmHandle,
    timeout_ms: c_int,
) -> c_int {
    // Three handles: validate and clone all up front, then lock.
    let client_obj = match handle::lookup(client, Kind::Client) {
        Ok(o) => o,
        Err(e) => return handle_err(e, "rcm_call"),
    };
    let req_obj = match handle::lookup(req, Kind::Msg) {
        Ok(o) => o,
        Err(e) => return handle_err(e, "rcm_call: request"),
    };
    let resp_obj = match handle::lookup(resp_out, Kind::Msg) {
        Ok(o) => o,
        Err(e) => return handle_err(e, "rcm_call: response"),
    };
    let mut client_g = handle::lock(&client_obj);
    let mut req_g = handle::lock(&req_obj);
    let mut resp_g = handle::lock(&resp_obj);
    let (Payload::Client(c), Payload::Msg(rq), Payload::Msg(rp)) =
        (&mut *client_g, &mut *req_g, &mut *resp_g)
    else {
        return RCM_ERR;
    };
    if rq.dds_type != c.req_dds || rp.dds_type != c.resp_dds {
        set_error("rcm_call: message type does not match client request/response type");
        return RCM_ERR_TYPE_MISMATCH;
    }
    // SAFETY: `rq.ptr`/`rp.ptr` are live messages of the client's types.
    let request = match unsafe { c.req_ty.encode(rq.ptr) } {
        Ok(request) => request,
        Err(e) => {
            set_error(format!("rcm_call: encode failed: {e}"));
            return RCM_ERR_ENCODE;
        }
    };
    let timeout = Duration::from_millis(u64::try_from(timeout_ms).unwrap_or(0));
    let Some(reply) = c.client.call(&request, timeout) else {
        return 0;
    };
    unsafe {
        c.resp_ty.fini(rp.ptr);
        match c.resp_ty.decode(&reply, rp.ptr) {
            Ok(()) => 1,
            Err(e) => {
                set_error(format!("rcm_call: decode failed: {e}"));
                RCM_ERR_DECODE
            }
        }
    }
}

/// Free a client handle. Returns 0 on success, negative if already stale.
#[no_mangle]
pub extern "C" fn rcm_client_free(client: RcmHandle) -> c_int {
    free_handle(client, Kind::Client, "rcm_client_free")
}

// ---- actions ----------------------------------------------------------------

/// XCDR1 little-endian encapsulation header (see `roswell::cdr`).
const CDR_LE_HEADER: [u8; 4] = [0x00, 0x01, 0x00, 0x00];
/// Length of a ROS action `goal_id` (a `unique_identifier_msgs/UUID`).
const GOAL_ID_LEN: usize = 16;

/// The byte offset of the field named `name` in a type's root message layout.
fn field_offset(ty: &DynamicType, name: &str) -> Option<usize> {
    ty.layout()
        .fields
        .iter()
        .find(|f| f.name == name)
        .map(|f| f.offset)
}

/// Create a runtime-typed action client for `action_name`, resolving the action
/// type from its `reference` (`pkg/action/Name`) against the workspace `roots`.
/// Returns `0` on error. Free with [`rcm_action_client_free`].
#[no_mangle]
pub unsafe extern "C" fn rcm_action_client(
    ctx: RcmHandle,
    action_name: *const c_char,
    reference: *const c_char,
    roots: *const *const c_char,
    n_roots: usize,
) -> RcmHandle {
    let (Some(action_name), Some(reference)) = (cstr(action_name), cstr(reference)) else {
        set_error("rcm_action_client: null argument");
        return 0;
    };
    let root_paths = collect_paths(roots, n_roots);
    let at: ActionTypes = match load_action_ref(reference, &root_paths) {
        Ok(at) => at,
        Err(e) => {
            set_error(format!("rcm_action_client: {e}"));
            return 0;
        }
    };
    let names = ActionNames::new(action_name);
    let policies = QosProfile::from_preset(Qos::Default).policies();
    with(ctx, Kind::Ctx, "rcm_action_client", 0, |p| {
        let Payload::Ctx(c) = p else { return 0 };
        let send_goal = RawClient::new(
            &c.dds,
            &names.send_goal,
            &at.send_goal_request.dds_type_name(),
            &at.send_goal_response.dds_type_name(),
        );
        let get_result = RawClient::new(
            &c.dds,
            &names.get_result,
            &at.get_result_request.dds_type_name(),
            &at.get_result_response.dds_type_name(),
        );
        let cancel = RawClient::new(
            &c.dds,
            &names.cancel_goal,
            <CancelGoalRequest as CdrMsg>::TYPE_NAME,
            <CancelGoalResponse as CdrMsg>::TYPE_NAME,
        );
        let feedback = RawDdsSubscriber::with_policies(
            c.dds.participant(),
            &names.feedback,
            &at.feedback_message.dds_type_name(),
            &policies,
        );
        handle::insert(
            Some(ctx),
            Payload::ActionClient(Box::new(RcmActionClient {
                goal_dds: at.goal.dds_type_name(),
                result_dds: at.result.dds_type_name(),
                feedback_dds: at.feedback.dds_type_name(),
                goal_ty: at.goal.clone(),
                result_ty: at.result.clone(),
                feedback_ty: at.feedback.clone(),
                gr_resp_ty: at.get_result_response.clone(),
                fb_msg_ty: at.feedback_message.clone(),
                send_goal,
                get_result,
                cancel,
                feedback,
            })),
        )
    })
}

/// Number of type handles [`rcm_action_load`] writes.
const ACTION_TYPE_COUNT: usize = 8;

/// Resolve an action reference (`pkg/action/Name`) against `roots` into its eight
/// component types, written to `out[0..8]` in this order: goal, result, feedback,
/// send_goal_request, send_goal_response, get_result_request, get_result_response,
/// feedback_message. Each is an owned type handle (free with [`rcm_type_free`]).
/// Standing up an action *server* from the plain service/topic primitives needs
/// the wrapper types; a client should use [`rcm_action_client`]. Returns 0 on
/// success, negative on failure.
#[no_mangle]
pub unsafe extern "C" fn rcm_action_load(
    reference: *const c_char,
    roots: *const *const c_char,
    n_roots: usize,
    out: *mut RcmHandle,
) -> c_int {
    let Some(reference) = cstr(reference) else {
        set_error("rcm_action_load: null reference");
        return RCM_ERR;
    };
    if out.is_null() {
        set_error("rcm_action_load: null out pointer");
        return RCM_ERR;
    }
    let root_paths = collect_paths(roots, n_roots);
    let at = match load_action_ref(reference, &root_paths) {
        Ok(at) => at,
        Err(e) => {
            set_error(format!("rcm_action_load: {e}"));
            return RCM_ERR;
        }
    };
    let handles = [
        at.goal,
        at.result,
        at.feedback,
        at.send_goal_request,
        at.send_goal_response,
        at.get_result_request,
        at.get_result_response,
        at.feedback_message,
    ]
    .map(|ty| handle::insert(None, Payload::Type(RcmType { ty })));
    let dst = slice::from_raw_parts_mut(out, ACTION_TYPE_COUNT);
    dst.copy_from_slice(&handles);
    0
}

/// The goal payload type (for allocating a goal message to send). Owned type
/// handle; free with [`rcm_type_free`]. `0` on error.
#[no_mangle]
pub extern "C" fn rcm_action_goal_type(client: RcmHandle) -> RcmHandle {
    action_type_handle(client, "rcm_action_goal_type", |a| &a.goal_ty)
}

/// The result payload type (for allocating a result message to receive). Owned
/// type handle; free with [`rcm_type_free`]. `0` on error.
#[no_mangle]
pub extern "C" fn rcm_action_result_type(client: RcmHandle) -> RcmHandle {
    action_type_handle(client, "rcm_action_result_type", |a| &a.result_ty)
}

/// The feedback payload type (for allocating a feedback message to receive).
/// Owned type handle; free with [`rcm_type_free`]. `0` on error.
#[no_mangle]
pub extern "C" fn rcm_action_feedback_type(client: RcmHandle) -> RcmHandle {
    action_type_handle(client, "rcm_action_feedback_type", |a| &a.feedback_ty)
}

fn action_type_handle(
    client: RcmHandle,
    what: &str,
    pick: impl Fn(&RcmActionClient) -> &DynamicType,
) -> RcmHandle {
    with(client, Kind::ActionClient, what, 0, |p| {
        let Payload::ActionClient(a) = p else {
            return 0;
        };
        let ty = pick(a).clone();
        handle::insert(None, Payload::Type(RcmType { ty }))
    })
}

/// True (1) once all three action services (`send_goal`, `get_result`,
/// `cancel_goal`) have discovered the server, 0 if not yet, negative on error.
#[no_mangle]
pub extern "C" fn rcm_action_server_ready(client: RcmHandle) -> c_int {
    with(
        client,
        Kind::ActionClient,
        "rcm_action_server_ready",
        RCM_ERR,
        |p| {
            let Payload::ActionClient(a) = p else {
                return RCM_ERR;
            };
            // No short-circuit: each probe also drains that client's status queue.
            let sg = a.send_goal.server_is_ready();
            let gr = a.get_result.server_is_ready();
            let cg = a.cancel.server_is_ready();
            c_int::from(sg && gr && cg)
        },
    )
}

/// Send goal message `goal` under a freshly generated goal id, blocking up to
/// `timeout_ms` for the server's accept/reject reply. Writes the 16-byte goal id
/// to `out_goal_id` and `1`/`0` accepted to `out_accepted`. Returns 1 on reply,
/// 0 on timeout, negative on error (`RCM_ERR_TYPE_MISMATCH` if `goal` is not of
/// the action's goal type).
#[no_mangle]
pub unsafe extern "C" fn rcm_action_send_goal(
    client: RcmHandle,
    goal: RcmHandle,
    timeout_ms: c_int,
    out_goal_id: *mut u8,
    out_accepted: *mut u8,
) -> c_int {
    if out_goal_id.is_null() || out_accepted.is_null() {
        set_error("rcm_action_send_goal: null out pointer");
        return RCM_ERR;
    }
    let (client_obj, msg_obj) = match dual(client, Kind::ActionClient, goal, "rcm_action_send_goal")
    {
        Ok(v) => v,
        Err(code) => return code,
    };
    let mut client_g = handle::lock(&client_obj);
    let mut msg_g = handle::lock(&msg_obj);
    let (Payload::ActionClient(a), Payload::Msg(m)) = (&mut *client_g, &mut *msg_g) else {
        return RCM_ERR;
    };
    if m.dds_type != a.goal_dds {
        set_error("rcm_action_send_goal: message type does not match the action goal type");
        return RCM_ERR_TYPE_MISMATCH;
    }
    let goal_id = GoalId::generate();
    // SAFETY: `m.ptr` is a live message of `a.goal_ty` (checked by dds_type).
    let goal_cdr = match unsafe { a.goal_ty.encode(m.ptr) } {
        Ok(cdr) => cdr,
        Err(e) => {
            set_error(format!("rcm_action_send_goal: encode failed: {e}"));
            return RCM_ERR_ENCODE;
        }
    };
    // A SendGoal_Request body is `uuid[16]` then the goal; since 16 is 8-aligned,
    // the goal body's CDR padding is identical whether at offset 0 or 16, so we
    // splice header + goal_id + goal-body rather than re-encode the wrapper.
    let mut req = Vec::with_capacity(GOAL_ID_LEN + goal_cdr.len());
    req.extend_from_slice(&goal_cdr[..4]);
    req.extend_from_slice(&goal_id.0);
    req.extend_from_slice(&goal_cdr[4..]);
    let timeout = Duration::from_millis(u64::try_from(timeout_ms).unwrap_or(0));
    let Some(resp) = a.send_goal.call(&req, timeout) else {
        return 0;
    };
    // SendGoal_Response body byte 0 is `bool accepted`.
    let accepted = resp.get(4).copied().unwrap_or(0);
    ptr::copy_nonoverlapping(goal_id.0.as_ptr(), out_goal_id, GOAL_ID_LEN);
    *out_accepted = u8::from(accepted != 0);
    1
}

/// Request the result for `goal_id` (16 bytes), blocking up to `timeout_ms`,
/// decoding the result payload into message `result` and writing the final goal
/// status to `out_status`. Returns 1 on reply, 0 on timeout, negative on error.
#[no_mangle]
pub unsafe extern "C" fn rcm_action_get_result(
    client: RcmHandle,
    goal_id: *const u8,
    result: RcmHandle,
    timeout_ms: c_int,
    out_status: *mut i8,
) -> c_int {
    if goal_id.is_null() || out_status.is_null() {
        set_error("rcm_action_get_result: null pointer");
        return RCM_ERR;
    }
    let (client_obj, msg_obj) =
        match dual(client, Kind::ActionClient, result, "rcm_action_get_result") {
            Ok(v) => v,
            Err(code) => return code,
        };
    let mut client_g = handle::lock(&client_obj);
    let mut msg_g = handle::lock(&msg_obj);
    let (Payload::ActionClient(a), Payload::Msg(m)) = (&mut *client_g, &mut *msg_g) else {
        return RCM_ERR;
    };
    if m.dds_type != a.result_dds {
        set_error("rcm_action_get_result: message type does not match the action result type");
        return RCM_ERR_TYPE_MISMATCH;
    }
    let mut req = Vec::with_capacity(GOAL_ID_LEN + 4);
    req.extend_from_slice(&CDR_LE_HEADER);
    req.extend_from_slice(slice::from_raw_parts(goal_id, GOAL_ID_LEN));
    let timeout = Duration::from_millis(u64::try_from(timeout_ms).unwrap_or(0));
    let Some(resp) = a.get_result.call(&req, timeout) else {
        return 0;
    };
    unwrap_nested_response(
        &a.gr_resp_ty,
        &a.result_ty,
        &resp,
        "status",
        "result",
        m.ptr,
    )
    .map_or_else(
        |code| code,
        |status| {
            *out_status = status;
            1
        },
    )
}

/// Take the next feedback sample (non-blocking), decoding the feedback payload
/// into message `feedback` and writing its 16-byte goal id to `out_goal_id` (may
/// be null). Returns 1 if a sample was decoded, 0 if none, negative on error.
#[no_mangle]
pub unsafe extern "C" fn rcm_action_poll_feedback(
    client: RcmHandle,
    feedback: RcmHandle,
    out_goal_id: *mut u8,
) -> c_int {
    let (client_obj, msg_obj) = match dual(
        client,
        Kind::ActionClient,
        feedback,
        "rcm_action_poll_feedback",
    ) {
        Ok(v) => v,
        Err(code) => return code,
    };
    let mut client_g = handle::lock(&client_obj);
    let mut msg_g = handle::lock(&msg_obj);
    let (Payload::ActionClient(a), Payload::Msg(m)) = (&mut *client_g, &mut *msg_g) else {
        return RCM_ERR;
    };
    if m.dds_type != a.feedback_dds {
        set_error("rcm_action_poll_feedback: message type does not match the action feedback type");
        return RCM_ERR_TYPE_MISMATCH;
    }
    let Some(sample) = a.feedback.take() else {
        return 0;
    };
    let cdr = sample.into_cdr();
    // A FeedbackMessage is `uuid[16] goal_id` then the feedback payload; decode
    // the wrapper into scratch memory, copy out the id, and re-extract the
    // payload into the caller's message via its own layout.
    let scratch = a.fb_msg_ty.alloc_zeroed();
    let rc = (|| -> Result<(), c_int> {
        if let Err(e) = a.fb_msg_ty.decode(&cdr, scratch) {
            set_error(format!("rcm_action_poll_feedback: decode failed: {e}"));
            return Err(RCM_ERR_DECODE);
        }
        if !out_goal_id.is_null() {
            let off = field_offset(&a.fb_msg_ty, "goal_id").unwrap_or(0);
            ptr::copy_nonoverlapping(scratch.add(off), out_goal_id, GOAL_ID_LEN);
        }
        extract_nested(&a.fb_msg_ty, &a.feedback_ty, scratch, "feedback", m.ptr);
        Ok(())
    })();
    a.fb_msg_ty.fini(scratch);
    a.fb_msg_ty.dealloc(scratch);
    match rc {
        Ok(()) => 1,
        Err(code) => code,
    }
}

/// Request cancellation of `goal_id` (16 bytes; all-zero cancels every goal),
/// blocking up to `timeout_ms`, writing the server's `return_code` to
/// `out_return_code`. Returns 1 on reply, 0 on timeout, negative on error.
#[no_mangle]
pub unsafe extern "C" fn rcm_action_cancel_goal(
    client: RcmHandle,
    goal_id: *const u8,
    timeout_ms: c_int,
    out_return_code: *mut i8,
) -> c_int {
    if goal_id.is_null() || out_return_code.is_null() {
        set_error("rcm_action_cancel_goal: null pointer");
        return RCM_ERR;
    }
    let mut id = [0u8; GOAL_ID_LEN];
    id.copy_from_slice(slice::from_raw_parts(goal_id, GOAL_ID_LEN));
    with(
        client,
        Kind::ActionClient,
        "rcm_action_cancel_goal",
        RCM_ERR,
        |p| {
            let Payload::ActionClient(a) = p else {
                return RCM_ERR;
            };
            let req = CancelGoalRequest {
                goal_info: GoalInfoMsg::new(GoalId(id), Time::now_system()),
            }
            .to_cdr(Endian::Little);
            let timeout = Duration::from_millis(u64::try_from(timeout_ms).unwrap_or(0));
            let Some(resp_cdr) = a.cancel.call(&req, timeout) else {
                return 0;
            };
            let Ok(resp) = CancelGoalResponse::from_cdr(&resp_cdr) else {
                set_error("rcm_action_cancel_goal: decode failed");
                return RCM_ERR_DECODE;
            };
            let code = resp.return_code;
            // SAFETY: `resp` owns its sequence and is consumed exactly once here.
            unsafe { resp.fini() };
            // SAFETY: out_return_code checked non-null above.
            unsafe { *out_return_code = code };
            1
        },
    )
}

/// Free an action-client handle. Returns 0 on success, negative if already stale.
#[no_mangle]
pub extern "C" fn rcm_action_client_free(client: RcmHandle) -> c_int {
    free_handle(client, Kind::ActionClient, "rcm_action_client_free")
}

/// Decode a `{ i8 <status_field>; <Nested> <payload_field> }` response wrapper
/// into scratch memory, returning the status byte and extracting the nested
/// payload into `out` (a message of `nested`'s type). The wrapper and nested
/// types share one resolved program, so the nested layout matches field-for-field.
unsafe fn unwrap_nested_response(
    wrapper: &DynamicType,
    nested: &DynamicType,
    cdr: &[u8],
    status_field: &str,
    payload_field: &str,
    out: *mut u8,
) -> Result<i8, c_int> {
    let scratch = wrapper.alloc_zeroed();
    let result = (|| -> Result<i8, c_int> {
        if let Err(e) = wrapper.decode(cdr, scratch) {
            set_error(format!("rcm_action_get_result: decode failed: {e}"));
            return Err(RCM_ERR_DECODE);
        }
        let status_off = field_offset(wrapper, status_field).unwrap_or(0);
        let status = scratch.add(status_off).cast::<i8>().read_unaligned();
        extract_nested(wrapper, nested, scratch, payload_field, out);
        Ok(status)
    })();
    wrapper.fini(scratch);
    wrapper.dealloc(scratch);
    result
}

/// Move the nested `field` of a decoded `wrapper` (living in `scratch`) into
/// caller message `out`. The wrapper and nested types share one resolved
/// program, so the field's struct layout IS the nested type's layout — the
/// bytes move over verbatim, and zeroing the source region transfers ownership
/// of heap members (a zeroed string/sequence triple is null/empty, which the
/// wrapper's `fini` skips). No CDR re-encode/decode round-trip.
unsafe fn extract_nested(
    wrapper: &DynamicType,
    nested: &DynamicType,
    scratch: *mut u8,
    field: &str,
    out: *mut u8,
) {
    let off = field_offset(wrapper, field).unwrap_or(0);
    nested.fini(out);
    ptr::copy_nonoverlapping(scratch.add(off), out, nested.size());
    ptr::write_bytes(scratch.add(off), 0, nested.size());
}

// ---- parameters -------------------------------------------------------------

/// A C-ABI scalar parameter value. `kind` is a `ParameterType` tag: 1 = bool,
/// 2 = integer, 3 = double, 4 = string. `boolean`/`integer`/`number` carry the
/// value for their kind; `text` carries the (NUL-terminated) string for kind 4
/// and is otherwise null. Array parameter kinds are not exposed here.
#[repr(C)]
pub struct RcmParamValue {
    pub kind: u8,
    pub boolean: u8,
    pub integer: i64,
    pub number: f64,
    pub text: *const c_char,
}

/// Convert a C parameter descriptor into a runtime [`ParameterValue`], or record
/// an error and return `None` for an unsupported/NotSet kind.
unsafe fn param_value_from_c(v: *const RcmParamValue, what: &str) -> Option<ParameterValue> {
    if v.is_null() {
        set_error(format!("{what}: null value"));
        return None;
    }
    let v = &*v;
    match ParameterType::from_u8(v.kind) {
        ParameterType::Bool => Some(ParameterValue::Bool(v.boolean != 0)),
        ParameterType::Integer => Some(ParameterValue::Integer(v.integer)),
        ParameterType::Double => Some(ParameterValue::Double(v.number)),
        ParameterType::String => Some(ParameterValue::String(
            cstr(v.text).unwrap_or("").to_owned(),
        )),
        _ => {
            set_error(format!(
                "{what}: unsupported parameter kind {} (scalar bool/int/double/string only)",
                v.kind
            ));
            None
        }
    }
}

/// Emit a scalar [`ParameterValue`] as a `{{"type":"double","value":..}}` JSON
/// object (arrays render their type with a null value).
fn param_value_json(value: &ParameterValue) -> String {
    let mut out = String::new();
    out.push_str("{\"type\":\"");
    out.push_str(match value.parameter_type() {
        ParameterType::NotSet => "not_set",
        ParameterType::Bool => "bool",
        ParameterType::Integer => "integer",
        ParameterType::Double => "double",
        ParameterType::String => "string",
        ParameterType::ByteArray => "byte_array",
        ParameterType::BoolArray => "bool_array",
        ParameterType::IntegerArray => "integer_array",
        ParameterType::DoubleArray => "double_array",
        ParameterType::StringArray => "string_array",
    });
    out.push_str("\",\"value\":");
    match value {
        ParameterValue::Bool(b) => out.push_str(if *b { "true" } else { "false" }),
        ParameterValue::Integer(i) => out.push_str(&i.to_string()),
        ParameterValue::Double(d) => out.push_str(&format_double(*d)),
        ParameterValue::String(s) => json_escape(&mut out, s),
        ParameterValue::ByteArray(v) => json_array(&mut out, v, |o, b| o.push_str(&b.to_string())),
        ParameterValue::BoolArray(v) => json_array(&mut out, v, |o, b| {
            o.push_str(if *b { "true" } else { "false" });
        }),
        ParameterValue::IntegerArray(v) => {
            json_array(&mut out, v, |o, i| o.push_str(&i.to_string()));
        }
        ParameterValue::DoubleArray(v) => {
            json_array(&mut out, v, |o, d| o.push_str(&format_double(*d)));
        }
        ParameterValue::StringArray(v) => json_array(&mut out, v, |o, s| json_escape(o, s)),
        ParameterValue::NotSet => out.push_str("null"),
    }
    out.push('}');
    out
}

/// Emit a slice as a JSON array using `write` for each element.
fn json_array<T>(out: &mut String, items: &[T], write: impl Fn(&mut String, &T)) {
    out.push('[');
    for (i, item) in items.iter().enumerate() {
        if i > 0 {
            out.push(',');
        }
        write(out, item);
    }
    out.push(']');
}

/// Render a double as valid JSON (finite values only; non-finite become null).
fn format_double(d: f64) -> String {
    if d.is_finite() {
        // A plain Display never yields Inf/NaN here and round-trips through f64.
        format!("{d:?}")
    } else {
        "null".to_owned()
    }
}

/// Create a parameter server for node `name` on `ctx`, spawning a background
/// thread that answers `get/set/list/describe` requests and publishes
/// `/parameter_events`. Returns `0` on error. Free with [`rcm_param_server_free`].
#[no_mangle]
pub unsafe extern "C" fn rcm_param_server(ctx: RcmHandle, name: *const c_char) -> RcmHandle {
    let Some(name) = cstr(name) else {
        set_error("rcm_param_server: null name");
        return 0;
    };
    let server = with(ctx, Kind::Ctx, "rcm_param_server", None, |p| {
        let Payload::Ctx(c) = p else { return None };
        Some(ParameterServer::new(&c.dds, name))
    });
    let Some(server) = server else { return 0 };
    let server = Arc::new(Mutex::new(server));
    let stop = Arc::new(std::sync::atomic::AtomicBool::new(false));
    let (srv, flag) = (Arc::clone(&server), Arc::clone(&stop));
    let thread = std::thread::spawn(move || {
        while !flag.load(std::sync::atomic::Ordering::Relaxed) {
            srv.lock()
                .unwrap_or_else(PoisonError::into_inner)
                .serve_pending();
            std::thread::sleep(Duration::from_millis(10));
        }
    });
    handle::insert(
        Some(ctx),
        Payload::ParamServer(RcmParamServer {
            server,
            stop,
            thread: Some(thread),
        }),
    )
}

/// Declare (or overwrite) parameter `name` with `value`, publishing a
/// `/parameter_events` update. Returns 0 on success, negative on error.
#[no_mangle]
pub unsafe extern "C" fn rcm_param_set(
    server: RcmHandle,
    name: *const c_char,
    value: *const RcmParamValue,
) -> c_int {
    let Some(name) = cstr(name) else {
        set_error("rcm_param_set: null name");
        return RCM_ERR;
    };
    let Some(value) = param_value_from_c(value, "rcm_param_set") else {
        return RCM_ERR;
    };
    with(server, Kind::ParamServer, "rcm_param_set", RCM_ERR, |p| {
        let Payload::ParamServer(s) = p else {
            return RCM_ERR;
        };
        s.server
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .set_local(name, value);
        0
    })
}

/// Declare (or overwrite) an **array** parameter `name`. `kind` is the
/// `ParameterType` tag (5 = byte, 6 = bool, 7 = integer, 8 = double);
/// `data`/`count` is a packed array of that kind's element (`u8`, `u8` 0/1,
/// `i64`, `f64`). String arrays go through [`rcm_param_set_string_array`].
/// Returns 0 on success, negative on error.
#[no_mangle]
pub unsafe extern "C" fn rcm_param_set_array(
    server: RcmHandle,
    name: *const c_char,
    kind: u8,
    data: *const u8,
    count: usize,
) -> c_int {
    let Some(name) = cstr(name) else {
        set_error("rcm_param_set_array: null name");
        return RCM_ERR;
    };
    if data.is_null() && count > 0 {
        set_error("rcm_param_set_array: null data");
        return RCM_ERR;
    }
    let value = match ParameterType::from_u8(kind) {
        ParameterType::ByteArray => {
            ParameterValue::ByteArray(slice::from_raw_parts(data, count).to_vec())
        }
        ParameterType::BoolArray => ParameterValue::BoolArray(
            slice::from_raw_parts(data, count)
                .iter()
                .map(|&b| b != 0)
                .collect(),
        ),
        // Unaligned reads: the caller's buffer has no alignment contract.
        ParameterType::IntegerArray => ParameterValue::IntegerArray(
            (0..count)
                .map(|i| data.add(i * 8).cast::<i64>().read_unaligned())
                .collect(),
        ),
        ParameterType::DoubleArray => ParameterValue::DoubleArray(
            (0..count)
                .map(|i| data.add(i * 8).cast::<f64>().read_unaligned())
                .collect(),
        ),
        _ => {
            set_error(format!(
                "rcm_param_set_array: kind {kind} is not a packed array kind (5..=8)"
            ));
            return RCM_ERR;
        }
    };
    param_server_set(server, "rcm_param_set_array", name, value)
}

/// Declare (or overwrite) a **string array** parameter `name` from `items[0..n]`
/// (NUL-terminated UTF-8). Returns 0 on success, negative on error.
#[no_mangle]
pub unsafe extern "C" fn rcm_param_set_string_array(
    server: RcmHandle,
    name: *const c_char,
    items: *const *const c_char,
    n: usize,
) -> c_int {
    let Some(name) = cstr(name) else {
        set_error("rcm_param_set_string_array: null name");
        return RCM_ERR;
    };
    if items.is_null() && n > 0 {
        set_error("rcm_param_set_string_array: null items");
        return RCM_ERR;
    }
    let strings: Vec<String> = if n == 0 {
        Vec::new()
    } else {
        slice::from_raw_parts(items, n)
            .iter()
            .map(|&p| cstr(p).unwrap_or("").to_owned())
            .collect()
    };
    param_server_set(
        server,
        "rcm_param_set_string_array",
        name,
        ParameterValue::StringArray(strings),
    )
}

/// Set `name` to `value` on a validated parameter-server handle.
fn param_server_set(server: RcmHandle, what: &str, name: &str, value: ParameterValue) -> c_int {
    with(server, Kind::ParamServer, what, RCM_ERR, |p| {
        let Payload::ParamServer(s) = p else {
            return RCM_ERR;
        };
        s.server
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
            .set_local(name, value);
        0
    })
}

/// The current value of parameter `name` as a JSON object (see
/// [`param_value_json`]). Owned string; free with [`rcm_string_free`]. Null on a
/// handle error or if `name` is undeclared (with `rcm_last_error` set).
#[no_mangle]
pub unsafe extern "C" fn rcm_param_get_json(server: RcmHandle, name: *const c_char) -> *mut c_char {
    let Some(name) = cstr(name) else {
        set_error("rcm_param_get_json: null name");
        return ptr::null_mut();
    };
    with(
        server,
        Kind::ParamServer,
        "rcm_param_get_json",
        ptr::null_mut(),
        |p| {
            let Payload::ParamServer(s) = p else {
                return ptr::null_mut();
            };
            let guard = s.server.lock().unwrap_or_else(PoisonError::into_inner);
            if let Some(value) = guard.get_local(name) {
                into_c_string(param_value_json(value))
            } else {
                set_error(format!("rcm_param_get_json: undeclared parameter '{name}'"));
                ptr::null_mut()
            }
        },
    )
}

/// The names of every declared parameter as a JSON array of strings. Owned
/// string; free with [`rcm_string_free`]. Null on a handle error.
#[no_mangle]
pub extern "C" fn rcm_param_list_json(server: RcmHandle) -> *mut c_char {
    with(
        server,
        Kind::ParamServer,
        "rcm_param_list_json",
        ptr::null_mut(),
        |p| {
            let Payload::ParamServer(s) = p else {
                return ptr::null_mut();
            };
            let guard = s.server.lock().unwrap_or_else(PoisonError::into_inner);
            let mut out = String::from("[");
            for (i, n) in guard.parameter_names().iter().enumerate() {
                if i > 0 {
                    out.push(',');
                }
                json_escape(&mut out, n);
            }
            out.push(']');
            into_c_string(out)
        },
    )
}

/// Stop the server's background thread and free its handle. Returns 0 on
/// success, negative if already stale.
#[no_mangle]
pub extern "C" fn rcm_param_server_free(server: RcmHandle) -> c_int {
    free_handle(server, Kind::ParamServer, "rcm_param_server_free")
}

// ---- graph & node identity --------------------------------------------------

/// Discover the ROS graph (listening `listen_ms` for node announcements) and
/// return it as JSON (topics/services/actions/nodes; see header). Owned string;
/// free with [`rcm_string_free`]. Null on error.
#[no_mangle]
pub extern "C" fn rcm_graph_json(ctx: RcmHandle, listen_ms: c_int) -> *mut c_char {
    with(ctx, Kind::Ctx, "rcm_graph_json", ptr::null_mut(), |p| {
        let Payload::Ctx(c) = p else {
            return ptr::null_mut();
        };
        let listen = Duration::from_millis(u64::try_from(listen_ms).unwrap_or(0));
        let graph = Graph::discover_with_nodes(&c.dds, listen);
        into_c_string(graph_json(&graph))
    })
}

/// Advertise a ROS node `name` under `namespace` on this context (so it appears
/// in `ros2 node list`). Repeated calls add more nodes. Returns 0 on success,
/// negative on error.
#[no_mangle]
pub unsafe extern "C" fn rcm_node(
    ctx: RcmHandle,
    name: *const c_char,
    namespace: *const c_char,
) -> c_int {
    let (Some(name), Some(namespace)) = (cstr(name), cstr(namespace)) else {
        set_error("rcm_node: null argument");
        return RCM_ERR;
    };
    with(ctx, Kind::Ctx, "rcm_node", RCM_ERR, |p| {
        let Payload::Ctx(c) = p else { return RCM_ERR };
        if c.discovery.is_none() {
            c.discovery = Some(DiscoveryInfo::new(&c.dds));
        }
        if let Some(d) = c.discovery.as_mut() {
            d.add_node(namespace, name);
        }
        0
    })
}

// ---- bags (MCAP) ------------------------------------------------------------

/// An MCAP bag writer (lz4-chunked by default), following the same conventions
/// as the Rust `bag_record` path. `writer` is `Option` so `finish` can consume
/// it on close while `Drop` still finishes an unclosed bag.
pub struct RcmBagWriter {
    writer: Option<McapWriter<BufWriter<File>>>,
}

impl Drop for RcmBagWriter {
    fn drop(&mut self) {
        if let Some(w) = self.writer.take() {
            let _ = w.finish();
        }
    }
}

/// A streaming MCAP bag reader with a one-sample cursor: [`rcm_bag_next`]
/// advances, and the info/data/decode calls address the current sample.
pub struct RcmBagReader {
    reader: RawSampleReader,
    current: Option<RawSample>,
}

/// Open `path` for writing as an MCAP bag. `compression` is `"lz4"` (default
/// when null) or `"none"`. Returns `0` on error. Close with [`rcm_bag_writer_close`].
#[no_mangle]
pub unsafe extern "C" fn rcm_bag_open_write(
    path: *const c_char,
    compression: *const c_char,
) -> RcmHandle {
    let Some(path) = cstr(path) else {
        set_error("rcm_bag_open_write: null path");
        return 0;
    };
    let compression = match cstr(compression) {
        None => Compression::Lz4,
        Some(s) => match s.parse::<Compression>() {
            Ok(c) => c,
            Err(e) => {
                set_error(format!("rcm_bag_open_write: {e}"));
                return 0;
            }
        },
    };
    let file = match File::create(path) {
        Ok(f) => f,
        Err(e) => {
            set_error(format!("rcm_bag_open_write: {path}: {e}"));
            return 0;
        }
    };
    match McapWriter::with_compression(BufWriter::new(file), compression) {
        Ok(writer) => handle::insert(
            None,
            Payload::BagWriter(RcmBagWriter {
                writer: Some(writer),
            }),
        ),
        Err(e) => {
            set_error(format!("rcm_bag_open_write: {e}"));
            0
        }
    }
}

/// Encode message `msg` and append it to the bag on `topic` at `ts_nanos`
/// (nanoseconds since the epoch; negative is an error). The schema name is
/// derived from the message's type (`pkg/msg/Name`). Returns 0 on success.
#[no_mangle]
pub unsafe extern "C" fn rcm_bag_write(
    bag: RcmHandle,
    topic: *const c_char,
    ts_nanos: i64,
    msg: RcmHandle,
) -> c_int {
    let Some(topic) = cstr(topic) else {
        set_error("rcm_bag_write: null topic");
        return RCM_ERR;
    };
    let (bag_obj, msg_obj) = match dual(bag, Kind::BagWriter, msg, "rcm_bag_write") {
        Ok(v) => v,
        Err(code) => return code,
    };
    let mut bag_g = handle::lock(&bag_obj);
    let mut msg_g = handle::lock(&msg_obj);
    let (Payload::BagWriter(b), Payload::Msg(m)) = (&mut *bag_g, &mut *msg_g) else {
        return RCM_ERR;
    };
    let Some(writer) = b.writer.as_mut() else {
        set_error("rcm_bag_write: bag already closed");
        return RCM_ERR;
    };
    // SAFETY: `m.ptr` is a live message of `m.ty`.
    let cdr = match m.ty.encode(m.ptr) {
        Ok(cdr) => cdr,
        Err(e) => {
            set_error(format!("rcm_bag_write: encode failed: {e}"));
            return RCM_ERR_ENCODE;
        }
    };
    let id = m.ty.root();
    let ros_type = format!("{}/msg/{}", id.package, id.name);
    match writer.write(topic, ts_nanos, &RawMsg::new(ros_type, cdr)) {
        Ok(()) => 0,
        Err(e) => {
            set_error(format!("rcm_bag_write: {e}"));
            RCM_ERR
        }
    }
}

/// Flush chunks, write the MCAP footer, and free the writer handle. Returns 0
/// on success, negative on a handle or I/O error.
#[no_mangle]
pub extern "C" fn rcm_bag_writer_close(bag: RcmHandle) -> c_int {
    let obj = match handle::remove(bag, Kind::BagWriter) {
        Ok(o) => o,
        Err(e) => return handle_err(e, "rcm_bag_writer_close"),
    };
    let Payload::BagWriter(b) = &mut *handle::lock(&obj) else {
        return RCM_ERR;
    };
    match b.writer.take().map(McapWriter::finish) {
        None | Some(Ok(_)) => 0,
        Some(Err(e)) => {
            set_error(format!("rcm_bag_writer_close: {e}"));
            RCM_ERR
        }
    }
}

/// Open the MCAP bag at `path` for streaming reads. Returns `0` on error. Free
/// with [`rcm_bag_reader_free`].
#[no_mangle]
pub unsafe extern "C" fn rcm_bag_open_read(path: *const c_char) -> RcmHandle {
    let Some(path) = cstr(path) else {
        set_error("rcm_bag_open_read: null path");
        return 0;
    };
    match RawSampleReader::open(path) {
        Ok(reader) => handle::insert(
            None,
            Payload::BagReader(RcmBagReader {
                reader,
                current: None,
            }),
        ),
        Err(e) => {
            set_error(format!("rcm_bag_open_read: {path}: {e}"));
            0
        }
    }
}

/// Advance to the next sample. Returns 1 if a sample is now current, 0 at end
/// of bag, negative on a parse error.
#[no_mangle]
pub extern "C" fn rcm_bag_next(bag: RcmHandle) -> c_int {
    with(bag, Kind::BagReader, "rcm_bag_next", RCM_ERR, |p| {
        let Payload::BagReader(b) = p else {
            return RCM_ERR;
        };
        match b.reader.next() {
            Some(Ok(sample)) => {
                b.current = Some(sample);
                1
            }
            None => {
                b.current = None;
                0
            }
            Some(Err(e)) => {
                b.current = None;
                set_error(format!("rcm_bag_next: {e}"));
                RCM_ERR_DECODE
            }
        }
    })
}

/// The current sample's metadata as JSON:
/// `{"topic":..,"type":..,"log_time":..,"publish_time":..,"size":..}` where
/// `size` is the CDR byte length (for [`rcm_bag_data`]). Owned string; free with
/// [`rcm_string_free`]. Null on error or if no sample is current.
#[no_mangle]
pub extern "C" fn rcm_bag_info_json(bag: RcmHandle) -> *mut c_char {
    with(
        bag,
        Kind::BagReader,
        "rcm_bag_info_json",
        ptr::null_mut(),
        |p| {
            let Payload::BagReader(b) = p else {
                return ptr::null_mut();
            };
            let Some(sample) = b.current.as_ref() else {
                set_error("rcm_bag_info_json: no current sample (call rcm_bag_next)");
                return ptr::null_mut();
            };
            let mut out = String::from("{\"topic\":");
            json_escape(&mut out, &sample.topic);
            out.push_str(",\"type\":");
            json_escape(&mut out, sample.msg.ros_type());
            out.push_str(",\"log_time\":");
            out.push_str(&sample.log_time.to_string());
            out.push_str(",\"publish_time\":");
            out.push_str(&sample.publish_time.to_string());
            out.push_str(",\"size\":");
            out.push_str(&sample.msg.cdr().len().to_string());
            out.push('}');
            into_c_string(out)
        },
    )
}

/// Copy the current sample's full CDR bytes (encapsulation header + body) into
/// `out[0..cap]`. Returns the number of bytes copied (`min(cap, size)`), or
/// negative on error.
#[no_mangle]
pub unsafe extern "C" fn rcm_bag_data(bag: RcmHandle, out: *mut u8, cap: usize) -> c_int {
    if out.is_null() {
        set_error("rcm_bag_data: null out pointer");
        return RCM_ERR;
    }
    with(bag, Kind::BagReader, "rcm_bag_data", RCM_ERR, |p| {
        let Payload::BagReader(b) = p else {
            return RCM_ERR;
        };
        let Some(sample) = b.current.as_ref() else {
            set_error("rcm_bag_data: no current sample (call rcm_bag_next)");
            return RCM_ERR;
        };
        let cdr = sample.msg.cdr();
        let n = cdr.len().min(cap);
        ptr::copy_nonoverlapping(cdr.as_ptr(), out, n);
        c_int::try_from(n).unwrap_or(c_int::MAX)
    })
}

/// Decode the current sample into message `out` (finalized first). The sample's
/// recorded type must match the message's (else `RCM_ERR_TYPE_MISMATCH`).
/// Returns 1 on success, negative on error.
#[no_mangle]
pub extern "C" fn rcm_bag_decode(bag: RcmHandle, out: RcmHandle) -> c_int {
    let (bag_obj, msg_obj) = match dual(bag, Kind::BagReader, out, "rcm_bag_decode") {
        Ok(v) => v,
        Err(code) => return code,
    };
    let mut bag_g = handle::lock(&bag_obj);
    let mut msg_g = handle::lock(&msg_obj);
    let (Payload::BagReader(b), Payload::Msg(m)) = (&mut *bag_g, &mut *msg_g) else {
        return RCM_ERR;
    };
    let Some(sample) = b.current.as_ref() else {
        set_error("rcm_bag_decode: no current sample (call rcm_bag_next)");
        return RCM_ERR;
    };
    if roswell_ros2_compat::raw::ros_type_to_dds_type(sample.msg.ros_type()) != m.dds_type {
        set_error(format!(
            "rcm_bag_decode: sample type {} does not match message type {}",
            sample.msg.ros_type(),
            m.dds_type
        ));
        return RCM_ERR_TYPE_MISMATCH;
    }
    // SAFETY: `m.ptr` is a live message of `m.ty` (checked by dds_type match).
    unsafe {
        m.ty.fini(m.ptr);
        match m.ty.decode(sample.msg.cdr(), m.ptr) {
            Ok(()) => 1,
            Err(e) => {
                set_error(format!("rcm_bag_decode: decode failed: {e}"));
                RCM_ERR_DECODE
            }
        }
    }
}

/// The recorded schema text for ROS type `ros_type` (e.g. concatenated `.msg`
/// source from a rosbag2 recording; empty for schema-less writers like ours).
/// Owned string; free with [`rcm_string_free`]. Null if the type has no parsed
/// schema record (or on a handle error).
#[no_mangle]
pub unsafe extern "C" fn rcm_bag_schema(bag: RcmHandle, ros_type: *const c_char) -> *mut c_char {
    let Some(ros_type) = cstr(ros_type) else {
        set_error("rcm_bag_schema: null type");
        return ptr::null_mut();
    };
    with(
        bag,
        Kind::BagReader,
        "rcm_bag_schema",
        ptr::null_mut(),
        |p| {
            let Payload::BagReader(b) = p else {
                return ptr::null_mut();
            };
            if let Some(text) = b.reader.schema_text(ros_type) {
                into_c_string(text.to_owned())
            } else {
                set_error(format!("rcm_bag_schema: no schema for '{ros_type}'"));
                ptr::null_mut()
            }
        },
    )
}

/// Free a bag reader handle. Returns 0 on success, negative if already stale.
#[no_mangle]
pub extern "C" fn rcm_bag_reader_free(bag: RcmHandle) -> c_int {
    free_handle(bag, Kind::BagReader, "rcm_bag_reader_free")
}

// ---- tf ---------------------------------------------------------------------

/// A tf2 buffer fed by `/tf` + `/tf_static` subscriptions on this context's
/// participant. Samples are drained into the buffer on every lookup. Raw
/// subscribers (not the typed `TfListener`) because typed `TFMessage` readers
/// hold raw-pointer message types that cannot live in the `Send` handle table.
pub struct RcmTf {
    dynamic: RawDdsSubscriber,
    statics: RawDdsSubscriber,
    /// Broadcast side for [`rcm_tf_broadcast`]. Created with the handle: a
    /// lazy publisher would need the (possibly already-freed) context back.
    dyn_pub: RawDdsPublisher,
    static_pub: RawDdsPublisher,
    buffer: TfBuffer,
}

impl RcmTf {
    /// Drain pending `/tf` + `/tf_static` samples into the buffer.
    fn poll(&mut self) {
        for sub in [&mut self.dynamic, &mut self.statics] {
            while let Some(raw) = sub.take() {
                if let Ok(mut msg) = tf2_msgs__TFMessage::from_cdr(raw.cdr()) {
                    self.buffer.insert_tf_message(&msg);
                    // SAFETY: `msg` owns its sequence and is consumed once here.
                    unsafe { msg.fini() };
                }
            }
        }
    }
}

/// Create a transform buffer + listener on `ctx` (subscribing `/tf` and, with
/// transient-local QoS, `/tf_static`). Returns `0` on error. Free with
/// [`rcm_tf_free`].
#[no_mangle]
pub extern "C" fn rcm_tf_buffer(ctx: RcmHandle) -> RcmHandle {
    let tf_type = <tf2_msgs__TFMessage as CdrMsg>::TYPE_NAME;
    let default_policies = QosProfile::from_preset(Qos::Default).policies();
    let latched_policies = QosProfile::from_preset(Qos::Latched).policies();
    with(ctx, Kind::Ctx, "rcm_tf_buffer", 0, |p| {
        let Payload::Ctx(c) = p else { return 0 };
        let dynamic =
            RawDdsSubscriber::with_policies(c.dds.participant(), "/tf", tf_type, &default_policies);
        let statics = RawDdsSubscriber::with_policies(
            c.dds.participant(),
            "/tf_static",
            tf_type,
            &latched_policies,
        );
        let dyn_pub =
            RawDdsPublisher::with_policies(c.dds.participant(), "/tf", tf_type, &default_policies);
        let static_pub = RawDdsPublisher::with_policies(
            c.dds.participant(),
            "/tf_static",
            tf_type,
            &latched_policies,
        );
        handle::insert(
            Some(ctx),
            Payload::Tf(Box::new(RcmTf {
                dynamic,
                statics,
                dyn_pub,
                static_pub,
                buffer: TfBuffer::new(),
            })),
        )
    })
}

/// Drain pending `/tf` + `/tf_static` samples into the buffer, then look up the
/// transform taking `source`-frame points into `target`-frame coordinates at
/// `time_nanos` (nanoseconds; negative means "latest available", tf2's time
/// zero). On success writes `[tx, ty, tz, qx, qy, qz, qw]` to `out[0..7]` and
/// returns 1. Returns 0 if the transform is not (yet) available — no path, or
/// the stamp is outside the buffered range — with the reason in
/// [`rcm_last_error`]. Negative on a handle error.
#[no_mangle]
pub unsafe extern "C" fn rcm_tf_lookup(
    tf: RcmHandle,
    target: *const c_char,
    source: *const c_char,
    time_nanos: i64,
    out: *mut f64,
) -> c_int {
    let (Some(target), Some(source)) = (cstr(target), cstr(source)) else {
        set_error("rcm_tf_lookup: null frame");
        return RCM_ERR;
    };
    if out.is_null() {
        set_error("rcm_tf_lookup: null out pointer");
        return RCM_ERR;
    }
    with(tf, Kind::Tf, "rcm_tf_lookup", RCM_ERR, |p| {
        let Payload::Tf(t) = p else { return RCM_ERR };
        t.poll();
        let looked_up = if time_nanos < 0 {
            t.buffer.lookup_latest(target, source)
        } else {
            t.buffer
                .lookup(target, source, Time::from_nanos(time_nanos))
        };
        match looked_up {
            Ok(transform) => {
                let vals = [
                    transform.translation[0],
                    transform.translation[1],
                    transform.translation[2],
                    transform.rotation[0],
                    transform.rotation[1],
                    transform.rotation[2],
                    transform.rotation[3],
                ];
                ptr::copy_nonoverlapping(vals.as_ptr(), out, vals.len());
                1
            }
            Err(e) => {
                set_error(format!("rcm_tf_lookup: {e:?}"));
                0
            }
        }
    })
}

/// Broadcast the transform `vals` (`[tx, ty, tz, qx, qy, qz, qw]`) from frame
/// `parent` to frame `child`, stamped `time_nanos`, on `/tf` — or, when
/// `is_static` is nonzero, on latched `/tf_static`. Message construction and
/// publishing are entirely in Rust. Returns 0 on success.
#[no_mangle]
pub unsafe extern "C" fn rcm_tf_broadcast(
    tf: RcmHandle,
    parent: *const c_char,
    child: *const c_char,
    time_nanos: i64,
    is_static: u8,
    vals: *const f64,
) -> c_int {
    let (Some(parent), Some(child)) = (cstr(parent), cstr(child)) else {
        set_error("rcm_tf_broadcast: null frame");
        return RCM_ERR;
    };
    if vals.is_null() {
        set_error("rcm_tf_broadcast: null transform values");
        return RCM_ERR;
    }
    let v = slice::from_raw_parts(vals, 7);
    let transform = TfTransform {
        translation: [v[0], v[1], v[2]],
        rotation: [v[3], v[4], v[5], v[6]],
    };
    with(tf, Kind::Tf, "rcm_tf_broadcast", RCM_ERR, |p| {
        let Payload::Tf(t) = p else { return RCM_ERR };
        let mut msg = tf2_msgs__TFMessage {
            transforms: RosSequence::alloc(vec![geometry_msgs__TransformStamped {
                header: std_msgs__Header {
                    stamp: Time::from_nanos(time_nanos).to_msg(),
                    frame_id: RosString::alloc(parent),
                },
                child_frame_id: RosString::alloc(child),
                transform: transform.into_msg(),
            }]),
        };
        let cdr = msg.to_cdr(Endian::Little);
        // SAFETY: `msg` owns its strings/sequence and is consumed once here.
        unsafe { msg.fini() };
        let raw = RawMsg::new(<tf2_msgs__TFMessage as CdrMsg>::TYPE_NAME, cdr);
        if is_static == 0 {
            t.dyn_pub.publish(&raw);
        } else {
            t.static_pub.publish(&raw);
        }
        0
    })
}

/// Free a tf buffer handle. Returns 0 on success, negative if already stale.
#[no_mangle]
pub extern "C" fn rcm_tf_free(tf: RcmHandle) -> c_int {
    free_handle(tf, Kind::Tf, "rcm_tf_free")
}

// ---- action server ----------------------------------------------------------

/// Per-goal bookkeeping for [`RcmActionServer`]: current wire status, whether a
/// cancel has been requested, and the encoded `GetResult` response once the
/// goal reaches a terminal state.
struct GoalState {
    status: i8,
    cancel_requested: bool,
    result_cdr: Option<Vec<u8>>,
}

/// A runtime-typed action server: the three action services as [`RawService`]s
/// plus the `feedback` topic and latched `status` topic as [`RawDdsPublisher`]s
/// — the same composition as the loopback mini-server in
/// `roswell-ros2-compat/tests/action_client_loopback.rs`, but type-blind. Goals are
/// auto-accepted on [`rcm_action_server_take_goal`]; the caller executes them
/// and reports the terminal status via [`rcm_action_server_finish`].
/// `get_result` requests arriving before the result are parked and answered by
/// `finish`.
pub struct RcmActionServer {
    goal_ty: DynamicType,
    result_ty: DynamicType,
    feedback_ty: DynamicType,
    sg_req_ty: DynamicType,
    sg_resp_ty: DynamicType,
    gr_resp_ty: DynamicType,
    fb_msg_ty: DynamicType,
    result_dds: String,
    feedback_dds: String,
    send_goal: RawService,
    get_result: RawService,
    cancel: RawService,
    feedback: RawDdsPublisher,
    status: RawDdsPublisher,
    /// Insertion-ordered goal table (small: a handful of live goals).
    goals: Vec<([u8; GOAL_ID_LEN], GoalState)>,
    /// `get_result` correlation tokens waiting for a goal's terminal result.
    parked_results: Vec<([u8; GOAL_ID_LEN], u64)>,
}

impl RcmActionServer {
    fn goal_state(&mut self, id: [u8; GOAL_ID_LEN]) -> Option<&mut GoalState> {
        self.goals
            .iter_mut()
            .find(|(gid, _)| *gid == id)
            .map(|(_, state)| state)
    }

    /// Publish the latched `GoalStatusArray` reflecting every known goal.
    fn publish_status(&mut self) {
        let now = Time::now_system();
        let statuses: Vec<GoalStatusMsg> = self
            .goals
            .iter()
            .map(|(id, state)| {
                GoalStatusMsg::new(GoalId(*id), now, GoalStatus::from_i8(state.status))
            })
            .collect();
        let msg = GoalStatusArrayMsg::new(statuses);
        let cdr = msg.to_cdr(Endian::Little);
        // SAFETY: `msg` owns its sequence and is consumed exactly once here.
        unsafe { msg.fini() };
        self.status
            .publish(&RawMsg::new(<GoalStatusArrayMsg as CdrMsg>::TYPE_NAME, cdr));
    }

    /// Answer pending cancel requests: matching live goals are flagged
    /// cancel-requested and moved to `Canceling`; an all-zero id cancels every
    /// live goal. Rejected when nothing matches.
    fn serve_cancel(&mut self) {
        while let Some((cdr, token)) = self.cancel.take_request() {
            let Ok(req) = CancelGoalRequest::from_cdr(&cdr) else {
                self.cancel.send_reply(
                    token,
                    reply_cancel(CancelGoalResponse::empty(
                        CancelGoalResponse::ERROR_REJECTED,
                    )),
                );
                continue;
            };
            let wanted = req.goal_info.goal_id.0;
            let all = wanted == [0; GOAL_ID_LEN];
            let now = Time::now_system();
            let mut canceling = Vec::new();
            for (id, state) in &mut self.goals {
                let live = !GoalStatus::from_i8(state.status).terminal();
                if live && (all || *id == wanted) {
                    state.cancel_requested = true;
                    state.status = GoalStatus::Canceling as i8;
                    canceling.push(GoalInfoMsg::new(GoalId(*id), now));
                }
            }
            let resp = if canceling.is_empty() {
                CancelGoalResponse::empty(CancelGoalResponse::ERROR_REJECTED)
            } else {
                CancelGoalResponse::with_goals(CancelGoalResponse::ERROR_NONE, canceling)
            };
            self.cancel.send_reply(token, reply_cancel(resp));
            if !self.goals.is_empty() {
                self.publish_status();
            }
        }
    }

    /// Answer pending `get_result` requests whose result is ready; park the
    /// rest until [`rcm_action_server_finish`] supplies it.
    fn serve_results(&mut self) {
        while let Some((cdr, token)) = self.get_result.take_request() {
            // GetResult_Request is just the 16-byte goal uuid after the header.
            let mut id = [0u8; GOAL_ID_LEN];
            let Some(bytes) = cdr.get(4..4 + GOAL_ID_LEN) else {
                continue; // malformed request; nothing to correlate a reply to
            };
            id.copy_from_slice(bytes);
            let ready = self
                .goal_state(id)
                .and_then(|state| state.result_cdr.clone());
            match ready {
                Some(reply) => {
                    self.get_result.send_reply(token, reply);
                }
                None => self.parked_results.push((id, token)),
            }
        }
    }
}

/// Serialize a [`CancelGoalResponse`], releasing its owned sequence.
fn reply_cancel(resp: CancelGoalResponse) -> Vec<u8> {
    let cdr = resp.to_cdr(Endian::Little);
    // SAFETY: `resp` owns its sequence and is consumed exactly once here.
    unsafe { resp.fini() };
    cdr
}

/// Create a runtime-typed action server for `action_name`, resolving the action
/// type from `reference` (`pkg/action/Name`) against the workspace `roots`.
/// Returns `0` on error. Free with [`rcm_action_server_free`].
#[no_mangle]
pub unsafe extern "C" fn rcm_action_server(
    ctx: RcmHandle,
    action_name: *const c_char,
    reference: *const c_char,
    roots: *const *const c_char,
    n_roots: usize,
) -> RcmHandle {
    let (Some(action_name), Some(reference)) = (cstr(action_name), cstr(reference)) else {
        set_error("rcm_action_server: null argument");
        return 0;
    };
    let root_paths = collect_paths(roots, n_roots);
    let at: ActionTypes = match load_action_ref(reference, &root_paths) {
        Ok(at) => at,
        Err(e) => {
            set_error(format!("rcm_action_server: {e}"));
            return 0;
        }
    };
    let names = ActionNames::new(action_name);
    let default_policies = QosProfile::from_preset(Qos::Default).policies();
    let latched_policies = QosProfile::from_preset(Qos::Latched).policies();
    with(ctx, Kind::Ctx, "rcm_action_server", 0, |p| {
        let Payload::Ctx(c) = p else { return 0 };
        let send_goal = RawService::new(
            &c.dds,
            &names.send_goal,
            &at.send_goal_request.dds_type_name(),
            &at.send_goal_response.dds_type_name(),
        );
        let get_result = RawService::new(
            &c.dds,
            &names.get_result,
            &at.get_result_request.dds_type_name(),
            &at.get_result_response.dds_type_name(),
        );
        let cancel = RawService::new(
            &c.dds,
            &names.cancel_goal,
            <CancelGoalRequest as CdrMsg>::TYPE_NAME,
            <CancelGoalResponse as CdrMsg>::TYPE_NAME,
        );
        let feedback = RawDdsPublisher::with_policies(
            c.dds.participant(),
            &names.feedback,
            &at.feedback_message.dds_type_name(),
            &default_policies,
        );
        let status = RawDdsPublisher::with_policies(
            c.dds.participant(),
            &names.status,
            <GoalStatusArrayMsg as CdrMsg>::TYPE_NAME,
            &latched_policies,
        );
        handle::insert(
            Some(ctx),
            Payload::ActionServer(Box::new(RcmActionServer {
                result_dds: at.result.dds_type_name(),
                feedback_dds: at.feedback.dds_type_name(),
                goal_ty: at.goal.clone(),
                result_ty: at.result.clone(),
                feedback_ty: at.feedback.clone(),
                sg_req_ty: at.send_goal_request.clone(),
                sg_resp_ty: at.send_goal_response.clone(),
                gr_resp_ty: at.get_result_response.clone(),
                fb_msg_ty: at.feedback_message.clone(),
                send_goal,
                get_result,
                cancel,
                feedback,
                status,
                goals: Vec::new(),
                parked_results: Vec::new(),
            })),
        )
    })
}

fn server_type_handle(
    server: RcmHandle,
    what: &str,
    pick: impl Fn(&RcmActionServer) -> &DynamicType,
) -> RcmHandle {
    with(server, Kind::ActionServer, what, 0, |p| {
        let Payload::ActionServer(s) = p else {
            return 0;
        };
        let ty = pick(s).clone();
        handle::insert(None, Payload::Type(RcmType { ty }))
    })
}

/// The goal payload type. Owned type handle; free with [`rcm_type_free`].
#[no_mangle]
pub extern "C" fn rcm_action_server_goal_type(server: RcmHandle) -> RcmHandle {
    server_type_handle(server, "rcm_action_server_goal_type", |s| &s.goal_ty)
}

/// The result payload type. Owned type handle; free with [`rcm_type_free`].
#[no_mangle]
pub extern "C" fn rcm_action_server_result_type(server: RcmHandle) -> RcmHandle {
    server_type_handle(server, "rcm_action_server_result_type", |s| &s.result_ty)
}

/// The feedback payload type. Owned type handle; free with [`rcm_type_free`].
#[no_mangle]
pub extern "C" fn rcm_action_server_feedback_type(server: RcmHandle) -> RcmHandle {
    server_type_handle(server, "rcm_action_server_feedback_type", |s| {
        &s.feedback_ty
    })
}

/// Service pending cancel and `get_result` requests (parking early result
/// requests). Call periodically between goals. Returns 0 on success.
#[no_mangle]
pub extern "C" fn rcm_action_server_spin(server: RcmHandle) -> c_int {
    with(
        server,
        Kind::ActionServer,
        "rcm_action_server_spin",
        RCM_ERR,
        |p| {
            let Payload::ActionServer(s) = p else {
                return RCM_ERR;
            };
            s.serve_cancel();
            s.serve_results();
            0
        },
    )
}

/// Take the next pending goal request: the goal payload is decoded into message
/// `out_goal` (which must be of the action's goal type), its 16-byte goal id is
/// written to `out_goal_id`, the goal is auto-accepted (reply sent, status
/// `EXECUTING` latched). Returns 1 if a goal was taken, 0 if none pending,
/// negative on error.
#[no_mangle]
pub unsafe extern "C" fn rcm_action_server_take_goal(
    server: RcmHandle,
    out_goal: RcmHandle,
    out_goal_id: *mut u8,
) -> c_int {
    if out_goal_id.is_null() {
        set_error("rcm_action_server_take_goal: null out pointer");
        return RCM_ERR;
    }
    let (srv_obj, msg_obj) = match dual(
        server,
        Kind::ActionServer,
        out_goal,
        "rcm_action_server_take_goal",
    ) {
        Ok(v) => v,
        Err(code) => return code,
    };
    let mut srv_g = handle::lock(&srv_obj);
    let mut msg_g = handle::lock(&msg_obj);
    let (Payload::ActionServer(s), Payload::Msg(m)) = (&mut *srv_g, &mut *msg_g) else {
        return RCM_ERR;
    };
    if m.dds_type != s.goal_ty.dds_type_name() {
        set_error("rcm_action_server_take_goal: message type does not match the action goal type");
        return RCM_ERR_TYPE_MISMATCH;
    }
    let Some((cdr, token)) = s.send_goal.take_request() else {
        return 0;
    };
    // Decode the SendGoal_Request wrapper (uuid goal_id + goal) into scratch,
    // copy out the id, and extract the goal payload into the caller's message.
    let scratch = s.sg_req_ty.alloc_zeroed();
    let mut goal_id = [0u8; GOAL_ID_LEN];
    let rc = (|| -> Result<(), c_int> {
        if let Err(e) = s.sg_req_ty.decode(&cdr, scratch) {
            set_error(format!("rcm_action_server_take_goal: decode failed: {e}"));
            return Err(RCM_ERR_DECODE);
        }
        let off = field_offset(&s.sg_req_ty, "goal_id").unwrap_or(0);
        ptr::copy_nonoverlapping(scratch.add(off), goal_id.as_mut_ptr(), GOAL_ID_LEN);
        extract_nested(&s.sg_req_ty, &s.goal_ty, scratch, "goal", m.ptr);
        Ok(())
    })();
    s.sg_req_ty.fini(scratch);
    s.sg_req_ty.dealloc(scratch);
    if let Err(code) = rc {
        return code;
    }
    // Accept: SendGoal_Response { bool accepted; Time stamp } with stamp zeroed.
    let resp_scratch = s.sg_resp_ty.alloc_zeroed();
    let accepted_off = field_offset(&s.sg_resp_ty, "accepted").unwrap_or(0);
    resp_scratch.add(accepted_off).write(1);
    let reply = s.sg_resp_ty.encode(resp_scratch);
    s.sg_resp_ty.fini(resp_scratch);
    s.sg_resp_ty.dealloc(resp_scratch);
    let reply = match reply {
        Ok(reply) => reply,
        Err(e) => {
            set_error(format!("rcm_action_server_take_goal: encode failed: {e}"));
            return RCM_ERR_ENCODE;
        }
    };
    s.send_goal.send_reply(token, reply);
    s.goals.push((
        goal_id,
        GoalState {
            status: GoalStatus::Executing as i8,
            cancel_requested: false,
            result_cdr: None,
        },
    ));
    s.publish_status();
    ptr::copy_nonoverlapping(goal_id.as_ptr(), out_goal_id, GOAL_ID_LEN);
    1
}

/// True (1) if a cancel has been requested for `goal_id` (16 bytes), 0 if not,
/// negative on error.
#[no_mangle]
pub unsafe extern "C" fn rcm_action_server_cancel_requested(
    server: RcmHandle,
    goal_id: *const u8,
) -> c_int {
    if goal_id.is_null() {
        set_error("rcm_action_server_cancel_requested: null goal id");
        return RCM_ERR;
    }
    let mut id = [0u8; GOAL_ID_LEN];
    id.copy_from_slice(slice::from_raw_parts(goal_id, GOAL_ID_LEN));
    with(
        server,
        Kind::ActionServer,
        "rcm_action_server_cancel_requested",
        RCM_ERR,
        |p| {
            let Payload::ActionServer(s) = p else {
                return RCM_ERR;
            };
            s.serve_cancel();
            s.goal_state(id)
                .map_or(RCM_ERR, |state| c_int::from(state.cancel_requested))
        },
    )
}

/// Publish message `feedback` (of the action's feedback type) for `goal_id`.
/// Returns 0 on success, negative on error.
#[no_mangle]
pub unsafe extern "C" fn rcm_action_server_publish_feedback(
    server: RcmHandle,
    goal_id: *const u8,
    feedback: RcmHandle,
) -> c_int {
    if goal_id.is_null() {
        set_error("rcm_action_server_publish_feedback: null goal id");
        return RCM_ERR;
    }
    let (srv_obj, msg_obj) = match dual(
        server,
        Kind::ActionServer,
        feedback,
        "rcm_action_server_publish_feedback",
    ) {
        Ok(v) => v,
        Err(code) => return code,
    };
    let mut srv_g = handle::lock(&srv_obj);
    let mut msg_g = handle::lock(&msg_obj);
    let (Payload::ActionServer(s), Payload::Msg(m)) = (&mut *srv_g, &mut *msg_g) else {
        return RCM_ERR;
    };
    if m.dds_type != s.feedback_dds {
        set_error(
            "rcm_action_server_publish_feedback: message type does not match the feedback type",
        );
        return RCM_ERR_TYPE_MISMATCH;
    }
    // Build the FeedbackMessage wrapper in scratch: goal_id bytes + the payload
    // re-decoded into the wrapper's nested field (shared layout, see
    // `extract_nested` for the inverse).
    let scratch = s.fb_msg_ty.alloc_zeroed();
    let rc = (|| -> Result<Vec<u8>, c_int> {
        let id_off = field_offset(&s.fb_msg_ty, "goal_id").unwrap_or(0);
        ptr::copy_nonoverlapping(goal_id, scratch.add(id_off), GOAL_ID_LEN);
        let fb_off = field_offset(&s.fb_msg_ty, "feedback").unwrap_or(0);
        inject_nested(&s.feedback_ty, m.ptr, scratch.add(fb_off))?;
        s.fb_msg_ty.encode(scratch).map_err(|e| {
            set_error(format!(
                "rcm_action_server_publish_feedback: encode failed: {e}"
            ));
            RCM_ERR_ENCODE
        })
    })();
    s.fb_msg_ty.fini(scratch);
    s.fb_msg_ty.dealloc(scratch);
    match rc {
        Ok(cdr) => {
            let dds = s.fb_msg_ty.dds_type_name();
            s.feedback.publish(&RawMsg::new(dds, cdr));
            0
        }
        Err(code) => code,
    }
}

/// Finish `goal_id` with terminal `status` (4 = succeeded, 5 = canceled,
/// 6 = aborted) and result payload `result` (of the action's result type). The
/// encoded `GetResult` response is stored and any parked `get_result` requests
/// for this goal are answered; the latched status array is republished. Returns
/// 0 on success, negative on error.
#[no_mangle]
pub unsafe extern "C" fn rcm_action_server_finish(
    server: RcmHandle,
    goal_id: *const u8,
    status: i8,
    result: RcmHandle,
) -> c_int {
    if goal_id.is_null() {
        set_error("rcm_action_server_finish: null goal id");
        return RCM_ERR;
    }
    let (srv_obj, msg_obj) = match dual(
        server,
        Kind::ActionServer,
        result,
        "rcm_action_server_finish",
    ) {
        Ok(v) => v,
        Err(code) => return code,
    };
    let mut srv_g = handle::lock(&srv_obj);
    let mut msg_g = handle::lock(&msg_obj);
    let (Payload::ActionServer(s), Payload::Msg(m)) = (&mut *srv_g, &mut *msg_g) else {
        return RCM_ERR;
    };
    if m.dds_type != s.result_dds {
        set_error("rcm_action_server_finish: message type does not match the action result type");
        return RCM_ERR_TYPE_MISMATCH;
    }
    let mut id = [0u8; GOAL_ID_LEN];
    id.copy_from_slice(slice::from_raw_parts(goal_id, GOAL_ID_LEN));
    if s.goal_state(id).is_none() {
        set_error("rcm_action_server_finish: unknown goal id");
        return RCM_ERR;
    }
    // Build the GetResult_Response wrapper: status byte + the result payload
    // re-decoded into the wrapper's nested field.
    let scratch = s.gr_resp_ty.alloc_zeroed();
    let rc = (|| -> Result<Vec<u8>, c_int> {
        let status_off = field_offset(&s.gr_resp_ty, "status").unwrap_or(0);
        scratch.add(status_off).cast::<i8>().write(status);
        let result_off = field_offset(&s.gr_resp_ty, "result").unwrap_or(0);
        inject_nested(&s.result_ty, m.ptr, scratch.add(result_off))?;
        s.gr_resp_ty.encode(scratch).map_err(|e| {
            set_error(format!("rcm_action_server_finish: encode failed: {e}"));
            RCM_ERR_ENCODE
        })
    })();
    s.gr_resp_ty.fini(scratch);
    s.gr_resp_ty.dealloc(scratch);
    let reply = match rc {
        Ok(reply) => reply,
        Err(code) => return code,
    };
    if let Some(state) = s.goal_state(id) {
        state.status = status;
        state.result_cdr = Some(reply.clone());
    }
    let mut parked = std::mem::take(&mut s.parked_results);
    parked.retain(|(gid, token)| {
        if *gid == id {
            s.get_result.send_reply(*token, reply.clone());
            false
        } else {
            true
        }
    });
    s.parked_results = parked;
    s.publish_status();
    0
}

/// Free an action-server handle. Returns 0 on success, negative if already
/// stale.
#[no_mangle]
pub extern "C" fn rcm_action_server_free(server: RcmHandle) -> c_int {
    free_handle(server, Kind::ActionServer, "rcm_action_server_free")
}

/// Copy a message of `nested`'s type from `src` into `dst` — the location of a
/// nested field of that type inside a wrapper's scratch memory — by
/// encode/decode through the nested type's own layout (the inverse of
/// [`extract_nested`]).
unsafe fn inject_nested(nested: &DynamicType, src: *mut u8, dst: *mut u8) -> Result<(), c_int> {
    let cdr = nested.encode(src).map_err(|e| {
        set_error(format!("action payload re-encode failed: {e}"));
        RCM_ERR_ENCODE
    })?;
    nested.decode(&cdr, dst).map_err(|e| {
        set_error(format!("action payload decode failed: {e}"));
        RCM_ERR_DECODE
    })
}

// ---- helpers ----------------------------------------------------------------

/// Validate `ty` as a type handle and clone its `DynamicType`, or record an
/// error and return `Err(())`.
fn clone_type(ty: RcmHandle, what: &str) -> Result<DynamicType, ()> {
    match handle::lookup(ty, Kind::Type) {
        Ok(obj) => {
            let Payload::Type(t) = &*handle::lock(&obj) else {
                return Err(());
            };
            Ok(t.ty.clone())
        }
        Err(e) => {
            handle_err(e, what);
            Err(())
        }
    }
}

/// A pair of looked-up, still-locked handle objects.
type ObjPair = (Arc<Mutex<Payload>>, Arc<Mutex<Payload>>);

/// Validate an endpoint handle and a message handle together, returning both
/// object `Arc`s or an error code (with `rcm_last_error` set).
fn dual(endpoint: RcmHandle, kind: Kind, msg: RcmHandle, what: &str) -> Result<ObjPair, c_int> {
    let e = handle::lookup(endpoint, kind).map_err(|err| handle_err(err, what))?;
    let m = handle::lookup(msg, Kind::Msg)
        .map_err(|err| handle_err(err, &format!("{what}: message")))?;
    Ok((e, m))
}

/// Remove a handle of `kind`, mapping any handle error to a status code.
fn free_handle(h: RcmHandle, kind: Kind, what: &str) -> c_int {
    match handle::remove(h, kind) {
        Ok(_) => 0,
        Err(e) => handle_err(e, what),
    }
}

unsafe fn cstr<'a>(p: *const c_char) -> Option<&'a str> {
    if p.is_null() {
        None
    } else {
        CStr::from_ptr(p).to_str().ok()
    }
}

unsafe fn collect_paths(deps: *const *const c_char, n_deps: usize) -> Vec<PathBuf> {
    if deps.is_null() || n_deps == 0 {
        return Vec::new();
    }
    slice::from_raw_parts(deps, n_deps)
        .iter()
        .filter_map(|&p| cstr(p).map(PathBuf::from))
        .collect()
}

fn into_c_string(s: String) -> *mut c_char {
    CString::new(s).map_or(ptr::null_mut(), CString::into_raw)
}

// ---- JSON emission (hand-rolled; serde is not a dependency) ------------------

fn json_escape(out: &mut String, s: &str) {
    out.push('"');
    for c in s.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            c if (c as u32) < 0x20 => out.push_str(&format!("\\u{:04x}", c as u32)),
            c => out.push(c),
        }
    }
    out.push('"');
}

fn id_key(id: &MsgId) -> String {
    format!("{}/{}", id.package, id.name)
}

/// Emit one message's C-ABI layout as a JSON object body (fields + size/align).
fn write_message_layout(out: &mut String, layout: &TypeLayout) {
    out.push_str("{\"size\":");
    out.push_str(&layout.size.to_string());
    out.push_str(",\"align\":");
    out.push_str(&layout.align.to_string());
    out.push_str(",\"fields\":[");
    for (i, f) in layout.fields.iter().enumerate() {
        if i > 0 {
            out.push(',');
        }
        out.push_str("{\"name\":");
        json_escape(out, &f.name);
        out.push_str(",\"offset\":");
        out.push_str(&f.offset.to_string());
        let mult = match f.multiplicity {
            Multiplicity::Scalar => "scalar",
            Multiplicity::Array(_) => "array",
            Multiplicity::Sequence => "sequence",
        };
        out.push_str(",\"multiplicity\":\"");
        out.push_str(mult);
        out.push('"');
        if let Multiplicity::Array(len) = f.multiplicity {
            out.push_str(",\"array_len\":");
            out.push_str(&len.to_string());
        }
        out.push_str(",\"element\":{\"size\":");
        out.push_str(&f.element.size.to_string());
        out.push_str(",\"align\":");
        out.push_str(&f.element.align.to_string());
        match &f.element.kind {
            ElemKind::Prim(p) => {
                out.push_str(",\"kind\":\"prim\",\"prim\":\"");
                out.push_str(p.cdr_fn());
                out.push('"');
            }
            ElemKind::String { wide, bound } => {
                out.push_str(",\"kind\":\"string\",\"wide\":");
                out.push_str(if *wide { "true" } else { "false" });
                if let Some(b) = bound {
                    out.push_str(",\"bound\":");
                    out.push_str(&b.to_string());
                }
            }
            ElemKind::Message(id) => {
                out.push_str(",\"kind\":\"message\",\"message\":");
                json_escape(out, &id_key(id));
            }
        }
        out.push_str("}}");
    }
    out.push_str("]}");
}

fn layout_json(ty: &DynamicType) -> String {
    let mut out = String::new();
    out.push_str("{\"root\":");
    json_escape(&mut out, &id_key(ty.root()));
    out.push_str(",\"dds_type\":");
    json_escape(&mut out, &ty.dds_type_name());
    out.push_str(",\"size\":");
    out.push_str(&ty.size().to_string());
    out.push_str(",\"align\":");
    out.push_str(&ty.align().to_string());
    out.push_str(",\"messages\":{");
    for (i, id) in ty.message_ids().into_iter().enumerate() {
        if i > 0 {
            out.push(',');
        }
        json_escape(&mut out, &id_key(id));
        out.push(':');
        // Present for every id returned by message_ids().
        write_message_layout(&mut out, ty.message_layout(id).unwrap());
    }
    out.push_str("}}");
    out
}

fn events_json(events: &[QosEvent]) -> String {
    let mut out = String::from("[");
    for (i, e) in events.iter().enumerate() {
        if i > 0 {
            out.push(',');
        }
        write_event(&mut out, *e);
    }
    out.push(']');
    out
}

fn write_event(out: &mut String, e: QosEvent) {
    match e {
        QosEvent::DeadlineMissed { count } => {
            out.push_str("{\"event\":\"deadline_missed\",\"count\":");
            out.push_str(&count.to_string());
            out.push('}');
        }
        QosEvent::IncompatibleQos { policy, count } => {
            out.push_str("{\"event\":\"incompatible_qos\",\"policy\":\"");
            out.push_str(incompatible_policy_name(policy));
            out.push_str("\",\"count\":");
            out.push_str(&count.to_string());
            out.push('}');
        }
        QosEvent::LivelinessChanged { alive, not_alive } => {
            out.push_str("{\"event\":\"liveliness_changed\",\"alive\":");
            out.push_str(&alive.to_string());
            out.push_str(",\"not_alive\":");
            out.push_str(&not_alive.to_string());
            out.push('}');
        }
        QosEvent::LivelinessLost { count } => {
            out.push_str("{\"event\":\"liveliness_lost\",\"count\":");
            out.push_str(&count.to_string());
            out.push('}');
        }
        QosEvent::SampleLost { count } => {
            out.push_str("{\"event\":\"sample_lost\",\"count\":");
            out.push_str(&count.to_string());
            out.push('}');
        }
        QosEvent::SampleRejected { count } => {
            out.push_str("{\"event\":\"sample_rejected\",\"count\":");
            out.push_str(&count.to_string());
            out.push('}');
        }
        QosEvent::SubscriptionMatched { current } => {
            out.push_str("{\"event\":\"subscription_matched\",\"current\":");
            out.push_str(&current.to_string());
            out.push('}');
        }
        QosEvent::PublicationMatched { current } => {
            out.push_str("{\"event\":\"publication_matched\",\"current\":");
            out.push_str(&current.to_string());
            out.push('}');
        }
    }
}

fn incompatible_policy_name(p: IncompatiblePolicy) -> &'static str {
    match p {
        IncompatiblePolicy::Reliability => "reliability",
        IncompatiblePolicy::Durability => "durability",
        IncompatiblePolicy::Deadline => "deadline",
        IncompatiblePolicy::Liveliness => "liveliness",
        IncompatiblePolicy::History => "history",
        IncompatiblePolicy::Other => "other",
    }
}

fn action_channel_name(c: ActionChannel) -> &'static str {
    match c {
        ActionChannel::SendGoal => "send_goal",
        ActionChannel::GetResult => "get_result",
        ActionChannel::CancelGoal => "cancel_goal",
        ActionChannel::Feedback => "feedback",
        ActionChannel::Status => "status",
    }
}

fn graph_json(graph: &Graph) -> String {
    let mut out = String::from("{\"topics\":[");
    for (i, t) in graph.topics.iter().enumerate() {
        if i > 0 {
            out.push(',');
        }
        out.push_str("{\"name\":");
        json_escape(&mut out, &t.name);
        out.push_str(",\"type\":");
        json_escape(&mut out, &t.ros_type);
        out.push('}');
    }
    out.push_str("],\"services\":[");
    for (i, s) in graph.services.iter().enumerate() {
        if i > 0 {
            out.push(',');
        }
        out.push_str("{\"name\":");
        json_escape(&mut out, &s.name);
        out.push_str(",\"request_type\":");
        json_escape(&mut out, &s.request_type);
        out.push_str(",\"response_type\":");
        json_escape(&mut out, &s.response_type);
        out.push('}');
    }
    out.push_str("],\"actions\":[");
    for (i, a) in graph.actions.iter().enumerate() {
        if i > 0 {
            out.push(',');
        }
        out.push_str("{\"name\":");
        json_escape(&mut out, &a.name);
        out.push_str(",\"channels\":[");
        for (j, c) in a.channels.iter().enumerate() {
            if j > 0 {
                out.push(',');
            }
            out.push('"');
            out.push_str(action_channel_name(*c));
            out.push('"');
        }
        out.push_str("]}");
    }
    out.push_str("],\"nodes\":[");
    for (i, n) in graph.nodes.iter().enumerate() {
        if i > 0 {
            out.push(',');
        }
        json_escape(&mut out, &n.full_name());
    }
    out.push_str("]}");
    out
}

// ---- FFI loopback + handle-safety tests -------------------------------------

#[cfg(test)]
mod tests;

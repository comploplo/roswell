//! Plain C-ABI FFI for the roscmp runtime: load ROS2 message/service types at
//! runtime, allocate their C-ABI struct memory, and publish/subscribe/serve/call
//! over real RTPS — all without a ROS installation, PyO3, or codegen.
//!
//! The surface is a small set of `rcm_`-prefixed `extern "C"` functions over
//! generation-counted [`handle`]s (a `u64` slot+generation, not a raw pointer).
//! Every entry point validates its handles — live slot, matching generation,
//! matching kind — so use-after-free, double-free, use-after-shutdown, and type
//! confusion become distinct `RCM_ERR_*` codes rather than undefined behaviour.
//! All message logic (parsing, layout, CDR, QoS, transport, correlation) lives
//! in [`roscmp`] / [`roscmp_dds`]; this crate is the thin, hardened C boundary.
//! The hand-written header in `include/roscmp.h` is the contract.
//!
//! # Errors
//! Every fallible function returns an `int` status (`0` = success, negative =
//! one of the `RCM_ERR_*` codes) or a `0`/null sentinel; on failure a
//! human-readable message is available from [`rcm_last_error`] (thread-local,
//! valid until the next failing call on the same thread).
//!
//! # Thread-safety
//! See the "Threading" section of `include/roscmp.h`. In short: the handle
//! table is internally synchronized, so any handle may be validated from any
//! thread; the intended pattern is one background thread driving
//! [`rcm_wait`]/[`rcm_take`] over a context's subscribers while another thread
//! publishes/calls on the same context.
#![allow(clippy::missing_safety_doc)] // safety contracts documented in roscmp.h
#![allow(unsafe_op_in_unsafe_fn)] // each extern fn is one cohesive unsafe boundary
#![allow(clippy::too_many_lines)]

mod handle;

use std::cell::RefCell;
use std::ffi::{c_char, c_int, CStr, CString};
use std::path::PathBuf;
use std::ptr;
use std::slice;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use roscmp::dynamic::{
    assign_prim_sequence, assign_string, load_message, load_service, DynamicType, ElemKind,
    Multiplicity, TypeLayout,
};
use roscmp::ir::MsgId;

use roscmp_dds::discovery::DiscoveryInfo;
use roscmp_dds::graph::{ActionChannel, Graph};
use roscmp_dds::qos::{DurabilityKind, IncompatiblePolicy, QosEvent, QosProfile, ReliabilityKind};
use roscmp_dds::raw::{RawClient, RawDdsPublisher, RawDdsSubscriber, RawMsg, RawService};
use roscmp_dds::transport::{Dds, Qos};

use handle::{HandleError, Kind, Payload, RcmHandle};

/// The ABI version. Bumped whenever the C signatures, struct layouts, or handle
/// encoding change incompatibly. `2` is the generation-counted-handle ABI;
/// version `1` was the original raw-opaque-pointer ABI (never shipped).
/// Mirrored as `RCM_ABI_VERSION` in the header; the loader checks the two match.
const RCM_ABI_VERSION: u32 = 2;

// ---- status codes (mirror RCM_ERR_* in roscmp.h) ----------------------------

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
    let cdr = match unsafe { p.ty.encode(m.ptr) } {
        Ok(cdr) => cdr,
        Err(e) => {
            set_error(format!("rcm_publish: encode failed: {e}"));
            return RCM_ERR_ENCODE;
        }
    };
    p.pubr.publish(&RawMsg::new(p.dds_type.clone(), cdr));
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

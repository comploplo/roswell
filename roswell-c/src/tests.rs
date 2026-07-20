//! FFI tests driven entirely through the handle-based C signatures: end-to-end
//! pub/sub and service loopback, plus the handle-table safety properties
//! (use-after-shutdown, wrong-kind, wrong-type, double-free, stale data) and a
//! concurrent wait+publish exercise.

use super::*;
use roswell::dynamic::sample_path;
use std::ffi::CString;

fn cs(s: &str) -> CString {
    CString::new(s).unwrap()
}

fn last_err() -> String {
    unsafe {
        CStr::from_ptr(rcm_last_error())
            .to_string_lossy()
            .into_owned()
    }
}

/// Store `s` into a String message's single leading string field (offset 0).
fn set_root_string(msg: RcmHandle, s: &str) {
    let b = s.as_bytes();
    let rc = unsafe { rcm_str_assign(msg, 0, b.as_ptr(), b.len()) };
    assert_eq!(rc, 0, "rcm_str_assign: {}", last_err());
}

/// Read a String message's single leading string field back out of its memory.
unsafe fn read_root_string(msg: RcmHandle) -> String {
    let base = rcm_msg_data(msg);
    assert!(!base.is_null());
    let data = base.cast::<*const u8>().read_unaligned();
    let size = base
        .add(std::mem::size_of::<usize>())
        .cast::<usize>()
        .read_unaligned();
    if data.is_null() || size == 0 {
        return String::new();
    }
    let bytes = slice::from_raw_parts(data, size);
    String::from_utf8_lossy(bytes).into_owned()
}

fn load_string_type() -> RcmHandle {
    let path = cs(sample_path("std_msgs/msg/String.msg").to_str().unwrap());
    let ty = unsafe { rcm_type_load(path.as_ptr(), ptr::null(), 0) };
    assert_ne!(ty, 0, "load failed: {}", last_err());
    ty
}

/// Drive the whole publish/subscribe path through the handle-based C signatures.
#[test]
fn pubsub_loopback_through_ffi() {
    unsafe {
        let ctx = rcm_init(0);
        assert_ne!(ctx, 0);
        let ty = load_string_type();

        let layout = rcm_type_layout_json(ty);
        let json = CStr::from_ptr(layout).to_str().unwrap().to_string();
        rcm_string_free(layout);
        assert!(json.contains("\"root\":\"std_msgs/String\""), "{json}");
        assert!(json.contains("\"name\":\"data\""), "{json}");

        let topic = cs("/rcm_ffi_chatter");
        let pubh = rcm_publisher(ctx, topic.as_ptr(), ty, ptr::null());
        let sub = rcm_subscriber(ctx, topic.as_ptr(), ty, ptr::null());
        assert_ne!(pubh, 0);
        assert_ne!(sub, 0);

        let msg = rcm_msg_alloc(ty);
        assert_ne!(msg, 0);
        set_root_string(msg, "hello ffi");

        let out = rcm_msg_alloc(ty);
        let mut got = String::new();
        for _ in 0..50 {
            assert_eq!(rcm_publish(pubh, msg), 0);
            if rcm_take(sub, out) == 1 {
                got = read_root_string(out);
                if !got.is_empty() {
                    break;
                }
            }
            std::thread::sleep(Duration::from_millis(100));
        }
        assert_eq!(got, "hello ffi");

        assert_eq!(rcm_msg_free(msg), 0);
        assert_eq!(rcm_msg_free(out), 0);
        assert_eq!(rcm_subscriber_free(sub), 0);
        assert_eq!(rcm_publisher_free(pubh), 0);
        assert_eq!(rcm_type_free(ty), 0);
        rcm_shutdown(ctx);
    }
}

/// Drive the split service path through the handle-based signatures.
#[test]
fn service_loopback_through_ffi() {
    use std::sync::atomic::{AtomicBool, Ordering};
    use std::sync::Arc;

    let stop = Arc::new(AtomicBool::new(false));
    let server_stop = Arc::clone(&stop);
    let server = std::thread::spawn(move || unsafe {
        let ctx = rcm_init(0);
        let path = cs(sample_path("example_interfaces/srv/AddTwoInts.srv")
            .to_str()
            .unwrap());
        let (mut req_ty, mut resp_ty) = (0u64, 0u64);
        assert_eq!(
            rcm_type_load_srv(
                path.as_ptr(),
                ptr::null(),
                0,
                &raw mut req_ty,
                &raw mut resp_ty
            ),
            0
        );
        let name = cs("/rcm_ffi_add");
        let svc = rcm_service(ctx, name.as_ptr(), req_ty, resp_ty);
        let req_buf = rcm_msg_alloc(req_ty);
        let resp_buf = rcm_msg_alloc(resp_ty);
        while !server_stop.load(Ordering::Relaxed) {
            let mut token = 0u64;
            if rcm_service_take_request(svc, req_buf, &raw mut token) == 1 {
                let base = rcm_msg_data(req_buf);
                let a = base.cast::<i64>().read_unaligned();
                let b = base.add(8).cast::<i64>().read_unaligned();
                rcm_msg_data(resp_buf).cast::<i64>().write_unaligned(a + b);
                rcm_service_send_reply(svc, token, resp_buf);
            }
            std::thread::sleep(Duration::from_millis(10));
        }
        rcm_msg_free(req_buf);
        rcm_msg_free(resp_buf);
        rcm_service_free(svc);
        rcm_type_free(req_ty);
        rcm_type_free(resp_ty);
        rcm_shutdown(ctx);
    });

    let sum = unsafe {
        let ctx = rcm_init(0);
        let path = cs(sample_path("example_interfaces/srv/AddTwoInts.srv")
            .to_str()
            .unwrap());
        let (mut req_ty, mut resp_ty) = (0u64, 0u64);
        assert_eq!(
            rcm_type_load_srv(
                path.as_ptr(),
                ptr::null(),
                0,
                &raw mut req_ty,
                &raw mut resp_ty
            ),
            0
        );
        let name = cs("/rcm_ffi_add");
        let client = rcm_client(ctx, name.as_ptr(), req_ty, resp_ty);
        std::thread::sleep(Duration::from_secs(2));
        let req = rcm_msg_alloc(req_ty);
        rcm_msg_data(req).cast::<i64>().write_unaligned(41);
        rcm_msg_data(req).add(8).cast::<i64>().write_unaligned(1);
        let resp = rcm_msg_alloc(resp_ty);
        let mut sum = 0i64;
        for _ in 0..20 {
            if rcm_call(client, req, resp, 2000) == 1 {
                sum = rcm_msg_data(resp).cast::<i64>().read_unaligned();
                break;
            }
        }
        rcm_msg_free(req);
        rcm_msg_free(resp);
        rcm_client_free(client);
        rcm_type_free(req_ty);
        rcm_type_free(resp_ty);
        rcm_shutdown(ctx);
        sum
    };

    stop.store(true, Ordering::Relaxed);
    let _ = server.join();
    assert_eq!(sum, 42);
}

/// ABI version and version string are exposed and sane.
#[test]
fn abi_version_and_string() {
    assert_eq!(rcm_abi_version(), RCM_ABI_VERSION);
    let s = unsafe { CStr::from_ptr(rcm_version_string()) }
        .to_str()
        .unwrap();
    assert_eq!(s, env!("CARGO_PKG_VERSION"));
}

/// After `rcm_shutdown`, endpoint handles created against the context validate
/// to a stale-handle error rather than crashing.
#[test]
fn use_after_shutdown_is_stale() {
    unsafe {
        let ctx = rcm_init(0);
        let ty = load_string_type();
        let topic = cs("/rcm_ffi_uaf");
        let pubh = rcm_publisher(ctx, topic.as_ptr(), ty, ptr::null());
        let msg = rcm_msg_alloc(ty);
        assert_ne!(pubh, 0);

        rcm_shutdown(ctx);

        // Publisher was a child of ctx: now stale, not a dangling deref.
        assert_eq!(rcm_publish(pubh, msg), RCM_ERR_STALE_HANDLE);
        assert!(last_err().contains("stale"), "{}", last_err());
        // A second shutdown / free of the same ctx is also a clean no-op path.
        rcm_shutdown(ctx);
        // The context handle itself is stale for context ops now.
        assert_eq!(rcm_graph_json(ctx, 0), ptr::null_mut());

        // Types and messages are NOT children of a context; still usable.
        assert_ne!(rcm_msg_data(msg), ptr::null_mut());
        assert_eq!(rcm_msg_free(msg), 0);
        assert_eq!(rcm_type_free(ty), 0);
    }
}

/// Publishing a message whose type differs from the publisher's is a clean
/// type-mismatch error.
#[test]
fn wrong_type_publish_is_type_mismatch() {
    unsafe {
        let ctx = rcm_init(0);
        let string_ty = load_string_type();
        // A structurally different type.
        let srv = cs(sample_path("example_interfaces/srv/AddTwoInts.srv")
            .to_str()
            .unwrap());
        let (mut req_ty, mut resp_ty) = (0u64, 0u64);
        assert_eq!(
            rcm_type_load_srv(
                srv.as_ptr(),
                ptr::null(),
                0,
                &raw mut req_ty,
                &raw mut resp_ty
            ),
            0
        );

        let topic = cs("/rcm_ffi_mismatch");
        let pubh = rcm_publisher(ctx, topic.as_ptr(), string_ty, ptr::null());
        // Allocate a message of the WRONG type (AddTwoInts request).
        let wrong = rcm_msg_alloc(req_ty);
        assert_eq!(rcm_publish(pubh, wrong), RCM_ERR_TYPE_MISMATCH);
        assert!(last_err().contains("does not match"), "{}", last_err());

        rcm_msg_free(wrong);
        rcm_publisher_free(pubh);
        rcm_type_free(string_ty);
        rcm_type_free(req_ty);
        rcm_type_free(resp_ty);
        rcm_shutdown(ctx);
    }
}

/// A wrong-kind handle (message handle passed where a publisher is expected) is
/// rejected without UB.
#[test]
fn wrong_kind_is_rejected() {
    let ctx = rcm_init(0);
    let ty = load_string_type();
    let msg = rcm_msg_alloc(ty);
    // Pass a message handle as the publisher handle.
    assert_eq!(rcm_publish(msg, msg), RCM_ERR_WRONG_KIND);
    // Pass a type handle to a subscriber-only op.
    assert_eq!(rcm_subscriber_events(ty), ptr::null_mut());
    rcm_msg_free(msg);
    rcm_type_free(ty);
    rcm_shutdown(ctx);
}

/// Double-free of a message handle returns a stale-handle error the second time.
#[test]
fn double_free_is_stale() {
    let ty = load_string_type();
    let msg = rcm_msg_alloc(ty);
    assert_eq!(rcm_msg_free(msg), 0);
    assert_eq!(rcm_msg_free(msg), RCM_ERR_STALE_HANDLE);
    // A stale data lookup is rejected too, not a dangling pointer.
    assert_eq!(rcm_msg_data(msg), ptr::null_mut());
    assert_eq!(rcm_type_free(ty), 0);
}

/// Concurrent `rcm_wait` on a background thread and `rcm_publish` on the main
/// thread over the same context run without panic or UB (handles are `u64`, the
/// table is internally synchronized). We also assert delivery works.
#[test]
fn concurrent_wait_and_publish() {
    use std::sync::atomic::{AtomicBool, Ordering};
    use std::sync::Arc;

    unsafe {
        let ctx = rcm_init(0);
        let ty = load_string_type();
        let topic = cs("/rcm_ffi_concurrent");
        let pubh = rcm_publisher(ctx, topic.as_ptr(), ty, ptr::null());
        let sub = rcm_subscriber(ctx, topic.as_ptr(), ty, ptr::null());

        let got = Arc::new(AtomicBool::new(false));
        let done = Arc::new(AtomicBool::new(false));
        let (got_t, done_t) = (Arc::clone(&got), Arc::clone(&done));

        // Background thread: wait for the subscriber, then take (handles are u64).
        let waiter = std::thread::spawn(move || {
            let out = rcm_msg_alloc(ty);
            while !done_t.load(Ordering::Relaxed) {
                let subs = [sub];
                if rcm_wait(subs.as_ptr(), 1, 100) == 0 && rcm_take(sub, out) == 1 {
                    got_t.store(true, Ordering::Relaxed);
                }
            }
            rcm_msg_free(out);
        });

        // Main thread publishes concurrently.
        let msg = rcm_msg_alloc(ty);
        set_root_string(msg, "concurrent");
        for _ in 0..50 {
            assert_eq!(rcm_publish(pubh, msg), 0);
            if got.load(Ordering::Relaxed) {
                break;
            }
            std::thread::sleep(Duration::from_millis(50));
        }
        done.store(true, Ordering::Relaxed);
        let _ = waiter.join();
        assert!(
            got.load(Ordering::Relaxed),
            "no message delivered concurrently"
        );

        rcm_msg_free(msg);
        rcm_subscriber_free(sub);
        rcm_publisher_free(pubh);
        rcm_type_free(ty);
        rcm_shutdown(ctx);
    }
}

/// Read a heap JSON string returned by an `rcm_param_*_json` call and free it.
unsafe fn take_json_str(ptr: *mut c_char) -> String {
    assert!(!ptr.is_null(), "null json: {}", last_err());
    let s = CStr::from_ptr(ptr).to_string_lossy().into_owned();
    rcm_string_free(ptr);
    s
}

/// Declaring scalar parameters through the FFI round-trips each type back out of
/// `rcm_param_get_json`, lists them, and rejects unsupported/undeclared reads.
#[test]
fn param_server_scalar_roundtrip() {
    unsafe {
        let ctx = rcm_init(0);
        let node = cs("param_ffi_node");
        let server = rcm_param_server(ctx, node.as_ptr());
        assert_ne!(server, 0, "rcm_param_server: {}", last_err());

        let set = |name: &str, v: &RcmParamValue| {
            let n = cs(name);
            assert_eq!(
                rcm_param_set(server, n.as_ptr(), std::ptr::from_ref(v)),
                0,
                "set {name}: {}",
                last_err()
            );
        };
        let get = |name: &str| -> String {
            let n = cs(name);
            take_json_str(rcm_param_get_json(server, n.as_ptr()))
        };

        set(
            "enabled",
            &RcmParamValue {
                kind: 1,
                boolean: 1,
                integer: 0,
                number: 0.0,
                text: ptr::null(),
            },
        );
        set(
            "gain",
            &RcmParamValue {
                kind: 2,
                boolean: 0,
                integer: 7,
                number: 0.0,
                text: ptr::null(),
            },
        );
        set(
            "speed",
            &RcmParamValue {
                kind: 3,
                boolean: 0,
                integer: 0,
                number: 1.5,
                text: ptr::null(),
            },
        );
        let frame = cs("base_link");
        set(
            "frame",
            &RcmParamValue {
                kind: 4,
                boolean: 0,
                integer: 0,
                number: 0.0,
                text: frame.as_ptr(),
            },
        );

        assert_eq!(get("enabled"), r#"{"type":"bool","value":true}"#);
        assert_eq!(get("gain"), r#"{"type":"integer","value":7}"#);
        assert_eq!(get("speed"), r#"{"type":"double","value":1.5}"#);
        assert_eq!(get("frame"), r#"{"type":"string","value":"base_link"}"#);

        // Overwrite publishes a changed event and reads back the new value.
        set(
            "gain",
            &RcmParamValue {
                kind: 2,
                boolean: 0,
                integer: 9,
                number: 0.0,
                text: ptr::null(),
            },
        );
        assert_eq!(get("gain"), r#"{"type":"integer","value":9}"#);

        // Every name appears in the JSON list.
        let list = take_json_str(rcm_param_list_json(server));
        for name in ["enabled", "gain", "speed", "frame"] {
            assert!(list.contains(name), "list {list} missing {name}");
        }

        // Undeclared read and unsupported array kind both fail cleanly.
        let missing = cs("nope");
        assert!(rcm_param_get_json(server, missing.as_ptr()).is_null());
        let arr = cs("arr");
        let bad = RcmParamValue {
            kind: 7,
            boolean: 0,
            integer: 0,
            number: 0.0,
            text: ptr::null(),
        };
        assert_eq!(
            rcm_param_set(server, arr.as_ptr(), std::ptr::from_ref(&bad)),
            RCM_ERR
        );

        assert_eq!(rcm_param_server_free(server), 0);
        // Server handle is stale after free.
        assert_eq!(rcm_param_server_free(server), RCM_ERR_STALE_HANDLE);
        rcm_shutdown(ctx);
    }
}

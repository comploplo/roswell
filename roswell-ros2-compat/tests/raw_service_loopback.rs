//! Loopback for the runtime-typed raw service path: a [`RawClient`] built with
//! only runtime type/topic strings calls the *existing typed* `AddTwoInts`
//! `Service` over a domain-0 RTPS loopback. The request bytes are produced and
//! the reply bytes consumed entirely through `roswell::dynamic` (no generated
//! request/response structs on the client side), proving a message loaded at
//! runtime can drive a real service end to end.

use std::path::Path;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;

use roswell::dynamic::{load_service, DynamicType};

use roswell_ros2_compat::codec::CdrMsg;
use roswell_ros2_compat::msgs::{
    example_interfaces__AddTwoInts_Request as Req, example_interfaces__AddTwoInts_Response as Resp,
};
use roswell_ros2_compat::raw::RawClient;
use roswell_ros2_compat::service::Service;
use roswell_ros2_compat::transport::Dds;

const SERVICE: &str = "/raw_add_two_ints";

/// Run the real typed `AddTwoInts` server (sum = a + b) until `stop` is set.
fn run_server(stop: &Arc<AtomicBool>) {
    let dds = Dds::new(0);
    let mut server = Service::<Req, Resp>::new(&dds, SERVICE);
    while !stop.load(Ordering::Relaxed) {
        server.serve_pending(|req| Resp { sum: req.a + req.b });
        std::thread::sleep(Duration::from_millis(20));
    }
}

#[test]
fn raw_client_calls_typed_service() {
    // Runtime types for request/response, loaded from the sample .srv.
    let path = roswell::dynamic::sample_path("example_interfaces/srv/AddTwoInts.srv");
    let (req_ty, resp_ty): (DynamicType, DynamicType) =
        load_service(&path, &[] as &[&Path]).unwrap();

    let stop = Arc::new(AtomicBool::new(false));
    let server_stop = Arc::clone(&stop);
    let server = std::thread::spawn(move || run_server(&server_stop));

    let dds = Dds::new(0);
    let mut client = RawClient::new(
        &dds,
        SERVICE,
        &req_ty.dds_type_name(),
        &resp_ty.dds_type_name(),
    );

    // Let discovery match the request/reply endpoint pair before the first call.
    std::thread::sleep(Duration::from_secs(3));

    // Build the request CDR bytes at runtime through the layout-driven codec,
    // reading a C-ABI request struct (a = 41, b = 1) as raw memory.
    let req_struct = Req { a: 41, b: 1 };
    let request = unsafe {
        req_ty
            .encode(core::ptr::from_ref(&req_struct).cast::<u8>())
            .unwrap()
    };

    // Volatile writers drop requests sent before the reply reader is matched, so
    // early calls legitimately time out — retry while discovery settles.
    let mut reply = None;
    for _ in 0..20 {
        if let Some(bytes) = client.call(&request, Duration::from_secs(2)) {
            reply = Some(bytes);
            break;
        }
    }
    stop.store(true, Ordering::Relaxed);
    let _ = server.join();

    let reply = reply.expect("no reply from typed service within timeout");
    // Decode the reply through the layout-driven codec into C-ABI memory and
    // read back the `sum` field (a lone i64 at offset 0).
    let sum = unsafe {
        let buf = resp_ty.alloc_zeroed();
        resp_ty.decode(&reply, buf).unwrap();
        let sum = buf.cast::<i64>().read_unaligned();
        resp_ty.fini(buf);
        resp_ty.dealloc(buf);
        sum
    };
    assert_eq!(sum, 42);
    // Cross-check against the generated decoder for good measure.
    assert_eq!(Resp::decode(&reply).unwrap().sum, 42);
}

//! Loopback integration test: our `ParameterClient` drives a real
//! `ParameterServer` over a domain-0 loopback, exercising the `ros2 param`
//! CLI verbs (get/set/set_atomically/list/describe) end to end over RTPS and
//! asserting that a set round-trips and that unknown names read back `NotSet`.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;

use roscmp_dds::parameters::{
    Parameter, ParameterClient, ParameterDescriptor, ParameterServer, ParameterType, ParameterValue,
};
use roscmp_dds::transport::Dds;

const NODE: &str = "param_test_node";

/// Run a `ParameterServer` seeded with a few parameters on domain 0 until
/// `stop` is set, pumping pending requests continuously.
fn run_server(stop: &Arc<AtomicBool>) {
    let dds = Dds::new(0);
    let mut server = ParameterServer::new(&dds, NODE);
    server.declare(
        ParameterDescriptor::new("speed", ParameterType::Double),
        ParameterValue::Double(1.0),
    );
    server.declare(
        ParameterDescriptor::new("camera.gain", ParameterType::Integer),
        ParameterValue::Integer(7),
    );
    server.declare(
        ParameterDescriptor::new("enabled", ParameterType::Bool),
        ParameterValue::Bool(false),
    );
    while !stop.load(Ordering::Relaxed) {
        server.serve_pending();
        std::thread::sleep(Duration::from_millis(20));
    }
}

/// Retry `f` while discovery settles — volatile writers drop requests sent
/// before the reply reader is matched, so early calls legitimately time out.
fn retry<T>(mut f: impl FnMut() -> Option<T>) -> Option<T> {
    for _ in 0..20 {
        if let Some(v) = f() {
            return Some(v);
        }
    }
    None
}

#[test]
fn parameter_client_crud_loopback() {
    let stop = Arc::new(AtomicBool::new(false));
    let server_stop = Arc::clone(&stop);
    let server = std::thread::spawn(move || run_server(&server_stop));

    let dds = Dds::new(0);
    let mut client = ParameterClient::new(&dds, NODE);

    // Let discovery match all six service endpoint pairs before the first write.
    std::thread::sleep(Duration::from_secs(3));

    let timeout = Duration::from_secs(2);

    // get() returns declared values and NotSet for an unknown name.
    let got = retry(|| {
        client.get(
            &[
                "speed".to_string(),
                "camera.gain".to_string(),
                "missing".to_string(),
            ],
            timeout,
        )
    })
    .expect("get timed out");
    assert_eq!(got[0].value, ParameterValue::Double(1.0));
    assert_eq!(got[1].value, ParameterValue::Integer(7));
    assert_eq!(
        got[2].value,
        ParameterValue::NotSet,
        "unknown name is NotSet"
    );

    // get_types() mirrors the declared types.
    let types = retry(|| client.get_types(&["speed".to_string(), "missing".to_string()], timeout))
        .expect("get_types timed out");
    assert_eq!(types, vec![ParameterType::Double, ParameterType::NotSet]);

    // describe() returns the declared descriptor.
    let descriptors = retry(|| client.describe(&["camera.gain".to_string()], timeout))
        .expect("describe timed out");
    assert_eq!(descriptors[0].name, "camera.gain");
    assert_eq!(descriptors[0].parameter_type, ParameterType::Integer);

    // set() succeeds and the new value round-trips through a subsequent get().
    let results = retry(|| {
        client.set(
            vec![Parameter::new("speed", ParameterValue::Double(2.5))],
            timeout,
        )
    })
    .expect("set timed out");
    assert!(results[0].successful, "set reported failure");

    let after = poll_until(&mut client, "speed", &ParameterValue::Double(2.5), timeout);
    assert_eq!(after, ParameterValue::Double(2.5), "set did not round-trip");

    // set_atomically() creates a brand-new parameter that then lists and reads.
    let atomic = retry(|| {
        client.set_atomically(
            vec![Parameter::new(
                "nav.mode",
                ParameterValue::String("fast".into()),
            )],
            timeout,
        )
    })
    .expect("set_atomically timed out");
    assert!(atomic.successful, "atomic set reported failure");

    let listed = poll_until(
        &mut client,
        "nav.mode",
        &ParameterValue::String("fast".into()),
        timeout,
    );
    assert_eq!(listed, ParameterValue::String("fast".into()));

    // list() with the `nav` prefix surfaces the newly-created name.
    let list = retry(|| client.list(&["nav".to_string()], 0, timeout)).expect("list timed out");
    assert!(
        list.names.contains(&"nav.mode".to_string()),
        "list missing nav.mode: {:?}",
        list.names
    );

    stop.store(true, Ordering::Relaxed);
    let _ = server.join();
}

/// Poll `get(name)` until it reports `expected` (the server applies a set
/// asynchronously, one `serve_pending` tick after replying), or give up.
fn poll_until(
    client: &mut ParameterClient,
    name: &str,
    expected: &ParameterValue,
    timeout: Duration,
) -> ParameterValue {
    let mut last = ParameterValue::NotSet;
    for _ in 0..50 {
        if let Some(got) = client.get(&[name.to_string()], timeout) {
            last = got[0].value.clone();
            if &last == expected {
                return last;
            }
        }
        std::thread::sleep(Duration::from_millis(50));
    }
    last
}

//! Loopback tests for the `tokio` feature: publisher -> async stream, and an
//! async service call against a blocking `Service` server, both on domain 0.
#![cfg(feature = "tokio")]

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;

use roswell_ros2_compat::async_rt::{subscribe, AsyncClient};
use roswell_ros2_compat::msgs::{
    example_interfaces__AddTwoInts_Request, example_interfaces__AddTwoInts_Response,
    std_msgs__String, RosString,
};
use roswell_ros2_compat::service::Service;
use roswell_ros2_compat::transport::{Dds, MsgPublisher, Qos, Transport};

#[tokio::test(flavor = "multi_thread")]
async fn publish_reaches_async_stream() {
    let dds = Dds::new(0);
    // A String payload also exercises the Send bridge for RosString-carrying
    // messages across the pump thread.
    let publisher = dds.publisher::<std_msgs__String>("/tokio_chatter", Qos::Default);
    let mut stream = subscribe::<std_msgs__String>(&dds, "/tokio_chatter", Qos::Default);

    // Publish on a ticker until the stream yields: discovery has no fixed
    // settle time, and volatile writers drop pre-match samples.
    let publisher_task = tokio::task::spawn_blocking(move || {
        for _ in 0..300 {
            publisher.publish(std_msgs__String {
                data: RosString::alloc("hello tokio"),
            });
            std::thread::sleep(Duration::from_millis(50));
        }
    });

    let msg = tokio::time::timeout(Duration::from_secs(15), stream.next())
        .await
        .expect("no message within 15s")
        .expect("stream ended");
    assert_eq!(msg.data.as_str(), "hello tokio");
    publisher_task.abort();
}

#[tokio::test(flavor = "multi_thread")]
async fn async_service_call_round_trips() {
    // Blocking server on its own thread, the way the sync examples run it.
    let stop = Arc::new(AtomicBool::new(false));
    let server_stop = Arc::clone(&stop);
    let server = std::thread::spawn(move || {
        let dds = Dds::new(0);
        let mut service = Service::<
            example_interfaces__AddTwoInts_Request,
            example_interfaces__AddTwoInts_Response,
        >::new(&dds, "/tokio_add_two_ints");
        while !server_stop.load(Ordering::Relaxed) {
            service.serve_pending(|req| example_interfaces__AddTwoInts_Response {
                sum: req.a + req.b,
            });
            std::thread::sleep(Duration::from_millis(20));
        }
    });

    let dds = Dds::new(0);
    let client: AsyncClient<
        example_interfaces__AddTwoInts_Request,
        example_interfaces__AddTwoInts_Response,
    > = AsyncClient::new(&dds, "/tokio_add_two_ints");

    // Await discovery, then still retry: matched != flushed.
    let ready_deadline = tokio::time::Instant::now() + Duration::from_secs(15);
    while !client.server_is_ready().await && tokio::time::Instant::now() < ready_deadline {
        tokio::time::sleep(Duration::from_millis(20)).await;
    }
    assert!(client.server_is_ready().await, "server never discovered");

    let mut sum = None;
    for _ in 0..10 {
        if let Some(resp) = client
            .call(
                example_interfaces__AddTwoInts_Request { a: 40, b: 2 },
                Duration::from_secs(2),
            )
            .await
        {
            sum = Some(resp.sum);
            break;
        }
    }

    stop.store(true, Ordering::Relaxed);
    let _ = server.join();
    assert_eq!(sum, Some(42), "no correct reply received");
}

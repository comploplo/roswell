//! Loopback integration test: our generic `ActionServer` serves our
//! `ActionClient` end to end on domain 0 — goal → feedback → status → result,
//! plus the standard cancel policy for an unknown goal id.

use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use roscmp_dds::action::{ActionClient, ActionServer, CancelGoalResponse, GoalId, GoalStatus};
use roscmp_dds::msgs::{
    example_interfaces__Fibonacci_Feedback, example_interfaces__Fibonacci_FeedbackMessage,
    example_interfaces__Fibonacci_GetResult_Request,
    example_interfaces__Fibonacci_GetResult_Response, example_interfaces__Fibonacci_Goal,
    example_interfaces__Fibonacci_Result, example_interfaces__Fibonacci_SendGoal_Request,
    example_interfaces__Fibonacci_SendGoal_Response, RosSequence,
};
use roscmp_dds::transport::Dds;

type FibServer = ActionServer<
    example_interfaces__Fibonacci_SendGoal_Request,
    example_interfaces__Fibonacci_SendGoal_Response,
    example_interfaces__Fibonacci_GetResult_Request,
    example_interfaces__Fibonacci_GetResult_Response,
    example_interfaces__Fibonacci_FeedbackMessage,
>;

type FibClient = ActionClient<
    example_interfaces__Fibonacci_SendGoal_Request,
    example_interfaces__Fibonacci_SendGoal_Response,
    example_interfaces__Fibonacci_GetResult_Request,
    example_interfaces__Fibonacci_GetResult_Response,
    example_interfaces__Fibonacci_FeedbackMessage,
>;

fn fibonacci(order: i32) -> Vec<i32> {
    let len = order.clamp(0, 47) as usize;
    let mut seq = Vec::with_capacity(len);
    for i in 0..len {
        let value = match i {
            0 => 0,
            1 => 1,
            _ => seq[i - 1] + seq[i - 2],
        };
        seq.push(value);
    }
    seq
}

/// Run a Fibonacci server on the generic `ActionServer` until `stop` is set.
fn run_server(stop: &Arc<AtomicBool>) {
    let dds = Dds::new(0);
    let mut server: FibServer = ActionServer::new(&dds, "/fibonacci_generic");
    let mut orders: HashMap<GoalId, i32> = HashMap::new();

    while !stop.load(Ordering::Relaxed) {
        server.serve_goals(|id, goal| {
            orders.insert(id, goal.order);
            true
        });
        server.serve_cancels();
        server.serve_results(|id| {
            let order = orders.get(&id).copied().unwrap_or(0);
            (
                GoalStatus::Succeeded,
                example_interfaces__Fibonacci_Result {
                    sequence: RosSequence::alloc(fibonacci(order)),
                },
            )
        });
        for (id, order) in &orders {
            server.publish_feedback(
                *id,
                example_interfaces__Fibonacci_Feedback {
                    partial_sequence: RosSequence::alloc(fibonacci(*order)),
                },
            );
        }
        server.publish_status();
        std::thread::sleep(Duration::from_millis(50));
    }
}

#[test]
fn action_server_serves_action_client_loopback() {
    let stop = Arc::new(AtomicBool::new(false));
    let server_stop = Arc::clone(&stop);
    let server = std::thread::spawn(move || run_server(&server_stop));

    let dds = Dds::new(0);
    let mut client: FibClient = ActionClient::new(&dds, "/fibonacci_generic");

    // Block until discovery has matched every service endpoint (a fixed sleep
    // flakes under full-suite load), then still retry: matched != flushed.
    let ready_deadline = Instant::now() + Duration::from_secs(15);
    while !client.server_is_ready() && Instant::now() < ready_deadline {
        std::thread::sleep(Duration::from_millis(20));
    }
    assert!(client.server_is_ready(), "server endpoints never matched");

    // Send the goal, retrying while discovery settles.
    let mut accepted = None;
    for _ in 0..10 {
        if let Some((id, ok)) = client.send_goal(
            example_interfaces__Fibonacci_Goal { order: 6 },
            Duration::from_secs(2),
        ) {
            assert!(ok, "server rejected the goal");
            accepted = Some(id);
            break;
        }
    }
    let goal_id = accepted.expect("goal was never accepted");

    // Collect feedback, a status sample for our goal, and the result.
    let deadline = Instant::now() + Duration::from_secs(30);
    let mut got_feedback = false;
    let mut got_status = false;
    let mut result = None;
    while Instant::now() < deadline && (result.is_none() || !got_feedback || !got_status) {
        for sample in client.poll_feedback() {
            if sample.goal_id == goal_id && !sample.feedback.partial_sequence.as_slice().is_empty()
            {
                got_feedback = true;
            }
        }
        if let Some(statuses) = client.poll_status() {
            if statuses.iter().any(|(id, _)| *id == goal_id) {
                got_status = true;
            }
        }
        if result.is_none() {
            if let Some(r) = client.get_result(goal_id, Duration::from_millis(200)) {
                result = Some(r);
            }
        } else {
            std::thread::sleep(Duration::from_millis(20));
        }
    }

    // Cancel policy: an unknown goal id gets ERROR_UNKNOWN_GOAL_ID.
    let mut cancel = None;
    for _ in 0..10 {
        if let Some(resp) = client.cancel_goal(GoalId([9; 16]), Duration::from_secs(2)) {
            cancel = Some(resp);
            break;
        }
    }

    stop.store(true, Ordering::Relaxed);
    let _ = server.join();

    let (status, mut res) = result.expect("no result received before timeout");
    assert_eq!(status, GoalStatus::Succeeded);
    let seq = res.sequence.as_slice().to_vec();
    unsafe { res.fini() };
    assert_eq!(seq, vec![0, 1, 1, 2, 3, 5], "fib(6) mismatch");
    assert!(got_feedback, "no feedback received");
    assert!(got_status, "no status array carried our goal");

    let cancel = cancel.expect("no cancel reply received");
    assert_eq!(
        cancel.return_code,
        CancelGoalResponse::ERROR_UNKNOWN_GOAL_ID
    );
    unsafe { cancel.fini() };
}

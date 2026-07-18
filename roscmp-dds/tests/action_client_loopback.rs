//! Loopback integration test: our `ActionClient` drives a minimal Fibonacci
//! action server (built from the same `Service`/publisher primitives the real
//! `fibonacci_action_server` bin uses) over a domain-0 loopback, asserting the
//! full goal → feedback → result handshake.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use roscmp_dds::action::{
    ActionClient, ActionNames, CancelGoalResponse, GoalId, GoalStatus, GoalStatusArrayMsg,
    GoalStatusMsg,
};
use roscmp_dds::msgs::{
    example_interfaces__Fibonacci_Feedback, example_interfaces__Fibonacci_FeedbackMessage,
    example_interfaces__Fibonacci_GetResult_Request,
    example_interfaces__Fibonacci_GetResult_Response, example_interfaces__Fibonacci_Goal,
    example_interfaces__Fibonacci_Result, example_interfaces__Fibonacci_SendGoal_Request,
    example_interfaces__Fibonacci_SendGoal_Response, unique_identifier_msgs__UUID, RosSequence,
};
use roscmp_dds::service::Service;
use roscmp_dds::time::Time;
use roscmp_dds::transport::{Dds, MsgPublisher, Qos, Transport};

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

/// Run a minimal Fibonacci action server on domain 0 until `stop` is set. Each
/// accepted goal is remembered; feedback is published continuously for every
/// live goal, `get_result` returns the completed sequence, and the status array
/// is latched.
fn run_server(stop: &Arc<AtomicBool>) {
    let dds = Dds::new(0);
    let names = ActionNames::new("/fibonacci");
    let mut send_goal = Service::<
        example_interfaces__Fibonacci_SendGoal_Request,
        example_interfaces__Fibonacci_SendGoal_Response,
    >::new(&dds, &names.send_goal);
    let mut get_result = Service::<
        example_interfaces__Fibonacci_GetResult_Request,
        example_interfaces__Fibonacci_GetResult_Response,
    >::new(&dds, &names.get_result);
    // A real action server always offers cancellation; the client's
    // server_is_ready() rightly refuses to report ready without it.
    let mut cancel_goal = Service::<roscmp_dds::action::CancelGoalRequest, CancelGoalResponse>::new(
        &dds,
        &names.cancel_goal,
    );
    let feedback = dds
        .publisher::<example_interfaces__Fibonacci_FeedbackMessage>(&names.feedback, Qos::Default);
    let status = dds.publisher::<GoalStatusArrayMsg>(&names.status, Qos::Latched);

    let mut goals: Vec<(GoalId, i32)> = Vec::new();

    while !stop.load(Ordering::Relaxed) {
        let now = Time::now_system();
        send_goal.serve_pending(|req| {
            let goal_id = GoalId(req.goal_id.uuid);
            goals.push((goal_id, req.goal.order));
            example_interfaces__Fibonacci_SendGoal_Response {
                accepted: true,
                stamp: now.to_msg(),
            }
        });

        cancel_goal
            .serve_pending(|_req| CancelGoalResponse::empty(CancelGoalResponse::ERROR_REJECTED));

        get_result.serve_pending(|req| {
            let goal_id = GoalId(req.goal_id.uuid);
            let order = goals
                .iter()
                .find(|(id, _)| *id == goal_id)
                .map_or(0, |(_, order)| *order);
            example_interfaces__Fibonacci_GetResult_Response {
                status: GoalStatus::Succeeded as i8,
                result: example_interfaces__Fibonacci_Result {
                    sequence: RosSequence::alloc(fibonacci(order)),
                },
            }
        });

        for (goal_id, order) in &goals {
            feedback.publish(example_interfaces__Fibonacci_FeedbackMessage {
                goal_id: unique_identifier_msgs__UUID { uuid: goal_id.0 },
                feedback: example_interfaces__Fibonacci_Feedback {
                    partial_sequence: RosSequence::alloc(fibonacci(*order)),
                },
            });
        }
        let statuses: Vec<GoalStatusMsg> = goals
            .iter()
            .map(|(goal_id, _)| GoalStatusMsg::new(*goal_id, now, GoalStatus::Succeeded))
            .collect();
        status.publish(GoalStatusArrayMsg::new(statuses));

        std::thread::sleep(Duration::from_millis(50));
    }
}

#[test]
fn action_client_goal_feedback_result_loopback() {
    let stop = Arc::new(AtomicBool::new(false));
    let server_stop = Arc::clone(&stop);
    let server = std::thread::spawn(move || run_server(&server_stop));

    let dds = Dds::new(0);
    let mut client: FibClient = ActionClient::new(&dds, "/fibonacci");

    // Block until discovery has matched every service endpoint (a fixed sleep
    // flakes under full-suite load), then still retry: matched != flushed.
    let ready_deadline = Instant::now() + Duration::from_secs(15);
    while !client.server_is_ready() && Instant::now() < ready_deadline {
        std::thread::sleep(Duration::from_millis(20));
    }
    assert!(client.server_is_ready(), "server endpoints never matched");

    // Send the goal, retrying while discovery settles (volatile writers drop
    // requests sent before the reply reader is matched).
    let mut accepted = None;
    for _ in 0..10 {
        if let Some((id, ok)) = client.send_goal(
            example_interfaces__Fibonacci_Goal { order: 5 },
            Duration::from_secs(2),
        ) {
            assert!(ok, "server rejected the goal");
            accepted = Some(id);
            break;
        }
    }
    let goal_id = accepted.expect("goal was never accepted");

    // Collect feedback and the result within a bounded window.
    let deadline = Instant::now() + Duration::from_secs(30);
    let mut got_feedback = false;
    let mut result = None;
    // The server feeds back every 50ms for as long as it runs, so keep polling
    // until BOTH arrive: the result can beat the first feedback sample.
    while Instant::now() < deadline && (result.is_none() || !got_feedback) {
        for sample in client.poll_feedback() {
            if sample.goal_id == goal_id {
                let seq = sample.feedback.partial_sequence.as_slice();
                if !seq.is_empty() {
                    got_feedback = true;
                }
            }
        }
        if result.is_none() {
            if let Some((status, res)) = client.get_result(goal_id, Duration::from_millis(200)) {
                result = Some((status, res));
            }
        } else {
            std::thread::sleep(Duration::from_millis(20));
        }
    }

    stop.store(true, Ordering::Relaxed);
    let _ = server.join();

    let (status, mut res) = result.expect("no result received before timeout");
    assert_eq!(status, GoalStatus::Succeeded);
    let seq = res.sequence.as_slice().to_vec();
    unsafe { res.fini() };
    assert_eq!(seq, vec![0, 1, 1, 2, 3], "fib(5) mismatch");
    assert!(got_feedback, "no feedback received");
}

#[test]
fn action_client_cancel_reaches_server() {
    // A stand-alone server that rejects cancellation, to exercise the client's
    // cancel_goal path end to end over the wire.
    let stop = Arc::new(AtomicBool::new(false));
    let server_stop = Arc::clone(&stop);
    let server = std::thread::spawn(move || {
        let dds = Dds::new(0);
        let names = ActionNames::new("/fibonacci_cancel");
        let mut cancel = Service::<roscmp_dds::action::CancelGoalRequest, CancelGoalResponse>::new(
            &dds,
            &names.cancel_goal,
        );
        while !server_stop.load(Ordering::Relaxed) {
            cancel.serve_pending(|_req| {
                CancelGoalResponse::empty(CancelGoalResponse::ERROR_REJECTED)
            });
            std::thread::sleep(Duration::from_millis(50));
        }
    });

    let dds = Dds::new(0);
    let mut client: FibClient = ActionClient::new(&dds, "/fibonacci_cancel");
    // This mini-server only offers the cancel service, so server_is_ready()
    // (which needs all three) never turns true; the retry loop below is the
    // discovery wait.
    let mut resp = None;
    for _ in 0..15 {
        if let Some(r) = client.cancel_goal(GoalId::generate(), Duration::from_secs(2)) {
            resp = Some(r);
            break;
        }
    }

    stop.store(true, Ordering::Relaxed);
    let _ = server.join();

    let resp = resp.expect("no cancel reply received");
    assert_eq!(resp.return_code, CancelGoalResponse::ERROR_REJECTED);
    unsafe { resp.fini() };
}

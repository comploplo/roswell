//! Minimal `example_interfaces/action/Fibonacci` action server.

use std::collections::HashMap;
use std::time::Duration;

use roscmp_dds::action::{
    ActionNames, CancelGoalRequest, CancelGoalResponse, GoalId, GoalStatus, GoalStatusArrayMsg,
    GoalStatusMsg,
};
use roscmp_dds::msgs::{
    example_interfaces__Fibonacci_Feedback, example_interfaces__Fibonacci_FeedbackMessage,
    example_interfaces__Fibonacci_GetResult_Request,
    example_interfaces__Fibonacci_GetResult_Response, example_interfaces__Fibonacci_Result,
    example_interfaces__Fibonacci_SendGoal_Request,
    example_interfaces__Fibonacci_SendGoal_Response, unique_identifier_msgs__UUID, RosSequence,
};
use roscmp_dds::service::Service;
use roscmp_dds::time::Time;
use roscmp_dds::transport::{Dds, MsgPublisher, Qos, Transport};

fn main() {
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
    let mut cancel_goal =
        Service::<CancelGoalRequest, CancelGoalResponse>::new(&dds, &names.cancel_goal);
    let feedback = dds
        .publisher::<example_interfaces__Fibonacci_FeedbackMessage>(&names.feedback, Qos::Default);
    let status = dds.publisher::<GoalStatusArrayMsg>(&names.status, Qos::Latched);
    let mut goals = HashMap::new();

    println!("fibonacci_action_server: serving /fibonacci for 45s");
    for _ in 0..450 {
        let now = Time::now_system();
        send_goal.serve_pending(|req| {
            let goal_id = GoalId(req.goal_id.uuid);
            let order = req.goal.order;
            goals.insert(goal_id, order);
            feedback.publish(feedback_msg(goal_id, &[0, 1]));
            example_interfaces__Fibonacci_SendGoal_Response {
                accepted: true,
                stamp: now.to_msg(),
            }
        });

        get_result.serve_pending(|req| {
            let goal_id = GoalId(req.goal_id.uuid);
            let order = goals.get(&goal_id).copied().unwrap_or(0);
            example_interfaces__Fibonacci_GetResult_Response {
                status: GoalStatus::Succeeded as i8,
                result: example_interfaces__Fibonacci_Result {
                    sequence: RosSequence::alloc(fibonacci(order)),
                },
            }
        });

        cancel_goal
            .serve_pending(|_req| CancelGoalResponse::empty(CancelGoalResponse::ERROR_REJECTED));

        let statuses = goals
            .keys()
            .copied()
            .map(|goal_id| GoalStatusMsg::new(goal_id, now, GoalStatus::Succeeded))
            .collect();
        status.publish(GoalStatusArrayMsg::new(statuses));
        std::thread::sleep(Duration::from_millis(100));
    }
}

fn fibonacci(order: i32) -> Vec<i32> {
    // `order` is untrusted (off the wire). fib(47) overflows i32, so clamp to
    // the honest ceiling: this bounds the allocation and avoids add overflow.
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

fn feedback_msg(
    goal_id: GoalId,
    partial_sequence: &[i32],
) -> example_interfaces__Fibonacci_FeedbackMessage {
    example_interfaces__Fibonacci_FeedbackMessage {
        goal_id: unique_identifier_msgs__UUID { uuid: goal_id.0 },
        feedback: example_interfaces__Fibonacci_Feedback {
            partial_sequence: RosSequence::alloc(partial_sequence.to_vec()),
        },
    }
}

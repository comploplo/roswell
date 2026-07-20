//! Minimal `example_interfaces/action/Fibonacci` action server, on the generic
//! [`ActionServer`].

use std::collections::HashMap;
use std::time::Duration;

use roswell_ros2_compat::action::{ActionServer, GoalId, GoalStatus};
use roswell_ros2_compat::msgs::{
    example_interfaces__Fibonacci_Feedback, example_interfaces__Fibonacci_FeedbackMessage,
    example_interfaces__Fibonacci_GetResult_Request,
    example_interfaces__Fibonacci_GetResult_Response, example_interfaces__Fibonacci_Result,
    example_interfaces__Fibonacci_SendGoal_Request,
    example_interfaces__Fibonacci_SendGoal_Response, RosSequence,
};
use roswell_ros2_compat::transport::Dds;

fn main() {
    let dds = Dds::new(0);
    let mut server: ActionServer<
        example_interfaces__Fibonacci_SendGoal_Request,
        example_interfaces__Fibonacci_SendGoal_Response,
        example_interfaces__Fibonacci_GetResult_Request,
        example_interfaces__Fibonacci_GetResult_Response,
        example_interfaces__Fibonacci_FeedbackMessage,
    > = ActionServer::new(&dds, "/fibonacci");
    let mut goals: HashMap<GoalId, i32> = HashMap::new();

    println!("fibonacci_action_server: serving /fibonacci for 45s");
    for _ in 0..450 {
        let mut accepted = Vec::new();
        server.serve_goals(|goal_id, goal| {
            goals.insert(goal_id, goal.order);
            accepted.push(goal_id);
            true
        });
        for goal_id in accepted {
            server.publish_feedback(
                goal_id,
                example_interfaces__Fibonacci_Feedback {
                    partial_sequence: RosSequence::alloc(vec![0, 1]),
                },
            );
        }

        server.serve_results(|goal_id| {
            let order = goals.get(&goal_id).copied().unwrap_or(0);
            (
                GoalStatus::Succeeded,
                example_interfaces__Fibonacci_Result {
                    sequence: RosSequence::alloc(fibonacci(order)),
                },
            )
        });

        server.serve_cancels();
        server.publish_status();
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

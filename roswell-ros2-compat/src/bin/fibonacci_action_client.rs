//! Minimal `example_interfaces/action/Fibonacci` action client.
//!
//! Sends one goal (order from argv, default 5), prints feedback as it arrives,
//! then prints the final result. Uses the standard ROS2 action protocol and QoS
//! so it talks to either our `fibonacci_action_server` or a vanilla ROS2 one.

use std::time::{Duration, Instant};

use roswell_ros2_compat::action::FibonacciActionClient;
use roswell_ros2_compat::msgs::{
    example_interfaces__Fibonacci_Feedback, example_interfaces__Fibonacci_Goal,
};
use roswell_ros2_compat::transport::Dds;

fn main() {
    let order: i32 = std::env::args()
        .nth(1)
        .and_then(|a| a.parse().ok())
        .unwrap_or(5);

    let dds = Dds::new(0);
    let mut client = FibonacciActionClient::new(&dds, "/fibonacci");

    // Give discovery time to match the three services and two subscriptions
    // before the first write; a volatile writer drops a request sent unmatched.
    println!("fibonacci_action_client: waiting for /fibonacci ...");
    std::thread::sleep(Duration::from_secs(2));

    // Send the goal, retrying while discovery settles.
    let mut goal = None;
    for _ in 0..10 {
        if let Some((id, accepted)) = client.send_goal(
            example_interfaces__Fibonacci_Goal { order },
            Duration::from_secs(2),
        ) {
            println!(
                "goal {}: {}",
                hex(&id.0),
                if accepted { "accepted" } else { "rejected" }
            );
            if accepted {
                goal = Some(id);
            }
            break;
        }
    }
    let Some(goal_id) = goal else {
        eprintln!("no goal accepted (is a /fibonacci server running?)");
        std::process::exit(1);
    };

    // Poll feedback while waiting for the result. Feedback and result race, so
    // keep draining feedback until the result lands or we time out.
    let deadline = Instant::now() + Duration::from_secs(30);
    let mut result = None;
    while Instant::now() < deadline {
        for sample in client.poll_feedback() {
            if sample.goal_id == goal_id {
                print_feedback(&sample.feedback);
            }
        }
        if let Some((status, res)) = client.get_result(goal_id, Duration::from_millis(200)) {
            result = Some((status, res));
            break;
        }
    }

    // Drain any feedback that arrived alongside the result.
    for sample in client.poll_feedback() {
        if sample.goal_id == goal_id {
            print_feedback(&sample.feedback);
        }
    }

    let Some((status, mut res)) = result else {
        eprintln!("timed out waiting for result");
        std::process::exit(1);
    };
    let seq = res.sequence.as_slice().to_vec();
    println!("result ({status:?}): {seq:?}");
    // SAFETY: `res` owns the sequence and is finalized once here.
    unsafe { res.fini() };
}

fn print_feedback(feedback: &example_interfaces__Fibonacci_Feedback) {
    let partial = feedback.partial_sequence.as_slice();
    println!("feedback: {partial:?}");
}

fn hex(bytes: &[u8; 16]) -> String {
    use std::fmt::Write as _;
    bytes.iter().fold(String::with_capacity(32), |mut s, b| {
        let _ = write!(s, "{b:02x}");
        s
    })
}

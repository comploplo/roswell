//! Round-trips generated messages through their CDR codec and the
//! `roscmp_dds` trait impls, proving the OUT_DIR bindings really compile and
//! encode/decode against the runtime crate.

#[allow(non_camel_case_types, non_upper_case_globals, dead_code, clippy::all, clippy::pedantic)]
mod msgs {
    include!(concat!(env!("OUT_DIR"), "/roscmp_msgs.rs"));
}

use msgs::{
    example_interfaces__AddTwoInts_Request, example_interfaces__Fibonacci_SendGoal_Request,
    example_interfaces__Fibonacci_Goal, geometry_msgs__Twist, geometry_msgs__Vector3, Endian,
};
use roscmp_dds::action::SendGoalRequest;
use roscmp_dds::codec::CdrMsg;

fn main() {
    // Message: round-trip through the CdrMsg impl (encode = header + body).
    let twist = geometry_msgs__Twist {
        linear: geometry_msgs__Vector3 {
            x: 1.5,
            y: -2.0,
            z: 0.25,
        },
        angular: geometry_msgs__Vector3 {
            x: 0.0,
            y: 0.5,
            z: -1.0,
        },
    };
    assert_eq!(
        geometry_msgs__Twist::TYPE_NAME,
        "geometry_msgs::msg::dds_::Twist_"
    );
    let back = geometry_msgs__Twist::decode(&twist.encode()).expect("twist decode");
    assert_eq!(back.linear.x, 1.5);
    assert_eq!(back.angular.z, -1.0);

    // Service request: round-trip through the raw to_cdr/from_cdr pair.
    let req = example_interfaces__AddTwoInts_Request { a: 7, b: -9 };
    let back = example_interfaces__AddTwoInts_Request::from_cdr(&req.to_cdr(Endian::Big))
        .expect("addtwoints decode");
    assert_eq!((back.a, back.b), (7, -9));

    // Action wire type: the generated roscmp_dds action-trait impls resolve.
    let sg = example_interfaces__Fibonacci_SendGoal_Request {
        goal_id: msgs::unique_identifier_msgs__UUID { uuid: [7; 16] },
        goal: example_interfaces__Fibonacci_Goal { order: 6 },
    };
    assert_eq!(sg.goal_id(), roscmp_dds::action::GoalId([7; 16]));
    assert_eq!(sg.goal().order, 6);
    let back = example_interfaces__Fibonacci_SendGoal_Request::decode(&sg.encode())
        .expect("send-goal decode");
    assert_eq!(back.goal.order, 6);

    println!("roundtrip ok");
}

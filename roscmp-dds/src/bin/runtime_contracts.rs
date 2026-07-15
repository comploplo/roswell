//! Exercise the core ROS runtime contracts from one node.
//!
//! Publishes `/clock`, `/rosout`, `/tf`, `/tf_static` and serves
//! `/runtime_contracts/change_state`, `/runtime_contracts/get_state`, and
//! `/runtime_contracts/get_type_description`, plus ROS parameter services.

use std::time::Duration as StdDuration;

use roscmp_dds::diagnostics::{DiagnosticLevel, DiagnosticStatus, Diagnostics, KeyValue};
use roscmp_dds::lifecycle::LifecycleServices;
use roscmp_dds::log::{LogSite, Rosout, Severity};
use roscmp_dds::msgs;
use roscmp_dds::node::Node;
use roscmp_dds::parameters::{ParameterDescriptor, ParameterType, ParameterValue};
use roscmp_dds::tf::{StaticTfBroadcaster, TfBroadcaster, Transform};
use roscmp_dds::time::{ClockMsg, Time};
use roscmp_dds::transport::{MsgPublisher, Qos};
use roscmp_dds::type_description::{TypeDescriptionRegistry, TypeDescriptionService};

fn main() {
    let mut node = Node::new("runtime_contracts");
    let clock = node.publisher::<ClockMsg>("/clock", Qos::Default);
    let mut rosout = Rosout::new(node.dds(), "runtime_contracts");
    let diagnostics = Diagnostics::new(node.dds());
    let tf = TfBroadcaster::new(node.dds());
    let tf_static = StaticTfBroadcaster::new(node.dds());

    let mut lifecycle = LifecycleServices::new(node.dds(), "runtime_contracts");
    let mut registry = TypeDescriptionRegistry::new();
    msgs::register_type_descriptions(&mut registry);
    let mut type_descriptions =
        TypeDescriptionService::new(node.dds(), "runtime_contracts", registry);
    let mut parameters = node.parameter_server();
    parameters.declare(
        ParameterDescriptor {
            description: "Example runtime parameter served over rcl_interfaces".into(),
            ..ParameterDescriptor::new("robot_name", ParameterType::String)
        },
        ParameterValue::String("roscmp".into()),
    );
    parameters.declare(
        ParameterDescriptor::new("max_speed", ParameterType::Double),
        ParameterValue::Double(1.0),
    );

    tf_static.send(
        "map",
        "odom",
        Transform {
            translation: [1.0, 0.0, 0.0],
            rotation: [0.0, 0.0, 0.0, 1.0],
        },
    );

    let site = LogSite::new(file!(), "runtime_contracts", line!());
    println!("runtime_contracts: serving runtime contracts for 45s");
    let mut i: i64 = 0;
    node.create_timer(StdDuration::from_millis(500), move |_dds| {
        let stamp = Time::from_millis(i * 500);
        clock.publish(ClockMsg {
            clock: stamp.to_msg(),
        });
        rosout.log(stamp, Severity::Info, "runtime contracts online", site);
        let mut status = DiagnosticStatus::new(
            DiagnosticLevel::Ok,
            "runtime_contracts",
            "runtime contracts online",
        );
        status.values.push(KeyValue::new("tick", i.to_string()));
        diagnostics.publish(stamp, vec![status]);
        tf.send(
            stamp,
            "odom",
            "base_link",
            Transform {
                translation: [f64::from(i as u32) * 0.01, 0.0, 0.0],
                rotation: [0.0, 0.0, 0.0, 1.0],
            },
        );

        let served = lifecycle.serve_pending()
            + type_descriptions.serve_pending()
            + parameters.serve_pending();
        if served > 0 {
            println!("runtime_contracts: served {served} requests");
        }
        i += 1;
    });
    node.spin_for(StdDuration::from_secs(45));
}

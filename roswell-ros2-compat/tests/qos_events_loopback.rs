use std::time::{Duration, Instant};

use roswell_ros2_compat::msgs::std_msgs__String;
use roswell_ros2_compat::qos::QosEvent;
use roswell_ros2_compat::transport::{Dds, Qos, Transport};

#[test]
fn incompatible_qos_surfaces_events_on_loopback() {
    let dds = Dds::new(0);
    // Best-effort writer cannot satisfy a reliable reader => incompatible QoS.
    let mut writer = dds.publisher::<std_msgs__String>("/qos_evt_test", Qos::SensorData);
    let mut reader = dds.subscriber::<std_msgs__String>("/qos_evt_test", Qos::Default);

    let deadline = Instant::now() + Duration::from_secs(10);
    let mut writer_evts = Vec::new();
    let mut reader_evts = Vec::new();
    while Instant::now() < deadline {
        writer_evts.extend(writer.poll_events());
        reader_evts.extend(reader.poll_events());
        let saw_incompat = writer_evts
            .iter()
            .chain(&reader_evts)
            .any(|e| matches!(e, QosEvent::IncompatibleQos { .. }));
        if saw_incompat {
            return;
        }
        std::thread::sleep(Duration::from_millis(50));
    }
    panic!(
        "no IncompatibleQos event within timeout; writer={writer_evts:?} reader={reader_evts:?}"
    );
}

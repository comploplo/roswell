use std::path::Path;

fn main() {
    let samples = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../../samples");
    roscmp_build::Config::new()
        .type_paths([samples])
        .compile([
            "geometry_msgs/msg/Twist",
            "example_interfaces/srv/AddTwoInts",
            "example_interfaces/action/Fibonacci",
        ])
        .expect("roscmp-build codegen failed");
}

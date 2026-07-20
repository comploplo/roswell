"""MCAP bag write/read roundtrip through the Rust reader/writer."""

import pytest

import roswell
import roswell.bag


@pytest.fixture
def node():
    n = roswell.Node("bag_node", domain=0)
    try:
        yield n
    finally:
        n.close()


def test_write_read_roundtrip(node, tmp_path):
    path = tmp_path / "round.mcap"
    String = node.load_type("std_msgs/msg/String")
    Imu = node.load_type("sensor_msgs/msg/Imu")

    with roswell.bag.open_write(path) as bag:
        s = String.alloc()
        s.data = "hello bag"
        bag.write("/chatter", s, 1_000)
        s.close()
        imu = Imu.alloc()
        imu.linear_acceleration.x = 9.81
        imu.orientation_covariance = [float(i) for i in range(9)]
        bag.write("/imu", imu, 2_000)
        imu.close()

    samples = list(roswell.bag.read(path))
    assert [(m.topic, m.type, m.log_time) for m in samples] == [
        ("/chatter", "std_msgs/msg/String", 1_000),
        ("/imu", "sensor_msgs/msg/Imu", 2_000),
    ]
    chatter, imu_s = samples
    assert not chatter.raw
    assert chatter.message.data == "hello bag"
    assert not imu_s.raw
    assert imu_s.message.linear_acceleration.x == 9.81
    assert list(imu_s.message.orientation_covariance) == [float(i) for i in range(9)]


def test_read_raw_bytes(node, tmp_path):
    path = tmp_path / "raw.mcap"
    String = node.load_type("std_msgs/msg/String")
    with roswell.bag.open_write(path, compression="none") as bag:
        s = String.alloc()
        s.data = "raw"
        bag.write("/chatter", s, 5)
        s.close()

    (sample,) = roswell.bag.read(path, decode=False)
    assert sample.raw
    assert isinstance(sample.message, bytes)
    # Full CDR: little-endian encapsulation header, then u32 length + "raw\0".
    assert sample.message[:2] == b"\x00\x01"
    assert b"raw" in sample.message


def test_unresolvable_type_falls_back_to_raw(tmp_path, fixture_dir):
    ws = fixture_dir / "ws"
    node = roswell.Node("bag_ws_node", domain=0, type_paths=[ws])
    try:
        Detection = node.load_type("robot_msgs/msg/Detection")
        path = tmp_path / "custom.mcap"
        with roswell.bag.open_write(path) as bag:
            d = Detection.alloc()
            d.label = "cone"
            bag.write("/detections", d, 7)
            d.close()

        # Without the workspace root the type cannot resolve: raw fallback.
        (sample,) = roswell.bag.read(path)
        assert sample.type == "robot_msgs/msg/Detection"
        assert sample.raw
        assert b"cone" in sample.message

        # With the workspace root it decodes.
        (decoded,) = roswell.bag.read(path, type_paths=[ws])
        assert not decoded.raw
        assert decoded.message.label == "cone"
    finally:
        node.close()


def test_writer_close_is_idempotent_and_repr(node, tmp_path):
    bag = roswell.bag.open_write(tmp_path / "x.mcap")
    assert "open" in repr(bag)
    bag.close()
    bag.close()
    assert "closed" in repr(bag)

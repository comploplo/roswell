"""Reference-based type resolution across a synthetic multi-package workspace.

Resolution and dependency walking happen in Rust (``rcm_type_resolve``); these
tests prove the Python ``type_paths`` surface finds custom packages laid out both
as a plain colcon tree and as an ament ``share/`` install prefix, and that nested
cross-package dependencies (into a second user package and into the bundled
sample tree) resolve.
"""

import time

import numpy as np

import roscmp


def _ws(fixture_dir):
    return fixture_dir / "ws"


def test_resolves_custom_type_with_bundled_and_cross_package_deps(fixture_dir):
    # Detection -> geometry_msgs/Point (bundled) + sensor_pkg/Reading (2nd user pkg)
    node = roscmp.Node("ws_node", domain=0, type_paths=[_ws(fixture_dir)])
    try:
        T = node.load_type("robot_msgs/msg/Detection")
        assert T.dds_type == "robot_msgs::msg::dds_::Detection_"
        ids = set(T.messages.keys())
        # nested from the bundled sample tree
        assert "geometry_msgs/Point" in ids
        assert "std_msgs/Header" in ids
        # nested from the second user package
        assert "sensor_pkg/Reading" in ids

        # And the fields are actually usable.
        msg = T.alloc()
        msg.label = "cone"
        msg.position.x = 1.5
        msg.reading.value = 42.0
        msg.reading.units = "cm"
        assert msg.label == "cone"
        assert msg.position.x == 1.5
        assert msg.reading.units == "cm"
        msg.close()
    finally:
        node.close()


def test_ament_share_layout_resolves(fixture_dir):
    install = fixture_dir / "ws_install"
    node = roscmp.Node("ws_ament", domain=0, type_paths=[install])
    try:
        T = node.load_type("foo_msgs/msg/Beacon")
        assert T.dds_type == "foo_msgs::msg::dds_::Beacon_"
        assert "geometry_msgs/Vector3" in T.messages  # bundled nested dep
    finally:
        node.close()


def test_env_var_search_root(fixture_dir, monkeypatch):
    monkeypatch.setenv("ROSCMP_TYPE_PATH", str(_ws(fixture_dir)))
    node = roscmp.Node("ws_env", domain=0)
    try:
        T = node.load_type("sensor_pkg/msg/Reading")
        assert T.dds_type == "sensor_pkg::msg::dds_::Reading_"
    finally:
        node.close()


def test_custom_type_pubsub_roundtrip(fixture_dir):
    node = roscmp.Node("ws_pubsub", domain=0, type_paths=[_ws(fixture_dir)])
    try:
        T = node.load_type("robot_msgs/msg/Detection")
        pub = node.publisher("/ws_detection", T)
        sub = node.subscribe("/ws_detection", T)

        got = None
        for _ in range(50):
            msg = pub.new()
            msg.label = "hello"
            msg.reading.value = 3.5
            pub.publish(msg)
            r = sub.take()
            if r is not None and r.label:
                got = r
                break
            time.sleep(0.1)

        assert got is not None, "no message received within timeout"
        assert got.label == "hello"
        assert np.isclose(got.reading.value, 3.5)
    finally:
        node.close()


def test_unknown_reference_raises(fixture_dir):
    node = roscmp.Node("ws_err", domain=0, type_paths=[_ws(fixture_dir)])
    try:
        try:
            node.load_type("nope_msgs/msg/Ghost")
        except roscmp.RoscmpError as e:
            assert "could not find" in str(e)
        else:
            raise AssertionError("expected RoscmpError for unknown reference")
    finally:
        node.close()

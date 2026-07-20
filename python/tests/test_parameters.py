"""Parameter declare/get/set/list plus /parameter_events, over real RTPS.

The node stands up a parameter server (a background thread answering
`ros2 param` verbs and publishing `/parameter_events`) on first parameter use.
CRUD is deterministic; the event assertion retries while discovery settles.
"""

import time

import roswell


def test_parameter_crud_roundtrip():
    node = roswell.Node("py_param_crud", domain=0)
    try:
        assert node.declare_parameter("speed", 1.5) == 1.5
        node.declare_parameter("gain", 7)
        node.declare_parameter("enabled", False)
        node.declare_parameter("frame", "base_link")

        assert node.get_parameter("speed") == 1.5
        assert node.get_parameter("gain") == 7
        assert node.get_parameter("enabled") is False
        assert node.get_parameter("frame") == "base_link"

        # Set overwrites and is observable through get.
        node.set_parameter("gain", 9)
        assert node.get_parameter("gain") == 9

        assert set(node.list_parameters()) == {"speed", "gain", "enabled", "frame"}
    finally:
        node.close()


def test_parameter_undeclared_raises():
    node = roswell.Node("py_param_missing", domain=0)
    try:
        with __import__("pytest").raises(roswell.RoswellError):
            node.get_parameter("nope")
    finally:
        node.close()


def test_parameter_events_published():
    node = roswell.Node("py_param_evt", domain=0)
    try:
        # Path-free reference resolves the dependency closure from samples/.
        Evt = node.load_type("rcl_interfaces/msg/ParameterEvent")
        sub = node.subscribe("/parameter_events", Evt)

        got = None
        for i in range(50):
            # Each set publishes a changed/new event; repeat until the reader,
            # matched after discovery, catches one.
            node.set_parameter("tick", i)
            msg = sub.take()
            if msg is not None and msg.node:
                got = msg.node
                break
            time.sleep(0.1)

        assert got is not None, "no /parameter_events sample received"
        assert got == "/py_param_evt"
    finally:
        node.close()

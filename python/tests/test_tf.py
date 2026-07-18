"""tf2 buffer loopback: broadcast static + dynamic transforms over real RTPS
(/tf_static latched, /tf default), then resolve the two-edge chain in Rust."""

import time

import pytest

import roscmp


@pytest.fixture
def node():
    n = roscmp.Node("tf_node", domain=0)
    try:
        yield n
    finally:
        n.close()


IDENTITY_Q = (0.0, 0.0, 0.0, 1.0)


def test_lookup_resolves_static_plus_dynamic_chain(node):
    tf = node.tf_buffer()
    static_t = roscmp.Transform(translation=(1.0, 2.0, 0.0), rotation=IDENTITY_Q)
    dynamic_t = roscmp.Transform(translation=(3.0, 0.0, 0.0), rotation=IDENTITY_Q)

    # Republish until DDS discovery lets the loopback samples land.
    deadline = time.monotonic() + 15.0
    while time.monotonic() < deadline:
        tf.broadcast("map", "odom", static_t, static=True)
        tf.broadcast("odom", "base", dynamic_t)
        if tf.can_transform("map", "base"):
            break
        time.sleep(0.1)
    assert tf.can_transform("map", "base"), "tf chain never resolved"

    got = tf.lookup_transform("map", "base")
    assert got.translation == (4.0, 2.0, 0.0)
    assert got.rotation == IDENTITY_Q

    # Reverse direction resolves through the inverse chain.
    back = tf.lookup_transform("base", "map")
    assert back.translation == (-4.0, -2.0, 0.0)

    tf.close()


def test_lookup_missing_frame_raises(node):
    tf = node.tf_buffer()
    assert not tf.can_transform("map", "nowhere")
    with pytest.raises(roscmp.RoscmpError):
        tf.lookup_transform("map", "nowhere")
    tf.close()

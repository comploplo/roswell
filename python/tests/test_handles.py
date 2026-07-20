"""Handle-table and view-lifetime safety, driven through the Python surface.

These exercise the hardening the C ABI added: numpy views pin their owning
message (so a view can't outlive the buffer it aliases), explicit ``close()`` is
idempotent and makes further access raise, and cross-type / stale handles map to
precise exceptions rather than crashing.
"""

import gc

import numpy as np
import pytest

import roswell


def _sample_type(node, fixture_dir):
    return node.load_type(fixture_dir / "test_msgs" / "msg" / "Sample.msg")


def test_view_keeps_message_alive(fixture_dir):
    node = roswell.Node("py_view", domain=0)
    try:
        T = _sample_type(node, fixture_dir)
        msg = T.alloc()
        msg.values = [1.0, 2.5, -3.25, 42.0]
        view = msg.values
        assert isinstance(view, np.ndarray)

        # Drop the only Python reference to the message and force collection.
        del msg
        gc.collect()

        # The view still aliases live memory because it pinned the message.
        np.testing.assert_allclose(view, [1.0, 2.5, -3.25, 42.0])

        # A slice keeps the chain alive too.
        sl = view[1:3]
        del view
        gc.collect()
        np.testing.assert_allclose(sl, [2.5, -3.25])
    finally:
        node.close()


def test_closed_message_access_raises(fixture_dir):
    node = roswell.Node("py_close", domain=0)
    try:
        T = _sample_type(node, fixture_dir)
        msg = T.alloc()
        msg.label = "hi"
        assert msg.label == "hi"

        msg.close()
        with pytest.raises(roswell.RoswellError):
            _ = msg.label
        with pytest.raises(roswell.RoswellError):
            msg.label = "again"
    finally:
        node.close()


def test_double_close_is_noop(fixture_dir):
    node = roswell.Node("py_dclose", domain=0)
    try:
        T = _sample_type(node, fixture_dir)
        msg = T.alloc()
        msg.close()
        msg.close()  # must not raise or double-free
    finally:
        node.close()


def test_wrong_type_publish_raises(fixture_dir, sample_dir):
    node = roswell.Node("py_mismatch", domain=0)
    try:
        T = _sample_type(node, fixture_dir)
        req_t, _resp_t = node.load_service(
            sample_dir / "example_interfaces" / "srv" / "AddTwoInts.srv"
        )
        pub = node.publisher("/py_mismatch", T)
        wrong = req_t.alloc()  # a different type than the publisher's
        with pytest.raises(roswell.TypeMismatchError):
            pub.publish(wrong)
    finally:
        node.close()


def test_use_after_shutdown_raises(fixture_dir):
    node = roswell.Node("py_uaf", domain=0)
    T = _sample_type(node, fixture_dir)
    pub = node.publisher("/py_uaf", T)
    msg = pub.new()
    node.close()  # shuts the context down, invalidating the publisher handle
    with pytest.raises(roswell.StaleHandleError):
        pub.publish(msg)

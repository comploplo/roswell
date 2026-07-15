"""End-to-end loopback tests over a real domain-0 RTPS transport.

Each test creates its own node and retries while DDS discovery settles, matching
the retry pattern used by the Rust loopback tests.
"""

import asyncio
import time
import warnings

import numpy as np
import pytest

import roscmp


def _sample_type(node, fixture_dir):
    return node.load_type(fixture_dir / "test_msgs" / "msg" / "Sample.msg")


def test_pubsub_roundtrip_string_and_float_array(fixture_dir):
    node = roscmp.Node("py_pubsub", domain=0)
    try:
        T = _sample_type(node, fixture_dir)
        pub = node.publisher("/py_sample", T)
        sub = node.subscribe("/py_sample", T)

        msg = pub.new()
        msg.label = "hello dynamic world"
        msg.values = [1.0, 2.5, -3.25, 42.0]

        got = None
        for _ in range(50):
            pub.publish(msg)
            r = sub.take()
            if r is not None and r.label:
                got = r
                break
            time.sleep(0.1)

        assert got is not None, "no message received within timeout"
        assert got.label == "hello dynamic world"

        # float64[] sequence exposed as a zero-copy numpy view.
        view = got.values
        assert isinstance(view, np.ndarray)
        assert view.dtype == np.float64
        np.testing.assert_allclose(view, [1.0, 2.5, -3.25, 42.0])
    finally:
        node.close()


def test_service_call_roundtrip(sample_dir):
    node = roscmp.Node("py_service", domain=0)
    try:
        req_t, resp_t = node.load_service(
            sample_dir / "example_interfaces" / "srv" / "AddTwoInts.srv"
        )

        def handler(req):
            resp = resp_t.alloc()
            resp.sum = req.a + req.b
            return resp

        svc = node.serve("/py_add", (req_t, resp_t), handler)
        client = node.client("/py_add", req_t, resp_t)
        time.sleep(2)

        req = client.new_request()
        req.a = 41
        req.b = 1

        reply = None
        for _ in range(20):
            try:
                reply = client.call_sync(req, timeout=2.0)
                break
            except roscmp.RoscmpTimeout:
                continue

        assert reply is not None, "no reply within timeout"
        assert reply.sum == 42
        svc.close()
    finally:
        node.close()


def test_incompatible_qos_warns(fixture_dir):
    node = roscmp.Node("py_qos", domain=0)
    try:
        T = _sample_type(node, fixture_dir)
        # best-effort publisher cannot satisfy a reliable subscriber.
        pub = node.publisher("/py_qos", T, qos=roscmp.QosProfile.preset("sensor_data"))
        sub = node.subscribe("/py_qos", T, qos=roscmp.QosProfile.preset("default"))

        msg = pub.new()
        msg.label = "x"

        caught = False
        with warnings.catch_warnings(record=True) as recorded:
            warnings.simplefilter("always")
            for _ in range(100):
                pub.publish(msg)
                sub.take()
                if any(
                    issubclass(w.category, roscmp.QosIncompatibleWarning)
                    for w in recorded
                ):
                    caught = True
                    break
                time.sleep(0.1)

        assert caught, "expected a QosIncompatibleWarning"
    finally:
        node.close()


def test_async_for_receives_messages(fixture_dir):
    async def run():
        node = roscmp.Node("py_async", domain=0)
        try:
            T = _sample_type(node, fixture_dir)
            pub = node.publisher("/py_async", T)

            async def keep_publishing():
                msg = pub.new()
                msg.label = "tick"
                for _ in range(80):
                    pub.publish(msg)
                    await asyncio.sleep(0.1)

            received = []
            async with node.subscribe("/py_async", T) as sub:
                task = asyncio.create_task(keep_publishing())
                try:
                    ait = sub.__aiter__()
                    for _ in range(3):
                        msg = await asyncio.wait_for(ait.__anext__(), timeout=10)
                        received.append(msg.label)
                finally:
                    task.cancel()
            return received
        finally:
            node.close()

    got = asyncio.run(run())
    assert len(got) == 3
    assert all(label == "tick" for label in got)

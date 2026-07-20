"""Loopback: our Python ``node.action_server`` vs our own ``node.action_client``
over real domain-0 RTPS — goal -> feedback -> result, plus cancel. The protocol
machinery (services, goal bookkeeping, status latching, result parking) runs in
Rust; Python supplies only the execute callback.
"""

import threading
import time

import pytest

import roswell

ACTION = "/fib_srv"
ACTION_TYPE = "example_interfaces/action/Fibonacci"

STATUS_SUCCEEDED = 4
STATUS_CANCELED = 5


def fibonacci(order: int) -> list:
    seq = [0, 1]
    for _ in range(order):
        seq.append(seq[-1] + seq[-2])
    return seq


def _execute(goal, handle):
    seq = [0, 1]
    for _ in range(int(goal.order)):
        if handle.is_cancel_requested:
            break
        seq.append(seq[-1] + seq[-2])
        fb = handle.new_feedback()
        fb.partial_sequence = seq
        handle.publish_feedback(fb)
        fb.close()
        time.sleep(0.05)
    result = handle.new_result()
    result.sequence = seq
    return result


@pytest.fixture
def server_and_client():
    server_node = roswell.Node("fib_srv_server", domain=0)
    client_node = roswell.Node("fib_srv_client", domain=0)
    cancel_events = []
    server = server_node.action_server(
        ACTION, ACTION_TYPE, _execute, cancel_callback=cancel_events.append
    )
    ac = client_node.action_client(ACTION, ACTION_TYPE)
    try:
        yield server, ac, cancel_events
    finally:
        ac.close()
        server.close()
        client_node.close()
        server_node.close()


def _send_goal(ac, order, tries=25):
    goal = ac.new_goal()
    goal.order = order
    for _ in range(tries):
        try:
            return ac.send_goal_sync(goal, timeout=2.0)
        except roswell.RoswellTimeout:
            continue
    pytest.fail("send_goal never got a reply")


def _get_result(ac, goal_id, tries=25):
    for _ in range(tries):
        try:
            return ac.get_result_sync(goal_id, timeout=2.0)
        except roswell.RoswellTimeout:
            continue
    pytest.fail("get_result never got a reply")


def test_goal_feedback_result(server_and_client):
    server, ac, _cancels = server_and_client
    assert ac.wait_for_server(timeout=15), "action server never discovered"

    goal_id, accepted = _send_goal(ac, 5)
    assert accepted
    assert len(goal_id) == 16

    feedbacks = []
    for fb in ac.feedback(timeout=4.0):
        feedbacks.append(fb)
        break
    assert feedbacks, "no feedback received"
    assert feedbacks[0].goal_id == goal_id
    assert len(list(feedbacks[0].message.partial_sequence)) >= 2

    status, result = _get_result(ac, goal_id)
    assert status == STATUS_SUCCEEDED
    assert list(result.sequence) == fibonacci(5)


def test_cancel_mid_goal(server_and_client):
    server, ac, cancels = server_and_client
    assert ac.wait_for_server(timeout=15)

    goal_id, accepted = _send_goal(ac, 60)  # long enough to cancel mid-flight
    assert accepted
    code = ac.cancel_sync(goal_id, timeout=5.0)
    assert code == 0  # ERROR_NONE

    status, result = _get_result(ac, goal_id)
    assert status == STATUS_CANCELED
    assert len(list(result.sequence)) < len(fibonacci(60))

    # The optional cancel callback fired once for the running goal.
    deadline = time.monotonic() + 2.0
    while not cancels and time.monotonic() < deadline:
        time.sleep(0.02)
    assert len(cancels) == 1
    assert cancels[0].goal_id == goal_id


def test_async_execute_callback():
    server_node = roswell.Node("fib_async_server", domain=0)
    client_node = roswell.Node("fib_async_client", domain=0)

    async def execute(goal, handle):
        result = server.new_result()
        result.sequence = fibonacci(int(goal.order))
        return result

    server = server_node.action_server(ACTION + "_async", ACTION_TYPE, execute)
    ac = client_node.action_client(ACTION + "_async", ACTION_TYPE)
    try:
        assert ac.wait_for_server(timeout=15)
        goal_id, accepted = _send_goal(ac, 4)
        assert accepted
        status, result = _get_result(ac, goal_id)
        assert status == STATUS_SUCCEEDED
        assert list(result.sequence) == fibonacci(4)
    finally:
        ac.close()
        server.close()
        client_node.close()
        server_node.close()

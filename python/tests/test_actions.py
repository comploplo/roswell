"""End-to-end action loopback over a real domain-0 RTPS transport.

The action *server* is built entirely from the existing Python primitives —
``node.serve`` for the three action services and ``node.publisher`` for the
feedback topic — using the wrapper types from ``node.load_action``. The client is
the runtime-typed :class:`roswell.ActionClient`. This exercises the full protocol:
send_goal -> feedback -> get_result, plus cancel.
"""

import threading
import time

import pytest

import roswell

ACTION = "/fib"
ACTION_TYPE = "example_interfaces/action/Fibonacci"

# action_msgs/msg/GoalStatus values
STATUS_SUCCEEDED = 4
STATUS_CANCELED = 5


def fibonacci(order: int) -> list:
    seq = [0, 1]
    for _ in range(order):
        seq.append(seq[-1] + seq[-2])
    return seq


def _uuid_bytes(uuid_field) -> bytes:
    return bytes(bytearray(uuid_field))


class FibServer:
    """A minimal Fibonacci action server over the plain service/topic API."""

    def __init__(self, node, action_name, types, cancel_types):
        self.types = types
        self._goals = {}
        self._lock = threading.Lock()
        self._fb_pub = node.publisher(action_name + "/_action/feedback", types.feedback_message)
        self._sg = node.serve(
            action_name + "/_action/send_goal",
            (types.send_goal_request, types.send_goal_response),
            self._on_send_goal,
        )
        self._gr = node.serve(
            action_name + "/_action/get_result",
            (types.get_result_request, types.get_result_response),
            self._on_get_result,
        )
        self._cg = node.serve(
            action_name + "/_action/cancel_goal",
            cancel_types,
            self._on_cancel,
        )

    def _canceled(self, gid) -> bool:
        with self._lock:
            g = self._goals.get(gid)
            return bool(g and g["canceled"])

    def _on_send_goal(self, req):
        gid = _uuid_bytes(req.goal_id.uuid)
        order = int(req.goal.order)
        with self._lock:
            self._goals[gid] = {"order": order, "seq": None, "canceled": False}
        threading.Thread(target=self._process, args=(gid, order), daemon=True).start()
        resp = self.types.send_goal_response.alloc()
        resp.accepted = True
        return resp

    def _process(self, gid, order):
        seq = [0, 1]
        for _ in range(order):
            seq.append(seq[-1] + seq[-2])
            if self._canceled(gid):
                return
            fb = self.types.feedback_message.alloc()
            fb.goal_id.uuid = list(gid)
            fb.feedback.partial_sequence = seq
            self._fb_pub.publish(fb)
            fb.close()
            time.sleep(0.05)
        with self._lock:
            g = self._goals.get(gid)
            if g and not g["canceled"]:
                g["seq"] = seq

    def _on_get_result(self, req):
        gid = _uuid_bytes(req.goal_id.uuid)
        g = None
        for _ in range(300):
            with self._lock:
                g = self._goals.get(gid)
            if g and (g["seq"] is not None or g["canceled"]):
                break
            time.sleep(0.01)
        resp = self.types.get_result_response.alloc()
        if g and g["canceled"]:
            resp.status = STATUS_CANCELED
            resp.result.sequence = []
        elif g and g["seq"] is not None:
            resp.status = STATUS_SUCCEEDED
            resp.result.sequence = g["seq"]
        else:
            resp.status = 6  # aborted
        return resp

    def _on_cancel(self, req):
        gid = _uuid_bytes(req.goal_info.goal_id.uuid)
        with self._lock:
            g = self._goals.get(gid)
            code = 0 if g else 2  # ERROR_NONE / ERROR_UNKNOWN_GOAL_ID
            if g:
                g["canceled"] = True
        resp = self._cg.new_response()
        resp.return_code = code
        return resp

    def close(self):
        self._sg.close()
        self._gr.close()
        self._cg.close()


@pytest.fixture
def server_and_client():
    server_node = roswell.Node("fib_server", domain=0)
    client_node = roswell.Node("fib_client", domain=0)
    types = server_node.load_action(ACTION_TYPE)
    cancel_types = server_node.load_service("action_msgs/srv/CancelGoal")
    server = FibServer(server_node, ACTION, types, cancel_types)
    ac = client_node.action_client(ACTION, ACTION_TYPE)
    try:
        yield server, ac
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


def test_action_send_feedback_result(server_and_client):
    server, ac = server_and_client
    assert ac.wait_for_server(timeout=15), "action server never discovered"

    goal_id, accepted = _send_goal(ac, 5)
    assert accepted
    assert len(goal_id) == 16

    # Collect at least one feedback sample.
    feedbacks = []
    for fb in ac.feedback(timeout=4.0):
        feedbacks.append(fb)
        if len(feedbacks) >= 1:
            break
    assert feedbacks, "no feedback received"
    assert feedbacks[0].goal_id == goal_id
    assert len(list(feedbacks[0].message.partial_sequence)) >= 2

    status, result = None, None
    for _ in range(25):
        try:
            status, result = ac.get_result_sync(goal_id, timeout=2.0)
            break
        except roswell.RoswellTimeout:
            continue
    assert status == STATUS_SUCCEEDED
    assert list(result.sequence) == fibonacci(5)


def test_action_cancel(server_and_client):
    server, ac = server_and_client
    assert ac.wait_for_server(timeout=15)

    goal_id, accepted = _send_goal(ac, 20)  # long enough to cancel mid-flight
    assert accepted
    code = ac.cancel_sync(goal_id, timeout=3.0)
    assert code == 0  # ERROR_NONE


def test_action_client_repr(server_and_client):
    server, ac = server_and_client
    assert repr(ac) == "<roswell.ActionClient name='/fib' type='example_interfaces/action/Fibonacci'>"

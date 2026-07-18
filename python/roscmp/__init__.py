"""roscmp — pip-installable Python that speaks ROS2 with zero ROS installation.

A thin, asyncio-native client over the roscmp Rust runtime (RTPS via RustDDS).
Load a ``.msg``/``.srv`` at runtime, then publish/subscribe/serve/call — no ROS
install, no ``rosidl`` codegen, no executor/callback-group ceremony.

    import asyncio, roscmp

    async def main():
        node = roscmp.Node("listener", domain=0)
        Twist = node.load_type("geometry_msgs/msg/Twist.msg",
                               deps=["geometry_msgs/msg/Vector3.msg"])
        async with node.subscribe("/cmd_vel", Twist) as sub:
            async for msg in sub:
                print(msg.linear.x)

    asyncio.run(main())
"""

from __future__ import annotations

import asyncio
import ctypes
import json
import os
import threading
import time
import time as _time
import warnings
from ctypes import byref, c_char_p, c_int8, c_uint8, c_uint64
from dataclasses import dataclass
from typing import Awaitable, Callable, Iterator, Optional, Sequence, Tuple, Union

from ._ffi import (
    PARAM_BOOL,
    PARAM_DOUBLE,
    PARAM_INTEGER,
    PARAM_STRING,
    QosError,
    QosIncompatibleWarning,
    RcmParamValue,
    RcmQos,
    RoscmpError,
    RoscmpTimeout,
    StaleHandleError,
    TypeMismatchError,
    WrongKindError,
    check_ptr,
    check_rc,
    error_for,
    lib,
    take_json,
)
from ._ffi import last_error as _last_error
from ._types import Message, MessageType
from . import _interfaces

__all__ = [
    "Node",
    "Publisher",
    "Subscriber",
    "Client",
    "Service",
    "ActionClient",
    "ActionServer",
    "ActionTypes",
    "Feedback",
    "ServerGoalHandle",
    "TfBuffer",
    "Transform",
    "Timer",
    "Message",
    "MessageType",
    "QosProfile",
    "RoscmpError",
    "QosError",
    "RoscmpTimeout",
    "StaleHandleError",
    "TypeMismatchError",
    "WrongKindError",
    "QosIncompatibleWarning",
]


# ---- QoS --------------------------------------------------------------------


@dataclass
class QosProfile:
    """A QoS profile, mapping to the C ``RcmQos`` descriptor.

    ``reliability``: ``"reliable"`` | ``"best_effort"``. ``durability``:
    ``"volatile"`` | ``"transient_local"``. Negative ms values mean unset.
    """

    reliability: str = "reliable"
    durability: str = "volatile"
    depth: int = 10
    keep_all: bool = False
    deadline_ms: int = -1
    lifespan_ms: int = -1

    @classmethod
    def preset(cls, name: str) -> "QosProfile":
        """The ``"default"``, ``"sensor_data"``, or ``"latched"`` ROS preset."""
        q = lib.rcm_qos_preset(name.encode())
        return cls(
            reliability="reliable" if q.reliability else "best_effort",
            durability="transient_local" if q.durability else "volatile",
            depth=q.depth,
            keep_all=bool(q.keep_all),
            deadline_ms=q.deadline_ms,
            lifespan_ms=q.lifespan_ms,
        )

    def _to_rcm(self) -> RcmQos:
        return RcmQos(
            reliability=1 if self.reliability == "reliable" else 0,
            durability=1 if self.durability == "transient_local" else 0,
            keep_all=1 if self.keep_all else 0,
            depth=self.depth,
            deadline_ms=self.deadline_ms,
            lifespan_ms=self.lifespan_ms,
        )


def _qos_arg(qos: Optional[QosProfile]):
    """A ``POINTER(RcmQos)`` argument (or ``None`` for the default preset)."""
    if qos is None:
        return None
    rcm = qos._to_rcm()
    return byref(rcm), rcm  # keep rcm alive alongside the pointer


def _emit_qos_warnings(events_json: str) -> None:
    try:
        events = json.loads(events_json)
    except (ValueError, TypeError):
        return
    for e in events:
        if e.get("event") == "incompatible_qos":
            warnings.warn(
                f"incompatible QoS with a peer (policy={e.get('policy')}, "
                f"count={e.get('count')})",
                QosIncompatibleWarning,
                stacklevel=3,
            )


# ---- endpoints --------------------------------------------------------------


class Publisher:
    """Publishes messages of one type on one topic."""

    def __init__(self, node: "Node", handle: int, mtype: MessageType, topic: str = ""):
        self._node = node
        self._handle = handle
        self.type = mtype
        self.topic = topic

    def __repr__(self) -> str:
        return f"<roscmp.Publisher topic={self.topic!r} type={self.type.root!r}>"

    def publish(self, msg: Message) -> None:
        check_rc(lib.rcm_publish(self._handle, msg.handle), "publish")
        _emit_qos_warnings(take_json(lib.rcm_publisher_events(self._handle)))

    def new(self) -> Message:
        """Allocate a default-initialized message of this publisher's type."""
        return self.type.alloc()

    def close(self) -> None:
        if self._handle:
            lib.rcm_publisher_free(self._handle)
            self._handle = None

    def __del__(self):
        self.close()


class Subscriber:
    """Receives messages of one type from one topic.

    Use it synchronously (:meth:`take`, :meth:`messages`) or asynchronously
    (``async for msg in sub``). A given subscriber should be driven one way at a
    time. Supports ``with`` and ``async with`` for scoped cleanup.
    """

    def __init__(self, node: "Node", handle: int, mtype: MessageType, topic: str = ""):
        self._node = node
        self._handle = handle
        self.type = mtype
        self.topic = topic
        self._registered = False

    def __repr__(self) -> str:
        return f"<roscmp.Subscriber topic={self.topic!r} type={self.type.root!r}>"

    # -- sync -------------------------------------------------------------

    def take(self) -> Optional[Message]:
        """Take the next message (a fresh owned :class:`Message`), or ``None``."""
        msg = self.type.alloc()
        r = lib.rcm_take(self._handle, msg.handle)
        _emit_qos_warnings(take_json(lib.rcm_subscriber_events(self._handle)))
        if r == 1:
            return msg
        if r < 0:
            raise error_for(r, "take")
        return None

    def messages(self, timeout: float = 1.0) -> Iterator[Message]:
        """Yield messages as they arrive until ``timeout`` seconds pass idle."""
        handles = (c_uint64 * 1)(self._handle)
        while True:
            msg = self.take()
            if msg is not None:
                yield msg
                continue
            if lib.rcm_wait(handles, 1, int(timeout * 1000)) < 0:
                return

    # -- async ------------------------------------------------------------

    def __aiter__(self) -> "Subscriber":
        self._node._register_async(self)
        self._registered = True
        return self

    async def __anext__(self) -> Message:
        queue = self._node._async_queue(self)
        return await queue.get()

    # -- scoping ----------------------------------------------------------

    def __enter__(self) -> "Subscriber":
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    async def __aenter__(self) -> "Subscriber":
        return self

    async def __aexit__(self, *exc) -> None:
        self.close()

    def close(self) -> None:
        # Serialize unregister + free against the node's wait loop so it never
        # touches a freed handle (it re-checks membership under the same lock).
        with self._node._lock:
            if self._registered:
                self._node._async.pop(id(self), None)
                self._registered = False
            if self._handle:
                lib.rcm_subscriber_free(self._handle)
                self._handle = None

    def __del__(self):
        self.close()


class Client:
    """Calls a service, awaiting the correlated reply."""

    def __init__(
        self, node: "Node", handle: int, req_t: MessageType, resp_t: MessageType, name: str = ""
    ):
        self._node = node
        self._handle = handle
        self.request_type = req_t
        self.response_type = resp_t
        self.name = name

    def __repr__(self) -> str:
        return f"<roscmp.Client name={self.name!r}>"

    def new_request(self) -> Message:
        return self.request_type.alloc()

    def call_sync(self, req: Message, timeout: float = 5.0) -> Message:
        resp = self.response_type.alloc()
        r = lib.rcm_call(self._handle, req.handle, resp.handle, int(timeout * 1000))
        if r == 1:
            return resp
        if r == 0:
            raise RoscmpTimeout(f"service call timed out after {timeout}s")
        raise error_for(r, "call")

    async def call(self, req: Message, timeout: float = 5.0) -> Message:
        """Await the reply without blocking the event loop."""
        return await asyncio.to_thread(self.call_sync, req, timeout)

    def close(self) -> None:
        if self._handle:
            lib.rcm_client_free(self._handle)
            self._handle = None

    def __del__(self):
        self.close()


Handler = Callable[[Message], Union[Message, Awaitable[Message]]]


class Service:
    """Serves a service on a background thread, dispatching to ``handler``.

    ``handler(request) -> response`` may be sync or ``async``. It receives the
    decoded request :class:`Message` and returns a response :class:`Message`
    (allocate one with :meth:`new_response`).
    """

    def __init__(
        self,
        node: "Node",
        handle: int,
        req_t: MessageType,
        resp_t: MessageType,
        handler: Handler,
        name: str = "",
    ):
        self._node = node
        self._handle = handle
        self.request_type = req_t
        self.response_type = resp_t
        self.name = name
        self._handler = handler
        self._is_async = asyncio.iscoroutinefunction(handler)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._serve, daemon=True)
        self._thread.start()

    def __repr__(self) -> str:
        return f"<roscmp.Service name={self.name!r}>"

    def new_response(self) -> Message:
        return self.response_type.alloc()

    def _invoke(self, req: Message) -> Message:
        if self._is_async:
            return asyncio.run(self._handler(req))
        return self._handler(req)

    def _serve(self) -> None:
        req_buf = self.request_type.alloc()
        token = c_uint64()
        while not self._stop.is_set():
            r = lib.rcm_service_take_request(self._handle, req_buf.handle, byref(token))
            if r == 1:
                resp = self._invoke(req_buf)
                lib.rcm_service_send_reply(self._handle, token.value, resp.handle)
            elif r < 0:
                # transient error; keep serving
                self._stop.wait(0.02)
            else:
                self._stop.wait(0.005)

    def close(self) -> None:
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)
        if self._handle:
            lib.rcm_service_free(self._handle)
            self._handle = None

    def __del__(self):
        self.close()


# ---- actions ----------------------------------------------------------------

_GOAL_ID_LEN = 16

#: The eight component types of an action, in the order rcm_action_load returns.
_ACTION_TYPE_NAMES = (
    "goal",
    "result",
    "feedback",
    "send_goal_request",
    "send_goal_response",
    "get_result_request",
    "get_result_response",
    "feedback_message",
)


@dataclass
class Feedback:
    """One action feedback sample: the goal it belongs to and its payload."""

    goal_id: bytes
    message: Message


class ActionTypes:
    """The component message types of an action (resolved from ``pkg/action/Name``).

    Exposes ``goal``, ``result``, ``feedback`` payload types plus the
    ``send_goal_request``/``send_goal_response``/``get_result_request``/
    ``get_result_response``/``feedback_message`` wrapper types — everything needed
    to stand up an action server from :meth:`Node.serve` and :meth:`Node.publisher`.
    """

    goal: MessageType
    result: MessageType
    feedback: MessageType
    send_goal_request: MessageType
    send_goal_response: MessageType
    get_result_request: MessageType
    get_result_response: MessageType
    feedback_message: MessageType

    def __init__(self, node: "Node", action_type):
        self.action_type = str(action_type)
        arr, n = _paths(node._search_roots())
        out = (c_uint64 * len(_ACTION_TYPE_NAMES))()
        if lib.rcm_action_load(self.action_type.encode(), arr, n, out) != 0:
            raise RoscmpError(f"load_action({action_type}): {_last_error()}")
        for name, handle in zip(_ACTION_TYPE_NAMES, out):
            setattr(self, name, MessageType(handle))

    def __repr__(self) -> str:
        return f"<roscmp.ActionTypes {self.action_type!r}>"


def _goal_id_buf(goal_id: Optional[bytes]):
    """A ``c_uint8[16]`` buffer from ``goal_id`` (all-zero means 'all goals')."""
    raw = bytes(_GOAL_ID_LEN) if goal_id is None else bytes(goal_id)
    if len(raw) != _GOAL_ID_LEN:
        raise ValueError(f"goal_id must be {_GOAL_ID_LEN} bytes, got {len(raw)}")
    return (c_uint8 * _GOAL_ID_LEN).from_buffer_copy(raw)


class ActionClient:
    """A runtime-typed ROS2 action client.

    Owns the three action services (``send_goal``/``get_result``/``cancel_goal``)
    and the feedback subscription. Message payloads are ordinary :class:`Message`
    objects of :attr:`goal_type` / :attr:`result_type` / :attr:`feedback_type`;
    the goal-id wrapping, result/feedback unwrapping, and correlation all happen
    in Rust. Sync and ``async`` variants are provided for each blocking call,
    matching the :class:`Client`/:class:`Subscriber` patterns.
    """

    def __init__(self, node: "Node", handle: int, action_name: str, action_type: str):
        self._node = node
        self._handle = handle
        self.name = action_name
        self.action_type = action_type
        self.goal_type = MessageType(lib.rcm_action_goal_type(handle))
        self.result_type = MessageType(lib.rcm_action_result_type(handle))
        self.feedback_type = MessageType(lib.rcm_action_feedback_type(handle))

    def __repr__(self) -> str:
        return f"<roscmp.ActionClient name={self.name!r} type={self.action_type!r}>"

    def new_goal(self) -> Message:
        """Allocate a default-initialized goal message to fill and send."""
        return self.goal_type.alloc()

    # -- server discovery -------------------------------------------------

    def server_is_ready(self) -> bool:
        """True once all three action services have discovered the server."""
        r = lib.rcm_action_server_ready(self._handle)
        if r < 0:
            raise error_for(r, "action_server_ready")
        return r == 1

    def wait_for_server(self, timeout: float = 5.0) -> bool:
        """Block until the server is discovered or ``timeout`` seconds elapse."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.server_is_ready():
                return True
            time.sleep(0.05)
        return self.server_is_ready()

    # -- send goal --------------------------------------------------------

    def send_goal_sync(self, goal: Message, timeout: float = 5.0) -> Tuple[bytes, bool]:
        """Send ``goal``; return ``(goal_id, accepted)``. Raises on timeout."""
        goal_id = (c_uint8 * _GOAL_ID_LEN)()
        accepted = c_uint8()
        r = lib.rcm_action_send_goal(
            self._handle, goal.handle, int(timeout * 1000), goal_id, byref(accepted)
        )
        if r == 1:
            return bytes(goal_id), bool(accepted.value)
        if r == 0:
            raise RoscmpTimeout(f"send_goal timed out after {timeout}s")
        raise error_for(r, "send_goal")

    async def send_goal(self, goal: Message, timeout: float = 5.0) -> Tuple[bytes, bool]:
        """Await :meth:`send_goal_sync` without blocking the event loop."""
        return await asyncio.to_thread(self.send_goal_sync, goal, timeout)

    # -- get result -------------------------------------------------------

    def get_result_sync(self, goal_id: bytes, timeout: float = 5.0) -> Tuple[int, Message]:
        """Fetch a goal's ``(status, result_message)``. Raises on timeout."""
        result = self.result_type.alloc()
        status = c_int8()
        r = lib.rcm_action_get_result(
            self._handle, _goal_id_buf(goal_id), result.handle, int(timeout * 1000), byref(status)
        )
        if r == 1:
            return status.value, result
        result.close()
        if r == 0:
            raise RoscmpTimeout(f"get_result timed out after {timeout}s")
        raise error_for(r, "get_result")

    async def get_result(self, goal_id: bytes, timeout: float = 5.0) -> Tuple[int, Message]:
        """Await :meth:`get_result_sync` without blocking the event loop."""
        return await asyncio.to_thread(self.get_result_sync, goal_id, timeout)

    # -- feedback ---------------------------------------------------------

    def poll_feedback(self) -> Optional[Feedback]:
        """Take the next feedback sample if one is queued, else ``None``."""
        fb = self.feedback_type.alloc()
        goal_id = (c_uint8 * _GOAL_ID_LEN)()
        r = lib.rcm_action_poll_feedback(self._handle, fb.handle, goal_id)
        if r == 1:
            return Feedback(bytes(goal_id), fb)
        fb.close()
        if r < 0:
            raise error_for(r, "poll_feedback")
        return None

    def feedback(self, timeout: float = 5.0) -> Iterator[Feedback]:
        """Yield feedback samples as they arrive until ``timeout`` seconds idle."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            item = self.poll_feedback()
            if item is not None:
                yield item
                deadline = time.monotonic() + timeout
            else:
                time.sleep(0.01)

    async def feedback_async(self, timeout: float = 5.0):
        """Async generator over feedback samples (idle ``timeout`` seconds)."""
        loop = asyncio.get_running_loop()
        deadline = loop.time() + timeout
        while loop.time() < deadline:
            item = await asyncio.to_thread(self.poll_feedback)
            if item is not None:
                yield item
                deadline = loop.time() + timeout
            else:
                await asyncio.sleep(0.01)

    # -- cancel -----------------------------------------------------------

    def cancel_sync(self, goal_id: Optional[bytes] = None, timeout: float = 5.0) -> int:
        """Cancel ``goal_id`` (or all goals if ``None``); return the return code."""
        code = c_int8()
        r = lib.rcm_action_cancel_goal(
            self._handle, _goal_id_buf(goal_id), int(timeout * 1000), byref(code)
        )
        if r == 1:
            return code.value
        if r == 0:
            raise RoscmpTimeout(f"cancel_goal timed out after {timeout}s")
        raise error_for(r, "cancel_goal")

    async def cancel(self, goal_id: Optional[bytes] = None, timeout: float = 5.0) -> int:
        """Await :meth:`cancel_sync` without blocking the event loop."""
        return await asyncio.to_thread(self.cancel_sync, goal_id, timeout)

    # -- scoping / lifetime ----------------------------------------------

    def __enter__(self) -> "ActionClient":
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def close(self) -> None:
        if self._handle:
            lib.rcm_action_client_free(self._handle)
            self._handle = None

    def __del__(self):
        self.close()


# ---- tf ---------------------------------------------------------------------


@dataclass
class Transform:
    """A rigid transform: ``translation`` (x, y, z) and quaternion ``rotation``
    (x, y, z, w)."""

    translation: Tuple[float, float, float]
    rotation: Tuple[float, float, float, float]


def _tf_time_ns(time: Optional[float]) -> int:
    """Seconds (or ``None`` for latest-available) to the FFI nanosecond arg."""
    return -1 if time is None else int(time * 1e9)


class TfBuffer:
    """A tf2 transform buffer fed by ``/tf`` + ``/tf_static`` (in Rust).

    The buffer, listener subscriptions, graph walk, and interpolation all live
    in Rust; this object only forwards lookups and broadcasts.
    """

    def __init__(self, node: "Node", handle: int):
        self._node = node
        self._handle = handle

    def __repr__(self) -> str:
        return f"<roscmp.TfBuffer node={self._node.name!r}>"

    def lookup_transform(
        self, target: str, source: str, time: Optional[float] = None
    ) -> Transform:
        """The transform taking ``source``-frame points into ``target`` frame
        coordinates at ``time`` (seconds; ``None`` = latest available). Raises
        :class:`RoscmpError` when the transform is not (yet) available.
        """
        out = (ctypes.c_double * 7)()
        r = lib.rcm_tf_lookup(
            self._handle, target.encode(), source.encode(), _tf_time_ns(time), out
        )
        if r == 1:
            return Transform(
                translation=(out[0], out[1], out[2]),
                rotation=(out[3], out[4], out[5], out[6]),
            )
        if r < 0:
            raise error_for(r, "lookup_transform")
        raise RoscmpError(f"lookup_transform({target}, {source}): {_last_error()}")

    def can_transform(
        self, target: str, source: str, time: Optional[float] = None
    ) -> bool:
        """True if :meth:`lookup_transform` would currently succeed."""
        out = (ctypes.c_double * 7)()
        r = lib.rcm_tf_lookup(
            self._handle, target.encode(), source.encode(), _tf_time_ns(time), out
        )
        if r < 0:
            raise error_for(r, "can_transform")
        return r == 1

    def broadcast(
        self,
        parent: str,
        child: str,
        transform: Transform,
        time: Optional[float] = None,
        static: bool = False,
    ) -> None:
        """Publish ``transform`` on ``/tf`` (or latched ``/tf_static`` when
        ``static``), stamped ``time`` seconds (``None`` = now)."""
        stamp_ns = int((time if time is not None else _time.time()) * 1e9)
        vals = (ctypes.c_double * 7)(*transform.translation, *transform.rotation)
        check_rc(
            lib.rcm_tf_broadcast(
                self._handle,
                parent.encode(),
                child.encode(),
                stamp_ns,
                1 if static else 0,
                vals,
            ),
            "tf broadcast",
        )

    def close(self) -> None:
        if self._handle:
            lib.rcm_tf_free(self._handle)
            self._handle = None

    def __del__(self):
        self.close()


# ---- action server ----------------------------------------------------------

#: Terminal ``action_msgs/GoalStatus`` values reported via ``finish``.
GOAL_STATUS_SUCCEEDED = 4
GOAL_STATUS_CANCELED = 5
GOAL_STATUS_ABORTED = 6


class ServerGoalHandle:
    """Handed to an action server's execute callback alongside the goal.

    Exposes :meth:`publish_feedback` and :attr:`is_cancel_requested`; the
    protocol machinery (goal intake, correlation, status/latching) is in Rust.
    """

    def __init__(self, server: "ActionServer", goal_id: bytes):
        self._server = server
        self.goal_id = goal_id

    def __repr__(self) -> str:
        return f"<roscmp.ServerGoalHandle goal_id={self.goal_id.hex()}>"

    def new_feedback(self) -> Message:
        """Allocate a default-initialized feedback message."""
        return self._server.feedback_type.alloc()

    def new_result(self) -> Message:
        """Allocate a default-initialized result message."""
        return self._server.result_type.alloc()

    def publish_feedback(self, msg: Message) -> None:
        """Publish ``msg`` (of the action's feedback type) for this goal."""
        check_rc(
            lib.rcm_action_server_publish_feedback(
                self._server._handle, _goal_id_buf(self.goal_id), msg.handle
            ),
            "publish_feedback",
        )

    @property
    def is_cancel_requested(self) -> bool:
        """True once a client has requested cancellation of this goal."""
        r = lib.rcm_action_server_cancel_requested(
            self._server._handle, _goal_id_buf(self.goal_id)
        )
        if r < 0:
            raise error_for(r, "cancel_requested")
        return r == 1


ExecuteCallback = Callable[[Message, ServerGoalHandle], Union[Message, Awaitable[Message]]]


class ActionServer:
    """A runtime-typed ROS2 action server driven by ``execute_callback``.

    Goals are auto-accepted and executed sequentially on a background thread.
    ``execute_callback(goal, handle)`` (sync or ``async``) returns the result
    :class:`Message` (allocate with :meth:`new_result`); the goal finishes
    ``SUCCEEDED`` — or ``CANCELED`` if a cancel was requested — and ``ABORTED``
    if the callback raises. The wire protocol (services, feedback/status
    topics, goal bookkeeping, result parking) lives in Rust.
    """

    def __init__(
        self,
        node: "Node",
        handle: int,
        name: str,
        action_type: str,
        execute_callback: ExecuteCallback,
        cancel_callback: Optional[Callable[[ServerGoalHandle], None]] = None,
    ):
        self._node = node
        self._handle = handle
        self.name = name
        self.action_type = action_type
        self.goal_type = MessageType(lib.rcm_action_server_goal_type(handle))
        self.result_type = MessageType(lib.rcm_action_server_result_type(handle))
        self.feedback_type = MessageType(lib.rcm_action_server_feedback_type(handle))
        self._execute = execute_callback
        self._cancel_callback = cancel_callback
        self._is_async = asyncio.iscoroutinefunction(execute_callback)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._serve, daemon=True)
        self._thread.start()

    def __repr__(self) -> str:
        return f"<roscmp.ActionServer name={self.name!r} type={self.action_type!r}>"

    def new_result(self) -> Message:
        """Allocate a default-initialized result message."""
        return self.result_type.alloc()

    def _invoke(self, goal: Message, handle: ServerGoalHandle) -> Message:
        if self._is_async:
            return asyncio.run(self._execute(goal, handle))
        return self._execute(goal, handle)

    def _watch_cancel(self, handle: ServerGoalHandle, done: threading.Event) -> None:
        while not done.wait(0.02):
            if handle.is_cancel_requested:
                self._cancel_callback(handle)
                return

    def _serve(self) -> None:
        goal_id = (c_uint8 * _GOAL_ID_LEN)()
        while not self._stop.is_set():
            lib.rcm_action_server_spin(self._handle)
            goal = self.goal_type.alloc()
            r = lib.rcm_action_server_take_goal(self._handle, goal.handle, goal_id)
            if r != 1:
                goal.close()
                self._stop.wait(0.005)
                continue
            handle = ServerGoalHandle(self, bytes(goal_id))
            watcher = None
            done = threading.Event()
            if self._cancel_callback is not None:
                watcher = threading.Thread(
                    target=self._watch_cancel, args=(handle, done), daemon=True
                )
                watcher.start()
            try:
                result = self._invoke(goal, handle)
                status = (
                    GOAL_STATUS_CANCELED
                    if handle.is_cancel_requested
                    else GOAL_STATUS_SUCCEEDED
                )
            except Exception:
                result = self.new_result()
                status = GOAL_STATUS_ABORTED
            finally:
                done.set()
                if watcher is not None:
                    watcher.join(timeout=1.0)
            lib.rcm_action_server_finish(
                self._handle, _goal_id_buf(handle.goal_id), status, result.handle
            )
            goal.close()

    def close(self) -> None:
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)
        if self._handle:
            lib.rcm_action_server_free(self._handle)
            self._handle = None

    def __enter__(self) -> "ActionServer":
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def __del__(self):
        self.close()


ParamValue = Union[bool, int, float, str, bytes, bytearray, list, tuple]


def _set_param_array(server: int, name: str, value) -> None:
    """Set an array parameter, inferring the ROS array kind from the elements."""
    from ctypes import c_char_p, c_double, c_int64

    from ._ffi import (
        PARAM_BOOL_ARRAY,
        PARAM_BYTE_ARRAY,
        PARAM_DOUBLE_ARRAY,
        PARAM_INTEGER_ARRAY,
    )

    if isinstance(value, (bytes, bytearray)):
        raw = bytes(value)
        buf = (c_uint8 * len(raw)).from_buffer_copy(raw) if raw else None
        rc = lib.rcm_param_set_array(server, name.encode(), PARAM_BYTE_ARRAY, buf, len(raw))
    else:
        items = list(value)
        if not items:
            raise TypeError(
                f"cannot infer the array type of empty parameter {name!r}; "
                "use b'' for a byte array or a scalar instead"
            )
        if all(isinstance(v, bool) for v in items):
            buf = (c_uint8 * len(items))(*[1 if v else 0 for v in items])
            rc = lib.rcm_param_set_array(
                server, name.encode(), PARAM_BOOL_ARRAY, buf, len(items)
            )
        elif all(isinstance(v, str) for v in items):
            arr = (c_char_p * len(items))(*[v.encode() for v in items])
            rc = lib.rcm_param_set_string_array(server, name.encode(), arr, len(items))
        elif all(isinstance(v, int) and not isinstance(v, bool) for v in items):
            buf = (c_int64 * len(items))(*items)
            rc = lib.rcm_param_set_array(
                server, name.encode(), PARAM_INTEGER_ARRAY, buf, len(items)
            )
        elif all(isinstance(v, (int, float)) and not isinstance(v, bool) for v in items):
            buf = (c_double * len(items))(*[float(v) for v in items])
            rc = lib.rcm_param_set_array(
                server, name.encode(), PARAM_DOUBLE_ARRAY, buf, len(items)
            )
        else:
            raise TypeError(
                f"parameter {name!r}: array elements must be uniformly "
                "bool, int, float, or str"
            )
    check_rc(rc, f"set_parameter({name})")


def _param_to_rcm(value: ParamValue) -> RcmParamValue:
    """Encode a Python scalar into an ``RcmParamValue`` (bool checked before int)."""
    if isinstance(value, bool):
        return RcmParamValue(kind=PARAM_BOOL, boolean=1 if value else 0)
    if isinstance(value, int):
        return RcmParamValue(kind=PARAM_INTEGER, integer=value)
    if isinstance(value, float):
        return RcmParamValue(kind=PARAM_DOUBLE, number=value)
    if isinstance(value, str):
        return RcmParamValue(kind=PARAM_STRING, text=value.encode())
    raise TypeError(
        f"parameter value must be bool, int, float, or str, not {type(value).__name__}"
    )


# ---- timers -----------------------------------------------------------------


class Timer:
    """A periodic callback driven by the asyncio event loop.

    Created by :meth:`Node.create_timer` from within a running loop; fires
    ``callback`` (sync or ``async``) every ``period`` seconds until cancelled.
    """

    def __init__(self, period: float, callback: Callable[[], Union[None, Awaitable[None]]]):
        self.period = period
        self._callback = callback
        self._task = asyncio.get_running_loop().create_task(self._run())

    async def _run(self) -> None:
        while True:
            await asyncio.sleep(self.period)
            result = self._callback()
            if asyncio.iscoroutine(result):
                await result

    def cancel(self) -> None:
        """Stop the timer. Idempotent."""
        if not self._task.done():
            self._task.cancel()

    def __repr__(self) -> str:
        state = "cancelled" if self._task.cancelled() else "active"
        return f"<roscmp.Timer period={self.period} {state}>"


# ---- node -------------------------------------------------------------------


class Node:
    """A ROS2 node: a DDS participant plus a factory for endpoints.

    Advertises itself on the ROS graph (visible to ``ros2 node list``). Async
    subscribers share one background thread that multiplexes all live readers
    with :func:`rcm_wait` and dispatches into their asyncio loops — never one
    thread per subscription, never a busy-poll.
    """

    def __init__(
        self,
        name: str,
        domain: int = 0,
        namespace: str = "/",
        type_paths: Sequence = (),
    ):
        self.name = name
        self.namespace = namespace
        self._type_paths = [str(p) for p in type_paths]
        self._ctx = check_ptr(lib.rcm_init(domain), "rcm_init")
        lib.rcm_node(self._ctx, name.encode(), namespace.encode())
        self._async: dict[int, tuple[Subscriber, asyncio.AbstractEventLoop, asyncio.Queue]] = {}
        self._lock = threading.Lock()
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._param_server: Optional[int] = None
        self._timers: list["Timer"] = []

    def __repr__(self) -> str:
        state = "closed" if self._ctx is None else f"namespace={self.namespace!r}"
        return f"<roscmp.Node name={self.name!r} {state}>"

    # -- types ------------------------------------------------------------

    def load_type(self, path, deps: Sequence = ()) -> MessageType:
        """Load a message type from a ``.msg`` file or a type reference.

        Pass an explicit ``.msg`` path (with ``deps`` for its dependency files),
        or a path-free reference like ``"robot_msgs/msg/Detection"`` that resolves
        against the node's search roots (``type_paths``, ``ROSCMP_TYPE_PATH``,
        ``AMENT_PREFIX_PATH``, the bundled interface tree). Nested cross-package
        dependencies are discovered in Rust.
        """
        if deps or os.path.exists(str(path)):
            arr, n = _paths(list(deps))
            handle = lib.rcm_type_load(str(path).encode(), arr, n)
        else:
            arr, n = _paths(self._search_roots())
            handle = lib.rcm_type_resolve(str(path).encode(), arr, n)
        return MessageType(check_ptr(handle, f"load_type({path})"))

    def load_service(self, path, deps: Sequence = ()) -> tuple[MessageType, MessageType]:
        """Load a ``.srv`` (path or ``pkg/srv/Name`` ref) into ``(request, response)``."""
        req, resp = c_uint64(), c_uint64()
        if deps or os.path.exists(str(path)):
            arr, n = _paths(list(deps))
            rc = lib.rcm_type_load_srv(str(path).encode(), arr, n, byref(req), byref(resp))
        else:
            arr, n = _paths(self._search_roots())
            rc = lib.rcm_type_resolve_srv(str(path).encode(), arr, n, byref(req), byref(resp))
        if rc != 0:
            raise RoscmpError(f"load_service({path}): {_last_error()}")
        return MessageType(req.value), MessageType(resp.value)

    def _search_roots(self) -> list:
        """Interface search-root directories for reference resolution."""
        return _interfaces.search_roots(self._type_paths)

    def load_action(self, action_type) -> "ActionTypes":
        """Resolve an action reference (``pkg/action/Name``) into its payload types.

        Returns an :class:`ActionTypes` bundle exposing the ``goal``, ``result``,
        and ``feedback`` message types plus the underlying ``send_goal`` /
        ``get_result`` request/response types — handy for standing up an action
        server from the plain service/topic primitives. For a client, prefer
        :meth:`action_client`, which owns the wire machinery.
        """
        return ActionTypes(self, action_type)

    def action_client(self, action_name: str, action_type) -> "ActionClient":
        """A runtime-typed action client for ``action_name``.

        ``action_type`` is a ``pkg/action/Name`` reference resolved against the
        node's search roots (in Rust). The client binds the three action services
        and the feedback subscription; send goals, iterate feedback, fetch the
        result, or cancel — see :class:`ActionClient`.
        """
        arr, n = _paths(self._search_roots())
        handle = lib.rcm_action_client(
            self._ctx, action_name.encode(), str(action_type).encode(), arr, n
        )
        return ActionClient(
            self, check_ptr(handle, f"action_client({action_name})"), action_name, str(action_type)
        )

    def action_server(
        self,
        action_name: str,
        action_type,
        execute_callback: "ExecuteCallback",
        cancel_callback=None,
    ) -> "ActionServer":
        """Serve action ``action_name`` with ``execute_callback`` on a
        background thread.

        ``action_type`` is a ``pkg/action/Name`` reference resolved against the
        node's search roots (in Rust). ``execute_callback(goal, handle)`` (sync
        or ``async``) receives the decoded goal :class:`Message` and a
        :class:`ServerGoalHandle` (``publish_feedback``, ``is_cancel_requested``)
        and returns the result :class:`Message`. ``cancel_callback(handle)``,
        if given, fires once when a cancel request arrives for the running goal.
        """
        arr, n = _paths(self._search_roots())
        handle = lib.rcm_action_server(
            self._ctx, action_name.encode(), str(action_type).encode(), arr, n
        )
        return ActionServer(
            self,
            check_ptr(handle, f"action_server({action_name})"),
            action_name,
            str(action_type),
            execute_callback,
            cancel_callback,
        )

    # -- tf ---------------------------------------------------------------

    def tf_buffer(self) -> "TfBuffer":
        """A :class:`TfBuffer` listening on ``/tf`` + ``/tf_static`` (and able
        to broadcast on them); buffer and graph logic live in Rust."""
        return TfBuffer(self, check_ptr(lib.rcm_tf_buffer(self._ctx), "tf_buffer"))

    # -- endpoints --------------------------------------------------------

    def publisher(
        self, topic: str, mtype: MessageType, qos: Optional[QosProfile] = None
    ) -> Publisher:
        arg = _qos_arg(qos)
        ptr = arg[0] if arg else None
        handle = lib.rcm_publisher(self._ctx, topic.encode(), mtype._handle, ptr)
        return Publisher(self, check_ptr(handle, f"publisher({topic})"), mtype, topic)

    def subscribe(
        self, topic: str, mtype: MessageType, qos: Optional[QosProfile] = None
    ) -> Subscriber:
        arg = _qos_arg(qos)
        ptr = arg[0] if arg else None
        handle = lib.rcm_subscriber(self._ctx, topic.encode(), mtype._handle, ptr)
        return Subscriber(self, check_ptr(handle, f"subscribe({topic})"), mtype, topic)

    def client(
        self, name: str, req_t: MessageType, resp_t: MessageType
    ) -> Client:
        handle = lib.rcm_client(self._ctx, name.encode(), req_t._handle, resp_t._handle)
        return Client(self, check_ptr(handle, f"client({name})"), req_t, resp_t, name)

    def serve(
        self,
        name: str,
        srv_types: tuple[MessageType, MessageType],
        handler: Handler,
    ) -> Service:
        req_t, resp_t = srv_types
        handle = lib.rcm_service(self._ctx, name.encode(), req_t._handle, resp_t._handle)
        return Service(self, check_ptr(handle, f"serve({name})"), req_t, resp_t, handler, name)

    # -- graph ------------------------------------------------------------

    def graph(self, listen_ms: int = 500) -> dict:
        """Discover the ROS graph as a dict (topics/services/actions/nodes)."""
        return json.loads(take_json(lib.rcm_graph_json(self._ctx, listen_ms)))

    # -- parameters -------------------------------------------------------

    def _params(self) -> int:
        """The node's parameter-server handle, created (and spun) on first use."""
        if self._param_server is None:
            handle = lib.rcm_param_server(self._ctx, self.name.encode())
            self._param_server = check_ptr(handle, "parameter server")
        return self._param_server

    def declare_parameter(self, name: str, value: ParamValue) -> ParamValue:
        """Declare (or overwrite) a parameter, returning its value.

        ``value`` is a ``bool``, ``int``, ``float``, or ``str`` — the scalar
        parameter types. The node stands up a parameter server on first call, so
        ``ros2 param get/set/list`` and ``/parameter_events`` see the parameter.
        """
        self.set_parameter(name, value)
        return value

    def set_parameter(self, name: str, value: ParamValue) -> None:
        """Set an existing (or new) parameter, emitting a ``/parameter_events`` update.

        Scalars (``bool``/``int``/``float``/``str``) and arrays (``bytes`` or a
        uniform ``list``/``tuple`` of bool/int/float/str) are supported; array
        kinds are inferred from the elements.
        """
        if isinstance(value, (bytes, bytearray, list, tuple)):
            _set_param_array(self._params(), name, value)
            return
        rcm = _param_to_rcm(value)
        check_rc(lib.rcm_param_set(self._params(), name.encode(), byref(rcm)), f"set_parameter({name})")

    def get_parameter(self, name: str) -> ParamValue:
        """The current value of a declared parameter (raises if undeclared)."""
        js = take_json(lib.rcm_param_get_json(self._params(), name.encode()))
        return json.loads(js)["value"]

    def list_parameters(self) -> list:
        """The names of every parameter this node has declared."""
        return json.loads(take_json(lib.rcm_param_list_json(self._params())))

    # -- timers -----------------------------------------------------------

    def create_timer(
        self, period: float, callback: Callable[[], Union[None, Awaitable[None]]]
    ) -> "Timer":
        """Call ``callback`` every ``period`` seconds on the running event loop.

        ``callback`` may be sync or ``async``. Returns a :class:`Timer`; call
        :meth:`Timer.cancel` (or close the node) to stop it. Must be called from
        within a running asyncio loop — timing is asyncio plumbing, not logic.
        """
        timer = Timer(period, callback)
        self._timers.append(timer)
        return timer

    # -- async wait loop --------------------------------------------------

    def _register_async(self, sub: Subscriber) -> None:
        loop = asyncio.get_running_loop()
        with self._lock:
            if id(sub) not in self._async:
                self._async[id(sub)] = (sub, loop, asyncio.Queue())
            self._ensure_thread()

    def _async_queue(self, sub: Subscriber) -> asyncio.Queue:
        with self._lock:
            return self._async[id(sub)][2]

    def _ensure_thread(self) -> None:
        if self._thread is None or not self._thread.is_alive():
            self._stop.clear()
            self._thread = threading.Thread(target=self._wait_loop, daemon=True)
            self._thread.start()

    def _wait_loop(self) -> None:
        while not self._stop.is_set():
            with self._lock:
                entries = list(self._async.values())
            if not entries:
                self._stop.wait(0.02)
                continue
            handles = (c_uint64 * len(entries))(*[e[0]._handle for e in entries])
            idx = lib.rcm_wait(handles, len(entries), 100)
            if idx >= 0:
                sub, loop, queue = entries[idx]
                # Re-check liveness under the lock: close() frees handles under
                # the same lock, so a closed subscriber is skipped, never used.
                with self._lock:
                    alive = id(sub) in self._async and sub._handle
                    msg = sub.take() if alive else None
                if msg is not None:
                    loop.call_soon_threadsafe(queue.put_nowait, msg)
            # Surface QoS warnings from every still-live async subscriber.
            for sub, _loop, _q in entries:
                with self._lock:
                    if id(sub) in self._async and sub._handle:
                        _emit_qos_warnings(
                            take_json(lib.rcm_subscriber_events(sub._handle))
                        )

    # -- lifecycle --------------------------------------------------------

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        for timer in self._timers:
            timer.cancel()
        self._timers.clear()
        with self._lock:
            self._async.clear()
        if self._param_server is not None:
            lib.rcm_param_server_free(self._param_server)
            self._param_server = None
        if self._ctx:
            lib.rcm_shutdown(self._ctx)
            self._ctx = None

    def __enter__(self) -> "Node":
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def __del__(self):
        self.close()


def _paths(deps: Sequence):
    """Build a ``(POINTER(c_char_p), n)`` argument pair from dependency paths."""
    items = [str(d).encode() for d in deps]
    if not items:
        return None, 0
    arr = (c_char_p * len(items))(*items)
    return arr, len(items)

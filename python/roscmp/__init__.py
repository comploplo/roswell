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
import warnings
from ctypes import byref, c_char_p, c_uint64
from dataclasses import dataclass
from typing import Awaitable, Callable, Iterator, Optional, Sequence, Union

from ._ffi import (
    QosError,
    QosIncompatibleWarning,
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

    def __init__(self, node: "Node", handle: int, mtype: MessageType):
        self._node = node
        self._handle = handle
        self.type = mtype

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

    def __init__(self, node: "Node", handle: int, mtype: MessageType):
        self._node = node
        self._handle = handle
        self.type = mtype
        self._registered = False

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

    def __init__(self, node: "Node", handle: int, req_t: MessageType, resp_t: MessageType):
        self._node = node
        self._handle = handle
        self.request_type = req_t
        self.response_type = resp_t

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
    ):
        self._node = node
        self._handle = handle
        self.request_type = req_t
        self.response_type = resp_t
        self._handler = handler
        self._is_async = asyncio.iscoroutinefunction(handler)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._serve, daemon=True)
        self._thread.start()

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


# ---- node -------------------------------------------------------------------


class Node:
    """A ROS2 node: a DDS participant plus a factory for endpoints.

    Advertises itself on the ROS graph (visible to ``ros2 node list``). Async
    subscribers share one background thread that multiplexes all live readers
    with :func:`rcm_wait` and dispatches into their asyncio loops — never one
    thread per subscription, never a busy-poll.
    """

    def __init__(self, name: str, domain: int = 0, namespace: str = "/"):
        self.name = name
        self.namespace = namespace
        self._ctx = check_ptr(lib.rcm_init(domain), "rcm_init")
        lib.rcm_node(self._ctx, name.encode(), namespace.encode())
        self._async: dict[int, tuple[Subscriber, asyncio.AbstractEventLoop, asyncio.Queue]] = {}
        self._lock = threading.Lock()
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()

    # -- types ------------------------------------------------------------

    def load_type(self, path, deps: Sequence = ()) -> MessageType:
        """Load a message type from a ``.msg`` file or a bundled reference.

        Pass an explicit ``.msg`` path (with ``deps`` for its dependency files),
        or a path-free reference like ``"geometry_msgs/msg/Twist"`` that resolves
        against the interfaces bundled in the wheel.
        """
        root, dep_paths = _resolve_inputs(path, deps, "msg")
        arr, n = _paths(dep_paths)
        handle = lib.rcm_type_load(str(root).encode(), arr, n)
        return MessageType(check_ptr(handle, f"load_type({path})"))

    def load_service(self, path, deps: Sequence = ()) -> tuple[MessageType, MessageType]:
        """Load a ``.srv`` (path or bundled ref) into ``(request, response)``."""
        root, dep_paths = _resolve_inputs(path, deps, "srv")
        arr, n = _paths(dep_paths)
        req, resp = c_uint64(), c_uint64()
        if lib.rcm_type_load_srv(str(root).encode(), arr, n, byref(req), byref(resp)) != 0:
            raise RoscmpError(f"load_service({path}): {_last_error()}")
        return MessageType(req.value), MessageType(resp.value)

    # -- endpoints --------------------------------------------------------

    def publisher(
        self, topic: str, mtype: MessageType, qos: Optional[QosProfile] = None
    ) -> Publisher:
        arg = _qos_arg(qos)
        ptr = arg[0] if arg else None
        handle = lib.rcm_publisher(self._ctx, topic.encode(), mtype._handle, ptr)
        return Publisher(self, check_ptr(handle, f"publisher({topic})"), mtype)

    def subscribe(
        self, topic: str, mtype: MessageType, qos: Optional[QosProfile] = None
    ) -> Subscriber:
        arg = _qos_arg(qos)
        ptr = arg[0] if arg else None
        handle = lib.rcm_subscriber(self._ctx, topic.encode(), mtype._handle, ptr)
        return Subscriber(self, check_ptr(handle, f"subscribe({topic})"), mtype)

    def client(
        self, name: str, req_t: MessageType, resp_t: MessageType
    ) -> Client:
        handle = lib.rcm_client(self._ctx, name.encode(), req_t._handle, resp_t._handle)
        return Client(self, check_ptr(handle, f"client({name})"), req_t, resp_t)

    def serve(
        self,
        name: str,
        srv_types: tuple[MessageType, MessageType],
        handler: Handler,
    ) -> Service:
        req_t, resp_t = srv_types
        handle = lib.rcm_service(self._ctx, name.encode(), req_t._handle, resp_t._handle)
        return Service(self, check_ptr(handle, f"serve({name})"), req_t, resp_t, handler)

    # -- graph ------------------------------------------------------------

    def graph(self, listen_ms: int = 500) -> dict:
        """Discover the ROS graph as a dict (topics/services/actions/nodes)."""
        return json.loads(take_json(lib.rcm_graph_json(self._ctx, listen_ms)))

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
        with self._lock:
            self._async.clear()
        if self._ctx:
            lib.rcm_shutdown(self._ctx)
            self._ctx = None

    def __enter__(self) -> "Node":
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def __del__(self):
        self.close()


def _resolve_inputs(path, deps: Sequence, ext: str):
    """Resolve ``(root_file, dep_files)`` for a load call.

    An explicit path (or any call that supplies ``deps``) is used as given; a
    path-free reference with no ``deps`` is looked up in the bundled interface
    tree. Unresolvable references fall through so the Rust loader reports the
    error against the original path.
    """
    if deps or os.path.exists(str(path)):
        return path, list(deps)
    hit = _interfaces.resolve(path, ext)
    if hit is not None:
        return hit
    return path, []


def _paths(deps: Sequence):
    """Build a ``(POINTER(c_char_p), n)`` argument pair from dependency paths."""
    items = [str(d).encode() for d in deps]
    if not items:
        return None, 0
    arr = (c_char_p * len(items))(*items)
    return arr, len(items)

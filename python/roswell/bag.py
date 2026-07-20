"""MCAP bag reading and writing.

Thin ctypes shim over the Rust MCAP machinery (``roswell-ros2-compat``'s reader/writer,
exposed through ``ros_bag_*`` handles): chunking, lz4 (de)compression, record
parsing, and message encode/decode all run in Rust. Python only walks the
cursor and materializes results.

    import roswell.bag

    with roswell.bag.open_write("out.mcap") as bag:
        bag.write("/chatter", msg, timestamp_ns)

    for sample in roswell.bag.read("out.mcap"):
        print(sample.topic, sample.message)
"""

from __future__ import annotations

import ctypes
import json
from dataclasses import dataclass
from typing import Dict, Iterator, Optional, Sequence, Union

from . import _interfaces
from ._ffi import check_handle, check_rc, error_for, lib, take_json
from ._types import Message, MessageType


@dataclass
class BagMessage:
    """One sample read from a bag.

    ``message`` is a decoded :class:`Message` when the sample's type could be
    resolved against the interface search roots, else the raw CDR bytes
    (encapsulation header + body) with ``schema`` carrying any recorded schema
    text (e.g. from a rosbag2 recording).
    """

    topic: str
    type: str
    log_time: int
    publish_time: int
    message: Union[Message, bytes]
    schema: Optional[str] = None

    @property
    def raw(self) -> bool:
        """True when ``message`` is undecoded CDR bytes."""
        return isinstance(self.message, bytes)


def read(
    path,
    type_paths: Sequence = (),
    decode: bool = True,
) -> Iterator[BagMessage]:
    """Yield every sample in the MCAP bag at ``path`` in file order.

    Types are resolved (and cached) per schema name against ``type_paths`` plus
    the standard search roots; unresolvable (or ``decode=False``) samples come
    back as raw bytes.
    """
    handle = check_handle(lib.ros_bag_open_read(str(path).encode()), f"read({path})")
    types: Dict[str, Optional[MessageType]] = {}
    roots = _interfaces.search_roots([str(p) for p in type_paths])
    try:
        while True:
            r = lib.ros_bag_next(handle)
            if r == 0:
                return
            if r < 0:
                raise error_for(r, "bag read")
            info = json.loads(take_json(lib.ros_bag_info_json(handle)))
            ros_type = info["type"]
            mtype = types.setdefault(
                ros_type, _resolve(ros_type, roots) if decode else None
            )
            message: Union[Message, bytes, None] = None
            if mtype is not None:
                msg = mtype.alloc()
                if lib.ros_bag_decode(handle, msg.handle) == 1:
                    message = msg
                else:
                    msg.close()
            schema = None
            if message is None:
                message = _raw_data(handle, info["size"])
                schema = _schema(handle, ros_type)
            yield BagMessage(
                topic=info["topic"],
                type=ros_type,
                log_time=info["log_time"],
                publish_time=info["publish_time"],
                message=message,
                schema=schema,
            )
    finally:
        lib.ros_bag_reader_free(handle)


def _resolve(ros_type: str, roots: Sequence[str]) -> Optional[MessageType]:
    from ctypes import c_char_p

    items = [str(r).encode() for r in roots]
    arr = (c_char_p * len(items))(*items) if items else None
    handle = lib.ros_type_resolve(ros_type.encode(), arr, len(items))
    return MessageType(handle) if handle else None


def _raw_data(handle: int, size: int) -> bytes:
    buf = ctypes.create_string_buffer(size)
    n = lib.ros_bag_data(handle, buf, size)
    if n < 0:
        raise error_for(n, "bag data")
    return buf.raw[:n]


def _schema(handle: int, ros_type: str) -> Optional[str]:
    ptr = lib.ros_bag_schema(handle, ros_type.encode())
    if not ptr:
        return None
    text = take_json(ptr)
    return text or None


class BagWriter:
    """Writes messages into an MCAP bag (lz4-chunked by default, in Rust)."""

    def __init__(self, handle: int, path: str):
        self._handle = handle
        self.path = path

    def __repr__(self) -> str:
        state = "closed" if not self._handle else "open"
        return f"<roswell.bag.BagWriter path={self.path!r} {state}>"

    def write(self, topic: str, msg: Message, timestamp_ns: int) -> None:
        """Append ``msg`` on ``topic`` at ``timestamp_ns`` (nanoseconds)."""
        check_rc(
            lib.ros_bag_write(self._handle, topic.encode(), timestamp_ns, msg.handle),
            f"bag write({topic})",
        )

    def close(self) -> None:
        """Flush chunks and write the MCAP footer. Idempotent."""
        if self._handle:
            check_rc(lib.ros_bag_writer_close(self._handle), "bag close")
            self._handle = None

    def __enter__(self) -> "BagWriter":
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass  # interpreter teardown: Rust's Drop still finishes the file


def open_write(path, compression: str = "lz4") -> BagWriter:
    """Open ``path`` for writing. ``compression`` is ``"lz4"`` or ``"none"``."""
    handle = lib.ros_bag_open_write(str(path).encode(), compression.encode())
    return BagWriter(check_handle(handle, f"open_write({path})"), str(path))

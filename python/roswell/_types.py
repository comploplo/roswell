"""Dynamic message objects over Rust-allocated C-ABI memory.

The Rust core is the single source of truth for layout: ``ros_type_layout_json``
describes every field's byte offset and element kind for the whole dependency
closure, and this module reads/writes fields at those offsets with ``ctypes``.
Owning buffers (strings, sequences) are always (re)allocated in Rust via
``ros_seq_assign`` / ``ros_str_assign`` (addressed by the message handle plus a
byte offset) so the codec's finalizer can free them.

Message memory is owned by a generation-counted handle. Each :class:`Message`
holds its handle (or, for a nested field, a reference to the root message that
does) and pins that root while any zero-copy view into it is alive, so a numpy
view can never outlive the buffer it aliases. :meth:`Message.close` frees the
buffer explicitly; a :mod:`weakref` finalizer frees it on GC otherwise.

Primitive arrays and sequences are exposed as zero-copy :mod:`numpy` views when
numpy is installed (valid while the owning message is live), and as plain lists
otherwise.
"""

from __future__ import annotations

import ctypes
import json
import weakref
from typing import Any

from ._ffi import RoswellError, check_handle, lib, take_json

try:
    import numpy as _np

    _HAS_NUMPY = True
except ImportError:  # pragma: no cover - exercised only without numpy
    _np = None
    _HAS_NUMPY = False

_CTYPE = {
    "bool": ctypes.c_bool,
    "u8": ctypes.c_uint8,
    "i8": ctypes.c_int8,
    "u16": ctypes.c_uint16,
    "i16": ctypes.c_int16,
    "u32": ctypes.c_uint32,
    "i32": ctypes.c_int32,
    "u64": ctypes.c_uint64,
    "i64": ctypes.c_int64,
    "f32": ctypes.c_float,
    "f64": ctypes.c_double,
}

_NPDTYPE = {
    "bool": "?",
    "u8": "u1",
    "i8": "i1",
    "u16": "u2",
    "i16": "i2",
    "u32": "u4",
    "i32": "i4",
    "u64": "u8",
    "i64": "i8",
    "f32": "f4",
    "f64": "f8",
}

_PTR = ctypes.sizeof(ctypes.c_void_p)


def _read_triple(addr: int) -> tuple[int, int]:
    """Return ``(data_ptr, size)`` of a ``{data,size,capacity}`` triple at ``addr``."""
    data = ctypes.c_void_p.from_address(addr).value or 0
    size = ctypes.c_size_t.from_address(addr + _PTR).value
    return data, size


def _free_handle(handle: int) -> None:
    """Finalize + free a message handle (idempotent on the Rust side)."""
    lib.ros_msg_free(handle)


class MessageType:
    """A runtime-loaded ROS message type and its closure layout.

    Wraps an owning type handle; freed when this object is collected.
    """

    def __init__(self, handle: int):
        self._handle = check_handle(handle, "type load")
        layout = json.loads(take_json(lib.ros_type_layout_json(handle)))
        self.root: str = layout["root"]
        self.dds_type: str = layout["dds_type"]
        self.size: int = layout["size"]
        self.messages: dict[str, Any] = layout["messages"]
        self._fieldmaps: dict[str, dict[str, Any]] = {
            mid: {f["name"]: f for f in m["fields"]}
            for mid, m in self.messages.items()
        }

    def field_map(self, mid: str) -> dict[str, Any]:
        return self._fieldmaps[mid]

    def alloc(self) -> "Message":
        """Allocate a zeroed, default-initialized message owned by the caller."""
        handle = lib.ros_msg_alloc(self._handle)
        if not handle:
            raise RoswellError("ros_msg_alloc failed")
        ptr = lib.ros_msg_data(handle)
        addr = ctypes.cast(ptr, ctypes.c_void_p).value
        return Message(self, addr, self.root, handle=handle)

    def __del__(self):
        handle = getattr(self, "_handle", None)
        if handle:
            lib.ros_type_free(handle)
            self._handle = None


class Message:
    """A view over one message's C-ABI memory with attribute field access.

    A root message returned by :meth:`MessageType.alloc` owns its buffer (via a
    message handle) and frees it on :meth:`close` or collection. Nested-message
    fields return non-owning views that reference the same root and keep it
    alive.
    """

    __slots__ = (
        "_t",
        "_addr",
        "_mid",
        "_handle",
        "_root",
        "_closed",
        "_fields",
        "_finalizer",
        "__weakref__",
    )

    def __init__(self, mtype: MessageType, addr: int, mid: str, handle=None, root=None):
        object.__setattr__(self, "_t", mtype)
        object.__setattr__(self, "_addr", addr)
        object.__setattr__(self, "_mid", mid)
        object.__setattr__(self, "_handle", handle)
        object.__setattr__(self, "_root", root if root is not None else self)
        object.__setattr__(self, "_closed", False)
        object.__setattr__(self, "_fields", mtype.field_map(mid))
        if handle is not None:
            object.__setattr__(
                self, "_finalizer", weakref.finalize(self, _free_handle, handle)
            )
        else:
            object.__setattr__(self, "_finalizer", None)

    # -- lifetime ---------------------------------------------------------

    @property
    def handle(self) -> int:
        """The root message's handle (for endpoint ops)."""
        return self._root._handle

    def close(self) -> None:
        """Finalize + free the message's buffer now. Idempotent.

        Only meaningful on an owning root message (the object returned by
        :meth:`MessageType.alloc`); a call on a non-owning nested view is a no-op.
        After close, field access on the message (or any view into it) raises.
        Any outstanding numpy view that pinned this message keeps it out of GC but
        does NOT keep the buffer valid — do not close while views are still in
        use. GC of an un-closed root frees it via a :mod:`weakref` finalizer.
        """
        if self._handle is None:
            return  # non-owning nested view: owns nothing to free
        if self._finalizer is not None and self._finalizer.alive:
            self._finalizer()  # runs _free_handle exactly once
        object.__setattr__(self, "_closed", True)

    # -- attribute access -------------------------------------------------

    def __getattr__(self, name: str):
        # __slots__ names never reach here; only unknown attrs / field names.
        if object.__getattribute__(self, "_root")._closed:
            raise RoswellError("message is closed")
        fields = object.__getattribute__(self, "_fields")
        f = fields.get(name)
        if f is None:
            raise AttributeError(name)
        return self._get(f)

    def __setattr__(self, name: str, value) -> None:
        if self._root._closed:
            raise RoswellError("message is closed")
        f = self._fields.get(name)
        if f is None:
            raise AttributeError(f"{self._mid} has no field {name!r}")
        self._set(f, value)

    def fields(self) -> list[str]:
        return list(self._fields.keys())

    def _root_offset(self, field_offset: int) -> int:
        """Byte offset of a field, measured from the root message's base."""
        return (self._addr - self._root._addr) + field_offset

    def __repr__(self) -> str:
        return f"<roswell.Message {self._mid} at 0x{self._addr:x}>"

    # -- getters ----------------------------------------------------------

    def _get(self, f):
        el = f["element"]
        addr = self._addr + f["offset"]
        mult = f["multiplicity"]
        if mult == "scalar":
            return self._get_elem(el, addr)
        if mult == "array":
            return self._get_array(el, addr, f["array_len"])
        data, size = _read_triple(addr)
        return self._get_seq(el, data, size)

    def _get_elem(self, el, addr: int):
        kind = el["kind"]
        if kind == "prim":
            return _CTYPE[el["prim"]].from_address(addr).value
        if kind == "string":
            data, size = _read_triple(addr)
            if not data or size == 0:
                return ""
            return ctypes.string_at(data, size).decode("utf-8", "replace")
        return Message(self._t, addr, el["message"], root=self._root)

    def _get_array(self, el, addr: int, n: int):
        kind = el["kind"]
        if kind == "prim":
            return _prim_view(el["prim"], addr, n, self._root)
        stride = el["size"]
        if kind == "string":
            return [self._get_elem(el, addr + i * stride) for i in range(n)]
        return [
            Message(self._t, addr + i * stride, el["message"], root=self._root)
            for i in range(n)
        ]

    def _get_seq(self, el, data: int, size: int):
        kind = el["kind"]
        if kind == "prim":
            return _prim_view(el["prim"], data, size, self._root)
        if size == 0 or not data:
            return []
        stride = el["size"]
        if kind == "string":
            return [self._get_elem(el, data + i * stride) for i in range(size)]
        return [
            Message(self._t, data + i * stride, el["message"], root=self._root)
            for i in range(size)
        ]

    # -- setters ----------------------------------------------------------

    def _set(self, f, value) -> None:
        el = f["element"]
        addr = self._addr + f["offset"]
        mult = f["multiplicity"]
        kind = el["kind"]
        if mult == "scalar":
            if kind == "prim":
                _CTYPE[el["prim"]].from_address(addr).value = value
            elif kind == "string":
                b = value.encode("utf-8") if isinstance(value, str) else bytes(value)
                off = self._root_offset(f["offset"])
                if lib.ros_str_assign(self.handle, off, b, len(b)) != 0:
                    raise RoswellError("ros_str_assign failed")
            else:
                raise RoswellError(
                    "cannot assign nested message field whole; mutate its fields"
                )
        elif mult == "array":
            if kind != "prim":
                raise RoswellError("assignment to non-primitive arrays is unsupported")
            self._set_prim_array(el, addr, f["array_len"], value)
        else:  # sequence
            if kind != "prim":
                raise RoswellError("assignment to non-primitive sequences is unsupported")
            self._set_prim_seq(el, f["offset"], value)

    def _set_prim_array(self, el, addr: int, n: int, value) -> None:
        src, count, _keep = _as_c_buffer(el["prim"], value)
        ctypes.memmove(addr, src, min(count, n) * el["size"])

    def _set_prim_seq(self, el, field_offset: int, value) -> None:
        src, count, _keep = _as_c_buffer(el["prim"], value)
        off = self._root_offset(field_offset)
        if lib.ros_seq_assign(self.handle, off, el["size"], el["align"], src, count) != 0:
            raise RoswellError("ros_seq_assign failed")


class _Owned:
    """Buffer-owning shim: numpy adopts it as an array's ``base`` (via the array
    interface), and it holds a strong reference to the owning root
    :class:`Message`, so GC keeps the message — and its buffer — alive for as
    long as the view (or any view derived from it) lives.
    """

    __slots__ = ("_msg", "__array_interface__")

    def __init__(self, msg: "Message", addr: int, typestr: str, n: int):
        self._msg = msg
        self.__array_interface__ = {
            "data": (addr, False),  # (address, read-only=False)
            "shape": (n,),
            "typestr": typestr,
            "version": 3,
        }


def _prim_view(prim: str, addr: int, n: int, owner: "Message"):
    """A numpy view (zero-copy, pinning ``owner``) or list over ``n`` primitives."""
    if n == 0 or not addr:
        return _np.empty(0, dtype=_NPDTYPE[prim]) if _HAS_NUMPY else []
    if _HAS_NUMPY:
        typestr = _np.dtype(_NPDTYPE[prim]).str
        return _np.asarray(_Owned(owner, addr, typestr, n))
    arr = (_CTYPE[prim] * n).from_address(addr)
    return list(arr)


def _as_c_buffer(prim: str, value):
    """Return ``(src, count, keepalive)`` for a primitive-sequence source.

    ``keepalive`` pins the backing buffer; the caller must hold it for the
    duration of the FFI call that reads ``src``.
    """
    if _HAS_NUMPY:
        arr = _np.ascontiguousarray(value, dtype=_NPDTYPE[prim])
        return ctypes.c_void_p(arr.ctypes.data), int(arr.size), arr
    seq = list(value)
    buf = (_CTYPE[prim] * len(seq))(*seq)
    return buf, len(seq), buf

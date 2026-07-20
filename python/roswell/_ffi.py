"""ctypes bindings to the roswell C shared library.

This module locates and loads the ``roswell_c`` cdylib, verifies its ABI version,
declares the C-ABI signatures from ``include/roswell.h``, and translates non-zero
status codes into the :mod:`roswell` exception hierarchy. It holds no message
logic — every operation is a call across the FFI boundary.

Objects are addressed by generation-counted ``RcmHandle`` values (``uint64``),
not raw pointers: a stale handle (use-after-free, use-after-shutdown) or a
wrong-kind/wrong-type handle validates to a precise error rather than undefined
behaviour. See ``include/roswell.h``.
"""

from __future__ import annotations

import ctypes
import os
import sys
from ctypes import (
    POINTER,
    c_char_p,
    c_int,
    c_int8,
    c_int64,
    c_size_t,
    c_uint8,
    c_uint32,
    c_uint64,
    c_void_p,
)
from pathlib import Path

#: ABI version this binding is written against; must match ``RCM_ABI_VERSION``
#: in the loaded library (and ``roswell.h``).
ABI_VERSION = 2

# Status codes mirroring the RCM_ERR_* codes in roswell.h.
RCM_OK = 0
RCM_ERR = -1
RCM_ERR_NULL_HANDLE = -2
RCM_ERR_STALE_HANDLE = -3
RCM_ERR_WRONG_KIND = -4
RCM_ERR_TYPE_MISMATCH = -5
RCM_ERR_ENCODE = -6
RCM_ERR_DECODE = -7
RCM_ERR_UNKNOWN_TOKEN = -8


class RoswellError(Exception):
    """Base class for all errors raised by roswell."""


class QosError(RoswellError):
    """A QoS-related failure."""


class RoswellTimeout(RoswellError, TimeoutError):
    """A blocking call (service call, wait) exceeded its timeout."""


class StaleHandleError(RoswellError):
    """A handle was used after it (or its context) was freed."""


class WrongKindError(RoswellError):
    """A handle of one kind was passed where another kind was expected."""


class TypeMismatchError(RoswellError):
    """A message's type did not match the endpoint it was used with."""


class QosIncompatibleWarning(UserWarning):
    """Emitted when an endpoint reports incompatible QoS with a peer."""


#: Map negative status codes to their precise exception classes.
_EXC_FOR_CODE = {
    RCM_ERR_NULL_HANDLE: StaleHandleError,
    RCM_ERR_STALE_HANDLE: StaleHandleError,
    RCM_ERR_WRONG_KIND: WrongKindError,
    RCM_ERR_TYPE_MISMATCH: TypeMismatchError,
}


def error_for(code: int, what: str) -> RoswellError:
    """The exception for a negative status ``code``, carrying ``ros_last_error``."""
    msg = f"{what}: {last_error()}"
    return _EXC_FOR_CODE.get(code, RoswellError)(msg)


def check_rc(code: int, what: str) -> None:
    """Raise the precise exception if ``code`` is a failure (< 0)."""
    if code < 0:
        raise error_for(code, what)


def _lib_name() -> str:
    if sys.platform == "darwin":
        return "libroswell_c.dylib"
    if sys.platform in ("win32", "cygwin"):
        return "roswell_c.dll"
    return "libroswell_c.so"


def _candidate_paths() -> list[Path]:
    """cdylib search order: env var > bundled next to package > dev build tree."""
    name = _lib_name()
    paths: list[Path] = []
    env = os.environ.get("ROSWELL_LIB")
    if env:
        paths.append(Path(env))
    here = Path(__file__).resolve().parent
    paths.append(here / "_lib" / name)  # bundled inside the wheel
    paths.append(here / name)  # legacy: alongside the package
    # Development: the workspace target directory (repo root is python/..).
    repo_root = here.parent.parent
    for profile in ("release", "debug"):
        paths.append(repo_root / "target" / profile / name)
    return paths


def _load() -> ctypes.CDLL:
    tried = []
    for path in _candidate_paths():
        tried.append(str(path))
        if path.is_file():
            return ctypes.CDLL(str(path))
    raise RoswellError(
        "could not locate the roswell_c shared library; set ROSWELL_LIB or run "
        "`cargo build -p roswell-c`. Tried:\n  " + "\n  ".join(tried)
    )


class RcmQos(ctypes.Structure):
    """Mirror of the C ``RcmQos`` struct."""

    _fields_ = [
        ("reliability", c_uint8),
        ("durability", c_uint8),
        ("keep_all", c_uint8),
        ("depth", c_uint32),
        ("deadline_ms", c_int64),
        ("lifespan_ms", c_int64),
    ]


class RcmParamValue(ctypes.Structure):
    """Mirror of the C ``RcmParamValue`` struct (scalar parameter value)."""

    _fields_ = [
        ("kind", c_uint8),
        ("boolean", c_uint8),
        ("integer", c_int64),
        ("number", ctypes.c_double),
        ("text", c_char_p),
    ]


#: ParameterType tags shared with the Rust ``ParameterType`` enum.
PARAM_BOOL = 1
PARAM_INTEGER = 2
PARAM_DOUBLE = 3
PARAM_STRING = 4
PARAM_BYTE_ARRAY = 5
PARAM_BOOL_ARRAY = 6
PARAM_INTEGER_ARRAY = 7
PARAM_DOUBLE_ARRAY = 8
PARAM_STRING_ARRAY = 9


_U8 = POINTER(c_uint8)
_H = c_uint64  # RcmHandle


def _declare(lib: ctypes.CDLL) -> None:
    """Attach argtypes/restypes to every function used by the package."""

    def sig(name: str, restype, argtypes) -> None:
        fn = getattr(lib, name)
        fn.restype = restype
        fn.argtypes = argtypes
        if name.startswith("rcm_"):
            setattr(lib, f"ros_{name[4:]}", fn)

    sig("rcm_abi_version", c_uint32, [])
    sig("rcm_version_string", c_char_p, [])

    sig("rcm_last_error", c_char_p, [])
    sig("rcm_string_free", None, [c_void_p])

    sig("rcm_qos_preset", RcmQos, [c_char_p])

    sig("rcm_init", _H, [c_int])
    sig("rcm_shutdown", None, [_H])

    sig("rcm_type_load", _H, [c_char_p, POINTER(c_char_p), c_size_t])
    sig(
        "rcm_type_load_srv",
        c_int,
        [c_char_p, POINTER(c_char_p), c_size_t, POINTER(_H), POINTER(_H)],
    )
    sig("rcm_type_resolve", _H, [c_char_p, POINTER(c_char_p), c_size_t])
    sig(
        "rcm_type_resolve_srv",
        c_int,
        [c_char_p, POINTER(c_char_p), c_size_t, POINTER(_H), POINTER(_H)],
    )
    sig("rcm_type_layout_json", c_void_p, [_H])
    sig("rcm_type_dds_name", c_void_p, [_H])
    sig("rcm_type_free", c_int, [_H])

    sig("rcm_msg_alloc", _H, [_H])
    sig("rcm_msg_data", _U8, [_H])
    sig("rcm_msg_fini", c_int, [_H])
    sig("rcm_msg_free", c_int, [_H])
    sig("rcm_seq_assign", c_int, [_H, c_size_t, c_size_t, c_size_t, c_void_p, c_size_t])
    sig("rcm_str_assign", c_int, [_H, c_size_t, c_void_p, c_size_t])

    sig("rcm_publisher", _H, [_H, c_char_p, _H, POINTER(RcmQos)])
    sig("rcm_publish", c_int, [_H, _H])
    sig("rcm_publisher_events", c_void_p, [_H])
    sig("rcm_publisher_free", c_int, [_H])

    sig("rcm_subscriber", _H, [_H, c_char_p, _H, POINTER(RcmQos)])
    sig("rcm_take", c_int, [_H, _H])
    sig("rcm_wait", c_int, [POINTER(_H), c_size_t, c_int])
    sig("rcm_subscriber_events", c_void_p, [_H])
    sig("rcm_subscriber_free", c_int, [_H])

    sig("rcm_service", _H, [_H, c_char_p, _H, _H])
    sig("rcm_service_take_request", c_int, [_H, _H, POINTER(c_uint64)])
    sig("rcm_service_send_reply", c_int, [_H, c_uint64, _H])
    sig("rcm_service_free", c_int, [_H])

    sig("rcm_client", _H, [_H, c_char_p, _H, _H])
    sig("rcm_call", c_int, [_H, _H, _H, c_int])
    sig("rcm_client_free", c_int, [_H])

    sig(
        "rcm_action_client",
        _H,
        [_H, c_char_p, c_char_p, POINTER(c_char_p), c_size_t],
    )
    sig("rcm_action_load", c_int, [c_char_p, POINTER(c_char_p), c_size_t, POINTER(_H)])
    sig("rcm_action_goal_type", _H, [_H])
    sig("rcm_action_result_type", _H, [_H])
    sig("rcm_action_feedback_type", _H, [_H])
    sig("rcm_action_server_ready", c_int, [_H])
    sig("rcm_action_send_goal", c_int, [_H, _H, c_int, _U8, _U8])
    sig("rcm_action_get_result", c_int, [_H, _U8, _H, c_int, POINTER(c_int8)])
    sig("rcm_action_poll_feedback", c_int, [_H, _H, _U8])
    sig("rcm_action_cancel_goal", c_int, [_H, _U8, c_int, POINTER(c_int8)])
    sig("rcm_action_client_free", c_int, [_H])

    sig("rcm_param_server", _H, [_H, c_char_p])
    sig("rcm_param_set", c_int, [_H, c_char_p, POINTER(RcmParamValue)])
    sig("rcm_param_set_array", c_int, [_H, c_char_p, c_uint8, c_void_p, c_size_t])
    sig(
        "rcm_param_set_string_array",
        c_int,
        [_H, c_char_p, POINTER(c_char_p), c_size_t],
    )
    sig("rcm_param_get_json", c_void_p, [_H, c_char_p])
    sig("rcm_param_list_json", c_void_p, [_H])
    sig("rcm_param_server_free", c_int, [_H])

    sig("rcm_graph_json", c_void_p, [_H, c_int])
    sig("rcm_node", c_int, [_H, c_char_p, c_char_p])

    sig("rcm_bag_open_write", _H, [c_char_p, c_char_p])
    sig("rcm_bag_write", c_int, [_H, c_char_p, c_int64, _H])
    sig("rcm_bag_writer_close", c_int, [_H])
    sig("rcm_bag_open_read", _H, [c_char_p])
    sig("rcm_bag_next", c_int, [_H])
    sig("rcm_bag_info_json", c_void_p, [_H])
    sig("rcm_bag_data", c_int, [_H, c_void_p, c_size_t])
    sig("rcm_bag_decode", c_int, [_H, _H])
    sig("rcm_bag_schema", c_void_p, [_H, c_char_p])
    sig("rcm_bag_reader_free", c_int, [_H])

    sig("rcm_tf_buffer", _H, [_H])
    sig(
        "rcm_tf_lookup",
        c_int,
        [_H, c_char_p, c_char_p, c_int64, POINTER(ctypes.c_double)],
    )
    sig(
        "rcm_tf_broadcast",
        c_int,
        [_H, c_char_p, c_char_p, c_int64, c_uint8, POINTER(ctypes.c_double)],
    )
    sig("rcm_tf_free", c_int, [_H])

    sig(
        "rcm_action_server",
        _H,
        [_H, c_char_p, c_char_p, POINTER(c_char_p), c_size_t],
    )
    sig("rcm_action_server_goal_type", _H, [_H])
    sig("rcm_action_server_result_type", _H, [_H])
    sig("rcm_action_server_feedback_type", _H, [_H])
    sig("rcm_action_server_spin", c_int, [_H])
    sig("rcm_action_server_take_goal", c_int, [_H, _H, _U8])
    sig("rcm_action_server_cancel_requested", c_int, [_H, _U8])
    sig("rcm_action_server_publish_feedback", c_int, [_H, _U8, _H])
    sig("rcm_action_server_finish", c_int, [_H, _U8, c_int8, _H])
    sig("rcm_action_server_free", c_int, [_H])


def _check_abi(lib: ctypes.CDLL) -> None:
    got = lib.ros_abi_version()
    if got != ABI_VERSION:
        ver = lib.ros_version_string()
        ver_s = ver.decode("utf-8", "replace") if ver else "?"
        raise RoswellError(
            f"roswell_c ABI mismatch: this binding expects ABI {ABI_VERSION}, "
            f"but the loaded library (crate {ver_s}) reports ABI {got}. "
            "Rebuild the cdylib with `cargo build -p roswell-c --release`."
        )


lib = _load()
# Declare the version probes before the full surface so the ABI check is safe.
lib.rcm_abi_version.restype = c_uint32
lib.rcm_abi_version.argtypes = []
lib.rcm_version_string.restype = c_char_p
lib.rcm_version_string.argtypes = []
lib.ros_abi_version = lib.rcm_abi_version
lib.ros_version_string = lib.rcm_version_string
_check_abi(lib)
_declare(lib)


def last_error() -> str:
    msg = lib.ros_last_error()
    return msg.decode("utf-8", "replace") if msg else ""


def take_json(ptr: int) -> str:
    """Decode and free a heap ``char*`` returned by an ``ros_*_json`` function."""
    if not ptr:
        raise RoswellError(last_error() or "null string returned")
    try:
        return ctypes.cast(ptr, c_char_p).value.decode("utf-8", "replace")
    finally:
        lib.ros_string_free(ptr)


def check_handle(handle: int, what: str) -> int:
    """Validate an allocating call: a 0 handle means failure."""
    if not handle:
        raise RoswellError(f"{what}: {last_error()}")
    return handle


# Backwards-compatible alias (endpoints/types are handles now, not pointers).
check_ptr = check_handle

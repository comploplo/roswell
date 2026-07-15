"""ctypes bindings to the roscmp C shared library.

This module locates and loads the ``roscmp_c`` cdylib, verifies its ABI version,
declares the C-ABI signatures from ``include/roscmp.h``, and translates non-zero
status codes into the :mod:`roscmp` exception hierarchy. It holds no message
logic — every operation is a call across the FFI boundary.

Objects are addressed by generation-counted ``RcmHandle`` values (``uint64``),
not raw pointers: a stale handle (use-after-free, use-after-shutdown) or a
wrong-kind/wrong-type handle validates to a precise error rather than undefined
behaviour. See ``include/roscmp.h``.
"""

from __future__ import annotations

import ctypes
import os
import sys
from ctypes import (
    POINTER,
    c_char_p,
    c_int,
    c_int64,
    c_size_t,
    c_uint8,
    c_uint32,
    c_uint64,
    c_void_p,
)
from pathlib import Path

#: ABI version this binding is written against; must match ``RCM_ABI_VERSION``
#: in the loaded library (and ``roscmp.h``).
ABI_VERSION = 2

# Status codes mirroring the RCM_ERR_* codes in roscmp.h.
RCM_OK = 0
RCM_ERR = -1
RCM_ERR_NULL_HANDLE = -2
RCM_ERR_STALE_HANDLE = -3
RCM_ERR_WRONG_KIND = -4
RCM_ERR_TYPE_MISMATCH = -5
RCM_ERR_ENCODE = -6
RCM_ERR_DECODE = -7
RCM_ERR_UNKNOWN_TOKEN = -8


class RoscmpError(Exception):
    """Base class for all errors raised by roscmp."""


class QosError(RoscmpError):
    """A QoS-related failure."""


class RoscmpTimeout(RoscmpError, TimeoutError):
    """A blocking call (service call, wait) exceeded its timeout."""


class StaleHandleError(RoscmpError):
    """A handle was used after it (or its context) was freed."""


class WrongKindError(RoscmpError):
    """A handle of one kind was passed where another kind was expected."""


class TypeMismatchError(RoscmpError):
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


def error_for(code: int, what: str) -> RoscmpError:
    """The exception for a negative status ``code``, carrying ``rcm_last_error``."""
    msg = f"{what}: {last_error()}"
    return _EXC_FOR_CODE.get(code, RoscmpError)(msg)


def check_rc(code: int, what: str) -> None:
    """Raise the precise exception if ``code`` is a failure (< 0)."""
    if code < 0:
        raise error_for(code, what)


def _lib_name() -> str:
    if sys.platform == "darwin":
        return "libroscmp_c.dylib"
    if sys.platform in ("win32", "cygwin"):
        return "roscmp_c.dll"
    return "libroscmp_c.so"


def _candidate_paths() -> list[Path]:
    """cdylib search order: env var > bundled next to package > dev build tree."""
    name = _lib_name()
    paths: list[Path] = []
    env = os.environ.get("ROSCMP_LIB")
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
    raise RoscmpError(
        "could not locate the roscmp_c shared library; set ROSCMP_LIB or run "
        "`cargo build -p roscmp-c`. Tried:\n  " + "\n  ".join(tried)
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


_U8 = POINTER(c_uint8)
_H = c_uint64  # RcmHandle


def _declare(lib: ctypes.CDLL) -> None:
    """Attach argtypes/restypes to every function used by the package."""

    def sig(name: str, restype, argtypes) -> None:
        fn = getattr(lib, name)
        fn.restype = restype
        fn.argtypes = argtypes

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

    sig("rcm_graph_json", c_void_p, [_H, c_int])
    sig("rcm_node", c_int, [_H, c_char_p, c_char_p])


def _check_abi(lib: ctypes.CDLL) -> None:
    got = lib.rcm_abi_version()
    if got != ABI_VERSION:
        ver = lib.rcm_version_string()
        ver_s = ver.decode("utf-8", "replace") if ver else "?"
        raise RoscmpError(
            f"roscmp_c ABI mismatch: this binding expects ABI {ABI_VERSION}, "
            f"but the loaded library (crate {ver_s}) reports ABI {got}. "
            "Rebuild the cdylib with `cargo build -p roscmp-c --release`."
        )


lib = _load()
# Declare the version probes before the full surface so the ABI check is safe.
lib.rcm_abi_version.restype = c_uint32
lib.rcm_abi_version.argtypes = []
lib.rcm_version_string.restype = c_char_p
lib.rcm_version_string.argtypes = []
_check_abi(lib)
_declare(lib)


def last_error() -> str:
    msg = lib.rcm_last_error()
    return msg.decode("utf-8", "replace") if msg else ""


def take_json(ptr: int) -> str:
    """Decode and free a heap ``char*`` returned by an ``rcm_*_json`` function."""
    if not ptr:
        raise RoscmpError(last_error() or "null string returned")
    try:
        return ctypes.cast(ptr, c_char_p).value.decode("utf-8", "replace")
    finally:
        lib.rcm_string_free(ptr)


def check_handle(handle: int, what: str) -> int:
    """Validate an allocating call: a 0 handle means failure."""
    if not handle:
        raise RoscmpError(f"{what}: {last_error()}")
    return handle


# Backwards-compatible alias (endpoints/types are handles now, not pointers).
check_ptr = check_handle

"""Hatchling build hook: compile the roscmp_c cdylib and bundle it, plus the
sample interface files, into a platform wheel tagged ``py3-none-<platform>``.

roscmp is pure ctypes over a C ABI — there is no CPython ABI coupling — so the
wheel carries the platform tag but the generic ``py3`` / ``none`` python/ABI
tags. The compiled library and the ``samples/`` interface tree are injected via
``force_include`` (no copies into the source tree)."""

from __future__ import annotations

import shutil
import subprocess
import sys
from pathlib import Path

from hatchling.builders.hooks.plugin.interface import BuildHookInterface

_INTERFACE_EXTS = (".msg", ".srv", ".action")


def _lib_filename() -> str:
    if sys.platform == "darwin":
        return "libroscmp_c.dylib"
    if sys.platform in ("win32", "cygwin"):
        return "roscmp_c.dll"
    return "libroscmp_c.so"


def _platform_tag() -> str:
    from packaging.tags import platform_tags

    return next(iter(platform_tags()))


class CustomBuildHook(BuildHookInterface):
    def initialize(self, version, build_data):  # noqa: ARG002
        root = Path(self.root)          # the python/ project directory
        workspace = root.parent         # the cargo workspace root
        lib_name = _lib_filename()

        cargo = shutil.which("cargo")
        if not cargo:
            raise RuntimeError(
                "cargo was not found on PATH; a Rust toolchain is required to "
                "build the roscmp wheel (install from https://rustup.rs)"
            )
        subprocess.run(
            [cargo, "build", "-p", "roscmp-c", "--release"],
            cwd=str(workspace),
            check=True,
        )
        built = workspace / "target" / "release" / lib_name
        if not built.is_file():
            raise RuntimeError(
                f"expected the compiled library at {built}, but it was not found"
            )

        force = build_data.setdefault("force_include", {})
        force[str(built)] = f"roscmp/_lib/{lib_name}"

        samples = workspace / "samples"
        for src in sorted(samples.rglob("*")):
            if src.is_file() and src.suffix in _INTERFACE_EXTS:
                rel = src.relative_to(samples).as_posix()
                force[str(src)] = f"roscmp/interfaces/{rel}"

        build_data["pure_python"] = False
        build_data["infer_tag"] = False
        build_data["tag"] = f"py3-none-{_platform_tag()}"

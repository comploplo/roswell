"""Hatchling build hook: compile the roswell_c cdylib and bundle it, plus the
sample interface files, into a platform wheel tagged ``py3-none-<platform>``.

roswell is pure ctypes over a C ABI — there is no CPython ABI coupling — so the
wheel carries the platform tag but the generic ``py3`` / ``none`` python/ABI
tags. The compiled library and the ``samples/`` interface tree are injected via
``force_include`` (no copies into the source tree)."""

from __future__ import annotations

import os
import platform
import shutil
import subprocess
import sys
from pathlib import Path

from hatchling.builders.hooks.plugin.interface import BuildHookInterface

_INTERFACE_EXTS = (".msg", ".srv", ".action")


def _lib_filename() -> str:
    if sys.platform == "darwin":
        return "libroswell_c.dylib"
    if sys.platform in ("win32", "cygwin"):
        return "roswell_c.dll"
    return "libroswell_c.so"


def _is_macos_universal2() -> bool:
    """True when cibuildwheel is driving a macOS universal2 build. It signals
    this through the compiler env (``ARCHFLAGS`` names both arches) and
    ``_PYTHON_HOST_PLATFORM`` — not a build-identifier var, which 3.x drops from
    the isolated build environment."""
    if sys.platform != "darwin":
        return False
    if os.environ.get("_PYTHON_HOST_PLATFORM", "").endswith("universal2"):
        return True
    archflags = os.environ.get("ARCHFLAGS", "")
    return "arm64" in archflags and "x86_64" in archflags


def _platform_tag() -> str:
    if sys.platform == "darwin":
        if _is_macos_universal2():
            return "macosx_11_0_universal2"
        return f"macosx_11_0_{platform.machine()}"

    from packaging.tags import platform_tags

    return next(iter(platform_tags()))


class CustomBuildHook(BuildHookInterface):
    def initialize(self, version, build_data):  # noqa: ARG002
        root = Path(self.root)          # the python/ project directory
        workspace = root if (root / "Cargo.toml").is_file() else root.parent
        lib_name = _lib_filename()

        cargo = shutil.which("cargo")
        if not cargo:
            raise RuntimeError(
                "cargo was not found on PATH; a Rust toolchain is required to "
                "build the roswell wheel (install from https://rustup.rs)"
            )
        if _is_macos_universal2():
            built = _build_macos_universal2(workspace, cargo, lib_name)
        else:
            subprocess.run(
                [cargo, "build", "-p", "roswell-c", "--release"],
                cwd=str(workspace),
                check=True,
            )
            built = workspace / "target" / "release" / lib_name
        if not built.is_file():
            raise RuntimeError(
                f"expected the compiled library at {built}, but it was not found"
            )

        force = build_data.setdefault("force_include", {})
        force[str(built)] = f"roswell/_lib/{lib_name}"

        for license_name in ("LICENSE-MIT", "LICENSE-APACHE"):
            license_file = workspace / license_name
            force[str(license_file)] = f"roswell/licenses/{license_name}"

        samples = workspace / "samples"
        for src in sorted(samples.rglob("*")):
            if src.is_file() and src.suffix in _INTERFACE_EXTS:
                rel = src.relative_to(samples).as_posix()
                force[str(src)] = f"roswell/interfaces/{rel}"

        build_data["pure_python"] = False
        build_data["infer_tag"] = False
        build_data["tag"] = f"py3-none-{_platform_tag()}"


def _build_macos_universal2(workspace: Path, cargo: str, lib_name: str) -> Path:
    targets = ["aarch64-apple-darwin", "x86_64-apple-darwin"]
    rustup = shutil.which("rustup")
    if rustup:
        subprocess.run([rustup, "target", "add", *targets], cwd=str(workspace), check=True)
    for target in targets:
        subprocess.run(
            [cargo, "build", "-p", "roswell-c", "--release", "--target", target],
            cwd=str(workspace),
            check=True,
        )

    out_dir = workspace / "target" / "universal2" / "release"
    out_dir.mkdir(parents=True, exist_ok=True)
    built = out_dir / lib_name
    subprocess.run(
        [
            "lipo",
            "-create",
            str(workspace / "target" / targets[0] / "release" / lib_name),
            str(workspace / "target" / targets[1] / "release" / lib_name),
            "-output",
            str(built),
        ],
        check=True,
    )
    return built

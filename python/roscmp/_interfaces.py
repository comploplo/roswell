"""Locate bundled ROS interface files for path-free ``load_type`` calls.

The wheel ships the sample interface tree under ``roscmp/interfaces/`` keeping
the ROS package shape (``<pkg>/{msg,srv,action}/<Name>.<ext>``). Given a
reference like ``"geometry_msgs/msg/Twist"`` this module finds the root file and
the full set of bundled ``.msg`` files to hand the Rust loader as candidate
dependencies — file discovery is plumbing; parsing and dependency resolution
stay in Rust, which resolves only the types the root actually references.
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import Optional

_HERE = Path(__file__).resolve().parent


def _roots() -> list[Path]:
    """Interface search roots: env override > bundled wheel tree > dev samples/."""
    roots: list[Path] = []
    env = os.environ.get("ROSCMP_INTERFACES")
    if env:
        roots.append(Path(env))
    roots.append(_HERE / "interfaces")           # bundled in the wheel
    roots.append(_HERE.parent.parent / "samples")  # development checkout
    return [r for r in roots if r.is_dir()]


def resolve(ref, ext: str) -> Optional[tuple[Path, list[Path]]]:
    """Resolve a ``pkg/{msg,srv,action}/Name`` reference against a bundled root.

    Returns ``(root_file, dep_files)`` where ``dep_files`` is every bundled
    ``.msg`` (minus the root itself), or ``None`` if no root matches.
    """
    name = str(ref)
    suffix = "." + ext
    if name.endswith(suffix):
        name = name[: -len(suffix)]
    for root in _roots():
        candidate = root / (name + suffix)
        if candidate.is_file():
            deps = [p for p in sorted(root.rglob("*.msg")) if p != candidate]
            return candidate, deps
    return None

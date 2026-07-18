"""Collect ROS interface search roots for reference-based ``load_type`` calls.

A *reference* like ``"robot_msgs/msg/Detection"`` is resolved entirely in Rust
(see ``rcm_type_resolve``): Python's only job is to hand the loader the list of
root directories to search. Roots come, in priority order, from the caller's
``type_paths``, the ``ROSCMP_TYPE_PATH`` and ``AMENT_PREFIX_PATH`` environment
variables (colon/``os.pathsep``-separated), the interface tree bundled in the
wheel, and — in a development checkout — the repo ``samples/`` tree. Each root is
probed both as a plain package tree (``<root>/<pkg>/msg/<Name>.msg``) and as an
ament install prefix (``<root>/share/<pkg>/msg/<Name>.msg``) by the Rust loader.
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import Sequence

_HERE = Path(__file__).resolve().parent


def _split_env(name: str) -> list[Path]:
    value = os.environ.get(name)
    if not value:
        return []
    return [Path(p) for p in value.split(os.pathsep) if p]


def search_roots(type_paths: Sequence = ()) -> list[str]:
    """The ordered, de-duplicated list of interface search-root directories."""
    roots: list[Path] = [Path(p) for p in type_paths]
    roots += _split_env("ROSCMP_TYPE_PATH")
    roots += _split_env("AMENT_PREFIX_PATH")
    roots += _split_env("ROSCMP_INTERFACES")  # legacy bundled-tree override
    roots.append(_HERE / "interfaces")  # bundled in the wheel
    roots.append(_HERE.parent.parent / "samples")  # development checkout

    seen: set[str] = set()
    out: list[str] = []
    for r in roots:
        if not r.is_dir():
            continue
        s = str(r)
        if s not in seen:
            seen.add(s)
            out.append(s)
    return out

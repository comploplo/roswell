"""The package must work with numpy absent: primitive arrays degrade to lists.

This deterministically forces the no-numpy path by flipping the module flags
`roscmp._types` reads at call time, so it runs (and is CI-able) even in a venv
that *has* numpy. A real numpy-less venv run is covered separately in CI; this
guards the fallback logic itself. It imports no numpy.
"""

import pytest

import roscmp
from roscmp import _types


@pytest.fixture
def no_numpy(monkeypatch):
    monkeypatch.setattr(_types, "_HAS_NUMPY", False)
    monkeypatch.setattr(_types, "_np", None)


def _sample_type(node, fixture_dir):
    return node.load_type(fixture_dir / "test_msgs" / "msg" / "Sample.msg")


def test_prim_sequence_is_list_without_numpy(no_numpy, fixture_dir):
    node = roscmp.Node("py_nonumpy", domain=0)
    try:
        T = _sample_type(node, fixture_dir)
        msg = T.alloc()
        msg.values = [1.0, 2.5, -3.25, 42.0]
        view = msg.values
        assert isinstance(view, list)
        assert view == [1.0, 2.5, -3.25, 42.0]
        # Strings still work.
        msg.label = "no numpy here"
        assert msg.label == "no numpy here"
    finally:
        node.close()


def test_empty_sequence_is_empty_list_without_numpy(no_numpy, fixture_dir):
    node = roscmp.Node("py_nonumpy_empty", domain=0)
    try:
        T = _sample_type(node, fixture_dir)
        msg = T.alloc()
        assert msg.values == []
    finally:
        node.close()

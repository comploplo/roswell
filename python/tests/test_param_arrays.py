"""Array parameters: values cross the FFI as packed arrays (no JSON parsing in
Rust — the hand-rolled JSON emitter renders them on the way back out)."""

import pytest

import roscmp


@pytest.fixture
def node():
    n = roscmp.Node("param_array_node", domain=0)
    try:
        yield n
    finally:
        n.close()


def test_array_parameter_roundtrips(node):
    node.set_parameter("ints", [1, 2, 3])
    assert node.get_parameter("ints") == [1, 2, 3]

    node.set_parameter("doubles", [1.5, -2.0, 3])  # mixed int/float -> double
    assert node.get_parameter("doubles") == [1.5, -2.0, 3.0]

    node.set_parameter("bools", [True, False, True])
    assert node.get_parameter("bools") == [True, False, True]

    node.set_parameter("strings", ["a", "b\"quote", "c"])
    assert node.get_parameter("strings") == ["a", 'b"quote', "c"]

    node.set_parameter("blob", b"\x00\x01\xff")
    assert node.get_parameter("blob") == [0, 1, 255]

    assert set(node.list_parameters()) >= {"ints", "doubles", "bools", "strings", "blob"}


def test_bad_arrays_raise(node):
    with pytest.raises(TypeError):
        node.set_parameter("empty", [])
    with pytest.raises(TypeError):
        node.set_parameter("mixed", [1, "two"])


def test_node_context_manager():
    with roscmp.Node("ctx_node", domain=0) as n:
        n.set_parameter("x", 1)
        assert n.get_parameter("x") == 1
    assert repr(n).endswith("closed>")

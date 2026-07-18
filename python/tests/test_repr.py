"""__repr__ on the public objects surfaces topic/name/type for observability."""

import roscmp


def _sample_type(node, fixture_dir):
    return node.load_type(fixture_dir / "test_msgs" / "msg" / "Sample.msg")


def test_reprs_show_identity(fixture_dir):
    node = roscmp.Node("py_repr", domain=0)
    try:
        assert "py_repr" in repr(node)
        T = _sample_type(node, fixture_dir)
        pub = node.publisher("/rp", T)
        sub = node.subscribe("/rp", T)
        assert "/rp" in repr(pub) and "Publisher" in repr(pub)
        assert "/rp" in repr(sub) and "Subscriber" in repr(sub)
    finally:
        node.close()
    assert "closed" in repr(node)

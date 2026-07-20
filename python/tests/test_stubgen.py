"""Typed-stub generation from the Rust layout JSON.

The generator is pure formatting: every field kind/offset comes from
``ros_type_layout_json`` (the same metadata the runtime consumes), so these
tests assert the emitted ``.pyi`` text for bundled, nested-closure, and
workspace-fixture custom types — and, when mypy is installed, that the stubs
actually typecheck a using snippet.
"""

import subprocess
import sys

import pytest

from roswell import stubgen


def test_generates_stub_modules_for_closures(tmp_path, fixture_dir):
    paths = stubgen.generate(
        ["std_msgs/msg/String", "sensor_msgs/msg/Imu", "robot_msgs/msg/Detection"],
        tmp_path,
        type_paths=[fixture_dir / "ws"],
    )
    names = {p.name for p in paths}
    # One module per package across all three dependency closures.
    assert {
        "std_msgs.pyi",
        "sensor_msgs.pyi",
        "geometry_msgs.pyi",
        "builtin_interfaces.pyi",
        "robot_msgs.pyi",
        "sensor_pkg.pyi",
    } <= names

    std = (tmp_path / "std_msgs.pyi").read_text()
    assert "class String:" in std
    assert "    data: str" in std
    assert "class Header:" in std  # pulled in by the Imu closure

    imu = (tmp_path / "sensor_msgs.pyi").read_text()
    assert "import geometry_msgs" in imu
    assert "import numpy" in imu
    assert "class Imu:" in imu
    assert "    header: std_msgs.Header" in imu
    assert "    orientation: geometry_msgs.Quaternion" in imu
    assert "    orientation_covariance: numpy.ndarray | list[float]" in imu

    det = (tmp_path / "robot_msgs.pyi").read_text()
    assert "    label: str" in det
    assert "    position: geometry_msgs.Point" in det
    assert "    reading: sensor_pkg.Reading" in det


def test_bare_package_reference_expands_to_all_messages(tmp_path):
    paths = stubgen.generate(["std_msgs"], tmp_path)
    std = next(p for p in paths if p.name == "std_msgs.pyi").read_text()
    assert "class String:" in std


def test_cli_entrypoint(tmp_path, capsys):
    assert stubgen.main(["std_msgs/msg/String", "-o", str(tmp_path)]) == 0
    out = capsys.readouterr().out
    assert "std_msgs.pyi" in out
    assert (tmp_path / "std_msgs.pyi").is_file()


def test_unknown_package_errors(tmp_path):
    import roswell

    with pytest.raises(roswell.RoswellError):
        stubgen.generate(["no_such_pkg"], tmp_path)


def _run_mypy(tmp_path, snippet: str) -> int:
    src = tmp_path / "snippet.py"
    src.write_text(snippet)
    proc = subprocess.run(
        [sys.executable, "-m", "mypy", "--no-error-summary", str(src)],
        capture_output=True,
        text=True,
        env={"MYPYPATH": str(tmp_path), "PATH": "/usr/bin:/bin"},
    )
    return proc.returncode


def test_stubs_typecheck_with_mypy(tmp_path):
    pytest.importorskip("mypy")
    stubgen.generate(["sensor_msgs/msg/Imu"], tmp_path)
    ok = _run_mypy(
        tmp_path,
        "import sensor_msgs\n"
        "def f(m: sensor_msgs.Imu) -> float:\n"
        "    return m.linear_acceleration.x\n",
    )
    assert ok == 0
    bad = _run_mypy(
        tmp_path,
        "import sensor_msgs\n"
        "def f(m: sensor_msgs.Imu) -> str:\n"
        "    return m.linear_acceleration.x\n",  # float, not str
    )
    assert bad != 0

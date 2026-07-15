import pathlib

import pytest

HERE = pathlib.Path(__file__).resolve().parent
REPO = HERE.parent.parent
SAMPLES = REPO / "samples"
FIXTURES = HERE / "fixtures"


@pytest.fixture
def sample_dir() -> pathlib.Path:
    return SAMPLES


@pytest.fixture
def fixture_dir() -> pathlib.Path:
    return FIXTURES

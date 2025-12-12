import random
from pathlib import Path
from unittest.mock import MagicMock

import pytest

from cbs_sipp.map.grid_map import GridMap


@pytest.fixture(autouse=True)
def test_seed():
    seed = 42
    random.seed(seed)


@pytest.fixture
def mock_path() -> MagicMock:
    mock_path = MagicMock(spec=Path)
    mock_path.is_file.return_value = True
    mock_path.suffix = ".toml"
    return mock_path


@pytest.fixture
def grid_map() -> GridMap:
    map = [
        [False, False, True, False, False, False, False, False],
        [False, False, False, True, False, False, False, False],
        [False, False, True, False, False, False, False, False],
        [False, False, False, False, False, False, False, False],
        [False, False, False, False, False, False, False, True],
        [False, False, False, False, False, True, False, False],
        [False, False, False, False, False, False, True, False],
        [False, False, False, False, False, False, False, False],
    ]
    return GridMap(map)

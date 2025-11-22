import re
from pathlib import Path
from unittest.mock import mock_open, patch

import pytest

from cbs_sipp.map.dynamic_env import import_dynamic_env

INSTANCES_PATH = "instances/dynamic/maps"


def test_import_dynamic_env_success():
    file = "8-8-simple.txt"

    expected_map = [
        ["..@....."],
        ["...@.a.."],
        ["..@....."],
        ["....cc..."],
        [".......@"],
        ["..b..@.."],
        ["......@."],
        ["........"],
    ]
    expected_dynamic_obstacles = {"a", "b", "cc"}

    assert expected_map, expected_dynamic_obstacles == import_dynamic_env(
        f"{INSTANCES_PATH}/{file}"
    )


@pytest.mark.parametrize(
    "map, error_text",
    [
        (
            """3 4\n@ . a .\nb @ . c .\n. d . .\n""",
            "Expected row to have 4 columns, got 5",
        ),
        (
            """3 4\n@ . a .\nb @ c\n. d . .\n""",
            "Expected row to have 4 columns, got 3",
        ),
        (
            """3 4\n@ . a .\nb @ c #\n. d . .\n""",
            "Unknown dynamic env symbol #. Must be one of '@', '.' or is a letter(s) in the alphabet",
        ),
        (
            """3 4\n@ . a .\nb @ c 7\n. d . .\n""",
            "Unknown dynamic env symbol 7. Must be one of '@', '.' or is a letter(s) in the alphabet",
        ),
        (
            """3 4\n@ . a .\nb @ c a\n. d . .\n""",
            "Dynamic obstacle a can not be placed more than once on the map",
        ),
    ],
    ids=[
        "mismatch_columns_1",
        "mismatch_columns_2",
        "Unknown_dynamic_env_symbol",
        "Unknown_dynamic_env_numerical_symbol",
        "duplicate_dynamic_obstacle",
    ],
)
def test_import_dynamic_env_fail(map: str, error_text: str):
    with patch("builtins.open", mock_open(read_data=map)):
        with patch.object(Path, "is_file", return_value=True):
            with pytest.raises(BaseException, match=re.escape(error_text)):
                import_dynamic_env(map)

import io
import re
from pathlib import Path
from typing import Set
from unittest.mock import MagicMock, mock_open, patch

import pytest

from cbs_sipp.map.dynamic_env import (
    DeterministicMovement,
    import_dynamic_env_instance,
    import_dynamic_env_map,
)
from cbs_sipp.map.dynamic_env import TrajectoryMovement as M

MAP_INSTANCES_PATH = "instances/dynamic/maps"
INSTANCES_PATH = "instances/dynamic/custom_instances"

# ----- TESTS FOR IMPORTING DYNAMIC ENV MAP ----- #


def test_import_dynamic_env_map_success():
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

    assert expected_map, expected_dynamic_obstacles == import_dynamic_env_map(
        f"{MAP_INSTANCES_PATH}/{file}"
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
def test_import_dynamic_env_map_fail(map: str, error_text: str):
    with patch("builtins.open", mock_open(read_data=map)):
        with patch.object(Path, "is_file", return_value=True):
            with pytest.raises(BaseException, match=re.escape(error_text)):
                import_dynamic_env_map("mock.txt")


# ----- TESTS FOR IMPORTING DYNAMIC ENV INSTANCE ----- #


def test_import_dynamic_env_instance_success():
    file = "8-8-simple-deterministic-only.toml"
    dynamic_obstacles = {"a", "b", "cc"}

    expected_obstacle_data = {
        "a": DeterministicMovement(
            "a", 4, True, True, [M.DOWN, M.DOWN, M.DOWN]
        ),
        "b": DeterministicMovement(
            "b",
            0,
            False,
            False,
            [
                M.DOWN,
                M.DOWN,
                M.DOWN,
                M.DOWN,
                M.DOWN,
                M.WAIT,
                M.LEFT,
                M.UP,
                M.UP,
                M.UP,
                M.UP,
                M.UP,
            ],
        ),
        "cc": DeterministicMovement(
            "cc",
            2,
            False,
            True,
            [
                M.LEFT,
                M.LEFT,
                M.WAIT,
                M.DOWN,
                M.DOWN,
                M.WAIT,
                M.RIGHT,
                M.RIGHT,
                M.WAIT,
                M.UP,
                M.UP,
            ],
        ),
    }

    assert expected_obstacle_data == import_dynamic_env_instance(
        f"{INSTANCES_PATH}/{file}", dynamic_obstacles
    )


def test_import_dynamic_env_instance_fail_missing_key():
    mock_path = MagicMock(spec=Path)
    mock_path.is_file.return_value = True
    mock_path.suffix = ".toml"

    data = [
        "[[dynamic_obstacle]]",
        "id = 'a'",
        "type = 'deterministic'",
        "start_t = 4",
        "hide_before_start = true",
        "disappear_after_end = true",
        "trajectory = ['D', 'D', 'D']",
    ]
    missing_keys = [
        "id",
        "type",
        "start_t",
        "hide_before_start",
        "disappear_after_end",
        "trajectory",
    ]

    for i in range(1, len(data)):
        # build the dynamic obstacle data, but drop a key
        missing_data = data[0:i] + data[i + 1 : len(data)]
        missing_error = (
            f"Missing key '{missing_keys[i - 1]}' in dynamic obstacle[0]"
        )

        # mock the TOML file
        file = io.BytesIO("\n".join(missing_data).encode("utf-8"))

        def mock_file(*args, **kwargs):
            file.seek(0)
            return file

        with patch("cbs_sipp.map.dynamic_env.Path", return_value=mock_path):
            with patch("builtins.open", mock_file):
                with pytest.raises(KeyError, match=re.escape(missing_error)):
                    import_dynamic_env_instance("mock.toml", {"a"})


def test_import_dynamic_env_instance_fail_wrong_types():
    mock_path = MagicMock(spec=Path)
    mock_path.is_file.return_value = True
    mock_path.suffix = ".toml"

    data = [
        "[[dynamic_obstacle]]",
        "id = 'a'",
        "type = 'deterministic'",
        "start_t = 4",
        "hide_before_start = true",
        "disappear_after_end = true",
        "trajectory = ['D', 'D', 'D']",
    ]
    keys = [
        "id",
        "type",
        "start_t",
        "hide_before_start",
        "disappear_after_end",
        "trajectory",
    ]
    wrong_data_type = [
        "id = 1",
        "type = [1, 2, 4]",
        "start_t = '4'",
        "hide_before_start = 1",
        "disappear_after_end = 0",
        "trajectory = 'UP'",
    ]
    expected_types = [str, str, int, bool, bool, list]
    actual_types = [int, list, str, int, int, str]

    for i in range(1, len(data)):
        # build the dynamic obstacle data, but with a incorrect data type
        missing_data = (
            data[0:i] + [wrong_data_type[i - 1]] + data[i + 1 : len(data)]
        )
        missing_error = f"'{keys[i - 1]}' is expected to be type {expected_types[i - 1]}; got type {actual_types[i - 1]}"

        # mock the TOML file
        file = io.BytesIO("\n".join(missing_data).encode("utf-8"))

        def mock_file(*args, **kwargs):
            file.seek(0)
            return file

        with patch("cbs_sipp.map.dynamic_env.Path", return_value=mock_path):
            with patch("builtins.open", mock_file):
                with pytest.raises(TypeError, match=re.escape(missing_error)):
                    import_dynamic_env_instance("mock.toml", {"a"})


@pytest.mark.parametrize(
    "data, error_msg, obstacle_ids",
    [
        (
            """
            [[dynamic_obstacle]]
            id = 'unknown'
            type = 'deterministic'
            start_t = 4
            hide_before_start = true
            disappear_after_end = true
            trajectory = ['D', 'D', 'D']
            """,
            "Obstacle unknown does not exist in the given dynamic map environment",
            {"a"},
        ),
        (
            """
            [[dynamic_obstacle]]
            id = 'a'
            type = 'unknown'
            start_t = 4
            hide_before_start = true
            disappear_after_end = true
            trajectory = ['D', 'D', 'D']
            """,
            "Unknown movement type 'unknown'",
            {"a"},
        ),
        (
            """
            [[dynamic_obstacle]]
            id = 'a'
            type = 'deterministic'
            start_t = 4
            hide_before_start = true
            disappear_after_end = true
            trajectory = ['D', 'D', 'D']
            """,
            "Missing obstacle data for {'A'}",
            {"a", "A"},
        ),
        (
            """
            [[dynamic_obstacle]]
            id = 'a'
            type = 'deterministic'
            start_t = 4
            hide_before_start = true
            disappear_after_end = true
            trajectory = ['D', 1, 'D']
            """,
            "Trajectory must only contain strings {'L', 'R', 'U', 'D', 'W'}; got 1",
            {"a"},
        ),
        (
            """
            [[dynamic_obstacle]]
            id = 'a'
            type = 'deterministic'
            start_t = 4
            hide_before_start = true
            disappear_after_end = true
            trajectory = ['D', 'J', 'D']
            """,
            "Trajectory must only contain strings {'L', 'R', 'U', 'D', 'W'}; got J",
            {"a"},
        ),
    ],
    ids=[
        "unknown_dynamic_obstacle_id",
        "unknown_movement_type",
        "missing_obstacle_data",
        "invalid_trajectory_type",
        "invalid_trajectory_value",
    ],
)
def test_import_dynamic_env_instance_fail_general_errors(
    data: str, error_msg: str, obstacle_ids: Set[str]
):
    mock_path = MagicMock(spec=Path)
    mock_path.is_file.return_value = True
    mock_path.suffix = ".toml"

    # mock the TOML file
    file = io.BytesIO(data.encode("utf-8"))

    def mock_file(*args, **kwargs):
        file.seek(0)
        return file

    with patch("cbs_sipp.map.dynamic_env.Path", return_value=mock_path):
        with patch("builtins.open", mock_file):
            with pytest.raises(
                (ValueError, TypeError), match=re.escape(error_msg)
            ):
                import_dynamic_env_instance("mock.toml", obstacle_ids)


# def test_import_dynamic_env_instance_fail_deterministic_movement():

import io
import re
from pathlib import Path
from string import Template
from unittest.mock import MagicMock, patch

import pytest

from cbs_sipp.map.dynamic_env import (
    DynamicObstacle,
    Point,
    Trajectory,
    import_dynamic_env_instance,
)

MAP_INSTANCES_PATH = "instances/maps/custom_instances"
INSTANCES_PATH = "instances/dynamic_instances/custom_instances"


@pytest.fixture
def mock_path() -> MagicMock:
    mock_path = MagicMock(spec=Path)
    mock_path.is_file.return_value = True
    mock_path.suffix = ".toml"
    return mock_path


class TestImportDynamicEnvInstance:
    def test_success(self):
        file = "8-8-simple-1.toml"

        a = DynamicObstacle("a")
        b = DynamicObstacle("b")
        c = DynamicObstacle("c")

        a.add_trajectory(
            Trajectory(
                [Point(1, 1, 0, 1), Point(5, 1, 5, 0.5), Point(5, 3, 10, 0.1)]
            )
        )
        a.add_trajectory(
            Trajectory(
                [Point(1, 1, 0, 1), Point(4, 1, 5, 0.5), Point(4, 3, 10, 0.1)]
            )
        )

        b.add_trajectory(
            Trajectory(
                [Point(2, 5, 0, 1), Point(3, 5, 3, 0.7), Point(5, 3, 15, 0.2)]
            )
        )
        b.add_trajectory(
            Trajectory(
                [Point(2, 5, 0, 1), Point(2, 5, 3, 0.6), Point(4, 3, 15, 0.9)]
            )
        )

        c.add_trajectory(
            Trajectory(
                [Point(6, 6, 0, 1), Point(4, 5, 3, 0.3), Point(5, 3, 7, 0.1)]
            )
        )
        c.add_trajectory(
            Trajectory(
                [Point(6, 6, 0, 1), Point(4, 1, 5, 0.5), Point(4, 3, 10, 0.9)]
            )
        )

        expected_results = {"a": a, "b": b, "c": c}

        assert expected_results == import_dynamic_env_instance(
            f"{INSTANCES_PATH}/{file}"
        )

    def test_missing_id_or_start_point_key(self, mock_path):
        data = "[[dynamic_obstacles]]"
        id_str = "id='a'"
        start_str = "start_point_ = {x = 1, y = 1}"

        def mock_file(*args, **kwargs):
            file.seek(0)
            return file

        id_err = "Missing key 'id' in dynamic obstacle[0]"
        start_err = "Missing key 'start_point' in dynamic obstacle[0]"

        with patch("cbs_sipp.map.dynamic_env.Path", return_value=mock_path):
            # mock the TOML file
            toml_data = "\n".join([data, start_str])
            file = io.BytesIO(toml_data.encode("utf-8"))
            with patch("builtins.open", mock_file):
                with pytest.raises(KeyError, match=re.escape(id_err)):
                    import_dynamic_env_instance("mock.toml")

            toml_data = "\n".join([data, id_str])
            file = io.BytesIO(toml_data.encode("utf-8"))
            with patch("builtins.open", mock_file):
                with pytest.raises(KeyError, match=re.escape(start_err)):
                    import_dynamic_env_instance("mock.toml")

    def test_duplicate_obstacle(self, mock_path):
        data = """
            [[dynamic_obstacles]]
            id = "a"
            start_point = {x = 1, y = 1}

            [[dynamic_obstacles.trajectories]]
            points = [
                {x = 5, y = 3, t = 10, p = 0.1},
            ]

            [[dynamic_obstacles]]
            id = "a"
            start_point = {x = 2, y = 5}

            [[dynamic_obstacles.trajectories]]
            points = [
                {x = 5, y = 3, t = 15, p = 0.2},
            ]
        """

        def mock_file(*args, **kwargs):
            file.seek(0)
            return file

        with patch("cbs_sipp.map.dynamic_env.Path", return_value=mock_path):
            file = io.BytesIO(data.encode("utf-8"))
            err = "Obstacle 'a' has already been defined in the dynamic environment instance"
            with patch("builtins.open", mock_file):
                with pytest.raises(ValueError, match=re.escape(err)):
                    import_dynamic_env_instance("mock.toml")

    @pytest.mark.parametrize(
        "bad_point, err",
        [
            (
                "start_point = {y = 1}",
                "Missing key 'x' in dynamic obstacle[0]",
            ),
            (
                "start_point = {x = 1}",
                "Missing key 'y' in dynamic obstacle[0]",
            ),
            (
                "start_point = {x = '1'}",
                "'x' is expected to be type <class 'int'>; got type <class 'str'>",
            ),
            (
                "start_point = {x = 1, y = false}",
                "'y' is expected to be type <class 'int'>; got type <class 'bool'>",
            ),
        ],
    )
    def test_invalid_start_point(self, mock_path, bad_point, err):
        data = """
            [[dynamic_obstacles]]
            id = "a"
        """

        def mock_file(*args, **kwargs):
            file.seek(0)
            return file

        with patch("cbs_sipp.map.dynamic_env.Path", return_value=mock_path):
            toml_data = "\n".join([data, bad_point])
            file = io.BytesIO(toml_data.encode("utf-8"))
            with patch("builtins.open", mock_file):
                with pytest.raises(
                    (KeyError, TypeError), match=re.escape(err)
                ):
                    import_dynamic_env_instance("mock.toml")

    @pytest.mark.parametrize(
        "bad_point, err",
        [
            ("{y = 2}", "Missing key 'x' in dynamic obstacle[0]"),
            ("{x = 2}", "Missing key 'y' in dynamic obstacle[0]"),
            ("{x = 2, y = 2}", "Missing key 't' in dynamic obstacle[0]"),
            (
                "{x = 1, y = 2, t = 2}",
                "Missing key 'p' in dynamic obstacle[0]",
            ),
            (
                "{x = '2'}",
                "'x' is expected to be type <class 'int'>; got type <class 'str'>",
            ),
            (
                "{x = 2, y = false}",
                "'y' is expected to be type <class 'int'>; got type <class 'bool'>",
            ),
            (
                "{x = 2, y = 2, t = {}}",
                "'t' is expected to be type <class 'int'>; got type <class 'dict'>",
            ),
            (
                "{x = 2, y = 2, t = 2, p = []}",
                "'p' is expected to be type <class 'float'>; got type <class 'list'>",
            ),
            ("{x = -1, y = 2, t = 2, p = 0.5}", "x position must be positive"),
            ("{x = 2, y = -2, t = 2, p = 0.5}", "y position must be positive"),
            ("{x = 2, y = 2, t = -1, p = 0.5}", "timestep t must be positive"),
            (
                "{x = 2, y = 2, t = 2, p = -0.00001}",
                "Uncertainty value must be a float in the range [0, 1.0]",
            ),
            (
                "{x = 2, y = 2, t = 2, p = 1.00001}",
                "Uncertainty value must be a float in the range [0, 1.0]",
            ),
        ],
    )
    def test_bad_points(self, mock_path, bad_point, err):
        data = Template("""
            [[dynamic_obstacles]]
            id = "a"
            start_point = {x = 1, y = 1}

            [[dynamic_obstacles.trajectories]]
            points = [$point]
        """)

        def mock_file(*args, **kwargs):
            file.seek(0)
            return file

        with patch("cbs_sipp.map.dynamic_env.Path", return_value=mock_path):
            toml_data = data.substitute(point=bad_point)
            file = io.BytesIO(toml_data.encode("utf-8"))
            with patch("builtins.open", mock_file):
                with pytest.raises(
                    (KeyError, TypeError, ValueError), match=re.escape(err)
                ):
                    import_dynamic_env_instance("mock.toml")

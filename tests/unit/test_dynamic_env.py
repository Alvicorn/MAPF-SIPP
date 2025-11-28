import io
import re
from string import Template
from unittest.mock import patch

import pytest

from cbs_sipp.map.dynamic_env import (
    DynamicObstacle,
    Point,
    Trajectory,
    import_dynamic_env_instance,
)

MAP_INSTANCES_PATH = "instances/maps/custom_instances"
INSTANCES_PATH = "instances/dynamic_instances/custom_instances"


class TestImportDynamicEnvInstance:
    def test_success(self, grid_map):
        file = "8-8-simple-1.toml"

        a = DynamicObstacle("a")
        b = DynamicObstacle("b")
        c = DynamicObstacle("c")

        a.add_trajectory(
            Trajectory(
                [Point(1, 1, 0, 1), Point(5, 1, 5, 0.5), Point(5, 3, 10, 0.1)],
                grid_map,
            )
        )
        a.add_trajectory(
            Trajectory(
                [Point(1, 1, 0, 1), Point(4, 1, 5, 0.5), Point(4, 3, 10, 0.1)],
                grid_map,
            )
        )

        b.add_trajectory(
            Trajectory(
                [Point(2, 5, 0, 1), Point(3, 5, 3, 0.7), Point(5, 3, 15, 0.2)],
                grid_map,
            )
        )
        b.add_trajectory(
            Trajectory(
                [Point(2, 5, 0, 1), Point(2, 5, 3, 0.6), Point(4, 3, 15, 0.9)],
                grid_map,
            )
        )

        c.add_trajectory(
            Trajectory(
                [Point(4, 4, 0, 1), Point(4, 5, 3, 0.3), Point(5, 3, 7, 0.1)],
                grid_map,
            )
        )
        c.add_trajectory(
            Trajectory(
                [Point(4, 4, 0, 1), Point(4, 1, 5, 0.5), Point(4, 3, 10, 0.9)],
                grid_map,
            )
        )

        expected_results = {"a": a, "b": b, "c": c}

        assert expected_results == import_dynamic_env_instance(
            f"{INSTANCES_PATH}/{file}", grid_map
        )

    def test_missing_id_or_start_point_key(self, mock_path, grid_map):
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
                    import_dynamic_env_instance("mock.toml", grid_map)

            toml_data = "\n".join([data, id_str])
            file = io.BytesIO(toml_data.encode("utf-8"))
            with patch("builtins.open", mock_file):
                with pytest.raises(KeyError, match=re.escape(start_err)):
                    import_dynamic_env_instance("mock.toml", grid_map)

    def test_duplicate_obstacle(self, mock_path, grid_map):
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
                    import_dynamic_env_instance("mock.toml", grid_map)

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
            (
                "start_point = {x = 0, y = 2}",
                "Initial start location for obstacle a is impeded by a static barrier",
            ),
        ],
    )
    def test_invalid_start_point(self, mock_path, grid_map, bad_point, err):
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
                    (KeyError, TypeError, ValueError), match=re.escape(err)
                ):
                    import_dynamic_env_instance("mock.toml", grid_map)

    @pytest.mark.parametrize(
        "bad_point, err",
        [
            ("{y = 4}", "Missing key 'x' in dynamic obstacle[0]"),
            ("{x = 4}", "Missing key 'y' in dynamic obstacle[0]"),
            ("{x = 4, y = 4}", "Missing key 't' in dynamic obstacle[0]"),
            (
                "{x = 4, y = 4, t = 4}",
                "Missing key 'p' in dynamic obstacle[0]",
            ),
            (
                "{x = '4'}",
                "'x' is expected to be type <class 'int'>; got type <class 'str'>",
            ),
            (
                "{x = 4, y = false}",
                "'y' is expected to be type <class 'int'>; got type <class 'bool'>",
            ),
            (
                "{x = 4, y = 4, t = {}}",
                "'t' is expected to be type <class 'int'>; got type <class 'dict'>",
            ),
            (
                "{x = 4, y = 4, t = 4, p = []}",
                "'p' is expected to be type <class 'float'>; got type <class 'list'>",
            ),
            (
                "{x = -1, y = 4, t = 4, p = 0.5}",
                "Initial start location for obstacle a is impeded by a static barrier",
            ),
            (
                "{x = 4, y = -1, t = 4, p = 0.5}",
                "Initial start location for obstacle a is impeded by a static barrier",
            ),
            ("{x = 4, y = 4, t = -1, p = 0.5}", "timestep t must be positive"),
            (
                "{x = 4, y = 4, t = 4, p = -0.00001}",
                "Uncertainty value must be a float in the range [0, 1.0]",
            ),
            (
                "{x = 4, y = 4, t = 4, p = 1.00001}",
                "Uncertainty value must be a float in the range [0, 1.0]",
            ),
            (
                "{x = 0, y = 2, t = 5, p = 0.5}",
                "Initial start location for obstacle a is impeded by a static barrier",
            ),
        ],
    )
    def test_bad_points(self, mock_path, grid_map, bad_point, err):
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
                    import_dynamic_env_instance("mock.toml", grid_map)


class TestDynamicObstacle:
    def test_get_possible_states_at_time(self, grid_map):
        a = DynamicObstacle("a")
        start = Point(1, 1, 0, 1)
        a.add_trajectory(
            Trajectory(
                [start, Point(5, 1, 5, 0.5), Point(7, 7, 10, 0.1)], grid_map
            )
        )
        a.add_trajectory(
            Trajectory(
                [start, Point(4, 3, 5, 0.5), Point(4, 3, 10, 0.1)], grid_map
            )
        )

        assert [(1, 1), (1, 1)] == a.get_possible_states_at_time(-1)
        assert [(1, 1), (1, 1)] == a.get_possible_states_at_time(0)
        assert [(5, 1), (4, 3)] == a.get_possible_states_at_time(5)
        assert [(7, 7), (4, 3)] == a.get_possible_states_at_time(10)
        assert [(7, 7), (4, 3)] == a.get_possible_states_at_time(11)
        assert [(5, 2), (4, 3)] == a.get_possible_states_at_time(7)

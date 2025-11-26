from __future__ import annotations

import tomllib
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List


@dataclass(frozen=True, slots=True)
class Point:
    x: int
    y: int
    t: int  # timestamp
    p: float  # uncertainty

    def __post_init__(self):
        if not (0 <= self.p <= 1.0):
            raise ValueError(
                "Uncertainty value must be a float in the range [0, 1.0]"
            )
        if self.x < 0:
            raise ValueError("x position must be positive")
        if self.y < 0:
            raise ValueError("y position must be positive")
        if self.t < 0:
            raise ValueError("timestep t must be positive")


@dataclass(slots=True)
class Trajectory:
    points: List[Point]

    def __post_init__(self):
        self.points.sort(key=lambda p: p.t)


@dataclass(slots=True)
class DynamicObstacle:
    id: str
    trajectories: List[Trajectory] = field(init=False, default_factory=list)

    def add_trajectory(self, trajectory: Trajectory) -> None:
        self.trajectories.append(trajectory)


def import_dynamic_env_instance(file: str) -> Dict[str, DynamicObstacle]:
    """
    Import the dynamic instance data from a TOML file.

    Args:
        file (str): TOML file with MAPF dynamic environment instance data.

    Returns:
        Dict[str, DynamicObstacle]: Map between dynamic obstacle names and its data.
    """
    f = Path(file)
    if not f.is_file():
        raise FileNotFoundError(file + " does not exist.")
    if not f.suffix == ".toml":
        raise ValueError(
            "The dynamic environment instance file must be a .toml file"
        )

    obstacle_data = {}

    with open(file, "rb") as f:
        instance_data = tomllib.load(f)

    # TODO: load agents

    # load the obstacles
    for i, data in enumerate(instance_data["dynamic_obstacles"]):
        id = _get_obstacle_data_str(data, "id", i)
        if id in obstacle_data:
            raise ValueError(
                f"Obstacle '{id}' has already been defined in the dynamic environment instance"
            )

        start_point = _get_obstacle_data_dict(data, "start_point", i)
        x_start = _get_obstacle_data_int(start_point, "x", i)
        y_start = _get_obstacle_data_int(start_point, "y", i)

        obstacle = DynamicObstacle(id)

        for trajectory in _get_obstacle_data_list(data, "trajectories", i):
            points = [Point(x_start, y_start, 0, 1)]
            for point in _get_obstacle_data_list(trajectory, "points", i):
                x = _get_obstacle_data_int(point, "x", i)
                y = _get_obstacle_data_int(point, "y", i)
                t = _get_obstacle_data_int(point, "t", i)
                p = _get_obstacle_data_float(point, "p", i)
                points.append(Point(x, y, t, p))

            obstacle.add_trajectory(Trajectory(points))

        obstacle_data[id] = obstacle

    return obstacle_data


# ----- HELPER FUNCTIONS FOR LOADING OBSTACLE DATA  ----- #


def _get_obstacle_data_str(data: Dict[str, Any], key: str, index: int) -> str:
    d = _get_obstacle_data(data, key, index)
    if not isinstance(d, str):
        raise TypeError(
            f"'{key}' is expected to be type {str}; got type {type(d)}"
        )
    return d.strip()


def _get_obstacle_data_int(data: Dict[str, Any], key: str, index: int) -> int:
    d = _get_obstacle_data(data, key, index)
    if type(d) is not int:
        raise TypeError(
            f"'{key}' is expected to be type {int}; got type {type(d)}"
        )
    return d


def _get_obstacle_data_float(
    data: Dict[str, Any], key: str, index: int
) -> float:
    d = _get_obstacle_data(data, key, index)
    if not isinstance(d, float):
        raise TypeError(
            f"'{key}' is expected to be type {float}; got type {type(d)}"
        )
    return d


def _get_obstacle_data_list(
    data: Dict[str, Any], key: str, index: int
) -> List[Any]:
    d = _get_obstacle_data(data, key, index)
    if not isinstance(d, list):
        raise TypeError(
            f"'{key}' is expected to be type {list}; got type {type(d)}"
        )
    return d


def _get_obstacle_data_dict(
    data: Dict[str, Any], key: str, index: int
) -> Dict[str, Any]:
    d = _get_obstacle_data(data, key, index)
    if not isinstance(d, dict):
        raise TypeError(
            f"'{key}' is expected to be type {dict}; got type {type(d)}"
        )
    return d


def _get_obstacle_data(data: Dict[str, Any], key: str, index: int) -> Any:
    if key not in data:
        raise KeyError(f"Missing key '{key}' in dynamic obstacle[{index}]")
    return data[key]

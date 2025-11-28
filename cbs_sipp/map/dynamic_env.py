from __future__ import annotations

import random
import tomllib
from bisect import bisect_left
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Tuple

from cbs_sipp.map.grid_map import GridMap
from cbs_sipp.path_utils import Path as GridPath
from cbs_sipp.path_utils import Vertex


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

    @property
    def loc(self) -> Vertex:
        return (self.x, self.y)


@dataclass(frozen=True, slots=True)
class Trajectory:
    points: Dict[int, Point] = field(init=False)
    timestamps: List[int] = field(init=False)
    min_t: int = field(init=False)
    max_t: int = field(init=False)
    sub_paths: Dict[Tuple[Point, Point], GridPath] = field(init=False)

    path: GridPath = field(init=False)

    def __init__(self, points: List[Point], grid_map: GridMap):
        object.__setattr__(self, "points", {p.t: p for p in points})
        object.__setattr__(
            self, "timestamps", sorted(list(self.points.keys()))
        )
        object.__setattr__(self, "min_t", self.timestamps[0])
        object.__setattr__(self, "max_t", self.timestamps[-1])

        # for every pair of timestamps, compute the shortest path
        sub_paths = {}
        path = []

        for i in range(1, len(self.timestamps)):
            t0, t1 = self.timestamps[i - 1], self.timestamps[i]
            p0, p1 = self.points[t0], self.points[t1]
            sub_path, _ = grid_map.shortest_path(p0.loc, p1.loc)
            sub_paths[(p0, p1)] = sub_path

            if i == 1:
                path = sub_path
            else:
                if path[-1] != sub_path[0]:  # sanity check
                    raise ValueError("Sub-paths do not align")
                path += sub_path[1:]

        object.__setattr__(self, "sub_paths", sub_paths)
        object.__setattr__(self, "path", path)

    def get_state_at_time(self, t: int) -> Vertex:
        if t in self.points:
            p = self.points[t]
            return p.loc

        if t <= self.min_t:
            p = self.points[self.min_t]
            return p.loc

        if t >= self.max_t:
            p = self.points[self.max_t]
            return p.loc

        t1_index = bisect_left(self.timestamps, t)

        t0 = self.timestamps[t1_index - 1]
        t1 = self.timestamps[t1_index]

        p0 = self.points[t0]
        p1 = self.points[t1]

        path = self.sub_paths[(p0, p1)]

        # select a random point on the this sub path
        pos = random.randint(0, len(path) - 1)
        return path[pos]


@dataclass(slots=True)
class DynamicObstacle:
    id: str
    trajectories: List[Trajectory] = field(init=False, default_factory=list)

    def add_trajectory(self, trajectory: Trajectory) -> None:
        self.trajectories.append(trajectory)

    @property
    def possible_trajectory_paths(self) -> List[GridPath]:
        return [traj.path for traj in self.trajectories]

    def get_possible_states_at_time(self, t: int) -> List[Tuple[int, int]]:
        return [traj.get_state_at_time(t) for traj in self.trajectories]


def import_dynamic_env_instance(
    file: str, grid_map: GridMap
) -> Dict[str, DynamicObstacle]:
    """
    Import the dynamic instance data from a TOML file.

    Args:
        file (str): TOML file with MAPF dynamic environment instance data.
        grid_map (GridMap): Map of the static environment (e.g. barriers).
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

        if not grid_map.is_free((x_start, y_start)):
            raise ValueError(
                f"Initial start location for obstacle {id} is impeded by a static barrier"
            )

        obstacle = DynamicObstacle(id)

        for trajectory in _get_obstacle_data_list(data, "trajectories", i):
            points = [Point(x_start, y_start, 0, 1)]
            for point in _get_obstacle_data_list(trajectory, "points", i):
                x = _get_obstacle_data_int(point, "x", i)
                y = _get_obstacle_data_int(point, "y", i)
                if not grid_map.is_free((x, y)):
                    raise ValueError(
                        f"Initial start location for obstacle {id} is impeded by a static barrier"
                    )

                t = _get_obstacle_data_int(point, "t", i)
                p = _get_obstacle_data_float(point, "p", i)
                points.append(Point(x, y, t, p))

            obstacle.add_trajectory(Trajectory(points, grid_map))

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

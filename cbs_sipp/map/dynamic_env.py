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
class Agent:
    id: int
    start: Vertex
    goal: Vertex


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
) -> Tuple[Dict[int, Agent], Dict[str, DynamicObstacle]]:
    """
    Import the dynamic instance data from a TOML file.

    Args:
        file (str): TOML file with MAPF dynamic environment instance data.
        grid_map (GridMap): Map of the static environment (e.g. barriers).
    Returns:
        Tuple[Dict[int, Agent], Dict[str, DynamicObstacle]]: Parsed Agent and Dynamic obstacle objects.
    """
    f = Path(file)
    if not f.is_file():
        raise FileNotFoundError(file + " does not exist.")
    if not f.suffix == ".toml":
        raise ValueError(
            "The dynamic environment instance file must be a .toml file"
        )

    with open(file, "rb") as f:
        instance_data = tomllib.load(f)

    if "agents" not in instance_data:
        raise KeyError(f"Missing key 'agents' in {f}")
    if "dynamic_obstacles" not in instance_data:
        raise KeyError(f"Missing key 'dynamic_obstacles' in {f}")

    return _import_agent_data(
        instance_data["agents"], grid_map
    ), _import_obstacle_data(instance_data["dynamic_obstacles"], grid_map)


def _import_agent_data(
    data: List[Dict[str, Any]], grid_map: GridMap
) -> Dict[int, Agent]:
    """
    Parse data into Agent objects.

    Args:
        data (List[Dict[str, Any]]): Imported agent data about the MAPF instance.
        grid_map (GridMap): Map of the MAPF instance.

    Returns:
        Dict[int, Agent]: Map of agent ids and agent data.
    """
    agents = {}
    for i, agent_data in enumerate(data):
        id = _get_obstacle_data_int(agent_data, "id", i)
        if id in agents:
            raise ValueError(
                f"Agent '{id}' has already been defined in the dynamic environment instance"
            )

        start_point = _get_point_data(
            grid_map, agent_data, "start_point", i, id
        )
        goal_point = _get_point_data(grid_map, agent_data, "goal_point", i, id)

        agents[id] = Agent(id, start_point, goal_point)

    return agents


def _import_obstacle_data(
    data: List[Dict[str, Any]], grid_map: GridMap
) -> Dict[str, DynamicObstacle]:
    """
    Parse data into Obstacle objects.

    Args:
        data (List[Dict[str, Any]]): Imported obstacle data about the MAPF instance.
        grid_map (GridMap): Map of the MAPF instance.

    Returns:
        Dict[str, Agent]: Map of obstacle ids and obstacle data.
    """
    obstacles = {}
    for i, obstacle_data in enumerate(data):
        id = _get_obstacle_data_str(obstacle_data, "id", i)
        if id in obstacles:
            raise ValueError(
                f"Obstacle '{id}' has already been defined in the dynamic environment instance"
            )

        x_start, y_start = _get_point_data(
            grid_map, obstacle_data, "start_point", i, id, is_agent=False
        )

        obstacle = DynamicObstacle(id)

        for trajectory in _get_obstacle_data_list(
            obstacle_data, "trajectories", i
        ):
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

        obstacles[id] = obstacle

    return obstacles


# ----- HELPER FUNCTIONS FOR LOADING TOML DATA  ----- #


def _get_point_data(
    grid_map: GridMap,
    data: Dict[str, Any],
    point_name: str,
    index: int,
    id: int | str,
    is_agent: bool = True,
) -> Vertex:
    """
    Extract point information and validate it.

    Args:
        grid_map (GridMap): Map of the MAPF instance
        data (Dict[str, Any]): MAPF instance data.
        point_name (str): Name of the point in the data dictionary.
        index (int): Index of the target point.
        id (int | str): ID of the object that is trying to parse to point.
        is_obstacle (bool): Object that is trying to parse the point is an agent. Defaults to True.

    Returns:
        Vertex: Parsed point as a Vertex object.
    """
    point = _get_obstacle_data_dict(data, point_name, index)
    x = _get_obstacle_data_int(point, "x", index)
    y = _get_obstacle_data_int(point, "y", index)
    if not grid_map.is_free((x, y)):
        dynamic_object = "agent" if is_agent else "obstacle"
        raise ValueError(
            f"{point_name} for {dynamic_object} '{id}' is impeded by a static barrier"
        )
    return (x, y)


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

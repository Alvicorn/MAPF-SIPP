from __future__ import annotations

import tomllib
from abc import ABC
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from string import Template
from typing import Any, Dict, List, Set, Tuple


class TrajectoryMovement(Enum):
    LEFT = "L"
    RIGHT = "R"
    UP = "U"
    DOWN = "D"
    WAIT = "W"

    @classmethod
    def parse_list(cls, raw_trajectory: List[str]) -> List[TrajectoryMovement]:
        valid_moves = {m.value for m in TrajectoryMovement}
        trajectory = []

        error_msg = Template(
            "Trajectory must only contain strings {'L', 'R', 'U', 'D', 'W'}; got $move"
        )
        for move in raw_trajectory:
            if not isinstance(move, str):
                raise TypeError(error_msg.substitute(move=move))

            move_key = move.strip().upper()
            if move_key not in valid_moves:
                raise ValueError(error_msg.substitute(move=move))

            trajectory.append(TrajectoryMovement(move_key))

        return trajectory


@dataclass
class DynamicObstacle(ABC):
    id: str
    start_t: int


@dataclass
class DeterministicMovement(DynamicObstacle):
    hide_before_start: bool
    disappear_after_end: bool
    trajectory: List[TrajectoryMovement]


def import_dynamic_env_map(file: str) -> Tuple[List[List[str]], Set[str]]:
    """
    Parse a dynamic environment into a nested list of strings and
    extract the unique set of dynamic obstacle names.

    Args:
        file (str): Path to the dynamic environment file.

    Returns:
        Tuple[List[List[str]], Set[str]]: The dynamic environment as a nested list and
                                          the set of dynamic obstacle names
    """
    f = Path(file)
    if not f.is_file():
        raise FileNotFoundError(file + " does not exist.")

    with open(file, "r") as f:
        # first line: #rows #columns
        line = f.readline()
        rows, columns = [int(x) for x in line.split(" ")]

        # #rows lines with the map
        my_map = []
        dynamic_obstacles = set()
        for _ in range(rows):
            line = f.readline().strip().split(" ")
            if len(line) != columns:
                raise BaseException(
                    f"Expected row to have {columns} columns, got {len(line)}"
                )
            my_map.append([])
            for cell in line:
                if cell == "@" or cell == "." or cell.isalpha():
                    if cell in dynamic_obstacles:
                        raise BaseException(
                            f"Dynamic obstacle {cell} can not be placed more than once on the map"
                        )
                    if cell.isalpha():
                        dynamic_obstacles.add(cell)

                    my_map[-1].append(cell)
                else:
                    raise BaseException(
                        f"Unknown dynamic env symbol {cell}. Must be one of '@', '.' or is a letter(s) in the alphabet"
                    )

    return my_map, dynamic_obstacles


def import_dynamic_env_instance(
    file: str, obstacle_names: Set[str]
) -> Dict[str, DynamicObstacle]:
    """
    Import the dynamic instance data from a TOML file.

    Args:
        file (str): TOML file with MAPF dynamic environment instance data.
        obstacle_names (Set[str]): Names of all the obstacles found in the MAPF map.

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

    # load the obstacles
    for i, data in enumerate(instance_data["dynamic_obstacle"]):
        id = _get_obstacle_data_str(data, "id", i)
        if id not in obstacle_names:
            raise ValueError(
                f"Obstacle {id} does not exist in the given dynamic map environment"
            )

        movement_type = _get_obstacle_data_str(data, "type", i)
        start_t = _get_obstacle_data_int(data, "start_t", i)

        match movement_type.lower():
            case "deterministic":
                obstacle_data[id] = _load_obstacle_with_deterministic_movement(
                    id, start_t, data, i
                )
            case _:
                raise ValueError(f"Unknown movement type '{movement_type}'")

    missing_obstacle_data = obstacle_names - set(obstacle_data.keys())
    if len(missing_obstacle_data) > 0:
        raise ValueError(f"Missing obstacle data for {missing_obstacle_data}")

    # TODO: load agents

    return obstacle_data


# ----- HELPER FUNCTIONS FOR GENERATION OBSTACLE MOVEMENT OBJECT  ----- #


def _load_obstacle_with_deterministic_movement(
    id: str, start_t: int, data: Dict[str, Any], index: int
) -> DeterministicMovement:
    hide_before_start = _get_obstacle_data_bool(
        data, "hide_before_start", index
    )
    disappear_after_end = _get_obstacle_data_bool(
        data, "disappear_after_end", index
    )

    raw_trajectory = _get_obstacle_data_list(data, "trajectory", index)
    trajectory = TrajectoryMovement.parse_list(raw_trajectory)

    return DeterministicMovement(
        id, start_t, hide_before_start, disappear_after_end, trajectory
    )


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
    if not isinstance(d, int):
        raise TypeError(
            f"'{key}' is expected to be type {int}; got type {type(d)}"
        )
    return d


def _get_obstacle_data_bool(
    data: Dict[str, Any], key: str, index: int
) -> bool:
    d = _get_obstacle_data(data, key, index)
    if not isinstance(d, bool):
        raise TypeError(
            f"'{key}' is expected to be type {bool}; got type {type(d)}"
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


def _get_obstacle_data(data: Dict[str, Any], key: str, index: int) -> Any:
    if key not in data:
        raise KeyError(f"Missing key '{key}' in dynamic obstacle[{index}]")
    return data[key]

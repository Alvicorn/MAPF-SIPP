import argparse
import os
import random
from copy import deepcopy
from pathlib import Path
from typing import Any, Dict, List, Set

import toml

from cbs_sipp.map.grid_map import GridMap
from cbs_sipp.path_utils import Vertex


def import_mapf_map_instance(filename: str) -> GridMap:
    """
    Import the MAPF map instance and return a GridMap object.

    Args:
        filename (str): Path to MAPF map instance.

    Returns:
        GridMap: Parsed map.
    """
    file = Path(filename)
    if not file.is_file():
        raise BaseException(filename + " does not exist.")
    with open(filename, "r") as f:
        line = f.readline()
        rows, columns = [int(x) for x in line.split(" ")]
        rows, columns = int(rows), int(columns)

        # #rows lines with the map
        my_map = []
        for _ in range(rows):
            line = f.readline()
            my_map.append([])
            for cell in line:
                if cell == "@":
                    my_map[-1].append(True)
                elif cell == ".":
                    my_map[-1].append(False)

    return GridMap(my_map)


def generate_agents(
    num_agents: int,
    start_free_spots: Set[Vertex],
    goal_free_spots: Set[Vertex],
) -> List[Dict[str, Any]]:
    """
    Randomly generate agents using the available free_spots

    Args:
        num_agents (int): Number of agents to generate.
        start_free_spots (Set[Vertex]): Free spots in the initial state.
        goal_free_spots (Set[Vertex]): Free spots in the goal state.

    Returns:
        List[Dict[str, Any]]: List of agents.
    """
    agents = []
    for j in range(num_agents):
        start = random.choice(list(start_free_spots))
        start_free_spots.remove(start)
        end = random.choice(list(goal_free_spots))
        goal_free_spots.remove(end)

        agents.append(
            {
                "id": j,
                "start_point": {"x": start[0], "y": start[1]},
                "goal_point": {"x": end[0], "y": end[1]},
            }
        )
    return agents


def increment_obstacle_id(s: str) -> str:
    """
    Helper function for incrementing to the next obstacle id.

    Args:
        s (str): Current obstacle id.

    Returns:
        str: Incremented obstacle id.
    """
    result = list(s)
    i = len(result) - 1

    while i >= 0:
        if result[i] == "z":
            result[i] = "a"
            i -= 1
        else:
            result[i] = chr(ord(result[i]) + 1)
            break
    else:
        result.insert(0, "a")

    return "".join(result)


def generate_obstacles(
    grid_map: GridMap,
    max_time: int,
    num_obstacles: int,
    max_num_trajectories: int,
    start_free_spots: Set[Vertex],
    goal_free_spots: Set[Vertex],
) -> List[Dict[str, Any]]:
    """
    Randomly generate obstacles using the available free spots.

    Args:
        grid_map (GridMap): Map of the MAPF instance.
        max_time (int): Maximum time allowed for trajectories.
        num_obstacles (int): Number of obstacles to generate.
        max_num_trajectories (_type_): Maximum number of trajectories for each obstacle,
        start_free_spots (Set[Vertex]): Free spots in the initial state.

    Returns:
        List[Dict[str, Any]]: List of obstacles
    """
    obstacles = []
    id = "a"

    for _ in range(num_obstacles):
        start = random.choice(list(start_free_spots))
        start_free_spots.remove(start)

        # build the trajectories
        trajectories = []
        possible_trajectories = random.randint(1, max_num_trajectories)
        for _ in range(possible_trajectories):
            map_size = len(grid_map.map) - 1
            num_points = random.randint(1, map_size)
            increment = max_time // num_points
            current_t = random.randint(1, increment)

            # generate the points in the trajectory
            points = []
            for k in range(1, num_points + 1):
                if k == num_points:
                    x, y = random.choice(list(goal_free_spots))
                    goal_free_spots.remove((x, y))
                else:
                    x = random.randint(0, map_size)
                    y = random.randint(0, map_size)

                    while not grid_map.is_free((x, y)):
                        x = random.randint(0, map_size)
                        y = random.randint(0, map_size)

                points.append({"x": x, "y": y, "t": current_t, "p": 0.1})
                current_t = random.randint(current_t + 1, increment * k)

            trajectories.append({"points": points})

        obstacles.append(
            {
                "id": id,
                "start_point": {"x": start[0], "y": start[1]},
                "trajectories": trajectories,
            }
        )
        id = increment_obstacle_id(id)

    return obstacles


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Generate dynamic environments instances"
    )
    parser.add_argument(
        "--map",
        type=str,
        default=None,
        help="The name of the map instance",
    )
    parser.add_argument(
        "--out",
        type=str,
        default=None,
        help="output directory",
    )
    parser.add_argument(
        "--num_instances",
        type=int,
        default=1,
        help="Number of random instances",
    )
    parser.add_argument(
        "--num_agents",
        type=int,
        default=1,
        help="Number of agents",
    )
    parser.add_argument(
        "--num_obstacles",
        type=int,
        default=1,
        help="Number of dynamic obstacles",
    )
    parser.add_argument(
        "--max_num_trajectories",
        type=int,
        default=1,
        help="Number of possible trajectories for a dynamic obstacle",
    )

    args = parser.parse_args()

    os.makedirs(args.out, exist_ok=True)
    map_name = args.map.split("/")[-1].removesuffix(".txt")

    grid_map = import_mapf_map_instance(args.map)
    max_time = (
        len(grid_map.map) ** len(grid_map.map[0])
    ) // 2  # magic number??

    free_positions = set()
    for i in range(len(grid_map.map)):
        for j in range(len(grid_map.map[0])):
            if grid_map.is_free((i, j)):
                free_positions.add((i, j))

    if args.num_agents + args.num_obstacles >= len(free_positions):
        raise Exception(
            "Not enough free position in the map to accommodated all agents and obstacles"
        )

    for i in range(args.num_instances):
        print(f"Generating instance {i}")
        start_free_spots = deepcopy(free_positions)
        goal_free_spots = deepcopy(free_positions)

        agents = generate_agents(
            args.num_agents, start_free_spots, goal_free_spots
        )
        obstacles = generate_obstacles(
            grid_map,
            max_time,
            args.num_obstacles,
            args.max_num_trajectories,
            start_free_spots,
            goal_free_spots,
        )
        data = {"agents": agents, "dynamic_obstacles": obstacles}

        with open(f"{args.out}/{map_name}-{i}.toml", "w") as f:
            toml.dump(data, f)


if __name__ == "__main__":
    main()

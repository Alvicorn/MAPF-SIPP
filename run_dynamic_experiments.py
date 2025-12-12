#!/usr/bin/python
import argparse
import glob
from pathlib import Path

from cbs_sipp.cbs.cbs import CBSSolver
from cbs_sipp.map.dynamic_env import import_dynamic_env_instance
from cbs_sipp.map.grid_map import GridMap
from cbs_sipp.path_utils import get_sum_of_cost
from cbs_sipp.visualize import Animation


def print_mapf_instance(my_map, starts, goals):
    print("Start locations")
    print_locations(my_map, starts)
    print("Goal locations")
    print_locations(my_map, goals)


def print_locations(my_map, locations):
    starts_map = [
        [-1 for _ in range(len(my_map[0]))] for _ in range(len(my_map))
    ]
    for i in range(len(locations)):
        starts_map[locations[i][0]][locations[i][1]] = i
    to_print = ""
    for x in range(len(my_map)):
        for y in range(len(my_map[0])):
            if starts_map[x][y] >= 0:
                to_print += str(starts_map[x][y]) + " "
            elif my_map[x][y]:
                to_print += "@ "
            else:
                to_print += ". "
        to_print += "\n"
    print(to_print)


def import_mapf_map_instance(filename) -> GridMap:
    file = Path(filename)
    if not file.is_file():
        raise BaseException(filename + " does not exist.")
    with open(filename, "r") as f:
        line = f.readline()
        rows, columns = [int(x) for x in line.split(" ")]
        rows = int(rows)
        columns = int(columns)
        # #rows lines with the map
        my_map = []
        for r in range(rows):
            line = f.readline()
            my_map.append([])
            for cell in line:
                if cell == "@":
                    my_map[-1].append(True)
                elif cell == ".":
                    my_map[-1].append(False)

    return GridMap(my_map)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Runs CBS on dynamic environments"
    )
    parser.add_argument(
        "--map",
        type=str,
        default=None,
        help="The name of the map instance",
    )
    parser.add_argument(
        "--instance",
        type=str,
        default=None,
        help="The name of the instance file(s)",
    )
    parser.add_argument(
        "--batch",
        action="store_true",
        default=False,
        help="Use batch output instead of animation",
    )
    parser.add_argument(
        "--disjoint",
        action="store_true",
        default=False,
        help="Use the disjoint splitting",
    )

    args = parser.parse_args()

    map = import_mapf_map_instance(args.map)

    for file in sorted(glob.glob(args.instance)):
        agents, obstacles = import_dynamic_env_instance(file, map)

        starts = [agent.start for agent in agents.values()]
        goals = [agent.goal for agent in agents.values()]

        print("***Import an instance***")
        print_mapf_instance(map.map, starts, goals)

        cbs = CBSSolver(map, starts, goals)
        paths = cbs.find_solution(args.disjoint)

        cost = get_sum_of_cost(paths)

        if not args.batch:
            for i, path in enumerate(paths):
                print(f"Agent {i}: {path}")
            print("***Test paths on a simulation***")

            animation = Animation(
                map.map,
                starts,
                goals,
                paths,
                dynamic_obstacles=list(obstacles.values()),
            )
            animation.save("output.mp4", 1.0)
            animation.show()

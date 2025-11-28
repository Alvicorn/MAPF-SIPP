#!/usr/bin/python
import argparse
import csv
import glob
from pathlib import Path
from typing import Dict

from cbs_sipp.cbs.cbs import CBSSolver
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


def import_mapf_instance(filename):
    f = Path(filename)
    if not f.is_file():
        raise BaseException(filename + " does not exist.")
    f = open(filename, "r")
    # first line: #rows #columns
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
    # #agents
    line = f.readline()
    num_agents = int(line)
    # #agents lines with the start/goal positions
    starts = []
    goals = []
    for a in range(num_agents):
        line = f.readline()
        sx, sy, gx, gy = [int(x) for x in line.split(" ")]
        starts.append((sx, sy))
        goals.append((gx, gy))
    f.close()
    return my_map, starts, goals


def load_test_expected_results(filename: str) -> Dict[str, int]:
    """
    Read in a csv file with expected results. The file should follow
    this format:

        instances/test_1.txt,41
        instances/test_10.txt,19
        instances/test_11.txt,35

    Args:
        filename (str): CSV file with the expected results.

    Returns:
        Dict[str, int]: Dictionary of test instance names and expected solution costs.
    """
    results = {}
    with open(filename, newline="") as f:
        reader = csv.reader(f)
        for row in reader:
            test_name, cost = row[:2]
            results[test_name] = int(cost)
    return results


def print_test_summary(total_tests: int, tests_passed: int) -> None:
    """
    Print the test results.

    Args:
        total_tests (int): Total tests executed.
        tests_passed (int): Number of tests that passed.
    """
    print("\nTest Summary")
    print(f"\tTest Count:\t{total_tests}")
    print(f"\tTests Passed:\t{tests_passed}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Runs CBS on static environments"
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
    parser.add_argument(
        "--test",
        type=str,
        default=None,
        help="Path to expected a csv with expected results of test instances",
    )

    args = parser.parse_args()

    results = None
    total_tests = 0
    tests_passed = 0
    if args.test is not None:
        results = load_test_expected_results(args.test)

    for file in sorted(glob.glob(args.instance)):
        my_map, starts, goals = import_mapf_instance(file)

        if args.test is None:
            print("***Import an instance***")
            print_mapf_instance(my_map, starts, goals)
        else:
            print(f"\nTesting {file}")
            total_tests += 1

        grid_map = GridMap(my_map)
        cbs = CBSSolver(grid_map, starts, goals)
        paths = cbs.find_solution(args.disjoint)

        cost = get_sum_of_cost(paths)
        if args.test and isinstance(results, dict):
            passed = cost == results.get(file, -1)
            if passed:
                print(f"Test result for {file}: \033[32mPASS\033[0m")
                tests_passed += 1
            else:
                print(f"Test result for {file}: \033[31mFAIL\033[0m")
                print(f"\tExpected {results.get(file, -1)}, got {cost}")

        if not args.batch:
            for i, path in enumerate(paths):
                print(f"Agent {i}: {path}")
            print("***Test paths on a simulation***")
            animation = Animation(my_map, starts, goals, paths)
            animation.save("output.mp4", 1.0)
            animation.show()

    if args.test:
        print_test_summary(total_tests, tests_passed)

import csv
import glob
from pathlib import Path
from typing import Dict

import pytest

from cbs_sipp.cbs.cbs import CBSSolver
from cbs_sipp.map.grid_map import GridMap
from cbs_sipp.path_utils import get_sum_of_cost

INSTANCES_PATH = "instances/static/tests"
INSTANCES = "test_*"
RESULTS = "min-sum-of-cost.csv"


@pytest.mark.parametrize(
    "is_disjoint", [False, True], ids=["is_not_disjoint", "is_disjoint"]
)
def test_static_environments(is_disjoint: bool):
    results = load_test_expected_results(f"{INSTANCES_PATH}/{RESULTS}")

    for file in sorted(glob.glob(f"{INSTANCES_PATH}/{INSTANCES}")):
        map, starts, goals = import_mapf_instance(file)
        static_map = GridMap(map)

        cbs = CBSSolver(static_map, starts, goals)
        paths = cbs.find_solution(is_disjoint)

        cost = get_sum_of_cost(paths)
        assert cost == results.get(file, -1), f"{file} failed!"


# --- HELPER TEST FUNCTIONS --- #


def load_test_expected_results(filename: str) -> Dict[str, int]:
    results = {}
    with open(filename, newline="") as f:
        reader = csv.reader(f)
        for row in reader:
            test_name, cost = row[:2]
            results[test_name] = int(cost)
    return results


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

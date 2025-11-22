from pathlib import Path
from typing import List, Set, Tuple


def import_dynamic_env(file: str) -> Tuple[List[List[str]], Set[str]]:
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

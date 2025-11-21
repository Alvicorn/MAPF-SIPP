from __future__ import annotations

from typing import List, Tuple

Vertex = Tuple[int, int]
Edge = Tuple[Vertex, Vertex]
Path = List[Vertex]


def get_location(path: Path, t: int) -> Vertex:
    """
    Get the vertex on a path given a timestep.

    Args:
        path (Path): Target path.
        t (int): Timestep to retrieve the vertex in the path.

    Returns:
        Vertex: Location at timestep t
    """
    if t < 0:
        raise Exception(f"Can not access a timestep less than 0: {t}")
    elif t < len(path):
        return path[t]
    else:
        return path[-1]  # wait at the goal location


def get_sum_of_cost(paths: List[Path]) -> int:
    """
    Get the sum of all paths.

    Args:
        paths (List[Path]): List of paths to calculate the cost.

    Returns:
        int: Sum of cost.
    """
    return sum(len(p) - 1 for p in paths)

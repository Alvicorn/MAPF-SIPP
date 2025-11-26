from __future__ import annotations

import heapq
from dataclasses import dataclass
from typing import Dict, List, Optional, Set, Tuple

from cbs_sipp.cbs.constraint import Constraint, ConstraintTable
from cbs_sipp.map.grid_map import GridMap
from cbs_sipp.path_utils import Path, Vertex


@dataclass(frozen=True, slots=True, repr=False, order=False, eq=False)
class AStarNode:
    loc: Tuple[int, int]
    g_val: int
    h_val: int
    parent: Optional[AStarNode]
    timestep: int
    evaluated: bool = False

    def __lt__(self, other: AStarNode) -> bool:
        return self.f_val < other.f_val

    @property
    def f_val(self) -> int:
        return self.g_val + self.h_val


def get_path(goal_node: AStarNode) -> Path:
    """
    Return the path to the goal node.

    Args:
        goal_node (AStarNode): Last node in the returned path.

    Returns:
        Path: Path to the goal node.
    """
    path = []
    curr = goal_node
    while curr is not None:
        path.append(curr.loc)
        curr = curr.parent
    path.reverse()
    return path


def push_node(open_list: List, node: AStarNode) -> None:
    heapq.heappush(open_list, (node.f_val, node.h_val, node.loc, node))


def pop_node(open_list: List) -> AStarNode:
    _, _, _, curr = heapq.heappop(open_list)
    return curr


def generate_children(
    grid_map: GridMap,
    h_values: Dict[Vertex, int],
    curr: AStarNode,
    constraint_table: ConstraintTable,
) -> List[AStarNode]:
    """
    Task 1.2/1.3: Check if a move from curr_loc to next_loc
    at time step next_time violates any given negative constraint.
    For efficiency the constraints are indexed in a
    constraint_table by time step, see build_constraint_table.

    Task 4.1: Check if a move from curr_loc to next_loc
    at time step next_time violates any given positive constraint.
    For efficiency the constraints are indexed in a
    constraint_table by time step, see build_constraint_table.

    Args:
        grid_map (GridMap): Binary obstacle map
        h_values (Dict[Vertex, int]): Heuristic values.
        curr (AStarNode): Current search node.
        constraint_table (ConstraintTable): Table of constraints.
        valid_locations (Set[Tuple[int, int]]): Pre-computed set of valid locations of my_map.

    Returns:
        List[AStarNode]: List of child search nodes from curr bounded by the
                         positive/negative constraints.
    """

    next_time = curr.timestep + 1
    neighbours = grid_map.get_neighbours(curr.loc) + [curr.loc]

    # identify a positive constraint child
    for next_loc in neighbours:
        if constraint_table.is_positively_constrained(
            curr.loc, next_loc, next_time
        ):
            return [
                AStarNode(
                    next_loc,
                    curr.g_val + 1,
                    h_values[next_loc],
                    curr,
                    next_time,
                )
            ]

    # no positive constraints; move onto negative constraints
    return [
        AStarNode(
            next_loc, curr.g_val + 1, h_values[next_loc], curr, next_time
        )
        for next_loc in neighbours
        if not constraint_table.is_negatively_constrained(
            curr.loc, next_loc, next_time
        )
    ]


def solution_found(
    curr: AStarNode,
    goal_loc: Tuple[int, int],
    constraint_table: ConstraintTable,
) -> Optional[Path]:
    """
    Check if a solution exists.

    Args:
        curr (AStarNode): Current search node popped from the open list.
        goal_loc (Tuple[int, int]): Goal location.
        constraint_table (ConstraintTable): Table of constraints.

    Returns:
        Optional[Path]: Solution path if one exists, None otherwise.
    """
    if curr.loc != goal_loc:
        return None

    for t, cons in constraint_table.negative_constraints.items():
        if t > curr.timestep and (goal_loc,) in cons:  # invalid future
            return None

    return get_path(curr)


def a_star(
    grid_map: GridMap,
    start_loc: Vertex,
    goal_loc: Vertex,
    h_values: Dict[Vertex, int],
    agent: int,
    constraints: Set[Constraint],
) -> Optional[Path]:
    """
    A* search using space and time domains.

    Args:
        grid_map (GridMap): Binary obstacle map
        start_loc (Vertex): Start position
        goal_loc (Vertex): Goal position
        h_values (Dict[Vertex, int]): Heuristic values
        agent (int): THe agent that is being re-planned
        constraints (Set[Constraint]): Constraints defining where robot
                                        should or cannot go at each timestep

    Returns:
        Optional[List[Tuple[int, int]]]: Solution path if one exists. None otherwise.
    """

    constraint_table = ConstraintTable(constraints, agent)
    open_list = []
    closed_list: Dict[
        Tuple[Vertex, int], int
    ] = {}  # [(vertex, timestep)]: g_val

    root = AStarNode(start_loc, 0, h_values[start_loc], None, 0)
    push_node(open_list, root)

    while open_list:
        curr = pop_node(open_list)

        key = (curr.loc, curr.timestep)
        old_g = closed_list.get(key)
        if old_g is not None and old_g <= curr.g_val:
            continue
        closed_list[key] = curr.g_val

        # goal test
        solution = solution_found(curr, goal_loc, constraint_table)
        if solution is not None:
            return solution

        for child in generate_children(
            grid_map, h_values, curr, constraint_table
        ):
            push_node(open_list, child)

    return None

from __future__ import annotations

import heapq
import random
import time as timer
from typing import Any, Collection, Dict, FrozenSet, List, Optional, Set, Tuple

from cbs_sipp.cbs.constraint import Collision, CollisionType, Constraint
from cbs_sipp.cbs.single_agent_planner import a_star
from cbs_sipp.map.grid_map import GridMap
from cbs_sipp.path_utils import Path, Vertex, get_location, get_sum_of_cost


def detect_collision(
    a1: int, path1: Path, a2: int, path2: Path
) -> Optional[Collision]:
    """
    Task 3.1: Return the first collision that occurs between two robot
              paths (or None if there is no collision) There are two
              types of collisions: vertex collision and edge collision.
              A vertex collision occurs if both robots occupy the same
              location at the same timestep An edge collision occurs if
              the robots swap their location at the same timestep. You
              should use "get_location(path, t)" to get the location of
              a robot at time t.

    Args:
        a1 (int): Target agent.
        path1 (Path): Path of a1.
        a2 (int): Competing agent.
        path2 (Path): Path of a2.

    Returns:
        Optional[Collision]: First collision between both paths.
    """
    max_t = max(len(path1), len(path2))
    get_loc = get_location

    for t in range(max_t):
        u = get_loc(path1, t)
        v = get_loc(path2, t)

        if u == v:  # vertex conflict
            return Collision(t, (u,), a1, a2)

        u_next = get_loc(path1, t + 1)
        v_next = get_loc(path2, t + 1)
        if (
            (u == v and u_next == v_next)  # same edge
            or (u == v_next and u_next == v)  # reverse edge
        ):  # edge collision
            return Collision(t + 1, (u, u_next), a1, a2)

    return None


def detect_collisions(paths: List[Path]) -> List[Collision]:
    """
    Task 3.1: Return a list of first collisions between all robot pairs.
              A collision can be represented as dictionary that contains
              the id of the two robots, the vertex or edge causing the collision,
              and the timestep at which the collision occurred. You should use
              your detect_collision function to find a collision between two robots.

    Args:
        paths (List[Path]): Collection of paths for each agent.

    Returns:
        List[Collision]: Collection of the first collisions between all paths.
    """
    collisions = []
    for a1, p1 in enumerate(paths):
        for a2, p2 in enumerate(paths[a1 + 1 :], start=a1 + 1):
            if (collision := detect_collision(a1, p1, a2, p2)) is not None:
                collisions.append(collision)
    return collisions


def standard_splitting(collision: Collision) -> List[Constraint]:
    """
    Task 3.2: Return a list of (two) constraints to resolve the given collision
              Vertex collision: the first constraint prevents the first agent to
                                be at the specified location at the specified
                                timestep, and the second constraint prevents the
                                second agent to be at the specified location at
                                the specified timestep.
              Edge collision: the first constraint prevents the first agent to
                              traverse the specified edge at the specified timestep,
                              and the second constraint prevents the second agent to
                              traverse the specified edge at the specified timestep

    Args:
        collision (Collision): Collision to split into negative constraints.

    Returns:
        List[Constraint]: Negative constraints as a result of the collision.
    """
    match collision.type:
        case CollisionType.VERTEX:
            return [
                Constraint(
                    collision.agent_1, collision.conflict, collision.timestep
                ),
                Constraint(
                    collision.agent_2, collision.conflict, collision.timestep
                ),
            ]
        case CollisionType.EDGE:
            return [
                Constraint(
                    collision.agent_1, collision.conflict, collision.timestep
                ),
                Constraint(
                    collision.agent_2,
                    collision.conflict[::-1],
                    collision.timestep,
                ),
            ]
        case _:
            raise BaseException(
                f"Unknown collision type provided: {collision.type}"
            )


def disjoint_splitting(collision: Collision) -> List[Constraint]:
    """
    Task 4.1: Return a list of (two) constraints to resolve the given collision
              Vertex collision: the first constraint enforces one agent to be at
                                the specified location at the specified timestep,
                                and the second constraint prevents the same agent
                                to be at the same location at the timestep.
              Edge collision: the first constraint enforces one agent to traverse
                              the specified edge at the specified timestep, and the
                              second constraint prevents the same agent to traverse
                              the specified edge at the specified timestep. Choose
                              the agent randomly

    Args:
        collision (Collision): Collision to split into positive constraints.

    Returns:
        List[Constraint]: A positive and negative constraint as a result of the collision.
    """
    colliding_agent = [collision.agent_1, collision.agent_2][
        random.randint(0, 1)
    ]
    match collision.type:
        case CollisionType.VERTEX:
            return [
                Constraint(
                    colliding_agent,
                    collision.conflict,
                    collision.timestep,
                    is_positive_constraint=True,
                ),
                Constraint(
                    colliding_agent,
                    collision.conflict,
                    collision.timestep,
                    is_positive_constraint=False,
                ),
            ]
        case CollisionType.EDGE:
            # flip the edge conflict as it is written with respect to agent_1
            conflict = (
                collision.conflict
                if colliding_agent == collision.agent_1
                else collision.conflict[::-1]
            )
            return [
                Constraint(
                    colliding_agent,
                    conflict,
                    collision.timestep,
                    is_positive_constraint=True,
                ),
                Constraint(
                    colliding_agent,
                    conflict,
                    collision.timestep,
                    is_positive_constraint=False,
                ),
            ]
        case _:
            raise BaseException(
                f"Unknown collision type provided: {collision.type}"
            )


class CBSSolver(object):
    """The high-level search of CBS."""

    def __init__(
        self, grid_map: GridMap, starts: List[Vertex], goals: List[Vertex]
    ):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = grid_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.CPU_time = 0

        self.low_level_cache: Dict[
            Tuple[int, FrozenSet[Constraint]], Path
        ] = {}
        self.open_list = []

        # compute heuristics for the low-level search
        self.heuristics = [
            self.my_map.compute_heuristics(goal) for goal in self.goals
        ]

    def push_node(self, node: Dict[str, Any]):
        heapq.heappush(
            self.open_list,
            (
                node["cost"],
                len(node["collisions"]),
                self.num_of_generated,
                node,
            ),
        )
        # print("Generate node {}".format(self.num_of_generated))
        self.num_of_generated += 1

    def pop_node(self) -> Dict[str, Any]:
        cost, _, id, node = heapq.heappop(self.open_list)
        # print(f"\nExpand node {id} with cost {cost}")
        self.num_of_expanded += 1
        return node

    @staticmethod
    def create_state_key(node: Dict[str, Any]) -> Tuple[Any, Any]:
        paths_key = tuple(tuple(p) for p in node["paths"])
        constraints_key = frozenset(node["constraints"])
        return (paths_key, constraints_key)

    def find_solution(self, disjoint: bool = True) -> List[Path]:
        """
        Finds paths for all agents from their start locations to their goal locations

        Args:
            disjoint (bool, optional): Use disjoint splitting. Defaults to True.

        Returns:
            List[Path]: Paths for all agents that do not collide.
        """
        self.start_time = timer.time()

        # Find initial path for each agent
        root = {"cost": 0, "constraints": set(), "paths": [], "collisions": []}

        paths = self._find_agent_paths(
            [a for a in range(self.num_of_agents)], root["constraints"]
        )
        if paths is None:
            raise BaseException("No solutions")
        root["paths"] = list(paths.values())
        root["cost"] = get_sum_of_cost(root["paths"])
        root["collisions"] = detect_collisions(root["paths"])

        self.push_node(root)
        closed_list = set()
        closed_list.add(self.create_state_key(root))

        while len(self.open_list) > 0:
            next_node = self.pop_node()

            # node has no collision, return solution
            if len(next_node["collisions"]) == 0:
                self.print_results(next_node)
                return next_node["paths"]

            # choose the first collision and convert to a list of constraints
            first_collision = next_node["collisions"].pop()

            constraints = (
                disjoint_splitting(first_collision)
                if disjoint
                else standard_splitting(first_collision)
            )

            # add a new child node to open list for each constraint
            for constraint in constraints:
                child = {
                    "cost": 0,
                    "constraints": next_node["constraints"].copy(),
                    "paths": list(next_node["paths"]),
                    "collisions": [],
                }
                child["constraints"].add(constraint)

                paths = self._find_agent_paths(
                    [constraint.agent], child["constraints"]
                )
                if paths is None:  # no solution for the given constraints
                    continue
                child["paths"][constraint.agent] = paths[constraint.agent]

                # additional check for disjoint CBS
                if disjoint and constraint.is_positive_constraint:
                    violating_agents = self._path_violates_positive_constraint(
                        constraint, child["paths"]
                    )
                    paths = self._find_agent_paths(
                        violating_agents, child["constraints"]
                    )

                    # at least one agent does not have a path with the
                    # given positive constraint
                    if paths is None:
                        continue

                    # update violating_agents paths with valid ones
                    for a, p in paths.items():
                        child["paths"][a] = p

                # only explore nodes that have not been visited yet
                state_key = self.create_state_key(child)
                if state_key not in closed_list:
                    child["collisions"] = detect_collisions(child["paths"])

                    child["cost"] = get_sum_of_cost(child["paths"])
                    self.push_node(child)
                    closed_list.add(state_key)

        self.print_results(root)
        return root["paths"]

    def print_results(self, node):
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.4f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node["paths"])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))

    def _path_violates_positive_constraint(
        self, constraint: Constraint, paths: List[Path]
    ) -> List[int]:
        """
        Identify the agents that new to find a new shortest path because
        of a given positive constraint

        Args:
            constraint (Constraint): Positive constraint being imposed.
            paths (List[Path]): Paths of agents to check.

        Returns:
            List[int]: Agent ids that need their paths re-evaluated.
        """
        rst = []

        for i in range(len(paths)):
            if i == constraint.agent:
                continue

            curr = get_location(paths[i], constraint.timestep)

            match constraint.type:
                case CollisionType.VERTEX:
                    if constraint.loc == (curr,):
                        rst.append(i)
                case CollisionType.EDGE:
                    prev = get_location(
                        paths[i], max(constraint.timestep - 1, 0)
                    )
                    edge_u, edge_v = constraint.loc
                    if (prev == edge_u and curr == edge_v) or (
                        prev == edge_v and curr == edge_u
                    ):
                        rst.append(i)
        return rst

    def _find_agent_paths(
        self, agents: Collection[int], constraints: Set[Constraint]
    ) -> Optional[Dict[int, Path]]:
        """
        Find the shortest paths of all agents under the given constraints.

        Args:
            agents (Collection[int]): Agents to search for the shortest path for.
            constraints (Set[Constraint]): Constraints in the given instance.

        Returns:
            Optional[Dict[int, Path]]: Map of agents to their shortest valid paths.
                                       None if at least one agent does not have a path under
                                       the given constraints.
        """
        frozen_constraints = frozenset(constraints)
        paths = {}
        for agent in agents:
            cache_key = (agent, frozen_constraints)
            if cache_key in self.low_level_cache:
                paths[agent] = self.low_level_cache[cache_key]
                continue

            path = a_star(
                self.my_map,
                self.starts[agent],
                self.goals[agent],
                self.heuristics[agent],
                agent,
                constraints,
            )
            if path is None:
                return None
            paths[agent] = path
            self.low_level_cache[cache_key] = path
        return paths

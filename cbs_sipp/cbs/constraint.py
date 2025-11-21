"""
Common data models shared among the MAPF solvers.
"""

from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass
from enum import Enum
from typing import DefaultDict, Set, Tuple

from cbs_sipp.path_utils import Vertex


class CollisionType(Enum):
    VERTEX = 1
    EDGE = 2


@dataclass(frozen=True, slots=True, repr=False, order=False, eq=False)
class Collision:
    """
    Model a collision between two competing agents.
    """

    timestep: int
    conflict: Tuple[Vertex, ...]
    agent_1: int
    agent_2: int

    @property
    def type(self) -> CollisionType:
        match len(self.conflict):
            case 1:
                return CollisionType.VERTEX
            case 2:
                return CollisionType.EDGE
            case _:
                raise Exception(f"Unknown conflict: {self.conflict}")


@dataclass(slots=True, frozen=True, repr=False, order=False)
class Constraint:
    """
    Model a Constraint applied to an agent.
    """

    agent: int
    loc: Tuple[Vertex, ...]
    timestep: int
    is_positive_constraint: bool = False

    @property
    def type(self) -> CollisionType:
        match len(self.loc):
            case 1:
                return CollisionType.VERTEX
            case 2:
                return CollisionType.EDGE
            case _:
                raise Exception(f"Unknown conflict: {self.loc}")


@dataclass(slots=True, repr=False, order=False)
class ConstraintTable:
    """
    Constraint table for a specific agent.
    """

    positive_constraints: DefaultDict[int, Set[Tuple[Vertex, ...]]]
    negative_constraints: DefaultDict[int, Set[Tuple[Vertex, ...]]]

    def __init__(self, constraints: Set[Constraint], agent: int):
        """
        Task 1.2/1.3: Build a table that contains the list of
        constraints of the given agent for each time step. The
        table can be used for a more efficient constraint
        violation check in the is_constrained function.

        Args:
            constraints (Set[Constraint]): Collection of all constraints
            agent (int): Target agent
        """
        self.positive_constraints = defaultdict(set)
        self.negative_constraints = defaultdict(set)

        for constraint in constraints:
            if constraint.agent == agent:
                if constraint.is_positive_constraint:
                    self.positive_constraints[constraint.timestep].add(
                        constraint.loc
                    )
                else:
                    self.negative_constraints[constraint.timestep].add(
                        constraint.loc
                    )
            else:
                if constraint.is_positive_constraint:
                    self.negative_constraints[constraint.timestep].add(
                        constraint.loc
                    )

                    # edge constraint; also add the reverse edge traversal
                    if len(constraint.loc) == 2:
                        self.negative_constraints[constraint.timestep].add(
                            constraint.loc[::-1]
                        )
                # continue

    def is_positively_constrained(
        self, curr_loc: Vertex, next_loc: Vertex, t: int
    ) -> bool:
        """
        Task 4.1: Check if a move from curr_loc to next_loc
        at time step next_time violates any given positive constraint.
        For efficiency the constraints are indexed in a
        constraint_table by time step, see build_constraint_table.

        Args:
            curr_loc (Vertex): Current agent location.
            next_loc (Vertex): Next agent location.
            next_time (int): Next timestep

        Returns:
            bool: True is a positive constraint is violated, false otherwise.
        """
        constraints = self.positive_constraints.get(t, set())
        return (next_loc,) in constraints or (
            curr_loc,
            next_loc,
        ) in constraints

    def is_negatively_constrained(
        self, curr_loc: Vertex, next_loc: Vertex, t: int
    ) -> bool:
        """
        Task 1.2/1.3: Check if a move from curr_loc to next_loc
        at time step next_time violates any given negative constraint.
        For efficiency the constraints are indexed in a
        constraint_table by time step, see build_constraint_table.

        Args:
            curr_loc (Vertex): Current agent location
            next_loc (Vertex): Next agent location
            next_time (int): Next timestep

        Returns:
            bool: True is a negative constraint is violated, false otherwise.
        """
        constraints = self.negative_constraints.get(t, set())
        return (next_loc,) in constraints or (
            curr_loc,
            next_loc,
        ) in constraints

    def get_negative_constraint_at(self, t: int) -> Set[Tuple[Vertex, ...]]:
        return self.negative_constraints.get(t, set())

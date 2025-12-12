from __future__ import annotations

import heapq
from dataclasses import dataclass
from functools import lru_cache
from typing import Dict, List, Tuple

from cbs_sipp.path_utils import Path, Vertex

DIRECTIONS = [(0, -1), (1, 0), (0, 1), (-1, 0)]


@dataclass(frozen=True, slots=True, eq=False, repr=False, order=False)
class GridMap:
    map: List[List[bool]]

    def in_bounds(self, loc: Vertex) -> bool:
        x, y = loc
        w = len(self.map)
        h = len(self.map[0])
        return 0 <= x < w and 0 <= y < h

    def is_free(self, loc: Vertex) -> bool:
        x, y = loc
        return self.in_bounds(loc) and not self.map[x][y]

    def get_neighbours(self, curr_loc: Vertex) -> List[Vertex]:
        x, y = curr_loc
        neighbours = []

        for dx, dy in DIRECTIONS:
            nx = x + dx
            ny = y + dy

            if self.is_free((nx, ny)):  # barriers in self.map are True values
                neighbours.append((nx, ny))

        return neighbours

    @lru_cache(maxsize=None)
    def compute_heuristics(self, goal: Vertex) -> Dict[Vertex, int]:
        """
        Build a heuristics table using Dijkstra to build
        a shortest-path tree rooted at the goal location

        Args:
            goal (Tuple[int, int]): Goal node.

        Returns:
            Dict[Vertex, int]: Dictionary of locations to heuristic values.
        """
        open_list = []
        closed_list: Dict[Vertex, int] = dict()

        heapq.heappush(open_list, (0, goal))
        closed_list[goal] = 0

        while len(open_list) > 0:
            cost, loc = heapq.heappop(open_list)

            if closed_list[loc] < cost:
                continue

            for child_loc in self.get_neighbours(loc):
                child_cost = cost + 1

                if (
                    child_loc not in closed_list
                    or child_cost < closed_list[child_loc]
                ):
                    closed_list[child_loc] = child_cost
                    heapq.heappush(open_list, (child_cost, child_loc))

        return closed_list

    def shortest_path(self, start: Vertex, goal: Vertex) -> Tuple[Path, int]:
        path, cost = self._shortest_path(start, goal)
        return list(path), cost

    @lru_cache(maxsize=None)
    def _shortest_path(
        self, start: Vertex, goal: Vertex
    ) -> Tuple[Tuple[Vertex], int]:
        """
        Find the shortest path from start to goal.

        Args:
            Start (Tuple[int, int]): Start node.
            goal (Tuple[int, int]): Goal node.

        Returns:
            Tuple[Path, int]: Resulting path and the cost.
        """
        if not self.is_free(start):
            raise ValueError(
                f"Can not find a shortest path starting from a barrier {start}"
            )
        if not self.is_free(goal):
            raise ValueError(
                f"Can not find a shortest path ending at a barrier {goal}"
            )

        open_list = []
        closed_list: Dict[Vertex, int] = dict()

        root = (0, start, None)

        heapq.heappush(open_list, root)
        closed_list[start] = 0

        while len(open_list) > 0:
            cost, loc, parent = heapq.heappop(open_list)

            if loc == goal:
                path = []
                curr = (cost, loc, parent)
                while curr is not None:
                    _, pos, parent = curr
                    path.append(pos)
                    curr = parent
                path.reverse()
                return tuple(path), cost

            if closed_list[loc] < cost:
                continue

            for child_loc in self.get_neighbours(loc):
                child_cost = cost + 1

                if (
                    child_loc not in closed_list
                    or child_cost < closed_list[child_loc]
                ):
                    closed_list[child_loc] = child_cost
                    heapq.heappush(
                        open_list, (child_cost, child_loc, (cost, loc, parent))
                    )

        raise ValueError(f"A path between {start} and {goal} does not exist")

from __future__ import annotations

import heapq
from dataclasses import dataclass
from typing import Dict, List

from cbs_sipp.path_utils import Vertex

DIRECTIONS = [(0, -1), (1, 0), (0, 1), (-1, 0)]


@dataclass(frozen=True, slots=True, eq=False, repr=False, order=False)
class StaticMap:
    map: List[List[bool]]

    def get_neighbours(self, curr_loc: Vertex) -> List[Vertex]:
        x, y = curr_loc
        w = len(self.map)
        h = len(self.map[0])
        neighbours = []

        for dx, dy in DIRECTIONS:
            nx = x + dx
            ny = y + dy

            if (
                0 <= nx < w and 0 <= ny < h and not self.map[nx][ny]
            ):  # barriers in self.map are True values
                neighbours.append((nx, ny))

        return neighbours

    def compute_heuristics(self, goal: Vertex) -> Dict[Vertex, int]:
        """
        Build a heuristics table using Dijkstra to build
        a shortest-path tree rooted at the goal location

        Args:
            my_map (List[List[bool]]): Search space.
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

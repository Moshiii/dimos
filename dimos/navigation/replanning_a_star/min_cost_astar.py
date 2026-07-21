# Copyright 2025-2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import heapq

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import VectorLike
from dimos.msgs.nav_msgs.OccupancyGrid import CostValues, OccupancyGrid
from dimos.msgs.nav_msgs.Path import Path
from dimos.utils.logging_config import setup_logger

# Try to import C++ extension for faster pathfinding
try:
    from dimos.navigation.replanning_a_star.min_cost_astar_ext import (
        min_cost_astar_cpp as _astar_cpp,
    )

    _USE_CPP = True
    _CPP_IMPORT_ERROR: ImportError | None = None
except ImportError as e:
    _USE_CPP = False
    _CPP_IMPORT_ERROR = e

logger = setup_logger()

# Define possible movements (8-connected grid with diagonal movements)
_directions = [
    (0, 1),
    (1, 0),
    (0, -1),
    (-1, 0),
    (1, 1),
    (1, -1),
    (-1, 1),
    (-1, -1),
]

# Cost for each movement (straight vs diagonal)
_sc = 1.0  # Straight cost
_dc = 1.42  # Diagonal cost (approximately sqrt(2))
_movement_costs = [_sc, _sc, _sc, _sc, _dc, _dc, _dc, _dc]


# Heuristic function (Octile distance for 8-connected grid)
def _heuristic(x1: int, y1: int, x2: int, y2: int) -> float:
    dx = abs(x2 - x1)
    dy = abs(y2 - y1)
    # Octile distance: optimal for 8-connected grids with diagonal movement
    return (dx + dy) + (_dc - 2 * _sc) * min(dx, dy)


def _reconstruct_path(
    parents: dict[tuple[int, int], tuple[int, int]],
    current: tuple[int, int],
    costmap: OccupancyGrid,
    start_tuple: tuple[int, int],
    goal_tuple: tuple[int, int],
) -> Path:
    frame_id = costmap.frame_id
    waypoints: list[PoseStamped] = []
    while current in parents:
        world_point = costmap.grid_to_world(current)
        pose = PoseStamped(
            frame_id=frame_id,
            position=[world_point.x, world_point.y, 0.0],
            orientation=Quaternion(0, 0, 0, 1),  # Identity quaternion
        )
        waypoints.append(pose)
        current = parents[current]

    start_world_point = costmap.grid_to_world(start_tuple)
    start_pose = PoseStamped(
        frame_id=frame_id,
        position=[start_world_point.x, start_world_point.y, 0.0],
        orientation=Quaternion(0, 0, 0, 1),
    )
    waypoints.append(start_pose)

    waypoints.reverse()

    # Add the goal position if it's not already included
    goal_point = costmap.grid_to_world(goal_tuple)

    if (
        not waypoints
        or (waypoints[-1].x - goal_point.x) ** 2 + (waypoints[-1].y - goal_point.y) ** 2 > 1e-10
    ):
        goal_pose = PoseStamped(
            frame_id=frame_id,
            position=[goal_point.x, goal_point.y, 0.0],
            orientation=Quaternion(0, 0, 0, 1),
        )
        waypoints.append(goal_pose)

    return Path(frame_id=frame_id, poses=waypoints)


def _reconstruct_path_from_coords(
    path_coords: list[tuple[int, int]],
    costmap: OccupancyGrid,
) -> Path:
    frame_id = costmap.frame_id
    waypoints: list[PoseStamped] = []

    for gx, gy in path_coords:
        world_point = costmap.grid_to_world((gx, gy))
        pose = PoseStamped(
            frame_id=frame_id,
            position=[world_point.x, world_point.y, 0.0],
            orientation=Quaternion(0, 0, 0, 1),
        )
        waypoints.append(pose)

    return Path(frame_id=frame_id, poses=waypoints)


def min_cost_astar(
    costmap: OccupancyGrid,
    goal: VectorLike,
    start: VectorLike = (0.0, 0.0),
    cost_threshold: int = 100,
    unknown_penalty: float = 0.8,
    path_length_weight: float = 0.0,
    cell_cost_weight: float = 1.0,
    heuristic_weight: float = 1.0,
    use_cpp: bool = True,
) -> Path | None:
    if cost_threshold <= 0:
        raise ValueError("cost_threshold must be positive")
    if path_length_weight < 0 or cell_cost_weight < 0 or heuristic_weight < 0:
        raise ValueError("A* objective weights must be non-negative")

    start_vector = costmap.world_to_grid(start)
    goal_vector = costmap.world_to_grid(goal)

    start_tuple = (int(start_vector.x), int(start_vector.y))
    goal_tuple = (int(goal_vector.x), int(goal_vector.y))

    if not (0 <= goal_tuple[0] < costmap.width and 0 <= goal_tuple[1] < costmap.height):
        return None

    if use_cpp:
        if _USE_CPP:
            path_coords = _astar_cpp(
                costmap.grid,
                start_tuple[0],
                start_tuple[1],
                goal_tuple[0],
                goal_tuple[1],
                cost_threshold,
                unknown_penalty,
                path_length_weight,
                cell_cost_weight,
                heuristic_weight,
            )
            if not path_coords:
                return None
            return _reconstruct_path_from_coords(path_coords, costmap)
        else:
            logger.warning(
                "C++ A* module could not be imported (%s). Using Python.",
                _CPP_IMPORT_ERROR,
            )

    open_set: list[tuple[float, float, tuple[int, int]]] = []  # Priority queue for nodes to explore
    closed_set: set[tuple[int, int]] = set()  # Set of explored nodes

    # f(n) = w1 * past_distance + w2 * past_safety + w3 * future_distance
    # Each component is accumulated bare (no weight) so the three knobs are independent.
    dist_score: dict[tuple[int, int], float] = {start_tuple: 0.0}
    safety_score: dict[tuple[int, int], float] = {start_tuple: 0.0}
    parents: dict[tuple[int, int], tuple[int, int]] = {}

    # Priority: (f(n), dist tiebreaker, node)
    h_dist = _heuristic(start_tuple[0], start_tuple[1], goal_tuple[0], goal_tuple[1])
    f_start = (
        path_length_weight * 0.0
        + cell_cost_weight * 0.0
        + heuristic_weight * h_dist
    )
    heapq.heappush(open_set, (f_start, h_dist, start_tuple))

    while open_set:
        _, _, current = heapq.heappop(open_set)
        current_x, current_y = current

        if current in closed_set:
            continue

        if current == goal_tuple:
            return _reconstruct_path(parents, current, costmap, start_tuple, goal_tuple)

        closed_set.add(current)

        for i, (dx, dy) in enumerate(_directions):
            neighbor_x, neighbor_y = current_x + dx, current_y + dy
            neighbor = (neighbor_x, neighbor_y)

            if not (0 <= neighbor_x < costmap.width and 0 <= neighbor_y < costmap.height):
                continue

            if neighbor in closed_set:
                continue

            neighbor_val = costmap.grid[neighbor_y, neighbor_x]

            if neighbor_val >= cost_threshold:
                continue

            if neighbor_val == CostValues.UNKNOWN:
                cell_cost = cost_threshold * unknown_penalty
                if cell_cost >= cost_threshold:
                    continue
            elif neighbor_val == CostValues.FREE:
                cell_cost = 0.0
            else:
                cell_cost = neighbor_val

            tentative_dist = dist_score[current] + _movement_costs[i]
            tentative_safety = safety_score[current] + (cell_cost / cost_threshold)

            # f(n) = w1 * past_distance + w2 * past_safety + w3 * future_distance
            h_dist = _heuristic(neighbor_x, neighbor_y, goal_tuple[0], goal_tuple[1])
            tentative_f = (
                path_length_weight * tentative_dist
                + cell_cost_weight * tentative_safety
                + heuristic_weight * h_dist
            )

            # Get the current scores for the neighbor or set to infinity if not yet explored.
            neighbor_dist = dist_score.get(neighbor, float("inf"))
            neighbor_safety = safety_score.get(neighbor, float("inf"))

            tentative_g = path_length_weight * tentative_dist + cell_cost_weight * tentative_safety
            neighbor_g = (
                path_length_weight * neighbor_dist + cell_cost_weight * neighbor_safety
            )
            if tentative_g < neighbor_g:
                # Update the neighbor's scores and parent
                parents[neighbor] = current
                dist_score[neighbor] = tentative_dist
                safety_score[neighbor] = tentative_safety

                priority_dist = tentative_dist + h_dist

                # Add the neighbor to the open set with its priority
                heapq.heappush(open_set, (tentative_f, priority_dist, neighbor))

    return None

# Copyright 2026 Dimensional Inc.
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

"""Scoring for the nav-3d evaluator: SPL against the walked route, the climb
limit, and the demonstrated reference a case is measured against."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

import numpy as np

if TYPE_CHECKING:
    from numpy.typing import NDArray

    from dimos.navigation.nav_3d.evaluator.config import EvalConfig
    from dimos.navigation.nav_3d.evaluator.recording import Trajectory


def path_length(waypoints: NDArray[np.float32]) -> float:
    if len(waypoints) < 2:
        return 0.0
    return float(np.linalg.norm(np.diff(waypoints, axis=0), axis=1).sum())


def arc_lengths(points: NDArray[np.floating[Any]]) -> NDArray[np.float64]:
    """Cumulative 3D arc length at each point, starting at zero."""
    steps = np.linalg.norm(np.diff(points, axis=0), axis=1)
    return np.concatenate([[0.0], np.cumsum(steps)]).astype(np.float64)


def goal_reached(
    waypoints: NDArray[np.float32], goal: tuple[float, float, float], tolerance: float
) -> bool:
    return bool(np.linalg.norm(waypoints[-1] - np.asarray(goal, dtype=np.float32)) <= tolerance)


# Floor on any length used as a divisor or a direction.
MIN_LENGTH_M = 1e-6


@dataclass
class KinematicsResult:
    """Steppability check of the path profile."""

    valid: bool
    violation_points: NDArray[np.float32]


def _resample(waypoints: NDArray[np.float32], spacing: float) -> NDArray[np.float32]:
    """Points every spacing meters of 3D arc length along the polyline."""
    arc = arc_lengths(waypoints)
    if arc[-1] <= spacing:
        return waypoints[[0, -1]]
    s = np.append(np.arange(0.0, arc[-1], spacing), arc[-1])
    return np.stack([np.interp(s, arc, waypoints[:, i]) for i in range(3)], axis=1).astype(
        np.float32
    )


def check_kinematics(waypoints: NDArray[np.float32], cfg: EvalConfig) -> KinematicsResult:
    """Reject paths that climb steeper than the robot can. Resampled at
    window_m of arc so cell quantization does not read as a cliff."""
    if len(waypoints) < 2:
        return KinematicsResult(True, waypoints[:0])
    profile = _resample(waypoints, cfg.kinematic_window_m)
    d = np.diff(profile, axis=0)
    rise = np.abs(d[:, 2])
    run = np.linalg.norm(d[:, :2], axis=1)
    bad = rise > np.maximum(run * cfg.max_slope, cfg.max_step_m)
    return KinematicsResult(not bad.any(), profile[1:][bad])


@dataclass
class Reference:
    """Demonstrated route between a case's endpoints."""

    length: float
    snapped: bool
    # When the robot stood at the start about to walk the route. Inf when
    # the endpoints are off the trajectory or no causal pair exists.
    start_ts: float
    # True when the goal was visited before the chosen start visit, so a
    # planner at start_ts targets a place the robot has already been.
    causal: bool


@dataclass
class _Visits:
    """Poses near each endpoint, with the walked length of every pairing."""

    foot: NDArray[np.float32]
    near_s: NDArray[np.int64]
    near_g: NDArray[np.int64]
    totals: NDArray[np.float64]


def _visits(
    trajectory: Trajectory,
    start: tuple[float, float, float],
    goal: tuple[float, float, float],
    cfg: EvalConfig,
) -> _Visits | None:
    """None when either endpoint is farther than snap_max_m from the trajectory."""
    foot = trajectory.foot(cfg.robot_height)
    ds = np.linalg.norm(foot - np.asarray(start, dtype=np.float32), axis=1)
    dg = np.linalg.norm(foot - np.asarray(goal, dtype=np.float32), axis=1)
    if ds.min() > cfg.visit_radius_m or dg.min() > cfg.visit_radius_m:
        return None
    arcs = trajectory.arc_lengths()
    near_s = np.flatnonzero(ds <= cfg.visit_radius_m)
    near_g = np.flatnonzero(dg <= cfg.visit_radius_m)
    totals = (
        np.abs(arcs[near_s][:, None] - arcs[near_g][None, :])
        + ds[near_s][:, None]
        + dg[near_g][None, :]
    )
    return _Visits(foot, near_s, near_g, totals)


def reference_length(
    trajectory: Trajectory,
    start: tuple[float, float, float],
    goal: tuple[float, float, float],
    cfg: EvalConfig,
) -> Reference:
    """Shortest walked length demonstrated between start and goal, minimized
    over every visit pairing and preferring causal ones. Falls back to
    straight-line distance when an endpoint is off the trajectory."""
    visits = _visits(trajectory, start, goal, cfg)
    if visits is None:
        straight = float(np.linalg.norm(np.asarray(goal) - np.asarray(start)))
        return Reference(straight, False, float("inf"), False)
    totals = visits.totals
    backward = trajectory.ts[visits.near_g][None, :] <= trajectory.ts[visits.near_s][:, None]
    causal = bool(backward.any())
    if causal:
        totals = np.where(backward, totals, np.inf)
    best = np.unravel_index(totals.argmin(), totals.shape)
    i = int(visits.near_s[best[0]])
    start_ts = float(trajectory.ts[i]) if causal else float("inf")
    return Reference(max(float(totals[best]), MIN_LENGTH_M), True, start_ts, causal)


def ground_truth_route(
    trajectory: Trajectory,
    start: tuple[float, float, float],
    goal: tuple[float, float, float],
    cfg: EvalConfig,
) -> NDArray[np.float32] | None:
    """Foot-level polyline of the shortest walk between start and goal.
    Ignores causality: it describes terrain, not what the robot knew."""
    visits = _visits(trajectory, start, goal, cfg)
    if visits is None:
        return None
    best = np.unravel_index(visits.totals.argmin(), visits.totals.shape)
    i = int(visits.near_s[best[0]])
    j = int(visits.near_g[best[1]])
    # Orient the slice start-to-goal so the route reads in the case's direction.
    route = visits.foot[i : j + 1] if i <= j else visits.foot[j : i + 1][::-1]
    return route.astype(np.float32)


def spl(success: bool, l_ref: float, p_len: float) -> float:
    if not success:
        return 0.0
    return l_ref / max(p_len, l_ref, MIN_LENGTH_M)


def timing_stats(samples_ms: list[float]) -> dict[str, float]:
    if not samples_ms:
        return {"p50": 0.0, "p95": 0.0, "max": 0.0}
    arr = np.asarray(samples_ms)
    return {
        "p50": float(np.percentile(arr, 50)),
        "p95": float(np.percentile(arr, 95)),
        "max": float(arr.max()),
    }

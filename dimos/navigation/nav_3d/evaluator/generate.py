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

"""Generate evaluation cases from a recorded trajectory.

Endpoint pairs come off the walked path, so both are proven reachable, and are
kept only when non-trivial and causal. Deterministic.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np

from dimos.navigation.nav_3d.evaluator import metrics
from dimos.navigation.nav_3d.evaluator.cases import Case
from dimos.navigation.nav_3d.evaluator.tagging import STAIRS_DZ_M, route_tags

if TYPE_CHECKING:
    from numpy.typing import NDArray

    from dimos.navigation.nav_3d.evaluator.config import EvalConfig
    from dimos.navigation.nav_3d.evaluator.final_map import FinalMap
    from dimos.navigation.nav_3d.evaluator.recording import Trajectory


# Beyond this a near-straight flat pair is trivial whatever the map holds.
MAX_TRIVIAL_SPAN_M = 30.0


# Candidate pairs must be this far apart along the walk and in a straight line.
MIN_SEPARATION_M = 3.0
MIN_EUCLID_M = 2.0
# A flat pair is only interesting if the walk detoured this much over the
# straight line.
DETOUR_RATIO_MIN = 1.3
# Endpoint pairs are binned this coarsely before ranking, so near-identical
# pairs compete for one slot.
BIN_SIZE_M = 2.0
WAYPOINT_SPACING_M = 1.0
# Two cases are duplicates when both endpoints land within this radius.
DEDUPE_RADIUS_M = 1.5
# Share of slots reserved for flat cases when the recording has them.
FLAT_FRACTION = 0.25
# Coverage sectors: a case earns a slot first by connecting a sector pair
# no accepted case connects yet.
SECTOR_SIZE_M = 8.0
SECTOR_Z_M = 1.5
# A sector may anchor at most this many selected cases, which prevents a
# single high-priority spot from becoming the hub of every case.
ENDPOINT_REUSE_MAX = 2
# Floor on the case count. When strict selection falls short, a relaxed pass
# ignores the sector caps and the flat quota to reach it.
MIN_CASES = 10


def resolve_max_cases(max_cases: int | None, walked_total_m: float) -> int:
    """Case count, scaled with the walked distance when not pinned."""
    if max_cases is not None:
        return max_cases
    return int(np.clip(walked_total_m / 25.0, 16, 48))


@dataclass
class Candidate:
    start: tuple[float, float, float]
    goal: tuple[float, float, float]
    walked_m: float
    detour_ratio: float
    dz: float

    @property
    def priority(self) -> float:
        return (
            min(self.detour_ratio, 3.0)
            + 2.0 * min(abs(self.dz), 3.0)
            + 0.5 * min(self.walked_m / 50.0, 2.0)
        )


def snap_to_surface(
    point: NDArray[np.float32],
    surface: NDArray[np.float32],
    snap_max_m: float,
) -> NDArray[np.float32] | None:
    """Nearest standable surface cell, or None when the point is off the map.
    Horizontal distance dominates so z drift cannot snap onto another floor."""
    if len(surface) == 0:
        return None
    hd = np.linalg.norm(surface[:, :2] - point[:2], axis=1)
    zd = np.abs(surface[:, 2] - point[2])
    score = hd + np.where(zd < 1.0, zd * 0.5, np.inf)
    best = int(score.argmin())
    if not np.isfinite(score[best]) or hd[best] > snap_max_m:
        return None
    return np.asarray(surface[best], dtype=np.float32)


def _subsample_indices(trajectory: Trajectory, spacing_m: float) -> NDArray[np.int64]:
    arcs = trajectory.arc_lengths()
    targets = np.arange(0.0, arcs[-1], spacing_m)
    return np.unique(np.searchsorted(arcs, targets))


def generate_cases(
    trajectory: Trajectory,
    final: FinalMap,
    surface: NDArray[np.float32],
    cfg: EvalConfig,
    max_cases: int | None = None,
    min_cases: int = MIN_CASES,
) -> list[Case]:
    map_keys = final.occupied_keys
    arcs = trajectory.arc_lengths()
    foot = trajectory.foot(cfg.robot_height)

    idx = _subsample_indices(trajectory, WAYPOINT_SPACING_M)
    snaps = np.full((len(idx), 3), np.nan, dtype=np.float32)
    for n, i in enumerate(idx):
        hit = snap_to_surface(foot[i], surface, cfg.snap_max_m)
        if hit is not None:
            snaps[n] = hit
    ok = np.isfinite(snaps[:, 0])
    way_arcs = arcs[idx]

    candidates: dict[tuple[int, ...], Candidate] = {}
    for ai in range(len(idx)):
        if not ok[ai]:
            continue
        sa = snaps[ai]
        near_a = np.linalg.norm(foot - sa, axis=1) <= cfg.snap_max_m
        last_visit_a = float(trajectory.ts[near_a].max()) if near_a.any() else -np.inf
        later = np.arange(ai + 1, len(idx))
        later = later[ok[later]]
        if not len(later):
            continue
        walked = way_arcs[later] - way_arcs[ai]
        deltas = snaps[later] - sa
        euclid = np.linalg.norm(deltas, axis=1)
        keep = (walked >= MIN_SEPARATION_M) & (euclid >= MIN_EUCLID_M)
        for bi, w, e in zip(later[keep], walked[keep], euclid[keep], strict=True):
            sb = snaps[bi]
            dz = float(sb[2] - sa[2])
            detour = float(w / e)
            # Backward in time is always causal. Forward only when the start
            # spot is revisited after the goal visit.
            directed = [(sb, sa, -dz)]
            if last_visit_a >= float(trajectory.ts[idx[bi]]):
                directed.append((sa, sb, dz))
            proposed = [
                (
                    Candidate(
                        start=(float(p_start[0]), float(p_start[1]), float(p_start[2])),
                        goal=(float(p_goal[0]), float(p_goal[1]), float(p_goal[2])),
                        walked_m=float(w),
                        detour_ratio=detour,
                        dz=d_dz,
                    ),
                    _bin_key(p_start, p_goal, d_dz, BIN_SIZE_M),
                )
                for p_start, p_goal, d_dz in directed
            ]
            # The sweep only decides admission, never priority, so a pair that
            # cannot win any of its bins never has to pay for one.
            if all(
                (best := candidates.get(key)) is not None and best.priority >= cand.priority
                for cand, key in proposed
            ):
                continue
            if detour < DETOUR_RATIO_MIN and abs(dz) < STAIRS_DZ_M:
                # A long near-straight flat pair is trivial. Not worth a sweep.
                if e > MAX_TRIVIAL_SPAN_M:
                    continue
                line = np.stack([sa, sb])
                if metrics.check_path(line, map_keys, cfg).valid:
                    continue
            for cand, key in proposed:
                best = candidates.get(key)
                if best is None or cand.priority > best.priority:
                    candidates[key] = cand

    ranked = sorted(candidates.values(), key=lambda c: (-c.priority, c.start, c.goal))
    selected = _select_diverse(ranked, resolve_max_cases(max_cases, float(arcs[-1])), min_cases)
    cases = []
    for n, cand in enumerate(selected):
        route = metrics.ground_truth_route(trajectory, cand.start, cand.goal, cfg)
        tags = route_tags(cand.start, cand.goal, route, map_keys, cfg)
        cases.append(_to_case(cand, n, tags))
    return cases


def _bin_key(
    start: NDArray[np.float32], goal: NDArray[np.float32], dz: float, bin_size_m: float
) -> tuple[int, ...]:
    bins = np.floor(np.array([*start[:2], *goal[:2]]) / bin_size_m).astype(int)
    return (*bins, int(np.sign(dz)) if abs(dz) >= STAIRS_DZ_M else 0)


def _is_duplicate(cand: Candidate, accepted: list[Candidate], radius: float) -> bool:
    a = np.array([*cand.start, *cand.goal])
    for other in accepted:
        b = np.array([*other.start, *other.goal])
        if np.linalg.norm(a[:3] - b[:3]) < radius and np.linalg.norm(a[3:] - b[3:]) < radius:
            return True
    return False


def _select_diverse(
    ranked: list[Candidate], max_cases: int, min_cases: int = MIN_CASES
) -> list[Candidate]:
    """Spread-greedy selection under a sector cap and flat quota, with a
    relaxed pass to reach min_cases."""
    if not ranked:
        return []
    flat_target = int(max_cases * FLAT_FRACTION)
    stairs_cap = max_cases - flat_target

    starts = np.array([c.start for c in ranked], dtype=np.float32)
    goals = np.array([c.goal for c in ranked], dtype=np.float32)
    priorities = np.array([c.priority for c in ranked], dtype=np.float32)
    is_stairs = np.array([abs(c.dz) >= STAIRS_DZ_M for c in ranked])
    spread_cap = 2.0 * SECTOR_SIZE_M

    def sector(p: NDArray[np.float32]) -> tuple[int, ...]:
        return (
            int(np.floor(p[0] / SECTOR_SIZE_M)),
            int(np.floor(p[1] / SECTOR_SIZE_M)),
            round(float(p[2]) / SECTOR_Z_M),
        )

    usage: dict[tuple[int, ...], int] = {}
    used_points: list[NDArray[np.float32]] = []
    alive = np.ones(len(ranked), dtype=bool)
    sector_capped: list[int] = []
    stairs: list[Candidate] = []
    flats: list[Candidate] = []

    def fill(target: int, relax: bool) -> None:
        while alive.any() and len(stairs) + len(flats) < target:
            if used_points:
                used = np.stack(used_points)
                d_start = np.linalg.norm(starts[:, None] - used[None], axis=2).min(axis=1)
                d_goal = np.linalg.norm(goals[:, None] - used[None], axis=2).min(axis=1)
                spread = np.minimum(d_start, spread_cap) + np.minimum(d_goal, spread_cap)
            else:
                spread = np.full(len(ranked), 2.0 * spread_cap, dtype=np.float32)
            score = priorities + 0.4 * spread
            score[~alive] = -np.inf
            if not relax and len(stairs) >= stairs_cap:
                score[is_stairs] = -np.inf
            if not np.isfinite(score).any():
                break
            n = int(score.argmax())
            alive[n] = False
            cand = ranked[n]
            sa, sb = sector(starts[n]), sector(goals[n])
            if not relax and (
                usage.get(sa, 0) >= ENDPOINT_REUSE_MAX or usage.get(sb, 0) >= ENDPOINT_REUSE_MAX
            ):
                sector_capped.append(n)
                continue
            bucket = stairs if is_stairs[n] else flats
            if _is_duplicate(cand, bucket, DEDUPE_RADIUS_M):
                continue
            usage[sa] = usage.get(sa, 0) + 1
            usage[sb] = usage.get(sb, 0) + 1
            used_points.append(starts[n])
            used_points.append(goals[n])
            bucket.append(cand)

    fill(max_cases, relax=False)
    min_cases = min(min_cases, max_cases)
    if len(stairs) + len(flats) < min_cases:
        alive[sector_capped] = True
        fill(min_cases, relax=True)

    return (stairs + flats)[:max_cases]


def _to_case(cand: Candidate, n: int, tags: list[str]) -> Case:
    kind = "up" if cand.dz >= STAIRS_DZ_M else "down" if cand.dz <= -STAIRS_DZ_M else "flat"
    return Case(id=f"auto_{n:02d}_{kind}", start=cand.start, goal=cand.goal, tags=["auto", *tags])

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

"""Replay case suites through a pipeline and score them.

Generated cases plan twice, online at their start time and again on the whole
recording. Curated and infeasible cases plan once, on the final map. The
headline score is validity-gated SPL on the incremental map.
"""

from __future__ import annotations

from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict, dataclass, field, replace
import itertools
from time import perf_counter
from typing import TYPE_CHECKING

import numpy as np

from dimos.navigation.nav_3d.evaluator import metrics
from dimos.navigation.nav_3d.evaluator.config import EvalConfig
from dimos.navigation.nav_3d.evaluator.final_map import (
    load_or_build_checkpoints,
    load_or_build_final_map,
)
from dimos.navigation.nav_3d.evaluator.pipeline import PipelineIntrospection, make_pipeline
from dimos.navigation.nav_3d.evaluator.recording import iter_world_frames, load_trajectory
from dimos.navigation.nav_3d.evaluator.voxel_keys import key_centers
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from numpy.typing import NDArray

    from dimos.navigation.nav_3d.evaluator.cases import Case, Suite
    from dimos.navigation.nav_3d.evaluator.pipeline import NavPipeline

logger = setup_logger()

MAX_COLLISIONS_KEPT = 50


@dataclass
class PlanOutcome:
    planned: bool
    reached: bool
    valid: bool
    # Every sample stands on final-map occupancy. Fabricated bridges fail.
    supported: bool
    # For an ordinary case: all of the above. For an expect_fail case: the
    # planner correctly refused the infeasible goal.
    success: bool
    length: float
    plan_ms: float
    spl: float
    # Gate margin along the path (see GateResult.min_clearance_m). None when
    # no path was planned.
    min_clearance: float | None
    waypoints: list[list[float]]
    # Indices of the colliding samples along the densified path, so a viewer
    # can redraw the exact body boxes the gate rejected.
    collision_indices: list[int]
    unsupported: list[list[float]]
    steep: list[list[float]]


@dataclass
class PlannerArtifacts:
    """Graph state a pipeline chose to expose. Not serialized to JSON."""

    surface_clearance: NDArray[np.float32]
    edges: NDArray[np.float32]


@dataclass
class CaseResult:
    id: str
    dataset: str
    start: tuple[float, float, float]
    goal: tuple[float, float, float]
    tags: list[str]
    l_ref: float
    online_voxels: int
    expect_fail: bool
    online: PlanOutcome
    final: PlanOutcome
    soft_progress: float
    # Scored on the final map only, so there is no distinct online score.
    final_only: bool = False
    # The online plan succeeded but a new obstacle in the final map blocks its
    # route, so the case looks like a dynamic obstacle rather than a bug.
    dynamic_candidate: bool = False
    # Where the online route is blocked by that newly-appeared occupancy.
    blocking_points: list[list[float]] = field(default_factory=list)
    # Planner graph on the incremental map, kept for the rerun recording.
    online_artifacts: PlannerArtifacts | None = None
    # Occupied voxel centers of the incremental map at plan time, kept for the
    # rerun recording.
    online_occupied: NDArray[np.float32] | None = None


@dataclass
class DatasetResult:
    dataset: str
    cases: list[CaseResult]
    final_voxels: int
    map_build_ms: float
    # Per-frame cost of feeding the pipeline, which is what map_update_ms
    # aggregates. The grading mapper's own replay cost is cached and not it.
    add_frame_ms: dict[str, float]
    frames: int
    final_artifacts: PlannerArtifacts | None = None


@dataclass
class TagStats:
    """Aggregate scores over every case carrying a given tag."""

    n: int
    # Cases with an online phase (excludes final-only manual/infeasible cases).
    n_online: int
    inc_score: float
    fin_score: float


@dataclass
class Report:
    score: float
    score_soft: float
    final_score: float
    n_cases: int
    # Cases with an online phase. The incremental score is over these only.
    n_online: int
    n_success: int
    n_success_final: int
    # The incremental and final runs are independent tests per case. These
    # count the four pass/fail combinations.
    outcome_counts: dict[str, int]
    # Score sliced by case tag (stairs, flat, up, down, ...), so a config's
    # effect on each terrain class is visible next to the aggregate.
    by_tag: dict[str, TagStats]
    plan_ms: dict[str, float]
    map_update_ms: dict[str, float]
    datasets: list[DatasetResult]
    # dataset/id of cases whose online route a new final obstacle blocks, the
    # candidates for an expect_final_fail label.
    dynamic_candidates: list[str] = field(default_factory=list)
    config: dict[str, object] = field(default_factory=dict)

    def to_dict(self) -> dict[str, object]:
        out = asdict(self)
        for dataset in out["datasets"]:
            dataset.pop("final_artifacts")
            for case in dataset["cases"]:
                case.pop("online_artifacts")
                case.pop("online_occupied")
        return out


def _run_plan(
    pipeline: NavPipeline,
    case: Case,
    l_ref: float,
    map_keys: NDArray[np.int64],
    support_keys: NDArray[np.int64],
    cfg: EvalConfig,
) -> tuple[PlanOutcome, NDArray[np.float32] | None]:
    t0 = perf_counter()
    waypoints = pipeline.plan(case.start, case.goal)
    plan_ms = (perf_counter() - t0) * 1000
    if waypoints is None or len(waypoints) == 0:
        return _no_plan(plan_ms), None

    reached = metrics.goal_reached(waypoints, case.goal, cfg.goal_tolerance)
    gate = metrics.check_path(waypoints, map_keys, cfg)
    support = metrics.check_support(waypoints, support_keys, cfg)
    kinematics = metrics.check_kinematics(waypoints, cfg)
    length = metrics.path_length(waypoints)
    success = reached and gate.valid and support.valid and kinematics.valid
    outcome = PlanOutcome(
        planned=True,
        reached=reached,
        valid=gate.valid,
        supported=support.valid,
        success=success,
        length=length,
        plan_ms=plan_ms,
        spl=metrics.spl(success, l_ref, length),
        min_clearance=gate.min_clearance_m,
        waypoints=waypoints.tolist(),
        collision_indices=gate.collision_indices[:MAX_COLLISIONS_KEPT].tolist(),
        unsupported=support.unsupported_points[:MAX_COLLISIONS_KEPT].tolist(),
        steep=kinematics.violation_points[:MAX_COLLISIONS_KEPT].tolist(),
    )
    return outcome, waypoints


def _no_plan(plan_ms: float) -> PlanOutcome:
    return PlanOutcome(
        planned=False,
        reached=False,
        valid=False,
        supported=True,
        success=False,
        length=0.0,
        plan_ms=plan_ms,
        spl=0.0,
        min_clearance=None,
        waypoints=[],
        collision_indices=[],
        unsupported=[],
        steep=[],
    )


def score_negative(raw: PlanOutcome) -> PlanOutcome:
    """Invert an outcome for a human-certified infeasible case.

    The planner succeeds by refusing. Any goal-reaching path it returns is a
    false positive scored zero, whether or not the gates would have caught
    it, because the planner claimed a route that does not exist.
    """
    refused = not (raw.planned and raw.reached)
    return replace(raw, success=refused, spl=1.0 if refused else 0.0)


def _dynamic_candidate(
    online: PlanOutcome,
    final: PlanOutcome,
    online_wp: NDArray[np.float32] | None,
    online_keys: NDArray[np.int64],
    final_keys: NDArray[np.int64],
    cfg: EvalConfig,
) -> tuple[bool, list[list[float]]]:
    """Flag a case whose online route is blocked only by new final occupancy.

    An online success with a final failure is either a dynamic obstacle that
    appeared after the robot passed or a planner or mapping bug. Gating the
    online path against the voxels gained since plan time tells them apart. A
    human confirms before labeling the case.
    """
    if online_wp is None or not online.success or final.success:
        return False, []
    # Both come from np.unique, so the sort in setdiff1d is pure waste.
    new_keys = np.setdiff1d(final_keys, online_keys, assume_unique=True)
    if not len(new_keys):
        return False, []
    gate = metrics.check_path(online_wp, new_keys, cfg)
    if gate.valid:
        return False, []
    return True, gate.collision_points[:MAX_COLLISIONS_KEPT].tolist()


def _snapshot(pipeline: NavPipeline) -> PlannerArtifacts | None:
    """Graph layers for the rerun recording, or None from a pipeline that keeps
    its internals to itself."""
    if not isinstance(pipeline, PipelineIntrospection):
        return None
    return PlannerArtifacts(
        surface_clearance=pipeline.surface_clearance_map(),
        edges=pipeline.node_edges(),
    )


def _final_only(case: Case) -> bool:
    """Whether a case is scored on the final map only, with no online phase.

    Hand-placed endpoints are not tied to the recording timeline, so there is
    no meaningful incremental map to replay against. Generated cases keep their
    online phase however their labels are later edited.
    """
    return case.expect_fail or "auto" not in case.tags


def run_suite(suite: Suite, cfg: EvalConfig, keep_artifacts: bool = False) -> DatasetResult:
    db_path = suite.db_path()
    trajectory = load_trajectory(db_path, suite.odom_stream, suite.end_ts_seconds())
    final = load_or_build_final_map(db_path, suite, cfg)
    map_keys = final.occupied_keys

    final_only = np.array([_final_only(c) for c in suite.cases], dtype=bool)
    refs: list[metrics.Reference] = []
    for i, case in enumerate(suite.cases):
        if case.expect_fail:
            # Infeasible by certification: no demonstrated route, no plan time.
            miss = float(np.linalg.norm(np.asarray(case.goal) - np.asarray(case.start)))
            refs.append(metrics.Reference(miss, False, float("inf"), False))
            continue
        ref = metrics.reference_length(trajectory, case.start, case.goal, cfg)
        if not final_only[i]:
            # Only online-replayed cases need a causal snap onto the trajectory.
            if not ref.snapped:
                logger.warning(
                    "%s/%s: start or goal is off the walked trajectory; "
                    "using straight-line reference and the full map",
                    suite.dataset,
                    case.id,
                )
            elif not ref.causal:
                logger.warning(
                    "%s/%s: goal is never visited before the start; planning on the full map",
                    suite.dataset,
                    case.id,
                )
        refs.append(ref)

    # Final-only cases never replay online, so they take no checkpoint and their
    # plan time drops out of the schedule.
    start_ts = np.array(
        [float("inf") if final_only[i] else r.start_ts for i, r in enumerate(refs)],
        dtype=np.float64,
    )
    checkpoints = load_or_build_checkpoints(db_path, suite, cfg, start_ts)
    case_ckpt = np.searchsorted(checkpoints.times, start_ts)
    case_ckpt[final_only] = -1

    pipeline = make_pipeline(cfg.pipeline, cfg)
    results: list[CaseResult | None] = [None] * len(suite.cases)
    online: dict[int, tuple[PlanOutcome, NDArray[np.float32] | None, NDArray[np.int64]]] = {}
    artifacts: dict[int, PlannerArtifacts | None] = {}
    occupied_at_plan: dict[int, NDArray[np.float32] | None] = {}

    def _result(case: Case, ref: metrics.Reference, **rest: object) -> CaseResult:
        """Fill the fields every case copies straight from its case and reference."""
        return CaseResult(
            id=case.id,
            dataset=suite.dataset,
            start=case.start,
            goal=case.goal,
            tags=case.tags,
            l_ref=ref.length,
            **rest,  # type: ignore[arg-type]
        )

    def plan_online(k: int, keys: NDArray[np.int64]) -> None:
        """Plan every case whose start time this checkpoint covers, against the
        pipeline as it stands after the frames seen so far."""
        for ci in np.flatnonzero(case_ckpt == k):
            case, ref = suite.cases[ci], refs[ci]
            if not len(keys):
                online[ci] = (_no_plan(0.0), None, keys)
                continue
            # Collisions are checked against the incremental map the evaluator
            # had at plan time, not the final map. Support still uses the final
            # map, since the ground exists whether or not it was mapped yet.
            outcome, waypoints = _run_plan(pipeline, case, ref.length, keys, map_keys, cfg)
            online[ci] = (outcome, waypoints, keys)
            artifacts[ci] = _snapshot(pipeline) if keep_artifacts else None
            occupied_at_plan[ci] = key_centers(keys, cfg.voxel_size) if keep_artifacts else None

    # One pass. Frames go to the pipeline in recording order and each case is
    # planned at the point in the stream its start time falls, so the pipeline
    # has seen exactly what the robot had seen by then. A pipeline's state
    # cannot be snapshotted from outside, which is why this is sequential.
    snapshots = checkpoints.iter_snapshots()
    add_ms: list[float] = []
    k = 0
    for frame in iter_world_frames(
        db_path, suite.lidar_stream, suite.odom_stream, cfg.align_tol, suite.end_ts_seconds()
    ):
        while k < len(checkpoints.times) and frame.ts > checkpoints.times[k]:
            plan_online(k, next(snapshots))
            k += 1
        t0 = perf_counter()
        pipeline.add_frame(frame.points, frame.origin, frame.ts)
        add_ms.append((perf_counter() - t0) * 1000)
    while k < len(checkpoints.times):
        plan_online(k, next(snapshots))
        k += 1

    # The stream is exhausted, so the pipeline now holds the whole recording.
    final_artifacts = _snapshot(pipeline) if keep_artifacts else None
    for ci, case in enumerate(suite.cases):
        ref = refs[ci]
        final_out, _ = _run_plan(pipeline, case, ref.length, map_keys, map_keys, cfg)
        if case.expect_fail or case.expect_final_fail:
            # Both labels certify that the final map holds no route, so the
            # planner passes by refusing. Applied before the final-only split
            # so a curated case is not scored zero for being right.
            final_out = score_negative(final_out)
        if final_only[ci]:
            results[ci] = _result(
                case,
                ref,
                online_voxels=len(final.occupied),
                expect_fail=case.expect_fail,
                online=final_out,
                final=final_out,
                soft_progress=final_out.spl,
                final_only=True,
            )
            continue
        online_out, online_wp, online_keys = online[ci]
        end = online_wp[-1] if online_wp is not None and len(online_wp) else None
        dynamic_candidate, blocking = (
            (False, [])
            if case.expect_final_fail
            else _dynamic_candidate(online_out, final_out, online_wp, online_keys, map_keys, cfg)
        )
        results[ci] = _result(
            case,
            ref,
            online_voxels=len(online_keys),
            expect_fail=False,
            online=online_out,
            final=final_out,
            soft_progress=metrics.soft_progress(end, case.start, case.goal),
            dynamic_candidate=dynamic_candidate,
            blocking_points=blocking,
            online_artifacts=artifacts.get(ci),
            online_occupied=occupied_at_plan.get(ci),
        )

    done = [r for r in results if r is not None]
    if len(done) != len(suite.cases):
        raise RuntimeError(f"{suite.dataset}: {len(suite.cases) - len(done)} cases not planned")
    return DatasetResult(
        dataset=suite.dataset,
        cases=done,
        final_voxels=len(final.occupied),
        map_build_ms=final.build_ms,
        add_frame_ms=metrics.timing_stats(add_ms),
        frames=len(add_ms),
        final_artifacts=final_artifacts,
    )


def evaluate(
    suites: list[Suite],
    cfg: EvalConfig | None = None,
    workers: int = 1,
    keep_artifacts: bool = False,
) -> Report:
    """Score every suite. A dataset is one sequential pass over its recording,
    so workers only spreads datasets across processes. keep_artifacts snapshots
    each pipeline's graph for the rerun recording."""
    cfg = cfg or EvalConfig()
    if workers > 1 and len(suites) > 1:
        with ProcessPoolExecutor(max_workers=min(workers, len(suites))) as pool:
            datasets = list(
                pool.map(run_suite, suites, itertools.repeat(cfg), itertools.repeat(keep_artifacts))
            )
    else:
        datasets = [run_suite(suite, cfg, keep_artifacts=keep_artifacts) for suite in suites]
    cases = [c for d in datasets for c in d.cases]
    if not cases:
        raise ValueError("no cases to evaluate")

    # Manual and infeasible cases have no online phase, so every incremental
    # aggregate is over the online cases only. Final aggregates cover them all.
    online = [c for c in cases if not c.final_only]

    def mean(values: list[float]) -> float:
        return float(np.mean(values)) if values else 0.0

    outcome_names = {
        (True, True): "both",
        (False, True): "final_only",
        (True, False): "incremental_only",
        (False, False): "neither",
    }
    outcome_counts = dict.fromkeys(outcome_names.values(), 0)
    for c in online:
        outcome_counts[outcome_names[c.online.success, c.final.success]] += 1

    by_tag: dict[str, TagStats] = {}
    for tag in sorted({t for c in cases for t in c.tags}):
        tc = [c for c in cases if tag in c.tags]
        oc = [c for c in tc if not c.final_only]
        by_tag[tag] = TagStats(
            n=len(tc),
            n_online=len(oc),
            inc_score=mean([c.online.spl for c in oc]),
            fin_score=mean([c.final.spl for c in tc]),
        )

    return Report(
        score=mean([c.online.spl for c in online]),
        score_soft=mean(
            [c.soft_progress if not c.online.success else c.online.spl for c in online]
        ),
        final_score=mean([c.final.spl for c in cases]),
        n_cases=len(cases),
        n_online=len(online),
        n_success=sum(c.online.success for c in online),
        n_success_final=sum(c.final.success for c in cases),
        outcome_counts=outcome_counts,
        by_tag=by_tag,
        plan_ms=metrics.timing_stats([c.online.plan_ms for c in online]),
        # Worst dataset's per-frame ingest cost: the budget asks whether any
        # pipeline failed to keep up with the sensor, not what the average was.
        map_update_ms={
            k: max(d.add_frame_ms.get(k, 0.0) for d in datasets) for k in ("p50", "p95", "max")
        },
        datasets=datasets,
        dynamic_candidates=[f"{c.dataset}/{c.id}" for c in cases if c.dynamic_candidate],
        config=asdict(cfg),
    )

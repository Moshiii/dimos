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

"""Replay case suites through a pipeline and score them. Generated cases plan
twice, online at their start time and again on the whole recording."""

from __future__ import annotations

from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict, dataclass, field, replace
import itertools
import multiprocessing
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
from dimos.navigation.nav_3d.evaluator.voxel_keys import key_centers
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from numpy.typing import NDArray

    from dimos.navigation.nav_3d.evaluator.cases import Case, Suite
    from dimos.navigation.nav_3d.evaluator.final_map import FinalMap, MapCheckpoints
    from dimos.navigation.nav_3d.evaluator.pipeline import NavPipeline
    from dimos.navigation.nav_3d.evaluator.recording import Trajectory

logger = setup_logger()

MAX_COLLISIONS_KEPT = 50


@dataclass
class PlanOutcome:
    planned: bool
    reached: bool
    valid: bool
    # Every sample stands on final-map occupancy. Fabricated bridges fail.
    supported: bool
    # All of the above, or for an infeasible case, that the planner refused.
    success: bool
    length: float
    plan_ms: float
    spl: float
    # Gate margin along the path. None when no path was planned.
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
    """Invert an outcome for a human-certified infeasible case: the planner
    succeeds by refusing, and any goal-reaching path it returns scores zero."""
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
    """Flag a case whose online route is blocked only by occupancy gained
    since plan time, which separates a dynamic obstacle from a planner bug."""
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
    """Whether a case is scored on the final map only. Hand-placed endpoints
    are not tied to the recording timeline, so they have no incremental map."""
    return case.expect_fail or "auto" not in case.tags


def _references(
    suite: Suite, trajectory: Trajectory, final_only: NDArray[np.bool_], cfg: EvalConfig
) -> list[metrics.Reference]:
    """The demonstrated route length each case is scored against."""
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
                    "endpoint off the walked trajectory, using a straight-line reference",
                    dataset=suite.dataset,
                    case=case.id,
                )
            elif not ref.causal:
                logger.warning(
                    "goal never visited before the start, planning on the full map",
                    dataset=suite.dataset,
                    case=case.id,
                )
        refs.append(ref)
    return refs


@dataclass
class _OnlineCase:
    """One case's online plan and the map it was planned against."""

    outcome: PlanOutcome
    waypoints: NDArray[np.float32] | None
    map_keys: NDArray[np.int64]
    artifacts: PlannerArtifacts | None = None
    occupied: NDArray[np.float32] | None = None


@dataclass
class _OnlinePass:
    """Everything the single replay pass produced, keyed by case index."""

    cases: dict[int, _OnlineCase]
    add_ms: list[float]
    final_artifacts: PlannerArtifacts | None


def _replay_online(
    pipeline: NavPipeline,
    suite: Suite,
    cfg: EvalConfig,
    checkpoints: MapCheckpoints,
    case_ckpt: NDArray[np.int64],
    refs: list[metrics.Reference],
    map_keys: NDArray[np.int64],
    keep_artifacts: bool,
) -> _OnlinePass:
    """Feed the recording to the pipeline once, planning each case where its
    start time falls, so the pipeline has seen what the robot had seen by then."""
    out = _OnlinePass({}, [], None)

    def plan_at(k: int, keys: NDArray[np.int64]) -> None:
        for ci in np.flatnonzero(case_ckpt == k):
            case, ref = suite.cases[ci], refs[ci]
            if not len(keys):
                out.cases[ci] = _OnlineCase(_no_plan(0.0), None, keys)
                continue
            # Collisions use the incremental map as of plan time. Support uses
            # the final map, since ground exists whether or not it was mapped.
            outcome, waypoints = _run_plan(pipeline, case, ref.length, keys, map_keys, cfg)
            out.cases[ci] = _OnlineCase(
                outcome,
                waypoints,
                keys,
                artifacts=_snapshot(pipeline) if keep_artifacts else None,
                occupied=key_centers(keys, cfg.voxel_size) if keep_artifacts else None,
            )

    snapshots = checkpoints.iter_snapshots()
    k = 0
    for frame in suite.world_frames(cfg.align_tol):
        while k < len(checkpoints.times) and frame.ts > checkpoints.times[k]:
            plan_at(k, next(snapshots))
            k += 1
        t0 = perf_counter()
        pipeline.add_frame(frame.points, frame.origin, frame.ts)
        out.add_ms.append((perf_counter() - t0) * 1000)
    while k < len(checkpoints.times):
        plan_at(k, next(snapshots))
        k += 1

    # The stream is exhausted, so the pipeline now holds the whole recording.
    out.final_artifacts = _snapshot(pipeline) if keep_artifacts else None
    return out


def _score_final(
    pipeline: NavPipeline,
    suite: Suite,
    cfg: EvalConfig,
    refs: list[metrics.Reference],
    final: FinalMap,
    final_only: NDArray[np.bool_],
    online: _OnlinePass,
) -> list[CaseResult]:
    """Plan every case against the completed map and combine both phases."""
    map_keys = final.occupied_keys
    results: list[CaseResult] = []
    for ci, case in enumerate(suite.cases):
        ref = refs[ci]
        final_out, _ = _run_plan(pipeline, case, ref.length, map_keys, map_keys, cfg)
        if case.expect_fail or case.expect_final_fail:
            # Both labels certify the final map holds no route, so the planner
            # passes by refusing. Before the split, so a curated case scores.
            final_out = score_negative(final_out)
        if final_only[ci]:
            results.append(
                CaseResult(
                    id=case.id,
                    dataset=suite.dataset,
                    start=case.start,
                    goal=case.goal,
                    tags=case.tags,
                    l_ref=ref.length,
                    online_voxels=len(final.occupied),
                    expect_fail=case.expect_fail,
                    online=final_out,
                    final=final_out,
                    soft_progress=final_out.spl,
                    final_only=True,
                )
            )
            continue
        run = online.cases[ci]
        end = run.waypoints[-1] if run.waypoints is not None and len(run.waypoints) else None
        dynamic_candidate, blocking = (
            (False, [])
            if case.expect_final_fail
            else _dynamic_candidate(
                run.outcome, final_out, run.waypoints, run.map_keys, map_keys, cfg
            )
        )
        results.append(
            CaseResult(
                id=case.id,
                dataset=suite.dataset,
                start=case.start,
                goal=case.goal,
                tags=case.tags,
                l_ref=ref.length,
                online_voxels=len(run.map_keys),
                expect_fail=False,
                online=run.outcome,
                final=final_out,
                soft_progress=metrics.soft_progress(end, case.start, case.goal),
                dynamic_candidate=dynamic_candidate,
                blocking_points=blocking,
                online_artifacts=run.artifacts,
                online_occupied=run.occupied,
            )
        )
    return results


def run_suite(suite: Suite, cfg: EvalConfig, keep_artifacts: bool = False) -> DatasetResult:
    db_path = suite.db_path()
    if not db_path.exists():
        # sqlite would otherwise create an empty db here and the failure would
        # surface as a missing odometry stream.
        raise FileNotFoundError(f"{suite.dataset}: recording not found at {db_path}")
    trajectory = suite.trajectory()

    final_only = np.array([_final_only(c) for c in suite.cases], dtype=bool)
    refs = _references(suite, trajectory, final_only, cfg)
    # Final-only cases never replay online, so they take no checkpoint and their
    # plan time drops out of the schedule.
    start_ts = np.array(
        [float("inf") if final_only[i] else r.start_ts for i, r in enumerate(refs)],
        dtype=np.float64,
    )
    # Before the final map, because a cold checkpoint build replays the whole
    # recording and fills the final cache on its way through.
    checkpoints = load_or_build_checkpoints(suite, cfg, start_ts)
    final = load_or_build_final_map(suite, cfg)
    case_ckpt = np.searchsorted(checkpoints.times, start_ts)
    case_ckpt[final_only] = -1

    pipeline = make_pipeline(cfg.pipeline, cfg)
    online = _replay_online(
        pipeline,
        suite,
        cfg,
        checkpoints,
        case_ckpt,
        refs,
        final.occupied_keys,
        keep_artifacts,
    )
    return DatasetResult(
        dataset=suite.dataset,
        cases=_score_final(pipeline, suite, cfg, refs, final, final_only, online),
        final_voxels=len(final.occupied),
        map_build_ms=final.build_ms,
        add_frame_ms=metrics.timing_stats(online.add_ms),
        frames=len(online.add_ms),
        final_artifacts=online.final_artifacts,
    )


def evaluate(
    suites: list[Suite],
    cfg: EvalConfig | None = None,
    workers: int = 1,
    keep_artifacts: bool = False,
) -> Report:
    """Score every suite. A dataset is one sequential pass over its recording,
    so workers only spreads datasets across processes."""
    cfg = cfg or EvalConfig()
    if workers > 1 and len(suites) > 1:
        # Spawn, not fork: the mapper and planner carry a native thread pool, and
        # a forked child inherits its mutexes locked if the parent ever built one.
        with ProcessPoolExecutor(
            max_workers=min(workers, len(suites)),
            mp_context=multiprocessing.get_context("spawn"),
        ) as pool:
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
        tagged = [c for c in cases if tag in c.tags]
        online_tagged = [c for c in tagged if not c.final_only]
        by_tag[tag] = TagStats(
            n=len(tagged),
            n_online=len(online_tagged),
            inc_score=mean([c.online.spl for c in online_tagged]),
            fin_score=mean([c.final.spl for c in tagged]),
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

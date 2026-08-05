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

from __future__ import annotations

from dataclasses import replace
import itertools
from pathlib import Path
from typing import TYPE_CHECKING, cast

import numpy as np
from numpy.typing import NDArray
import pytest
import typer

from dimos.memory2.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.nav_3d.evaluator import final_map, metrics, runner
from dimos.navigation.nav_3d.evaluator.cases import (
    Case,
    Suite,
    load_suite,
    save_suite,
)
from dimos.navigation.nav_3d.evaluator.cli import _apply_overrides
from dimos.navigation.nav_3d.evaluator.config import EvalConfig
from dimos.navigation.nav_3d.evaluator.curation import CaseStore, CurationError, _curated_tags
from dimos.navigation.nav_3d.evaluator.final_map import (
    FinalMap,
    MapCheckpoints,
    _save_npz,
    encode_deltas,
    replay_frames,
)
from dimos.navigation.nav_3d.evaluator.generate import (
    Candidate,
    _select_diverse,
    generate_cases,
    snap_to_surface,
)
from dimos.navigation.nav_3d.evaluator.picker import pick_along_ray
from dimos.navigation.nav_3d.evaluator.pipeline import PIPELINES
from dimos.navigation.nav_3d.evaluator.recording import (
    Frame,
    Trajectory,
    iter_world_frames,
    load_trajectory,
)
from dimos.navigation.nav_3d.evaluator.runner import (
    _dynamic_candidate,
    _final_only,
    _run_plan,
    score_negative,
)
from dimos.navigation.nav_3d.evaluator.tagging import retag_suite, route_tags
from dimos.navigation.nav_3d.evaluator.voxel_keys import (
    cylinder_offsets,
    keys_contain,
    offset_deltas,
    voxel_keys,
)

if TYPE_CHECKING:
    from dimos.mapping.ray_tracing.voxel_map import VoxelRayMapper
    from dimos.navigation.nav_3d.evaluator.pipeline import NavPipeline

VOXEL = 0.1
Point = tuple[float, float, float]


def _cfg(**overrides: object) -> EvalConfig:
    return replace(EvalConfig(voxel_size=VOXEL), **overrides)


def _wall(x: float) -> NDArray[np.float32]:
    ys, zs = np.meshgrid(np.arange(-1, 1, VOXEL), np.arange(0.05, 1.5, VOXEL))
    return np.stack([np.full(ys.size, x), ys.ravel(), zs.ravel()], axis=1, dtype=np.float32)


def _ywall(y: float, x_lo: float, x_hi: float) -> NDArray[np.float32]:
    """A wall parallel to +x travel, at constant y, spanning body height."""
    xs, zs = np.meshgrid(np.arange(x_lo, x_hi, VOXEL), np.arange(0.05, 1.5, VOXEL))
    return np.stack([xs.ravel(), np.full(xs.size, y), zs.ravel()], axis=1, dtype=np.float32)


def _gate(waypoints: NDArray[np.float32], obstacles: NDArray[np.float32]) -> metrics.GateResult:
    keys = np.unique(voxel_keys(obstacles, VOXEL))
    return metrics.check_path(waypoints, keys, _cfg())


def test_gate_blocks_wall_crossing() -> None:
    path = np.array([[-1, 0, 0], [1, 0, 0]], dtype=np.float32)
    result = _gate(path, _wall(0.0))
    assert not result.valid
    assert len(result.collision_points) > 0
    # Collisions fall within the box half-length (0.35) of the wall.
    assert np.all(np.abs(result.collision_points[:, 0]) < 0.45)


def test_gate_box_uses_travel_orientation() -> None:
    """The body box is long along travel (0.7) and narrow across it (0.31)."""
    path = np.array([[-0.5, 0, 0], [0.5, 0, 0]], dtype=np.float32)
    # 0.25 m ahead along travel is inside the 0.35 m half-length.
    ahead = np.array([[0.25, 0.0, 0.35]], dtype=np.float32)
    assert not _gate(path, ahead).valid
    # The same 0.25 m offset to the side is outside the 0.155 m half-width.
    beside = np.array([[0.0, 0.25, 0.35]], dtype=np.float32)
    assert _gate(path, beside).valid


def test_gate_ignores_ground() -> None:
    xs, ys = np.meshgrid(np.arange(-2, 2, VOXEL), np.arange(-2, 2, VOXEL))
    floor = np.stack([xs.ravel(), ys.ravel(), np.full(xs.size, -0.05)], axis=1, dtype=np.float32)
    path = np.array([[-1, 0, 0], [1, 0, 0]], dtype=np.float32)
    assert _gate(path, floor).valid


def test_gate_pitch_clears_rising_step() -> None:
    """A voxel ahead-and-up is inside a flat box but beyond the pitched one."""
    step = np.array([[0.3, 0.0, 0.4]], dtype=np.float32)
    # Level travel: the step sits in the horizontal body band and collides.
    assert not _gate(np.array([[0, 0, 0], [1, 0, 0]], dtype=np.float32), step).valid
    # Climbing at 45 degrees: the box pitches up, so the same voxel falls beyond
    # the tilted body and clears.
    assert _gate(np.array([[0, 0, 0], [1, 0, 1]], dtype=np.float32), step).valid


def test_gate_tolerates_stair_slope() -> None:
    """Terrain rising at stair slope inside the body box must not trigger the gate."""
    xs, ys = np.meshgrid(np.arange(-1, 2, VOXEL), np.arange(-1, 1, VOXEL))
    slope = np.stack([xs.ravel(), ys.ravel(), xs.ravel() * 0.7 - 0.05], axis=1, dtype=np.float32)
    path = np.stack(
        [np.arange(-0.5, 1.5, 0.1), np.zeros(20), np.arange(-0.5, 1.5, 0.1) * 0.7], axis=1
    ).astype(np.float32)
    assert _gate(path, slope).valid


def test_gate_reports_clearance_margin() -> None:
    wall = _wall(10.0)
    graze = np.array([[9.7, -0.5, 0], [9.7, 0.5, 0]], dtype=np.float32)
    result = _gate(graze, wall)
    assert result.valid
    # Travel is +y here, so the wall 0.35 m away in x sits off the box's
    # 0.155 m half-width.
    assert result.min_clearance_m == pytest.approx(0.35 - 0.155, abs=0.02)
    crossing = _gate(np.array([[9, 0, 0], [11, 0, 0]], dtype=np.float32), wall)
    assert not crossing.valid
    assert crossing.min_clearance_m < 0
    far = _gate(np.array([[2, 0, 0], [3, 0, 0]], dtype=np.float32), wall)
    assert far.min_clearance_m == metrics.MARGIN_CAP_M


def test_reference_length_snaps_to_trajectory() -> None:
    positions = np.stack(
        [np.linspace(0, 10, 101), np.zeros(101), np.full(101, 0.3)], axis=1
    ).astype(np.float32)
    traj = Trajectory(ts=np.linspace(0, 10, 101), positions=positions)
    # Walking toward a never-yet-visited goal is not causal.
    ref = metrics.reference_length(traj, (0, 0, 0), (10, 0, 0), _cfg())
    assert ref.snapped
    assert ref.length == pytest.approx(10.0, abs=0.01)
    assert not ref.causal
    assert ref.start_ts == float("inf")
    # Returning to the walk's origin is causal.
    ref = metrics.reference_length(traj, (10, 0, 0), (0, 0, 0), _cfg())
    assert ref.causal
    assert 9.0 <= ref.start_ts <= 10.0
    ref = metrics.reference_length(traj, (0, 5, 0), (10, 0, 0), _cfg())
    assert not ref.snapped
    assert ref.start_ts == float("inf")


def test_reference_length_uses_shortest_revisit() -> None:
    """An out-and-back trajectory must not inflate the reference with the loop."""
    out = np.stack([np.linspace(0, 10, 101), np.zeros(101), np.full(101, 0.3)], axis=1)
    detour = np.stack([np.full(101, 10.0), np.linspace(0, 30, 101), np.full(101, 0.3)], axis=1)
    back = detour[::-1]
    positions = np.concatenate([out, detour, back]).astype(np.float32)
    traj = Trajectory(ts=np.linspace(0, 30, len(positions)), positions=positions)
    ref = metrics.reference_length(traj, (0, 0, 0), (10, 0, 0), _cfg())
    assert ref.snapped
    assert ref.length == pytest.approx(10.0, abs=0.2)


def test_checkpoint_deltas_roundtrip() -> None:
    snapshots = [
        np.array([1, 2, 3], dtype=np.int64),
        np.array([2, 3, 4, 5], dtype=np.int64),
        np.array([4, 5], dtype=np.int64),
    ]
    added, removed = encode_deltas(snapshots)
    ckpt = MapCheckpoints(times=np.arange(3, dtype=np.float64), added=added, removed=removed)
    for original, keys in zip(snapshots, ckpt.iter_snapshots(), strict=True):
        assert np.array_equal(original, keys)


class _StubMapper:
    """Keeps every point it is handed. replay_frames only calls these two."""

    def __init__(self) -> None:
        self._points: list[NDArray[np.float32]] = []

    def add_frame(self, points: NDArray[np.float32], origin: tuple[float, float, float]) -> None:
        self._points.append(points)

    def global_map(self) -> NDArray[np.float32]:
        if not self._points:
            return np.zeros((0, 3), dtype=np.float32)
        return np.concatenate(self._points)


def test_replay_frames_snapshots_grow_with_time() -> None:
    """Each checkpoint must contain exactly the frames seen up to its time."""
    mapper = cast("VoxelRayMapper", _StubMapper())

    def frame_at(ts: float, x: float) -> Frame:
        return Frame(ts=ts, points=_wall(x), origin=(x - 2.0, 0.0, 0.5))

    frames = [frame_at(0.0, 5.0), frame_at(1.0, 8.0), frame_at(2.0, 11.0)]
    times = np.array([0.5, 1.5, np.inf])
    final, snapshots = replay_frames(frames, mapper, VOXEL, times)
    sizes = [len(s) for s in snapshots]
    assert 0 < sizes[0] < sizes[1] < sizes[2]
    assert np.array_equal(snapshots[2], final.occupied_keys)
    for earlier, later in itertools.pairwise(snapshots):
        assert keys_contain(later, earlier).all()
    # Each checkpoint holds the walls mapped by its time: wall 1 by the first,
    # wall 3 only at the end.
    wall1 = np.unique(voxel_keys(_wall(5.0), VOXEL))
    wall3 = np.unique(voxel_keys(_wall(11.0), VOXEL))
    assert keys_contain(snapshots[0], wall1).all()
    assert not keys_contain(snapshots[0], wall3).any()
    assert keys_contain(snapshots[2], wall3).all()


def test_check_kinematics_rejects_cliff_jumps() -> None:
    stairs = np.array([[0, 0, 0], [0.4, 0, 0.16], [0.8, 0, 0.32]], dtype=np.float32)
    assert metrics.check_kinematics(stairs, _cfg(max_slope=1.0)).valid
    riser = np.array([[0, 0, 0], [0.08, 0, 0.16]], dtype=np.float32)
    assert metrics.check_kinematics(riser, _cfg(max_slope=1.0)).valid
    # A double riser between adjacent cells is quantization, not a cliff.
    quantized = np.array(
        [[0, 0, 0], [0.4, 0, 0.08], [0.56, 0, 0.4], [0.96, 0, 0.48]], dtype=np.float32
    )
    assert metrics.check_kinematics(quantized, _cfg(max_slope=1.0)).valid
    cliff = np.array([[0, 0, 0], [0.2, 0, 0.9], [1, 0, 0.9]], dtype=np.float32)
    result = metrics.check_kinematics(cliff, _cfg(max_slope=1.0))
    assert not result.valid
    assert len(result.violation_points) >= 1


class _StubPipeline:
    """Returns a fixed path regardless of what it was fed, for gaming the scorer."""

    def __init__(self, waypoints: NDArray[np.float32] | None) -> None:
        self._waypoints = waypoints
        self.frames = 0

    def add_frame(
        self, points: NDArray[np.float32], origin: tuple[float, float, float], ts: float
    ) -> None:
        self.frames += 1

    def plan(
        self, start: tuple[float, float, float], goal: tuple[float, float, float]
    ) -> NDArray[np.float32] | None:
        return self._waypoints


def _stub(waypoints: NDArray[np.float32] | None) -> NavPipeline:
    return cast("NavPipeline", _StubPipeline(waypoints))


def _floor(x_lo: float = 0.0, x_hi: float = 20.0) -> NDArray[np.float32]:
    xs, ys = np.meshgrid(np.arange(x_lo, x_hi, VOXEL), np.arange(-2, 6, VOXEL))
    return np.stack([xs.ravel(), ys.ravel(), np.full(xs.size, -0.05)], axis=1, dtype=np.float32)


def _meta_scene() -> tuple[np.ndarray, EvalConfig, Case]:
    """A floored corridor with a wall at x=10 between x=2 and x=18."""
    scene = np.concatenate([_floor(), _wall(10.0)])
    keys = np.unique(voxel_keys(scene, VOXEL))
    cfg = EvalConfig(voxel_size=VOXEL)
    case = Case(id="meta", start=(2.0, 0.0, 0.0), goal=(18.0, 0.0, 0.0))
    return keys, cfg, case


def _u_route() -> NDArray[np.float32]:
    return np.array([[2, 0, 0], [2, 4, 0], [18, 4, 0], [18, 0, 0]], dtype=np.float32)


def test_meta_straight_line_cheat_scores_zero() -> None:
    """A planner that ignores the map and beelines must not score."""
    keys, cfg, case = _meta_scene()
    line = np.array([case.start, case.goal], dtype=np.float32)
    out = _run_plan(_stub(line), case, 24.0, keys, keys, cfg)
    assert out.planned and out.reached and out.supported
    assert not out.valid
    assert out.spl == 0.0
    assert out.min_clearance is not None and out.min_clearance < 0


def test_meta_no_path_scores_zero() -> None:
    keys, cfg, case = _meta_scene()
    out = _run_plan(_stub(None), case, 24.0, keys, keys, cfg)
    assert not out.planned
    assert out.spl == 0.0
    assert out.min_clearance is None


def test_meta_demonstrated_route_scores_full() -> None:
    """The route the robot actually walked must earn full SPL."""
    keys, cfg, case = _meta_scene()
    route = _u_route()
    l_ref = metrics.path_length(route)
    out = _run_plan(_stub(route), case, l_ref, keys, keys, cfg)
    assert out.success
    assert out.spl == pytest.approx(1.0)
    assert out.min_clearance == metrics.MARGIN_CAP_M


def test_meta_floating_bridge_fails_support() -> None:
    """A path across a floor gap collides with nothing but must still fail."""
    gapped = np.concatenate([_floor(0.0, 6.0), _floor(14.0, 20.0)])
    keys = np.unique(voxel_keys(gapped, VOXEL))
    _, cfg, case = _meta_scene()
    line = np.array([case.start, case.goal], dtype=np.float32)
    out = _run_plan(_stub(line), case, 24.0, keys, keys, cfg)
    assert out.planned and out.reached and out.valid
    assert not out.supported
    assert out.spl == 0.0
    assert out.unsupported
    gap_x = np.asarray(out.unsupported, dtype=np.float32)[:, 0]
    assert gap_x.min() > 5.5 and gap_x.max() < 14.5


def test_meta_negative_case_scoring() -> None:
    """A certified-infeasible case scores 1.0 for refusal, 0.0 for any claim."""
    keys, cfg, case = _meta_scene()
    refused = _run_plan(_stub(None), case, 16.0, keys, keys, cfg)
    out = score_negative(refused)
    assert out.success
    assert out.spl == 1.0
    claimed = _run_plan(_stub(_u_route()), case, 16.0, keys, keys, cfg)
    out = score_negative(claimed)
    assert not out.success
    assert out.spl == 0.0
    # A path that wanders but never reaches the goal is still a refusal.
    wander = np.array([case.start, [4.0, 2.0, 0.0]], dtype=np.float32)
    partial = _run_plan(_stub(wander), case, 16.0, keys, keys, cfg)
    assert score_negative(partial).success


def test_dynamic_candidate_flags_route_blocked_by_new_occupancy() -> None:
    """Online success with a final failure from newly-appeared occupancy flags."""
    _, cfg, case = _meta_scene()
    open_keys = np.unique(voxel_keys(_floor(), VOXEL))
    final_keys = np.unique(voxel_keys(np.concatenate([_floor(), _wall(10.0)]), VOXEL))
    line = np.array([case.start, case.goal], dtype=np.float32)

    online = _run_plan(_stub(line), case, 24.0, open_keys, open_keys, cfg)
    assert online.success
    final = _run_plan(_stub(line), case, 24.0, final_keys, final_keys, cfg)
    assert not final.success

    flagged, blocking = _dynamic_candidate(online, final, line, open_keys, final_keys, cfg)
    assert flagged
    assert blocking

    # No occupancy appeared between the two maps, so the final failure is not a
    # dynamic obstacle and must not be flagged.
    unflagged, _ = _dynamic_candidate(online, final, line, final_keys, final_keys, cfg)
    assert not unflagged

    # A clean final plan is never a candidate.
    assert not _dynamic_candidate(online, online, line, open_keys, final_keys, cfg)[0]


def test_chord_direction_spans_robot_length() -> None:
    """Heading comes from the body-length chord, not the local step, so it is
    steady across stepped terrain instead of flipping tread-to-riser."""
    xs = np.arange(0, 3.0, 0.1)
    zs = np.floor(xs / 0.2) * 0.1  # stairs: 0.1 m rise every 0.2 m, mean slope 0.5
    path = np.stack([xs, np.zeros_like(xs), zs], axis=1).astype(np.float32)
    fwd = metrics.chord_directions(path, span=0.7)
    local = np.diff(path.astype(np.float64), axis=0)
    local /= np.linalg.norm(local, axis=1, keepdims=True)
    assert fwd[5:-5, 2].std() < 0.5 * local[:, 2].std()
    assert fwd[5:-5, 2].mean() > 0.2  # steadily pitched up the stairs


def test_ground_truth_route_returns_walked_slice() -> None:
    """The route is the shortest walked slice between the endpoints, not a line."""
    out = np.stack([np.linspace(0, 10, 101), np.zeros(101), np.full(101, 0.3)], axis=1)
    detour = np.stack([np.full(101, 10.0), np.linspace(0, 6, 101), np.full(101, 0.3)], axis=1)
    positions = np.concatenate([out, detour]).astype(np.float32)
    traj = Trajectory(ts=np.linspace(0, 20, len(positions)), positions=positions)
    route = metrics.ground_truth_route(traj, (0, 0, 0), (10, 6, 0), _cfg())
    assert route is not None
    # Foot level (sensor height removed) and it walks the full out-and-detour.
    assert abs(route[0, 2]) < 1e-5
    assert metrics.path_length(route) == pytest.approx(16.0, abs=0.2)
    assert metrics.ground_truth_route(traj, (0, 20, 0), (10, 6, 0), _cfg()) is None


def test_ground_truth_route_orients_start_to_goal() -> None:
    """The route runs start-to-goal even when the start was walked later, so its
    elevation is not read backward."""
    xs = np.linspace(0, 10, 101)
    positions = np.stack([xs, np.zeros(101), xs * 0.25], axis=1).astype(np.float32)
    traj = Trajectory(ts=np.linspace(0, 10, 101), positions=positions)
    # Start high at x=8, goal low at x=2: a downhill traverse.
    route = metrics.ground_truth_route(traj, (8, 0, 1.7), (2, 0, 0.2), _cfg())
    assert route is not None
    assert route[0, 0] > 6.0 and route[-1, 0] < 4.0
    assert route[-1, 2] < route[0, 2]


def _tags(route: NDArray[np.float32], keys: np.ndarray) -> list[str]:
    """Tag a synthetic route, taking its endpoints for elevation."""
    start = (float(route[0, 0]), float(route[0, 1]), float(route[0, 2]))
    goal = (float(route[-1, 0]), float(route[-1, 1]), float(route[-1, 2]))
    return route_tags(start, goal, route, keys, _cfg())


def test_route_tags_narrow_passage() -> None:
    """Walls under a body-plus-clearance apart the whole way make a corridor."""
    route = np.array([[0, 0, 0], [4, 0, 0]], dtype=np.float32)
    walls = np.concatenate([_ywall(-0.4, 0, 4), _ywall(0.4, 0, 4)])
    keys = np.unique(voxel_keys(walls, VOXEL))
    tags = _tags(route, keys)
    assert "narrow" in tags and "corridor" in tags
    assert "doorway" not in tags  # sustained squeeze, not a short pinch


def test_route_tags_doorway_is_a_short_pinch() -> None:
    """An open corridor that pinches and reopens is a doorway, down to a pinch
    only a couple of voxels long."""
    route = np.array([[0, 0, 0], [5, 0, 0]], dtype=np.float32)
    far = np.concatenate([_ywall(-2.0, 0, 5), _ywall(2.0, 0, 5)])
    for x_lo, x_hi in ((2.0, 2.6), (2.4, 2.6)):
        pinch = np.concatenate([_ywall(-0.35, x_lo, x_hi), _ywall(0.35, x_lo, x_hi)])
        keys = np.unique(voxel_keys(np.concatenate([far, pinch]), VOXEL))
        tags = _tags(route, keys)
        assert "doorway" in tags and "narrow" in tags


def test_route_tags_stairs_from_endpoints() -> None:
    """Elevation comes from the endpoints: a climb past the threshold is stairs,
    and a big rise is long, whatever the route in between does."""
    up = np.stack([np.linspace(0, 4, 40), np.zeros(40), np.linspace(0, 2.0, 40)], axis=1).astype(
        np.float32
    )
    tags = _tags(up, np.array([], dtype=np.int64))
    assert "stairs" in tags and "up" in tags and "long" in tags
    assert "down" in _tags(up[::-1].copy(), np.array([], dtype=np.int64))


def test_route_tags_gate_excludes_detour_routes() -> None:
    """Shape tags need a near-direct route. Between the same endpoints, a
    straight walk through the corridor is tagged, but a long detour is not:
    its terrain cannot be attributed to the case, so it gets elevation only."""
    walls = np.concatenate([_ywall(-0.4, 0, 20), _ywall(0.4, 0, 20)])
    keys = np.unique(voxel_keys(walls, VOXEL))
    direct = np.array([[0, 0, 0], [4, 0, 0]], dtype=np.float32)
    assert "corridor" in route_tags((0.0, 0.0, 0.0), (4.0, 0.0, 0.0), direct, keys, _cfg())
    # The detour runs the same corridor, so its terrain would tag identically.
    # Only the arc-length gate can tell the two apart.
    detour = np.array([[0, 0, 0], [20, 0, 0], [4, 0, 0]], dtype=np.float32)
    assert route_tags((0.0, 0.0, 0.0), (4.0, 0.0, 0.0), detour, keys, _cfg()) == ["flat"]


def test_retag_recomputes_geometry_and_leaves_the_rest_alone() -> None:
    """A retag rewrites an auto case's geometric tags from the map as it stands
    now, keeps its provenance, and skips curated and off-trajectory cases."""
    positions = np.column_stack([np.linspace(0, 10, 101), np.zeros(101), np.full(101, 0.3)]).astype(
        np.float32
    )
    traj = Trajectory(ts=np.linspace(0, 20, 101), positions=positions)
    suite = Suite(
        dataset="demo",
        cases=[
            Case(id="auto_00", start=(1, 0, 0), goal=(9, 0, 0), tags=["auto", "stairs", "up"]),
            Case(id="manual_00", start=(1, 0, 0), goal=(9, 0, 0), tags=["manual", "stairs"]),
            Case(id="auto_01", start=(1, 40, 0), goal=(9, 40, 0), tags=["auto", "flat"]),
        ],
    )
    recomputed = retag_suite(suite, traj, np.array([], dtype=np.int64), _cfg())

    # The stale climb tags go, the walk is flat, and "auto" survives as provenance.
    assert recomputed == {"auto_00": ["auto", "flat"]}
    assert [c.tags for c in suite.cases] == [
        ["auto", "stairs", "up"],
        ["manual", "stairs"],
        ["auto", "flat"],
    ]


def _final_map(points: np.ndarray) -> FinalMap:
    return FinalMap(
        voxel_size=VOXEL,
        occupied=points,
        occupied_keys=np.unique(voxel_keys(points, VOXEL)),
        build_ms=0.0,
    )


def _surface_keys(points: NDArray[np.float32], robot_height: float = 0.3) -> NDArray[np.float32]:
    return np.unique(voxel_keys(_final_map(points).standable_surface(robot_height), VOXEL))


def test_standable_surface_is_the_top_of_each_column() -> None:
    """A wall contributes its cap and not its face, and the floor it stands on
    stops being standable."""
    floor, wall = _floor(0.0, 2.0), _wall(1.0)
    keys = _surface_keys(np.concatenate([floor, wall]))

    def probe(point: np.ndarray) -> bool:
        return bool(keys_contain(keys, voxel_keys(point.reshape(1, 3), VOXEL))[0])

    # Take one real column of the wall so the checks land on cells that exist.
    top = wall[wall[:, 2].argmax()]
    assert probe(top)
    assert not probe(top - np.array([0, 0, 0.5], dtype=np.float32))
    # Floor buried under that column, versus floor out in the open.
    assert not probe(np.array([top[0], top[1], -0.05], dtype=np.float32))
    assert probe(np.array([top[0] - 0.5, top[1], -0.05], dtype=np.float32))


def test_standable_surface_keeps_floor_under_high_ceiling() -> None:
    """Headroom is what matters: a ceiling above the robot's height leaves the
    floor standable, one inside it does not."""
    floor = _floor(0.0, 2.0)
    floor_keys = np.unique(voxel_keys(floor, VOXEL))
    high = _surface_keys(np.concatenate([floor, floor + np.array([0, 0, 1.2], dtype=np.float32)]))
    assert keys_contain(high, floor_keys).all()
    low = _surface_keys(np.concatenate([floor, floor + np.array([0, 0, 0.2], dtype=np.float32)]))
    assert not keys_contain(low, floor_keys).any()


def test_snap_to_surface() -> None:
    xs, ys = np.meshgrid(np.arange(0, 2, VOXEL), np.arange(0, 2, VOXEL))
    surface = np.stack([xs.ravel(), ys.ravel(), np.zeros(xs.size)], axis=1, dtype=np.float32)
    snapped = snap_to_surface(np.array([1.0, 1.0, 0.4], dtype=np.float32), surface, 1.0)
    assert snapped is not None
    assert abs(snapped[2]) < 1e-6
    assert np.linalg.norm(snapped[:2] - [1.0, 1.0]) < VOXEL
    assert snap_to_surface(np.array([9.0, 9.0, 0.0], dtype=np.float32), surface, 1.0) is None
    assert snap_to_surface(np.array([1.0, 1.0, 5.0], dtype=np.float32), surface, 1.0) is None


def test_generate_cases_around_wall() -> None:
    """A U-shaped walk around a wall must yield non-trivial cases spanning it."""
    final = _final_map(_wall(10.0))
    xs, ys = np.meshgrid(np.arange(0, 20, VOXEL), np.arange(-3, 6, VOXEL))
    surface = np.stack([xs.ravel(), ys.ravel(), np.zeros(xs.size)], axis=1, dtype=np.float32)

    legs = [
        np.stack([np.linspace(2, 8, 40), np.zeros(40)], axis=1),
        np.stack([np.full(40, 8.0), np.linspace(0, 4, 40)], axis=1),
        np.stack([np.linspace(8, 12, 40), np.full(40, 4.0)], axis=1),
        np.stack([np.full(40, 12.0), np.linspace(4, 0, 40)], axis=1),
        np.stack([np.linspace(12, 18, 40), np.zeros(40)], axis=1),
    ]
    xy = np.concatenate(legs)
    positions = np.column_stack([xy, np.full(len(xy), 0.3)]).astype(np.float32)
    traj = Trajectory(ts=np.linspace(0, 60, len(positions)), positions=positions)

    cases = generate_cases(traj, final, surface, _cfg(), max_cases=10)
    assert cases
    assert [c for c in cases if (c.start[0] - 10) * (c.goal[0] - 10) < 0]
    for c in cases:
        assert abs(c.start[2]) < 1e-5 and abs(c.goal[2]) < 1e-5
        assert "flat" in c.tags


def test_select_diverse_backfills_to_min_cases() -> None:
    """Sector caps must not starve a dataset below the case floor, and the
    floor must never push it past max_cases."""
    candidates = [
        Candidate(
            start=(float(x), 0.0, 0.0),
            goal=(float(x), 20.0, 0.0),
            walked_m=30.0,
            detour_ratio=1.5,
            dz=0.0,
        )
        for x in np.arange(0.0, 16.0, 2.0)
    ]
    strict = _select_diverse(candidates, max_cases=12, min_cases=0)
    backfilled = _select_diverse(candidates, max_cases=12, min_cases=6)
    assert len(strict) < 6
    assert len(backfilled) == 6
    assert len({(c.start, c.goal) for c in backfilled}) == len(backfilled)
    assert len(_select_diverse(candidates, max_cases=3, min_cases=6)) == 3


def test_pick_along_ray() -> None:
    wall = _wall(10.0)
    origin = np.array([0.0, 0.0, 0.5])
    target = np.array([10.0, 0.35, 0.75])
    direction = target - origin
    direction /= np.linalg.norm(direction)
    picked = pick_along_ray(wall, origin, direction, VOXEL)
    assert picked is not None
    assert np.linalg.norm(picked - target) < 0.15
    # The nearest surface along the ray wins over one behind it.
    two_walls = np.concatenate([_wall(10.0), _wall(15.0)])
    picked = pick_along_ray(two_walls, origin, direction, VOXEL)
    assert picked is not None
    assert abs(picked[0] - 10.0) < 0.2
    # A ray into empty space picks nothing.
    up = np.array([0.0, 0.0, 1.0])
    assert pick_along_ray(wall, origin, up, VOXEL) is None


def test_save_suite_roundtrip(tmp_path: Path) -> None:
    suite = Suite(
        dataset="demo",
        cases=[
            Case(id="a", start=(0.0, 0.0, 0.0), goal=(1.0, 2.0, 3.0), tags=["x"]),
            Case(id="neg", start=(0.0, 0.0, 0.0), goal=(5.0, 5.0, 5.0), expect_fail=True),
            Case(id="dyn", start=(0.0, 0.0, 0.0), goal=(3.0, 0.0, 0.0), expect_final_fail=True),
        ],
        lidar_stream="other_lidar",
        db="~/recordings/demo.db",
    )
    loaded = load_suite(save_suite(suite, tmp_path / "demo.yaml"))
    assert loaded.dataset == "demo"
    assert loaded.lidar_stream == "other_lidar"
    assert loaded.db_path() == Path.home() / "recordings/demo.db"
    assert loaded.cases[0].goal == (1.0, 2.0, 3.0)
    assert loaded.cases[0].tags == ["x"]
    assert loaded.cases[1].expect_fail
    assert loaded.cases[2].expect_final_fail


def test_gate_band_follows_the_tilted_body_axis() -> None:
    """On a slope the band is measured up the body axis, so an obstacle at the
    body center collides and one down in the leg zone does not."""
    vox = 0.02
    cfg = _cfg(voxel_size=vox)
    t = np.arange(-3, 3.01, 0.1)
    c = 1 / np.sqrt(2)
    path = np.stack([t * c, np.zeros_like(t), t * c], axis=1).astype(np.float32)
    samples = metrics.densify(path, vox / 2)
    i = len(samples) // 2
    _, _, up = metrics.body_frames(samples, cfg.robot_length)
    mid_h = (cfg.ground_margin + cfg.body_clearance) / 2.0

    def hits(point: np.ndarray) -> bool:
        keys = np.unique(voxel_keys(np.asarray([point], dtype=np.float32), vox))
        return not metrics.check_path(path, keys, cfg).valid

    assert hits(samples[i] + mid_h * up[i])
    assert not hits(samples[i] + 0.15 * up[i])


def test_offset_deltas_match_packing_the_summed_indices() -> None:
    """Adding a packed delta must equal packing the summed indices, which is
    what lets the gate probe a neighborhood without unpacking."""
    pts = np.array([[0.05, -3.2, 1.4], [12.0, 0.0, -2.0]], dtype=np.float32)
    offs = cylinder_offsets(0.4, -0.2, 0.3, VOXEL)
    idx = np.floor(pts.astype(np.float64) / VOXEL).astype(np.int64)[:, None, :] + offs[None, :, :]
    packed = voxel_keys((idx.reshape(-1, 3) * VOXEL + VOXEL / 2).astype(np.float32), VOXEL)
    shifted = voxel_keys(pts, VOXEL)[:, None] + offset_deltas(offs)[None, :]
    assert np.array_equal(shifted.ravel(), packed)


def test_body_frames_survive_a_zero_length_path() -> None:
    """A zero-length path must still yield a usable body frame."""
    point = np.array([[1.0, 1.0, 0.0], [1.0, 1.0, 0.0]], dtype=np.float32)
    fwd, lateral, up = metrics.body_frames(point, 0.7)
    assert np.allclose(np.linalg.norm(fwd, axis=1), 1.0)
    assert np.linalg.det(np.stack((fwd, lateral, up), axis=-1)[0]) > 0


def _store(tmp_path: Path, cases: list[Case]) -> CaseStore:
    manifest = tmp_path / "demo.yaml"
    save_suite(Suite(dataset="demo", cases=cases), manifest)
    xs, ys = np.meshgrid(np.arange(0, 8, VOXEL), np.arange(-2, 2, VOXEL))
    surface = np.stack([xs.ravel(), ys.ravel(), np.zeros(xs.size)], axis=1, dtype=np.float32)
    final = FinalMap(
        voxel_size=VOXEL,
        occupied=surface,
        occupied_keys=np.unique(voxel_keys(surface, VOXEL)),
        build_ms=0.0,
    )
    return CaseStore(load_suite(manifest), surface, _cfg(), final)


def test_editing_a_generated_case_keeps_it_in_the_incremental_score(tmp_path: Path) -> None:
    """Relabelling a case must not silently change what it measures."""
    case = Case(
        id="auto_00_flat", start=(1.0, 0.0, 0.0), goal=(5.0, 0.0, 0.0), tags=["auto", "flat"]
    )
    store = _store(tmp_path, [case])
    store.update("auto_00_flat", "auto_00_flat", ["flat", "doorway"], expect_fail=False)
    edited = store.get("auto_00_flat")
    assert "manual" not in edited.tags
    assert not _final_only(edited)
    # A hand-added case is curated, and stays a final-map-only test.
    added = store.add((1.0, 0.0, 0.0), (4.0, 0.0, 0.0), ["flat"])
    assert added.tags[0] == "manual"
    assert _final_only(added)


def test_curation_rejects_bad_requests(tmp_path: Path) -> None:
    store = _store(tmp_path, [Case(id="manual_00", start=(1.0, 0.0, 0.0), goal=(5.0, 0.0, 0.0))])
    with pytest.raises(CurationError, match="already exists"):
        store.add((1.0, 0.0, 0.0), (5.0, 0.0, 0.0), [], case_id="manual_00")
    with pytest.raises(CurationError, match="standable surface"):
        store.add((1.0, 0.0, 0.0), (400.0, 0.0, 0.0), [])
    # An infeasible goal may sit where nothing is standable.
    assert store.add((1.0, 0.0, 0.0), (400.0, 0.0, 0.0), [], expect_fail=True).expect_fail
    with pytest.raises(CurationError, match="not found"):
        store.get("nope")
    assert _curated_tags(["flat", "negative"], True) == ["manual", "negative", "flat"]


def test_a_curated_dynamic_case_scores_for_refusing() -> None:
    """expect_final_fail inverts the final outcome whatever the provenance."""
    keys, cfg, case = _meta_scene()
    for tags in (["auto"], ["manual"]):
        marked = replace(case, tags=tags, expect_final_fail=True)
        refused = _run_plan(_stub(None), marked, 16.0, keys, keys, cfg)
        assert score_negative(refused).spl == 1.0
        assert _final_only(marked) == ("auto" not in tags)


def test_cache_writes_are_atomic(tmp_path: Path) -> None:
    """An interrupted write must leave the previous cache loadable."""
    cache = tmp_path / "nested" / "x.abc123.npz"
    _save_npz(cache, frames=np.array(7))
    assert int(np.load(cache)["frames"]) == 7
    assert not list(cache.parent.glob("*.tmp"))
    with pytest.raises(KeyboardInterrupt):
        with pytest.MonkeyPatch.context() as mp:
            mp.setattr(np, "savez_compressed", _raise_interrupt)
            _save_npz(cache, frames=np.array(9))
    assert int(np.load(cache)["frames"]) == 7
    assert not list(cache.parent.glob("*.tmp"))


def _raise_interrupt(*args: object, **kwargs: object) -> None:
    raise KeyboardInterrupt


def _write_recording(
    path: Path,
    clouds: list[tuple[float, np.ndarray, str]],
    poses: list[tuple[float, tuple[float, float, float]]],
) -> None:
    """A minimal mem2 recording: sensor-frame clouds plus odometry."""
    with SqliteStore(path=str(path)) as store:
        lidar = store.stream("lidar", PointCloud2)
        for ts, pts, frame_id in clouds:
            lidar.append(PointCloud2.from_numpy(pts, frame_id=frame_id, timestamp=ts), ts=ts)
        odom = store.stream("odom", Odometry)
        for ts, (x, y, z) in poses:
            odom.append(Odometry(ts=ts, pose=Pose(x, y, z)), ts=ts)


def test_recording_registers_clouds_and_honors_end_ts(tmp_path: Path) -> None:
    """Clouds arrive sensor-frame and are placed by their aligned odometry."""
    local = np.array([[1.0, 0.0, 0.0], [0.0, 2.0, 0.0]], dtype=np.float32)
    db = tmp_path / "rec.db"
    _write_recording(
        db,
        [(1.0, local, "lidar"), (2.0, local, "lidar"), (3.0, local, "lidar")],
        [(1.0, (10.0, 0.0, 0.5)), (2.0, (20.0, 0.0, 0.5)), (3.0, (30.0, 0.0, 0.5))],
    )
    frames = list(iter_world_frames(db, "lidar", "odom"))
    assert [f.ts for f in frames] == [1.0, 2.0, 3.0]
    # Translated by the odometry position, and the origin is that position.
    assert np.allclose(frames[0].points, local + np.array([10.0, 0.0, 0.5]))
    assert frames[1].origin == (20.0, 0.0, 0.5)
    # end_ts drops frames at or after it.
    assert [f.ts for f in iter_world_frames(db, "lidar", "odom", end_ts=2.5)] == [1.0, 2.0]

    traj = load_trajectory(db, "odom")
    assert len(traj.positions) == 3
    assert np.allclose(traj.foot(0.5)[:, 2], 0.0)
    assert load_trajectory(db, "odom", end_ts=2.5).positions.shape[0] == 2


def test_recording_rejects_pre_registered_clouds(tmp_path: Path) -> None:
    """World-frame clouds would be registered twice, so they are refused."""
    db = tmp_path / "legacy.db"
    pts = np.array([[1.0, 0.0, 0.0]], dtype=np.float32)
    _write_recording(db, [(1.0, pts, "world")], [(1.0, (0.0, 0.0, 0.0))])
    with pytest.raises(ValueError, match="world-frame"):
        list(iter_world_frames(db, "lidar", "odom"))
    with pytest.raises(ValueError, match="no odometry"):
        load_trajectory(db, "missing_stream")


def test_apply_overrides_is_the_sweep_interface() -> None:
    """--set is how a sweep varies the harness and the pipeline."""
    cfg = _apply_overrides(
        EvalConfig(), ["goal_tolerance=0.4", "planner.wall_clearance_m=0.0", "pipeline=mls"]
    )
    assert cfg.goal_tolerance == pytest.approx(0.4)
    assert isinstance(cfg.goal_tolerance, float)
    assert cfg.planner == {"wall_clearance_m": 0.0}
    assert cfg.pipeline == "mls"
    for bad in (["no_equals_sign"], ["not_a_field=1"], ["planner=0.5"]):
        with pytest.raises(typer.BadParameter):
            _apply_overrides(EvalConfig(), bad)
    # An override that inverts the gate band is caught after it is applied,
    # since __post_init__ already ran on the config being mutated.
    with pytest.raises(typer.BadParameter, match="body_clearance"):
        _apply_overrides(EvalConfig(), ["ground_margin=0.46"])


def test_an_inverted_body_band_is_rejected_not_silently_passed() -> None:
    """With ground_margin above body_clearance the gate admits nothing, so every
    path would pass and the score would read as perfect."""
    with pytest.raises(ValueError, match="body_clearance"):
        EvalConfig(ground_margin=0.5, body_clearance=0.45)
    for name in ("voxel_size", "robot_length", "robot_width", "max_range"):
        with pytest.raises(ValueError, match=name):
            EvalConfig(**{name: 0.0})


class _RecordingPipeline:
    """Returns the fixed path and remembers how many frames it had each time."""

    def __init__(self, waypoints: NDArray[np.float32] | None) -> None:
        self._waypoints = waypoints
        self.frames = 0
        self.frames_at_plan: list[int] = []

    def add_frame(
        self, points: NDArray[np.float32], origin: tuple[float, float, float], ts: float
    ) -> None:
        self.frames += 1

    def plan(
        self, start: tuple[float, float, float], goal: tuple[float, float, float]
    ) -> NDArray[np.float32] | None:
        self.frames_at_plan.append(self.frames)
        return self._waypoints


def _stub_harness(mp: pytest.MonkeyPatch, tmp_path: Path, pipeline: object) -> None:
    """Detach run_suite from both native modules: the pipeline under test and
    the evaluator's own grading mapper. Caches land under tmp_path."""
    mp.setitem(PIPELINES, "stub", lambda _cfg: pipeline)
    mp.setattr(EvalConfig, "make_mapper", lambda _self: _StubMapper())
    mp.setattr(final_map, "CACHE_SUBDIR", tmp_path / "cache")


WALK_HEIGHT = 0.5


def _corridor_recording(path: Path) -> Suite:
    """Ten frames of floor along +x, walked start to finish.

    Clouds are sensor-frame, so each slab is written relative to the pose that
    registers it and lands back at z = -0.05 in the world.
    """
    slabs = [
        (
            float(t),
            _floor(float(t) - 1.0, float(t) + 1.0)
            - np.array([t, 0, WALK_HEIGHT], dtype=np.float32),
        )
        for t in range(1, 11)
    ]
    _write_recording(
        path,
        [(ts, pts, "lidar") for ts, pts in slabs],
        [(float(t), (float(t), 0.0, WALK_HEIGHT)) for t in range(1, 11)],
    )
    return Suite(
        dataset="corridor", cases=[], db=str(path), lidar_stream="lidar", odom_stream="odom"
    )


def test_run_suite_plans_each_case_on_the_map_as_of_its_start_time(tmp_path: Path) -> None:
    """The framework's core claim: a case is scored against exactly the frames
    the robot had seen by its start time, not the whole recording."""
    suite = _corridor_recording(tmp_path / "corridor.db")
    # Both walk backwards, so the goal was visited before the start and the
    # reference is causal. The second starts later in the recording.
    suite.cases = [
        Case(id="auto_early", start=(4.0, 0.0, 0.0), goal=(2.0, 0.0, 0.0), tags=["auto"]),
        Case(id="auto_late", start=(9.0, 0.0, 0.0), goal=(7.0, 0.0, 0.0), tags=["auto"]),
    ]
    pipeline = _RecordingPipeline(np.array([[4, 0, 0], [2, 0, 0]], dtype=np.float32))
    with pytest.MonkeyPatch.context() as mp:
        _stub_harness(mp, tmp_path, pipeline)
        result = runner.run_suite(suite, _cfg(pipeline="stub"))

    assert result.frames == 10
    # Two online plans, then one final plan per case on the full recording.
    online_early, online_late = pipeline.frames_at_plan[:2]
    assert 0 < online_early < online_late < 10
    assert pipeline.frames_at_plan[2:] == [10, 10]


class _PerCasePipeline:
    """Plans only between the endpoints it was given, refusing anything else."""

    def __init__(self, routes: dict[tuple[Point, Point], np.ndarray]) -> None:
        self._routes = routes

    def add_frame(self, points: NDArray[np.float32], origin: Point, ts: float) -> None:
        pass

    def plan(self, start: Point, goal: Point) -> NDArray[np.float32] | None:
        return self._routes.get((start, goal))


def test_evaluate_scores_only_online_cases_in_the_headline(tmp_path: Path) -> None:
    """The headline score covers online cases only, so a final-only case that
    fails must drop final_score while leaving score untouched."""
    suite = _corridor_recording(tmp_path / "corridor.db")
    walked: Point = (4.0, 0.0, 0.0)
    goal: Point = (2.0, 0.0, 0.0)
    suite.cases = [
        Case(id="auto_00", start=walked, goal=goal, tags=["auto"]),
        # Same endpoints, but a manual case never replays online.
        Case(id="manual_00", start=walked, goal=goal, tags=["manual"]),
    ]
    route = np.array([walked, (3.0, 0.0, 0.0), goal], dtype=np.float32)
    with pytest.MonkeyPatch.context() as mp:
        # Only the auto case's plan call is answered; the manual one is refused.
        _stub_harness(mp, tmp_path, _PerCasePipeline({(walked, goal): route}))
        report = runner.evaluate([suite], _cfg(pipeline="stub"))

    assert report.n_cases == 2
    assert report.n_online == 1
    assert [c.final_only for d in report.datasets for c in d.cases] == [False, True]
    # The online case succeeds on the route the robot demonstrated.
    assert report.score == pytest.approx(1.0)
    assert report.n_success == 1
    # Both cases plan against the final map, and both succeed there, so the
    # final score stays at 1.0 while covering twice as many cases.
    assert report.n_success_final == 2
    assert report.final_score == pytest.approx(1.0)
    assert report.by_tag["manual"].n_online == 0


def test_spl_scales_with_how_far_the_path_overshoots_the_reference() -> None:
    """Only 0.0 and 1.0 are asserted elsewhere, so the ratio itself is unpinned."""
    assert metrics.spl(True, 10.0, 20.0) == pytest.approx(0.5)
    assert metrics.spl(True, 10.0, 12.5) == pytest.approx(0.8)
    # A path shorter than the demonstrated route earns 1.0, never more.
    assert metrics.spl(True, 10.0, 4.0) == pytest.approx(1.0)
    # A failed case scores zero however short the path.
    assert metrics.spl(False, 10.0, 10.0) == 0.0


def test_soft_progress_is_the_share_of_the_gap_closed() -> None:
    """soft_progress drives the headline soft score and had no test."""
    start: Point = (0.0, 0.0, 0.0)
    goal: Point = (10.0, 0.0, 0.0)
    assert metrics.soft_progress(np.array([5.0, 0.0, 0.0], dtype=np.float32), start, goal) == (
        pytest.approx(0.5)
    )
    assert metrics.soft_progress(np.array([10.0, 0.0, 0.0], dtype=np.float32), start, goal) == (
        pytest.approx(1.0)
    )
    # No path at all, and moving away from the goal, both floor at zero.
    assert metrics.soft_progress(None, start, goal) == 0.0
    assert metrics.soft_progress(np.array([-5.0, 0.0, 0.0], dtype=np.float32), start, goal) == 0.0
    # A coincident start and goal has no gap to close.
    assert metrics.soft_progress(np.array([0.0, 0.0, 0.0], dtype=np.float32), start, start) == 0.0


def test_recording_applies_the_odometry_rotation(tmp_path: Path) -> None:
    """Clouds are sensor-frame, so a yawed pose must rotate them, not just shift."""
    db = tmp_path / "yaw.db"
    ahead = np.array([[1.0, 0.0, 0.0]], dtype=np.float32)
    # 90 degrees about +z, so the sensor's +x points along world +y.
    quarter = np.sqrt(0.5)
    with SqliteStore(path=str(db)) as store:
        store.stream("lidar", PointCloud2).append(
            PointCloud2.from_numpy(ahead, frame_id="lidar", timestamp=1.0), ts=1.0
        )
        store.stream("odom", Odometry).append(
            Odometry(ts=1.0, pose=Pose(5.0, 0.0, 0.0, 0.0, 0.0, quarter, quarter)), ts=1.0
        )

    frames = list(iter_world_frames(db, "lidar", "odom"))
    assert np.allclose(frames[0].points, [[5.0, 1.0, 0.0]], atol=1e-5)


def test_final_map_cache_key_tracks_the_recording(tmp_path: Path) -> None:
    """Without this a re-ingested dataset is graded against the previous map."""
    db = tmp_path / "rec.db"
    _write_recording(db, [(1.0, _floor(0.0, 1.0), "lidar")], [(1.0, (0.0, 0.0, 0.5))])
    suite = Suite(dataset="rec", cases=[], db=str(db), lidar_stream="lidar", odom_stream="odom")
    before = final_map._cache_path(db, final_map._final_params(db, suite, _cfg()))

    # Same path, same stem, same config: only the bytes change.
    _write_recording(db, [(1.0, _floor(0.0, 5.0), "lidar")], [(1.0, (0.0, 0.0, 0.5))])
    after = final_map._cache_path(db, final_map._final_params(db, suite, _cfg()))
    assert before != after


def test_checkpoints_reload_from_cache(tmp_path: Path) -> None:
    """The npz delta format is only exercised on a cache hit, which no other
    test reaches because each one gets a fresh tmp_path."""
    db = tmp_path / "rec.db"
    _write_recording(
        db,
        [(float(t), _floor(float(t), float(t) + 1.0), "lidar") for t in range(1, 5)],
        [(float(t), (float(t), 0.0, 0.0)) for t in range(1, 5)],
    )
    suite = Suite(dataset="rec", cases=[], db=str(db), lidar_stream="lidar", odom_stream="odom")
    times = np.array([2.5, np.inf])
    with pytest.MonkeyPatch.context() as mp:
        mp.setattr(EvalConfig, "make_mapper", lambda _self: _StubMapper())
        mp.setattr(final_map, "CACHE_SUBDIR", tmp_path / "cache")
        cold = final_map.load_or_build_checkpoints(suite, _cfg(), times)
        warm = final_map.load_or_build_checkpoints(suite, _cfg(), times)

    assert [s.tolist() for s in cold.iter_snapshots()] == [
        s.tolist() for s in warm.iter_snapshots()
    ]
    assert len(list(warm.iter_snapshots())) == 2


def test_a_pipeline_without_graph_layers_still_records(tmp_path: Path) -> None:
    """Recording a run must not require a pipeline to expose its internals,
    which is the whole point of the optional introspection protocol."""
    suite = _corridor_recording(tmp_path / "corridor.db")
    suite.cases = [Case(id="auto_00", start=(4.0, 0.0, 0.0), goal=(2.0, 0.0, 0.0), tags=["auto"])]
    route = np.array([[4, 0, 0], [3, 0, 0], [2, 0, 0]], dtype=np.float32)
    with pytest.MonkeyPatch.context() as mp:
        _stub_harness(mp, tmp_path, _StubPipeline(route))
        result = runner.run_suite(suite, _cfg(pipeline="stub"), keep_artifacts=True)

    assert result.final_artifacts is None
    assert result.cases[0].online_artifacts is None
    # The occupied cloud is the evaluator's own, so it is kept either way.
    assert result.cases[0].online_occupied is not None

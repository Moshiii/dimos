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

"""Run grasp proposal and connected motion planning without robot execution."""

from __future__ import annotations

from collections import Counter
from collections.abc import Callable, Sequence
from dataclasses import asdict, dataclass
import json
from pathlib import Path
from typing import Any

import numpy as np

from dimos.manipulation.demo_graspgenx.fixture import load_demo_clouds
from dimos.manipulation.demo_graspgenx.render import SweepVolumeLike, render_grasp_image
from dimos.manipulation.grasping.grasp_gen_spec import GraspGenSpec
from dimos.manipulation.grasping.grasp_gen_x import (
    IDENTITY_TRANSFORM,
    GraspGenXModule,
    RigidTransform,
)
from dimos.manipulation.manipulation_module import ConnectedPoseSequenceResult
from dimos.manipulation.pick_and_place_module import PickAndPlaceModule
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.manipulation_msgs.GraspCandidate import GraspCandidate
from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.std_msgs.Header import Header
from dimos.robot.manipulators.xarm.config import make_xarm7_sim_robot_config
from dimos.robot.manipulators.xarm.grasp_config import make_xarm_graspgenx_config

CloudLoader = Callable[[], tuple[PointCloud2, PointCloud2]]
Renderer = Callable[
    [Path, PointCloud2, PointCloud2, GraspCandidateArray, SweepVolumeLike, int], Path
]

_REJECTION_BY_INDEX = (
    "pre_grasp_infeasible",
    "grasp_infeasible",
    "retreat_infeasible",
)


def render_selected_grasp_image(
    output_path: Path,
    scene: PointCloud2,
    object_cloud: PointCloud2,
    candidates: GraspCandidateArray,
    gripper: SweepVolumeLike,
    rank: int,
) -> Path:
    """Render one selected grasp while preserving its original proposal rank."""
    return render_grasp_image(
        output_path,
        scene,
        object_cloud,
        candidates,
        gripper,
        ranks=(rank,),
        title=f"Selected connected grasp — proposal rank #{rank}",
    )


@dataclass(frozen=True)
class CandidateOutcome:
    """Planning outcome for one ranked proposal."""

    rank: int
    score: float
    status: str
    rejection: str | None = None


@dataclass(frozen=True)
class PipelineDemoResult:
    """Artifacts and candidate outcomes produced by one offline run."""

    success: bool
    output_dir: Path
    summary_path: Path
    plans_path: Path
    image_path: Path | None
    candidate_count: int
    selected_rank: int | None
    selected_score: float | None
    outcomes: tuple[CandidateOutcome, ...]
    failure_reason: str | None = None


def _relocate_clouds(
    scene: PointCloud2,
    object_cloud: PointCloud2,
    workspace_center: Sequence[float],
) -> tuple[PointCloud2, PointCloud2]:
    object_points = object_cloud.points_f32()
    if not len(object_points):
        raise ValueError("recorded target cloud is empty")
    center = np.asarray(workspace_center, dtype=np.float32)
    if center.shape != (3,) or not np.all(np.isfinite(center)):
        raise ValueError("workspace center must contain three finite values")
    translation = center - np.mean(object_points, axis=0)
    relocated_scene = PointCloud2.from_numpy(
        scene.points_f32() + translation,
        frame_id=scene.frame_id,
        timestamp=scene.ts,
    )
    relocated_object = PointCloud2.from_numpy(
        object_points + translation,
        frame_id=object_cloud.frame_id,
        timestamp=object_cloud.ts,
    )
    return relocated_scene, relocated_object


def _joint_state_dict(state: JointState) -> dict[str, Any]:
    return {
        "names": list(state.name),
        "positions": [float(value) for value in state.position],
    }


def _pose_dict(pose: Pose) -> dict[str, list[float]]:
    return {
        "position": [
            float(pose.position.x),
            float(pose.position.y),
            float(pose.position.z),
        ],
        "orientation_xyzw": [
            float(pose.orientation.x),
            float(pose.orientation.y),
            float(pose.orientation.z),
            float(pose.orientation.w),
        ],
    }


def _grasp_frame_candidate(
    candidate: GraspCandidate,
    grasp_frame_to_tcp: RigidTransform,
) -> GraspCandidate:
    """Convert a planned TCP candidate back to its sweep-geometry frame."""
    world_to_tcp = np.eye(4, dtype=float)
    world_to_tcp[:3, :3] = candidate.pose.orientation.to_rotation_matrix()
    world_to_tcp[:3, 3] = np.asarray(candidate.pose.position.as_tuple, dtype=float)
    world_to_grasp = world_to_tcp @ np.linalg.inv(np.asarray(grasp_frame_to_tcp, dtype=float))
    grasp_pose = Pose(
        Vector3(world_to_grasp[:3, 3]),
        Quaternion.from_rotation_matrix(world_to_grasp[:3, :3]),
    )
    return GraspCandidate(grasp_pose, candidate.score)


def _segment_dicts(
    names: Sequence[str],
    result: ConnectedPoseSequenceResult,
) -> list[dict[str, Any]]:
    return [
        {
            "name": name,
            "waypoints": [_joint_state_dict(state) for state in path],
        }
        for name, path in zip(names[: len(result.paths)], result.paths, strict=True)
    ]


def _write_artifacts(
    *,
    output_dir: Path,
    success: bool,
    frame: str,
    scene_points: int,
    object_points: int,
    candidate_count: int,
    outcomes: Sequence[CandidateOutcome],
    selected_rank: int | None,
    selected: GraspCandidate | None,
    segments: Sequence[dict[str, Any]],
    image_path: Path | None,
    failure_reason: str | None,
) -> tuple[Path, Path]:
    output_dir.mkdir(parents=True, exist_ok=True)
    rejection_counts = Counter(
        outcome.rejection for outcome in outcomes if outcome.rejection is not None
    )
    summary: dict[str, Any] = {
        "success": success,
        "failure_reason": failure_reason,
        "frame": frame,
        "scene_points": scene_points,
        "object_points": object_points,
        "candidate_count": candidate_count,
        "checked_count": len(outcomes),
        "selected": (
            {
                "rank": selected_rank,
                "score": float(selected.score),
                "pose": _pose_dict(selected.pose),
            }
            if selected is not None
            else None
        ),
        "candidate_outcomes": [asdict(outcome) for outcome in outcomes],
        "rejection_counts": dict(sorted(rejection_counts.items())),
        "execution_performed": False,
        "artifacts": {
            "plans": "plans.json",
            "visualization": image_path.name if image_path is not None else None,
        },
    }
    plans = {
        "frame": frame,
        "execution_performed": False,
        "segments": list(segments),
    }
    summary_path = output_dir / "summary.json"
    plans_path = output_dir / "plans.json"
    summary_path.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    plans_path.write_text(json.dumps(plans, indent=2) + "\n", encoding="utf-8")
    return summary_path, plans_path


def run_demo(
    proposer: GraspGenSpec,
    planner: PickAndPlaceModule,
    output_dir: Path,
    *,
    gripper: SweepVolumeLike,
    max_candidates: int = 20,
    workspace_center: Sequence[float] = (0.45, 0.0, 0.25),
    sequence_start: JointState | None = None,
    grasp_frame_to_tcp: RigidTransform = IDENTITY_TRANSFORM,
    cloud_loader: CloudLoader = load_demo_clouds,
    renderer: Renderer = render_selected_grasp_image,
) -> PipelineDemoResult:
    """Run proposal and connected planning, save artifacts, and never execute."""
    if max_candidates <= 0:
        raise ValueError("max_candidates must be positive")

    scene, object_cloud = _relocate_clouds(*cloud_loader(), workspace_center)
    proposals = proposer.propose_grasps(object_cloud)
    if (
        proposals.header.frame_id != object_cloud.frame_id
        or proposals.header.timestamp != object_cloud.ts
    ):
        raise ValueError("proposer changed the target cloud frame or timestamp")
    ranked = sorted(proposals.candidates, key=lambda candidate: candidate.score, reverse=True)
    robot = planner._get_robot("arm")
    if robot is None:
        raise ValueError("offline planner does not contain the xArm planning model")
    pre_grasp_offset = float(robot[2].pre_grasp_offset)
    approach = Vector3(planner.config.grasp_approach_vector)

    segments: list[dict[str, Any]] = []
    lift_pose = planner._safety_lift_pose("arm")
    if lift_pose is not None:
        lift_result = planner._plan_connected_pose_sequence(
            (lift_pose,),
            "arm",
            sequence_start,
        )
        segments.extend(_segment_dicts(("safety_lift",), lift_result))
        if lift_result.failed_index is not None:
            summary_path, plans_path = _write_artifacts(
                output_dir=output_dir,
                success=False,
                frame=proposals.header.frame_id,
                scene_points=len(scene),
                object_points=len(object_cloud),
                candidate_count=len(ranked),
                outcomes=(),
                selected_rank=None,
                selected=None,
                segments=segments,
                image_path=None,
                failure_reason="safety_lift_infeasible",
            )
            return PipelineDemoResult(
                False,
                output_dir,
                summary_path,
                plans_path,
                None,
                len(ranked),
                None,
                None,
                (),
                "safety_lift_infeasible",
            )
        sequence_start = lift_result.endpoint

    outcomes: list[CandidateOutcome] = []
    selected: GraspCandidate | None = None
    selected_rank: int | None = None
    candidate_segments: list[dict[str, Any]] = []
    for rank, candidate in enumerate(ranked[:max_candidates], start=1):
        if not planner._valid_candidate(candidate):
            outcomes.append(CandidateOutcome(rank, candidate.score, "rejected", "invalid"))
            continue
        pre_grasp = planner._compute_pre_grasp_pose(
            candidate.pose,
            pre_grasp_offset,
            approach,
        )
        retreat = planner._compute_pre_grasp_pose(
            candidate.pose,
            pre_grasp_offset,
            approach,
        )
        result = planner._plan_connected_pose_sequence(
            (pre_grasp, candidate.pose, retreat),
            "arm",
            sequence_start,
        )
        if result.failed_index is not None:
            outcomes.append(
                CandidateOutcome(
                    rank,
                    candidate.score,
                    "rejected",
                    _REJECTION_BY_INDEX[result.failed_index],
                )
            )
            continue
        selected = candidate
        selected_rank = rank
        candidate_segments = _segment_dicts(("pre_grasp", "grasp", "retreat"), result)
        outcomes.append(CandidateOutcome(rank, candidate.score, "selected"))
        break

    image_path: Path | None = None
    if selected is not None:
        assert selected_rank is not None
        output_dir.mkdir(parents=True, exist_ok=True)
        image_path = renderer(
            output_dir / "selected-grasp.png",
            scene,
            object_cloud,
            GraspCandidateArray(
                Header(proposals.header.timestamp, proposals.header.frame_id),
                [_grasp_frame_candidate(selected, grasp_frame_to_tcp)],
            ),
            gripper,
            selected_rank,
        )
        segments.extend(candidate_segments)

    summary_path, plans_path = _write_artifacts(
        output_dir=output_dir,
        success=selected is not None,
        frame=proposals.header.frame_id,
        scene_points=len(scene),
        object_points=len(object_cloud),
        candidate_count=len(ranked),
        outcomes=outcomes,
        selected_rank=selected_rank,
        selected=selected,
        segments=segments,
        image_path=image_path,
        failure_reason=None if selected is not None else "no_complete_candidate",
    )
    return PipelineDemoResult(
        selected is not None,
        output_dir,
        summary_path,
        plans_path,
        image_path,
        len(ranked),
        selected_rank,
        float(selected.score) if selected is not None else None,
        tuple(outcomes),
        None if selected is not None else "no_complete_candidate",
    )


def run_contributor_demo(
    *,
    output_dir: Path,
    max_candidates: int = 20,
    workspace_center: Sequence[float] = (0.45, 0.0, 0.25),
) -> PipelineDemoResult:
    """Build real GraspGenX and xArm planning modules for one offline run."""
    grasp_config = make_xarm_graspgenx_config()
    robot_config = make_xarm7_sim_robot_config()
    proposer = GraspGenXModule(
        **grasp_config.model_dump(exclude={"rpc_transport", "tf_transport", "g"})
    )
    planner = PickAndPlaceModule(
        robots=[robot_config],
        planning_timeout=10.0,
        visualization={"backend": "none"},
        floor_z=None,
    )
    # This standalone module has no blueprint streams to subscribe to.
    standalone_planner: Any = planner
    standalone_planner.coordinator_joint_state = None
    standalone_planner.objects = None
    proposer_started = False
    planner_started = False
    try:
        proposer.start()
        proposer_started = True
        planner.start()
        planner_started = True
        if robot_config.home_joints is None:
            raise ValueError("xArm demo configuration has no home joint state")
        synthetic_start = JointState(
            name=list(robot_config.get_coordinator_joint_names()),
            position=list(robot_config.home_joints),
        )
        planner._on_joint_state(synthetic_start)
        return run_demo(
            proposer,
            planner,
            output_dir,
            gripper=grasp_config.gripper,
            max_candidates=max_candidates,
            workspace_center=workspace_center,
            sequence_start=synthetic_start,
            grasp_frame_to_tcp=grasp_config.grasp_frame_to_tcp,
        )
    finally:
        if planner_started:
            planner.stop()
        if proposer_started:
            proposer.stop()

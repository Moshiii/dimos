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

"""Backend-neutral grasp visualization geometry and layer builders."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
from enum import Enum
from typing import Any, Protocol

import numpy as np
from numpy.typing import NDArray

from dimos.manipulation.visualization.layers import (
    LineSetElement,
    PointCloudElement,
    VisualizationLayer,
)
from dimos.msgs.manipulation_msgs.GraspCandidate import GraspCandidate
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

OBJECT_CLOUD_LAYER_ID = "grasp/object-cloud"
PROPOSAL_LAYER_ID = "grasp/proposals"


class SweepVolumeLike(Protocol):
    extents_open: tuple[float, float, float]
    offset_open: tuple[float, float, float]
    extents_half_open: tuple[float, float, float]
    offset_half_open: tuple[float, float, float]


class GraspCandidateVisualState(str, Enum):
    """Operator-facing state for one grasp wireframe."""

    PENDING = "pending"
    CURRENT = "current"
    REJECTED = "rejected"
    SELECTED = "selected"
    RANKED = "ranked"


@dataclass(frozen=True)
class VisualizedGraspCandidate:
    """One ranked candidate and its current display state."""

    candidate: GraspCandidate
    rank: int
    state: GraspCandidateVisualState

    def __post_init__(self) -> None:
        if self.rank <= 0:
            raise ValueError("grasp rank must be positive")


def _rotation(quaternion: Any) -> NDArray[np.float64]:
    x, y, z, w = (
        float(quaternion.x),
        float(quaternion.y),
        float(quaternion.z),
        float(quaternion.w),
    )
    return np.asarray(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def _fork_strips_local(gripper: SweepVolumeLike) -> tuple[NDArray[np.float64], ...]:
    open_extents = np.asarray(gripper.extents_open, dtype=float)
    half_extents = np.asarray(gripper.extents_half_open, dtype=float)
    open_offset = np.asarray(gripper.offset_open, dtype=float)
    half_offset = np.asarray(gripper.offset_half_open, dtype=float)
    rear_center = np.asarray([half_offset[0], 0.0, half_offset[2] - half_extents[2] / 2.0])
    mouth_center = np.asarray([open_offset[0], 0.0, open_offset[2] + open_extents[2] / 2.0])
    if mouth_center[2] <= rear_center[2]:
        raise ValueError("configured sweep profiles must open toward increasing local +Z")
    rear_half_width = max(float(half_extents[0]) / 2.0, 1e-6)
    mouth_half_width = max(float(open_extents[0]) / 2.0, rear_half_width)
    rear_left = rear_center + np.asarray([-rear_half_width, 0.0, 0.0])
    rear_right = rear_center + np.asarray([rear_half_width, 0.0, 0.0])
    mouth_left = mouth_center + np.asarray([-mouth_half_width, 0.0, 0.0])
    mouth_right = mouth_center + np.asarray([mouth_half_width, 0.0, 0.0])
    return (
        np.asarray([rear_center, [0.0, 0.0, 0.0]]),
        np.asarray([rear_left, rear_right]),
        np.asarray([rear_left, mouth_left]),
        np.asarray([rear_right, mouth_right]),
    )


def gripper_wireframe_strips(
    candidate: GraspCandidate,
    gripper: SweepVolumeLike,
    grasp_frame_to_tcp: Sequence[Sequence[float]] | NDArray[np.generic],
) -> tuple[NDArray[np.float64], ...]:
    """Convert one TCP proposal into world-frame sweep-volume wireframe strips."""
    position = candidate.pose.position
    world_to_tcp = np.eye(4, dtype=float)
    world_to_tcp[:3, :3] = _rotation(candidate.pose.orientation)
    world_to_tcp[:3, 3] = np.asarray([position.x, position.y, position.z], dtype=float)
    grasp_to_tcp = np.asarray(grasp_frame_to_tcp, dtype=float)
    if grasp_to_tcp.shape != (4, 4):
        raise ValueError("grasp_frame_to_tcp must have shape (4, 4)")
    world_to_grasp = world_to_tcp @ np.linalg.inv(grasp_to_tcp)
    rotation = world_to_grasp[:3, :3]
    translation = world_to_grasp[:3, 3]
    return tuple((rotation @ strip.T).T + translation for strip in _fork_strips_local(gripper))


def gripper_wireframe_geometry(
    candidate: GraspCandidate,
    gripper: SweepVolumeLike,
    grasp_frame_to_tcp: Sequence[Sequence[float]] | NDArray[np.generic],
) -> tuple[NDArray[np.float32], NDArray[np.int32]]:
    """Return indexed world-frame vertices and edges for one grasp proposal."""
    strips = gripper_wireframe_strips(candidate, gripper, grasp_frame_to_tcp)
    vertices = np.vstack(strips).astype(np.float32)
    edges = np.arange(len(vertices), dtype=np.int32).reshape((-1, 2))
    return vertices, edges


def _rank_color(index: int, count: int) -> NDArray[np.uint8]:
    fraction = 0.0 if count <= 1 else index / (count - 1)
    start = np.asarray([0, 220, 80], dtype=float)
    end = np.asarray([255, 140, 0], dtype=float)
    return np.asarray(np.rint(start + fraction * (end - start)), dtype=np.uint8)


def _state_color(state: GraspCandidateVisualState, index: int, count: int) -> NDArray[np.uint8]:
    colors = {
        GraspCandidateVisualState.PENDING: np.asarray([120, 130, 140], dtype=np.uint8),
        GraspCandidateVisualState.CURRENT: np.asarray([255, 220, 0], dtype=np.uint8),
        GraspCandidateVisualState.REJECTED: np.asarray([230, 50, 50], dtype=np.uint8),
        GraspCandidateVisualState.SELECTED: np.asarray([0, 220, 80], dtype=np.uint8),
    }
    return _rank_color(index, count) if state is GraspCandidateVisualState.RANKED else colors[state]


def build_grasp_object_cloud_layer(pointcloud: PointCloud2) -> VisualizationLayer:
    """Build the stable display-only object-cloud layer."""
    points, colors = pointcloud.as_numpy()
    return VisualizationLayer(
        OBJECT_CLOUD_LAYER_ID,
        pointcloud.frame_id,
        (PointCloudElement("object", points, colors),),
    )


def build_grasp_proposals_layer(
    candidates: Sequence[VisualizedGraspCandidate],
    *,
    frame_id: str,
    gripper: SweepVolumeLike,
    grasp_frame_to_tcp: Sequence[Sequence[float]] | NDArray[np.generic],
) -> VisualizationLayer:
    """Build one complete candidate-state layer replacement."""
    elements = []
    for index, visualized in enumerate(candidates):
        vertices, edges = gripper_wireframe_geometry(
            visualized.candidate,
            gripper,
            grasp_frame_to_tcp,
        )
        elements.append(
            LineSetElement(
                f"rank-{visualized.rank}",
                vertices,
                edges,
                colors=_state_color(visualized.state, index, len(candidates)),
                line_width=2.5,
            )
        )
    return VisualizationLayer(PROPOSAL_LAYER_ID, frame_id, tuple(elements))

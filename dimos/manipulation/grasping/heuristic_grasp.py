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

"""Lightweight geometric grasp proposal provider."""

from __future__ import annotations

import math
from typing import Any

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.manipulation.grasping.grasp_gen_spec import GraspGenSpec
from dimos.manipulation.grasping.grasp_proposal import GraspProposalInput
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.manipulation_msgs.GraspCandidate import GraspCandidate
from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.msgs.std_msgs.Header import Header

FAR_OCCLUSION_XY_THRESHOLD = 0.8
TALL_OBJECT_MIN_HEIGHT = 0.06


def occlusion_offset(center: Vector3, size: Vector3, inset: float = 0.02) -> tuple[float, float]:
    """Offset an object center toward the robot to compensate for occlusion."""
    xy_dist = (center.x**2 + center.y**2) ** 0.5
    if xy_dist > 1e-3:
        dx, dy = -center.x / xy_dist, -center.y / xy_dist
        half_depth = max(size.x, size.y) / 2.0
        offset = half_depth - inset
        return center.x + dx * offset, center.y + dy * offset
    return center.x, center.y


def grasp_orientation(gx: float, gy: float, xy_dist: float) -> Quaternion:
    """Return the existing distance-adaptive top-down grasp orientation."""
    near = 0.6
    far = 1.0
    max_tilt = math.pi / 4
    if xy_dist <= near:
        tilt = 0.0
    elif xy_dist >= far:
        tilt = max_tilt
    else:
        tilt = max_tilt * (xy_dist - near) / (far - near)
    yaw = math.atan2(gy, gx)
    pitch = math.pi - tilt
    return Quaternion.from_euler(Vector3(0.0, pitch, yaw))


class HeuristicGraspModule(Module, GraspGenSpec):
    """Propose the former built-in geometric grasp through `GraspGenSpec`."""

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)

    @rpc
    def propose_grasps(self, proposal_input: GraspProposalInput) -> GraspCandidateArray:
        cloud = proposal_input.object_pointcloud
        if cloud.ts is None:
            raise ValueError("object pointcloud must have a timestamp")
        if not cloud.frame_id:
            raise ValueError("object pointcloud frame_id must not be empty")

        center = proposal_input.object_center
        size = proposal_input.object_size
        xy_dist = (center.x**2 + center.y**2) ** 0.5
        inset = 0.01 if xy_dist < FAR_OCCLUSION_XY_THRESHOLD else 0.05
        gx, gy = occlusion_offset(center, size, inset=inset)
        gz = center.z + size.z * 0.2 if size.z > TALL_OBJECT_MIN_HEIGHT else center.z
        grasp_dist = (gx**2 + gy**2) ** 0.5
        pose = Pose(Vector3(gx, gy, gz), grasp_orientation(gx, gy, grasp_dist))
        return GraspCandidateArray(
            Header(float(cloud.ts), cloud.frame_id),
            [GraspCandidate(pose=pose, score=0.0)],
        )

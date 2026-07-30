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

"""Hermetic tests for the explicit heuristic grasp provider."""

from unittest.mock import patch

import numpy as np
import pytest

from dimos.core.module import ModuleBase
from dimos.manipulation.grasping.grasp_proposal import GraspProposalInput
from dimos.manipulation.grasping.heuristic_grasp import HeuristicGraspModule
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


def propose(
    center: tuple[float, float, float],
    size: tuple[float, float, float],
) -> GraspCandidateArray:
    cloud = PointCloud2.from_numpy(
        np.asarray([[99.0, 99.0, 99.0]], dtype=np.float32),
        frame_id="world",
        timestamp=12.5,
    )
    with patch.object(ModuleBase, "__init__", lambda self, config_args: None):
        module = HeuristicGraspModule()
    return module.propose_grasps(GraspProposalInput(cloud, Vector3(center), Vector3(size)))


def test_provider_preserves_near_tall_object_heuristic() -> None:
    result = propose((0.5, 0.0, 0.3), (0.1, 0.1, 0.1))

    candidate = result.candidates[0]
    assert result.header.frame_id == "world"
    assert result.header.timestamp == pytest.approx(12.5)
    assert candidate.score == 0.0
    assert candidate.pose.position.x == pytest.approx(0.46)
    assert candidate.pose.position.y == pytest.approx(0.0)
    assert candidate.pose.position.z == pytest.approx(0.32)
    assert candidate.pose.orientation.y == pytest.approx(1.0)


def test_provider_preserves_far_orientation_and_inset() -> None:
    near = propose((0.3, 0.0, 0.2), (0.04, 0.04, 0.04)).candidates[0]
    far = propose((1.0, 0.0, 0.2), (0.1, 0.1, 0.04)).candidates[0]

    assert far.pose.position.x == pytest.approx(1.0)
    assert far.pose.position.z == pytest.approx(0.2)
    assert far.pose.orientation.y != pytest.approx(near.pose.orientation.y)


def test_provider_uses_detection_geometry_instead_of_cloud_bounds() -> None:
    result = propose((0.0, 0.0, 0.25), (0.04, 0.04, 0.04))

    position = result.candidates[0].pose.position
    assert position.as_tuple == pytest.approx((0.0, 0.0, 0.25))

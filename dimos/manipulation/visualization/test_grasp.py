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

"""Tests for backend-neutral grasp visualization builders."""

from types import SimpleNamespace

import numpy as np

from dimos.manipulation.visualization.grasp import (
    GraspCandidateVisualState,
    VisualizedGraspCandidate,
    build_grasp_object_cloud_layer,
    build_grasp_proposals_layer,
)
from dimos.manipulation.visualization.layers import LineSetElement, PointCloudElement
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.manipulation_msgs.GraspCandidate import GraspCandidate
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

IDENTITY = (
    (1.0, 0.0, 0.0, 0.0),
    (0.0, 1.0, 0.0, 0.0),
    (0.0, 0.0, 1.0, 0.0),
    (0.0, 0.0, 0.0, 1.0),
)
GRIPPER = SimpleNamespace(
    extents_open=(0.1, 0.05, 0.1),
    offset_open=(0.0, 0.0, 0.1),
    extents_half_open=(0.05, 0.05, 0.05),
    offset_half_open=(0.0, 0.0, 0.05),
)


def candidate(rank: int, state: GraspCandidateVisualState) -> VisualizedGraspCandidate:
    return VisualizedGraspCandidate(
        GraspCandidate(Pose(float(rank), 0.0, 0.2), float(rank)),
        rank,
        state,
    )


def test_object_cloud_builder_preserves_points_and_stable_identity() -> None:
    cloud = PointCloud2.from_numpy(
        np.asarray([[0.1, 0.2, 0.3]], dtype=np.float32),
        frame_id="world",
        timestamp=1.0,
    )

    layer = build_grasp_object_cloud_layer(cloud)

    assert layer.id == "grasp/object-cloud"
    assert layer.frame_id == "world"
    assert isinstance(layer.elements[0], PointCloudElement)
    np.testing.assert_allclose(layer.elements[0].points, [[0.1, 0.2, 0.3]])


def test_proposal_builder_maps_selection_states_to_operator_colors() -> None:
    layer = build_grasp_proposals_layer(
        [
            candidate(1, GraspCandidateVisualState.REJECTED),
            candidate(2, GraspCandidateVisualState.CURRENT),
            candidate(3, GraspCandidateVisualState.PENDING),
            candidate(4, GraspCandidateVisualState.SELECTED),
        ],
        frame_id="world",
        gripper=GRIPPER,
        grasp_frame_to_tcp=IDENTITY,
    )

    assert layer.id == "grasp/proposals"
    assert [element.id for element in layer.elements] == [
        "rank-1",
        "rank-2",
        "rank-3",
        "rank-4",
    ]
    assert all(isinstance(element, LineSetElement) for element in layer.elements)
    np.testing.assert_array_equal(layer.elements[0].colors, [230, 50, 50])
    np.testing.assert_array_equal(layer.elements[1].colors, [255, 220, 0])
    np.testing.assert_array_equal(layer.elements[2].colors, [120, 130, 140])
    np.testing.assert_array_equal(layer.elements[3].colors, [0, 220, 80])


def test_selected_only_replacement_retains_original_rank() -> None:
    layer = build_grasp_proposals_layer(
        [candidate(2, GraspCandidateVisualState.SELECTED)],
        frame_id="world",
        gripper=GRIPPER,
        grasp_frame_to_tcp=IDENTITY,
    )

    assert [element.id for element in layer.elements] == ["rank-2"]

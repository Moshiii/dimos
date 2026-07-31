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

"""Recorded-data smoke coverage for prompted object localization."""

import pytest

from dimos.manipulation.grasping.grasp_proposal import GraspProposalInput
from dimos.memory2.store.sqlite import SqliteStore
from dimos.perception.memory.prompted_object_localizer import (
    PromptedObjectLocalizationRuntime,
    latest_recording_window,
)
from dimos.utils.data import get_data


@pytest.mark.self_hosted_large
def test_recorded_marker_cloud_feeds_grasp_input_in_memory() -> None:
    dataset = get_data(
        "xarm6_worldbelief_realsense_d435i_stationery_calibrated/"
        "xarm6_worldbelief_20260729_203624_161992.db"
    )

    with SqliteStore(path=dataset, must_exist=True) as store:
        start, end = latest_recording_window(store.streams.color_image)
        with PromptedObjectLocalizationRuntime(store) as runtime:
            best = runtime.localize("white and red marker", start, end)

    assert best is not None
    proposal_input = GraspProposalInput.from_pointcloud(best.pointcloud)
    assert proposal_input.object_pointcloud is best.pointcloud
    assert len(proposal_input.object_pointcloud.points_f32()) > 0
    assert proposal_input.object_pointcloud.frame_id == "world"

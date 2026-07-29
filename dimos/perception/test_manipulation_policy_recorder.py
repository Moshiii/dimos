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

from collections.abc import Iterator

import pytest

from dimos.memory2.store.sqlite import SqliteStore
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.perception.manipulation_policy_recorder import ManipulationPolicyRecorder

EXPECTED_POLICY_STREAMS = {
    "camera_info",
    "color_image",
    "coordinator_joint_state",
    "depth_camera_info",
    "depth_image",
    "detections_2d",
    "detections_3d",
    "objects",
    "pointcloud",
}


@pytest.fixture
def recorder(tmp_path) -> Iterator[ManipulationPolicyRecorder]:
    module = ManipulationPolicyRecorder(db_path=tmp_path / "policy.db", record_tf=False)
    try:
        yield module
    finally:
        module.stop()


def test_recorder_declares_raw_and_derived_policy_streams(
    recorder: ManipulationPolicyRecorder,
) -> None:
    assert set(recorder.inputs) == EXPECTED_POLICY_STREAMS


def test_derived_and_proprioceptive_observations_are_typed_and_queryable(
    recorder: ManipulationPolicyRecorder,
) -> None:
    recorder._prepare_streams()
    joint_state = JointState(name=["joint1"], position=[0.25])
    pointcloud = PointCloud2(frame_id="world", ts=12.0)
    recorder.store.stream("coordinator_joint_state", JointState).append(joint_state)
    recorder.store.stream("pointcloud", PointCloud2).append(pointcloud)
    recorder.store.stream("objects", list, codec="lz4+pickle").append([])

    reader = SqliteStore(path=recorder.recording_path(), must_exist=True)
    reader.start()
    try:
        stored_joint_state = reader.streams.coordinator_joint_state.last().data
        stored_pointcloud = reader.streams.pointcloud.last().data
        stored_objects = reader.streams.objects.last().data

        assert isinstance(stored_joint_state, JointState)
        assert stored_joint_state.position == [0.25]
        assert isinstance(stored_pointcloud, PointCloud2)
        assert stored_pointcloud.frame_id == "world"
        assert stored_objects == []
    finally:
        reader.stop()

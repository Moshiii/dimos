# Copyright 2025-2026 Dimensional Inc.
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

"""Unit tests for PickAndPlaceModule pure logic (no Drake required)."""

from __future__ import annotations

from types import SimpleNamespace
from unittest.mock import MagicMock, call, patch

import open3d as o3d
import pytest

from dimos.agents.skill_result import SkillResult
from dimos.core.module import ModuleBase
from dimos.manipulation.pick_and_place_module import PickAndPlaceModule
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.perception.experimental.object import Object as DetObject


def _make_det_object(
    name: str = "cup",
    object_id: str = "abc12345",
    center: tuple[float, float, float] = (0.5, 0.0, 0.3),
    size: tuple[float, float, float] = (0.05, 0.05, 0.10),
) -> DetObject:
    """Create a DetObject with the given attributes and sensible defaults."""
    return DetObject(
        name=name,
        object_id=object_id,
        center=Vector3(x=center[0], y=center[1], z=center[2]),
        size=Vector3(x=size[0], y=size[1], z=size[2]),
        pose=PoseStamped(),
        pointcloud=PointCloud2(o3d.geometry.PointCloud()),
        bbox=(0.0, 0.0, 1.0, 1.0),
        track_id=0,
        class_id=0,
        confidence=1.0,
        ts=0.0,
        image=Image(),
    )


@pytest.fixture
def module() -> PickAndPlaceModule:
    """Create a PickAndPlaceModule with heavy base init (RPC, config) patched out."""
    with patch.object(ModuleBase, "__init__", lambda self, config_args: None):
        return PickAndPlaceModule()


class TestFindObjectInDetections:
    """Test object lookup logic in detection snapshot."""

    def test_find_by_exact_name(self, module):
        det = _make_det_object(name="cup")
        module._detection_snapshot = [det]

        result = module._find_object_in_detections("cup")
        assert result is det

    def test_find_by_partial_name(self, module):
        det = _make_det_object(name="red cup")
        module._detection_snapshot = [det]

        result = module._find_object_in_detections("cup")
        assert result is det

    def test_find_by_object_id(self, module):
        det = _make_det_object(object_id="abc12345")
        module._detection_snapshot = [det]

        # Truncated prefix match
        result = module._find_object_in_detections("anything", object_id="abc1")
        assert result is det

    def test_find_by_object_id_ambiguous_returns_none(self, module):
        det1 = _make_det_object(object_id="abc12345")
        det2 = _make_det_object(object_id="abc19999")
        module._detection_snapshot = [det1, det2]

        result = module._find_object_in_detections("anything", object_id="abc1")
        assert result is None

    def test_find_missing_returns_none(self, module):
        module._detection_snapshot = [_make_det_object(name="bottle")]

        result = module._find_object_in_detections("keyboard")
        assert result is None

    def test_empty_snapshot_returns_none(self, module):
        module._detection_snapshot = []

        result = module._find_object_in_detections("cup")
        assert result is None


class TestGraspHeuristics:
    """Test grasp orientation and occlusion offset static methods."""

    def test_occlusion_offset_toward_robot(self):
        center = Vector3(x=0.5, y=0.0, z=0.3)
        size = Vector3(x=0.1, y=0.1, z=0.1)

        ox, oy = PickAndPlaceModule._occlusion_offset(center, size)
        # Offset should shift x closer to robot origin (smaller x)
        assert ox < center.x
        assert abs(oy - center.y) < 1e-6  # y should stay ~0

    def test_occlusion_offset_at_origin(self):
        center = Vector3(x=0.0, y=0.0, z=0.3)
        size = Vector3(x=0.1, y=0.1, z=0.1)

        ox, oy = PickAndPlaceModule._occlusion_offset(center, size)
        # At origin, no shift should occur (division-by-zero guard)
        assert abs(ox) < 1e-3
        assert abs(oy) < 1e-3

    def test_grasp_orientation_near_is_top_down(self):
        q = PickAndPlaceModule._grasp_orientation(gx=0.3, gy=0.0, xy_dist=0.3)
        # Near object: pitch = 180° (top-down), tilt = 0, yaw = 0
        # RPY(0, π, 0) → quaternion (x=0, y=1, z=0, w=0)
        assert abs(q.x) < 0.01
        assert abs(q.y - 1.0) < 0.01
        assert abs(q.z) < 0.01
        assert abs(q.w) < 0.01

    def test_grasp_orientation_far_differs_from_near(self):
        q_near = PickAndPlaceModule._grasp_orientation(gx=0.3, gy=0.0, xy_dist=0.3)
        q_far = PickAndPlaceModule._grasp_orientation(gx=1.0, gy=0.0, xy_dist=1.0)
        # Far object should have different orientation (tilted)
        assert not (
            abs(q_near.x - q_far.x) < 0.01
            and abs(q_near.y - q_far.y) < 0.01
            and abs(q_near.z - q_far.z) < 0.01
            and abs(q_near.w - q_far.w) < 0.01
        )

    def test_exact_simulation_detection_does_not_apply_occlusion_offset(self, module):
        det = _make_det_object(name="can", center=(0.5, 0.0, 0.19))
        det.identity_basis = "pimsim_scene"
        module._detection_snapshot = [det]

        grasps = module._generate_grasps_for_pick("can")

        assert grasps is not None
        assert grasps[0].position.x == pytest.approx(0.5)
        assert grasps[0].position.y == pytest.approx(0.0)


class TestPlaceBack:
    """Test place_back guard logic."""

    def test_place_back_no_pick_pose_errors(self, module):
        module._last_pick_pose = None

        result = module.place_back()
        assert not result.is_success()
        assert result.error_code == "NO_PRIOR_POSE"
        assert "pick" in result.message.lower()


class TestPickPlaceObstacleLifecycle:
    def _prepare_motion(self, module: PickAndPlaceModule) -> None:
        module._get_robot = MagicMock(
            return_value=("arm", "robot_1", SimpleNamespace(pre_grasp_offset=0.1), None)
        )
        module._lift_if_low = MagicMock(return_value=SkillResult.ok())
        module._preview_execute_wait = MagicMock(return_value=SkillResult.ok())
        module._set_gripper_position = MagicMock(return_value=True)
        module._world_monitor = MagicMock()
        module._world_monitor.remove_object_obstacle.return_value = True

    def test_pick_removes_target_for_contact_and_holds_gripper_state(self, module):
        self._prepare_motion(module)
        target = _make_det_object(name="can", object_id="pimsim:manip_can")
        module._detection_snapshot = [target]
        grasp = Pose(Vector3(0.5, 0.0, 0.25), Quaternion())
        module._generate_grasps_for_pick = MagicMock(return_value=[grasp])
        module.plan_to_pose = MagicMock(return_value=True)

        with patch("dimos.manipulation.pick_and_place_module.time.sleep"):
            result = module.pick("can")

        assert result.is_success()
        module._world_monitor.remove_object_obstacle.assert_called_once_with("pimsim:manip_can")
        assert module._set_gripper_position.call_args_list == [call(0.85, "arm"), call(0.0, "arm")]
        assert module._held_object_id == "pimsim:manip_can"

    def test_pick_restores_target_when_contact_plan_fails(self, module):
        self._prepare_motion(module)
        target = _make_det_object(name="can", object_id="pimsim:manip_can")
        module._detection_snapshot = [target]
        module._generate_grasps_for_pick = MagicMock(
            return_value=[Pose(Vector3(0.5, 0.0, 0.25), Quaternion())]
        )
        module.plan_to_pose = MagicMock(side_effect=[True, False])
        module.refresh_obstacles = MagicMock(return_value=[])

        with patch("dimos.manipulation.pick_and_place_module.time.sleep"):
            result = module.pick("can")

        assert not result.is_success()
        module.refresh_obstacles.assert_called_once_with()
        assert module._held_object_id is None

    def test_place_releases_then_restores_perception_obstacles(self, module):
        self._prepare_motion(module)
        module._held_object_id = "pimsim:manip_can"
        module.plan_to_pose = MagicMock(return_value=True)
        module.refresh_obstacles = MagicMock(return_value=[])

        with patch("dimos.manipulation.pick_and_place_module.time.sleep"):
            result = module.place(0.45, 0.1, 0.19)

        assert result.is_success()
        module._set_gripper_position.assert_called_once_with(0.85, "arm")
        module.refresh_obstacles.assert_called_once_with()
        assert module._held_object_id is None

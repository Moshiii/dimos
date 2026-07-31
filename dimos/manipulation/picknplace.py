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

"""Request-driven perception interface for the pick-and-place workflow."""

import math
import threading
from typing import Literal

import numpy as np

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.manipulation.grasping.grasp_gen_spec import GraspGenSpec
from dimos.manipulation.visualization.layers import (
    LineSetElement,
    PointCloudElement,
    VisualizationLayer,
)
from dimos.manipulation.visualization.pose_overlay import draw_pose_axes
from dimos.manipulation.visualization_spec import ManipulationVisualizationSpec
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.vision_msgs.Detection3DArray import Detection3DArray
from dimos.perception.detection.type.detection3d.object import (
    Object as DetObject,
    to_detection3d_array,
)
from dimos.perception.object_scene_registration_spec import ObjectSceneRegistrationSpec


class PickNPlaceConfig(ModuleConfig):
    """Configuration for PickNPlaceModule."""

    align_grasp_yaw: bool = False
    grasp_strategy: Literal["obb_center", "graspgenx"] = "obb_center"
    graspgenx_pregrasp_offset: float = 0.10


class PickNPlaceModule(Module):
    """Provide request-driven perception and target selection for pick and place."""

    config: PickNPlaceConfig
    _scene: ObjectSceneRegistrationSpec
    _grasp_generator: GraspGenSpec | None
    _visualization: ManipulationVisualizationSpec
    objects: In[list[DetObject]]
    camera_info: In[CameraInfo]
    basic_grasp_overlay: Out[Image]
    graspgenx_candidates: Out[GraspCandidateArray]

    def __init__(self, **kwargs: object) -> None:
        super().__init__(**kwargs)
        self._objects_condition = threading.Condition()
        self._latest_objects: tuple[DetObject, ...] = ()
        self._objects_version = 0
        self._camera_info: CameraInfo | None = None
        self._goal_pose: PoseStamped | None = None
        self._pre_grasp_pose: PoseStamped | None = None
        self._grasp_candidates: GraspCandidateArray | None = None
        self._selected_object: DetObject | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self.objects.subscribe(self._on_objects)
        self.camera_info.subscribe(self._on_camera_info)

    def _on_objects(self, objects: list[DetObject]) -> None:
        with self._objects_condition:
            self._latest_objects = tuple(objects)
            self._objects_version += 1
            self._objects_condition.notify_all()

    def _on_camera_info(self, camera_info: CameraInfo) -> None:
        with self._objects_condition:
            self._camera_info = camera_info

    @rpc
    def scan_scene(self, prompt: str | None = None) -> Detection3DArray:
        """Run one RGB-D detection pass, optionally targeting one text prompt."""
        with self._objects_condition:
            objects_version = self._objects_version
        if prompt:
            self._scene.set_prompts([prompt])
        detections = self._scene.scan_scene()
        with self._objects_condition:
            received_result = self._objects_condition.wait_for(
                lambda: self._objects_version > objects_version,
                timeout=5.0,
            )
            objects = self._latest_objects
        if received_result:
            # Stream delivery crosses process boundaries and can lag the OSR RPC response.
            # Return the same snapshot used by the object/grasp APIs, not the prior response.
            return to_detection3d_array(
                list(objects),
                frame_id=objects[0].frame_id if objects else detections.frame_id,
                ts=objects[0].ts if objects else detections.ts,
            )
        return detections

    @rpc
    def get_scene_info(self) -> list[dict[str, object]]:
        """Return the number, name, and confidence for current detections."""
        with self._objects_condition:
            objects = self._latest_objects
        return [
            {
                "number": number,
                "name": obj.name,
                "confidence": obj.confidence,
            }
            for number, obj in enumerate(objects, 1)
        ]

    @rpc
    def get_goal_pose(self, number: int) -> PoseStamped | None:
        """Select an object and return its downward-facing, floor-clamped grasp goal."""
        selection = self._basic_grasp(number)
        if selection is None:
            return None
        grasp, obj = selection
        if self.config.grasp_strategy == "graspgenx":
            if self._grasp_generator is None:
                raise RuntimeError("GraspGenX is not configured for this pick-and-place blueprint")
            candidates = self._grasp_generator.propose_grasps(obj.pointcloud)
            self._selected_object = obj
            self._grasp_candidates = candidates
            if not candidates.candidates:
                return None
            return self._select_graspgenx_candidate(0)
        yaw = self._grasp_yaw(obj) if self.config.align_grasp_yaw else 0.0
        self._grasp_candidates = None
        self._selected_object = None
        self.graspgenx_candidates.publish(GraspCandidateArray())
        self._goal_pose = PoseStamped(
            ts=grasp.ts,
            frame_id=grasp.frame_id,
            position=Vector3(grasp.position.x, grasp.position.y, max(grasp.position.z, 0.100)),
            orientation=Quaternion.from_euler(Vector3(-math.pi, 0.0, yaw)),
        )
        self._pre_grasp_pose = None
        return self._goal_pose

    @rpc
    def select_grasp_candidate(self, rank: int) -> PoseStamped | None:
        """Select one ranked GraspGenX proposal as the goal and Rerun highlight."""
        return self._select_graspgenx_candidate(rank)

    def _select_graspgenx_candidate(self, rank: int) -> PoseStamped | None:
        candidates = self._grasp_candidates
        if candidates is None or rank < 0 or rank >= len(candidates.candidates):
            return None
        candidates.selected_index = rank
        self.graspgenx_candidates.publish(candidates)
        candidate = candidates.candidates[rank]
        self._goal_pose = PoseStamped(
            ts=candidates.header.timestamp,
            frame_id=candidates.header.frame_id,
            position=candidate.pose.position,
            orientation=candidate.pose.orientation,
        )
        self._pre_grasp_pose = None
        return self._goal_pose

    @rpc
    def get_pre_grasp_pose(self) -> PoseStamped | None:
        """Return the selected goal offset 100 mm opposite its final approach direction."""
        if self._goal_pose is None:
            return None
        if self.config.grasp_strategy == "graspgenx":
            offset = self._goal_pose.orientation.rotate_vector(
                # GraspGenX local +Z points in the direction of the final
                # approach. A pre-grasp retreats along the opposite axis.
                Vector3(0.0, 0.0, -self.config.graspgenx_pregrasp_offset)
            )
        else:
            offset = Vector3(0.0, 0.0, 0.100)
        self._pre_grasp_pose = PoseStamped(
            ts=self._goal_pose.ts,
            frame_id=self._goal_pose.frame_id,
            position=Vector3(
                self._goal_pose.position.x + offset.x,
                self._goal_pose.position.y + offset.y,
                self._goal_pose.position.z + offset.z,
            ),
            orientation=self._goal_pose.orientation,
        )
        self._publish_viser_selection()
        return self._pre_grasp_pose

    def _publish_viser_selection(self) -> None:
        """Show the selected object and TCP targets without mutating the planning scene."""
        obj = self._selected_object
        goal = self._goal_pose
        pre_grasp = self._pre_grasp_pose
        if obj is None or goal is None or pre_grasp is None:
            return
        points = obj.pointcloud.points_f32()
        if len(points) == 0:
            return
        cloud_colors = np.repeat(np.array([[255, 190, 70]], dtype=np.uint8), len(points), axis=0)
        vertices: list[np.ndarray] = []
        edges: list[list[int]] = []
        colors: list[list[int]] = []
        for pose, color in ((goal, [255, 70, 70]), (pre_grasp, [70, 255, 120])):
            start = len(vertices)
            origin = np.asarray(pose.position.as_tuple, dtype=np.float32)
            axes = pose.orientation.to_rotation_matrix().astype(np.float32) * 0.06
            vertices.extend((origin, origin + axes[:, 0], origin + axes[:, 1], origin + axes[:, 2]))
            edges.extend(((start, start + 1), (start, start + 2), (start, start + 3)))
            colors.extend((color, color, color))
        self._visualization.set_visualization_layer(
            VisualizationLayer(
                "picknplace/selection",
                "world",
                (
                    PointCloudElement("object", points, cloud_colors, point_size=0.003),
                    LineSetElement(
                        "tcp-targets",
                        np.asarray(vertices),
                        np.asarray(edges),
                        np.asarray(colors),
                        line_width=0.5,
                    ),
                ),
            )
        )

    @rpc
    def get_grasp_candidates(self) -> GraspCandidateArray:
        """Return the GraspGenX proposals generated for the selected object."""
        return self._grasp_candidates or GraspCandidateArray()

    def _basic_grasp(self, number: int) -> tuple[PoseStamped, DetObject] | None:
        """Return the selected cloud's OBB-center grasp frame and object geometry."""
        with self._objects_condition:
            if number < 1 or number > len(self._latest_objects):
                return None
            obj = self._latest_objects[number - 1]
            camera_info = self._camera_info
        grasp = PoseStamped(
            ts=obj.ts,
            frame_id=obj.frame_id,
            position=obj.center,
            orientation=obj.pose.orientation,
        )
        if camera_info is not None and obj.camera_transform is not None and obj.image is not None:
            if overlay := draw_pose_axes(
                obj.image, grasp, obj.camera_transform.inverse(), camera_info
            ):
                self.basic_grasp_overlay.publish(overlay)
        return grasp, obj

    @staticmethod
    def _grasp_yaw(obj: DetObject) -> float:
        """Align the gripper's local Y closing axis with the narrowest horizontal OBB axis."""
        rotation = obj.pose.orientation.to_rotation_matrix()
        extents = (obj.size.x, obj.size.y, obj.size.z)
        horizontal_axes = sorted(range(3), key=lambda axis: abs(rotation[2, axis]))[:2]
        narrow_axis = min(horizontal_axes, key=lambda axis: extents[axis])
        return math.atan2(rotation[1, narrow_axis], rotation[0, narrow_axis]) - math.pi / 2

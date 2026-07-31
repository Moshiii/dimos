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

"""Rerun configuration for the pick-and-place workflow."""

from functools import partial
from typing import Any, cast

import rerun.blueprint as rrb


def picknplace_rerun_config() -> dict[str, Any]:
    """Return the Rerun layout and message conversions for pick and place."""
    return {
        "blueprint": _blueprint,
        "topic_to_entity": _topic_to_entity,
        "visual_override": {
            "world/color_camera": partial(
                _camera_info_to_rerun,
                image_topic="world/color_camera/color_image",
            ),
            "world/pointcloud": _pointcloud_to_rerun,
            "world/detections_3d": None,
            "world/depth_camera": None,
            "world/depth_camera/depth_image": None,
        },
    }


def _blueprint() -> rrb.Blueprint:
    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Vertical(
                rrb.Spatial2DView(origin="world/annotated_image", name="YOLO-E Segmentation"),
                rrb.Spatial2DView(origin="world/basic_grasp_overlay", name="Grasp Pose"),
                rrb.Spatial2DView(origin="world/color_camera/color_image", name="RGB"),
            ),
            rrb.Spatial3DView(origin="world", name="Filtered Objects"),
        )
    )


def _topic_to_entity(topic: Any) -> str:
    topic_name = str(getattr(topic, "name", topic)).split("#", 1)[0]
    return {
        "/color_image": "world/color_camera/color_image",
        "/camera_info": "world/color_camera",
        "/depth_image": "world/depth_camera/depth_image",
        "/depth_camera_info": "world/depth_camera",
        "/basic_grasp_overlay": "world/basic_grasp_overlay",
        "/detections_3d": "world/detections_3d",
        "/pointcloud": "world/pointcloud",
    }.get(topic_name, f"world/{topic_name.lstrip('/')}")


def _camera_info_to_rerun(msg: Any, image_topic: str) -> list[tuple[str, Any]]:
    return cast(
        "list[tuple[str, Any]]",
        msg.to_rerun(image_topic=image_topic, optical_frame=getattr(msg, "frame_id", None)),
    )


def _pointcloud_to_rerun(msg: Any) -> Any:
    return msg.to_rerun(voxel_size=0.001, mode="points")

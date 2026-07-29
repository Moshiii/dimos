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
import math
from typing import Any

from dimos_lcm.vision_msgs.Detection3DArray import Detection3DArray as LCMDetection3DArray

from dimos.types.timestamped import to_timestamp
from dimos.visualization.rerun.in_frame import framed
from dimos.visualization.rerun.palette import color_for_label

_FLAT_HALF_Z = 0.005
"""Half-thickness rerun renders a flat (z=0) detection with."""


def _finite(*values: float) -> bool:
    return all(math.isfinite(v) for v in values)


class Detection3DArray(LCMDetection3DArray):  # type: ignore[misc]
    msg_name = "vision_msgs.Detection3DArray"

    # for _get_field_type() to work when decoding in _decode_one()
    __annotations__ = LCMDetection3DArray.__annotations__

    @property
    def ts(self) -> float:
        return to_timestamp(self.header.stamp)

    @property
    def frame_id(self) -> str:
        return str(self.header.frame_id)

    def to_rerun(self) -> Any:
        """Boxes once the detector solves a pose, points until then.

        Both states are one Boxes3D: rerun draws a zero-extent box as a point, and
        mixing archetypes on the entity leaves the unused one resolving to NaN.
        """
        import rerun as rr

        centers: list[tuple[float, float, float]] = []
        half_sizes: list[tuple[float, float, float]] = []
        quaternions: list[tuple[float, float, float, float]] = []
        labels: list[str] = []

        for detection in self.detections[: self.detections_length]:
            bbox = detection.bbox
            center = bbox.center.position
            orientation = bbox.center.orientation
            size = bbox.size

            if _finite(
                center.x,
                center.y,
                center.z,
                size.x,
                size.y,
                size.z,
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w,
            ):
                half_z = _FLAT_HALF_Z if size.z == 0.0 else size.z / 2.0
                centers.append((center.x, center.y, center.z))
                half_sizes.append((size.x / 2.0, size.y / 2.0, half_z))
                quaternions.append((orientation.x, orientation.y, orientation.z, orientation.w))
            else:
                point = _point_for_detection(detection)
                if point is None:
                    continue
                centers.append(point)
                half_sizes.append((0.0, 0.0, 0.0))  # degenerate: drawn as a point
                quaternions.append((0.0, 0.0, 0.0, 1.0))
            labels.append(_label_for_detection(detection))

        boxes = rr.Boxes3D(
            centers=centers,
            half_sizes=half_sizes,
            quaternions=quaternions,
            labels=labels,
            colors=[color_for_label(label) for label in labels],
        )
        return framed(boxes, self.frame_id)


def _point_for_detection(detection: Any) -> tuple[float, float, float] | None:
    """Where a detection sits when its pose hasn't been solved, if known at all."""
    for result in detection.results[: detection.results_length]:
        position = result.pose.pose.position
        if _finite(position.x, position.y, position.z):
            return (position.x, position.y, position.z)
    return None


def _label_for_detection(detection: Any) -> str:
    marker_id = str(getattr(detection, "id", "")).strip()
    for result in detection.results[: detection.results_length]:
        class_id = str(result.hypothesis.class_id).strip()
        if marker_id and class_id:
            return f"{class_id} id={marker_id}"
        if class_id:
            return class_id
    if marker_id:
        return f"id={marker_id}"
    return ""

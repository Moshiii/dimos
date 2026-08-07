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

"""Absolute Cartesian pose leaf over the shared bounded Pink IK task."""

from __future__ import annotations

from dataclasses import dataclass
import threading
from typing import Any

from pydantic import Field

from dimos.control.task import CoordinatorState
from dimos.control.tasks.pose_target_ik import (
    FrameTargetSnapshot,
    PoseTargetIKTask,
    PoseTargetIKTaskConfig,
)
from dimos.manipulation.planning.kinematics.config import PinkKinematicsConfig
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.protocol.service.spec import BaseConfig


@dataclass(frozen=True)
class CartesianIKTaskConfig(PoseTargetIKTaskConfig):
    """Configuration for one absolute Cartesian target frame."""

    def __post_init__(self) -> None:
        if len(self.target_frames) != 1:
            raise ValueError("CartesianIKTask requires exactly one target frame")


class CartesianIKTask(PoseTargetIKTask):
    """Track one stream of absolute poses with the shared Pink control core."""

    def __init__(self, name: str, config: CartesianIKTaskConfig) -> None:
        self._lock = threading.Lock()
        self._target_pose: PoseStamped | None = None
        self._last_update_time = 0.0
        self._active = False
        super().__init__(name, config)

    def is_active(self) -> bool:
        with self._lock:
            return self._active and self._target_pose is not None

    def on_cartesian_command(self, pose: Pose | PoseStamped, t_now: float) -> bool:
        """Accept an absolute target pose and activate tracking."""
        target = PoseStamped(
            ts=pose.ts if isinstance(pose, PoseStamped) else 0.0,
            frame_id=pose.frame_id if isinstance(pose, PoseStamped) else "",
            position=pose.position,
            orientation=pose.orientation,
        )
        with self._lock:
            self._target_pose = target
            self._last_update_time = t_now
            self._active = True
        return True

    def start(self) -> None:
        with self._lock:
            self._active = True

    def stop(self) -> None:
        with self._lock:
            self._active = False

    def clear(self) -> None:
        with self._lock:
            self._target_pose = None
            self._active = False

    def is_tracking(self) -> bool:
        return self.is_active()

    def _frame_target_snapshot(self, state: CoordinatorState) -> FrameTargetSnapshot | None:
        with self._lock:
            if not self._active or self._target_pose is None:
                return None
            return FrameTargetSnapshot(
                targets={self._config.target_frames[0]: self._target_pose},
                last_update_time=self._last_update_time,
            )

    def _on_target_timeout(self) -> None:
        self.clear()

    def _on_pose_target_preempted(self, by_task: str, joints: frozenset[str]) -> None:
        self.clear()


class CartesianIKTaskParams(BaseConfig):
    """Task-owned parameters carried inside the generic task envelope."""

    robot_model: RobotModelConfig
    target_frame: str
    pink: PinkKinematicsConfig = Field(default_factory=PinkKinematicsConfig)
    timeout: float = 0.5
    max_joint_delta_deg: float = 15.0


def create_task(cfg: Any, hardware: Any) -> CartesianIKTask:
    """Create an absolute Cartesian Pink task from a registry configuration."""
    params = CartesianIKTaskParams.model_validate(cfg.params)
    return CartesianIKTask(
        cfg.name,
        CartesianIKTaskConfig(
            joint_names=tuple(cfg.joint_names),
            robot_model=params.robot_model,
            target_frames=(params.target_frame,),
            pink=params.pink,
            priority=cfg.priority,
            timeout=params.timeout,
            max_joint_delta_deg=params.max_joint_delta_deg,
        ),
    )

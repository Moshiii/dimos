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

"""Shared bounded Pink IK machinery for streaming pose-target tasks."""

from __future__ import annotations

from abc import abstractmethod
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
import threading
from typing import TYPE_CHECKING

import numpy as np

from dimos.control.task import (
    BaseControlTask,
    ControlMode,
    CoordinatorState,
    JointCommandOutput,
    ResourceClaim,
)
from dimos.manipulation.planning.kinematics.config import PinkKinematicsConfig
from dimos.manipulation.planning.kinematics.pink_ik import PinkIK, PinkIKFeedbackLimitError
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped

logger = setup_logger()

_FEEDBACK_LIMIT_WARNING_INTERVAL_S = 1.0
_HARD_TRACKING_ERROR_MULTIPLIER = 2.0


@dataclass(frozen=True)
class PoseTargetIKTaskConfig:
    """Configuration shared by absolute and Quest pose-target tasks."""

    joint_names: tuple[str, ...]
    robot_model: RobotModelConfig
    target_frames: tuple[str, ...]
    pink: PinkKinematicsConfig = field(default_factory=PinkKinematicsConfig)
    priority: int = 10
    timeout: float = 0.5
    max_joint_delta_deg: float = 5.0
    max_joint_velocity_rad_s: float | None = None
    max_command_tracking_error_deg: float = 10.0


@dataclass(frozen=True)
class FrameTargetSnapshot:
    """One atomic control-tick snapshot produced by a task leaf."""

    targets: Mapping[str, PoseStamped]
    last_update_time: float
    extra_joint_positions: Mapping[str, float] = field(default_factory=dict)


class PoseTargetIKTask(BaseControlTask):
    """Turn frame targets into a feedback-bounded persistent command trajectory."""

    def __init__(
        self,
        name: str,
        config: PoseTargetIKTaskConfig,
        *,
        additional_claimed_joints: Sequence[str] = (),
        ik: PinkIK | None = None,
    ) -> None:
        if not config.joint_names:
            raise ValueError(f"PoseTargetIKTask '{name}' requires at least one joint")
        if len(set(config.joint_names)) != len(config.joint_names):
            raise ValueError(f"PoseTargetIKTask '{name}' requires unique joint names")
        if not config.target_frames:
            raise ValueError(f"PoseTargetIKTask '{name}' requires at least one target frame")
        if len(set(config.target_frames)) != len(config.target_frames):
            raise ValueError(f"PoseTargetIKTask '{name}' requires unique target frames")
        if not np.isfinite(config.max_joint_delta_deg) or config.max_joint_delta_deg <= 0.0:
            raise ValueError(
                f"PoseTargetIKTask '{name}' requires a positive finite joint delta limit"
            )
        if config.max_joint_velocity_rad_s is not None and (
            not np.isfinite(config.max_joint_velocity_rad_s)
            or config.max_joint_velocity_rad_s <= 0.0
        ):
            raise ValueError(
                f"PoseTargetIKTask '{name}' requires a positive finite joint velocity limit"
            )
        if (
            not np.isfinite(config.max_command_tracking_error_deg)
            or config.max_command_tracking_error_deg <= 0.0
        ):
            raise ValueError(
                f"PoseTargetIKTask '{name}' requires a positive finite command tracking error"
            )

        additional_joints = tuple(additional_claimed_joints)
        if set(config.joint_names) & set(additional_joints):
            raise ValueError(
                f"PoseTargetIKTask '{name}' has duplicate IK and additional claimed joints"
            )
        if len(set(additional_joints)) != len(additional_joints):
            raise ValueError(f"PoseTargetIKTask '{name}' requires unique additional joints")

        self._name = name
        self._config = config
        self._joint_names = config.joint_names
        self._additional_claimed_joints = additional_joints
        self._feedback_limit_warning_times: dict[tuple[str, str], float] = {}
        self._command_state_lock = threading.Lock()
        self._command_state: JointState | None = None
        self._command_state_generation = 0
        self._command_tracking_faulted = False
        self._ik = ik or PinkIK(config.pink)
        self._ik.validate_frame_targets(
            config.robot_model, config.target_frames, config.joint_names
        )

    def claim(self) -> ResourceClaim:
        """Claim IK-controlled and leaf-provided additional joints."""
        return ResourceClaim(
            joints=frozenset((*self._joint_names, *self._additional_claimed_joints)),
            priority=self._config.priority,
            mode=ControlMode.SERVO_POSITION,
        )

    def compute(self, state: CoordinatorState) -> JointCommandOutput | None:
        """Perform at most one bounded Pink update for this coordinator tick."""
        snapshot = self._frame_target_snapshot(state)
        if snapshot is None:
            return None
        if self._config.timeout > 0.0:
            age = state.t_now - snapshot.last_update_time
            if age > self._config.timeout:
                self._reset_command_state()
                self._on_target_timeout()
                return None

        measured_state = self._measured_joint_state(state)
        if measured_state is None:
            self._reset_command_state()
            return None
        with self._command_state_lock:
            if self._command_tracking_faulted:
                return None
            if self._command_state is None:
                self._command_state = _copy_joint_state(measured_state)
            command_state = _copy_joint_state(self._command_state)
            command_generation = self._command_state_generation

            tracking_errors = np.abs(
                np.asarray(command_state.position, dtype=np.float64)
                - np.asarray(measured_state.position, dtype=np.float64)
            )
            hard_limit_rad = np.deg2rad(
                self._config.max_command_tracking_error_deg * _HARD_TRACKING_ERROR_MULTIPLIER
            )
            if np.any(tracking_errors > hard_limit_rad):
                worst_index = int(np.argmax(tracking_errors))
                self._command_state = None
                self._command_state_generation += 1
                self._command_tracking_faulted = True
                logger.warning(
                    "Pose-target command exceeded hard tracking error",
                    task=self._name,
                    joint=self._joint_names[worst_index],
                    error_deg=float(np.rad2deg(tracking_errors[worst_index])),
                    limit_deg=float(np.rad2deg(hard_limit_rad)),
                )
                return None
        try:
            result = self._ik.step_frame_targets(
                robot_model=self._config.robot_model,
                frame_targets=snapshot.targets,
                controlled_joints=self._joint_names,
                command_state=command_state,
                measured_state=measured_state,
                max_command_tracking_error_rad=float(
                    np.deg2rad(self._config.max_command_tracking_error_deg)
                ),
                dt=state.dt,
                max_joint_delta_rad=float(np.deg2rad(self._config.max_joint_delta_deg)),
                max_joint_velocity_rad_s=self._config.max_joint_velocity_rad_s,
            )
        except PinkIKFeedbackLimitError as exc:
            warning_key = (exc.joint_name, exc.boundary)
            last_warning = self._feedback_limit_warning_times.get(warning_key)
            if (
                last_warning is None
                or state.t_now - last_warning >= _FEEDBACK_LIMIT_WARNING_INTERVAL_S
            ):
                logger.warning(
                    "Measured joint feedback exceeds Pink limit tolerance",
                    task=self._name,
                    joint=exc.joint_name,
                    value=exc.value,
                    lower=exc.lower,
                    upper=exc.upper,
                    tolerance=exc.tolerance,
                )
                self._feedback_limit_warning_times[warning_key] = state.t_now
            self._reset_command_state()
            return None
        # Pink/QP failures derive directly from Exception rather than a stable
        # built-in subtype. A failed streaming tick must not stop the coordinator.
        except Exception as exc:
            logger.warning("Pink control step failed", task=self._name, error=str(exc))
            return None
        if result.name != list(self._joint_names) or len(result.position) != len(self._joint_names):
            logger.warning("Pink control step returned an invalid joint result", task=self._name)
            return None

        current = np.asarray(command_state.position, dtype=np.float64)
        positions = np.asarray(result.position, dtype=np.float64)
        if not np.all(np.isfinite(positions)):
            logger.warning("Pink control step returned non-finite positions", task=self._name)
            return None
        delta_deg = np.rad2deg(np.abs(positions - current))
        if np.any(delta_deg > self._config.max_joint_delta_deg):
            worst_index = int(np.argmax(delta_deg))
            logger.warning(
                "Rejecting Pink control step above joint delta limit",
                task=self._name,
                joint=self._joint_names[worst_index],
                delta_deg=float(delta_deg[worst_index]),
                limit_deg=self._config.max_joint_delta_deg,
            )
            return None

        output_names = list(self._joint_names)
        output_positions = positions.tolist()
        for joint_name in self._additional_claimed_joints:
            if joint_name not in snapshot.extra_joint_positions:
                logger.warning(
                    "Pose-target task snapshot omitted an additional claimed joint",
                    task=self._name,
                    joint=joint_name,
                )
                return None
            output_names.append(joint_name)
            output_positions.append(float(snapshot.extra_joint_positions[joint_name]))
        with self._command_state_lock:
            if (
                self._command_tracking_faulted
                or self._command_state_generation != command_generation
            ):
                return None
            self._command_state = JointState(
                name=list(self._joint_names),
                position=positions.tolist(),
            )
        return JointCommandOutput(
            joint_names=output_names,
            positions=output_positions,
            mode=ControlMode.SERVO_POSITION,
        )

    def current_frame_poses(
        self, state: CoordinatorState, frame_names: Sequence[str]
    ) -> dict[str, PoseStamped] | None:
        """Return live poses for task frames, or ``None`` without full feedback."""
        measured_state = self._measured_joint_state(state)
        if measured_state is None:
            return None
        return self._ik.frame_poses(
            self._config.robot_model,
            frame_names,
            self._joint_names,
            measured_state,
        )

    def on_preempted(self, by_task: str, joints: frozenset[str]) -> None:
        """Notify the leaf when any of its claimed joints are preempted."""
        if joints & self.claim().joints:
            self._reset_command_state()
            self._on_pose_target_preempted(by_task, joints)

    def _measured_joint_state(self, state: CoordinatorState) -> JointState | None:
        positions: list[float] = []
        for joint_name in self._joint_names:
            position = state.joints.get_position(joint_name)
            if position is None:
                return None
            positions.append(position)
        return JointState(name=list(self._joint_names), position=positions)

    def _reset_command_state(self) -> None:
        """Discard the active command trajectory and clear a tracking fault."""
        with self._command_state_lock:
            self._command_state = None
            self._command_state_generation += 1
            self._command_tracking_faulted = False

    @abstractmethod
    def _frame_target_snapshot(self, state: CoordinatorState) -> FrameTargetSnapshot | None:
        """Return the leaf's atomic target snapshot for this tick."""

    def _on_target_timeout(self) -> None:
        """Allow a leaf to clear state after a stale snapshot."""

    def _on_pose_target_preempted(self, by_task: str, joints: frozenset[str]) -> None:
        """Allow a leaf to reset semantics after preemption."""


def _copy_joint_state(state: JointState) -> JointState:
    return JointState(name=list(state.name), position=list(state.position))

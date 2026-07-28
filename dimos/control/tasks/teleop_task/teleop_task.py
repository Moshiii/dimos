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

"""Engagement-relative teleop control through measured-state Pink IK."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Literal

import numpy as np
import pinocchio
from pydantic import Field, FiniteFloat

from dimos.control.coordinator import TaskConfig
from dimos.control.task import CoordinatorState, JointCommandOutput, ResourceClaim
from dimos.control.tasks.cartesian_ik_task.cartesian_ik_task import (
    CartesianIKTask,
    CartesianIKTaskConfig,
)
from dimos.control.tasks.cartesian_ik_task.pink_control_ik import PinkControlIKConfig
from dimos.protocol.service.spec import BaseConfig
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from numpy.typing import NDArray

    from dimos.msgs.geometry_msgs.Pose import Pose
    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
    from dimos.teleop.quest.quest_types import Buttons

logger = setup_logger()


class TeleopControlIKConfig(PinkControlIKConfig):
    """Pink control policy for engagement-relative arm teleoperation."""

    max_velocity: FiniteFloat = Field(2.0, gt=0.0)
    position_cost: FiniteFloat = Field(1.0, ge=0.0)
    orientation_cost: FiniteFloat = Field(1.0, ge=0.0)
    posture_cost: FiniteFloat = Field(0.0, ge=0.0)
    damping_cost: FiniteFloat = Field(1e-3, ge=0.0)


@dataclass
class TeleopIKTaskConfig(CartesianIKTaskConfig):
    """Configuration for engagement-relative teleop IK."""

    max_joint_delta_deg: float = 5.0
    max_step_deg_per_tick: float | None = None
    max_target_offset_m: float | None = None
    max_target_rot_deg: float | None = None
    joint_limit_margin_deg: float = 0.0
    tool_offset_m: tuple[float, float, float] | None = None
    rotation_frame: Literal["world", "local"] = "world"
    rotation_deadband_deg: float = 0.0
    hand: Literal["left", "right"] | None = None
    gripper_joint: str | None = None
    gripper_open_pos: float = 0.0
    gripper_closed_pos: float = 0.0

    def __post_init__(self) -> None:
        super().__post_init__()
        positive_optional = {
            "max_step_deg_per_tick": self.max_step_deg_per_tick,
            "max_target_offset_m": self.max_target_offset_m,
            "max_target_rot_deg": self.max_target_rot_deg,
        }
        for field_name, value in positive_optional.items():
            if value is not None and (not np.isfinite(value) or value <= 0.0):
                raise ValueError(f"TeleopIKTask {field_name} must be positive and finite")
        if not np.isfinite(self.joint_limit_margin_deg) or self.joint_limit_margin_deg < 0.0:
            raise ValueError("TeleopIKTask joint_limit_margin_deg must be finite and non-negative")
        if not np.isfinite(self.rotation_deadband_deg) or self.rotation_deadband_deg < 0.0:
            raise ValueError("TeleopIKTask rotation_deadband_deg must be finite and non-negative")
        if self.tool_offset_m is not None and (
            len(self.tool_offset_m) != 3 or not np.all(np.isfinite(self.tool_offset_m))
        ):
            raise ValueError("TeleopIKTask tool_offset_m must contain three finite values")


class TeleopIKTask(CartesianIKTask):
    """Cartesian IK specialization for engagement-relative teleoperation."""

    _config: TeleopIKTaskConfig

    def __init__(self, name: str, config: TeleopIKTaskConfig) -> None:
        if config.hand not in ("left", "right"):
            raise ValueError(f"TeleopIKTask '{name}' requires hand='left' or 'right'")
        super().__init__(name, config)
        self._tool_offset = (
            pinocchio.SE3(np.eye(3), np.asarray(config.tool_offset_m, dtype=np.float64))
            if config.tool_offset_m is not None
            else None
        )
        self._tool_offset_inv = (
            self._tool_offset.inverse() if self._tool_offset is not None else None
        )
        self._initial_ee_pose: pinocchio.SE3 | None = None
        self._prev_primary = False
        self._estopped = False
        self._rot_deadband_ref: NDArray[np.float64] | None = None
        self._gripper_target = config.gripper_open_pos
        # Telemetry (TELEM ik line, 1 Hz while tracking). seed_oob counts
        # ticks where the MEASURED joints sit outside the model limits:
        # the vendor URDF limits are about 1 degree tighter than the real
        # arm, and a measured seed below a bound fails the Pink solve, so
        # this counter measures how much of a session that costs before
        # any remedy is chosen.
        self._telem_last_emit = 0.0
        self._telem_attempts = 0
        self._telem_computes = 0
        self._telem_rejects = 0
        self._telem_seed_oob = 0
        self._telem_seed_oob_worst = 0.0
        self._telem_step_sat = 0
        self._telem_ask = None
        self._telem_lag_m = 0.0
        self._telem_rot_lag_rad = 0.0
        self._telem_solve_ms_max = 0.0
        self._was_tracking = False
        self._engage_t0 = 0.0
        self._engage_lag_max = 0.0
        self._engage_rejects = 0
        self._engage_computes = 0

    def claim(self) -> ResourceClaim:
        """Claim arm joints and the optional gripper joint."""
        claim = super().claim()
        if self._config.gripper_joint is None:
            return claim
        return ResourceClaim(
            joints=claim.joints | frozenset([self._config.gripper_joint]),
            priority=claim.priority,
            mode=claim.mode,
        )

    def is_active(self) -> bool:
        """Run only when a non-E-STOPped pose target is active."""
        with self._lock:
            return not self._estopped and self._active and self._target_pose is not None

    def is_tracking(self) -> bool:
        """Report whether teleop currently participates in control."""
        return self.is_active()

    def set_estop(self, estopped: bool) -> None:
        """Latch or clear E-STOP without retaining replayable commands."""
        with self._lock:
            self._estopped = estopped
            if estopped:
                self._active = False
                self._target_pose = None
                self._initial_ee_pose = None
                self._prev_primary = False
                self._rot_deadband_ref = None

    def _prepare_target(
        self,
        state: CoordinatorState,
        q_current: NDArray[np.float64],
        dt: float,
    ) -> pinocchio.SE3 | None:
        """Compose the controller delta with the measured engagement baseline."""
        delta = super()._prepare_target(state, q_current, dt)
        if delta is None:
            return None

        with self._lock:
            if self._estopped or self._target_pose is None:
                return None
            baseline = self._initial_ee_pose

        if baseline is None:
            captured = self._tool_fk(q_current)
            values = np.concatenate((captured.translation, captured.rotation.reshape(-1)))
            if not np.all(np.isfinite(values)):
                return None
            with self._lock:
                if self._estopped or self._target_pose is None:
                    return None
                if self._initial_ee_pose is None:
                    self._initial_ee_pose = captured.copy()
                baseline = self._initial_ee_pose

        target_rotation = (
            baseline.rotation @ delta.rotation
            if self._config.rotation_frame == "local"
            else delta.rotation @ baseline.rotation
        )
        target_rotation = self._apply_rotation_deadband(target_rotation)
        target = pinocchio.SE3(
            target_rotation,
            baseline.translation + delta.translation,
        )
        values = np.concatenate((target.translation, target.rotation.reshape(-1)))
        if not np.all(np.isfinite(values)):
            return None
        # Telemetry: the raw ask (pre-window) is what the operator feels
        # the arm lagging behind.
        self._telem_ask = target.copy()
        target = self._windowed_target(q_current, target)
        if self._tool_offset_inv is not None:
            target = target * self._tool_offset_inv
        return target

    def compute(self, state: CoordinatorState) -> JointCommandOutput | None:
        """Run the inherited Pink solve and append the optional gripper target."""
        tracking = bool(getattr(self, "is_tracking", lambda: False)())
        if tracking:
            self._telem_attempts += 1
            q_seed = self._get_current_joints(state)
            limits = getattr(self._ik, "position_limits", None)
            if q_seed is not None and limits is not None:
                lower, upper = limits
                below = np.clip(lower - q_seed, 0.0, None)
                above = np.clip(q_seed - upper, 0.0, None)
                worst = float(np.max(np.maximum(below, above)))
                if worst > 0.0:
                    self._telem_seed_oob += 1
                    self._telem_seed_oob_worst = max(self._telem_seed_oob_worst, worst)
        import time as _time

        t0 = _time.perf_counter()
        output = super().compute(state)
        if tracking:
            self._telem_solve_ms_max = max(
                self._telem_solve_ms_max, (_time.perf_counter() - t0) * 1000.0
            )
            if output is None:
                self._telem_rejects += 1
                self._engage_rejects += 1
            else:
                self._telem_computes += 1
                self._engage_computes += 1
            now_emit = float(getattr(state, "t_now", 0.0))
            if self._telem_last_emit == 0.0:
                self._telem_last_emit = now_emit
            elif now_emit - self._telem_last_emit >= 1.0:
                q_now = self._get_current_joints(state)
                ask = self._telem_ask
                if q_now is not None and ask is not None:
                    current = self._tool_fk(q_now)
                    self._telem_lag_m = float(
                        np.linalg.norm(ask.translation - current.translation)
                    )
                    rot_vec = pinocchio.log3(current.rotation.T @ ask.rotation)
                    self._telem_rot_lag_rad = float(np.linalg.norm(rot_vec))
                    self._engage_lag_max = max(self._engage_lag_max, self._telem_lag_m)
                self._emit_telemetry(state)
        now_t = float(getattr(state, "t_now", 0.0))
        if tracking and not self._was_tracking:
            self._engage_t0 = now_t
            self._engage_lag_max = 0.0
            self._engage_rejects = 0
            self._engage_computes = 0
        elif self._was_tracking and not tracking:
            logger.info(
                "TELEM engage %s: duration_s=%.1f computes=%d rejects=%d lag_max_cm=%.1f",
                self._name,
                now_t - self._engage_t0,
                self._engage_computes,
                self._engage_rejects,
                self._engage_lag_max * 100.0,
            )
        self._was_tracking = tracking
        if output is None:
            return output
        q_current = self._get_current_joints(state)
        if q_current is None or output.positions is None:
            return None
        positions = np.asarray(output.positions, dtype=np.float64)
        limits = getattr(self._ik, "position_limits", None)
        if limits is not None and self._config.joint_limit_margin_deg > 0.0:
            lower, upper = limits
            margin = np.deg2rad(self._config.joint_limit_margin_deg)
            positions = np.clip(positions, lower + margin, upper - margin)
        if self._config.max_step_deg_per_tick is not None:
            step = np.deg2rad(self._config.max_step_deg_per_tick)
            stepped = np.clip(positions - q_current, -step, step)
            if np.max(np.abs(positions - q_current)) > step:
                self._telem_step_sat += 1
            positions = q_current + stepped
        if self._config.gripper_joint is None:
            return JointCommandOutput(
                joint_names=output.joint_names,
                positions=positions.tolist(),
                mode=output.mode,
            )
        with self._lock:
            if self._estopped:
                return None
            gripper_target = self._gripper_target
        return JointCommandOutput(
            joint_names=[*output.joint_names, self._config.gripper_joint],
            positions=[*positions.tolist(), gripper_target],
            mode=output.mode,
        )

    def _emit_telemetry(self, state: CoordinatorState) -> None:
        """One TELEM ik line per second while tracking, then reset counters."""
        now = float(getattr(state, "t_now", 0.0))
        logger.info(
            "TELEM ik %s: computes_hz=%d rejects=%d attempts=%d hand_lag_cm=%.1f "
            "rot_lag_deg=%.1f solve_ms_max=%.1f seed_oob=%d seed_oob_worst_deg=%.2f "
            "step_sat=%d limit_margin_deg=%.1f",
            self._name,
            self._telem_computes,
            self._telem_rejects,
            self._telem_attempts,
            self._telem_lag_m * 100.0,
            np.rad2deg(self._telem_rot_lag_rad),
            self._telem_solve_ms_max,
            self._telem_seed_oob,
            np.rad2deg(self._telem_seed_oob_worst),
            self._telem_step_sat,
            self._config.joint_limit_margin_deg,
        )
        self._telem_last_emit = now
        self._telem_attempts = 0
        self._telem_computes = 0
        self._telem_rejects = 0
        self._telem_seed_oob = 0
        self._telem_seed_oob_worst = 0.0
        self._telem_step_sat = 0
        self._telem_solve_ms_max = 0.0

    def _apply_rotation_deadband(self, target_rotation: NDArray[np.float64]) -> NDArray[np.float64]:
        """Apply a soft angular deadband against the previous target."""
        deadband = np.deg2rad(self._config.rotation_deadband_deg)
        if deadband <= 0.0:
            return target_rotation
        with self._lock:
            reference = self._rot_deadband_ref
            if reference is None:
                self._rot_deadband_ref = target_rotation
                return target_rotation
            rotation_vector = pinocchio.log3(reference.T @ target_rotation)
            angle = float(np.linalg.norm(rotation_vector))
            if angle <= deadband:
                return reference
            filtered = np.asarray(
                reference @ pinocchio.exp3(rotation_vector * ((angle - deadband) / angle))
            )
            self._rot_deadband_ref = filtered
            return filtered

    def _windowed_target(
        self,
        q_current: NDArray[np.float64],
        target: pinocchio.SE3,
    ) -> pinocchio.SE3:
        """Clamp the target into a recentered neighborhood of measured state."""
        if self._config.max_target_offset_m is None and self._config.max_target_rot_deg is None:
            return target
        current = self._tool_fk(q_current)
        position = target.translation
        rotation = target.rotation
        if self._config.max_target_offset_m is not None:
            offset = position - current.translation
            distance = float(np.linalg.norm(offset))
            if distance > self._config.max_target_offset_m:
                position = current.translation + offset * (
                    self._config.max_target_offset_m / distance
                )
        if self._config.max_target_rot_deg is not None:
            rotation_vector = pinocchio.log3(current.rotation.T @ rotation)
            angle = float(np.linalg.norm(rotation_vector))
            max_angle = np.deg2rad(self._config.max_target_rot_deg)
            if angle > max_angle:
                rotation = current.rotation @ pinocchio.exp3(rotation_vector * (max_angle / angle))
        return pinocchio.SE3(rotation, position)

    def _tool_fk(self, q: NDArray[np.float64]) -> pinocchio.SE3:
        """Return the controlled tool point pose."""
        pose = self._ik.forward_kinematics(q)
        if self._tool_offset is not None:
            pose = pose * self._tool_offset
        return pose

    def on_buttons(self, msg: Buttons) -> bool:
        """Use the configured primary button as press-and-hold engagement."""
        is_left = self._config.hand == "left"
        primary = msg.left_primary if is_left else msg.right_primary
        trigger = msg.left_trigger_analog if is_left else msg.right_trigger_analog

        with self._lock:
            if self._estopped:
                return False
            if primary and not self._prev_primary:
                self._initial_ee_pose = None
                self._rot_deadband_ref = None
            elif not primary and self._prev_primary:
                self._active = False
                self._target_pose = None
                self._initial_ee_pose = None
                self._rot_deadband_ref = None
            self._prev_primary = primary

        if self._config.gripper_joint is not None:
            self.on_gripper_trigger(trigger)
        return True

    def on_teleop_buttons(self, msg: Buttons, t_now: float) -> bool:
        """Uniform stream handler for broadcast controller buttons."""
        return self.on_buttons(msg)

    def on_cartesian_command(self, pose: Pose | PoseStamped, t_now: float) -> bool:
        """Accept an engagement-relative pose delta unless E-STOP is latched."""
        with self._lock:
            if self._estopped:
                return False
            self._target_pose = pose
            self._last_update_time = t_now
            self._active = True
        return True

    def on_gripper_trigger(self, value: float, _t_now: float = 0.0) -> bool:
        """Map an analog trigger value onto the configured gripper range."""
        if self._config.gripper_joint is None or not np.isfinite(value):
            return False
        clamped = max(0.0, min(1.0, value))
        position = (
            self._config.gripper_open_pos
            + (self._config.gripper_closed_pos - self._config.gripper_open_pos) * clamped
        )
        with self._lock:
            if self._estopped:
                return False
            self._gripper_target = position
        return True

    def _on_timeout(self) -> None:
        """Discard the baseline while the parent holds the task lock."""
        self._initial_ee_pose = None
        self._prev_primary = False
        self._rot_deadband_ref = None

    def stop(self) -> None:
        """Stop output and discard engagement-relative state."""
        super().stop()
        with self._lock:
            self._active = False
            self._target_pose = None
            self._initial_ee_pose = None
            self._prev_primary = False
            self._rot_deadband_ref = None

    def clear(self) -> None:
        """Clear output and discard engagement-relative state."""
        super().clear()
        with self._lock:
            self._active = False
            self._target_pose = None
            self._initial_ee_pose = None
            self._prev_primary = False
            self._rot_deadband_ref = None


class TeleopIKTaskParams(BaseConfig):
    control_ik: TeleopControlIKConfig
    hand: Literal["left", "right"] | None = None
    timeout: float = 0.5
    max_joint_delta_deg: float = 5.0
    max_step_deg_per_tick: float | None = None
    max_target_offset_m: float | None = None
    max_target_rot_deg: float | None = None
    joint_limit_margin_deg: float = 0.0
    tool_offset_m: tuple[float, float, float] | None = None
    rotation_frame: Literal["world", "local"] = "world"
    rotation_deadband_deg: float = 0.0
    min_dt: FiniteFloat = 1e-4
    max_dt: FiniteFloat = 0.05
    gripper_joint: str | None = None
    gripper_open_pos: float = 0.0
    gripper_closed_pos: float = 0.0


def create_task(cfg: TaskConfig, hardware: object) -> TeleopIKTask:
    """Create a Pink-backed teleop task from declarative configuration."""
    params = TeleopIKTaskParams.model_validate(cfg.params)
    return TeleopIKTask(
        cfg.name,
        TeleopIKTaskConfig(
            joint_names=cfg.joint_names,
            control_ik=params.control_ik,
            priority=cfg.priority,
            timeout=params.timeout,
            max_joint_delta_deg=params.max_joint_delta_deg,
            max_step_deg_per_tick=params.max_step_deg_per_tick,
            max_target_offset_m=params.max_target_offset_m,
            max_target_rot_deg=params.max_target_rot_deg,
            joint_limit_margin_deg=params.joint_limit_margin_deg,
            tool_offset_m=params.tool_offset_m,
            rotation_frame=params.rotation_frame,
            rotation_deadband_deg=params.rotation_deadband_deg,
            min_dt=params.min_dt,
            max_dt=params.max_dt,
            hand=params.hand,
            gripper_joint=params.gripper_joint,
            gripper_open_pos=params.gripper_open_pos,
            gripper_closed_pos=params.gripper_closed_pos,
        ),
    )

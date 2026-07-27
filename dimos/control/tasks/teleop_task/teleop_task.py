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

"""Teleop cartesian control task with internal Pinocchio IK solver.

Accepts streaming cartesian delta poses from teleoperation and computes
inverse kinematics internally to output joint commands. Deltas are applied
relative to the EE pose captured at engage time.

Participates in joint-level arbitration.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import threading
import time
from typing import TYPE_CHECKING, Any, Literal

import numpy as np
import pinocchio

from dimos.control.task import (
    BaseControlTask,
    ControlMode,
    CoordinatorState,
    JointCommandOutput,
    ResourceClaim,
)
from dimos.manipulation.planning.kinematics.pinocchio_ik import (
    PinocchioIK,
    check_joint_delta,
    pose_to_se3,
)
from dimos.protocol.service.spec import BaseConfig
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from numpy.typing import NDArray

    from dimos.msgs.geometry_msgs.Pose import Pose
    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
    from dimos.teleop.quest.quest_types import Buttons

logger = setup_logger()


@dataclass
class TeleopIKTaskConfig:
    """Configuration for teleop IK task.

    Attributes:
        joint_names: List of joint names this task controls (must match model DOF)
        model_path: Path to URDF or MJCF file for IK solver
        ee_joint_id: End-effector joint ID in the kinematic chain
        priority: Priority for arbitration (higher wins)
        timeout: If no command received for this many seconds, go inactive (0 = never)
        max_joint_delta_deg: Maximum allowed joint change per tick (safety limit)
        hand: "left" or "right" — which controller's primary button to listen to
        gripper_joint: Optional joint name for the gripper (e.g. "arm/gripper").
        gripper_open_pos: Gripper position (adapter units) at trigger value 0.0 (no press).
        gripper_closed_pos: Gripper position (adapter units) at trigger value 1.0 (full press).
    """

    joint_names: list[str]
    model_path: str | Path
    ee_joint_id: int
    priority: int = 10
    timeout: float = 0.5
    max_joint_delta_deg: float = 5.0  # ~500°/s at 100Hz
    # Accepted solutions execute as bounded per-tick steps instead of all
    # at once; without this, a solution inside max_joint_delta_deg but
    # larger than one tick of safe motion wedges tracking from poses where
    # small cartesian targets need large joint motion.
    max_step_deg_per_tick: float | None = None
    # Chase window: clamp the target into this neighborhood of the current
    # EE, recentered every tick, so the solve stays local and stream gaps
    # cannot wedge it. Does not cap chase speed.
    max_target_offset_m: float | None = None
    max_target_rot_deg: float | None = None
    joint_limit_margin_deg: float = 0.0  # keep commands inside URDF limits
    orientation_weight: float = 1.0  # pink only: orientation vs position cost
    posture_weight: float | None = None  # pink only: null-space anchor
    # Fixed tool point in the ee_joint frame (e.g. a grasp center past a
    # gripper). Task-space work happens at this point; solve targets
    # convert back to the ee_joint frame, so solvers run unchanged.
    tool_offset_m: tuple[float, float, float] | None = None
    rotation_frame: Literal["world", "local"] = "world"
    # Soft angular deadband against the previous target so hand tremor
    # does not ripple the wrist; deliberate rotation follows shortened.
    rotation_deadband_deg: float = 0.0
    # pink fails construction when its dependencies are missing; there is
    # no silent fallback. urdf_joint_names (model-namespace, command
    # order) is required for pink and ignored by dls.
    solver: Literal["dls", "pink"] = "dls"
    urdf_joint_names: list[str] | None = None
    hand: Literal["left", "right"] | None = None
    gripper_joint: str | None = None
    gripper_open_pos: float = 0.0
    gripper_closed_pos: float = 0.0


class TeleopIKTask(BaseControlTask):
    """Teleop cartesian control task with internal Pinocchio IK solver.

    Accepts streaming cartesian delta poses via on_cartesian_command() and computes IK
    internally to output joint commands. Deltas are applied relative to the EE pose
    captured at engage time (first compute).

    Uses current joint state from CoordinatorState as IK warm-start for fast convergence.
    Outputs JointCommandOutput and participates in joint-level arbitration.

    Example:
        >>> from dimos.utils.data import get_data
        >>> piper_path = get_data("piper_description")
        >>> task = TeleopIKTask(
        ...     name="teleop_arm",
        ...     config=TeleopIKTaskConfig(
        ...         joint_names=["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
        ...         model_path=piper_path / "mujoco_model" / "piper_no_gripper_description.xml",
        ...         ee_joint_id=6,
        ...         priority=10,
        ...         timeout=0.5,
        ...         hand="right",
        ...     ),
        ... )
        >>> coordinator.add_task(task)
        >>> task.start()
        >>>
        >>> # From teleop callback:
        >>> task.on_cartesian_command(delta_pose, t_now=time.perf_counter())
    """

    def __init__(self, name: str, config: TeleopIKTaskConfig) -> None:
        """Initialize teleop IK task.

        Args:
            name: Unique task name
            config: Task configuration
        """
        if not config.joint_names:
            raise ValueError(f"TeleopIKTask '{name}' requires at least one joint")
        if not config.model_path:
            raise ValueError(f"TeleopIKTask '{name}' requires model_path for IK solver")
        if config.hand not in ("left", "right"):
            raise ValueError(f"TeleopIKTask '{name}' requires hand='left' or 'right'")

        self._name = name
        self._config = config
        self._joint_names = frozenset(config.joint_names)
        self._joint_names_list = list(config.joint_names)
        self._num_joints = len(config.joint_names)

        # Create IK solver from model. Requesting pink with its
        # dependencies missing raises (PinkIKDependencyError): a solver
        # the hardware was validated with must never silently downgrade.
        self._ik: Any
        if config.solver == "pink":
            from dimos.manipulation.planning.kinematics.pink_ik import PinkStepper

            if config.urdf_joint_names is None:
                raise ValueError(f"TeleopIKTask '{name}': solver 'pink' requires urdf_joint_names")
            self._ik = PinkStepper(
                str(config.model_path),
                config.ee_joint_id,
                config.urdf_joint_names,
                orientation_cost=config.orientation_weight,
                posture_cost=(config.posture_weight if config.posture_weight is not None else 1e-2),
            )
            logger.info(f"TeleopIKTask {name}: solver=pink backend=proxqp")
        else:
            self._ik = PinocchioIK.from_model_path(config.model_path, config.ee_joint_id)
            logger.info(f"TeleopIKTask {name}: solver=dls")

        # Validate DOF matches joint names
        if self._ik.nq != self._num_joints:
            logger.warning(
                f"TeleopIKTask {name}: model DOF ({self._ik.nq}) != "
                f"joint_names count ({self._num_joints})"
            )

        self._tool_offset: pinocchio.SE3 | None = None
        self._tool_offset_inv: pinocchio.SE3 | None = None
        if config.tool_offset_m is not None:
            self._tool_offset = pinocchio.SE3(np.eye(3), np.array(config.tool_offset_m))
            self._tool_offset_inv = self._tool_offset.inverse()

        # Thread-safe target state
        self._lock = threading.Lock()
        self._target_pose: Pose | PoseStamped | None = None
        self._last_update_time: float = 0.0
        self._active = False
        self._estopped = False

        # Initial EE pose for delta application
        self._initial_ee_pose: pinocchio.SE3 | None = None
        self._prev_primary: bool = False
        # Rotation deadband reference; reset each engage.
        self._rot_deadband_ref: NDArray[np.floating[Any]] | None = None

        self._gripper_target: float = config.gripper_open_pos

        # Telemetry (TELEM lines, 1 Hz while active)
        self._telem_last_emit = 0.0
        self._telem_computes = 0
        self._telem_rejects = 0
        self._telem_lag_m = 0.0
        self._telem_rot_lag_rad = 0.0
        self._telem_solve_s = 0.0
        self._telem_scale_min = 1.0

        logger.info(
            f"TeleopIKTask {name} initialized with model: {config.model_path}, "
            f"ee_joint_id={config.ee_joint_id}, joints={config.joint_names}"
        )

    def claim(self) -> ResourceClaim:
        """Declare resource requirements."""
        joints = self._joint_names
        if self._config.gripper_joint:
            joints = joints | frozenset([self._config.gripper_joint])
        return ResourceClaim(
            joints=joints,
            priority=self._config.priority,
            mode=ControlMode.SERVO_POSITION,
        )

    def is_active(self) -> bool:
        """Check if task should run this tick."""
        with self._lock:
            return not self._estopped and self._active and self._target_pose is not None

    def set_estop(self, estopped: bool) -> None:
        """Latch/clear E-STOP. On latch, disengage and drop the target so the
        task goes inert (is_active() False → compute() is skipped)."""
        with self._lock:
            self._estopped = estopped
            if estopped:
                self._active = False
                self._target_pose = None
                self._initial_ee_pose = None

    def compute(self, state: CoordinatorState) -> JointCommandOutput | None:
        """Compute IK and output joint positions.

        Args:
            state: Current coordinator state (contains joint positions for IK warm-start)

        Returns:
            JointCommandOutput with positions, or None if inactive/timed out/IK failed
        """
        with self._lock:
            if not self._active or self._target_pose is None:
                return None

            # Timeout safety: stop if teleop stream drops
            if self._config.timeout > 0:
                time_since_update = state.t_now - self._last_update_time
                if time_since_update > self._config.timeout:
                    logger.warning(
                        f"TeleopIKTask {self._name} timed out "
                        f"(no update for {time_since_update:.3f}s)"
                    )
                    self._target_pose = None
                    self._active = False
                    return None
            raw_pose = self._target_pose

        # Convert to SE3 right before use
        delta_se3 = pose_to_se3(raw_pose)
        # Capture initial EE pose if not set (first command after engage)
        with self._lock:
            need_capture = self._initial_ee_pose is None

        if need_capture:
            q_current = self._get_current_joints(state)
            if q_current is None:
                logger.debug(
                    f"TeleopIKTask {self._name}: cannot capture initial pose, joint state unavailable"
                )
                return None
            initial_pose = self._tool_fk(q_current)
            with self._lock:
                self._initial_ee_pose = initial_pose

        # Apply delta to initial pose: target = initial + delta
        with self._lock:
            if self._initial_ee_pose is None:
                return None
            target_rotation = self._target_rotation(delta_se3.rotation)
            target_rotation = self._apply_rotation_deadband(target_rotation)
            target_pose = pinocchio.SE3(
                target_rotation,
                self._initial_ee_pose.translation + delta_se3.translation,
            )

        # Get current joint positions for IK warm-start
        q_current = self._get_current_joints(state)
        if q_current is None:
            logger.debug(f"TeleopIKTask {self._name}: missing joint state for IK warm-start")
            return None

        ee_now = self._tool_fk(q_current)
        raw_lag = float(np.linalg.norm(target_pose.translation - ee_now.translation))
        raw_rot_lag = float(
            np.linalg.norm(pinocchio.log3(ee_now.rotation.T @ target_pose.rotation))
        )
        solve_t0 = time.perf_counter()

        windowed = (
            self._config.max_target_offset_m is not None
            or self._config.max_target_rot_deg is not None
        )
        scales = (1.0, 0.25) if windowed else (1.0,)
        margin = np.deg2rad(self._config.joint_limit_margin_deg)
        q_low = self._ik.model.lowerPositionLimit + margin
        q_high = self._ik.model.upperPositionLimit - margin
        q_solution = None
        q_candidate = q_current
        scale = 1.0
        for scale in scales:
            candidate_target = target_pose
            if windowed:
                candidate_target = self._windowed_target(q_current, target_pose, scale)
            solve_target = candidate_target
            if self._tool_offset_inv is not None:
                solve_target = candidate_target * self._tool_offset_inv
            q_candidate, converged, final_error = self._ik.solve(solve_target, q_current)
            if not converged:
                logger.debug(
                    f"TeleopIKTask {self._name}: IK did not converge "
                    f"(error={final_error:.4f}), using partial solution"
                )
            # Clamp into the joint limits so every command is achievable;
            # out-of-range commands make firmware clamp and tracking stall.
            q_candidate = np.clip(q_candidate, q_low, q_high)
            if check_joint_delta(q_candidate, q_current, self._config.max_joint_delta_deg):
                q_solution = q_candidate
                break

        self._telem_computes += 1
        self._telem_lag_m = max(self._telem_lag_m, raw_lag)
        self._telem_rot_lag_rad = max(self._telem_rot_lag_rad, raw_rot_lag)
        self._telem_solve_s = max(self._telem_solve_s, time.perf_counter() - solve_t0)
        if q_solution is not None:
            self._telem_scale_min = min(self._telem_scale_min, scale)
        if state.t_now - self._telem_last_emit >= 1.0:
            logger.info(
                "TELEM ik %s: solver=%s computes_hz=%d rejects=%d hand_lag_cm=%.1f "
                "rot_lag_deg=%.1f solve_ms_max=%.1f window_scale_min=%.2f",
                self._name,
                self._config.solver,
                self._telem_computes,
                self._telem_rejects,
                self._telem_lag_m * 100.0,
                np.rad2deg(self._telem_rot_lag_rad),
                self._telem_solve_s * 1000.0,
                self._telem_scale_min,
            )
            self._telem_last_emit = state.t_now
            self._telem_computes = 0
            self._telem_rejects = 0
            self._telem_lag_m = 0.0
            self._telem_rot_lag_rad = 0.0
            self._telem_solve_s = 0.0
            self._telem_scale_min = 1.0

        if q_solution is None:
            self._telem_rejects += 1
            worst = float(np.max(np.abs(q_candidate - q_current)))
            logger.warning(
                f"TeleopIKTask {self._name}: joint delta {np.rad2deg(worst):.1f}° exceeds "
                f"{self._config.max_joint_delta_deg}° at the smallest window, rejecting"
            )
            return None

        # Bounded catch-up: execute at most one tick's worth of motion
        # toward the accepted solution rather than the whole displacement.
        if self._config.max_step_deg_per_tick is not None:
            step = np.deg2rad(self._config.max_step_deg_per_tick)
            q_solution = q_current + np.clip(q_solution - q_current, -step, step)

        joint_names = list(self._joint_names_list)
        positions = q_solution.flatten().tolist()

        # Append gripper joint if configured — routed to ConnectedHardware by tick loop
        if self._config.gripper_joint:
            with self._lock:
                gripper_pos = self._gripper_target
            joint_names.append(self._config.gripper_joint)
            positions.append(gripper_pos)

        return JointCommandOutput(
            joint_names=joint_names,
            positions=positions,
            mode=ControlMode.SERVO_POSITION,
        )

    def _target_rotation(
        self, command_rotation: NDArray[np.floating[Any]]
    ) -> NDArray[np.floating[Any]]:
        """Compose the rotation delta onto the engage anchor. Caller holds the lock."""
        assert self._initial_ee_pose is not None
        if self._config.rotation_frame == "local":
            # Delta arrives in the hand's own frame; compose in the EE
            # frame so a hand twist maps to the same gripper-local twist.
            return np.asarray(self._initial_ee_pose.rotation @ command_rotation)
        return np.asarray(command_rotation @ self._initial_ee_pose.rotation)

    def _apply_rotation_deadband(
        self, target_rotation: NDArray[np.floating[Any]]
    ) -> NDArray[np.floating[Any]]:
        """Soft angular deadband against the previous target. Caller holds the lock."""
        deadband = np.deg2rad(self._config.rotation_deadband_deg)
        if deadband <= 0.0:
            return target_rotation
        ref = self._rot_deadband_ref
        if ref is None:
            self._rot_deadband_ref = target_rotation
            return target_rotation
        w = pinocchio.log3(ref.T @ target_rotation)
        angle = float(np.linalg.norm(w))
        if angle <= deadband:
            return ref
        out = np.asarray(ref @ pinocchio.exp3(w * ((angle - deadband) / angle)))
        self._rot_deadband_ref = out
        return out

    def _windowed_target(
        self,
        q_current: NDArray[np.floating[Any]],
        target_pose: pinocchio.SE3,
        scale: float,
    ) -> pinocchio.SE3:
        """Clamp the target into the chase window around the current EE pose."""
        ee_now = self._tool_fk(q_current)
        position = target_pose.translation
        rotation = target_pose.rotation
        if self._config.max_target_offset_m is not None:
            offset = position - ee_now.translation
            dist = float(np.linalg.norm(offset))
            max_dist = self._config.max_target_offset_m * scale
            if dist > max_dist:
                position = ee_now.translation + offset * (max_dist / dist)
        if self._config.max_target_rot_deg is not None:
            w = pinocchio.log3(ee_now.rotation.T @ rotation)
            angle = float(np.linalg.norm(w))
            max_angle = np.deg2rad(self._config.max_target_rot_deg) * scale
            if angle > max_angle:
                rotation = ee_now.rotation @ pinocchio.exp3(w * (max_angle / angle))
        return pinocchio.SE3(rotation, position)

    def _tool_fk(self, q: NDArray[np.floating[Any]]) -> pinocchio.SE3:
        """Pose of the controlled point: the tool point if configured, else the ee_joint."""
        pose = self._ik.forward_kinematics(q)
        if self._tool_offset is not None:
            pose = pose * self._tool_offset
        return pose

    def _get_current_joints(self, state: CoordinatorState) -> NDArray[np.floating[Any]] | None:
        """Get current joint positions from coordinator state."""
        positions = []
        for joint_name in self._joint_names_list:
            pos = state.joints.get_position(joint_name)
            if pos is None:
                return None
            positions.append(pos)
        return np.array(positions)

    def on_preempted(self, by_task: str, joints: frozenset[str]) -> None:
        """Handle preemption by higher-priority task.

        Args:
            by_task: Name of preempting task
            joints: Joints that were preempted
        """
        if joints & self._joint_names:
            logger.warning(f"TeleopIKTask {self._name} preempted by {by_task} on joints {joints}")

    def on_buttons(self, msg: Buttons) -> bool:
        """Press-and-hold engage: hold primary button to track, release to stop."""
        is_left = self._config.hand == "left"
        primary = msg.left_primary if is_left else msg.right_primary

        if primary and not self._prev_primary:
            logger.info(f"TeleopIKTask {self._name}: engage")
            with self._lock:
                self._initial_ee_pose = None
                self._rot_deadband_ref = None
        elif not primary and self._prev_primary:
            logger.info(f"TeleopIKTask {self._name}: disengage")
            with self._lock:
                self._target_pose = None
                self._initial_ee_pose = None
                self._rot_deadband_ref = None
        self._prev_primary = primary

        if self._config.gripper_joint:
            trigger = msg.left_trigger_analog if is_left else msg.right_trigger_analog
            self.on_gripper_trigger(trigger)

        return True

    def on_teleop_buttons(self, msg: Buttons, t_now: float) -> bool:
        """Uniform stream handler; ``on_buttons`` predates the (msg, t_now) contract."""
        return self.on_buttons(msg)

    def on_cartesian_command(self, pose: Pose | PoseStamped, t_now: float) -> bool:
        """Handle incoming cartesian command (delta pose from teleop)"""
        with self._lock:
            self._target_pose = pose  # Store raw, convert to SE3 in compute()
            self._last_update_time = t_now
            self._active = True

        return True

    def on_gripper_trigger(self, value: float, _t_now: float = 0.0) -> bool:
        """Map analog trigger (0-1) to gripper position"""
        if not self._config.gripper_joint:
            return False

        clamped = max(0.0, min(1.0, value))
        pos = (
            self._config.gripper_open_pos
            + (self._config.gripper_closed_pos - self._config.gripper_open_pos) * clamped
        )

        with self._lock:
            self._gripper_target = pos

        return True

    def start(self) -> None:
        """Activate the task (start accepting and outputting commands)."""
        with self._lock:
            self._active = True
        logger.info(f"TeleopIKTask {self._name} started")

    def stop(self) -> None:
        """Deactivate the task (stop outputting commands)."""
        with self._lock:
            self._active = False
        logger.info(f"TeleopIKTask {self._name} stopped")


class TeleopIKTaskParams(BaseConfig):
    model_path: str | Path
    ee_joint_id: int = 6
    timeout: float = TeleopIKTaskConfig.timeout
    hand: Literal["left", "right"] | None = None
    gripper_joint: str | None = None
    gripper_open_pos: float = 0.0
    gripper_closed_pos: float = 0.0
    max_joint_delta_deg: float = TeleopIKTaskConfig.max_joint_delta_deg
    max_step_deg_per_tick: float | None = None
    max_target_offset_m: float | None = None
    max_target_rot_deg: float | None = None
    joint_limit_margin_deg: float = 0.0
    orientation_weight: float = 1.0
    posture_weight: float | None = None
    tool_offset_m: tuple[float, float, float] | None = None
    rotation_frame: Literal["world", "local"] = "world"
    rotation_deadband_deg: float = 0.0
    solver: Literal["dls", "pink"] = "dls"
    urdf_joint_names: list[str] | None = None


def create_task(cfg: Any, hardware: Any) -> TeleopIKTask:
    params = TeleopIKTaskParams.model_validate(cfg.params)
    return TeleopIKTask(
        cfg.name,
        TeleopIKTaskConfig(
            joint_names=cfg.joint_names,
            model_path=params.model_path,
            ee_joint_id=params.ee_joint_id,
            priority=cfg.priority,
            timeout=params.timeout,
            hand=params.hand,
            gripper_joint=params.gripper_joint,
            gripper_open_pos=params.gripper_open_pos,
            gripper_closed_pos=params.gripper_closed_pos,
            max_joint_delta_deg=params.max_joint_delta_deg,
            max_step_deg_per_tick=params.max_step_deg_per_tick,
            max_target_offset_m=params.max_target_offset_m,
            max_target_rot_deg=params.max_target_rot_deg,
            joint_limit_margin_deg=params.joint_limit_margin_deg,
            orientation_weight=params.orientation_weight,
            posture_weight=params.posture_weight,
            tool_offset_m=params.tool_offset_m,
            rotation_frame=params.rotation_frame,
            rotation_deadband_deg=params.rotation_deadband_deg,
            solver=params.solver,
            urdf_joint_names=params.urdf_joint_names,
        ),
    )

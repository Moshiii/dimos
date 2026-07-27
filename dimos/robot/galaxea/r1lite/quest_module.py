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

"""Quest teleoperation module for the Galaxea R1 Lite.

One headset drives arms, grippers, and chassis. Arms use the inherited
per-hand press-and-hold engage with task-name routing to the per-arm
TeleopIK tasks. Grippers map each trigger to the vendor 0-100 scale,
streamed every control tick while that hand is engaged, because the
gripper controller acts only on a continuous stream. Chassis: left stick
drives forward and strafe, right stick X yaws, any stick press commands
zero. Twist and gripper commands publish from the 50 Hz control loop so
the chassis dead-man stays fed; a controller with a stale Joy stream
contributes zero velocity.

Motion still requires the connection to be ARMED through the preflight
tool; this module publishes commands regardless, and the connection drops
them while disarmed.
"""

from __future__ import annotations

import math
import time
from typing import Any

from dimos.core.core import rpc
from dimos.core.stream import Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.Joy import Joy
from dimos.robot.galaxea.r1lite import config as cfg
from dimos.teleop.quest.quest_extensions import VideoArmTeleopConfig, VideoArmTeleopModule
from dimos.teleop.quest.quest_types import Hand, QuestControllerState
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class R1LiteQuestTeleopConfig(VideoArmTeleopConfig):
    """Configuration for R1LiteQuestTeleopModule."""

    linear_speed: float = 0.2  # m/s at full stick deflection
    angular_speed: float = 0.4  # rad/s at full stick deflection
    deadzone: float = 0.1
    joy_timeout: float = 0.5  # seconds without Joy before a controller stops driving
    motion_gain: float = 1.0  # scales hand position deltas
    # Soft radial deadband on the raw hand delta, applied before gain:
    # motion under this radius is ignored and larger motion is shortened
    # by it, so there is no jump at the threshold. Absorbs hand tremor and
    # the arc the controller origin sweeps during a wrist roll.
    position_deadband_m: float = 0.0
    # Publish the orientation delta in the hand's own frame; pairs with
    # rotation_frame "local" on the teleop tasks.
    local_rotation: bool = False
    gripper_open: float = cfg.GRIPPER_OPEN
    gripper_closed: float = cfg.GRIPPER_CLOSED


class R1LiteQuestTeleopModule(VideoArmTeleopModule):
    """Quest teleop for the R1 Lite: arms, grippers and chassis from one headset."""

    config: R1LiteQuestTeleopConfig

    cmd_vel: Out[Twist]
    gripper_left_command: Out[JointState]
    gripper_right_command: Out[JointState]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._joy_rx_ts: dict[Hand, float] = {Hand.LEFT: 0.0, Hand.RIGHT: 0.0}
        # Telemetry (TELEM lines, 1 Hz from the control loop)
        self._telem_joy_count: dict[Hand, int] = {Hand.LEFT: 0, Hand.RIGHT: 0}
        self._telem_joy_gap_max: dict[Hand, float] = {Hand.LEFT: 0.0, Hand.RIGHT: 0.0}
        self._telem_pose_count = 0
        self._telem_loop_gap_max = 0.0
        self._telem_last_tick = 0.0
        self._telem_last_emit = 0.0

    @rpc
    def start(self) -> None:
        super().start()

    @rpc
    def stop(self) -> None:
        # One zero Twist so the base halts if teleop dies mid-motion.
        try:
            self.cmd_vel.publish(Twist.zero())
        except Exception:
            logger.exception("Failed to publish stop Twist")
        super().stop()

    def _get_output_pose(self, hand: Hand) -> PoseStamped | None:
        current = self._current_poses.get(hand)
        initial = self._initial_poses.get(hand)
        if current is None or initial is None:
            return None
        dx = current.position.x - initial.position.x
        dy = current.position.y - initial.position.y
        dz = current.position.z - initial.position.z
        deadband = self.config.position_deadband_m
        if deadband > 0.0:
            norm = math.sqrt(dx * dx + dy * dy + dz * dz)
            if norm <= deadband:
                dx = dy = dz = 0.0
            else:
                shrink = (norm - deadband) / norm
                dx, dy, dz = dx * shrink, dy * shrink, dz * shrink
        gain = self.config.motion_gain
        dx, dy, dz = dx * gain, dy * gain, dz * gain
        if self.config.local_rotation:
            orientation = initial.orientation.inverse() * current.orientation
        else:
            orientation = current.orientation * initial.orientation.inverse()
        return PoseStamped(
            position=Vector3(dx, dy, dz),
            orientation=orientation,
            ts=current.ts,
            frame_id=current.frame_id,
        )

    def _on_joy_bytes(self, data: bytes) -> None:
        msg = Joy.lcm_decode(data)
        hand = Hand.LEFT if msg.frame_id == "left" else Hand.RIGHT
        try:
            controller = QuestControllerState.from_joy(msg, is_left=(hand == Hand.LEFT))
        except ValueError:
            logger.warning(
                f"Malformed Joy for {hand.name}: axes={len(msg.axes or [])}, "
                f"buttons={len(msg.buttons or [])}"
            )
            return
        with self._lock:
            self._controllers[hand] = controller
            now = time.monotonic()
            prev = self._joy_rx_ts[hand]
            if prev > 0.0:
                self._telem_joy_gap_max[hand] = max(self._telem_joy_gap_max[hand], now - prev)
            self._telem_joy_count[hand] += 1
            # Local receive time, not msg.ts: headset and robot clocks are
            # not synchronized.
            self._joy_rx_ts[hand] = now

    def _on_pose_bytes(self, data: bytes) -> None:
        super()._on_pose_bytes(data)
        with self._lock:
            self._telem_pose_count += 1

    def _deadzone(self, v: float) -> float:
        return 0.0 if abs(v) < self.config.deadzone else v

    def _fresh(self, hand: Hand, now: float) -> bool:
        return (now - self._joy_rx_ts[hand]) < self.config.joy_timeout

    def _chassis_twist(
        self,
        left: QuestControllerState | None,
        right: QuestControllerState | None,
        now: float,
    ) -> Twist:
        twist = Twist()
        twist.linear = Vector3(0.0, 0.0, 0.0)
        twist.angular = Vector3(0.0, 0.0, 0.0)
        left_live = left if (left is not None and self._fresh(Hand.LEFT, now)) else None
        right_live = right if (right is not None and self._fresh(Hand.RIGHT, now)) else None
        if (left_live is not None and left_live.thumbstick_press) or (
            right_live is not None and right_live.thumbstick_press
        ):
            return twist
        if left_live is not None:
            twist.linear.x = -self._deadzone(left_live.thumbstick.y) * self.config.linear_speed
            twist.linear.y = -self._deadzone(left_live.thumbstick.x) * self.config.linear_speed
        if right_live is not None:
            twist.angular.z = -self._deadzone(right_live.thumbstick.x) * self.config.angular_speed
        return twist

    def _gripper_command(self, trigger: float) -> JointState:
        clamped = max(0.0, min(1.0, trigger))
        span = self.config.gripper_closed - self.config.gripper_open
        return JointState(position=[self.config.gripper_open + span * clamped])

    def _publish_button_state(
        self,
        left: QuestControllerState | None,
        right: QuestControllerState | None,
    ) -> None:
        super()._publish_button_state(left, right)
        now = time.monotonic()
        if self._telem_last_tick > 0.0:
            self._telem_loop_gap_max = max(self._telem_loop_gap_max, now - self._telem_last_tick)
        self._telem_last_tick = now
        if now - self._telem_last_emit >= 1.0:
            logger.info(
                "TELEM quest: joyL_hz=%d joyR_hz=%d pose_hz=%d "
                "joy_gap_ms L=%.0f R=%.0f loop_gap_ms=%.0f engaged L=%s R=%s",
                self._telem_joy_count[Hand.LEFT],
                self._telem_joy_count[Hand.RIGHT],
                self._telem_pose_count,
                self._telem_joy_gap_max[Hand.LEFT] * 1000.0,
                self._telem_joy_gap_max[Hand.RIGHT] * 1000.0,
                self._telem_loop_gap_max * 1000.0,
                self._is_engaged[Hand.LEFT],
                self._is_engaged[Hand.RIGHT],
            )
            self._telem_last_emit = now
            self._telem_joy_count = {Hand.LEFT: 0, Hand.RIGHT: 0}
            self._telem_joy_gap_max = {Hand.LEFT: 0.0, Hand.RIGHT: 0.0}
            self._telem_pose_count = 0
            self._telem_loop_gap_max = 0.0
        self.cmd_vel.publish(self._chassis_twist(left, right, now))
        if left is not None and self._is_engaged[Hand.LEFT] and self._fresh(Hand.LEFT, now):
            self.gripper_left_command.publish(self._gripper_command(left.trigger))
        if right is not None and self._is_engaged[Hand.RIGHT] and self._fresh(Hand.RIGHT, now):
            self.gripper_right_command.publish(self._gripper_command(right.trigger))

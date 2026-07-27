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

"""Deterministic tests for the R1 Lite quest teleop module and blueprints.

No headset, no ROS, no threads: the module is constructed with rpc
disabled and the event loop factory stubbed, and Out streams are replaced
per test.
"""

from __future__ import annotations

import base64
import json
from pathlib import Path
import sys
import time
import types
from typing import Any
import xml.etree.ElementTree as ET

import numpy as np
import pinocchio
import pytest

from dimos.control.coordinator import ControlCoordinator
import dimos.core.module as module_mod
from dimos.manipulation.planning.kinematics.pinocchio_ik import PinocchioIK
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Joy import Joy
from dimos.protocol.rpc.spec import RPCSpec
from dimos.robot.galaxea.r1lite import config as cfg
from dimos.robot.galaxea.r1lite.blueprints.basic.r1lite_quest_teleop import (
    r1lite_quest_teleop,
    r1lite_quest_teleop_sim,
)
from dimos.robot.galaxea.r1lite.quest_module import R1LiteQuestTeleopModule
from dimos.teleop.quest.quest_types import Hand, QuestControllerState, ThumbstickState

_FIXTURE = Path(__file__).parent / "testdata" / "quest_replay_fixture.jsonl"


class _NoRpc(RPCSpec):
    """Module.__init__ treats a ValueError from the rpc factory as rpc disabled."""

    def __init__(self, **kw: Any) -> None:
        raise ValueError("rpc disabled for unit tests")


@pytest.fixture(autouse=True)
def _no_background_threads(monkeypatch: Any) -> None:
    monkeypatch.setattr(module_mod, "get_loop", lambda: (types.SimpleNamespace(), None))


class _FakeOut:
    def __init__(self) -> None:
        self.msgs: list[Any] = []

    def publish(self, msg: Any) -> None:
        self.msgs.append(msg)


def _module(**config_kwargs: Any) -> R1LiteQuestTeleopModule:
    m = R1LiteQuestTeleopModule(rpc_transport=_NoRpc, **config_kwargs)
    for stream in (
        "cmd_vel",
        "gripper_left_command",
        "gripper_right_command",
        "teleop_buttons",
        "left_controller_output",
        "right_controller_output",
    ):
        setattr(m, stream, _FakeOut())
    return m


def _controller(
    *,
    is_left: bool = True,
    stick_x: float = 0.0,
    stick_y: float = 0.0,
    stick_press: bool = False,
    trigger: float = 0.0,
    primary: bool = False,
) -> QuestControllerState:
    return QuestControllerState(
        is_left=is_left,
        trigger=trigger,
        primary=primary,
        thumbstick_press=stick_press,
        thumbstick=ThumbstickState(x=stick_x, y=stick_y),
    )


def _mark_fresh(m: R1LiteQuestTeleopModule, *hands: Hand) -> float:
    now = time.monotonic()
    for hand in hands:
        m._joy_rx_ts[hand] = now
    return now


def _coordinator_tasks(blueprint: Any) -> list[Any]:
    kwargs = next(atom.kwargs for atom in blueprint.blueprints if atom.module is ControlCoordinator)
    return list(kwargs["tasks"])


# Chassis and grippers


def test_left_stick_maps_to_linear_velocity() -> None:
    m = _module()
    now = _mark_fresh(m, Hand.LEFT)
    twist = m._chassis_twist(_controller(stick_y=-1.0, stick_x=0.5), None, now)
    assert twist.linear.x == pytest.approx(0.2)
    assert twist.linear.y == pytest.approx(-0.1)
    assert twist.angular.z == 0.0


def test_stale_controller_contributes_zero() -> None:
    m = _module()
    now = _mark_fresh(m, Hand.RIGHT)
    m._joy_rx_ts[Hand.LEFT] = now - 10.0
    twist = m._chassis_twist(
        _controller(stick_y=-1.0),
        _controller(is_left=False, stick_x=1.0),
        now,
    )
    assert twist.linear.x == 0.0
    assert twist.angular.z == pytest.approx(-0.4)


def test_any_thumbstick_press_zeros_twist() -> None:
    m = _module()
    now = _mark_fresh(m, Hand.LEFT, Hand.RIGHT)
    twist = m._chassis_twist(
        _controller(stick_y=-1.0),
        _controller(is_left=False, stick_x=1.0, stick_press=True),
        now,
    )
    assert (twist.linear.x, twist.linear.y, twist.angular.z) == (0.0, 0.0, 0.0)


def test_gripper_command_maps_trigger_to_percent() -> None:
    m = _module()
    assert m._gripper_command(0.0).position[0] == pytest.approx(100.0)
    assert m._gripper_command(1.0).position[0] == pytest.approx(0.0)


def test_gripper_streams_only_while_engaged_and_fresh() -> None:
    m = _module()
    left = _controller(trigger=1.0, primary=True)
    _mark_fresh(m, Hand.LEFT)
    m._publish_button_state(left, None)
    assert m.gripper_left_command.msgs == []
    m._is_engaged[Hand.LEFT] = True
    m._publish_button_state(left, None)
    assert len(m.gripper_left_command.msgs) == 1
    m._joy_rx_ts[Hand.LEFT] = time.monotonic() - 10.0
    m._publish_button_state(left, None)
    assert len(m.gripper_left_command.msgs) == 1


def test_publish_tick_always_streams_twist() -> None:
    m = _module()
    m._publish_button_state(None, None)
    assert len(m.cmd_vel.msgs) == 1
    assert len(m.teleop_buttons.msgs) == 1


# Hand-delta mapping


def test_position_deadband_zeroes_small_deltas() -> None:
    m = _module(position_deadband_m=0.02)
    m._is_engaged[Hand.LEFT] = True
    m._initial_poses[Hand.LEFT] = PoseStamped()
    m._current_poses[Hand.LEFT] = PoseStamped(position=[0.01, 0.01, 0.0])
    out = m._get_output_pose(Hand.LEFT)
    assert (out.position.x, out.position.y, out.position.z) == (0.0, 0.0, 0.0)


def test_position_deadband_is_soft_and_applies_before_gain() -> None:
    m = _module(position_deadband_m=0.02, motion_gain=2.0)
    m._is_engaged[Hand.LEFT] = True
    m._initial_poses[Hand.LEFT] = PoseStamped()
    m._current_poses[Hand.LEFT] = PoseStamped(position=[0.05, 0.0, 0.0])
    out = m._get_output_pose(Hand.LEFT)
    assert out.position.x == pytest.approx((0.05 - 0.02) * 2.0)
    assert out.position.y == 0.0


def test_motion_gain_scales_position_delta_only() -> None:
    m = _module(motion_gain=1.3)
    m._is_engaged[Hand.LEFT] = True
    m._initial_poses[Hand.LEFT] = PoseStamped()
    m._current_poses[Hand.LEFT] = PoseStamped(position=[0.10, 0.0, 0.20])
    out = m._get_output_pose(Hand.LEFT)
    assert out.position.x == pytest.approx(0.13)
    assert out.position.z == pytest.approx(0.26)


def test_local_rotation_uses_hand_frame_delta() -> None:
    m = _module(local_rotation=True)
    m._is_engaged[Hand.LEFT] = True
    initial = PoseStamped(orientation=Quaternion.from_euler(Vector3(0.0, 0.0, 1.0)))
    current = PoseStamped(
        orientation=initial.orientation * Quaternion.from_euler(Vector3(0.5, 0.0, 0.0))
    )
    m._initial_poses[Hand.LEFT] = initial
    m._current_poses[Hand.LEFT] = current
    out = m._get_output_pose(Hand.LEFT)
    expected = initial.orientation.inverse() * current.orientation
    got = out.orientation
    dot = sum(
        a * b
        for a, b in zip(
            (got.x, got.y, got.z, got.w),
            (expected.x, expected.y, expected.z, expected.w),
            strict=False,
        )
    )
    assert abs(abs(dot) - 1.0) < 1e-6


# Blueprint contracts


def test_teleop_tasks_use_arm_slices_and_pink() -> None:
    for blueprint in (r1lite_quest_teleop, r1lite_quest_teleop_sim):
        tasks = {t.name: t for t in _coordinator_tasks(blueprint)}
        left = tasks["teleop_left_arm"]
        right = tasks["teleop_right_arm"]
        assert left.type == right.type == "teleop_ik"
        assert left.priority == right.priority == 20
        assert left.joint_names == cfg.LEFT_ARM_JOINTS
        assert right.joint_names == cfg.RIGHT_ARM_JOINTS
        for task, urdf_names in (
            (left, cfg.LEFT_ARM_URDF_JOINTS),
            (right, cfg.RIGHT_ARM_URDF_JOINTS),
        ):
            assert task.params["solver"] == "pink"
            assert task.params["urdf_joint_names"] == urdf_names
            assert task.params["rotation_frame"] == "local"
            assert task.params["rotation_deadband_deg"] == 4.0
            assert task.params["orientation_weight"] == 1.0
            assert task.params["posture_weight"] == 0.05
            assert task.params["tool_offset_m"] == (0.17, 0.0, 0.0)
            assert task.params["max_joint_delta_deg"] == 45.0
            assert task.params["max_step_deg_per_tick"] == 1.5
            assert task.params["max_target_offset_m"] == 0.08
            assert task.params["max_target_rot_deg"] == 20.0


def test_module_rotation_pairing_and_no_default_recording() -> None:
    for blueprint in (r1lite_quest_teleop, r1lite_quest_teleop_sim):
        kwargs = next(
            atom.kwargs for atom in blueprint.blueprints if atom.module is R1LiteQuestTeleopModule
        )
        assert kwargs["local_rotation"] is True
        # Recording is opt-in per session, never a blueprint default.
        assert "record_path" not in kwargs


def test_hardware_blueprint_teleop_overrides() -> None:
    from dimos.robot.galaxea.r1lite.connection import R1LiteConnection

    kwargs = next(
        atom.kwargs for atom in r1lite_quest_teleop.blueprints if atom.module is R1LiteConnection
    )
    assert kwargs["tracking_speed"] == 1.25
    assert kwargs["enable_cameras"] is False


# Shipped arm models


@pytest.mark.parametrize("model", [cfg.R1LITE_LEFT_ARM_MODEL, cfg.R1LITE_RIGHT_ARM_MODEL])
def test_arm_model_is_the_a1x_chain(model: Path) -> None:
    # Pin the vendor-derived chain: 6 revolute joints, the A1X axis
    # pattern, the 300 mm forearm, and the vendor limit convention.
    root = ET.parse(model).getroot()
    joints = [j for j in root.findall("joint") if j.get("type") == "revolute"]
    assert len(joints) == 6
    axes = [j.find("axis").get("xyz") for j in joints]
    assert axes == ["0 0 1", "0 1 0", "0 1 0", "0 1 0", "0 0 1", "1 0 0"]
    j3_x = float(joints[2].find("origin").get("xyz").split()[0])
    assert j3_x == pytest.approx(-0.3)
    limit1 = joints[0].find("limit")
    assert float(limit1.get("upper")) == pytest.approx(2.8623467075)


@pytest.mark.parametrize("model", [cfg.R1LITE_LEFT_ARM_MODEL, cfg.R1LITE_RIGHT_ARM_MODEL])
def test_arm_model_header_names_provenance(model: Path) -> None:
    text = model.read_text()
    assert "userguide-galaxea/URDF" in text
    assert "2e5d31e1784481a34d178006c0d0e18e0a84a82a" in text


# Offline chase regression (the wedge class)


def test_teleop_chases_through_folded_home_and_teleports() -> None:
    # From the folded home, small cartesian targets need large joint
    # motion, and pose-stream gaps teleport the target. The chase window
    # recentered on the EE, bounded steps, and the 45 degree backstop must
    # reach a far target with zero rejections.
    ik = PinocchioIK.from_model_path(cfg.R1LITE_LEFT_ARM_MODEL, ee_joint_id=6)
    q0 = np.zeros(6)
    ee0 = ik.forward_kinematics(q0)
    target = pinocchio.SE3(ee0.rotation, ee0.translation + np.array([0.25, 0.05, 0.15]))

    hard = np.deg2rad(45.0)
    step = np.deg2rad(1.5)
    win_t, win_r = 0.08, np.deg2rad(20.0)

    def windowed(q: Any, scale: float) -> pinocchio.SE3:
        ee = ik.forward_kinematics(q)
        off = target.translation - ee.translation
        dist = np.linalg.norm(off)
        max_t = win_t * scale
        pos = target.translation if dist <= max_t else ee.translation + off * (max_t / dist)
        w = pinocchio.log3(ee.rotation.T @ target.rotation)
        angle = np.linalg.norm(w)
        max_r = win_r * scale
        rot = (
            target.rotation if angle <= max_r else ee.rotation @ pinocchio.exp3(w * (max_r / angle))
        )
        return pinocchio.SE3(rot, pos)

    q = q0
    ticks = 0
    while np.linalg.norm(ik.forward_kinematics(q).translation - target.translation) >= 0.005:
        ticks += 1
        assert ticks <= 600, "teleported target not reached in 600 ticks"
        # Production behavior: full window first, quartered backoff if the
        # solution trips the branch-flip gate. Zero final rejections.
        accepted = None
        for scale in (1.0, 0.25):
            q_sol, _, _ = ik.solve(windowed(q, scale), q)
            if np.max(np.abs(q_sol - q)) < hard:
                accepted = q_sol
                break
        assert accepted is not None, "rejection at the smallest window"
        q = q + np.clip(accepted - q, -step, step)


# Recorder and replay fixture


def test_recorder_writes_replayable_frames(tmp_path: Path) -> None:
    record = tmp_path / "session.jsonl"
    m = _module(record_path=str(record))
    frames = [b"\x01\x02fingerprint-a", b"\x03\x04fingerprint-b"]
    for frame in frames:
        m._record_frame(frame)
    m._close_recorder()
    entries = [json.loads(line) for line in record.read_text().splitlines()]
    assert [base64.b64decode(e["data"]) for e in entries] == frames
    assert entries[1]["t"] >= entries[0]["t"]


def test_recorder_off_by_default() -> None:
    m = _module()
    m._record_frame(b"frame")
    m._close_recorder()
    assert m._record_file is None
    assert m._record_count == 0


def test_replay_fixture_is_sanitized_and_loadable() -> None:
    scripts = str(Path(__file__).resolve().parents[4] / "scripts" / "r1lite_test")
    sys.path.insert(0, scripts)
    try:
        from replay_quest_stream import load_frames
    finally:
        sys.path.remove(scripts)
    frames = load_frames(_FIXTURE, 0.0, float("inf"))
    assert len(frames) == 90
    assert frames[0][0] == 0.0
    engage_count = 0
    prev = 0
    for _, data in frames:
        msg: Any
        try:
            msg = Joy.lcm_decode(data)
        except Exception:
            msg = PoseStamped.lcm_decode(data)
            continue
        primary = msg.buttons[4] if len(msg.buttons) > 4 else 0
        if primary and not prev:
            engage_count += 1
        prev = primary
    assert engage_count == 1

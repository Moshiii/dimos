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

"""Deterministic unit tests for R1LiteConnection.

Run without ROS or hardware: RawROS is faked and the ROS message modules
the handlers import lazily are injected into sys.modules. The publish
loop thread never starts; tests drive single ticks directly.
"""

from __future__ import annotations

import ast
from pathlib import Path
import sys
import threading
import time
import types
from typing import Any

import pytest

import dimos.core.module as module_mod
from dimos.hardware.whole_body.spec import VEL_STOP
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.std_msgs.String import String
from dimos.protocol.rpc.spec import RPCSpec
from dimos.robot.galaxea.r1lite import config as cfg, connection as conn_mod
from dimos.robot.galaxea.r1lite.connection import ConnectionState, R1LiteConnection

_CONN_SRC = Path(conn_mod.__file__)


class _Msg:
    def __init__(self, **kw: Any) -> None:
        self.__dict__.update(kw)


class _FakeRos:
    def __init__(self, stamp_available: bool = True) -> None:
        self.published: list[tuple[Any, Any]] = []
        self._stamp_available = stamp_available

    def now_stamp(self) -> Any:
        if not self._stamp_available:
            return None
        return _Msg(sec=0, nanosec=0)

    def publish(self, topic: Any, message: Any) -> None:
        self.published.append((topic, message))

    def stop(self) -> None:
        pass


class _FakeOut:
    def __init__(self) -> None:
        self.msgs: list[Any] = []

    def publish(self, msg: Any) -> None:
        self.msgs.append(msg)


def _ns(**kw: Any) -> types.SimpleNamespace:
    return types.SimpleNamespace(**kw)


@pytest.fixture(autouse=True)
def _fake_ros_msgs(monkeypatch: Any) -> None:
    class _RosJointState:
        def __init__(self) -> None:
            self.header = _ns(stamp=None)
            self.name: list[str] = []
            self.position: list[float] = []
            self.velocity: list[float] = []
            self.effort: list[float] = []

    class _TwistStamped:
        def __init__(self) -> None:
            self.header = _ns(stamp=None)
            self.twist = _ns(
                linear=_ns(x=0.0, y=0.0, z=0.0),
                angular=_ns(x=0.0, y=0.0, z=0.0),
            )

    class _Bool:
        def __init__(self, data: bool = False) -> None:
            self.data = data

    for mod_name, attrs in (
        ("sensor_msgs", {"JointState": _RosJointState}),
        ("geometry_msgs", {"TwistStamped": _TwistStamped}),
        ("std_msgs", {"Bool": _Bool}),
    ):
        top = types.ModuleType(mod_name)
        sub = types.ModuleType(f"{mod_name}.msg")
        for k, v in attrs.items():
            setattr(sub, k, v)
        top.msg = sub  # type: ignore[attr-defined]
        monkeypatch.setitem(sys.modules, mod_name, top)
        monkeypatch.setitem(sys.modules, f"{mod_name}.msg", sub)


class _NoRpc(RPCSpec):
    """Module.__init__ treats a ValueError from the rpc factory as rpc disabled."""

    def __init__(self, **kw: Any) -> None:
        raise ValueError("rpc disabled for unit tests")


@pytest.fixture(autouse=True)
def _no_background_threads(monkeypatch: Any) -> None:
    monkeypatch.setattr(module_mod, "get_loop", lambda: (types.SimpleNamespace(), None))


def _fresh_segment(segment: Any, values: list[float] | None = None) -> None:
    segment.seen = True
    segment.rx_monotonic = time.monotonic()
    segment.stamp = time.time()
    if values is not None:
        segment.q[:] = values


def _construct() -> R1LiteConnection:
    c = R1LiteConnection(rpc_transport=_NoRpc)
    # Module.__init__ leaves rpc unset when the factory raises ValueError;
    # Module.stop() reads it.
    c.rpc = None  # type: ignore[assignment]
    return c


def _bare(state: ConnectionState = ConnectionState.READY_DISARMED) -> R1LiteConnection:
    c = _construct()
    c._ros = _FakeRos()  # type: ignore[assignment]
    c._cmd_left_topic = "left"  # type: ignore[assignment]
    c._cmd_right_topic = "right"  # type: ignore[assignment]
    c._cmd_gripper_left_topic = "gl"  # type: ignore[assignment]
    c._cmd_gripper_right_topic = "gr"  # type: ignore[assignment]
    c._speed_topic = "speed"  # type: ignore[assignment]
    c._acc_topic = "acc"  # type: ignore[assignment]
    c._brake_topic = "brake"  # type: ignore[assignment]
    _fresh_segment(c._left, [0.1] * 6)
    _fresh_segment(c._right, [0.2] * 6)
    _fresh_segment(c._torso, [0.3] * 4)
    c._state = state
    c._arming_nonce = "abc123"
    for stream in (
        "motor_states",
        "torso_states",
        "imu_chassis",
        "imu_torso",
        "gripper_left_state",
        "gripper_right_state",
        "odom",
        "connection_status",
    ):
        setattr(c, stream, _FakeOut())
    return c


def _armed(c: R1LiteConnection) -> R1LiteConnection:
    c._on_arming(String(data="ARM RC5 abc123"))
    assert c._armed
    return c


def _motor_cmd(num_joints: int = 12) -> Any:
    q = [float(i) for i in range(num_joints)]
    return _Msg(num_joints=num_joints, q=q, dq=[0.0] * num_joints)


def _vendor_arm_msgs(c: R1LiteConnection) -> list[tuple[Any, Any]]:
    return [(t, m) for t, m in c._ros.published if t in ("left", "right")]  # type: ignore[union-attr]


def _fb(position: list[float], velocity: list[float] | None = None, stamp_sec: int = 5) -> Any:
    return _Msg(
        header=_ns(stamp=_ns(sec=stamp_sec, nanosec=0)),
        position=position,
        velocity=velocity if velocity is not None else [],
        effort=[],
    )


# Wire contract


def test_motor_command_12_joints_sliced_left_then_right() -> None:
    c = _armed(_bare())
    c._on_motor_command(_motor_cmd())
    assert c._run_one_tick()
    msgs = _vendor_arm_msgs(c)
    assert [t for t, _ in msgs] == ["left", "right"]
    assert msgs[0][1].position == [0.0, 1.0, 2.0, 3.0, 4.0, 5.0]
    assert msgs[1][1].position == [6.0, 7.0, 8.0, 9.0, 10.0, 11.0]


@pytest.mark.parametrize("count", [11, 13, 16])
def test_motor_command_wrong_length_rejected_whole(count: int) -> None:
    c = _armed(_bare())
    c._on_motor_command(_motor_cmd(count))
    assert c._arm_cmd is None
    assert c._run_one_tick()
    assert _vendor_arm_msgs(c) == []


def test_motor_states_is_12_joints_left_then_right() -> None:
    c = _bare()
    c._publish_feedback_streams()
    out = c.motor_states  # type: ignore[assignment]
    assert len(out.msgs) == 1
    msg = out.msgs[0]
    assert msg.name == cfg.R1LITE_ARM_JOINTS
    assert msg.position == [0.1] * 6 + [0.2] * 6


def test_torso_stream_separate_and_isolated() -> None:
    c = _bare()
    c._publish_feedback_streams()
    assert c.torso_states.msgs[0].name == cfg.R1LITE_TORSO_JOINTS
    # Torso staleness never pauses motor_states.
    c._torso.rx_monotonic -= 10.0
    c._publish_feedback_streams()
    assert len(c.motor_states.msgs) == 2
    assert len(c.torso_states.msgs) == 1


def test_arm_staleness_pauses_motor_states() -> None:
    c = _bare()
    c._left.rx_monotonic -= 10.0
    c._publish_feedback_streams()
    assert c.motor_states.msgs == []


def test_malformed_feedback_rejected_without_partial_copy() -> None:
    c = _bare()
    before = list(c._left.q)
    c._on_arm_feedback("left", _fb([1.0] * 5))
    assert c._left.q == before
    c._on_arm_feedback("left", _fb([9.0] * 6, velocity=[1.0] * 4))
    assert c._left.q == before


# Timestamps


def test_motor_states_ts_is_older_arm_stamp() -> None:
    c = _bare()
    c._left.stamp = 100.0
    c._right.stamp = 90.0
    c._publish_feedback_streams()
    assert c.motor_states.msgs[0].ts == 90.0


def test_zero_vendor_stamp_falls_back_and_counts() -> None:
    c = _bare()
    c._on_arm_feedback("left", _fb([1.0] * 6, stamp_sec=0))
    assert c._left.stamp_fallbacks == 1
    assert c._left.stamp > 1e9


def test_vendor_stamp_preserved() -> None:
    c = _bare()
    c._on_arm_feedback("left", _fb([1.0] * 6, stamp_sec=42))
    assert c._left.stamp == 42.0


def test_gripper_feedback_passthrough_stamp() -> None:
    c = _bare()
    c._on_gripper_feedback("left", _fb([55.0], stamp_sec=7))
    assert c.gripper_left_state.msgs[0].ts == 7.0
    assert c.gripper_left_state.msgs[0].position == [55.0]


def test_odom_derived_from_chassis_speed_with_vendor_stamp() -> None:
    c = _bare()

    def speed(sec: int, vx: float) -> Any:
        return _Msg(
            header=_ns(stamp=_ns(sec=sec, nanosec=0)),
            twist=_ns(linear=_ns(x=vx, y=0.0, z=0.0), angular=_ns(x=0.0, y=0.0, z=0.0)),
        )

    c._on_chassis_speed(speed(10, 1.0), None)
    c._on_chassis_speed(speed(11, 1.0), None)
    assert len(c.odom.msgs) == 1
    pose = c.odom.msgs[0]
    assert pose.frame_id == "odom"
    assert pose.ts == 11.0
    assert pose.position.x == pytest.approx(1.0)


# Arming and disarmed behavior


def test_disarmed_drops_all_actuator_inputs() -> None:
    c = _bare()
    c._on_motor_command(_motor_cmd())
    c._on_cmd_vel(Twist(linear=Vector3(0.5, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0)))
    c._on_gripper_command("left", JointState(position=[50.0]))
    assert c._arm_cmd is None
    assert c._latest_cmd_vel is None
    assert c._gripper_targets["left"] is None
    assert c._run_one_tick()
    assert c._ros.published == []  # type: ignore[union-attr]


def test_arm_requires_exact_nonce_and_state() -> None:
    c = _bare()
    c._on_arming(String(data="ARM RC5 wrong"))
    assert not c._armed
    c._on_arming(String(data="ARM abc123"))
    assert not c._armed
    c._on_arming(String(data="ARM RC5 abc123"))
    assert c._armed
    assert c._state is ConnectionState.ARMED
    # The used nonce never arms again.
    assert c._arming_nonce != "abc123"


def test_arm_rejected_when_feedback_stale() -> None:
    c = _bare()
    c._left.rx_monotonic -= 10.0
    c._on_arming(String(data="ARM RC5 abc123"))
    assert not c._armed


def test_disarm_clears_caches() -> None:
    c = _armed(_bare())
    c._on_motor_command(_motor_cmd())
    c._on_cmd_vel(Twist(linear=Vector3(0.5, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0)))
    c._on_arming(String(data="DISARM"))
    assert not c._armed
    assert c._arm_cmd is None
    assert c._latest_cmd_vel is None


def test_stale_feedback_disarms_and_recovery_never_rearms() -> None:
    c = _armed(_bare())
    c._on_motor_command(_motor_cmd())
    c._left.rx_monotonic -= 10.0
    assert c._run_one_tick()
    assert not c._armed
    assert c._state is ConnectionState.READY_DISARMED
    assert c._arm_cmd is None
    published_before = len(c._ros.published)  # type: ignore[union-attr]
    # Feedback returns fresh: still disarmed, still nothing published.
    _fresh_segment(c._left)
    c._on_motor_command(_motor_cmd())
    assert c._run_one_tick()
    assert not c._armed
    assert len(c._ros.published) == published_before  # type: ignore[union-attr]


# Publish-gate linearization


def test_stop_transition_blocks_all_later_publication() -> None:
    c = _armed(_bare())
    c._on_motor_command(_motor_cmd())
    with c._publish_gate, c._lifecycle_lock:
        c._state = ConnectionState.STOPPING
        c._armed = False
        c._clear_command_caches_locked()
    assert c._run_one_tick() is False
    assert _vendor_arm_msgs(c) == []


def test_inflight_tick_completes_before_stop_transition() -> None:
    c = _armed(_bare())
    c._on_motor_command(_motor_cmd())
    entered = threading.Event()
    release = threading.Event()
    original = c._publish_vendor_commands

    def blocking_publish(snap: Any) -> None:
        entered.set()
        assert release.wait(timeout=5.0)
        original(snap)

    c._publish_vendor_commands = blocking_publish  # type: ignore[method-assign]
    tick = threading.Thread(target=c._run_one_tick)
    tick.start()
    assert entered.wait(timeout=5.0)

    stopper = threading.Thread(target=c.stop)
    stopper.start()
    time.sleep(0.05)
    # stop() is blocked on the gate: the transition has not happened.
    assert c._state is ConnectionState.ARMED
    published_at_release = len(_vendor_arm_msgs(c))
    assert published_at_release == 0
    release.set()
    tick.join(timeout=5.0)
    stopper.join(timeout=10.0)
    # The in-flight publication landed, then stop transitioned; nothing after.
    msgs = _vendor_arm_msgs(c)
    assert len(msgs) == 2
    assert c._state is ConnectionState.STOPPED
    assert c._run_one_tick() is False
    assert len(_vendor_arm_msgs(c)) == 2


# Stop sequence and lifecycle


def test_stop_streams_chassis_zero_and_reports_settled() -> None:
    c = _armed(_bare())
    c.config.stop_zero_duration_s = 0.02
    c._on_cmd_vel(Twist(linear=Vector3(0.5, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0)))
    c._last_chassis_fb_ts = time.monotonic() + 10.0
    c._last_chassis_lin = 0.0
    c._last_chassis_ang = 0.0
    c.stop()
    speeds = [m for t, m in c._ros.published if t == "speed"]  # type: ignore[union-attr]
    assert speeds
    assert all(m.twist.linear.x == 0.0 for m in speeds)
    assert c._state is ConnectionState.STOPPED


def test_stop_zero_path_reserved_for_stop() -> None:
    c = _armed(_bare())
    with pytest.raises(RuntimeError):
        c._stream_chassis_zero(0.01)


def test_stop_idempotent_and_from_created() -> None:
    c = _construct()
    c.stop()
    assert c._state is ConnectionState.STOPPED
    c.stop()
    assert c._state is ConnectionState.STOPPED


def test_single_use_start_after_stop_raises() -> None:
    c = _bare()
    c.config.stop_zero_duration_s = 0.01
    c.stop()
    with pytest.raises(RuntimeError):
        c.start()


def test_partial_start_unwinds_cleanup_stack_in_reverse() -> None:
    c = _construct()
    order: list[str] = []

    def failing_start() -> None:
        c._cleanup_stack.append(("first", lambda: order.append("first")))
        c._cleanup_stack.append(("second", lambda: order.append("second")))
        raise RuntimeError("resource three failed")

    c._start_resources = failing_start  # type: ignore[method-assign]
    with pytest.raises(RuntimeError):
        c.start()
    assert order == ["second", "first"]
    assert c._cleanup_stack == []
    assert c._state is ConnectionState.FAILED
    c.stop()
    assert c._state is ConnectionState.STOPPED


# Chassis dead-man and command mapping


def test_chassis_streams_fresh_then_zeros_when_stale() -> None:
    c = _armed(_bare())
    c._on_cmd_vel(Twist(linear=Vector3(0.5, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0)))
    assert c._run_one_tick()
    speeds = [m for t, m in c._ros.published if t == "speed"]  # type: ignore[union-attr]
    assert speeds[-1].twist.linear.x == pytest.approx(0.5)
    c._latest_cmd_vel_ts -= 1.0
    assert c._run_one_tick()
    speeds = [m for t, m in c._ros.published if t == "speed"]  # type: ignore[union-attr]
    assert speeds[-1].twist.linear.x == 0.0
    accs = [m for t, m in c._ros.published if t == "acc"]  # type: ignore[union-attr]
    brakes = [m for t, m in c._ros.published if t == "brake"]  # type: ignore[union-attr]
    assert accs and brakes and brakes[-1].data is False


def test_tracking_velocity_mapping() -> None:
    c = _bare()
    c.config.tracking_speed = 1.25
    assert c._tracking_velocities([0.0, VEL_STOP, 2.0]) == [1.25, 1.25, 2.0]


def test_stale_arm_command_cache_not_republished() -> None:
    c = _armed(_bare())
    c._on_motor_command(_motor_cmd())
    c._arm_cmd_ts -= 1.0
    assert c._run_one_tick()
    assert _vendor_arm_msgs(c) == []


def test_gripper_range_validated_and_streamed_while_fresh() -> None:
    c = _armed(_bare())
    c._on_gripper_command("left", JointState(position=[150.0]))
    assert c._gripper_targets["left"] is None
    c._on_gripper_command("left", JointState(position=[50.0]))
    assert c._run_one_tick()
    gl = [m for t, m in c._ros.published if t == "gl"]  # type: ignore[union-attr]
    assert gl and gl[0].position == [50.0]


# Import hygiene


def test_no_ros_import_at_module_level() -> None:
    tree = ast.parse(_CONN_SRC.read_text())
    banned = {"rclpy", "sensor_msgs", "geometry_msgs", "std_msgs", "builtin_interfaces"}
    for node in tree.body:
        if isinstance(node, ast.Import):
            names = {alias.name.split(".")[0] for alias in node.names}
        elif isinstance(node, ast.ImportFrom):
            names = {(node.module or "").split(".")[0]}
        else:
            continue
        assert not (names & banned), f"module-level ROS import: {names & banned}"

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

"""Galaxea R1 Lite connection: ROS 2 control and sensor streams.

One Module owns all ROS 2 traffic: a control RawROS node plus an isolated
rclpy context for camera subscriptions.

Wire contract: motor_states and motor_command carry exactly 12 values
ordered left_arm[0:6], right_arm[0:6]. Torso feedback is a separate
read-only stream; torso is never commanded (parallelogram linkage).
Grippers use the vendor 0-100 scale (0 closed).

Safety contract: the connection starts DISARMED and publishes no actuator
commands until an operator arms it through the arming stream with the
nonce it announces (scripts/r1lite_test/preflight.py). Stale arm feedback
while armed disarms; recovery never rearms. All actuator publication runs
on one thread gated by a publication lock, so stop() can guarantee nothing
publishes after its transition. The chassis node latches its last target
with no timeout of its own, so the chassis command is streamed with a
dead-man and stop() ends on a bounded zero stream.

Lifecycle: single-use. start() builds resources on a cleanup stack and
unwinds it on partial failure; stop() is idempotent from every state.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, field
import enum
import math
import queue
import secrets
import threading
from threading import Thread
import time
from typing import TYPE_CHECKING, Any

from pydantic import Field
from reactivex.disposable import Disposable

if TYPE_CHECKING:
    from dimos.protocol.pubsub.impl.rospubsub import RawROS, RawROSTopic

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.hardware.whole_body.spec import VEL_STOP
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray
from dimos.msgs.std_msgs.String import String
from dimos.robot.galaxea.r1lite import config as cfg
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_ARM_DOF = cfg.ARM_DOF
_NUM_COMMAND_JOINTS = 2 * _ARM_DOF
_LEFT = slice(0, _ARM_DOF)
_RIGHT = slice(_ARM_DOF, _NUM_COMMAND_JOINTS)

_FEEDBACK_DISCOVERY_TIMEOUT_S = 5.0
_WARN_PERIOD_S = 5.0

_COMPRESSED_CAMERAS: dict[str, str] = {
    "head_left_color": "/hdas/camera_head/left_raw/image_raw_color/compressed",
    "head_right_color": "/hdas/camera_head/right_raw/image_raw_color/compressed",
    "wrist_left_color": "/hdas/camera_wrist_left/color/image_raw/compressed",
    "wrist_right_color": "/hdas/camera_wrist_right/color/image_raw/compressed",
}
_DEPTH_CAMERAS: dict[str, str] = {
    "wrist_left_depth": "/hdas/camera_wrist_left/aligned_depth_to_color/image_raw",
    "wrist_right_depth": "/hdas/camera_wrist_right/aligned_depth_to_color/image_raw",
}


class ConnectionState(enum.Enum):
    CREATED = "created"
    STARTING = "starting"
    READY_DISARMED = "ready_disarmed"
    ARMED = "armed"
    STOPPING = "stopping"
    STOPPED = "stopped"
    FAILED = "failed"


@dataclass
class _Segment:
    """One feedback segment: cached values plus freshness bookkeeping."""

    dof: int
    q: list[float] = field(default_factory=list)
    dq: list[float] = field(default_factory=list)
    eff: list[float] = field(default_factory=list)
    seen: bool = False
    rx_monotonic: float = 0.0
    # Vendor header stamp in epoch seconds; receive wall time when the
    # vendor stamp was missing or zero (stamp_fallbacks counts those).
    stamp: float = 0.0
    stamp_fallbacks: int = 0

    def __post_init__(self) -> None:
        self.q = [0.0] * self.dof
        self.dq = [0.0] * self.dof
        self.eff = [0.0] * self.dof


@dataclass(frozen=True)
class _TickSnapshot:
    """One atomic authorization snapshot for the publish loop."""

    state: ConnectionState
    armed: bool
    arm_cmd: tuple[list[float], list[float], list[float], list[float]] | None
    gripper_left: float | None
    gripper_right: float | None
    cmd_vel: Twist | None
    chassis_active: bool


def _make_qos() -> Any:
    """BEST_EFFORT QoS for subscriptions (the robot publishes best-effort)."""
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    return QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


def _make_cmd_qos() -> Any:
    """RELIABLE QoS for command publishers.

    A reliable publisher reaches both reliable and best-effort subscribers;
    the robot has at least one reliable subscriber on target_speed_chassis.
    """
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    return QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


def _header_stamp_or_fallback(msg: Any, segment: _Segment) -> None:
    """Record the vendor header stamp, or receive wall time when absent."""
    stamp = getattr(getattr(msg, "header", None), "stamp", None)
    seconds = 0.0
    if stamp is not None:
        seconds = float(stamp.sec) + float(stamp.nanosec) * 1e-9
    if seconds > 0.0:
        segment.stamp = seconds
    else:
        segment.stamp = time.time()
        segment.stamp_fallbacks += 1


class R1LiteConnectionConfig(ModuleConfig):
    publish_rate_hz: float = Field(default=100.0)
    # rad/s used when MotorCommand.dq is the VEL_STOP sentinel or 0.
    tracking_speed: float = Field(default=0.5)
    # Camera decode costs real CPU (four JPEG streams); teleop turns it off.
    enable_cameras: bool = Field(default=True)
    publish_odom: bool = Field(default=True)
    acc_limit_x: float = Field(default=0.5)
    acc_limit_y: float = Field(default=0.5)
    acc_limit_yaw: float = Field(default=0.5)
    # Cached commands older than these stream nothing (dead-man; the
    # chassis node latches its last target forever, so its timeout must
    # live here).
    cmd_vel_timeout_s: float = Field(default=0.3)
    arm_cmd_timeout_s: float = Field(default=0.2)
    gripper_cmd_timeout_s: float = Field(default=0.2)
    # Feedback older than this marks a segment stale. Arm staleness pauses
    # motor_states, drops arm commands, and disarms.
    feedback_stale_after_s: float = Field(default=0.5)
    # On stop(), stream chassis zeros for this long before ROS teardown.
    stop_zero_duration_s: float = Field(default=0.3)
    frame_id: str = Field(default="r1lite_base_link")


class R1LiteConnection(Module):
    """R1 Lite module: ROS 2 control node, isolated sensor context, arming."""

    config: R1LiteConnectionConfig

    # Control inputs.
    motor_command: In[MotorCommandArray]
    cmd_vel: In[Twist]
    gripper_left_command: In[JointState]
    gripper_right_command: In[JointState]
    arming: In[String]

    # Feedback.
    motor_states: Out[JointState]
    torso_states: Out[JointState]
    imu_chassis: Out[Imu]
    imu_torso: Out[Imu]
    gripper_left_state: Out[JointState]
    gripper_right_state: Out[JointState]
    odom: Out[PoseStamped]
    connection_status: Out[String]

    # Perception.
    head_left_color: Out[Image]
    head_right_color: Out[Image]
    wrist_left_color: Out[Image]
    wrist_left_depth: Out[Image]
    wrist_right_color: Out[Image]
    wrist_right_depth: Out[Image]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)

        # Lock order everywhere: _publish_gate outer, _lifecycle_lock inner.
        # The gate serializes ordinary vendor publication against the
        # STOPPING transition; the lifecycle lock guards state, the armed
        # latch, caches, and feedback segments.
        self._publish_gate = threading.Lock()
        self._lifecycle_lock = threading.Lock()
        self._state = ConnectionState.CREATED
        self._armed = False
        self._arming_nonce = ""
        # Resources created by start(), released in reverse on failure/stop.
        self._cleanup_stack: list[tuple[str, Callable[[], None]]] = []

        self._ros: RawROS | None = None
        self._cmd_left_topic: RawROSTopic | None = None
        self._cmd_right_topic: RawROSTopic | None = None
        self._cmd_gripper_left_topic: RawROSTopic | None = None
        self._cmd_gripper_right_topic: RawROSTopic | None = None
        self._speed_topic: RawROSTopic | None = None
        self._acc_topic: RawROSTopic | None = None
        self._brake_topic: RawROSTopic | None = None

        self._left = _Segment(_ARM_DOF)
        self._right = _Segment(_ARM_DOF)
        self._torso = _Segment(cfg.TORSO_DOF)

        # Command caches; the publish loop is the only vendor publisher.
        self._arm_cmd: tuple[list[float], list[float], list[float], list[float]] | None = None
        self._arm_cmd_ts = 0.0
        self._gripper_targets: dict[str, float | None] = {"left": None, "right": None}
        self._gripper_ts: dict[str, float] = {"left": 0.0, "right": 0.0}
        self._latest_cmd_vel: Twist | None = None
        self._latest_cmd_vel_ts = 0.0
        self._cmd_vel_active = False

        self._latest_imu_chassis: Imu | None = None
        self._latest_imu_torso: Imu | None = None
        self._last_chassis_lin = 0.0
        self._last_chassis_ang = 0.0
        self._last_chassis_fb_ts = 0.0

        # Odom dead-reckoning, integrated from chassis speed feedback.
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_yaw = 0.0
        self._odom_last_ts: float | None = None

        # Rate-limited log bookkeeping.
        self._drop_warn_ts = 0.0
        self._bad_fb_warn_ts = 0.0
        self._state_stale_logged = False

        # Telemetry.
        self._telem_cmd_left_q: list[float] | None = None
        self._telem_cmd_right_q: list[float] | None = None
        self._telem_arm_cmd_count = 0
        self._telem_cmd_vel_count = 0
        self._telem_tick = 0

        # Sensor isolated context.
        self._sensor_context: Any = None
        self._sensor_node: Any = None
        self._sensor_executor: Any = None
        self._sensor_spin_thread: Thread | None = None
        self._sensor_stop = threading.Event()
        self._sensor_workers: list[Thread] = []
        self._cam_queues: dict[str, queue.Queue[Any]] = {}
        self._depth_queues: dict[str, queue.Queue[Any]] = {}
        self._imu_chassis_q: queue.Queue[Any] = queue.Queue(maxsize=4)
        self._imu_torso_q: queue.Queue[Any] = queue.Queue(maxsize=4)

        self._stop_event = threading.Event()
        self._publish_thread: Thread | None = None
        # Set only inside stop(); gates the internal chassis-zero path.
        self._stop_zero_allowed = False

    # Lifecycle

    @rpc
    def start(self) -> None:
        with self._lifecycle_lock:
            if self._state is not ConnectionState.CREATED:
                raise RuntimeError(
                    f"R1LiteConnection is single-use; start() from {self._state.name}"
                )
            self._state = ConnectionState.STARTING
        super().start()

        try:
            self._start_resources()
        except Exception:
            self._release_resources()
            with self._lifecycle_lock:
                self._state = ConnectionState.FAILED
            raise

        with self._lifecycle_lock:
            self._state = ConnectionState.READY_DISARMED
            self._arming_nonce = secrets.token_hex(3)
            nonce = self._arming_nonce
        logger.info(
            "R1LiteConnection started DISARMED. Arm with preflight.py "
            "(nonce %s); actuator output is inert until then.",
            nonce,
        )

    def _start_resources(self) -> None:
        """Create every connection-owned resource, pushing cleanups as we go."""
        # Lazy import: rclpy must not load at import time in ROS-less envs.
        from dimos.protocol.pubsub.impl.rospubsub import RawROS

        ros = RawROS(node_name=cfg.ROS_NODE_NAME)
        ros.start()
        self._ros = ros
        self._cleanup_stack.append(("control_ros", self._release_control_ros))

        self._setup_control_topics()
        self._setup_sensor_streams()

        deadline = time.monotonic() + _FEEDBACK_DISCOVERY_TIMEOUT_S
        while time.monotonic() < deadline:
            with self._lifecycle_lock:
                if self._left.seen and self._right.seen and self._torso.seen:
                    break
            time.sleep(0.05)
        with self._lifecycle_lock:
            seen = (self._left.seen, self._right.seen, self._torso.seen)
        if not all(seen):
            logger.warning(
                "Feedback discovery timeout: left=%s right=%s torso=%s; "
                "streams gate their first publish until feedback arrives.",
                *seen,
            )

        self.register_disposable(Disposable(self.motor_command.subscribe(self._on_motor_command)))
        self.register_disposable(Disposable(self.cmd_vel.subscribe(self._on_cmd_vel)))
        self.register_disposable(
            Disposable(
                self.gripper_left_command.subscribe(
                    lambda msg: self._on_gripper_command("left", msg)
                )
            )
        )
        self.register_disposable(
            Disposable(
                self.gripper_right_command.subscribe(
                    lambda msg: self._on_gripper_command("right", msg)
                )
            )
        )
        self.register_disposable(Disposable(self.arming.subscribe(self._on_arming)))

        thread = Thread(target=self._publish_loop, name="r1lite-publish", daemon=True)
        thread.start()
        self._publish_thread = thread
        self._cleanup_stack.append(("publish_thread", self._release_publish_thread))

    @rpc
    def stop(self) -> None:
        # Step 1: atomic STOPPING + disarm behind the publication gate, so
        # no ordinary vendor publication straddles or follows the transition.
        with self._publish_gate:
            with self._lifecycle_lock:
                if self._state in (ConnectionState.STOPPED, ConnectionState.CREATED):
                    # Idempotent: repeated stop and stop-before-start no-op.
                    self._state = ConnectionState.STOPPED
                    return
                was_running = self._state in (
                    ConnectionState.READY_DISARMED,
                    ConnectionState.ARMED,
                )
                self._state = ConnectionState.STOPPING
                self._armed = False
                self._clear_command_caches_locked()

        self._stop_event.set()
        self._sensor_stop.set()
        if self._publish_thread is not None and self._publish_thread.is_alive():
            self._publish_thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)

        if was_running:
            self._stop_zero_allowed = True
            try:
                self._stream_chassis_zero(self.config.stop_zero_duration_s)
            finally:
                self._stop_zero_allowed = False

        self._release_resources()
        with self._lifecycle_lock:
            self._state = ConnectionState.STOPPED
        logger.info("R1LiteConnection stopped")
        super().stop()

    def _clear_command_caches_locked(self) -> None:
        """Drop every cached actuator command. Caller holds _lifecycle_lock."""
        self._arm_cmd = None
        self._gripper_targets = {"left": None, "right": None}
        self._latest_cmd_vel = None
        self._cmd_vel_active = False

    def _release_resources(self) -> None:
        """Release connection-owned resources in reverse creation order."""
        self._stop_event.set()
        self._sensor_stop.set()
        while self._cleanup_stack:
            name, release = self._cleanup_stack.pop()
            try:
                release()
            except Exception as exc:
                logger.warning(f"cleanup of {name} raised: {exc}")

    def _release_publish_thread(self) -> None:
        if self._publish_thread is not None and self._publish_thread.is_alive():
            self._publish_thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        self._publish_thread = None

    def _release_control_ros(self) -> None:
        if self._ros is not None:
            self._ros.stop()
        self._ros = None

    # Control RawROS setup

    def _setup_control_topics(self) -> None:
        from geometry_msgs.msg import TwistStamped
        from sensor_msgs.msg import JointState as RosJointState
        from std_msgs.msg import Bool

        from dimos.protocol.pubsub.impl.rospubsub import RawROSTopic

        assert self._ros is not None
        qos = _make_qos()
        cmd_qos = _make_cmd_qos()

        self._cmd_left_topic = RawROSTopic(cfg.CMD_ARM_LEFT, RosJointState, qos=cmd_qos)
        self._cmd_right_topic = RawROSTopic(cfg.CMD_ARM_RIGHT, RosJointState, qos=cmd_qos)
        self._cmd_gripper_left_topic = RawROSTopic(cfg.CMD_GRIPPER_LEFT, RosJointState, qos=cmd_qos)
        self._cmd_gripper_right_topic = RawROSTopic(
            cfg.CMD_GRIPPER_RIGHT, RosJointState, qos=cmd_qos
        )
        self._speed_topic = RawROSTopic(cfg.CMD_CHASSIS_SPEED, TwistStamped, qos=cmd_qos)
        self._acc_topic = RawROSTopic(cfg.CMD_CHASSIS_ACC_LIMIT, TwistStamped, qos=cmd_qos)
        self._brake_topic = RawROSTopic(cfg.CMD_BRAKE_MODE, Bool, qos=cmd_qos)

        subscriptions: list[tuple[str, Any, Callable[[Any, Any], None]]] = [
            (cfg.FB_ARM_LEFT, RosJointState, lambda m, _t: self._on_arm_feedback("left", m)),
            (cfg.FB_ARM_RIGHT, RosJointState, lambda m, _t: self._on_arm_feedback("right", m)),
            (cfg.FB_TORSO, RosJointState, self._on_torso_feedback),
            (
                cfg.FB_GRIPPER_LEFT,
                RosJointState,
                lambda m, _t: self._on_gripper_feedback("left", m),
            ),
            (
                cfg.FB_GRIPPER_RIGHT,
                RosJointState,
                lambda m, _t: self._on_gripper_feedback("right", m),
            ),
            # Gate 1: the chassis node runs its control path only while
            # someone subscribes to its measured speed. Also drives odom.
            (cfg.FB_CHASSIS_SPEED, TwistStamped, self._on_chassis_speed),
        ]
        for topic_name, msg_type, callback in subscriptions:
            topic = RawROSTopic(topic_name, msg_type, qos=qos)
            unsub = self._ros.subscribe(topic, callback)
            self._cleanup_stack.append((f"sub:{topic_name}", unsub))

    # Sensor isolated-context setup

    def _setup_sensor_streams(self) -> None:
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.node import Node as RclpyNode

        try:
            from sensor_msgs.msg import CompressedImage, Image as RosImage, Imu as RosImu
        except ImportError:
            logger.warning("sensor_msgs not available; sensor streams disabled")
            return

        qos = _make_qos()

        # Isolated DDS participant: control traffic must not contend with
        # fragmented camera UDP.
        context = Context()
        rclpy.init(context=context)
        self._sensor_context = context
        self._cleanup_stack.append(("sensor_context", self._release_sensor_context))
        self._sensor_node = RclpyNode("r1lite_sensors", context=context)
        self._cleanup_stack.append(("sensor_node", self._release_sensor_node))
        self._sensor_executor = MultiThreadedExecutor(num_threads=4, context=context)
        self._sensor_executor.add_node(self._sensor_node)
        self._cleanup_stack.append(("sensor_executor", self._release_sensor_executor))

        camera_items = _COMPRESSED_CAMERAS.items() if self.config.enable_cameras else ()
        for stream_name, ros_topic in camera_items:
            cam_q: queue.Queue[Any] = queue.Queue(maxsize=1)
            self._cam_queues[stream_name] = cam_q
            self._sensor_node.create_subscription(
                CompressedImage,
                ros_topic,
                lambda msg, q=cam_q: _enqueue_drop_oldest(q, msg),
                qos,
            )
            self._sensor_workers.append(
                Thread(
                    target=self._compressed_decode_loop,
                    args=(stream_name, cam_q),
                    daemon=True,
                    name=f"r1lite-{stream_name}",
                )
            )

        depth_items = _DEPTH_CAMERAS.items() if self.config.enable_cameras else ()
        for stream_name, ros_topic in depth_items:
            depth_q: queue.Queue[Any] = queue.Queue(maxsize=1)
            self._depth_queues[stream_name] = depth_q
            self._sensor_node.create_subscription(
                RosImage,
                ros_topic,
                lambda msg, q=depth_q: _enqueue_drop_oldest(q, msg),
                qos,
            )
            self._sensor_workers.append(
                Thread(
                    target=self._depth_decode_loop,
                    args=(stream_name, depth_q),
                    daemon=True,
                    name=f"r1lite-{stream_name}",
                )
            )

        self._sensor_node.create_subscription(
            RosImu,
            "/hdas/imu_chassis",
            lambda msg: _enqueue_drop_oldest(self._imu_chassis_q, msg),
            qos,
        )
        self._sensor_node.create_subscription(
            RosImu,
            "/hdas/imu_torso",
            lambda msg: _enqueue_drop_oldest(self._imu_torso_q, msg),
            qos,
        )
        self._sensor_workers.append(
            Thread(
                target=self._imu_decode_loop,
                args=(self._imu_chassis_q, "imu_chassis"),
                daemon=True,
                name="r1lite-imu_chassis",
            )
        )
        self._sensor_workers.append(
            Thread(
                target=self._imu_decode_loop,
                args=(self._imu_torso_q, "imu_torso"),
                daemon=True,
                name="r1lite-imu_torso",
            )
        )

        for t in self._sensor_workers:
            t.start()
        self._cleanup_stack.append(("sensor_workers", self._release_sensor_workers))

        self._sensor_spin_thread = Thread(
            target=self._sensor_spin, daemon=True, name="r1lite-sensor-spin"
        )
        self._sensor_spin_thread.start()
        self._cleanup_stack.append(("sensor_spin", self._release_sensor_spin))

        logger.info("R1Lite sensor streams up (isolated DDS context)")

    def _release_sensor_spin(self) -> None:
        if self._sensor_spin_thread is not None and self._sensor_spin_thread.is_alive():
            self._sensor_spin_thread.join(timeout=2.0)
        self._sensor_spin_thread = None

    def _release_sensor_workers(self) -> None:
        all_queues: list[queue.Queue[Any]] = [
            *self._cam_queues.values(),
            *self._depth_queues.values(),
            self._imu_chassis_q,
            self._imu_torso_q,
        ]
        for q in all_queues:
            try:
                q.put_nowait(None)
            except queue.Full:
                pass
        for t in self._sensor_workers:
            t.join(timeout=1.0)
        self._sensor_workers.clear()
        self._cam_queues.clear()
        self._depth_queues.clear()

    def _release_sensor_executor(self) -> None:
        if self._sensor_executor is not None:
            self._sensor_executor.shutdown(timeout_sec=1.0)
        self._sensor_executor = None

    def _release_sensor_node(self) -> None:
        if self._sensor_node is not None:
            self._sensor_node.destroy_node()
        self._sensor_node = None

    def _release_sensor_context(self) -> None:
        if self._sensor_context is not None:
            import rclpy

            rclpy.shutdown(context=self._sensor_context)
        self._sensor_context = None

    def _sensor_spin(self) -> None:
        executor = self._sensor_executor
        ctx = self._sensor_context
        if executor is None or ctx is None:
            return
        while not self._sensor_stop.is_set() and ctx.ok():
            try:
                executor.spin_once(timeout_sec=0.1)
            except Exception as exc:
                if not ctx.ok() or "context is not valid" in str(exc):
                    logger.warning(f"Sensor context invalid, exiting spin: {exc}")
                    break
                logger.warning(f"sensor spin_once raised (continuing): {exc}", exc_info=True)

    # Arming

    def _on_arming(self, msg: String) -> None:
        payload = msg.data.strip()
        with self._lifecycle_lock:
            if payload == "DISARM":
                if self._armed:
                    self._armed = False
                    self._clear_command_caches_locked()
                    logger.info("R1LiteConnection disarmed by operator")
                return
            expected = f"ARM RC5 {self._arming_nonce}" if self._arming_nonce else None
            if expected is None or payload != expected:
                logger.warning("arming message rejected (wrong payload or stale nonce)")
                return
            if self._state is not ConnectionState.READY_DISARMED:
                logger.warning(f"arming rejected in state {self._state.name}")
                return
            now = time.monotonic()
            if self._segment_stale(self._left, now) or self._segment_stale(self._right, now):
                logger.warning("arming rejected: arm feedback unseen or stale")
                return
            self._state = ConnectionState.ARMED
            self._armed = True
            # A used nonce never arms again; disarm-rearm mints a new one.
            self._arming_nonce = secrets.token_hex(3)
        logger.info("R1LiteConnection ARMED by operator")

    def _segment_stale(self, segment: _Segment, now: float) -> bool:
        return not segment.seen or (now - segment.rx_monotonic) > self.config.feedback_stale_after_s

    # Control input handlers: cache only, never publish (see _publish_loop).

    def _drop_disarmed(self, what: str) -> None:
        now = time.monotonic()
        if now - self._drop_warn_ts >= _WARN_PERIOD_S:
            self._drop_warn_ts = now
            logger.warning(f"{what} dropped: connection is not armed")

    def _on_motor_command(self, msg: MotorCommandArray) -> None:
        if msg.num_joints != _NUM_COMMAND_JOINTS:
            now = time.monotonic()
            if now - self._bad_fb_warn_ts >= _WARN_PERIOD_S:
                self._bad_fb_warn_ts = now
                logger.warning(
                    f"motor_command rejected: expected {_NUM_COMMAND_JOINTS} joints, "
                    f"got {msg.num_joints}"
                )
            return
        with self._lifecycle_lock:
            if not self._armed:
                self._drop_disarmed("motor_command")
                return
            left_q = [float(v) for v in msg.q[_LEFT]]
            right_q = [float(v) for v in msg.q[_RIGHT]]
            left_dq = self._tracking_velocities(list(msg.dq[_LEFT]))
            right_dq = self._tracking_velocities(list(msg.dq[_RIGHT]))
            self._arm_cmd = (left_q, left_dq, right_q, right_dq)
            self._arm_cmd_ts = time.monotonic()
            self._telem_cmd_left_q = left_q
            self._telem_cmd_right_q = right_q
            self._telem_arm_cmd_count += 1

    def _tracking_velocities(self, dqs: list[float]) -> list[float]:
        """Map MotorCommand.dq to the vendor tracker's velocity field.

        The coordinator sends dq=0.0 for plain position tracking, so 0.0
        collapses to the configured tracking speed, as does VEL_STOP.
        """
        speed = self.config.tracking_speed
        return [speed if (v == 0.0 or v == VEL_STOP) else float(v) for v in dqs]

    def _on_gripper_command(self, side: str, msg: JointState) -> None:
        if not msg.position:
            return
        target = float(msg.position[0])
        if not (cfg.GRIPPER_CLOSED <= target <= cfg.GRIPPER_OPEN + 5.0):
            logger.warning(f"gripper_{side} target {target} outside 0-100 range; ignoring")
            return
        with self._lifecycle_lock:
            if not self._armed:
                self._drop_disarmed(f"gripper_{side}_command")
                return
            self._gripper_targets[side] = target
            self._gripper_ts[side] = time.monotonic()

    def _on_cmd_vel(self, msg: Twist) -> None:
        with self._lifecycle_lock:
            if not self._armed:
                self._drop_disarmed("cmd_vel")
                return
            self._latest_cmd_vel = msg
            self._latest_cmd_vel_ts = time.monotonic()
            self._cmd_vel_active = True
            self._telem_cmd_vel_count += 1

    # Feedback callbacks

    def _on_arm_feedback(self, side: str, msg: Any) -> None:
        segment = self._left if side == "left" else self._right
        with self._lifecycle_lock:
            if self._copy_segment(msg, segment):
                segment.seen = True
                segment.rx_monotonic = time.monotonic()
                _header_stamp_or_fallback(msg, segment)
            else:
                self._warn_bad_feedback(side, msg)

    def _on_torso_feedback(self, msg: Any, _topic: Any) -> None:
        with self._lifecycle_lock:
            if self._copy_segment(msg, self._torso):
                self._torso.seen = True
                self._torso.rx_monotonic = time.monotonic()
                _header_stamp_or_fallback(msg, self._torso)
            else:
                self._warn_bad_feedback("torso", msg)

    def _on_gripper_feedback(self, side: str, msg: Any) -> None:
        if not msg.position:
            return
        stamp = getattr(getattr(msg, "header", None), "stamp", None)
        seconds = 0.0
        if stamp is not None:
            seconds = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        out = self.gripper_left_state if side == "left" else self.gripper_right_state
        out.publish(
            JointState(
                ts=seconds if seconds > 0.0 else time.time(),
                frame_id=f"r1lite_gripper_{side}",
                name=[f"r1lite/{side}_gripper"],
                position=[float(msg.position[0])],
                velocity=[float(msg.velocity[0])] if msg.velocity else [],
                effort=[float(msg.effort[0])] if msg.effort else [],
            )
        )

    @staticmethod
    def _copy_segment(msg: Any, segment: _Segment) -> bool:
        """Copy one feedback frame. Rejects any nonempty wrong-length field
        so a malformed frame never partially overwrites cached state.
        """
        if len(msg.position) != segment.dof:
            return False
        if msg.velocity and len(msg.velocity) != segment.dof:
            return False
        if msg.effort and len(msg.effort) != segment.dof:
            return False
        segment.q[:] = msg.position[:]
        if msg.velocity:
            segment.dq[:] = msg.velocity[:]
        if msg.effort:
            segment.eff[:] = msg.effort[:]
        return True

    def _warn_bad_feedback(self, segment_name: str, msg: Any) -> None:
        now = time.monotonic()
        if now - self._bad_fb_warn_ts < _WARN_PERIOD_S:
            return
        self._bad_fb_warn_ts = now
        logger.warning(
            "malformed %s feedback rejected: position=%d velocity=%d effort=%d",
            segment_name,
            len(msg.position),
            len(msg.velocity),
            len(msg.effort),
        )

    # Chassis Gate 1 + odom integration

    def _on_chassis_speed(self, msg: Any, _topic: Any) -> None:
        with self._lifecycle_lock:
            self._last_chassis_lin = max(abs(msg.twist.linear.x), abs(msg.twist.linear.y))
            self._last_chassis_ang = abs(msg.twist.angular.z)
            self._last_chassis_fb_ts = time.monotonic()
        if not self.config.publish_odom:
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp <= 0.0:
            stamp = time.time()
        if self._odom_last_ts is None:
            self._odom_last_ts = stamp
            return
        dt = stamp - self._odom_last_ts
        self._odom_last_ts = stamp
        if dt <= 0.0 or dt > 1.0:
            return
        vx = msg.twist.linear.x
        vy = msg.twist.linear.y
        wz = msg.twist.angular.z
        cy, sy = math.cos(self._odom_yaw), math.sin(self._odom_yaw)
        self._odom_x += (cy * vx - sy * vy) * dt
        self._odom_y += (sy * vx + cy * vy) * dt
        self._odom_yaw += wz * dt

        from dimos.msgs.geometry_msgs.Quaternion import Quaternion
        from dimos.msgs.geometry_msgs.Vector3 import Vector3

        half = self._odom_yaw * 0.5
        self.odom.publish(
            PoseStamped(
                ts=stamp,
                frame_id="odom",
                position=Vector3(self._odom_x, self._odom_y, 0.0),
                orientation=Quaternion(0.0, 0.0, math.sin(half), math.cos(half)),
            )
        )

    # Publish loop: the single vendor publisher

    def _tick_snapshot(self) -> _TickSnapshot:
        """One atomic authorization snapshot. Caller holds _publish_gate."""
        now = time.monotonic()
        with self._lifecycle_lock:
            # Stale arm feedback while armed disarms; recovery never rearms.
            if self._armed and (
                self._segment_stale(self._left, now) or self._segment_stale(self._right, now)
            ):
                self._armed = False
                self._state = ConnectionState.READY_DISARMED
                self._clear_command_caches_locked()
                self._arming_nonce = secrets.token_hex(3)
                logger.warning(
                    "arm feedback stale while armed: DISARMED, caches cleared; "
                    "re-arm required after recovery (nonce %s)",
                    self._arming_nonce,
                )
            arm_cmd = None
            if (
                self._arm_cmd is not None
                and (now - self._arm_cmd_ts) < self.config.arm_cmd_timeout_s
            ):
                arm_cmd = self._arm_cmd
            gl = self._gripper_targets["left"]
            if (
                gl is not None
                and (now - self._gripper_ts["left"]) >= self.config.gripper_cmd_timeout_s
            ):
                gl = None
            gr = self._gripper_targets["right"]
            if (
                gr is not None
                and (now - self._gripper_ts["right"]) >= self.config.gripper_cmd_timeout_s
            ):
                gr = None
            cmd_vel = self._latest_cmd_vel
            cmd_fresh = (
                cmd_vel is not None
                and (now - self._latest_cmd_vel_ts) < self.config.cmd_vel_timeout_s
            )
            if not cmd_fresh:
                # Dead-man: one stale transition latches zeros until fresh.
                self._latest_cmd_vel = None
                cmd_vel = None
            return _TickSnapshot(
                state=self._state,
                armed=self._armed,
                arm_cmd=arm_cmd,
                gripper_left=gl,
                gripper_right=gr,
                cmd_vel=cmd_vel,
                chassis_active=self._cmd_vel_active,
            )

    def _publish_vendor_commands(self, snap: _TickSnapshot) -> None:
        """Publish this tick's actuator commands. Caller holds _publish_gate."""
        if snap.state is not ConnectionState.ARMED or not snap.armed:
            return
        from sensor_msgs.msg import JointState as RosJointState

        ros = self._ros
        if ros is None:
            return
        stamp = ros.now_stamp()
        if stamp is None:
            return

        if snap.arm_cmd is not None:
            left_q, left_dq, right_q, right_dq = snap.arm_cmd
            for topic, qs, dqs in (
                (self._cmd_left_topic, left_q, left_dq),
                (self._cmd_right_topic, right_q, right_dq),
            ):
                if topic is None:
                    continue
                cmd = RosJointState()
                cmd.header.stamp = stamp
                cmd.name = [""]
                cmd.position = qs
                cmd.velocity = dqs
                cmd.effort = [0.0]
                ros.publish(topic, cmd)

        for _side, target, topic in (
            ("left", snap.gripper_left, self._cmd_gripper_left_topic),
            ("right", snap.gripper_right, self._cmd_gripper_right_topic),
        ):
            if target is None or topic is None:
                continue
            cmd = RosJointState()
            cmd.header.stamp = stamp
            cmd.name = [""]
            cmd.position = [target]
            cmd.velocity = [0.0]
            cmd.effort = [0.0]
            ros.publish(topic, cmd)

        if snap.chassis_active:
            zero = Twist()
            self._publish_chassis_command(snap.cmd_vel if snap.cmd_vel is not None else zero)

    def _publish_chassis_command(self, twist: Twist) -> bool:
        """One chassis tick: acc_limit + brake=false + speed target."""
        from geometry_msgs.msg import TwistStamped
        from std_msgs.msg import Bool

        ros = self._ros
        if ros is None or self._acc_topic is None or self._speed_topic is None:
            return False
        stamp = ros.now_stamp()
        if stamp is None:
            return False

        acc = TwistStamped()
        acc.header.stamp = stamp
        acc.twist.linear.x = self.config.acc_limit_x
        acc.twist.linear.y = self.config.acc_limit_y
        acc.twist.angular.z = self.config.acc_limit_yaw
        ros.publish(self._acc_topic, acc)
        if self._brake_topic is not None:
            ros.publish(self._brake_topic, Bool(data=False))
        cmd = TwistStamped()
        cmd.header.stamp = stamp
        cmd.twist.linear.x = twist.linear.x
        cmd.twist.linear.y = twist.linear.y
        cmd.twist.angular.z = twist.angular.z
        ros.publish(self._speed_topic, cmd)
        return True

    def _stream_chassis_zero(self, duration_s: float) -> None:
        """Best-effort stop stream, callable only from stop().

        Streams zero velocity for a bounded window, then checks fresh
        chassis feedback for linear and angular settling. Cannot run on
        SIGKILL or power loss; the e-stop and RC remain the hard stop
        authority.
        """
        if not self._stop_zero_allowed:
            raise RuntimeError("_stream_chassis_zero is reserved for stop()")
        zero = Twist()
        period = 1.0 / float(self.config.publish_rate_hz)
        ticks = max(1, int(duration_s / period))
        started = time.monotonic()
        sent = 0
        for _ in range(ticks):
            try:
                if self._publish_chassis_command(zero):
                    sent += 1
            except Exception as exc:
                logger.error("chassis zero publish failed during stop: %s", exc)
            time.sleep(period)
        if sent == 0:
            logger.error("chassis stop published no zero commands; chassis may be latched")
            return
        with self._lifecycle_lock:
            fb_ts = self._last_chassis_fb_ts
            lin = self._last_chassis_lin
            ang = self._last_chassis_ang
        if fb_ts < started:
            logger.warning("chassis stop unconfirmed: no fresh chassis feedback after stop")
        elif lin > 0.02 or ang > 0.05:
            logger.error("chassis not settled after stop: lin=%.3f ang=%.3f", lin, ang)

    def _publish_feedback_streams(self) -> None:
        """Publish motor/torso states and IMUs. Not actuator output."""
        now = time.monotonic()
        with self._lifecycle_lock:
            left_ok = self._left.seen and not self._segment_stale(self._left, now)
            right_ok = self._right.seen and not self._segment_stale(self._right, now)
            torso_ok = self._torso.seen and not self._segment_stale(self._torso, now)
            positions = list(self._left.q) + list(self._right.q)
            velocities = list(self._left.dq) + list(self._right.dq)
            efforts = list(self._left.eff) + list(self._right.eff)
            arm_stamp = min(self._left.stamp, self._right.stamp)
            torso = (
                (
                    list(self._torso.q),
                    list(self._torso.dq),
                    list(self._torso.eff),
                    self._torso.stamp,
                )
                if torso_ok
                else None
            )
            imu_chassis = self._latest_imu_chassis
            imu_torso = self._latest_imu_torso

        arms_fresh = left_ok and right_ok
        if not arms_fresh and not self._state_stale_logged:
            self._state_stale_logged = True
            logger.warning(
                "arm feedback unseen or stale (left=%s right=%s); pausing motor_states",
                left_ok,
                right_ok,
            )
        elif arms_fresh and self._state_stale_logged:
            self._state_stale_logged = False
            logger.info("arm feedback fresh again; motor_states resumed")

        if arms_fresh:
            self.motor_states.publish(
                JointState(
                    ts=arm_stamp,
                    frame_id=self.config.frame_id,
                    name=cfg.R1LITE_ARM_JOINTS,
                    position=positions,
                    velocity=velocities,
                    effort=efforts,
                )
            )
        if torso is not None:
            t_q, t_dq, t_eff, t_stamp = torso
            self.torso_states.publish(
                JointState(
                    ts=t_stamp,
                    frame_id=self.config.frame_id,
                    name=cfg.R1LITE_TORSO_JOINTS,
                    position=t_q,
                    velocity=t_dq,
                    effort=t_eff,
                )
            )
        if imu_chassis is not None:
            self.imu_chassis.publish(imu_chassis)
        if imu_torso is not None:
            self.imu_torso.publish(imu_torso)

    def _publish_telemetry(self) -> None:
        with self._lifecycle_lock:
            state = self._state
            armed = self._armed
            nonce = self._arming_nonce
            cl, cr = self._telem_cmd_left_q, self._telem_cmd_right_q
            fl, fr = list(self._left.q), list(self._right.q)
            n_arm, n_vel = self._telem_arm_cmd_count, self._telem_cmd_vel_count
            self._telem_arm_cmd_count = 0
            self._telem_cmd_vel_count = 0
            cv = self._latest_cmd_vel
            fallbacks = (
                self._left.stamp_fallbacks
                + self._right.stamp_fallbacks
                + self._torso.stamp_fallbacks
            )
        # track_err compares latest cached command with latest cached
        # feedback: session-level following evidence, not per-command
        # physical confirmation.
        err_l = max(abs(a - b) for a, b in zip(cl, fl, strict=False)) if cl else float("nan")
        err_r = max(abs(a - b) for a, b in zip(cr, fr, strict=False)) if cr else float("nan")
        logger.info(
            "TELEM conn: state=%s armed=%s arm_cmd_hz=%d cmdvel_hz=%d "
            "track_err_deg L=%.1f R=%.1f cmd_vel=(%.2f,%.2f,%.2f) stamp_fallbacks=%d",
            state.name,
            armed,
            n_arm,
            n_vel,
            math.degrees(err_l) if not math.isnan(err_l) else float("nan"),
            math.degrees(err_r) if not math.isnan(err_r) else float("nan"),
            cv.linear.x if cv else 0.0,
            cv.linear.y if cv else 0.0,
            cv.angular.z if cv else 0.0,
            fallbacks,
        )
        self.connection_status.publish(String(data=f"state={state.name} nonce={nonce}"))

    def _run_one_tick(self) -> bool:
        """One publish tick. Returns False once the connection is stopping."""
        with self._publish_gate:
            snap = self._tick_snapshot()
            if snap.state in (
                ConnectionState.STOPPING,
                ConnectionState.STOPPED,
                ConnectionState.FAILED,
            ):
                return False
            self._publish_vendor_commands(snap)

        self._publish_feedback_streams()
        self._telem_tick += 1
        if self._telem_tick % max(1, int(self.config.publish_rate_hz)) == 0:
            self._publish_telemetry()
        return True

    def _publish_loop(self) -> None:
        period = 1.0 / float(self.config.publish_rate_hz)
        next_tick = time.perf_counter()
        while not self._stop_event.is_set():
            if not self._run_one_tick():
                break
            next_tick += period
            sleep_for = next_tick - time.perf_counter()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                next_tick = time.perf_counter()

    # Sensor decode workers

    def _compressed_decode_loop(self, stream_name: str, q: queue.Queue[Any]) -> None:
        import cv2
        import numpy as np

        from dimos.msgs.sensor_msgs.Image import ImageFormat

        out = getattr(self, stream_name)
        while not self._sensor_stop.is_set():
            try:
                msg = q.get(timeout=0.5)
            except queue.Empty:
                continue
            if msg is None:
                break
            try:
                arr = np.frombuffer(bytes(msg.data), np.uint8)
                bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                if bgr is None:
                    continue
                ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                out.publish(Image(bgr, format=ImageFormat.BGR, frame_id=stream_name, ts=ts))
            except Exception:
                logger.exception(f"R1Lite camera {stream_name} decode error")

    def _depth_decode_loop(self, stream_name: str, q: queue.Queue[Any]) -> None:
        from dimos.protocol.pubsub.impl.rospubsub_conversion import ros_to_dimos

        out = getattr(self, stream_name)
        while not self._sensor_stop.is_set():
            try:
                msg = q.get(timeout=0.5)
            except queue.Empty:
                continue
            if msg is None:
                break
            try:
                out.publish(ros_to_dimos(msg, Image))
            except Exception:
                logger.exception(f"R1Lite {stream_name} decode error")

    def _imu_decode_loop(self, q: queue.Queue[Any], which: str) -> None:
        from dimos.protocol.pubsub.impl.rospubsub_conversion import ros_to_dimos

        target_attr = "_latest_imu_chassis" if which == "imu_chassis" else "_latest_imu_torso"
        while not self._sensor_stop.is_set():
            try:
                msg = q.get(timeout=0.5)
            except queue.Empty:
                continue
            if msg is None:
                break
            try:
                imu = ros_to_dimos(msg, Imu)
                with self._lifecycle_lock:
                    setattr(self, target_attr, imu)
            except Exception:
                logger.exception(f"R1Lite {which} decode error")


def _enqueue_drop_oldest(q: queue.Queue[Any], item: Any) -> None:
    """Latest-frame-wins enqueue for size-1 sensor queues."""
    try:
        q.put_nowait(item)
    except queue.Full:
        try:
            q.get_nowait()
        except queue.Empty:
            pass
        q.put_nowait(item)

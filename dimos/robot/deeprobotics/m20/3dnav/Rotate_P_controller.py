#!/usr/bin/env python3
# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
"""Rotate-then-drive P-controller path follower for 3D navigation.

Extracted from the 2D ReplanningAStarPlanner's PController logic and adapted
for standalone use with the MLS 3D planner's Path output.
"""

from __future__ import annotations

import math
from threading import Event, RLock, Thread
import time
from typing import Any, Literal, TypeAlias

from dimos_lcm.std_msgs import Bool  # type: ignore[import-untyped]
import numpy as np
from numpy.typing import NDArray
from reactivex.disposable import Disposable

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.nav_msgs.Path import Path
from dimos.utils.logging_config import setup_logger
from dimos.utils.trigonometry import angle_diff

logger = setup_logger()

FollowerState: TypeAlias = Literal["idle", "initial_rotation", "path_following", "final_rotation", "arrived"]


class PControllerFollowerConfig(ModuleConfig):
    speed: float = 0.5
    control_frequency: float = 10.0
    goal_tolerance: float = 0.3
    orientation_tolerance: float = 0.35

    # Lookahead — how far ahead along the path to target.
    lookahead_time_s: float = 1.5
    min_lookahead_m: float = 0.4
    max_lookahead_m: float = 1.5

    # P-controller gains
    k_angular: float = 0.5
    rotation_threshold_rad: float = math.pi / 4  # 45° — rotate in place when yaw error exceeds this
    min_linear_velocity: float = 0.2
    min_angular_velocity: float = 0.6  # M20 default


class PControllerFollower(Module):
    """Follow a path with rotate-then-drive P-controlled heading.

    At large yaw errors the robot rotates in place; once aligned it drives
    forward with proportional angular correction.  Supports both 2D and 3D
    paths — z is used for lookahead distance computation but heading is
    steered from the xy projection alone.
    """

    config: PControllerFollowerConfig

    path: In[Path]
    odometry: In[Odometry]
    stop_movement: In[Bool]

    nav_cmd_vel: Out[Twist]
    goal_reached: Out[Bool]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = RLock()
        self._current_odom: PoseStamped | None = None
        self._waypoints: NDArray[np.float32] | None = None
        self._stop_event = Event()
        self._thread: Thread | None = None
        self._state: FollowerState = "idle"
        self._pose_index: int = 0
        self._prev_yaw_error: float = 0.0
        self._prev_angular_velocity: float = 0.0
        self._last_cmd_vel = Twist()

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.odometry.subscribe(self._on_odometry)))
        self.register_disposable(Disposable(self.path.subscribe(self._on_path)))
        if self.stop_movement.transport is not None:
            self.register_disposable(Disposable(self.stop_movement.subscribe(self._on_stop)))
        self._thread = Thread(target=self._follow, daemon=True)
        self._thread.start()

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        self.nav_cmd_vel.publish(Twist())
        super().stop()

    # ── callbacks ──────────────────────────────────────────────────

    def _on_odometry(self, msg: Odometry) -> None:
        with self._lock:
            self._current_odom = msg.to_pose_stamped()

    def _on_path(self, path: Path) -> None:
        if len(path.poses) == 0:
            with self._lock:
                self._waypoints = None
            self.nav_cmd_vel.publish(Twist())
            return
        waypoints = np.array(
            [[p.position.x, p.position.y, p.position.z] for p in path.poses],
            dtype=np.float32,
        )
        with self._lock:
            self._waypoints = waypoints
            self._state = "path_following"
            self._pose_index = 0

    def _on_stop(self, msg: Bool) -> None:
        if msg.data:
            with self._lock:
                self._waypoints = None
            self.nav_cmd_vel.publish(Twist())

    # ── state machine ──────────────────────────────────────────────

    def _follow(self) -> None:
        period = 1.0 / self.config.control_frequency
        while not self._stop_event.is_set():
            start_time = time.perf_counter()
            with self._lock:
                odom = self._current_odom
                waypoints = self._waypoints
            if odom is not None and waypoints is not None:
                self._step(odom, waypoints)
            elapsed = time.perf_counter() - start_time
            self._stop_event.wait(max(0.0, period - elapsed))

    def _step(self, odom: PoseStamped, waypoints: NDArray[np.float32]) -> None:
        position = np.array(
            [odom.position.x, odom.position.y, odom.position.z], dtype=np.float32
        )

        # Goal check
        goal = waypoints[-1]
        if float(np.linalg.norm(goal - position)) < self.config.goal_tolerance:
            self.nav_cmd_vel.publish(Twist())
            with self._lock:
                if self._waypoints is waypoints:
                    self._waypoints = None
            self.goal_reached.publish(Bool(True))
            logger.info("Goal reached")
            return

        # Compute lookahead point in 3D
        target_pt = self._lookahead_point(waypoints, position)

        # Heading toward the lookahead xy
        robot_yaw = odom.orientation.euler[2]
        desired_yaw = math.atan2(
            target_pt[1] - position[1], target_pt[0] - position[0]
        )
        yaw_error = angle_diff(desired_yaw, robot_yaw)

        # Rotate-then-drive
        angular = self._compute_angular_velocity(yaw_error)

        if abs(yaw_error) > self.config.rotation_threshold_rad:
            # Large heading error — rotate in place.
            cmd = Twist(Vector3(0, 0, 0), Vector3(0, 0, angular))
        else:
            # Aligned — drive forward with proportional angular correction.
            ratio = abs(yaw_error) / self.config.rotation_threshold_rad
            linear = self.config.speed * (1.0 - ratio)
            linear = self._apply_min_velocity(linear, self.config.min_linear_velocity)
            cmd = Twist(Vector3(linear, 0, 0), Vector3(0, 0, angular))

        self._last_cmd_vel = cmd
        self.nav_cmd_vel.publish(cmd)

    # ── angular control ────────────────────────────────────────────

    def _compute_angular_velocity(self, yaw_error: float) -> float:
        angular = self.config.k_angular * yaw_error
        angular = float(np.clip(angular, -self.config.speed, self.config.speed))
        angular = self._apply_min_velocity(angular, self.config.min_angular_velocity)
        return angular

    @staticmethod
    def _apply_min_velocity(velocity: float, min_v: float) -> float:
        if velocity == 0.0:
            return 0.0
        if abs(velocity) < min_v:
            return min_v if velocity > 0 else -min_v
        return velocity

    # ── lookahead ──────────────────────────────────────────────────

    def _lookahead_point(
        self, waypoints: NDArray[np.float32], position: NDArray[np.float32]
    ) -> NDArray[np.float32]:
        """Walk forward along the 3D path until the accumulated arc length
        reaches the configured lookahead distance."""
        if len(waypoints) == 1:
            return waypoints[0].copy()

        _, start_pt = self._project_onto_path(waypoints, position)
        remaining = max(
            self.config.min_lookahead_m,
            min(self.config.max_lookahead_m,
                self.config.lookahead_time_s * self.config.speed),
        )
        for i in range(self._pose_index, len(waypoints) - 1):
            end_pt = waypoints[i + 1]
            seg = end_pt - start_pt
            seg_len = float(np.linalg.norm(seg))
            if seg_len >= remaining:
                return start_pt + (remaining / seg_len) * seg
            remaining -= seg_len
            start_pt = end_pt
        return waypoints[-1].copy()

    def _project_onto_path(
        self, waypoints: NDArray[np.float32], position: NDArray[np.float32]
    ) -> tuple[int, NDArray[np.float32]]:
        best_idx = self._pose_index
        best_point = waypoints[self._pose_index].copy()
        best_dist = float("inf")
        for i in range(self._pose_index, len(waypoints) - 1):
            a = waypoints[i]
            ab = waypoints[i + 1] - a
            denom = float(np.dot(ab, ab))
            t = 0.0 if denom == 0 else float(np.clip(np.dot(position - a, ab) / denom, 0.0, 1.0))
            proj = a + t * ab
            dist = float(np.linalg.norm(position - proj))
            if dist < best_dist:
                best_dist = dist
                best_idx = i
                best_point = proj
        self._pose_index = best_idx
        return best_idx, best_point

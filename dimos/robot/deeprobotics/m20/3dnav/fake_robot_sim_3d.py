#!/usr/bin/env python3
# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
"""3D holonomic kinematics simulator with z tracking from the planner path."""

from __future__ import annotations

import math

import reactivex as rx
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.nav_msgs.Path import Path
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class FakeRobotSim3DConfig(ModuleConfig):
    initial_x: float = -1.0
    initial_y: float = -3.5
    initial_z: float = 0.0  # ground-level z
    initial_yaw: float = 0.0
    odom_hz: float = 20.0
    robot_height: float = 0.6  # added to odom z; planner subtracts it back


class FakeRobotSim3D(Module):
    """Integrate cmd_vel into a 3D holonomic pose, tracking z from the planner path.

    x / y / yaw are driven by cmd_vel (same as the 2D FakeRobotSim).
    z snaps to the nearest path waypoint (by 3D distance) with per-tick clamping.
    """

    config: FakeRobotSim3DConfig
    cmd_vel: In[Twist]
    path: In[Path]
    slam_odom: Out[Odometry]  # ground-level z (PController, TF)
    planner_odom: Out[Odometry]  # body-height z (GoalRelay → Planner)

    _x: float = 0.0
    _y: float = 0.0
    _z: float = 0.0
    _yaw: float = 0.0
    _running: bool = False
    _latest_twist: Twist | None = None
    _path_waypoints: list[tuple[float, float, float]] = []

    @rpc
    def start(self) -> None:
        super().start()
        self._x = self.config.initial_x
        self._y = self.config.initial_y
        self._z = self.config.initial_z
        self._yaw = self.config.initial_yaw
        self._running = True
        self._latest_twist = Twist()
        self.register_disposable(Disposable(self.cmd_vel.subscribe(self._on_cmd_vel)))
        self.register_disposable(Disposable(self.path.subscribe(self._on_path)))
        period = 1.0 / max(self.config.odom_hz, 1.0)
        self.register_disposable(rx.interval(period).subscribe(on_next=self._tick))
        self._publish_odom()
        logger.warning(
            "FakeRobotSim3D started, odom at %.0f Hz, initial (%.2f, %.2f, %.2f)",
            self.config.odom_hz,
            self._x,
            self._y,
            self._z,
        )

    @rpc
    def stop(self) -> None:
        self._running = False
        super().stop()

    def _on_cmd_vel(self, twist: Twist) -> None:
        self._latest_twist = twist

    def _on_path(self, path: Path) -> None:
        if not path.poses:
            return
        pts = path.poses
        interior = pts[1:-1] if len(pts) > 2 else pts
        self._path_waypoints = [
            (p.position.x, p.position.y, p.position.z) for p in interior
        ]

    def _tick(self, _idx: int) -> None:
        if not self._running:
            return
        twist = self._latest_twist
        if twist is not None:
            dt = 1.0 / max(self.config.odom_hz, 1.0)
            vx = float(twist.linear.x)
            vy = float(twist.linear.y)
            wz = float(twist.angular.z)
            if abs(vx) > 1e-9 or abs(vy) > 1e-9 or abs(wz) > 1e-9:
                self._yaw += wz * dt
                self._x += (vx * math.cos(self._yaw) - vy * math.sin(self._yaw)) * dt
                self._y += (vx * math.sin(self._yaw) + vy * math.cos(self._yaw)) * dt

        self._z = self._interpolate_z(self._x, self._y)
        self._publish_odom()

    # ------------------------------------------------------------------
    # z interpolation
    # ------------------------------------------------------------------

    _MAX_Z_SNAP_DIST = 1.5  # m

    def _interpolate_z(self, x: float, y: float) -> float:
        """Snap z to the 3D-nearest path waypoint, clamped per tick."""
        wp = self._path_waypoints
        if not wp:
            return self._z
        if len(wp) == 1:
            return wp[0][2]

        best_z = self._z
        best_d2 = self._MAX_Z_SNAP_DIST * self._MAX_Z_SNAP_DIST

        for (wx, wy, wz) in wp:
            d2 = (x - wx) ** 2 + (y - wy) ** 2 + (self._z - wz) ** 2
            if d2 < best_d2:
                best_d2 = d2
                best_z = wz

        # Clamp z change — no instant drops.
        max_step = 0.04  # m / tick
        if best_z > self._z + max_step:
            best_z = self._z + max_step
        elif best_z < self._z - max_step:
            best_z = self._z - max_step

        return best_z

    # ------------------------------------------------------------------
    # odometry
    # ------------------------------------------------------------------

    def _publish_odom(self) -> None:
        q = Quaternion.from_euler(Vector3(0.0, 0.0, self._yaw))
        # Ground-level odom: PController goal check, TF base_link
        odom_ground = Odometry(
            ts=0.0,
            frame_id="map",
            child_frame_id="base_link",
            pose=Pose(Vector3(self._x, self._y, self._z), q),
        )
        self.slam_odom.publish(odom_ground)
        # Body-height odom: Planner subtracts robot_height for start_pose
        odom_planner = Odometry(
            ts=0.0,
            frame_id="map",
            child_frame_id="base_link",
            pose=Pose(Vector3(self._x, self._y, self._z + self.config.robot_height), q),
        )
        self.planner_odom.publish(odom_planner)

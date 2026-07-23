# Copyright 2025-2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# ...

"""2D holonomic kinematics simulator and PCD map loader for offline nav simulation."""

from __future__ import annotations

import math
from typing import Any

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
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class FakeRobotSimConfig(ModuleConfig):
    initial_x: float = -1.0
    initial_y: float = -3.5
    initial_yaw: float = 0.0
    odom_hz: float = 20.0


class FakeRobotSim(Module):
    """Integrate cmd_vel into a 2D holonomic pose, publishing odometry."""

    config: FakeRobotSimConfig
    cmd_vel: In[Twist]
    slam_odom: Out[Odometry]

    _x: float = 0.0
    _y: float = 0.0
    _yaw: float = 0.0
    _running: bool = False
    _latest_twist: Twist | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self._x = self.config.initial_x
        self._y = self.config.initial_y
        self._yaw = self.config.initial_yaw
        self._running = True
        self._latest_twist = Twist()
        self.register_disposable(Disposable(self.cmd_vel.subscribe(self._on_cmd_vel)))
        period = 1.0 / max(self.config.odom_hz, 1.0)
        self.register_disposable(rx.interval(period).subscribe(on_next=self._tick))
        self._publish_odom()
        logger.warning("FakeRobotSim started, odom publishing at %.0f Hz", self.config.odom_hz)

    @rpc
    def stop(self) -> None:
        self._running = False
        super().stop()

    def _on_cmd_vel(self, twist: Twist) -> None:
        self._latest_twist = twist

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
        self._publish_odom()

    def _publish_odom(self) -> None:
        q = Quaternion.from_euler(Vector3(0.0, 0.0, self._yaw))
        odom = Odometry(
            ts=0.0,
            frame_id="map",
            child_frame_id="base_link",
            pose=Pose(Vector3(self._x, self._y, 0.0), q),
        )
        self.slam_odom.publish(odom)

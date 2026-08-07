#!/usr/bin/env python3
# Copyright 2026 Dimensional Inc.
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

"""PCD-based offline simulation of the M20 3D MLS navigation stack.

Loads a static PCD global map and runs the MLS planner + path follower in a
closed simulation loop with FakeRobotSim.  Useful for tuning planner parameters
and validating the surface graph against a known map without a live robot.

Usage
-----
.. code-block:: bash

   cd dimos-test
   python -m dimos.run \\
       dimos.robot.deeprobotics.m20.3dnav.m20_mls_3Dnav_pcd_sim:m20_mls_3Dnav_pcd_sim
"""

from __future__ import annotations

import math
import threading
import time
from pathlib import Path
from typing import Any

import numpy as np

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.core import rpc
from dimos.core.global_config import global_config
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.nav_3d.mls_planner.goal_relay import GoalRelay
from dimos.navigation.nav_3d.mls_planner.mls_planner_native import MLSPlannerNative
from dimos.robot.deeprobotics.m20.nav.pcd_sim import FakeRobotSim
import importlib

_3d = importlib.import_module("dimos.robot.deeprobotics.m20.3dnav.fake_robot_sim_3d")
FakeRobotSim3D = _3d.FakeRobotSim3D
_ctrl = importlib.import_module("dimos.robot.deeprobotics.m20.3dnav.Rotate_P_controller")
PControllerFollower = _ctrl.PControllerFollower
from dimos.robot.deeprobotics.m20.tf import M20TF
from dimos.visualization.rerun.bridge import RerunBridgeModule
from dimos.visualization.rerun.websocket_server import RerunWebSocketServer
from dimos.web.websocket_vis.websocket_vis_module import WebsocketVisModule

# ---------------------------------------------------------------------------
# PCD map loader
# ---------------------------------------------------------------------------

PCD_PATH = str(Path(__file__).resolve().parent / "pcd" / "m20_global_map.pcd")


class PCDMapConfig(ModuleConfig):
    pcd_path: str = PCD_PATH
    publish_hz: float = 0.5  # Static map: publish infrequently


class PCDMap(Module):
    """Load a .pcd file and publish it as a PointCloud2 on /global_map."""

    config: PCDMapConfig
    global_map: Out[PointCloud2]

    _running: bool = False

    @rpc
    def start(self) -> None:
        super().start()
        pts = _load_pcd(self.config.pcd_path)
        msg = PointCloud2.from_numpy(pts, frame_id="map", timestamp=0.0)
        self._running = True
        period = 1.0 / max(self.config.publish_hz, 0.1)

        def _loop() -> None:
            while self._running:
                self.global_map.publish(msg)
                time.sleep(period)

        t = threading.Thread(target=_loop, daemon=True)
        t.start()

    @rpc
    def stop(self) -> None:
        self._running = False
        super().stop()


def _load_pcd(path: str) -> np.ndarray:
    """Load a .pcd file (ASCII or binary), returning (N,3) float32 array.

    Parses the FIELDS / SIZE / TYPE / COUNT / DATA header lines to locate the
    x, y, z fields regardless of their position or surrounding extra fields.
    """
    p = Path(path)
    if not p.is_file():
        raise FileNotFoundError(f"PCD map not found: {p}")

    raw = p.read_bytes()
    # Find the DATA line boundary — reliable for both ASCII and binary.
    data_marker = raw.find(b"DATA ")
    if data_marker < 0:
        raise ValueError("PCD missing DATA line")
    header_end = raw.index(b"\n", data_marker) + 1
    header_text = raw[:header_end].decode("utf-8", errors="replace")

    fields: list[str] = []
    size: list[int] = []
    type_: list[str] = []
    count_: list[int] = []
    num_points = 0
    binary = False

    for line in header_text.splitlines():
        parts = line.split()
        if not parts:
            continue
        key = parts[0]
        if key == "FIELDS":
            fields = parts[1:]
        elif key == "SIZE":
            size = [int(v) for v in parts[1:]]
        elif key == "TYPE":
            type_ = parts[1:]
        elif key == "COUNT":
            count_ = [int(v) for v in parts[1:]]
        elif key == "POINTS":
            num_points = int(parts[1])
        elif key == "DATA":
            binary = (parts[1] == "binary")

    if "x" not in fields or "y" not in fields or "z" not in fields:
        raise ValueError(f"PCD must contain x, y, z fields. Found: {fields}")

    # ── locate x/y/z ────────────────────────────────────────────────
    ix = fields.index("x")
    iy = fields.index("y")
    iz = fields.index("z")

    # Byte offsets and strides for binary mode.
    strides = [count_[j] * size[j] for j in range(len(fields))]
    point_bytes = sum(strides)
    offset_x = sum(strides[:ix])
    offset_y = sum(strides[:iy])
    offset_z = sum(strides[:iz])

    out = np.empty((num_points, 3), dtype=np.float32)

    if binary:
        # ── binary path ──────────────────────────────────────────
        data = raw[header_end:]
        dtype_map = {"F": "f4", "U": "u1", "I": "i4"}
        x_dt = np.dtype(dtype_map.get(type_[ix], "f4"))
        y_dt = np.dtype(dtype_map.get(type_[iy], "f4"))
        z_dt = np.dtype(dtype_map.get(type_[iz], "f4"))

        for k in range(num_points):
            base = k * point_bytes
            # Handle 1-byte x which is unsigned → cast to float.
            if type_[ix] == "U" and count_[ix] == 1:
                out[k, 0] = float(data[base + offset_x])
            else:
                out[k, 0] = np.frombuffer(data, x_dt, 1, base + offset_x)[0]
            out[k, 1] = np.frombuffer(data, y_dt, 1, base + offset_y)[0]

            # z field may be COUNT 1 (float) or COUNT 4 (4 x U8 padding).
            if type_[iz] == "U":
                # unsigned padding — z is the first sub-byte of 4
                out[k, 2] = float(data[base + offset_z])
            else:
                out[k, 2] = np.frombuffer(data, z_dt, 1, base + offset_z)[0]

        return out

    # ── ASCII path (backwards compatible) ────────────────────────────
    text = raw[header_end:].decode("utf-8", errors="replace")
    lines = text.splitlines()
    for i, line in enumerate(lines):
        if i >= num_points:
            break
        parts = line.split()
        nf = len(fields)
        if len(parts) >= max(ix, iy, iz) + 1:
            out[i, 0] = float(parts[ix])
            out[i, 1] = float(parts[iy])
            out[i, 2] = float(parts[iz])
    return out


# ---------------------------------------------------------------------------
# Rerun visualization
# ---------------------------------------------------------------------------

# Set to True for per-layer debug view (danger_cells, edge_cells,
# nodes_nms / nodes_add).  False = clean view (surface_map only).
DEBUG_VIZ = True


def _render_global_map(msg: Any) -> Any:
    return msg.to_rerun(colors=[60, 100, 200])  # blue


def _render_path(msg: Any) -> Any:
    if len(msg.poses) == 0:
        return None
    return msg.to_rerun(color=(0, 255, 128), z_offset=0.05, radii=0.06)


def _render_node_edges(msg: Any) -> Any:
    return msg.to_rerun(z_offset=0.0)


def _render_surface(msg: Any) -> Any:
    return msg.to_rerun(radii=0.03, colors=[60, 100, 200])  # blue


def _render_nodes(msg: Any) -> Any:
    return msg.to_rerun(radii=0.06, colors=[75, 200, 75])  # green


def _render_danger_cells(msg: Any) -> Any:
    return msg.to_rerun(radii=0.03, colors=[40, 40, 40])  # dark grey


def _render_edge_cells(msg: Any) -> Any:
    return msg.to_rerun(radii=0.03, colors=[220, 30, 20])  # red


def _render_nodes_nms(msg: Any) -> Any:
    return msg.to_rerun(radii=0.06, colors=[75, 200, 75])  # green


def _render_nodes_add(msg: Any) -> Any:
    return msg.to_rerun(radii=0.06, colors=[255, 100, 50])  # orange-red


def _static_scene(rr: Any) -> list[tuple[str, Any]]:
    return [
        ("world/tf/map", rr.TransformAxes3D(axis_length=1.0)),
        ("world/tf/base_link", rr.TransformAxes3D(axis_length=0.3)),
        ("world/tf/base_footprint", rr.TransformAxes3D(axis_length=0.4)),
    ]


def _m20_sim_rerun_blueprint() -> Any:
    import rerun as rr
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Spatial3DView(
            origin="world",
            name="3D",
            background=rrb.Background(kind="SolidColor", color=[0, 0, 0]),
            line_grid=rrb.LineGrid3D(plane=rr.components.Plane3D.XY.with_distance(0.5)),
        ),
        rrb.TimePanel(state="hidden"),
        rrb.SelectionPanel(state="hidden"),
    )


rerun_sim = autoconnect(
    RerunBridgeModule.blueprint(
        blueprint=_m20_sim_rerun_blueprint,
        memory_limit="4GB",
        max_hz={
            "world/global_map": 2.0,
            "world/surface_map": 2.0,
            **({} if not DEBUG_VIZ else {
                "world/danger_cells": 2.0,
                "world/edge_cells": 2.0,
                "world/nodes_nms": 2.0,
                "world/nodes_add": 2.0,
            }),
            "world/node_edges": 2.0,
            "world/path": 0,
        },
        visual_override={
            "world/camera_info": None,
            "world/color_image": None,
            "world/global_map": _render_global_map,
            "world/surface_map": _render_surface,
            **({} if not DEBUG_VIZ else {
                "world/danger_cells": _render_danger_cells,
                "world/edge_cells": _render_edge_cells,
                "world/nodes_nms": _render_nodes_nms,
                "world/nodes_add": _render_nodes_add,
            }),
            "world/node_edges": _render_node_edges,
            "world/path": _render_path,
        },
        static={"world/tf/base_link": _static_scene},
    ),
    RerunWebSocketServer.blueprint(),
    WebsocketVisModule.blueprint(),
)

# ---------------------------------------------------------------------------
# Planner parameters (tuned for M20)
# ---------------------------------------------------------------------------

VOXEL_SIZE = 0.08  # 8 cm voxels

# M20 robot geometry / capabilities
M20_ROBOT_HEIGHT = 0.6  # body + headroom above surface
M20_WALL_CLEARANCE = 0.25  # hard clearance from walls (wider than go2)
M20_WALL_BUFFER = 0.75  # soft standoff zone
M20_WALL_BUFFER_WEIGHT = 100.0  # strong wall repulsion
M20_STEP_THRESHOLD = 0.5  # max traversable step
M20_STEP_PENALTY = 4.0  # climb penalty (prefer flat routes)
M20_SURFACE_CLOSING = 0.3  # hole-filling radius
M20_NODE_SPACING = 1.0  # graph node spacing
M20_MAX_OVERHEAD = 2.0  # ignore surface above this height over robot

# Simulation
FAKE_ROBOT_START_X = -1.0
FAKE_ROBOT_START_Y = -3.5
FAKE_ROBOT_START_YAW = 0.0

# Path follower
FOLLOWER_SPEED = 0.5

# ---------------------------------------------------------------------------
# Assemble
# ---------------------------------------------------------------------------

m20_mls_3Dnav_pcd_sim = autoconnect(
    # --- Visualization ---
    rerun_sim,

    # --- Static global map from PCD ---
    PCDMap.blueprint(pcd_path=PCD_PATH, publish_hz=0.5),

    # --- MLS 3D planner (full rebuild on each global_map message) ---
    # local_map / region_bounds are left unconnected; the planner runs
    # purely on the static global_map.
    MLSPlannerNative.blueprint(
        world_frame="map",
        voxel_size=VOXEL_SIZE,
        robot_height=M20_ROBOT_HEIGHT,
        max_overhead_m=M20_MAX_OVERHEAD,
        surface_closing_radius=M20_SURFACE_CLOSING,
        node_spacing_m=M20_NODE_SPACING,
        wall_clearance_m=M20_WALL_CLEARANCE,
        wall_buffer_m=M20_WALL_BUFFER,
        wall_buffer_weight=M20_WALL_BUFFER_WEIGHT,
        step_threshold_m=M20_STEP_THRESHOLD,
        step_penalty_weight=M20_STEP_PENALTY,
        chord_step_threshold_m=0.0,
        chord_wall_clearance_m=-1.0,
        chord_wall_buffer_weight=-1.0,
        chord_step_penalty_weight=-1.0,
        viz_publish_hz=1.0,
    ).remappings([(MLSPlannerNative, "local_map", "local_map_unused")]),

    # --- Goal relay: odometry -> start_pose, clicked goal -> goal_pose ---
    # Uses planner_odom (body-height z) so the planner's start_pose
    # ground-projection subtracts robot_height back to the correct surface z.
    GoalRelay.blueprint().remappings([(GoalRelay, "odometry", "planner_odom")]),

    # --- Rotate-then-drive P-controller path follower ---
    PControllerFollower.blueprint(
        speed=FOLLOWER_SPEED,
    ).remappings([(PControllerFollower, "odometry", "slam_odom")]),

    # --- Movement manager (muxes nav_cmd_vel + clicked_point -> cmd_vel) ---
    MovementManager.blueprint(),

    # --- Fake robot simulation (closed-loop: cmd_vel -> odometry) ---
    FakeRobotSim3D.blueprint(
        initial_x=FAKE_ROBOT_START_X,
        initial_y=FAKE_ROBOT_START_Y,
        initial_yaw=FAKE_ROBOT_START_YAW,
    ).remappings([
        (FakeRobotSim3D, "slam_odom", "slam_odom"),
        (FakeRobotSim3D, "planner_odom", "planner_odom"),
    ]),

    # --- TF tree: odometry -> map->base_link ---
    M20TF.blueprint().remappings([(M20TF, "odometry", "slam_odom")]),

).global_config(
    n_workers=10,
    robot_model="m20",
    robot_ip="127.0.0.1",
    robot_width=1.1,
)

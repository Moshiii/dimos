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

"""Render one static point-cloud image with grasp proposal annotations."""

from __future__ import annotations

from collections.abc import Sequence
import os
from pathlib import Path
import tempfile
from typing import Any, Protocol

from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.figure import Figure
from matplotlib.lines import Line2D
import numpy as np

from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

DISPLAYED_CANDIDATES = 5


class SweepVolumeLike(Protocol):
    extents_open: tuple[float, float, float]
    offset_open: tuple[float, float, float]
    extents_half_open: tuple[float, float, float]
    offset_half_open: tuple[float, float, float]


def _rotation(q: Any) -> np.ndarray:
    x, y, z, w = (float(q.x), float(q.y), float(q.z), float(q.w))
    return np.asarray(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def _score_colors(scores: np.ndarray) -> np.ndarray:
    if not len(scores):
        return np.empty((0, 3), dtype=float)
    low, high = float(scores.min()), float(scores.max())
    normalized = (
        np.linspace(1.0, 0.0, len(scores)) if high == low else (scores - low) / (high - low)
    )
    from matplotlib import colormaps

    return np.asarray(colormaps["viridis"](normalized))[:, :3]


def _fork_strips_local(gripper: SweepVolumeLike) -> tuple[np.ndarray, ...]:
    open_extents = np.asarray(gripper.extents_open, dtype=float)
    half_extents = np.asarray(gripper.extents_half_open, dtype=float)
    open_offset = np.asarray(gripper.offset_open, dtype=float)
    half_offset = np.asarray(gripper.offset_half_open, dtype=float)

    rear_center = np.asarray([half_offset[0], 0.0, half_offset[2] - half_extents[2] / 2.0])
    mouth_center = np.asarray([open_offset[0], 0.0, open_offset[2] + open_extents[2] / 2.0])
    if mouth_center[2] <= rear_center[2]:
        raise ValueError("configured sweep profiles must open toward increasing local +Z")

    rear_half_width = max(float(half_extents[0]) / 2.0, 1e-6)
    mouth_half_width = max(float(open_extents[0]) / 2.0, rear_half_width)
    rear_left = rear_center + np.asarray([-rear_half_width, 0.0, 0.0])
    rear_right = rear_center + np.asarray([rear_half_width, 0.0, 0.0])
    mouth_left = mouth_center + np.asarray([-mouth_half_width, 0.0, 0.0])
    mouth_right = mouth_center + np.asarray([mouth_half_width, 0.0, 0.0])
    return (
        np.asarray([rear_center, [0.0, 0.0, 0.0]]),
        np.asarray([rear_left, rear_right]),
        np.asarray([rear_left, mouth_left]),
        np.asarray([rear_right, mouth_right]),
    )


def _fork_strips(candidate: Any, gripper: SweepVolumeLike) -> tuple[np.ndarray, ...]:
    return gripper_wireframe_strips(candidate, gripper, np.eye(4, dtype=float))


def gripper_wireframe_strips(
    candidate: Any,
    gripper: SweepVolumeLike,
    grasp_frame_to_tcp: Sequence[Sequence[float]] | np.ndarray,
) -> tuple[np.ndarray, ...]:
    """Convert one TCP proposal into world-frame sweep-volume wireframe strips."""
    p, q = candidate.pose.position, candidate.pose.orientation
    world_to_tcp = np.eye(4, dtype=float)
    world_to_tcp[:3, :3] = _rotation(q)
    world_to_tcp[:3, 3] = np.asarray([p.x, p.y, p.z], dtype=float)
    grasp_to_tcp = np.asarray(grasp_frame_to_tcp, dtype=float)
    if grasp_to_tcp.shape != (4, 4):
        raise ValueError("grasp_frame_to_tcp must have shape (4, 4)")
    world_to_grasp = world_to_tcp @ np.linalg.inv(grasp_to_tcp)
    rotation = world_to_grasp[:3, :3]
    translation = world_to_grasp[:3, 3]
    return tuple((rotation @ strip.T).T + translation for strip in _fork_strips_local(gripper))


def gripper_wireframe_geometry(
    candidate: Any,
    gripper: SweepVolumeLike,
    grasp_frame_to_tcp: Sequence[Sequence[float]] | np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Return indexed world-frame vertices and edges for one grasp proposal."""
    strips = gripper_wireframe_strips(candidate, gripper, grasp_frame_to_tcp)
    vertices = np.vstack(strips).astype(np.float32)
    edges = np.arange(len(vertices), dtype=np.int32).reshape((-1, 2))
    return vertices, edges


def _set_equal_axes(axis: Any, points: np.ndarray) -> None:
    minimum = points.min(axis=0)
    maximum = points.max(axis=0)
    center = (minimum + maximum) / 2.0
    radius = max(float(np.max(maximum - minimum)) / 2.0, 0.05)
    axis.set_xlim(center[0] - radius, center[0] + radius)
    axis.set_ylim(center[1] - radius, center[1] + radius)
    axis.set_zlim(center[2] - radius, center[2] + radius)


def render_grasp_image(
    output_path: Path,
    scene: PointCloud2,
    object_cloud: PointCloud2,
    candidates: GraspCandidateArray,
    gripper: SweepVolumeLike,
    *,
    ranks: Sequence[int] | None = None,
    title: str | None = None,
) -> Path:
    """Write a single PNG containing the scene, target cloud, and top proposals."""
    scene_points, _ = scene.as_numpy()
    object_points, _ = object_cloud.as_numpy()
    top = list(candidates.candidates[:DISPLAYED_CANDIDATES])
    displayed_ranks = list(ranks) if ranks is not None else list(range(1, len(top) + 1))
    if len(displayed_ranks) != len(top):
        raise ValueError("one displayed rank is required for each rendered candidate")
    scores = np.asarray([candidate.score for candidate in top], dtype=float)
    colors = _score_colors(scores)

    figure = Figure(figsize=(10, 8), dpi=150, facecolor="#10171c")
    FigureCanvasAgg(figure)
    axis = figure.add_subplot(111, projection="3d", facecolor="#10171c")
    for pane in (axis.xaxis.pane, axis.yaxis.pane, axis.zaxis.pane):
        pane.set_facecolor("#10171c")
        pane.set_edgecolor("#58707b")
    axis.scatter(
        scene_points[:, 0],
        scene_points[:, 1],
        scene_points[:, 2],
        s=1.0,
        c="#6f8793",
        alpha=0.18,
        depthshade=False,
    )
    axis.scatter(
        object_points[:, 0],
        object_points[:, 1],
        object_points[:, 2],
        s=2.5,
        c="#f5bf1f",
        alpha=0.9,
        depthshade=False,
    )

    annotation_points = [scene_points]
    legend_handles: list[Line2D] = []
    for rank, candidate, color in zip(displayed_ranks, top, colors, strict=True):
        strips = _fork_strips(candidate, gripper)
        annotation_points.extend(strips)
        for strip in strips:
            axis.plot(
                strip[:, 0],
                strip[:, 1],
                strip[:, 2],
                color=color,
                linewidth=2.5,
            )
        legend_handles.append(
            Line2D(
                [0],
                [0],
                color=color,
                linewidth=3,
                label=f"#{rank}  score {candidate.score:.3f}",
            )
        )

    if legend_handles:
        legend = axis.legend(
            handles=legend_handles,
            loc="upper right",
            frameon=True,
            facecolor="#10171c",
            edgecolor="#58707b",
            labelcolor="white",
        )
        legend.get_frame().set_alpha(0.9)

    all_points = np.vstack(annotation_points)
    _set_equal_axes(axis, all_points)
    axis.view_init(elev=24, azim=-58)
    axis.set_xlabel("X (m)", color="#b8c7ce")
    axis.set_ylabel("Y (m)", color="#b8c7ce")
    axis.set_zlabel("Z (m)", color="#b8c7ce")
    axis.tick_params(colors="#90a4ae")
    figure.suptitle(
        title or f"GraspGenX proposals — {len(candidates)} candidates, top {len(top)} shown",
        color="white",
        fontsize=16,
        y=0.98,
    )
    figure.tight_layout(rect=(0.0, 0.0, 1.0, 0.95))

    final_path = output_path.expanduser().resolve()
    final_path.parent.mkdir(parents=True, exist_ok=True)
    temporary: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            dir=final_path.parent,
            prefix=f".{final_path.name}.",
            suffix=".png",
            delete=False,
        ) as handle:
            temporary = Path(handle.name)
        figure.savefig(temporary, format="png", facecolor=figure.get_facecolor())
        temporary.chmod(0o644)
        os.replace(temporary, final_path)
        temporary = None
    finally:
        figure.clear()
        if temporary is not None:
            temporary.unlink(missing_ok=True)
    return final_path

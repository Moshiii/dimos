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

"""A triangle mesh addressed to a scene entity path.

Published by modules that build world geometry (city tiles, nav surfaces).
The message carries plain arrays plus the tf frame its vertices live in, so
any renderer can consume it; the Rerun bridge renders it via ``to_rerun()``.
``op="clear"`` removes the entity subtree instead of setting geometry.
"""

from __future__ import annotations

from io import BytesIO
import struct
import time
from typing import TYPE_CHECKING, Literal

import numpy as np

from dimos.types.timestamped import Timestamped

if TYPE_CHECKING:
    from dimos.visualization.rerun.bridge import RerunMulti

Op = Literal["set", "clear"]

_OPS: tuple[Op, Op] = ("set", "clear")


def _rgba(colors: np.ndarray) -> np.ndarray:
    """Vertex colors canonicalized to (N, 4), so the wire format is fixed."""
    if colors.ndim == 2 and colors.shape[1] == 3:
        alpha = np.full((len(colors), 1), 255, dtype=colors.dtype)
        return np.hstack([colors, alpha])
    return colors


class EntityMesh(Timestamped):
    """Vertices + triangles (+ optional RGBA vertex colors) for one scene entity."""

    msg_name = "visualization_msgs.EntityMesh"

    def __init__(
        self,
        path: str,
        frame_id: str = "",
        vertices: np.ndarray | None = None,
        triangles: np.ndarray | None = None,
        colors: np.ndarray | None = None,
        edges: list[np.ndarray] | None = None,
        edge_color: tuple[int, int, int] = (212, 228, 255),
        edge_radius: float = 0.12,
        op: Op = "set",
        ts: float | None = None,
    ) -> None:
        self.path = path
        self.frame_id = frame_id
        self.op: Op = op
        self.vertices = (
            np.zeros((0, 3), np.float32) if vertices is None else np.asarray(vertices, np.float32)
        )
        self.triangles = (
            np.zeros((0, 3), np.uint32) if triangles is None else np.asarray(triangles, np.uint32)
        )
        self.colors = None if colors is None else _rgba(np.asarray(colors, np.uint8))
        self.edges = [np.asarray(e, np.float32) for e in edges] if edges else []
        self.edge_color = tuple(int(c) for c in edge_color)
        self.edge_radius = float(edge_radius)
        self.ts = ts if ts is not None else time.time()

    @classmethod
    def clear(cls, path: str, ts: float | None = None) -> EntityMesh:
        return cls(path=path, op="clear", ts=ts)

    def __repr__(self) -> str:
        return (
            f"EntityMesh({self.path!r}, op={self.op!r}, frame_id={self.frame_id!r}, "
            f"{len(self.vertices)} vertices, {len(self.triangles)} triangles)"
        )

    # -- serialization --

    def encode(self) -> bytes:
        buf = BytesIO()
        for s in (self.path, self.frame_id):
            raw = s.encode()
            buf.write(struct.pack(">H", len(raw)))
            buf.write(raw)
        buf.write(
            struct.pack(
                ">BdIIB",
                _OPS.index(self.op),
                self.ts,
                len(self.vertices),
                len(self.triangles),
                self.colors is not None,
            )
        )
        buf.write(np.ascontiguousarray(self.vertices, "<f4").tobytes())
        buf.write(np.ascontiguousarray(self.triangles, "<u4").tobytes())
        if self.colors is not None:
            buf.write(np.ascontiguousarray(self.colors, "u1").tobytes())
        # Wireframe edges (appended late; absent in old recordings)
        buf.write(struct.pack(">I", len(self.edges)))
        if self.edges:
            buf.write(struct.pack(">3Bf", *self.edge_color, self.edge_radius))
            for strip in self.edges:
                buf.write(struct.pack(">I", len(strip)))
                buf.write(np.ascontiguousarray(strip, "<f4").tobytes())
        return buf.getvalue()

    @classmethod
    def decode(cls, data: bytes) -> EntityMesh:
        buf = BytesIO(data)
        path, frame_id = (buf.read(struct.unpack(">H", buf.read(2))[0]).decode() for _ in range(2))
        op, ts, n_vertices, n_triangles, has_colors = struct.unpack(">BdIIB", buf.read(18))
        vertices = np.frombuffer(buf.read(n_vertices * 12), "<f4").reshape(n_vertices, 3)
        triangles = np.frombuffer(buf.read(n_triangles * 12), "<u4").reshape(n_triangles, 3)
        colors = (
            np.frombuffer(buf.read(n_vertices * 4), "u1").reshape(n_vertices, 4)
            if has_colors
            else None
        )
        edges: list[np.ndarray] = []
        edge_color = (212, 228, 255)
        edge_radius = 0.12
        head = buf.read(4)
        if len(head) == 4 and (n_edges := struct.unpack(">I", head)[0]):
            r, g, b, edge_radius = struct.unpack(">3Bf", buf.read(7))
            edge_color = (r, g, b)
            for _ in range(n_edges):
                (k,) = struct.unpack(">I", buf.read(4))
                edges.append(np.frombuffer(buf.read(k * 12), "<f4").reshape(k, 3))
        return cls(
            path=path,
            frame_id=frame_id,
            vertices=vertices,
            triangles=triangles,
            colors=colors,
            edges=edges,
            edge_color=edge_color,
            edge_radius=edge_radius,
            op=_OPS[op],
            ts=ts,
        )

    # -- LCM compat (so autoconnect assigns LCMTransport, not pLCM) --

    def lcm_encode(self) -> bytes:
        return self.encode()

    @classmethod
    def lcm_decode(cls, data: bytes, **kwargs: object) -> EntityMesh:
        return cls.decode(data)

    # -- Rerun conversion --

    def to_rerun(self) -> RerunMulti:
        """A ``set`` yields the mesh plus a reparent onto ``tf#/<frame_id>``.

        A theme alpha rides the vertex colors, but the viewer selects its
        translucent pipeline off the *material's* albedo alpha — vertex alpha
        on an opaque material renders solid. Move the alpha to
        ``albedo_factor`` and keep the vertices fully opaque so it applies
        exactly once.
        """
        import rerun as rr

        if self.op == "clear":
            return [(self.path, rr.Clear(recursive=True))]

        colors = self.colors
        albedo_factor = None
        if colors is not None and colors.ndim == 2 and colors.shape[1] == 4:
            albedo_factor = [255, 255, 255, int(colors[:, 3].max())]
            colors = np.column_stack([colors[:, :3], np.full(len(colors), 255, dtype=colors.dtype)])
        out: RerunMulti = [
            (
                self.path,
                rr.Mesh3D(
                    vertex_positions=self.vertices,
                    triangle_indices=self.triangles,
                    vertex_colors=colors,
                    albedo_factor=albedo_factor,
                ),
            )
        ]
        if self.edges:
            out.append(
                (
                    f"{self.path}/edges",
                    rr.LineStrips3D(self.edges, colors=[self.edge_color], radii=[self.edge_radius]),
                )
            )
        if self.frame_id:
            out.append((self.path, rr.Transform3D(parent_frame=f"tf#/{self.frame_id}")))
        return out

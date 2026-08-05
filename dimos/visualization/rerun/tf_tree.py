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

"""Axis triads for every frame of a tf tree."""

from __future__ import annotations

from dataclasses import dataclass
import threading
from typing import TYPE_CHECKING, TypeVar

from dimos.memory2.transform import Transformer

if TYPE_CHECKING:
    from collections.abc import Iterable, Iterator

    import rerun as rr
    from rerun._baseclasses import Archetype

    from dimos.memory2.stream import Stream
    from dimos.memory2.type.observation import Observation
    from dimos.msgs.geometry_msgs.Transform import Transform
    from dimos.msgs.tf2_msgs.TFMessage import TFMessage

T = TypeVar("T")

DEFAULT_TF_ROOT = "world/tf"
# Where the transforms are declared. Rerun pins a frame to its declaring entity
# for the life of a recording, so these cannot move with the tree.
DEFAULT_LINKS_ROOT = "tf_links"
DEFAULT_AXIS_LENGTH = 0.5
DEFAULT_TIMELINE = "ts"
# Each level's triad relative to its parent's.
DEPTH_SCALE = 0.8
# Rerun's own TransformAxes3D draws at 1.0.
AXIS_WIDTH_UI_POINTS = 2.0
AXIS_COLORS = [[255, 0, 0], [0, 255, 0], [0, 0, 255]]


def triad(length: float) -> rr.Arrows3D:
    """XYZ arrows, red green blue."""
    import rerun as rr

    return rr.Arrows3D(
        origins=[[0.0, 0.0, 0.0]] * 3,
        vectors=[[length, 0.0, 0.0], [0.0, length, 0.0], [0.0, 0.0, length]],
        colors=AXIS_COLORS,
        radii=rr.components.Radius.ui_points(AXIS_WIDTH_UI_POINTS),
    )


@dataclass(frozen=True)
class Placement:
    """Where a frame's triad is drawn, and how big."""

    path: str
    depth: int


class TFTreeVis:
    """Draws the tf tree, one nested entity per frame carrying a triad."""

    def __init__(
        self,
        axis_length: float = DEFAULT_AXIS_LENGTH,
        root: str = DEFAULT_TF_ROOT,
        links: str = DEFAULT_LINKS_ROOT,
    ) -> None:
        self.axis_length = axis_length
        self.root = root
        self.links = links
        self._lock = threading.Lock()
        self._parents: dict[str, str] = {}
        self._drawn: dict[str, Placement] = {}
        self._pending = False

    def log(self, msg: TFMessage, archetypes: Iterable[Archetype]) -> None:
        """Declare the transforms, then redraw once a message adds nothing new.

        Publishers split one tree across several messages.
        """
        import rerun as rr

        if not msg.transforms:
            return
        with self._lock:
            for transform, archetype in zip(msg.transforms, archetypes, strict=True):
                child = rr.escape_entity_path_part(transform.child_frame_id)
                rr.log(f"{self.links}/{child}", archetype)
            if self._learn(msg.transforms):
                self._pending = True
            elif self._pending:
                self._pending = False
                self._redraw()

    def flush(self) -> None:
        """Draw a pending change nothing else triggered."""
        with self._lock:
            if self._pending:
                self._pending = False
                self._redraw()

    def placements(self) -> dict[str, Placement]:
        with self._lock:
            return dict(self._drawn)

    def _learn(self, transforms: Iterable[Transform]) -> bool:
        changed = False
        for transform in transforms:
            if self._parents.get(transform.child_frame_id) != transform.frame_id:
                self._parents[transform.child_frame_id] = transform.frame_id
                changed = True
        return changed

    def _layout(self) -> dict[str, Placement]:
        import rerun as rr

        placed: dict[str, Placement] = {}

        def place(frame: str, walked: frozenset[str]) -> Placement:
            known = placed.get(frame)
            if known is not None:
                return known
            parent = self._parents.get(frame)
            if parent is None or parent in walked:
                spot = Placement(f"{self.root}/{rr.escape_entity_path_part(frame)}", 0)
            else:
                above = place(parent, walked | {frame})
                spot = Placement(
                    f"{above.path}/{rr.escape_entity_path_part(frame)}", above.depth + 1
                )
            placed[frame] = spot
            return spot

        for frame in (*self._parents, *self._parents.values()):
            place(frame, frozenset())
        return placed

    def _redraw(self) -> None:
        """Move the tree to match the shape tf has now."""
        import rerun as rr

        layout = self._layout()

        for frame, was in self._drawn.items():
            now = layout.get(frame)
            if now is None or now.path != was.path:
                rr.log(was.path, rr.Arrows3D(origins=[], vectors=[]), static=True)

        for frame, spot in layout.items():
            if self._drawn.get(frame) != spot:
                rr.log(
                    spot.path,
                    rr.CoordinateFrame(f"tf#/{frame}"),
                    triad(self.axis_length * DEPTH_SCALE**spot.depth),
                    static=True,
                )

        self._drawn = layout


class RerunTFTree(Transformer[T, T]):
    """Logs a recorded tf stream in step with the stream it passes through."""

    def __init__(self, tf: Stream[TFMessage]) -> None:
        self._tf = tf
        self._vis = TFTreeVis()

    def __call__(self, upstream: Iterator[Observation[T]]) -> Iterator[Observation[T]]:
        import rerun as rr

        pending = iter(self._tf)
        head = next(pending, None)
        floor: float | None = None
        try:
            for obs in upstream:
                if floor is None:
                    # tf older than the replay would otherwise all land on frame one.
                    floor = obs.ts
                while head is not None and head.ts <= obs.ts:
                    if head.ts >= floor:
                        self._log(head)
                    head = next(pending, None)
                rr.set_time(DEFAULT_TIMELINE, timestamp=obs.ts)
                yield obs
        finally:
            self._vis.flush()

    def _log(self, tf_obs: Observation[TFMessage]) -> None:
        import rerun as rr

        rr.set_time(DEFAULT_TIMELINE, timestamp=tf_obs.ts)
        self._vis.log(tf_obs.data, [archetype for _, archetype in tf_obs.data.to_rerun()])

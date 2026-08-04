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

"""Labeled axis triads for every frame of a tf tree."""

from __future__ import annotations

from typing import TYPE_CHECKING, TypeVar

from dimos.memory2.transform import Transformer
from dimos.protocol.tf.tf import MultiTBuffer
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from collections.abc import Iterable, Iterator

    import rerun as rr

    from dimos.memory2.stream import Stream
    from dimos.memory2.type.observation import Observation
    from dimos.msgs.geometry_msgs.Transform import Transform
    from dimos.msgs.tf2_msgs.TFMessage import TFMessage

T = TypeVar("T")

logger = setup_logger()

DEFAULT_TF_ROOT = "world/tf"
DEFAULT_AXIS_LENGTH = 0.5
DEFAULT_TIMELINE = "ts"
# Seconds of tf to collect before handing out entity paths. Static mount trees
# publish at 5 Hz, so this covers several full cycles.
SETTLE_SECONDS = 1.0
# Each level's triad relative to its parent's, so deeper frames read as smaller.
DEPTH_SCALE = 0.8
# Arrow width in UI points. Rerun's own TransformAxes3D draws at 1.0.
AXIS_WIDTH_UI_POINTS = 2.0
AXIS_COLORS = [[255, 0, 0], [0, 255, 0], [0, 0, 255]]


def triad(length: float) -> rr.Arrows3D:
    """XYZ arrows for one frame, red/green/blue for x/y/z.

    Drawn by hand rather than with ``TransformAxes3D`` because that archetype
    fixes its own width and carries a frame label that cannot be styled.
    """
    import rerun as rr

    return rr.Arrows3D(
        origins=[[0.0, 0.0, 0.0]] * 3,
        vectors=[[length, 0.0, 0.0], [0.0, length, 0.0], [0.0, 0.0, length]],
        colors=AXIS_COLORS,
        radii=rr.components.Radius.ui_points(AXIS_WIDTH_UI_POINTS),
    )


class TFTreeVis:
    """Draws each tf frame as a labeled triad, nested by entity path.

    Placement still comes from the tf graph: every ``Transform3D`` keeps its
    explicit ``tf#/parent`` and ``tf#/child`` frames, so anything attached to a
    named frame is unaffected. The entity path only mirrors the tree
    (``world/tf/odom/base_link/mid360_link``) so the viewer's entity panel shows
    its shape.

    The rerun bridge drives this off live tf. To replay a recorded stream, use
    :class:`RerunTFTree` rather than driving it by hand.
    """

    def __init__(
        self,
        buffer: MultiTBuffer | None = None,
        axis_length: float = DEFAULT_AXIS_LENGTH,
        root: str = DEFAULT_TF_ROOT,
        settle: float = SETTLE_SECONDS,
    ) -> None:
        self.buffer = buffer if buffer is not None else MultiTBuffer()
        self.axis_length = axis_length
        self.root = root
        self.settle = settle
        self._paths: dict[str, str] = {}
        self._depths: dict[str, int] = {}
        self._parents: dict[str, str | None] = {}
        self._axes_logged: set[str] = set()
        self._reparented: set[str] = set()
        self._settle_deadline: float | None = None
        self._flushed = False

    def log(self, msg: TFMessage) -> None:
        """Feed a tf message into the buffer, then log its transforms."""
        if not msg.transforms:
            return
        self.buffer.receive_tfmessage(msg)
        if not self._settled(msg.transforms):
            return
        transforms = msg.transforms
        if self.settle > 0 and not self._flushed:
            # An edge published only while the tree settled, like a root sent
            # twice at startup, would otherwise never be drawn.
            transforms = self.buffer.latest_transforms()
        self._flushed = True
        self._log_transforms(transforms)

    def _settled(self, transforms: Iterable[Transform]) -> bool:
        """Whether the tree has had time to fill in.

        A path is frozen the first time its frame is seen, so a frame that gets
        its path before its own parent arrives stays a root for the session.
        Publishers put a full tree on the wire within a few messages, and the tf
        that falls in this window is republished right after it.
        """
        if self.settle <= 0:
            return True
        latest = max(transform.ts for transform in transforms)
        if self._settle_deadline is None:
            self._settle_deadline = latest + self.settle
        return latest >= self._settle_deadline

    def _log_transforms(self, transforms: Iterable[Transform]) -> None:
        import rerun as rr

        for transform in transforms:
            parent_path = self.path(transform.frame_id)
            child_path = self.path(transform.child_frame_id)
            self._warn_on_reparent(transform)
            self._log_axes(transform.frame_id, parent_path)
            rr.log(child_path, transform.to_rerun())
            self._log_axes(transform.child_frame_id, child_path)

    def frame_paths(self) -> dict[str, str]:
        """Entity path assigned to each frame seen so far."""
        return dict(self._paths)

    def path(self, frame: str) -> str:
        """Entity path of a frame, assigned the first time the frame is seen.

        Rerun forbids a child frame's declaring entity from changing over time,
        and tf trees re-root late, so a path never moves once handed out.
        """
        import rerun as rr

        known = self._paths.get(frame)
        if known is not None:
            return known

        chain: list[str] = []
        base = self.root
        depth = 0
        node: str | None = frame
        visited: set[str] = set()
        while node is not None and node not in visited:
            visited.add(node)
            if node in self._paths:
                base = self._paths[node]
                depth = self._depths[node] + 1
                break
            chain.append(node)
            node = self.buffer.get_parent(node)

        # Whatever the walk stopped on is the parent of the top of the chain.
        parent = node
        for name in reversed(chain):
            base = f"{base}/{rr.escape_entity_path_part(name)}"
            self._paths[name] = base
            self._depths[name] = depth
            self._parents[name] = parent
            parent = name
            depth += 1

        return self._paths[frame]

    def _warn_on_reparent(self, transform: Transform) -> None:
        child = transform.child_frame_id
        if self._parents.get(child) == transform.frame_id or child in self._reparented:
            return
        self._reparented.add(child)
        logger.warning(
            "tf frame re-parented after its entity path was assigned, panel nesting is stale",
            frame=child,
            new_parent=transform.frame_id,
            entity_path=self._paths[child],
        )

    def _log_axes(self, frame: str, path: str) -> None:
        import rerun as rr

        if path in self._axes_logged:
            return
        self._axes_logged.add(path)
        if self.buffer.get_parent(frame) is None:
            # A root is never a child_frame_id, so nothing else declares it.
            rr.log(path, rr.Transform3D(child_frame=f"tf#/{frame}"))
        rr.log(
            path,
            # Without this the arrows sit in the entity path's implicit frame,
            # which is pinned to the parent path and never moves.
            rr.CoordinateFrame(f"tf#/{frame}"),
            triad(self.axis_length * DEPTH_SCALE ** self._depths[frame]),
            static=True,
        )


class RerunTFTree(Transformer[T, T]):
    """Draw the tf tree's triads in step with the stream it passes through.

    Drop it into a replay pipeline and every tf frame gets its labeled triad,
    each logged at its own place on the timeline rather than in one lump up
    front::

        pipeline = lidar.transform(RerunTFTree(store.stream("tf", TFMessage)))

    Window the tf stream the same way as the pipeline, or the tf that predates
    the first observation all lands on that first frame.
    """

    def __init__(
        self,
        tf: Stream[TFMessage],
        axis_length: float = DEFAULT_AXIS_LENGTH,
        timeline: str = DEFAULT_TIMELINE,
        root: str = DEFAULT_TF_ROOT,
    ) -> None:
        self._tf = tf
        self._timeline = timeline
        self._vis = TFTreeVis(axis_length=axis_length, root=root, settle=0.0)

    @property
    def vis(self) -> TFTreeVis:
        return self._vis

    def __call__(self, upstream: Iterator[Observation[T]]) -> Iterator[Observation[T]]:
        import rerun as rr

        # Topology first, so no frame is given a path before its parent is known.
        for tf_obs in self._tf:
            self._vis.buffer.receive_tfmessage(tf_obs.data)

        pending = iter(self._tf)
        head = next(pending, None)
        for obs in upstream:
            while head is not None and head.ts <= obs.ts:
                rr.set_time(self._timeline, timestamp=head.ts)
                self._vis.log(head.data)
                head = next(pending, None)
            rr.set_time(self._timeline, timestamp=obs.ts)
            yield obs

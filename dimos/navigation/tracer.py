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

from __future__ import annotations

import math
from typing import TYPE_CHECKING, Any, get_args, get_origin, get_type_hints

from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.memory2.transform import Transformer
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.nav_msgs.Path import Path
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from collections.abc import Iterator

    from dimos.memory2.type.observation import Observation

logger = setup_logger()

TRACEABLE = (Odometry, PoseStamped, PointStamped)

Traceable = Odometry | PoseStamped | PointStamped


def _to_pose_stamped(msg: Traceable) -> PoseStamped:
    if isinstance(msg, PoseStamped):
        return msg
    if isinstance(msg, PointStamped):
        return msg.to_pose_stamped()
    pose = msg.pose.pose
    return PoseStamped(
        ts=msg.ts, frame_id=msg.frame_id, position=pose.position, orientation=pose.orientation
    )


class Trace:
    """Accumulates positions into a breadcrumb ``Path``."""

    def __init__(self, resolution: float = 0.25) -> None:
        self._resolution = resolution
        self._path: Path | None = None

    def push(self, msg: Traceable) -> Path | None:
        """Add a position; returns the grown ``Path``, or None if it moved < resolution."""
        pose = _to_pose_stamped(msg)
        if self._path is None:
            self._path = Path(ts=pose.ts, frame_id=pose.frame_id or "world", poses=[pose])
        else:
            last = self._path.poses[-1]
            if math.dist((last.x, last.y, last.z), (pose.x, pose.y, pose.z)) < self._resolution:
                return None
            self._path = self._path.push(pose)
            self._path.ts = pose.ts
        return self._path


class TraceTransformer(Transformer[Traceable, Path]):
    """memory2 counterpart of :class:`Tracer`: position observations -> growing Path."""

    def __init__(self, resolution: float = 0.25) -> None:
        self._resolution = resolution

    def __call__(self, upstream: Iterator[Observation[Traceable]]) -> Iterator[Observation[Path]]:
        trace = Trace(self._resolution)
        for obs in upstream:
            path = trace.push(obs.data)
            if path is not None:
                yield obs.derive(data=path)


class TracerConfig(ModuleConfig):
    # Minimum distance between consecutive path points, meters.
    resolution: float = 0.25


class Tracer(Module):
    """Trace position streams into breadcrumb ``Path``\\ s.

    Subclass with the streams you want traced::

        class MyTracer(Tracer):
            odometry: In[Odometry]
            clicked: In[PointStamped]

    Inputs may be ``Odometry``, ``PoseStamped`` or ``PointStamped``. Each
    ``In`` port gets a matching ``<name>_path: Out[Path]`` (here
    ``odometry_path``, ``clicked_path``) that republishes the accumulated
    path, appending a point only when the source has moved at least
    ``config.resolution`` from the last traced point.
    """

    config: TracerConfig

    def __init_subclass__(cls, **kwargs: Any) -> None:
        try:
            hints = get_type_hints(cls, include_extras=True)
        except (NameError, AttributeError, TypeError):
            hints = {}

        anns = cls.__dict__.get("__annotations__")
        if anns is None:
            anns = {}
            cls.__annotations__ = anns

        for name, ann in hints.items():
            if get_origin(ann) is not In:
                continue
            inner, *_ = get_args(ann) or (Any,)
            if not (isinstance(inner, type) and issubclass(inner, TRACEABLE)):
                raise TypeError(
                    f"{cls.__name__}.{name}: Tracer inputs must be Odometry, "
                    f"PoseStamped or PointStamped, got {inner!r}"
                )
            anns.setdefault(f"{name}_path", Out[Path])

        super().__init_subclass__(**kwargs)

    @rpc
    def start(self) -> None:
        super().start()
        if not self.inputs:
            logger.warning("Tracer has no In ports — nothing to trace, subclass the Tracer")
        for name, port in self.inputs.items():
            self._trace(port, self.outputs[f"{name}_path"])
            logger.info(
                "Tracing %s -> %s_path (resolution %.2fm)", name, name, self.config.resolution
            )

    def _trace(self, port: In[Any], out: Out[Path]) -> None:
        trace = Trace(self.config.resolution)

        def on_msg(msg: Traceable) -> None:
            path = trace.push(msg)
            if path is not None:
                out.publish(path)

        self.register_disposable(Disposable(port.subscribe(on_msg)))

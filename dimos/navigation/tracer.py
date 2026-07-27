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
from typing import Any, get_args, get_origin, get_type_hints

from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.nav_msgs.Path import Path
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

TRACEABLE = (Odometry, PoseStamped, PointStamped)


def _to_pose_stamped(msg: Odometry | PoseStamped | PointStamped) -> PoseStamped:
    if isinstance(msg, PoseStamped):
        return msg
    if isinstance(msg, PointStamped):
        return msg.to_pose_stamped()
    pose = msg.pose.pose
    return PoseStamped(
        ts=msg.ts, frame_id=msg.frame_id, position=pose.position, orientation=pose.orientation
    )


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
        path: Path | None = None

        def on_msg(msg: Odometry | PoseStamped | PointStamped) -> None:
            nonlocal path
            pose = _to_pose_stamped(msg)
            if path is None:
                path = Path(ts=pose.ts, frame_id=pose.frame_id or "world", poses=[pose])
            else:
                last = path.poses[-1]
                if (
                    math.dist((last.x, last.y, last.z), (pose.x, pose.y, pose.z))
                    < self.config.resolution
                ):
                    return
                path = path.push(pose)
                path.ts = pose.ts
            out.publish(path)

        self.register_disposable(Disposable(port.subscribe(on_msg)))

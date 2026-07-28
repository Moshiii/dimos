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

"""Render any memory2 store into rerun.

Generic: walks the store's streams and logs every observation whose payload
implements ``to_rerun()`` (the :class:`RerunConvertible` convention). Streams
whose payload has no ``to_rerun`` are skipped. Each stream becomes an entity
path; observations share one ``time`` timeline (relative to the store's earliest
observation, so streams stay aligned). Writes a ``.rrd`` and opens the viewer.
"""

from __future__ import annotations

from pathlib import Path
import shutil
import subprocess
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from dimos.memory2.store.base import Store


def _open_viewer(rrd: str) -> None:
    exe = shutil.which("rerun")
    if exe:
        subprocess.Popen([exe, rrd])
        print(f"  opening {rrd} in rerun")
    else:
        print(f"  rerun viewer not found on PATH; open manually:\n    rerun {rrd}")


def render_store(
    store: Store,
    *,
    out: str | None = None,
    seconds: float | None = None,
    no_gui: bool = False,
    root: str | None = None,
) -> str:
    """Render ``store`` to a ``.rrd`` and (unless ``no_gui``) open the rerun viewer.

    Logs every observation (full res); ``seconds`` bounds the time window from
    the start. ``root`` nests every stream under that entity path
    (``<root>/<name>``) — except a stream whose name matches ``root``'s last
    segment, which stays at ``<root>`` itself. Returns the ``.rrd`` path.
    """
    import rerun as rr

    from dimos.memory2.utils.progress import progress
    from dimos.visualization.rerun.init import rerun_init

    if out is None:
        src = getattr(store.config, "path", None) or "store"
        out = str(Path(src).with_suffix(".rrd"))

    base = root.strip("/") if root else ""

    def entity(name: str) -> str:
        # <root>/<name>, but a stream named like root's last segment stays at <root>.
        if not base:
            return name
        return base if name == base.rsplit("/", 1)[-1] else f"{base}/{name}"

    from dimos.msgs.foxglove_msgs.CompressedVideo import CompressedVideo
    from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
    from dimos.msgs.sensor_msgs.Image import Image
    from dimos.msgs.tf2_msgs.TFMessage import TFMessage

    # Discover renderable streams (payload has a working to_rerun) + shared anchor.
    renderable = []
    t0: float | None = None
    image_entity: dict[str, str] = {}  # frame_id -> entity of an image/video stream
    for name in store.list_streams():
        stream = store.streams[name]
        try:
            first = stream.first()
        except LookupError:
            continue
        data = first.data
        if not hasattr(data, "to_rerun"):
            print(f"  skip {name}: {type(data).__name__} has no to_rerun()")
            continue
        try:
            data.to_rerun()
        except Exception as e:
            print(f"  skip {name}: to_rerun() failed ({e})")
            continue
        if isinstance(data, (Image, CompressedVideo)) and data.frame_id:
            image_entity[data.frame_id] = entity(name)
        renderable.append((name, stream))
        t0 = first.ts if t0 is None else min(t0, first.ts)

    # A Pinhole only projects its own entity and children, so log a CameraInfo
    # stream onto the image/video entity sharing its frame_id.
    redirect = {
        name: image_entity[stream.first().data.frame_id]
        for name, stream in renderable
        if isinstance(stream.first().data, CameraInfo)
        and stream.first().data.frame_id in image_entity
    }

    if t0 is None:
        print("nothing renderable in this store")
        return out

    # Fuse the recorded tf tree into the scene: frames only resolve if connected
    # to the render root's frame (tf#/), so hang each tf root frame (a parent
    # that is never a child) off it with a static identity edge.
    parents: set[str] = set()
    tf_children: set[str] = set()
    for _, stream in renderable:
        if isinstance(stream.first().data, TFMessage):
            for obs in stream:
                for t in obs.data:
                    parents.add(t.frame_id)
                    tf_children.add(t.child_frame_id)

    rerun_init("dimos mem rerun")
    rr.save(out)

    for tf_root in sorted(parents - tf_children):
        rr.log(
            f"tf_root/{tf_root}",
            rr.Transform3D(parent_frame="tf#/", child_frame=f"tf#/{tf_root}"),
            static=True,
        )

    for name, stream in renderable:
        with progress(stream.count(), label=name) as report:
            for obs in stream:
                if seconds is not None and obs.ts - t0 > seconds:
                    break  # the context manager finalizes the windowed (sub-100%) bar
                if obs.data is None:  # e.g. a truncated/corrupt frame that failed to decode
                    report(obs)
                    continue
                rr.set_time("time", duration=obs.ts - t0)
                data = obs.data.to_rerun()
                path = redirect.get(name) or entity(name)
                if isinstance(data, list):  # RerunMulti: [(subpath, archetype), ...]
                    for sub, arch in data:
                        rr.log(f"{path}/{sub}", arch)
                else:
                    rr.log(path, data)
                report(obs)

    rr.rerun_shutdown()  # flush + close the .rrd before opening it
    print(f"wrote {out}")
    if not no_gui:
        _open_viewer(out)
    return out

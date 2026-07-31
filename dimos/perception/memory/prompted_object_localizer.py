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

"""Prompted object localization over a Memory2 RGB-D recording."""

from __future__ import annotations

from collections.abc import Callable
from pathlib import Path
from typing import Any, ClassVar, Protocol

import numpy as np
from pydantic import Field

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.memory2.embed import EmbedImages
from dimos.memory2.store.sqlite import SqliteStore
from dimos.memory2.tf import StreamTF
from dimos.memory2.transform import throttle
from dimos.models.embedding.clip import CLIPModel
from dimos.models.segmentation.edge_tam import EdgeTAMImageSegmenter
from dimos.models.vl.moondream import MoondreamVlModel
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.perception.detection.project import ProjectDepthTo3D, sees
from dimos.perception.detection.type.detection3d.pointcloud import Detection3DPC
from dimos.spec.utils import Spec
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

DEFAULT_OPTICAL_FRAME = "camera_color_optical_frame"
DEFAULT_LOOKBACK_SECONDS = 30.0
_MAX_MATCHES = 12
_MAX_VERIFY_FRAMES = 32
_SPEED_MAX = 0.02


class ActiveRecordingSpec(Spec, Protocol):
    """RPC boundary for a module that owns an active recording."""

    @rpc
    def recording_path(self) -> str: ...


def latest_recording_window(
    color_stream: Any,
    lookback_seconds: float = DEFAULT_LOOKBACK_SECONDS,
) -> tuple[float, float]:
    """Return the latest bounded interval available from a color stream."""
    lo, hi = color_stream.get_time_range()
    return max(lo, hi - lookback_seconds), hi


def strongest_detection(detections_3d: Any) -> Detection3DPC | None:
    """Select the reconstructed detection containing the most 3D points."""
    return max(
        (detection for observation in detections_3d for detection in observation.data),
        key=lambda detection: len(detection.pointcloud),
        default=None,
    )


class PromptedObjectLocalizationRuntime:
    """One prompted localization run plus debug streams derived during it."""

    def __init__(
        self,
        store: SqliteStore,
        *,
        optical_frame: str = DEFAULT_OPTICAL_FRAME,
        report: Callable[[str], None] | None = None,
    ) -> None:
        self.store = store
        self.optical_frame = optical_frame
        self.report = report or (lambda _message: None)
        tf = StreamTF.from_store(store)
        if tf is None:
            raise RuntimeError("recording does not contain a TF stream")
        self.tf: StreamTF = tf
        self.camera_info = store.streams.camera_info.first().data
        self.images: Any = None
        self.embedded: Any = None
        self.matches: Any = None
        self.detections: Any = None
        self.detections_3d: Any = None
        self.observing: Any = None
        self.verified_3d: Any = None
        self._moondream: MoondreamVlModel | None = None
        self._segmenter: EdgeTAMImageSegmenter | None = None
        self._project: ProjectDepthTo3D | None = None
        self._query = ""

    def close(self) -> None:
        """Release localization model resources."""
        if self._segmenter is not None:
            self._segmenter.stop()  # type: ignore[attr-defined]
            self._segmenter = None
        if self._moondream is not None:
            self._moondream.stop()
            self._moondream = None

    def __enter__(self) -> PromptedObjectLocalizationRuntime:
        return self

    def __exit__(self, *_args: Any) -> None:
        self.close()

    def camera_pose(self, timestamp: float) -> Any:
        """Return the world pose of the wrist-mounted optical frame."""
        return (-self.world_to_optical(timestamp)).to_pose()

    def world_to_optical(self, timestamp: float) -> Any:
        """Return the recording transform or fail when TF is unavailable."""
        transform = self.tf.get(self.optical_frame, "world", timestamp, 0.5)
        if transform is None:
            raise LookupError(f"camera transform unavailable at {timestamp}")
        return transform

    def camera_speed(self, timestamp: float, dt: float = 0.06) -> float:
        """Estimate linear camera speed around a recording timestamp."""
        before = self.camera_pose(timestamp - dt)
        after = self.camera_pose(timestamp + dt)
        return float((after.position - before.position).magnitude() / (2 * dt))

    def still(self, timestamp: float, envelope: float = 0.15) -> bool:
        """Return whether the wrist remains still throughout a capture envelope."""
        return all(
            self.camera_speed(timestamp + offset) <= _SPEED_MAX
            for offset in (-envelope, 0.0, envelope)
        )

    def depth_at(self, observation: Any) -> Any:
        """Resolve the aligned depth frame nearest a color observation."""
        try:
            nearest = self.store.streams.depth_image.at(observation.ts, 0.05).first()
        except LookupError:
            return None
        return nearest.data if nearest is not None else None

    def _detect(self, frames: Any) -> Any:
        moondream = self._moondream
        segmenter = self._segmenter
        if moondream is None or segmenter is None:
            raise RuntimeError("localization detection models are not initialized")
        return (
            frames.map_data(
                lambda observation: moondream.query_detections(observation.data, self._query)
            )
            .map_data(
                lambda observation: observation.data.filter(
                    lambda detection: detection.bbox_2d_volume() > 3000
                )
            )
            .filter(lambda observation: len(observation.data) > 0)
            .map_data(lambda observation: segmenter.segment(observation.data))
        )

    def localize(
        self,
        query: str,
        start: float,
        end: float,
    ) -> Detection3DPC | None:
        """Run retrieval, prompted detection, segmentation, and depth projection."""
        if end <= start:
            return None
        self._query = query
        self.images = self.store.streams.color_image.after(start).before(end)

        clip = CLIPModel()
        try:
            self.embedded = (
                self.images.transform(throttle(1.0))
                .map(
                    lambda observation: observation.derive(
                        data=observation.data,
                        pose=self.camera_pose(observation.ts),
                    )
                )
                .transform(EmbedImages(clip))
                .materialize()
            )
            speeds = [
                self.camera_speed(observation.ts)
                for observation in self.embedded.after(start).before(end)
            ]
            rejection_rate = (
                float(np.mean([speed > _SPEED_MAX for speed in speeds])) if speeds else 0.0
            )
            self.report(f"motion gate: > {_SPEED_MAX} m/s rejects {rejection_rate:.0%}")
            self.matches = (
                self.embedded.search(clip.embed_text(query), k=self.embedded.count())
                .after(start)
                .before(end)
                .filter(lambda observation: self.still(observation.ts))
                .limit(_MAX_MATCHES)
                .materialize()
            )
        finally:
            clip.stop()

        if self.matches.count() == 0:
            return None

        self._moondream = MoondreamVlModel()
        self._moondream.start()
        self._segmenter = EdgeTAMImageSegmenter()
        self.detections = self._detect(self.matches).materialize()
        self.report(f"{self.detections.count()} frames with 2d detections")
        if self.detections.count() == 0:
            return None

        self._project = ProjectDepthTo3D(
            self.depth_at,
            self.camera_info,
            tf=self.tf,
            optical_frame=self.optical_frame,
            time_tolerance=0.5,
            filters=[],
        )
        self.detections_3d = (
            self.detections.transform(self._project)
            .filter(lambda observation: len(observation.data) > 0)
            .materialize()
        )
        self.report(f"{self.detections_3d.count()} frames with 3d detections")
        return strongest_detection(self.detections_3d)

    def verify_cross_view(
        self,
        best: Detection3DPC,
        *,
        start: float,
        end: float,
    ) -> None:
        """Populate debug-only cross-view streams for the selected detection."""
        if self.images is None or self._project is None:
            raise RuntimeError("localize must run before cross-view verification")
        self.observing = (
            self.images.transform(throttle(0.25))
            .filter(
                sees(
                    best.pose,
                    self.camera_info,
                    tf=self.tf,
                    optical_frame=self.optical_frame,
                    time_tolerance=0.5,
                )
            )
            .filter(lambda observation: self.still(observation.ts))
            .materialize()
        )
        self.report(
            f"{self.observing.count()} frames observing the detection at {best.pose.position}"
        )
        verify_frames = (
            self.observing.after(start).before(end).limit(_MAX_VERIFY_FRAMES).materialize()
        )
        self.verified_3d = (
            self._detect(verify_frames)
            .transform(self._project)
            .filter(lambda observation: len(observation.data) > 0)
            .materialize()
        )
        self.report(f"cross-view: {self.verified_3d.count()} observing frames re-detect in 3d")


class PromptedObjectLocalizerConfig(ModuleConfig):
    """Configuration for prompted localization over an active recording."""

    db_path: str | Path
    lookback_seconds: float = Field(default=DEFAULT_LOOKBACK_SECONDS, gt=0.0)
    optical_frame: str = Field(default=DEFAULT_OPTICAL_FRAME, min_length=1)


class PromptedObjectLocalizerModule(Module):
    """Resolve text prompts to segmented world-frame point clouds."""

    config: PromptedObjectLocalizerConfig  # pyright: ignore[reportIncompatibleVariableOverride]
    dedicated_worker: ClassVar[bool] = True
    recorder: ActiveRecordingSpec | None = None

    def _recording_path(self) -> str:
        if self.config.g.replay:
            return str(self.config.db_path)
        if self.recorder is None:
            raise RuntimeError("live prompted localization requires an active recorder")
        path = self.recorder.recording_path()
        if not path:
            raise RuntimeError("active recorder returned an empty recording path")
        return path

    @rpc
    def localize(self, query: str) -> PointCloud2 | None:
        """Return the strongest segmented object cloud for a text query."""
        if not query.strip():
            return None
        with SqliteStore(path=self._recording_path(), must_exist=True) as store:
            try:
                start, end = latest_recording_window(
                    store.streams.color_image,
                    self.config.lookback_seconds,
                )
            except LookupError:
                return None
            with PromptedObjectLocalizationRuntime(
                store,
                optical_frame=self.config.optical_frame,
                report=logger.info,
            ) as runtime:
                best = runtime.localize(query, start, end)
                return best.pointcloud if best is not None else None

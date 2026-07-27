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

"""SDK-free ZED stereo camera (plain UVC / V4L2, no pyzed).

Every ZED enumerates as a standard UVC webcam that delivers both eyes packed
side-by-side in one frame (left eye in the left half). This module grabs that
combined frame with OpenCV, splits it down the middle, h264-encodes each eye,
and publishes the two streams as ``color_image_left`` / ``color_image_right``
(``CompressedVideo``) — no ZED SDK, no CUDA, no depth.

Encoding happens *here*, in the capture process, on purpose: a raw 2.6 MB
stereo frame at 60 fps is ~300 MB/s, which swamps the pub/sub transport and
throttles the whole recording to single-digit fps. The h264 packets are a few
KB, so the wire and recorder keep up at 60 fps. Decode downstream with
:class:`~dimos.robot.unitree.go2.dds.video.H264Decoder`.

Defaults target the 60 fps HD720 stereo mode (2 x 1280x720 packed as 2560x720).
The requested mode must be one the camera actually supports (see ZED datasheet:
2.2K@15, 1080p@30, 720p@60, VGA@100); UVC silently falls back otherwise, so the
actual negotiated mode is logged at start.
"""

from __future__ import annotations

import configparser
from fractions import Fraction
from pathlib import Path
import threading
import time

import av
import cv2
import numpy as np
from pydantic import Field

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.msgs.foxglove_msgs.CompressedVideo import CompressedVideo
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# Stereolabs .conf section suffix by per-eye (width, height).
_RESOLUTION_SUFFIX = {(2208, 1242): "2K", (1920, 1080): "FHD", (1280, 720): "HD", (672, 376): "VGA"}
# Where the ZED SDK caches factory calibration confs (populated on first open).
_ZED_SETTINGS_DIR = Path("/usr/local/zed/settings")


def _camera_info_from_conf(
    conf_path: Path, side: str, width: int, height: int, frame_id: str
) -> CameraInfo:
    """One eye's raw (unrectified) CameraInfo from a Stereolabs .conf.

    K from fx/fy/cx/cy; D as OpenCV plumb_bob ``[k1, k2, p1, p2, k3]`` (p1/p2
    absent from most ZED confs → 0), matching zed-open-capture's initCalibration.
    """
    suffix = _RESOLUTION_SUFFIX.get((width, height))
    if suffix is None:
        raise ValueError(f"no .conf section for {width}x{height}")
    cp = configparser.ConfigParser()
    cp.read(conf_path)
    s = cp[f"{side}_CAM_{suffix}"]
    fx, fy, cx, cy = (float(s[k]) for k in ("fx", "fy", "cx", "cy"))
    d = [float(s.get(k, "0.0")) for k in ("k1", "k2", "p1", "p2", "k3")]
    return CameraInfo(
        height=height,
        width=width,
        distortion_model="plumb_bob",
        D=d,
        K=[fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0],
        P=[fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0],
        frame_id=frame_id,
    )


def find_zed_device() -> int:
    """The lowest /dev/videoN index whose v4l2 device name contains "ZED"."""
    candidates: list[int] = []
    for node in Path("/sys/class/video4linux").glob("video*"):
        try:
            name = (node / "name").read_text()
        except OSError:
            continue
        if "zed" in name.lower():
            candidates.append(int(node.name.removeprefix("video")))
    if not candidates:
        raise RuntimeError("No ZED UVC device found (no /dev/video* names contain 'ZED')")
    return min(candidates)


class ZedUvcCameraConfig(ModuleConfig):
    camera_index: int | None = None  # /dev/videoN; None = auto-detect by v4l2 name
    # Per-eye resolution; the UVC device is asked for a 2*width x height frame.
    width: int = 1280
    height: int = 720
    fps: float = Field(default=60.0, gt=0.0)
    left_frame_id: str = "zed_left_optical_frame"
    right_frame_id: str = "zed_right_optical_frame"
    # Factory calibration .conf for camera_info; None = auto-find the newest
    # SNxxxx.conf the SDK cached in /usr/local/zed/settings. Camera_info is
    # skipped (with a warning) if no conf is found.
    conf_path: str | None = None
    camera_info_hz: float = Field(default=1.0, gt=0.0)  # republish rate
    # h264 encoding. libx264 (CPU) is the portable default; "h264_nvenc" offloads
    # to the GPU. Keyframe interval bounds how long a decoder waits to sync.
    encoder: str = "libx264"
    bitrate: int = Field(default=4_000_000, gt=0)  # bits/s per eye
    keyframe_interval: int = Field(default=30, gt=0)  # frames between IDRs


def _make_encoder(name: str, width: int, height: int, fps: float, bitrate: int, gop: int):  # type: ignore[no-untyped-def]
    try:
        enc = av.CodecContext.create(name, "w")
    except Exception as exc:  # unknown/unavailable encoder (e.g. nvenc on a non-GPU box)
        if name == "libx264":
            raise
        logger.warning("ZED UVC: encoder %r unavailable (%s) — falling back to libx264", name, exc)
        return _make_encoder("libx264", width, height, fps, bitrate, gop)
    enc.width = width
    enc.height = height
    enc.pix_fmt = "yuv420p"
    enc.bit_rate = bitrate
    enc.gop_size = gop
    enc.framerate = Fraction(round(fps), 1)
    enc.time_base = Fraction(1, 1000)  # ms; pts stamped from capture time
    # zerolatency + no B-frames → one packet per frame, in order, so each packet
    # pairs with the frame just fed in; repeat-headers puts SPS/PPS on every IDR
    # so a fresh decoder can sync mid-stream.
    if name == "libx264":
        enc.options = {
            "tune": "zerolatency",
            "preset": "ultrafast",
            "bf": "0",
            "x264-params": "repeat-headers=1",
        }
    else:
        enc.options = {"bf": "0"}
    return enc


class ZedUvcCamera(Module):
    """Publishes the two ZED eyes as independent h264 ``CompressedVideo`` streams."""

    config: ZedUvcCameraConfig

    color_image_left: Out[CompressedVideo]
    color_image_right: Out[CompressedVideo]
    camera_info_left: Out[CameraInfo]
    camera_info_right: Out[CameraInfo]

    def __init__(self, *args, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__(*args, **kwargs)
        self._capture: cv2.VideoCapture | None = None
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._index = -1
        self._camera_info: tuple[CameraInfo, CameraInfo] | None = None
        self._enc_left: av.CodecContext | None = None
        self._enc_right: av.CodecContext | None = None

    def _resolve_conf_path(self) -> Path | None:
        if self.config.conf_path:
            return Path(self.config.conf_path)
        confs = sorted(_ZED_SETTINGS_DIR.glob("SN*.conf"), key=lambda p: p.stat().st_mtime)
        return confs[-1] if confs else None

    def _load_camera_info(self) -> None:
        conf = self._resolve_conf_path()
        if conf is None or not conf.exists():
            logger.warning(
                "ZED UVC: no factory .conf found (looked in %s) — camera_info will not be published",
                self.config.conf_path or _ZED_SETTINGS_DIR,
            )
            return
        w, h = self.config.width, self.config.height
        self._camera_info = (
            _camera_info_from_conf(conf, "LEFT", w, h, self.config.left_frame_id),
            _camera_info_from_conf(conf, "RIGHT", w, h, self.config.right_frame_id),
        )
        logger.info("ZED UVC: loaded camera_info from %s", conf.name)

    @rpc
    def start(self) -> None:
        super().start()
        if self._thread and self._thread.is_alive():
            return

        self._index = (
            self.config.camera_index if self.config.camera_index is not None else find_zed_device()
        )
        capture = cv2.VideoCapture(self._index)
        if not capture.isOpened():
            raise RuntimeError(f"Failed to open ZED UVC device {self._index}")

        capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.config.width * 2)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config.height)
        capture.set(cv2.CAP_PROP_FPS, self.config.fps)

        actual_w = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = capture.get(cv2.CAP_PROP_FPS)
        logger.info(
            "ZED UVC device %d negotiated %dx%d @ %.0f fps (requested %dx%d @ %.0f)",
            self._index,
            actual_w,
            actual_h,
            actual_fps,
            self.config.width * 2,
            self.config.height,
            self.config.fps,
        )
        if actual_w != self.config.width * 2 or actual_h != self.config.height:
            logger.warning(
                "ZED UVC mode mismatch — splitting the %dx%d frame down the middle anyway",
                actual_w,
                actual_h,
            )

        self._load_camera_info()
        w, h, fps = self.config.width, self.config.height, self.config.fps
        self._enc_left = _make_encoder(
            self.config.encoder, w, h, fps, self.config.bitrate, self.config.keyframe_interval
        )
        self._enc_right = _make_encoder(
            self.config.encoder, w, h, fps, self.config.bitrate, self.config.keyframe_interval
        )
        self._capture = capture
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()

    def _publish_camera_info(self, ts: float) -> None:
        if self._camera_info is None:
            return
        left, right = self._camera_info
        left.ts = right.ts = ts
        self.camera_info_left.publish(left)
        self.camera_info_right.publish(right)

    @staticmethod
    def _encode(enc: av.CodecContext, eye_rgb: np.ndarray, pts: int) -> list[bytes]:
        vf = av.VideoFrame.from_ndarray(eye_rgb, format="rgb24")
        vf.pts = pts
        return [bytes(pkt) for pkt in enc.encode(vf)]

    def _capture_loop(self) -> None:
        assert self._enc_left is not None and self._enc_right is not None
        ci_period = 1.0 / self.config.camera_info_hz
        next_ci = 0.0
        t0 = time.time()
        while self._capture and not self._stop_event.is_set():
            ret, frame = self._capture.read()
            if not ret:
                if self._stop_event.is_set():
                    break
                logger.warning("ZED UVC device %d dropped a frame", self._index)
                continue
            ts = time.time()
            pts = int((ts - t0) * 1000)  # ms, matches encoder time_base
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            half = frame_rgb.shape[1] // 2
            left = np.ascontiguousarray(frame_rgb[:, :half])
            right = np.ascontiguousarray(frame_rgb[:, half:])
            for packet in self._encode(self._enc_left, left, pts):
                self.color_image_left.publish(
                    CompressedVideo(
                        packet, format="h264", frame_id=self.config.left_frame_id, ts=ts
                    )
                )
            for packet in self._encode(self._enc_right, right, pts):
                self.color_image_right.publish(
                    CompressedVideo(
                        packet, format="h264", frame_id=self.config.right_frame_id, ts=ts
                    )
                )
            if ts >= next_ci:
                self._publish_camera_info(ts)
                next_ci = ts + ci_period

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        self._thread = None
        if self._capture:
            self._capture.release()
            self._capture = None
        super().stop()

# Copyright 2025-2026 Dimensional Inc.
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

"""Drives the excitation battery against a live ControlCoordinator, records
segments in-process, and fits+tunes+emits at the end.

Same shape as benchmark.py's Benchmarker: pure pub/sub, own recorder, operator
gate between runs. Reuses autotune's own tested pieces (drive.play_run,
excitation.step_battery, live sinks, runner.autotune_offline) -- this file is
just the live wiring + a segment recorder, which is the one piece that didn't
exist yet.
"""

from __future__ import annotations

from dataclasses import asdict
import json
import math
from pathlib import Path
import pickle
import queue
import threading
import time
from typing import Any, Literal

import numpy as np
from reactivex.disposable import Disposable

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT, STATE_DIR
from dimos.control.autotune.drive import play_run
from dimos.control.autotune.excitation import step_battery
from dimos.control.autotune.live import EpisodeStatusSink, TwistCommandSink, WallClock
from dimos.control.autotune.profile import BatteryConfig, Channel, RobotProfile
from dimos.control.autotune.runner import Segment, autotune_offline
from dimos.control.benchmarking.gate import GATE_QUIT, GATE_SKIP
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.learning.collection.episode_monitor import EpisodeStatus
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.std_msgs.Int8 import Int8
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

DEFAULT_OUT_DIR = STATE_DIR / "autotune"


class SegmentRecorder:
    """Buffers one channel's feedback while a run is active. Velocity-domain
    reads coordinator_joint_state; pose-domain reads a PoseStamped directly --
    it must never differentiate (see profile.py's OdomType docstring)."""

    def __init__(self, odom_type: str, joint_prefix: str) -> None:
        self._odom_type = odom_type
        self._joint_prefix = joint_prefix
        self._channel: str | None = None
        self._amplitude = 0.0
        self._t0 = 0.0
        self._t: list[float] = []
        self._y: list[float] = []

    def start(self, channel: str, amplitude: float, t0: float) -> None:
        self._channel, self._amplitude, self._t0 = channel, amplitude, t0
        self._t, self._y = [], []
        self._origin: tuple[float, float, float] | None = None

    def stop(self) -> Segment:
        y = np.unwrap(self._y) if self._channel == "wz" else np.array(self._y)
        segment = (np.array(self._t), y, self._amplitude)
        self._channel = None
        return segment

    def on_joint_state(self, msg: JointState, now: float) -> None:
        if self._channel is None or self._odom_type != "velocity":
            return
        name = f"{self._joint_prefix}{self._channel}"
        if name in msg.name:
            self._t.append(now - self._t0)
            self._y.append(msg.velocity[msg.name.index(name)])

    def on_pose(self, msg: PoseStamped, now: float) -> None:
        if self._channel is None or self._odom_type != "pose":
            return
        x, y, theta = msg.position.x, msg.position.y, msg.orientation.euler[2]
        if self._channel == "vx":
            if self._origin is None:
                self._origin = (x, y, theta)
            x0, y0, theta0 = self._origin
            # Heading-frame forward distance, not raw world-frame x -- otherwise
            # any heading drift during the step contaminates vx with wz.
            value = (x - x0) * math.cos(theta0) + (y - y0) * math.sin(theta0)
        else:
            value = {"vy": y, "wz": theta}.get(self._channel)
        if value is not None:
            self._t.append(now - self._t0)
            self._y.append(value)


class AutotuneDriverConfig(ModuleConfig):
    robot_id: str
    joint_prefix: str  # e.g. "go2/" for make_twist_base_joints("go2")
    channels: dict[str, float]  # channel name -> vmax
    odom_type: Literal["pose", "velocity"]
    fitter: Literal["pose", "velocity"]
    controller_form: str = "velocity_pi"
    expected_tau_s: float | None = None
    amplitude_fractions: tuple[float, ...] = (0.25, 0.5, 0.75)
    repeats: int = 3
    step_duration_s: float = 4.0
    settle_s: float = 1.0
    tick_hz: float = 20.0
    gate_source: Literal["stream", "auto"] = "stream"
    out_dir: Path | None = None


class AutotuneDriver(Module):
    config: AutotuneDriverConfig

    twist_command: Out[Twist]
    coordinator_joint_state: In[JointState]
    pose: In[PoseStamped]
    operator_command: In[Int8]
    status: Out[EpisodeStatus]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._gate_queue: queue.Queue[int] = queue.Queue()
        self._recorder = SegmentRecorder(self.config.odom_type, self.config.joint_prefix)
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None

    def _build_profile(self) -> RobotProfile:
        cfg = self.config
        return RobotProfile(
            name=cfg.robot_id,
            command_interface="twist",
            odom_type=cfg.odom_type,
            channels=[Channel(name=n, vmax=v) for n, v in cfg.channels.items()],
            fitter=cfg.fitter,
            controller_form=cfg.controller_form,
            command_stream="twist_command",
            feedback_stream="coordinator_joint_state" if cfg.odom_type == "velocity" else "pose",
            expected_tau_s=cfg.expected_tau_s,
            battery=BatteryConfig(amplitude_fractions=cfg.amplitude_fractions, repeats=cfg.repeats),
        )

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(
            Disposable(self.coordinator_joint_state.subscribe(self._on_joint_state))
        )
        self.register_disposable(Disposable(self.pose.subscribe(self._on_pose)))
        if self.config.gate_source == "stream":
            self.register_disposable(
                Disposable(self.operator_command.subscribe(self._on_gate_event))
            )
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, name="autotune_driver", daemon=True)
        self._thread.start()

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        self._gate_queue.put(GATE_QUIT)
        if self._thread is not None:
            self._thread.join(DEFAULT_THREAD_JOIN_TIMEOUT)
        super().stop()

    def _on_joint_state(self, msg: JointState) -> None:
        self._recorder.on_joint_state(msg, time.monotonic())

    def _on_pose(self, msg: PoseStamped) -> None:
        self._recorder.on_pose(msg, time.monotonic())

    def _on_gate_event(self, msg: Int8) -> None:
        self._gate_queue.put(int(msg.data))

    def _drain_gate(self) -> int:
        dropped = 0
        while True:
            try:
                self._gate_queue.get_nowait()
                dropped += 1
            except queue.Empty:
                return dropped

    def _wait_gate(self) -> int:
        stale = self._drain_gate()
        if stale:
            logger.info(f"discarded {stale} stale gate event(s)")
        return self._gate_queue.get()

    def _run(self) -> None:
        cfg = self.config
        profile = self._build_profile()
        logger.info(f"[autotune] profile: {asdict(profile)}")

        runs = step_battery(profile, duration_s=cfg.step_duration_s)
        logger.info(f"[autotune] {len(runs)} excitation run(s) over {profile.channel_names}")

        sink = TwistCommandSink(self.twist_command.publish)
        episodes = EpisodeStatusSink(self.status.publish)
        clock = WallClock()
        segments_by_channel: dict[str, list[Segment]] = {ch: [] for ch in profile.channel_names}

        for i, run in enumerate(runs):
            if self._stop_event.is_set():
                return
            if cfg.gate_source == "stream":
                logger.info(f"[{i + 1}/{len(runs)}] {run.label} -- ENTER to run (K=skip, Backspace=quit)")
                ev = self._wait_gate()
                if self._stop_event.is_set():
                    return
                if ev == GATE_QUIT:
                    logger.info("[autotune] operator quit -- ending session")
                    break
                if ev == GATE_SKIP:
                    logger.info("  skipped")
                    continue
            self._recorder.start(run.channel, run.amplitude, clock.now())
            play_run(run, sink, episodes, clock, tick_hz=cfg.tick_hz)
            segments_by_channel[run.channel].append(self._recorder.stop())
            if cfg.settle_s > 0:
                sink.stop()
                clock.sleep(cfg.settle_s)

        self._emit(profile, segments_by_channel)

    def _emit(self, profile: RobotProfile, segments_by_channel: dict[str, list[Segment]]) -> None:
        cfg = self.config
        missing = [ch for ch in profile.channel_names if not segments_by_channel.get(ch)]
        if missing:
            logger.warning(f"[autotune] no segments for {missing} -- skipping fit")
            return

        outputs = autotune_offline(profile, segments_by_channel, robot_id=cfg.robot_id, sim_or_hw="hw")
        out_dir = (cfg.out_dir or DEFAULT_OUT_DIR / cfg.robot_id).expanduser()
        out_dir.mkdir(parents=True, exist_ok=True)
        (out_dir / "tuned_config.json").write_text(json.dumps(outputs.artifact, indent=2))
        (out_dir / "characterization_report.json").write_text(json.dumps(outputs.report, indent=2))
        # Raw (t, y, amplitude) segments, kept so the fit can be re-run offline
        # (e.g. with different fitter bounds) without re-driving the robot.
        with (out_dir / "segments.pkl").open("wb") as f:
            pickle.dump(segments_by_channel, f)
        logger.info(f"[autotune] wrote {out_dir} (+ segments.pkl)")
        for ch, tuning in outputs.tunings.items():
            logger.info(f"[autotune] {ch}: {tuning}")

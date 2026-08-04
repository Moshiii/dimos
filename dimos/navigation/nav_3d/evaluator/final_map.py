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

"""The evaluator's own map of a recording, and the checkpoints along it.

Not ground truth, just the most complete occupancy the mapper produces.
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import os
from time import perf_counter
from typing import TYPE_CHECKING, Any

import numpy as np

from dimos.constants import CACHE_DIR
from dimos.navigation.nav_3d.evaluator.metrics import timing_stats
from dimos.navigation.nav_3d.evaluator.recording import iter_world_frames
from dimos.navigation.nav_3d.evaluator.voxel_keys import key_centers, keys_contain, voxel_keys
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from collections.abc import Iterable, Iterator
    from pathlib import Path

    from numpy.typing import NDArray

    from dimos.mapping.ray_tracing.voxel_map import VoxelRayMapper
    from dimos.navigation.nav_3d.evaluator.cases import Suite
    from dimos.navigation.nav_3d.evaluator.config import EvalConfig
    from dimos.navigation.nav_3d.evaluator.recording import Frame

logger = setup_logger()


@dataclass
class FinalMap:
    voxel_size: float
    occupied: NDArray[np.float32]
    occupied_keys: NDArray[np.int64]
    frames: int
    add_frame_ms: dict[str, float]
    build_ms: float

    def standable_surface(self, robot_height: float) -> NDArray[np.float32]:
        """Occupied cells with robot_height of free space directly above them.
        Decides where an endpoint may sit, not where a path may go."""
        keys = self.occupied_keys
        blocked = np.zeros(len(keys), dtype=bool)
        for dz in range(1, int(np.ceil(robot_height / self.voxel_size)) + 1):
            blocked |= keys_contain(keys, keys + dz)
        return key_centers(keys[~blocked], self.voxel_size)


@dataclass
class MapCheckpoints:
    """The mapper's occupied set at increasing times, delta-encoded."""

    times: NDArray[np.float64]
    added: list[NDArray[np.int64]]
    removed: list[NDArray[np.int64]]

    def iter_snapshots(self) -> Iterator[NDArray[np.int64]]:
        """Yield the full sorted occupied key set at each time, in order."""
        keys = np.array([], dtype=np.int64)
        for add, rem in zip(self.added, self.removed, strict=True):
            keys = np.delete(keys, np.searchsorted(keys, rem))
            keys = np.insert(keys, np.searchsorted(keys, add), add)
            yield keys


CACHE_VERSION = 3
CHECKPOINT_CACHE_VERSION = 3


def _save_npz(cache: Path, **arrays: Any) -> None:
    """Publish a cache atomically, so an interrupted build cannot poison later runs."""
    cache.parent.mkdir(parents=True, exist_ok=True)
    tmp = cache.with_name(f"{cache.name}.{os.getpid()}.tmp")
    try:
        # Written through a handle because savez appends .npz to a bare path.
        with tmp.open("wb") as fh:
            np.savez_compressed(fh, **arrays)
        os.replace(tmp, cache)
    finally:
        tmp.unlink(missing_ok=True)


CACHE_SUBDIR = CACHE_DIR / "nav3d_eval"


def _cache_path(db_path: Path, params: dict[str, float | int | str]) -> Path:
    digest = hashlib.sha1(json.dumps(params, sort_keys=True).encode()).hexdigest()[:10]
    return CACHE_SUBDIR / f"{db_path.stem}.{digest}.npz"


def _final_params(db_path: Path, suite: Suite, cfg: EvalConfig) -> dict[str, float | int | str]:
    # The recording is part of the key. Without it a re-ingested dataset keeps
    # its stem, and the stale map silently becomes the grading occupancy.
    stat = db_path.stat()
    params: dict[str, float | int | str] = {
        **cfg.mapper_fingerprint(),
        "align_tol": cfg.align_tol,
        "lidar_stream": suite.lidar_stream,
        "odom_stream": suite.odom_stream,
        "cache_version": CACHE_VERSION,
        "db": str(db_path.resolve()),
        "db_size": stat.st_size,
        "db_mtime_ns": stat.st_mtime_ns,
    }
    if suite.end_ts is not None:
        params["end_ts"] = suite.end_ts
    return params


def replay_frames(
    frames: Iterable[Frame],
    mapper: VoxelRayMapper,
    voxel_size: float,
    times: NDArray[np.float64],
) -> tuple[FinalMap, list[NDArray[np.int64]]]:
    """Feed frames through the mapper in order, snapshotting at each requested
    time. A snapshot holds exactly the frames with ts <= its time."""
    snapshots: list[NDArray[np.int64]] = []
    add_ms: list[float] = []
    t0 = perf_counter()
    for frame in frames:
        while len(snapshots) < len(times) and frame.ts > times[len(snapshots)]:
            snapshots.append(np.unique(voxel_keys(mapper.global_map(), voxel_size)))
        t1 = perf_counter()
        mapper.add_frame(frame.points, frame.origin)
        add_ms.append((perf_counter() - t1) * 1000)
    build_ms = (perf_counter() - t0) * 1000
    occupied = mapper.global_map()
    occupied_keys = np.unique(voxel_keys(occupied, voxel_size))
    while len(snapshots) < len(times):
        snapshots.append(occupied_keys)
    final = FinalMap(
        voxel_size=voxel_size,
        occupied=occupied,
        occupied_keys=occupied_keys,
        frames=len(add_ms),
        add_frame_ms=timing_stats(add_ms),
        build_ms=build_ms,
    )
    return final, snapshots


def _save_final(cache: Path, final: FinalMap) -> None:
    _save_npz(
        cache,
        occupied=final.occupied,
        occupied_keys=final.occupied_keys,
        frames=final.frames,
        **{f"add_{k}": v for k, v in final.add_frame_ms.items()},
    )
    logger.info("final map cached", cache=cache.name, voxels=len(final.occupied))


def load_or_build_final_map(db_path: Path, suite: Suite, cfg: EvalConfig) -> FinalMap:
    cache = _cache_path(db_path, _final_params(db_path, suite, cfg))
    if cache.exists():
        data = np.load(cache)
        return FinalMap(
            voxel_size=cfg.voxel_size,
            occupied=data["occupied"],
            occupied_keys=data["occupied_keys"],
            frames=int(data["frames"]),
            add_frame_ms={k: float(data[f"add_{k}"]) for k in ("p50", "p95", "max")},
            build_ms=0.0,
        )

    logger.info("building final map (cache miss)", recording=db_path.name)
    final, _ = replay_frames(
        iter_world_frames(
            db_path, suite.lidar_stream, suite.odom_stream, cfg.align_tol, suite.end_ts_seconds()
        ),
        cfg.make_mapper(),
        cfg.voxel_size,
        np.array([], dtype=np.float64),
    )
    _save_final(cache, final)
    return final


def encode_deltas(
    snapshots: list[NDArray[np.int64]],
) -> tuple[list[NDArray[np.int64]], list[NDArray[np.int64]]]:
    added: list[NDArray[np.int64]] = []
    removed: list[NDArray[np.int64]] = []
    prev = np.array([], dtype=np.int64)
    for keys in snapshots:
        added.append(np.setdiff1d(keys, prev, assume_unique=True))
        removed.append(np.setdiff1d(prev, keys, assume_unique=True))
        prev = keys
    return added, removed


def load_or_build_checkpoints(
    db_path: Path, suite: Suite, cfg: EvalConfig, times: NDArray[np.float64]
) -> MapCheckpoints:
    """Occupied key sets at the requested times, deduped and sorted.

    A cache miss replays the recording once and fills the final cache too.
    """
    times = np.unique(np.asarray(times, dtype=np.float64))
    params: dict[str, float | int | str] = {
        **_final_params(db_path, suite, cfg),
        "kind": "checkpoints",
        "times_sha": hashlib.sha1(times.tobytes()).hexdigest()[:10],
        "checkpoint_version": CHECKPOINT_CACHE_VERSION,
    }
    cache = _cache_path(db_path, params)
    if cache.exists():
        data = np.load(cache)
        n = len(data["times"])
        return MapCheckpoints(
            times=data["times"],
            added=[data[f"add_{i}"] for i in range(n)],
            removed=[data[f"rem_{i}"] for i in range(n)],
        )

    logger.info("building map checkpoints (cache miss)", n=len(times), recording=db_path.name)
    final, snapshots = replay_frames(
        iter_world_frames(
            db_path, suite.lidar_stream, suite.odom_stream, cfg.align_tol, suite.end_ts_seconds()
        ),
        cfg.make_mapper(),
        cfg.voxel_size,
        times,
    )
    final_cache = _cache_path(db_path, _final_params(db_path, suite, cfg))
    if not final_cache.exists():
        _save_final(final_cache, final)
    added, removed = encode_deltas(snapshots)
    arrays: dict[str, NDArray[np.int64] | NDArray[np.float64]] = {"times": times}
    arrays |= {f"add_{i}": a for i, a in enumerate(added)}
    arrays |= {f"rem_{i}": r for i, r in enumerate(removed)}
    _save_npz(cache, **arrays)
    logger.info("checkpoints cached", cache=cache.name)
    return MapCheckpoints(times=times, added=added, removed=removed)

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

"""ATE comparison of TUM trajectories against a reference, with a top-down plot.

Every estimate is associated to the reference by nearest timestamp (within
``--max-dt``), SE3-aligned (Umeyama, no scale), and scored: ATE RMSE / mean /
median / max, plus endpoint error. All estimates go through the identical
procedure, so numbers are directly comparable.

    python -m dimos.hardware.sensors.lidar.fastlivo.scripts.compare_trajectories \
        --ref gtsam_odom.tum \
        --est fastlivo=/tmp/fastlivo_odom.tum --est pointlio=/tmp/pointlio_odom.tum \
        --plot /tmp/comparison.png
"""

from __future__ import annotations

import argparse
from typing import Any

import numpy as np


def load_tum(path: str) -> tuple[np.ndarray, np.ndarray]:
    """Return (timestamps[N], positions[N,3])."""
    rows = np.loadtxt(path, comments="#")
    if rows.ndim == 1:
        rows = rows[None, :]
    return rows[:, 0], rows[:, 1:4]


def associate(
    ref_ts: np.ndarray, est_ts: np.ndarray, max_dt: float
) -> tuple[np.ndarray, np.ndarray]:
    """Nearest-timestamp association; returns (ref_idx, est_idx) pairs."""
    idx = np.searchsorted(ref_ts, est_ts)
    idx = np.clip(idx, 1, len(ref_ts) - 1)
    left = ref_ts[idx - 1]
    right = ref_ts[idx]
    ref_idx = np.where(np.abs(est_ts - left) < np.abs(est_ts - right), idx - 1, idx)
    dt = np.abs(ref_ts[ref_idx] - est_ts)
    keep = dt <= max_dt
    return ref_idx[keep], np.nonzero(keep)[0]


def umeyama_align(src: np.ndarray, dst: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Rigid SE3 (R, t) minimizing ||R @ src + t - dst||, no scale."""
    mu_src = src.mean(axis=0)
    mu_dst = dst.mean(axis=0)
    cov = (dst - mu_dst).T @ (src - mu_src) / len(src)
    u, _, vt = np.linalg.svd(cov)
    s = np.eye(3)
    if np.linalg.det(u) * np.linalg.det(vt) < 0:
        s[2, 2] = -1
    rot = u @ s @ vt
    t = mu_dst - rot @ mu_src
    return rot, t


# Resampling rate for the speed profiles used in time-offset recovery.
_PROFILE_HZ = 10.0


def find_time_offset(
    ref_ts: np.ndarray, ref_p: np.ndarray, est_ts: np.ndarray, est_p: np.ndarray
) -> float:
    """Recover a global time shift (add to est_ts) via speed-profile correlation.

    Needed for estimators that stamp odometry with wall time at replay rather
    than the data timeline (pointlio does this). Speed over ground is
    invariant to the SE3 alignment, so cross-correlating the two speed
    profiles finds the lag without needing poses in a common frame.
    """
    dt = 1.0 / _PROFILE_HZ

    def speed_profile(ts: np.ndarray, p: np.ndarray) -> tuple[float, np.ndarray]:
        grid = np.arange(ts[0], ts[-1], dt)
        interp = np.column_stack([np.interp(grid, ts, p[:, i]) for i in range(3)])
        speed = np.linalg.norm(np.diff(interp, axis=0), axis=1) / dt
        return float(grid[0]), speed

    ref_t0, ref_speed = speed_profile(ref_ts, ref_p)
    est_t0, est_speed = speed_profile(est_ts, est_p)
    ref_c = ref_speed - ref_speed.mean()
    est_c = est_speed - est_speed.mean()
    corr = np.correlate(ref_c, est_c, mode="full")
    lag_samples = int(np.argmax(corr)) - (len(est_c) - 1)
    offset = (ref_t0 - est_t0) + lag_samples * dt
    return float(offset)


def evaluate(
    name: str, ref_ts: np.ndarray, ref_p: np.ndarray, est_path: str, max_dt: float
) -> dict[str, Any] | None:
    est_ts, est_p = load_tum(est_path)
    # Auto-recover the time base when the estimate doesn't overlap the
    # reference timeline (wall-stamped replay output).
    overlap = min(ref_ts[-1], est_ts[-1]) - max(ref_ts[0], est_ts[0])
    time_offset = 0.0
    if overlap < 0.5 * (est_ts[-1] - est_ts[0]):
        time_offset = find_time_offset(ref_ts, ref_p, est_ts, est_p)
        est_ts = est_ts + time_offset
        print(f"{name}: recovered time offset {time_offset:+.3f}s (speed-profile correlation)")
    ref_idx, est_idx = associate(ref_ts, est_ts, max_dt)
    if len(ref_idx) < 10:
        print(f"{name}: only {len(ref_idx)} associations (need >=10) — skipping")
        return None
    ref_m = ref_p[ref_idx]
    est_m = est_p[est_idx]
    rot, t = umeyama_align(est_m, ref_m)
    est_aligned = est_m @ rot.T + t
    err = np.linalg.norm(est_aligned - ref_m, axis=1)
    result = {
        "name": name,
        "pairs": len(err),
        "coverage_s": float(est_ts[est_idx][-1] - est_ts[est_idx][0]),
        "rmse": float(np.sqrt(np.mean(err**2))),
        "mean": float(np.mean(err)),
        "median": float(np.median(err)),
        "max": float(np.max(err)),
        "endpoint": float(err[-1]),
        "aligned": est_aligned,
        "ref_matched": ref_m,
    }
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ref", required=True, help="reference TUM file")
    parser.add_argument(
        "--est", action="append", required=True, help="name=path TUM estimate (repeatable)"
    )
    parser.add_argument("--max-dt", type=float, default=0.05)
    parser.add_argument("--plot", default=None, help="write a top-down comparison PNG here")
    args = parser.parse_args()

    ref_ts, ref_p = load_tum(args.ref)
    order = np.argsort(ref_ts)
    ref_ts, ref_p = ref_ts[order], ref_p[order]

    results = []
    for spec in args.est:
        name, _, path = spec.partition("=")
        r = evaluate(name, ref_ts, ref_p, path, args.max_dt)
        if r is not None:
            results.append(r)

    print(f"\nreference: {args.ref} ({len(ref_ts)} poses, {ref_ts[-1] - ref_ts[0]:.0f}s)")
    header = f"{'estimate':<12} {'pairs':>7} {'cover(s)':>9} {'RMSE':>8} {'mean':>8} {'median':>8} {'max':>8} {'endpoint':>9}"
    print(header)
    for r in results:
        print(
            f"{r['name']:<12} {r['pairs']:>7} {r['coverage_s']:>9.0f} {r['rmse']:>8.3f} "
            f"{r['mean']:>8.3f} {r['median']:>8.3f} {r['max']:>8.3f} {r['endpoint']:>9.3f}"
        )

    if args.plot:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig, ax = plt.subplots(figsize=(12, 12))
        ax.plot(ref_p[:, 0], ref_p[:, 1], "k-", linewidth=1.0, label="reference", alpha=0.7)
        for r in results:
            ax.plot(
                r["aligned"][:, 0],
                r["aligned"][:, 1],
                linewidth=0.9,
                label=f"{r['name']} (RMSE {r['rmse']:.2f}m)",
            )
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        ax.set_title("Top-down trajectory comparison (SE3-aligned to reference)")
        ax.axis("equal")
        ax.grid(True, alpha=0.3)
        ax.legend()
        fig.tight_layout()
        fig.savefig(args.plot, dpi=130)
        print(f"plot written to {args.plot}")


if __name__ == "__main__":
    main()

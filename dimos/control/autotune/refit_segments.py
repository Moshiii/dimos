#!/usr/bin/env python3
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

"""Re-fit a saved AutotuneDriver segments.pkl with different pose-fitter
search bounds, without re-driving the robot.

Diagnostic tool: on Go2 hardware, ``tau``/``L`` landed exactly on the
default DEFAULT_TAU_BOUNDS/DEFAULT_L_BOUNDS ceiling across two independent
runs (fit/pose_fopdt.py). This re-runs the same fit Mustafa's code does,
unmodified, just with wider bounds, to see whether the channel converges to
a sane, non-clipped value or is broken for some other reason (segments.pkl
is only written by AutotuneDriver after the segment-persistence change).

    python -m dimos.control.autotune.refit_segments \\
        ~/.local/state/dimos/autotune/go2/segments.pkl \\
        --l-bounds 0.05,1.0 --tau-bounds 0.03,1.5
"""

from __future__ import annotations

import argparse
import pickle

from dimos.control.autotune.fit.pose_fopdt import estimate_deadtime, fit_pose_fopdt_multi
from dimos.control.autotune.runner import Segment


def refit_channel(
    segments: list[Segment], l_bounds: tuple[float, float], tau_bounds: tuple[float, float]
) -> tuple[float, object]:
    """Same two-step pose fit runner.py::fit_channel does, bounds overridden."""
    biggest = max(segments, key=lambda s: abs(s[2]))
    L, _, _ = estimate_deadtime(
        biggest[0], biggest[1], biggest[2], l_bounds=l_bounds, tau_bounds=tau_bounds
    )
    joint = fit_pose_fopdt_multi(segments, L, tau_bounds=tau_bounds, l_bounds=l_bounds)
    return L, joint


def _pinned(value: float, bounds: tuple[float, float]) -> str:
    return " <- PINNED" if abs(value - bounds[0]) < 1e-6 or abs(value - bounds[1]) < 1e-6 else ""


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("segments_pkl")
    ap.add_argument("--l-bounds", default="0.05,1.0", help="min,max seconds")
    ap.add_argument("--tau-bounds", default="0.03,1.5", help="min,max seconds")
    args = ap.parse_args()

    l_bounds = tuple(float(x) for x in args.l_bounds.split(","))
    tau_bounds = tuple(float(x) for x in args.tau_bounds.split(","))

    with open(args.segments_pkl, "rb") as f:
        segments_by_channel = pickle.load(f)

    print(f"l_bounds={l_bounds}  tau_bounds={tau_bounds}\n")
    for channel, segments in segments_by_channel.items():
        if not segments:
            print(f"{channel}: no segments")
            continue
        L, joint = refit_channel(segments, l_bounds, tau_bounds)
        print(
            f"{channel}: K={joint.K:.4f}  tau={joint.tau:.4f}{_pinned(joint.tau, tau_bounds)}  "
            f"L={L:.4f}{_pinned(L, l_bounds)}  r2={joint.r_squared:.3f}  valid={joint.valid}"
        )


if __name__ == "__main__":
    main()

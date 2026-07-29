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

"""Scan a saved segments.pkl for yaw-wrap discontinuities.

Diagnostic for the hypothesis that wz's bad fit (r2~0.43, K near zero or
negative) is caused by orientation.euler[2] wrapping at +-pi mid-run rather
than a real plant problem. A genuine FOPDT step response changes smoothly;
a sample-to-sample jump near +-2*pi (~6.28 rad) has no physical explanation
other than the angle representation wrapping around.

    python -m dimos.control.autotune.check_yaw_wrap \\
        ~/.local/state/dimos/autotune/go2/segments.pkl
"""

from __future__ import annotations

import argparse
import pickle

import numpy as np

# A real per-tick change should be well under a radian at these sample rates;
# only a wrap explains a jump this close to 2*pi.
WRAP_THRESHOLD = np.pi


def scan_segment(t: np.ndarray, y: np.ndarray) -> list[tuple[float, float]]:
    """Returns (time, jump_size) for every suspiciously large sample-to-sample
    change in ``y``."""
    if len(y) < 2:
        return []
    jumps = np.diff(y)
    hits = np.flatnonzero(np.abs(jumps) > WRAP_THRESHOLD)
    return [(float(t[i + 1]), float(jumps[i])) for i in hits]


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("segments_pkl")
    args = ap.parse_args()

    with open(args.segments_pkl, "rb") as f:
        segments_by_channel = pickle.load(f)

    for channel, segments in segments_by_channel.items():
        print(f"\n{channel}: {len(segments)} segment(s)")
        any_wrap = False
        for i, (t, y, amp) in enumerate(segments):
            hits = scan_segment(t, y)
            if hits:
                any_wrap = True
                for time_s, jump in hits:
                    print(f"  seg[{i}] amp={amp:+.3f}  t={time_s:.2f}s  jump={jump:+.3f} rad  <- WRAP")
            else:
                span = float(y.max() - y.min()) if len(y) else 0.0
                print(f"  seg[{i}] amp={amp:+.3f}  no jump  (range covered: {span:.3f})")
        if not any_wrap:
            print(f"  -> no wraps detected on {channel}")


if __name__ == "__main__":
    main()

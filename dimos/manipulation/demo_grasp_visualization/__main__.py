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

"""Visualize the banana point cloud and GraspGenX proposals in Viser."""

import argparse
from collections.abc import Sequence
from pathlib import Path

from .demo import DEFAULT_MAX_CANDIDATES, run_contributor_demo


def _positive_int(value: str) -> int:
    parsed = int(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("must be positive")
    return parsed


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--max-candidates",
        type=_positive_int,
        default=DEFAULT_MAX_CANDIDATES,
        help="Maximum score-ranked grasp wireframes to display.",
    )
    parser.add_argument(
        "--object-cloud",
        type=Path,
        help="Segmented object point cloud in PLY or PCD format. Defaults to the banana fixture.",
    )
    parser.add_argument(
        "--frame-id",
        default="world",
        help="Coordinate frame of --object-cloud (default: world).",
    )
    return parser


def _install_hint(error: BaseException) -> str | None:
    current: BaseException | None = error
    while current is not None:
        if isinstance(current, ModuleNotFoundError):
            return (
                "Install the visualization and GraspGenX dependencies, then retry: "
                "`uv sync --extra manipulation --extra graspgenx`"
            )
        current = current.__cause__
    return None


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        if args.object_cloud is None:
            run_contributor_demo(max_candidates=args.max_candidates)
        else:
            run_contributor_demo(
                max_candidates=args.max_candidates,
                object_cloud_path=args.object_cloud,
                object_frame_id=args.frame_id,
            )
    except KeyboardInterrupt:
        print("grasp-visualization-demo stopped", flush=True)
        return 0
    except Exception as error:
        hint = _install_hint(error)
        if hint is None:
            raise
        print(f"grasp-visualization-demo failed: {error}\n{hint}", flush=True)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

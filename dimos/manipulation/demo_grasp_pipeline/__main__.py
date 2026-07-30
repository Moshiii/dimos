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

"""Run the offline GraspGenX-to-connected-motion-planning pipeline."""

import argparse
from collections.abc import Sequence
from pathlib import Path
import sys

from .demo import run_contributor_demo


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("grasp-pipeline-demo"),
        help="Directory for summary.json, plans.json, and the selected-grasp PNG.",
    )
    parser.add_argument(
        "--max-candidates",
        type=int,
        default=20,
        help="Maximum number of ranked proposals to plan.",
    )
    parser.add_argument(
        "--workspace-center",
        nargs=3,
        type=float,
        metavar=("X", "Y", "Z"),
        default=(0.45, 0.0, 0.25),
        help="Target-cloud centroid in the synthetic xArm world, in metres.",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        result = run_contributor_demo(
            output_dir=args.output_dir,
            max_candidates=args.max_candidates,
            workspace_center=args.workspace_center,
        )
    except Exception as exc:
        cause: BaseException | None = exc
        while cause is not None:
            if isinstance(cause, ModuleNotFoundError) and cause.name == "graspgenx":
                print(
                    "GraspGenX is not installed. Run this demo with "
                    "`uv run --extra graspgenx python -m "
                    "dimos.manipulation.demo_grasp_pipeline ...` "
                    f"({cause})",
                    file=sys.stderr,
                    flush=True,
                )
                return 2
            cause = cause.__cause__
        raise
    for outcome in result.outcomes:
        detail = f" rejection={outcome.rejection}" if outcome.rejection else ""
        print(
            f"candidate rank={outcome.rank} score={outcome.score:.6f} "
            f"status={outcome.status}{detail}",
            flush=True,
        )
    status = "selected" if result.success else result.failure_reason
    print(
        f"grasp-pipeline-demo status={status} candidates={result.candidate_count} "
        f"summary={result.summary_path} plans={result.plans_path} "
        f"visualization={result.image_path}",
        flush=True,
    )
    return 0 if result.success else 1


if __name__ == "__main__":
    raise SystemExit(main())

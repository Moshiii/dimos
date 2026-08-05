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

from __future__ import annotations

import argparse
import os
from pathlib import Path
import sys

from dimos.benchmark.agent_eval.case import EvalCase
from dimos.benchmark.agent_eval.config import RuntimeCredential
from dimos.benchmark.agent_eval.pi_process import NodePiSessionFactory
from dimos.benchmark.short_horizon_qa.eval import run_frozen_case
from dimos.benchmark.short_horizon_qa.models import MapperSettings
from dimos.benchmark.short_horizon_qa.prepare import prepare_bundle
from dimos.benchmark.short_horizon_qa.service import serve_bundle


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="python -m dimos.benchmark.short_horizon_qa")
    commands = parser.add_subparsers(dest="command", required=True)
    prepare = commands.add_parser("prepare", help="prepare frozen Memory2 cutoffs")
    prepare.add_argument("--recording", required=True)
    prepare.add_argument(
        "--progress",
        type=float,
        action="append",
        help="normalized recording progress in [0, 1]; repeat for multiple selections",
    )
    prepare.add_argument(
        "--cutoff-seconds",
        type=float,
        action="append",
        help="legacy seconds after recording start; repeat for multiple selections",
    )
    prepare.add_argument("--output", type=Path, required=True)
    prepare.add_argument("--voxel-size", type=float, default=0.05)
    prepare.add_argument("--device", default="CUDA:0")
    serve = commands.add_parser("serve", help="serve one frozen cutoff over MCP")
    serve.add_argument("--bundle", type=Path, required=True)
    selection = serve.add_mutually_exclusive_group(required=True)
    selection.add_argument("--progress", type=float)
    selection.add_argument("--cutoff-seconds", type=float, help="legacy relative cutoff")
    serve.add_argument("--mcp-port", type=int, default=9990)
    evaluate = commands.add_parser(
        "evaluate", help="run one credentialed frozen-QA case through Pi"
    )
    evaluate.add_argument("--case", type=Path, required=True)
    evaluate.add_argument("--bundle", type=Path, required=True)
    evaluate.add_argument("--private-root", type=Path, required=True)
    evaluate.add_argument("--output", type=Path, required=True)
    auth = evaluate.add_mutually_exclusive_group(required=True)
    auth.add_argument("--credential-env")
    auth.add_argument("--credential-path", type=Path)
    evaluate.add_argument("--turn-timeout", type=float, default=180.0)
    args = parser.parse_args(argv)
    if args.command == "prepare":
        manifest = prepare_bundle(
            args.recording,
            args.cutoff_seconds or [],
            args.output,
            progress=args.progress or [],
            mapper=MapperSettings(voxel_size_m=args.voxel_size, device=args.device),
        )
        print(
            f"prepared {len(manifest.cutoffs)} cutoffs at {args.output} from {manifest.source_path}"
        )
        return 0
    if args.command == "serve":
        serve_bundle(
            args.bundle,
            args.cutoff_seconds,
            progress=args.progress,
            mcp_port=args.mcp_port,
        )
        return 0
    if args.command == "evaluate":
        try:
            result = _evaluate(args)
        except Exception as exc:
            print(
                f"frozen-QA preflight failed: {type(exc).__name__}: {exc}",
                file=sys.stderr,
            )
            return 2
        print(
            f"{result.outcome.task_result}: {result.outcome.reason} "
            f"(artifacts: {result.attempt_path})"
        )
        return 0 if result.outcome.attempt_status == "completed" else 1
    return 2


def _evaluate(args: argparse.Namespace):
    case_path = args.case.resolve()
    bundle = args.bundle.resolve()
    private_root = args.private_root.resolve()
    output = args.output.resolve()
    case = EvalCase.model_validate_json(case_path.read_bytes())
    if not (bundle / "manifest.v1.json").is_file():
        raise FileNotFoundError("prepared bundle manifest does not exist")
    if not private_root.is_dir():
        raise FileNotFoundError("private validator root does not exist")
    output.mkdir(parents=True, exist_ok=True)
    probe = output / ".write-probe"
    probe.touch(exist_ok=False)
    probe.unlink()
    adapter = (
        Path(__file__).resolve().parents[3]
        / "packages"
        / "pi-spatial-adapter"
        / "dist"
        / "code-policy-main.js"
    )
    if not adapter.is_file():
        raise FileNotFoundError(
            "Pi adapter is not built; run npm run build in packages/pi-spatial-adapter"
        )
    if args.credential_env is not None:
        value = os.environ.get(args.credential_env)
        if not value:
            raise ValueError(f"credential environment variable {args.credential_env!r} is unset")
        credential = RuntimeCredential(
            auth_mode="environment",
            binding_name=args.credential_env,
            value=value,
        )
    else:
        credential_path = args.credential_path.resolve()
        if not credential_path.is_file():
            raise FileNotFoundError("subscription credential file does not exist")
        credential = RuntimeCredential(
            auth_mode="subscription",
            binding_name=str(credential_path),
            value=None,
        )
    factory = NodePiSessionFactory(
        command=("node", str(adapter)),
        credential=credential,
        model="gpt-5.6-luna",
        thinking_level="medium",
        startup_timeout_s=args.turn_timeout,
    )
    return run_frozen_case(
        case=case,
        bundle=bundle,
        private_root=private_root,
        output_root=output,
        pi_factory=factory,
        turn_timeout_s=args.turn_timeout,
    )


if __name__ == "__main__":
    raise SystemExit(main())

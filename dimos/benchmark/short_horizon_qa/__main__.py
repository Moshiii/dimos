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
from pathlib import Path

from dimos.benchmark.short_horizon_qa.models import MapperSettings
from dimos.benchmark.short_horizon_qa.prepare import prepare_bundle
from dimos.benchmark.short_horizon_qa.service import serve_bundle


def main() -> int:
    parser = argparse.ArgumentParser(prog="python -m dimos.benchmark.short_horizon_qa")
    commands = parser.add_subparsers(dest="command", required=True)
    prepare = commands.add_parser("prepare", help="prepare frozen Memory2 cutoffs")
    prepare.add_argument("--recording", required=True)
    prepare.add_argument("--cutoff-seconds", type=float, action="append", required=True)
    prepare.add_argument("--output", type=Path, required=True)
    prepare.add_argument("--voxel-size", type=float, default=0.05)
    prepare.add_argument("--device", default="CUDA:0")
    serve = commands.add_parser("serve", help="serve one frozen cutoff over MCP")
    serve.add_argument("--bundle", type=Path, required=True)
    serve.add_argument("--cutoff-seconds", type=float, required=True)
    serve.add_argument("--mcp-port", type=int, default=9990)
    args = parser.parse_args()
    if args.command == "prepare":
        manifest = prepare_bundle(
            args.recording,
            args.cutoff_seconds,
            args.output,
            mapper=MapperSettings(voxel_size_m=args.voxel_size, device=args.device),
        )
        print(
            f"prepared {len(manifest.cutoffs)} cutoffs at {args.output} from {manifest.source_path}"
        )
        return 0
    if args.command == "serve":
        serve_bundle(args.bundle, args.cutoff_seconds, mcp_port=args.mcp_port)
        return 0
    return 2


if __name__ == "__main__":
    raise SystemExit(main())

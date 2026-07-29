#!/usr/bin/env bash
set -euo pipefail

uv run dimos --simulation run xarm-perception-sim-agent --daemon \
  -o pickandplacemodule.visualization.backend=none
trap 'uv run dimos stop' EXIT INT TERM

uv run dimos agent-send \
  "Use python_exec to inspect the latest joint state and detected objects from memory. Print a concise summary. If no objects are recorded, call scan_objects once, query memory again, and report the result. Do not pick or move anything else."

echo "Following the DimOS log. Press Ctrl-C to stop the simulation."
uv run dimos log -f

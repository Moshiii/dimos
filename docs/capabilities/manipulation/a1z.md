---
title: "Galaxea A1Z"
---

The A1Z integration uses the vendor's 250 Hz position-control loop, the G1Z
gravity model, and the G1Z gripper.

## Linux setup

The A1Z has no brakes. Support the arm and keep its workspace clear whenever
motors may enable or disable.

```bash
uv sync --extra manipulation
uv pip install "a1z @ git+https://github.com/userguide-galaxea/GALAXEA-A1Z.git@e931ecd0e25ad35df251097ba42921b3d2fa7224"
uv run --no-sync dimos a1z setup
```

`setup` verifies the pinned SDK, configures the HHS adapter as the stable
SocketCAN interface `a1zcan` at 1 Mbit/s, and tests transmission. It asks before
using `sudo`. If the HHS driver rejects transmission on Jetson, follow the
[Galaxea driver guide](https://galaxea-ai.feishu.cn/docx/XF2ed4pmhoervNxODlfc11Gvnbb)
for the exact running kernel; do not install a desktop kernel or copy a module
from another machine.

## Run

```bash
uv run --no-sync dimos run keyboard-teleop-a1z
```

This launches keyboard teleoperation, the control coordinator, trajectory
execution, and `ManipulationModule`. You do not need `--can-port` when setup
created `a1zcan`; otherwise add `--can-port <interface>`.

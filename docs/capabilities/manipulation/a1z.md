---
title: "Galaxea A1Z"
---

The A1Z integration uses the vendor's 250 Hz position-control loop, the G1Z
gravity model, and the G1Z gripper.

## Linux setup

Run these commands from the root of a DimOS source checkout. The PyPI package
does not include the A1Z host setup.

Install `can-utils`, synchronize DimOS, then install the pinned, gripper-capable
A1Z SDK. Run the SDK installation after `uv sync`; a later exact sync may remove
packages that are not in `uv.lock`.

```bash
sudo apt-get install can-utils
uv sync --extra manipulation
uv pip install "a1z @ git+https://github.com/userguide-galaxea/GALAXEA-A1Z.git@e931ecd0e25ad35df251097ba42921b3d2fa7224"
uv run --no-sync dimos a1z setup
```

Run the setup command as your normal user. It first verifies that the SDK
supports the G1Z gripper. It then asks for confirmation before using `sudo` to:

1. Load the Linux `gs_usb` kernel driver.
2. Bind the HHS USB-CANFD adapter.
3. Create the stable `a1zcan` SocketCAN interface at 1 Mbit/s.
4. Send a safe probe and verify that the adapter transmitted it.

Do not start the arm unless setup prints `A1Z CAN setup passed`. To verify only
the Python SDK, run:

```bash
uv run --no-sync dimos a1z setup --sdk-only
```

After rebooting or reconnecting the HHS adapter, configure and test SocketCAN
again:

```bash
uv run --no-sync dimos a1z can-setup
```

## Run

The A1Z has no brakes or hardware e-stop button; the PSU switch is the hardware
kill switch. Support the arm and clear its workspace before starting or
stopping DimOS. Disabling the motors makes the arm fall.

```bash
uv run --no-sync dimos run keyboard-teleop-a1z
```

This launches keyboard teleoperation, the control coordinator, trajectory
execution, and `ManipulationModule`. Startup waits for feedback from all six arm
motors, validates the measured state, holds the measured pose, and then ramps
gravity compensation.

The blueprint uses `a1zcan` by default. If you configured another verified
SocketCAN interface, pass it explicitly:

```bash
uv run --no-sync dimos run keyboard-teleop-a1z --can-port can0
```

## Troubleshooting

- **The interface is UP, but the arm does not respond.** Some Linux `gs_usb`
  drivers create an interface but drop every transmission through the HHS
  adapter. Update to a kernel with the endpoint-discovery fix. On Jetson or
  another pinned-kernel system, follow the
  [Galaxea driver guide](https://galaxea-ai.feishu.cn/docx/XF2ed4pmhoervNxODlfc11Gvnbb)
  and build the driver for the exact running kernel. Do not install a desktop
  kernel or copy a kernel module from another machine.
- **The bus behaves strangely after a crash.** Replug the HHS adapter and rerun
  `dimos a1z can-setup`.
- **The gripper remains stiff after shutdown.** The adapter retries the disable
  command, but a degraded bus can still lose it. Support the arm and turn off
  the PSU.

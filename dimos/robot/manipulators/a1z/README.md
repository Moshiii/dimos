# Galaxea A1Z hardware

The real A1Z integration uses the vendor's 250 Hz MIT position-control loop,
the G1Z model for gravity compensation, and the G1Z gripper. Linux uses native
SocketCAN; macOS uses the HHS USB-CANFD adapter through a userspace gs-usb
transport.

## Safety

The A1Z has no brakes. Disabling its motors lets the arm fall freely. Support
the arm, keep the workspace clear, and be ready to remove power before
connecting or enabling it.

The adapter fails closed when the CAN interface is missing, down, or attached
to an unexpected kernel driver. Startup also waits for all motor feedback and
rejects unsafe position, velocity, or error states before enabling control.

## Host setup

From a DimOS source checkout, install the pinned, gripper-capable vendor SDK:

```bash
uv sync --locked --inexact --group galaxea-a1z
```

Then verify the SDK and configure the platform transport:

```bash
dimos a1z setup
```

The CLI does not install or modify Python packages. To verify only the
already-installed SDK without checking attached hardware:

```bash
dimos a1z setup --sdk-only
```

Linux hosts can rerun SocketCAN setup after boot or reconnecting the adapter:

```bash
dimos a1z can-setup
```

Run these commands as your normal user. The CLI asks for confirmation and
requests `sudo` only for `modprobe`, driver binding, and `ip link` operations.
Do not start the robot unless it reports that A1Z CAN setup passed. The default
stable interface is `a1zcan` at 1 Mbit/s.

On macOS, install system libusb first (`brew install libusb`). The setup command
opens the adapter in listen-only mode, verifies that userspace USB-CAN works,
and closes it without transmitting.

## Linux HHS adapter compatibility

The HHS USB-CANFD adapter shipped with the A1Z uses USB ID `a8fa:8598`. Some
Linux `gs_usb` drivers create a normal-looking, UP SocketCAN interface but
cannot transmit because they hardcode bulk endpoint `0x02`; this adapter may
use endpoint `0x01`. `dimos a1z can-setup` detects this by sending an empty
extended-ID probe that cannot match an A1Z motor command and checking the
kernel transmit/drop counters.

If the CLI reports that the driver rejected the transmission:

- On an ordinary x86-64 Ubuntu host, update to a distribution kernel containing
  the corrected `gs_usb` endpoint discovery, reboot, and rerun the command.
  Galaxea currently recommends Ubuntu kernel `6.8.0-124` or newer.
- On an NVIDIA Jetson or another pinned-kernel system, do not install a generic
  desktop kernel and do not copy a module from another machine. Patch and build
  `gs_usb` for the exact running kernel and architecture, install it under
  `/lib/modules/$(uname -r)`, run `sudo depmod -a`, and reboot.

References:

- [Galaxea HHS driver patch guide](https://galaxea-ai.feishu.cn/docx/XF2ed4pmhoervNxODlfc11Gvnbb)
- [Upstream Linux endpoint-discovery fix](https://github.com/torvalds/linux/commit/889b2ae9139a87b3390f7003cb1bb3d65bf90a26)

If transmission does not increment either counter, check arm power, CAN
cabling, termination, and whether another process is resetting the adapter.

## Run

After setup:

```bash
uv run dimos run coordinator-a1z
```

Use `--can-port <interface>` to select a different verified SocketCAN
interface. `--simulation` keeps the existing mock-hardware behavior.

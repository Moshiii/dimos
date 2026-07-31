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

Run the setup wrapper as your normal user:

```bash
./dimos/robot/manipulators/a1z/scripts/setup_a1z.sh
```

It installs the locked, gripper-capable vendor SDK and validates the platform
CAN transport. On Linux it requests `sudo` only for SocketCAN setup. To install
and verify Python dependencies without touching attached hardware:

```bash
./dimos/robot/manipulators/a1z/scripts/setup_a1z.sh --sdk-only
```

Linux hosts can rerun only the CAN setup after boot or reconnecting the
adapter:

```bash
sudo ./dimos/robot/manipulators/a1z/scripts/setup_a1z_can.sh
```

Do not start the robot unless the script reports that A1Z CAN setup passed.
The script configures the stable interface name `a1zcan` at 1 Mbit/s and checks
that the USB transmit path works.

## Run

After setup:

```bash
uv run dimos run coordinator-a1z
```

Use `--can-port <interface>` to select a different verified SocketCAN
interface. `--simulation` keeps the existing mock-hardware behavior.

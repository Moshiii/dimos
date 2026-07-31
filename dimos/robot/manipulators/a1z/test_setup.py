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

from pathlib import Path
from unittest.mock import Mock

import tomllib
from typer.testing import CliRunner

from dimos.robot.manipulators.a1z import cli as a1z_cli

REPOSITORY_ROOT = Path(__file__).parents[4]
PYPROJECT_PATH = REPOSITORY_ROOT / "pyproject.toml"
runner = CliRunner()


def test_locked_a1z_group_contains_vendor_and_macos_transport() -> None:
    project = tomllib.loads(PYPROJECT_PATH.read_text())

    dependencies = set(project["dependency-groups"]["galaxea-a1z"])

    assert dependencies == {
        "a1z @ git+https://github.com/userguide-galaxea/GALAXEA-A1Z.git@e931ecd0e25ad35df251097ba42921b3d2fa7224",
        "gs-usb==0.3.1; sys_platform == 'darwin'",
        "pyusb==1.3.1; sys_platform == 'darwin'",
    }


def test_setup_help_documents_sdk_only() -> None:
    result = runner.invoke(a1z_cli.app, ["setup", "--help"])

    assert result.exit_code == 0
    assert "--sdk-only" in result.stdout
    assert "Verify the A1Z SDK" in result.stdout


def test_setup_sdk_only_does_not_check_hardware(monkeypatch) -> None:
    monkeypatch.setattr(a1z_cli, "_verify_sdk", Mock(return_value="/sdk/a1z"))
    configure = Mock()
    monkeypatch.setattr(a1z_cli, "_configure_linux_can", configure)

    result = runner.invoke(a1z_cli.app, ["setup", "--sdk-only"])

    assert result.exit_code == 0, result.output
    assert "A1Z vendor SDK check passed: /sdk/a1z" in result.output
    configure.assert_not_called()


def test_setup_reports_missing_sdk_without_installing(monkeypatch) -> None:
    monkeypatch.setattr(
        a1z_cli,
        "_verify_sdk",
        Mock(side_effect=RuntimeError("install the pinned SDK")),
    )

    result = runner.invoke(a1z_cli.app, ["setup", "--sdk-only"])

    assert result.exit_code == 1
    assert "install the pinned SDK" in result.output


def test_can_setup_rejection_does_not_request_privileges(monkeypatch) -> None:
    monkeypatch.setattr(a1z_cli.platform, "system", Mock(return_value="Linux"))
    monkeypatch.setattr(a1z_cli.typer, "confirm", Mock(return_value=False))
    configure = Mock()
    monkeypatch.setattr(a1z_cli, "_configure_linux_can", configure)

    result = runner.invoke(a1z_cli.app, ["can-setup"])

    assert result.exit_code == 1
    assert "Aborted." in result.output
    configure.assert_not_called()


def test_can_setup_confirms_before_configuring(monkeypatch) -> None:
    monkeypatch.setattr(a1z_cli.platform, "system", Mock(return_value="Linux"))
    monkeypatch.setattr(a1z_cli.typer, "confirm", Mock(return_value=True))
    configure = Mock()
    monkeypatch.setattr(a1z_cli, "_configure_linux_can", configure)

    result = runner.invoke(
        a1z_cli.app,
        ["can-setup", "--interface", "can7", "--bitrate", "500000"],
    )

    assert result.exit_code == 0, result.output
    configure.assert_called_once_with("can7", 500000)


def test_macos_setup_uses_listen_only_transport_check(monkeypatch) -> None:
    monkeypatch.setattr(a1z_cli, "_verify_sdk", Mock(return_value="/sdk/a1z"))
    monkeypatch.setattr(a1z_cli.platform, "system", Mock(return_value="Darwin"))
    verify_macos = Mock()
    monkeypatch.setattr(a1z_cli, "_verify_macos_can", verify_macos)

    result = runner.invoke(a1z_cli.app, ["setup"])

    assert result.exit_code == 0, result.output
    verify_macos.assert_called_once_with()


def test_linux_can_setup_limits_privileged_commands(
    monkeypatch,
    tmp_path: Path,
) -> None:
    usb_root = tmp_path / "usb"
    usb_device = usb_root / "1-1"
    usb_interface = usb_root / "1-1:1.0" / "net" / "can0"
    usb_device.mkdir(parents=True)
    usb_interface.mkdir(parents=True)
    (usb_device / "idVendor").write_text("a8fa\n")
    (usb_device / "idProduct").write_text("8598\n")
    sys_class_net = tmp_path / "net"
    sys_class_net.mkdir()
    privileged = Mock()
    verify = Mock()
    monkeypatch.setattr(a1z_cli, "_SYS_USB_DEVICES", usb_root)
    monkeypatch.setattr(a1z_cli, "_SYS_CLASS_NET", sys_class_net)
    monkeypatch.setattr(a1z_cli, "_run_privileged", privileged)
    monkeypatch.setattr(a1z_cli, "_verify_can_transmit", verify)

    a1z_cli._configure_linux_can("a1zcan", 1_000_000)

    assert [call.args[0] for call in privileged.call_args_list] == [
        ["modprobe", "gs_usb"],
        ["ip", "link", "set", "can0", "down"],
        ["ip", "link", "set", "can0", "name", "a1zcan"],
        [
            "ip",
            "link",
            "set",
            "a1zcan",
            "type",
            "can",
            "bitrate",
            "1000000",
        ],
        ["ip", "link", "set", "a1zcan", "up"],
    ]
    verify.assert_called_once_with("a1zcan", usb_device)

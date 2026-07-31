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

from unittest.mock import MagicMock

from dimos.manipulation import pnpconsole


def test_client_scans_scene_and_quits(monkeypatch) -> None:  # type: ignore[no-untyped-def]
    pnp = MagicMock()
    pnp.scan_scene.return_value = MagicMock(detections_length=3)
    app = MagicMock(PickNPlaceModule=pnp)
    monkeypatch.setattr(pnpconsole.Dimos, "connect", lambda: app)
    choices = iter(["1", "q"])
    monkeypatch.setattr("builtins.input", lambda _prompt: next(choices))

    pnpconsole.main()

    pnp.scan_scene.assert_called_once_with()


def test_client_does_not_execute_without_a_plan(monkeypatch) -> None:  # type: ignore[no-untyped-def]
    app = MagicMock()
    manipulation = app.ManipulationModule
    monkeypatch.setattr(pnpconsole.Dimos, "connect", lambda: app)
    choices = iter(["5", "q"])
    monkeypatch.setattr("builtins.input", lambda _prompt: next(choices))

    pnpconsole.main()

    manipulation.execute_and_wait.assert_not_called()


def test_client_goes_home(monkeypatch) -> None:  # type: ignore[no-untyped-def]
    app = MagicMock()
    manipulation = app.ManipulationModule
    monkeypatch.setattr(pnpconsole.Dimos, "connect", lambda: app)
    choices = iter(["13", "q"])
    monkeypatch.setattr("builtins.input", lambda _prompt: next(choices))

    pnpconsole.main()

    manipulation.go_home.assert_called_once_with("arm")


def test_preview_replays_three_times() -> None:
    manipulation = MagicMock()

    pnpconsole._preview(manipulation)

    assert manipulation.preview_plan.call_count == 3
    manipulation.preview_plan.assert_called_with(duration=1.0)

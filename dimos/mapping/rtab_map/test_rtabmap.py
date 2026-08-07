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

"""Smoke tests for the RTAB-Map native module.

Split by what they need, so the cheap ones stay in the default suite:

* Declaration tests import the module and check its streams and config. No binary, no
  camera -- these catch a rename or a dropped stream, which unwires the module from its
  blueprint silently.
* The config-parity test is the one that matters most here. The native side parses its
  config struct with a strict one-to-one key check: every field required, no unknowns. So
  a field added on either side alone is a startup crash, and it crashes in a worker
  process where the traceback is easy to miss.
* ``self_hosted`` tests additionally run the built binary, and are skipped unless the nix
  build output is present.

Deliberately *not* covered: mapping accuracy. That needs a recording and a ground truth,
which is a benchmark rather than a smoke test.
"""

from __future__ import annotations

from pathlib import Path as FilePath
import re
import subprocess

import pytest

from dimos.core.native_module import NativeModule
from dimos.mapping.rtab_map.rtabmap import RERUN_CONFIG, RtabmapConfig, RtabmapSlam
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.protocol.pubsub.patterns import pattern_matches

MODULE_DIR = FilePath(__file__).resolve().parent
BINARY = MODULE_DIR / "result" / "bin" / "rtabmap_slam"
SOURCE = MODULE_DIR / "src" / "rtabmap_slam.cpp"

needs_binary = pytest.mark.skipif(not BINARY.exists(), reason=f"native binary not built: {BINARY}")


def test_is_a_native_module() -> None:
    assert issubclass(RtabmapSlam, NativeModule)


def test_declares_the_expected_streams() -> None:
    """A dropped or renamed stream silently unwires the module from its blueprint."""
    annotations = RtabmapSlam.__annotations__
    expected = {
        "color_image": Image,
        "depth_image": Image,
        "camera_info": CameraInfo,
        "odometry": Odometry,
        "corrected_odometry": Odometry,
        "map_tf": Odometry,
        "cloud_map": PointCloud2,
        "tf": TFMessage,
    }
    for name, payload in expected.items():
        assert name in annotations, f"stream {name} disappeared"
        assert payload.__name__ in str(annotations[name]), (
            f"stream {name} changed payload type: {annotations[name]}"
        )


def test_config_is_constructible_with_defaults() -> None:
    assert RtabmapConfig() is not None


def test_blueprint_factory_exists() -> None:
    assert hasattr(RtabmapSlam, "blueprint")


def test_config_fields_match_the_native_struct() -> None:
    """The native struct requires every field and rejects unknowns, so drift is fatal.

    Parsed out of the source rather than out of the binary so this runs without a build:
    the failure it guards against is editing one side and forgetting the other, which is
    a source-level mistake.
    """
    body = re.search(r"struct RtabmapConfig \{(.*?)\n\};", SOURCE.read_text(), re.DOTALL)
    assert body is not None, "could not find struct RtabmapConfig in the native source"

    declaration = re.compile(r"^\s{4}(?:[\w:<>,\s]*?)\b(\w+);\s*(?://.*)?$", re.MULTILINE)
    native_fields = set(declaration.findall(body.group(1)))
    python_fields = set(RtabmapConfig().to_config_dict())

    assert native_fields == python_fields, (
        f"config drift -- only in C++: {sorted(native_fields - python_fields)}, "
        f"only in Python: {sorted(python_fields - native_fields)}"
    )


def test_frame_defaults_form_the_standard_tree() -> None:
    """map -> odom -> base_link. Two publishers of one edge fight, so the names matter."""
    config = RtabmapConfig()
    assert (config.map_frame, config.odom_frame, config.base_frame) == (
        "map",
        "odom",
        "base_link",
    )


def test_rerun_path_overrides_match_their_entity_paths() -> None:
    """A missed override is silent: the path just renders half a metre off.

    The bridge matches a plain ``str`` pattern by exact equality against the whole
    entity path, and ``Glob("*")`` does not cross a ``/``. Both of those produce zero
    matches with no error, so the pattern is asserted rather than assumed.
    """
    overrides = RERUN_CONFIG["visual_override"]
    assert isinstance(overrides, dict)

    matched = {
        entity: [pattern for pattern in overrides if pattern_matches(pattern, entity)]
        for entity in ("world/path", "world/d435if/path")
    }
    for entity, patterns in matched.items():
        assert len(patterns) == 1, f"{entity} matched {len(patterns)} overrides, expected 1"

    # ...and nothing else is caught by them.
    for entity in ("world/odometry", "world/cloud_map", "world/corrected_odometry"):
        assert not [p for p in overrides if pattern_matches(p, entity)], (
            f"{entity} should not be overridden"
        )


@needs_binary
def test_binary_is_executable() -> None:
    import os

    assert os.access(BINARY, os.X_OK), f"{BINARY} is not executable"


@pytest.mark.self_hosted
@needs_binary
def test_binary_exits_cleanly_without_config() -> None:
    """Run with no stdin config the binary must fail deliberately, not crash.

    A signal death here means the rtabmap shared libraries failed to load, which is the
    most common way this module breaks on a fresh machine.
    """
    completed = subprocess.run(
        [str(BINARY)],
        capture_output=True,
        text=True,
        timeout=60,
        check=False,
        input="",
        env={"DIMOS_TRANSPORT": "lcm", "PATH": "/usr/bin:/bin"},
    )
    assert completed.returncode > 0, (
        f"binary died on signal {-completed.returncode} with no config; "
        "usually a missing or mismatched rtabmap shared library"
    )

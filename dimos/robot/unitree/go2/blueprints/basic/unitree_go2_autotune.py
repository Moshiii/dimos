#!/usr/bin/env python3
# Copyright 2025-2026 Dimensional Inc.
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

"""Go2 control-autotune: coordinator + excitation battery + FOPDT fit/tune.

vx/wz only, matching this repo's existing go2 SI precedent (non-strafing).
Pose fitter: go2's leg odometry is too slow to differentiate cleanly.

    dimos run unitree-go2-autotune
    # ENTER to run each excitation (K=skip, Backspace=quit), same gate as the
    # existing controller benchmarks. Writes tuned_config.json +
    # characterization_report.json under ~/.local/state/dimos/autotune/go2.
"""

from __future__ import annotations

from dimos.control.autotune.autotune_driver import AutotuneDriver
from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_coordinator import (
    unitree_go2_coordinator,
)
from dimos.robot.unitree.keyboard_teleop import KeyboardTeleop

unitree_go2_autotune = (
    autoconnect(
        unitree_go2_coordinator,
        KeyboardTeleop.blueprint(publish_only_when_active=True),
        AutotuneDriver.blueprint(
            robot_id="go2",
            joint_prefix="go2/",
            channels={"vx": 1.0, "wz": 1.5},
            odom_type="pose",
            fitter="pose",
        ),
    )
    .remappings([(AutotuneDriver, "pose", "go2_odom")])
    .global_config(obstacle_avoidance=False)
)

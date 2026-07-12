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

"""FAST-LIVO2 blueprints: livox + realsense streams in, odometry out.

Unlike the pointlio/fastlio2 stacks, the odometry module here does not touch
the sensors itself — the Mid360 module owns the lidar (publishing per-point
timing on the lidar stream), the RealSense module owns the camera, and
FastLivo consumes their streams over LCM.
"""

from dimos.core.coordination.blueprints import autoconnect
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.hardware.sensors.lidar.fastlivo.module import FastLivo
from dimos.hardware.sensors.lidar.livox.module import Mid360
from dimos.visualization.vis_module import vis_module

mid360_realsense_fastlivo = autoconnect(
    Mid360.blueprint(),
    RealSenseCamera.blueprint(),
    FastLivo.blueprint(),
    vis_module("rerun"),
).global_config(n_workers=3, robot_model="mid360_realsense_fastlivo")

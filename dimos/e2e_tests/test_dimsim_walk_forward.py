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

import pytest


@pytest.mark.self_hosted_large
def test_walk_forward(
    lcm_spy,
    start_blueprint,
    wait_for_agent_ready,
    human_input,
    scene_control,
    simulator_name,
) -> None:
    scene_args = ("--dimsim-scene=empty",) if simulator_name == "dimsim" else ()
    start_blueprint(
        *scene_args,
        "run",
        "--disable",
        "spatial-memory",
        "--disable",
        "security-module",
        "unitree-go2-agentic",
        simulator=simulator_name,
        scene_package="none",
    )
    wait_for_agent_ready(timeout=1200.0)

    origin_x, origin_y = 1, 2
    scene_control.set_agent_position(origin_x, origin_y)
    lcm_spy.save_topic("/global_costmap#nav_msgs.OccupancyGrid")
    scene_control.add_wall(-1, -1, 6, -1)
    scene_control.add_wall(-1, 5, 6, 5)
    scene_control.add_wall(-1, -1, -1, 5)
    scene_control.add_wall(6, -1, 6, 5)
    lcm_spy.wait_for_saved_topic("/global_costmap#nav_msgs.OccupancyGrid", timeout=30)

    human_input("move forward 3 meter")

    lcm_spy.wait_until_odom_position(origin_x + 3, origin_y, threshold=0.4, timeout=120)

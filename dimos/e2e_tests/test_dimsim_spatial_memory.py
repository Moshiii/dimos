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

from dimos.e2e_tests.simulation_scenarios import (
    APARTMENT_SEMANTIC_NAVIGATION_SCENARIOS,
    APARTMENT_TASK_START,
    GO_TO_BED,
    SemanticNavigationScenario,
)
from dimos.msgs.std_msgs.Bool import Bool

_GOAL_REACHED_TOPIC = "/goal_reached#std_msgs.Bool"


def _run_semantic_navigation_scenario(
    scenario: SemanticNavigationScenario,
    *,
    lcm_spy,
    start_blueprint,
    wait_for_agent_ready,
    wait_for_robot_odometry,
    human_input,
    scene_control,
    simulator_name,
    explore_house,
) -> None:
    start_blueprint(
        "run",
        "unitree-go2-agentic",
        simulator=simulator_name,
    )
    wait_for_agent_ready(timeout=1200.0)
    wait_for_robot_odometry(timeout=120.0)

    target_bounds = scene_control.semantic_object_bounds(scenario.target_query)
    explore_house()
    scene_control.set_agent_position(*APARTMENT_TASK_START)
    lcm_spy.wait_until_odom_position(
        APARTMENT_TASK_START[0],
        APARTMENT_TASK_START[1],
        threshold=0.25,
        timeout=30.0,
    )

    # Subscribe before sending the task so an immediate completion cannot race
    # the assertion. The semantic bounds remain test-only ground truth.
    lcm_spy.save_topic(_GOAL_REACHED_TOPIC)
    human_input(scenario.command)
    lcm_spy.wait_for_saved_message_result(
        _GOAL_REACHED_TOPIC,
        Bool,
        predicate=lambda message: message.data is True,
        fail_message=f"Navigation did not report completion for {scenario.command!r}",
        timeout=scenario.navigation_timeout_s,
    )
    lcm_spy.wait_until_odom_near_bounds(
        target_bounds,
        max_distance=scenario.max_target_distance_m,
        timeout=30.0,
    )


@pytest.mark.self_hosted_large
def test_go_to_the_bed(
    lcm_spy,
    start_blueprint,
    wait_for_agent_ready,
    human_input,
    scene_control,
    simulator_name,
    explore_house,
    wait_for_robot_odometry,
) -> None:
    _run_semantic_navigation_scenario(
        GO_TO_BED,
        lcm_spy=lcm_spy,
        start_blueprint=start_blueprint,
        wait_for_agent_ready=wait_for_agent_ready,
        wait_for_robot_odometry=wait_for_robot_odometry,
        human_input=human_input,
        scene_control=scene_control,
        simulator_name=simulator_name,
        explore_house=explore_house,
    )


@pytest.mark.self_hosted_large
@pytest.mark.parametrize(
    "scenario",
    APARTMENT_SEMANTIC_NAVIGATION_SCENARIOS,
    ids=lambda scenario: scenario.scenario_id,
)
def test_apartment_semantic_navigation(
    scenario,
    lcm_spy,
    start_blueprint,
    wait_for_agent_ready,
    wait_for_robot_odometry,
    human_input,
    scene_control,
    simulator_name,
    explore_house,
) -> None:
    _run_semantic_navigation_scenario(
        scenario,
        lcm_spy=lcm_spy,
        start_blueprint=start_blueprint,
        wait_for_agent_ready=wait_for_agent_ready,
        wait_for_robot_odometry=wait_for_robot_odometry,
        human_input=human_input,
        scene_control=scene_control,
        simulator_name=simulator_name,
        explore_house=explore_house,
    )

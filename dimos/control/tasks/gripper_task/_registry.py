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

TASK_FACTORIES = {
    "gripper": "dimos.control.tasks.gripper_task.gripper_task:create_task",
}

# The two streams that carry gripper *intent* — never a joint value. A browser
# toggle says "closed"; a trigger says "squeezed 42%". Neither sender knows the
# gripper's travel, and neither should: vendor ranges stay below this task.
# Numeric targets arrive through TASK_EXPOSES via task_invoke.
TASK_CONSUMES: dict[str, dict[str, tuple[str, str]]] = {
    "gripper": {
        "gripper_command": ("on_gripper_command", "broadcast"),
        "teleop_buttons": ("on_teleop_buttons", "broadcast"),
    },
}

TASK_EXPOSES: dict[str, list[str]] = {
    "gripper": [
        "set_position",
        "set_normalized",
        "set_sweep",
        "set_reference_pose",
        "get_position",
        "get_state",
    ],
}

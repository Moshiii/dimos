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

"""Interactive RPC client for the stepwise ``picknplace-perception`` pipeline.

Start the blueprint first, then run:

    uv run python -m dimos.manipulation.pnpconsole
"""

from __future__ import annotations

from pprint import pprint
import time
from typing import Any

from dimos import Dimos
from dimos.manipulation.planning.planners.config import RoboPlanCartesianPathConfig
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped


def _object_number() -> int | None:
    value = input("Object number: ").strip()
    try:
        number = int(value)
    except ValueError:
        print("Enter a positive whole number.")
        return None
    if number < 1:
        print("Enter a positive whole number.")
        return None
    return number


def _print_pose(pose: Any) -> None:
    if pose is None:
        print("No pose is available.")
        return
    if frame_id := getattr(pose, "frame_id", None):
        print(f"frame: {frame_id}")
    print(f"position: {pose.position.as_tuple}")
    print(
        "orientation: "
        f"({pose.orientation.x}, {pose.orientation.y}, {pose.orientation.z}, {pose.orientation.w})"
    )


def _cartesian_waypoints(manipulation: Any, target: Any) -> list[PoseStamped] | None:
    current = manipulation.get_ee_pose("arm")
    if current is None:
        return None
    return [
        PoseStamped(frame_id="world", position=current.position, orientation=current.orientation),
        PoseStamped(frame_id="world", position=target.position, orientation=target.orientation),
    ]


def _preview(manipulation: Any) -> None:
    print(f"Viser preview: {manipulation.get_visualization_url()}")
    for _ in range(3):
        print(manipulation.preview_plan(duration=1.0))


def _print_grasp_candidates(candidates: Any) -> None:
    if not candidates.candidates:
        return
    print(f"GraspGenX proposals: {len(candidates.candidates)}")
    for rank, candidate in enumerate(candidates.candidates[:5]):
        pose = candidate.pose
        print(
            f"{rank}: score={candidate.score:.3f} position={pose.position.as_tuple} "
            f"orientation={pose.orientation.to_tuple()}"
        )


def main() -> None:
    """Connect to PickNPlaceModule and run one explicit pick-pipeline stage."""
    print("Connecting to PickNPlaceModule...")
    app = Dimos.connect()
    pnp = app.PickNPlaceModule
    manipulation = app.ManipulationModule
    goal = None
    pre_grasp = None
    approach_planned = False
    approach_executed = False
    descent_planned = False
    descent_executed = False
    gripper_closed = False
    ascent_planned = False
    ascent_executed = False
    print("Connected. Every planned motion is previewed in Viser before execution.")

    while True:
        print("\n1) Scan  2) Info  3) Select target  4) Plan/preview approach")
        print("5) Execute approach  6) Plan/preview descent  7) Execute descent")
        print("8) Close  9) Plan/preview ascent  10) Execute ascent  11) Open")
        print("12) Current EE  13) Go home  q) Quit")
        choice = input("Select: ").strip().lower()
        try:
            if choice == "q":
                return
            if choice == "1":
                detections = pnp.scan_scene()
                print(f"Detected {detections.detections_length} object(s).")
            elif choice == "2":
                pprint(pnp.get_scene_info())
            elif choice == "3":
                if (number := _object_number()) is not None:
                    goal = pnp.get_goal_pose(number)
                    pre_grasp = pnp.get_pre_grasp_pose()
                    _print_grasp_candidates(pnp.get_grasp_candidates())
                    approach_planned = False
                    approach_executed = False
                    descent_planned = False
                    descent_executed = False
                    gripper_closed = False
                    ascent_planned = False
                    ascent_executed = False
                    print("Goal:")
                    _print_pose(goal)
                    print("Pre-grasp:")
                    _print_pose(pre_grasp)
            elif choice == "4":
                if pre_grasp is None:
                    print("Select a target first.")
                else:
                    approach_planned = manipulation.plan_to_pose(
                        Pose(pre_grasp.position, pre_grasp.orientation), "arm"
                    )
                    print(approach_planned)
                    if approach_planned:
                        _preview(manipulation)
            elif choice == "5":
                if not approach_planned:
                    print("Plan the approach first.")
                else:
                    approach_executed = manipulation.execute_and_wait()
                    print(approach_executed)
            elif choice == "6":
                if goal is None or not approach_executed:
                    print("Execute the approach first.")
                else:
                    waypoints = _cartesian_waypoints(manipulation, goal)
                    descent_planned = False
                    descent_planned = manipulation.plan_cartesian_targets(
                        {"arm/manipulator": waypoints},
                        RoboPlanCartesianPathConfig(max_linear_speed=0.03),
                    )
                    print(descent_planned)
                    if descent_planned:
                        _preview(manipulation)
            elif choice == "7":
                if not descent_planned:
                    print("Plan the descent first.")
                else:
                    descent_executed = manipulation.execute_and_wait()
                    print(descent_executed)
            elif choice == "8":
                if not descent_executed:
                    print("Execute the descent first.")
                else:
                    gripper_closed = manipulation.close_gripper("arm").is_success()
                    print(gripper_closed)
            elif choice == "9":
                if pre_grasp is None or not gripper_closed:
                    print("Close the gripper before planning ascent.")
                else:
                    waypoints = _cartesian_waypoints(manipulation, pre_grasp)
                    ascent_planned = False
                    ascent_planned = manipulation.plan_cartesian_targets(
                        {"arm/manipulator": waypoints},
                        RoboPlanCartesianPathConfig(max_linear_speed=0.03),
                    )
                    print(ascent_planned)
                    if ascent_planned:
                        _preview(manipulation)
            elif choice == "10":
                if not ascent_planned:
                    print("Plan the ascent first.")
                else:
                    # Gripper commands are asynchronous on xArm. Reassert close before
                    # lift and let that command settle before dispatching the trajectory.
                    gripper_closed = manipulation.close_gripper("arm").is_success()
                    if not gripper_closed:
                        print("Failed to keep the gripper closed; ascent was not executed.")
                    else:
                        time.sleep(1.5)
                        ascent_executed = manipulation.execute_and_wait()
                        print(ascent_executed)
            elif choice == "11":
                if not ascent_executed:
                    print("Execute the ascent before opening the gripper.")
                else:
                    print(manipulation.open_gripper("arm"))
            elif choice == "12":
                _print_pose(manipulation.get_ee_pose("arm"))
            elif choice == "13":
                print(manipulation.go_home("arm"))
            else:
                print("Choose 1-13 or q.")
        except Exception as exc:
            print(f"RPC failed: {exc}")


if __name__ == "__main__":
    main()

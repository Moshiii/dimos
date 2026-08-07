## Scenario coverage

Every scenario in the change's two capability specs is covered by the focused
tests below. Construction assertions inspect serialized task configuration and
blueprint wiring without starting physical hardware.

### Multi-frame IK control

| Spec scenarios | Coverage |
|---|---|
| Valid single/multi-frame configuration; invalid model binding | `test_pose_target_ik.py` constructor tests, `test_pink_ik.py::test_step_frame_targets_rejects_unknown_frame`, and the OpenArm construction test |
| One/two active frame targets; bounded one-step execution | `test_pink_ik.py` frame-target step tests and `test_pose_target_ik.py::test_compute_calls_one_pink_step_and_preserves_output_order` |
| Planning translates group tips; control calls frame targets directly | `test_pink_ik.py` planning-group and multi-target solve tests plus the pose-target compute test |
| Concrete Pink tuning | `test_pink_ik.py::test_step_frame_targets_builds_both_frame_tasks_with_tuning` and the OpenArm construction assertion |
| Valid/unsafe/missing-seed output | `test_pose_target_ik.py` ordered-output, unsafe-result, and incomplete-state tests |
| Kinematics-neutral coordinator arbitration | `test_coordinator_routing.py` card-routing and lifecycle tests |

### Quest arm teleoperation

| Spec scenarios | Coverage |
|---|---|
| One/two bindings and invalid collections | `test_quest_teleop_ik_task.py` configuration, single-binding, and bimanual tests |
| Relative controller mapping | `test_single_binding_tracks_relative_controller_motion` |
| One-hand inactive, atomic two-hand engage, either-hand release | `test_bimanual_task_requires_both_hands_and_releases_atomically` |
| Stale side disables all; two fresh targets share one Pink step | bimanual timeout and combined-step tests |
| Independent triggers while active; no gripper output while disengaged | combined-step test and OpenArm coordinator component test, including post-release no-output assertion |
| One task receives both routed streams; separate tasks receive one each | `test_coordinator_routing.py::TestByTaskNameRouting` |
| Mock-default OpenArm construction, Viser-backed manipulation, trajectory execution task, and combined command | `test_openarm_teleop.py` construction and end-to-end coordinator tests |
| Migrated single-arm and mixed-arm construction | `test_blueprints.py` Quest blueprint construction assertions |

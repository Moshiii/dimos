## 1. Extract the bounded Pink frame-target core

- [x] 1.1 Add Pink kinematics tests for one-step single-frame and two-frame updates from a `RobotModelConfig`, including controlled-joint ordering, target-frame validation, and concrete tuning propagation.
- [x] 1.2 Refactor Pink model/context, frame-task construction, joint mapping, and differential-update logic into a concrete bounded frame-target step that does not require `WorldSpec` or planning groups.
- [x] 1.3 Refactor `PinkIK.solve` and `PinkIK.solve_pose_targets` to translate planning inputs into frame targets and iterate the shared step while preserving current planning results, attempts, tolerances, and collision behavior.
- [x] 1.4 Run the focused Pink and planning kinematics tests and fix any single-target or multi-target regression.

## 2. Introduce the shared pose-target control task

- [x] 2.1 Add unit tests for a `PoseTargetIKTask` core covering model/frame validation, live-state warm starts, one Pink step per tick, ordered output, missing state, non-finite results, joint-delta rejection, timeout, and resource claims.
- [x] 2.2 Implement the shared pose-target task configuration and base class using `RobotModelConfig`, top-level `TaskConfig.joint_names`, named target frames, and `PinkKinematicsConfig`.
- [x] 2.3 Refactor `CartesianIKTask` into a thin absolute-pose leaf over the shared base and replace its `model_path`/`ee_joint_id` registry configuration with robot-model and target-frame configuration.
- [x] 2.4 Migrate every in-repository Cartesian IK task configuration to the new shape and remove the obsolete parser-specific configuration and duplicated Pinocchio IK code.
- [x] 2.5 Run the Cartesian IK task, task-registry, and affected manipulator blueprint tests.

## 3. Implement unified Quest arm teleoperation

- [x] 3.1 Add configuration tests for valid one- and two-binding Quest tasks and rejection of empty, oversized, duplicate-hand, duplicate-frame, unknown-frame, and unknown-joint bindings.
- [x] 3.2 Implement immutable Quest hand bindings and refactor the Quest task as `QuestTeleopIKTask`, a thin leaf over `PoseTargetIKTask` that owns controller samples, reference poses, relative target mapping, and per-hand gripper state.
- [x] 3.3 Add task tests proving single-binding engagement and relative motion continue to work through the shared Pink core.
- [x] 3.4 Add bimanual safety tests proving one engaged hand emits nothing, two-hand engagement captures both references atomically, release of either hand clears the whole task, stale input on either side disables output, and re-engagement recaptures both references.
- [x] 3.5 Add bimanual output tests proving both fresh frame targets enter one Pink step and independently mapped grippers appear once in the combined ordered command only while fully engaged.
- [x] 3.6 Replace the legacy Quest task factory/configuration with the unified hand-binding configuration and remove obsolete single-hand parser-specific paths rather than retaining a compatibility registration.

## 4. Preserve controller identity through coordinator routing

- [x] 4.1 Add distinct left and right Cartesian pose inputs to `ControlCoordinator` without adding hand-binding, frame, kinematic, Pink, or planning-group logic to the coordinator.
- [x] 4.2 Register Quest task consumption so each pose input routes by task name to its corresponding left or right callback while buttons continue to broadcast.
- [x] 4.3 Add coordinator routing tests for two controller streams targeting one task, two streams targeting separate tasks, unknown task names, and subscription cleanup.
- [x] 4.4 Update `ArmTeleopModule` blueprint wiring and all in-repository single-arm and mixed-arm Quest blueprints to use the distinct ports and new one-binding configurations.
- [x] 4.5 Run the coordinator routing, Quest module, task-registry, and migrated teleoperation blueprint tests.

## 5. Add the mock-default OpenArm bimanual blueprint

- [x] 5.1 Add an explicit OpenArm in-memory whole-body hardware constructor whose adapter is always `mock_whole_body`, independent of the simulation default used by physical-capable blueprints.
- [x] 5.2 Define one OpenArm Quest task with both arm joints, both grippers, the existing bimanual `RobotModelConfig`, left/right grasp-frame bindings, and concrete Pink tuning.
- [x] 5.3 Compose and expose `teleop-quest-openarm` with both Quest pose streams naming the same task and with the explicit in-memory hardware component.
- [x] 5.4 Add blueprint construction tests asserting one bimanual task, both bindings, complete joint claims, the bimanual model, correct stream remappings, and an unconditional `mock_whole_body` adapter.
- [x] 5.5 Add an end-to-end component test that drives both Quest poses, buttons, and triggers through the task/coordinator path and observes one combined mock-hardware command; also assert release of either hand stops output.
- [x] 5.6 Regenerate `dimos/robot/all_blueprints.py` and verify `teleop-quest-openarm` is discoverable through the built-in blueprint registry.

## 6. Documentation and final verification

- [x] 6.1 Update control and Quest teleoperation documentation for frame-based task configuration, one/two hand bindings, two-hand bimanual engagement, and the mock-default OpenArm launch command.
- [x] 6.2 Run the focused control, Quest, OpenArm blueprint, Pink kinematics, and blueprint-generation test suites.
- [x] 6.3 Run formatting, linting, and relevant strict mypy checks on all changed modules and resolve failures.
- [x] 6.4 Validate the OpenSpec change and confirm every scenario in both capability specs is covered by an automated test or an explicitly documented construction assertion.

## 7. Complete the OpenArm manipulation stack

- [x] 7.1 Extend the OpenArm blueprint construction test to require one Viser-backed `ManipulationModule` using the bimanual model and one coordinator-side trajectory task over both arm joint sets with priority over Quest teleoperation.
- [x] 7.2 Wire the manipulation module, Viser visualization, and joint-trajectory task into `teleop-quest-openarm` without changing its unconditional mock hardware default.
- [x] 7.3 Update OpenArm/Quest documentation and scenario coverage for planning visualization and trajectory execution, then run focused blueprint, registry, formatting, lint, and OpenSpec validation checks.

## 8. Preserve canonical OpenArm startup and one-group planning

- [x] 8.1 Add regression coverage for the canonical all-zero OpenArm model/mock start, bounded bimanual control from zero, and a one-attempt real-Pink one-arm planning solve that holds the unselected arm.
- [x] 8.2 Remove the OpenArm home-posture workaround, add opt-in inward joint-limit posture tuning shared by OpenArm control/planning, and update locked joints through a writable configuration copy plus Pink's public update API.
- [x] 8.3 Update the startup/planning documentation and run focused Pink/OpenArm tests, formatting, lint, typing, and OpenSpec validation.

## 9. Harden streaming control at measured joint limits

- [x] 9.1 Record the asymmetric feedback-tolerance and command-margin safety contract in the glossary, ADR, and multi-frame control specification.
- [x] 9.2 Add regression coverage for tolerated and rejected feedback, inward seed normalization, output saturation, unbounded joints, narrow joint ranges, warning throttling, and the reported OpenArm boundary value.
- [x] 9.3 Implement configurable streaming-only Pink feedback tolerance and command margin while retaining Pink's safety break and the task's joint-delta guard.
- [x] 9.4 Run the focused Pink, shared pose-target, Cartesian, Quest, OpenArm, formatting, linting, typing, and OpenSpec verification checks.

## Why

DimOS arm teleoperation currently models each Quest hand as a separate single-end-effector IK task, which cannot command both arms of a coupled bimanual robot through one kinematic model. OpenArm now has a proper bimanual model, making this the right time to add a safe dual-arm teleoperation path while consolidating the duplicated Cartesian and teleoperation IK machinery.

## What Changes

- Add a shared pose-target IK control-task base that can solve one or more named end-effector frame targets with one robot model and one concrete Pink backend.
- Keep Cartesian IK and Quest teleoperation as thin sibling tasks over the shared control core; single-arm and bimanual Quest operation use the same Quest task class with one or two hand bindings.
- Route left and right Quest pose streams distinctly to one control task without teaching the control coordinator about planning groups or robot kinematics.
- Require both Quest hands to be engaged before a two-binding task emits commands, and disengage the whole task when either hand releases.
- Add an OpenArm dual-arm Quest teleoperation blueprint that uses the existing single bimanual URDF, solves both grasp-frame targets together, controls both grippers, selects in-memory whole-body hardware by default, and includes manipulation planning with Viser plus coordinator-side joint-trajectory execution.
- Reuse Pink's internal frame-target step for bounded control ticks and iterative planning solves, while retaining the concrete Pink tuning configuration.
- **BREAKING**: Replace parser-specific Cartesian and teleoperation IK task configuration (`model_path`, `ee_joint_id`, and a single `hand`) with robot-model, controlled-joint, target-frame, and hand-binding configuration. Migrate all in-repository blueprint callers without a compatibility layer.

## Capabilities

### New Capabilities

- `multi-frame-ik-control`: Streaming pose-target control for one or more named frames through one robot model, with shared Pink solving, joint mapping, safety validation, and coordinator arbitration.
- `quest-arm-teleoperation`: Single-arm and bimanual Quest arm teleoperation through configurable hand bindings, atomic bimanual engagement, gripper control, and a mock-default OpenArm blueprint.

### Modified Capabilities

None. This repository has no existing OpenSpec capability specifications.

## Impact

- Refactors `dimos/control/tasks/cartesian_ik_task/` and `dimos/control/tasks/teleop_task/` around a shared pose-target IK task implementation.
- Extends task stream registration and `ControlCoordinator` inputs so two pose streams can retain their identity while targeting one task.
- Extends the Pink kinematics implementation with a bounded frame-target step shared by control and planning.
- Updates existing manipulator and Quest teleoperation blueprint task configurations to the new configuration shape.
- Adds an OpenArm Quest teleoperation blueprint, explicit mock whole-body hardware configuration, Viser-backed manipulation planning, and joint-trajectory execution under `dimos/robot/manipulators/openarm/` and `dimos/teleop/quest/`.
- Adds focused unit, routing, safety, blueprint construction, and registry-generation coverage. No new solver abstraction or dependency is introduced.

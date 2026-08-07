## Why

Robots need different Pink objectives to achieve responsive, natural teleoperation, but the current backend hardcodes one frame/posture task layout and exposes customization mainly as a growing scalar configuration. A subclass extension point is needed now so G1, OpenArm, and future robots can reuse the common solve and safety algorithms while declaring only their robot-specific Pink tasks.

## What Changes

- Add a protected, composable task-stack API to `PinkIK` based on an ordered `dict[str, pink.tasks.Task]`.
- Reserve and validate one `frame/<frame_name>` task for every commanded frame while allowing subclasses to tune, replace, or add named auxiliary tasks.
- Retain task instances per exclusively owned IK control context and update their small mutable targets or temporal fields on each control tick.
- Add before- and after-solve hooks for auxiliary tasks that require per-tick inputs or solver history, without making the solve, integration, mapping, or safety algorithms overridable.
- Preserve default `PinkIK` behavior, including the per-tick measured-posture target and existing planning/control configuration.
- Continue injecting a `PinkIK` instance into the generic pose-target control task; robot-specific control-task subclasses and a universal task-tuning schema are not introduced.

## Capabilities

### New Capabilities

- `pink-task-customization`: Defines named Pink task-stack composition, inheritance, ownership, persistence, validation, and lifecycle hooks for planning and streaming IK.

### Modified Capabilities

None.

## Impact

- Refactors `dimos/manipulation/planning/kinematics/pink_ik.py` and its tests while retaining the existing public planning and streaming entry points.
- Keeps `PoseTargetIKTask`, `CartesianIKTask`, and `QuestTeleopIKTask` behavior and dependency-injection API unchanged.
- Establishes a protected subclass contract that robot packages can use for direct Pink task declarations without adding robot fields to the global kinematics configuration.
- May extract private Pink model/mapping or planning helpers to keep the public backend navigable; no new runtime dependency is required.

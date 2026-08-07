## Why

OpenArm's generic dual-arm Pink objective tracks both grasp frames, but it treats translation, orientation, and every joint uniformly. Near its canonical straight-arm zero pose this leaves redundant joints under-shaped and makes singular or infeasible Quest targets more likely to produce unstable teleoperation.

## What Changes

- Add an OpenArm teleop-specific `PinkIK` subclass with translation-dominant frame tracking, dynamically weighted current-posture regularization, and one low-weight position-manipulability task per grasp frame.
- Let declarative Quest task configuration carry a `PinkIK` backend class so the generic factory constructs a fresh backend instance for every control-task instance.
- Wire the OpenArm Quest blueprint to the custom backend while leaving its manipulation planner on plain `PinkIK`.
- Constrain every streaming Pink solve to the pose-target task's per-tick joint-delta budget so valid QP output does not repeatedly fail the outer safety gate.
- Preserve the canonical all-zero hardware start, common engagement and timeout behavior, hard Pink limits, feedback tolerance, command saturation, and command-delta safety.

## Capabilities

### New Capabilities

- `robot-specific-pink-teleop`: Defines fresh backend-class injection for generic Quest tasks and the OpenArm-specific task objective used for robust bimanual teleoperation.

### Modified Capabilities

None.

## Impact

- Extends the generic Quest task parameter/helper interface with an optional `type[PinkIK]` dependency provider.
- Adds an OpenArm-specific Pink backend and wires it into `teleop-quest-openarm`.
- Adds focused generic-factory and OpenArm objective/streaming tests.
- Extends the streaming `PinkIK.step_frame_targets()` interface with an optional caller-owned command-delta bound; planning calls remain unchanged.
- Requires no new runtime dependency beyond the existing manipulation extra.

## 1. Declarative Backend Injection

- [x] 1.1 Add a defaulted `ik_backend_type` provider to generic Quest parameters and the shared blueprint helper.
- [x] 1.2 Construct a fresh configured backend in the generic Quest factory and pass it through the existing `ik=` constructor seam.
- [x] 1.3 Add factory tests proving default compatibility and distinct custom backend instances across repeated construction.

## 2. OpenArm Pink Objective

- [x] 2.1 Add `OpenArmTeleopPinkIK` with translation-dominant inherited frame tasks and mirrored dynamic posture costs.
- [x] 2.2 Add one named position-only manipulability task per commanded OpenArm grasp frame.
- [x] 2.3 Wire only the OpenArm Quest control task to the custom backend while retaining plain Pink planning and canonical zero initialization.

## 3. OpenArm Verification

- [x] 3.1 Add objective-contract tests for deterministic task order, exact costs, target frames, and independent task state.
- [x] 3.2 Verify real Pink bimanual streaming from canonical zero remains finite, moves both elbows inward, tracks reachable targets, and respects existing command bounds.
- [x] 3.3 Re-run generic Quest safety/timeout tests, OpenArm blueprint and coordinator tests, Ruff, targeted mypy, `git diff --check`, and strict OpenSpec validation.

## 4. Bounded Streaming Solves

- [x] 4.1 Pass the pose-target task's validated per-tick delta budget into each streaming Pink solve while preserving unbounded direct/planning calls.
- [x] 4.2 Intersect controlled-joint command intervals measured from raw feedback with Pink's existing velocity, configuration, and floating-base constraints.
- [x] 4.3 Add common limit-contract and task-plumbing tests plus a deterministic real OpenArm regression for the previously over-limit target.
- [x] 4.4 Re-run focused control/Pink/OpenArm suites, Ruff, targeted mypy, `git diff --check`, and strict OpenSpec validation.

## 5. OpenArm Command Budget

- [x] 5.1 Raise only the OpenArm Quest task's outer joint-delta budget from 5 to 10 degrees, yielding a 9.9-degree QP budget.
- [x] 5.2 Verify the OpenArm blueprint contract, focused regression suite, formatting, and strict OpenSpec validation.

## 6. OpenArm Streaming Velocity

- [x] 6.1 Add an optional task-owned streaming velocity cap to common pose-target and Pink QP interfaces.
- [x] 6.2 Configure only OpenArm Quest teleoperation at `1.0 rad/s` while preserving the 10-degree emergency gate and URDF limits.
- [x] 6.3 Add limit-intersection, task-plumbing, blueprint, and real OpenArm regression coverage.
- [x] 6.4 Re-run focused suites, Ruff, targeted mypy, `git diff --check`, and strict OpenSpec validation.

## 7. Generic Quest Streaming Default

- [x] 7.1 Default generic single-arm and dual-arm Quest tasks to `1.0 rad/s` while preserving per-robot overrides and non-teleop behavior.
- [x] 7.2 Remove the redundant OpenArm override and verify the resolved factory/blueprint configuration.
- [x] 7.3 Re-run focused suites, Ruff, targeted mypy, `git diff --check`, and strict OpenSpec validation.

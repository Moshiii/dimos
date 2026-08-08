## 1. Characterize Existing Behavior

- [x] 1.1 Add characterization tests for plain `PinkIK` frame-task parameters, task order, and per-tick measured-posture targets in both single- and multi-frame streaming control.
- [x] 1.2 Add characterization tests proving planning and streaming retain their current public results, joint ordering, limit handling, and failure behavior before refactoring task construction.

## 2. Named Task-Stack API

- [x] 2.1 Add the protected `_create_tasks()` contract with ordered `frame/<frame_name>` entries and the default current-posture auxiliary task.
- [x] 2.2 Validate the most-derived task mapping for every mandatory frame key, compatible task type, exact frame identity, and deterministic order before solving.
- [x] 2.3 Freeze validated task mappings structurally while leaving contained Pink task objects mutable.
- [x] 2.4 Add typed `_before_solve()` and `_after_solve()` hooks and invoke the latter only after a successful Pink velocity solve.

## 3. Persistent Control Contexts

- [x] 3.1 Refactor the streaming cache around one combined context keyed by robot model identity, controlled-joint order, and ordered target-frame selection.
- [x] 3.2 Reuse the combined context's named task instances across ticks while rebuilding the Pink configuration from each measured seed and mutating all frame targets in place.
- [x] 3.3 Preserve per-tick current-posture regularization for plain `PinkIK` without reconstructing its posture task.
- [x] 3.4 Keep feedback-limit normalization, common Pink invocation, integration, output mapping, and command-limit saturation outside all subclass hooks.

## 4. Planning Integration

- [x] 4.1 Refactor iterative planning to consume the same named task-construction contract and stable task ordering.
- [x] 4.2 Invoke task lifecycle hooks consistently for each planning iteration while preserving convergence, retry, locked-joint, joint-limit, and collision-checking behavior.

## 5. Subclass Contract Tests

- [x] 5.1 Add a test subclass that calls `super()`, tunes one reserved frame task, replaces the posture task, and adds an arbitrary auxiliary Pink task.
- [x] 5.2 Add a multi-level inheritance test proving a derived subclass can modify one parent-declared named task without recreating or disturbing the remaining stack.
- [x] 5.3 Add tests proving task identities persist across control ticks, frame targets mutate in place, and separate backend instances do not share task state.
- [x] 5.4 Add tests for read-only per-tick mappings, missing or mismatched frame entries, temporal before/after hooks, and absence of an after-solve callback on solver failure.
- [x] 5.5 Verify an injected `PinkIK` subclass changes the objective used by the generic `TeleopIKTask` without changing its engagement, synchronization, timeout, or output-safety semantics.

## 6. Documentation and Verification

- [x] 6.1 Finalize the Pink task-stack and IK control-context glossary entries and ADR describing the subclass boundary and rejected alternatives.
- [x] 6.2 Document the protected subclass API with a concise example that composes native Pink tasks directly in code and does not introduce a universal tuning schema.
- [x] 6.3 Run focused Pink, pose-target, Cartesian, Quest, OpenArm, and planning tests; run Ruff, targeted mypy, `git diff --check`, and strict OpenSpec validation.

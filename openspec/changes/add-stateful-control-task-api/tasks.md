## 1. Immutable Observation and Task Context

- [x] 1.1 Add focused tests for immutable `CoordinatorState` and `JointStateSnapshot` fields and mappings, then convert the observation types and affected annotations or fixtures to read-only structures.
- [x] 1.2 Add tests for an unbound, bound, rebound, and detached `BaseControlTask`, then implement frozen `ControlTaskContext`, the read-only `context` property, and private binding lifecycle hooks.

## 2. Coordinator Observation Ownership

- [x] 2.1 Add tick-loop tests proving a complete observation is published before active-task filtering and shared by reference with `compute()`, then add the observation callback to `TickLoop`.
- [x] 2.2 Add coordinator tests for pre-first-tick reads, atomic state replacement, and command/tick concurrency, then implement `_latest_state`, `_state_lock`, the shared context, and the coordinator getter/callback.
- [x] 2.3 Add registration tests for binding, rollback, cross-coordinator rejection, structural-task compatibility, removal, and re-add, then integrate context lifecycle into `add_task()` and `remove_task()`.
- [x] 2.4 Add stop/restart and runtime-reset tests, then clear cached observations at both discontinuities while preserving registered task bindings.

## 3. Strict Generic Task Commands

- [x] 3.1 Update command characterization tests so missing tasks raise and unknown or existing-but-undeclared methods raise `AttributeError`.
- [x] 3.2 Remove reflective fallback dispatch from `task_invoke()` while preserving signature binding, handler exception propagation, the existing method name, and unchanged result passthrough.
- [x] 3.3 Audit in-repo `task_invoke()` callers and task manifests; declare every supported remote command and migrate or remove undeclared access.

## 4. Joint Trajectory Task Migration

- [x] 4.1 Add tests for canonical JTT construction and rejection of noncanonical generic configuration, then define `JOINT_TRAJECTORY_TASK_NAME = "joint_trajectory"` and remove constructor or blueprint name overrides.
- [x] 4.2 Rewrite JTT execution tests to bind context state, then change `execute()` to accept only a trajectory, pull current positions from `context.get_state()`, and preserve start-state validation outcomes.
- [x] 4.3 Remove `NO_TRAJECTORY_TASK` from execution and cancellation status enums and update serialization, callers, and tests.

## 5. Coordinator and Manipulation Decoupling

- [x] 5.1 Add coordinator tests proving task-type-agnostic operation with zero or one canonical JTT, then remove trajectory imports, `_trajectory_task`, singleton checks, and coordinator-level execute/cancel methods.
- [x] 5.2 Update `PlanExecutionManager` tests and implementation to call `task_invoke()` for `execute` and `cancel` on `joint_trajectory`, preserve typed result mapping, and apply existing invocation-failure policy.
- [x] 5.3 Migrate manipulator blueprint helpers and all shipped blueprints from hardware-derived or custom JTT names to the canonical name.
- [x] 5.4 Remove obsolete coordinator-client trajectory helpers and migrate remaining examples, E2E tests, and direct callers to generic task commands.

## 6. Documentation and Verification

- [x] 6.1 Update control and manipulation documentation to describe task context access, strict `TASK_EXPOSES`, the canonical JTT name, and removal of coordinator trajectory RPCs.
- [x] 6.2 Run focused control, coordinator-command, trajectory-task, manipulation execution, blueprint, and E2E tests affected by the migration.
- [x] 6.3 Run Ruff format/check, targeted mypy, `git diff --check`, and `pytest dimos/robot/test_all_blueprints_generation.py`; commit any generated blueprint registry update.

## Context

`PinkIK` is the shared backend for iterative manipulation planning and one-step pose-target control. It currently rebuilds a Pink `Configuration`, frame tasks, and posture task whenever `_configuration_and_tasks()` is called. Scalar fields on `PinkKinematicsConfig` cover the default task layout, but cannot express robot-specific combinations such as anisotropic frame costs, weighted nominal posture, damping, coupling, manipulability, or temporal objectives without growing a global schema.

`PoseTargetIKTask` already accepts an injected `PinkIK` instance. That is the correct ownership seam: Cartesian and Quest tasks retain their input, engagement, timeout, arbitration, and output-safety behavior, while a robot-specific `PinkIK` subclass changes only the objective passed to the common solver.

## Goals / Non-Goals

**Goals:**

- Make Pink task composition directly subclassable using native Pink task objects.
- Support incremental multi-level inheritance where a subclass calls `super()` and changes only named entries.
- Guarantee that every commanded frame remains represented in the objective.
- Reuse task instances in streaming control and mutate only their targets or explicit temporal fields per tick.
- Preserve the common solve, integration, mapping, limit handling, and command-safety algorithms.
- Preserve the behavior of plain `PinkIK` for planning and streaming callers.

**Non-Goals:**

- Define a universal configuration schema for every possible Pink task.
- Add robot-specific `PoseTargetIKTask`, `CartesianIKTask`, or `QuestTeleopIKTask` subclasses.
- Add an empty `DualArmPinkIK` taxonomy; an intermediate subclass is justified only by shared objective declarations.
- Move teleoperation lifecycle or reset semantics into Pink or the control coordinator.
- Implement the G1 PR's complete robot-specific task stack in this change.

## Decisions

### Use an ordered named dictionary as the subclass contract

`PinkIK._create_tasks(configuration, target_frames)` returns `dict[str, pink.tasks.Task]`. Plain `PinkIK` creates reserved `frame/<frame_name>` entries and its default current-posture regularizer. A subclass calls `super()`, then tunes a task object, replaces a named auxiliary task, or adds a new entry. Python dictionary insertion order defines the stable task order passed to `pink.solve_ik`.

This is preferred over an anonymous list because stable names support selective overrides, multi-level inheritance, lifecycle lookup, diagnostics, and precise tests. It is preferred over task-specific configuration fields because native task construction can express Pink tasks that the generic backend does not know about.

### Reserve frame entries and validate the most-derived result

For every requested frame, the final dictionary MUST contain `frame/<frame_name>` with a compatible Pink frame-target task for that exact frame. Validation occurs after the most-derived `_create_tasks()` returns, before the stack is cached or solved. Auxiliary names are subclass-defined and MUST be unique by dictionary construction.

The common algorithm owns frame-target updates. Subclasses may tune or replace a frame task but cannot silently omit a commanded end effector.

### Keep task structure fixed and task values mutable

Streaming control caches one combined control context keyed by robot model identity, controlled-joint order, and ordered target-frame selection. The context owns the Pinocchio model/mapping, the persistent Pink task objects, and any backend-private history. After creation and validation, the dictionary is exposed to tick hooks as a read-only `Mapping`; its keys and task identities cannot change, while each task's small mutable target/history fields can.

The Pink `Configuration` is still refreshed from measured joint feedback for every streaming tick. Reusing task objects therefore does not make commanded configuration stale. Planning constructs a task stack for a solve attempt and reuses it across that attempt's iterations.

### Provide narrow per-step lifecycle hooks

`_before_solve(tasks, configuration, dt)` runs after mandatory frame targets are updated and before `pink.solve_ik`. `_after_solve(tasks, velocity, dt)` runs after a successful solve and before integration. Default implementations preserve plain-Pink semantics; subclasses use these hooks only to mutate dynamic auxiliary-task inputs or record explicit temporal state such as `LowAccelerationTask`'s previous integration.

The backend does not add a generic teleoperation reset operation. Most Pink tasks retain parameters or targets that are overwritten, not temporal history. A subclass that introduces temporal state owns its discontinuity semantics, and its `PinkIK` instance is never shared between control-task instances.

### Keep public algorithms and safety checks fixed

Robot subclasses do not override planning convergence, `step_frame_targets`, Pink invocation, configuration integration, joint mapping, feedback-limit normalization, command-limit saturation, or task-level command-delta validation. Both planning and streaming use the same task-construction hooks, while separate backend instances allow a teleoperation-specific subclass to leave the manipulation planner on plain `PinkIK`.

### Inject a backend instance into the generic control task

Robot composition constructs a fresh subclass instance and passes it through the existing `ik=` constructor parameter. The generic task does not accept a class, import path, robot name, or tuning profile and does not instantiate robot subclasses. Registry or blueprint construction remains deployment plumbing and does not create another behavioral inheritance hierarchy.

## Risks / Trade-offs

- **Protected hooks become a compatibility surface** → Document their invariants, type them, and cover multi-level composition with contract tests.
- **A subclass mutates reserved frame entries incorrectly** → Validate reserved keys, frame identity, task compatibility, and stable structure before solving.
- **Persistent tasks accidentally retain stale temporal history** → Keep ownership exclusive and require the subclass introducing temporal state to test its before/after behavior and discontinuity policy.
- **Persistent default posture changes existing control feel** → Refresh the default current-posture target from each tick's measured configuration, matching current behavior.
- **One `PinkIK` file remains difficult to navigate** → Private model/mapping and planning helpers may be extracted without creating additional public solver classes or widening the subclass contract.

## Migration Plan

1. Add the named task-construction and lifecycle hooks with the existing frame/posture behavior as their default implementation.
2. Refactor planning and streaming paths to consume the named mapping and preserve all current results and safety checks.
3. Add persistent combined control contexts and verify task identities remain stable while targets update.
4. Add test-only subclasses covering selective replacement, auxiliary tasks, multi-level inheritance, lifecycle hooks, and validation failures.
5. Robot packages may adopt subclasses incrementally; existing callers continue constructing plain `PinkIK` without configuration changes.

Rollback is mechanical: restore per-call task construction and the existing `_configuration_and_tasks()` helper. No serialized data or external API migration is involved.

## Open Questions

None.

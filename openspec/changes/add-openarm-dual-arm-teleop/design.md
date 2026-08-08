## Context

The current `CartesianIKTask` and `TeleopIKTask` independently construct single-end-effector `PinocchioIK` solvers from a model path and numeric end-effector joint ID. The Quest arm module can publish both controllers, but coordinator routing sends both through one Cartesian-command input and the teleoperation task stores only one target. This works for a mixed-arm setup because each controller names a separate task, but it cannot represent OpenArm: OpenArm has one coupled bimanual `RobotModelConfig`, two grasp frames, and must solve both pose targets in one Pink problem.

The planning stack already has a concrete Pink backend and an OpenArm bimanual model. Control must reuse that backend without importing planning-group concepts into `ControlCoordinator`, and must keep every control tick bounded. Quest teleoperation also has a safety constraint: a bimanual task is active only while both operator hands are engaged.

## Goals / Non-Goals

**Goals:**

- Support single-frame and multi-frame streaming pose-target IK through one shared control-task core.
- Use one concrete Pink implementation and expose the existing `PinkKinematicsConfig` tuning surface to control tasks.
- Represent single-arm and bimanual Quest teleoperation with the same task class configured with one or two hand bindings.
- Preserve controller identity when two Quest pose streams target one task.
- Run OpenArm bimanual Quest teleoperation end to end against in-memory whole-body hardware by default.
- Keep planning groups and Quest engagement semantics out of the coordinator.

**Non-Goals:**

- A generic control IK backend protocol or support for multiple control solvers.
- Independent engagement of one arm within a bimanual task.
- A new Quest session/envelope message combining both controllers.
- Converting mixed-arm teleoperation into one coupled task.
- On-hardware OpenArm validation or changes to the Quest client UI.
- Per-frame Pink task-type schemas beyond the existing concrete Pink configuration.

## Decisions

### 1. Use sibling leaf tasks over a shared pose-target IK base

The control task hierarchy will be:

```text
BaseControlTask
└── PoseTargetIKTask
    ├── CartesianIKTask
    └── TeleopIKTask
```

`PoseTargetIKTask` owns model/context construction, current-state seeding, frame-target solving, coordinator-to-model joint mapping, resource claims, joint-delta validation, timeout support, and `JointCommandOutput` construction. Leaf tasks own how inputs become a snapshot of absolute frame targets and when that snapshot is active.

`CartesianIKTask` remains an absolute-pose leaf. `TeleopIKTask` owns Quest controller state, engagement edges, captured controller and robot reference poses, relative-to-absolute mapping, and gripper targets.

This avoids making Quest teleoperation inherit single-end-effector Cartesian semantics. A separate Quest mapper module was rejected because it would add an unnecessary stream synchronization and lifecycle boundary around state used only by the Quest task.

### 2. Configure control with robot frames, not planning groups

The common task configuration uses:

- one `RobotModelConfig`;
- the controlled coordinator joint names, supplied by the existing top-level `TaskConfig.joint_names` field;
- named end-effector frames;
- `PinkKinematicsConfig`;
- common priority, timeout, and joint-delta limits.

`RobotModelConfig.joint_name_mapping` maps coordinator joint names to model joint names. Construction validates every controlled joint and target frame before the task can run.

Planning remains responsible for translating `PlanningGroup.tip_link` values into frame targets. The coordinator sees only input streams, task names, resource claims, priorities, and joint commands.

Numeric end-effector joint IDs and standalone model paths are removed from the Cartesian and Quest task configuration rather than retained as compatibility alternatives.

### 3. Use one collection-based Quest task configuration

`TeleopIKTaskConfig` contains one or two immutable hand bindings. Each binding contains:

- `hand`: left or right operator hand;
- `target_frame`: the robot-model frame controlled by that hand;
- optional gripper joint;
- open and closed gripper positions.

The task rejects empty collections, more than two bindings, duplicate hands, duplicate target frames, unknown frames, and unknown controlled or gripper joints. A one-binding task is the single-arm case; a two-binding task is the bimanual case. No single/dual subclasses are added.

### 4. Keep bimanual engagement and reference capture atomic

For a two-binding task, the engagement predicate is `left_primary and right_primary`. The task emits no command until that predicate becomes true. On the rising edge it captures both controller references and both current robot frame poses as one state transition. On the falling edge caused by either hand, it clears both references and all pending frame targets, making the task inert.

Pose callbacks update the latest controller samples but cannot independently activate either side. Missing or stale input for either bound hand invalidates the complete bimanual snapshot. Gripper trigger values are read per binding, but are emitted only while the complete task is active.

Independent per-hand engagement was rejected because it violates the agreed two-hand deadman behavior.

### 5. Add distinct coordinator pose inputs with task-name routing

`ControlCoordinator` declares distinct left and right Cartesian pose inputs. The Quest task registry card maps them to `on_left_cartesian_command` and `on_right_cartesian_command` using task-name routing, while the existing button input remains broadcast. `ArmTeleopModule` stamps both controller messages with the same task name for OpenArm and with different names for the existing mixed-arm blueprint.

This retains operator-hand identity without introducing a combined message and without making the coordinator interpret either hand bindings or robot frames.

### 6. Expose a bounded concrete Pink frame-target step

The concrete Pink implementation gains an operation equivalent to:

```python
step_frame_targets(
    robot_model: RobotModelConfig,
    frame_targets: Mapping[str, PoseStamped],
    controlled_joints: Sequence[str],
    seed: JointState,
    dt: float,
) -> JointState
```

The exact return type may retain the existing Pink result metadata, but one call performs one bounded differential-IK update. The solver caches model and frame context by robot model and frame set. It uses the complete target mapping in one QP so bimanual targets are solved together.

Planning's `solve` and `solve_pose_targets` methods keep their current public planning contracts. Internally they translate planning selections into frame targets and repeat the same Pink step until tolerances or iteration limits are reached. Control calls the step once per coordinator tick. Both paths use `PinkKinematicsConfig`; no new solver interface is introduced.

Using `solve_pose_targets(..., max_iterations=1)` directly was rejected because its planning contract includes convergence, attempts, world access, and collision behavior that do not belong in a bounded control tick.

### 7. Make the OpenArm blueprint safe by construction

The default `teleop-quest-openarm` blueprint composes:

- `ArmTeleopModule` with both outputs naming one OpenArm Quest task;
- one coordinator containing one two-binding `TeleopIKTask`;
- `openarm_bimanual_model_config()`;
- left and right grasp-frame bindings;
- both OpenArm arm joint sets and gripper joints;
- an explicit `mock_whole_body` hardware component.
- one `ManipulationModule` configured with the same bimanual model and Viser visualization;
- one coordinator-side joint-trajectory task over both arm joint sets.

It must not derive its default adapter from `global_config.simulation`, because that setting currently selects physical OpenArm hardware when false. Physical hardware requires a separately explicit blueprint/configuration path; the default bimanual teleoperation blueprint never silently connects to it.

The trajectory task has priority 20 and the Quest task priority 10. Planned execution therefore preempts teleoperation through the coordinator's existing resource arbitration, which also clears the Quest engagement session. The trajectory task claims the fourteen modeled arm joints; the two grippers remain direct Quest bindings because they are not part of the planning model.

### 8. Preserve canonical startup and lock unselected planning joints safely

OpenArm's canonical model and mock-hardware start remains all zero. The blueprint
does not silently substitute a different physical home pose. Control and planning
continue to seed from measured state. Both use the same opt-in Pink joint-limit
posture margin: when a seed coordinate is within that margin of a finite position
limit, the low-cost posture target moves that coordinate deterministically inward.
This changes neither the measured seed nor the number of QPs per control tick.

When planning targets only one group of a multi-group robot, Pink keeps every
unselected joint at its seed position. Pink configurations expose their joint vector
as read-only, so the shared step copies the integrated vector, applies all locks to
that writable copy, and commits it through `Configuration.update()`. This preserves
one-arm Viser planning without mutating Pink internals directly and leaves the
multi-target control path unchanged.

## Risks / Trade-offs

- **[Pink planning refactor changes established IK behavior]** → Keep planning's public methods and result semantics intact, add regression tests for single- and multi-target planning, and share only the internal frame-step machinery.
- **[Two asynchronously arriving controller poses can represent slightly different sample times]** → Snapshot both latest samples under one lock at the control tick and require both to be fresh; avoid a larger synchronized-envelope protocol until measured timing shows it is necessary.
- **[A bad joint-name mapping can command the wrong model coordinate]** → Validate coordinator names, model names, uniqueness, and output ordering at construction, with OpenArm mapping tests.
- **[One-step differential IK may not reach a moving target immediately]** → Warm-start from live coordinator state on every tick and allow normal repeated ticks to track the target; retain joint-delta rejection.
- **[Task configuration migration breaks external custom blueprints]** → Make the break explicit, migrate every in-repository caller, and fail fast with the new required fields rather than accepting ambiguous legacy inputs.
- **[A default blueprint could reach real hardware accidentally]** → Construct it with the explicit mock whole-body adapter and test the resolved adapter type.
- **[Pink exposes its integrated configuration as read-only]** → Apply unselected planning-group locks to a writable copy and commit it with Pink's public update method; cover one selected OpenArm arm with real Pink.
- **[The canonical all-zero OpenArm pose is singular and places both joint-4 coordinates at their lower limits]** → Use the same opt-in inward joint-limit posture target for OpenArm control and planning; verify one-attempt planning and bounded bimanual control from zero without random control retries.

## Migration Plan

1. Extract and test the concrete Pink frame-target step while preserving planning behavior.
2. Introduce `PoseTargetIKTask`, then migrate Cartesian IK to the new robot-model/frame configuration.
3. Replace the existing Quest-specific `TeleopIKTask` implementation and registry entry with `TeleopIKTask`; update all repository blueprint callers in the same change.
4. Add distinct Quest pose routing, migrate mixed-arm wiring, and verify one- and two-binding tasks.
5. Add the mock-default OpenArm blueprint, regenerate the blueprint registry, and run focused control, kinematics, Quest, and blueprint tests.

Because the change is not deployed independently from its callers, rollback is a normal source revert of the task, routing, and blueprint commits together. No stored data migration is involved.

## Open Questions

None. Exact private method names and file placement may be chosen during implementation without changing these boundaries.

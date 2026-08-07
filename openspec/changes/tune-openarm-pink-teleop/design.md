## Context

The generic `QuestTeleopIKTask` already accepts an injected `PinkIK` instance, and `PinkIK` now exposes named task construction. Declarative coordinator creation does not currently carry that dependency through its Quest factory, so the OpenArm blueprint can only select plain `PinkIK` scalar configuration. OpenArm's canonical zero pose must remain all zero even though both elbow joints begin at their lower limit and the arm is poorly conditioned.

## Goals / Non-Goals

**Goals:**

- Construct a fresh robot-specific Pink backend for every declarative Quest task.
- Improve OpenArm tracking robustness through task composition while retaining generic Quest and safety behavior.
- Produce streaming IK commands inside the task-owned joint-delta budget rather than relying on repeated post-solve rejection.
- Preserve canonical zero startup and dynamic measured-posture targets.

**Non-Goals:**

- Add an OpenArm control-task type or subclass `QuestTeleopIKTask`.
- Apply the OpenArm teleop objective to manipulation planning.
- Add low-acceleration history, fixed nominal posture, duplicate soft joint limits, or streaming self-collision geometry/barriers.

## Decisions

### Inject a backend class through the generic Quest factory

`QuestTeleopIKTaskParams` and the shared blueprint helper carry `ik_backend_type: type[PinkIK]`, defaulting to `PinkIK`. The factory calls the class with the resolved `PinkKinematicsConfig` and passes the fresh instance through the existing `ik=` constructor argument. A class provider is preferred over a prebuilt instance because task stacks are mutable and must not be shared when a declaration is instantiated more than once. A robot-specific registered task factory is rejected because it would duplicate generic Quest construction and routing.

### Use dynamic weighted posture rather than a fixed nominal

`OpenArmTeleopPinkIK` retains the inherited `posture/current` entry so common code refreshes its target from measured configuration every tick and moves near-limit coordinates inward. It changes only the cost to a mirrored per-arm vector based on the G1 proximal/elbow weighting: `1e-3 * [4, 3, 0.1, 3, 1, 1, 0.1]`. A fixed zero target is rejected because OpenArm joint 4 is at its lower limit at zero; a fixed bent target is deferred because it introduces persistent Cartesian bias.

### Prefer translation and add weak position manipulability

Each inherited frame task uses position cost `1.0` and orientation cost `0.2`. Each commanded grasp frame also gets a native `ManipulabilityTask` with position mask, cost `0.005`, and desired rate `0.05`. The objective is deliberately secondary to Cartesian tracking and hard limits. `DampingTask` is not added because current-posture regularization already supplies joint damping; `LowAccelerationTask` is not added because its history would span Quest disengagement without a task-local reset signal.

### Keep safety and planning unchanged

Plain `PinkIK` remains the manipulation planner backend. Streaming configuration/velocity limits, feedback tolerance, command saturation, finite checks, and per-tick joint-delta validation stay in shared code. Pink self-collision uses barriers and collision geometry rather than task composition and is deferred to a separate design.

### Bound streaming commands inside the Pink QP

`PoseTargetIKTask` passes its existing `max_joint_delta_deg` contract to `PinkIK.step_frame_targets()` in radians. Pink intersects 99% of that displacement budget with the model's existing velocity limits for each controlled one-DoF joint. The interval is measured from the original sensor seed, not the normalized configuration, so tolerated feedback error near a position limit cannot consume hidden command delta. The QP can redistribute motion across the remaining joints; independent post-solve joint clipping is rejected because it distorts coordinated Cartesian motion. Callers that omit the bound retain Pink's default limits, including manipulation planning. The outer task check stays at 100% as a final invariant against solver tolerance or implementation defects.

OpenArm teleoperation uses a `10`-degree outer command budget and therefore a `9.9`-degree QP budget. Generic Quest arm teleoperation separately defaults normal streaming velocity to `1.0 rad/s`; at the nominal 100 Hz rate this is approximately `0.573` degrees per tick. Pink intersects this task velocity with the URDF mechanical velocity and emergency displacement budgets. Robot declarations may override or explicitly disable the Quest default, while non-teleop pose-target tasks retain their existing configured limits.

## Risks / Trade-offs

- **Manipulability competes with target accuracy or destabilizes the QP** → Testing against the OpenArm model found that cost `0.01` can produce an 11.9-degree single-tick jump, while `0.005` remains below 3.2 degrees in the same canonical-zero motion. Use cost `0.005`, position-only masking, and retain the command safety gate.
- **Backend class is reused by repeated declarations** → Instantiate the class inside `create_task()` and test distinct backend identities.
- **Direct Pink imports widen optional-dependency exposure** → Place the import in the OpenArm backend module, which is loaded only by the manipulation-dependent OpenArm teleop blueprint.
- **The URDF permits a larger step than the task gate** → Replace Pink's default velocity-limit object only for bounded streaming calls with the intersection of the URDF interval and 99% of the caller's command interval; retain configuration and floating-base limits.
- **Feedback normalization changes the QP origin** → Build the command interval relative to the raw measured seed and the normalized Pink configuration.
- **URDF velocity limits describe actuator capability rather than comfortable teleoperation** → Keep robot descriptions intact and default generic Quest arm tasks to a task-owned `1.0 rad/s` streaming cap inside the same QP constraint.

## Migration Plan

Existing Quest declarations omit `ik_backend_type` and continue receiving plain `PinkIK`. Rollback removes the one OpenArm blueprint argument and custom backend module; no serialized state or hardware configuration changes.

## Open Questions

None.

## Context

The generic `TeleopIKTask` already accepts an injected `PinkIK` instance, and `PinkIK` now exposes named task construction. Declarative coordinator creation does not currently carry that dependency through its Quest factory, so the OpenArm blueprint can only select plain `PinkIK` scalar configuration. OpenArm's canonical zero pose must remain all zero even though both elbow joints begin at their lower limit and the arm is poorly conditioned.

## Goals / Non-Goals

**Goals:**

- Construct a fresh robot-specific Pink backend for every declarative Quest task.
- Improve OpenArm tracking robustness through task composition while retaining generic Quest and safety behavior.
- Produce streaming IK commands inside one final safety envelope rather than relying on repeated post-solve rejection.
- Preserve canonical zero startup and dynamic measured-posture targets.

**Non-Goals:**

- Add an OpenArm control-task type or subclass `TeleopIKTask`.
- Apply the OpenArm teleop objective to manipulation planning.
- Add low-acceleration history, fixed nominal posture, duplicate soft joint limits, or streaming self-collision geometry/barriers.

## Decisions

### Inject a backend class through the generic Quest factory

`TeleopIKTaskParams` and the shared blueprint helper carry `ik_backend_type: type[PinkIK]`, defaulting to `PinkIK`. The factory calls the class with the resolved `PinkKinematicsConfig` and passes the fresh instance through the existing `ik=` constructor argument. A class provider is preferred over a prebuilt instance because task stacks are mutable and must not be shared when a declaration is instantiated more than once. A robot-specific registered task factory is rejected because it would duplicate generic Quest construction and routing.

### Use dynamic weighted posture rather than a fixed nominal

`OpenArmTeleopPinkIK` retains the inherited `posture/current` entry so common code refreshes its target from measured configuration every tick and moves near-limit coordinates inward. It changes only the cost to a mirrored per-arm vector based on the G1 proximal/elbow weighting: `1e-3 * [4, 3, 0.1, 3, 1, 1, 0.1]`. A fixed zero target is rejected because OpenArm joint 4 is at its lower limit at zero; a fixed bent target is deferred because it introduces persistent Cartesian bias.

### Prefer translation and add weak position manipulability

Each inherited frame task uses position cost `1.0` and orientation cost `0.2`. Each commanded grasp frame also gets a native `ManipulabilityTask` with position mask, cost `0.005`, and desired rate `0.05`. The objective is deliberately secondary to Cartesian tracking and hard limits. `DampingTask` is not added because current-posture regularization already supplies joint damping; `LowAccelerationTask` is not added because its history would span Quest disengagement without a task-local reset signal.

### Keep safety and planning unchanged

Plain `PinkIK` remains the manipulation planner backend. Streaming position and velocity limits, feedback tolerance, command saturation, and finite checks stay in shared code. Pink self-collision uses barriers and collision geometry rather than task composition and is deferred to a separate design.

### Apply one final streaming command envelope

Pink solves from the previous command state so hardware feedback delay does not repeatedly erase progress toward the target. The candidate is then clamped once to the intersection of three independent bounds: the task's configured velocity cap intersected with each URDF velocity limit, the URDF position range with a small command margin, and the maximum permitted distance from current measured feedback. This keeps normal delayed feedback responsive while preventing the command reference from running arbitrarily far ahead of hardware.

Generic arm teleoperation defaults to `5.0 rad/s`; robot declarations may choose a lower value. The measured-feedback distance remains a separate safety parameter and does not latch the task when ordinary execution lag reaches it.

## Risks / Trade-offs

- **Manipulability competes with target accuracy or destabilizes the QP** → Testing against the OpenArm model found that cost `0.01` can produce an 11.9-degree single-tick jump, while `0.005` remains below 3.2 degrees in the same canonical-zero motion. Use cost `0.005`, position-only masking, and retain the command safety gate.
- **Backend class is reused by repeated declarations** → Instantiate the class inside `create_task()` and test distinct backend identities.
- **Direct Pink imports widen optional-dependency exposure** → Place the import in the OpenArm backend module, which is loaded only by the manipulation-dependent OpenArm teleop blueprint.
- **The URDF permits uncomfortable teleoperation speeds** → Intersect URDF velocity with a task-owned `5.0 rad/s` default in the final command envelope.
- **Hardware feedback lags the command reference** → Solve from the previous command while bounding the final command around measured feedback.
- **A candidate crosses a position limit** → Clamp the final command inside the URDF range with the configured safety margin.

## Migration Plan

Existing Quest declarations omit `ik_backend_type` and continue receiving plain `PinkIK`. Rollback removes the one OpenArm blueprint argument and custom backend module; no serialized state or hardware configuration changes.

## Open Questions

None.

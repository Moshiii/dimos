## Context

GraspGenX is an import-safe, dedicated-worker module implementing `GraspGenSpec.propose_grasps(PointCloud2) -> GraspCandidateArray`. Object perception already implements `ObjectSceneRegistrationSpec`, including stable-ID/name point-cloud lookup, and emits world-frame `DetObject` instances consumed by `PickAndPlaceModule`.

The current `pick` skill already owns planning, execution, gripper control, perception obstacle integration, and `place_back` state, but `_generate_grasps_for_pick` produces a single hand-authored pose. Its sequence begins moving after only the pre-grasp plan succeeds, uses fixed waits for gripper commands, and reports success without checking whether an object prevented full closure. The final approach necessarily intersects the target, so planning-scene suppression is the wrong abstraction; the motion primitive should express intentional unchecked contact directly.

The natural missing seam is orchestration inside `PickAndPlaceModule`:

```text
object name / id
       |
       v
ObjectSceneRegistrationSpec -----> object PointCloud2 (world)
       |
       v
GraspGenSpec --------------------> ranked TCP candidates
       |
       v
PickAndPlaceModule
  resolve -> validate -> feasibility gate -> execute -> verify -> retreat
       |                                           |
       +------ planning world / coordinator -------+
```

The model worker remains independent and optional. The high-level skill owns the transaction because it already owns manipulation state and can guarantee cleanup across planning, gripper, and obstacle mutations.

## Goals / Non-Goals

**Goals:**

- Make `pick` a complete learned-grasp pipeline with precise, testable phase and failure semantics.
- Reuse the existing perception, proposal, planning, coordinator, and `SkillResult` interfaces.
- Reject bad candidates before physical motion where possible.
- Preserve collision-aware motion to pre-grasp while leaving the planning scene unchanged.
- Verify physical closure using gripper feedback on the initial xArm integration.
- Keep the high-level agent interface stable.

**Non-Goals:**

- Retraining, fine-tuning, or changing GraspGenX inference.
- Adding grasp-quality calibration across proposal backends.
- Visual-servoing or closed-loop pose correction during the final approach.
- Force/torque-based grasp verification, slip detection, or automatic regrasp after contact.
- General attached-object collision geometry during subsequent place motions; that should be a follow-up capability.
- Enabling GraspGenX on every manipulator blueprint in the first change.

## Decisions

### 1. Deepen `PickAndPlaceModule` instead of adding a peer skill container

`PickAndPlaceModule` will declare injected perception and grasp proposal Spec attributes and will orchestrate the transaction behind its existing `pick` skill. This keeps planning state, obstacle state, execution, gripper control, and `_last_pick_pose` under one lifecycle owner.

Alternative: create a separate `GraspPipelineSkillContainer` that calls manipulation RPCs. Rejected because the current manipulation API has no transaction-level Spec, would expose partially coordinated state across RPC threads, and would duplicate cleanup and error translation.

### 2. Preserve `pick` as the public skill

The existing signature remains `pick(object_name, object_id=None, robot_name=None)`. Internally, candidate generation becomes a provider strategy: injected `GraspGenSpec` first, with the existing heuristic generator only when explicitly enabled in `PickAndPlaceModuleConfig`.

Alternative: add `grasp_pick` or `pick_with_graspgenx`. Rejected because agents would have to choose between overlapping high-level skills and the orchestration is backend-independent even though GraspGenX is the first provider.

### 3. Resolve through the perception RPC, not the local detection snapshot

The snapshot remains useful for agent display and obstacle synchronization, but the pipeline obtains the proposal input using `ObjectSceneRegistrationSpec` by stable ID or unique name. The returned point cloud and proposal header must match the configured planning frame (initially `world`). Frame mismatch is an error; this change does not add a hidden TF dependency.

Name-only lookup must first establish uniqueness from the current detection snapshot. This avoids the existing perception RPC's “first matching name” behavior silently selecting the wrong duplicate.

### 4. Separate candidate feasibility from physical execution

Candidates remain in generator score order. For each candidate up to `max_grasp_candidates_to_check`, the pipeline:

1. validates finite rigid-pose data and frame agreement;
2. derives pre-grasp from a fixed configured offset along the approach axis and derives retreat by combining the configured backward offset with a world-Z lift bias;
3. checks collision-aware connected feasibility to pre-grasp and collision-free sequential IK reachability for grasp and retreat without dispatching motion;
4. chooses the first candidate passing the gate.

The actual plans are regenerated from live state immediately before each phase because stored plans become stale after execution. A planning or execution failure after motion begins terminates the transaction rather than jumping to another candidate from a changed robot state.

Pre-grasp is the candidate grasp pose retracted by `grasp_pre_grasp_offset` along the configured grasp approach axis. It must pass collision-aware IK and the connected approach-path check. The shipped xArm learned-pick configuration uses a fixed 25 cm offset. Failures remain `pre_grasp_infeasible` so public rejection reporting stays stable.

Retreat preserves the grasp-local backward displacement defined by `grasp_retreat_offset`, then adds `grasp_retreat_lift_offset` in world Z. The xArm default is a 10 cm pullback plus a 1 cm lift, avoiding a purely horizontal drag without replacing the natural retraction direction.

Alternative: attempt candidates sequentially and retry after any failure. Rejected because after the first approach the robot is no longer at the common evaluated start state, making retry safety and cleanup ambiguous.

### 5. Represent intentional contact as unchecked linear motion

`ManipulationModule.plan_linear` generates a straight TCP trajectory and requires an explicit collision policy; it never executes the plan. A private execution helper composes it with preview, dispatch, and completion handling. The pick pipeline uses that helper with collision checking disabled only for pre-grasp-to-grasp and grasp-to-retreat; motion to pre-grasp remains collision-aware. Placement uses the same boundary: approach to pre-place is collision-aware, while lowering into contact and retracting after release are unchecked linear motions. Unchecked trajectories are built from chained collision-disabled IK samples instead of invoking the scene-backed Cartesian planner, making the bypass independent of backend collision behavior.

The RoboPlan Cartesian configuration and `plan_linear` default collision checking on. The agent-facing `move_relative` skill deliberately selects the unchecked policy and makes that fact explicit in its name-independent schema, warnings, and success result. It accepts only a world-frame delta and preserves the current TCP orientation. The target and all other obstacles remain registered throughout the transaction, so no suppression, restoration, or obstacle-monitor race handling is required. Unchecked means planning-scene collision queries are bypassed; finite inputs, Cartesian tracking, kinematic feasibility, joint limits, timing, controller acceptance, and execution-result handling remain enforced.

Alternative: temporarily remove the target obstacle. Rejected because it mutates shared planning-scene state to encode a property of one motion segment and introduces restoration and concurrent-refresh failure modes.

### 6. Model the pipeline as an explicit transaction

A private transaction object records the selected object, proposal source, current phase, candidate rank/score, and closure state. A module lock rejects concurrent `pick` calls. The phases are:

```text
RESOLVE -> PROPOSE -> SELECT -> PREPARE -> APPROACH -> GRASP
                                                   |
                                                   v
                                      CLOSE -> VERIFY -> RETREAT -> DONE
```

No automatic rollback motion is promised. Before gripper closure, failures leave the gripper state explicit in the result. After closure, failures never auto-open the gripper because an object may be held.

### 7. Use feedback-based closure verification with robot-specific configuration

The initial xArm blueprint configures:

- open and closed command endpoints in the units already expected by the coordinator path;
- a held-object closure threshold and comparison direction;
- command and verification timeouts plus polling interval.

The pipeline first checks the close command result, then polls `get_gripper`. Reaching the empty-closed region is a verification failure; remaining beyond the held threshold is success. Configuration validation ensures the threshold lies between the open and closed endpoints.

This is a contact proxy, not proof against slip. The result should say “grasp verified by gripper closure feedback,” not claim force or object identity verification.

Alternative: fixed sleep followed by unconditional success. Rejected because it cannot distinguish an accepted command from a successful physical pick.

### 8. Return structured phase-specific failures

Extend manipulation errors with at least:

- `GRASP_PROVIDER_UNAVAILABLE`
- `GRASP_INPUT_INVALID`
- `GRASP_FRAME_MISMATCH`
- `GRASP_VERIFICATION_FAILED`
- `PICK_BUSY`

Existing `OBJECT_NOT_DETECTED`, `GRASP_GENERATION_FAILED`, `GRASP_ATTEMPTS_EXHAUSTED`, `PLANNING_FAILED`, `GRIPPER_FAILED`, execution errors, and timeouts remain applicable. Human-readable details include phase, candidate rank/score when selected, and whether the gripper may hold an object.

## Risks / Trade-offs

- [Single-view point clouds can produce geometrically plausible but poor grasps] → retain score ordering, validate scene feasibility, expose candidate rank/score, and leave visual servoing/regrasp for follow-up.
- [Gripper aperture is an imperfect grasp signal, especially for thin objects] → make thresholds robot-specific, test boundary behavior, and describe verification as a closure proxy.
- [Planning feasibility checks may be expensive across many GPU proposals] → cap candidates checked, stop at the first feasible candidate, and record rejection metrics for tuning.
- [A fixed pre-grasp distance may reject candidates that a shorter pose could reach] → keep candidate fallback and expose `pre_grasp_infeasible` rejection counts for tuning.
- [Unchecked contact motion can intersect more than the selected object] → constrain pipeline use to short generated contact legs, retain collision-aware approach planning, and label the general-purpose `move_relative` skill as unsafe in both its tool schema and agent prompt.
- [The held object is not modeled as attached geometry] → explicitly defer payload collision geometry and keep subsequent general-purpose planning collision-aware.
- [GraspGenX increases GPU memory and startup time] → retain a dedicated worker, lazy optional runtime imports, and blueprint-level opt-in.
- [Planning can still fail after an earlier feasibility gate because the robot/world changed] → regenerate plans from live state and stop safely rather than retrying from an unanalysed state.

## Migration Plan

1. Add configuration, error codes, and private transaction/candidate helpers behind the unchanged `pick` signature.
2. Add collision-policy control to RoboPlan Cartesian paths, the plan-only `plan_linear` RPC, and the explicitly unsafe `move_relative` skill.
3. Wire the perception and proposal Specs into `PickAndPlaceModule`; use linear contact legs and keep heuristic fallback explicitly enabled in legacy blueprints during transition.
4. Add a distinct GraspGenX-enabled xArm perception blueprint with xArm-specific gripper sweep/TCP and verification configuration, then compose its agentic variant. Keep the existing blueprint dependency footprint unchanged.
5. Validate in deterministic unit tests, recorded/replay perception, MuJoCo where sensor support permits, and finally real xArm hardware with a guarded test matrix.
6. Update the agent prompt, blueprint registry if a new runnable blueprint is introduced, and manipulation documentation.

Rollback is blueprint-level: remove the GraspGenX module and restore explicit heuristic fallback. The public `pick` signature does not require caller migration.

## Open Questions

- What xArm closure threshold has been validated for the physical gripper, and does it need object-width-aware tolerance?
- Should candidate selection reuse the runtime sampled-IK contact-path generator for hypothetical-start dry runs, rather than its current endpoint-only unchecked IK gate?
- Which approach axis encoded by the GraspGenX TCP transform should define pre-grasp and retreat offsets for the configured xArm gripper?

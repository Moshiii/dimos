## 1. Contracts and Configuration

- [x] 1.1 Add phase-specific grasp pipeline error codes to `ManipulationSkillError` and unit-test their `SkillResult` serialization/logging behavior.
- [x] 1.2 Add validated `PickAndPlaceModuleConfig` fields for provider fallback, planning frame, input age, candidate limit, pre-grasp/retreat offsets, and gripper-feedback verification.
- [x] 1.3 Declare optional injected `ObjectSceneRegistrationSpec` and `GraspGenSpec` dependencies on `PickAndPlaceModule`, and add blueprint build tests for present, absent, and ambiguous providers.
- [x] 1.4 Define private typed transaction, phase, candidate, rejection, and verification-result models so state that changes together is not stored in parallel fields.

## 2. Intentional Contact Motion

- [x] 2.1 Add a default-on collision policy to RoboPlan Cartesian path configuration and skip DimOS post-validation only when explicitly disabled.
- [x] 2.2 Add a plan-only `plan_linear` RPC with explicit collision policy and an execution helper that retains Cartesian tracking, joint limits, timing, and execution-result handling.
- [x] 2.3 Keep motion to pre-grasp collision-aware and use unchecked linear motion only for grasp and retreat contact legs.
- [x] 2.4 Remove object-suppression state and APIs from the pick transaction and obstacle monitors.

## 3. Object Resolution and Candidate Selection

- [x] 3.1 Implement unique object resolution by stable ID or unambiguous current name, returning actionable failures without motion.
- [x] 3.2 Retrieve and validate the selected object's point cloud through `ObjectSceneRegistrationSpec`, including non-empty data, timestamp age, and planning-frame checks.
- [x] 3.3 Call `GraspGenSpec.propose_grasps`, validate the candidate-array header and poses, and preserve stable descending generator-score order.
- [x] 3.4 Derive pre-grasp and retreat targets from the configured TCP approach axis and candidate pose.
- [x] 3.5 Implement no-motion feasibility gating for pre-grasp, grasp, and retreat targets, capped by configuration and reporting rejection counts by reason.
- [x] 3.6 Retain the heuristic generator only behind explicit fallback configuration and identify the selected proposal source in results.
- [x] 3.7 Add unit tests for duplicate names, ID prefixes, missing/stale/wrong-frame clouds, provider failures, malformed candidates, stable score ties, candidate limits, and lower-ranked feasible selection.

## 4. Pick Transaction and Verification

- [x] 4.1 Add a single-active-pick guard that rejects concurrent transactions without mutating robot, gripper, proposal, or obstacle state.
- [x] 4.2 Implement the `PREPARE`, `APPROACH`, `GRASP`, `CLOSE`, `VERIFY`, and `RETREAT` phase runner behind the existing `pick` signature.
- [x] 4.3 Check every planning, execution, wait, and gripper-command result; regenerate motion plans from live state at each phase and terminate after the first post-motion failure.
- [x] 4.4 Replace fixed grasp sleeps with timeout-bounded gripper feedback polling and robot-specific held/empty threshold evaluation.
- [x] 4.5 Preserve a closed gripper on every post-closure failure, include “object may be held” context, and store `_last_pick_pose` only after verified closure and successful retreat.
- [x] 4.6 Leave the planning scene unchanged on every pick exit path.
- [x] 4.7 Add phase-by-phase unit tests for success, command rejection, planning/execution failure, timeout, empty close, retreat failure, and concurrent calls.

## 5. Blueprint and Agent Integration

- [x] 5.1 Define reviewed xArm sweep-volume, grasp-frame-to-TCP, approach-axis, and closure-verification configuration without performing model or hardware work at import time.
- [x] 5.2 Add a distinct GraspGenX-enabled xArm perception blueprint and agentic blueprint that compose exactly one perception provider, proposal provider, manipulation module, MCP server, and MCP client.
- [x] 5.3 Keep existing xArm perception blueprints free of the `graspgenx` runtime requirement and explicitly configure their intended heuristic fallback behavior.
- [x] 5.4 Update the manipulation agent prompt to keep `pick` as the sole high-level pick tool and document exact-name/object-ID disambiguation plus safe recovery.
- [x] 5.5 Regenerate `dimos/robot/all_blueprints.py` through `test_all_blueprints_generation.py` and verify the new names appear in `dimos list`.

## 6. End-to-End Validation and Documentation

- [x] 6.1 Add integration tests with fake perception, proposal, planner/coordinator, and gripper feedback providers that exercise the full RPC/Spec-wired pipeline.
- [x] 6.2 Add a replay or fixture-based test proving object/proposal/planning frame consistency with real `PointCloud2` and `GraspCandidateArray` messages.
- [ ] 6.3 Validate the GraspGenX-enabled blueprint startup and one successful/one infeasible candidate flow with the `graspgenx` extra on a GPU-capable environment.
- [ ] 6.4 Calibrate and record the xArm empty-close versus held-object threshold across representative object widths before enabling verification on hardware.
- [ ] 6.5 Run guarded real-xArm tests for success, empty grasp, unreachable proposals, execution interruption, and retreat failure; verify the gripper never auto-opens after closure.
- [x] 6.6 Update manipulation capability documentation with architecture, configuration, failure semantics, and the distinction between closure-proxy verification and force/slip verification.
- [x] 6.7 Run focused pytest suites, blueprint-generation validation, formatting/lint checks, and mypy on touched modules.

## 7. Linear Contact Cleanup

- [x] 7.1 Cover default collision rejection and explicit collision bypass in RoboPlan Cartesian unit tests.
- [x] 7.2 Cover `plan_linear`, sequential unchecked IK selection, and pick phase failure behavior with hermetic unit tests.
- [x] 7.3 Remove suppression-specific tests and verify no production suppression references remain.
- [x] 7.4 Run focused manipulation tests, formatting/lint checks, and mypy for the cleanup.

## 8. Explicit Unchecked Control and Placement

- [x] 8.1 Expose `move_relative` as an agent skill whose schema and result explicitly state that planning-scene collision checking is disabled.
- [x] 8.2 Use unchecked linear motion for place lowering and retract while keeping the pre-place approach collision-aware.
- [x] 8.3 Update unit tests, the manipulation agent prompt, and capability documentation for the split `plan_linear`/`move_relative` interface.

## 9. Fixed Grasp Clearance

- [x] 9.1 Use a fixed configurable pre-grasp offset and set the xArm simulation pipeline to 25 cm.
- [x] 9.2 Preserve grasp-local backward retreat and add a configurable 1 cm world-Z lift bias.
- [x] 9.3 Cover fixed pre-grasp geometry, candidate fallback, and retreat geometry with hermetic unit tests and update documentation.

## 10. Backend-Independent Unchecked Contact Motion

- [x] 10.1 Route collision-disabled `plan_linear` calls through chained unchecked IK samples instead of the scene-backed Cartesian planner.
- [x] 10.2 Preserve kinematic feasibility, joint limits, requested speed bounds, preview, execution, and result handling for the unchecked path.
- [x] 10.3 Add regression coverage proving unchecked grasp/retreat motion does not invoke Cartesian scene planning, and update capability documentation.

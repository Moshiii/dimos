## Why

GraspGenX can now produce ranked grasp poses, but the agent-facing `pick` skill still uses one heuristic pose and has no end-to-end path from a detected object to a verified physical pick. A pipeline is needed now to connect object point-cloud lookup, learned proposals, motion feasibility, collision-world handling, gripper actuation, and clear recovery semantics.

## What Changes

- Upgrade the existing `pick` skill to resolve a unique detected object and obtain its world-frame point cloud through the perception RPC interface.
- Request ranked TCP grasp candidates through `GraspGenSpec`, preserving the generator's score order while rejecting invalid or motion-infeasible candidates.
- Execute a safe pick sequence: pre-grasp approach, gripper open, grasp, gripper close, and retreat.
- Keep the planning scene unchanged: plan collision-aware motion to pre-grasp, then use explicitly unchecked linear TCP motion for the grasp and retreat contact legs.
- Use a fixed 25 cm pre-grasp distance along the grasp approach axis, and bias the normal backward retreat upward by 1 cm to reduce table rubbing.
- Split linear control into plan-only `plan_linear` and an explicitly unsafe agent-facing `move_relative` command, and use the same unchecked execution path for place lowering/retract.
- Verify grasp closure using configured gripper feedback when available and return structured failure codes that distinguish perception, proposal, feasibility, execution, and verification failures.
- Keep the heuristic grasp path available only as an explicit configuration fallback for blueprints that do not include a grasp proposal module.
- Add a GraspGenX-enabled xArm perception manipulation blueprint with gripper-specific configuration, leaving the existing non-GPU blueprint available, and update the manipulation agent prompt to describe the learned-pick behavior.

## Capabilities

### New Capabilities

- `grasp-pipeline-skill`: End-to-end behavior and failure semantics for resolving an object, proposing and selecting feasible grasps, executing a pick, and verifying the result.

### Modified Capabilities

None.

## Impact

- Affected modules: `ManipulationModule`, the RoboPlan Cartesian adapter, `PickAndPlaceModule`, `GraspGenSpec`, `ObjectSceneRegistrationSpec`, manipulation error types, and xArm perception blueprints/prompts.
- The new xArm learned-pick blueprint requires the `graspgenx` optional dependency, a dedicated GraspGenX worker, and robot-specific sweep-volume/TCP configuration; existing xArm blueprints remain runnable without that extra.
- Existing `pick(object_name, object_id, robot_name)` callers remain source-compatible; observed candidate selection and failure results become more precise.
- No change is proposed to GraspGenX inference itself or to generic motion-planner algorithms.

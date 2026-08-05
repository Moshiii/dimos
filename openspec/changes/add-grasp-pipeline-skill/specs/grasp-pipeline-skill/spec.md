## ADDED Requirements

### Requirement: Unique object resolution
The `pick` skill SHALL resolve exactly one detected object before requesting grasp proposals. It SHALL prefer an explicitly supplied stable object ID, SHALL reject ambiguous ID prefixes or names, and SHALL return `OBJECT_NOT_DETECTED` when no object matches.

#### Scenario: Object ID selects one of several same-named objects
- **WHEN** the caller supplies an object ID that uniquely identifies one detected object
- **THEN** the pipeline uses that object's point cloud regardless of other objects with the same name

#### Scenario: Object name is ambiguous
- **WHEN** the caller supplies only a name and multiple current detections match it
- **THEN** the pipeline performs no robot motion and returns a failure that asks the caller to provide an object ID

### Requirement: Proposal input and frame contract
The pipeline SHALL retrieve the selected object's `PointCloud2` through `ObjectSceneRegistrationSpec`, SHALL reject empty or stale input according to configured limits, and SHALL require the proposal frame to match the manipulation planning frame. It MUST NOT silently interpret a candidate in a different frame.

#### Scenario: Valid world-frame point cloud
- **WHEN** perception returns a non-empty, sufficiently recent object point cloud in the manipulation planning frame
- **THEN** the pipeline passes that cloud unchanged to `GraspGenSpec.propose_grasps`

#### Scenario: Proposal frame differs from planning frame
- **WHEN** the returned candidate array identifies a frame other than the configured manipulation planning frame
- **THEN** the pipeline performs no robot motion and returns a frame-mismatch failure

### Requirement: Ranked feasibility selection
The pipeline SHALL examine candidates in descending generator-score order, up to a configurable attempt limit. It SHALL derive pre-grasp using a fixed configured offset along the grasp retraction direction and validate the connected collision-aware approach path. It SHALL reject non-finite or malformed poses, candidates without a valid pre-grasp approach, and candidates whose grasp or retreat fails sequential unchecked IK reachability. Generator scores SHALL be treated only as relative ranking values, not calibrated probabilities.

#### Scenario: Highest-scored candidate is infeasible
- **WHEN** the first candidate cannot satisfy approach or grasp feasibility and a lower-scored candidate can
- **THEN** the pipeline selects the first feasible lower-scored candidate without moving for the rejected candidate

#### Scenario: No candidate is feasible
- **WHEN** every candidate within the configured attempt limit fails validation or feasibility
- **THEN** the pipeline performs no gripper closure, leaves the robot in a safe pre-pick state, and returns `GRASP_ATTEMPTS_EXHAUSTED` with rejection counts by reason

#### Scenario: Fixed pre-grasp approach is feasible
- **WHEN** the pose 25 cm behind the grasp along the configured approach axis has a collision-aware connected path
- **THEN** the pipeline uses that pose as the candidate's pre-grasp

#### Scenario: Fixed pre-grasp approach is infeasible
- **WHEN** the fixed pre-grasp pose or its connected approach path is infeasible
- **THEN** the candidate is rejected as `pre_grasp_infeasible` without robot motion

### Requirement: Intentional linear contact motion
The pipeline SHALL leave the planning scene unchanged. Motion to pre-grasp SHALL remain collision-aware. Motion from pre-grasp to grasp and from grasp to retreat SHALL follow straight TCP paths generated from sequential collision-disabled IK samples, without invoking scene-backed Cartesian planning or collision validation, while retaining kinematic tracking, joint limits, timing, and execution-result validation.

#### Scenario: Target remains registered as an obstacle
- **WHEN** the selected object is present in the planning world as a perception obstacle
- **THEN** the pipeline does not remove or suppress that object or any other planning obstacle

#### Scenario: Contact motion bypasses collision rejection
- **WHEN** a reachable straight pre-grasp-to-grasp or grasp-to-retreat trajectory intersects registered collision geometry
- **THEN** the linear contact primitive may execute it without collision queries while still enforcing non-collision motion constraints

#### Scenario: Retreat avoids table rubbing
- **WHEN** the robot retreats after closing on an object
- **THEN** the retreat target combines the configured grasp-local backward offset with the configured positive world-Z lift offset while preserving grasp orientation

#### Scenario: Place retract starts in contact
- **WHEN** placement reaches the object pose, releases the gripper, and the current configuration intersects registered object geometry
- **THEN** the pipeline retracts to pre-place with unchecked linear motion instead of invoking collision-aware path planning from the contact state

### Requirement: Explicit unchecked relative control
The manipulation module SHALL expose `plan_linear` as a plan-only interface with collision checking enabled by default. It SHALL expose `move_relative` as an agent skill that accepts world-frame displacement, always disables planning-scene collision checking, and explicitly labels that behavior as unsafe in its tool description and result. The unchecked skill SHALL continue to enforce finite input, kinematic feasibility, joint and controller limits, timing, and execution-result validation.

#### Scenario: Agent inspects unchecked motion tool
- **WHEN** the agent receives the `move_relative` tool schema
- **THEN** the description states that collision checking is disabled and that the command may collide with the robot, environment, objects, or people

#### Scenario: Relative unchecked motion succeeds
- **WHEN** the caller supplies a finite, non-zero world-frame displacement with a feasible Cartesian trajectory
- **THEN** the module preserves TCP orientation, executes with collision checking disabled, and reports the disabled collision policy in the success result

### Requirement: Safe pick execution
For a selected feasible candidate, the pipeline SHALL execute the ordered phases `PREPARE`, `APPROACH`, `GRASP`, `CLOSE`, `VERIFY`, and `RETREAT`. `APPROACH` SHALL use collision-aware planning; `GRASP` and `RETREAT` SHALL use unchecked linear TCP motion. It SHALL stop at the first failed phase, SHALL report that phase in the result, and MUST NOT open the gripper automatically after closure because the robot may be holding the object.

#### Scenario: Successful pick sequence
- **WHEN** all motion plans execute, gripper commands are accepted, verification succeeds, and retreat completes
- **THEN** the skill returns success including the selected candidate rank and score and stores the grasp pose for `place_back`

#### Scenario: Retreat fails after closure
- **WHEN** gripper closure and verification succeed but retreat planning or execution fails
- **THEN** the skill returns a retreat failure, leaves the gripper closed, and reports that the object may still be held

### Requirement: Grasp verification
The learned-pick blueprint SHALL configure gripper-feedback verification. Verification SHALL poll feedback until a configurable timeout and SHALL distinguish an object-blocked closure from a fully closed empty gripper using robot-specific command units and thresholds.

#### Scenario: Feedback indicates an object is held
- **WHEN** the final gripper position remains on the configured held-object side of the closure threshold before timeout
- **THEN** verification succeeds and the pipeline proceeds to retreat

#### Scenario: Feedback indicates an empty close
- **WHEN** the gripper reaches the configured empty-closed region
- **THEN** the pipeline returns `GRASP_VERIFICATION_FAILED`, leaves the gripper closed, and does not report a successful pick

### Requirement: Explicit heuristic fallback
Blueprints without a grasp proposal provider SHALL fail learned-pick requests by default. A blueprint MAY explicitly enable the existing heuristic pose generator as a fallback, and the skill result SHALL identify when that fallback was used.

#### Scenario: Grasp provider is unavailable and fallback is disabled
- **WHEN** `pick` is called without an injected `GraspGenSpec`
- **THEN** the skill performs no motion and returns `GRASP_PROVIDER_UNAVAILABLE`

#### Scenario: Grasp provider is unavailable and fallback is enabled
- **WHEN** `pick` is called without an injected `GraspGenSpec` on a blueprint that explicitly enables heuristic fallback
- **THEN** the pipeline uses the heuristic candidate path and identifies the proposal source in its result

### Requirement: Single active pick transaction
The module SHALL allow at most one pick pipeline transaction at a time. A concurrent request SHALL be rejected without changing motion, gripper, proposal, or planning-scene state.

#### Scenario: Concurrent pick request
- **WHEN** a second `pick` call arrives while another pick transaction is active
- **THEN** the second call returns a busy failure and does not enter any pipeline phase

### Requirement: Blueprint and agent exposure
The xArm perception manipulation stack SHALL compose `PickAndPlaceModule`, `ObjectSceneRegistrationModule`, and `GraspGenXModule` with compatible world-frame and gripper/TCP configuration. The agent prompt SHALL continue to expose `pick` as the single high-level picking skill and SHALL describe its object-ID disambiguation and failure recovery behavior.

#### Scenario: Learned-pick blueprint is built
- **WHEN** the xArm learned-pick blueprint is constructed with the `graspgenx` extra installed
- **THEN** blueprint Spec injection resolves one perception provider and one grasp proposal provider for the pick module

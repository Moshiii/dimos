## ADDED Requirements

### Requirement: Prompted localization provides an in-memory object cloud
The system SHALL expose an internal typed localization RPC that accepts an object text query and returns either a world-frame `PointCloud2` for the selected object instance or `None`.

#### Scenario: Object is localized
- **WHEN** prompted localization reconstructs one or more detections matching the query
- **THEN** the RPC returns the point cloud of the detection containing the greatest number of reconstructed 3D points

#### Scenario: Object is not localized
- **WHEN** prompted localization cannot reconstruct a usable detection matching the query
- **THEN** the RPC returns `None`

### Requirement: Localization uses recent active recording data
Each prompted localization request SHALL search the latest 30 seconds available in the configured active Memory2 recording.

#### Scenario: Pick requests fresh localization
- **WHEN** two pick requests are made at different times
- **THEN** each request performs localization against the latest available 30-second recording window at the time of that request

### Requirement: Runtime and debug localization share processing behavior
The runtime localizer and standalone localization tool SHALL use the same core retrieval, prompted detection, segmentation, depth projection, and best-detection selection implementation.

#### Scenario: Equivalent inputs are processed
- **WHEN** the runtime localizer and standalone tool receive the same query, recording streams, transforms, and time range
- **THEN** they select the same reconstructed object cloud before debug-only output handling

### Requirement: Pick accepts a prompted target source
The prompted picking deployment SHALL resolve the existing `pick(object_name)` argument through the prompted localizer and SHALL pass the resulting `PointCloud2` directly to the existing grasp proposal and execution pipeline without a filesystem handoff.

#### Scenario: Prompted target proceeds through the grasp pipeline
- **WHEN** `pick` receives an object name and prompted localization returns a usable point cloud
- **THEN** the system uses that cloud for grasp proposal, reachability evaluation, motion planning, grasp selection, visualization, and execution

#### Scenario: Existing candidate evaluation remains unchanged
- **WHEN** grasp candidates are generated from a prompted target cloud
- **THEN** the system applies the existing IK-first filtering, candidate planning, selection, and execution behavior

### Requirement: Localization failure prevents manipulation
The prompted picking deployment SHALL stop the pick transaction before grasp generation or robot motion when localization returns no usable point cloud or reports a localization failure.

#### Scenario: No matching cloud is returned
- **WHEN** the prompted localizer returns `None`
- **THEN** `pick` reports a localization failure and does not invoke grasp generation, motion planning, arm motion, or gripper motion

#### Scenario: Localization raises an error
- **WHEN** the prompted localizer cannot complete because its processing pipeline fails
- **THEN** `pick` reports a localization failure and does not try another target provider or invoke manipulation

### Requirement: Target-provider selection is explicit
Each manipulation blueprint SHALL select either its existing object-scene target source or the prompted localizer target source through module composition, and the pick runtime MUST NOT automatically fall back between providers.

#### Scenario: Existing blueprint uses object-scene lookup
- **WHEN** an existing object-scene-based manipulation blueprint handles `pick`
- **THEN** it retains its current object lookup, validation, and target suppression behavior without invoking prompted localization

#### Scenario: Prompted blueprint uses prompted localization
- **WHEN** the prompted xArm6 blueprint handles `pick`
- **THEN** it invokes prompted localization without first invoking or falling back to object-scene lookup

### Requirement: Prompted clouds remain non-authoritative for collision
The prompted target cloud SHALL be used as grasp input and visualization data but MUST NOT be inserted into the planner collision scene by this integration.

#### Scenario: Prompted target is visualized and planned toward
- **WHEN** prompted localization returns a target cloud
- **THEN** the visualization displays the target and grasp evaluation while collision checking continues to use only the blueprint's existing configured obstacles

#### Scenario: Prompted pick has no target object identity
- **WHEN** the prompted pick pipeline begins planning
- **THEN** it does not perform object-ID lookup or target-obstacle suppression for the prompted cloud

### Requirement: Real xArm6 prompted-pick blueprint is directly testable
The system SHALL provide a real xArm6 blueprint that composes active Memory2 recording, prompted localization, the grasp pipeline, manipulation visualization, xArm6 arm and gripper control, and `McpServer` without an LLM agent.

#### Scenario: Blueprint exposes the existing pick skill
- **WHEN** the prompted xArm6 blueprint is running
- **THEN** a user can invoke `pick` through `dimos mcp call pick --arg object_name="<query>"`

#### Scenario: Blueprint uses the supported xArm6 grasp configuration
- **WHEN** the prompted xArm6 blueprint is built
- **THEN** it configures xArm6 hardware with gripper support, a gripper-enabled robot model, `XARM_GRIPPER_SWEEP`, and `XARM_GRASP_FRAME_TO_TCP`

#### Scenario: Existing perception blueprint is unchanged
- **WHEN** the prompted-pick blueprint is added
- **THEN** the existing `xarm6-worldbelief` blueprint retains its current perception and debugging composition

### Requirement: Prompted picking performs no automatic pre-scan
The prompted picking deployment MUST NOT move the robot or camera to collect additional views before localization.

#### Scenario: Pick begins with localization
- **WHEN** a user invokes prompted `pick`
- **THEN** the first physical-object processing step searches the existing recording window without commanding pre-scan motion

### Requirement: Pick execution can be disabled after planning
The pick pipeline SHALL provide a configuration flag that preserves localization, grasp generation, reachability evaluation, full candidate planning, selection, and visualization while disabling physical arm and gripper execution after a candidate is selected.

#### Scenario: Plan-only prompted pick succeeds
- **WHEN** `execute_pick` is disabled and the prompted pipeline selects a fully planned grasp
- **THEN** `pick` returns success with the selected candidate and `planning_only=true` metadata without commanding arm motion, gripper motion, or trajectory execution

#### Scenario: Plan-only pick does not claim an object was picked
- **WHEN** a plan-only pick completes
- **THEN** the system does not update the last physical pick pose or report that the object is held

#### Scenario: Combined operation stops before place
- **WHEN** `pick_and_place` receives a successful plan-only pick result
- **THEN** it returns that result without invoking the physical place phase

#### Scenario: Execution remains compatible by default
- **WHEN** `execute_pick` is not explicitly disabled
- **THEN** the selected grasp follows the existing physical execution sequence

## ADDED Requirements

### Requirement: Automatic object-cloud visualization
The `pick` pipeline SHALL publish the validated segmented object point cloud as the display-only layer `grasp/object-cloud` through its initialized `VisualizationSpec`. It MUST NOT add, remove, or modify planning-world collision geometry while publishing the layer.

#### Scenario: Provider input is accepted
- **WHEN** the pipeline validates a segmented object cloud for a pick transaction and visualization is enabled
- **THEN** it replaces `grasp/object-cloud` with that cloud in the manipulation planning frame

#### Scenario: Visualization is disabled
- **WHEN** the pipeline validates a segmented object cloud without an initialized visualization backend
- **THEN** proposal generation, planning, and execution continue without visualization work

### Requirement: Candidate evaluation visualization
The pipeline SHALL visualize only candidates within the configured feasibility-check limit. It SHALL represent rejected candidates in red, the candidate currently under evaluation in yellow, and pending candidates in gray. After a candidate passes the complete connected pre-grasp, grasp, and retreat planning check, the pipeline SHALL replace `grasp/proposals` with only that selected candidate in green.

#### Scenario: Candidate evaluation begins
- **WHEN** the pipeline begins validating a ranked candidate
- **THEN** `grasp/proposals` identifies that candidate as current, earlier rejected candidates as rejected, and later candidates as pending

#### Scenario: Lower-ranked candidate is selected
- **WHEN** one or more candidates fail and a later candidate passes the complete connected planning check
- **THEN** the pipeline replaces `grasp/proposals` with only the selected lower-ranked candidate in green before physical execution begins

#### Scenario: Every checked candidate fails
- **WHEN** every candidate within the feasibility-check limit is rejected
- **THEN** `grasp/proposals` retains the evaluated candidates in red

### Requirement: Provider-independent visualization
Heuristic and learned `GraspGenSpec` providers SHALL use the same object-cloud and candidate-state visualization flow. The proposal layer SHALL use robot-specific gripper sweep geometry and the configured grasp-frame-to-TCP transform rather than provider-specific renderer handles.

#### Scenario: Heuristic provider supplies a candidate
- **WHEN** a heuristic-provider blueprint executes `pick`
- **THEN** the pipeline visualizes its object cloud and candidate through the same stable layer IDs used by a learned-provider blueprint

#### Scenario: Learned provider supplies candidates
- **WHEN** a GraspGenX blueprint executes `pick`
- **THEN** the pipeline visualizes its object cloud and candidates without importing Viser into the provider or pick-selection logic

### Requirement: Best-effort visualization lifecycle
Pipeline visualization SHALL be asynchronous and best-effort. A layer-construction, submission, rendering, disconnection, or closed-backend failure MUST NOT change the pick result or delay candidate evaluation. The final selected or failed layer state SHALL remain available after the skill returns and SHALL be replaced by a subsequent pick using the same stable layer IDs.

#### Scenario: Candidate checks outrun rendering
- **WHEN** planning publishes candidate states faster than the backend renders them
- **THEN** intermediate states may be skipped while the newest submitted state remains the eventual layer value

#### Scenario: Visualization submission fails
- **WHEN** layer construction or `VisualizationSpec.set_layer` raises during a pick
- **THEN** the pipeline logs the visualization failure and continues proposal evaluation, planning, and execution

#### Scenario: Pick completes
- **WHEN** a pick succeeds or fails after publishing grasp layers
- **THEN** the final object-cloud and proposal contents remain visible until a later pick replaces them

### Requirement: Learned xArm Viser exposure
The learned xArm grasp blueprint SHALL configure Viser as its manipulation visualization backend and SHALL expose `grasp/object-cloud` and `grasp/proposals` in the existing hierarchical layer selector.

#### Scenario: Learned xArm stack starts
- **WHEN** the learned xArm grasp blueprint initializes successfully
- **THEN** its manipulation visualization URL refers to Viser and grasp layers appear under the `Grasp` selector group after `pick` publishes them

### Requirement: Shared grasp-layer construction
The standalone grasp visualization demo and live pick pipeline SHALL construct object-cloud and proposal layers through the same backend-neutral functions.

#### Scenario: Demo and pipeline visualize equivalent input
- **WHEN** the demo and pipeline receive the same cloud, candidates, gripper geometry, transform, and candidate states
- **THEN** they produce equivalent `VisualizationLayer` geometry and stable layer identifiers

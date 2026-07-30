## ADDED Requirements

### Requirement: Explicit grasp proposal providers
Every `PickAndPlaceModule` blueprint SHALL compose exactly one `GraspGenSpec` provider. The pick pipeline MUST NOT select an internal heuristic fallback at runtime, and its result SHALL remain independent of the provider implementation.

#### Scenario: Blueprint has no provider
- **WHEN** a blueprint contains `PickAndPlaceModule` without a matching `GraspGenSpec`
- **THEN** blueprint construction fails before the stack starts

#### Scenario: Blueprint has multiple providers
- **WHEN** a blueprint contains more than one matching `GraspGenSpec`
- **THEN** blueprint construction fails as ambiguous before the stack starts

#### Scenario: Provider returns candidates
- **WHEN** the configured provider returns a valid candidate array
- **THEN** the pipeline applies the same ranking, connected planning, execution, and result semantics regardless of the provider implementation

### Requirement: Heuristic grasp proposal provider
The system SHALL provide a lightweight `GraspGenSpec` provider that preserves the existing heuristic grasp algorithm. It SHALL use the proposal input's latest detected center and size for occlusion compensation, tall-object adjustment, and distance-adaptive orientation, and SHALL return one candidate with score `0.0` in the input frame.

#### Scenario: Non-learned xArm stack requests a proposal
- **WHEN** the heuristic provider receives valid current object geometry
- **THEN** it returns the same grasp pose that the former internal heuristic path would have generated

#### Scenario: Heuristic provider receives proposal context
- **WHEN** the proposal input contains both an accumulated object cloud and latest detection geometry
- **THEN** the heuristic calculation uses the latest center and size rather than refitting the accumulated cloud

## MODIFIED Requirements

### Requirement: Proposal input and frame contract
The pipeline SHALL retrieve the selected object's `PointCloud2` through `ObjectSceneRegistrationSpec`, SHALL reject empty or stale input according to configured limits, and SHALL construct one provider-neutral proposal input containing that cloud plus the selected detection's latest center and size. It SHALL require the proposal frame to match the manipulation planning frame and MUST NOT silently interpret a candidate in a different frame.

#### Scenario: Valid world-frame proposal input
- **WHEN** perception returns a non-empty, sufficiently recent object point cloud in the manipulation planning frame and current detection geometry
- **THEN** the pipeline passes the cloud unchanged with the selected detection's center and size to `GraspGenSpec.propose_grasps`

#### Scenario: Proposal frame differs from planning frame
- **WHEN** the returned candidate array identifies a frame other than the configured manipulation planning frame
- **THEN** the pipeline performs no robot motion and returns a frame-mismatch failure

### Requirement: Blueprint and agent exposure
The xArm perception manipulation stacks SHALL compose `PickAndPlaceModule`, `ObjectSceneRegistrationModule`, and exactly one compatible grasp proposal provider. The non-learned real and simulation stacks SHALL use the heuristic provider, while the learned stack SHALL use GraspGenX. The agent prompt SHALL continue to expose `pick` as the single high-level picking skill and SHALL describe its object-ID disambiguation and failure recovery behavior without asking the agent to choose a provider.

#### Scenario: Learned-pick blueprint is built
- **WHEN** the xArm learned-pick blueprint is constructed with the `graspgenx` extra installed
- **THEN** blueprint Spec injection resolves one perception provider and GraspGenX as its only grasp proposal provider

#### Scenario: Non-learned perception blueprint is built
- **WHEN** an xArm real or simulation perception blueprint is constructed without GraspGenX
- **THEN** blueprint Spec injection resolves one perception provider and the heuristic module as its only grasp proposal provider

## REMOVED Requirements

### Requirement: Explicit heuristic fallback
**Reason**: Runtime fallback obscures which proposal algorithm is active and duplicates proposal logic inside the pick orchestrator.

**Migration**: Compose the standalone heuristic `GraspGenSpec` provider in non-learned blueprints or compose GraspGenX in learned blueprints. Remove `heuristic_grasp_fallback` configuration and provider-source result handling.

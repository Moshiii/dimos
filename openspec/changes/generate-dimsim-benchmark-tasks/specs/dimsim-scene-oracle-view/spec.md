## ADDED Requirements

### Requirement: Versioned typed scene oracle view
DimSim SHALL expose a private, read-only, versioned `SceneOracleView` for benchmark generation. The view SHALL identify the scene and semantic-schema revision and SHALL use strict typed records that reject unknown or malformed fields.

#### Scenario: Export a supported scene
- **WHEN** benchmark generation requests the oracle view for a coherently reset supported scene
- **THEN** DimSim returns a typed view containing the scene identity, semantic-schema revision, frame contract, entities, regions, and provenance

#### Scenario: Reject an unsupported semantic schema
- **WHEN** the DimSim integration receives an oracle view with an unsupported schema version or malformed required record
- **THEN** it fails with an actionable compatibility error instead of silently omitting or coercing semantic facts

### Requirement: Authoritative entity semantics
Each entity exposed for benchmark generation SHALL have a stable simulator-owned identifier, one explicit semantic class, agent-facing names or aliases, transform, query geometry, current semantic state where applicable, and provenance for every authored or policy-derived semantic field. The oracle view MUST NOT infer semantic class, state, orientation, or region membership from display-title substring matching.

#### Scenario: Export a stateful television
- **WHEN** the loaded scene contains the benchmark television
- **THEN** its oracle entity record exposes a stable identifier, semantic class `television`, approved names, query geometry, and an explicit canonical power state such as `ON` or `OFF`

#### Scenario: Required semantics are absent
- **WHEN** an entity has geometry or a display title but lacks an authoritative semantic class needed by a generator
- **THEN** the oracle view does not fabricate that class and the dependent task candidate is rejected

### Requirement: Coherent scene state
One `SceneOracleView` SHALL represent one coherent scene reset. Entity transforms, current states, region membership, and derived relationships in the view SHALL refer to the same reset state, and the view SHALL identify the reset or state revision used to produce it.

#### Scenario: Read canonical reset state
- **WHEN** the apartment is reset to its canonical state and its oracle view is exported
- **THEN** the television power value and all entity transforms in the view correspond to that same reset

#### Scenario: Scene state changes during export
- **WHEN** DimSim cannot guarantee a coherent reset state while constructing the oracle view
- **THEN** export fails rather than combining values observed at different scene states

### Requirement: Metric spatial query contract
The oracle view SHALL declare its coordinate frame, handedness, units, gravity axis, and transform convention. Every task-addressable entity SHALL expose geometry sufficient for the supported predicates, including a world-space footprint or bounds suitable for outer-edge and surface-distance calculations. Canonical orientation SHALL be exposed only when authored or produced by a declared versioned policy.

#### Scenario: Compute distance to an object's outer edge
- **WHEN** a generator evaluates a destination or comparison predicate
- **THEN** it can compute the required metric surface distance from declared entity geometry without substituting center-point distance

#### Scenario: Canonical orientation is undeclared
- **WHEN** an asset has a world rotation but no declared model-local canonical front
- **THEN** the oracle view does not claim that a canonical left, right, front, or rear relation is available

### Requirement: Semantic regions and membership
The oracle view SHALL represent semantic regions with stable identifiers, geometry, semantic class, and provenance, and SHALL expose authoritative entity-region membership where such membership is declared. A generator MUST reject region-dependent candidates whose membership is missing, multiple, or boundary-sensitive under the configured tolerance.

#### Scenario: Resolve an entity to one region
- **WHEN** a future region-dependent task examines an entity with one stable authoritative region membership
- **THEN** the oracle view provides the region identifier and the provenance of that membership

#### Scenario: Entity lies on an ambiguous region boundary
- **WHEN** an entity's region membership is not unique within the configured boundary tolerance
- **THEN** the oracle view or consuming generator rejects the candidate rather than choosing a region

### Requirement: Canonical digest and provenance
The DimSim integration SHALL canonically serialize the semantic content of a `SceneOracleView` and compute a content digest over that serialization. Benchmark output SHALL be able to record the digest, scene revision, semantic-schema revision, and policy revisions without treating a copied oracle view as a second source of semantic truth.

#### Scenario: Repeat an unchanged export
- **WHEN** the same DimSim scene revision, canonical reset, semantics, and derivation policies are exported twice
- **THEN** both exports produce the same canonical semantic content digest

#### Scenario: Authoritative semantics change
- **WHEN** an entity class, state, query geometry, region membership, or derivation policy changes
- **THEN** the resulting oracle-view digest or recorded semantic revision changes

### Requirement: Private oracle boundary
The `SceneOracleView` SHALL be exposed only through the private benchmark-generation boundary. It SHALL NOT be published as a normal agent observation, public scene-inspection skill, or public benchmark artifact.

#### Scenario: Build a public benchmark package
- **WHEN** generated public artifacts are scanned
- **THEN** they contain no private oracle view, entity identifiers, semantic provenance, expected answers, or executable oracle contracts

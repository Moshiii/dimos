## ADDED Requirements

### Requirement: Normalized recording progress selection
The preparation and case-authoring interfaces SHALL accept a finite normalized recording progress in the inclusive range `[0, 1]` and resolve it linearly over the sealed recording range. Progress `0` SHALL resolve to the exact recording start, progress `1` SHALL resolve to the exact recording end, and every resolved cutoff SHALL retain the existing inclusive timestamp semantics.

#### Scenario: Select the complete recording
- **WHEN** an author selects normalized progress `1.0`
- **THEN** preparation resolves the cutoff to the exact maximum timestamp across nonempty recording streams and the frozen view includes every source observation

#### Scenario: Select an intermediate point
- **WHEN** an author selects normalized progress strictly between `0` and `1`
- **THEN** preparation resolves the cutoff as recording start plus progress times recording duration and includes only observations at or before that timestamp

#### Scenario: Reject invalid progress
- **WHEN** progress is non-finite or outside the inclusive range `[0, 1]`
- **THEN** case validation fails before map generation or output publication

## MODIFIED Requirements

### Requirement: Reproducible cutoff manifest
Each prepared bundle SHALL include a strict versioned manifest with source identity, derived identity, recording range, mapper settings, and per-cutoff stream and map boundaries. A cutoff selected by normalized progress SHALL retain both its authored progress and exact resolved absolute timestamp, and service selection SHALL use the prepared progress record rather than recomputing against mutable inputs.

#### Scenario: Record cutoff provenance
- **WHEN** a normalized-progress cutoff is prepared
- **THEN** its manifest entry contains authored progress, resolved relative and absolute cutoff time, each visible stream's count and last observation identity, and the selected map observation, timestamp, and integrated frame count

#### Scenario: Detect changed bundle inputs
- **WHEN** the source or derived database size or SHA-256 differs from the manifest at service startup
- **THEN** the service refuses to expose the cutoff

#### Scenario: Select only a prepared cutoff
- **WHEN** an operator or case requests a normalized progress that is absent from the manifest
- **THEN** the service fails and reports the available prepared progress selections

### Requirement: Offline frozen CodePolicy service
The system SHALL provide a standalone MCP service whose CodePolicy session exposes the selected frozen `memory`, omits the live DimOS `app`, and retains `python_exec` as its sole model-facing policy skill. The canonical service SHALL execute without constructing a DimOS module, blueprint, module worker, or coordinator.

#### Scenario: Query bounded memory through MCP
- **WHEN** the service starts for a valid prepared cutoff and policy code queries its streams
- **THEN** the query sees the manifest-bounded source observations and selected derived map through `memory`

#### Scenario: Omit live robot access
- **WHEN** the frozen policy kernel is bootstrapped
- **THEN** `app` is absent from its namespace and frozen configuration cannot select the live environment

#### Scenario: Preserve persistent policy execution
- **WHEN** several `python_exec` calls are made to the same frozen service
- **THEN** imports and variables persist according to the existing CodePolicy session contract while the memory cutoff remains unchanged

#### Scenario: Serve without a DimOS deployment
- **WHEN** the frozen service is started by a QA attempt or developer command
- **THEN** it owns its MCP transport and kernel processes directly and requires no running DimOS blueprint or module coordinator

## ADDED Requirements

### Requirement: Inclusive frozen Memory2 view
The system SHALL expose a frozen Memory2 view that returns only observations whose timestamps are less than or equal to one absolute cutoff, across source and derived streams.

#### Scenario: Include the boundary observation
- **WHEN** a stream contains observations before, exactly at, and after the configured cutoff
- **THEN** the frozen view returns the observations before and at the cutoff and excludes the later observation

#### Scenario: Overlay a derived map stream
- **WHEN** the source recording and derived sidecar contain disjoint stream names
- **THEN** the frozen view lists and queries both sets through one `memory` interface with the same cutoff

#### Scenario: Reject ambiguous overlay streams
- **WHEN** the source recording and derived sidecar contain the same stream name
- **THEN** construction fails rather than shadowing either stream

### Requirement: Read-only recording access
The system SHALL open source and derived SQLite recordings in read-only query mode and SHALL reject append, stream creation, stream deletion, and stream retyping through the frozen memory interface.

#### Scenario: Reject stream mutation
- **WHEN** policy code attempts to append to or delete a frozen stream
- **THEN** the operation fails with an explicit permission error and the database remains unchanged

#### Scenario: Avoid journal side effects
- **WHEN** a sealed SQLite recording is opened and queried read-only
- **THEN** the system does not create WAL or shared-memory sidecar files

#### Scenario: Reject creation in read-only SQLite
- **WHEN** a caller requests a stream that is absent from a read-only SQLite store
- **THEN** the store fails without creating registry or stream tables

### Requirement: Runtime-equivalent map snapshots
The preparation workflow SHALL generate `global_map` snapshots with the production voxel map transformer, configured mapper settings, and runtime emission cadence, and SHALL select the latest emitted map at or before each cutoff.

#### Scenario: Select the latest scheduled emission
- **WHEN** a cutoff follows several LiDAR frames but precedes the next configured map emission
- **THEN** the selected map is the most recent scheduled emission and does not include the intervening frames

#### Scenario: Deduplicate nearby cutoffs
- **WHEN** two requested cutoffs resolve to the same runtime map emission
- **THEN** both cutoff records reference one stored derived-map observation

#### Scenario: Reject a cutoff before the first map
- **WHEN** the runtime mapper has not emitted a map by a requested cutoff
- **THEN** preparation fails without publishing a synthetic partial map or output bundle

#### Scenario: Reject an unsupported LiDAR frame
- **WHEN** recorded LiDAR is not in the configured mapper frame
- **THEN** preparation fails rather than integrating points with an assumed transform

### Requirement: Immutable prepared bundle
The preparation workflow SHALL leave the source recording unchanged and SHALL atomically create a non-overwriting bundle containing a sealed derived database and a versioned manifest.

#### Scenario: Preserve the source recording
- **WHEN** preparation completes successfully
- **THEN** the source database hash is unchanged and no source stream has been added or modified

#### Scenario: Refuse an existing output
- **WHEN** the requested output path already exists
- **THEN** preparation fails rather than replacing or merging with that output

#### Scenario: Reject an active source WAL
- **WHEN** the source recording has a non-empty WAL sidecar
- **THEN** preparation fails because the source does not represent a sealed immutable input

#### Scenario: Reject a recorded global map collision
- **WHEN** the source recording already contains a `global_map` stream
- **THEN** preparation fails rather than shadowing recorded map evidence with a derived stream

### Requirement: Reproducible cutoff manifest
Each prepared bundle SHALL include a strict versioned manifest with source identity, derived identity, recording range, mapper settings, and per-cutoff stream and map boundaries.

#### Scenario: Record cutoff provenance
- **WHEN** a cutoff is prepared
- **THEN** its manifest entry contains relative and absolute cutoff time, each visible stream's count and last observation identity, and the selected map observation, timestamp, and integrated frame count

#### Scenario: Detect changed bundle inputs
- **WHEN** the source or derived database size or SHA-256 differs from the manifest at service startup
- **THEN** the service refuses to expose the cutoff

#### Scenario: Select only a prepared cutoff
- **WHEN** an operator requests a relative cutoff that is absent from the manifest
- **THEN** the service fails and reports the available prepared cutoffs

### Requirement: Offline frozen CodePolicy service
The system SHALL provide a standalone MCP service whose CodePolicy session exposes the selected frozen `memory`, omits the live DimOS `app`, and retains `python_exec` as its model-facing policy skill.

#### Scenario: Query bounded memory through MCP
- **WHEN** the service starts for a valid prepared cutoff and policy code queries its streams
- **THEN** the query sees the manifest-bounded source observations and selected derived map through `memory`

#### Scenario: Omit live robot access
- **WHEN** the frozen policy kernel is bootstrapped
- **THEN** `app` is absent from its namespace and configuration cannot enable `connect_app` for frozen memory

#### Scenario: Preserve persistent policy execution
- **WHEN** several `python_exec` calls are made to the same frozen service
- **THEN** imports and variables persist according to the existing CodePolicy session contract while the memory cutoff remains unchanged

### Requirement: QA harness separation
The frozen-memory service SHALL NOT generate benchmark questions, reference answers, or scores as part of bundle preparation or serving.

#### Scenario: Hand off prepared evidence
- **WHEN** a bundle and service are ready
- **THEN** an external harness can select the manifest cutoff, submit questions through MCP, and retain manifest and execution evidence without the service assigning correctness

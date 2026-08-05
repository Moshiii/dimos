## ADDED Requirements

### Requirement: True read-only SQLite access
Frozen Memory2 source and derived databases SHALL be opened using SQLite read-only mode with query-only enforcement. All public mutation paths SHALL reject writes, and opening or reading a frozen view SHALL not create WAL or other database mutation sidecars.

#### Scenario: Read a frozen recording
- **GIVEN** existing source and derived SQLite stores
- **WHEN** DimOS opens them as a frozen view and reads observations
- **THEN** the observations are available without modifying either database
- **AND** no WAL file is created by the frozen access

#### Scenario: Attempt a mutation
- **GIVEN** an open frozen view or stream
- **WHEN** a caller appends an observation, deletes a stream, creates a stream, or invokes another mutation path
- **THEN** the operation fails with a read-only error
- **AND** source and derived bytes remain unchanged

### Requirement: Inclusive authored cutoff
Every stream exposed through a frozen Memory2 view SHALL include observations whose timestamps are less than or equal to the authored cutoff and SHALL hide all observations after it.

#### Scenario: Observe the exact cutoff boundary
- **GIVEN** observations immediately before, exactly at, and immediately after a cutoff
- **WHEN** the stream is queried through the frozen view
- **THEN** observations before and exactly at the cutoff are visible
- **AND** the observation after the cutoff is absent

### Requirement: Source and derived overlay
A frozen view SHALL expose the union of source and derived stream names under the same cutoff. It SHALL reject ambiguous overlays in which both stores contain the same stream name and SHALL not allow callers to create or retype streams through the overlay.

#### Scenario: Access a derived map with source observations
- **GIVEN** a source recording and a derived store containing a non-colliding `global_map` stream
- **WHEN** a caller lists and reads frozen streams
- **THEN** both source streams and `global_map` are available through one memory object
- **AND** the inclusive cutoff applies to all of them

#### Scenario: Reject colliding streams
- **GIVEN** source and derived stores with the same stream name
- **WHEN** DimOS constructs the overlay
- **THEN** construction fails with the colliding names identified

### Requirement: Deterministic frozen bundle preparation
DimOS SHALL resolve normalized progress over the sealed recording range, materialize or reuse a derived frozen bundle, and retain a manifest describing the selected cutoff and source/derived integrity. Progress `1.0` SHALL resolve to the recording end inclusively.

#### Scenario: Prepare the final recording state
- **GIVEN** a named recording and normalized progress `1.0`
- **WHEN** DimOS prepares a frozen bundle
- **THEN** the selected cutoff equals the recording end
- **AND** the derived map and manifest describe only data available through that cutoff

#### Scenario: Reject invalid progress
- **GIVEN** non-finite progress or a value outside `[0, 1]`
- **WHEN** bundle preparation validates the source
- **THEN** preparation fails before attempt execution

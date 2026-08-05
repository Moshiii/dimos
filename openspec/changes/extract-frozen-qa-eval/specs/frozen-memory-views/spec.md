## ADDED Requirements

### Requirement: True read-only SQLite access
Frozen Memory2 source and derived databases SHALL use SQLite URI read-only mode and `PRAGMA query_only=ON`. Read-only initialization SHALL skip WAL configuration and table creation. Public frozen-store mutation operations SHALL fail, and reading SHALL not create database sidecars.

#### Scenario: Read without mutation
- **GIVEN** existing source and derived SQLite stores
- **WHEN** a frozen view opens and reads them
- **THEN** the databases remain byte-identical
- **AND** no WAL or SHM sidecar is created

#### Scenario: Reject mutation
- **GIVEN** an open frozen view
- **WHEN** a caller creates, retypes, deletes, or appends to a stream
- **THEN** the operation fails as read-only

### Requirement: Inclusive cutoff overlay
A frozen view SHALL expose the deterministic union of non-colliding source and derived stream names. Every returned stream SHALL use existing time-range filtering to include observations with timestamps `<= cutoff` and hide later observations.

#### Scenario: Read the cutoff boundary
- **GIVEN** observations before, exactly at, and after the cutoff
- **WHEN** the frozen stream is read
- **THEN** the first two observations are visible
- **AND** the later observation is hidden

#### Scenario: Reject collision
- **GIVEN** source and derived stores containing the same stream name
- **WHEN** the overlay is created
- **THEN** construction fails and identifies the collision

### Requirement: Metadata-based frozen bundle cache
Bundle preparation SHALL resolve progress over the recording range and cache a derived `global_map`. Cache reuse SHALL compare recording identity, source file size/mtime, mapper settings, progress, and cutoff metadata. It SHALL not hash full database files.

#### Scenario: Reuse an unchanged bundle
- **GIVEN** matching source metadata, mapper settings, and normalized progress
- **WHEN** preparation runs again
- **THEN** it reuses the derived bundle without remapping or hashing the databases

#### Scenario: Rebuild stale metadata
- **GIVEN** changed source metadata, mapper settings, or cutoff inputs
- **WHEN** preparation runs
- **THEN** it rebuilds the derived bundle before evaluation

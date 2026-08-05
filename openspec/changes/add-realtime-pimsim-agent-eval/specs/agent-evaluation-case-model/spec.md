## ADDED Requirements

### Requirement: Case-bound simulator scene source
The canonical evaluation case model SHALL provide a strict simulator-scene source containing the scene, simulation provider, robot, DimOS blueprint, and optional versioned source-preparation recipe. All supplied fields SHALL participate in case identity, and the single-case CLI MUST NOT override them.

#### Scenario: Compile a PiMSim scene case
- **WHEN** an authored case selects the apartment scene, PiMSim provider, Go2 robot, ordinary Go2 DimOS blueprint, and apartment spatial-memory preparation
- **THEN** the compiled source retains those selections and changing any selection changes or invalidates the case fingerprint

#### Scenario: Reject incomplete live source configuration
- **WHEN** a simulator-scene source omits its scene, provider, robot, or blueprint
- **THEN** strict case validation fails before simulator or agent startup

### Requirement: Bounded live interaction contract
The canonical evaluation case model SHALL provide a live CodePolicy interaction with a positive case-bound timeout and one-attempt session lifetime. The interaction SHALL identify an agent-facing lifecycle with updating read-only `memory` and a Porcelain-backed `app` supplied by a ready live DimOS runtime.

#### Scenario: Compile a bounded live interaction
- **WHEN** an embodied case selects live CodePolicy with a valid timeout
- **THEN** the timeout and live interaction revision participate in case identity

#### Scenario: Reject an invalid deadline
- **WHEN** a live interaction supplies a missing, non-finite, zero, or negative timeout
- **THEN** strict case validation fails before source preparation

### Requirement: Periodic private goal validator reference
The canonical evaluation case model SHALL provide a periodic-goal validator reference containing a revision, safe relative private path, and content digest. The referenced private document SHALL select a typed read-only predicate and positive polling interval without exposing that material in the public case projection.

#### Scenario: Resolve a private live goal
- **WHEN** the runner loads a periodic-goal case package
- **THEN** it verifies the private document against the case reference before starting Pi and retains only the validator kind and safe public metadata outside private evidence

#### Scenario: Reject changed private goal material
- **WHEN** the private goal file is missing, outside the case package, malformed, or does not match its compiled digest
- **THEN** preflight fails without starting the simulator, DimOS, CodePolicy, or Pi

### Requirement: Case-bound live execution selection
For a simulator-scene case, concrete simulation and DimOS execution SHALL derive from the compiled source rather than a runtime-selection command option. Agent condition, credential binding, output location, and presentation options SHALL remain attempt configuration and MUST NOT alter case identity.

#### Scenario: Run without a runtime option
- **WHEN** a developer invokes `dimos eval run <case>` for a valid simulator-scene case
- **THEN** the runner selects the source-declared provider and blueprint without requiring or accepting a separate runtime profile

#### Scenario: Change the evaluated agent
- **WHEN** a developer supplies supported agent model or authentication options
- **THEN** the case fingerprint remains unchanged and the resolved agent condition is retained with the attempt

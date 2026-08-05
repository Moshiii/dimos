## ADDED Requirements

### Requirement: One-case frozen evaluation command
DimOS SHALL provide `dimos eval run CASE --output=DIR` for one synchronous frozen-memory integer-question case. The case SHALL use strict tagged source, task, and validator models and SHALL reject unknown fields, unsafe oracle paths, non-finite progress, and unsupported kinds.

#### Scenario: Run a supported case
- **GIVEN** a valid case, prepared recording, built Pi extension, and API-key environment variable
- **WHEN** the user runs `dimos eval run CASE --output=DIR`
- **THEN** DimOS runs one fresh agent turn and privately scores its terminal integer answer
- **AND** publishes a compact result even when the semantic score fails

#### Scenario: Reject an unsafe case
- **GIVEN** a case with an escaped oracle path, invalid progress, unknown field, or unsupported kind
- **WHEN** preflight parses the case
- **THEN** the command exits `2` before starting CodePolicy or Pi

### Requirement: API-key-only authentication
The evaluator SHALL read an OpenAI API key from `OPENAI_API_KEY` or a user-selected environment variable. The key MUST be passed only to the Pi subprocess environment and MUST NOT appear in argv, the Jupyter kernel environment, MCP data, results, transcripts, stderr, or cache metadata.

#### Scenario: Run with the default key environment
- **GIVEN** a nonempty `OPENAI_API_KEY`
- **WHEN** the evaluator launches Pi
- **THEN** Pi receives the value through its environment
- **AND** CodePolicy cannot read that value from the Jupyter kernel environment

### Requirement: Exact private integer scoring
The evaluator SHALL accept only final text with exactly one marker ending in `ANSWER: <integer>`. The private oracle SHALL be loaded only by the scorer and SHALL never enter the model-facing prompt or runtime.

#### Scenario: Score a valid answer
- **GIVEN** final text ending in exactly one `ANSWER: 4`
- **WHEN** the private oracle expects `4`
- **THEN** the compact result records parsed integer `4` and task result `passed`

#### Scenario: Score malformed or mismatched output
- **GIVEN** a missing, repeated, non-integer, trailing, or mismatched answer
- **WHEN** scoring completes
- **THEN** the run is a completed semantic failure
- **AND** the command exits `0`

### Requirement: Compact non-overwriting output
The `--output` value SHALL be the exact directory for one run. A nonempty target SHALL fail preflight. The evaluator SHALL atomically publish `result.json`, the Pi-native transcript when available, and bounded nonempty Node stderr when present. It SHALL NOT copy source databases, cache manifests, case files, oracle files, prompts, MCP inventories, kernel records, or duplicate call logs.

#### Scenario: Publish a completed run
- **GIVEN** an unused output path and a completed agent turn
- **WHEN** scoring finishes
- **THEN** `result.json` contains case/source/model, final response, prediction, score, duration, and tool count
- **AND** the native Pi transcript is the sole tool/assistant trajectory

#### Scenario: Publish a caught infrastructure failure
- **GIVEN** Pi or CodePolicy fails after preflight
- **WHEN** cleanup completes
- **THEN** `result.json` records an infrastructure error and the command exits `1`
- **AND** any available native transcript or nonempty bounded stderr is retained

### Requirement: Output channel contract
The final human or JSON result SHALL go to stdout. Coarse runtime status SHALL go to stderr and `--quiet` SHALL suppress it. Credentials and oracle contents MUST NOT appear on either channel.

#### Scenario: Consume JSON output
- **GIVEN** `--json` with progress enabled
- **WHEN** a run finishes
- **THEN** stdout contains exactly one JSON value
- **AND** coarse status appears only on stderr

### Requirement: Demo fixture identity
The shipped fixture directory SHALL start with `demo_`, its case ID SHALL start with `demo-`, and its README SHALL state that oracle value `0` tests plumbing rather than authoritative room-count accuracy.

#### Scenario: Agent disagrees with the demo oracle
- **GIVEN** a completed answer other than `0`
- **WHEN** the demo case scores the answer
- **THEN** it reports semantic failure with exit `0`
- **AND** documentation does not characterize the result as an agent or mapping regression

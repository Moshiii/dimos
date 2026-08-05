## ADDED Requirements

### Requirement: Single-case evaluation CLI
DimOS SHALL provide `dimos eval run CASE` as a synchronous command for exactly one immutable evaluation case. The command SHALL accept only the `pi` backend, `gpt-5.6-luna` model, and `medium` thinking level, and SHALL reject unsupported values or semantic overrides.

#### Scenario: Run a supported case
- **GIVEN** a valid frozen-memory case, available recording, built adapter, and valid credentials
- **WHEN** the user runs `dimos eval run CASE`
- **THEN** DimOS executes one attempt to a terminal infrastructure outcome
- **AND** prints the final result even when live progress is suppressed

#### Scenario: Request an unsupported agent condition
- **GIVEN** a valid case
- **WHEN** the user supplies an unsupported backend, model, or thinking level
- **THEN** the command rejects the request before starting an attempt

### Requirement: Credential-safe authentication selection
The CLI SHALL support Codex OAuth and an OpenAI API-key environment binding without accepting secret values directly. Explicit authentication mode SHALL win; otherwise a nonempty `OPENAI_API_KEY` SHALL select API-key authentication and Codex OAuth SHALL be the fallback. Credential contents MUST NOT appear in command arguments, progress, results, or retained artifacts.

#### Scenario: Infer API-key authentication
- **GIVEN** no explicit authentication mode and a nonempty `OPENAI_API_KEY`
- **WHEN** the command resolves runtime authentication
- **THEN** it selects API-key authentication using the environment binding
- **AND** does not serialize the key value

#### Scenario: Resolve OAuth authentication
- **GIVEN** no API key and no explicit authentication mode
- **WHEN** the command resolves runtime authentication
- **THEN** it selects Codex OAuth using `--agent.auth.path`, `PI_SPATIAL_AUTH_PATH`, or `~/.pi/agent/auth.json` in precedence order
- **AND** fails preflight if the selected credential file is unavailable

### Requirement: Immutable and private case contract
An evaluation case SHALL consist of a frozen recording source, integer question, one-attempt frozen interaction, and exact-integer validator reference. Unknown fields, unsafe validator paths, non-finite progress, fingerprint mismatches, and runtime-specific fields in the semantic case SHALL be rejected. Runtime paths, credentials, ports, and output locations SHALL remain outside the case fingerprint.

#### Scenario: Validate a case before agent dispatch
- **GIVEN** a case containing a safe case-relative oracle path and expected SHA-256
- **WHEN** the command performs preflight
- **THEN** it verifies the case fingerprint and exact oracle bytes before starting the agent
- **AND** rejects an escaped path or digest mismatch

#### Scenario: Produce an agent-safe projection
- **GIVEN** a validated private case
- **WHEN** DimOS creates the public case projection and prompt
- **THEN** the projection omits the validator reference and all oracle content
- **AND** the private expected answer is not exposed to the agent runtime

### Requirement: Exact terminal integer scoring
The response parser SHALL succeed only when final text contains exactly one `ANSWER:` marker and ends with `ANSWER: <integer>`. A malformed answer or validator mismatch SHALL be a completed semantic failure, not an infrastructure failure.

#### Scenario: Parse a valid terminal answer
- **GIVEN** final agent text containing exactly one terminal `ANSWER: -3`
- **WHEN** DimOS parses and validates the response
- **THEN** it records integer prediction `-3`
- **AND** compares it privately with the exact-integer oracle

#### Scenario: Reject malformed answer text
- **GIVEN** final text with multiple markers, a non-integer marker, or trailing content after the answer
- **WHEN** DimOS parses the response
- **THEN** it records an invalid prediction and a completed failed task
- **AND** returns process exit code `0`

### Requirement: Immutable attempt evidence
Each started attempt SHALL reserve a fresh mode-`0700` `attempt_<uuid>` directory beneath the output root while holding a nonblocking output-root lock. Artifacts SHALL use safe attempt-relative paths, exclusive creation, SHA-256 descriptors, fsync, and atomic terminal publication. Concurrent attempts targeting the same output root SHALL not interleave.

#### Scenario: Retain a completed attempt
- **GIVEN** an attempt reaches scoring and cleanup succeeds
- **WHEN** DimOS publishes the terminal outcome
- **THEN** the attempt contains private and public case projections, source evidence, MCP and session evidence, tool-call and execution records, Pi evidence, prediction, private score, ordered lifecycle events, manifest, and terminal outcome
- **AND** existing artifacts are never overwritten

#### Scenario: Reject a concurrent attempt
- **GIVEN** one attempt holds the lock for an output root
- **WHEN** another attempt targets the same root
- **THEN** the second attempt fails cleanly without creating an interleaved attempt

### Requirement: Failure classification and lock release
Failures before attempt reservation SHALL exit `2`. Infrastructure, finalization, or cleanup failures normalized into a reserved attempt SHALL produce a failed attempt and exit `1`. Completed semantic pass or failure SHALL exit `0`. The output lock MUST be released after success, failure, interruption, partial startup, and artifact-publication failure.

#### Scenario: Preflight failure
- **GIVEN** an invalid case, unavailable oracle, missing credentials, missing adapter, or source-preparation error before reservation
- **WHEN** the error escapes preflight
- **THEN** the command exits `2`
- **AND** no attempt is reported as completed

#### Scenario: Attempt cleanup failure
- **GIVEN** an otherwise completed attempt whose process or resource cleanup fails
- **WHEN** DimOS finalizes the attempt
- **THEN** it reports a failed infrastructure attempt and exits `1`
- **AND** releases the output lock even if terminal artifact publication also fails

### Requirement: Machine-readable output and private progress
With `--json`, stdout SHALL contain exactly one compact result while progress and tool-call rendering remain on stderr. `--quiet` SHALL suppress progress but not the final result. Neither channel SHALL expose private oracle material or credentials.

#### Scenario: Consume compact JSON
- **GIVEN** a valid invocation using `--json`
- **WHEN** the attempt runs with progress enabled
- **THEN** stdout remains parseable as exactly one JSON result
- **AND** progress appears only on stderr

### Requirement: Synthetic plumbing fixture
The shipped Hong Kong office room-count smoke case SHALL preserve its `case.json`, private oracle, and warning README. The expected value `0` MUST be described as a synthetic plumbing sentinel and MUST NOT be presented as the authoritative room count.

#### Scenario: Agent disagrees with the sentinel
- **GIVEN** the fixture and an agent response other than `ANSWER: 0`
- **WHEN** infrastructure and scoring complete
- **THEN** the task may report semantic failure with exit code `0`
- **AND** documentation does not characterize that outcome as a mapping or agent regression

# frozen-qa-agent-evaluation Specification

## Purpose
TBD - created by archiving change standalone-code-policy-frozen-qa-eval. Update Purpose after archive.
## Requirements
### Requirement: Frozen QA case compilation
The frozen-QA evaluator SHALL compile a strict case containing a frozen Memory2 source selection, public question and typed response contract, standalone frozen CodePolicy interaction, and private validator. Compiled identities SHALL bind the source recording and derived bundle digests, authored and resolved progress, task, interaction revision, and validator revision.

#### Scenario: Compile an integer QA case
- **WHEN** an authored case selects a prepared recording progress and integer response
- **THEN** compilation resolves one verified cutoff, retains the public prompt without the oracle, and binds an exact-integer private validator

### Requirement: Fresh Pi CodePolicy attempt
For every frozen-QA attempt, the runner SHALL start and own one fresh standalone frozen CodePolicy service and one fresh native Pi session. Pi SHALL receive exactly the compatible `python_exec` tool, the kernel SHALL contain read-only `memory` and no `app`, and no DimOS blueprint or simulator SHALL be required.

#### Scenario: Dispatch one frozen question
- **WHEN** source verification and standalone service readiness succeed
- **THEN** the runner starts Pi with the exact public task, brokers only `python_exec`, and waits for the configured terminal agent response

#### Scenario: Reject an incompatible MCP surface
- **WHEN** the standalone service is missing `python_exec`, exposes a duplicate, or advertises an incompatible schema
- **THEN** the runner fails before agent dispatch and retains an infrastructure-failure outcome

### Requirement: Typed final-answer prediction
An integer QA task SHALL instruct Pi to emit exactly one terminal `ANSWER: <integer>` marker. The evaluator SHALL retain Pi's complete final text separately, parse the marked integer deterministically, and emit a typed prediction bound to the case, Pi session, CodePolicy session, parser revision, and attempt.

#### Scenario: Parse a valid answer
- **WHEN** Pi's final response contains exactly one valid terminal integer marker
- **THEN** the evaluator retains the response and emits that integer as the canonical prediction

#### Scenario: Reject an ambiguous answer
- **WHEN** Pi omits the terminal marker, supplies multiple markers, or supplies a non-integer value
- **THEN** the attempt remains operationally completed and the validator records a failed task result with the parser failure evidence

### Requirement: Private exact-integer validation
The exact-integer validator SHALL compare the canonical typed prediction with a human-reviewed private oracle and emit a private score. The oracle SHALL contain the expected count, counting policy, auditable room inventory, and evidence notes, and SHALL remain unavailable through the source, task, CodePolicy namespace, MCP results, Pi evidence, and public outcome.

#### Scenario: Score the expected count
- **WHEN** the parsed integer equals the private expected count
- **THEN** the score and normalized outcome record a passed task

#### Scenario: Score a different count
- **WHEN** the parsed integer differs from the private expected count
- **THEN** the score and normalized outcome record a failed task while the attempt remains operationally completed

### Requirement: Frozen QA attempt evidence
Every frozen-QA attempt SHALL retain an immutable case snapshot, verified source manifest, MCP inventory, CodePolicy session receipt and call records, native Pi session and prompt evidence, final response, typed prediction, private score when validation completes, lifecycle events, attempt manifest, and exactly one normalized terminal outcome.

#### Scenario: Inspect a completed attempt
- **WHEN** an attempt completes with either a correct or incorrect answer
- **THEN** an authorized reviewer can reconstruct the source cutoff, agent interface, policy calls, response parsing, validator revision, score, and operational outcome from retained artifacts

#### Scenario: Preserve a failed prefix
- **WHEN** an infrastructure failure occurs after some evidence is produced
- **THEN** the runner retains the valid evidence prefix, records no fabricated prediction or score, writes an infrastructure-failure outcome, and terminates its owned processes

### Requirement: Hong Kong office north-star
The repository SHALL include one authored north-star case using the sealed `go2_hongkong_office` recording at normalized progress `1.0`, the public question `How many rooms in total?`, the frozen standalone CodePolicy interaction, the pinned existing Pi code-policy agent configuration, and a human-reviewed private exact-integer oracle.

#### Scenario: Run the credentialed north-star
- **WHEN** a developer runs the documented command with the recording, prepared derived map, valid Pi credentials, and approved private oracle
- **THEN** the runner needs no DimOS blueprint, Pi makes at least one successful `python_exec` call against frozen `memory`, the answer is parsed and scored, all required evidence is retained, and the owned service is stopped

#### Scenario: Accept the change north-star
- **WHEN** the credentialed north-star produces a complete attempt whose typed prediction matches the approved oracle
- **THEN** the attempt reports completed infrastructure and passed task success and serves as the direct acceptance evidence for this change

### Requirement: Canonical single-case CLI
The system SHALL expose `dimos eval run <case>` for one synchronous local
attempt. The positional case document SHALL be decoded as the strict immutable
Pydantic `EvalCase`; the command SHALL NOT expose overrides for source, task,
interaction, validator, recording selection, prepared memory, question, or
expected answer. Private validator paths SHALL resolve relative to the case
document and SHALL be verified against the compiled reference before agent
startup.

#### Scenario: Run the static case with defaults
- **WHEN** a developer invokes `dimos eval run <case>` without agent overrides
- **THEN** the runner uses the typed Pi defaults, resolves the frozen Memory2 binding and private validator from the case, creates one attempt, and does not require a DimOS blueprint or replay flag

#### Scenario: Reject a changed or incomplete case package
- **WHEN** the case fingerprint, relative validator path, validator digest, or frozen source binding cannot be verified
- **THEN** preflight fails without substituting data from a CLI override and without dispatching the agent

### Requirement: Typed agent and authentication configuration
The single-case command SHALL configure execution through a strict Pydantic
runtime model whose nested fields are exposed as dotted long options, including
`--agent.backend`, `--agent.model`, `--agent.thinking-level`, and backend-specific
`--agent.auth.*` fields. Pi SHALL default to the repository-supported model and
medium thinking. When no authentication option is supplied, the CLI SHALL use
the standard `OPENAI_API_KEY` binding when it is nonempty and SHALL otherwise
fall back to Codex OAuth. Explicit authentication options SHALL take precedence
over inference. Authentication options SHALL name a credential binding only;
raw secrets SHALL NOT be accepted on the command line, stored in the case, or
retained in attempt evidence.

#### Scenario: Override the agent condition
- **WHEN** a developer supplies supported dotted agent options
- **THEN** Pydantic validates the complete discriminated backend configuration before the attempt and the resolved agent condition is retained in the attempt manifest

#### Scenario: Use API-key authentication
- **WHEN** the developer selects API-key authentication and names an environment variable
- **THEN** the runner reads the secret from that environment binding at runtime and retains only sanitized authentication provenance

#### Scenario: Infer standard API-key authentication
- **WHEN** `OPENAI_API_KEY` is nonempty and the developer supplies no authentication options
- **THEN** the CLI selects API-key authentication through that environment binding without retaining or printing the secret

#### Scenario: Preserve an explicit authentication selection
- **WHEN** the developer explicitly selects Codex OAuth while `OPENAI_API_KEY` is also available
- **THEN** the CLI uses Codex OAuth and does not replace it with inferred API-key authentication

### Requirement: Human and machine-readable single-case result
The single-case command SHALL pretty-print case identity, normalized operational
and task results, parsed prediction when present, resolved agent condition, tool
call count, duration, and attempt artifact path. `--json` SHALL instead emit one
compact Pydantic-encoded summary to standard output. `--output` MAY select the
attempt root without changing case identity. The complete evidence manifest and
sidecars SHALL remain authoritative on disk.

#### Scenario: Print a completed result
- **WHEN** an attempt completes with either a passed or failed task result
- **THEN** the default terminal view clearly distinguishes the semantic result and reports the artifact path without printing the private expected answer

#### Scenario: Emit compact JSON
- **WHEN** the developer supplies `--json`
- **THEN** standard output contains one strict summary object with explicit `attempt_status` and `task_result` literals while the full manifest remains in the attempt directory

### Requirement: Live concise Pi progress
The single-case command SHALL display a concise live evaluation and Pi activity
trace on standard error by default. The trace SHALL include lifecycle status,
an initial public case/question header with a pending-answer marker, visible
assistant text, bounded `python_exec` calls and results, and terminal completion
while excluding model thinking deltas, raw adapter protocol, private validator
material, and credentials. Progress rendering SHALL NOT affect case identity,
execution, scoring, evidence finalization, or cleanup.

#### Scenario: Identify the question before agent work
- **WHEN** a developer starts a single case without `--quiet`
- **THEN** the trace displays the case, source selection, public question, and `Answer: pending` before source preparation and agent activity without displaying the private oracle

#### Scenario: Observe a running attempt
- **WHEN** a developer runs a single case without `--quiet`
- **THEN** standard error reports source preparation, Pi startup, visible assistant text, and bounded policy calls before the normalized result is printed

#### Scenario: Preserve JSON standard output
- **WHEN** a developer runs with `--json`
- **THEN** progress remains on standard error and standard output contains only the compact result object

#### Scenario: Suppress presentation progress
- **WHEN** a developer supplies `--quiet`
- **THEN** no live progress is rendered and the attempt executes with identical semantic and evidence behavior

#### Scenario: Isolate a broken observer
- **WHEN** the configured presentation callback raises while processing progress
- **THEN** the evaluator ignores that presentation failure and continues the attempt normally

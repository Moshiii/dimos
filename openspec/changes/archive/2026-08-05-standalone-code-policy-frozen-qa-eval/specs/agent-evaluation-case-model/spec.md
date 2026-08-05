## ADDED Requirements

### Requirement: Canonical evaluation case contract
Every new agent evaluation implemented through the generic agent-evaluation package SHALL represent one immutable semantic case as separate `source`, `task`, `interaction`, and private `validator` contracts. The case identity SHALL bind canonical forms or immutable references for all four contracts.

#### Scenario: Compile a case
- **WHEN** an authored evaluation case is compiled
- **THEN** the compiled case identifies what evidence or environment exists, what the agent is asked to do, the executable protocol by which the agent receives evidence and acts, and the private rule that determines success

#### Scenario: Reject an incomplete case
- **WHEN** any source, task, interaction, or validator contract is absent or fails strict validation
- **THEN** the evaluator rejects the case before reserving an attempt

### Requirement: Source and task separation
The source contract SHALL describe an immutable environment or evidence selection without containing questions, expected answers, validators, live process handles, or agent sessions. The task contract SHALL contain the agent-visible question, instruction, or goal and its response contract without containing private correctness material.

#### Scenario: Reuse one source for independent questions
- **WHEN** several questions use the same frozen recording selection
- **THEN** each compiled case may reference the same source identity while retaining a separate task and validator identity

#### Scenario: Keep private answers out of the task
- **WHEN** a public task is delivered to an agent
- **THEN** its payload and referenced public material contain no expected answer or private validator configuration

### Requirement: Executable interaction contract
The interaction contract SHALL identify a versioned executable driver and strict configuration that defines the actual agent-facing lifecycle, including source materialization or attachment, exposed variables or tools, task delivery, completion, evidence capture, and cleanup. A label such as offline, realtime, recording, or simulation SHALL NOT by itself satisfy the interaction contract.

#### Scenario: Run a frozen CodePolicy interaction
- **WHEN** a case selects the frozen CodePolicy interaction
- **THEN** the driver creates a fresh standalone CodePolicy service, binds the selected frozen memory as the agent-visible `memory`, delivers the task to the configured agent, captures the result, and tears the service down

#### Scenario: Run a live CodePolicy interaction
- **WHEN** a case selects the live CodePolicy interaction against a ready DimOS runtime
- **THEN** the driver binds read-only live `memory` and a porcelain-backed `app` in standalone CodePolicy and captures realtime agent activity without hosting CodePolicy inside the blueprint

### Requirement: Private validator lifecycle
The validator contract SHALL remain unavailable to the agent and SHALL support both post-hoc answer validation and validators that observe private live environment state during an interaction. Validation SHALL emit a typed private score bound to the case, prediction or terminal agent outcome, validator revision, and attempt.

#### Scenario: Score a frozen answer after completion
- **WHEN** a frozen QA interaction yields a typed prediction
- **THEN** the private validator compares it with its private oracle after the agent turn and emits a score without exposing the oracle

#### Scenario: Observe a live environment privately
- **WHEN** a live simulation validator must observe state while the agent acts
- **THEN** the evaluation engine starts and stops the private validator lifecycle independently of the agent-facing interaction and does not add validator state to the agent interface

### Requirement: Attempt binding and normalized outcome
An attempt SHALL bind one compiled case to one agent condition, concrete runtime binding, seed where applicable, fresh interaction session, and immutable evidence location. Operational completion SHALL remain separate from task success, and a completed incorrect answer SHALL be represented as a valid failed task rather than an infrastructure failure.

#### Scenario: Complete with an incorrect answer
- **WHEN** the interaction and validator complete normally but the prediction does not satisfy the validator
- **THEN** the attempt reports completed infrastructure and failed task success with a retained prediction and private score

#### Scenario: Fail before validation
- **WHEN** source verification, service startup, agent startup, or required evidence retention fails
- **THEN** the attempt reports infrastructure failure and does not claim a passed or failed task score

### Requirement: Shared evaluation execution foundation
Future evaluation runners added under the generic agent-evaluation package SHALL consume the canonical case contract and shared attempt, Pi-session, prediction, validation, and evidence primitives. Interaction-specific behavior SHALL live behind drivers rather than parallel top-level runner architectures.

#### Scenario: Preserve the live DimSim smoke
- **WHEN** the existing live DimSim Pi smoke is migrated to the shared foundation
- **THEN** its reset, realtime interaction, native validation, cleanup, and retained evidence behavior remain testable through a live interaction and validator implementation

#### Scenario: Add another evaluation kind
- **WHEN** a future evaluation uses a different source or validation mechanism
- **THEN** it adds typed source, interaction, and validator implementations without redefining Pi session ownership, generic attempt outcomes, or evidence identity

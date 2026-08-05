## ADDED Requirements

### Requirement: Fresh isolated runtime per attempt
Each attempt SHALL start one fresh standalone CodePolicy process on an ephemeral loopback port and one fresh Node/Pi process. The CodePolicy namespace SHALL preload frozen `memory`, SHALL NOT expose live DimOS `app`, and SHALL be disposed after the attempt.

#### Scenario: Start a frozen policy session
- **GIVEN** a prepared frozen bundle
- **WHEN** an attempt starts its interaction
- **THEN** the policy can inspect the bounded `memory` view
- **AND** no live robot RPC object is present

#### Scenario: Run consecutive attempts
- **GIVEN** two sequential evaluations
- **WHEN** each attempt starts
- **THEN** each receives distinct CodePolicy and Pi session identities and processes
- **AND** Python state does not leak between attempts

### Requirement: Exactly one Pi tool
The Pi session SHALL disable built-in tools, extensions, skills, prompt templates, and context files, and SHALL expose exactly one custom tool named `python_exec`. The runtime SHALL fail closed if the activated inventory differs before or after session creation.

#### Scenario: Validate tool inventory
- **GIVEN** a Pi session configured for frozen evaluation
- **WHEN** the session reports its active tools
- **THEN** the ordered inventory is exactly `["python_exec"]`
- **AND** any additional or missing tool causes infrastructure failure

### Requirement: Pinned model and authentication runtime
The adapter SHALL run the pinned supported Pi libraries with model `gpt-5.6-luna` and medium thinking. It SHALL support Codex OAuth by credential-file path and OpenAI authentication by key supplied through the selected environment binding, without placing secret values in process arguments or evidence.

#### Scenario: Launch with OAuth
- **GIVEN** a valid OAuth credential file
- **WHEN** the Pi process starts
- **THEN** it resolves the configured OAuth model runtime from the supplied path
- **AND** no credential bytes are emitted on the line protocol or retained in attempt evidence

### Requirement: Bounded validated line protocol
Python and Node SHALL communicate through newline-delimited JSON with stdout reserved for protocol frames and diagnostics directed to bounded stderr evidence. Every inbound frame SHALL be size-bounded and schema-validated, and tool calls and replies SHALL be correlated by unique IDs. Unknown, duplicate, malformed, or oversized frames SHALL fail closed.

#### Scenario: Broker a valid Python call
- **GIVEN** an idle Pi session and a valid `python_exec` request
- **WHEN** Node emits the correlated tool call and Python returns its result
- **THEN** the matching reply completes the pending call
- **AND** the call and bounded execution record are retained

#### Scenario: Receive an invalid reply
- **GIVEN** no pending call with a supplied reply ID
- **WHEN** the adapter receives that reply
- **THEN** it reports a protocol error without applying the reply to another call

### Requirement: Observable readiness and complete evidence
The standalone runtime SHALL retry both connection failures and read timeouts while waiting for MCP readiness. It SHALL retain the MCP inventory, CodePolicy session receipt and bounded execution records, broker call log, Pi prompt/session evidence, and bounded adapter stderr under the attempt directory.

#### Scenario: Server becomes ready after transient failures
- **GIVEN** a starting loopback MCP process that initially refuses connections or times out reads
- **WHEN** readiness is polled within the configured deadline
- **THEN** polling continues until initialization succeeds or the deadline expires

### Requirement: Reliable cancellation and cleanup
Abort, timeout, interrupt, normal completion, partial startup, and disposal SHALL terminate or kill remaining child processes within bounded cleanup periods. Cleanup failures SHALL be surfaced to the attempt result, and no child process SHALL remain after command termination.

#### Scenario: Interrupt an active turn
- **GIVEN** an active Pi turn with an outstanding tool call
- **WHEN** the command is interrupted
- **THEN** pending broker calls are rejected, Pi is aborted and disposed, and CodePolicy is stopped
- **AND** the output lock is released

### Requirement: Trusted unsandboxed execution disclosure
DimOS SHALL describe CodePolicy as trusted, persistent, unsandboxed Python. Read-only Memory2 SHALL be presented as protection against accidental API mutation, not as an operating-system security boundary.

#### Scenario: Review the evaluation documentation
- **GIVEN** a user preparing to run a frozen evaluation
- **WHEN** they read the capability documentation
- **THEN** they are warned not to execute hostile policy code without an external container or OS sandbox

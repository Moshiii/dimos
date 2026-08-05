## ADDED Requirements

### Requirement: Shared Jupyter CodePolicy session
DimOS SHALL provide a module-independent `CodePolicySession` that owns one Jupyter kernel, persistent namespace, serialized execution, timeout interruption, restart recovery, and bounded shutdown. Frozen bootstrap SHALL expose read-only `memory`, omit live `app`, and scrub credentials from the kernel environment.

#### Scenario: Start a frozen session
- **GIVEN** prepared source and derived Memory2 paths
- **WHEN** a frozen CodePolicy session starts
- **THEN** `memory` is available in the persistent namespace
- **AND** `app` and API-key environment variables are absent

#### Scenario: Recover from timeout
- **GIVEN** an execution exceeds its timeout
- **WHEN** CodePolicy interrupts it
- **THEN** the session restarts and re-applies the frozen bootstrap
- **AND** later calls execute in a usable clean kernel

### Requirement: In-process official MCP server
The evaluator SHALL expose CodePolicy through the official Python MCP SDK running in the evaluator process. It SHALL register exactly one tool named `python_exec`, bind a pre-created loopback port-`0` socket, and stop through direct evaluator ownership. It SHALL expose no HTTP control API.

#### Scenario: Start without a port race
- **GIVEN** a new evaluator run
- **WHEN** it starts MCP
- **THEN** the evaluator binds the socket before serving and passes the actual URL to Pi
- **AND** the client observes exactly one tool named `python_exec`

### Requirement: Stock Pi CLI and official MCP extension
The evaluator SHALL launch pinned Pi `0.80.10` in one-shot JSON mode with built-in tools, implicit extensions, skills, prompt templates, themes, and context files disabled. One explicit extension using `@modelcontextprotocol/client==2.0.0` SHALL register `python_exec` and call the official Python MCP server directly.

#### Scenario: Complete one Pi turn
- **GIVEN** an initialized one-tool MCP server
- **WHEN** the stock Pi CLI receives the authored question
- **THEN** it emits official JSON events and persists its native session transcript
- **AND** Python derives final text, stop reason, and tool count without a custom protocol

#### Scenario: Reject tool drift
- **GIVEN** the extension observes an MCP inventory other than exactly `python_exec`
- **WHEN** Pi initializes the extension
- **THEN** startup fails before the model turn

### Requirement: Bounded cleanup
The evaluator SHALL stop Pi before stopping MCP and Jupyter. Pi timeout SHALL escalate terminate to kill. Normal completion, caught failure, interruption, and partial startup SHALL leave no live Pi process, MCP server thread, or Jupyter kernel.

#### Scenario: Pi fails during startup
- **GIVEN** MCP and Jupyter have started but Pi fails
- **WHEN** the evaluator handles the error
- **THEN** it stops the server thread and kernel within bounded deadlines
- **AND** publishes a compact infrastructure failure result

### Requirement: Trusted execution disclosure
Documentation SHALL describe CodePolicy as trusted persistent unsandboxed Python. Read-only Memory2 and a scrubbed environment SHALL not be presented as an operating-system sandbox.

#### Scenario: Read the operator guide
- **GIVEN** a user preparing a frozen evaluation
- **WHEN** they read its safety section
- **THEN** they are warned to use an external OS sandbox or container for hostile code

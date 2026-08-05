## ADDED Requirements

### Requirement: Module-independent CodePolicy service
The system SHALL provide a standalone CodePolicy service that owns its persistent Python kernel and MCP transport without constructing a DimOS `Module`, `Blueprint`, module worker, or `ModuleCoordinator`.

#### Scenario: Start frozen CodePolicy without DimOS
- **WHEN** a caller starts the service with a valid frozen-memory environment
- **THEN** the service exposes its MCP endpoint and executes policy code without any running DimOS blueprint

#### Scenario: Avoid an internal RPC hop
- **WHEN** MCP invokes `python_exec`
- **THEN** the standalone host calls its owned CodePolicy session directly rather than forwarding through DimOS module RPC

### Requirement: Sole model-facing policy tool
The standalone MCP inventory SHALL expose exactly one model-facing tool named `python_exec` with the compatible existing schema and trusted-unsandboxed description. Session reset, control, evidence, readiness, and observer operations SHALL remain host-side controls and SHALL NOT appear as model-facing MCP tools.

#### Scenario: Inspect the standalone inventory
- **WHEN** an agent host lists tools on a ready standalone service
- **THEN** the inventory contains exactly one compatible `python_exec` definition

### Requirement: Fixed CodePolicy environments
The service SHALL support a frozen environment whose Python namespace contains read-only `memory` and no `app`, and a live DimOS environment whose namespace contains read-only live `memory` and `app` connected to the selected running system through `dimos.porcelain`. The initial service SHALL NOT expose an arbitrary user-defined binding registry.

#### Scenario: Bootstrap frozen memory
- **WHEN** the service starts with a verified frozen source and cutoff
- **THEN** `memory` queries the bounded source and derived streams, `app` is undefined, and Memory2 mutation through the bound API is rejected

#### Scenario: Bootstrap live DimOS
- **WHEN** the service starts with a live memory database and a discoverable DimOS runtime
- **THEN** `memory` observes the live database read-only and `app` provides the porcelain interface to that runtime

### Requirement: Persistent isolated attempt session
The service SHALL preserve imports and Python variables across `python_exec` calls within one session, SHALL support host-controlled reset to a fresh session identity, and SHALL retain bounded execution records bound to session and execution identities. Evaluation use SHALL create a fresh service or fresh verified session for every attempt.

#### Scenario: Preserve state within one attempt
- **WHEN** one call defines a Python variable and a later call reads it in the same session
- **THEN** the later call observes the value and both records share the same CodePolicy session identity

#### Scenario: Reset between attempts
- **WHEN** an evaluation attempt finishes and another begins
- **THEN** the next attempt cannot observe the prior Python namespace and receives a new CodePolicy session identity

### Requirement: Runner-owned lifecycle and evidence
The evaluation interaction driver SHALL own standalone service startup, readiness, session identity capture, interruption, evidence collection, and shutdown. A pre-existing shared service MAY be supported for manual development but SHALL NOT be a conforming benchmark execution mode.

#### Scenario: Clean up a failed attempt
- **WHEN** agent startup, policy execution, validation, or evidence finalization fails
- **THEN** the interaction driver best-effort interrupts active execution, preserves available CodePolicy records, and terminates its owned service

### Requirement: Compatibility migration
The existing `CodePolicyModule` MAY remain temporarily as a wrapper over the extracted session core, but canonical frozen and live agent-evaluation blueprints SHALL remove CodePolicy and its agent-facing MCP server from their module composition.

#### Scenario: Run a legacy blueprint during migration
- **WHEN** an existing blueprint still instantiates `CodePolicyModule`
- **THEN** its compatible `python_exec`, reset, and evidence behavior delegates to the extracted core without becoming the topology used by new evaluations


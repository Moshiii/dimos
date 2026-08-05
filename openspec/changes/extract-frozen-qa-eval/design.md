## Context

The complete frozen QA path exists at reference commit `30e5f1c0e` on `cc/frontier`, primarily in commits `7cbf13845` and `10ca9bf15`. Those commits depend on earlier agent-evaluation and Pi adapter history and also contain accidental imports from live DimSim and spatial benchmark packages. The feature must therefore be ported file-by-file onto the recorded `origin/main` base rather than cherry-picked.

The target is one source-checkout command that evaluates one frozen Memory2 recording synchronously. The implementation spans SQLite access, derived map preparation, trusted CodePolicy execution, a Node/Pi subprocess, private validation, evidence storage, and Typer registration. The base DimOS CLI must remain importable without the `agents` extra.

## Goals / Non-Goals

**Goals:**

- Preserve the reference frozen-QA behavior behind `dimos eval run` with a focused dependency graph.
- Keep semantic cases immutable and independent of credentials, paths, ports, and output locations.
- Enforce private-oracle isolation and read-only, inclusive frozen-memory access.
- Run fresh CodePolicy and Pi processes per attempt with exactly `python_exec` exposed.
- Retain durable, non-overwriting evidence and always release processes and locks.
- Resolve all ported code against current `main` APIs and optional-dependency conventions.

**Non-Goals:**

- Dataset batching, retries, containers, scheduling, or distributed execution.
- Live DimSim, simulation episodes, replay blueprints, robot control, or hardware evaluation.
- Legacy smoke-runner configuration, backend abstraction for unrelated evaluators, or agentic blueprint integration.
- Treating read-only SQLite or trusted CodePolicy as a security sandbox.
- Shipping the Node adapter inside a Python wheel in this initial source-checkout slice.

## DimOS Architecture

The runtime flow is:

```text
case.json + private oracle
        |
        v
single-case preflight -----> frozen bundle cache
        |                         |
        v                         v
AttemptStore lock -------> FrozenMemoryStore
        |                         |
        v                         v
Standalone CodePolicy MCP (`memory`, no `app`)
        ^
        | python_exec MCP calls
        v
dedicated Node/Pi process (exactly one tool)
        |
        v
prediction -> private validator -> terminal evidence -> cleanup/unlock
```

The focused Python package is split by dependency direction:

- `agent_eval/base.py`: strict common Pydantic configuration.
- `agent_eval/json.py`: local UTF-8 canonical JSON using sorted keys and compact separators.
- `agent_eval/artifacts.py`: artifact references, typed IDs, and lifecycle records without benchmark-specific imports.
- `agent_eval/auth.py`: the runtime credential transport record without DimSim imports.
- `agent_eval/case.py`: only the supported frozen source, integer task, frozen interaction, exact validator, request, prediction, score, and outcome contracts.
- `agent_eval/interfaces.py`, `engine.py`, and `store.py`: adapter Protocols, lifecycle orchestration, durable evidence, and locking; no DimSim or spatial types.
- `agent_eval/pi*.py`: MCP binding, evidence log, Node subprocess, and progress transport.
- `agent_eval/single_case.py`: preflight, authentication resolution, adapter discovery, bundle preparation, and compact result projection.
- `short_horizon_qa/*`: frozen preparation, service binding, parser, validator, and interaction drivers.
- `agents/code_policy_core.py` and `code_policy_server.py`: persistent trusted Python execution and loopback-only standalone MCP hosting.
- `memory2/store/frozen.py` plus narrow existing Memory2 changes: read-only overlay and inclusive cutoff.

The Node package `packages/pi-code-policy-adapter` contains only its line protocol, `python_exec` definition, pinned Pi session/auth setup, evidence retention, and tests. It does not depend on spatial tool definitions or adapter code. Python resolves its built `dist/code-policy-main.js` from the source checkout and reports an actionable preflight error when it is absent.

No DimOS `Spec` Protocol, module stream, blueprint, or generated registry is added. Internal Python adapter Protocols define the source, interaction, validator, evidence, and Pi session seams. The only MCP-visible surface is the standalone `python_exec` tool created for the attempt.

## Decisions

1. **Port current file content, not commits.** Cherry-picking would import the full benchmark graph and unrelated lockfile changes. Each focused file is copied from `30e5f1c0e`, pruned, and reconciled with current `main`.

2. **Use three small foundation modules instead of the reference generic `models.py` and `config.py`.** Canonical JSON, artifact records, and runtime credentials form a dependency floor that does not import DimSim or spatial packages. `case.py` omits live discriminated variants so unsupported inputs fail during schema validation.

3. **Reserve attempts only after preflight.** Case/oracle validation, source preparation, credential resolution, and adapter discovery happen before attempt reservation and map to exit `2`. Once the store is reserved, normalized infrastructure failures map to exit `1`; semantic results map to exit `0`.

4. **Make lock release structurally unconditional.** The attempt engine owns the store in an outer `try/finally`. Resource cleanup and terminal artifact publication may affect the outcome but cannot bypass store closure. Fault-injection tests cover fsync, manifest, event, and terminal-publication failures.

5. **Treat private data as an information-flow boundary.** The public projection contains no validator. Oracle bytes are loaded only by the validator, and tests scan prompts, progress, compact results, Pi evidence, broker logs, and CodePolicy state for private material.

6. **Implement real SQLite read-only mode.** Source and derived stores use SQLite `mode=ro` and `query_only`; writable connections alone configure WAL. Mutation methods and streams reject writes. A `ThroughFilter` implements `ts <= cutoff` and is applied to every overlaid stream.

7. **Extract a dedicated one-tool Node package.** The session accepts a supplied `python_exec` definition, disables built-ins, and verifies the exact inventory. Node `>=22.19.0`, `@earendil-works/pi-ai` `0.80.10`, and `@earendil-works/pi-coding-agent` `0.80.10` remain pinned until upgraded deliberately.

8. **Keep optional imports off the base CLI path.** `dimos.cli.eval` is a dependency-light Typer shell or imports the heavy implementation only inside the `run` callback. Jupyter, FastAPI, and Uvicorn are added to the `agents` extra, not base dependencies.

9. **Use source-checkout adapter discovery initially.** Users build the dedicated package with npm before evaluation. Installed-wheel adapter distribution is deferred to a separately scoped packaging change.

## Safety / Simulation / Replay

This change never commands live hardware and does not start a robot, simulation, or replay blueprint. It reads a sealed recording and creates a derived map cache. The self-hosted mechanics gate uses `CPU:0`; normal first-time preparation may retain the reference mapper defaults and require CUDA.

CodePolicy executes trusted Python persistently and without an OS sandbox. Read-only Memory2 prevents mutation through provided APIs but cannot prevent arbitrary filesystem or process access by hostile code. Documentation and runtime receipts must not imply stronger isolation. Untrusted execution requires a future container or OS sandbox.

## Risks / Trade-offs

- **Finalization can fail after useful work:** unconditional `finally` cleanup and prefix evidence reduce lock/process leakage; the terminal file may still be absent when storage itself fails.
- **Private data can leak through a new evidence path:** maintain distinct public/private models and add sentinel scans over every agent-visible channel.
- **Optional dependencies can leak into basic commands:** use callback-local imports and test in a subprocess that blocks agents-only modules.
- **Node/Python protocol drift:** share a version field, validate all frames, bound frames/stderr, correlate IDs, and run both sides' protocol tests.
- **Source-only adapter discovery limits installed users:** document the prerequisite and fail clearly; do not silently search ambiguous global locations.
- **Long or stuck agent turns complicate cleanup:** propagate abort, use bounded process waits, escalate terminate to kill, and record cleanup failures.
- **The smoke oracle can be mistaken for benchmark truth:** preserve warnings in the fixture, docs, and tests and describe semantic disagreement as expected plumbing behavior.

## Migration / Rollout

Create a fresh feature branch from `origin/main` SHA `e8a985d83a85c9827fa89ed7526e40a822eb1ae3`. Land in dependency order: Memory2 read-only support, CodePolicy runtime, generic evaluation foundation, Node adapter, frozen drivers, CLI, fixture, and docs. Update `pyproject.toml` narrowly and regenerate `uv.lock`; do not carry unrelated reference changes.

Run Python and Node unit suites first, then the self-hosted Hong Kong mechanics gate, and finally the exact credentialed CLI smoke. No blueprint registry generation is required because no blueprint or module registry input changes. Rollback consists of removing the additive CLI registration and focused new packages; existing Memory2 read-only parameters remain backward-compatible defaults.

## Open Questions

None blocking this change. Distribution of the built Node adapter in Python wheels is explicitly deferred; this slice supports a source checkout with a documented adapter build step.

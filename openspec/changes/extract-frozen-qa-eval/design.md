## Context

The target is one source-checkout command:

```bash
uv run dimos eval run \
  dimos/benchmark/short_horizon_qa/cases/demo_go2_hongkong_office-room-count-smoke/case.json \
  --output=/tmp/dimos-eval-smoke
```

The first implementation on PR #3378 was intentionally defensive, but it produced 17 attempt files and three copies of the agent trajectory. Review established that Memory2 already owns environmental observations and Pi already owns its native transcript. The evaluator needs only a compact result beside that transcript.

## Goals

- Keep the direct command small and understandable.
- Preserve extension seams only for source, task, validator, and the shared CodePolicy session.
- Prevent writes to source and derived Memory2 databases.
- Keep the private oracle and API key outside the model-facing namespace.
- Use official MCP transports and Pi's stock process lifecycle.
- Leave no child process after normal completion, caught failure, timeout, or interruption.
- Pass the repository's existing CI with focused Python and Node unit tests.

## Non-Goals

- Generic attempt engines, adapter registries, dataset scheduling, retries, or distributed execution.
- Crash-forensic event logs, artifact descriptors, cryptographic result attestations, or replay guarantees.
- OAuth, live robot RPCs, simulation, DimSim, blueprints, or agentic module integration.
- Migrating the existing repository-wide DimOS MCP implementation.
- Treating Jupyter or read-only Memory2 as a hostile-code sandbox.

## Architecture

```text
case.json + private oracle
        |
        v
frozen bundle cache -----> FrozenMemoryStore
                                |
                                v
evaluator process -----> CodePolicySession -----> Jupyter kernel child
        |                       ^
        | official MCP server  | python_exec
        v                       |
stock Pi CLI child ---- tiny official-MCP extension
        |
        v
native JSON events + transcript -> parse -> private score -> result.json
```

### Python modules

- `dimos/agents/code_policy.py`: the reusable Jupyter session, environment bootstrap, timeout recovery, and credential-scrubbed kernel launch. It knows nothing about MCP, Pi, or evaluation.
- `dimos/memory2/store/frozen.py`: source/derived overlay and inclusive cutoff.
- `dimos/benchmark/agent_eval/models.py`: strict tagged case and compact result contracts.
- `dimos/benchmark/short_horizon_qa/prepare.py`: recording resolution and derived map cache.
- `dimos/benchmark/short_horizon_qa/eval.py`: in-process MCP host, stock Pi launcher/event parser, answer parser, private scorer, cleanup, and output publication.
- `dimos/cli/eval.py`: dependency-light Typer shell with callback-local runtime imports.

Small helpers may remain separate only when they own a concrete lifecycle boundary; generic artifact, engine, broker, and Protocol layers are removed.

### Official MCP

Pin the official Python `mcp==2.0.0` SDK and register one `python_exec` tool. The evaluator pre-binds a loopback socket to port `0`, supplies it to Uvicorn, and runs the SDK's ASGI application on a server thread. The evaluator directly owns server shutdown and the `CodePolicySession`; no `/control` HTTP surface or extra Python process exists.

Pin `@modelcontextprotocol/client==2.0.0` in the Node extension. The extension validates the one-tool inventory, calls `python_exec`, uses a timeout longer than the kernel execution timeout, and closes the MCP session during Pi shutdown.

### Stock Pi process

Launch pinned Pi `0.80.10` with `--mode json`, built-in tools disabled, and one explicit extension. Python consumes Pi's official JSON event stream and inspects the final assistant stop reason rather than trusting the process exit code alone. The Pi-native session JSONL is the sole trajectory record.

API-key material is passed only in the Pi subprocess environment. It is absent from argv and removed from the environment supplied to the Jupyter kernel.

### Output

`--output` is the exact directory for one run. A non-empty target fails preflight. Work occurs in a sibling temporary directory and is atomically renamed on completion or caught failure.

Published files are:

- `result.json`: case/source/model, response, prediction, score, timing, tool count, and optional infrastructure error.
- `transcript.jsonl`: Pi's native session when available.
- `stderr.log`: bounded Node diagnostics only when nonempty.

There are no nested attempt IDs, locks, manifests, copied cases/oracles/cache manifests, hashes, lifecycle logs, or duplicate call records.

## Decisions

1. **Lean feature-specific runner.** A second evaluator can justify a generic engine later.
2. **Real shared CodePolicy core.** This PR owns the production Jupyter session; experimental PR #3259 can later wrap it as a DimOS module.
3. **In-process MCP, Jupyter child.** Jupyter already supplies the execution process boundary, interrupt, restart, and shutdown.
4. **Official SDKs on both sides.** The new standalone boundary does not reuse or expand DimOS's legacy hand-written MCP transport.
5. **Stock Pi JSON mode.** A tiny extension replaces the custom adapter and broker protocol.
6. **Minimal durable output.** Memory2 owns observations; Pi owns trajectory; `result.json` owns scoring.
7. **Minimal Memory2 edits.** Read-only connection propagation is required because existing constructors configure WAL and create tables. Existing time-range filtering supplies the inclusive cutoff; general stream/filter APIs are not expanded.
8. **No cryptographic framework.** Strict parsing, safe paths, and cache metadata are sufficient for this local demo stage.
9. **API key only.** Default to `OPENAI_API_KEY`, with a named environment override for later extension.
10. **Existing CI plus required Node tests.** Python test/lint groups receive the minimal runtime dependencies; the small Node job gates the aggregate check.

## Safety and cleanup

The evaluator owns cleanup in one `finally`: stop Pi, stop the MCP server thread, and shut down Jupyter. Timeouts escalate Pi terminate to kill. The kernel environment is scrubbed of common credential variables. Private oracle values never enter prompts, MCP metadata, transcripts, results, or stderr.

CodePolicy remains trusted unsandboxed execution because kernel code may access the host filesystem and start processes. Documentation must state this directly.

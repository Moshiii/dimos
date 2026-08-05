## Why

DimOS needs one direct command that asks an agent an integer question about a frozen Memory2 recording. The first extraction proved the path, but review showed that it introduced a generic evaluation framework, a custom Node protocol, duplicated evidence, and integrity machinery before those abstractions had a second use.

This revision keeps the production seams that matter: true read-only Memory2, a reusable Jupyter-backed CodePolicy session, official MCP libraries, a stock Pi process, strict case/result models, and private scoring. It removes the custom orchestration and audit framework.

## What Changes

- Add `dimos eval run CASE --output=DIR` for one synchronous frozen QA run.
- Keep a compact tagged case model for source, task, and validator kinds.
- Add true read-only source/derived Memory2 views with an inclusive cutoff.
- Add a module-independent Jupyter `CodePolicySession` and expose its sole `python_exec` operation through the official Python MCP SDK in the evaluator process.
- Launch the stock Pi CLI in one-shot JSON mode with one small TypeScript extension using the official MCP client.
- Support API-key authentication through a named environment variable only.
- Publish only `result.json`, the native Pi transcript, and failure-only stderr in one non-overwriting output directory.
- Rename the Hong Kong fixture and case ID with `demo_`/`demo-` to make its synthetic `0` oracle unmistakable.
- Remove fingerprints, artifact manifests, lifecycle logs, broker logs, duplicated execution records, and the custom Python/Node protocol.

## Capabilities

### New Capabilities

- `frozen-agent-evaluation`: one-case CLI execution, private integer scoring, compact output, and exit behavior.
- `frozen-memory-views`: read-only source/derived Memory2 access through an inclusive authored cutoff.
- `standalone-code-policy-runtime`: reusable Jupyter session plus an in-process official MCP adapter and stock Pi CLI integration.

### Modified Capabilities

None.

## Impact

The change affects Memory2 SQLite opening, agent optional dependencies, the DimOS CLI, one focused benchmark package, a small Node extension package, CI dependencies, and agent documentation. It adds no blueprint, robot skill, live evaluation path, simulator, or generated blueprint entry.

CodePolicy remains trusted unsandboxed Python. The Jupyter kernel receives a scrubbed environment without API credentials, but it is not an operating-system sandbox.

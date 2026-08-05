## Why

DimOS has a working frozen short-horizon QA evaluation path on the long-running `cc/frontier` branch, but the implementation is entangled with unrelated DimSim, spatial benchmark, manipulation, runtime, and UI work. That makes the final feature commits unsafe to cherry-pick and prevents the focused `dimos eval run` workflow from landing on `main` with a reviewable dependency boundary.

This change extracts the proven frozen-recording path as a single-case, synchronous evaluation capability. It preserves private-oracle isolation, immutable evidence, read-only Memory2 access, fresh agent sessions, and deterministic scoring while explicitly excluding live robot evaluation and the broader benchmark stack.

## What Changes

- Add the public `dimos eval run CASE` CLI for one immutable frozen-memory evaluation case.
- Add strict frozen source, integer-question, one-attempt interaction, and exact-integer validator contracts with deterministic case fingerprints.
- Add read-only Memory2 snapshots that combine source and derived streams through an inclusive timestamp cutoff.
- Add a standalone loopback CodePolicy MCP process and a dedicated Node/Pi adapter exposing exactly one tool, `python_exec`.
- Add append-only attempt storage, private/public evidence separation, progress streaming, exact terminal-answer parsing, and explicit exit semantics.
- Add the Hong Kong office plumbing fixture with a clearly non-authoritative synthetic oracle.
- Add only the optional Python and Node dependencies required by this focused path.
- No existing public API is removed or changed; this is an additive CLI capability.

## Affected DimOS Surfaces

- Modules/streams: Memory2 SQLite stores, stream filters, frozen source/derived overlays, standalone CodePolicy runtime, and generic agent-evaluation orchestration.
- Blueprints/CLI: new `dimos eval run` command; no blueprint composition or generated blueprint registry changes.
- Skills/MCP: loopback MCP exposure of one trusted `python_exec` tool; no robot `@skill` additions.
- Hardware/simulation/replay: consumes an existing recording and derived map only; no hardware control, live DimSim evaluation, simulation scheduling, or replay blueprint changes.
- Docs/generated registries: new agent-evaluation capability documentation and fixture warning; no `all_blueprints.py` regeneration.

## Capabilities

### New Capabilities

- `frozen-agent-evaluation`: Single-case CLI execution, immutable case contracts, scoring, evidence, progress, privacy, and exit behavior.
- `frozen-memory-views`: Read-only source/derived Memory2 views bounded by an inclusive authored cutoff.
- `standalone-code-policy-runtime`: Fresh loopback CodePolicy and Pi processes with credential-safe setup, a one-tool inventory, bounded protocol handling, and reliable cleanup.

### Modified Capabilities

None. The affected behavior is not currently represented by an OpenSpec capability spec on `main`.

## Impact

Users gain a reproducible command for evaluating one frozen QA case and inspecting immutable attempt artifacts. The base CLI remains usable without the `agents` extra; evaluation requires the agents dependencies plus a built, pinned Node adapter. Credentials remain runtime-only and are never accepted as CLI secret values or serialized into evidence.

The primary compatibility risks are optional-dependency import leakage, SQLite mutation through an allegedly frozen view, subprocess cleanup failures, private-oracle disclosure, and source-checkout discovery of the Node entrypoint. Validation therefore includes focused Python and Node suites, minimal-dependency CLI subprocess tests, failure-injection and lock-release tests, a self-hosted recording mechanics gate, and one credentialed end-to-end smoke run.

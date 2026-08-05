## 1. Extraction Baseline and Dependencies

- [x] 1.1 Ensure the implementation branch is based on `origin/main` SHA `e8a985d83a85c9827fa89ed7526e40a822eb1ae3`, record that base in the PR, and use `30e5f1c0e` only as the file-content reference.
- [x] 1.2 Add the focused `agent_eval` foundation modules for strict base models, canonical JSON, artifact/lifecycle records, and runtime credentials without importing live DimSim or spatial benchmark packages.
- [x] 1.3 Update `pyproject.toml` so the `agents` extra contains the Jupyter kernel/client, nbformat, pyzmq, FastAPI, and Uvicorn dependencies required by CodePolicy, then regenerate `uv.lock` without carrying unrelated reference changes.
- [x] 1.4 Add an import-boundary test that recursively checks the focused evaluation slice for forbidden live DimSim and spatial benchmark imports.

## 2. Frozen Memory2 Views

- [x] 2.1 Add a read-only option to Memory2 SQLite connection helpers using SQLite URI `mode=ro` and `PRAGMA query_only=ON`, while retaining WAL configuration only for writable connections.
- [x] 2.2 Propagate read-only mode through the Memory2 registry, observation store, SQLite store, and stream APIs, rejecting stream creation, append, deletion, and every other mutation path.
- [x] 2.3 Add inclusive through-time filtering (`observation.ts <= cutoff`) and ensure transformed read-only streams preserve the mutation boundary.
- [x] 2.4 Add the frozen source/derived overlay with deterministic stream listing, collision rejection, no stream creation/retyping, and the inclusive cutoff applied to every stream.
- [x] 2.5 Add Memory2 tests for exact-boundary visibility, source/derived union, collisions, mutation rejection, unchanged database bytes, and absence of WAL sidecars.

## 3. Standalone CodePolicy Runtime

- [x] 3.1 Port the module-independent trusted CodePolicy kernel runtime with lazy Jupyter imports and the actionable `uv sync --extra agents` error.
- [x] 3.2 Add frozen-memory environment setup that preloads read-only `memory`, omits live `app`, bounds execution output/records, and retains session receipts.
- [x] 3.3 Add the standalone FastAPI/Uvicorn MCP process on an ephemeral loopback port, including startup receipt, control endpoint, bounded shutdown, and terminate-to-kill escalation.
- [x] 3.4 Update MCP readiness polling to retry both connection failures and read timeouts until the configured deadline.
- [x] 3.5 Add CodePolicy and MCP tests for fresh sessions, namespace isolation, exactly one exposed tool, readiness retry, evidence retention, timeout/interruption behavior, partial startup, and child-process cleanup.

## 4. Evaluation Contracts, Storage, and Engine

- [x] 4.1 Add frozen-only source, integer-question, one-attempt interaction, exact-integer validator, request, prediction, private score, and terminal outcome contracts with strict unknown-field rejection and deterministic fingerprints.
- [x] 4.2 Add tests proving fingerprints exclude credentials, ports, output/host paths, reject semantic overrides and unsafe oracle paths, and produce validator-free public projections.
- [x] 4.3 Add the append-only attempt store with mode-`0700` attempt directories, a nonblocking output-root lock, safe relative paths, exclusive artifact creation, SHA-256 descriptors, fsync, monotonic lifecycle events, and atomic terminal publication.
- [x] 4.4 Add the generic source/interaction/validator/agent adapter Protocols and attempt engine with private/public evidence separation and resource cleanup in reverse dependency order.
- [x] 4.5 Structure attempt execution so store closure and lock release occur in an outer `finally`, regardless of event, manifest, fsync, terminal-publication, cleanup, interruption, or partial-startup failures.
- [x] 4.6 Add fault-injection tests for events, artifact fsync, directory fsync, manifest, terminal-link publication, cleanup, and interruption failures, asserting retained prefixes, correct status, no live children, and immediate lock reacquisition.

## 5. Dedicated Node/Pi Adapter

- [x] 5.1 Create `packages/pi-code-policy-adapter` with Node `>=22.19.0`, pinned Pi dependencies `0.80.10`, compatible TypeBox/TypeScript dependencies, build/typecheck/test configuration, lockfile, and a source-checkout README.
- [x] 5.2 Extract and simplify the newline-delimited code-policy protocol with bounded frames, strict inbound validation, unique call/reply correlation, protocol-only stdout, and bounded diagnostic stderr.
- [x] 5.3 Implement the dedicated `python_exec` definition and Pi session setup without spatial tools, disabling built-ins/extensions/skills/templates/context files and asserting the exact one-tool inventory before and after activation.
- [x] 5.4 Implement OAuth and API-key runtime setup, pinned model/thinking validation, fresh session creation, prompt/session evidence, abort/dispose propagation, and no secret process arguments or evidence fields.
- [x] 5.5 Add Node tests for entrypoint behavior, valid calls, malformed/unknown/duplicate/oversized frames, tool-inventory drift, authentication configuration, prompt/session evidence, abort, disposal, and stdout/stderr separation.
- [x] 5.6 Add the adapter's npm test command to the appropriate required CI workflow without adding generated `dist` or `node_modules` content.

## 6. Frozen QA Preparation and Execution

- [x] 6.1 Port normalized-progress validation, recording resolution, derived `global_map` preparation, cached bundle manifests, cutoff receipts, and source/derived integrity descriptors using current `main` mapping and Memory2 APIs.
- [x] 6.2 Add the frozen source driver, exact-integer oracle loader and SHA-256 verification, terminal `ANSWER: <integer>` parser, private validator, and frozen CodePolicy interaction driver.
- [x] 6.3 Add the Python/Pi broker and process wrapper with bounded line frames, bounded progress/stderr, correlated tool replies, evidence references, startup/turn timeouts, abort, disposal, and terminate-to-kill cleanup.
- [x] 6.4 Add single-case preflight and execution orchestration with source-checkout discovery of `packages/pi-code-policy-adapter/dist/code-policy-main.js` and an actionable missing-build error.
- [x] 6.5 Add tests for bundle reuse and integrity, progress `0`/`1` boundaries, exact answer parsing, validator mismatch, malformed semantic failure, fresh process/session identities, cleanup, and complete attempt evidence.
- [x] 6.6 Add privacy tests that seed unique oracle and credential sentinels and prove they do not appear in the public projection, prompt, progress, compact result, CodePolicy namespace, Pi transcript/evidence, broker log, or serialized runtime configuration.

## 7. CLI and Fixture

- [x] 7.1 Add the dependency-light `dimos eval` Typer shell and callback-local heavy imports so ordinary base CLI commands do not require the `agents` extra.
- [x] 7.2 Implement the documented options, auth inference/precedence, default output root, compact/human results, stderr progress, `--quiet`, and exit codes `0`, `1`, and `2` at the preflight/attempt boundary.
- [x] 7.3 Add subprocess CLI tests for typed help, unsupported values, auth combinations, stdout/stderr separation, quiet mode, semantic failure, infrastructure failure, preflight failure, missing agents dependencies, and missing Node build.
- [x] 7.4 Port exactly the Hong Kong smoke fixture's `case.json`, private oracle, and warning README, preserving oracle SHA-256 and the explicit synthetic-`0` warning.
- [x] 7.5 Confirm no blueprint, module registry input, or `all_blueprints.py` output changes are introduced; no blueprint-regeneration task is required.

## 8. Documentation

- [x] 8.1 Add `docs/capabilities/agents/evaluation.md` with setup, adapter build, CLI options, authentication, output/exit behavior, evidence/privacy, cleanup, and trusted-unsandboxed warnings.
- [x] 8.2 Link the evaluation guide from the agent capability index and update `docs/development/testing.md` with Python, Node, self-hosted, and credentialed smoke procedures.
- [x] 8.3 Inspect `docs/coding-agents/index.md` and update it only if it enumerates feature-specific validation surfaces; leave `AGENTS.md` unchanged.
- [x] 8.4 Retain the fixture warning and decide in the PR whether `frozen-qa-main-extraction-handoff.md` remains as provenance or the OpenSpec artifacts supersede it.

## 9. Verification and Manual QA

- [x] 9.1 Run `openspec validate extract-frozen-qa-eval` and resolve all proposal/spec/design/docs/task validation errors.
- [x] 9.2 Run `npm ci --prefix packages/pi-code-policy-adapter`, its typecheck/build/tests, and verify no unexpected generated files are tracked.
- [x] 9.3 Run the focused Python CLI, agent-evaluation, frozen-QA, CodePolicy, MCP adapter, and frozen-Memory2 pytest targets listed in the handoff.
- [x] 9.4 Run the minimal-dependency base CLI subprocess regression, relevant lint/format checks, and `uv run mypy` for the added focused packages.
- [x] 9.5 Run `uv run doclinks` and the repository-supported executable-Markdown check for `docs/capabilities/agents/evaluation.md` when applicable.
- [x] 9.6 On a host with the LFS recording, run the `self_hosted` Hong Kong mechanics gate using `CPU:0` and verify its expected cutoff/map evidence.
- [ ] 9.7 Build the Node adapter and run the exact credentialed `uv run dimos eval run ... --output=/tmp/dimos-eval-smoke` command, accepting semantic failure against the synthetic oracle only when infrastructure and evidence complete.
- [x] 9.8 After the operational smoke, verify Pi and CodePolicy processes are gone, stdout/stderr obey the contract, credentials/oracle material are absent from public evidence, artifacts pass their descriptors, and the output lock can be immediately reacquired.

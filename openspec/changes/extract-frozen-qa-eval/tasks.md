## 1. Rebaseline

- [x] 1.1 Rewrite the OpenSpec proposal, design, capability specs, and tasks for the reviewer-approved lean architecture.
- [x] 1.2 Replace optional dependencies with official MCP SDK pins and the minimal existing-CI dependency set; regenerate Python and Node locks.

## 2. Frozen Memory2

- [x] 2.1 Retain read-only propagation only through the SQLite helper, registry, observation store, and SQLite store.
- [x] 2.2 Remove the new general `ThroughFilter` and stream writability APIs; implement the inclusive cutoff inside the frozen facade with existing filters.
- [x] 2.3 Keep focused tests for no-WAL reads, source/derived overlay, exact cutoff, collisions, and rejected mutations.

## 3. Production CodePolicy and MCP

- [x] 3.1 Collapse the Jupyter implementation into a reusable module-independent `CodePolicySession` with frozen bootstrap, timeout recovery, shutdown, and scrubbed kernel credentials.
- [x] 3.2 Replace the hand-written standalone process/server/control API with an in-process official MCP server exposing exactly `python_exec` on a pre-bound loopback socket.
- [x] 3.3 Add hermetic tests for fresh namespaces, no `app`, no credentials, interrupt/restart, exact tool inventory, race-free startup, and bounded shutdown.

## 4. Stock Pi CLI Extension

- [x] 4.1 Replace `pi-code-policy-adapter` with the minimal `pi-code-policy-extension` package using pinned Pi `0.80.10` and official MCP client `2.0.0`.
- [x] 4.2 Register exactly `python_exec`, validate the MCP inventory, forward calls directly, and close the client on Pi shutdown.
- [x] 4.3 Launch stock Pi `--mode json`; parse official events for final text, stop reason, tool count, and native transcript without a custom protocol or Python broker.
- [x] 4.4 Add focused Node extension tests and hermetic Python event-parser/process-cleanup tests.

## 5. Lean Evaluation and CLI

- [x] 5.1 Consolidate strict source/task/validator/result contracts in `agent_eval/models.py`; remove interaction/runtime/agent/fingerprint/artifact models and generic adapter Protocols.
- [x] 5.2 Simplify frozen bundle caching to recording metadata, mapper settings, and cutoffs without cryptographic descriptors.
- [x] 5.3 Replace the attempt engine/store with one runner that privately loads the oracle, runs CodePolicy and Pi, parses `ANSWER: <integer>`, and publishes compact output atomically.
- [x] 5.4 Persist only `result.json`, the native transcript when available, and nonempty bounded stderr; refuse a non-empty output directory.
- [x] 5.5 Support API-key environment authentication only and preserve exit codes `0` completed, `1` caught infrastructure failure, and `2` preflight failure.
- [x] 5.6 Rename the fixture directory and case ID with `demo_`/`demo-`, retaining the synthetic-`0` warning.
- [x] 5.7 Keep the base CLI dependency-light and add behavior-focused tests for case validation, privacy, scoring, output, cleanup, and CLI stdout/stderr.

## 6. Documentation and CI

- [x] 6.1 Update the evaluation guide, fixture README, agent index, and testing guide for the stock Pi extension, API-key-only auth, compact output, and trusted-unsandboxed boundary.
- [x] 6.2 Make existing Python test/lint environments install the minimal evaluation dependencies and make the Node extension job gate aggregate CI.
- [x] 6.3 Remove documentation and tests for obsolete fingerprints, evidence stores, protocols, OAuth, and attempt locks.

## 7. Verification and Review

- [x] 7.1 Validate OpenSpec and run focused Python, Node, Ruff, mypy, doclinks, and executable-Markdown checks.
- [x] 7.2 Run the real Hong Kong self-hosted CPU mechanics gate.
- [ ] 7.3 Build the extension and run the exact API-key smoke command when credentials are available; verify compact output and child cleanup. (The extension and preflight path are verified; this workspace has no `OPENAI_API_KEY` for the live model call.)
- [x] 7.4 Commit and push the redesign, update draft PR #3378, and reply to all review threads with the agreed resolutions.

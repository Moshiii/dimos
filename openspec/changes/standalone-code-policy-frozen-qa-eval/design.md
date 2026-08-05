## Context

There are currently two adjacent but mismatched paths:

- `dimos.benchmark.short_horizon_qa` prepares a read-only recording plus derived `global_map`, then constructs a two-worker DimOS blueprint containing `CodePolicyModule` and `McpServer`. It deliberately leaves question authoring and scoring to an unspecified external harness.
- `dimos.benchmark.agent_eval` runs the existing Pi code-policy agent against an already-running DimSim evaluation blueprint. Its Pi process and evidence components are reusable, but its top-level models and runner are specialized around DimSim release selection, reset, native evaluation, motion cancellation, and native terminal results.

`CodePolicyModule` itself has no typed streams and uses no robot-module composition. It owns a Jupyter kernel, bootstraps `memory` and optionally `app`, exposes one `python_exec` skill, records execution evidence, and provides host-side lifecycle RPCs. The blueprint supplies process spawning and an indirect MCP-to-RPC call path rather than a domain capability.

This change uses a direct north-star to shape the cleanup: run the existing Pi code-policy agent over the complete Hong Kong office recording, ask `How many rooms in total?`, and validate the typed integer answer. The complete recording is authored as normalized progress `1.0`, not a special end flag or a fragile relative-second literal.

## Goals / Non-Goals

**Goals:**

- Establish `source + task + interaction + validator` as the canonical semantic case shape for future generic agent evaluations.
- Make `interaction` an executable description of the actual agent-facing feed and lifecycle, not a coarse runtime or source label.
- Extract CodePolicy into a standalone, runner-owned MCP service with frozen and live environment profiles.
- Keep agent-visible memory read-only; preserve automatic CodePolicy and Pi evidence separately from any future agent-authored log.
- Refactor shared Pi/session/attempt/prediction/validation/evidence behavior out of the live DimSim-specific runner.
- Migrate the live DimSim smoke to the shared foundation without changing its native reset and scoring semantics.
- Add a frozen-QA runner and one real, inspectable Hong Kong office acceptance attempt.

**Non-Goals:**

- Migrate the hardened spatial Pi baseline, experiment scheduler, or every historical benchmark in this change.
- Define one universal source payload, tool set, simulator protocol, or validator implementation.
- Add writable Memory2 access, an agent journal, or a general arbitrary Python-binding registry.
- Sandbox trusted policy code or make leaderboard/fairness claims.
- Add scheduling, retries, parallel execution, publication, aggregation, or a review UI.
- Change the Hong Kong recording, silently manufacture its oracle, or infer correctness from the tested agent itself.
- Add a second model-facing tool solely to submit answers; the canonical interaction remains the one-tool CodePolicy interface.

## Decisions

### 1. Make the four-part case the stable semantic boundary

The generic compiled case has four immutable components:

```python
class EvalCase(BaseModel):
    case_id: str
    source: SourceSpec
    task: TaskSpec
    interaction: InteractionSpec
    validator: ValidatorRef
    fingerprint: str
```

Their responsibilities are deliberately narrow:

```text
source       what immutable evidence or environment exists
task         what the agent is asked to do and response shape
interaction  exactly how evidence, tools, timing, and completion reach the agent
validator    private executable definition of success
```

The public projection contains source provenance safe for the agent, the task, and the public interaction description. The compiled private record binds the validator reference and all content digests. Runtime objects, ports, process IDs, credentials, open stores, and simulator handles never enter semantic case identity.

Multiple questions may reference one source selection, but each question compiles to an independent case and receives a fresh agent/CodePolicy session. A future explicitly multi-turn benchmark can define a multi-question task and interaction; it must not obtain shared history accidentally through source reuse.

Alternative considered: retain the current pattern where each runner defines a single config joining source, agent, reset, and scoring fields. Rejected because it makes the agent-visible interface implicit and produces another top-level architecture for every evaluation kind.

### 2. Separate semantic interaction from infrastructure runtime binding

`InteractionSpec` selects a versioned typed driver and the configuration that affects what the agent experiences. `AttemptRequest` separately binds an agent condition and machine/runtime details:

```python
class AttemptRequest(BaseModel):
    case: EvalCase
    agent: AgentSpec
    runtime: RuntimeBinding
    seed: int | None
```

Changing from frozen `memory` to live `memory + app`, changing task-delivery timing, or sharing a session changes the interaction and therefore the case. Changing a temporary port, output root, worker machine, or credential binding changes the attempt/runtime but not the case.

An interaction driver owns the concrete lifecycle:

```python
class InteractionDriver(Protocol):
    def run(
        self,
        *,
        prepared_source: PreparedSource,
        task: TaskSpec,
        agent: AgentAdapter,
        attempt: AttemptContext,
        evidence: EvidenceSink,
    ) -> AgentOutcome: ...
```

This is intentionally a deep interface. A frozen driver can launch one local service and await one answer; a live driver can attach through porcelain, observe realtime activity, continue a Pi session, and coordinate cancellation. They share results and evidence contracts without pretending their lifecycle steps are identical.

Alternative considered: describe interactions as generic lists of tools, streams, prompts, and timeouts. Rejected because that becomes a weak workflow interpreter and still cannot faithfully express reset, readiness, native observation, or cleanup.

### 3. Keep validation private and independently executable

The evaluation engine resolves the source, starts any private validator lifecycle, runs the public interaction, and asks the validator to finish:

```text
compile case
   │
   ▼
prepare source ───────────────► private source handle
   │                                  │
   │ public binding                   ▼
   ├──────────────────────────► validator session
   │                                  │
   ▼                                  │
interaction driver → AgentOutcome     │
   │                                  │
   └──────────────────────────────► finish/evaluate
                                      │
                                      ▼
                                  PrivateScore
```

For frozen integer QA, validator preparation only loads the private oracle and evaluation is post-hoc. For live DimSim, validator preparation starts native observation before agent dispatch and finishing consumes the correlated native result. Neither path places validator state in `memory`, `app`, prompts, MCP results, or public outcome material.

Alternative considered: make the interaction driver return a boolean task result. Rejected because it couples how the agent receives evidence to how correctness is established and cannot preserve the public/private boundary.

### 4. Use normalized recording progress as the authored time selection

A recording selection is authored with finite `progress` in `[0, 1]`. Preparation first seals and inspects the source range:

```python
recording_start = min(nonempty_stream_starts)
recording_end = max(nonempty_stream_ends)
cutoff = recording_start + progress * (recording_end - recording_start)
```

The endpoints resolve to the exact observed range endpoints; in particular, `1.0` is the exact inclusive end. Interior selections resolve linearly. The manifest records:

- authored normalized progress;
- resolved seconds from recording start;
- resolved absolute timestamp;
- per-stream last included identity and timestamp;
- selected runtime-cadence map emission and integrated frame count.

Prepared service startup selects the manifest record by canonical progress and uses its already-resolved absolute cutoff. It does not recompute against the source. Numeric relative-second preparation may remain as a low-level compatibility input during migration, but new authored cases use progress.

Alternative considered: add `end_of_recording`. Rejected because it solves only one boundary case and does not provide a stable dataset-relative selection for intermediate horizons.

### 5. Extract a plain CodePolicy session and direct standalone MCP host

The extracted structure is:

```text
dimos/agents/code_policy/
├── models/config       environment and evidence records
├── session             kernel, execution, reset, recovery
├── environments        frozen and live bootstrap generation
├── server              direct MCP host and host controls
└── module              temporary compatibility wrapper
```

The exact file split may follow repository naming conventions, but dependencies point inward:

```text
CodePolicyModule ───────┐
                       ▼
Standalone MCP ───► CodePolicySession ───► Jupyter kernel
```

`CodePolicySession` is a plain Python object. It owns the execution lock, kernel manager/client, session and execution identities, bounded output, interruption/recovery, observer descriptors, and records. It has no `Module`, `ModuleConfig`, `@rpc`, `@skill`, blueprint, or transport dependency.

The standalone host exposes one direct MCP callable, `python_exec`, and a separate host-side control surface for readiness, reset, interruption, session receipts, observer preparation, and record collection. Control operations must not appear in MCP tool inventory. The evaluation driver owns the service process and control client, and chooses an available endpoint rather than baking port `9990` into case identity.

The generic existing `McpServer` may be refactored to accept a direct callable registry if that produces a clean non-module dependency. Otherwise the standalone host implements the narrow one-tool MCP protocol without importing module RPC machinery.

Alternative considered: keep `autoconnect(CodePolicyModule, McpServer)` behind a new CLI. Rejected because it preserves the unnecessary coordinator, two module workers, RPC hop, and blueprint coupling that this cleanup exists to remove.

### 6. Support exactly two initial environment profiles

The environment union is intentionally closed:

```python
class FrozenMemoryEnvironment(BaseModel):
    kind: Literal["frozen_memory"]
    source_path: Path
    derived_path: Path
    cutoff_timestamp: float


class LiveDimosEnvironment(BaseModel):
    kind: Literal["live_dimos"]
    memory_path: Path
    porcelain_target: PorcelainTarget
```

Their kernel namespaces are authoritative:

| Profile | `memory` | `app` |
|---|---|---|
| frozen | read-only `FrozenMemoryStore` | absent |
| live | read-only live Memory2 store | `Dimos.connect(...)` through porcelain |

Memory mutation through the provided API is rejected in both modes. Automatic execution transcripts remain private evidence. If an agent-authored persistent journal is needed later, it will be a separate explicit binding such as `agent_log`, with its own provenance and retention policy; it will not make world memory writable.

Alternative considered: a generic mapping from variable names to arbitrary import/bootstrap snippets. Rejected because it weakens the stable agent interface and creates avoidable configuration and security surface before a real third environment exists.

### 7. Refactor generic evaluation around shared engine and drivers

The current `LocalAgentEvalRunner` is decomposed into shared primitives and live-specific drivers:

```text
generic agent_eval
├── case models/compiler
├── attempt engine and store
├── source preparation protocol
├── interaction protocol
├── validator protocol
├── Pi session/process adapter
├── prediction and score records
└── normalized outcome

drivers
├── frozen Memory2 source
├── standalone frozen CodePolicy interaction
├── live DimOS/DimSim source and interaction
├── exact-integer validator
└── DimSim native validator
```

`PiTurn`, Pi process ownership, authentication binding, `PythonExecBroker`, CodePolicy call logging, native Pi session retention, artifact references, and infrastructure/task outcome rules become shared. Frozen QA does not import DimSim models. Live DimSim preserves authoritative reset, correlated native evaluation, continuation behavior, motion cancellation, and native-result evidence behind its source/interaction/validator implementations.

The current external-Pi DimSim evaluation blueprint removes `CodePolicyModule` and its agent-facing `McpServer`. The live interaction starts standalone CodePolicy and connects `app` to the already-running blueprint through porcelain. Other normal robot MCP services are not part of the evaluated model interface.

Alternative considered: add a small frozen runner beside `LocalAgentEvalRunner`. Rejected because it would validate the north-star while leaving the repository with the same duplicated lifecycle problem the change is meant to clean up.

### 8. Treat Pi final text as a typed prediction only for QA

The existing Node Pi code-policy adapter already returns `final_text` while exposing exactly `python_exec`. Frozen integer tasks add a public terminal protocol:

```text
End your final response with exactly: ANSWER: <integer>
```

The parser accepts exactly one terminal marker, retains the full final response, and writes a separate immutable prediction record. Explanatory prose may precede the marker. Missing, duplicate, non-integer, or non-terminal markers yield a parser-failure prediction state and a completed failed task, not infrastructure failure.

This does not add `submit_answer`, because doing so would violate the authoritative one-tool CodePolicy interaction and would make frozen and live agent interfaces diverge.

Alternative considered: extract the first integer or use an LLM judge. Rejected because room-count reasoning and explanations can contain unrelated numbers, while an LLM judge makes the first north-star nondeterministic and harder to audit.

### 9. Make the Hong Kong case a real acceptance gate

The north-star artifacts include:

- public authored case selecting `go2_hongkong_office`, progress `1.0`, the exact question, integer response, frozen CodePolicy interaction, and pinned Pi condition;
- private oracle with expected integer, explicit counting policy, room inventory, and human evidence notes;
- prepared source/derived manifest with hashes and exact cutoff boundaries;
- a documented one-command credentialed run.

Acceptance requires a real Pi session, at least one successful `python_exec` call, a parsed integer prediction equal to the approved oracle, complete attempt evidence, no running DimOS blueprint, and confirmed standalone service teardown. Deterministic fake-agent and scripted integrations protect mechanics in automated tests; the credentialed run is the direct behavioral gate and is not placed in ordinary credential-free CI.

The evaluator must not derive the oracle from the tested Pi run. Two reviewers or one author plus an explicit review step should approve the room inventory before enabling the acceptance assertion.

### 10. Preserve evidence and distinguish completion from correctness

One attempt directory contains immutable case/source receipts, events, MCP inventory, CodePolicy session/calls, native Pi session and prompt sidecars, final response, prediction, private score, manifest, and exactly one outcome. Existing evidence helpers are reused where their contracts are generic; simulation-specific fields are moved into backend sidecars rather than made nullable fields in every frozen record.

Outcome semantics remain:

| Infrastructure | Validation | Attempt status | Task result |
|---|---|---|---|
| complete | pass | completed | passed |
| complete | wrong/malformed | completed | failed |
| incomplete | unavailable | failed | not evaluated |

Exit status continues to distinguish operational validity from task correctness for reusable evaluation commands. The north-star acceptance wrapper additionally requires `task_result=passed`.

### 11. Make one static case the first public evaluation CLI boundary

The first general command is a synchronous single-attempt entry point, not a
blueprint and not a dataset scheduler:

```bash
dimos eval run ./case.json \
  --agent.backend=pi \
  --agent.model=gpt-5.6-luna \
  --output=./results
```

`EvalCase` and all persisted result records derive from one strict immutable
`BaseEvalModel` and use Pydantic `model_validate_json()` / `model_dump_json()`.
The positional case completely fixes source, task, interaction, and validator;
the CLI cannot replace any semantic component. An exact-integer validator's safe
relative path is resolved against the case document's directory, and its digest
is checked before agent dispatch.

Execution configuration is a separate typed Pydantic tree. Backend-discriminated
agent configuration is exposed through dotted long options rather than the
blueprint-oriented `-o` escape hatch. Pi defaults to the repository-supported
model and medium thinking. When no authentication option is supplied, the CLI
selects the standard `OPENAI_API_KEY` environment binding if it is nonempty and
otherwise falls back to Codex OAuth. An explicit mode, path, or environment
binding takes precedence over inference. Authentication configuration names an
environment or file binding and never accepts a credential literal. The
resolved condition and sanitized binding provenance enter attempt evidence, not
case identity.

The default view is a concise human summary over the typed engine result.
`--json` emits one compact summary object; complete case snapshots, prediction,
private score, events, Pi/CodePolicy evidence, manifest, and terminal outcome stay
in the attempt directory. `--output` changes artifact placement only.

Frozen storage paths and any derived-map sidecar remain private Memory2 binding
details. The agent receives only read-only `memory`, and the public CLI does not
accept `--replay`, `--recording`, `--bundle`, `--source-root`, `--private-root`,
question, or validator overrides.

Alternative considered: `dimos --replay run evals -o ...`. Rejected because
frozen evaluation is not a blueprint replay, and because generic blueprint
overrides would permit semantic case mutation and leak storage implementation
details. Dataset batching, containers, parallel execution, retries, and
scheduling remain later layers over the same single-attempt engine.

### 12. Stream a concise presentation-only Pi trace

`dimos eval run` emits progress to standard error by default while reserving
standard output for the terminal pretty result or the single `--json` object.
Immediately after decoding the public case, it prints a compact session header
containing case identity, source selection, the public question, and an
`Answer: pending` marker. It never substitutes the private expected answer. The
Node adapter forwards only agent lifecycle and visible assistant text deltas.
Python emits the `python_exec` start and completion events at the broker boundary,
where the actual bounded code, result, status, and duration are known.

The trace deliberately excludes Pi thinking deltas, raw protocol frames, and
credential material. Displayed code, results, and diagnostics are bounded and
marked when truncated. A progress observer is best-effort: rendering failures
cannot change execution, evidence, prediction, validation, or cleanup. `--quiet`
disables progress without changing any semantic or runtime binding.

The existing native Pi session and CodePolicy call log remain the authoritative
attempt evidence. The terminal trace is not added to the case fingerprint or
duplicated as another evidence artifact.

Because the pinned Pi SDK streams visible assistant output but may resolve
`session.prompt()` without returning text, the adapter accumulates the visible
text deltas from the latest Pi turn and uses them as canonical `final_text` when
the prompt return value is empty. Thinking deltas never enter that accumulator.

The single-case path pins a 600-second whole-turn timeout and records it in the
runtime binding. This accommodates several bounded `python_exec` calls, each of
which may legitimately consume up to the CodePolicy execution limit, without
turning timeout selection into a semantic case override.

Alternative considered: print the raw Pi session stream to standard output.
Rejected because it would expose hidden reasoning and protocol noise, break
`--json` pipelines, and make presentation failures part of evaluation behavior.

## Risks / Trade-offs

- **Room count is semantically ambiguous** → Approve a private room inventory and counting policy before enabling the gate; preferably include enough counting guidance in the public task to make the question fair without leaking the answer.
- **Current `python_exec` results are text-oriented, while room understanding may benefit from visual inspection** → Keep the north-star honest and inspect the real failure evidence. Do not silently give the agent a reconstructed-map answer or private labels. If multimodal CodePolicy output is required, propose it as an explicit interaction-contract change rather than weakening scoring.
- **A real model run is nondeterministic and credentialed** → Use deterministic contract/integration tests for mechanics and retain the real run as a documented acceptance gate with complete evidence. Do not substitute fake-agent success for the north-star.
- **Refactoring the live runner can regress simulator behavior** → Preserve its existing focused test suite and run the completed live smoke through compatibility adapters before deleting old orchestration paths.
- **Standalone control operations could leak into the model tool inventory** → Keep host controls on a distinct client/control channel and assert exact one-tool inventory in both service and Pi tests.
- **Live read-only SQLite semantics may lag or cache newly committed observations** → Add a live-store integration test that appends through the system writer and verifies a long-lived read-only CodePolicy view observes committed data according to Memory2's supported consistency contract.
- **Temporary compatibility wrapper prolongs two topologies** → Mark it non-canonical, remove it from evaluation blueprints immediately, and leave deletion as a named migration checkpoint rather than allowing new consumers.
- **Normalized floating-point progress can be recomputed inconsistently** → Canonicalize the authored value, resolve only during preparation, store the exact timestamp, and always serve the prepared manifest record.
- **The cleanup broadens the change beyond one QA runner** → Sequence work so shared contracts and standalone service land behind tests before migrating live DimSim and enabling the credentialed north-star.
- **Live output could leak reasoning or break JSON consumers** → Forward only
  visible text deltas and bounded tool activity to standard error, discard
  thinking deltas, and retain `--quiet` for silent execution.

## Migration Plan

1. Add canonical case, interaction, validator, prediction, score, and outcome contracts without changing current runners.
2. Add normalized progress preparation and manifest records while retaining relative-second compatibility for existing bundles.
3. Extract `CodePolicySession`; make current `CodePolicyModule` delegate to it and keep existing tests green.
4. Add the standalone host and frozen/live environment profiles; migrate the frozen developer service off blueprints.
5. Decompose shared Pi/evidence/attempt behavior from the live runner and implement frozen source, interaction, and exact-integer validator drivers.
6. Migrate the live DimSim smoke to the canonical case engine and standalone live CodePolicy while preserving its tests and manual smoke.
7. Author and review the Hong Kong room oracle, add the progress-`1.0` case, run deterministic real-recording integration, and then run the credentialed Pi acceptance.
8. Document the canonical extension points for future evals and mark direct new top-level runner architectures unsupported.

Rollback keeps the compatibility `CodePolicyModule` and relative-second preparation path until both frozen and live acceptance runs pass. The old live runner orchestration is removed only after its behavior is covered through the new drivers.

## Open Questions

- What is the approved Hong Kong room count and exact counting policy, including hallway, open-plan, and merely-visible-room treatment?
- Can the current text-only `python_exec` result surface support reliable room counting from the recording, or will a separately specified multimodal CodePolicy result be required after the first evidence-backed attempt?
- When the compatibility wrapper has no remaining blueprint consumers, should its deletion occur in this change or in a short follow-up removal change?

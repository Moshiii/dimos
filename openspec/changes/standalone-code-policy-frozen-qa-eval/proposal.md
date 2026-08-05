## Why

Frozen Memory2 recordings can already be prepared and exposed through CodePolicy, but doing so still deploys `CodePolicyModule` and `McpServer` inside a DimOS blueprint even though the policy service uses no DimOS module behavior beyond process spawning. Agent evaluation is also split between a live-DimSim-specific runner and benchmark-specific structures, with no canonical case boundary for what the source is, what the agent must do, how the source is presented, and how the result is validated.

The change needs one falsifiable north-star: at normalized progress `1.0` of the Hong Kong office recording, a real Pi agent receives the read-only `memory` API through standalone CodePolicy, answers the authored room-count question, and produces an inspectable private score.

## What Changes

- Extract the persistent CodePolicy kernel, session, evidence, and `python_exec` MCP surface into a standalone service that does not require a DimOS blueprint, module worker, or coordinator.
- Define two explicit CodePolicy environments: frozen recording with read-only `memory`, and live DimOS with read-only live `memory` plus `app` connected through `dimos.porcelain`.
- Retain a temporary `CodePolicyModule` compatibility wrapper while making the standalone service the canonical evaluation topology.
- Introduce the canonical future evaluation case shape as four separate, immutable contracts: `source`, `task`, executable `interaction`, and private `validator`. The interaction describes the actual agent-facing feed and lifecycle rather than merely naming a data type or runtime.
- Clean up `dimos.benchmark.agent_eval` into shared case, attempt, Pi-session, prediction, validation, and evidence primitives plus interaction-specific drivers; preserve the existing live DimSim smoke behavior through the live CodePolicy interaction instead of keeping a second runner architecture.
- Replace authored relative-second-only selections with normalized recording progress in the inclusive range `[0, 1]`; resolve progress to an exact source timestamp and retain both values. Progress `1.0` means the inclusive end of the recording.
- Add a frozen-QA case and runner that separates source, task, CodePolicy interaction, and private validator; owns one fresh standalone service and Pi session per attempt; parses a typed integer answer from Pi's final response; and retains prediction, score, calls, session, and source evidence.
- Add the Hong Kong office north-star case at progress `1.0` with the question “How many rooms in total?”, backed by a human-reviewed private integer oracle and room inventory.
- Add deterministic component/integration coverage plus one documented credentialed real-Pi acceptance run.
- Add a canonical `dimos eval run <case>` entry point for one local attempt. The
  command loads the immutable case through Pydantic, accepts only typed agent and
  operational configuration, and renders the normalized result directly while
  retaining the complete attempt evidence.
- Stream a concise Pi activity trace during single-case execution by default,
  including visible assistant text and bounded `python_exec` activity while
  excluding hidden reasoning, raw protocol traffic, and credentials. Begin the
  trace with the public case, source, question, and a pending-answer marker.
- Infer OpenAI API-key authentication from a nonempty `OPENAI_API_KEY` for the
  single-case CLI when no authentication options are supplied, while preserving
  explicit authentication selections.

## Capabilities

### New Capabilities

- `agent-evaluation-case-model`: Canonical source/task/interaction/validator case contract, executable interaction-driver boundary, private validation, and shared attempt outcomes for future agent evaluations.
- `standalone-code-policy-service`: Independent CodePolicy session and MCP lifecycle, fixed frozen/live environment bindings, execution evidence, and compatibility with live DimOS through porcelain.
- `frozen-qa-agent-evaluation`: Immutable frozen-QA cases, Pi execution through the sole `python_exec` tool, typed prediction parsing, private validation, attempt evidence, and the Hong Kong office north-star.

### Modified Capabilities

- `frozen-short-horizon-qa`: Add normalized progress selection and exact resolution provenance, and require frozen CodePolicy serving through the standalone service rather than a DimOS blueprint deployment.

## Impact

- Refactors `dimos/agents/code_policy.py` into a plain session/core plus standalone MCP host while retaining a compatibility module during migration.
- Refactors `dimos/benchmark/agent_eval/` around the canonical case and interaction-driver boundary, deduplicating Pi, attempt, prediction, validation, and evidence lifecycle code while keeping the existing live DimSim smoke executable.
- Replaces the blueprint-backed service in `dimos/benchmark/short_horizon_qa/` and extends its source models, preparation CLI, manifest, documentation, and tests.
- Adds a focused frozen-QA evaluation runner that reuses the existing Node Pi code-policy session adapter, authentication binding, one-tool broker, native Pi evidence, and attempt storage patterns without importing the live DimSim lifecycle.
- Adds a top-level single-case CLI without making evaluation a blueprint or
  exposing source, task, interaction, validator, recording, bundle, or oracle
  overrides.
- Adds presentation-only live progress on standard error plus a `--quiet`
  opt-out, preserving machine-readable JSON on standard output and existing
  evidence as the authoritative transcript.
- Removes the need to repeat API-key mode and binding options when
  `OPENAI_API_KEY` is already available in the process environment or `.env`.
- Adds an authored Hong Kong office case and a private human-reviewed validator artifact; the expected answer remains unavailable to the agent.
- Establishes the required shape for future evals but does not migrate the hardened spatial Pi baseline, scheduler, or unrelated benchmark families in this change.
- Does not add scheduling, publication, leaderboard claims, writable agent memory, a general binding registry, or a replacement for `dimos.porcelain`.

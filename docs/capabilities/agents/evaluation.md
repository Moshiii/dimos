---
title: "Frozen recording evaluation"
---

`dimos eval run` asks one integer question about one immutable Memory2 recording. It
prepares a read-only map, gives a fresh Pi agent one `python_exec` tool, validates
the terminal answer against a private oracle, and writes durable attempt evidence.
It does not start a robot, simulation, replay blueprint, or live DimOS module.

## Setup

Install the Python agent dependencies and build the dedicated Node adapter from a
source checkout:

```bash
uv sync --extra agents
npm ci --prefix packages/pi-code-policy-adapter
npm run build --prefix packages/pi-code-policy-adapter
```

The adapter requires Node 22.19.0 or newer. The Python command reports a preflight
error if `packages/pi-code-policy-adapter/dist/code-policy-main.js` is absent.

## Run one case

Run the Hong Kong office plumbing fixture with:

```bash
uv run dimos eval run \
  dimos/benchmark/short_horizon_qa/cases/go2_hongkong_office-room-count-smoke/case.json \
  --output=/tmp/dimos-eval-smoke
```

The fixture's expected count is the synthetic sentinel `0`, not a reviewed room
count. A completed attempt may therefore report a semantic failure even when the
agent, map, and evidence pipeline work correctly.

`eval run` accepts these options:

| Option | Default | Purpose |
| --- | --- | --- |
| `--agent.backend` | `pi` | Select the pinned agent backend. |
| `--agent.model` | `gpt-5.6-luna` | Select the pinned model. |
| `--agent.thinking-level` | `medium` | Select the pinned thinking level. |
| `--agent.auth.mode` | inferred | Use `codex-oauth` or `openai-api-key`. |
| `--agent.auth.path` | `~/.pi/agent/auth.json` | Select a Codex OAuth file. |
| `--agent.auth.env` | `OPENAI_API_KEY` | Name the API-key environment variable. |
| `--output` | DimOS state directory | Set the append-only attempt root. |
| `--json` | off | Print one compact JSON result to stdout. |
| `--quiet` | off | Suppress live progress on stderr. |

Do not pass credential values as command-line arguments. With no explicit mode,
an auth path selects OAuth, an auth environment name selects API-key auth, and a
set `OPENAI_API_KEY` selects API-key auth. Otherwise, the command uses Codex OAuth.
An explicit `--agent.auth.mode` takes precedence over environment inference.

## Output and exit status

The final human or JSON result goes to stdout. Live case, agent, and tool progress
goes to stderr, so scripts can parse `--json` output safely. `--quiet` suppresses
the progress stream without suppressing the final result.

The command uses three exit codes:

| Code | Meaning |
| --- | --- |
| `0` | The attempt completed. Its private score may be passed or failed. |
| `1` | The attempt started but infrastructure failed. |
| `2` | Preflight failed before an attempt was reserved. |

Preflight verifies the case, private oracle digest, recording bundle, credential
binding, and built Node entrypoint. Once an attempt starts, its mode-`0700`
directory contains lifecycle events, public and private artifacts, content
descriptors, CodePolicy receipts, broker calls, and Pi evidence. Files are created
exclusively; reruns create new attempts instead of overwriting old evidence.

## Privacy and trust boundary

The agent receives the case's public projection: recording identity, cutoff,
question, and interaction contract. It does not receive the validator path,
oracle content, or credential value. Private oracle and score files remain private
attempt artifacts. Public prompts, progress, compact results, broker logs, Pi
evidence, and serialized runtime configuration contain no oracle or credential
material.

CodePolicy is trusted, persistent, and unsandboxed Python execution. Its supplied
`memory` API is read-only and cutoff-limited, but Python code can still access the
host filesystem and processes. Run only trusted evaluation agents and code. Use an
OS sandbox or container for hostile code.

Each attempt creates fresh Pi and CodePolicy processes. Normal completion,
failure, timeout, and interruption close those processes and release the output
lock. If a smoke run is interrupted externally, confirm no `code-policy` or
`pi-code-policy-adapter` child remains before retrying.

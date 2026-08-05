# Frozen short-horizon QA

This workflow presents an agent with exactly the Memory2 observations available at a
chosen recording time. It also provides the `global_map` that DimOS's runtime
`VoxelMapTransformer` would most recently have emitted by that time. The source
recording is not copied or modified.

## Prepare the Hong Kong office recording

Choose all normalized recording progress points needed for an experiment in one
preparation run. Progress is inclusive and ranges from `0` at the exact recording
start to `1` at the exact recording end. Interior values resolve linearly over the
sealed recording range, and the manifest retains the exact resolved timestamp.

```bash
uv run python -m dimos.benchmark.short_horizon_qa prepare \
  --recording go2_hongkong_office \
  --progress 0.25 \
  --progress 0.5 \
  --progress 1.0 \
  --output artifacts/go2_hongkong_office-short-horizon
```

Preparation uses the runtime mapper defaults: 5 cm voxels, two million blocks,
column carving, the `world` frame, CUDA, and one map emission per five LiDAR
frames. Use `--device CPU:0` on a machine without CUDA or `--voxel-size` to run a
deliberately different mapping condition.

The output directory contains:

- `manifest.v1.json`: source identity, mapper settings, per-stream boundaries,
  and the selected map observation for every cutoff.
- `derived.db`: deduplicated `global_map` snapshots. Nearby selections share an
  observation when the runtime mapper would not have emitted a newer map.

Preparation refuses to overwrite an output directory, rejects recordings with an
active WAL, and fails if LiDAR is not already in the mapper's configured frame.
It also fails if the source contains `global_map`, because silently shadowing a
recorded stream would make the benchmark ambiguous.

## Serve one frozen frame

Code-policy execution requires the agents dependency group:

```bash
uv sync --extra agents
uv run python -m dimos.benchmark.short_horizon_qa serve \
  --bundle artifacts/go2_hongkong_office-short-horizon \
  --progress 1.0 \
  --mcp-port 9990
```

Startup verifies both database hashes against the manifest. The service exposes
one MCP skill, `python_exec`, at `http://localhost:9990/mcp`. The persistent Python
session contains `memory` but intentionally has no live robot `app`:

```bash
uv run dimos mcp list-tools
uv run dimos mcp call python_exec --json-args \
  '{"code":"[(name, memory.stream(name).count()) for name in memory.list_streams()]"}'
uv run dimos mcp call python_exec --json-args \
  '{"code":"m = memory.streams.global_map.last(); (m.ts, m.tags, len(m.data))"}'
```

Every source and derived stream is filtered with `observation.ts <= cutoff`.
Append, stream creation, and deletion through `memory` are rejected. The original
SQLite files are opened in read-only/query-only mode and do not create WAL files.

This is an accidental-mutation boundary, not a security sandbox. `python_exec` is
explicitly trusted and unsandboxed Python; hostile policy code could use ordinary
filesystem APIs or construct another database connection. Run untrusted agents in
an external OS/container sandbox.

## Canonical evaluation case

New generic evaluations use one immutable four-part case:

- `source`: the recording/environment selection, including normalized progress;
- `task`: the public question or embodied instruction and response protocol;
- `interaction`: the concrete agent-facing feed and lifecycle;
- `validator`: a private, digest-bound executable definition of success.

Runtime ports, credentials, output paths, open databases, and simulator handles
belong to the attempt binding, not the case identity. Add a typed source,
interaction, or validator driver under `dimos.benchmark.agent_eval` when a future
evaluation needs a new lifecycle. Do not add a parallel top-level runner that
implicitly combines these concerns.

The frozen interaction starts a fresh standalone CodePolicy and Pi session for
each case, exposes only `python_exec`, supplies read-only `memory`, and omits
`app`. The private exact-integer validator accepts exactly one terminal
`ANSWER: <integer>` marker. Its oracle never enters the public case, prompt,
CodePolicy namespace, or public outcome.

After authoring a compiled case plus independently reviewed private oracle, run
one local attempt with the canonical CLI:

```bash
uv run dimos eval run cases/go2_hongkong_office-room-count/case.json
```

The command streams a concise live trace to standard error by default. It shows
the public case, source selection, question, and `Answer: pending` at the start,
followed by source preparation, Pi lifecycle, visible assistant text, and
bounded `python_exec` calls and results. The pending marker is not the private
expected answer. Pi thinking deltas, raw adapter frames, credentials, and
private validator material are never rendered. Use `--quiet` to suppress
progress:

```bash
uv run dimos eval run cases/go2_hongkong_office-room-count/case.json --quiet
```

Progress stays on standard error when `--json` is selected, so standard output
remains exactly one compact result object suitable for piping.

The case is immutable Pydantic data. Source, task, interaction, validator,
recording, question, and expected-answer overrides are intentionally absent.
The private validator path is relative to the case document and hash-verified;
frozen recording and derived-map paths are resolved behind the Memory2 source
driver rather than exposed to the agent or command line.

Pi defaults to `gpt-5.6-luna` and medium thinking. With no authentication flags,
the CLI automatically uses `OPENAI_API_KEY` when it is nonempty (including when
loaded from the repository `.env`); otherwise it falls back to Codex OAuth. Its
typed dotted configuration can be made explicit:

```bash
uv run dimos eval run cases/go2_hongkong_office-room-count/case.json \
  --agent.backend=pi \
  --agent.model=gpt-5.6-luna \
  --agent.thinking-level=medium \
  --agent.auth.mode=codex-oauth \
  --agent.auth.path=/path/to/pi/auth.json \
  --output=artifacts/frozen-qa-attempts
```

For the standard API-key binding, no auth flags are needed; never put a secret on
the command line:

```bash
OPENAI_API_KEY=... uv run dimos eval run cases/example/case.json
```

Use `--agent.auth.env=MY_OPENAI_KEY` for a nonstandard environment-variable
name; this option implies API-key mode. An explicit `--agent.auth.mode` always
wins over environment-based inference.

The default output is a concise result including the case, prediction, semantic
task result, resolved agent condition, duration, and artifact path. Use `--json`
for one compact Pydantic-encoded result object. Each attempt still retains public
and private case projections, source receipt, MCP inventory, CodePolicy receipt
and calls, Pi evidence, typed prediction, private score, events, manifest, and
the normalized outcome.

This command executes exactly one case synchronously. Dataset batching,
containers, parallel workers, retries, and scheduling are deliberately deferred;
future orchestration will call the same single-attempt engine.

`--cutoff-seconds` remains available as a legacy preparation and serving input
for existing workflows. New authored evaluation cases use normalized progress so
their selection remains meaningful without copying fragile timestamps.

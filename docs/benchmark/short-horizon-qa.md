# Frozen short-horizon QA

This workflow presents an agent with exactly the Memory2 observations available at a
chosen recording time. It also provides the `global_map` that DimOS's runtime
`VoxelMapTransformer` would most recently have emitted by that time. The source
recording is not copied or modified.

## Prepare the Hong Kong office recording

Choose all cutoffs needed for an experiment in one preparation run. Cutoffs are
seconds after the earliest observation in the recording and are inclusive.

```bash
uv run python -m dimos.benchmark.short_horizon_qa prepare \
  --recording go2_hongkong_office \
  --cutoff-seconds 30 \
  --cutoff-seconds 60 \
  --cutoff-seconds 90 \
  --output artifacts/go2_hongkong_office-short-horizon
```

Preparation uses the runtime mapper defaults: 5 cm voxels, two million blocks,
column carving, the `world` frame, CUDA, and one map emission per five LiDAR
frames. Use `--device CPU:0` on a machine without CUDA or `--voxel-size` to run a
deliberately different mapping condition.

The output directory contains:

- `manifest.v1.json`: source identity, mapper settings, per-stream boundaries,
  and the selected map observation for every cutoff.
- `derived.db`: deduplicated `global_map` snapshots. Nearby cutoffs share an
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
  --cutoff-seconds 60 \
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

Question authoring and answer scoring remain separate from this service. A harness
can select a manifest cutoff, start this endpoint, submit its questions through
MCP, and retain the manifest plus CodePolicy execution records as evidence.

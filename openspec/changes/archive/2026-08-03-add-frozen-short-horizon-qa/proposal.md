## Why

Short-horizon spatial QA needs a reproducible way to place an agent at an exact point in a recorded run without leaking later observations or granting live robot control. The benchmark also needs the map that DimOS would actually have produced by that point, rather than an independently reconstructed final map.

## What Changes

- Add inclusive, read-only Memory2 views bounded by an absolute observation timestamp.
- Add immutable SQLite read mode that does not create or mutate database journal sidecars.
- Add preparation of deduplicated runtime `global_map` snapshots for one or more recording-relative cutoffs using the production voxel mapper cadence and configuration.
- Add a versioned manifest recording source and derived hashes, mapper settings, stream boundaries, cutoff timestamps, and map provenance.
- Add an offline CodePolicy and MCP service that exposes frozen `memory` while prohibiting a live DimOS `app` connection.
- Add an operator workflow for preparing and serving frozen Hong Kong office recording cutoffs.
- Keep question authoring and answer scoring outside this capability.

## Capabilities

### New Capabilities

- `frozen-short-horizon-qa`: Defines bounded recording views, runtime-map sidecars, reproducible manifests, and the offline CodePolicy service used by short-horizon QA.

### Modified Capabilities

None.

## Impact

- Memory2 stream filtering, SQLite stores, and stream mutation behavior.
- `CodePolicyModule` configuration and kernel bootstrap behavior.
- A new `dimos.benchmark.short_horizon_qa` preparation and serving package.
- MCP-based benchmark harnesses can select a prepared cutoff and query it through `python_exec`.
- Preparation requires Open3D and the existing voxel mapping stack; serving CodePolicy requires the `agents` dependency group.

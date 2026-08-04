## Context

Memory2 recordings contain the time-ordered perception and message streams needed for spatial QA, but opening a recording directly gives policy code access to its full future. A final reconstructed map is also unfair at an intermediate cutoff because it incorporates observations that had not yet arrived. The solution crosses Memory2 query/storage behavior, runtime mapping, artifact provenance, CodePolicy bootstrap, and MCP deployment.

The source LFS database can be several gigabytes, so copying one database per cutoff is undesirable. The policy executor is trusted, unsandboxed Python; the design must prevent accidental writes and accidental live control without representing the boundary as protection from hostile code.

## Goals / Non-Goals

**Goals:**

- Present every source stream as an inclusive `ts <= cutoff` view.
- Reuse the original recording while guaranteeing that normal Memory2 operations do not mutate it.
- Provide the most recent map the production voxel mapper would have emitted by each cutoff.
- Make every prepared cutoff reproducible and auditable through a versioned manifest.
- Give offline policy code the familiar `memory` interface without a deployed robot `app`.
- Prepare multiple cutoffs in one mapper pass and deduplicate identical map emissions.

**Non-Goals:**

- Sandboxing hostile Python policy code.
- Reconstructing maps with MapAnything or another non-runtime algorithm.
- Transforming sensor-frame LiDAR without recorded transform support.
- Generating benchmark questions, reference answers, or scores.
- Copying or truncating the source database for every cutoff.

## Decisions

### Bound streams at the query layer

Add an inclusive `ThroughFilter` and propagate a stream-level writable flag. `FrozenMemoryStore` overlays the source and derived stores, applies the same absolute cutoff to every stream, and returns read-only stream views.

This preserves one source database and makes cutoff semantics composable with existing Memory2 queries. Physically copying a truncated database was rejected because it multiplies storage, preparation time, and opportunities for inconsistent stream boundaries.

### Open frozen SQLite databases in immutable read mode

SQLite connections use `mode=ro` and `PRAGMA query_only=ON`; registry and observation-store startup verify existing tables instead of creating them. Store and stream mutation methods fail before reaching SQLite.

Relying only on SQLite write errors was rejected because it produces inconsistent errors and can still create WAL/SHM sidecars when opened in normal WAL mode.

### Reproduce runtime map emissions into a derived sidecar

Preparation runs the existing `VoxelMapTransformer` with recorded mapper settings and its normal emission cadence. For each requested cutoff, it selects the latest emitted map whose timestamp is at or before the cutoff. Only selected unique emissions are written to `derived.db` as `global_map`.

A final-map reconstruction was rejected because it leaks future geometry. Storing a map per cutoff was rejected because several nearby cutoffs can correctly refer to the same runtime emission.

The source and sidecar must have disjoint stream names. Preparation fails when the source already contains `global_map` rather than silently shadowing recorded evidence.

### Describe and seal preparation with a versioned manifest

`manifest.v1.json` records the source path, size and SHA-256, the derived SHA-256, recording range, complete mapper configuration, each absolute and relative cutoff, per-stream counts and last observations, and selected map provenance. Serving verifies both hashes before starting.

Paths alone were rejected as evidence because LFS assets and derived files can be replaced while retaining the same filename.

### Make frozen mode an explicit CodePolicy configuration

CodePolicy retains its existing live defaults. Frozen mode requires a source path, derived path, and absolute cutoff together, opens both databases read-only, wraps them in `FrozenMemoryStore`, and requires `connect_app=False`. The kernel namespace therefore contains `memory` but no `app`. The standalone blueprint combines this module with `McpServer` and no agent or robot stack.

Removing `app` after connecting was rejected because the connection itself would already create an unnecessary live-control boundary. A separate policy implementation was rejected because it would duplicate persistent-kernel behavior and the `python_exec` contract.

### Treat read-only policy memory as an accidental-mutation boundary

Normal calls through `memory` cannot append, create, delete, or see future observations. `python_exec` remains trusted and unsandboxed, so hostile code could use filesystem APIs or construct another database connection. Untrusted evaluations require an external OS or container sandbox.

## Risks / Trade-offs

- **Large source hashes add startup latency** → Verify once at service startup and keep the persistent service alive for the selected cutoff.
- **Mapper configuration drift can change derived maps** → Persist every relevant mapper setting and use the production transformer rather than a benchmark-only implementation.
- **A cutoff before the first scheduled map emission has no fair map** → Fail preparation explicitly instead of synthesizing an early partial map.
- **LiDAR outside the configured map frame would produce invalid geometry** → Reject the recording until transform-aware preparation is implemented.
- **Trusted Python can bypass wrapper protections** → Document the boundary and require external isolation for adversarial code.
- **Absolute floating-point timestamps can be awkward to select manually** → Operators select manifest entries by recording-relative seconds; the service resolves the recorded absolute timestamp.

## Migration Plan

The change is additive. Existing `SqliteStore` and `CodePolicyModule` behavior remains unchanged unless read-only or frozen configuration is explicitly selected. Prepare new bundles from existing recordings, verify them with the service, and delete the derived bundle to roll back; the source recording is never modified.

## Open Questions

- Which question-authoring and scoring harness will consume the prepared cutoff manifest?
- Should a future format retain recorded transforms so sensor-frame LiDAR can be mapped during preparation?
- Should benchmark orchestration cache successful source-hash verification across several cutoff services?

## Why

DimSim scenes contain authoritative object identity, transforms, and state, but DimOS has no typed private interface for consuming the complete semantics needed to generate objective embodied tasks. Without that boundary, benchmark questions would depend on asset-title heuristics, duplicated benchmark annotations, or manually supplied answers that can drift from the simulated scene.

## What Changes

- Extend the DimSim integration contract with a private, typed, read-only `SceneOracleView` for one coherently reset scene.
- Require DimSim-owned semantic identity, classes, aliases, state, geometry, regions, and provenance for facts used by benchmark generators.
- Add deterministic offline generation of one validated smoke task in each category:
  - destination navigation;
  - targeted state QA;
  - broad-exploration count QA;
  - multi-hop spatial comparison QA.
- Materialize immutable, versioned public task records separately from private executable contracts and expected answers.
- Add stable content-derived identities, canonical serialization, source-view digests, generation diagnostics, and mandatory ambiguity/objectivity gates.
- Add a golden apartment smoke fixture for the agreed bathtub, television, dining-chair, and sofa-relative tasks.
- Exclude runtime DimSim execution, Pi-baseline integration, agent submission, scoring, and scene generation from this change.

## Capabilities

### New Capabilities

- `dimsim-scene-oracle-view`: Private typed export of authoritative DimSim scene semantics and state for offline benchmark generation.
- `dimsim-benchmark-task-generation`: Deterministic generation, validation, and public/private packaging of objective DimSim benchmark tasks.

### Modified Capabilities

None.

## Impact

- Affects the DimSim scene/integration boundary under `dimos/simulation/dimsim/` and requires a compatible DimSim scene-schema/oracle revision.
- Adds a benchmark-generation package under `dimos/benchmark/` with strict models, generators, canonical bundle writing, validation, and smoke fixtures.
- Reuses established static spatial-corpus conventions where applicable: strict immutable records, stable opaque IDs, canonical JSON/JSONL, deterministic templates, and physically separable public/oracle roots.
- Does not change simulator runtime modules, benchmark runners, Pi SDK execution, submission protocols, or score ledgers.

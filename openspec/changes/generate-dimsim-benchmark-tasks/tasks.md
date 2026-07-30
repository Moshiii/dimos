## 1. Models and Policy Configuration

- [ ] 1.1 Create the DimSim benchmark-generation package and define versioned generator, predicate, template, frame, clearance, and comparison-margin configuration constants.
- [ ] 1.2 Add strict frozen `SceneOracleView` models for scene/reset revisions, frame contract, embodiment and spawn, navigation geometry, entities, semantic properties, regions, and grouped field provenance.
- [ ] 1.3 Add canonical oracle-view serialization and digest utilities with tests proving stable bytes for equivalent content and changed digests for truth-bearing changes.
- [ ] 1.4 Add strict public manifest/task models and private contract, expected-outcome, source-provenance, diagnostic, and generation-report models using discriminated unions.
- [ ] 1.5 Generate or expose JSON Schemas for every persisted corpus record and test rejection of unknown fields, malformed references, unsupported schema versions, and invalid answer types.

## 2. DimSim Oracle Provider

- [ ] 2.1 Define a private `SceneOracleProvider` protocol that returns one validated in-memory `SceneOracleView`, plus an in-memory fixture implementation for unit tests.
- [ ] 2.2 Add a private `SceneClient` oracle command that negotiates the supported semantic-schema version and returns one coherent reset payload without exposing it as an agent skill or normal observation.
- [ ] 2.3 Extend the DimSim-owned apartment semantics with stable classes, aliases, power state, footprints, navigation geometry, canonical spawn, and provenance required by the four smoke contracts.
- [ ] 2.4 Select and pin or otherwise identify the compatible upstream DimSim revision, update the DimOS integration as narrowly required, and record that revision in oracle-view provenance.
- [ ] 2.5 Add contract tests that run the live provider against the compatible apartment scene and fail clearly for missing semantics, incoherent reset state, or an unsupported schema revision.

## 3. Deterministic Candidate Generation

- [ ] 3.1 Implement exact semantic entity resolution and deterministic ordering without display-title substring matching, including cardinality and missing-provenance diagnostics.
- [ ] 3.2 Implement validated 2-D footprint geometry, outer-edge distance, polygon surface distance, stopping-band construction, collision clearance, and canonical-spawn reachability utilities with boundary tests.
- [ ] 3.3 Implement the bathtub `navigate-within-outer-footprint` generator with the one-metre threshold and destination feasibility gates.
- [ ] 3.4 Implement the television `entity-state` generator with authoritative `power` vocabulary validation and typed `ON`/`OFF` outcomes.
- [ ] 3.5 Implement the scene-scoped `count-semantic-class` generator for exact class `dining-chair`, including exclusion of `work-chair`.
- [ ] 3.6 Implement the sofa-anchored `argmin-surface-distance` generator for bathtub versus television with a versioned non-zero stability margin.
- [ ] 3.7 Implement controlled public templates, semantic task identity payloads that exclude expected answers, stable opaque IDs, and fixed four-category smoke ordering.
- [ ] 3.8 Add rejection tests for duplicate entities, missing semantic classes, visual-only state, invalid footprints, unreachable destinations, center-distance disagreement, and comparison ties or near-ties.

## 4. Corpus Writing and Validation

- [ ] 4.1 Implement canonical writing of `manifest.json`, `public/tasks.jsonl`, `oracle/task_contracts.jsonl`, `oracle/expected_outcomes.jsonl`, and `oracle/generation_report.json`.
- [ ] 4.2 Implement one-to-one public/contract/outcome reference validation and reconstruct every stable ID from its declared semantic identity payload.
- [ ] 4.3 Implement release-blocking validation for schema validity, exact category cardinality, answer typing, geometry and reachability gates, comparison stability, source digest/provenance, and deterministic regeneration.
- [ ] 4.4 Implement recursive public-package leakage scanning for expected values, private entity bindings, executable contracts, oracle-view digests, semantic provenance, and private paths.
- [ ] 4.5 Ensure failed generation writes deterministic private diagnostics but never marks the manifest complete or leaves a partially publishable public release.
- [ ] 4.6 Add round-trip loader tests proving the public root can be distributed independently and the full corpus joins only through opaque task IDs.

## 5. Apartment Smoke Corpus

- [ ] 5.1 Add a minimal typed apartment oracle fixture sourced from the authoritative DimSim contract, including one bathtub, one television, four dining chairs, one work chair, one sofa, navigable geometry, and canonical reset state.
- [ ] 5.2 Add a smoke-generation entrypoint that produces exactly the four approved public tasks and no additional retained tasks.
- [ ] 5.3 Add golden assertions for the terminal bathtub predicate, television enum answer, dining-chair integer answer, and sofa-relative entity-choice answer.
- [ ] 5.4 Verify byte-equivalent regeneration from the same fixture and verify that state or predicate-policy changes update only the intended identities, outcomes, digests, and provenance.
- [ ] 5.5 Run the smoke generator against the compatible live DimSim apartment oracle view and require the same schema, cardinality, objectivity, and leakage gates as the fixture path.

## 6. Documentation and Quality Gates

- [ ] 6.1 Document the private `SceneOracleView` contract, generator entrypoint, corpus layout, policy versioning, rejection diagnostics, and distinction between fixture data and simulator-owned production truth.
- [ ] 6.2 Document that scene generation, evaluated-agent execution, Pi-baseline integration, submission, scoring, and result publication remain outside this change.
- [ ] 6.3 Run focused unit and integration tests for the new package, `git diff --check`, and the repository's required formatting, linting, typing, and pre-commit checks.

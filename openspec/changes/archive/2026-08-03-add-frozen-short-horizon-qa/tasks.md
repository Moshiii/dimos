## 1. Bounded Memory2 Storage

- [x] 1.1 Add inclusive through-timestamp filtering with SQLite query pushdown
- [x] 1.2 Add read-only stream views that reject appends while preserving query composition
- [x] 1.3 Add immutable SQLite store, registry, and observation-store open modes
- [x] 1.4 Add a frozen source-and-derived overlay with collision and mutation rejection
- [x] 1.5 Cover inclusive cutoff, overlay, mutation, and journal-side-effect behavior with tests

## 2. Runtime Map Preparation

- [x] 2.1 Define strict versioned manifest, mapper, stream-boundary, and cutoff models
- [x] 2.2 Run the production voxel transformer once across ordered cutoffs
- [x] 2.3 Select and deduplicate the latest runtime map emission for each cutoff
- [x] 2.4 Seal the derived database and atomically publish a non-overwriting bundle
- [x] 2.5 Reject active WAL inputs, unsupported frames, early cutoffs, and map-stream collisions
- [x] 2.6 Test map cadence, provenance, source immutability, deduplication, and failure cleanup

## 3. Frozen Policy Service

- [x] 3.1 Extend CodePolicy configuration and bootstrap for bounded source and derived memory
- [x] 3.2 Enforce that frozen CodePolicy cannot connect a live DimOS app
- [x] 3.3 Verify bundle hashes and resolve only manifest-declared relative cutoffs
- [x] 3.4 Compose the standalone CodePolicy and MCP blueprint and add prepare/serve CLI commands
- [x] 3.5 Test configuration invariants, kernel namespace, bundle integrity, and blueprint wiring

## 4. Validation and Operations

- [x] 4.1 Document preparation, serving, querying, artifact contents, and the trusted-code boundary
- [x] 4.2 Run focused and broader Memory2 and CodePolicy regression suites
- [x] 4.3 Prepare a real Hong Kong office cutoff with CUDA and verify stream/map boundaries
- [x] 4.4 Start the real MCP service, query frozen memory, confirm `app` is absent, and confirm writes fail

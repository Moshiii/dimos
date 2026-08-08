## ADDED Requirements

### Requirement: Case-bound external benchmark episode source
The canonical evaluation case model SHALL provide a strict external-benchmark episode source containing benchmark kind, upstream and dataset revisions, split, episode ID, scene ID, source asset references and digests, OCI image recipe and digest, preparation recipe, DimOS blueprint, public bridge protocol revision, and native result schema revision. All supplied fields SHALL participate in case identity, and one source SHALL bind exactly one episode.

#### Scenario: Compile one external benchmark episode
- **WHEN** an authored case selects a VLN-CE R2R episode and its case-bound runtime metadata
- **THEN** the compiled source binds one immutable episode and changing any pinned source, runtime, protocol, blueprint, or result-schema field changes or invalidates the case fingerprint

#### Scenario: Reject an incomplete external source
- **WHEN** an external-benchmark source omits an episode identity, scene identity, verified asset reference, image digest, blueprint, protocol revision, or native result revision
- **THEN** strict case validation fails before dependency preparation or process startup

### Requirement: External benchmark task binding
An external-benchmark case task SHALL contain the exact agent-visible instruction and response/submission guidance separately from the source. The compiled case SHALL bind the task instruction digest to the selected episode, and private preflight SHALL verify exact agreement with the official episode before delivering the task to the agent.

#### Scenario: Deliver a verified benchmark instruction
- **WHEN** private preflight confirms that the task instruction matches the selected official episode
- **THEN** the evaluator delivers that exact public instruction and the case-declared submission guidance to the agent

#### Scenario: Reject a mismatched copied instruction
- **WHEN** the task instruction or its digest differs from the instruction selected by the private official episode binding
- **THEN** preflight fails without starting the benchmark episode, DimOS runtime, CodePolicy, or Pi

### Requirement: Benchmark-native result validator contract
The canonical case model SHALL provide a private benchmark-native result validator that identifies the expected terminal artifact schema, authoritative native success metric, identity fields, and digest requirements. The validator SHALL validate and normalize the benchmark-emitted terminal artifact without reimplementing, recomputing, or adding task metrics.

#### Scenario: Normalize a valid native result
- **WHEN** the external runtime writes a valid result bound to the active case, attempt, episode, scene, and revisions
- **THEN** the evaluator retains it unchanged and derives only generic attempt status, task result, terminal reason, and native-result reference

#### Scenario: Reject a foreign or malformed result
- **WHEN** a result has the wrong identity, schema, revision, digest, success field, or atomic-finalization evidence
- **THEN** the attempt reports infrastructure failure and task result `not_evaluated`

### Requirement: Automatic case-bound external preparation
An external-benchmark source preparation recipe SHALL be executable by `dimos eval run` and SHALL resolve or create checksum-pinned public assets and the case-pinned OCI image under content-addressed caches and exclusive preparation locks. Preparation MUST complete before the attempt starts its evaluated agent and MUST NOT require a separate user-facing preparation command.

#### Scenario: Reuse verified preparation caches
- **WHEN** every required image and public asset already exists with the compiled digest
- **THEN** the evaluator reuses them without network access or rebuilding and proceeds to the fresh attempt

#### Scenario: Materialize an absent public dependency
- **WHEN** a required non-gated asset or build product is absent and its pinned source is available
- **THEN** the evaluator prepares and verifies it before starting the case-bound runtime

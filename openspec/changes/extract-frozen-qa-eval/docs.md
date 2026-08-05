## User-Facing Docs

- Add `docs/capabilities/agents/evaluation.md` covering:
  - the exact `dimos eval run CASE` command and typed options;
  - installation of the `agents` extra and the dedicated Node adapter build step;
  - OAuth and API-key environment selection without secret CLI values;
  - stdout/stderr behavior, `--json`, `--quiet`, and exit codes;
  - attempt artifact locations, privacy boundaries, and cleanup expectations;
  - the trusted, persistent, unsandboxed nature of CodePolicy;
  - the difference between a completed semantic failure and infrastructure failure.
- Link the new guide from the existing agent capability index under `docs/capabilities/agents/`.
- Preserve the fixture-local `dimos/benchmark/short_horizon_qa/cases/go2_hongkong_office-room-count-smoke/README.md`, prominently stating that oracle `0` is a synthetic plumbing sentinel rather than benchmark truth.

## Contributor Docs

- Add a focused section to `docs/development/testing.md` for:
  - building and testing `packages/pi-code-policy-adapter`;
  - running the focused Python evaluation suite;
  - running the `self_hosted` Hong Kong mechanics gate with its data prerequisite;
  - performing the credentialed operational smoke and checking child-process and lock cleanup.
- No general module, blueprint, configuration, or hardware contributor documentation changes are required.

## Coding-Agent Docs

- Update `docs/coding-agents/index.md` only if it maintains a list of feature-specific validation surfaces; otherwise no coding-agent documentation change is needed.
- Do not modify `AGENTS.md`: the existing rules for optional dependencies, testing, imports, generated blueprints, and security are sufficient for this implementation.
- Retain `frozen-qa-main-extraction-handoff.md` as implementation context if the team wants the branch provenance in-repository; the OpenSpec artifacts become the normative implementation plan.

## Doc Validation

Run the repository-supported documentation checks after inspecting `docs/development/writing_docs.md` for the exact invocation:

```bash
uv run doclinks
uv run md-babel-py run docs/capabilities/agents/evaluation.md
```

If the new page contains no executable Markdown blocks, record that `md-babel-py` has nothing to execute rather than adding artificial examples. No diagram generation is planned.

## No Docs Needed

Documentation is required because this change adds a public CLI, optional installation steps, credential handling, non-obvious exit semantics, and a trusted-unsandboxed execution boundary.

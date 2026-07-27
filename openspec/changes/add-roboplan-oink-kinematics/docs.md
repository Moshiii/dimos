## User-Facing Docs

Update `docs/capabilities/manipulation/index.md` to:

- describe automatic kinematics selection: RoboPlan world selects OInK, while
  other worlds select Pink;
- show how an explicit `kinematics.backend` overrides that default;
- add `roboplan` to the valid backend combinations;
- document that the initial OInK backend accepts world-frame targets, performs
  optional endpoint collision validation, and does not use a self-collision
  barrier; and
- clarify that the existing manipulation extra includes OInK as part of the
  RoboPlan distribution.

Update `dimos/manipulation/planning/README.md` where it describes factory
defaults and the planning package layout so contributor-facing examples match
the new selection behavior.

## Contributor Docs

Keep the architecture rationale in:

- `docs/adr/0001-use-roboplan-world-for-native-kinematics.md`
- `docs/adr/0002-select-world-native-kinematics-by-default.md`

No general contributor-process documentation under `docs/development/` needs
to change.

## Coding-Agent Docs

No update to `docs/coding-agents/` or `AGENTS.md` is needed. This change does not
alter repository workflows, generated registries, or coding-agent conventions.

## Doc Validation

Run:

```bash
uv run doclinks --dry-run docs/capabilities/manipulation/index.md
uv run doclinks --dry-run dimos/manipulation/planning/README.md
uv run md-babel-py run docs/capabilities/manipulation/index.md
```

No diagrams are added, so `bin/gen-diagrams` is unnecessary.

## No Docs Needed

Not applicable. The world-sensitive default and safety limitations are
user-visible and require documentation.

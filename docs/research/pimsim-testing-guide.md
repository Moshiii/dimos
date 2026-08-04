# PiMSim + DimOS branch testing guide

Verified 2026-08-03 against:

- DimOS `feat/pimsim-simulation-provider` at `08e9013c59198f5b42d959880df2f7efbd68d4ee`
- PiMSim `main` at `794f202db9dc9fc5f0785f3c76986aa25be9f209`

PiMSim is currently a private GitHub repository, so cloning it and opening the
PiMSim source links below require repository access.

## What this branch integrates

DimOS owns the application blueprints, control, planning, navigation,
perception, agents, and normal typed streams. PiMSim supplies mechanics,
scenes, simulated sensors/state, episode/reset behavior, and simulator-owned
visualization data. The integration is a separate `pimsim-dimos` distribution;
it does not install PiMSim-specific application blueprints. It registers
`pimsim` in both the `dimos.simulation.providers` and
`dimos.simulation.scene_controls` entry-point groups
([integration metadata](https://github.com/dimensionalOS/pimsim/blob/794f202db9dc9fc5f0785f3c76986aa25be9f209/integrations/dimos/pyproject.toml#L19-L26),
[integration boundary](https://github.com/dimensionalOS/pimsim/blob/794f202db9dc9fc5f0785f3c76986aa25be9f209/integrations/dimos/README.md#L1-L54)).

DimOS discovers providers by entry point and passes a `SimulationRequest`
containing the robot model and optional model, mesh, and scene paths. The
provider returns a `SimulationBinding` containing a backend blueprint, adapter
type/address, and Rerun configuration
([DimOS provider API](https://github.com/dimensionalOS/dimos/blob/08e9013c59198f5b42d959880df2f7efbd68d4ee/dimos/simulation/providers.py#L24-L64)).
PiMSim currently handles `unitree_go2`, `unitree_g1`, and `xarm7`
([provider implementation](https://github.com/dimensionalOS/pimsim/blob/794f202db9dc9fc5f0785f3c76986aa25be9f209/integrations/dimos/src/pimsim_dimos/provider.py#L51-L137)).

## Prerequisites

- Linux or macOS.
- `uv` and Git.
- Python 3.11 or 3.12. PiMSim core declares Python 3.11+, but the DimOS
  integration requires `>=3.11,<3.13`; 3.11/3.12 is therefore the compatible
  intersection
  ([getting-started requirements](https://github.com/dimensionalOS/pimsim/blob/794f202db9dc9fc5f0785f3c76986aa25be9f209/docs/getting-started.md#L7-L14),
  [integration package constraint](https://github.com/dimensionalOS/pimsim/blob/794f202db9dc9fc5f0785f3c76986aa25be9f209/integrations/dimos/pyproject.toml#L5-L14)).
- GitHub credentials authorized for the private `dimensionalOS/pimsim` repo.
- This DimOS branch at or after `08e9013c5`. The checked-out branch is exactly
  that required minimum revision
  ([upstream branch pin](https://github.com/dimensionalOS/pimsim/blob/794f202db9dc9fc5f0785f3c76986aa25be9f209/integrations/dimos/README.md#L23-L27)).
- A graphical desktop if using Rerun, Viser, or the optional MuJoCo window.
- Materialized DimOS Git LFS assets. A fresh checkout may contain small Git LFS
  pointer files instead of usable apartment meshes. Pull them before an
  apartment launch:

  ```bash
  git lfs pull --include="misc/DimSim/scenes/apartment/**"
  head -n 1 misc/DimSim/scenes/apartment/structure.glb
  ```

  The `head` output must not be `version https://git-lfs.github.com/spec/v1`.
  xArm also depends on DimOS LFS-backed model assets, so materializing all
  repository LFS objects with `git lfs pull` is the safest setup when storage
  permits.

## Install into this DimOS checkout

Clone PiMSim anywhere convenient. Keeping the repositories next to one another
matches the upstream examples:

```bash
cd /path/to/parent
git clone https://github.com/dimensionalOS/pimsim.git
```

Create/sync both development environments:

```bash
cd /path/to/pimsim
uv sync --all-extras

cd /path/to/feat-pimsim-simulation-provider
uv sync --extra all
```

Install both PiMSim distributions into the **DimOS** virtual environment:

```bash
cd /path/to/feat-pimsim-simulation-provider
uv pip install --python .venv/bin/python \
  -e /path/to/pimsim \
  -e /path/to/pimsim/integrations/dimos
source .venv/bin/activate
```

The documented sibling-checkout form is `uv pip install -e ../pimsim -e
../pimsim/integrations/dimos`
([official install instructions](https://github.com/dimensionalOS/pimsim/blob/794f202db9dc9fc5f0785f3c76986aa25be9f209/README.md#L91-L122)).
The explicit `--python` above makes it unambiguous that the packages go into
DimOS's `.venv`, not PiMSim's separate `.venv`.

Confirm discovery before launching a large blueprint:

```bash
python -c 'from importlib.metadata import entry_points; print([(e.group, e.name, e.value) for e in entry_points() if e.group.startswith("dimos.simulation") and e.name == "pimsim"])'
python -c 'from dimos.simulation.providers import load_simulation_provider; print(load_simulation_provider("pimsim"))'
```

The first command should show both entry points; the second should print a
`PimSimProvider` rather than `ValueError: Simulation provider 'pimsim' is not
installed`.

## Optional xArm7 smoke test

This check exercises the provider, neutral manipulator shared-memory adapter,
PiMSim mechanics, Rerun configuration, and Viser planning controls:

```bash
dimos --simulation mujoco \
  --simulation-provider pimsim \
  --transport zenoh \
  --viewer rerun \
  run xarm-perception-sim
```

Open the Viser URL printed at startup. Then:

1. Select the arm planning group and drag the end-effector target.
2. Check that the feasibility/IK preview updates.
3. Click **Plan**, optionally preview, then **Execute**.
4. Exercise **Go Home**, **Open Gripper**, **Close Gripper**, and **Reset**.
5. Check Rerun for the tabletop, robot transforms, and scene entity transforms.
6. Stop with `Ctrl-C`, or from another terminal run `dimos stop`.

The supported controls and current limitations are documented upstream:
mechanics and interactive planning work, but full physical pick/lift/place and
RGB-D perception are still open acceptance gates
([xArm integration status](https://github.com/dimensionalOS/pimsim/blob/794f202db9dc9fc5f0785f3c76986aa25be9f209/integrations/dimos/README.md#L109-L128)).
The DimOS blueprint hard-requires MuJoCo plus the `pimsim` provider and requests
the `xarm-tabletop-v1` package
([branch blueprint](https://github.com/dimensionalOS/dimos/blob/08e9013c59198f5b42d959880df2f7efbd68d4ee/dimos/robot/manipulators/xarm/blueprints/simulation.py#L38-L78)).

## Go2 system smoke test

Run the ordinary Go2 application in the cooked apartment. This verified command
opens Rerun in a browser and enables the optional native MuJoCo mechanics
window:

```bash
PIMSIM_MUJOCO_VIEWER=1 uv run dimos \
  --simulation mujoco \
  --simulation-provider pimsim \
  --scene-package dimsim-apartment \
  --transport zenoh \
  --viewer rerun \
  --rerun-open web \
  run unitree-go2
```

After startup, use these interfaces:

- DimOS command center: <http://localhost:7779>
- Rerun web viewer: <http://localhost:9878>
- Native MuJoCo window: asynchronous physics and contact inspection

In another terminal:

```bash
cd /path/to/feat-pimsim-simulation-provider
uv run dimos status
uv run dimos log -f
```

The provider's Go2 boundary supplies velocity commands, odometry, lidar, RGB,
camera calibration, joint state, and TF. DimOS still owns mapping, costmaps,
planning, movement management, and visualization
([Go2 boundary](https://github.com/dimensionalOS/pimsim/blob/794f202db9dc9fc5f0785f3c76986aa25be9f209/integrations/dimos/README.md#L67-L92)).

To exercise lifecycle and motion RPCs interactively over the maintained Zenoh
transport, run:

```bash
uv run dimos --transport zenoh shell
```

The documented Python client equivalent is:

```python
from dimos import Dimos

app = Dimos.connect()
app.PimSimGo2.respawn_at(x=0.0, y=0.0, yaw=0.0)
app.stop()
```

([lifecycle example](https://github.com/dimensionalOS/pimsim/blob/794f202db9dc9fc5f0785f3c76986aa25be9f209/integrations/dimos/README.md#L147-L161)).

For an agent-driven manual test, export `OPENAI_API_KEY`, replace `unitree-go2`
with `unitree-go2-agentic`, wait for `Discovered tools from MCP server`, and
send a bounded command such as:

```bash
uv run dimos --transport zenoh agent-send "move forward one meter, then stop"
```

That adds the DimOS agent/API-key dependencies; it is not needed to validate
the simulator provider itself.

The maintained integration path is Zenoh. The default Go2 lidar/localization
path is deliberately deterministic system-evaluation lidar with exact
simulator localization; it is not a fidelity model of rolling MID-360
acquisition, PointLIO drift, or device networking
([transport and fidelity boundary](https://github.com/dimensionalOS/pimsim/blob/794f202db9dc9fc5f0785f3c76986aa25be9f209/integrations/dimos/README.md#L78-L92),
[maintained transport](https://github.com/dimensionalOS/pimsim/blob/794f202db9dc9fc5f0785f3c76986aa25be9f209/integrations/dimos/README.md#L163-L170)).

## G1 smoke test

```bash
dimos --simulation mujoco \
  --simulation-provider pimsim \
  --scene-package dimsim-apartment \
  --transport zenoh \
  --viewer rerun \
  run unitree-g1-groot-wbc
```

This exercises PiMSim mechanics and robot-equivalent sensor/state streams with
DimOS's GR00T whole-body control, arbitration, mapping, and navigation
([G1 boundary](https://github.com/dimensionalOS/pimsim/blob/794f202db9dc9fc5f0785f3c76986aa25be9f209/integrations/dimos/README.md#L94-L107)).

For collision/contact inspection, add `PIMSIM_MUJOCO_VIEWER=1` before a launch
command. This opens an asynchronous mechanics viewer; it does not own or pace
physics
([native viewer contract](https://github.com/dimensionalOS/pimsim/blob/794f202db9dc9fc5f0785f3c76986aa25be9f209/integrations/dimos/README.md#L130-L145)).

## Automated tests, from cheapest to most complete

### 1. PiMSim core

```bash
cd /path/to/pimsim
uv run pytest
uv run mypy src
uv run ruff check .
```

These are the documented local contract checks
([PiMSim development checks](https://github.com/dimensionalOS/pimsim/blob/794f202db9dc9fc5f0785f3c76986aa25be9f209/README.md#L241-L247)).
For exact CI parity, PiMSim CI uses `uv sync --locked --dev`, the same three
checks (pytest with `-q`), and `uv build`
([CI workflow](https://github.com/dimensionalOS/pimsim/blob/794f202db9dc9fc5f0785f3c76986aa25be9f209/.github/workflows/ci.yml#L15-L35)).

### 2. PiMSim-DimOS integration contracts

PiMSim core config limits normal pytest discovery to `src`, so the separate
integration tests must be named explicitly
([pytest configuration](https://github.com/dimensionalOS/pimsim/blob/794f202db9dc9fc5f0785f3c76986aa25be9f209/pyproject.toml#L59-L61)).
After installing the editable provider into DimOS:

```bash
cd /path/to/feat-pimsim-simulation-provider
.venv/bin/pytest /path/to/pimsim/integrations/dimos/tests -v
```

This command is inferred from the upstream test layout rather than documented
verbatim. The suite covers provider composition and transports, apartment
alignment, G1 navigation lidar, and the xArm mechanics/reset contract
([integration tests](https://github.com/dimensionalOS/pimsim/tree/794f202db9dc9fc5f0785f3c76986aa25be9f209/integrations/dimos/tests)).

### 3. DimOS provider and adapter contracts

```bash
cd /path/to/feat-pimsim-simulation-provider
.venv/bin/pytest \
  dimos/simulation/test_providers.py \
  dimos/hardware/manipulators/sim/test_shm_adapter.py \
  -v
```

These validate external provider discovery and the simulator-neutral
manipulator shared-memory boundary before exercising an entire application.

### 4. Cross-simulator system acceptance

The branch's end-to-end scenarios select the simulator through the registered
`SceneControl` contract. The documented PiMSim command is:

```bash
cd /path/to/feat-pimsim-simulation-provider
DIMOS_TRANSPORT=zenoh DIMOS_E2E_SIMULATOR=pimsim \
  .venv/bin/pytest -o addopts='' -m self_hosted_large \
  dimos/e2e_tests/test_dimsim_spatial_memory.py
```

([DimOS cross-simulator test instructions](https://github.com/dimensionalOS/dimos/blob/08e9013c59198f5b42d959880df2f7efbd68d4ee/docs/development/testing.md#L72-L98)).
These are large system tests: they start processes, may initialize perception
models and an agent, and can have long readiness/task timeouts. Run them only
after the focused contracts and manual smoke test pass. Other relevant files
can be substituted explicitly, including:

```text
dimos/e2e_tests/test_dimsim_walk_forward.py
dimos/e2e_tests/test_dimsim_path_replaning.py
dimos/e2e_tests/test_dimsim_spatial_memory.py
```

Evaluation fixtures should use the neutral scene-control provider, whose
operations include respawn, wall insertion, goal publication, and semantic
ground-truth queries
([PiMSim scene control](https://github.com/dimensionalOS/pimsim/blob/794f202db9dc9fc5f0785f3c76986aa25be9f209/integrations/dimos/src/pimsim_dimos/scene_control.py#L27-L72)).

## Local validation snapshot

The following results were observed in this worktree at the two revisions at
the top of this note:

- DimOS focused provider/adapter contracts: **4 passed**.
- Cross-repository `pimsim-dimos` provider tests: **10 passed**.
- Blueprint configuration test suite: **36 passed**.
- The Go2 agentic stack started with PimSim, published camera and navigation
  data, served the command center and Rerun web viewer, and accepted Zenoh RPC
  shell connections.

These results support the provider wiring and focused integration contracts,
but they are not a claim that every PiMSim or DimOS test is green.

## PiMSim's standalone scalar API

For deterministic orchestration tests that do not need DimOS or physical
fidelity, PiMSim exposes `SimSession`, `ExecutionConfig`, `WorldSpec`,
`RobotSpec`, `ActionBatch`, and a reference backend. The complete runnable
example is in the
[getting-started guide](https://github.com/dimensionalOS/pimsim/blob/794f202db9dc9fc5f0785f3c76986aa25be9f209/docs/getting-started.md#L32-L66).
Every action is tied to the current session, episode, robot set, and control
tick; stale or cross-episode actions are rejected before physics advances
([scalar-session contract](https://github.com/dimensionalOS/pimsim/blob/794f202db9dc9fc5f0785f3c76986aa25be9f209/docs/contracts/scalar-session.md)).

## Common failure interpretation

- **`Simulation provider 'pimsim' is not installed`**: `pimsim-dimos` was not
  installed into the DimOS `.venv`, or the command is using a different Python
  environment.
- **xArm requirement error**: launch with both `--simulation mujoco` and
  `--simulation-provider pimsim`.
- **Camera transport error**: use `--transport zenoh` for the maintained path;
  the current provider camera implementation supports Zenoh and LCM, not every
  DimOS transport
  ([camera transport dispatch](https://github.com/dimensionalOS/pimsim/blob/794f202db9dc9fc5f0785f3c76986aa25be9f209/integrations/dimos/src/pimsim_dimos/provider.py#L193-L198)).
- **No GUI window**: Rerun/Viser need a graphical session. The native MuJoCo
  window is opt-in through `PIMSIM_MUJOCO_VIEWER=1`.
- **GLB parse/load failures or implausibly tiny apartment assets**: check the
  first line of the `.glb`. If it is a Git LFS pointer header, run `git lfs
  pull` before debugging the simulator or scene cooker.
- **Go2 result differs from physical MID-360/PointLIO**: this is expected for
  the default deterministic evaluation sensor/localization tier.
- **Full xArm pick/RGB-D test does not exist yet**: upstream explicitly marks
  that workflow as in progress; validate mechanics, interactive planning,
  reset/home, and gripper operations without treating them as proof of the
  unfinished perception/pick acceptance gate.

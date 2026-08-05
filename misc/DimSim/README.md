# DimSim

Browser-based 3D simulator (Three.js + Rapier) plus a Deno bridge that talks LCM/WS to [dimos](https://github.com/dimensionalOS/dimos). Lives inside dimos as `misc/DimSim/`.

```
src/        — browser engine (vite-bundled)
cli/        — Deno CLI + bridge server + headless launcher + LCM vendor
evals/      — simulator-local harness (browser) + runner (Deno) + rubrics
scenes/     — user-authored scenes (JS) + optional engine-level workflows
public/     — static assets (agent GLB, logo)
docs/       — guides
```

## Run

dimsim is launched by dimos directly when you pick `--simulation dimsim`:

```bash
cd <dimos-repo>
uv run dimos --simulation dimsim --dimsim-scene=apartment run unitree-go2-agentic
```

On first run, `cli/cli.ts` will build `dist/` via Vite (dimsim ships its frontend as source — Deno+Vite materializes it in ~20s).

## Docs

- [docs/getting-started.md](docs/getting-started.md) — 5-minute tour
- [docs/scenes.md](docs/scenes.md) — create + edit scenes
- [docs/evals.md](docs/evals.md) — simulator-local workflows vs DimOS system tests

## Install the CLI (optional)

If you want `dimsim` as a global command:

```bash
cd misc/DimSim/cli
deno install -gAf --unstable-net --name=dimsim --config=./deno.json ./cli.ts
```

After install:

```bash
dimsim dev --scene apartment              # standalone dev server + browser
dimsim eval list                          # list workflows under scenes/*/evals/
dimsim eval <workflow>                    # run a simulator-local workflow
dimsim eval --headless --scene apartment --workflow <workflow>
```

## Build manually

```bash
npm install      # browser deps (three, rapier, vite)
npm run build    # → dist/
```

# VLN-CE R2R evaluation backend

This package integrates one official VLN-CE R2R episode with the canonical
`dimos eval run <case>` command. The checked-in development case uses Habitat's
public `17DRP5sb8fy` MP3D example and official R2R/VLN-CE training episode 515.

The repository retains only source URLs, identities, instructions, and
checksums. It does not redistribute scene meshes, episode archives, semantic
annotations, reference paths, or trained models. Preparation downloads public
development inputs into the DimOS cache and mounts them read-only into the
one-shot benchmark container. Full MP3D splits remain user-supplied data under
the Matterport3D Terms of Use.

VLN-CE code is MIT licensed. R2R data and Matterport3D-derived assets carry
separate terms; the MIT license does not cover them. See
`upstream-manifest.v1.json` for pinned sources and license links.

Results from this development case use a complete public navigability map and
the DimOS planar velocity interface. They are training-scene integration
results, not standard VLN-CE validation, test, or leaderboard results.

## Run the case

```bash
uv run dimos eval run \
  dimos/benchmark/vlnce_r2r/cases/mp3d-example-episode-515/case.json \
  --output=/tmp/dimos-vlnce-r2r
```

The command prepares both public archives, verifies every pinned checksum,
resolves the pinned OCI image, starts Habitat and the case-bound DimOS stack,
runs one Pi/CodePolicy session, waits for VLN-CE STOP or timeout, and tears down
everything it started. The first run downloads about 68 MB of scene data and
2.6 MB of episode data. Later runs verify and reuse the content-addressed cache,
including offline runs.

Evaluation runs are headless by default. Use
`uv run dimos --viewer rerun --rerun-open web --rerun-web eval run ...` for
browser visualization. The viewer receives only public DimOS streams; changing
it does not change the case fingerprint or official scoring.

To record the official Habitat RGB observation beside a private top-down
trajectory view, add `--render native`:

```bash
uv run dimos eval run \
  dimos/benchmark/vlnce_r2r/cases/mp3d-example-episode-515/case.json \
  --output=/tmp/dimos-vlnce-r2r \
  --render native
```

The attempt contains `native-render.mp4` and `native-render.v1.json`. The video
uses simulated action time: it holds the initial and terminal states for one
second and records one frame per accepted 0.1-second control period. The map,
pose, and traveled trajectory are private operator evidence and are never
exposed through the agent's public gateway. A recording failure is reported in
the metadata without changing official benchmark scoring.

## Interpret the result

The container computes the official VLN-CE measures. DimOS retains the native
`terminal-private/vlnce-result.v1.json` bytes and maps only official `SUCCESS`
to the compact pass/fail result. `SUCCESS=0` is a completed task failure. A
missing, malformed, or foreign result is an infrastructure failure and remains
not evaluated.

The result includes navigation error, oracle success, success, SPL, nDTW, path
length, and step count. Treat every result as
`dimos_geometry_training_scene_development`, not as a standard VLN-CE
validation/test leaderboard result.

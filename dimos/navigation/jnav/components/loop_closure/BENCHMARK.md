# gsc_pgo loop-closure benchmark

Self-consistency of PGO-corrected trajectory vs raw odometry, across every usable
recording in `~/datasets/mid360_recordings/` and `~/datasets/go2_recordings/`.

- **sensor** — `go2` = onboard L1 short-range lidar (go2 config); `mid360` = mounted
  Livox (module defaults). One config per sensor, applied to every map of that sensor.
- **odom** — the odometry variant replayed (pointlio preferred, fastlio fallback; go2
  uses the recording's native `odom`/`go2_odom`).
- **raw_spread / corr_spread** — start↔end position gap (m) before/after correction. A
  loop that truly closes drops corr_spread toward 0; `corr > raw` means the map was
  deformed (worse).
- **tag_impr** — AprilTag re-sighting spread ratio (higher better). **vox_impr** — lidar
  voxel-agreement improvement (higher better).
- **verdict** — `improve` (loop visibly closes), `no-harm` (already-tight map preserved),
  `HARM` (correction deforms the map).

Metrics are self-consistency, not vs a ground-truth SLAM (FAST-LIO/Point-LIO are the
input odometry, not a baseline to beat). Two PGO implementations are compared: the Rust
`gsc_pgo` (scan-context + gtsam ISAM2, module default config) and Ivan's pure-module
`IvanPGO` (KDTree-radius + ICP, `dimos/mapping/loop_closure/pgo.py` core).

## gsc_pgo (primary)

| recording | sensor | odom | closures | keyframes | raw_spread | corr_spread | tag_impr | vox_impr | note | verdict |
|---|---|---|--:|--:|--:|--:|--:|--:|---|---|
| outdoor_small_loop | go2 | odom | 105 | 945 | 0.122 | 0.868 | -6.126 | -0.0544 |  | HARM |
| outdoor_small_loop | mid360 | fastlio_odometry | 77 | 1154 | 0.122 | 0.131 | -0.072 | -0.0007 |  | no-harm |
| grassy_field | go2 | odom | 115 | 1057 | 61.010 | 57.915 | 0.051 | 0.0518 | no-harm target | improve |
| grassy_field | mid360 | fastlio_odometry | 35 | 1308 | 61.010 | 58.498 | 0.041 | -0.0002 | no-harm target | no-harm |
| gir_stairs1 | go2 | odom | 184 | 1057 | 0.035 | 1.672 | -47.201 | 0.0634 |  | HARM |
| gir_stairs1 | mid360 | pointlio_odometry | 111 | 1283 | 0.035 | 0.051 | -0.461 | 0.0000 |  | no-harm |
| gir_stairs2 | go2 | odom | 400 | 1677 | 0.969 | 6.166 | -5.363 | -0.1087 |  | HARM |
| gir_stairs2 | mid360 | pointlio_odometry | 307 | 1937 | 0.969 | 0.072 | 0.926 | 0.0259 |  | improve |
| gir_park1 | go2 | odom | 174 | 1203 | 0.048 | 8.266 | -172.581 | 0.0231 |  | HARM |
| gir_park1 | mid360 | pointlio_odometry | 142 | 1699 | 0.048 | 0.093 | -0.951 | -0.0011 |  | HARM |
| gir_park1_2 | go2 | go2_odom | 93 | 535 | 0.081 | 16.622 | -204.265 | -0.0174 |  | HARM |
| gir_park1_2 | mid360 | fastlio_odometry | 26 | 606 | 0.081 | 0.136 | -0.685 | 0.0006 |  | HARM |
| huge_loop_go2 | go2 | odom | 228 | 2467 | 10.809 | 300.800 | -26.828 | -0.0700 |  | HARM |
| huge_loop_go2 | mid360 | fastlio_odometry | 171 | 2649 | 59.557 | 57.670 | 0.032 | 0.0034 |  | no-harm |
| huge_loop_go2 | mid360 | pointlio_odometry | 292 | 2479 | 10.809 | 0.329 | 0.970 | 0.0108 | loop closes | improve |
| huge_loop_realsense | mid360 | pointlio_odometry | 387 | 2716 | — | — | — | 0.0128 | voxel-only | improve |
| china_office1 | go2 | go2_odom | 329 | 1194 | 0.242 | 1.974 | -7.171 | 0.0142 |  | HARM |
| china_office1 | mid360 | pointlio_odometry | 229 | 1579 | 0.242 | 0.239 | 0.012 | 0.0005 |  | no-harm |
| sf_office_survey1 | go2 | odom | — | — | — | — | — | — | not run (see below) | — |
| sf_office_survey2 | go2 | odom | — | — | — | — | — | — | not run (see below) | — |

## IvanPGO (comparison — same recordings, same odom)

| recording | sensor | odom | closures | keyframes | raw_spread | corr_spread | tag_impr | vox_impr |
|---|---|---|--:|--:|--:|--:|--:|--:|
| outdoor_small_loop | go2 | odom | 24 | 941 | 0.122 | 0.662 | -4.435 | -0.0819 |
| outdoor_small_loop | mid360 | fastlio_odometry | 43 | 1125 | 0.122 | 0.118 | 0.029 | -0.0079 |
| grassy_field | go2 | odom | 38 | 1057 | 61.010 | 62.199 | -0.019 | 0.0028 |
| grassy_field | mid360 | fastlio_odometry | 20 | 1267 | 61.010 | 60.582 | 0.007 | -0.0073 |
| gir_stairs1 | go2 | odom | 54 | 1048 | 0.035 | 0.662 | -18.088 | 0.0065 |
| gir_stairs1 | mid360 | pointlio_odometry | 61 | 1258 | 0.035 | 0.041 | -0.179 | -0.0019 |
| gir_stairs2 | go2 | odom | 196 | 1671 | 0.969 | 2.847 | -1.938 | 0.0812 |
| gir_stairs2 | mid360 | pointlio_odometry | 197 | 1789 | 0.969 | 0.096 | 0.901 | 0.0119 |
| gir_park1 | go2 | odom | 62 | 1201 | 0.048 | 1.381 | -28.011 | -0.0457 |
| gir_park1 | mid360 | pointlio_odometry | 127 | 1562 | 0.048 | 0.087 | -0.822 | -0.0049 |
| gir_park1_2 | go2 | go2_odom | 15 | 534 | 0.081 | 1.100 | -12.589 | -0.0118 |
| gir_park1_2 | mid360 | fastlio_odometry | 14 | 601 | 0.081 | 0.962 | -10.875 | -0.0065 |
| huge_loop_go2 | go2 | odom | 68 | 2463 | 10.809 | 22.767 | -1.106 | -0.0674 |
| huge_loop_go2 | mid360 | pointlio_odometry | 64 | 2175 | 10.809 | 10.015 | 0.073 | -0.0067 |
| huge_loop_go2 | mid360 | fastlio_odometry | 57 | 2317 | 59.557 | 57.957 | 0.027 | -0.0144 |
| huge_loop_realsense | mid360 | pointlio_odometry | 233 | 2440 | — | — | — | 0.0055 |
| china_office1 | go2 | go2_odom | 138 | 1185 | 0.242 | 2.096 | -7.680 | 0.0451 |
| china_office1 | mid360 | pointlio_odometry | 131 | 1551 | 0.242 | 0.241 | 0.003 | -0.0022 |

On the huge loop, gsc_pgo (pointlio) drops corr_spread to **0.329 m** (loop closes);
IvanPGO leaves it at **10.015 m** (barely corrected). gsc_pgo closes it; IvanPGO does not.

## Skipped / not run

- `misc/`, `_deleted_gt_streams_backup` — not recordings.
- `sf_office_survey1`, `sf_office_survey2` (`~/datasets/go2_recordings/`) — bare-Go2 L1
  recordings, wired into the runner but **not benched**: every attempt wedged in zenoh
  peer discovery because a co-resident `dim-lcm-constellation` process (deno server +
  `spy` monitor) shares the multicast bus and its stale peers hang the eval's session
  handshake. Not a code fault; needs a clean shell without that tool running. Re-run:
  `python temp_dont_commit.ignore/hk_scratch/demo_bench.py --only sf_office_survey1`
  (and `--only sf_office_survey2`).

## Reproduce

Runner: `temp_dont_commit.ignore/hk_scratch/demo_bench.py` (slim-copies each recording to
scratch, never touches originals). Per-recording before/after top-down + isometric PNGs
land in `eval_results/<recording>__<label>__PGO/` (gitignored). go2 config =
`dimos/robot/unitree/go2/blueprints/smart/unitree_go2_pgo.py`; mid360 config = gsc_pgo
module defaults.

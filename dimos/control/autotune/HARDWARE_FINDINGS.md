# Go2 autotune — hardware run findings

Three consecutive hardware runs of `unitree-go2-autotune`. Run 3 is the first
with `velocity_api=True` (see below); the robot shook during that run.

| | Run 1 | Run 2 | Run 3 (`velocity_api=True`) |
|---|---|---|---|
| `vx.K` | 0.072 | 0.071 | **−0.017** |
| `vx.tau` | 0.600 (ceiling) | 0.399 | **0.030 (floor)** |
| `wz.K` | −0.176 | −0.212 | **−0.0067** |
| `wz.tau` | 0.600 (ceiling) | 0.600 (ceiling) | **0.030 (floor)** |
| `L` (both channels, every run) | 0.300 (ceiling) | 0.300 (ceiling) | 0.300 (ceiling) |

Bounds: `DEFAULT_TAU_BOUNDS=(0.03, 0.6)`, `DEFAULT_L_BOUNDS=(0.05, 0.30)` in
`fit/pose_fopdt.py`. `L` pinned at its ceiling in all 6 channel-fits across all
3 runs. `tau` pinned at the floor on both channels only in run 3.

Confirmed separately: `wz`'s sign is not a command/feedback axis mismatch —
runs 31/32 watched directly, robot turns left for positive `wz` as expected.
One real yaw-wrap discontinuity found in run 2's `wz` segment 15 (jump of
+6.264 rad, i.e. ~2*pi) via `check_yaw_wrap.py`, but 17/18 `wz` segments were
clean, so the wrap alone doesn't explain the fit quality.

Read: run 3's collapse is most likely explained by the robot's physical
shaking corrupting the recorded motion (see `velocity_api` note below), not a
new/different root cause from runs 1-2.

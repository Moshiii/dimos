# Go2 fit quality — all runs, for discussion with Mustafa

## The numbers, across all hardware runs

`L` pinned at its search ceiling (0.300) in every single fit, both channels, all
4 runs — 8 for 8. `K` is negative in most fits, which real hardware watching
confirms is wrong (robot turns the correct direction; the fit doesn't).

| | Run 1 | Run 2 | Run 3 (shaking) | Run 4 (4s) | Run 5 (6s) | Run 6 (8s, no zenoh) | Run 7 (8s, wz unwrapped, no zenoh) | Run 8 (zenoh back, L<=1.0) | Run 9 (zenoh, L<=1.0) | Run 10 (no zenoh, vx only) | **Run 11 (2nd robot, zenoh out)** | Old tool's ground truth |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| `vx.K` | 0.072 | 0.071 | −0.017 | −0.524 | +0.572 | +0.779 | — (not run) | — (not characterized) | +0.275 | +0.271 | **−0.279** | 0.80–0.92 |
| `vx.tau` | 0.600 (edge) | 0.399 | 0.030 (edge) | 0.334 | 0.363 | 0.262 | — | — | 0.061 | 0.157 | **0.030 (edge)** | 0.30–0.40 |
| `vx.L` | 0.300 (edge) | 0.300 (edge) | 0.300 (edge) | 0.300 (edge) | 0.300 (edge) | 0.300 (edge) | — | — | 0.528 | 0.421 | **0.538** | **0.065–0.15** |
| `vx.verdict` | marginal | marginal | — | marginal | marginal | pass | — | absent (identity placeholder, caveat) | marginal | marginal | **marginal/worse** | — |
| `wz.K` | −0.176 | −0.212 | −0.0067 | −0.038 | +0.319 | +0.044 | +0.779 | +0.775 | +0.782 | — (not run) | **+0.828** | 0.90–2.45 |
| `wz.tau` | 0.600 (edge) | 0.600 (edge) | 0.030 (edge) | 0.030 (edge) | 0.051 | 0.600 (edge) | 0.300 (in range) | 0.129 | 0.264 | — | **0.045** | 0.3–0.60 |
| `wz.L` | 0.300 (edge) | 0.300 (edge) | 0.300 (edge) | 0.300 (edge) | 0.300 (edge) | 0.300 (edge) | 0.300 (edge, old bound) | 0.430 (not pinned) | 0.335 (not pinned) | — | **0.545 (not pinned)** | **0.05–0.15** |
| `wz.verdict` | absent | absent | — | — | pass | absent | pass | pass | pass | — | **pass** | — |

**Run 7 confirms the `np.unwrap()` fix.** `wz.K` jumped from 0.044 to 0.779 -- by far the closest to the reference range of any run, channel, ever. `wz.tau` came fully off its pinned edge for the first time and landed inside the reference band (0.3-0.60). This is strong, direct evidence the yaw-wrap corruption was the real problem, not duration.

**`wz` is now solid and repeatable across four runs (7, 8, 9, 11), including a second physical robot** -- `K` has landed 0.779 / 0.775 / 0.782 / 0.828, verdict `pass` every time. The unwrap fix plus the wider `L` bound look like a real, stable, generalizing solution for `wz`.

**`vx` is broken, systematically, not from zenoh or the bounds change.** Two controlled tests ruled those out directly:
- Run 9 (zenoh) vs Run 10 (zenoh removed, same robot, same duration): nearly identical bad result (`K` 0.275 vs 0.271). Zenoh isn't it.
- Refit of Run 10's real data back down to the *old* `L<=0.30` bound: `K` stayed at 0.271, didn't recover Run 6's 0.779. The bounds widening isn't an artifact that broke a previously-good fit.

**Run 11 (a second, different physical robot) also fails on `vx`** -- worse, in fact (`K` goes negative, `tau` pins at the opposite/low edge) -- while `wz` stays clean on that same robot. That rules out "just this one robot's mechanical condition" too. Three runs, three different bad `vx` signatures, `wz` clean throughout: this is the same shape of problem `wz`'s yaw-wrap bug had -- one channel structurally broken, reproducing across hardware -- not yet root-caused.

**Leading hypothesis, unconfirmed:** `SegmentRecorder.on_pose` (`autotune_driver.py:95`) reads `vx` as raw world-frame `msg.position.x`, not a body-frame/forward-distance measure. Any real heading drift during a `vx` step would corrupt this channel's step-response shape while leaving `wz` (yaw itself) unaffected. Needs `check_yaw_wrap.py`'s segment range-covered output (or the raw trajectory shape) against a real `vx` run to confirm.

## `L` pinning: not a bounds bug -- real deadtime is ~0.45s

Refit Run 7's `wz` segments offline (`refit_segments.py`, no robot needed)
with `L` allowed up to 2.0s instead of the default 0.30s ceiling
(`DEFAULT_L_BOUNDS` in `fit/pose_fopdt.py`):

```
wz: K=0.7778  tau=0.1656  L=0.4538  r2=0.981  valid=True
```

`L` did **not** pin at the new 2.0s ceiling -- it converged to 0.4538s, with a
strong fit (r2=0.981) and `K` essentially unchanged from the bounded run
(0.778). `tau` dropped to 0.166 to compensate (L and tau trade off against
each other describing the same overall response lag, so this is expected,
not a red flag).

**Conclusion: the 13/13 pinning wasn't a search-bounds bug. The real deadtime
in the current setup is ~0.45s -- roughly 3-9x the old tool's reference range
(0.05-0.15s).** That's a genuine question for Mustafa, not a fitter issue:
why does the current live-wiring path have ~0.3-0.4s more real
command-to-response delay than whatever the old characterization tool
measured on? Candidates, untested: `velocity_api=True` (SPORT_MOD Move API)
vs whatever mode the old tool used, or WebRTC round-trip latency in the
current path that the old tool's setup may not have gone through.

Practical note: `DEFAULT_L_BOUNDS` (0.05, 0.30) should probably be widened
regardless of the root cause, since it's currently clipping every fit's `L`
below its real value -- worth raising with Mustafa alongside the root-cause
question.

**Update:** widened to `(0.05, 1.0)` (commit `33604213c`). Run 8 (full
36-step battery) will confirm.

**`vx` improved monotonically with duration: 4s → 6s → 8s got steadily better every time** (`K` climbing toward the reference range, `verdict` reaching `pass`, saturation dropping from 53% to 18%). Clean, consistent signal — this channel is a time problem, and increasing time is fixing it.

**`wz` did not improve monotonically — it peaked at 6s and got *worse* at 8s.** `tau` came off its pinned edge at 6s (0.051) then went right back to the edge at 8s (0.600); verdict went `pass` → `absent`. More time helped up to a point, then actively hurt. See the analysis below — this is the key evidence that `wz`'s remaining problem is not purely about duration.

## Where the "old tool's ground truth" column comes from

A separate, earlier pipeline — `dimos/utils/characterization` — the one this
autotune module's own README says it supersedes. It's Mustafa's own prior
work (all 8 commits building it are his). Same robot, same pose-domain
approach, nearly line-for-line identical fitting math and search bounds as
the current `fit/pose_fopdt.py` — so the algorithm isn't what's different.

The vendored artifact currently shipped with `HolonomicPoseFollowerTask`
(`dimos/control/tasks/rpp_path_follower_task/artifacts/go2_posedomain.json`)
was produced by that same old tool, on real Go2 hardware, June 23 — its own
`provenance` block names the recording session directly. That's the artifact
we've been using as the "known good" reference throughout.

## The one concrete, checkable methodology difference found so far

**The old tool held each step for 8 seconds. Ours held it for 4 by default —
now confirmed, not just a guess, for `vx`.** `step_duration_s` was raised in
two increments (6s, then 8s matching the old tool exactly) and `vx` improved
every single time. `wz` improved at 6s then regressed at 8s — see below.

(`dimos/utils/benchmarking/plant.py`, `GO2_PLANT_PROFILE.step_s = 8.0`, vs.
`AutotuneDriverConfig.step_duration_s` in the current module, now raised to
match.)

## Verdict: is this a time problem or a logic problem?

**Both — split by channel.**

**`vx` is a time problem, and it's solved.** Every increase in duration made
it strictly better, with no reversals: 4s → 6s → 8s tracked cleanly toward
the reference values, ending in a `pass` verdict with low saturation. That is
exactly the signature of "the recording wasn't long enough to see the
plateau" — more time, better answer, every time.

**`wz` is not simply a time problem — the data doesn't support that.** If it
were, more duration should help monotonically, the same way it did for `vx`.
Instead `wz` peaked at 6s and got *worse* at 8s (`tau` re-pinned, verdict
lost). A pure timing problem cannot produce that shape — something else is
actively getting worse as duration increases.

The leading candidate is the yaw-wrap issue found earlier (`wz` is measured
as raw yaw, which wraps at +-180 degrees). At 8s, the largest `wz` amplitude
sweeps roughly 480 degrees — more than a full rotation, virtually guaranteeing
at least one wrap crossing, likely more for the larger-amplitude repeats.
Longer recordings mean *more* opportunities for this specific corruption on
`wz`, which would explain why more time helps up to a point (short recordings
still weren't enough to see the response) and then reverses (long recordings
accumulate more wrap damage than the extra time is worth). That is a data/logic
problem in how `wz` is measured, not a duration problem — and it means `wz`
may need a different fix than "give it more time" (e.g. unwrapping the angle
before fitting, or capping its amplitude so a run never crosses a full turn),
not simply matching `vx`'s duration.

## The `wz` fix: missing `np.unwrap()` -- confirmed by Run 7

**Problem:** `wz` is recorded as raw yaw, which snaps by 360° whenever the
robot's real rotation crosses ±180° — a fake discontinuity, not a real
plant response. **What the old tool does differently:**
`dimos/utils/characterization/recording_io.py` calls `np.unwrap(yaw) - yaw0`
before handing the channel to its fitter; our live recorder
(`autotune_driver.py`'s `SegmentRecorder.on_pose`) uses the raw angle as-is,
with no unwrapping. **Why this is likely the fix:** `np.unwrap()` is exactly
designed to undo this class of artifact — it turns the sawtooth-looking
wrapped signal back into the smooth curve it physically is, which is the
same difference between the old tool's clean `wz.K=0.899` and ours landing
on near-zero or negative values.

**Confirmed on hardware, Run 7:** one-line change
(`autotune_driver.py::SegmentRecorder.stop()`, unwrap applied to `wz` only),
`step_duration_s=8.0` unchanged. `wz.K` went from 0.044 -> 0.779, `tau` came
off its pinned edge into the reference band, verdict `pass`. Not a guess
anymore -- this was the real bug.

## Open issue: intermittent command timeouts during some wz runs

During the run-5 session, `JointVelocityTask vel_go2 timed out (no update for
~0.2s)` fired repeatedly during some of the early `wz` runs, correlating with
what looked like little or no actual robot movement on at least one run.
Not yet root-caused. Worth checking directly: re-run
`check_yaw_wrap.py` against the fresh `segments.pkl` and look at its
"range covered" output for every `wz` segment — any segment with a
near-zero range is a real, corrupted data point that should be treated as
noise, not signal, in that run's fit. Possible causes not yet ruled in or
out: WebRTC signal quality changing with robot orientation while rotating,
or general link reliability under real conditions.

## Why did run 4 (velocity_api=True, zenoh removed) stop shaking?

Honest answer: **we don't actually know for certain yet — two things changed
at once.** Between run 3 (shaking) and run 4 (clean), two changes landed
together:

1. A fix to `KeyboardTeleop` — it was continuously publishing zero-commands
   onto the same `/cmd_vel` channel `AutotuneDriver` was using, fighting with
   the intended step commands. Fixed by setting
   `publish_only_when_active=True`.
2. Zenoh's go2 connection code was temporarily removed from the branch, as a
   diagnostic (unrelated code path — not imported by anything the autotune
   blueprint runs).

**Update: this is genuinely unresolved, and the obvious guess turned out to be
wrong.** A direct test was run: `KeyboardTeleop` fix applied, zenoh still
present, `velocity_api=True` — and it **still shook**. Only after zenoh was
also removed did it stop. That contradicts the code-level reasoning below
(zenoh's files aren't imported by the autotune blueprint at all, a port scan
for its default port found nothing listening, and WebRTC worked reliably
throughout — none of which point at zenoh mattering).

No code-level mechanism has been found to explain it. Current best
explanation is **run-to-run physical variability** (battery charge, surface,
warm-up state) rather than an actual zenoh effect, but that is not confirmed
either. Flagging as open rather than guessing further — worth asking Mustafa
if he has a theory.

## Closing note: exhausted the code-level search

Went back through every import path to see whether removing
`dimos/robot/unitree/go2/zenoh/blueprints.py` / `zenohconnection.py` could
change behavior even when the autotune blueprint never references them
directly. Two things checked, both dead ends:

1. **`zenoh` the package loads on every run regardless.**
   `dimos/core/transport.py` imports `zenohpubsub.py` at module top level,
   which does `import zenoh` and (via `zenohservice.py`)
   `zenoh.init_log_from_env_or("warn")` at import time. `transport.py` is
   pulled in by our own `unitree_go2_coordinator.py`
   (`from dimos.core.transport import LCMTransport`), so this fires on
   *every* `dimos run`, shaking or not. This is a real correction to the
   earlier claim that zenoh isn't in our path — but it's identical in both
   conditions, so it can't be what changed.
2. **Blueprint registration is lazy, not eager.** `get_blueprint_by_name`
   (`dimos/robot/get_all_blueprints.py`) only imports a blueprint's module
   when that blueprint is actually selected on the CLI. So having the
   `go2-zenoh-*` entries present in `all_blueprints.py` does not cause
   `zenohconnection.py` to be imported just by existing — running
   `unitree-go2-autotune` never touches that file whether it's present or
   removed. Rules out "registering it eagerly loads/opens something."

No remaining code path in this repo explains the difference. The one
untested angle is **state on the Go2's own companion computer**, outside
this repo: if zenoh's `peer` mode with discovery was ever actually run
against this robot and the session wasn't cleanly closed, that could leave
stale peer/discovery traffic independent of which blueprint runs now. Not
confirmed — would need to SSH into the robot to check, which is currently
blocked. Worth asking Mustafa if he's seen anything like this before, or
knows whether Ivan's code was ever actually exercised against this
particular robot.

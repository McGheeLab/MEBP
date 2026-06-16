# Print Execution Logs (`logs/prints/`)

One JSONL file per print run, written by
`SupportClasses/PrintExecutionLogger.py` (v7.5.x). Every print — Quick Print
(discrete mode), standard Printing mode (hybrid plan), and trajectory
playback — produces `print_YYYYMMDD_HHMMSS_<job>.jsonl` here automatically.
Disable with env var `MEBP_PRINT_LOG=0`.

These logs exist so that a debugging session (human or Claude) can reconstruct
**what the software commanded vs. what the hardware actually did**, with
timestamps, without needing the hardware present.

## Reading a log

Each line is one JSON event. Common fields:

| Field | Meaning |
|-------|---------|
| `t`   | Seconds since job start (monotonic clock) |
| `ev`  | Event type (below) |

Frames: `*_mm` XY values are **zero-ref mm** (what the print job commands);
`abs_*_um` are **absolute Prior stage µm** (`zero-ref mm × 1000 + zero_position`).
The `job_start` manifest records `zero_position` and `safety_limits` so the
conversion can be replayed offline. Z values are zero-ref mm in the raw motor
frame (on ME3B V1 the needle DESCENDS as raw Z increases).

## Event types

- **`job_start`** — manifest: `settings` (full PrintSettings), `zero_position`,
  `safety_limits`, `connection` (xy/zp/simulated), and `command_plan` — the
  full discrete command list with XY targets and per-path stats
  (`n_points`, `bbox_mm`, `path_len_mm`, `seg_mm_min/max/mean`). Diff this
  *plan* against the *actual* events below.
- **`command_start` / `command_end`** — each discrete `PrintCommand`
  (`i`, `total`, `type`, `label`, `duration_s`).
- **`path_start`** — a PRINT_PATH begins: `n_points`, `pump`,
  `flow_rate_uL_s`, `speed_mm_s`, path stats.
- **`path_segment`** — one segment of a PRINT_PATH: target (`x_mm`, `y_mm`,
  `abs_x_um`, `abs_y_um`), `seg_mm`, `mv_s` (planned move time), `slp_s`
  (actual sleep), `vol_uL` (pump volume commanded; `vol_dropped: true` means
  the volume was below the 0.001 µL send-threshold and was **skipped**), and
  `drift_s` — how far the command loop's wall clock has run ahead of its own
  sleep schedule (accumulated serial/computation overhead). `clamped`,
  `clamp_dx_um`, `clamp_dy_um` appear when the safety envelope altered the
  target.
- **`path_end`** — `wall_s` vs `planned_s`. **Note:** this measures the
  *command loop* only; with open-loop pacing the stage may still be
  physically tracing the path. Compare with subsequent `sample` events.
- **`xy_cmd`** — a non-path XY move command (`context`: `travel`, `home`,
  `path_start`, `blocking_travel`) with clamp check.
- **`xy_arrival`** — result of a blocking arrival wait (`ok`, `duration_s`).
- **`settle_wait`** — closed-loop settle result (`ok`, `reason`
  `timeout|aborted`, `duration_s`, `final_err_um`). A timeout **does not stop
  the print** — it silently continues; look here first for desync bugs.
- **`z_move`** — Z command (`context`: `move_z`, `travel_up`, `travel_down`,
  `blocking`; blocking moves include `ok` + `duration_s`).
- **`extrude`** — pump command (`pump`, `vol_uL`, `rate_uL_s`, `context`).
- **`speed_set`** — XY stage speed change (`mm_s`, `context`).
- **`traj_start` / `traj_wp` / `traj_end`** — trajectory-mode playback;
  `traj_wp` is logged every 25th waypoint or whenever the executor is
  >0.25 s late (`late_s`); `traj_end` compares `wall_s` to `plan_s`.
- **`plan_step`** — hybrid-mode plan step boundary (`type` = PRINT/WASTE/…).
- **`sample`** — background sampler (default 5 Hz, cached poller positions —
  zero extra serial traffic): actual `x_um`/`y_um` (absolute stage µm),
  `z`/`p1`/`p2`/`p3` (logical mm), and `lag_um` — distance between the actual
  position and the most recently commanded XY target. **This is the ground
  truth for "is the stage keeping up with the commands".**
- **`state`**, **`abort_requested`**, **`error`** (with traceback),
  **`job_end`** (`status`, `wall_s`).

## How to diagnose the two known bug patterns

1. **Print ends early / drives to 0,0** — find the last `path_segment`, then
   the `xy_cmd` with `context: "home"`. If `sample.lag_um` is large (≫ the
   settle tolerance) when the home command fires, the stage was still tracing
   the path when the job's final HOME_XY preempted it.
2. **Jumpy / lengthening moves near path end** — plot `sample.lag_um` over
   `t` during the path: steadily growing lag = open-loop pacing falling
   behind (stage slower than `slp_s` schedule). Also check `drift_s` growth
   in `path_segment` (loop overhead) and `seg_mm` (geometry spacing).

Quick triage in Python:

```python
import json, pathlib
events = [json.loads(l) for l in open(sorted(pathlib.Path('logs/prints').glob('*.jsonl'))[-1], encoding='utf-8')]
lags = [(e['t'], e.get('lag_um')) for e in events if e['ev'] == 'sample']
```

# MEBP v7.5.x — XY↔ZP Print Timing Calibration workflow

## Objective

Empirically MEASURE the XY↔ZP sync error in the discrete print path so the
print pacing can be corrected. After the ZP feedrate root-cause fix + the
end-of-path XY drain, the open question (raised by the user: *"will the pump
stop before the XY stage finishes its travel? if so we won't be printing
anything"*) is real: the pump (Marlin, bounded by the M400 barrier) tracks the
commands, while the XY (Prior) is streamed open-loop and lags — so the pump
finishes the path's volume before the needle finishes tracing it and the tail
prints dry. The magnitude depends on the real Prior accel/queue behavior, which
is uncertain on paper — so we measure it, over long runs, and build progressively
better estimates.

A multi-agent analysis workflow (`print-pump-xy-sync`, 5 agents) confirmed the
mechanism and recommended a lookahead-gated pacing fix; this calibration tool
provides the real-hardware numbers that fix needs (and validates it).

## What it does

New **Workflows → XY↔ZP Timing Calibration** tile. On a worker thread it drives
a looping circle path around the plate centre using the **exact pacing of the
real print path** — same per-segment `time.sleep` estimate, the same M400 pump
barrier every 8 segments, position poller + port-health watchdog suspended like
a real `PRINT_PATH` — for a configurable duration (**1 min / 5 min** presets),
while sampling the **actual** XY position to quantify the lag and its buildup.

The needle stays **retracted at the Safe Z the entire run** — this is a pure
motion-timing measurement, never a descent toward the plate.

Outputs (live grid + event log + saved estimate + per-sample JSONL):
- **Lag (mm)** = commanded arc − physically-traced arc = how far AHEAD the
  pump's deposition is of the needle (the dry-tail length).
- **Lag (s)** = lag ÷ speed = how long the pump finishes before the needle.
- **Lag rate (mm/min)** = how fast the sync error builds up.
- **Effective speed (mm/s)** = the speed the stage actually sustains (< commanded).
- **Correction ×** = commanded ÷ effective = the factor to slow the print loop's
  command-issue rate so the pump stays locked to the needle.
- **End drain (s / mm)** = measured time/distance for the needle to reach the
  final point AFTER the last command — the direct dry-tail duration.

## Files

| File | Change |
|------|--------|
| `SupportClasses/PrintTimingCalibrationStore.py` | **New.** Per-speed sync model persisted to `config/hardware/print_timing_calibration.json` (atomic write). `update()` merges each run via a run-count-weighted running mean (so estimates get "better and better"); stores `effective_speed_mm_s`, `correction_factor`, `lag_rate_mm_per_s`, `seg_len_mm`, `runs`, `samples`. `get_store()` singleton. |
| `gui/pages/workflows/timing_calibration_workflow.py` | **New.** `TimingCalibrationWorkflowPage` — config (duration + 1/5 min presets, speed, radius, segment length, optional pump flow, sample interval), worker thread that replicates the real print pacing/barriers/suspends and samples actual XY, live monitor grid + event log, refine-into-store + JSONL to `logs/timing/`. Mirrors the Stress Test page structure (bridge signals, MainWindow hooks, StandardJogContextPanel). Safety gate: XY+ZP connected + Safe Z; always retract in `finally`. |
| `gui/pages/workflows/workflow_picker.py` | New `timing_calibration` tile (⏱️, enabled). |
| `gui/pages/workflows_mode.py` | Import + wire the new page (`elif tile.workflow_id == "timing_calibration"`). |
| `tests/test_v75x_print_timing_calibration.py` | **New (12):** store correction math + cross-run refinement + persistence; tile registration; offscreen build; safety gate (no Safe Z / not connected); worker integration (completes, retracts, persists, streams pump when flow>0, honors stop). |

## Testing Notes

- `python -m unittest tests.test_v75x_print_timing_calibration` → 12 OK.
- Stress-test + quick-print workflow regressions → 37 OK; `workflows_mode`
  imports cleanly with the new tile.
- **Real-HW (the point), on ME3B V1:**
  1. Run at your real print speed for **1 min**, then **5 min**. Compare the
     **Lag (mm/s)** — if the 5-min lag ≈ 5× the 1-min lag, the error accumulates
     linearly (the Prior queues); if it plateaus, the lag is bounded (the Prior
     re-targets). Either way the **Lag (s)** at the end is the dry-tail you'd
     see, and **Correction ×** is what to slow the print pacing by.
  2. Re-run a few times — the stored `correction_factor` refines toward a stable
     value. Inspect `config/hardware/print_timing_calibration.json` and the
     per-sample `logs/timing/timing_*.jsonl`.

## Rev 2 — optical (camera) ground truth + FPS adequacy

Operator bench note: *"it said done, but the stage was still moving in a circle."*
Root cause: the Prior's position query (`P`) can report the **target**, not the
live encoder position, so the position-based drain detection saw "arrived" while
the stage was still physically circling. So position is NOT trustworthy for the
stop time — the **camera is**.

Added (`timing_calibration_workflow.py`):
- `_OpticalMotionTracker` — frame-differences the **microscope** feed
  (downscaled grayscale, `cv2.absdiff` mean). Moving stage → high diff; stopped
  → camera-noise floor. The worker measures the stationary **noise floor** at
  the start (stage at rest) → `motion_threshold = max(3×noise, 1.5)`.
- During the run it logs live "▶ MOVING / ■ still" + tracks the moving peak.
- **Buffer drain (optical):** after the last command, the drain ends when the
  frame-diff stays below threshold for a 0.4 s window — `optical_drain =
  (now − last_command) − window`. This is the trustworthy
  *controller-said-done → physically-stopped* delay (the operator's symptom,
  quantified). Falls back to position arrival if the camera is unavailable or
  the view has too little texture (`motion_peak < threshold` → logged).
- **Camera FPS / resolution:** reads the CameraWidget frame counter
  (`frame_count_value`) at loop start/end → true sensor FPS, logs
  `±(1000/fps) ms` resolution, and **warns when the measured drain is within
  ~2 frame-intervals** (can't be resolved — raise FPS / run slower-larger).
- Microscope camera auto-started on the GUI thread in `_on_start` (if optical
  enabled), stopped in `_on_finished`; new `optical` config (checkbox, default
  on when cv2 present). `PrintTimingCalibrationStore.update(buffer_drain_s=…)`
  merges the optical drain (run-weighted) into the per-speed entry.

Tests still 12/12 (worker degrades to position-only with no camera_manager).

## Rev 3 — direction-change ("turn-lag") sweep over distance × speed

Operator direction: *"do timing challenges like a star shape, see the optical-
flow direction-change timing vs the controller's reported timing, as a function
of distance and XY speed; ensure the camera frame rate resolves it."* And:
*"process the optical flow in a separate continuous processor; compare it to the
expected commanded direction; the two diverge over the run — that divergence is
what we calibrate."*

A direction change is a cleaner, repeatable signal than the end-of-run stop, and
gives one lag sample per turn → robust stats fast. Added a **second mode** to the
tile (a "Mode" combo: *Drain (circle)* vs *Turn-lag sweep*).

- **`SupportClasses/TurnLagAnalyzer.py`** (new, pure + unit-tested):
  - `make_shape("line"|"square"|"star", leg_len_mm)` — challenge vertices.
    Line = 180° reversals, square = 90° turns, star ({5/2}) = sharp turns.
  - `analyze_turn_lags(flow_samples, turn_times, leg_time)` — given the optical-
    flow vector series and the commanded turn instants, returns each turn's lag
    by learning the leg's flow direction empirically (so NO µm/px or rotation
    calibration is needed — only the relative direction pattern matters) and
    finding when the flow swings from the old leg's direction to the next.
  - `analyze_divergence(...)` — per-turn lag vs WHEN it was commanded (the
    divergence curve) + a linear `divergence_rate_s_per_s` (how fast the
    measured/expected directions drift apart) + `final_lag_s`.
  - `parse_float_list("2,5,10")` for the sweep inputs.
- **`_OpticalMotionTracker.flow()`** — optical-flow VECTOR via
  `cv2.phaseCorrelate` (Hann-windowed) between consecutive frames.
- **`_FlowProcessor(threading.Thread)`** — the continuous processor the operator
  asked for: computes flow at frame rate on its own thread into a timestamped
  buffer (`clear()` per combo, `snapshot()` to analyze); the streaming thread
  only records the commanded turn instants.
- **Sweep worker `_run_turn_sweep`** — iterates the grid (selected shapes ×
  speeds × leg-lengths); per combo: drive the shape (each leg issue = a
  commanded direction change), snapshot the flow, `analyze_divergence`, measure
  **camera FPS** for that combo and **warn if < 4 frames/leg** (the operator's
  frame-rate-resolution prerequisite), and refine
  `PrintTimingCalibrationStore.update_turn(shape, speed, distance, …)`
  (new `by_turn` section, run-weighted). Per-combo + per-turn detail to
  `logs/timing/turnlag_*.jsonl`. Needle retracted throughout; camera REQUIRED.

Tests: `tests/test_v75x_turn_lag_analyzer.py` (11 — shapes, parser, synthetic-
flow lag recovery, divergence) + `tests/test_v75x_print_timing_calibration.py`
extended to 17 (store `by_turn` refinement, mode toggle, sweep-worker
orchestration with a fake stage+flow). All 28 green. (Fixed a noise-floor bug in
the analyzer that used `hypot(timestamp, dx)` instead of `hypot(dx, dy)`.)

## Rev 4 — real-time visualization (live feed + flow arrow + direction plot)

Operator direction: *"the continuous time-tracked optical flow is better (syncs
in time, not limited by frame-registration speed). I want to see it in real
time: the live camera feed with an optical-flow direction ARROW overlaid at
centre; a PLOT over time of the measured optical-flow direction AND the
commanded XY direction; and the calibration result afterwards."*

Added to `timing_calibration_workflow.py`:
- **Live microscope feed** (`CameraFeedView`) on the page, in a horizontal
  splitter beside the monitor (`_build_main_area`); started on `showEvent` /
  run start, stopped on `hideEvent` (when idle). The **optical-flow arrow** is
  drawn at the image centre via `CameraFeedView.set_overlay_vector` — a
  fixed-length unit vector so the *direction* reads clearly regardless of the
  (small) pixel magnitude; cleared on finish.
- **`_DirectionPlot`** — a custom-painted rolling strip-chart (no matplotlib;
  smooth at the flow rate) plotting two angle-vs-time traces: **measured flow**
  (blue) and **commanded direction** (peach). They coincide at the start and
  drift apart over the run — the divergence, made visible. Angle-wrap aware
  (breaks the line across ±180° jumps), windowed (default 20 s).
- **Real-time data path:** `_FlowProcessor` gained an `on_sample` callback →
  the page emits `_TimingBridge.flow_sample` (throttled ~30 Hz) from the
  processor thread (queued to the GUI); the streaming thread emits
  `cmd_dir` at each leg issue (turn-lag) / per segment (drain = circle
  tangent). Slots `_on_flow_sample` (plot + arrow) and `_on_cmd_dir` (plot)
  share one clock (`_live_t0`, reset per run) so the two traces align in time.
- **Result afterwards:** the plot is NOT cleared on finish (it freezes showing
  the full divergence trace); the banner shows the headline and the event log
  holds the per-combo table — so the result is visible after the run.

Tests: `test_v75x_print_timing_calibration.py` extended to 21 (`_DirectionPlot`
buffering/window, flow-arrow unit-vector, commanded-angle, shared-clock). All
green; workflow regressions green.

## Rev 5 — APPLY the calibration to printing (automatic, with a toggle)

Operator question: *"after we run the calibration, does it apply automatically
or do I confirm/set something?"* — answered + then chosen: **automatic, with a
single toggle (default on)**. Until now the calibration only measured + stored;
nothing consumed it.

- **`PrintTimingCalibrationStore`** gains a single persisted `enabled` flag
  (`is_enabled()`/`set_enabled()`, default True) and `correction_for(speed)` →
  the stored `correction_factor` (≥1) for that speed bucket, else None.
- **`PrintManager._execute_print_path`** now, at path start, reads
  `get_store()` (guarded import): if enabled and a `correction_factor` exists
  for the print speed, it multiplies the per-segment XY transit budget
  (`move_time * _pace_corr`) — pacing the command-issue rate down to the stage's
  MEASURED effective speed so the commands (and the M400-bounded pump) stay
  locked to the physical needle and the tail isn't dragged dry. `pump_move_s`
  stays a floor. No-op (×1) when disabled or uncalibrated for that speed. Logged
  (`path_start timing_correction=`, INFO line). Discrete path only.
- **Toggle UI:** an "Apply calibration to prints" checkbox on the calibration
  tile's run row, bound to `store.set_enabled` / reflecting `is_enabled()`.

So the workflow is now: run the sweep a few times (refines the per-speed
`correction_factor`) → with the toggle on, every discrete print at a calibrated
speed automatically paces to the measured effective speed. No per-print
confirmation. Turn the checkbox off to pace as before.

Tests: `test_v75x_print_path_planner_barrier.py` +3 (correction stretches the
pacing; disabled toggle skips it; uncalibrated speed → unchanged) and
`test_v75x_print_timing_calibration.py` +1 (enabled toggle + `correction_for`
persistence). Full `test_v75x*` = 848 (1 pre-existing unrelated CV failure).
**Needs real-HW verification on ME3B V1.**

## Rev 6 — signal quality, phase delay + corrected overlay, verify rerun

Four operator issues from the first real runs:

1. **Noisy flow / picks up camera noise** → `_OpticalMotionTracker._gray` now
   **Gaussian-blurs** the downscaled frame before diff/phase-correlation (kills
   per-pixel sensor noise at zero temporal lag, so it can't bias the phase
   delay). The LIVE display additionally gates sub-noise samples
   (`_FLOW_DISPLAY_FLOOR`) and EMA-smooths the arrow/plotted angle — DISPLAY
   ONLY (the raw recorded samples used for the calibration are untouched, so no
   lag bias).
2. **Flow stopped when the commanded motion finished** → it must keep measuring
   the drain (that IS the lag). New `_FlowProcessor.recent_max_mag()` +
   `_wait_until_flat()`: after the commanded legs the sweep keeps the flow
   recording until the image is physically still (recent magnitude < a baseline-
   derived threshold for a window) or times out — capturing the last turn's
   post-command drain. Drain mode also keeps emitting flow through its drain.
3. **Post-process + phase delay + corrected overlay** → after the run the plot
   `freeze()`s (full run stays visible), `denoise_measured()` (zero-phase
   centered moving-average — filters noise without shifting timing), and draws a
   THIRD green trace `build_corrected(phase_delay)` = the commanded direction
   shifted by the measured phase delay (should land on the measured trace). The
   worker emits a `result` (median per-turn lag = phase delay, median divergence
   rate) → `_on_result` builds the overlay; the headline is logged.
4. **Verify rerun** → a "Verify (rerun w/ correction)" button reruns the
   turn-lag challenge with `_run_turn_sweep(corrected=True)`: each leg's pacing
   is stretched by the stored `correction_factor` (exactly what prints do), and
   the residual **divergence rate should drop to ≈ 0** (the calibration removed
   the growing phase lag). A verify pass does NOT write back to the store.

Decision: the "phase delay" reported is the median per-turn lag (frame-
independent, already robust); the corrected overlay = commanded + that delay.
The verify confirms the DRIFT (growing divergence) is removed — the constant
per-move latency remains (and is expected); what the pacing correction fixes is
the *accumulation*, which is what desyncs the pump from the needle over a path.

Tests: `test_v75x_print_timing_calibration.py` extended to 26 (display gate,
plot denoise/corrected/freeze, `recent_max_mag`, verify-emits-result-and-doesn't-
write); analyzer + print-path + workflow regressions green.

## Rev 7 — PIVOT: segment settle-delay sweep (optical flow REMOVED)

Operator verdict on the optical-flow direction approach (Rev 3–6): *"this is a
very big fail."* The phase-correlation flow was noisy and over-engineered.
Replaced with a simpler, robust method (operator-specified):

  > Issue a move command in a line, watch for the camera frames to STOP
  > changing, log the delay. Issue two line commands in a row, then three. Don't
  > use optical flow — just an algorithm to see if the frames look the same or
  > not. The build-up of this delay per line segment gives the controller's
  > phase lag to the issued commands over time.

Implementation (full rewrite of `gui/pages/workflows/timing_calibration_workflow.py`):
- **`_FrameMotion`** — frame-DIFFERENCE only (mean abs diff of consecutive
  downscaled, blurred grayscale frames). "Are the frames the same?" → moving vs.
  still. No optical flow, no direction. (`SupportClasses/TurnLagAnalyzer.py` and
  its test were DELETED.)
- **Sweep**: for N = 1..max, move to the line start + wait still, stream N
  collinear segment moves back-to-back (paced like a print), then time how long
  after the LAST command the frames stop changing → `delay[N]`. Repeats averaged.
- **Result**: least-squares fit of `delay` vs `N` → **slope = per-segment phase
  lag**, intercept = base settle. The slope is the timing that must be allocated
  per segment to keep moves synced with the ZP stage (a single jog-speed scale
  cannot fix it — the operator's point). Stored per (speed, seg) in the store's
  new `by_phase` section (`update_phase`/`get_phase`, run-weighted).
- **Live view**: camera feed (frame-diff still detection) + a `_MotionStrip`
  (frame-change vs time with command/still markers — watch the frames go flat) +
  a `_DelayPlot` (delay-vs-N with the fitted phase-lag line). Needle retracted
  throughout; microscope required.
- **Reverted the Rev 5 print-path auto-correction** (the multiplicative
  `correction_factor` pacing) — the operator rejected jog-speed-style correction.
  The new phase model is **measurement only** for now; the principled fix it
  implies is an ADDITIVE per-segment pacing (add `slope` per segment), to be
  wired once the operator has reviewed the numbers. Store keeps the legacy
  `by_speed`/`correction_for` methods (unused) to avoid churn.

Tests: `tests/test_v75x_print_timing_calibration.py` rewritten (15: phase-model
store, `_fit_line`, registration/build/gate, plot widgets, worker sweep against
a backlog-modelling fake stage → positive slope + stored). Removed
`TestTimingCorrectionPacing` (the reverted print correction). Full `test_v75x*`
= 827 (1 pre-existing unrelated CV failure).

## Rev 8 — detection fix: reported 0 ms but the stage clearly moved >1 s

The first real run of the settle sweep read **0 ms delay** while the operator
saw a >1 s settle — the frame-difference detector was declaring "still"
immediately. Three causes, all fixed:

1. **Threshold could sit ABOVE the motion.** The old threshold was a fixed
   multiple of an at-rest baseline (`max(rest×2.5, 0.6)`); on a low-contrast/
   noisy scene that can exceed the actual motion diff → always "still". Replaced
   with `_calibrate_threshold`: settle, measure the rest **floor** (median), do
   a real one-segment **test move** and measure the motion **peak**, then set
   `threshold = floor + 0.3·(peak − floor)`. If `peak − floor` isn't clearly
   above the floor, **abort with guidance** ("camera not detecting stage motion
   — check focus / texture / lighting / FPS") instead of reporting a bogus 0 ms.
2. **Stillness accepted with no motion seen.** `_watch_until_still` now requires
   MOTION to be observed (change ≥ threshold) BEFORE it accepts a sustained
   below-threshold window — so an undetected move surfaces as a timeout ("no
   stop detected"), never a false 0 ms. Split out `_wait_quiet` (settle
   confirmation, no motion-first) for the pre-measurement wait.
3. **Duplicate frames read as 0.** Polling faster than the camera FPS returns
   the SAME frame → diff exactly 0 → false "still". `_FrameMotion.metric` now
   skips duplicates (`np.array_equal` vs the previous frame → return None, keep
   `_prev`), so every measured value is a real inter-frame difference and a
   frozen/dead camera surfaces as "no frames" rather than 0 ms.

Also surfaced the live numbers: the monitor now shows **Frame change** (live)
and **Still threshold** so the operator can see the signal vs. the threshold;
the `_MotionStrip` already draws the threshold line + command/still markers.

Tests: `tests/test_v75x_print_timing_calibration.py` (19 — added duplicate-skip,
motion-first stillness, quiet-wait, and the worker sweep still fits a positive
slope against a backlog-modelling fake). Workflow/print regressions green.

## Rev 9 — ROOT CAUSE of the XY being ~2.8× too slow (software, not hardware)

First real sweep (0.5 mm @ 1 mm/s) showed the settle delay growing ~875 ms PER
SEGMENT (N=1 1.3 s → N=8 9.2 s). Operator: *"if it were hardware the delay
wouldn't increase monotonically; the mm/s conversion must be wrong."* Correct.
A 5-agent workflow (`xy-timing-rootcause`) confirmed two software causes:

1. **mm/s→SMS-% conversion uses an unverified, never-loaded max speed.**
   `XYStage._protocol_max_speed_um_s` was assigned ONLY in the simulator branch,
   so on real hardware `set_speed_mm_s(1.0)` fell back to a hardcoded 50000 µm/s
   assumption and sent `SMS,2` (2%). The stage then cruises at ~2% of its TRUE
   top speed, not 1 mm/s. The loop, meanwhile, paces at `seg_len/speed` = 500 ms
   while each move actually takes ~1375 ms → the Prior backlogs the streamed `G`
   commands → the delay grows ~linearly with N (the smoking gun: a hardware
   limit or a target-replacing controller would be constant/saturate, not grow).
2. **Acceleration is never set** → short (0.5 mm) point-to-point moves are
   accel-dominated; an unset/low Prior accel adds large ramp time on top.

Fixes (`SupportClasses/XYStage.py`, `PrintManager.py`, `timing_calibration_workflow.py`):
- New `XYStage._max_speed_um_s()` resolves the top speed in priority order:
  measured override (`set_max_speed_um_s`, new) → sim value → protocol
  `parameters.max_speed` → 50000 (logged once as an UNVERIFIED guess).
  `set_speed_mm_s`/`set_velocity` now use it (and `round`, not `int`-truncate).
  This fixes the never-loaded bug and makes the max **correctable per machine**.
- `_set_xy_speed_for_print` + the timing tool now call `set_acceleration(80)`
  before moving (and the duplicated hardcoded-50000 fallback was removed —
  routes through `set_velocity(µm/s)`).

**Honest caveat / load-bearing unknown:** the conversion now *resolves* the max
from the protocol, but proscan_iii.json's `max_speed` is still 50000 — an
unverified guess. So the conversion fix is correct PLUMBING, but the value won't
change until the operator supplies the **real** 100%-SMS top speed (via
`set_max_speed_um_s` / a measurement). Therefore the **acceleration** fix is what
may help immediately (if short moves were accel-starved); the *speed* is only
corrected once the true max is set. Next step (offered): an automated top-speed
measurement — sweep move distance at SMS,100, fit time-vs-distance (slope =
1/top_speed), store as the override — to pin the real max and make 1 mm/s mean
1 mm/s.

Tests: `tests/test_v75x_xy_speed_conversion.py` (8 — max resolution priority,
mm/s→SMS rounding, override). XY/print/timing/workflow regressions (88) green.

## Rev 10 — top-speed measurement (pins the true max) + selectable start

Closes the Rev 9 load-bearing unknown (the stage's TRUE 100%-SMS speed) and adds
a selectable start location.

- **"Measure top speed" button** → `_run_top_speed`: at FULL speed (`SMS,100`)
  it sweeps single-move DISTANCES (5 points up to a configurable max, default
  10 mm) along +X from the start, times each to stillness (reuses `_measure_n`
  with N=1 + the frame-difference detector + `_calibrate_threshold`), and fits
  time-vs-distance. The slope = 1/top-speed (the constant accel/decel +
  still-detection overhead falls into the intercept, so it cancels). The result
  is **stored** (`PrintTimingCalibrationStore.set/get_xy_max_speed_um_s`) and
  **applied live** (`XYStage.set_max_speed_um_s`), so the mm/s↔SMS-% conversion
  becomes correct (1 mm/s really commands 1 mm/s). `StageController.connect_stages`
  re-applies the stored value on every real XY connect, so prints/jog use it
  across restarts without opening the tool. The `_DelayPlot` is parametrised
  (headline/x/y labels) to render either "phase lag/segment" or
  "top speed mm/s" + a JSONL to `logs/timing/topspeed_*.jsonl`.
- **Selectable start location:** "Use current position" captures the live XY
  (jog there on the context panel while watching the feed) as the sweep start;
  "Plate centre" resets. `_resolve_start_mm()` (override → plate centre) feeds
  both workers; the start note reminds to leave envelope room along +X.
- Recommended flow: **Measure top speed → re-run the settle sweep** (the
  per-segment lag should now be small) → then the segment lag reflects real
  queueing/accel, not the speed-conversion bug.

Tests: `tests/test_v75x_print_timing_calibration.py` (+ start-location capture/
reset, top-speed measure→store→apply against the distance-dependent fake → ~20
mm/s recovered). Full `test_v75x*` = 882 (1 pre-existing unrelated CV failure).

## Issues & Decisions

- **Why a circle + arc-length lag (not Euclidean-to-target):** Euclidean
  distance to the last commanded point saturates at the circle diameter when the
  stage is far behind. Cumulative *arc* (commanded − traced) does not saturate
  and is robust whether the Prior queues (lag grows) or re-targets/corner-cuts
  (traced < commanded → lag still grows). The end-drain measurement is the
  unambiguous cross-check.
- **Pump flow default 0:** the XY lag IS the pump-ahead distance because the
  pump is bounded by the M400 barrier and tracks the commands (per the analysis),
  so XY-only measurement is valid. Flow > 0 streams the pump into air (needle
  retracted) so the serial load matches a real print; it dispenses
  `flow × duration` µL (noted in the log).
- **Next step (separate change):** apply the measured `correction_factor` to the
  print loop — the recommended lookahead-gated XY pacing (analysis Option A):
  advance the loop only once XY has physically closed to within ~one segment of
  the prior target, so the pump can never finish while XY still has travel left.
  This calibration provides the numbers to tune it and to verify it afterward.

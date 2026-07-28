# MEBP v7.5.x — Quick Print closed-loop velocity following (+ confirmed-segment fallback)

## Objective

Fix "the XY trajectory is very wrong when it lays down ink" in Quick Print. Ink
pickup / washing were correct; only the ink-laydown pass was distorted. The
operator wants the stage to **continuously update its velocity from polled
position** to correct itself — not stop-and-go.

## Root cause (from the operator's own run)

Diagnosed from `logs/prints/print_20260727_171512_..._B6.jsonl`:

- PRINT_PATH is streamed **open-loop** — each segment's XY move is fire-and-forget
  (`ctrl.move_xy_absolute`), paced only by `time.sleep(seg_len / print_speed)`.
- The Prior stage physically runs **~2.5× slower** than commanded
  (`set_speed_mm_s 2.5 mm/s = SMS 11%`, `max=22282 µm/s`; optimistic SMS%↔speed
  cal + short ~0.48 mm accel-dominated segments), so the physical stage falls
  PROGRESSIVELY behind the commanded stream: sampled `lag_um` grew
  **0 → 6144 (median) → 10736 (max) µm** across a ~10 mm print, and the
  end-of-path drain **timed out after 30 s still 2.09 mm short**.
- The pump extrudes on the fast software schedule while XY lags → smeared pattern.

Pickup/wash use point-to-point `safe_travel_to` moves that WAIT for arrival, so a
slow stage only makes them slow, not misplaced. PRINT_PATH is the only open-loop
streamed path, so a slow stage makes it *wrong*.

## Fix

Two executor modes were added to `PrintManager._execute_print_path`, both gated by
new `PrintSettings` flags (both default False → legacy open-loop unchanged):

### 1. `velocity_follow` — CLOSED-LOOP velocity following (Quick Print default)

`PrintManager._execute_print_path_velocity` runs a real-time control loop
(`_VEL_CONTROL_HZ ≈ 25 Hz`) that, every tick:

1. polls the stage's ACTUAL position (`get_xy_position(cached=False)` — the
   background poller is already suspended for PRINT_PATH),
2. projects it onto the toolpath → real **arc-length** progress `s`
   (`project_on_polyline`),
3. deposits pump volume in proportion to the REAL `Δs` travelled
   (`vol_per_mm = flow_rate_uL_s / print_speed`), accumulated + emitted
   non-blocking (no M400 in the loop — that would stall the moving stage),
4. places a **carrot** a fixed lookahead (`_VEL_LOOKAHEAD_MM`) ahead of `s` and
   commands a **velocity vector** toward it (`send_velocity_xy` → Prior `VS,vx,vy`
   µm/s) at the print speed, ramping down over the last `_VEL_DECEL_MM`.

Because the carrot is tied to the stage's real progress (pure pursuit,
arc-length parametrized), it can **never run ahead of a slow stage** — the
open-loop failure mode — and position feedback pulls the stage back onto the path
each tick. The pump tracks real distance so the bead is correct at any speed.
The stage top speed (SMS) is set to max + a brisk accel so `VS` can reach the
commanded velocity; velocity is clamped to `safety_limits.max_xy_speed`.

**Safety:** always stops the stage (`VS 0,0`) on every exit (abort / pause /
ZP-drop / stale-position / runaway / completion / exception, via `finally`).
A sustained large cross-track error (`> _VEL_MAX_CROSS_TRACK_MM` for
`_VEL_RUNAWAY_TICKS` ticks — e.g. a `VS` sign inversion driving the stage the
wrong way) trips a **runaway guard** → `RuntimeError` → the outer `_execute_loop`
sets ERROR and retracts to safe Z. Requires a continuous-velocity command; on a
stage without one (Ludl HLC) it falls back to the discrete path.

New pure, unit-tested helpers: `polyline_arclength`, `point_at_arclength`,
`project_on_polyline`.

### 2. `confirm_each_segment` — stop-and-go fallback (not the Quick Print default)

Landed first, retained as a general executor fallback: after each segment's XY
move, wait for physical arrival (`_wait_for_xy_settle`, ± `_SEGMENT_SETTLE_TOL_UM`)
and drain the pump board (M400). Correct geometry but stop-start (slower, and the
operator preferred continuous motion) — so Quick Print uses `velocity_follow`
instead, not this.

### 3. Timing-Calibration "Run sweep" button greyed out

`TimingCalibrationWorkflowPage._update_button_state` gates Start/Measure on live
XY **and** ZP connection, but was only called on construction / calibration-data
pushes — never on `showEvent` or the periodic `on_status_update`. So a connect
made after opening the page left the button stuck greyed. Fix: call
`_update_button_state()` in `showEvent` and `on_status_update` so it tracks live
connection state.

## Files Modified

- `SupportClasses/PrintManager.py` — module helpers `polyline_arclength` /
  `point_at_arclength` / `project_on_polyline`; `PrintSettings.velocity_follow` +
  `confirm_each_segment`; `_VEL_*` + `_SEGMENT_SETTLE_TOL_UM` constants;
  `_execute_print_path` dispatch; `_execute_print_path_velocity` + `_emit_pump`;
  confirmed-segment branch + path_start log field.
- `gui/pages/workflows/quick_print_workflow.py` — `_build_settings` sets
  `velocity_follow=True`.
- `gui/pages/workflows/timing_calibration_workflow.py` — refresh button state on
  `showEvent` + `on_status_update`.
- `tests/test_v75x_quick_print_velocity_follow.py` — new (11).
- `tests/test_v75x_quick_print_confirmed_segments.py` — confirmed-segment mode (7).

## Implementation Steps

- [x] `confirm_each_segment` executor mode + tests.
- [x] Arc-length helpers + `velocity_follow` executor (`_execute_print_path_velocity`).
- [x] Dispatch in `_execute_print_path`; Quick Print enables `velocity_follow`.
- [x] Runaway guard + always-stop-on-exit safety.
- [x] Timing-Calibration sweep-button refresh on show/tick.
- [x] Tests (velocity 11 + confirmed 7 + barrier 10 green; print/quick-print/
      timing/seam/pick-place/multi-ink/exec-log regression green, 158).

## Testing Notes

`python -m unittest tests.test_v75x_quick_print_velocity_follow` (incl. an
integrating fake stage that follows `VS`, deposits volume per real distance, and
trips the runaway guard) + regression pass. **Needs real-HW verification on
ME3B V1:**

- Run the same Quick Print — the deposited pattern should now be correct (no
  smear) and the stage should move continuously (not stop-and-go). The per-print
  JSONL `path_start` shows `mode: "velocity"`, and `vel_sample` events show
  `cross_um` staying small across the path.
- **Verify the `VS` direction first** on a low, safe path: if the stage flies off
  the path, the runaway guard aborts + retracts (a `VS` sign vs position-readout
  mismatch) — flip the sign convention in `_execute_print_path_velocity`'s `vx/vy`
  if so. This is the #1 thing to check on hardware.
- Tune `_VEL_LOOKAHEAD_MM` (smaller = tighter corners, larger = smoother) and
  `_VEL_CONTROL_HZ` to the stage's real `VS`/poll round-trip latency.
- The "Run sweep" button should now enable as soon as XY+ZP are connected.

## Addendum (2026-07-27) — spiral projection-snap runaway, fixed

First real-HW run tripped the runaway guard: `logs/prints/print_20260727_184045_
..._C5.jsonl`. NOT a sign inversion — the log proves it: tracking was excellent
(cross-track ~100 µm) for the first ~6.5 mm, the stage followed the curve, and
position responded in the commanded `VS` direction. Then `s` jumped **+4.8 mm in
one logged interval while the position moved only 0.2 mm** (6.506 → 11.313 mm),
after which cross-track blew up to 3.57 mm → abort.

Root cause: the toolpath is a **spiral** (M_Sketch — 59 mm path in a ~10 mm box =
concentric loops). The projection's forward search window was
`_VEL_LOOKAHEAD_MM + 4.0 = 4.6 mm` of arc length; an inner spiral loop's
circumference is *smaller* than that, so the projection **snapped to the next
loop** (which passes near the same (x,y)), teleporting the carrot to a distant
part of the path → the stage chased it → runaway. (It also caused a one-tick
pump over-deposit, since a 4.8 mm `Δs` dispenses in one shot.)

Fix: bound the projection's forward search AND the per-tick `s` advance to the
physically-plausible distance `print_speed × real_dt × _VEL_SNAP_GUARD`
(ceiling `_VEL_MAX_SNAP_MM`). This is far below one loop's circumference (loops
are a full revolution apart in arc length) yet well above the true per-tick
advance, so `s` progresses continuously and can never jump to a later loop; the
pump `Δs` is bounded too. Locked by `test_bounded_window_prevents_loop_snap`
(boustrophedon: large window snaps, bounded window doesn't) +
`test_completes_multiloop_spiral_without_snap` (shrinking spiral runs to the
centre). Tuning note: on the *innermost* loops the fixed `_VEL_LOOKAHEAD_MM`
(0.6 mm) is a large fraction of the circumference → some corner rounding; reduce
lookahead (or make it adaptive to local curvature) if the inner loops print too
round.

## Addendum 2 (2026-07-27) — "the old version worked on my other setup; why not here?" + Motion-mode selector

Operator pushback: the original open-loop path printed well on their other
machine before the recent changes. Investigation (git + the machine's own configs
+ logs):

- The open-loop path is correct **when the stage tracks the commanded speed**.
  This machine does **not** — it delivers only ~40 % of commanded print speed, and
  this is NOT a recent regression: `config/hardware/print_timing_calibration.json`
  (stamped **2026-06-18**) already records `commanded 5.0 mm/s → effective
  1.85 mm/s` (correction 2.71). Today's open-loop B6 log independently agrees
  (2.49 mm/s commanded → ~1.0 mm/s actual). So the ~2.5× shortfall predates the
  "last few days."
- Acceleration is fine: `proscan_iii.json acceleration_range = [1,100]`, and the
  print path sets `SAS,80` (80 %) — brisk, in-range.
- Quick Print anchors its speed % to a **measured top speed** (`22282 µm/s`,
  measured on this machine on long single moves) — the stage's *cruise* speed,
  which it does not reach on a print's short back-to-back segments. So the
  commanded speed is ~2.5× higher than the stage delivers during a print → the
  open-loop pacing runs ahead → smear. The other machine either lacks this stored
  value or its stage genuinely reaches its rated speed.
- The recent (uncommitted) changes mostly command the stage *more* correctly, not
  less; the smear is a property of this machine's speed calibration + short
  sketch segments, which the open-loop path is sensitive to and the other machine
  isn't.

Decision (operator: "do both, A/B on hardware"): Quick Print's settings popout
gains a **Motion mode** selector (`_motion_mode_combo` in the "Print" section) →
`_build_settings` maps it to the `PrintSettings` flags via `_motion_mode()`:
`open_loop` (default — the original streamed path, i.e. what worked on the other
setup), `velocity` (closed-loop follower), `confirm` (stop-and-go). So the same
print can be run each way on this machine and compared directly. **Default is now
`open_loop`** (velocity/confirm are opt-in), so "just hit Print" reproduces the
old behaviour. Persisted with the popout profile.

## Addendum 3 (2026-07-27) — camera-free timing calibration (stage-position detector)

Operator: the XY↔ZP timing calibrator solved the open-loop lag before (via the
correction it measures), but on this rig the microscope is a poor motion sensor —
"set up this calibrator a different way." Hunch: this stage is simply slower than
the other one.

Insight: the calibrator only uses the camera as a **still-vs-moving detector**.
The stage's OWN reported position is a far better one (rest ≈ 0–1 µm/poll, motion
≈ hundreds of µm/poll; immune to focus/texture/lighting). So:

- New `_EncoderMotion` — a drop-in for `_FrameMotion` (same `available`/`reset`/
  `metric`), where `metric()` = |Δposition| in µm since the last
  `get_xy_position(cached=False)` (the poller is suspended during a run, so
  queries are uncontended).
- A **Detector** selector on the workflow — **"Stage position (no camera)"
  (default)** vs "Microscope camera". `_make_tracker()` returns the chosen
  tracker; `_preflight` only requires the camera in camera mode. So both existing
  measurements — **"Measure top speed"** and the **settle sweep** — now run with
  no camera and feed the same store (`set_xy_max_speed_um_s` /
  `correction_for`) + the same `_apply_measured_xy_top_speed` fan-out.

This makes the operator's hunch directly testable: run **Measure top speed** (no
camera) — if the true top speed comes back well below the stored `22282 µm/s`,
that confirms the stage is slower than assumed, and storing the measured value
fixes the mm/s↔SMS conversion so a commanded print speed is real and the
open-loop path stops running ahead.

**Follow-up (2026-07-27) — the settle sweep couldn't get a measurement on the
encoder.** Top speed (big fast moves) worked, but the sweep (short, slow, streamed
segments) kept timing out. Cause: it reused the camera's **scene-relative
threshold calibration** (`_calibrate_threshold`) — on a short segment the
calibration's "peak" caught a single poll spanning the whole ~260 µm move and set
the still/moving threshold high (~78 µm), but during the actual sweep each poll
only advances ~40 µm, *below* threshold → "motion" is never detected →
`_watch_until_still` times out → no data. Fix: the stage-position detector reports
absolute µm, so it doesn't need scene calibration — new `_encoder_threshold`
measures only the at-rest jitter (no move — the move-peak is the fragile part) and
places the still threshold just above it (`max(STILL_UM, floor×3, floor+3)`), far
below any real per-poll motion. Both `_run` and `_run_top_speed` use it when the
tracker is `_EncoderMotion`. Tests: `tests/test_v75x_timing_encoder_detector.py`
(13, +threshold-above-jitter/below-motion + floor) + timing suite, 33 green.

Open follow-up (not yet wired): the settle sweep already stores a per-speed
`correction_factor`; wiring it into the open-loop `_execute_print_path` pacing
(pace each segment to the stage's measured effective speed) is the remaining lever
if fixing the top-speed anchor isn't enough on short segments — it was tried once
before and reverted, so revisit only with the camera-free measurement in hand.

## Addendum 4 (2026-07-28) — XY Printing Challenge (path-following compare + tune)

Operator: add a calibration that introduces **challenge shapes** and, for each of
the three print modes (open-loop, confirmed per-segment, velocity closed-loop),
**compares actual path to ideal path and tunes the parameters** for better path
following per mode. AskUserQuestion → **Both** (manual compare/adjust + auto-tune
sweep).

Built as a self-contained dialog (`gui/dialogs/xy_challenge_dialog.py`) launched
by an **"XY Printing Challenge…"** button on the timing workflow — isolated from
the existing page. Ground truth is the stage's own encoder (no camera).

- **Shapes** (`SupportClasses/XYChallenge.py`, pure/tested): Square (corners),
  Circle (curvature), Star (sharp reversals), Zigzag (repeated accel reversals),
  Spiral (dense loops) — each a polyline resampled to a chosen segment length.
- **Error metric** (`path_error`): min perpendicular distance from each recorded
  ACTUAL position to the ideal polyline → RMS + max + mean µm. Lower = better
  following.
- **Runner**: needle retracted to Safe Z, poller+watchdog suspended, always stops
  the stage (VS 0,0) on exit. Three XY-only drivers MIRROR PrintManager's
  executors so tuning transfers: `_drive_open_loop` (stream + paced sleep),
  `_drive_confirm` (move + wait-for-arrival), `_drive_velocity` (pure-pursuit VS,
  reusing PrintManager's `polyline_arclength`/`point_at_arclength`/
  `project_on_polyline` + the same per-tick snap bound + soft runaway guard).
  Each records the actual path from `get_xy_position(cached=False)`.
- **UI**: shape/size/segment/mode/speed + per-mode param spins; **Run** (one
  mode), **Compare all modes** (runs all three, logs each RMS/max, names the
  best), **Auto-tune mode** (sweeps the mode's primary parameter — open-loop
  `pace_correction`, confirm `settle_tol_um`, velocity `lookahead_mm` — over a
  grid, keeps the lowest-RMS value + saves it), **Save tuned params**. A
  `_PathOverlay` widget draws ideal (grey) + actual (per-mode colour) auto-fit,
  with the error readout.
- **Closed loop back to real prints**: tuned params persist to
  `PrintTimingCalibrationStore.set_mode_params("open_loop"|"confirm"|"velocity")`;
  PrintManager's executors read them from new `PrintSettings` fields
  (`pace_correction`, `segment_settle_tol_um`, `vel_lookahead_mm`/`vel_control_hz`/
  `vel_decel_mm` — the velocity class constants promoted to overridable settings,
  0/absent → the class defaults so legacy behaviour is byte-identical); Quick
  Print `_build_settings` stamps them from the store. So what you tune here is
  what prints. Open-loop pacing now also honours `pace_correction` (multiply the
  per-segment sleep) — the "pace to the stage's measured effective speed" lever
  that was flagged as an unwired follow-up, now driven by the challenge tuner.

**Needs real-HW verification on ME3B V1** (run each shape/mode; confirm the
velocity mode gives the lowest RMS, auto-tune converges a sensible lookahead,
tuned params carry into a real Quick Print). Tests:
`tests/test_v75x_xy_challenge.py` (15 — shapes/error, store round-trip, settings
plumbing + Quick-Print stamping, and the three drivers with fake teleport/VS
stages); regression across velocity/confirmed/barrier/timing/pick-place/
print-setup suites, 132 green.

## Issues & Decisions

- The operator explicitly rejected stop-and-go (`confirm_each_segment`) in favour
  of continuous velocity correction — so Quick Print uses the arc-length pure-
  pursuit velocity follower; `confirm_each_segment` stays as a general fallback.
- Reused the existing `send_velocity_xy` → `move_stage_at_velocity` (`VS`)
  primitive (already used by the Xbox jog) and the poller-suspended PRINT_PATH
  context — no new hardware plumbing.
- Chose arc-length (pure-pursuit) parametrization over the existing time-based
  `VelocityExecutor`/`MotionController` because a time-parametrized target still
  runs ahead of a slow stage; arc-length ties the target to real progress.
- Left the open-loop path as the default so Full Print / Print Setup and all
  existing tests are byte-identical.

---

## Addendum (2026-07-28) — comms-frequency checker + open-loop = feed-forward velocity streaming

Two operator requests, both about the velocity control scheme and its limits.

### (A) Communication-frequency checker (part of XY calibration)

Operator: "introduce a communication frequency checker to understand the ability
to communicate WHILE giving commands of motion." Distinct from the existing
`StageController.test_command_rate` (poll-only, no motion). The real closed-loop
control cadence — one `send_velocity_xy` + `get_xy_position(cached=False)` cycle
WHILE the stage is moving — is the `loop_period` that governs the velocity
follower's max stable speed (`v_max ≈ lookahead / (loop_period · safety)`), so it
belongs in the XY↔ZP Timing Calibration workflow next to top-speed / settle.

- **Backend `StageController.measure_control_loop_rate(iterations, speed_um_s,
  amplitude_um)`** — interleaves a velocity command + a fresh position read for N
  iterations while the stage oscillates within `±amplitude_um` of its start
  (velocity reversed at the bound → net displacement ~0). Probe speed clamped to
  `safety_limits.max_xy_speed`; SMS raised so the commanded `VS` actually moves;
  poller suspended; **always** stops (`VS 0,0`) + returns to start on exit. Returns
  `avg/min/max_period_ms`, `control_hz`, `avg_read_ms`, `avg_cmd_ms`, `moved`,
  `max_excursion_um` (or `{"error": …}`). Falls back on a no-VS controller (the
  command exercises whatever `send_velocity_xy` maps to, e.g. Ludl pulsed jog).
- **Store**: `PrintTimingCalibrationStore.get/set_control_loop_ms` (persists the
  measured period) + `stable_velocity_speed_mm_s(lookahead, safety=2)` = the max
  stable velocity-follow speed for the measured loop period.
- **UI**: a **"Check comms rate"** button on the timing workflow run row (gated on
  XY connected; needs Safe Z only to retract; no camera / no start location).
  Worker retracts the needle, runs the probe, logs
  `loop … ms/cycle → … Hz (read … + cmd …)`, persists the period, and reports the
  derived **max stable velocity-follow speed**.
- Tests: `tests/test_v75x_comms_frequency_check.py` (12).

### (B) Velocity-follower investigation → "back-and-forth over the same line"

Root cause = a **dead-time limit cycle**. The follower's real tick period is set
by serial latency (a non-cached read + a `VS` write + a 50 ms `_drain_ack`), so it
free-runs at ~5–10 Hz, not the nominal 25 Hz, and the `VS` velocity keeps running
the whole tick. When `speed × loop_period > lookahead` the stage overshoots the
carrot within one tick → reverses → limit cycle. It bites hardest **at the
endpoint / sharp reversals**: the decel ramp floors speed at 15 % of print_speed,
so near the end the stage still travels > the 40 µm arrival tolerance per tick and
dithers around the endpoint until the wall-time cap. Straight runs are fine
(carrot always ahead) — matching "pickup/wash/straight fine." **Not** a sign bug
(that would be a runaway, which the cross-track guard catches). The comms checker
(A) measures the loop_period that bounds the stable speed; the dead-time-aware
speed clamp / endpoint-termination fixes are the next step (proposed, not yet
landed).

### (C) Open-loop mode redefined: feed-forward velocity streaming (not point-to-point)

Operator: "on the open-loop streamed mode, I want the controller to just be
updating the velocity vector needed, not moving to prescribed motion points —
that should be the confirmed per-segment mode." So the three modes are now cleanly
differentiated:

- **`open_loop`** → NEW `PrintManager._execute_print_path_open_velocity`: a target
  advances ALONG the path by wall-clock time at the commanded speed, and each tick
  commands a continuous `VS` vector along the path **tangent** at the target
  (feed-forward, NO position reads, NO point-to-point moves). Smooth continuous
  velocity for even ink laydown; pump deposits ∝ the time-based target advance
  (`vol_per_mm = flow/print_speed`, so `pace_correction` stretches time, not
  volume — bead width invariant). Decel ramp near the end; a final positioning
  move lands exactly on the endpoint + drains XY (termination, not control). No
  runaway guard (no feedback) — the `VS` direction must be correct; bounded by a
  wall-time cap + always stops. Falls back to the discrete point-stream when the
  controller has no `send_velocity_xy`. New pure helper `tangent_at_arclength`.
- **`confirm`** → the point-to-point move + wait-for-arrival mode (unchanged).
- **`velocity`** → the closed-loop pure-pursuit follower (unchanged).

`PrintSettings.velocity_open_loop`; Quick Print maps mode `open_loop` →
`velocity_open_loop=True` (combo relabelled "Open-loop velocity (streamed
vectors)" / "Confirmed per-segment (point-to-point)"; tooltip updated). The XY
Challenge dialog's `_drive_open_loop` rewritten to the same feed-forward streamer
(labels/help updated). Needs real-HW verification on ME3B V1 (open-loop velocity
lays a continuous bead; VERIFY VS direction on a low safe path first). Tests:
`tests/test_v75x_open_loop_velocity.py` (12: tangent helper, dispatch + no-VS
fallback, feed-forward run proves streamed velocity not point-to-point, time-based
volume, pace trims time-not-volume, abort). Regression across
open-loop-velocity / comms / velocity-follow / confirmed / challenge / timing /
encoder / barrier suites, 104 green.

---

## Addendum 5 (2026-07-28) — corner-aware closed-loop control + shared law + XY-Challenge rework

Operator: the velocity closed-loop is the good one; (1) it must consume the
calibration we already measure (comms frequency, max speed, phase delay);
(2) reduce corner overshoot — corner-aware tuning + an auto-tunable PID;
(3) recolour the challenge overlay + show only the applicable params per mode +
a robustness panel; (4) confirmed-per-segment must stop only at CORNERS (it was
pausing at every node → very bad), making it ideal for straight-edged shapes
(the star). AskUserQuestion: **scheduling + cross-track PID**; **both** a ZN-relay
PID tuner and a coordinate-descent workflow; **configurable small** robustness
matrix (PASS/FAIL vs a resolution element, default 30 µm).

**NEW `SupportClasses/VelocityControl.py` — the shared, pure control law** (no Qt/
serial/numpy) so the XY-Challenge bench and the real `PrintManager` follower run
the SAME code (tuning transfers). The arc-length helpers (`polyline_arclength` /
`point_at_arclength` / `tangent_at_arclength` / `project_on_polyline`) MOVED here
(canonical home) and `PrintManager` re-exports them (existing imports unchanged).
Adds: `corner_flags`/`corner_turn_angle_deg`; `plan_speed_limits` (per-corner
speed limit from the turn angle, V-shaped ramp over `decel_mm`, reachable);
`PursuitState` + `pursuit_step` (one tick: bounded projection → carrot lookahead
ahead of REAL progress → steer + a perpendicular **cross-track PID** `kp/kd`;
magnitude clamped to `min(speed_cap, corner-limit)`); `resolve_control` (grounds
`control_hz` in `control_loop_ms`, SMS/clamp in `xy_max_speed_um_s`, and a
**dead-time speed cap** `v ≤ lookahead/((loop+phase)·safety)` — the fix that kills
the pure-pursuit overshoot "back-and-forth" limit-cycle); `relay_ultimate_gain`
(Åström `Ku=4d/(πa)`, `Tu`).

**`PrintManager`:** `_execute_print_path_velocity` rewritten onto
`resolve_control` + `plan_speed_limits` + `pursuit_step` — so it now uses the
measured max speed (SMS/clamp), the dead-time speed cap, corner slowdowns, and
the cross-track PID (all default-off/legacy when uncalibrated). End-of-path decel
now ramps from the EFFECTIVE (capped) speed, not the raw print speed, so a capped
run stops cleanly (no endpoint dither). The runaway guard / abort / ZP-safety /
pump-per-real-distance are unchanged. `_execute_print_path` **confirm branch**:
precompute `corner_flags(points, confirm_corner_angle_deg)`; wait-for-arrival +
pump-drain ONLY at corner vertices + the last point; straight-edge nodes STREAM
(short paced sleep) and the periodic planner-buffer barrier now runs on streamed
nodes in both modes. New `PrintSettings` fields (all default 0 = legacy):
`xy_max_speed_um_s`, `control_loop_ms`, `phase_lag_s`, `xy_accel_pct`(=80 now
declared), `vel_corner_angle_deg`, `vel_corner_speed_factor`, `vel_pid_kp`,
`vel_pid_kd`, `confirm_corner_angle_deg`.

**`PrintTimingCalibrationStore`:** `PATH_TUNING_DEFAULTS` extended — velocity +=
`corner_angle_deg`(30)/`corner_speed_factor`(0.4)/`pid_kp`/`pid_kd`, confirm +=
`corner_angle_deg`(30). New `get/set_resolution_element_um` (30 µm default),
`get/set_robustness_matrix` (3 shapes × 2 sizes × 3 speeds default), and
`get_phase_lag_s` (mean `by_phase` intercept).

**`gui/dialogs/xy_challenge_dialog.py` (reworked):** overlay recolours by ROLE —
ideal grey, newest **red**, best-so-far (min RMS) **green**, all worse attempts
low-alpha **blue**. Params are per-mode rows shown/hidden by the mode combo
(open_loop: pace/rate/decel · confirm: tol/corner-angle · velocity: lookahead/
rate/decel/corner-angle/corner-factor/Kp/Kd), + a shared **Resolution** (µm)
field. `_drive_velocity`/`_drive_confirm` now call the shared `VelocityControl`
law / `corner_flags` (bench == print). Auto-tune = **coordinate descent** over the
mode's ordered param list (carries the running best forward, persists each) +
a dedicated **Auto-tune PID (ZN)** button (velocity: relay oscillation on cross-
track error → `Ku/Tu` → `compute_zn_pid_gains` → Kp/Kd). New **Robustness panel**
button sweeps the shape×size×speed matrix in velocity mode, logging PASS/FAIL vs
the resolution element. **Quick Print `_build_settings`** stamps all the new
tuned + machine-measured fields onto `PrintSettings`.

**Reused (not reinvented):** `MotionController.compute_zn_pid_gains`;
`TrajectoryPlanner.detect_corners` logic (mirrored in `corner_flags`).

**Needs real-HW verification on ME3B V1** (VERIFY VS direction on a low safe path
first; velocity mode hits corners tighter + no back-and-forth; ZN + coordinate-
descent converge sensible values; confirm stops only at the star's tips; robust-
ness goes green ≤ resolution element; a real velocity Quick Print lays a
continuous on-shape bead). Tests: `test_v75x_velocity_control.py` (17) +
`test_v75x_xy_challenge_panel.py` (8, incl. follower dead-time cap + corner
slowdown + coordinate-descent convergence) + updated
`test_v75x_quick_print_confirmed_segments.py` (corner-only) +
`test_v75x_xy_challenge.py` (new defaults); regression across velocity-follow /
open-loop / confirmed / comms / timing / encoder / barrier + quick-print pick-
place/multi-ink/seam/print-setup/simple-pm/exec-log/gentle-descent/safe-Z/z-
retract suites — 125 + 124 + 32 green.

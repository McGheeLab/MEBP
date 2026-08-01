# MEBP v7.5.x — XY Printing Challenge upgrade

## Objective

Make the XY Printing Challenge a trustworthy tuning instrument for the closed-loop velocity
follower, and fix the three machine-level problems it exposed:

1. prints are capped at **0.314 mm/s** against a stage measured at 5.9 mm/s;
2. the follower can **deadlock** (dither in place with progress frozen) and nothing detects it;
3. the auto-tune **"just brings the velocity down"** and gives a different answer every run.

Operator requirement: **hold the commanded velocity on straights and slow down only at corners.**

## "Before" numbers (this machine, 2026-07-28) — so the "after" is comparable

| Quantity | Value | Note |
|---|---|---|
| `xy_max_speed_um_s` | 5945.6 (5.9 mm/s) | vs `max_speed: 50000` declared in `proscan_iii*.json` → **8.4×** |
| `control_loop_ms` | 31.75 (31.5 Hz) | consistent with the real follower rate |
| `get_phase_lag_s()` | **0.2872 s** | mean of non-negative `by_phase.intercept_s` = 0.1215, 0.5766, 0.1634 |
| resolved `speed_cap` | **0.3136 mm/s** | `0.2 / ((0.03175 + 0.2872) × 2)` |
| tuned velocity params | `lookahead 0.2`, `corner_speed_factor 0.2`, `pid_kp 4.0`, `pid_kd 0.0` | |
| `pace_correction` | 3.0 | top of its grid |
| tracking (completed run 18:30) | cross median 73 µm, p90 115, max 438 | vs a 30 µm resolution element |
| deadlock run 19:53 | `s` 0.121 → 0.509 mm over 37 s; 12.4 mm travel for 1.0 mm net | cross pinned ~640 µm |

## Root causes (all verified numerically or from logs)

1. **`lookahead` IS the speed knob** — `speed_cap = lookahead/((loop+phase)·safety)`, so sweeping
   lookahead sweeps speed 0.314 → 2.038 mm/s and minimising RMS *is* minimising speed. The operator's
   sweep shows RMS 8 → 104 µm tracking exactly that.
2. **Corner slowdown is inert** — at `print_speed = 5` the corner limits are 1.00–4.25 mm/s, all above
   the 0.314 cap, so `min(cap, corner_limit)` is always the cap. The operator's sweep shows RMS 8 µm
   flat across the whole `corner_speed_factor` grid. "Slow only at corners" is unreachable until the
   cap is fixed.
3. **The cap is driven by one bad sample** — the dominant 0.5766 s intercept is a single-point fit of
   one run (`by_phase["5.00|0.50"]`, `n_points: 1`). The set also contains two impossible negatives.
   (`slope_s_per_seg` is NOT the fix — it is seconds *per segment*, a unit error.)
4. **Unfinished runs win** — every best-scoring run logged `wall-time cap hit`; the truncated trace is
   scored anyway and `path_error` has no completion term.
5. **The metric is blind** — one-sided, unsigned, untimed, visited-samples-only. Back-and-forth
   retracing scores **zero** error.
6. **The cross-track PD can overwhelm forward motion** — `pursuit_step` adds the normal correction to
   a full-magnitude pursuit vector with no re-clamp; at `kp = 4.938`, `d = 0.64 mm` the command is
   76 % perpendicular.
7. **The lock is unrecoverable** — progress is forward-only and `project_on_polyline` searches forward
   only, so once off-path it can neither advance nor re-lock; the 3 mm runaway guard never trips at
   0.64 mm and there is no stall detector.
8. **The tuner chases noise** — one evaluation per candidate against ~73 µm noise, acceptance
   threshold 1.0 µm, `best_rms` starts at `inf` so the incumbent is never measured, store written
   mid-descent.
9. **The bench is unobservable** — no machine-readable log at all, so bench runs cannot be diffed
   against print JSONL; bench↔print equivalence is convention, not enforced, and they already differ.

## Files Modified

(see the approved plan for full detail — kept in sync as work lands)

| File | Change |
|---|---|
| `SupportClasses/VelocityControl.py` | `PursuitTuning`; bounded composition; d-filter/`ki`; re-acquire; stall/dither; lead predictor; `plan_speed_profile`; `resolve_control` levers; `FollowerConfig`/`build_follower`/`follower_tick` |
| `SupportClasses/XYChallenge.py` | `Sample`; `path_report`; `composite_score`; bucket-indexed scoring; new shapes |
| `SupportClasses/XYDeadTime.py` · `XYSpeedProbe.py` · `XYSettle.py` | new — dead-time identification, shared top-speed sweep, shared settle |
| `SupportClasses/ChallengeTuner.py` · `ChallengeRunLog.py` | new — optimiser (hardware-free), run records |
| `SupportClasses/PrintTimingCalibrationStore.py` | new tuning keys (0 = legacy); `velocity_dead_time_s`; objective weights; robustness results |
| `SupportClasses/PrintManager.py` | shared follower; guard dispatch; `PrintSettings.vel_tuning`; new JSONL fields |
| `SupportClasses/PrintExecutionLogger.py` | `file_prefix`; `sample_hz=0` disables the sampler |
| `SupportClasses/StageController.py` | `measure_control_loop_rate(axis=…)` |
| `gui/dialogs/xy_challenge_dialog.py` | `PARAM_SPEC`; per-mode params; shared follower; JSONL; tabs; Apply/Revert; `closeEvent` veto |
| `gui/widgets/challenge_*.py` | new — overlay, deviation strip, run list, robustness, tune trace, calibration panel |
| `gui/pages/workflows/quick_print_workflow.py` | stamp `vel_tuning` + `xy_jerk_pct` |
| `gui/pages/workflows/timing_calibration_workflow.py` | gate `_open_challenge`; inject `settings`; delegate the speed sweep |

## Implementation Steps

### Stage 0 — prerequisites (no behaviour change) — **DONE**
- [x] 0.1 Fixed the test-isolation leaks (`test_v75x_xy_challenge.py` restored `TCS._STORE`, a name
      that does not exist; `test_v75x_xy_challenge_panel.py` never restored at all). New
      `tests/support/store_fixture.py` (`temp_store` / `use_temp_store` / `make_temp_store`, restoring
      via `addCleanup`) + a guard test that proves the restore happens.
- [x] 0.2 Golden legacy-identity tests: `pursuit_step` (grid of paths × gains × offsets, compared to
      a verbatim inline transcription at 1e-12), `resolve_control` (240 parameter combinations),
      `plan_speed_limits` (profile sampled at 41 points per case), and the forward-only projection.
      Also pins **this machine's 0.3136 mm/s cap** and proves the corner limits all sit above it.
- [x] 0.3 Collapsed the seven parallel param structures into one declarative `PARAM_SPEC`
      (`ParamSpec` dataclass + `PARAM_SPECS`); `MODE_PARAMS`, `AUTOTUNE`, `_PARAM_KEY`, the spin
      constructors, the grid layout, `_mode_params`, `_apply_tuned` and `_load/_save_params_to_store`
      are all generated from it. Verified byte-identical to the originals and locked by a test.
      Needed a `grid_modes` field: the shared corner-angle spec would otherwise have leaked its grid
      into the velocity sweep (4 extra hardware runs per tune).

### Stage 1 — decouple speed from lookahead; measure the real dead time — **DONE**
- [x] 1.1 `SupportClasses/XYDeadTime.py::measure_velocity_dead_time` — step response, ±X/±Y/±diag,
      rest-jitter-derived motion threshold, poll-interval correction, FOPDT cruise fit for the
      apparent lag / rise time / cruise speed, medians + MAD spread. Always stops the stage and
      returns to start on **every** exit path; suspends the poller; clamps to the envelope; never
      touches Z. Plus `stable_speed_mm_s` so the UI and the follower share one arithmetic.
- [x] 1.2 Store `get/set_velocity_dead_time_s` (+ provenance meta) and `effective_dead_time_s()`
      returning `(value, source)`; `resolve_control(dead_time_s=…)` supersedes `phase_lag_s`;
      `get_phase_lag_s` docstring now states plainly that it is a settle time, lists this machine's
      five polluted values, and says why the slope is not a substitute either.
- [x] 1.3 `resolve_control` levers `lead_time_frac` / `min_lookahead_frac` / `max_speed_frac`
      (all 0 = legacy). Now also returns the **resolved** lookahead, the dead time, the lead, and a
      `cap_reason` naming the binding term.
- [x] 1.4 `hold_speed` — pins `speed_cap = print_speed` so the corner limit is the only modulation.
- [x] 1.5 Wired through to the print: `PrintSettings.vel_tuning` (one dict, not ~25 scalars) +
      `xy_jerk_pct` + `velocity_tuning()` accessor; `_execute_print_path_velocity` consumes all the
      levers, uses the resolved lookahead, applies jerk, and logs `cap_reason` / `dead_time_s` /
      `lead_s` / `lookahead_resolved` / the whole tuning dict in `path_start`. Quick Print stamps the
      velocity bucket verbatim and prefers the measured dead time. The bench's `_resolve` makes the
      **same call with the same arguments**, so tuning still transfers.
- [x] 1.7 Speed-budget readout in the dialog + a **"Measure dead time"** button. On the operator's
      real numbers it reads: *commanded 0.31 mm/s — limited by dead_time … dead time from by_phase
      settle mean ⚠ … ⚠ Corner tuning is INERT here: the sharpest corner limit is 1.00 mm/s, above
      the 0.31 mm/s cap.* After storing a measured 45 ms it reads 1.30 mm/s (4.1× faster) and the
      inert warning disappears; with `hold_speed` it reads 5.00 mm/s.
- [ ] 1.6 `measure_control_loop_rate(axis="x"|"y"|"diag")` — deferred (the probe currently oscillates
      in X only, while the follower commands both axes every tick).
- [ ] 1.8 `XYSpeedProbe.measure_top_speed` extracted + shared, R² ≥ 0.95 gate, and the
      declared-vs-measured (50000 vs 5946, 8.4×) reconciliation table — deferred.

### Stage 2 — honest scoring — **DONE**
- [x] 2.1 `Sample` (a **tuple subclass**, not a dataclass, so indexing / iteration / `for x, y in …`
      unpacking all keep working and the overlay painter, `offset_path` and `_point_to_segment_um`
      need no branching) + `as_samples(seq, dt=)`. `path_error` keeps its exact signature and keys.
- [x] 2.2 Metrics: `signed_cross_track` (inside-cut vs outside-overshoot), `two_sided_deviation`
      (adds the ideal→actual direction = coverage), `coverage`, `completion_fraction`,
      `path_length_ratio` (**the dither detector**), `reversal_count`, `corner_overshoot`,
      `along_track_lag`, `throughput`, `dwell_events` — all gathered by `path_report`, which keeps
      `legacy = path_error(...)` inside it so historical RMS numbers stay comparable.
- [x] 2.3 `composite_score` with **graded** hard failures (`1000 + penalty`, not `inf`) so coordinate
      descent retains a gradient out of a failing region — which matters because the shipped tuning
      *is* in one. Weights in `OBJECTIVE_DEFAULTS`, overridable and recorded per run. Plus
      `verdict_for` requiring **both** `rms ≤ element` and `max ≤ 2× element`.
- [x] 2.4 `_SegGrid` uniform spatial index — **exact**, not approximate (the ring search only stops
      once no unexamined cell can beat the best distance), proved equal to the retained
      `_path_error_bruteforce` on every shape including far-off-path samples.
- [x] 2.5 New shapes **Line-Reversal** (single axis, pure 180° reversals — isolates backlash with no
      curvature confound), **Comb** (tooth pitches 2×/1.5×/1×/0.75× the element, so the finest pair
      is deliberately below it), **Dwell-Stitch**; `SHAPE_PARAMS` + `shape_meta` expose the
      previously-private `points`/`rows`/`turns` and report corner/dwell arc lengths.
- [x] 2.6 Fixed a real bug found while validating: `completion_fraction` used a GLOBAL nearest
      projection, so on a **closed** shape the final point (spatially identical to the start)
      projected to `s = 0` and a perfect run read as 98.8 % complete → every perfect run would have
      hard-failed. Now uses a forward-windowed monotone cursor (`_monotone_progress`), which is also
      spiral-loop-snap safe. Locked by `TestPerfectRunPasses` over all eight shapes.

### PID tuner fix (out of order — operator: *"the PID tuner is not doing a good job"*) — **DONE**

Observed: `pid_kp` walked **4.938 → 6.216 → 6.763 → 7.052** across consecutive presses.

**The key realisation:** the cross-track plant is known in closed form. A
perpendicular velocity command integrates *directly* into cross-track position, so from
`v_n` to `d` it is a **pure integrator with transport delay** — which fixes
`Ku = π/(2L)` and `Tu = 4L` exactly (`L = dead_time + loop_period`). The relay experiment
was trying to *measure* a quantity that can simply be *computed*.

- [x] **`VelocityControl.plant_ultimate_gain`** — analytic `(Ku, Tu)` from the dead time.
- [x] **`VelocityControl.pid_gains_from_dead_time`** — ZN rule over the analytic `Ku`/`Tu`;
      exactly repeatable, zero hardware time, `gain_margin` and `use_kd` options.
- [x] **`VelocityControl.relay_sanity`** — a measured `Ku` above `π/(2L)` is *physically
      impossible*; the gate catches it and reports which half of the measurement failed.
- [x] **Three relay defects fixed** in `_worker_zn`: (a) the switch had **no hysteresis**, so
      near the line the sign chattered on µm encoder quantisation and manufactured spurious
      tiny half-cycles; (b) the amplitude was the **mean** over half-cycles, which those
      spurious cycles dragged down — and since `Ku ∝ 1/a` an under-measured amplitude
      *inflates* the gain; (c) no repeat, no sanity check. Now: hysteresis band scaled off the
      resolution element, **median of per-cycle peak** amplitudes with the first cycles dropped
      as transient, ≥6 flips required, and the physical gate with fallback to the analytic gains.
- [x] **New "PID from dead time" button** — instant, no stage motion, not gated on a connected
      stage, identical every press.
- [x] **`kd` and `ki` now deliberately 0**, with the reasoning recorded: differentiating a
      µm-quantised encoder at 25 Hz injects more noise than the D term removes (the filter is
      Stage 3.2), and the plant is *already* an integrator so P alone has zero steady-state
      error — a second integrator is a classic limit-cycle source. `ki` was previously discarded
      by accident; it is now discarded on purpose and documented.
- [x] **Widened the `pid_kp` descent grid** `[0, 0.5, 1, 2, 4]` → `[0, 1, 2, 3.5, 5, 7, 9]`. The
      correct gain for this machine is ≈5.24, **above the old grid's maximum** — the descent was
      structurally unable to find it. (Caught because the operator's store showed `kp = 2.0`, a
      grid value, not a ZN value.)

**Evidence the model is right:** the operator's own relay run measured `Tu = 0.391 s` against
the predicted **0.396 s (~1 %)** — while its `Ku` of 18.8 was 1.19× the physical maximum of
15.9. `Tu` is a timing and `Ku` an amplitude, so a matching period with an inflated gain is
precisely the signature of the amplitude bug.

**Also confirmed from the store:** the operator ran "Measure dead time" on real hardware and it
returned **67 ms**, against the 287 ms the `by_phase` table was claiming — a 4.3× correction
that both raises the speed cap and moves the correct `kp` from 1.8 to 5.24.

### ⭐ ROOT CAUSE OF THE DEADLOCK FOUND AND FIXED — the projection window

Found while bench-simulating the one-click calibrator: the derived settings failed
verification at 2 mm/s but *passed* at 3 mm/s. Slower should be easier, so that
was a bug, not tuning.

`pursuit_step` bounds the projection search to `max_ds = max(0.15, speed·dt·6)`
ahead of the cursor, and `project_on_polyline` broke out of its scan as soon as the
next segment's far end exceeded that window. **When the window is smaller than the
local segment, only the CURRENT segment is ever examined** — so `s` pins at that
segment's end, `seg_i` never advances, the carrot stops moving, and the stage sits
there while the cross-track term thrashes it about.

The operator's numbers fit exactly:

| | |
|---|---|
| print speed / loop rate | 0.996 mm/s @ 25 Hz |
| ⇒ projection window | `max(0.15, 0.996·0.04·6)` = **0.239 mm** |
| toolpath mean segment (`path_start.seg_mm_mean`) | **0.4859 mm** |
| window < segment? | **yes** — the next segment is unreachable |
| logged frozen progress | `s` 0.121 → **0.509 mm** ≈ one segment |
| logged travel vs net | 12.4 mm travelled, 1.0 mm net |

- [x] **Fix:** `MIN_SEGMENTS_EXAMINED = 2` — the scan always examines at least the
      segment *after* the cursor, whatever the window says. Two segments is
      nothing like one revolution of a spiral, so the loop-snap protection the
      window exists for is untouched, and the per-tick *committed* advance is
      still bounded by `max_ds`, so pump deposition and the snap guard are
      unchanged (both asserted).
- [x] Regression tests: the exact failing configuration, a range of windows, the
      operator's literal numbers, the per-tick bound, and a no-snap check.

**⚠ This is a behaviour change on the default path, not an opt-in** — deliberately,
because the old behaviour is a deadlock. Verified against the existing spiral
loop-snap tests (`test_bounded_window_prevents_loop_snap`,
`test_completes_multiloop_spiral_without_snap`), which still pass.

**Effect in simulation** (delay-pipeline stage, 55 ms dead time + 25 ms rise):
every shape × speed now completes with `completion 1.000` and a dither ratio of
0.9–1.0, where before it froze at ~0.1 completion with a dither ratio of 40–75.

Residual deviation is now pure-pursuit **corner cutting**, which scales with the
lookahead (p95 ≈ 117 µm at 0.34 mm lookahead → 587 µm at 1.68 mm). That is exactly
what the cross-track gain exists to remove — and it stays withheld until the
bounded composition lands, so Stage 3.1 is the accuracy step.

### All-in-one calibration tool — **DONE (measure/derive/apply); verification pending HW**

Operator: *"an all-in-one calibration tool that does everything in the XY stage
timing calibration in one shot, and the best print settings are then applied for
all prints"* + *"any tuning is a systematic one-click process"*.

**The insight that makes it systematic:** almost nothing needs to be *searched*.
Three measurements — loop period, dead time, top speed — plus closed-form physics
fix everything else, so the same machine measured twice yields the same settings.

- [x] **NEW `SupportClasses/XYAutoCalibration.py`** — pure, hardware-free:
      `MeasuredMachine` / `CalibrationPolicy` / `derive_settings` (every value
      carries a `Note` with its reasoning) / `validate` / `CALIBRATION_STEPS` (the
      dependency-ordered plan) / store read-write helpers.
- [x] **Derivations:** `lookahead = target·L·safety·headroom` (**inverted** so the
      lookahead is sized *from* the desired speed instead of dictating it),
      `control_hz` from the measured period, `pid_kp` from `Ku = π/(2L)`,
      `decel_mm ≥ coast + ramp`, `d_filter_hz = control_hz/8`, `settle_tol` = the
      resolution element, plus the Stage-3 guard values.
- [x] **`lookahead_margin` (1.5)** — solving for the target puts the operating
      point exactly ON the stability boundary; a bench sweep showed that dithers,
      so the derived lookahead now carries headroom.
- [x] **One-click worker** `_worker_calibrate_all` + a **⚙ Calibrate XY (one
      click)** button: centre → comms → dead time → derive → apply → verify on
      Square + Star with the completion/dither-aware score, logging every derived
      value *and its reason*.
- [x] **Operator requirement — never touch the travel extents.** Every probe moves
      *relative* to where it starts, and a clamped probe does not fail loudly: it
      silently measures the clamp ("no motion detected", or a truncated distance
      fit). So `envelope_center_um` / `usable_test_radius_um` / `check_fits` /
      `fit_shape_size_mm` / `center_stage` drive to the middle of the reachable
      envelope first, shrink any excursion that would not fit with a 2 mm margin,
      and refuse outright when the envelope is unknown. `XYDeadTime` centres by
      default (`center_first=True`) and shape sizes are auto-shrunk. Centring goes
      through `safe_travel_to(..., target_z_mm=None)` so Z retracts first and never
      descends.
- [x] **"Applied to ALL prints" — a real gap closed.** Only Quick Print stamped any
      of the tuning; **Full Print stamped none of it**, so a bench session had
      literally no effect there. One shared `stamp_print_settings` is now called by
      both `quick_print_workflow._build_settings` and
      `print_setup_legacy._get_settings`.
- [x] **Safety gate on the gains:** `VC.SUPPORTS_BOUNDED_CROSS_TRACK = False`.
      While the law adds the PD to a full-magnitude pursuit vector with no
      re-clamp, `derive_settings` deliberately emits `pid_kp = 0` (pure pursuit),
      reports the analytic gain it *would* have used, and explains why — so an
      automated calibration cannot hand the machine a gain the law mishandles.
      Flip the flag with Stage 3.1.
- [x] Corrected a derivation error found in simulation: `pace_correction` was being
      computed as top-speed ÷ the dead-time probe's cruise speed, but that probe
      raises SMS to ~1.5× its own step magnitude so its cruise reads back the
      *probe* speed — a meaningless ratio (it read 1.98 purely because the probe
      ran at 3 mm/s against a 5.9 mm/s stage). Now left to the dedicated sweep.

**Derived for ME3B V1 at a 5 mm/s target** (measured L = 99 ms):

| param | was | derived | why |
|---|---|---|---|
| `lookahead_mm` | 0.2 | **1.49** | sized for 5 mm/s ⇒ unlocks it (0.2 capped at 1.0 mm/s) |
| `control_hz` | 25.0 | **31.5** | the measured period |
| `decel_mm` | 0.3 | **0.565** | 0.3 was *shorter* than the 495 µm coast ⇒ endpoint overshoot |
| `pid_kp` | 2.0 | **0** (5.24 pending) | withheld until the composition is bounded |
| `hold_speed` | off | **on** | straights at full speed, corners the only slowdown |
| `max_speed_frac` | — | **0.9** | never command above 90 % of the measured top speed |

## REAL-HARDWARE SESSION (ME3B V1, 2026-07-28) — measured, not simulated

Run headless against the real Prior ProScan on **COM6**, XY-only. The Marlin/ZP
board on **COM4** (STM32 VCP) was deliberately never opened — opening it asserts
DTR and resets the board, losing Z while a needle is installed — so no Z command
was reachable from the process at all. The app's clean-shutdown record had left
Z = 44.0 = the calibrated Safe Z (plate bottom 21.13, `z_up_sign=+1`), i.e. the
needle retracted.

**A hazard caught before any motion:** a freshly-constructed `StageController`
carries the ±130000/±85000 *default* safety limits, whose "centre" is **(0, 0)** —
the CORNER of this machine's real `0..116327 / 0..74235` envelope. Centring would
have driven straight into the extent. The real limits must be loaded from
`settings.json` first (`SafetyLimits.from_dict`).

### ✅ `VS` direction verified — the long-standing unknown

Every plan doc said "verify the VS direction FIRST". Done, with 300 µm/s for 0.35 s
per axis and a 1200 µm abort bound:

| commanded | measured | |
|---|---|---|
| +X | +112.0 µm | OK |
| −X | −111.0 µm | OK |
| +Y | +111.0 µm | OK |
| −Y | −112.0 µm | OK |

**Correct on all four axes**, and it returned to the start with 0.0 µm drift. No
sign flip is needed anywhere.

### Measurements

| quantity | value |
|---|---|
| control-loop period | **32.0 ms (31.3 Hz)** — `moved=True`, 761 µm excursion |
| dead time | **40.0 ms** (all 5 trials identical) |
| apparent lag (FOPDT) | **67.1 ms ± 0.34 ms** — a 0.5 % spread |
| rise time τ | **27.1 ms** |
| cruise cross-check | **2996 µm/s** commanded 3000 ⇒ the stored 5946 top speed is credible |
| absolute-move accuracy | **6/6 landed to 0.7 µm** |

⇒ `L = 99.1 ms`, `Ku = π/(2L) = 15.85`, derived lookahead **0.892 mm**, achievable
cap **4.50 mm/s** against a 3.0 mm/s target, `pid_kp` pending **5.231**.

### Verification (pure pursuit — the gain is withheld)

| shape | p95 | rms | max | completion | dither | wall |
|---|---|---|---|---|---|---|
| **Circle** | 84 µm | 74 µm | 83 µm | **1.000** | 0.98 | 8.7 s |
| **Star** | 364 µm | 89 µm | 250 µm | **1.000** | 0.88 | 9.1 s |
| Square | — | — | — | — | — | *invalidated, see below* |

Both completed with no dither. Star's p95 is corner overshoot at the tips —
the pure-pursuit corner cut, which is what the withheld cross-track gain removes.

### 🐞 NEW BUG FOUND — Prior ack corruption (diagnosed, NOT yet fixed)

`_send_protocol_command` → `send_command` is a **bare write with no ack drain**,
but the Prior answers a bare `R` to *every* command including the parameter setters
(`SMS`/`SAS`). That unread `R` is then consumed by the next position query:

```
set_speed_mm_s: 5.9 mm/s = SMS 100%
Failed to parse XY position: Expected 3 values, got 1: R
```

— which is in **the operator's own app log**, and reproduced here exactly: six
absolute moves landed to 0.7 µm, then the first read *after* `set_speed_mm_s`
returned `None`. The corrupted read makes `wait_for_xy_arrival` poll garbage and
time out **even though the stage arrived**, so a print can proceed believing it is
somewhere it is not. That is what invalidated the Square: its move to the shape
start silently never landed (`target=(54.16, 33.12), actual=(57.44, 37.12)`) and the
follower then drove from 4.6 mm off-path.

Also observed: after `measure_control_loop_rate` the stream is left desynchronised
(`moved=False`, 12 µm excursion, then reads fail outright) — so the **"Check comms
rate" button is suspect for the same reason**.

**⚠ A naive fix made it worse and was REVERTED.** Adding `write + _drain_ack` under
`_serial_lock` to `_send_protocol_command` caused every subsequent read to come back
**empty** — the Prior stopped answering `P` at all. So the interaction is subtler
than a missing drain (timing, lock nesting with the poller, or the detection-time
setters). Reverted to known-good and confirmed healthy (reads restored). The
`send_command(drain_ack=...)` capability is left in place, documented and unused by
default, for a bench session with a raw serial trace.

- [x] `center_stage` now **verifies arrival** instead of assuming it — it had
      reported `ok=True` while the stage was still ~0.7 mm short (the wait's return
      value was ignored, and reads can be corrupted by the above). Every probe would
      otherwise be measured about the wrong origin.
- [ ] **Prior ack drain** — needs a raw-serial bench session. This is the top
      remaining hardware issue: it corrupts arrival waits, so it undermines
      confirm-mode printing and any point-to-point positioning.

## STAGE MODEL + PATH SIMULATOR + GEOMETRY PANEL + PRINTABILITY (2026-07-28, second HW session) — **DONE**

The stage's measured dynamics are now a **saved artefact** that can be simulated
against, so tunings and print geometry can be evaluated *offline* — and the model
was validated cell-by-cell against a real 18-run geometry panel on ME3B V1.

**New modules (pure, no Qt/serial/numpy):**

- `SupportClasses/XYStageModel.py` — `StageCharacteristics` (dead time, τ, top
  speed, loop period, readout quantum; `from_store` reads what the calibration
  flow already persists — `velocity_dead_time_s` + meta `tau_s`,
  `control_loop_ms`, `xy_max_speed_um_s`; `to_dict/from_dict/from_machine`) +
  `XYStageModel`, a FOPDT integrator: commands enter a **dead-time pipeline**
  (each takes effect `dead_time_s` after issue — NOT a resettable timer, which
  an earlier ad-hoc sim got wrong and froze forever under re-commanding),
  first-order velocity lag toward the active command, top-speed magnitude
  clamp, quantised readout, deterministic explicit-`advance` time.
- `SupportClasses/XYPathSimulator.py` — **ONE follower loop, `follow_path`,
  over injected I/O bindings** (`FollowIO`): an exact transcription of the
  bench's `_drive_velocity` tick (same `resolve_control`/`plan_speed_limits`/
  `pursuit_step`, same `max_ds`/decel-taper/arrive/runaway/wall-cap semantics,
  always VS 0,0 on exit). `model_io(model)` binds it to the stage model with a
  **virtual clock** (deterministic, machine-speed); `controller_io(ctrl)` binds
  it to the real controller — so simulated and hardware runs differ only by the
  stage, never by loop logic. Plus: `simulate_follow` → `SimResult` (samples,
  `path_report`, `composite_score`, verdict, signed `cross_profile`,
  `fail_regions` in world coords), `fail_regions_from_profile` (severity mirrors
  `verdict_for`: warn > 1× element, fail > 2×), `tuning_from_store` /
  `resolve_for` (store zeros → bench spin defaults; levers stay meaningfully 0),
  `simulate_panel` (shape × size matrix),
  `printing_subpaths_from_trajectory` (Nx7 → printing XY sub-paths; travel =
  pump-flat & XY-moving, the `_travel_mask` rule; Z-only/pump-only segments
  neither extend nor break; weld nodes don't split) and
  `check_toolpath`/`check_trajectory` (per-sub-path SimResults + aggregate
  verdict + fail regions with `path_index`).

**GUI:**

- `gui/dialogs/xy_geometry_panel_dialog.py` — shape × size matrix, each cell
  ideal (grey) vs simulated (dashed) and/or hardware (solid) trace coloured by
  verdict + metric caption; **Simulate panel** (instant, no motion, gated on a
  measured machine) and **Run on stage** (gated on XY connected + Safe-Z when ZP
  present; centres the stage first via `AC.center_stage`, suspends
  poller/watchdog, goto verified by position polling — NOT
  `wait_for_xy_arrival`, see the ack bug — skips cells that don't fit the
  usable radius, VS 0,0 + resume in `finally`). Results persist to
  `logs/challenge/geometry_panel_<ts>.json` and old panels (including the
  hardware-script ones) re-load via **Load panel…** (world-frame records are
  re-centred per cell). Launched from the challenge dialog's new
  **📐 Geometry panel…** button.
- `gui/dialogs/sketch_printability_dialog.py` + a **🎯 Check printability**
  button on Print Builder → Sketch — splits the compiled trajectory, simulates
  every printing sub-path with the SAVED characteristics + CURRENT tuning, and
  shows: predicted path per sub-path coloured by verdict, rings at predicted
  fail/warn regions (world mm), a per-path detail list ("path 2 stalls at
  38 %"), and a speed spin that re-simulates instantly. Verdicts apply
  `PREDICTION_HEADROOM = 2.0` (see validation below) — a prediction must clear
  a 2× tighter bar to be called PASS.

**HARDWARE VALIDATION (ME3B V1, 18 cells, 6 shapes × 2/5/10 mm, 3 mm/s,
lookahead 0.892 + hold_speed + max_speed_frac 0.9 + the store's kp 2.0 /
corner_factor 0.2 / decel 0.3; each cell simulated FIRST, then driven through
the SAME shipped `follow_path`):**

| result | value |
|---|---|
| verdict agreement | **16/18** (misses: Circle 5 pass→fail, Circle 10 pass→marginal — both boundary cells) |
| stall prediction | **all 4 real stalls predicted** (Star 2 mm; Line-Reversal 2/5/10) with wall-times matching to ~0.1 s; 1 false stall (Star 10 predicted stall, actually completed) |
| magnitude | median actual/predicted p95 = **1.56** (model is deterministic — no encoder noise/comms jitter) → `PREDICTION_HEADROOM = 2.0` |
| headline demo | **Star 2 mm: predicted deadlock (p95 1111 µm, wall-cap) → actual deadlock (p95 1154 µm, dither 22.8, wall-cap)** — a hardware failure called before the stage moved |

Recorded: `logs/challenge/geometry_panel_20260728_164836.json`. Stage re-centred
and disconnected cleanly; ZP never opened.

**Two OPERATIONAL findings from the panel (both reproduced in sim AND on HW):**

1. **180° reversals stall the velocity follower** — all three Line-Reversal
   cells sat at the first reversal vertex until wall-cap on both sim and HW.
   The sketch **retrace features** (back-trace, overlap-travel retrace along a
   bead) produce exactly this geometry, so velocity-mode prints of retraced
   sketches will stall until Stage 3.3 (re-acquire / reversal handling) lands.
   The printability checker now flags this at design time.
2. **Feature size below ~2× lookahead risks a pursuit orbit** at low speed
   (`max_ds = max(0.15, speed·dt·6)` shrinks while the carrot pulls the stage
   inside the notch): Star 2 mm vs lookahead 0.892 deadlocked at 3 mm/s; in sim
   the same happens to Star 5 mm below ~2.5 mm/s. Curvature/feature-aware
   lookahead is Stage 3.6 material; until then the checker catches it.

Tests: `tests/test_v75x_xy_stage_model.py` (14 — pipeline-not-timer regression,
FOPDT step response matching the measured 86 ms half-speed crossing, clamp/
quantisation/odometer, store round-trips) +
`tests/test_v75x_xy_path_simulator.py` (24 — always-stops-on-exit, stop/wall
reasons, determinism, corner-cut ∝ lookahead, the orbit-failure prediction, the
**hardware-verified operating point pinned in a loose band** (Star 364 ± 200,
Circle 84 ± 60), fail-region severity/coords, trajectory splitting incl. weld
nodes and Z-lifts, toolpath aggregation) +
`tests/test_v75x_geometry_panel_and_printability.py` (11 — gates, simulated
panel end-to-end, world-frame panel loading, unmeasured-machine guidance,
pass/fail sketches, speed re-check). Challenge/calibration regression 195 +
sketch-page 21 green.

- [ ] **Model fidelity follow-up (optional):** add a measured disturbance/noise
      term so the ~1.56× optimism shrinks; re-derive `PREDICTION_HEADROOM` from
      the next panel.
- [x] **Stage-3 tie-in — SUPERSEDED by the feed planner (below):** the planned
      panel re-ran the same 18 cells the same day and every previous failure
      passed; the planner is the deterministic answer to reversals + small
      features, ahead of (and independent of) the control-law re-acquire work.

## FEED PLANNER — 18/18 SHAPES ≤ 30 µm ON REAL HARDWARE (2026-07-28, 17:22 panel) — **DONE**

Operator: *"we should be able to hit any shape by just accounting for different
features of the print path — we know the print path a priori"* and *"essentially
we need to be able to get all 18 shapes have less than 30 microns from ideal."*
Both delivered, measured, same day, same stage.

**Why tuning alone could not do it (simulated first, then confirmed):** the
DERIVED optimal settings still left 16/18 failing — pure-pursuit corner cutting
(~0.4·lookahead ≈ 220 µm at the speed-stable lookahead) and the reversal stalls
are structural properties of one-fixed-tuning following, not bad parameter
values.

**NEW `SupportClasses/XYFeedPlan.py`** — a deterministic, feature-aware motion
plan built FROM the known path:

  • **Sharp corners (> 55°) and 180° reversals are section SPLITS** — the plan
    decelerates and stops ON the vertex (10 µm arrive + settle dwell), then
    continues as a fresh section. Zero corner cut and zero reversal stall by
    construction.
  • **Curved sections get a curvature-sized lookahead** `la = 0.8·√(2·R·δ)`
    (pursuit standoff ≈ la²/2R inverted for the budget δ = element/1.6
    headroom) and the dead-time stability rule sets that section's speed
    `v = la/(L·safety)`. A 2 mm circle plans itself at ~0.79 mm/s; straights
    run the full target speed with the full lookahead.
  • `build_plan` (every section carries its reasoning string) → `run_plan`
    (sections are consecutive `follow_path` runs over the SAME injected I/O —
    simulation and hardware execute identical code) → `simulate_plan` (scored
    against the FULL original path so splits can't hide anything) →
    `plan_rows` (the deterministic time-matched X/Y/Z/pump trajectory: pump
    volume ∝ arc length, stop dwells at splits — the all-axis artefact; the
    runtime follower ALSO deposits per measured Δs, so the bead stays correct
    even off-schedule).

**The enabling fix — lag-aware end taper** (`follow_path` gains opt-in
`arrive_mm` / `end_lag_s` / `end_floor_frac`, defaults = legacy, so bench and
print are untouched): the naive `remaining/decel` taper commands speeds for
where the stage WAS one dead-time ago, so a 3 mm/s section crossed its endpoint
at ~1.5 mm/s and overshot **153 µm** (measured on the model; the constant
~163 µm max in the first planner sweep). The taper now subtracts the MEASURED
velocity (successive position reads — the honest estimator that transfers to
hardware; the commanded value under-predicts during braking because of τ + the
pipeline) times the lag from the remaining distance, with an absolute ~80 µm/s
floor → crossing coast ≈ 8 µm.

**PLANNED HARDWARE PANEL (ME3B V1, 17:22, `geometry_panel_planned_20260728_172210.json`):**

| shape (worst previous) | before (fixed tuning) | after (planned) |
|---|---|---|
| Star 2 mm | **1154 µm, DEADLOCK** | **7 µm** |
| Square 2 mm | 348 µm | **1 µm** |
| Line-Reversal 5 mm | STALL (never turned) | **1 µm** |
| Circle 5 mm | 86 µm | 17 µm |
| Star 5 mm | 361 µm | 27 µm (panel worst) |
| **all 18 cells** | 0/18 ≤ 30 µm | **18/18 ≤ 30 µm** |

Simulation predicted 18/18 before the stage moved (predicted p95 0–16 µm;
actual 1–27 µm). Wall-time cost of the stops ≈ 10–20 % vs the raw-speed run.
Figures: `logs/challenge/geometry_panel_planned_20260728_172210.png`,
`geometry_panel_before_after.png`; renderer `tools_render_geometry_panel.py`
(repo root, newest panel by default).

**Quick Print integration (the "display the simulation + realtime progress"
ask):** `PrintTrajectoryMonitorView` gains `set_predicted_path(segments, note)`
(dashed peach = the SIMULATED stage path, included in the auto-fit bbox) and
live **progress tracking** — while recording, each live position is projected
onto the plan with a forward-only cursor (`VC.project_on_polyline`, the same
primitive the follower uses) and the readout band shows
`print 42% · 12.3/29.0 mm · dev 38 µm`. `QuickPrintWorkflowPage` runs the
simulation on a background thread after every plan refresh (velocity mode +
measured machine only; generation-tagged so a stale sim can't paint over a new
selection) via a new `_PrintBridge.predicted` signal; the motion-mode combo
refreshes it. **Wiring the FeedPlan into the print executor itself is the
follow-up** (pump semantics need their own pass); today the plan drives the
bench/panel and the simulation, and Quick Print shows the prediction.

Tests: `tests/test_v75x_xy_feed_plan.py` (17 — geometry analysis, curvature
sizing, splits, the ≤30 µm bar locked in sim for each previously-failing shape
class, lag-taper bounded vs the naive taper's measured overshoot, stop event,
`plan_rows` time-monotone/volume-by-arclength/stop-dwells) +
`tests/test_v75x_quick_print_predicted_overlay.py` (8 — predicted layer, bbox,
progress monotone → 1.0 across a travel, deviation, reset, bridge gen-guard).

### Stage 3 — control law
- [ ] 3.1 `PursuitTuning` + extended `PursuitState`; bounded normal / forward floor.
      **⚠ Until this lands, a correct `kp` is still applied through an UNBOUNDED composition**
      (the PD is added to a full-magnitude pursuit vector with no re-clamp), so a large gain
      still steers mostly sideways. This is the next thing to do.
- [ ] 3.2 Derivative LP filter (`d_filter_hz` is already in the store, unconsumed) — required
      before `kd` can safely be non-zero; `ki` with anti-windup (default 0, off the tune grid).
- [ ] 3.3 `project_on_polyline(max_behind=…)` + re-acquire state machine + `path_loop_pitch_mm`.
- [ ] 3.4 Stall + dither detectors returning `state.status`.
- [ ] 3.5 Lead predictor with the measured/predicted accounting split.
- [ ] 3.6 `plan_speed_profile` / `SpeedProfile` (curvature limit + reachability sweep, O(1) `at`).
- [ ] 3.7 `jerk_pct` plumbed to `XYStage.set_jerk`.

### Stage 4 — shared follower
- [ ] 4.1 Module constants + `FollowerConfig` + `resolve_follower_config`.
- [ ] 4.2 `build_follower` / `follower_tick`.
- [ ] 4.3 `PrintManager._execute_print_path_velocity` onto the shared follower + guard dispatch.
- [ ] 4.4 `_drive_velocity` onto the shared follower; close the arrive-tol / accel / end-settle /
      open-loop divergences; `XYSettle` extraction.
- [ ] 4.5 Bench≡print config equality test + structural field-coverage test.
- [ ] 4.6 Per-mode `_params` model; `closeEvent` veto.

### Stage 5 — run records + observability
- [ ] 5.1 `PrintExecutionLogger` `file_prefix` + `sample_hz=0`.
- [ ] 5.2 `ChallengeRunLog` + `ChallengeRun`; `_run_one` funnel; JSONL with print-identical events.
- [ ] 5.3 `PathOverlay` extracted with per-attempt ideals, scale bar, legend, corner markers.
- [ ] 5.4 Deviation-vs-arclength strip; run-history list + export.

### Stage 6 — repeatable tuner
- [ ] 6.1 `ChallengeTuner` — noise floor (MAD), incumbent-first, coarse pass, Hooke–Jeeves,
      paired A/B/A/B confirmation, abort-safe snapshot.
- [ ] 6.2 Budget presets (Quick 40 / Standard 120 / Thorough 240) + measured wall-clock estimate.
- [ ] 6.3 Deterministic trial protocol + variance reduction (fixed approach, trim ends, uniform
      arc-length resample).
- [ ] 6.4 Convergence trace + per-parameter sensitivity.
- [ ] 6.5 Robust ZN (median of M, real lookahead carrot, validation, Apply/Revert).

### Stage 7 — robustness panel
- [ ] 7.1 Editable matrix widget + `set_robustness_matrix` caller.
- [ ] 7.2 Persisted per-cell results with `params_fingerprint`; re-run failures only; report export.

## Testing Notes

Baseline before this work: **36 tests green** across `tests/test_v75x_velocity_control.py`,
`tests/test_v75x_xy_challenge.py`, `tests/test_v75x_xy_challenge_panel.py`.

**After Stages 0–2: 117 green** across those three plus the new
`tests/test_v75x_xy_challenge_metrics.py` (42) and `tests/test_v75x_xy_dead_time.py` (18).
Regression green: follower/timing suites **89**
(`quick_print_velocity_follow`, `quick_print_confirmed_segments`, `open_loop_velocity`,
`comms_frequency_check`, `print_path_planner_barrier`, `print_timing_calibration`,
`timing_encoder_detector`) and wider print/sketch **205**
(`quick_print_pick_and_place`, `quick_print_multi_ink`, `quick_print_travel_split`,
`multi_object_print_seam`, `print_setup_routine`, `print_execution_logging`,
`simple_print_manager`, `print_always_safe_z`, `z_retract_before_xy_travel`,
`gentle_descent_slow_final`, `extrusion_volume_calc`, `sketch_overlap_start_end_markers`).
**411 total.**

Demonstrated on synthetic reconstructions of the operator's own failures:

| Case | old `path_error` | new score |
|---|---|---|
| perfect run, all 8 shapes | rms 0 | PASS, score 0.000 |
| **the logged deadlock** (12.4 mm travel / 1.0 mm net) | **rms 0.0 µm — blind** | FAIL `incomplete` + `dither` + `runaway`; completion 0.012, dither ratio 29.5, 38 reversals |
| **exact retrace of the first 1 mm** | **rms 0.00 µm — blind** | FAIL `incomplete` |
| half the path, perfectly tracked | rms 0.00 µm | FAIL `incomplete`; ideal→actual p95 9000 µm |
| 25 µm tracking error | — | PASS, score 1.25 |
| 150 µm tracking error | — | FAIL, score 9.50 |
| same geometry, 10× slower | identical | score 0.000 → 0.250 (the time term) |

The four decisive tests:
1. **Deadlock regression** — a fake stage *with transport delay* (today's `_FakeCtrl` is zero-latency
   and cannot reproduce it) started 0.64 mm off-path with the stored gains: legacy tuning locks, new
   tuning completes.
2. **Blind-metric regression** — the logged dither rebuilt synthetically: `path_error` ≈ 0 while
   `composite_score` fails on completion *and* dither ratio.
3. **Tuner repeatability** — interior optimum + σ = 73 µm noise over 20 seeds: winner within one step,
   never on a grid endpoint; pure-noise objective changes nothing; store untouched until Apply.
4. **Bench ≡ print** config equality + structural field coverage.

Real-hardware order: verify `VS` direction on a low safe path → measure dead time → reliability at old
settings → `hold_speed` corners-only → bench-vs-print `cross_um` overlay → tune three times and
compare → re-run ZN → robustness matrix → real velocity print.

## Issues & Decisions

- **2026-07-28 — pure lead predictor over `KalmanMotionController`.** The existing Kalman controller
  hard-imports numpy and inverts a 6×6 per tick (breaking `VelocityControl`'s math-only purity in a
  25 Hz loop shared by a Qt worker and the print thread), is time-parametrised against an Nx7
  trajectory when everything here is arc-length, and carries its own gain/lookahead/rate-limiter that
  would fight the pursuit law. We need one number — `v̂` — from a µm-quantised 25 Hz stream where its
  acceleration states are essentially unobservable. Kalman noted as the fallback.
- **2026-07-28 — Hooke–Jeeves over Nelder–Mead.** NM's reflect/contract steps are driven by the rank
  order of noisy values; with σ ≫ signal it collapses onto a noise artefact and has no natural
  "step is below the noise, stop" rule. Compass search moves only on better-than-noise-band results
  and its step-shrink schedule gives a principled stopping rule.
- **2026-07-28 — a newly measured `velocity_dead_time_s`, not the `by_phase` intercept OR slope.** The
  intercept is an optically-measured *settle* time polluted by a single-point 0.577 s fit and two
  impossible negatives; the slope is seconds *per segment* — a unit error if substituted. Measure the
  quantity the cap actually needs, by step response.
- **2026-07-28 — graded failure penalties (`1000 + penalty`) rather than `inf`.** Coordinate descent
  compares with `<`; a flat `inf` across an all-failing grid cannot move, and the current tuning is
  itself in a failing region.
- **2026-07-28 — reuse `PrintExecutionLogger` with a new directory.** Diffability comes from the
  schema, not the path; a separate `logs/challenge/` keeps a 120-run tune session from drowning
  `logs/prints/`, which is documented as one-file-per-print. `sample_hz=0` is load-bearing: the
  background sampler reads a cached position while the bench has suspended the poller.
- **2026-07-28 — optical/bead scoring deferred** (operator decision, encoder-only for now). Scan
  primitives exist and `scikit-image` is already a dependency, but `VisionDetector` has no
  line/ridge/skeleton code and it is the only part needing the needle down. If picked up later:
  normal-profile sampling along the ideal path (not skeletonisation), and build the scoring mosaic
  with registration **off** so tile placement cannot absorb the error being measured.

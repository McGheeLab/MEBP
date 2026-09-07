# MEBP v7.21.2 — One XY Calibration, and it actually completes

## Objective

Operator: *"the one-click xy calibration is not working correctly. 1) remove all of the
single setting buttons like sweep or xy-speed etc 2) add a single option called
xy-calibration that does (a) communications speed test for xy motion with polling (b)
deadtime (c) xy speed test (d) auto tune with gradient descent for a star print with 2mm
feature size. do not do a robust sampeling start with a grid of variable points and find
the best, the user selects the variable grid size as very fine to very coarse. we dont
want this calibration process to take more than 3 minutes. The user will estimate the top
speed of the xy-stage in hardware setup. After calibration we need to override all
settings.json for anything related to xy top speed. All surfaces that use the top speed
should by updated accordingly."*

Then: *"ensure the quick print has the ability to utilize these settings after
calibration. also ensure that these settings are persistant for this setup."*

---

## Root cause — an unbreakable loop

v7.21.1 ("XY max speed is authoritative") correctly stopped the timing worker silently
overwriting the operator's declared top speed and pointed Hardware Setup at
`settings.json`. **It never repointed the readers of the old home.**

`PrintTimingCalibrationStore.set_xy_max_speed_um_s()` was left with **zero production
callers**, and this rig's `config/hardware/ME3B_01/print_timing_calibration.json` has no
`xy_max_speed_um_s` key, so `get_xy_max_speed_um_s()` returns `None`. Seven readers
depend on it:

1. `XYAutoCalibration.measured_from_store` → `top_speed_um_s = 0` → `is_complete()` False
   → the one-click run aborted at the derive step with *"Still need: top speed — run
   'Measure top speed' on the Timing Calibration page, then retry."* **That button now
   writes `settings.json`, not the store, so retrying always landed in the same place.**
2. `XYStageModel.StageCharacteristics.from_store` → incomplete. Worse than a refusal:
   `XYStageModel.command_velocity` clamps only `if top > 0`, so a simulation against an
   empty store models an **unclamped, infinitely fast stage**.
3. `stamp_print_settings` stamped `xy_max_speed_um_s = 0.0` onto every print job.

Two further defects made the rest of the request necessary:

- **The one-click never measured the top speed** — it only read it. The measurement lived
  inside the GUI page (`_run_top_speed`), tangled with the motion detector and plot
  widgets, and never called `check_fits`, so a clamped sweep silently reported a **low**
  top speed (which then makes every commanded mm/s run proportionally **fast**).
- **The auto-tune could not fit 3 minutes and optimised the wrong thing.** It drove every
  candidate on hardware (velocity mode = 23 runs ≈ 8–22 min) and minimised
  `path_error()["rms_um"]` — the one-sided, untimed metric its own docstring forbids as an
  auto-tune objective, under which a crawling or early-stopped run scores *best*. The
  store's `lookahead_mm: 0.2` and `corner_speed_factor: 0.2` were both that descent's
  **grid minimum** — its fingerprint.

---

## Decisions (AskUserQuestion ×6)

| Decision | Choice |
|---|---|
| Where candidates are scored | **Simulation** for the whole grid; only the winner is driven on the stage |
| Home of the single control | **Workflows → XY↔ZP Timing Calibration**; the Challenge dialog becomes a bench diagnostic |
| Measured vs typed top speed | **Overwrite everywhere, automatically** (with an `r²`/3× sanity gate) |
| Grid axes | `lookahead_mm` × `corner_speed_factor`, coarseness = points per axis |
| Tune target | **Speed is an OUTPUT** — find the fastest that holds the element |
| The timing store | **Re-armed** — write all five homes together |

---

## 🔴 The finding that decides the grid

A design review concluded *"no `(lookahead, corner_factor)` pair holds a 2 mm star to
30 µm — it is geometry, not tuning."* **That was wrong**, and the reason is the whole
design. It bracketed the lookahead at `[la0/4, 2·la0]` around the closed-form value.
Measured head to head on ME3B_01's real dynamics:

| target | `la0` | axis `[la0/4 … 2·la0]` | axis `[corner_budget … 2·la0]` |
|---|---|---|---|
| 0.6 mm/s | 0.252 | p95 14.9 µm PASS | p95 **12.6 µm** PASS |
| 1.0 mm/s | 0.419 | p95 19.6 µm PASS | p95 **13.1 µm** PASS |
| 2.0 mm/s | 0.838 | p95 40.2 µm **FAIL** | p95 **15.3 µm** **PASS** |

`derive_settings` sizes the lookahead so the target SPEED is *permitted*; the corner
budget sizes it for *accuracy*. On a 2 mm star the tips (142° turns) tolerate 0.040 mm
against a closed-form 0.838 mm — 21× apart. **The floor is
`corner_budget_lookahead_mm(sharpest turn, element/1.6)`**, and
`test_the_la0_over_4_floor_would_have_missed_it` pins the contrast so the straddle test
cannot pass vacuously.

## 🔴 Three silently-inert-axis traps

1. **The simulator's tuning keys are not the store's.** `XYPathSimulator` uses
   `lookahead`/`corner_factor`; the store uses `lookahead_mm`/`corner_speed_factor`.
   `resolve_for` does `p.update(tuning)` then reads `p["lookahead"]`, so a store-named key
   is **silently ignored** and every cell scores identically. My first sweep hit exactly
   this and produced a flat, meaningless grid. All conversion goes through
   `tuning_from_store_keys`.
2. **`min_lookahead_frac` re-raises every candidate.** `resolve_control` does
   `la = max(la, mlf · speed · L)`, and `derive_settings` sets `mlf` precisely so the
   runtime re-derives `la0` — so inheriting it puts EVERY grid candidate back at `la0`.
   `hold_speed` similarly pins `speed_cap` to the request, defeating the dead-time cap a
   small lookahead is supposed to impose. Both are neutralised per candidate
   (`candidate_values`) and **re-derived from the winner** (`finalise_values`) so what is
   persisted reproduces what was simulated.
3. **The requested speed is not the speed that runs.** At the winning point a requested
   3.0 mm/s runs at **0.43 mm/s**, `cap_reason="dead_time"`. Everything reported quotes
   `resolve_for(...)["speed_cap_mm_s"]`.

**The sweep runs from the LARGEST lookahead down.** The cap is monotonic in the
lookahead, so the first passing row already holds the fastest passing candidate — not a
heuristic, the optimum under this objective — and it is also far cheaper, because
simulation cost is proportional to path DURATION, so the fast candidates examined first
are the ~20 ms ones and the ~160 ms ones are only reached when nothing passes.

---

## Files Modified

| File | Change |
|---|---|
| `SupportClasses/XYTopSpeed.py` | **NEW** — GUI-free top-speed sweep + `EncoderMotion`, extracted from the page; adds the `check_fits` bound the GUI version never had, works in absolute stage µm, alternates direction so the return leg is the next measurement |
| `SupportClasses/XYCalibrationRun.py` | **NEW** — `CalibrationRequest`/`StepProgress`/`GridCandidate`/`GridResult`/`CalibrationResult`, `build_grid`, `score_grid`, `candidate_values`, `finalise_values`, `commit_top_speed`, `verify_on_hardware`, `persist`, `run_calibration` |
| `SupportClasses/XYDeadTime.py` | `_fit_line` → public `fit_line` (alias kept) so the sweep shares one least-squares |
| `SupportClasses/StageController.py` | `PositionPoller` suspend **refcounted** (`_suspend_depth`, `_suspended` now a property so every existing reader is unchanged); `stop_evt` on `measure_control_loop_rate` |
| `gui/pages/workflows/timing_calibration_workflow.py` | one **Run XY Calibration** button + grid combo + progress/step/clock; deleted `_start_btn`/`_speed_btn`/`_apply_speed_btn`/`_comms_btn`, the settle sweep `_run` and `_write_jsonl`; detector + bench + new targets moved into the ⚙ popout |
| `gui/dialogs/xy_challenge_dialog.py` | stripped to **Run + Geometry panel + Stop**; eight buttons and the launch dispatch removed |
| `gui/pages/hardware/control_panel.py` | `_write_axis_max("xy")` delegates to `commit_top_speed` |
| `gui/pages/hardware/stage_panel.py` | NEW `refresh_speed_limits()`; the hint now states **provenance** (measured vs typed) |
| `gui/app.py` | `_stage_panel` added to the speed-limit fan-out |
| `gui/pages/workflows_mode.py` | NEW `refresh_speed_limits()` forwarder so workflow pages get the broadcast |
| `gui/pages/workflows/quick_print_workflow.py` | NEW `refresh_speed_limits()` — re-anchors the speed cap and clears the "not characterised" warning live |
| `gui/pages/hardware/device_profile.py` | an undeclared profile now **clears** `xy_max_speed_um_s` instead of skipping it |

---

## Quick Print + persistence (the operator's follow-up)

- **Utilises the settings**: `_build_settings` already calls
  `AC.stamp_print_settings(settings, get_store())` on **every** print, so the next print
  reads the freshly persisted tuning straight out of the store — no restart, no extra
  wiring. The new `refresh_speed_limits()` re-syncs what is *displayed* (the top-speed cap
  and the "⚠ Stage motion not characterised" warning, which was permanently stuck on
  because home D was empty).
- **Persistent for this setup**: the tuning lives in the **per-machine**
  `config/hardware/<machine-id>/print_timing_calibration.json` (via `resolve_machine_path`),
  and the top speed additionally in `settings.json` (`safety_limits.max_xy_speed` +
  `device_profile.xy_max_speed_um_s`) and therefore in the saved device profile.

---

## Testing Notes

**NEW `tests/test_v7212_one_click_xy_calibration.py` — 25, green in 8.9 s.** Built on this
rig's own measured dynamics (dead time 81.7 ms, τ 37.8 ms, loop 58.0 ms, top speed
6.036 mm/s) so the tests describe a stage that exists.

- `test_empty_store_cannot_derive_then_commit_unsticks_it` — the operator's bug, as an
  executable claim.
- `test_an_empty_store_models_an_unclamped_stage` — guard the guard: shows *why* the store
  write matters (an uncharacterised model accepts 500 mm/s).
- `test_the_la0_over_4_floor_would_have_missed_it` — the grid-floor contrast above.
- `test_lookahead_changes_the_result` / `test_store_named_keys_are_ignored_by_the_simulator`
  — the inert-axis traps.
- `test_all_homes_and_the_store_goes_first` — asserts the call **sequence**, not just the
  end state.
- `test_ranking_prefers_the_fastest_passing_candidate` — the slower candidate has the
  BETTER deviation and must still lose, or the tune reproduces "just bring the velocity
  down".

**Mutation matrix — 6/6 CAUGHT**, anchors verified against live source, all sources
restored byte-identical: store write dropped (2 failures) · lookahead floor back to
`la0/4` (1) · `min_lookahead_frac` un-neutralised (4) · ranking by lowest deviation (1) ·
`_write_axis_max` reverted to safety-only (1) · poller suspend back to a plain bool (1).

**Regression:** 134 green (v7211 · common-axis-speed-source · axis-max-speed-inputs ·
jog-navigation · device-page-layout · suite-hygiene · the new suite · print-timing-
calibration) · 193 green (xy-auto-calibration · xy-dead-time · xy-path-simulator ·
xy-stage-model · xy-challenge · xy-challenge-metrics · xy-feed-plan · xy-speed-conversion)
· 162 green (z-retract · no-descent-before-arrival · jog-travel-off-gui-thread ·
section-promotion · workflow-settings-popout · quick-print-travel-split) · plus a
`gui.app` import smoke and offscreen builds of the **real** timing page and the **real**
challenge dialog.

**10 pre-existing failures PROVED not ours** in a `git worktree` at committed HEAD (never
`git stash` in this repo): the 9 documented `_note_move_estimate_xy` errors in
`test_v75x_xy_envelope_absolute`, and
`test_v76_two_param_and_overlays::test_no_warning_when_within_limits`. Both reproduce
identically at HEAD.

**Test contract changes (deleted, not renamed):** `TestWorkerSweep` (the settle-delay
sweep is retired — the purpose-measured dead time supersedes the legacy `by_phase` model,
whose ME3B_01 entries included two physically impossible negatives) and
`test_apply_button_is_what_makes_it_authoritative` (there is no Apply step any more).
`test_measures_and_reports_without_writing_anything` became
`test_measures_and_reaches_the_timing_store` — the store is a legitimate home again.

---

## Issues & Decisions

**🐞 A test-isolation defect I introduced and caught.** Routing
`control_panel._write_axis_max` through `commit_top_speed` means a GUI edit now writes the
per-machine timing store — and `get_store()` is a process-wide singleton pointing at the
**real rig's calibration file**. `test_v75x_axis_max_speed_inputs` promptly wrote
`xy_max_speed_um_s: 40000.0` into `config/hardware/ME3B_01/print_timing_calibration.json`,
and every later test in the process then saw a machine that had been "declared" (three
`test_v7211` failures). The file was restored (the real measurements — dead time, τ, loop
— were untouched) and the class now takes `use_temp_store` in `setUp`. Worth recording:
the production change is correct; it is the *blast radius* of a singleton store that
changed.

**⚠ The heredoc trap.** Backslash escapes inside a quoted heredoc were collapsed by this
shell, turning `\n` inside Qt tooltip strings into real newlines and breaking the file;
a too-aggressive repair script then over-merged and the file had to be restored from HEAD.
Edit escape-heavy source with the file tools, not shell heredocs. (The v7.21.1 additions
to that file were superseded by this change, so nothing was lost.)

**⚠ Supersedes a recorded decision.** `MEBP_v7211_XY_MAX_SPEED_AUTHORITATIVE.md` chose
*"measurement reports + explicit Apply"*. The operator's new instruction is auto-commit;
the `r²`/3× gate in `commit_top_speed` replaces that protection. That doc's "Apply to
hardware config" button no longer exists.

**Deliberately kept:** the Challenge dialog's now-unreachable worker methods
(`_worker_tune`, `_worker_pid_analytic`, …). Their buttons are gone — which is what was
asked — and three existing tests drive them directly; deleting them would churn a passing
suite for no operator-visible gain.

**Disclosed capability deletion:** nothing writes `by_phase` any more. `get_phase_lag_s()`
still returns stored historical values and `effective_dead_time_s()` keeps its fallback,
so nothing breaks, but the legacy phase-lag model is now frozen.

---

## Measured time budget (ME3B_01)

| Step | Typical | Conservative |
|---|---|---|
| centre / comms / dead time | 6 / 3.5 / 7 s | 15 / 4 / 15 s |
| top speed | 22 s | 35 s |
| commit + derive | 0.5 s | 1 s |
| **grid (simulation)** | **2.1 s** (medium) | **10.2 s** (very fine) |
| verify (1 shape, hardware) | 8 s | 12 s |
| persist + retract | 4 s | 6 s |
| **TOTAL** | **≈ 53 s** | **≈ 98 s** |

Measured grid times, all coarseness levels, on the real dynamics: very coarse 1.2 s ·
coarse 1.4 s · medium 2.1 s · fine 5.5 s · very fine 10.2 s — every level passes the
30 µm element on a 2 mm star. Budget enforcement degrades rather than overruns (skip the
verify, and say so).

---

## Needs HW verification on ME3B_01, IN ORDER

1. Hardware Setup → Device → XY Stage Calibration → **Max speed ≈ 6000 µm/s**, Save.
   Nothing below is trusted until the log reads `max=6000`, not `max=50000`.
2. Workflows → XY↔ZP Timing Calibration → **Run XY Calibration** (Medium). It must
   complete **without asking you to press another button**, in **under 3 minutes**.
3. `print_timing_calibration.json` gains `xy_max_speed_um_s`; both `settings.json` keys and
   the Hardware Setup field show the measured value **live, without a restart** (the field
   hint should read *"Measured by XY Calibration <date>"*).
4. Quick Print's *"Stage motion not characterised"* warning is **gone**, and its top-speed
   spin re-anchors to the measured maximum.
5. Print the 2 mm star and compare the bead against the reported p95 and the reported
   achievable speed (`cap_reason` names what binds).
6. Re-run at **Very fine**: same winner within a grid step, still under 3 minutes.
7. **Abort at each step**: the stage stops, the needle ends at Safe Z, nothing is
   persisted.
8. Confirm the ⚙ popout still reaches the XY Printing Challenge bench, and that the bench
   has only Run / Geometry panel / Stop.

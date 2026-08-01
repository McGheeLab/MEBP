# MEBP v7.6 — Quick Print: two driving parameters, feed-plan printing, overlays, live position, hard abort

## Objective

The feed planner (`SupportClasses/XYFeedPlan.py`) got **18/18 geometry-panel shapes under the
30 µm element on real hardware** where one fixed follower tuning managed 0/18 — but the accuracy
is bought with **time** (a full stop on every sharp corner, curvature-limited speed through
curves). The operator asked for that trade to become explicit and user-controlled:

> *"The user should set two parameters: the resolution they want, and the top speed of the XY
> stage for the print. Everything should calculate based on these parameters."*

plus, from the same message:

- **warnings** when the needle's pump-flow limit makes a speed unattainable, or the requested
  resolution is below what the hardware can hold;
- all of it **wired into the Quick Print simulation** of the stage path;
- **optional overlays** on the print path — XY speed, flow rate, error, time;
- **real-time needle position** in the print path that never stalls on axis motion or camera work;
- **abort** must knock out all motion instantly and retract the needle to a safe position.

**Operator decisions (AskUserQuestion):** the feed plan **drives the real velocity-mode print**
(not just the preview); the speed parameter is an **absolute top speed in mm/s** (replacing the
"% of max" knob, with saved profiles migrated).

## "Before" state (verified by exploration, not assumed)

| Area | Before |
|---|---|
| Speed knob | `speed_pct` (% of the measured max); flow and speed both scaled off the same % |
| Flow limit | `_flow_limited_xy_max_mm_s` clamped **silently** — no warning, ever |
| Resolution | Not a print parameter at all (`PrintSettings` had no resolution field) |
| Corner accuracy | Pure pursuit cut every corner by ≈0.4·lookahead (~220 µm measured); 180° reversals stalled forever |
| Live position | PRINT_PATH **suspends the poller**; the loop's own 25–31 Hz reads went only to the JSONL; `cached=False` never back-filled the cache → the trajectory monitor **froze for the whole print** |
| Abort | Flags only: **no XY stop, no pump stop**; a fire-and-forget `move_z_absolute` that was **not raise-only** (could descend) issued **on the GUI thread**, able to block 10–180 s behind an in-flight M400; `flush_moves` / `_wait_pump_move_complete` / `wait_for_z_arrival` all abort-unaware |

## Files Modified

| File | Change |
|---|---|
| `SupportClasses/ZPStage.py` | `quickstop()` (M410, bounded, contended raw-write path), `resync_position()`, `flush_moves(abort_event=)`, EMERGENCY_PARSER capability parsed from M115 at connect |
| `SupportClasses/StageController.py` | `abort_all_motion(reason)`; `abort_event=` threaded through `wait_for_z_arrival` / `_wait_pump_move_complete` / `_finish_pump_submove` / `move_pump_uL` / `_retract_z_slow_then_fast` / `ensure_retracted_to` / `safe_travel_to`; **`PositionPoller.note_xy` / `note_zp`** + back-fill from `get_xy_position(cached=False)` |
| `SupportClasses/PrintManager.py` | `abort()` rework + `_abort_worker` + `_ABORT_UNWIND_S`; abort-aware wrappers `_flush_moves` / `_wait_z` / `_pump_uL`; **`feed_plan_enabled` / `feed_plan_element_um` / `feed_plan_corner_split_deg`** on `PrintSettings`; `_feed_plan_char`, `_execute_print_path_feed_plan`, `_run_plan_section`, `_PLAN_STOP_DWELL_S` |
| `SupportClasses/PickAndPlaceManager.py` | `_settled_pump_move(abort_event=)`; executor `_pump_move` / `_safe_travel` route all 29 pump sites + travel through this operation's abort flag |
| `SupportClasses/XYFeedPlan.py` | `min_attainable_resolution_um`; `_tuning_for` → public `tuning_for` (+alias); new `section_end_taper` |
| `SupportClasses/XYAutoCalibration.py` | `stamp_print_settings` also stamps `vel_tuning["tau_s"]` so the JOB carries a complete `StageCharacteristics` |
| `gui/dialogs/workflow_settings_dialog.py` | `migrate=` hook applied in `apply()` (covers last-used, profiles, imports) |
| `gui/pages/workflows/quick_print_workflow.py` | `_top_speed_spin` + `_resolution_spin` (replacing `_speed_pct_spin`); `_top_speed_mm_s` / `_resolution_um` / `_migrate_legacy_settings` / `_update_top_speed_cap`; reworked `_resolved_print_kinematics`; `_append_limit_warnings`; feed-plan stamp; `_kick_prediction` → `build_plan`+`simulate_plan` + overlay channels; `_overlay_channels` (pure); overlay pills + `_apply_overlay_mode`; `_set_print_live` + 10 Hz `_live_pos_timer`; `_kick_abort_all_motion` in all three abort routes |
| `gui/widgets/print_trajectory_monitor.py` | `set_overlay` / `overlay_mode`, 5-bucket ramp, per-point coloured predicted path, legend in the readout band |
| `gui/widgets/camera_widget.py`, `camera_feed_view.py` | `set_throttled(on)` — halves the GUI-thread grab rate during a print |

## Implementation Steps

### B — Hard abort (done first: the safety net the new corner stops rely on)
- [x] B1 `ZPStage.quickstop(lock_timeout_s=0.25)` — M410. Lock acquired → `M410\n`. **Contended**
      (an in-flight `flush_moves` M400 can hold `_serial_lock` 10–180 s) → raw `\nM410\n` WITHOUT
      the lock: the holder is in its *read* loop so the write side is idle, the leading newline
      terminates any partial line, and the OS queues each `write()` whole. **Self-healing**: with
      EMERGENCY_PARSER the M410 stops motion → the M400 completes → the blocked wait releases the
      lock, so the abort un-sticks the very wait that was blocking it.
- [x] B1b `resync_position()` — double `M114` (first absorbs the stray ok from the unanswered
      M410, second is clean). M410 loses position accuracy, so this is mandatory.
- [x] B1c EMERGENCY_PARSER parsed from the M115 capability report at connect →
      `ZPStage.emergency_parser`, logged by every abort so an abort is self-describing.
- [x] B2 `StageController.abort_all_motion(reason)` — Prior `I` + `VS 0,0` + `quickstop` +
      `resync`; any thread, never raises, each step independently guarded, ~2.5 s bounded.
- [x] B3 `abort_event=` (default None = byte-identical) through every blocking wait; passed at the
      print's confirm/barrier/pump sites and the pick&place executor. **Never** passed by
      `_retract_to_safe_z` — the safety retract of record must always complete.
- [x] B4 `PrintManager.abort()` → quick bookkeeping + a daemon `_abort_worker`
      (`abort_all_motion` → join the print thread ≤20 s → backstop `_retract_to_safe_z`).
      **Deleted** the GUI-thread `move_z_absolute` (descend hazard + UI freeze).
- [x] B5 `_kick_abort_all_motion()` on all three Quick Print abort routes.

### A — FeedPlan drives the real print
- [x] A1 `PrintSettings.feed_plan_enabled / feed_plan_element_um / feed_plan_corner_split_deg`
      (defaults = legacy). Top speed **reuses `print_speed_mm_s`** — a second speed field would
      desync `vol_per_mm = flow/print_speed` from the plan target.
- [x] A2 `stamp_print_settings` stamps `tau_s` → the job carries a complete `StageCharacteristics`,
      so the plan is reproducible from the job with no execute-time store read.
- [x] A3/A4 `_feed_plan_char` gate + delegation at the top of `_execute_print_path_velocity`;
      returns False **with no motion issued** → falls through to the legacy loop (`feed_plan_fallback`
      logged). Flag off = the plan method is never even consulted.
- [x] A5 `_execute_print_path_feed_plan`: `build_plan` at execute time, `path_start` gains
      `feed_plan/n_sections/n_stops/plan_est_s/element_um/budget_um`, per-section `plan_section`
      events, inter-section `VS 0,0` + 0.3 s dwell in 50 ms abort-checked chunks, tiny-section skip
      guard, and **GLOBAL pump bookkeeping** (`base_s` + monotone global `s`) so the deposited
      volume is `vol_per_mm × total` regardless of where the splits fall, with nothing deposited
      during a stop and an exactness top-up at the end.
- [x] A6 `_run_plan_section` — the legacy tick body parametrized per section, with the **lag-aware
      end taper** and every legacy guard retained (abort per tick, ZP drop, stale, snap bound,
      runaway → RuntimeError, VS clamp, `vel_sample` with `sec=`).
- [x] A7 Quick Print stamps the fields after `stamp_print_settings`.

### C — The two parameters
- [x] C1 `XYFeedPlan.min_attainable_resolution_um` = arrive tolerance + post-arrival coast +
      encoder quantum. ⚠ **Deliberately NOT scaled by `PREDICTION_HEADROOM`** — that factor
      corrects the *simulator's* optimism, while every term here is measured directly. Including
      it gave a 36.8 µm floor, which would have warned that 30 µm is unattainable **on the machine
      that demonstrably held it** (panel worst cell 27 µm). Correct value: **23.0 µm**.
- [x] C2 `_top_speed_spin` (mm/s) + `_resolution_spin` (µm) replace `speed_pct`; both refresh the
      status, summary AND the prediction.
- [x] C3 `_migrate_legacy_settings` via the new dialog `migrate=` hook — `speed_pct` → mm/s.
      Verified against the real saved profile: `speed_pct 2.0` → **0.12 mm/s**, i.e. faithful (and
      a reminder of how throttled this machine was).
- [x] C4 `_resolved_print_kinematics` = `min(top_speed, stage max, flow-limited max)`; flow
      FOLLOWS the speed, so volume-per-mm is speed-invariant (locked by test).
- [x] C5 Warnings (a) flow ceiling (b) stage max (c) resolution floor (d) uncharacterised machine
      (e) **time estimate** `resolution 30 µm → est 42 s (12 corner stops)` — the trade, priced.
- [x] C6 `_kick_prediction` now runs `build_plan` + `simulate_plan` (the same sections the print
      executes) and produces the overlay channels.

### D — Overlays
- [x] D1 `set_overlay(mode, per_segment_values, unit, vmin, vmax)` — colours the PREDICTED path
      only (planned blue / executed green untouched); mismatched counts fall back to the dashed
      render; a new prediction clears stale values; legend in the readout band.
- [x] D2 `_overlay_channels` (pure): speed = Δs/Δt, flow = speed × vol_per_mm, error = |cross|
      via bisect into `cross_profile`, time = global print clock across segments.
- [x] D3/D4 4-arg `predicted` bridge signal + exclusive pills (None / XY speed / Flow / Error / Time).

### E — Never-stalling live position
- [x] E1 `PositionPoller.note_xy` + back-fill from `get_xy_position(cached=False)`. The print
      loop's own 25–31 Hz reads now keep the cache live **through the poller suspension**, so every
      cached reader (status bar, monitor, context panels) unfreezes with no new wiring. The
      odometer and ZP liveness stay poll-thread-only, so travel cannot double-count.
- [x] E2 `_set_print_live(on)` + a 10 Hz `_live_pos_timer` during prints (the shared 300 ms app
      tick is too coarse to draw a moving needle).
- [x] E4 tier 1 `set_throttled` halves the GUI-thread camera grab rate during a print.
- [ ] E4 tier 2 (deferred, gated on backend thread-affinity verification): move the blocking
      `backend.read()` to a per-widget reader thread. Also note: **open-loop mode does no position
      reads at all**, so the marker still freezes in that mode — a ~2 Hz `cached=False` read in the
      open-loop streamer would fix it via E1 automatically.

## Testing Notes

| Suite | Tests | Covers |
|---|---|---|
| `tests/test_v76_hard_abort.py` | 28 | abort-aware `flush_moves` (<0.5 s vs waiting out 30 s); quickstop locked vs contended raw-write (no deadlock, `\nM410\n`); `abort_all_motion` order + never-raises + XY-only/ZP-only rigs; `abort()` returns <0.2 s with the print thread parked in a blocking wait; **descend-hazard regression** (no downward Z command on abort); backstop retract only when the thread doesn't unwind; all three Quick Print routes; PickPlace flag forwarding |
| `tests/test_v76_feed_plan_print.py` | 25 | fallbacks (uncharacterised → legacy, no motion); flag-off never consults the plan; L-path → 2 sections + stops; reversal completes; **volume conservation** (square, monotone deposits, tiny-section skip); abort mid-section / in dwell / before start; ZP drop; `tuning_for`/`section_end_taper` refactor keeps the simulation identical; resolution floor below the proven element; `tau_s` stamp |
| `tests/test_v76_two_param_and_overlays.py` | 44 | kinematics min() + speed-invariant bead; all warnings incl. the floor (unconditional, via a stamped temp store); overlay channels; monitor overlay set/clear/range/mismatch/paint; pill routing + gen guard; profile migration + dialog hook; **cache back-fill** (incl. odometer untouched + thread safety); live timer; camera throttle |

Regression: 245 green across challenge / velocity-follow / open-loop / confirmed / auto-calibration
/ pick-place / cell / print-logging / simple-PM / needle-flow, plus 129 across ZP / jog / safe-Z /
seam / stress / jog-navigation, plus 163 across the v7.6 + simulator suites.

**One pre-existing failure, confirmed not mine:**
`test_v75x_zp_auto_reconnect_and_fast_z::test_no_z_max_keeps_defaults` (expects 200, gets 500 from
`get_max_z_feedrate_mm_min`). Present in HEAD; my diff has **zero** hits on that symbol; already
documented in CLAUDE.md's `MEBP_v75x_JOG_TRAVEL_OFF_GUI_THREAD` row.

## HARDWARE VERIFICATION — PERFORMED 2026-07-29 (no pumps; Z ±5 mm substituted)

Operator authorised a full real-hardware test with the pumps excluded ("instead of the pump just
control Z 5 mm up and down") and confirmed the stage/Z were positioned safely for that travel.
Run staged, escalating only after each stage verified. Both boards live: Prior XY on **COM6**,
Marlin ZP on **COM4**.

**⚠ Machine-specific safety facts established first (they changed how the test was run):**
- `axis_map` is `P1→X, P2→Y, P3→E` — **Marlin X/Y/E ARE THE PUMPS.** The test therefore commands
  **Marlin Z only**; a stray `G0 X/Y` on that board would have driven a pump.
- Z envelope (zero-ref) 0–60 mm; safe Z 44.0; plate bottom 21.13; **but the calibrated max-retraction
  reference is 45.0** — only ~1 mm of headroom above the start. So the ±5 mm excursion was run
  **DOWN-then-back-up** (44.0 → 39.0 → 44.0, ~17.9 mm of plate clearance) rather than up-first,
  which would have pushed past the max-retraction reference toward a possible hard stop.
- The board did **not** lose position on connect (`_board_reset_detected: False`; raw −59.14 =
  zero-ref 44.0, exactly the saved `zp_last_position`). Position was re-declared anyway (G92, no
  motion) so the soft limits operated in the true frame.

### ⚠ FIRST ATTEMPT WAS INVALID — axis map not applied (my error, disclosed in full)

The first run applied the safety limits and the Z convention but **never called
`apply_device_settings()`**, which is what pushes the per-machine `axis_map`. The live ZPStage
therefore kept its DEFAULT map (`Z→X, P1→Y, P2→Z, P3→E`) instead of this machine's
(`Z→Z, P1→X, …`), so every "Z" command was routed to Marlin **X — which is pump P1**.

What actually happened in that run:
- **The needle Z never moved at all** (Marlin Z stayed at raw −59.14 = zero-ref 44.0 throughout),
  so the "Z holds during the path" result was trivially true and told us nothing.
- **Pump P1 was moved** — the thing the test was explicitly told not to do. The moves were
  relative and net-zero (−5 then +5, twice), so the plunger returned to its starting position
  each time and no fluid was net-displaced, and it stayed inside P1's −30…0 envelope.
- P1's **position counter** was corrupted to −59.14 (by a G92 the code believed was addressing Z)
  and was **restored to its pre-session −19.7 with a software-only G92, no motion**. All four axes
  then matched the saved `zp_last_position` exactly (Z 44.0, P1 −19.7, P2 0.0, P3 0.0).
- The readbacks all looked perfect (0.000 mm errors) because the write and the read went through
  the same wrong map — **self-consistent and wrong**, which is exactly why it wasn't obvious.
- The XY star result from that run was unaffected (the Prior stage is a separate controller).

**Lesson, now enforced in the harness:** four guards run *before any motion* — push
`apply_device_settings()`, assert the live map equals the profile map, assert the Marlin letter that
logical Z maps to is not any pump's letter, and snapshot every pump counter to assert zero drift at
the end. All results below are from the **corrected** run
(`logs/hwtest/star_run_correct_20260729_114303.json`).

### ✅ HW-1 ANSWERED — `EMERGENCY_PARSER: True`

The gating question for the whole abort design. This SKR Mini E3 V3 **does** report
`Cap:EMERGENCY_PARSER:1`, so `M410` executes as the bytes arrive: `quickstop()` is genuinely
immediate, the contended raw-write path's self-healing argument holds, and the bounded-latency
claim is real rather than conditional.

### Z axis (the pump substitute), staged 0.5 → 2 → 5 mm

| commanded | measured down | measured up | end position |
|---|---|---|---|
| 0.5 mm | −0.500 | +0.500 | 44.000 |
| 2.0 mm | −2.000 | +2.000 | 44.000 |
| 5.0 mm | −5.000 | +5.000 | 44.000 |

Every stage exact to **0.000 mm**, both directions, returning to the start height.
Trace: `logs/hwtest/z_probe_20260729_113009.json`.

### Star print — print object → feed plan → hardware

A real print object was created through the normal path (`save_trajectory_as_print_object` →
**`HW_Test_Star_8mm`**, `csv_import` + sibling CSV, present in the Print Library), then loaded back
and driven at 3 mm/s with a 30 µm element. Plan: **10 sections, 9 corner stops**.

| check | result |
|---|---|
| XY tracks the star | **p95 25 µm**, rms 11 µm, max 37 µm — **PASS** vs the 30 µm element |
| path completed | completion **1.000**, `arrived`, no runaway/stall |
| **Z holds during the path** | range **0.0 µm** over 206 samples across 21 s |
| pump substitute 1 | **5.000 mm** (44.000 → 39.000 → 44.000) |
| pump substitute 2 | **5.000 mm** |
| XY still during the Z moves | span **0.001 mm** |
| travel phase | Z held 44.000 while XY moved 4.000 mm (retract-first, no descent) |
| ends at safe Z | **44.000** |

Trace `logs/hwtest/star_run_20260729_113406.json`; figure `logs/hwtest/v76_star_hw_run.png`.
**The axes are cleanly decoupled: Z moves only when commanded and is dead flat through the entire
path, while XY is motionless through every Z excursion.**

### ✅ The v7.6 live-position back-fill confirmed on hardware

The sampler read X/Y from the **poller cache** (`cached=True`) while the follower did the fresh
reads, and the cache swept the full **7.61 mm** of star extent during the path. Before this change
that cache was frozen for the whole PRINT_PATH — this is the monitor-freeze fix, verified on the
real machine rather than in simulation.

### 🔧 One defect found and fixed by the run: the time estimate was 1.7× optimistic

Measured: 28.64 mm at 3 mm/s = 9.5 s of pure motion, but the path took **21.4 s** → **1.31 s per
corner stop** (only ~0.45 s of it is XY standing still; the rest is decel into the 10 µm arrive
tolerance, the inter-section settle, and re-acquiring speed). `build_plan` assumed 0.35 s. Since
this estimate is exactly how the operator prices "finer resolution costs time", it must be honest:
extracted as `XYFeedPlan.STOP_COST_S = 1.3` with the measurement recorded. The same star now
estimates **21.2 s against 21.4 s measured (−1 %)**, and all 42 feed-plan tests still pass.

### ✅ ALL AXES SIMULTANEOUS — re-run after the operator's correction

Operator: *"during a print all axis must move simultaneously."* Correct, and my first two runs got
this wrong: they put the Z excursion **before and after** the path, which is sequential. A print
advances the pump *continuously while XY moves*, so the substitute has to be driven from the same
place the pump is — the follower's **per-Δs deposit hook**.

Re-run with Z emitted from that hook (a triangle across the arc length: 44.0 → 39.0 → 44.0 over the
28.6 mm star, 293 non-blocking Z increments):

| measurement | result |
|---|---|
| **concurrency** | of 237 samples with XY moving, **192 (81 %) had Z moving too** |
| speeds while co-moving | Z **0.79 mm/s** while XY ran **2.27 mm/s** |
| Z follows its commanded profile | \|Z − Z_cmd\| **p50 5 µm · p95 22 µm · max 46 µm** |
| Z excursion | **4.980 mm** of the commanded 5.000 |
| **XY unaffected by the concurrent Z** | **p95 26 µm · rms 11 µm · completion 1.000 · PASS** |
| pumps | drift **0.000** on P1/P2/P3 — untouched |

The remaining 19 % is Z's 0.02 mm increment quantisation plus the triangle apex where Z reverses —
not sequencing. Trace `logs/hwtest/star_simultaneous_20260729_114609.json`; figure
`logs/hwtest/v76_simultaneous_axes.png`.

**The headline:** driving Z concurrently did **not** degrade XY tracking (p95 26 µm vs 27 µm for the
sequential run — inside run-to-run noise), so the two axes are genuinely independent on this
machine and a real print's XY+pump concurrency is safe.

⚠ **One honest caveat about "all axes simultaneously":** the feed plan deliberately **stops XY at
each sharp corner** (9 stops here), and because Z/pump advance is proportional to XY progress, all
axes pause together at those corners. That is the mechanism that buys ≤30 µm accuracy — pure pursuit
cannot turn a sharp corner inside the element. If continuous never-stopping motion matters more than
corner accuracy for a given print, that is the opposite end of the same trade and needs an explicit
"no corner stops" mode (raise `corner_split_deg` above 180 to disable splitting), which would restore
the ~220 µm corner cut. Worth an operator decision; not silently chosen.

### ⭐ END-TO-END QUICK PRINT THROUGH THE PRODUCTION EXECUTOR — 2026-07-29 (closes the gap above)

Operator: *"this should be testing the print with the simulated trajectory that we are making via
quick print … make a quick print and oversee the process so that we can test the code on real
hardware and compare what we think is going to happen with actual performance … instead of using the
pump axis we should use the z axis … at the end, i want a pannel of real vs simulated vs ideal."*

The prior star runs drove `run_plan`/`follow_path` **directly**. This run goes through the whole
production chain and nothing in the harness re-implements it:

- `QuickPrintWorkflowPage._build_settings` — the real settings builder (invoked on a `__new__`
  partial page with stub spins, so the GUI is not booted against live hardware but the *stamping* is
  the production code): resolved kinematics, `stamp_print_settings`, `feed_plan_enabled`,
  `feed_plan_element_um`, prime, `z_up_sign`, `plate_axis_sign`, line-move speeds.
- `build_well_plate_job` → 7 commands (`comment, travel_up, move_xy, move_z, extrude, print_path,
  travel_up`; `return_home=False`).
- `PrintManager.start` → `_execute_loop` → confirmed descent → prime →
  `_execute_print_path_feed_plan` → `_run_plan_section` ×10 → suckback → retract.
- `XYFeedPlan.build_plan` + `simulate_plan` — the SAME prediction Quick Print draws, captured
  **before** the run so the comparison is a prediction, not a postdiction.

**PUMPS NOT USED — one choke point, not a per-call-site patch.** Both pump entry points on the
controller instance are replaced before `start()`: `move_pump_uL` accumulates the signed volume and
issues a NON-BLOCKING `move_z_absolute` (so Z advances *while* XY moves, the pump's own contract),
hard-clamped to `[39.0, 44.0]`; `move_pump_relative` is refused and logged (it was never reached —
`blocked_move_pump_relative: []`). A plunger advances monotonically through a print, so Z descends
monotonically 44.0 → 39.0 across the path and the job's own `TRAVEL_UP` restores 44.0: the requested
5 mm down-and-up, driven by the print's real deposition bookkeeping. **Print Z was set EQUAL to the
travel/safe Z (44.0)** so the needle never descends toward the plate; 39.0 is still 17.9 mm above
the plate bottom. Pre-motion guards as before, plus two new ones: a `_feed_plan_char` pre-check
(**an incomplete stamp silently falls back to the legacy follower — that would not have been a
feed-plan test at all**) and an XY-envelope fit check that refuses to run rather than trace a
clamped edge.

A **dry run on the simulators first** caught two harness faults for free (the simulator's persisted
Z can't reach print height → re-datum with G92; my JSONL parser keyed on `event` when the field is
`ev`, which had made the run look like it logged nothing).

**Result — `logs/hwtest/quickprint_hw_20260729_121442.json`, JSONL
`print_20260729_121416_….jsonl`:** state COMPLETED; `path_start feed_plan=true, n_sections=10,
n_stops=9, plan_est_s=21.2`; all **10 `plan_section` events** with global `base_s_mm` advancing
0 → 25.78; `path_end status=arrived, sections_done=10, s_mm=28.644 == tot_mm`; **0
`feed_plan_fallback`**. Wall **21.7 s vs 21.2 s estimated (+2 %)** — the corrected `STOP_COST_S`
holds on a second, independent path. 156 Z increments, commanded volume 3.9377 µL ≙ 5 mm of Z, **0
clamps**; **pump drift 0.000 on P1/P2/P3**; ended at Z 44.000.

| | simulated | real (executor's own samples) |
|---|---|---|
| p95 deviation | 7 µm | **27 µm** |
| rms | 2 µm | 12 µm |
| max | 9 µm | 36 µm |
| completion | 1.000 | 1.000 |
| verdict vs 30 µm | PASS | **PASS** |

An independent 20 Hz sampler reading the **cached** position agreed at p95 26 µm — the v7.6
back-fill again keeping the GUI's position live through a real `PRINT_PATH`. The executor's own
`cross_um` channel: p50 4.7 · p95 27.5 · max 35.9.

Panel: `logs/hwtest/v76_quickprint_panel.png` (ideal vs simulated vs real; a 0.32 mm corner zoom;
deviation vs arc length; error CDF; Z-as-pump vs time with the commanded-volume axis; the numbers).

**🔎 What the comparison actually reveals — the error is section RESTARTS, not corner cutting.**
Deviation decomposed about the section splits (±0.5 mm):

| | near a section end | mid-section |
|---|---|---|
| simulated | p95 7.7 µm | p95 2.4 µm |
| real | p95 **31.7** µm | p95 **17.8** µm |

The vertex itself is *hit* (that is what the stop buys); each peak is the transient as the next
section spins up, and it decays before the following corner. The real stage also carries a
~6 µm mid-section floor the FOPDT model has no term for.

**⚠ FINDING THAT NEEDS A DECISION — `PREDICTION_HEADROOM = 2.0` is not an upper bound.** Real/sim
p95 here is **3.75×**. Re-deriving the ratio from both stored hardware panels rather than trusting
the earlier summary: the tuning-only panel medians **1.55×** (4 of 17 cells over 2.0), and on the
planned panel the *curved* shapes are consistently optimistic — Circle 10 mm **4.28×**, Star 10 mm
3.26×, Star 5 mm 3.21×, Circle 5 mm 3.18×, Circle 2 mm 3.16× (5 of 18 over 2.0) — while the polygon
shapes are pessimistic (Square/Zigzag/Comb 0.13–0.25×). So today's 3.75× is **consistent with prior
hardware data, not an anomaly**, and 2.0 is a middling figure being used as a safety factor. At the
shipped 2.0 a shape predicted at 16 µm is called PASS for a 30 µm element but could measure ~60 µm.
Raising it (≈4.0 for curved geometry) would require predicted ≤ 7.5 µm to pass a 30 µm element —
safer, but it will flag designs that would in fact print. **Deliberately not changed unilaterally**:
it trades false-pass against false-fail and that is an operator call.

### Still outstanding (needs the pumps, or a deliberate abort drill)

- HW-2 Prior `I` mid-`VS`; HW-3 a live abort drill mid-print (M410 stop latency measured against a
  long move, and the raw-write-during-M400 case) — these deliberately weren't triggered during a
  clean run.
- HW-4 with real ink: corner blobs from ooze during the 0.3 s stops.
- The `PREDICTION_HEADROOM` decision above.
- A run with the real pumps (this run proves the *executor* and the *bookkeeping*; the plunger
  itself, its compliance, and corner ooze are still untested).

### Earlier notes (pre-run)

⚠ Opening the ZP port asserts DTR and can reset the Marlin board; M410/M112 are firmware-level
motion kills.

1. **HW-1 (gates the abort-latency claims):** `M115` → is `Cap:EMERGENCY_PARSER:1` present on this
   SKR Mini E3 V3? Then a long pump G0 → M410 → measure the stop latency; and M410 raw-written
   during an in-flight M400 (the M400's ok should arrive promptly and the port stay sane). Without
   EMERGENCY_PARSER, M410 queues → quickstop degrades to "no early stop" (never worse than before,
   and the software still unwinds fast). The code logs which case applies on every abort.
2. HW-2: Prior `I` mid-`VS` — immediate stop, next command clean.
3. HW-3: abort drill mid-print at print Z — GUI responsive, pumps dead, needle rises **raise-only**
   to travel Z, M114 resync sane. **Pump volume after an abort is indeterminate by design** (a
   move is cut mid-stroke) — the abort logs a warning; re-check syringe fill before the next run.
4. HW-4: supervised FeedPlan print of the geometry-panel shapes; compare the `plan_section` events
   to the bench's 18/18 ≤30 µm; **inspect the section boundaries for corner blobs** (ink can ooze
   during the 0.3 s stop — mitigations would be a shorter dwell or a micro-suckback at splits).
5. HW-5: live monitor tracks the needle through a whole velocity print at 10 Hz with a camera feed
   running; abort mid-print from the GUI.

## Issues & Decisions

- **The resolution floor must not include the prediction headroom.** Caught before it shipped: a
  36.8 µm floor would have flagged the 30 µm element as unattainable on the machine that measured
  1–27 µm at it. The headroom corrects *model* optimism; the floor is built from *measured* terms.
- **A silently-swallowed `NameError` disabled every new warning.** `get_store` wasn't imported at
  module scope and the `except Exception: return warn` hid it — one test skipped rather than
  failing, which nearly let it through. Fixed by a module-level import, a logged (not silent)
  handler, and rewriting the test to use a stamped temp store so the assertion is unconditional.
- **Stage-max warning needs a plausibility gate.** With an unconfigured stage it printed "exceeds
  the measured stage max (0.00 mm/s)". Now requires ≥ 0.1 mm/s to be treated as a measurement.
- **Chunked sleeps must be bounded by count, not the wall clock.** The first abort-aware fallback
  sleep spun ~37 M times under a patched `time.sleep` (the pump suite went 0.16 s → 301 s). The
  legacy single-sleep path is preserved exactly when no abort event is supplied.
- **Global, not per-section, pump bookkeeping.** Section-local arc length resets at every split,
  so tracking it naively would under- or double-deposit at every corner. Locked by a
  volume-conservation test.
- Abort deliberately uses **M410, not M112**: M112 kills Marlin and needs a board reset, so it
  stays reserved for the operator's Escape-key E-stop — the routine abort must leave the board
  alive so the needle can still be retracted.
- Camera reader-thread offload deferred: it needs ToupCam/Andor thread-affinity verification, and
  the cheap throttle plus the cache back-fill already address the observed starvation.

# MEBP v7.5.x — Quick Print: needle-prep + ink-pickup before printing

**Status:** Implemented + tests green (offscreen) — **Needs real-HW
verification on ME3B V1.**

## Objective

Make **Quick Print** behave like a real bioprint run instead of assuming the
needle is already loaded. Pressing **Print** now runs, on a worker thread, the
full sequence:

```
confirm hardware is set up (up-front dialog)
  → (optional) condition the needle  : waste → oil → wash → buffer
  → pick up the print's ink           : safe-travel to the ink's reagent well, aspirate
  → travel to the print well (retracted) → confirm needle position
  → descend to print Z → prime → print → end at safe Z
```

Scope decisions (from the operator):
- **One ink per run** — keep Quick Print's single-object / single-well model;
  the "list" of inks has one entry: the ink the selected pump dispenses.
- **Pickup volume = computed from the print** — `flow × (path_len / print_speed)
  + prime`, × a safety factor; aspirated at the ink well.
- **Ink override = choose the ink for the pump from the library** — a combo of
  *printable* inks (those with a reagent location), defaulting to the pump's
  assigned ink.

## Files Modified

| File | Rationale |
|------|-----------|
| `gui/pages/workflows/_reagent_prep.py` | **New.** Shared free functions (`SERVICE_ROLES`, `service_well_names`, `resolve_service_positions`, `needle_volume_uL`) extracted from the spheroid logic so Quick Print reuses them without coupling. Spheroid page left untouched. |
| `SupportClasses/PickAndPlaceManager.py` | **New** `PickPlaceExecutor.aspirate_ink(well_pos, volume_uL, *, bore, z_mm, rate_uL_s=None)` — safe-Z travel to an ink reagent well + aspirate, abort-aware (same ZP-down guard as `_safe_move_to`). |
| `gui/pages/workflows/quick_print_workflow.py` | Ink row (override combo + readout), prep row (clone of spheroid's), setup-status gate, pickup-volume math, preflight worker orchestration + abort routing. **+ Print-speed % control** (see below). |
| `tests/test_v75x_quick_print_pick_and_place.py` | **New** tests. |
| `CLAUDE.md` | Add a row to the *Existing Update Plans* table. |

No change to `workflows_mode.py` — Quick Print already receives
controller + settings + camera_manager and the calibration/hw fanout.

## Reuse (no reimplementation)

- `PickPlaceExecutor.run_prep()` (waste→oil→wash→buffer) + its config fields
  (`do_prep`, `prep_bore`, `needle_volume_uL`, `oil_needles=1`,
  `buffer_needles=4`, `service_z_mm`, `wash_cycles`, `*_well_pos`).
- `StageController.safe_travel_to` / `ensure_retracted_to` / `move_pump_uL` /
  `print_height_to_zref` / `print_z_dir` — all motion is polarity-safe and
  retract-before-XY (CLAUDE.md safety invariants).
- `build_well_plate_job(..., path_segments=, return_home=False)` + `PrintManager`
  discrete mode — Quick Print's existing print path, unchanged.

## Implementation Steps

- [x] 1. `_reagent_prep.py` — shared reagent helpers.
- [x] 2. `PickPlaceExecutor.aspirate_ink(...)`.
- [x] 3. Quick Print UI: ink-override combo + readout; prep row (prep checkbox,
      Service Z, Wash cycles, Ink dip Z, Pickup safety ×); setup-status label.
- [x] 4. `_compute_pickup_volume_uL()` (path length × flow + prime × safety).
- [x] 5. Restructure `_on_print`: gate (connections, ink→source well, safe Z +
      plate bottom, prep service wells) → up-front confirm → preflight worker
      (prep → aspirate_ink → preposition, try/finally retract) → HANDS-FREE
      `_on_prepositioned` (no mid-run confirm; ZP-recheck + arrival-confirmed
      guard) → `PrintManager` job.
- [x] 6. Abort routing (executor during preflight, PrintManager during print);
      `_update_button_state` treats preflight as busy.
- [x] 7. Tests + regression.

### Follow-on (operator request): Post-print needle cleanup

- [x] After a Quick Print COMPLETES, **clean + reset the needle**: expel a
      configurable multiple of the needle to waste (default **6×**), wash, then
      **refill oil to reset the syringe's oil level**.
      - Backend `PickPlaceExecutor.run_print_cleanup()` (waste → wash → oil),
        with fields `cleanup_waste_needles` (6), `cleanup_oil_needles` (1,
        fallback), `cleanup_oil_baseline_uL`. The oil step returns the plunger
        to its **pre-run position** (`move_pump_uL(bore, baseline − current)`,
        read live at the oil well after the waste expel — sign-safe because
        `get_pump_position_uL`/`move_pump_uL` share the same µL↔mm frame),
        clamped to ±20× needle and falling back to a fixed `cleanup_oil_needles`
        draw if the position read is missing/implausible.
      - GUI: a "Clean needle after print (waste → wash → reset oil)" row
        (checkbox **default ON** + Waste ×needle + Oil refill ×needle fallback;
        reuses Service Z + Wash cycles). `_on_print` captures the pre-run pump
        position as the oil baseline + gates on waste/wash/oil wells; the
        cleanup runs on a worker thread triggered by `PrintState.COMPLETED` in
        `_on_state` (skipped on abort/error — the print's own finally already
        left the needle at safe Z). Abort routes to the cleanup executor; the
        worker's finally always retracts to safe Z.

### Follow-on (operator request): Print-speed % of max

- [x] 8. **`Print speed: N% of max`** spin (1–100%, default 25%) in the config
      row. A single master lever that scales BOTH the XY traverse and the pump
      flow for every print-path segment, so the deposited bead width
      (volume-per-mm) stays constant across the speed range.
      - `_xy_max_mm_s()` — the 100% anchor: prefers the measured top speed
        (`PrintTimingCalibrationStore.get_store().get_xy_max_speed_um_s()`), else
        `controller.safety_limits.max_xy_speed` (both µm/s → mm/s), else a
        10 mm/s fallback.
      - `_resolved_print_kinematics()` → `(print_speed_mm_s, flow_uL_s,
        prime_uL)`: `print_speed = pct × XY max`; `flow = pct × Flow@100%`
        (the Flow knob is relabelled **Flow @100%**); `prime = flow × _PREFLOW_S`.
      - `_build_settings` + `_compute_pickup_volume_uL` both consume the resolved
        kinematics; the setup-status line shows the resolved `mm/s · µL/s`.
      - Scoped to Quick Print (sets the job's `PrintSettings`, which
        `PrintManager._execute_print_path` already paces by
        `max(xy_move_time, pump_move_s)` — both now scale together). Print Setup
        is unchanged. The Z descent feedrate (one-time MOVE_Z) is out of scope.

## Design notes

- **Ink-override combo** lists library inks with a reagent location whose
  `ink_type` ∉ {wash, waste, buffer, oil}. Default = the selected pump's
  assigned ink if it qualifies. Source well = `ink_locations[ink][0]`, resolved
  to absolute stage µm via the calibrated `well_positions`.
- **Pickup volume** floored to the prime (degenerate dot path → path_len 0).
  Path length summed per-segment from `_path_segments_for_selection()`
  (inter-segment hops don't extrude).
- **Gating in `_on_print`, not the button** — ink/service/calibration
  requirements surface as status text on Print (mirrors spheroid `_on_start`),
  so the existing button-gating tests stay green.
- **Hands-free: no mid-run position-confirm** — the only gate is the up-front
  "confirm setup" dialog; after positioning, the print starts directly (with the
  ZP-still-connected + arrival-confirmed safety checks). See Issues & Decisions.
- **Prep defaults ON.** See Issues & Decisions.

## Testing Notes

- New: `python -m unittest tests.test_v75x_quick_print_pick_and_place` (23+
  tests; the repo uses unittest, not pytest) — ink combo membership + pump
  default; service-well + needle-volume resolution; pickup-volume math;
  **print-speed % scaling (XY max source, both-axes scaling, constant
  volume-per-mm)**; setup-status gate strings; preflight worker wiring (mock
  `PickPlaceExecutor` — prep params + service positions + `aspirate_ink` called
  with the computed volume); `_on_print` gate messages.
- Regression: `test_v75x_quick_print_workflow.py`, `..._position_confirm.py`,
  `..._preposition_async.py`, `..._trajectory_view.py`, spheroid + pick-place +
  `safe_travel_to`-consumer suites.
- **Real-HW (ME3B V1):** verify prep dips waste/oil/wash/buffer at Service Z;
  needle picks up ink at the ink well, retracts, travels to the target, descends
  to print Z, primes, prints, ends at safe Z; needle retracts before every
  cross-well hop.

## Issues & Decisions

- **Prep defaults ON in Quick Print** (operator request; matches the spheroid
  page). The preamble (prep + ink pickup) engages whenever prep is on OR an ink
  is selected — which is the default — so the up-front "confirm setup" dialog
  normally shows. Turning prep OFF *and* selecting ink "(none)" restores the
  legacy plain-print path (no preamble, no dialog) — used by the preposition /
  position-confirm regression tests, which set prep OFF in their fixtures.
- **Ink/service/calibration gating lives in `_on_print`** (status text), not
  `_update_button_state`, so the Print button stays enabled on
  connection+well+object and the existing button-gating tests are preserved.
- **Hands-free: the mid-run "confirm needle position" dialog was REMOVED**
  (operator request — matches the original step list, which had no per-print
  confirm). The single gate is the up-front "confirm setup" dialog (shown when a
  preamble is active). `_on_prepositioned` then prints directly, but still
  refuses if the preamble aborted/errored, if the ZP board dropped during
  positioning, or if the positioning move did not confirm arrival (no operator
  to verify → auto-abort, needle left at safe Z).
- **Adversarial review (7-agent workflow) — HIGH abort gap fixed.** An Abort
  clicked during a *blocking* preamble move (pump / `safe_travel_to`) only SET
  the executor's `_abort_flag` — which is polled only at `_check_abort()` points
  — so no `AbortException` raised, and `_on_prepositioned` started the print
  anyway (dropping the operator's Abort). Fix: a sticky `_preflight_abort_requested`
  (set in `_on_abort`, consumed in `_on_prepositioned` — GUI-thread, race-free)
  PLUS a worker-`finally` capture of `executor._abort_flag.is_set()` → both force
  the "aborted" outcome so the print does not start. Tests:
  `test_abort_flag_set_during_blocking_move_no_print`,
  `test_sticky_abort_request_blocks_print`. Three review-flagged test gaps also
  closed: `test_prep_defaults_on`, `test_prep_on_no_ink_runs_prep_only`,
  `test_lower_bound_1pct_small_but_positive`; stale `_build_prep_row` docstring
  ("defaults OFF") corrected to "defaults ON".
- **Print-speed % scales both XY and the pump flow** (operator-chosen): the Flow
  knob became **Flow @100%**, and `pct` scales speed and flow together, so the
  deposited volume-per-mm is invariant to the % (only overall speed changes).
  100% anchor = measured XY top speed, else `safety_limits.max_xy_speed`. Default
  25% (delicate-print default; the resolved mm/s · µL/s is shown in the status).
  Scoped to Quick Print only.

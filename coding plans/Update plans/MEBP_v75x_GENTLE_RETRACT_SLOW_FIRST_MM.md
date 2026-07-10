# MEBP v7.5.x — Gentle "slow first mm" retract across ALL print execution

## Objective

When the needle lifts OUT of a print/deposit, run the **first ~1 mm of the lift
slowly**, then finish at the fast retract feedrate. At high speed the
back-pressure / surface tension can peel the just-deposited bead up off the plate
with the needle; a slow initial release breaks the bead cleanly.

This started as a per-sketch feature in the Print Builder Sketch compiler
(`MEBP_v75x_SKETCH_LIFT_AND_THICKNESS.md` follow-ons). The operator then asked
for the **same behavior baked into every other print-execution path** (discrete
`PrintManager`, hybrid, the shared travel primitives, the debug
`SimplePrintManager`).

## Design — one shared primitive, every retract funnels through it

The canonical retract primitives already centralize "raise the needle before
XY": `StageController.ensure_retracted_to` (raise-only, polarity-safe, confirmed)
and `StageController.safe_travel_to` (raise → XY → lower). Per the CLAUDE.md
safety rules, **all** print-execution retracts already go through these (or call
them indirectly). So the slow-lift lives there.

### `StageController`
- New instance config (default ON): `_retract_slow_dist_mm = 1.0`,
  `_retract_slow_feedrate = 60.0` mm/min (= 1 mm/s) — matches the sketch
  defaults. Setter `set_retract_slow_lift(dist_mm, feedrate_mm_min=None)`.
- New shared helper `_retract_z_slow_then_fast(cur_zref, target_zref, fast_fr,
  timeout_s, tol_mm)`:
  - When the move is a net **LIFT** in the polarity-safe HEIGHT frame, issue a
    slow segment first (raise `min(slow_dist, remaining)` at the slow feedrate),
    then the fast segment to the target. A **descent** (or `cur_zref is None`,
    or `slow_dist == 0`) is a single move — descents are never slowed.
  - Only the FINAL position is confirmed (Marlin runs the two queued moves in
    order); every move carries an explicit feedrate (never a bare `G0 Z` — the
    modal-feedrate-inheritance ZP-hang bug).
  - Reads the slow params via `getattr(..., 0.0)` so the many `__new__` test
    stubs (which don't set the attr) stay on the legacy single-move path → no
    existing-test churn.
- `ensure_retracted_to` and `safe_travel_to` Step 1 now delegate their raise to
  the helper (reusing the current Z they already read; `safe_travel_to` reads it
  only when `slow_dist > 0`).

### `PrintManager` (discrete)
- `TRAVEL_UP` handler now retracts via `ensure_retracted_to(travel_z)` (gentle +
  confirmed + raise-only) instead of a bare `move_z_absolute`; falls back to the
  explicit-feedrate move on an older controller. This covers the **inter-well**
  lift (the next well's `TRAVEL_UP` lifts out of the previous well's print) and
  the **final** end-of-print lift.
- Already covered (no change needed, they call `ensure_retracted_to`):
  `_retract_for_travel` (`MOVE_XY`/`HOME_XY` + the inter-object hop),
  `_retract_to_safe_z` (the end-of-print safe-Z guarantee for ALL modes).

### Hybrid (`HybridPlanExecutor` / `DirectCommandExecutor`)
- New `DirectCommandExecutor.raise_z(z, feedrate, timeout)` — prefers
  `ctrl.ensure_retracted_to` (gentle), falls back to confirmed `move_z`.
- Raise sites switched from `move_z` → `raise_z`: `travel_to_well` Phase 1,
  `raise_from_well`, and `HybridPlanExecutor` `MOVE_SAFE_Z` / `TRAVEL_XY` /
  `RETURN_HOME`. Descents in `travel_to_well` Phase 3 keep `move_z` (not slowed).

### `SimplePrintManager` (debug)
- No change — `_safe_retract` / `_retract_to_safe_z` already call
  `ensure_retracted_to`, so they inherit the slow lift automatically.

## Files Modified
- `SupportClasses/StageController.py` — slow-lift config + `set_retract_slow_lift`
  + `_retract_z_slow_then_fast`; `ensure_retracted_to` and `safe_travel_to`
  delegate the raise.
- `SupportClasses/PrintManager.py` — `TRAVEL_UP` gentle; `DirectCommandExecutor.
  raise_z`; hybrid raise sites use it.
- `tests/test_v75x_gentle_retract_slow_lift.py` — new (16).
- `tests/test_v75x_zp_feedrate_inheritance_fix.py` — TRAVEL_UP test updated
  (delegates to `ensure_retracted_to`; + older-controller fallback test).

## Status
- [x] StageController slow-lift config + setter + helper
- [x] ensure_retracted_to / safe_travel_to delegate to helper
- [x] discrete TRAVEL_UP gentle
- [x] hybrid raise_z + 5 raise sites
- [x] SimplePrintManager inherits (verified, no change)
- [x] tests green

## Testing Notes
- `tests/test_v75x_gentle_retract_slow_lift.py` (16): slow-then-fast on a lift;
  single move on a descent / no-current-Z / slow_dist 0; short lift entirely
  slow; both Z polarities; arrival-timeout → False; `ensure_retracted_to` /
  `safe_travel_to` two-phase; `__new__` stub stays legacy; TRAVEL_UP and hybrid
  `raise_z` delegation + fallback.
- Regression green: zp-feedrate-inheritance, z-retract-before-xy, print-always-
  safe-z, jog-navigation (real `safe_travel_to`), simple-print-manager,
  printing-mode-calibrated-wells, quick-print-pick-and-place / position-confirm,
  spheroid-pick-place-z, pump-settle, print-setup-routine, multi-object-seam,
  print-path-barrier, cell-targeting/labeling, stress-test, z-axis-unified,
  last-known-calibration, fluorescence-mosaic. (`test_real_24_well_mosaic` is the
  documented pre-existing unrelated CV failure.)
- **Needs real-HW verification on ME3B V1**: run a multi-well print and confirm
  the lift out of each well/print starts slow for ~1 mm then speeds up; the bead
  no longer peels off the plate; prints still end at safe Z.

## Issues & Decisions
- **Default ON, globally (1 mm @ 1 mm/s).** Matches the operator request and the
  sketch defaults. The cost is ~1 s per *real* lift-out; lifts that start already
  retracted are a confirmed no-op (the early-out / no slow segment), so e.g. the
  retracted-throughout mosaic scan pays nothing. Tunable via
  `set_retract_slow_lift` (no GUI knob added — global behavior).
- **`getattr(..., 0.0)` default in the helper** keeps the dozens of `__new__`
  controller test stubs on the legacy single-move path, so only the two
  intentionally-changed TRAVEL_UP assertions needed updating.
- **Descents are never slowed** — only a net height-frame *lift* gets the slow
  lead-in.
- **Known gap — pure-trajectory in-path retracts.** `TrajectoryExecutor` follows
  planner-authored waypoints; its END retract is gentle (via
  `_retract_to_safe_z`), and a **sketch** trajectory already carries its slow
  lift in the waypoint timing, but a planner-generated trajectory's *in-path*
  retract legs are not re-timed here. This is the de-emphasized path on ME3B V1
  (the planner still has the documented `ZDIR=+1` Z-geometry gap); revisit when
  that planner gets its polarity pass.

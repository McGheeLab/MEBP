# MEBP v7.5.x — Spheroid Pick & Place execution: pump API + pick/place Z

## Objective

Fix the Spheroid Pick & Place *execution* (after the click→stage scaling fix in
`MEBP_v75x_SPHEROID_PICK_CLICK_SCALING.md`). Three operator-reported problems:

1. **Crash mid-operation:**
   `move_pump_uL() got an unexpected keyword argument 'feedrate_mm_min'`.
2. **Z never went to the pick/place location** (and it took ~30 s to "move").
3. **Setup needs separate pick / place Z offsets from the plate bottom** — the
   pick and place heights must be configurable and independent.

## Root Causes

### Crash — wrong pump kwarg

`StageController.move_pump_uL(pump, volume_uL, rate_uL_s=None)` takes a **µL/s**
flow rate and does the µL/s→mm/min conversion (and flow clamp) itself. The
executor pre-converted via `_uL_s_to_mm_min` and passed it as
`feedrate_mm_min=…`, a kwarg `move_pump_uL` doesn't accept → `TypeError`. Present
at every `move_pump_uL` call (spheroid ×2, trypsin ×4, fluorescent ×5).

### Z wrong + slow

- The spheroid page created the `PickPlaceExecutor` but **never set** its
  `safe_z_mm` / operating Z / well positions, so it used the defaults
  `safe_z_mm=5.0`, `operating_z_mm=0.0` — nowhere near the real plate.
- The clicked targets carry `well_name=""`. `_safe_move_to` compared
  `target.well_name != self._current_well` (`"" != ""` is False) → the move was
  treated as **intra-well**, doing a relative ±`intra_well_retract_mm` (1 mm) Z
  jiggle around `operating_z_mm=0.0` instead of travelling to the operate
  height. The two `wait_for_z_arrival` calls (targets `-1.0`, `0.0`) could never
  reach the real Z (~40 mm zero-ref) → two 15 s timeouts ≈ the 30 s "slow move".

### No pick/place Z control

`SpheroidPickupConfig` had a single implicit operate height; nothing let the
operator set independent pick vs place depths.

## Fix

- **`SupportClasses/PickAndPlaceManager.py`**
  - All `move_pump_uL(..., feedrate_mm_min=feedrate)` → `move_pump_uL(...,
    rate_uL_s=<speed_uL_s>)` (lets `move_pump_uL` convert + flow-clamp; removes
    the redundant `_uL_s_to_mm_min` pre-conversion, now unused).
  - `SpheroidPickupConfig` gains `pick_z_offset_mm` (default 0.10) and
    `place_z_offset_mm` (default 0.50) — heights above the plate bottom (mm).
  - `PickPlaceExecutor` gains `pick_z_mm` / `place_z_mm` (zero-ref mm; None →
    `operating_z_mm`).
  - `_safe_move_to(target, target_z_mm=None)` lowers to `target_z_mm`; the
    intra-well shortcut now fires only for a **non-empty** well name that
    matches the previous move — an empty well name (arbitrary clicked points)
    always takes the full safe-Z travel. `_intra_well_move` gained the same
    `target_z_mm`.
  - `_execute_spheroid_pickup` moves the source to `pick_z_mm` and the dest to
    `place_z_mm`.
- **`gui/pages/workflows/spheroid_pickup_workflow.py`**
  - Config row gains **Pick Z (↑ bottom)** + **Place Z (↑ bottom)** spin boxes
    (mm, independent defaults 0.10 / 0.50).
  - `_current_config` passes the offsets; new `_plate_offset_to_zref` resolves a
    height-above-bottom → zero-ref Z via `controller.print_height_to_zref`
    (fallback: the page's `plate_bottom_z` reference × `print_z_dir`).
  - `_on_start` resolves `pick_z`/`place_z`, gates on Safe Z + a calibrated
    plate bottom (clear status message otherwise), and sets `executor.safe_z_mm
    / pick_z_mm / place_z_mm`.

## Files Modified

| File | Change |
|------|--------|
| `SupportClasses/PickAndPlaceManager.py` | pump `rate_uL_s` (all sites); `SpheroidPickupConfig` pick/place offsets; executor `pick_z_mm`/`place_z_mm`; `_safe_move_to`/`_intra_well_move` `target_z_mm` + empty-well = full safe travel |
| `gui/pages/workflows/spheroid_pickup_workflow.py` | pick/place Z spin boxes; `_plate_offset_to_zref`; `_on_start` resolves + gates + wires executor Z |
| `tests/test_v75x_spheroid_pick_place_z.py` | new (5 tests) |

## Implementation Steps

- [x] Pump `rate_uL_s` at every call site
- [x] `SpheroidPickupConfig` pick/place Z offsets
- [x] Executor `pick_z_mm`/`place_z_mm` + `_safe_move_to` target_z + empty-well full-travel
- [x] Spheroid page Z-offset controls + resolve + gate + wire
- [x] Tests (5) + page build smoke
- [x] Affected suites green

## Testing Notes

- `tests/test_v75x_spheroid_pick_place_z.py` (5): no pump kwarg crash + uses
  `rate_uL_s`; source/dest use distinct operate Z; empty well names always full
  safe travel (no `move_z_relative` jiggle); `safe_z_mm` propagated; pick/place
  fall back to `operating_z_mm` when unset.
- `tests/test_v75x_spheroid_picker_scaling.py`, `test_v75x_z_retract_before_xy_travel`
  green; offscreen page-build smoke OK (Pick/Place Z spins build, config reads
  0.10 / 0.50).
- **Needs real-HW verification on ME3B V1**: run a pick & place — it should
  travel to the pick at `plate_bottom + pick_z`, aspirate, travel to the place
  at `plate_bottom + place_z`, dispense; no pump TypeError; no 15 s Z timeouts.

## Addendum — retract-before-XY hole when the ZP board is disconnected

A 4-agent audit (verifying "the needle retracts before every cross-position XY
move in pick & place") confirmed the **normal (ZP-connected) path is safe**: both
the pick and the place route through `_safe_move_to` → `safe_travel_to`, which
raises Z to safe height, confirms via M400 + `wait_for_z_arrival`, **aborts the
XY move if Z isn't confirmed**, then moves XY. The intra-well 1 mm shortcut is
unreachable for spheroid targets (empty `well_name` → `bool("")` False → always
the full safe-travel branch).

The adversarial pass found a real **HIGH** gap, though: `safe_travel_to` wraps its
entire retract+confirm step in `if self.is_zp_connected:` (StageController.py
~3343) and the XY move in a *separate* `if self.is_xy_connected:` block (~3365).
If the **ZP board drops off USB** (the documented mid-session CH340 disconnect)
while the ProScan XY stays connected, the retract is silently skipped and the XY
move still runs — dragging a needle that is typically left **down** (the spheroid
op has no final retract, so it ends at `place_z`). `_on_start` did not gate on
`is_zp_connected` (only Safe Z + plate bottom), and the executor didn't re-check
it, so a ZP drop at-start or mid-run could drag the needle across the plate.

**Fix (contained to the pick & place path):**
- `PickPlaceExecutor._safe_move_to` raises `AbortException` (refusing the XY
  move, aborting the run) when `controller.is_zp_connected` is False — covers a
  mid-run drop and is defense-in-depth behind the GUI gate.
- `SpheroidPickupWorkflowPage._on_start` gates on `is_zp_connected` with a clear
  message (mirrors the manual click-to-travel handlers).

Tests: `test_zp_disconnected_refuses_xy_move`, `test_zp_drop_midrun_blocks_place_leg`
in `tests/test_v75x_spheroid_pick_place_z.py`.

## Addendum 2 — root-cause hardening of `safe_travel_to` + end-at-safe-Z (both done)

A 3-agent recon (callers / tests / signals) confirmed there is **no XY-only /
no-needle production caller** — every `safe_travel_to` caller is the ME3B V1
bioprinter, and `is_zp_connected` short-circuits to **True in simulation**, so a
real-hardware `False` reliably means "needle present, board dropped." Only one
test (`test_v731_jog_navigation.py::test_xy_only`) encoded the old
"XY-proceeds-when-ZP-down" contract, and its controller has no needle signal, so
it stays correct under the new behavior.

**(a) `safe_travel_to` hardened (StageController.py).** New `_needle_present()`
= connected now **or** `_zp_ever_connected` (sticky session flag, set True on the
first real ZP connect, never reset) **or** `_hardware_config.pumps` configured.
The retract block (`if self.is_zp_connected:`) now has an `elif
self._needle_present(): return False` — so when the ZP board is disconnected but
a needle exists, the XY move is **refused** (returns False, poller still resumed
in the `finally`) instead of driving an unretracted needle. A genuine no-needle
rig (no board, never connected, no pumps) falls through and moves XY as before.
Fixes the root for **all** callers (jog/calibration/quick-print/stress/spheroid).

**(b) Pick & place end-at-safe-Z (PickAndPlaceManager.py).** `execute_queue` now
wraps its loop in `try/finally` and calls new `_retract_to_safe_z()` →
`controller.ensure_retracted_to(safe_z_mm)` (raise-only, polarity-safe, no-op
when already retracted or ZP disconnected, never raises) on **every** exit
(completion / abort / AbortException / error). Mirrors
`MEBP_v75x_ALL_PRINTS_END_AT_SAFE_Z.md` — a completed/aborted pick & place no
longer leaves the needle parked down at `place_z`.

Tests: `tests/test_v731_jog_navigation.py` +
`test_refuses_xy_when_needle_present_but_zp_down`,
`test_refuses_xy_when_pumps_configured_but_zp_down` (and `test_xy_only` reframed
as the true no-needle case); `tests/test_v75x_spheroid_pick_place_z.py` +
`test_run_ends_at_safe_z`, `test_retract_on_safe_z_runs_even_after_failure`
(now 11). Affected `safe_travel_to`-consumer suites green (jog-nav, integration,
quick-print confirm/async, stress, needle-location, z-retract, spheroid, mosaic
fakes; the lone `test_real_24_well_mosaic` 23/24 is the documented pre-existing
CV failure).

## Issues & Decisions

- **Empty well name ⇒ full safe travel.** Spheroid targets are arbitrary clicked
  points with no well context, so the 1 mm intra-well retract is unsafe (the two
  points may be far apart / different wells). Only a non-empty, matching well
  name (trypsin/fluorescent named-well flows) keeps the intra-well shortcut.
- **`rate_uL_s` over a pre-computed feedrate.** `move_pump_uL` already converts
  and applies the flow-rate safety clamp — passing µL/s keeps that protection;
  the manual `_uL_s_to_mm_min` bypassed it.
- **Gate on plate-bottom + Safe Z.** Without them the executor would fall back to
  a bogus operate Z; the page now blocks with a clear message instead.

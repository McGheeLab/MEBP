# MEBP v7.5.x — Jog page syringe readout shows the LIVE plunger fill

## Objective

Fix the operator-reported bug: **"syringe in the jog page does not show the
current amount of volume and does not automatically update, something is
broken."**

The Jog page pump rack must show, per pump, the **live plunger fill** (how much
fluid is in the syringe right now), updating every status tick as the plunger
moves — the same readout the Hardware Control Panel already shows.

## Root Cause

The Jog page pump rack (`gui/widgets/pump_rack.py` → `PumpColumn._refresh`)
rendered `fc.ink_volume_uL` where `fc = self._loadout.fluid_column`. The
`fluid_column` came from the **static** `pump_cfg.fluid_column` config object.
`FluidColumn` is a layered oil/buffer/ink model that is only mutated by
`aspirate_ink` / `dispense` calls **during a print or workflow** — it is never
updated during manual jogging. So on the Jog page it displayed a frozen config
value (typically `0.0 µL`) that never tracked the plunger.

`JogControlPage._refresh_pump_panel()` *did* pass the live plunger position into
`PumpLoadout.current_position_mm` every ~300 ms tick, but the display never read
it — it read `fluid_column.ink_volume_uL`. So both halves of the complaint had
the same single cause: the displayed number was a static value, not derived from
the live plunger position.

The status-tick wiring was never broken: `app.py::_tick` → `_update_status` →
`page.on_status_update()` → `_refresh_pump_panel()` fires every
`polling.position_interval_ms` (default 300 ms) while the Jog page is active.

The Hardware **Control Panel** (`gui/pages/hardware/control_panel.py`) was
already reframed in v7.5.x (per `MEBP_v75x_PUMP_PLUNGER_CAL_AND_ASPIRATE_DISPENSE.md`)
to show live fill µL via `controller.raw_to_pump_fill_uL(pump, raw_mm)`, falling
back to raw mm when the plunger isn't calibrated. The Jog page pump rack was
simply never migrated to that API.

## Fix

Mirror the proven control-panel approach: derive the live fill from the live
plunger position via the controller's plunger-calibration API and feed it to the
rack each tick.

- **Calibrated pump** → fill in µL (0 = empty/fully dispensed → capacity =
  full/fully aspirated) + a proportional bar; low-fill warning reused.
- **Configured but un-calibrated pump** → the raw plunger position in mm (still
  updates live) + a "calibrate plunger for µL" hint (we cannot express a true
  volume without the calibration).
- **No syringe configured** → the existing "not configured" placeholder.

## Files Modified

| File | Change |
|------|--------|
| `gui/widgets/pump_rack.py` | New `PumpColumn.set_live_fill(*, fill_uL, capacity_uL, raw_mm, syringe, ink_spec, calibrated)` + `PumpRack.update_live_fills(fills)`. Extracted `_set_vol_style` / `_set_sub_style` helpers. Normalises the `-0.0` artifact at the empty datum. Reads the ink accent from `InkSpec.color` (the real attribute — not `display_color`). Docstring updated. `SyringeSpec` added to the import. (Old `set_pump_loadout` / `update_from_workspace` / `_refresh` path left in place but unused — back-compat, no other caller.) |
| `gui/pages/jog_control.py` | Rewrote `JogControlPage._refresh_pump_panel()` to build a per-pump live-fill snapshot from `is_pump_plunger_calibrated` / `raw_to_pump_fill_uL` / `pump_capacity_uL` (RAW Marlin mm in; µL out) and call `self._pump_panel.update_live_fills(...)`. The un-calibrated fallback shows raw `p_raw` (matches the embedded control panel). Dropped the dead `PumpLoadout`/`FluidColumn` construction. |
| `tests/test_v75x_jog_pump_fill_readout.py` | NEW — 12 tests (widget + Jog-page integration with real `StageController` fill math; incl. ink-accent + raw-frame regression guards). |

## Implementation Steps

- [x] Root-cause: confirm the displayed value is the static `fluid_column`, not the live position; confirm the status-tick wiring fires `_refresh_pump_panel`.
- [x] Add `PumpColumn.set_live_fill` + `PumpRack.update_live_fills` (calibrated / uncalibrated / unconfigured paths).
- [x] Rewrite `_refresh_pump_panel` to use the plunger-calibration fill API and the raw-mm fallback.
- [x] Normalise the `-0.0 µL` artifact at the empty datum (keep genuine negatives visible).
- [x] Tests (widget + Jog-page integration); jog + pump regression suites green (118).
- [x] Adversarial review workflow over the diff (3 lenses → verify each finding).

## Testing Notes

- `python -m unittest tests.test_v75x_jog_pump_fill_readout` — 12 pass.
- Regression: `test_v75x_pump_plunger_setup`, `test_v731_jog_navigation`,
  `test_v75x_axis_map_jog`, `test_v75x_jog_direction_z_up_sign`,
  `test_v75x_pump_settle_and_prime_time`, `test_v75x_jog_pump_fill_readout` —
  123 pass.

**Needs real-HW verification on ME3B V1:** open the Jog page with a calibrated
pump and jog the plunger — the µL readout (and bar) must track the plunger live;
an un-calibrated pump shows live raw mm + the calibrate hint; an unconfigured
pump shows "not configured". (Per the plunger-cal plan, the plunger calibration
must have been run on the machine for the µL readout; otherwise the raw-mm
fallback is expected.)

## Issues & Decisions

- **Why not just fix `_refresh` to read `current_position_uL`?** `PumpLoadout.current_position_uL = current_position_mm × uL_per_mm` is a signed displacement from the "set zero" point, not a true fill (no calibrated direction/empty datum). The plunger-calibration fill (`raw_to_pump_fill_uL`) is the v7.5.x canonical answer and is what the Control Panel already shows — so the two surfaces agree.
- **Uncalibrated fallback frame:** the Jog rack shows the **raw Marlin** plunger position (`p_raw`, no zero-ref subtraction) to match the embedded Hardware Control Panel's pump readout (the Jog page hosts that panel via `StandardJogContextPanel`, so both pump readouts are on one screen). This was changed from an initial zero-ref version after the review flagged the divergence (see below). Both update live; neither is a true volume.
- **Layered oil/buffer/ink barrel dropped on the Jog page:** there is no live layer tracking during manual jogging, so the barrel now renders a single fill level (how full the syringe is). The layered model remains correct/meaningful only inside the execution paths that mutate it.
- **`-0.0 µL`:** signed arithmetic at the empty datum yields `aspirate_sign·0 = -0.0`; normalised to `0.0` for display while preserving a genuine past-empty negative.

### Adversarial review (3 lenses → verify each finding)

A review workflow over the diff raised 6 findings; 1 confirmed material + 1 real
defect in the new code were both fixed; the rest were verified non-material.

- **[FIXED] Uncalibrated mm frame divergence (confirmed, low):** the rack showed
  zero-ref mm while the control panel below it shows raw Marlin mm → two
  different numbers for one plunger when `zero_position[pid] ≠ 0`. Fix: the
  fallback now shows raw `p_raw` (also corrects the calibrated tooltip's "raw"
  label).
- **[FIXED] `InkSpec.display_color` does not exist (real, cosmetic):** the field
  is `color`; the `getattr(ink_spec, "display_color", …)` silently fell through
  so the ink accent never rendered (always neutral mauve). Fix: read `.color`.
  (This is a repo-wide latent misnomer — `syringe_display.py` / `print_monitor.py`
  / the old `pump_rack` path share it; only the new code was corrected here.)
- **[WON'T FIX] Orphaned loadout API (nit):** `set_pump_loadout` /
  `update_from_workspace` / the `_refresh` loadout branch now have no caller.
  Left in place (back-compat, getattr-guarded, no regression risk) — documented
  above.
- **[WON'T FIX] Exception containment (low, no repro):** `_refresh_pump_panel`
  only wraps the fill calls; verified there is no actual raise path (all APIs
  return safe types) and broad try/except in a status-tick display fn would mask
  real bugs. Pre-existing dispatch behavior, not introduced here.

# MEBP v7.5.x — Plate Location: Z-Heights-First Ordering + Retract Before Each Well Hop

## Objective

Two related safety fixes for the Plate Location calibration:

1. **Order the workflow Z-heights → plate.** The Needle Offset Calibration (Z heights) tab
   now precedes Plate Location, and a Plate Location run is blocked until the Safe/Move Z
   height is set.
2. **Retract the needle before every inter-well move.** During the manual click-rim flow
   the operator jogs Z down to focus on each well's edge; the move to the *next* well must
   first lift the needle to the Safe Z so it doesn't drag across / crash into the plate.

## Root Cause

The run retracts **once** before the scan (`_ploc_run_queue` pre-scan `ensure_retracted_to`),
but each per-well move (`_ploc_begin_well_click_rim` anchor, `_ploc_advance` freeform hop,
`_ploc_auto_fit_well` initial move, `_ploc_begin_well_manual` start) was a bare
`move_xy_absolute_um` at whatever Z the operator left the needle — so after jogging Z down
to focus on well *N*, the hop to well *N+1* travelled with the needle still low. The
pre-scan retract also fell back to `safe_z = 0.0` when Safe Z was unset, which is not a
real safe height (and on ME3B V1's ZDIR=-1 a descent).

## Changes — all in `gui/pages/calibration.py`

### Workflow ordering
- Reordered the workflow tabs to **Needle Location → Needle Offset Calibration → Plate
  Location → Custom** (Z heights before plate); `_zoff_tab_index` updated 2 → 1. Only
  `_zoff_tab_index` depended on the order (used by `_on_workflow_tab_changed` to start the
  microscope feed on that tab).

### Run gate
- `_ploc_run_queue` now blocks at the top (after the plate check, applies to all runs) when
  a ZP/needle is connected (`controller.is_zp_connected`) **and** `_safe_z is None`, with a
  message directing the operator to capture the Safe/Move Z on the Needle Offset tab first.
  (Gated on ZP-connected so a needle-less / XY-only setup isn't blocked — nothing to
  retract there.)

### Retract before each inter-well hop
- New `_ploc_safe_goto(x_um, y_um)`: retracts the needle to Safe Z and travels XY via
  `_safe_navigate_to(x, y, lower_z=False)` (raise-Z → wait → fast XY → wait, no descent);
  bare `move_xy_absolute_um` only as a last-resort fallback if Safe Z is somehow unset.
- Routed the inter-well moves through it: `_ploc_begin_well_click_rim` (manual anchor),
  `_ploc_advance` (freeform point), `_ploc_auto_fit_well` (initial centred move),
  `_ploc_begin_well_manual` (3-point fallback start). **Intra-well rim-sample moves**
  (`_ploc_multi_edge_fit`) stay bare — they orbit the rim at the imaging Z and must not
  retract/refocus each step.

## Files Modified

| File | Change |
|------|--------|
| `gui/pages/calibration.py` | Tab reorder + `_zoff_tab_index`; Safe-Z run gate; `_ploc_safe_goto` retract-first hop wired into all inter-well moves |
| `tests/test_v75x_plate_location_manual_click_rim.py` | +2 `TestSafeGoto` tests; `TestBeginClickRim` stub binds `_ploc_safe_goto` |

## Implementation Steps

- [x] Reorder tabs (Z heights before Plate Location); fix `_zoff_tab_index`
- [x] Safe-Z run gate in `_ploc_run_queue` (gated on ZP connected)
- [x] `_ploc_safe_goto` helper (retract via `_safe_navigate_to(lower_z=False)`, bare fallback)
- [x] Route anchor / freeform / auto-initial / 3-point-start moves through it; keep rim
  samples bare
- [x] Tests + offscreen build (verify tab order + index)

## Testing Notes

- `tests/test_v75x_plate_location_manual_click_rim.py::TestSafeGoto` — routes through
  `_safe_navigate_to(..., lower_z=False)` when Safe Z is set; bare `move_xy_absolute_um`
  only when Safe Z is `None`.
- `TestBeginClickRim` still asserts exactly one anchor move to the predicted centre (now via
  `_ploc_safe_goto`).
- Offscreen build: tab order is `['Needle Location','Needle Offset Calibration','Plate
  Location','Custom']`, `_zoff_tab_index == 1`.
- Regression: manual-click-rim, predict-well, plate-centering, freeform-warp, jog-navigation,
  **Z-retract-before-XY-travel (22)**, Z-autocal, simulated-camera all green.
- On hardware: Plate Location refuses to run until the Safe/Move Z is taught; during a run,
  after jogging Z down to a well's edge, the hop to the next well first lifts the needle to
  Safe Z (visible Z retract + wait) before the XY move.

## Issues & Decisions

- **Gate on ZP-connected**, not unconditionally, so an XY-only rig (no needle to retract)
  isn't blocked; on a needle machine (ME3B V1) it always fires until Safe Z is set.
- **Intra-well rim moves stay bare** — retracting between the 8 rim samples would refocus
  the microscope and slow the scan; only moves to a *new* well retract.
- Complements `MEBP_v75x_Z_RETRACT_BEFORE_XY_TRAVEL.md` (which added the one-time pre-scan
  retract + the `ensure_retracted_to`/`safe_travel_to` helpers); this extends the guarantee
  to every per-well hop within the Plate Location run.

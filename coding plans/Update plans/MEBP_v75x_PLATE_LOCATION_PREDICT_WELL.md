# MEBP v7.5.x — Plate Location "Snap to Well Center" Lands Between Wells

## Objective

Fix a bug reported on the Calibration page → **Plate Location** sub-tab: after the
well plate has been mapped roughly correctly to the physical setup, snapping a well
into the queue (Snap mode) and running it does **not** center on the well — the stage
drives to the space *between* wells, even though the plate is drawn correctly on the
canvas.

## Root Cause

The Plate Location run state machine moves to each queued well via
`CalibrationPage._predict_well_xy(well_name)`, whose return value is passed to
`controller.move_xy_absolute_um(cx, cy)` (absolute stage µm). Two independent defects
in that one method:

1. **Dead primary branch.** `_predict_well_xy` first consulted
   `getattr(self, '_predicted_wells', None)` — but `_predicted_wells` is assigned
   **nowhere** in the codebase (repo-wide grep finds only this read). So the branch
   never fired and every call fell through to the geometry fallback.
2. **Wrong-frame fallback.** With no taught A1, the fallback returned
   `zero_position + (well − A1) grid offset` — i.e. it pinned the well grid to stage
   zero, as if A1 sat at the stage origin.

Meanwhile the canvas draws from `_wells_in_zero_ref()` →
`_calibrated_positions or _predicted_positions` (absolute stage µm, centered on the
**XY safety-envelope midpoint** via `default_plate_center_um()`), minus `zero_position`
for the zero-ref display frame. So **display and motion used different frames**: the
needle drove to a constant offset from each drawn well. On a ~9 mm-pitch plate any
constant offset still looks grid-aligned on the map but lands in the gaps — exactly the
"between wells" symptom.

Corroboration: `tests/test_v75x_well_radius_conversion.py` documents a *prior*
`_predict_well_xy` bug that lived in the **fallback** path (an `_mm`-suffix
AttributeError) — confirming the fallback was always the live path and the
`_predicted_wells` branch was always dead.

The dead branch also meant a fitted warp/affine (`_calibrated_positions`) never fed
back into subsequent queue runs — every run re-predicted from raw geometry regardless
of how well the plate was calibrated.

## Changes

### Fix — `_predict_well_xy` reads the same dict the canvas draws

`gui/pages/calibration.py::_predict_well_xy`:

- **Primary path** now reads `self._calibrated_positions or self._predicted_positions`
  (same precedence as `_wells_in_zero_ref` / the canvas) and returns that well's value
  **directly**. These dicts are already absolute stage µm — the canvas subtracts
  `zero_position` only for drawing — so they go straight to `move_xy_absolute_um`
  **without** the old `zero_position` offset. (The old `zero + value` was only ever
  correct for the zero-ref dict that never existed.)
- **Fallbacks** (only when no position dict exists yet) preserved/clarified:
  - taught-A1 anchor → `taught_a1 + grid offset` (same formula as
    `WellPlate.get_all_positions_from_a1`).
  - **new:** no taught A1 → mirror the canvas geometry-only seed
    (`default_plate_center_um()` → `get_all_positions_from_plate_center`) so the move
    matches the drawn map even before any calibration. `try/except` falls to the
    last-resort zero anchor if the controller can't supply a center.

Net effect: in every reachable state, `_predict_well_xy(well)` returns exactly
`(drawn canvas position + zero_position)`, so the stage drives onto the well the
operator sees and snapped. No new motion/coordinate math; no change to the callers.

## Files Modified

| File | Change |
|------|--------|
| `gui/pages/calibration.py` | `_predict_well_xy` reads `_calibrated_positions or _predicted_positions` (absolute µm, returned directly); envelope-center seed fallback; dead `_predicted_wells` lookup removed |
| `tests/test_v75x_plate_location_predict_well.py` | New — 10 tests (primary path + fallbacks + source guard) |

## Implementation Steps

- [x] Diagnose: confirm `_predicted_wells` is never assigned; confirm position dicts are absolute stage µm at all assignment sites; confirm `_wells_in_zero_ref` (canvas) and `move_xy_absolute_um` frames
- [x] Rewrite `_predict_well_xy` primary path to use the displayed dicts, returned directly
- [x] Add envelope-center seed fallback matching the canvas geometry-only seed
- [x] Verify both callers (`_ploc_auto_fit_well`, `_ploc_begin_well_manual`) feed `move_xy_absolute_um` and need absolute µm
- [x] Add regression tests; run new + existing suites
- [x] Update plan doc + CLAUDE.md table

## Testing Notes

- `tests/test_v75x_plate_location_predict_well.py` (10) — calls the real (unbound)
  `_predict_well_xy` with a stub `self` (no Qt instance):
  - calibrated returned verbatim, **not** offset by `zero_position` (core regression);
  - calibrated precedence over predicted; predicted used when no calibrated;
    empty-dict `{}` falls through to predicted;
  - returned value matches the canvas frame (`got − zero == drawn zero-ref`);
  - no-state fallback uses the envelope-center seed, **not** the zero-anchored grid;
    taught-A1 anchor; last-resort zero anchor when no center available;
  - source guards: the dead `getattr(self, '_predicted_wells'…)` lookup is gone and the
    method reads the displayed position dicts.
- `tests/test_v75x_well_radius_conversion.py` (6) still passes (attribute/unit contract
  the fallback depends on).
- On hardware: map the plate (or run a 3-well/manual fit), snap a well in Snap mode →
  Run → the stage centers on that well (camera edge-fit then refines), not the gap
  between wells.

## Issues & Decisions

- **Calibrated-before-predicted precedence** matches `_wells_in_zero_ref`, so the move
  target equals the best available estimate and a fitted warp now actually feeds back
  into later queue runs (previously every run re-predicted from raw geometry).
- **Envelope-center seed in the no-state fallback** was added so the pre-calibration
  move matches the drawn map; the legacy taught-A1 and zero-anchor branches are kept as
  ordered fallbacks (taught A1 normally implies `_predicted_positions` is populated, so
  the taught-A1 branch is effectively a guard).
- **Adversarial verification workflow** (4 dimensions: frame correctness, caller
  compatibility, display/move consistency, regression/edge-cases) was launched but its
  agents died on a session limit; the review was completed manually against ground-truth
  reads. Findings: fix correct; only an unreachable theoretical divergence (taught A1 set
  while `_predicted_positions` is None — canvas would seed from envelope center while the
  method honors taught A1), which is not a regression and arguably more correct.

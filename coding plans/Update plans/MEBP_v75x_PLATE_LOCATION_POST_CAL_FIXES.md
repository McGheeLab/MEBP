# MEBP v7.5.x — Plate Location Post-Calibration UI / Safety Fixes

## Objective

Three issues reported right after running the manual click-rim plate calibration
(3 wells, 3 edge clicks each, then the fit):

1. **The plate view "changes size" and obscures the Free / Snap-to-well buttons.**
2. **A second XY coordinate view appears; clicking drove the physical stage off to a
   wrong extreme** ("off its range").
3. (Root cause behind 1 & 2) the fit could produce **off-plate well positions**.

## Root Causes & Fixes

### A. Free/Snap toggle obscured — `gui/widgets/jog_workspace_view.py`

`paintEvent` painted the segmented mode toggle **first**, then the envelope / plate /
wells **on top**. A well rendered near the top-left corner (e.g. from an off-plate
calibration) drew over the toggle, hiding the buttons.

**Fix:** clip the workspace drawing to `_content_rect()` (so content can't bleed into
the reserved toggle band) and paint the toggle **last**, so the Free / Snap buttons are
always on top and clickable. Toggle hit-testing is unchanged (still checked first in
`mousePressEvent`).

### B. Degenerate manual fit drove the stage to an envelope corner — `gui/pages/calibration.py` + `SupportClasses/PlateWarpCalibrator.py`

A manual fit from 3 close-together / near-collinear / noisy clicks yields an affine
whose **linear part** is wildly stretched. `correct_positions` then extrapolates distant
wells far off the plate. Navigating to such a well (via `_cal_plate_view`/`_navigate_to_well`
→ `safe_travel_to`, or the Jog page) **clamps to the XY envelope corner** — the stage
drives to a wrong extreme. (Navigation already clamps to the envelope, so this is not a
true out-of-range move, but it is a dangerous-looking jump.)

**Fix — reject the fit instead of committing it.** New
`PlateWarpCalibrator.linear_scale_range()` returns the min/max singular value of the
affine linear part. New `CalibrationPage._warp_is_plausible(warp)` rejects a fit when
either axis is scaled outside `[0.8, 1.25]×` or rotation exceeds `30°`; `_manual_fit_xy`
calls it after `warp.solve()` and, on rejection, keeps the previous (predicted) positions
and asks the operator to re-teach with 3+ well-separated wells. The bounds are on the
**linear part only** — translation is inherently limited by the reachable stage range,
and the plate may legitimately be **larger than the travel** (ME3B V1), so an absolute
position / envelope-containment test would wrongly reject every fit.

### C. Custom-tab plate diagram shrank to a speck — `gui/pages/calibration.py`

`_update_cal_view_plate` fed `_cal_plate_view.set_taught_a1(self._taught_a1)` with the
page's A1 in **absolute µm**, but that QGraphicsView positions its predicted/calibrated
dots as `well_abs_mm − a1_mm` (i.e. it expects A1 in **absolute mm**). The 1000× frame
error flung the dots ~kilometers away, ballooned `itemsBoundingRect`, and made
`fitInView` shrink the plate to a dot — matching "the plate view changes size" / "another
coordinate system." **Fix:** convert to mm (`/1000`) at that call site (now consistent
with the other `set_taught_a1` caller).

## Files Modified

| File | Change |
|------|--------|
| `gui/widgets/jog_workspace_view.py` | Clip workspace to content rect; paint Free/Snap toggle last |
| `gui/pages/calibration.py` | `_warp_is_plausible` + `_manual_fit_xy` reject gate; `_cal_plate_view.set_taught_a1` µm→mm fix |
| `SupportClasses/PlateWarpCalibrator.py` | New `linear_scale_range()` (min/max singular value of linear part) |
| `tests/test_v75x_plate_location_manual_click_rim.py` | +4 fit-sanity tests (`TestFitSanityCheck`) |

## Implementation Steps

- [x] Map the two plate views (`_ploc_plate_view` on Plate Location vs `_cal_plate_view` on Custom) and confirm Plate-Location clicks only queue (never move the stage)
- [x] Confirm `move_xy_absolute*` and `safe_travel_to` clamp to the envelope (so the off-range move was a clamp-to-corner from a bad fit)
- [x] Toggle z-order + content clip in `JogWorkspaceView`
- [x] `linear_scale_range()` + `_warp_is_plausible()` reject gate in `_manual_fit_xy`
- [x] Reconsider envelope-containment test → dropped (plate may exceed travel); bound the linear part instead
- [x] `_cal_plate_view` A1 µm→mm frame fix
- [x] Tests + offscreen GUI build smoke

## Testing Notes

- `tests/test_v75x_plate_location_manual_click_rim.py::TestFitSanityCheck` (4) — accepts a
  near-identity fit and a **large translation** (plate offset is legitimate); rejects an
  **anisotropic 3× scale** and a **45° rotation**.
- Full affected suite green (81): manual-click-rim, predict-well, plate-centering,
  freeform-warp (the gate does not reject legitimate warps), well-radius, jog-navigation.
- Offscreen render: `JogWorkspaceView` with an off-envelope well renders without the
  toggle being covered, and the toggle hit-rects stay registered (clickable). Full
  `CalibrationPage` builds offscreen with `_warp_is_plausible` present.
- On hardware: after a 3-well manual fit, if the result is degenerate the page now shows
  "⚠ Calibration rejected: …" and keeps the prior positions instead of driving the stage
  to a corner; a good fit (3 well-separated wells) is accepted as before.

## Issues & Decisions

- **Bound the linear part, not absolute position.** The first cut also rejected fits whose
  corrected wells fell outside the XY envelope — but ME3B V1's travel is smaller than a
  standard plate, so some wells are *always* outside it; that test would reject every
  calibration. Scale (both axes) + rotation are plate-size- and translation-independent.
- **Scope.** Navigation already clamps to the envelope (no true out-of-range motion); the
  fix removes the *cause* (off-plate positions) rather than adding another clamp. The
  `_cal_plate_view` µm/mm fix is a real latent bug surfaced while diagnosing and is the
  best explanation for "the plate view changes size."
- The operator was unsure which view they clicked and confirmed the **physical stage
  moved**, so the fixes target every navigate path (via the shared envelope clamp) and the
  fit that produced the bad target.

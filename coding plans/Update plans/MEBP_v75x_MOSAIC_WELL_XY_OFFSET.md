# MEBP v7.5.x — Mosaic well↔XY correlation off by ≫ one field of view

## Objective

Fix the reported bug: after a full-plate **Mosaic scan**, the stitched image is
well made and the automatic well fit looks great, **but the correlation between
each detected well and its commanded XY stage location is off by much more than
one field of view**. Driving to a fitted well lands several mm away.

## Root cause

The mosaic pipeline is *internally* consistent — tile placement and detected-well
back-projection are exact inverses — **except** for the global alignment shift.

1. `MosaicBuilder._blend_tile_to_composite` places every tile's pixels at the
   **raw stage-encoder frame**: `px = (stage_µm − _canvas_origin_um)·scale`. The
   stage is trusted; the shift is **never** applied to the pixels.
2. `MosaicBuilder.canvas_extent_um` (the property) **adds** `_global_shift_um`
   to the origin so the *overlay* gets a single systematic display correction.
3. `CalibrationPage._ploc_fit_from_mosaic_detections` back-projected each detected
   well with `stage = canvas_extent_um[0:2] + px/scale` — i.e. the **shifted**
   origin. So every well it teaches to the warp is translated by the shift, the
   warp absorbs it as a clean translation (fit looks great, residuals tiny), and
   navigation to the well is off by exactly the shift.

The shift was supposed to be a sub-FOV stitch residual, but `_global_shift_um`
is **pre-seeded** from `MosaicAlignmentStore.get_shift_um(key)` and is *unbounded*.
On ME3B V1 the stored value was **(−3170, −5135) µm** (≈ 2–3 FOV), written by a
`manual_align` overlay nudge and later relabeled `quick_fov`. Proof from disk:

* `config/hardware/well_training/24_.../labels.json`: A1 `um = (5520.3, 4921.4)`,
  which equals `extent_origin + px/scale` with the shift baked in.
* `config/hardware/last_calibration.json`: `taught_a1 = (5520.3, 4921.4)` — the
  saved calibration carries the same offset.
* True A1 (raw placement frame) ≈ `(8690, 10056)`. Difference = the shift.

The overlay and the (corrupt) saved wells were shifted *together*, so everything
looked aligned in the view while every commanded XY was wrong.

## Fix

`gui/pages/calibration.py`:

1. **New `_ploc_mosaic_world_shift()`** — returns the live builder's
   `_global_shift_um` (or `(0,0)` when no builder is live, e.g. GUI/test direct
   calls that already pass an unshifted extent).
2. **`_ploc_fit_from_mosaic_detections`** now back-projects in the trusted-stage
   frame: `ox = extent[0] − shift_x`, `oy = extent[1] − shift_y` — the exact
   inverse of the tile placement. Navigation/calibration coordinates are now
   independent of any overlay/registration/manual shift.
3. **`_ploc_start_mosaic_scan`** bounds the seeded `init_shift` to ±2 FOV; a
   larger stored value is treated as corrupt, logged, and ignored (a stitch
   residual is physically sub-FOV). The overlay still uses the shift for display,
   so the `manual_align` feature is unchanged — it just can no longer corrupt the
   commanded XY of a well.

Data remediation (already applied):
* Removed the corrupt `shift_um (−3170, −5135)` from
  `config/hardware/mosaic_alignment.json` (kept the learned `um_per_px`).

The **existing saved calibration** (`last_calibration.json`) is still offset and
is **not** hand-edited (the warp may be TPS/non-uniform). It self-heals: re-run
the Mosaic scan (or re-teach) with the fixed code and correct positions are
produced and saved.

## Files Modified

| File | Change |
|------|--------|
| `gui/pages/calibration.py` | `_ploc_mosaic_world_shift()` helper; trusted-frame back-projection in `_ploc_fit_from_mosaic_detections`; ±2-FOV bound on the seeded `init_shift` in `_ploc_start_mosaic_scan` |
| `config/hardware/mosaic_alignment.json` | removed corrupt `shift_um`/`frames` |
| `tests/test_v75x_plate_mosaic.py` | +`test_world_shift_helper`, +`test_fit_removes_display_shift_for_navigation` |

## Implementation Steps

- [x] Trace the pixel→stage pipeline; confirm placement vs back-projection frames
- [x] Identify the unbounded display shift folded into `canvas_extent_um`
- [x] Confirm the magnitude empirically from `labels.json` / `last_calibration.json`
- [x] Decouple navigation back-projection from the display shift
- [x] Bound the seeded `init_shift`; clear the corrupt stored value
- [x] Tests (114 in file green, incl. 2 new)
- [ ] **Real-HW verification on ME3B V1**: re-run Mosaic scan, drive to several
      fitted wells, confirm the needle lands centered (was off by ≈3.2 / 5.1 mm).

## Testing Notes

`python -m unittest tests.test_v75x_plate_mosaic` → 114 pass. New tests prove a
live builder shift of (−3170, −5135) is removed so each well back-projects to its
true predicted centre (≤2 µm).

## Issues & Decisions

* **Why not fix the overlay too?** Navigation correctness is the bug. The overlay
  remains display-only; with the corrupt stored shift cleared and the seed
  bounded, a fresh scan's overlay sits in the trusted frame and registers with
  the corrected wells. Ripping the shift out of the overlay would break the
  deliberate `manual_align` feature and its tests for no navigation benefit.
* **Why not patch the saved calibration coordinates?** A re-scan regenerates them
  correctly; reconstructing a possibly-TPS warp by hand risks a subtler error.

## Addendum (2026-06-22) — salvage existing mosaic + manual re-anchor + panel layout

After the back-projection fix, two follow-ons surfaced on ME3B V1:

**(A) Salvage the already-scanned mosaic without re-scanning.** The stitch was
sound (pixels at true stage coords); only the persisted *extent* carried the
(−3170, −5135) shift. Corrected the saved `plate_mosaics.json` extent and both
`well_training/*/labels.json` (extent + per-well `um`) by +(3170, 5135) (backups
`*.bak-mosaicfix`). Independent confirmation: the shifted extent origin was
**(−349, −4129)** — negative absolute stage µm, impossible; corrected →
**(2820, 1006)**, valid. After this, "Map wells…" on the existing overlay
regenerates a correct calibration + saves a template — no re-scan.

**(B) Manual single-point re-anchor — the robust fallback for auto re-register.**
The automatic *Quick re-register* drives to the template's 3 ref-well coords then
auto-detects; if the template anchor is off by more than ~½ FOV the well isn't in
frame and detection misses (and it needs a saved template at all). New
**`Re-anchor map (1 point)…`** button + 2-click flow in `gui/pages/calibration.py`:
1. operator clicks a well / known point on the plate overview → stage fast-travels
   there (`_ploc_safe_goto`, retract→XY) — `_ploc_reanchor_overview`;
2. operator clicks that same feature in the live microscope view —
   `_ploc_reanchor_live_click`: `E = (stage + pixel_to_stage_offset) − commanded
   target`.
`_ploc_apply_global_translation(E)` shifts the WHOLE map by E and **persists via
the warp**: rebuilds the warp as predicted→(current + E) pairs (re-solve), so
`_load_calibration`'s `warp.correct_positions(predicted)` reproduces it after a
restart ("for all time"). `_taught_a1` / the predicted grid are left untouched —
the warp carries the entire correction; `reference_markers` + in-memory
`_xy_teach_points` shift too; `_three_well_calibration` nulled (warp supersedes).
The math: navigation undershoots by T ⇒ commanding `calibrated[W]=true−T` lands the
true well at pixel offset +T ⇒ E=+T ⇒ map+E=true. Guards mirror re-register
(stage/cam/µm-px/Safe-Z); a >30 mm correction asks to confirm (mis-click guard).
Re-register's failure dialogs now point to this button.

**(C) Plate Location right panel too wide.** The six mosaic action buttons were in
one horizontal row, forcing the control panel wide and squeezing the views. Moved
them to a compact 2-column grid in a "Mosaic" group box; capped the panel at
`max width 360` and changed the main splitter stretch 3:2 → 5:1 toward the views.

Files: `gui/pages/calibration.py` (re-anchor flow + state + panel layout),
`config/hardware/plate_mosaics.json` + `well_training/*/labels.json` (re-anchored).
Tests: `tests/test_v75x_plate_mosaic.py` `TestManualReanchor` (2) — proves the
translation shifts all wells AND reproduces through the warp on reload. Affected
suites green (the one `test_real_24_well_mosaic` failure is the documented
pre-existing CV issue). **Needs real-HW verification on ME3B V1.**

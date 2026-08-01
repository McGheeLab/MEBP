# MEBP v7.5.x — Startup well map offset vs the full-plate mosaic

## Objective

Operator (2026-07-29): *"every time I start the software, the wells are not
mapped correctly to the mosaic for the full plate. I have to press map wells
to plate each time to get it to work."*

Root-cause the per-restart misalignment between the calibrated well map and
the full-plate mosaic, and make the mapping the operator confirmed in
Map wells… survive a restart byte-identically.

## Root cause (verified against the operator's own settings.json)

The saved `plate_warp` stores raw (predicted → measured) control-point PAIRS
and `_load_calibration` re-applies the warp to a freshly RECOMPUTED predicted
grid. That contract silently requires the fit-time predicted frame and the
load-time predicted frame to be THE SAME — and they are not:

1. **Fit time:** `CalibrationPage.set_hardware_config` re-seeds
   `_predicted_positions` with the geometry-only **XY-envelope-centred**
   default grid on EVERY config push (`calibration.py` ~1733), even when a
   real calibration exists. Any config propagation after startup (needle /
   ink / pump edits, app.py fan-outs) leaves the session's predicted grid
   envelope-anchored. When the operator then runs **Map wells…**,
   `_ploc_feed_affine → _manual_fit_xy` pairs the measured centres against
   that envelope frame (it only recomputes predictions when they are `None`)
   — and also sets `_taught_a1` to the MEASURED A1.
2. **Load time:** `_load_calibration` calls `_compute_predicted_positions()`
   which anchors the grid at the (measured) `taught_a1`, then evaluates
   `warp.correct_positions(predicted)` — a frame shifted from the warp's
   control points by Δ = measured A1 − envelope-predicted A1.

Since the warp's linear part ≈ identity, every well comes back offset by
≈ Δ. On the operator's machine (settings.json, plate
`plate-24_Rosette in A2`): warp src A1 = (104993.5, 66167.35) — exactly the
envelope-centre grid — while `taught_a1` = measured (105617.56, 65890.35)
⇒ **every well ~(+624, −277) µm (≈0.68 mm) off the mosaic at every startup**.
Pressing Map wells… re-fits in-session (exact at control points → looks
right), saves a new warp again fitted against the envelope frame (because the
next config push re-seeds it), and the cycle repeats every launch.

`_ploc_apply_global_translation` (re-anchor) rebuilds its warp against the
same unstable `_predicted_positions`, so its documented reload contract had
the same fragility.

## Fix (two layers)

**A — stabilise the frame (root):** in `set_hardware_config`, only seed the
envelope-centred geometry grid when NO calibration exists (mirror of the
`recenter_default_plate` gate). When a taught A1 / calibrated map / pending
3-well fit exists, rebuild predictions **anchored at the taught A1** — the
exact frame `_load_calibration` reconstructs — so every subsequent warp fit
and warp reload agree by construction. (When a calibration exists but no
taught A1, leave the predictions alone rather than moving the grid under a
live fit.)

**B — make the restart byte-identical (guarantee + heals installs):**
`_save_calibration` now persists `calibrated_positions` (name → absolute
stage µm) **also when a warp exists** (previously only the no-warp re-derive
path wrote it). `_load_calibration` restores the explicit dict as the
authoritative positions and STILL restores the warp object (needed by
re-anchor folding / labels / plausibility), only falling back to
`warp.correct_positions(predicted)` for legacy saves without the explicit
dict. What the operator saw at save time is what comes back.

Existing installs: the already-on-disk warp is envelope-framed with no
explicit dict, so the FIRST launch after this fix still shows the old offset
— run **Map wells…** (or Re-derive) once more; from then on every restart
reproduces the mapping exactly.

## Files Modified

- `gui/pages/calibration.py` —
  - `set_hardware_config`: gate the envelope-centre `_predicted_positions`
    re-seed on "no calibration"; recompute from taught A1 otherwise.
  - `_save_calibration`: drop the `warp is None` gate on the
    `calibrated_positions` key.
  - `_load_calibration`: warp restore no longer skipped when explicit
    positions are present — the warp object + affine mirror are restored,
    but the explicit positions stay authoritative (positions are only
    recomputed from the warp when no explicit dict was saved).
- `tests/test_v75x_startup_well_map_persistence.py` — NEW (see below).
- `CLAUDE.md` — plan table entry.

## Implementation Steps

- [x] Root-cause from the operator's settings.json (warp src = envelope grid,
      taught_a1 = measured ⇒ Δ ≈ (+624, −277) µm at load).
- [x] Fix A: A1-anchored prediction rebuild in `set_hardware_config`.
- [x] Fix B: persist + restore explicit `calibrated_positions` alongside the
      warp.
- [x] Regression tests.
- [x] Run affected suites.

## Testing Notes

`tests/test_v75x_startup_well_map_persistence.py` (7):
- config push with a live calibration keeps predictions anchored at taught A1
  (not the envelope centre); uncalibrated page keeps the envelope seed;
  calibration-without-A1 leaves predictions untouched.
- `_save_calibration` writes BOTH `plate_warp` and `calibrated_positions`.
- **The operator's exact failure reproduced:** warp fitted against an
  envelope-anchored frame + `taught_a1` = measured A1 → save → fresh page
  load → positions equal the measured mapping (pre-fix they came back
  ~0.68 mm off); the warp object is still restored.
- Legacy save (warp only, no explicit dict) still reconstructs via the warp.
- Orientation-flip guard still drops the explicit dict (existing test).

Regression: `test_v75x_mosaic_orientation_remap.py`,
`test_v75x_plate_mosaic.py`, `test_v75x_freeform_warp.py`,
`test_v75x_last_known_calibration.py`, `test_v75x_plate_types.py`,
`test_v75x_plate_location_predict_well.py`, `test_v75x_plate_centering.py`,
`test_v75x_rosette_tab_auto_reanchor.py`,
`test_v75x_single_well_mosaic_reregister.py`.

**Needs real-HW verification on ME3B V1:** after updating, run Map wells…
once; restart the app → the well rings sit on the mosaic wells with no
re-map; restart again to confirm stability; a re-anchor followed by a restart
keeps the shifted map.

## Issues & Decisions

- Pre-existing HANG confirmed NOT from this change (reproduced in a clean
  HEAD worktree): `test_v75x_plate_mosaic.py::TestManualAlignPage::
  test_auto_finish_keeps_manual_align` blocks forever in
  `_ploc_on_mosaic_finished → _ploc_open_well_mapping → dlg.exec()` — the
  v7.5.x "auto-open Map wells on scan finish" raises a MODAL dialog, and on
  a machine whose REAL `config/hardware/plate_mosaics.json` has a stored
  mosaic for the test's plate key the store gate passes, so the headless
  test waits on the dialog. Needs its own fix (isolate the mosaic store in
  `_CalBase` and/or stub `_ploc_open_well_mapping` in that test; arguably
  the auto-open should also be suppressible). The rest of the suite (104
  tests, `TestFilledWellDetector` excluded = the long-documented
  `test_real_24_well_mosaic` CV failure) runs green with this change.
- Pre-existing failure confirmed NOT from this change:
  `test_v75x_plate_template_reregister.py::TestRosetteSubwell::
  test_scan_well_guarded_without_rosette` calls
  `page._ploc_scan_rosette_well()`, a method that exists nowhere in the
  repo (zero grep hits in the working tree AND at HEAD) — it was superseded
  by `_ploc_scan_single_well(name, is_rosette)` in the rosette-tab rework;
  the test was never updated. All other 66 tests in that batch pass.

- Considered auto-applying the mosaic-stored `wells_um` at startup (what the
  manual button chain does) — rejected: it would silently clobber any NEWER
  manual teach / click-rim calibration done after the last Map wells (no
  timestamps to arbitrate). Fixing the persistence contract keeps every
  teach path authoritative.
- Explicit-positions load previously nulled `_plate_warp`; now the warp is
  restored beside them because `_ploc_apply_global_translation` folds
  re-anchor corrections into it. The no-warp re-derive save is unchanged
  (nothing to restore).

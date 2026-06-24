# MEBP v7.5.x — Full-plate mosaic scan + detect-all-wells + toggleable overlay

## Objective

The Plate Location **Auto (per-well edge-detect)** path drives to each predicted
well centre and runs `HoughCircles` on a *single* frame; if the rim isn't
crisply framed (focus/lighting/partial rim/off prediction) the per-well detect
fails. Operator's fix, implemented here:

1. **Raster the whole plate**, capture a camera snapshot at each tile, and
   **stitch** them into one composite (using the accurate stage encoders for
   placement).
2. **Circle-detect every well at once** on that large, blended, higher-SNR
   field, map detections back to stage µm, and fit the plate warp.
3. Expose the stitched composite as a **toggleable background overlay** on the
   Plate Location map *and* the Jog page.

The existing `SupportClasses/MosaicBuilder.py` already did raster generation,
stage-coordinate stitching (phase-correlation + feathered blend), and the
`canvas_extent_um` / `_mosaic_scale` transform — this work wires it into an
operator-facing flow, adds multi-circle detection + persistence + an overlay.

## Files Modified

| File | Rationale |
|------|-----------|
| `SupportClasses/MosaicStore.py` | **NEW** — per-machine persistence of the stitched mosaic: PNG under `config/hardware/mosaics/<plate_key>.png` + metadata (absolute-µm extent, um/px, scale, frames) in `config/hardware/plate_mosaics.json`. Atomic metadata write; keyed by plate key; zero GUI deps. Singleton via `get_store()`. |
| `SupportClasses/VisionDetector.py` | New `WellDetector.detect_wells(...)` — multi-circle HoughCircles returning *every* hit within a radius window (vs `detect_well`'s single best-near-centre), for detecting all wells on a stitched mosaic. **(rev)** New `detect_filled_wells(...)` — robust FILLED-disc detection (brightness max-channel Otsu → morphology → connected components → area gate → **chord-robust circle fit** `_fit_circle_robust`: minEnclosingCircle seed + inlier-only Taubin refine, so a well cut by a straight chord — clipped at the image border OR an interior mosaic margin — recovers its TRUE radius/centre and the circle hugs the visible arc, extending off-image; gate on the circle inlier-fraction); colour/illumination-agnostic, replaces Hough on mosaics. New `fit_well_grid(centers, n_rows, n_cols)` — fits the known plate lattice (iterative snap + LSQ affine, percentile seed + inlier refit) → per-blob (row,col), rejects off-grid spurious, fills missing, handles rotation. Validated on the real 24-well mosaic (24/24). |
| `gui/widgets/jog_workspace_view.py` | Module `pixmap_from_bgr()` (numpy BGR → QPixmap via `Format_BGR888`). Overlay state + `set_mosaic_overlay(pixmap, extent_abs_um)` / `set_mosaic_visible()` / `has_mosaic()`. `_paint_mosaic_overlay()` drawn between envelope and wells: extent is absolute µm → shifted by `_zero_off` (like the envelope), and the image is rotated 180° about the rect centre under `_FLIP_DISPLAY_180` so it registers with the wells/needle. |
| `gui/pages/calibration.py` | Plate Location tab: new **Mosaic scan** button + **Show mosaic** checkbox. New `_ploc_start_mosaic_scan` / `_ploc_mosaic_tick` / `_ploc_finish_mosaic_scan` / `_ploc_cancel_mosaic_scan` (QTimer raster loop modelled on `_start_well_scan`; reuses `_safe_navigate_to(lower_z=False)`, `capture_fresh_frame`, the objective gate, and the camera/µm-px resolution from `_ploc_run_queue`). `_ploc_detect_and_fit_from_mosaic` (px→abs-µm via extent/scale, match each predicted well → nearest unused detection within tolerance, route matched pairs through the existing `_ploc_feed_affine`→`_manual_fit_xy` warp path). `_ploc_apply_mosaic` (persist + overlay + emit `calibration_data_changed`). `_ploc_set_overlay_image` / `_ploc_load_persisted_mosaic` / `_ploc_plate_key` (prefers live `WellPlate.format`). Mosaic reload wired into the build, `set_hardware_config`, and `_on_plate_changed`. Cancel button also stops a mosaic scan. |
| `gui/pages/jog_control.py` | **Show plate mosaic** checkbox in the XY Workspace card. `_load_mosaic_overlay()` (reads `MosaicStore` for the active plate; enables/disables the toggle) called from `set_calibration_data` (so it refreshes after a scan via `calibration_data_changed`) and `load_startup_plate`. `_jog_plate_key` / `_on_show_mosaic_toggled`. |
| `SupportClasses/MosaicAlignmentStore.py` | **NEW** (rev) — learned per-camera+objective correction (`config/hardware/mosaic_alignment.json`): effective `um_per_px` + residual global `shift_um`, keyed `<cam-identity>\|<objective>`. Seeds future mosaics; written by Quick FOV cal, full-scan registration (`mosaic_scan`), and manual align (`manual_align`). |
| `gui/dialogs/mosaic_calibration_dialog.py` | **NEW** (rev) — Calibrate… pop-out: builds a small user-defined NxN mosaic, runs the global registration; `applied_settings()` / `result_overlay()` hand tuned settings + the composite back to the page. **2-col:** settings + **X/Y gap % + opacity sliders** on the LEFT, the live `MosaicRegistrationView` (individual tiles + outlines, zoom/pan) on the RIGHT, buttons pinned. **Spacing calibration:** sliders move tiles toward/away to align overlaps → corrected effective **µm/px = assumed/mean(k)**, stored via `set_um_per_px` (+ FOV field for "Apply to full mosaic"), seeded from stored µm/px, live readout (µm/px·FOV·step). Auto global-shift store + `manual_align`-authoritative + Clear-stored retained. No longer uses `JogWorkspaceView`. |
| `gui/widgets/mosaic_registration_view.py` | **NEW** (rev) — `QGraphicsView` mosaic viewer for the Calibrate… pop-out: renders **individual semi-transparent tiles** + a peach outline each, wheel-zoom about cursor + drag-pan + fit, and `set_spacing_factor(kx,ky)` scaling tile positions about the centroid (move images toward/away to align overlaps). |
| `SupportClasses/MosaicBuilder.py` | (rev) `tile_rects_px()` (footprints) + `tile_images_px()` (`(frame,left,top,w,h)` per tile) — stage-placed (same math as `_blend_tile_to_composite`), for the mosaic view's individual tiles + outlines. |
| `gui/dialogs/mosaic_settings_dialog.py` | **NEW** (rev) — full mosaic-scan settings (FOV/overlap/settle/fresh-frames/spacing/register/max-shift/orient/cal-grid). |
| `gui/widgets/jog_workspace_view.py` | (rev) manual-align: `_mosaic_user_shift` + `set_mosaic_shift` / `mosaic_shift` / `set_mosaic_opacity`; `_paint_mosaic_overlay` adds the user shift to the world extent before `_um_to_px`; `set_mosaic_overlay` zeros the nudge. **Opt-in zoom/pan** (default off): `set_zoom_enabled` / `set_zoom` / `reset_view`; `_scale()` folds in `_zoom`+`_pan`; `wheelEvent` zooms about the cursor; mouse handling refactored (`_handle_click_at`, left-drag pans when zoom on, <4 px = click) so the zoom-off click-to-teach/travel path is byte-unchanged. |
| `gui/pages/calibration.py` | (rev) `_ploc_microscope_um_per_px` (resolution rescale); Manual align group (Δx/Δy/opacity sliders + Store/Reset) + `_ploc_on_align_changed` / `_ploc_store_manual_align` (idempotent, prior+nudge, re-bakes cached overlay) / `_ploc_reset_manual_align`; overlay cache in `_ploc_set_overlay_image`; `_ploc_on_mosaic_finished` gates auto-store on `n_meas>0` + manual hint. |
| `SupportClasses/WellTrainingStore.py` | **NEW** (rev) — labeled mosaic dataset for detection R&D: per-sample folder `config/hardware/well_training/<plate>_<ts>/` with `mosaic.png` + `labels.json` (extent/scale/µm-px + per-well centres in mosaic px AND stage µm). `save_sample` / `list_samples` / `get_store`. |
| `gui/dialogs/mosaic_well_mapping_dialog.py` | **NEW** (rev) — manual well mapping on the stitched mosaic: zoom/pan image view (`_MappingView`: wheel-zoom, middle-drag pan, left-click), click the **3 corner wells** → `solve_affine_3` auto-fills ALL wells (`apply_affine` over `WellPlate.get_well_position`) as draggable circles → Confirm maps mosaic px→stage µm (`extent[0:2]+px/scale`) into `results()` + saves a `WellTrainingStore` sample. `corner_well_names` picks 3 non-collinear corners by nominal extremes. |
| `gui/pages/calibration.py` | (rev) Plate Location **Map wells…** button → `_ploc_open_well_mapping` (loads stored mosaic from `MosaicStore`, opens the dialog, on confirm feeds `_ploc_feed_affine(results)` + reference markers + logs the training sample). Mosaic-finish status hints "Map wells…" when `<3` wells auto-detected. |
| `gui/dialogs/mosaic_well_mapping_dialog.py` | (rev) **Auto-detect wells** button + auto-run on open (`_auto_detect`): runs `WellDetector.detect_filled_wells` + `fit_well_grid`, places every well (detected centre or grid-filled) named via the plate layout; the 3-corner click workflow stays as the fallback. |
| `gui/pages/calibration.py` | (rev) `_ploc_detect_and_fit_from_mosaic` + the off-thread `_MosaicScanWorker` now try `detect_filled_wells` (grid-filtered when the plate grid is known) FIRST, Hough fallback — so Mosaic scan auto-finds wells. |
| `coding plans/well_detection_rnd/` | **NEW** — detection R&D area: `well_finder_viz.py` (annotated-overlay debug tool over the shipping detector) + `README.md`. |
| `SupportClasses/PlateTemplateStore.py` | **NEW** (rev) — reusable as-built well-map template keyed `<plate>\|<camera>\|<objective>` (`config/hardware/plate_templates.json`): well centres (stage µm) + 3 reference wells + µm/px. "Scan once" → later re-register from 3 wells. `save_template`/`get_wells`/`get_ref_wells`/`has`/`clear`/`corner_ref_wells`. |
| `SupportClasses/PlateWarpCalibrator.py` | (rev) module fn `register_from_template(template_wells, measured_wells)` — fits a warp template→measured (shared names; ≥3 affine, 2 similarity) via `PlateWarpCalibrator` and applies it to ALL template wells → registered positions. |
| `gui/pages/calibration.py` | (rev) Plate Location **Quick re-register** button + `_ploc_quick_reregister` (drives to the template's 3 ref wells, `capture_fresh_frame` → `detect_filled_wells` → `pixel_to_stage_offset` → measured stage µm, `register_from_template` → `_calibrated_positions`; gated on template + camera/objective + Safe-Z; `_warp_is_plausible` reject). Saves a template on Map-wells confirm (`_ploc_save_plate_template`); `_ploc_refresh_reregister_button` enables the button only when a matching template exists. |
| `tests/test_v75x_plate_mosaic.py` | **NEW** — 112 tests (rev: +manual align, µm/px rescale, calibration handoff, source-precedence/clear, zoom/pan, height cap, tile_rects/images_px, MosaicRegistrationView spacing, spacing-calibration µm/px+FOV + seed, WellTrainingStore, well-mapping affine/dialog/page-guard, **filled-well detector real+synthetic + clipped-radius recovery + grid robustness + dialog auto-detect**). |
| `tests/test_v75x_plate_template_reregister.py` | **NEW** (rev) — 15 tests: PlateTemplateStore round-trip + camera/objective keying, `register_from_template` recovers a known transform from 3 wells, page save-template + button gate + quick-re-register guard, **rosette `_SubPlate` adapter + parent detection + sub-well scan bounds + Scan-well guard**. |
| `gui/pages/calibration.py` | (rev) **Per-well rosette mosaic:** `_SubPlate` adapter (sub-wells of one parent as a minimal plate); Plate Location **Scan well…** button → `_ploc_scan_rosette_well` (pick rosette parent → single-well bounds `_ploc_subwell_scan_bounds` → `_ploc_scan_bounds_override` + reuse the mosaic scan); `_ploc_on_mosaic_finished` routes a sub-well scan to `_ploc_open_subwell_mapping` (Map-wells dialog over `_SubPlate` → merge sub-well centres into `_calibrated_positions`); `_ploc_rosette_parent_wells`. |

## Implementation Steps

- [x] `MosaicStore` (PNG + extent persistence, singleton).
- [x] `WellDetector.detect_wells` multi-circle detector.
- [x] `JogWorkspaceView` overlay (API + paint + `pixmap_from_bgr`).
- [x] Plate Location raster scan → stitch → detect-all-wells → warp fit → persist → overlay (new button + Cancel handling + live build feedback).
- [x] Plate Location "Show mosaic" toggle + persisted-mosaic reload on build / plate change / config change.
- [x] Jog page "Show plate mosaic" toggle + load from store.
- [x] Tests + regression.

## Coordinate / registration notes

- **px → stage µm:** `MosaicBuilder` places tiles by `px = (stage−origin)·scale`
  with image pixel (0,0) = world `(extent[0], extent[1])`, no Y flip. Inverse:
  `stage = (extent[0]+px/scale, extent[1]+py/scale)`. Used for both detection
  back-projection and overlay placement.
- **Overlay extent** is stored/passed **absolute** stage µm; the widget shifts
  by `_zero_off` at paint time (matches the envelope), so the overlay tracks
  re-zeroing live.
- **180° display flip:** the overlay is rotated 180° about the screen rect
  centre when `_FLIP_DISPLAY_180` is on, so the world-min corner lands on its
  flipped screen position — registering pixel-for-pixel with the wells/needle.

## Testing Notes

- `python -m unittest tests.test_v75x_plate_mosaic` → 16 green (MosaicStore
  round-trip/persistence/clear; `detect_wells` finds 4 synthetic circles;
  overlay set/clear/paint + `pixmap_from_bgr`; detect→stage→match fit lands
  each well back within 2 µm + sets `_calibrated_positions`; UI button/toggle;
  Jog overlay load enables the toggle).
- Regression: `test_v75x_plate_location_manual_click_rim` (27),
  `test_v75x_plate_location_z_side_view` (6), `test_v731_jog_navigation`,
  `test_v75x_plate_centering`, `test_v75x_z_retract_before_xy_travel`,
  `test_v731_mosaic`, `test_mosaic_diagnostic` → 110 green. Full `test_v75x*`
  = 529 green.

## Issues & Decisions

- **Position-based stitching (not feature-based)** — the Prior II encoders are
  accurate, so `MosaicBuilder` places each tile by stage coordinates (with
  phase-correlation refinement in `build_mosaic`). Robust where per-frame
  feature matching is fragile.
- **New button, not a replacement** (per operator) — the failing "Auto"
  per-well mode is kept as a fallback; "Mosaic scan" is additive.
- **Persisted to disk** (per operator) — survives restart and is shown on Jog +
  Plate Location. Machine-level store (like `CameraCalibrationStore`), keyed by
  plate, since the mosaic's absolute-µm extent is a property of the physical
  plate on the stage.
- **Imaging Z:** the raster stays **retracted** at Safe Z (`lower_z=False`) —
  the microscope images the plate from overhead, so the needle never descends
  during the cross-plate sweep (honours the Z-retract-before-XY safety rule).
- **Scan runs on the Qt timer / GUI thread** (single-shot tick per tile), like
  the existing 3-well and per-well scans — no new threading.
- **Detection still uses HoughCircles**, but on the full stitched field where
  every well is large and well-contrasted; `target_mosaic_px=3000` keeps the
  composite a manageable size while preserving rim detail.

**Needs real-HW verification on ME3B V1** (stitch quality, detection yield,
overlay registration against the live plate). Logic verified by tests + the
shared `MosaicBuilder`/coordinate pipeline.

---

## Revision 2 — operator bench feedback (threaded scan + FOV/overlap + bounds)

Three reported defects + two follow-on asks, all fixed:

1. **Camera couldn't keep up / stale frames.** The GUI-thread QTimer tick did a
   single `capture_fresh_frame()` (a direct backend `read()`) right after the
   move — OpenCV returns *buffered* (pre-move) frames. Now: `CameraWidget` has a
   monotonic frame counter (`_frame_seq`, incremented in `_grab_frame`) +
   thread-safe `frame_count_value()`; after each move the worker waits for N
   genuinely-new frames from the display grab timer (draining the backlog) then
   samples `get_current_frame()`. `capture_fresh_frame(discard_n_frames, settle_ms)`
   gained a drain loop + settle (used by the per-well scan: `(5, 120)`).

2. **Freezing.** The scan ran on the GUI thread, blocking the camera grab timer
   ("the image grabber") + UI. Now the whole loop (move → capture → stitch) +
   detection runs on a background `_MosaicScanWorker(QThread)`. It only commands
   thread-safe stage moves and samples frames thread-safely — it never touches
   the camera backend (no race with the grabber). cv2/numpy release the GIL, so
   stitch/detect genuinely run off the GUI thread.

3. **FOV / 25% overlap.** Raster spaced by the true camera FOV
   (`frame_px × µm/px`) at `_PLOC_MOSAIC_OVERLAP = 0.25` (was 0.30).

4. **Edge clamping.** Scan bounds are clipped to the reachable XY envelope
   (`_ploc_clip_scan_bounds`; ME3B V1 travel < plate footprint), and grid points
   outside the envelope are filtered — no boundary clamping.

5. **Live mosaic view.** A "Mosaic (live)" preview `QLabel` next to the
   microscope feed updates per stitched tile (plus the registered plate overlay).

### Adversarial review fixes (5 confirmed of 19 findings)

- **HIGH:** `_MosaicScanWorker` was not stopped on app shutdown
  (`MainWindow.closeEvent` only stops detection workers) → "QThread destroyed
  while running" if closed mid-scan. Added `CalibrationPage._shutdown_mosaic_worker()`
  (stop+wait+disconnect), called from `closeEvent` and `MainWindow.closeEvent`'s
  page loop.
- **MEDIUM (coordinate frame):** detection now runs on the **incremental**
  composite (`builder.composite`, placed via `_mosaic_scale`/`_canvas_origin_um`),
  NOT `build_mosaic()` — whose phase-correlation branch recomputes a local
  origin/scale it never writes back, desyncing detections from `canvas_extent_um`
  (≈half-FOV shift + wrong scale). Position-based placement is reliable with
  accurate encoders.
- **MEDIUM (silent timeout):** the worker warns per dropped tile and aborts
  (`failed`) after `_MAX_CONSEC_NONE` (8) consecutive no-frame tiles; the finish
  message reports dropped count (was: silent partial mosaic at 100%).
- **MEDIUM (re-entrancy):** `_ploc_start_mosaic_scan` now claims the busy flag +
  disables Run BEFORE the modal objective dialog, and `_ploc_run_queue` guards on
  `_ploc_mosaic_running` — a Run click during the dialog can no longer launch a
  concurrent queue run (conflicting XY).
- **LOW:** `_ploc_on_mosaic_tile/_progress/_finished/_failed` guard on
  `_ploc_mosaic_running` so stale queued signals after a cancel are ignored.

Tests: `tests/test_v75x_plate_mosaic.py` (30 — incl. worker move/capture/finish,
consecutive-None abort, stop-early, fresh-frame drain, bounds clip + empty,
25% overlap, re-entrancy guard, shutdown). Full `test_v75x*` = 543 green.
**Still needs real-HW verification on ME3B V1.**

### Hotfix — "Mosaic scan does nothing" (silent no-op)

Root cause: `_ploc_reset_mosaic_preview` called `QPixmap()` but `QPixmap` was
**not imported** in `calibration.py` → `NameError` inside the button's slot →
Qt swallows it (traceback to the terminal, no UI) → click appears to do nothing;
worse, the exception fired *after* the busy flag was set, so every later click
then hit the silent re-entry guard. Found by an offscreen smoke that drives
`_ploc_start_mosaic_scan` with mocks (camera ready/calibrated). Fixes:
- **Import `QPixmap`** in `calibration.py` (the actual bug).
- **`_ploc_start_mosaic_scan` is now a thin wrapper** around
  `_ploc_start_mosaic_scan_impl` with try/except that logs + shows a dialog and
  resets the busy flag on ANY failure — a click can never again be a silent
  no-op or wedge the tab. The re-entry guard now shows an info dialog too.
- **Camera-not-ready tolerance:** if `get_current_frame()` is None right after
  start, retry once via `capture_fresh_frame(settle_ms=300)` before the
  "not producing frames" message.
- **Tile-count confirmation:** a full-plate mosaic at the microscope FOV can be
  thousands of tiles (a 6-well plate ≈ 4154; the wells span the plate, clipped
  to the reachable envelope), i.e. a long unattended run — a Yes/No dialog now
  shows the tile count + rough time estimate before starting so it isn't
  mistaken for a hang. Regression test: `test_preview_helpers_do_not_raise` (31
  total in the suite).

### Settings dialog + camera-timing controls

Operator reported the camera wasn't getting enough time to capture each tile.
Added a **"Settings…"** button next to "Mosaic scan" that pops out
`gui/dialogs/mosaic_settings_dialog.py::MosaicScanSettingsDialog` (modal) to tune
every scan parameter, persisted in `settings.json` (`mosaic_scan` section) and
applied to the next scan:

- **Camera timing:** *Settle after move* (ms, default **300** — the key fix:
  the worker now sleeps this long after each move before counting fresh frames,
  giving a slow/long-exposure camera time to expose a sharp post-move frame),
  *Fresh frames to wait* (default 3), *Frame wait timeout* (s, default 2.5).
- **Raster:** *Tile overlap* (%, default 25), *Mosaic resolution* (px, 3000).
- **Well detection:** *Detection sensitivity* (HoughCircles `param2`, default 30),
  *Radius tolerance* (%, default 35) — passed through to `WellDetector.detect_wells`.

`_MosaicScanWorker` gained `settle_ms` / `detect_param2` / `detect_tolerance`
params; `_ploc_start_mosaic_scan_impl` reads `self._mosaic_settings` (loaded via
`merged_settings(settings.get_section("mosaic_scan"))`) for overlap / target_px /
timing / detection. Tests: `tests/test_v75x_plate_mosaic.py` (36 — incl.
merged_settings, dialog round-trip/defaults, worker param storage, page load
from store, open-dialog apply+persist). Full `test_v75x*` = 572 green.

### Raster direction / orientation ("images out of order")

Operator: "the mosaic rasters top-left→bottom-right but the stage rasters
bottom-right→top-left, so the images are out of order." Placement is by actual
stage coords (visit order is irrelevant), so this is an *orientation* mismatch:
the composite is built in stage-coordinate space (min-stage at top-left), but on
this stage +X/+Y point toward the operator's top-left (why `JogWorkspaceView`
uses `_FLIP_DISPLAY_180`). The plate-map **overlay** already flips 180° (registers
correctly), but the **live preview** showed the raw composite — so it filled the
opposite corner from the stage. Fixes:
- **Live preview now flips 180°** (gated on `JogWorkspaceView._FLIP_DISPLAY_180`)
  so it builds in the same direction the stage moves and matches the plate map.
- **Per-tile "Tile orientation (camera mount)"** setting (None / Rotate 180° /
  Flip H / Flip V), applied to each captured frame in the worker
  (`_orient_frame`) before stitching — the safety net if the camera is mounted
  rotated/mirrored so neighbouring tiles don't line up. Default **None** (no
  change to coherent setups). Persisted with the other mosaic settings.

Tests: `tests/test_v75x_plate_mosaic.py` (37, +`test_frame_orient_transforms_tile`);
full `test_v75x*` = 583 green.

### Grid spacing / FOV override + registration ("transparency + relaxation")

Operator: grid points didn't match the actual camera FOV; wants to set grid
spacing; and tiles should blend with transparency + register/align rather than
being pasted on top. Added (all in the Settings dialog, persisted):

- **Camera FOV width (µm)** — `fov_um` (0 = auto from µm/px). A manual FOV sets the
  effective µm/px for the mosaic (`eff = fov_um / frame_w`), which fixes BOTH the
  tile size on the canvas AND the FOV-derived auto spacing — the real cure for an
  inaccurate FOV.
- **Grid spacing (µm)** — `spacing_um` (0 = auto = FOV × (1 − overlap)).
  `MosaicBuilder.generate_raster_positions` gained `step_x_um`/`step_y_um`
  overrides; the page passes the configured spacing.
- **Global alignment (registration)** — `register` (default **on**) + **Max
  alignment shift (µm)** `max_shift_um` (0 = auto = 20% FOV). **GLOBAL, not
  per-tile** (operator: the stage is accurate, so relative placement is trusted
  and a single bad/featureless tile must never be moved on its own).
  `MosaicBuilder` gained `register`/`max_shift_um`/`register_mode`;
  `_blend_tile_to_composite` places every tile at its raw stage position and
  (global mode) only *measures* each textured overlap via new
  `_measure_tile_shift` (reusing `_phase_correlate_overlap`).
  `finalize_global_shift()` aggregates those into ONE robust **median** shift,
  bounded by `max_shift_um`, applied uniformly through the `canvas_extent_um`
  property (no tile re-placed; detection + overlay get the single systematic
  correction). **Featureless tiles** (between wells / no edge) fall below a
  grayscale-std gate (`_REGISTER_MIN_STD`) → they contribute no measurement,
  can't inject a bogus shift, and just inherit the global shift. The
  **per-tile** mode (`register_mode="per_tile"`, bounded inline nudge) is
  **retained for future apps** (e.g. a drifting/open-loop stage) but unused by
  the plate scan. Overlaps blend with the existing feathered transparency.

`_ploc_start_mosaic_scan_impl` builds the `MosaicBuilder` with `eff_um_per_px` /
`register` / `max_shift_um` (default `register_mode="global"`) and passes
`step_x_um=step_y_um=spacing_um`; the worker calls `finalize_global_shift()`
after stitching, before detection/emit. Tests: `tests/test_v75x_plate_mosaic.py`
(44 — +step-override, +max-shift math, +global-shift median/extent,
+featureless-no-shift, +per-tile-mode retained); full `test_v75x*` = 610 green.

### Learned correction per camera + objective + Quick FOV calibration

So a mosaic is accurate from the first build, the per-camera+objective
correction is learned and reused. New `SupportClasses/MosaicAlignmentStore.py`
(`config/hardware/mosaic_alignment.json`, keyed by `"<camera-identity>|<objective>"`,
falling back to objective): stores the effective **µm/px** + the residual
**global shift** (µm).

- **Calibrate… pop-out** (`gui/dialogs/mosaic_calibration_dialog.py::MosaicCalibrationDialog`,
  opened by `_ploc_quick_mosaic_calibrate`): the operator defines a **small NxN
  grid (default 5×5)** + the mosaic settings (overlap, settle, fresh frames, FOV,
  max shift, centre = plate-centre well / current XY); it builds that small
  mosaic (centred via `centered_scan_bounds`, clipped to the XY envelope) on its
  own `_MosaicScanWorker` (detection skipped, `expected_d_px=0`), runs the
  **global registration**, and stores the resulting **global delta** (shift, µm)
  for the current camera+objective. Live preview + result readout in the dialog.
  A 5×5 (or larger) grid gives enough textured overlaps to estimate the
  systematic shift robustly. Replaces the earlier 2-move µm/px quick-cal.
  (Smoke-verified end-to-end: 3×3 grid builds + stores a delta.)
- **Reuse:** `_ploc_start_mosaic_scan_impl` picks the effective µm/px as
  explicit FOV override > learned (store) > camera µm/px, so tiles are sized
  correctly and the live global registration has little to do; it also
  pre-seeds `MosaicBuilder(initial_shift_um=...)` with the learned global shift
  (so the mosaic is registered from the first tile — `finalize_global_shift`
  keeps the pre-seed when no fresh measurements, else replaces it with the fresh
  median).
- **Learn from a full scan:** `_ploc_on_mosaic_finished` stores the finalized
  global shift back to the store for that camera+objective.

Tests: `tests/test_v75x_plate_mosaic.py` (store round-trip/persist/clear,
initial-shift pre-seed/keep, camera-objective key, centered-grid math, dialog
build + grid defaults, finished→store delta, settings preserve undisplayed keys).
Full `test_v75x*` = 620 green.

**Adversarial review fixes (4 confirmed):** (1+2+3, CRITICAL/HIGH, same root)
the "Calibrate…" opener lacked the Safe-Z gate the full scan has, so with the
ZP connected and Safe Z unset, `safe_z` fell back to `0.0` and the worker's
tile-0 `safe_travel_to(0.0)` would drive the needle to the plate datum on ME3B V1
(ZDIR=-1) → crash. Fixed: `_ploc_quick_mosaic_calibrate` now mirrors the
`zp_connected and _safe_z is None` gate, AND `MosaicCalibrationDialog._start_calibration`
refuses to run in that state (defense-in-depth). (4, HIGH) the dialog stored the
finalized shift unconditionally, so a calibration over a featureless region
(0 registered overlaps → `(0,0)`) overwrote a previously-good learned shift.
Fixed: the dialog pre-seeds the builder with the stored shift
(`initial_shift_um`) and only writes back when `n_meas > 0` (else keeps the prior
and tells the operator to retry over texture). Tests +Safe-Z-gate, +no-overlap-
preserves-prior; full `test_v75x*` = 622 green.

### Fix — ZP "Write timeout" stopped the build mid-raster

Bench log: a 1292-tile scan died ~8 min in with `ZPStage: ZP send_data error:
Write timeout`. Root cause: the worker called `safe_travel_to` for **every
tile**, which re-confirms Z on the **ZP/Marlin board (COM5)** each tile — even
though the needle never moves in Z during the raster (constant safe Z). Those
~1292 ZP confirms, plus the background position poller's M114 reads, congested
the ZP serial until a write timed out (XY moves go to the ProScan on COM4 — the
ZP is otherwise idle, so it never needed touching per tile). Fix
(`_MosaicScanWorker.run`): only **tile 0** does a full `safe_travel_to` (retract
+ confirm safe Z; the page also pre-retracts); **every later tile** does a PURE
`move_xy_absolute_um` + `wait_for_xy_arrival` (ProScan only — no ZP traffic).
Also **suspend the position poller** for the whole raster (`suspend/resume_position_poller`
in try/finally) so its ZP reads don't contend either. Safety rule intact: Z is
retracted+confirmed before the raster and held constant throughout (never
lowered). Tests updated (`_FakeCtrl` gains `move_xy_absolute_um`/
`wait_for_xy_arrival`/poller methods; asserts tile0=safe, rest=xy, poller
resumed); full `test_v75x*` = 616 green. **Needs real-HW re-run on ME3B V1.**

### µm/px resolution rescale + manual global registration (slide-by-eye)

Two operator-driven follow-ons after "the global registration didn't work" + "the
grid spacing needs 50% overlap just to keep the edges in view — are we calculating
the grid from the camera pixel size or the objective pixel size?".

**(A) µm/px resolution mismatch (root cause of needing 50% overlap).** The mosaic
FOV (hence tile step) was sized from the objective calibration's
`measured_um_per_px`, but that value was measured at one capture resolution; the
BUC3D camera had since switched 912→1832 px wide (the SDK even logs "µm/px may need
redoing"). µm/px scales **inversely** with capture width, so the mosaic thought each
tile covered ~2× the real area → tiles barely overlapped unless the operator dialed
in ~50% overlap. New `CalibrationPage._ploc_microscope_um_per_px(frame_w, fallback)`
rescales the objective µm/px by `cal_width / current_width` (reads
`ObjectiveCalibration.get_calibration(camera_spec.name, objective)` →
`measured_um_per_px` + recorded `resolution`); falls back to the live
CameraManager value (not resolution-scaled) when no objective cal with a resolution
is on file. Wired into the full scan (`eff_um_per_px` fallback) and the Calibrate…
opener (`um_per_px_camera`). Width-only rescale (assumes aspect-preserving res
change — true for this camera's eSizes).

**(B) Manual global registration.** When auto phase-correlation can't lock on
(featureless tiles between/within wells), the operator slides the **whole** stitched
mosaic onto the rendered well grid **by eye** — one global shift, never per-tile
(the stage is accurate). New `JogWorkspaceView` API: `_mosaic_user_shift` +
`set_mosaic_shift(dx_um, dy_um)` / `mosaic_shift()` / `set_mosaic_opacity(a)`;
`_paint_mosaic_overlay` adds the user shift to the world extent **before**
`_um_to_px` (so the nudged preview and a later baked-extent redraw land at the exact
same screen pixel — same transform); `set_mosaic_overlay` zeros the nudge on every
fresh overlay. New "Manual align" group on the Plate Location control panel:
Δx/Δy µm sliders (±10 mm, 5 µm single-step / 200 µm page-step), an opacity slider
(5–100 %, live), **Store alignment**, **Reset**. `_ploc_on_align_changed` live-pushes
the sliders to the overlay; `_ploc_store_manual_align` writes the corrected shift
(`prior_stored + nudge`) to `MosaicAlignmentStore.set_shift_um(key, …,
source="manual_align")` — which seeds `MosaicBuilder(initial_shift_um=…)` on the
next full scan — and re-bakes the **cached** overlay (numpy + extent, cached in
`_ploc_set_overlay_image`) at the shifted extent (persisting to MosaicStore when the
plate has a stored mosaic), then re-pushes it (zeroing the live nudge + sliders).
The calibration pop-out already pushes its composite to the **same** plate overlay,
so manual align serves **both** the full and the calibration mosaic.

**Auto/manual interaction (deliberate):** `_ploc_on_mosaic_finished` now stores the
auto shift only when `n_meas > 0` (never overwrites a manual/learned shift with a
null) and appends a "use Manual align" hint when 0 overlaps registered; the
Calibrate… dialog gives the same guidance + always hands back its composite
(`result_overlay()`) even on 0 overlaps. A later scan that *does* register replaces
the stored shift with the absolute measured median (auto-working > manual fallback);
the manual shift persists for as long as auto keeps failing, which is the target
scenario.

Adversarial review (1 round, 16 tool-uses): no safety defect (manual-align path is
display + persistence only — no stage/Z motion); invariants confirmed (nudged
preview ≡ baked redraw; sliders never desync from the view). Two fixes applied:
**M1** slider range ±3 mm → ±10 mm (a systematic camera↔stage offset can exceed
3 mm); **M3** the store is now **idempotent** — re-baking the cached overlay (vs.
only the on-disk copy) means a double-click of Store can't double-apply the shift,
and it works for a calibration composite that was never persisted. Tests:
`tests/test_v75x_plate_mosaic.py` +`TestWorkspaceManualShift` (3),
`TestMicroscopeUmPerPxResolution` (3), `TestManualAlignPage` (5, incl. idempotency +
prior+nudge bake + guard), `TestCalibrationDialogHandoff` (2) → file 69, full
`test_v75x*` = 635 green. **Needs real-HW verification on ME3B V1.**

### Manual delta sliders INSIDE the Calibrate… pop-out (in-dialog manual cal)

Follow-on: "within the calibration I need a set of delta sliders to do a manual
calibration." The Plate Location manual-align sliders couldn't be reached while the
**modal** Calibrate… dialog was open, so the dialog now hosts its OWN by-eye
alignment. `MosaicCalibrationDialog` gains optional render-context kwargs (`plate`,
`well_positions` [zero-ref µm], `safety_limits`, `zero_offset`, `needle_od_um`) and
**embeds a `JogWorkspaceView`** (`self._view`) — the same widget the Plate Location
tab uses — showing the well grid + the calibration mosaic overlay, registered
identically (wells zero-ref µm, mosaic extent absolute µm, `set_zero_offset` =
`zero_position`, the view's own 180° flip). New `_push_overlay(composite, extent)`
feeds the **unflipped** pixmap + absolute extent to the view (it flips internally)
and a flipped thumbnail to the small `_preview` QLabel; `_on_tile`/`_on_finished`
route through it. A **Manual align** group (Δx/Δy ±10 mm + opacity sliders,
**Store alignment (manual)** / **Reset**) is disabled until a mosaic exists
(`_set_align_enabled`, off during a build, on at finish). `_store_manual_align`
mirrors the page logic — `total = prior_stored + nudge` →
`MosaicAlignmentStore.set_shift_um(key, …, source="manual_align")` (seeds the next
full scan), bakes the nudge into `_result_ext` (so `result_overlay()` hands the page
an already-aligned composite), re-pushes, zeroes the nudge → **idempotent**. The
`n_meas==0` hint now points at the in-dialog sliders ("slide … then Store") rather
than the Plate Location tab. The page (`_ploc_quick_mosaic_calibrate`) passes the
render context (`_wells_in_zero_ref()`, `controller.safety_limits`,
`zero_position`, needle Ø). Both manual-align surfaces share one store key, so a
nudge in either composes (additive from whatever is displayed). Tests:
+`TestCalibrationDialogManualAlign` (3: sliders nudge the embedded view; Store
prior+nudge + bakes `_result_ext` + idempotent second-click; guard with no mosaic);
file 72, full `test_v75x*` = 638 green; offscreen render smoke confirms the embedded
view paints the grid + nudged overlay.

**Adversarial review (3-lens workflow, 7 agents): 1 confirmed MED, 3 false alarms.**
Confirmed: a later auto-registration (`mosaic_scan`/`quick_cal`) **silently
overwrote** a stored `manual_align` shift — `finalize_global_shift` REPLACES the seed
with the measured median, and no writer consulted the `source` field, so a deliberate
by-eye calibration vanished without warning on the next textured scan. Fix
(precedence policy): new `MosaicAlignmentStore.is_manual(key)`; both auto-write paths
(`_ploc_on_mosaic_finished`, dialog `_on_finished`) now **refuse to overwrite a
`manual_align` record** and surface a note ("kept your manual alignment"); a new
**Clear stored** button on BOTH manual-align panels (`_ploc_clear_stored_align` /
dialog `_clear_stored_align` → `store.clear(key)`) lets the operator explicitly
release the manual lock so auto can take over again. So `manual_align` is
authoritative until an explicit manual re-Store or Clear. Tests: +`is_manual`,
+page/dialog `clear`, +page/dialog `auto_finish_keeps_manual_align` → file 77, full
`test_v75x*` = 643 green. **Needs real-HW verification on ME3B V1.**

### Calibrate… pop-out: zoomable view + height fix + visible live translation

Three operator-reported issues with the in-dialog manual cal: **(1)** the dialog was
taller than the screen so the **Build button fell off the bottom**; **(2)** no way to
**enlarge / zoom** the mosaic to register by eye; **(3)** sliding Δx/Δy **didn't
visibly move** the mosaic (it was sub-pixel — the view auto-fit the whole ~114 mm
envelope into ~380 px, so a few-hundred-µm nudge was <1 px).

**Opt-in zoom/pan on `JogWorkspaceView`** (the shared widget; default OFF so Jog /
Plate Location are unchanged). New `set_zoom_enabled` / `set_zoom` / `zoom()` /
`reset_view`; `_zoom` (mult on the fit scale) + `_pan` (px) folded into `_scale()`
when enabled. `wheelEvent` zooms **about the cursor** (keeps the µm point under the
cursor fixed by adjusting `_pan`; snaps to fit at min zoom; clamp [1, 60]). Mouse
handling refactored: extracted `_handle_click_at`; with zoom on, left-drag **pans**
and a <4 px press is still a click (so click-to-teach/travel survive); with zoom off,
press behaves exactly as before. This fixes **#2** and, by magnifying, makes the
manual Δ produce a **visible** translation → **#3**.

**Adversarial review (3-lens workflow, 9 agents): 2 confirmed LOW, 4 false alarms.**
Both fixed: (a) a wheel tick *during* an active left-drag pan used the stale
press-time `_pan_anchor_val` → one-frame overlay jump; `wheelEvent` now **re-anchors**
the in-progress pan (`_pan_anchor_px`/`_pan_anchor_val`) to the just-updated `_pan`.
(b) the height cap was computed in `__init__` (where `self.screen()` may report the
primary, not the monitor a modal opens on); extracted to `_apply_height_cap` and
**re-applied in `showEvent`** for the actual display. The shared-widget click-to-teach/
travel path (zoom off) verified unchanged. Tests +`test_wheel_during_pan_reanchors`,
+`test_height_capped_to_finite_value` → file 85, full `test_v75x*` = 651 green.

### Calibrate… pop-out redesign: dedicated mosaic view (drop the plate view)

Operator follow-up: "I don't need the plate view — I need the mosaic. You forgot to
add the mosaic we're building to the pop-out. Settings on the left, the live mosaic
on the right; delta sliders under the settings; zoom + pan the mosaic; show the
outline of each image taken." The dialog was embedding `JogWorkspaceView` (plate +
wells) and pushing the composite as a tiny overlay — the actual stitched mosaic was
never prominently shown. Replaced with a purpose-built view + two-column layout.

**New `gui/widgets/mosaic_registration_view.py::MosaicRegistrationView`** — a
`QGraphicsView` (zoom/pan **for free**): a `QGraphicsPixmapItem` for the live
composite + one cosmetic-pen `QGraphicsRectItem` per captured tile (the **outline of
each image taken**). Wheel-zoom about the cursor (`AnchorUnderMouse`), `ScrollHandDrag`
pan, `fit`/`reset_view`. The composite is built by **stage position**
(`MosaicBuilder._blend_tile_to_composite`), so the global shift is a uniform offset —
therefore the delta moves the **image item** while the **tile outlines stay fixed**
(`set_shift_um` → `_img_item.setPos(Δ·scale)`), giving visible feedback. New
`MosaicBuilder.tile_rects_px()` returns each tile's footprint `(x,y,w,h)` in mosaic px
(same math as the blend).

**Dialog (`MosaicCalibrationDialog`) rebuilt**: a `QSplitter` — **left** = a scrollable
column with the collapsible settings (top) + the **Δx/Δy/opacity sliders** (bottom) +
status; **right** = the `MosaicRegistrationView` ("Mosaic (live)" + Reset view). Action
buttons pinned below; height cap retained. `_on_tile`/`_on_finished` now feed
`_show_mosaic` (composite + `tile_rects_px()` + scale). **Manual delta switched to an
ABSOLUTE model**: the sliders ARE the stored shift (µm), **seeded from the store on
finish** (`_seed_sliders_from_store`), live-translate the image, and `_store_manual_align`
writes them directly (idempotent — no prior+nudge accumulation, no snap-back); the page
handoff extent = `base canvas extent + Δ` (`_recompute_result_ext`). The
`manual_align`-is-authoritative precedence + Clear-stored + the auto-store-on-finish are
unchanged. `JogWorkspaceView` is no longer used by the dialog (its opt-in zoom/pan
remains a tested latent capability). The page stopped passing the now-unused plate/well
render context. Tests: +`TestMosaicBuilderTileRects`, +`TestMosaicRegistrationView` (2),
dialog tests updated to the absolute model (+`test_seed_sliders_from_store`) → file 89,
full `test_v75x*` = 655 green; offscreen smoke builds a 63-tile mosaic → 63 outlines,
sliders seed, Δ translates the image, view paints. **Needs real-HW verification on
ME3B V1.**

### Calibrate… pivots to SPACING calibration (find the FOV/step for the full mosaic)

Operator clarified the real goal: "the delta should move the images toward/away from
each other so I can align features in the OVERLAP regions; from that delta we adjust the
grid step so the overlap is correct — the best settings for the full mosaic." The
prior design translated the whole mosaic uniformly (a global shift), which — confirmed
against `_blend_tile_to_composite` — can NOT change inter-tile overlap (tiles are placed
by stage position; the global shift is a uniform offset). The actual unknown is the
**effective FOV (µm/px)**: if it's wrong, adjacent tiles' overlap features are mis-scaled
and double. So the dialog now calibrates **spacing → µm/px**, which
`MosaicAlignmentStore.um_per_px` already exists for and the full scan already prefers
(`learned > camera µm/px`).

`MosaicRegistrationView` reworked to render **individual semi-transparent tiles**
(`MosaicBuilder.tile_images_px()` → `(frame, left, top, w, h)` per record) + a peach
outline each, scaled about the tile-grid **centroid** by a per-axis factor `(kx, ky)`:
`set_spacing_factor` → `drawn = centroid + k·(nominal − centroid)`, so `k>1` spreads
tiles apart and `k<1` pulls them together (move images toward/away; overlaps blend via
opacity). Wheel-zoom/drag-pan retained. The dialog sliders became **X gap % / Y gap %**
(−50…+50, default 0) + opacity (default 60%) + a live readout ("Effective µm/px ≈ … ·
FOV … µm · step ≈ … @ N% overlap"). The geometry: spreading to align (k>1) means the
assumed FOV was too large → corrected **µm/px = assumed / mean(kx,ky)**. **Store
FOV/spacing** → `set_um_per_px(key, corrected, source="quick_fov")` AND writes the
corrected FOV into the FOV spin so "Apply to full mosaic" carries it; idempotent. Sliders
seed from any stored µm/px (`k = assumed/stored → pct = (k−1)·100`). The auto-registration
global-shift store + `manual_align` precedence + Clear-stored are retained (shift is a
separate, residual concern from µm/px). Tests updated to the spacing model
(+`tile_images_px`, spacing-factor-moves-tiles, corrected-µm/px+FOV, seed-from-µm/px) →
file 91, full `test_v75x*` = 657 green; offscreen smoke: 63 tiles + 63 outlines, +20% gap
spreads the tiles (250→300 px), readout live, Store corrects FOV 800→667 µm, view paints.
**Needs real-HW verification on ME3B V1.**

### Manual well mapping on the full mosaic + training-data export

The full-plate **Mosaic scan** auto-detect (`detect_wells` → match → warp) found no
wells on a real plate. Operator's fix, in 3 parts:

**(1+3) Save the labeled mosaic for detection R&D.** New `SupportClasses/
WellTrainingStore.py` writes each hand-mapped sample to
`config/hardware/well_training/<plate>_<YYYYmmdd_HHMMSS>/` — `mosaic.png` + a
`labels.json` (plate key, absolute-µm extent, mosaic_scale, camera µm/px, image size,
and **every well's centre in BOTH mosaic px and absolute stage µm**). The accumulated
corpus lets us develop/tune better auto-detection offline via the Claude interface
(NOT in the running software yet) against real, labeled plate mosaics.

**(2) Manual mapping with 3-corner auto-fill.** New `gui/dialogs/
mosaic_well_mapping_dialog.py::MosaicWellMappingDialog` shows the stitched mosaic
IMAGE (zoom = wheel, pan = middle-drag, in `_MappingView`). The operator clicks the
**3 corner wells** (chosen by nominal-position extremes — origin / +X-end / +Y-end —
via `corner_well_names`, robust to naming/custom plates); `solve_affine_3` fits the
nominal-grid→mosaic-px affine through those 3 points and `apply_affine` over every
`WellPlate.get_well_position` **auto-places all wells** as draggable green circles. The
operator drags any to refine. **Confirm** converts each well centre mosaic-px→absolute
stage µm (`extent[0:2] + px/scale` — the inverse of `MosaicBuilder` placement, same as
`_ploc_fit_from_mosaic_detections`) into `results()`, AND saves the `WellTrainingStore`
sample. Page wiring: a **Map wells…** button on Plate Location
(`_ploc_open_well_mapping`) loads the stored mosaic (`MosaicStore`), opens the dialog,
and on confirm routes `results()` through the existing `_ploc_feed_affine` warp fit +
reference markers (identical to auto-detect/manual-teach). The mosaic-finish status now
hints "Map wells…" when `<3` wells auto-detect.

Tests: `tests/test_v75x_plate_mosaic.py` +`TestWellTrainingStore` (2),
+`TestWellMappingMath` (affine recover / collinear-None / corner-distinct),
+`TestWellMappingDialog` (3 corners → 96/96 auto-fill → results in µm + sample saved),
+`TestWellMappingPageGuard` (no-mosaic guard) → file 98, full `test_v75x*` = 664 green;
offscreen smoke: 96-well plate, A1/A12/H1 corners → 96 wells placed, drag honored, view
paints, confirm saves. **Needs real-HW verification on ME3B V1.**

### Robust filled-disc well finder (validated on the real 24-well mosaic)

The full-plate auto-detect (HoughCircles) found no wells on the real
`config/hardware/mosaics/24.png` (24-well, 4×6). Analysis of that image: 24 bright
filled blue discs on black, leftmost column clipped at the edge; the blue channel
is the signal (luma ≈ 0 since it's pure blue) and the **top 24 connected components
are exactly the wells** (20 full ≈129k px + 4 clipped ≈108k px, then a cliff to
noise ≤33 px) — i.e. Hough (edge-based, finicky) was the wrong tool for
high-contrast FILLED circles. Replaced with a threshold→blob→circle-fit→grid
pipeline:

- **`WellDetector.detect_filled_wells`** — brightness = max over BGR (colour/
  illumination-agnostic) → Otsu → morphology → connected components → area gate
  (≥ frac of the largest blob, rejecting specks + mid-size smudges) → **chord-
  robust circle fit** (`_fit_circle_robust`: minEnclosingCircle seed + inlier-only
  Taubin refine) so a well cut by a straight chord — at the image border OR an
  interior mosaic margin — keeps its TRUE radius/centre and the circle hugs the
  visible arc (may extend off-image); accept gate = circle inlier-fraction.
  *(Clip-fix: the real 24.png left column is cut at an interior chord x≈37, not
  x=0 — a naive whole-contour Taubin undersized r 187 vs 201; the robust fit
  recovers ~201.)*
- **`WellDetector.fit_well_grid(centers, n_rows, n_cols)`** — fits the KNOWN plate
  lattice: percentile-seeded iterative snap-to-nearest-node + inlier-only LSQ
  affine; assigns each blob a (row,col), **rejects off-grid spurious blobs**,
  **fills missing wells** from the grid, and handles rotation/skew.

Wired in as the PRIMARY detector (Hough fallback) for the Mosaic scan auto-detect
(`_ploc_detect_and_fit_from_mosaic` + the worker — feeding the existing
match-to-predicted so well names/orientation stay correct) and as **Auto-detect**
in the Map wells dialog (runs on open; names via the plate grid; user drags to
refine; 3-corner click is the fallback).

Validated on the real mosaic: **24/24 wells**, grid residual median ≈ 5 px (≪ the
513 px pitch), clipped left column centres recovered (visually verified). Stress
tests (synthetic + degraded real): missing wells filled (C4, B5), spurious blobs
rejected, 4° rotation + Gaussian noise + 40% dimming all → 24/24. Tests:
`TestFilledWellDetector` (5: real-image, synthetic, colour-agnostic, grid fill +
outlier reject) + `TestWellMappingAutoDetect` (1). Dev tool +
`coding plans/well_detection_rnd/` (viz over the shipping detector). File 111, full
`test_v75x*` = 681 green; `*vision*` regression 56 green. **The corpus
(`well_training/`) + this detector are the basis for further detection tuning via
the Claude interface.**

**Dialog (`MosaicCalibrationDialog`)**: embeds the view with `set_zoom_enabled(True)`;
the settings form is now a **collapsible** `QGroupBox` (`_form_box` → `_form_inner`
hidden when unchecked) to reclaim height; the redundant thumbnail `_preview` is built
**only** when there's no embedded view (headless) — all uses guarded; a **Reset view**
button; and at the end of `_build_ui` the dialog **caps `setMaximumHeight` to the
screen's available height − margin** and resizes to a sensible default. Because the
view holds the layout stretch and the action buttons are the pinned last row, a capped
height shrinks the **view**, never the **Build/Close** buttons → **#1**. Offscreen
smoke confirms the cap (720 on an 800 px screen), collapse, zoom, and that a slider
sets the view's `mosaic_shift`. Tests: +`TestWorkspaceZoomPan` (6: default-off,
disabled ignores zoom/pan, zoom magnifies the px delta ≥5×, clamp, pan offsets the
map, Δ translates visibly when zoomed) → file 83, full `test_v75x*` = 649 green;
`test_v731_jog_navigation` (26) green (shared-widget click path intact). **Needs
real-HW verification on ME3B V1.**

### Three-way plate-view mode (ideal well / mosaic / overlay)

Replaced the binary **Show mosaic** checkbox (Jog page + Calibration → Plate
Location) with a 3-option **Plate view:** combo so any well-plate view can show
the **idealized well grid**, the **stitched mosaic** on its own, or the **mosaic
overlaid on the well grid**. `JogWorkspaceView` gains `_wells_visible` (default
True) + `set_wells_visible(bool)` (paint-only — `_paint_plate_and_wells` early-returns
when hidden; snapping/click logic untouched) and a unified
`set_plate_display_mode("well"|"mosaic"|"overlay")` that drives both
`_wells_visible` and `_mosaic_visible` (unknown → "well"). The two pages each own a
`QComboBox` (data = "well"/"mosaic"/"overlay") whose mosaic-dependent items are
disabled (and the selection reverts to "Ideal well") when no mosaic exists for the
active plate (`_set_mosaic_modes_enabled`). Calibration's auto-show-after-build sites
+ the manual-align entry now call `_ploc_show_mosaic_mode()` (switches to "overlay" so
the operator compares the stitched mosaic against the ideal grid; keeps an already-
active mosaic mode). The pick-and-place `WorkspaceTargetView` shows wells but loads no
mosaic, so it keeps the default well-only view (no toggle added — would be inert
without mosaic-loading wiring). Tests: updated `test_v75x_plate_mosaic.py`
(`TestMosaicUI` view-combo existence + well/mosaic/overlay visibility transitions;
`TestJogOverlayLoad` checks the combo's mosaic items enable on load) — 91 green.
**Needs real-HW verification on ME3B V1.**

### Zoom + pan on every well-plate overview (+/-/hand, wheel, Shift-drag)

Promoted `JogWorkspaceView`'s previously dormant opt-in zoom/pan to an
**always-on** feature so the operator can magnify small features (e.g. spots seen
in the mosaic) on **any** page that hosts the view (Jog, Calibration → Plate
Location, and the pick-and-place `WorkspaceTargetView`). Changes (all in
`gui/widgets/jog_workspace_view.py`): `_zoom_enabled` now defaults **True**; three
floating overlay controls (`QToolButton` children, top-right, translucent rounded)
— **+** / **−** (`_zoom_about_center`, zoom about the canvas centre) and a checkable
**hand** tool (`_pan_tool_active`) — built in `_build_zoom_controls`, positioned in
`resizeEvent`/`_layout_zoom_controls`, hidden when zoom is disabled. **Mouse wheel**
zooms about the cursor (existing `wheelEvent`, now active by default). **Pan trigger
reworked** so click-to-travel is preserved: a left-drag pans **only** when the hand
tool is on **or Shift is held** (`mousePressEvent` checks
`Qt.KeyboardModifier.ShiftModifier`); a plain left-press still travels/snaps/teaches
(handled on press as before). `mouseReleaseEvent` just ends the pan; cursor follows
the mode (open/closed hand vs cross via `_update_pan_cursor`). New `hand` glyph added
to `gui/widgets/icons.py`. `set_zoom_enabled(False)` still locks a view to
fit-to-envelope (clears the hand tool, hides the controls). No page wiring needed —
the default-on widget covers every host. (The QGraphicsView-based plate diagrams
[`well_plate_view`, `well_preview`] and the separate Quick-Print
`print_trajectory_monitor` keep their own view behaviour; not in scope here.)
Tests: `test_v75x_plate_mosaic.py` — flipped the two default-state assertions to
on-by-default + new `TestWorkspaceZoomControls` (8: controls exist/visible, hidden
when disabled, +/- zoom, zoom-out-at-min resets pan, hand-tool toggle, Shift-drag
pans while plain click travels, hand-tool drag pans without Shift) → 105 green;
`test_v731_jog_navigation` (26) + rosette/trajectory suites green. **Needs real-HW
verification on ME3B V1.**

### Scan-once → 3-well quick re-registration (plate template)

A full mosaic scan is slow (1716 tiles for the 24-well). Once it's done + the wells
are mapped, the well centres are an accurate **as-built template** of THIS plate as
seen by THIS camera + objective. We persist it (`PlateTemplateStore`, keyed
`<plate>|<camera>|<objective>` because the µm/px geometry is only consistent for the
same optics) so the operator scans ONCE; later they re-measure just **3 reference
wells** and the whole plate re-registers.

- **Save:** the Map-wells dialog confirm (and any mosaic auto-fit) now also calls
  `_ploc_save_plate_template(results)` — stores the well map + 3 corner ref wells +
  µm/px.
- **Algorithm:** `PlateWarpCalibrator.register_from_template(template_wells,
  measured_wells)` fits template→measured over the shared (3) wells (≥3 → affine,
  exact through 3) and applies it to ALL template wells → registered positions. A
  known rotate+scale+translate is recovered from 3 wells to **0 µm** over all 24
  (test).
- **Quick re-register workflow** (`_ploc_quick_reregister`, gated on a matching
  template + XY connected + Safe-Z + microscope µm/px): drives to each of the 3 ref
  wells (`_ploc_safe_goto` — retract + travel), `capture_fresh_frame` →
  `detect_filled_wells` → nearest-to-centre → `pixel_to_stage_offset` → measured
  stage µm; `register_from_template` → `_calibrated_positions`; `_warp_is_plausible`
  rejects a mis-detection (same plate+camera ⇒ scale≈1). The button auto-enables
  only when a template exists for the current plate+camera+objective.

Tests: `tests/test_v75x_plate_template_reregister.py` (10). **The drive/detect path
needs real-HW verification on ME3B V1.**

### Per-well rosette mosaic (map a rosette well's sub-wells)

When a well has a rosette, scan THAT one well at high resolution and map its sub-wells
(user choice: "Scan well… → map sub-wells", sub-well layout "from the plate design").
Reuses the full mosaic-scan machinery via a bounds override:

- New **Scan well…** button → `_ploc_scan_rosette_well`: lists rosette parent wells
  (`_ploc_rosette_parent_wells` — wells with flattened `is_subwell`/`parent_well`
  sub-wells), the operator picks one, and the scan runs over JUST that well's bounds
  (`_ploc_subwell_scan_bounds` = bbox of the parent's sub-well centres + radius +
  margin). The scan sets `_ploc_scan_bounds_override` + `_ploc_scan_subwell_parent`
  and calls the standard `_ploc_start_mosaic_scan` — which now honours the bounds
  override (clipped to the envelope) and keeps all the retract-safe raster + worker
  machinery. Because `target_px` is fixed, scanning one ~16 mm well gives ~7× the
  whole-plate resolution → the small sub-well discs resolve.
- On finish, `_ploc_on_mosaic_finished` routes a sub-well scan to
  `_ploc_open_subwell_mapping(parent, …)` instead of the whole-plate detect: it opens
  the Map-wells dialog over a `_SubPlate` adapter (a minimal WellPlate-like view of
  just that parent's sub-wells; `rows=cols=0` so the dialog uses the 3-corner affine
  method — rosette patterns aren't a regular grid; the sub-well nominal positions come
  from the plate design). Confirm merges the measured sub-well centres into
  `_calibrated_positions` (sub-wells are flattened regular wells). Override/parent
  state is reset on finish/fail/cancel.

Tests: `tests/test_v75x_plate_template_reregister.py::TestRosetteSubwell` (5:
`_SubPlate` adapter, parent detection, no-rosette guard, sub-well scan bounds, Scan
well guard). Full `test_v75x*` = 697 green. **The scan/map path needs real-HW
verification on ME3B V1.**

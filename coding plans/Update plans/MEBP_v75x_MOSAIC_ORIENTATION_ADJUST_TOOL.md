# MEBP v7.5.x — Interactive mosaic orientation fix → save to camera

## Objective

Operator (2026-07-24): "For any mosaic that we build, I need the option to
reconfigure the image. Currently if the camera has a rotation or mirror, the
mosaic looks all messed up. I need buttons that flip / rotate the individual
images to match the correct view. These settings should then be applied to that
camera setup — so if we find out the camera is mirrored or the rotation is
wrong, we can continuously change that and set the actuals."

Confirmed via AskUserQuestion: correction applies to **feed + mosaic**; controls
are **flip H/V + 90° rotate + fine angle**; re-render is **instant re-blend from
retained tiles**.

## Design — orientation applied per consumer on RAW frames

The two prior same-day cuts applied the mirror at the frame *source*
(`CameraWidget`) and rotation downstream. That prevented re-blending a mosaic
against a *changing* orientation. This reworks it into a clean model: frames stay
**raw**, and the camera's `(mirrored, rotation_deg)` (persisted per identity in
`CameraCalibrationStore`, live on `CameraManager`) is applied in three
independent places that all read the same calibration:

1. **Display** — `CameraFeedView.set_view_orientation(mirrored, rotation_deg)`
   transforms the SHOWN pixmap (flip + rotate, matching `_orient_tile`) so the
   operator sees an upright, un-mirrored feed, and **inverts clicks back to raw
   frame coords** (via `QImage.trueMatrix` + `.inverted()`), so
   `pixel_to_stage_offset` is unaffected. Overlays (crosshair / reference markers
   / vector) are mapped through the same transform. Opt-in per view; a no-op at
   `(False, 0°)`; suppressed in `_edge_pick_mode` (needle row=Z geometry).
2. **Click→stage** — `CameraManager.pixel_to_stage_offset` applies mirror
   (`dx→−dx`) then `R(θ)` on the raw-frame click (mirror term restored).
3. **Mosaic** — `MosaicBuilder._orient_tile` applies mirror + `R(θ)` per tile;
   the builder **retains the small canvas-res tiles** and re-blends on demand.

### Key changes

**`gui/widgets/camera_widget.py`** — reverted the source flip: `_mirrored` /
`set_mirrored` / `mirrored` are a stored FLAG only; `_grab_frame` /
`capture_fresh_frame` return raw frames (removed `_maybe_mirror_frame`).

**`gui/widgets/camera_manager.py`** — `pixel_to_stage_offset` re-applies the
mirror flip; `get_rotation_deg` / `get_mirrored` / `_widget` are getattr-safe;
new `view_orientation(cam_idx) → (mirrored, rotation_deg)`.

**`gui/widgets/camera_feed_view.py`** — `set_view_orientation` /
`set_edge_pick_mode` / `_orient_qimage` (returns `(displayed_qimage,
true_xform)`); `_render_frame` orients first + stores `_view_true_xform` +
`_displayed_image_size`; `_widget_to_image` inverts the transform → raw coords;
`_draw_reference_markers` / `_draw_overlay_vector` map through the transform.

**`SupportClasses/MosaicBuilder.py`** — re-added `frame_mirrored` +
`retain_for_reorient` ctor params; `_orient_tile` mirror + rotation; the pre-orient
canvas-res tile is stored in `_reorient_tiles`; the feather-blend core is factored
into `_accumulate_oriented_tile`; new `set_frame_orientation` /
`reblend_reoriented` (zero the accumulators + re-blend every retained tile) /
`has_reorient_tiles` / `free_reorient`; `free_accumulators` is a no-op while
`retain_for_reorient` (the tool needs the buffers).

**`gui/pages/calibration.py`** — `_ploc_microscope_frame_orientation()` returns
`(rotation, mirrored)` (fed into the full-plate + `_well_scan_builder` builders
with `retain_for_reorient=True`); `_ploc_reorient_builder` kept alive past the
scan-finish null; live feed corrected via `_ploc_apply_feed_orientation` (in
`_ploc_ensure_live_camera` + `_refresh_ploc_view`); a **"Fix orientation"** card
under the mosaic buttons — `⇄ Flip H` / `⇅ Flip V` / `⟲ 90°` / `⟳ 90°`, a
Rotation-° spin (fine, −180..180, 0.5° step), Reset, and **Apply to camera
setup**. Each control updates the working `(mirror, rotation)`
(`_ploc_orient_set`, normalised; flip-H = `rot→−rot`+toggle, flip-V =
`rot→180−rot`+toggle, rotate = `±90`), re-blends the mosaic
(`set_frame_orientation` + `reblend_reoriented` → overlay + preview) AND pushes
live to the feed + manager. Apply persists to `CameraCalibrationStore`
(`set_rotation` + `set_mirrored`) + emits `calibration_data_changed`. Enabled
after a full-plate scan completes.

## Files Modified
- `gui/widgets/camera_widget.py`, `gui/widgets/camera_manager.py`,
  `gui/widgets/camera_feed_view.py`, `SupportClasses/MosaicBuilder.py`,
  `gui/pages/calibration.py`, **`gui/dialogs/mosaic_calibration_dialog.py`**
  (the flip/rotate tool's home — see addendum).
- Tests: `tests/test_v75x_mosaic_orientation_adjust.py` (new — builder
  retain/reblend + feed-view transform/click-inverse); updated
  `test_v75x_camera_rotation_cal_and_monitor.py` (mirror back in mapping; widget
  flag-only; `view_orientation`) + `test_v75x_mapping_camera_orient_and_rosette_z.py`
  (builder mirror back).

## Testing
- New suite (12) + updated mirror/orient suites green (58 across the three);
  camera store/liveview/correction/hw-controls/**picker-scaling** (getattr-safe
  fix clears the 5 pre-existing `_rotation_deg` errors) + mosaic/reanchor/
  fluorescence/plate-location-page (178) green.
- **Needs real-HW verification on ME3B V1:** scan a plate with a mirrored/rotated
  microscope camera → mosaic looks wrong → `⇄`/`⟲`/`⟳`/fine until it reads
  correctly and the live feed is upright/un-mirrored → **Apply** → re-scan →
  correct with no adjustment; live-view clicks + mapped wells land right;
  restart → restored. Confirm needle edge-pick + rotation calibration still work
  (raw frames). If a control moves the mosaic the WRONG way, flip the one-line
  sign in `_orient_tile` / `_orient_qimage` (same caveat as the rotation cal).

## Notes / risks
- `CameraFeedView` transform is opt-in per view + no-op by default + skipped in
  edge-pick, so needle workflows are untouched.
- Full-plate scans keep the small canvas-res tiles + the float64 accumulators
  (~190 MB) alive AFTER the scan for the tool (freed on the next scan /
  `free_reorient`) — a deliberate memory trade for instant re-blend.
- Fine (non-right-angle) rotation shows black corners on the feed + clips tile
  corners in the mosaic (overlap covers it) — documented.
- V1 wires the tool to the FULL-PLATE mosaic; single-well/rosette reblend is a
  documented follow-up (the reorient builder is set for both, but the enable +
  overlay push are wired for the full-plate finish).

## Status
- [x] Raw frames + per-consumer orientation (widget/manager/feed-view)
- [x] MosaicBuilder mirror + retain + reblend
- [x] Calibration Fix-orientation card + feed correction + apply
- [x] Tests + regression
- [ ] Operator real-HW verification

## Addendum (2026-07-24) — relocated to the single-well Calibrate-mosaic dialog

Operator: *"No, this should be applied to a calibrate mosaic button, where a
single well is scanned, then I am able to do this to each image, and also the
manual slide image gaps function is embedded as well. Since it's just a single
well, it's fast to adopt."*

The `MosaicCalibrationDialog` (opened by the Plate Location **"Calibrate mosaic
scan"** button) already scans a small grid AND has the **manual slide/image-gaps
registration** (`MosaicRegistrationView` + X/Y gap sliders + "Store FOV/spacing").
The orientation tool moved THERE (per-image, fast):

- **Removed** the "Fix orientation" card + `_ploc_orient_*` handlers +
  `_ploc_reorient_builder` + `retain_for_reorient` from Plate Location. The
  full-plate + well-scan builders keep `frame_rotation_deg`/`frame_mirrored`
  (the plate mosaic still renders oriented) and the live feed correction
  (`_ploc_apply_feed_orientation`) stays.
- **`MosaicBuilder.tile_images_px`** now applies `_orient_tile` per tile
  (no-op at default) — so the dialog's per-image registration view reflects the
  orientation; re-called after `set_frame_orientation` to re-render.
  (`retain_for_reorient`/`reblend_reoriented` remain as a general, unused-in-prod
  capability + tests.)
- **`gui/dialogs/mosaic_calibration_dialog.py`**: seeds `(mirror, rotation)`
  from `CameraManager.view_orientation(cam_idx)`; builds the calibration mosaic
  with that orientation; keeps the builder alive after the build (retained
  frames); a **"Camera orientation (flip / rotate images)"** group — `⇄ Flip H`
  / `⇅ Flip V` / `⟲ 90°` / `⟳ 90°` / Rotation-° spin / **Apply to camera
  setup** — each control re-orients the shown tiles (`set_frame_orientation` +
  `_show_tiles`) and pushes the camera live (feed + clicks + future scans);
  Apply persists to `CameraCalibrationStore`. Enabled after a build, disabled
  during/after stop/fail.
- Tests: `test_v75x_mosaic_orientation_adjust.py` grew `TestTileImagesOrientation`
  (tile_images_px orients) + `TestCalibrationDialogOrientation` (seed from camera,
  flip/rotate compose, Apply persists — modal mocked). 14 in the suite; mirror +
  camera-store/liveview + mosaic/plate-location/reanchor suites green.

## Addendum 2 (2026-07-24) — Hardware Setup "Mirrored view" checkbox now flips the preview

Operator: *"when I press mirror camera it doesn't get mirrored in the view."* The
Hardware Setup → Cameras slot checkbox (`_apply_slot_mirror` / `_apply_slot_rotation`)
set the flag + persisted but never told the slot's live PREVIEW to flip — after
the rework the display correction lives in `CameraFeedView.set_view_orientation`,
which wasn't being called. Fix: new `HardwareSetupPage._push_slot_view_orientation(cam_idx)`
reads `CameraManager.view_orientation(cam_idx)` and calls the slot preview's
`set_view_orientation(mir, rot)`; invoked from the shared
`_refresh_slot_rotation_displays` loop (so it fires on toggle, on rotation apply,
on calibration restore, and on camera start — idempotent). Test:
`test_v75x_camera_rotation_cal_and_monitor.py::TestSlotMirrorUI.test_mirror_flips_the_live_preview`
+ real-`CameraFeedView` smoke (checkbox → `_view_mirror` True/False). Suite 38 green.

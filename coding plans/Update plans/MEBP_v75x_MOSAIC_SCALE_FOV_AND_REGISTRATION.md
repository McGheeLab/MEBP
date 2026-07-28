# MEBP v7.5.x — Mosaic calibration: true-resolution FOV, stage-motion scale/FOV auto-cal, better registration, camera-aspect video

## Objective

Operator (2026-07-24): *"the mosaic calibration is not working as expected. I try
to overlay the outputs, and it tries to autocalculate the camera FOV but it
assumes a camera resolution — it should pull from the camera resolution as set in
the camera hardware. Also the camera setup should do an auto calibration of the
pixel to micron conversion based on the stage motion. Also we should be able to
calculate the extents of the camera in microns by looking at the features near the
edge and moving to those features to do an auto registration and alignment — this
should all check out with the known pixels incoming from the camera and the micron
to pixel space, and flow into the mosaic. For the camera calibration we should be
able to mirror the camera output in that view as well. We need a better image
registration algorithm too — the current feature detection doesn't work great when
moving the stage. Also the video output everywhere should match up with the
camera's output, not be a fixed aspect ratio."*

Decisions (AskUserQuestion): **stage-motion edge-feature auto-cal** for µm/px +
FOV (drives items 1-3); **full pairwise + global-optimize** registration (item 5).

## Investigation (root causes)

Read-only agents mapped the pipeline before any edits:

1. **"FOV assumes a resolution."** The mosaic paths DO grab a live frame
   (`frame.shape`) for pixel dims, so the *pixel* count is real. The bug is in
   **µm/px**: the mosaic FOV = `frame_w × _ploc_microscope_um_per_px(fw, um0)` and
   that helper reads the OBJECTIVE store's `measured_um_per_px` + its stored
   `resolution` (`cal_w`), returning `meas × cal_w/frame_w` → FOV = `meas × cal_w`.
   So the FOV is pinned to the resolution stamped on the objective calibration.
   `ObjectiveCalibrationCard._on_calibrate_clicked` stamped
   `config.camera_config.active_resolution` — which can be **stale/assumed** →
   wrong `cal_w` → wrong FOV. (Fix: stamp the camera's ACTUAL captured
   resolution.)
2. **Registration.** Tiles are placed **open-loop by stage position**; the only
   "registration" during a scan is a single **global median** phase-correlation
   shift applied to the display *extent* (not pixels), with a weak confidence
   metric and no illumination/rotation handling. A richer pairwise + global-LSQ
   batch path exists (`build_mosaic`) but is gated off (>200-tile skip +
   `retain_frames=False` frees the frames it needs).
3. **Aspect ratio.** Content is NOT distorted (every feed uses `KeepAspectRatio`
   with the true `frame.shape`); the operator sees **letterbox bars** because the
   widget box shape doesn't track the camera W:H. No `heightForWidth` existed.

## Design / changes

### Item 6 — camera video matches the camera aspect
`CameraFeedView` gains `hasHeightForWidth()`/`heightForWidth(w)` driven by
`_last_image_size` (via `_display_wh`, which SWAPS W/H when the view is rotated
~90°), a `QSizePolicy` with `setHeightForWidth(True)`, and `_maybe_update_geometry`
(fires only when the displayed aspect changes — on new frame / rotation / edge-pick
toggle). Box layouts + splitters now allocate a camera-shaped cell the
`KeepAspectRatio` pixmap fills edge-to-edge; layouts that ignore heightForWidth
(QGridLayout) keep the undistorted fit — **no regression, never distorted**. One
base class → every live feed (needle / objective / picker / calibration /
workflow) inherits it.

### Item 4 — mirror the camera output in the calibration view
`PixelCalibrationDialog` seeds its feed with the camera's saved `(mirror,
rotation)` via `set_view_orientation` (shows the corrected upright view) and adds a
**"Camera shows a mirrored image"** checkbox → flips the DISPLAY live, pushes
`set_mirrored` to the manager, and persists per identity (`CameraCalibrationStore`).
µm/px is measured on RAW frames, so the mirror is display-only and cannot corrupt
the result. The new Scale+FOV dialog carries the same control.

### Items 1-3 — stage-motion scale + FOV auto-calibration
- **`VisionDetector.select_trackable_patch`** — picks the most textured square
  patch in the central region (survives a move); paired with the existing
  `find_template` (TM_CCOEFF_NORMED) it tracks a feature across a LARGE stage
  baseline (where whole-frame phase correlation loses lock).
- **NEW `gui/dialogs/scale_fov_calibration_dialog.py::ScaleFovCalibrationDialog`**
  — live feed (+ mirror control); auto-sizes the baseline (~40 % of the frame from
  the current µm/px so the feature stays in view); a QTimer state machine moves +X
  then +Y, tracks the feature each time, and derives `µm/px = move/|disp|` per axis
  (they agree for square pixels), `FOV = frame_px × µm/px` (the EXTENT), and the
  in-plane rotation (`plus_column_direction_deg`, same convention as the needle
  µm/px cal). Everything is anchored to the **captured `frame.shape`** — the real
  pixels — and exposed as `result_um_per_px / _rotation_deg / _resolution /
  _fov_um`. A >15 % X-vs-Y disagreement warns (feature left the frame on one axis).
- **`ObjectiveCalibrationCard`** gains an **"Auto-calibrate scale + FOV…"** button
  (`_on_autocal_scale_fov_clicked`) that launches it and persists per-objective
  (`ObjectiveCalibrationStore.set_calibration(cam_key, obj, µm/px, resolution,
  rotation_deg)`) + live to the manager + rotation per identity — the SAME sinks
  the mosaic reads, so it flows straight into the mosaic FOV.
- **Item 1 fix:** new `_true_capture_resolution(cam_idx)` reads the LIVE frame
  shape (authoritative), then HW settings, then config — used by BOTH the auto-cal
  and the existing `_on_calibrate_clicked`, so µm/px is always stamped with the
  REAL captured resolution and the mosaic FOV stops "assuming" one.

### Item 5 — full pairwise + global-optimize registration
- **`MosaicBuilder._register_overlap_cv2`** — the stronger pairwise estimator:
  CLAHE-normalise both overlap crops (kills illumination-gradient bias), Hanning
  window, `cv2.phaseCorrelate` with its **real cross-power peak as confidence**
  (vs skimage's weak `1−|error|`). Returns the sub-pixel correction to ADD to
  tile B to align onto A (sign locked by test).
- **`MosaicBuilder.optimize_registration()`** — registers EVERY overlapping tile
  pair over the retained canvas-res tiles, feeds a weighted global least-squares
  solve (`_global_optimize_positions`, tile 0 anchored) for globally-consistent
  positions, then re-blends at those positions (`_reblend_at_positions`). Each
  correction is **bounded** to the trusted stage placement (`_max_shift_px`) so a
  bad match can't fling a tile; a degenerate solve (too few confident overlaps) is
  a **no-op** that keeps the open-loop composite. Operates on the SMALL canvas-res
  tiles (`retain_for_reorient`), so memory stays bounded and no raw frames / re-scan
  are needed. It scales fine (only true neighbours get correlated), so the batch
  path's >200-tile gate is irrelevant here.
- **Wiring (`calibration.py::_MosaicScanWorker`):** the full-plate scan builder now
  passes `retain_for_reorient=True`; after the tile loop the worker runs
  `optimize_registration()` (best-effort) → `finalize_global_shift` (extent offset,
  complementary) → detection on the improved composite → then `free_reorient()`
  (frees the tiles AND the ~190 MB accumulators, keeping the finished display
  cache) instead of `free_accumulators`.

## Files Modified
- `gui/widgets/camera_feed_view.py` — heightForWidth aspect tracking (item 6).
- `gui/dialogs/pixel_calibration_dialog.py` — mirror control + view orientation
  seed (item 4).
- **NEW `gui/dialogs/scale_fov_calibration_dialog.py`** — stage-motion scale+FOV
  auto-cal (items 1-3).
- `SupportClasses/VisionDetector.py` — `select_trackable_patch` (tracking helper).
- `gui/pages/hardware/objective_calibration_card.py` — "Auto-calibrate scale +
  FOV…" button + handler, `_true_capture_resolution`, item-1 resolution fix.
- `SupportClasses/MosaicBuilder.py` — `_register_overlap_cv2`,
  `optimize_registration`, `_reblend_at_positions` (item 5).
- `gui/pages/calibration.py` — full-plate scan retains canvas-res tiles + runs
  `optimize_registration` + frees via `free_reorient` (item 5).
- Tests: `tests/test_v75x_camera_scale_fov_and_registration.py` (11).

## Testing
- New suite (11): patch select/track round-trip; `_register_overlap_cv2` sign +
  confidence; **`optimize_registration` corrects a +7 px injected tile error** (the
  decisive end-to-end + sign-lock test); `_reblend_at_positions`;
  `CameraFeedView.heightForWidth` tracks the camera aspect (+90° swap); Scale+FOV
  `_compute` derives µm/px + FOV + resolution from measured displacements.
- Regression green: mosaic-orientation-adjust / v731-mosaic / mosaic-memory /
  mosaic-orientation-remap (58); objective-calibration / camera-rotation-cal /
  mapping-camera-orient-rosette-z (77); camera-store / image-correction (49);
  camera-cal-liveview / hardware-controls (32).
- **Needs real-HW verification on ME3B V1:**
  1. Hardware Setup → Cameras → an objective → **Auto-calibrate scale + FOV…** →
     aim at a textured region → Measure → plausible µm/px, FOV extent, resolution
     (= the camera's real capture size), rotation; X≈Y µm/px. Accept.
  2. Scan a plate → the mosaic tiles line up better than before (global optimize);
     if a control/registration goes the WRONG way, flip the one-line sign in
     `_register_overlap_cv2` (test-locked to +7 px correction).
  3. Every live video fills its box with the camera's aspect (no dark bars).
  4. In the µm/px + Scale/FOV dialogs, "Camera shows a mirrored image" flips the
     displayed feed and persists.
  5. Mosaic FOV/overlap now sizes from the true captured resolution (no more
     "assumes a resolution").

## Status
- [x] Item 6 — CameraFeedView heightForWidth (aspect).
- [x] Item 4 — mirror control in the calibration view.
- [x] Items 1-3 — VisionDetector patch tracker + Scale/FOV auto-cal dialog + card
  button + true-resolution stamping.
- [x] Item 5 — MosaicBuilder pairwise + global-optimize + calibration wiring.
- [x] Tests + regression.
- [ ] Operator real-HW verification on ME3B V1.

## Addendum (2026-07-24) — operator follow-ups after first real use

Three follow-up reports; subagents were spend-limit-blocked so root-caused +
fixed directly.

**(A) "The calibration did not correctly assign the values to the mosaic — it was
still wrong."** ROOT CAUSE: the mosaic µm/px precedence in
`calibration.py` (`eff_um_per_px`) was `learned (MosaicAlignmentStore) → else
objective/camera`, so a **stale "Store FOV/spacing" learned value** (written by
the mosaic-calibration dialog's `_store_manual_align`) **shadowed** a fresh
objective/scale calibration — and it wasn't even rescaled to the scan resolution.
Fixes: (1) a fresh calibration now CLEARS that learned value —
`ObjectiveCalibrationCard._clear_stale_mosaic_fov(cam_idx, objective)` +
`_mosaic_align_key` (matches `_ploc_camera_objective_key`
`{identity}|{objective}`), called from both cal handlers; (2) the mosaic now
rescales BOTH the learned value (by its stored resolution) AND the objective
fallback (via `CameraManager.effective_um_per_px(cam_idx, fw)`, not the raw
value) to the scan's frame width, so a fresh calibration drives the FOV
correctly.

**(B) "The calibration should do a quick mosaic to check itself."** Added a
**"🔍 Verify with a quick mosaic"** button to `ScaleFovCalibrationDialog`
(enabled after a measurement): builds a 3×3 mosaic at the current position with
the just-measured µm/px (reusing `_MosaicScanWorker` + `MosaicBuilder`, running
the same global-optimize registration), applying the camera's mirror/rotation,
and displays the composite in a preview pane so the operator sees whether tiles
register. **Safe by construction** — it only moves XY at the current height
(retract Z = the current Z, `target_z=None` → it NEVER descends), so it can't
crash the needle regardless of plate.

**(C) "The live view is not consistent with the view used to calibrate the
objective."** New opt-in `CameraFeedView(auto_orient=True)`: the view re-reads the
camera's saved mirror/rotation from `CameraManager.view_orientation` each frame
and applies it (display-only + click-inverting; suppressed in edge-pick), so the
calibration view stays locked to the same orientation the live/mosaic views use.
Enabled in both `PixelCalibrationDialog` and `ScaleFovCalibrationDialog`. (NOTE:
`CameraManager` has no role getter, so auto-orient is scoped to the calibration
dialogs rather than applied blanket — a blanket default would wrongly rotate the
~45° needle feeds. If a SPECIFIC operation live view — a workflow microscope feed
— is still inconsistent, enable `auto_orient=True` on that `CameraFeedView`.)

Tests (added to `tests/test_v75x_camera_scale_fov_and_registration.py`, now 19):
auto-orient sync + edge-pick suppression + off-by-default; align-key convention +
fresh-cal-clears-learned + learned-resolution-rescale; verify button gating +
`_show_mosaic` render. Regression green: mosaic-orientation-adjust /
objective-calibration / camera-rotation-cal / plate-mosaic worker+registration
+ calibration-dialog-handoff (94); mapping-camera-orient / camera-cal-liveview /
reanchor / rosette-tab (90). Also fixed a PRE-EXISTING test gap surfaced en route:
`test_v75x_plate_mosaic._FakeCtrl.safe_travel_to` didn't accept the
`apply_insert_floor` kwarg the (earlier, uncommitted) rosette-Z work added.

## Addendum 2 (2026-07-24) — second operator follow-up batch

Three more reports (subagents/workflow still spend-limit-blocked → direct).

**(1) "The microscope camera setup block was where we pulled the camera
resolution from, but if the camera resolution changed it did not update that
block."** ROOT CAUSE: the setup-block combo drives the device
(`_maybe_apply_resolution_to_device`), but the camera **settings gear**
(`camera_settings_dialog._on_resolution_changed` → `set_capture_resolution`)
changed the DEVICE resolution WITHOUT updating the block's `active_resolution`.
Fix — a resolution-change signal chain: `CameraSettingsDialog.resolution_applied
= Signal(cam_idx, w, h)` (emitted after the device adopts a resolution) →
`CameraFeedView.resolution_changed` (re-emitted from the gear dialog) →
`HardwareSetupPage._on_slot_resolution_changed` (updates
`camera_config.active_resolution` + syncs the block's resolution combo when it's
the microscope slot). So the block now tracks the true device resolution and
downstream µm/px stamping / mosaic FOV use the real pixel count.

**(2) "The x,y translation is not mapped correctly to the camera in the mosaic —
the stage direction and where the image was placed was flipped."** This is the
camera-axis-vs-stage ORIENTATION being wrong (the tile CONTENT is mirrored
relative to its stage placement, so adjacent tiles don't line up / the mosaic
reads flipped). The fix is operator control (item 3) — the flip-to-X/Y buttons
correct it, and once applied it persists per camera identity + propagates, so it
is a one-time correction per machine mount.

**(3) "In the autocalibrate + scale quick mosaic check I want a full-featured
mosaic correction via manual sliders just like the full mosaic calibration, plus
flip-camera-to-X/Y-axis buttons; anything changed here is ground truth and
propagates everywhere."** The Scale/FOV dialog's **"🔍 Verify & correct
(mosaic)…"** button now opens the full `MosaicCalibrationDialog` (reused, not
duplicated) seeded with the measured µm/px + the camera orientation + current
position + a no-descent safe Z + the objective keys. That dialog already has the
X/Y spacing sliders + Store-FOV/spacing + rotate + Apply; its **flip buttons were
relabelled "⇄ Flip X axis" / "⇅ Flip Y axis"** (they mirror the X/Y axis). GROUND
TRUTH propagation: new `MosaicCalibrationDialog._propagate_um_per_px` (called from
"Store FOV/spacing") pushes the corrected µm/px to the **live manager**
(click mapping) AND, when the objective is known (`cam_key`/`objective` now
threaded through), the **objective store** (mosaic fallback + restore) — so a
correction takes effect everywhere, not just as the mosaic learned value; the
orientation "Apply to camera setup" already pushed to the manager +
`CameraCalibrationStore`. After the correction dialog closes, the Scale/FOV dialog
re-syncs its result from the manager so Accept can't clobber it. ORDERING FIX: the
card now clears the stale learned FOV BEFORE opening the dialog (not after), so a
fresh correction made in Verify survives.

Tests (suite now 26): resolution signals exist + `_on_slot_resolution_changed`
updates active_resolution/combo (and ignores non-microscope slots); flip buttons
labelled X/Y; `_propagate_um_per_px` writes manager + objective store (rotation
preserved) and manager-only without an objective; Scale/FOV threads the objective
keys. Regression green: scale-fov+registration (26); mosaic-orientation-adjust /
camera-cal-liveview / camera-hardware-controls / plate-mosaic
dialog+handoff+manual-align (78); camera-rotation / objective / store / correction
(112); mosaic worker/registration/dialog + mapping + reanchor (52).

## Addendum 3 (2026-07-24) — third operator follow-up batch

Three more reports (subagents/workflow still spend-limit-blocked → direct).

**(1) "The full-plate mosaic scan didn't apply the rotations and scales calibrated
from the objective."** The scan already read `_ploc_microscope_frame_orientation()`
(rotation/mirror) + `eff_um_per_px` (scale) and passed them to `MosaicBuilder`, so
the wiring existed — the failure was a stale/unread value. Hardening:
`_ploc_microscope_frame_orientation` now falls back to the persisted per-identity
`CameraCalibrationStore` value when the live manager carries no orientation (e.g.
it wasn't re-synced after calibrating on another page); and the scan logs a
diagnostic line — `Mosaic scan applying: resolution … µm/px … FOV … rotation …
mirror … registration …` — so the operator can confirm exactly what was applied.

**(2) "The registration to detect a frame's translation is very bad — explore
better methods (e.g. Fourier-Mellin); and as a backup a manual version."** NEW
`MosaicBuilder._register_overlap_fourier_mellin`: the FFT MAGNITUDE is
translation-invariant, so a log-polar resampling of it turns rotation → a row
shift and scale → a column shift that phase correlation recovers; B is then
de-rotated/de-scaled and phase-correlated with A for the residual translation. It
tolerates rotation + modest scale drift (+ CLAHE for illumination), where the
plain estimator loses lock; for adjacent tiles (same camera → rotation≈0, scale≈1)
it reduces to a robust translation estimate. `optimize_registration(method=…)`
uses it by DEFAULT (falls back to phase correlation on failure) and MEDIAN-
aggregates the per-pair rotation/scale as a diagnostic (a large tile-to-tile
residual flags an off camera calibration). Signs verified on synthetic data
(recovers −9,−4 for a +9,+4 shift; −8°/×0.909 for a +8°/×1.1 warp). **Manual
backup:** a `registration_method` builder param + a "Registration method" combo in
the mosaic-scan settings — **Fourier-Mellin (auto)** / **Phase correlation (auto)**
/ **Off — manual / stage only**. "Off" makes `optimize_registration` a NO-OP so
the tiles stand at their trusted stage positions and the operator aligns by hand
with the existing manual-align sliders + Store-FOV/spacing.

**(3) "The camera resolution must be defined in one spot and read by the camera as
ground truth."** There were TWO competing sources — `camera_config.active_resolution`
(the setup block) and `hw_controls.resolution` (per identity) — and on microscope
start active_resolution was applied only IF hw_controls had none. Fix: the
MICROSCOPE resolution now has ONE source of truth (`active_resolution`) — on start
the camera is ALWAYS driven to it (`_on_camera_started_hw` no longer gates on
hw_controls; `_apply_hw_controls` skips resolution for the microscope; non-
microscope cameras keep their per-identity resolution); `_maybe_apply_resolution_
to_device` reads `active_resolution` (combo only as fallback). Combined with the
Addendum-2 gear→block sync, the setup block is the single authoritative spot and
the camera reads it as ground truth.

Tests (suite now 34): Fourier-Mellin recovers translation + rotation/scale + flat
→ None; `optimize_registration` honours method = fourier_mellin / phase / off
(off = no-op); mosaic-settings `reg_method` default + round-trip. Updated
`test_v75x_camera_calibration_store.test_active_resolution_is_single_source_for_
microscope` (active_resolution now wins over hw_controls for the microscope — the
intended contract change) + the settings round-trip dict for the new `reg_method`
key. Regression green: plate-mosaic settings/worker/spacing + mapping + cal-
liveview + hardware-controls (63); objective / rotation / reanchor / store (115).

## Addendum 4 (2026-07-24) — "I literally just calibrated it" + wrong flip/rotation

Two reports right after calibrating the 4× objective.

**(A) The mosaic's objective-confirm warned "⚠ No stored per-objective
calibration" even though the µm/px in use (7.9067) was the calibrated value.**
KEY MISMATCH: `_ploc_confirm_objective` looked the objective store up with
`_get_camera_model_for_idx()` — which extracts a MODEL TOKEN (e.g. "BUC3D-1000C"
from "Bestscope BUC3D-1000C (ToupTek …)") — but the objective card WRITES under
the full `camera_spec.name` and the mosaic FOV read (`_ploc_microscope_um_per_px`)
also uses the full `camera_spec.name`. So the calibration was stored + the mosaic
DID use it; only the confirm note looked at the wrong key and warned falsely.
Fix: `_ploc_confirm_objective` now uses `camera_spec.name` (matching the write +
the FOV read).

**(B) "The full mosaic is not using the correct camera axis flipping and
rotation."** `_ploc_microscope_frame_orientation` read the LIVE manager
(`get_rotation_deg` / `get_mirrored`), which can come up UN-SYNCED after
navigating from the Hardware Setup page where the operator calibrated → the scan
saw 0°/unmirrored and applied no flip/rotation. Fix: per-field precedence — the
PERSISTED per-identity `CameraCalibrationStore` value (what the operator set via
the objective / flip-to-X/Y correction) is now GROUND TRUTH, with the live manager
filling any field the store lacks; a diagnostic line logs the resolved rotation /
mirror + which source each came from. (Both rotation AND mirror are applied
per-tile by `MosaicBuilder._orient_tile`; the stale "mirror corrected at the frame
source" comment was removed. If the applied direction is wrong, it's an
`_orient_tile` SIGN issue — the same one the flip-to-X/Y buttons let the operator
correct, now read reliably.)

Tests (suite 36): orientation reads from the store when the manager is un-synced;
the manager fills a field the store lacks. Regression green: mapping / reanchor /
worker / rosette (83); camera-store / mosaic-dialog / objective (64).

## Addendum 5 (2026-07-24) — "when I run the mosaic it is not flipping the x axis"

Even with Addendum 4 (the scan reads the persisted store as ground truth), the
flip still didn't reach the mosaic because the correction dialog's flip/rotate
(`MosaicCalibrationDialog._orient_set`) only pushed the orientation LIVE to the
manager and persisted to the store **only when the operator clicked "Apply to
camera setup"**. If they flipped X and closed without Apply, nothing was
persisted → the full scan (reading the store) saw no mirror → tiles were placed
un-flipped. Fix: `_orient_set` now **persists immediately** (new
`_persist_orientation` → `CameraCalibrationStore.set_rotation`/`set_mirrored` per
identity) on every flip / rotate / fine-angle change, so the flip is ground truth
the moment it's made and the full-plate scan applies it via
`MosaicBuilder._orient_tile` when it places each tile. The status text now says
"saved (applied to the mosaic + live view)" instead of "Apply to save". Tests
(suite 38): "Flip X axis" persists `mirrored=True`; rotate persists
`rotation_deg`. Regression green: scale-fov+registration / mosaic-adjust / mapping
(65). NOTE: if the applied direction is still wrong (flips the WRONG axis), that
is an `_orient_tile` SIGN issue, not a persistence one — flip the specific
sign/axis in `_orient_tile` to match the mount.

## Addendum 6 (2026-07-24) — per-camera Flip X / Flip Y / rotation (ONE unified system)

Two reports + an AskUserQuestion. (A) The mosaic's objective-confirm still warned
"No stored per-objective calibration" — a KEY MISMATCH: `_ploc_confirm_objective`
looked the objective store up with `_get_camera_model_for_idx()` (a model TOKEN)
while the card WRITES + the FOV read use the full `camera_spec.name`; fixed to use
`camera_spec.name`. (B) Operator: "on each camera I want a checkbox to flip axis
X, Y, and apply a custom rotation." AskUserQuestion → **keep ONE system: flip X /
flip Y / rotation affect the live view AND the mosaic AND click-mapping**;
controls **per-slot on Hardware Setup → Cameras**.

Extended the existing (mirror = flip X, rotation) orientation with an INDEPENDENT
**flip Y** so the transform is `R(θ)·diag(sx, sy)` everywhere: `MosaicBuilder.
_orient_tile` + ctor `frame_flip_y` + `set_frame_orientation(rot, mirrored,
flip_y)`; `CameraFeedView.set_view_orientation(..., flip_y)` + `_orient_qimage`
(`diag(mx, my)`) + `auto_orient` reads `full_orientation`;
`CameraManager.pixel_to_stage_offset` negates `dy`; `CameraManager.get/set_flip_y`
+ `full_orientation()→(flip_x, flip_y, rotation)`; `CameraCalibrationStore.get/
set_flip_y` (sibling of `mirrored`, False pops the key);
`_ploc_microscope_frame_orientation → (rotation, flip_x, flip_y)` (persisted store
= ground truth, manager fills gaps) threaded into both scan builders + the live
feed. Hardware Setup → Cameras per slot: **⇄ Flip X axis** + **⇅ Flip Y axis**
checkboxes + a **Rotation °** spin, each pushing live to ALL consumers +
persisting per identity (`_apply_slot_flip_y` / `_apply_slot_rotation_value`;
restored + enabled + synced). The calibrate dialog's flip buttons now toggle
flip_x / flip_y INDEPENDENTLY (`_orient_set(rot, flip_x, flip_y)`), persisting all
three. Tests: scale-fov suite 44 (+`_orient_tile` flip Y / independent X&Y /
manager+store flip_y / `pixel_to_stage_offset` dy / display transform); rotation-
monitor 42 (per-slot push+persist+restore + controls exist); updated the mosaic-
adjust dialog tests (independent flips, 3-arg `_orient_set`) + the slot-preview
fake signature. Regression green: scale-fov / mosaic-adjust / cal-liveview /
hardware-controls (90); rotation-monitor (42); mapping / camera-store / reanchor
(91); spheroid-picker / plate-mosaic / objective (48).

**Addendum 7 (2026-07-27, "the placement of the images is not on the correct side … stage +X ≠ camera +X, same for Y"):** the operator had to manually **flip X and Y both** to make the objective-view calibration mosaic stitch, then it came out **upside down**, and the full mosaic placed tiles **on the wrong side**. Root cause (traced through the transform composition): `MosaicBuilder._orient_tile` + stage placement is a mathematically-correct **stage-frame** orthophoto given the true camera→stage transform — but the orientation it was fed came from the operator's **eyeball live-view flips**, and a **mirror cannot be expressed as a rotation**, so a mirrored/180° microscope was never actually MEASURED. `PixelCalibrationDialog.plus_column_direction_deg` (the rotation model) **explicitly assumes a non-mirrored image**, so the scale/FOV auto-cal derived rotation only and **discarded the handedness** — even though both the +X and +Y stage-move displacements were captured (`_disp_x`, `_disp_y`), whose **cross-product sign encodes the mirror**. So the mosaic's tile orientation was guesswork, and combined with the plate view's `plate_flip_180` (=True, default from `None`) 180° display flip it double-counted to raw-content-at-flipped-positions = exactly the reported symptom. **Fix (measure it, don't guess):** new module helper `scale_fov_calibration_dialog.derive_camera_stage_orientation(dxX, dyX, dxY, dyY)` decomposes the FULL camera→stage 2×2 (image→stage `M = -(d/µmpx)·P⁻¹`) into the canonical `(rotation_deg, flip_x, flip_y)` that `_orient_tile` / `pixel_to_stage_offset` (`R(θ)·diag(mx,my)`) consume — `sign(det) = mx·my` puts the handedness on `flip_y`, θ from `M`'s first column. **Sign-locked** by `test_derive_camera_stage_orientation` (identity, mirror-X → `(180°, flipY)` ≡ flip-X, mirror-Y → `(0, flipY)`, **180° mount → `(180°, NO flip)`** since that's a proper rotation not a mirror, 90°, 90°+mirror), and provably reduces to `plus_column_direction_deg` for any non-mirrored camera (no regression). `_compute` now stores `result_flip_x`/`result_flip_y`; the auto-cal caller (`ObjectiveCalibrationCard._on_autocal_scale_fov_clicked`) pushes them to the manager (`set_mirrored`/`set_flip_y`) AND persists per camera identity alongside the rotation — so the live view, the mosaic (`_orient_tile`), and click-mapping all use the ONE **measured** orientation and tiles land on the correct side. The Verify-step re-sync now reads `full_orientation` (captures a manual correction's flip_y too). **Also — WYSIWYG calibration view:** the objective/calibration mosaic (`MosaicRegistrationView`) rendered in the STAGE frame while the full plate view (`JogWorkspaceView`) shows the PLATE frame (180° when `plate_flip_180`), so an orientation tuned in the dialog looked "upside down / wrong side" vs the full mosaic. New `MosaicRegistrationView.set_flip_180` (a whole-view 180° rotation, re-asserted in `fit`) + `MosaicCalibrationDialog(plate_flip_180=None)` (auto-resolves from `controller.plate_flip_180()`) seeds it → the calibration view now matches the full mosaic exactly. **Manual flips remain as override.** **Needs real-HW verification on ME3B V1/V3** (run the scale/FOV auto-cal → it reports the measured rotation + flips; the full mosaic then stitches with tiles on the correct side; if the GLOBAL sign is inverted on this camera build — the documented `plus_column_direction_deg` / `phaseCorrelate` caveat — one manual flip corrects it and persists). Tests: scale-fov suite +10 (`TestDeriveCameraStageOrientation` ×8 sign-lock + `TestScaleFovComputeDerivesFlips` ×2), mosaic-adjust +5 (`set_flip_180` view rotation + dialog plate-frame seeding); regression green (camera-scale-fov / mosaic-adjust / rotation-monitor / camera-store 145).

**Addendum 7b (2026-07-27, "on the calibration everything works … the image is built correctly the first try. When I hit the mosaic scan for the entire plate it still is not doing it correctly"):** the DECISIVE root cause of the full-plate divergence — a **double orientation**. The full-plate `_MosaicScanWorker` applied a **legacy coarse per-tile transform** `_orient_frame` (`none/rot180/fliph/flipv`, from `mosaic_scan.frame_orient`) to each raw frame BEFORE `add_raster_frame`, and then the builder applied the **calibrated** `_orient_tile` (rotation + flip X + flip Y from the ground-truth store) on top → orientation applied TWICE. The operator's `settings.json` had a stale `mosaic_scan.frame_orient = "rot180"` (set long ago to compensate before the calibrated system existed), so the whole-plate scan was rotated an extra 180°. The **calibration dialog worked** because it constructs its `MosaicCalibrationDialog` with `settings={}` → `frame_orient` defaults to `"none"` → no coarse transform → only the calibrated `_orient_tile` runs (exactly "calibration correct, full plate wrong"). **Fix:** the coarse `frame_orient` is **retired** in `_MosaicScanWorker` — `self._frame_orient` is forced `"none"`, `_orient_frame` is now an identity no-op, and the `frame = self._orient_frame(frame)` call is removed, so tile orientation is applied ONCE by the builder's calibrated `_orient_tile` on BOTH the full-plate scan and the calibration-dialog worker (same class). The Plate-Location scan setup no longer reads/passes `frame_orient`. `settings.json`'s stale `rot180` is left as-is because the SEPARATE fluorescence-mosaic worker (`_SingleWellMosaicWorker`, an "unoriented" builder) still reads that shared key for its own coarse orientation (documented follow-up) — the plate mosaic now simply ignores it. Tests: `test_v75x_plate_mosaic.TestMosaicWorker.test_worker_does_not_double_orient` (a stale `frame_orient="rot180"` is ignored + `_orient_frame` is a no-op); plate-mosaic worker / fluorescence-`frame_orient` / mapping / scale-fov / mosaic-adjust suites green (121). **This is the fix for the whole-plate scan; combined with 7 (measured handedness) the full mosaic should now assemble the same as the calibration.**

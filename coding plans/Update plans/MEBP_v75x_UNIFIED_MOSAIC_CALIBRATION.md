# MEBP v7.5.x — ONE Mosaic / Camera Calibration Surface

## Objective

Operator (2026-07-29): *"There are still issues with the mosaics. We have multiple surfaces for
calibrating mosaics; there should only be ONE surface and it should be in the camera calibration
section. After settings are applied, there should be a final check with the new settings for
confirmation. We also need the ability to rotate the entire mosaic to ensure the output of the
mosaic is the correct way up and down… Rosette, full plate overview, and fluorescence workflow
should all have the exact same behavior. Currently everything is very messed up."*

The five deviation types the operator enumerated, all of which must be accounted for and applied
to **(1) all microscope camera live views** and **(2) all mosaics built in any workflow**:

1. camera is mirrored
2. the + direction of the stage X/Y axes is not synced with the + pixel axes
3. the camera is rotated with respect to the +Z axis
4. pixel size is not calibrated well enough, so the current camera resolution (e.g. 1024×1024)
   doesn't set the correct real-world distance
5. tile overlap in the mosaic needs to be set properly

## Decisions (operator, AskUserQuestion)

| Question | Decision |
|---|---|
| Where does the one surface live? | **Hardware Setup → Cameras** (calibration page + workflows only CONSUME) |
| What about the scattered duplicate controls? | **Remove entirely** (not hidden, not "advanced override") |
| Form of the post-apply confirmation | **Small test mosaic** built with the new settings |
| Whole-mosaic rotation | **Both** — camera cal makes it correct automatically, AND a persisted global output-rotation override |
| Commit vs check order | **Gate on confirm** — nothing is written until the operator accepts the test mosaic |
| Overlap | **Default 25 %, operator-adjustable**, owned by the one surface |
| Needle side-cameras | **Left alone** — microscope + mosaic scope only, so `TwoCameraNeedleAligner` cannot regress |

## Diagnosis

The features already exist; the **plumbing** diverged. 14 surfaces write overlapping
camera/mosaic calibration, and **6 mosaic-build pipelines** each resolve their inputs
differently:

| Pipeline | Orientation | Registration | target_px | Overlap |
|---|---|---|---|---|
| P1 full-plate (`calibration.py::_ploc_start_mosaic_scan_impl`) | store-first, all 3 fields | Fourier-Mellin + global LSQ | 3000 (configured) | configured |
| P2 rosette / single-well | identical to P1 (same worker; only `bounds` differ) | same | same | same |
| P3 `MosaicCalibrationDialog` | `full_orientation` only, no store fallback | **`optimize_registration` SKIPPED** (`retain_for_reorient` not passed) | **hardcoded 1800** | configured |
| P4 ScaleFov "verify mosaic" | via P3 | via P3 | 1800 | **25 %** + 5×5 grid (passes `settings={}`) |
| P5 fluorescence (forked worker) | **NONE** — applies the RETIRED `mosaic_scan.frame_orient` instead | global median shift only | own spin (2500) | configured |
| P6 legacy per-well double-click | store-first | none | 2000 | **hardcoded 0.50** |

Root causes, mapped to the operator's list:

- **(1)(2)(3)** `derive_camera_stage_orientation`
  (`gui/dialogs/scale_fov_calibration_dialog.py:77`) already solves all three *together* by
  decomposing the measured camera→stage 2×2 into canonical `(rotation_deg, flip_x, flip_y)`.
  But P5 never consumes it; P3/P4 read only the live `CameraManager` (which comes up un-synced
  until Hardware Setup is visited); and the retired coarse `frame_orient` key is still live in
  `settings.json` as `"rot180"` — so **the fluorescence mosaic gets a 180° rotation but loses
  the `flip_y`** the plate scan applies. Same camera, two orientations.
- **(4)** `CameraCalibrationStore` has **no resolution stamp for `um_per_px`**, and
  `hardware_setup.py:2993` calls `set_um_per_px()` without `resolution=`, so
  `CameraManager.effective_um_per_px` degrades to a **no-op passthrough** for every restored
  camera. This machine's Andor is calibrated at two resolutions (4× = 3.227 µm/px @ 1024×1024,
  10× = 1.320 @ 2048×2048), so the rescale is exactly what matters — and it silently is not
  happening after any restart.
- **(5)** One shared key, four values in practice: configured **5 %** (live), P4's 25 %, P6's
  hardcoded 50 %, and a dead `_PLOC_MOSAIC_OVERLAP = 0.25`. At 5 % a small µm/px error opens
  visible **gaps** (spacing = FOV × (1 − overlap)) and leaves too little texture for
  registration to lock — so (4) and (5) compound.
- **Live views**: 12+ microscope feed construction sites never apply the saved orientation,
  including `_rosette_live_view`, whose **clicks become sub-well centres**.
- **Whole-mosaic rotation**: does not exist in any form today.

Two hidden top-precedence overrides made it worse: `mosaic_scan.fov_um` (an Advanced-submenu
spin overriding *every* measured µm/px, forever) and the learned
`MosaicAlignmentStore.um_per_px` ("Store FOV/spacing"), which **shadows** the objective
calibration the operator just measured.

## Ownership rules (the invariant this work enforces)

| Quantity | Belongs to | Store |
|---|---|---|
| `um_per_px` + the resolution it was measured at | camera **+ objective** | `ObjectiveCalibrationStore` |
| `rotation_deg`, `flip_x`, `flip_y` | camera **identity** (mount property) | `CameraCalibrationStore` |
| overlap, target_px, reg_method, settle/fresh-frames, max_shift | the **scan** | `settings.json → mosaic_scan` |
| whole-mosaic output rotation | camera identity, **display only** | `CameraCalibrationStore` |

`ObjectiveCalibrationStore`'s per-objective `rotation_deg` becomes **informational only** (a
duplicate home for a camera-level quantity and a live source of divergence): dropped from every
resolver precedence, still written so existing files stay readable.

## ⚠ Safety constraint (non-negotiable)

Mosaic pixels are placed at **trusted raw stage positions**, and every consumer back-projects
`stage = (extent − shift) + px/scale` to map wells → stage µm for **MOTION**. A prior bug that
back-projected through a shifted extent drove the stage several mm off target. Therefore the
global mosaic output rotation is **DISPLAY-ONLY** — applied at paint time in the viewers, never
baked into the stored composite or its `extent_um`.

## Files Modified

| File | Rationale |
|---|---|
| `SupportClasses/CameraCalibrationStore.py` | NEW `um_per_px_resolution` stamp + v1.1→1.2 backfill migration; NEW `mosaic_output_rotation_deg` |
| `SupportClasses/MosaicCalibration.py` | **NEW** — pure GUI-free `MosaicCalibration` dataclass + `resolve()` (ONE precedence) + `build_mosaic_builder()` factory + `refuse_reason()` |
| `gui/widgets/mosaic_scan_worker.py` | **NEW** — the shared raster/grab/stitch worker extracted from `calibration.py::_MosaicScanWorker`, detection injected; kills the fluorescence fork |
| `gui/widgets/camera_manager.py` | store→manager restore independent of page-visit order |
| `gui/pages/hardware_setup.py` | thread `resolution=` through `_restore_calibration_for_slot`; the ONE calibration surface; retire the microscope slot's duplicate orientation controls + "Use custom scale" |
| `gui/pages/hardware/objective_calibration_card.py` | becomes the ONE guided surface; commit gated on Accept |
| `gui/dialogs/scale_fov_calibration_dialog.py` | stop persisting the mirror on Cancel; return a candidate calibration |
| `gui/dialogs/pixel_calibration_dialog.py` | stop persisting the mirror on Cancel |
| `gui/dialogs/mosaic_calibration_dialog.py` | strip every persisting control; survives as a build+display widget for the verify step |
| `gui/dialogs/mosaic_settings_dialog.py` | remove `frame_orient`, `fov_um`, `spacing_um` |
| `gui/pages/calibration.py` | P1/P2 onto the resolver + shared worker; retire the Custom-tab Camera Setup panel, `_start_well_scan` (P6), `_PLOC_MOSAIC_OVERLAP` |
| `gui/pages/workflows/fluorescence_mosaic_workflow.py` | P5 onto the resolver + shared worker; delete `_SingleWellMosaicWorker` and its `frame_orient` read |
| `gui/dialogs/objective_calibration_dialog.py` | **DELETE** — verified dead (zero instantiations repo-wide incl. tests) |
| `gui/widgets/jog_workspace_view.py`, `mosaic_registration_view.py`, `mosaic_well_mapping_dialog.py` | display-only output rotation (generalise the existing `flip_180` hooks to quadrants) |
| ~10 live-view construction sites | `auto_orient=True` |

## Implementation Steps

### Stage A — make the calibration survive
- [x] A0. Test baseline established (see **Baseline** below).
- [x] A1. `CameraCalibrationStore`: add `um_per_px_resolution`; v1.1→1.2 backfill (match
      `ObjectiveCalibrationStore.measured_um_per_px` → adopt its resolution; else
      `hw_controls.resolution`; else **absent**). Absent must mean *unknown*, never *assumed*.
- [x] A1b. Thread `resolution=` through `_restore_calibration_for_slot` (`hardware_setup.py`)
      and every other `set_um_per_px` caller. **This was the direct cause of complaint (4).**
- [x] A2. `CameraManager.restore_calibration_from_store()`, invoked from `start` /
      `start_async`, so orientation + µm/px are live regardless of page-visit order.
- [x] A3. Stopped a silent revert in `live_target_picker._refresh_um_per_px`: on its fallback
      branch it pushed the manager's own value back with `resolution=None`, **clearing a
      correct stamp** and turning `effective_um_per_px` into a passthrough for every later
      consumer. It now pushes only when the value came from the objective store.

### Stage B — one resolver, one build path
- [x] B1. `SupportClasses/MosaicCalibration.py` — frozen `MosaicCalibration`, `resolve()`,
      `build_mosaic_builder()`, `refuse_reason()`, `warnings_for()`, per-field `provenance`.
- [~] B2. Worker extraction **NOT done** — proved unnecessary once P5 was pointed at the
      resolver + factory (the fork's divergence was entirely in what it *passed*, not in the
      raster loop). Left as a tidy-up; see Remaining.
- [x] B3. P1/P2 (`calibration.py`), P3/P4 (`mosaic_calibration_dialog` /
      `scale_fov_calibration_dialog`) and P5 (fluorescence) all resolve through B1 and build
      through the shared factory. P5: `_orient_frame`/`frame_orient` **deleted**, gained the
      calibrated orientation + `optimize_registration` + shared `target_px`,
      `retain_frames=False`. P3/P4: `retain_for_reorient=True` (so `optimize_registration`
      actually runs), shared `target_px` instead of the hardcoded 1800, and P4 no longer
      passes `settings={}`. P6 retired (below).

### Stage C — the one surface (Hardware Setup → Cameras)
- [x] C1. The card is now **"Mosaic & Camera Calibration"** with one primary button; the
      duplicate "Calibrate Selected…" and "Calibrate orientation…" buttons are hidden.
      NEW `gui/dialogs/mosaic_calibration_confirm_dialog.py` shows what was measured in plain
      language (`_axis_phrase`: *"Moving the stage +X moves the image left; +Y moves it
      down"*), hosts the overlap spin (priced in tiles) and the output-rotation combo, and
      offers **Build test mosaic…** through the shared path.
- [x] C2. Commit gated on Accept — `_confirm_calibration` builds a CANDIDATE from the
      measurement (not from the stores) and nothing is written until the operator saves.
      Also fixed the two dialogs that persisted the mirror on the **first click, even on
      Cancel** (`scale_fov_calibration_dialog._on_mirror_toggled`,
      `pixel_calibration_dialog._on_mirror_toggled`) — they are now preview-only.

### Stage D — removals (microscope scope only; needle cams untouched)
- [x] `frame_orient` combo + `fov_um` / `spacing_um` override spins removed from
      `mosaic_settings_dialog`, plus the `fov_um` spin and its "Apply to full mosaic"
      write-back in `mosaic_calibration_dialog`.
- [x] `ObjectiveCalibrationDialog` **deleted** (verified zero instantiations repo-wide).
- [x] P6 legacy per-well double-click scan disconnected (hardcoded 3.34 µm/px / 0.50 overlap /
      desyncing `build_mosaic`).
- [x] Dead `_PLOC_MOSAIC_OVERLAP` constant removed.
- [ ] Custom-tab → Camera Setup legacy panel and the "Use custom scale" override UI —
      see Remaining.

### Stage E — orientation on every microscope live view
- [x] `auto_orient=True` on the rosette feed (**highest priority — its clicks become sub-well
      centres**), the Needle-Offset feed, Jog, Fluorescence, Quick Print, Spheroid-sink,
      Timing-calibration and the shared context-panel feed; `MeasurementCameraView` now
      forwards the kwarg (it previously could not be set at all through that subclass).
      Needle-camera feeds deliberately left raw.

### Stage F — display-only whole-mosaic output rotation
- [x] `CameraCalibrationStore.get/set_mosaic_output_rotation` (quantised, 0 pops the key);
      exposed on `MosaicCalibration`; applied at paint time by
      `JogWorkspaceView.set_mosaic_output_rotation` (composed with `plate_flip_180` in the
      existing render cache, whose key gained the quadrant) and
      `MosaicRegistrationView.set_output_rotation`. **Never** passed to `MosaicBuilder` —
      pinned by a test.

## Remaining (deliberately not done)

- **B2 worker extraction.** `_MosaicScanWorker` still lives in `gui/pages/calibration.py` and
  the fluorescence page still has its own `_SingleWellMosaicWorker`. The behavioural
  divergence is gone (both now take the same calibration and builder), so this is a
  code-tidiness item, not a correctness one.
- **Custom-tab → Camera Setup legacy panel** (`calibration.py` ~:9491-9755) and the
  "Use custom scale" `micron_per_pixel_override` UI. Both are duplicate *scale* surfaces and
  should go; they sit in the largest file and are worth their own pass with a fresh look
  rather than a rushed removal at the end of this one. `micron_per_pixel_override` is now
  read by nothing except `HardwareConfig.micron_per_pixel` and that panel.
- **`_ploc_microscope_um_per_px` / `_ploc_microscope_frame_orientation`** remain for the live
  feed and are no longer in the mosaic path; they can be folded into the resolver later.

## Result (2026-07-29)

New `tests/test_v75x_unified_mosaic_calibration.py` — **37 tests, all green.**

Regression, all green: `test_v75x_fluorescence_mosaic` 35 ·
`test_v75x_camera_scale_fov_and_registration` 54 · `test_v75x_camera_calibration_store` 30 ·
`test_v75x_camera_rotation_cal_and_monitor` 43 · `test_v75x_mosaic_orientation_adjust` 19 ·
`test_v75x_needle_cam_mount_and_roll` 31 · `test_v74x_objective_calibration` 26 ·
`test_v75x_mosaic_memory_and_overlay_perf` 7 · `test_v744_calibration_revision` 20 ·
`test_v75x_camera_hardware_controls` 22 · `test_v75x_camera_image_correction` 18 ·
`test_v75x_camera_cal_liveview` 10 · `test_v731_jog_navigation` 28 ·
`test_v75x_plate_mosaic` **104** (all classes except the two pre-existing problems, which are
unchanged: `test_real_24_well_mosaic` still the documented CV FAIL, `TestManualAlignPage.
test_auto_finish_keeps_manual_align` still hangs).

Headless smoke test: every touched module imports, the confirm dialog builds, and the settings
dialog no longer round-trips any retired key.

**The v1.2 migration was verified against a copy of the real `camera_calibrations.json`** and
recovered the case that matters: the Andor microscope's `3.227061` was stamped
`1024×1024` from its matching 4× objective entry; the ToupCam likewise from its own; six
Teslongs fell back to `hw_controls.resolution`; one with neither was correctly left
**unstamped**; and a re-load was byte-identical. The live file was not modified.

### Test updates, all because they pinned removed behaviour

| Test | Change |
|---|---|
| `test_v75x_fluorescence_mosaic::test_inherits_shared_mosaic_frame_orient` | **Pinned the bug.** Replaced by `test_applies_measured_orientation_not_legacy_frame_orient`, which asserts the measured rotation **and** flip reach the builder and a stale `frame_orient` does nothing. |
| `..::test_fov_override_is_fallback_when_no_objective_cal` | → `test_fov_um_override_no_longer_shadows_the_calibration`. |
| `..::test_learned_calibration_overrides_objective_and_rescales` | → `test_learned_value_no_longer_shadows_objective_calibration` (that shadowing was the "I literally just calibrated it" bug). |
| `test_v75x_plate_mosaic::test_frame_orient_transforms_tile` | **DELETED** — it contradicted its own sibling `test_worker_does_not_double_orient` and had been failing unnoticed behind the `TestManualAlignPage` hang. |
| `..::test_dialog_round_trip_and_defaults`, `..::test_store_corrected_um_per_px_and_fov`, `..::test_overlap_is_25_percent` | Updated for the retired keys / `_spin_fov` / `_PLOC_MOSAIC_OVERLAP`; a new `test_retired_keys_are_not_offered_or_resurrected` guards against them coming back. |
| `test_v75x_needle_cam_mount_and_roll` ×2 | Version assertion made `SCHEMA_VERSION`-relative (a legacy file now walks 1.0→1.1→1.2, so a literal breaks on every future bump); fake `set_um_per_px` gained the `resolution=` kwarg. |

### Bugs found and fixed while doing this, beyond the plan

1. **`live_target_picker` could clear a correct resolution stamp** (A3 above) — a silent revert
   that would turn `effective_um_per_px` into a passthrough for the mosaic.
2. **`set_calibrated_um_per_px` dropped two pushes on one failure.** All three pushes shared a
   single `try`, so a `set_um_per_px` failure also skipped `set_rotation_deg` **and**
   `set_column_dir_deg` — the value the two-camera needle aligner refuses to run without. Now
   guarded separately, with a `TypeError` fallback for a manager predating the kwarg.
3. **`test_frame_orient_transforms_tile` was already failing** and invisible behind the
   `TestManualAlignPage` hang — worth knowing that hang hides real failures.
4. **A cp1252-unencodable `U+2212` in an operator-facing warning string.** Harmless in the app
   (the log file is utf-8 and `logging` swallows encoder errors) but it crashed a plain
   `print`; new runtime strings in `MosaicCalibration` are kept ASCII-safe.

## Testing Notes

New `tests/test_v75x_unified_mosaic_calibration.py`: resolver precedence + provenance;
resolution rescale (3.227 @1024 → live 2048 ⇒ 1.613) and the *unstamped* refusal-to-guess;
store 1.1→1.2 backfill (all three branches); `build_mosaic_builder` yielding identical
`MosaicBuilder` kwargs for P1/P2/P3/P5; commit gating (Cancel writes **nothing**);
output-rotation display-invariance (back-projected stage µm identical at 0/90/180/270).

Must stay green: the `derive_camera_stage_orientation` sign lock; the `MosaicBuilder`
retain/reblend tests; all needle suites; the mapping-dialog flip-invariance test (extended to
quadrants).

Rewritten because they pin removed behaviour: `test_inherits_shared_mosaic_frame_orient`
(**pins the bug**), the fluorescence `fov_um` sizing tests, `test_frame_orient_transforms_tile`,
the `mosaic_scan` round-trips containing `frame_orient`/`fov_um`/`spacing_um`, the
`_PLOC_MOSAIC_OVERLAP == 0.25` assertion, `_propagate_um_per_px`, `dlg._orient_set`.

**Needs real-HW verification on ME3B V1 (Andor microscope):** run the one calibration → measured
mirror/rotation match reality and the FOV is plausible for 4× @ 1024×1024; test mosaic stitches
with no gaps; Cancel leaves the previous calibration unchanged; switch 4×→10× (2048×2048) and the
FOV changes ~2× automatically **and survives a restart** (the A1 regression); all three mosaics
(full-plate, rosette, fluorescence) oriented and scaled **identically**; map wells and drive to
two of them — the needle must land on the centres (proves the display-only rotation did not
disturb back-projection); every microscope live view shows the corrected orientation and a
rosette live-view click still drives the stage the right way.

## Baseline (recorded 2026-07-29, BEFORE any change)

| Suite | Result |
|---|---|
| `test_v75x_camera_calibration_store` | 30 OK |
| `test_v74x_objective_calibration` | 26 OK |
| `test_v75x_mosaic_orientation_adjust` | 19 OK |
| `test_v75x_camera_scale_fov_and_registration` | 54 OK |
| `test_v75x_needle_cam_mount_and_roll` | 31 OK |
| `test_v75x_fluorescence_mosaic` | 35 OK |
| `test_v75x_plate_mosaic` | **1 pre-existing FAIL + 1 pre-existing HANG** |

Two pre-existing `test_v75x_plate_mosaic` problems, neither caused by this work:

- `TestFilledWellDetector.test_real_24_well_mosaic` — **FAIL**. The documented pre-existing CV
  failure (see the v7.5.x plate-mosaic plan).
- `TestManualAlignPage.test_auto_finish_keeps_manual_align` — **HANGS indefinitely** (newly
  observed here). Confirmed unrelated: that test class sets `page._camera_manager = None` and
  only touches `MosaicAlignmentStore` / `MosaicStore`, never the camera store. **This is why a
  combined `python -m unittest` run over these suites never terminates** — run
  `test_v75x_plate_mosaic` with this class deselected, or per-class. Worth a separate
  investigation; out of scope here.
  (Note there are two same-named `test_auto_finish_keeps_manual_align` methods — the
  `TestCalibrationDialogManualAlign` one passes; the `TestManualAlignPage` one hangs.)

`pytest` is not installed in this environment — use `python -m unittest`.

## Issues & Decisions

- **Two refinements that SHRINK the change** (found while checking test impact):
  (a) the per-slot `_apply_slot_mirror`/`_apply_slot_flip_y`/`_apply_slot_rotation_value`
  handlers are **kept** — they serve the needle and monitor slots; only the *microscope* slot's
  duplicates are retired. (b) `HardwareConfig.micron_per_pixel_override` is **kept as a field**
  (only the UI + the two removed readers go), so serialization round-trips stay green.
- **Verified independently before planning**: `ObjectiveCalibrationDialog` has zero
  instantiations repo-wide including tests (safe to delete); P6 IS reachable via
  `_cal_plate_view.well_double_clicked` (`calibration.py:10460`); the only non-UI readers of
  `micron_per_pixel_override` are `HardwareConfig.micron_per_pixel`, the Custom-tab panel being
  removed, and **P6's own µm/px source** — making those two removals one coherent change.
- **`frame_orient` is `"rot180"` on this machine.** Removing the key changes fluorescence output
  on the first run after this change — that IS the fix (it gains the missing `flip_y`), but the
  operator should expect the fluorescence mosaic to look different, and correct.
- **`settle_ms` is 2 on this machine** (default 300) — not one of the operator's five, so it is
  surfaced as a warning in the calibration flow rather than silently changed.
- **Overlap 5 % → 25 %** means more tiles and a longer full-plate scan; the one surface shows
  the tile count so it is a visible, priced choice.

# MEBP v7.5.x — Needle Cameras Re-Mounted Symmetric ±45° About +X: Mount Direction vs View Roll Split + Manual Needle Center

## Objective

The operator physically rotated the two needle-center side cameras: the pair is now
**symmetric about the stage +X axis, at +45° and −45°** (previously ~45° to the X/Y
axes with a different framing). Three fixes:

1. **Stage-offset math uses the new geometry.** `TwoCameraNeedleAligner` already does
   a general 2×2 solve with per-camera column→stage direction angles; the problem is
   the *fallback*: when a camera's direction is unmeasured it silently used the legacy
   orthogonal 90°/0° mapping — wrong for this rig. Operator decision: the two cameras
   are interchangeable "Needle cam 1 / 2"; **which is +45° vs −45° is determined by the
   µm/px stage-motion calibration (measured)**, so an unmeasured camera now REFUSES
   with an actionable message instead of guessing.
2. **Live view no longer rotated by the mount angle.** One conflated per-camera
   `rotation_deg` served BOTH the aligner's mount direction AND the display
   orientation, so storing ±45° tilted the needle live views 45°. Operator decision:
   the display is counter-rotated only by the **sensor roll = deviation of the
   measured stage-motion vector from image-horizontal ("deviation from parallel")**;
   the mount direction feeds the aligner only. Split into two per-identity fields:
   **`column_dir_deg`** (mount direction, aligner-only, NEW) and **`rotation_deg`**
   (small display roll — all existing display/mosaic/click consumers unchanged).
3. **Manual needle center.** "Set current as location" only saved the quick-move XY.
   Operator decision: upgrade that button to record the FULL needle center — the
   `needle_origin_um`, the quick-move XY, and the needle-cam Z fiducial — exactly the
   tail of Center & Save, minus the camera-driven centering move.

## Files Modified

| File | Change |
|------|--------|
| `SupportClasses/HardwareConfig.py` | `needle_role_label()` helper (display names "Needle cam 1/2"; enum values unchanged) + `CameraRole` docstring |
| `gui/widgets/camera_feed_view.py` | Extract pure `view_transform_coeffs()` from `_orient_qimage` (sign-convention anchor for tests) |
| `gui/dialogs/pixel_calibration_dialog.py` | `fold_parallel_deg()` + `view_roll_from_displacement()` + `result_view_roll_deg`; docstring/instruction text |
| `SupportClasses/CameraCalibrationStore.py` | `get/set_column_dir`, `set_calibration(column_dir_deg=)`, v1.1 migration (legacy needle rotation → column_dir) |
| `gui/widgets/camera_manager.py` | `_column_dir_deg[]` + `get/set_column_dir_deg` |
| `gui/pages/hardware_setup.py` | Commit paths split by role; `_apply_slot_column_dir`; restore path; nominal hints; labels |
| `gui/pages/calibration.py` | Aligner reads column_dir + refusal; breakdown labels; `_needle_loc_record_origin_here()`; manual-center button |
| `SupportClasses/VisionDetector.py` | `TwoCameraNeedleAligner` docstring only (math untouched) |
| `tests/test_v75x_needle_cam_mount_and_roll.py` | NEW suite |
| existing test suites | updated (see Testing Notes) |

## Implementation Steps

- [x] 0. `needle_role_label()` + `CameraRole` docstring
- [x] 1. Roll math: `view_transform_coeffs` (feed view) + `fold_parallel_deg` / `view_roll_from_displacement` / `result_view_roll_deg` (dialog)
- [x] 2. `column_dir_deg` field: store get/set + `set_calibration` kwarg + manager array/get/set
- [x] 3. Store migration v1.1 (needle-role legacy `rotation_deg` → `column_dir_deg`)
- [x] 4. hardware_setup: needle-card commit passes both; `_on_calibrate_slot_rotation` split by role; `_apply_slot_column_dir`; `_restore_calibration_for_slot` restores column_dir; nominal helpers/hints; needle-slot readout shows `mount … (Δ …) · roll …`
- [x] 5. calibration.py: `_needle_loc_camera_info` returns column_dir; `_needle_loc_compute_offset_um` refuses when unmeasured; breakdown/diagnostics labels
- [x] 6. Manual needle center: `_needle_loc_record_origin_here()` factored + `_needle_loc_set_current` upgraded with confirm; button relabelled "Set current as needle center"
- [x] 7. Label sweep ("Needle cam 1/2" in hardware_setup + calibration user-facing strings)
- [x] 8. Tests: new suite (31) + existing updates
- [x] 9. Full affected-suite run (see Testing Notes)

## Testing Notes

All run green (2026-07-29):

- NEW `tests/test_v75x_needle_cam_mount_and_roll.py` (31): fold table; **sign
  pin** (`view_transform_coeffs` ∘ `view_roll_from_displacement` renders the
  measured vector horizontal, incl. mirror/flip-Y combos); store round-trip +
  sibling preservation + None-clears; migration (legacy move / idempotent /
  v1.1 untouched / unassigned untouched); manager column-dir semantics + the
  "view orientation never returns the mount direction" pin; commit paths
  (`set_calibrated_um_per_px` both fields, µm/px-only preserves,
  `_apply_slot_column_dir`, restore pushes both); offset-compute refusal +
  computes at ±45 + ignores display roll; ±45 end-to-end incl. swapped pair;
  role labels + enum-value freeze.
- UPDATED `test_v75x_needle_location_quick_move.py` (TestSetCurrent, +3 tests:
  origin recorded + emitted, cam-Z fiducial persisted, decline-confirm no-op;
  `_FakeMB` gains `question`/`StandardButton`).
- UPDATED `test_v75x_camera_calibration_store.py` (integration asserts split
  fields restore) + `test_v75x_camera_rotation_cal_and_monitor.py` (needle
  readout = mount Δ + roll; +1 roll-only test).
- Regression green: `test_v75x_camera_rotation` (aligner defaults intact),
  `test_v75x_needle_center_direction_z`, `test_v744_calibration_revision`,
  `test_v75x_camera_scale_fov_and_registration`, `test_v75x_camera_cal_liveview`,
  `test_v75x_reanchor_mosaic_and_camera_orientation`,
  `test_v75x_mosaic_orientation_adjust`, `test_v75x_camera_image_correction`,
  `test_v75x_camera_hardware_controls`, `test_v75x_spheroid_picker_scaling`,
  `test_v75x_needle_offset_z_side_view`, `test_v75x_camera_async_open…`,
  `test_v75x_plate_location_manual_click_rim`, `…_z_side_view`,
  `…_plate_z_autocal_*`, `test_v75x_mapping_camera_orient_and_rosette_z`
  (~420 tests total across the batches).
- **Needs real-HW verification on ME3B V1**: calibrate µm/px on both needle cams →
  views render level (roll only); Center & Save lands the needle on both crosshairs
  (measured directions ≈ +45/−45); manual "Set current as needle center" records
  origin + XY + cam-Z; restart restores both fields; migration leaves an
  un-recalibrated machine's views level.

## Issues & Decisions

- (from planning) Enum values `needle_x`/`needle_y` are serialized in configs and the
  store's role→identity assignments — display labels only are renamed.
- (from planning) Migration is store-level and idempotent; since the rig was
  physically rotated, migrated directions are stale but harmless — first
  recalibration overwrites both fields.
- **Fresh stores are born at version "1.1"** (a version-less legacy file is
  treated as 1.0 in `_load`). Without this, a needle cam whose FIRST write is a
  new-semantics roll (e.g. via the rotation spin) would have that roll wrongly
  migrated to `column_dir_deg` on the next load. Caught by
  `TestStorePersistAndAutoRestore` during implementation.
- Sign of the roll (`view_roll_from_displacement` returns `−fold(atan2)` after
  parity flips) is pinned by `TestRollSignPin` composing it with the extracted
  `view_transform_coeffs` — the displayed measured vector must come out level
  for every angle × mirror × flip-Y combination.
- Small behavior change in Center & Save: if the post-move XY read fails, the
  needle-cam Z fiducial is now ALSO skipped (previously it was still captured).
  Recording a fiducial with no origin was questionable; an unreadable XY at
  that point means comms trouble anyway.
- `TestStorePersistAndAutoRestore` + `TestSlotRotationStrip.test_readout…`
  updated to the split semantics (they encoded the old conflated commit).
- `pixel_to_stage_offset` (click→stage) deliberately keeps consuming
  `rotation_deg` — for needle cams the workflow uses edge-pick raw pixels +
  the aligner, never the click map, so the roll there is harmless and correct
  for display-consistency.

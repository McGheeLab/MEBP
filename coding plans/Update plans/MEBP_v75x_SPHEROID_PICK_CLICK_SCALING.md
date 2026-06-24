# MEBP v7.5.x — Spheroid Pick & Place click→stage conversion fixes

## Objective

Fix two bugs in the Spheroid Pick & Place workflow's live-view target picking,
both reported by the operator:

1. **Picked locations don't match when the wrong camera resolution is selected**
   — the µm/px conversion ignored the live frame resolution.
2. **The scale of expected motion is "way off" / "orders of magnitude" off** —
   surfaced on HW as
   `wait_for_xy_arrival timeout (30.0s): target=(64735.02, 49557.15), actual=(114.332, 76.645)`
   (the stage drove to the XY-envelope corner `114332×76645 µm`).

## Root Causes

### Bug A — 1000× unit error (the catastrophic one)

`StageController.get_xy_position(cached=True)` returns **absolute stage µm**
(`get_xy_position_mm` divides it by 1000; the Jog page uses
`xy[0] - zero["x"]` with no scaling). The spheroid workflow treated it as **mm**
and multiplied by 1000:

- `LiveTargetPicker._read_stage_xy_mm` returned the µm value labelled "mm", and
  `_pixel_to_stage_um` did `sx_um = stage_xy_mm[0] * 1000.0` → every picked
  target ~1000× too large. With the stage at ~64735 µm, a target became
  ~64,735,000 µm; the executor's `safe_travel_to`/`move_xy_absolute_um` clamped
  it to the envelope corner, and `wait_for_xy_arrival((target-zero)/1000 ≈
  64735 mm)` timed out — exactly the logged numbers.
- `SpheroidPickupWorkflowPage._refresh_position_indicators` had the same
  `xy[0] * 1000.0 - zero["x"]` error for the workspace position dot + XZ needle
  position (display-only, but still wrong/off-canvas).
- `LiveTargetPicker._refresh_stage_position` pushed `xy_mm * 1000` into the
  overlay's stage centre.

### Bug B — resolution µm/px scaling

Objective µm/px is measured at a specific frame resolution
(`objectives.json` records e.g. `916×686`), but was applied to the live frame
regardless of its actual size. A wider live frame samples the same FOV across
more pixels, so µm/px ∝ 1/width; using the calibration value at a 2× / 4× live
resolution gave a 2× / 4× scale error. The objective card already *recorded*
the calibration resolution (and warns ⚠ on mismatch in its table) — nothing
*used* it in the live transform.

## Fix

Made the shared pixel→stage transform resolution-aware, and corrected the
spheroid workflow's unit handling.

- **`gui/widgets/camera_manager.py`**
  - New `_um_per_px_res[]` tracks the (w,h) each slot's µm/px was measured at.
  - `set_um_per_px(cam_idx, value, resolution=None)` — optional resolution,
    paired with the value (None clears it → no rescale).
  - New `effective_um_per_px(cam_idx, live_width)` = `value × calib_w /
    live_w` (or value if resolution unknown). New `get_um_per_px_resolution`.
  - `pixel_to_stage_offset` now resolves µm/px via `effective_um_per_px` against
    the **live** `image_w` passed in (getattr-guarded so lightweight test
    doubles exposing only `get_um_per_px` still work). This also fixes the
    Plate-Location click-rim path for free.
- **`gui/pages/hardware/objective_calibration_card.py`** — both
  `set_um_per_px` call sites (objective swap + after a calibration) now pass
  `resolution=` so the manager can rescale.
- **`gui/widgets/live_target_picker.py`**
  - `_read_stage_xy_mm` → `_read_stage_xy_um`: returns `get_xy_position` µm
    directly (no ×1000). `_pixel_to_stage_um` adds the in-frame µm offset to
    the absolute µm stage centre.
  - `_refresh_stage_position` pushes raw µm to the overlay (no ×1000) and syncs
    µm/px to the live resolution.
  - `_refresh_um_per_px` resolves `(base, calib_resolution)` from the objective
    store, pushes both to the manager, and applies the live-rescaled value.
  - New `_sync_live_um_per_px(force=False)`: recomputes the effective µm/px for
    the current live frame width (debounced on width change) and pushes it to
    the overlay view + readout (which shows `cal X@W → live Wpx` when scaled).
- **`gui/pages/workflows/spheroid_pickup_workflow.py`** —
  `_refresh_position_indicators` uses `xy[0] - zero["x"]` (no ×1000), matching
  the Jog page.

## Files Modified

| File | Change |
|------|--------|
| `gui/widgets/camera_manager.py` | resolution-aware µm/px (`_um_per_px_res`, `set_um_per_px(resolution=)`, `effective_um_per_px`, `get_um_per_px_resolution`, `pixel_to_stage_offset`) |
| `gui/pages/hardware/objective_calibration_card.py` | pass `resolution=` on both `set_um_per_px` pushes |
| `gui/widgets/live_target_picker.py` | fix 1000× (`_read_stage_xy_um`), resolution-scaled µm/px + view sync |
| `gui/pages/workflows/spheroid_pickup_workflow.py` | `_refresh_position_indicators` no ×1000 |
| `tests/test_v75x_spheroid_picker_scaling.py` | new (12 tests) |

## Implementation Steps

- [x] CameraManager resolution-aware µm/px transform
- [x] Objective card passes calibration resolution
- [x] LiveTargetPicker 1000× fix + live-resolution µm/px sync
- [x] Spheroid page position-indicator 1000× fix
- [x] Regression tests (12)
- [x] Existing affected suites green

## Testing Notes

- `tests/test_v75x_spheroid_picker_scaling.py` (12): `effective_um_per_px`
  rescaling, resolution-aware `pixel_to_stage_offset`, picker click → absolute
  µm (centre click returns the stage position, not 1000× it), live-resolution
  scaling, and `_refresh_stage_position` pushing µm not mm.
- Affected suites green: `test_v75x_plate_location_manual_click_rim` (uses the
  real `pixel_to_stage_offset` via a fake), `test_v75x_z_retract_before_xy_travel`
  (spheroid click handler), `test_v74x_objective_calibration`,
  `test_v75x_camera_rotation`, `test_v75x_camera_calibration_store`,
  `test_v75x_camera_cal_liveview`, `test_v75x_camera_hardware_controls`,
  `test_v75x_camera_image_correction`, `test_v75x_needle_center_direction_z`,
  `test_v75x_quick_print_workflow`, `test_v744_calibration_revision`.
  (`test_v75x_plate_mosaic::test_real_24_well_mosaic` 23/24 is the documented
  pre-existing unrelated OpenCV failure.)
- **Needs real-HW verification on ME3B V1**: pick a spheroid in the live view —
  the stage should travel to the clicked spot (not the envelope corner), and the
  µm/px readout should follow the camera resolution combo (the picker header
  shows `cal X@W → live Wpx` when the live resolution differs from calibration).

## Issues & Decisions

- **Why central `pixel_to_stage_offset` rescaling vs. picker-local?** The
  transform already receives `image_w` (the live frame size), so the manager is
  the natural home; it fixes the Plate-Location click-rim path too. Kept
  getattr-guarded so existing fake CameraManagers (which expose only
  `get_um_per_px`) are unaffected.
- **Overlay consistency.** The overlay draws µm→pixel with the *view's*
  `_um_per_px`, so the picker pushes the live-rescaled value to the view
  (`_sync_live_um_per_px`) to keep it consistent with the click transform.
- `get_xy_position` returns **µm** — see also `MEBP_v75x_*` XY envelope plans
  and the `xy-move-unit-contract` memory.

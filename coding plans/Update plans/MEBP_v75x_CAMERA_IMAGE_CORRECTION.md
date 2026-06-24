# MEBP v7.5.x — Per-Camera Image Correction (Gamma / Brightness / Contrast)

## Objective

Give the operator a per-camera **gamma / brightness / contrast** correction so
each physical camera's live feed can be tuned independently (a washed-out
needle cam, a dim microscope, etc.). The correction is **display-only** — it
affects every live view of that camera (its `frame_captured` QImage, hence all
`CameraFeedView` previews) but **NOT** the raw `_current_frame` buffer used by
detection / calibration, so vision math keeps operating on unmodified sensor
data. Settings persist **per physical camera** (device identity) so they
auto-restore next session, exactly like the µm/px calibration.

## Background / Existing State

- `CameraWidget` already had software **brightness** (`set_brightness`, beta
  offset) and **gamma** (`set_gamma`, LUT) applied in `_grab_frame` *after* the
  raw frame is cached for detection. There was **no contrast**.
- Those controls only had UI when `show_controls=True`, but `CameraManager`
  builds every widget with `show_controls=False`, so in practice no UI ever
  exposed them and nothing persisted them.
- `CameraCalibrationStore` already persists per-device (identity-keyed) camera
  properties (µm/px, rotation) in `config/hardware/camera_calibrations.json` and
  auto-restores them via `HardwareSetupPage._restore_calibration_for_slot`.

## Files Modified

| File | Rationale |
|------|-----------|
| `gui/widgets/camera_widget.py` | Add `_contrast` + `set_contrast`/`contrast`; combine brightness+contrast in one `convertScaleAbs` pivoted on mid-gray (backward-compatible at contrast=1.0); add a contrast slider to the built-in settings panel; `reset_image_correction()` + `image_correction()` snapshot helpers. |
| `gui/widgets/camera_manager.py` | Per-camera `get/set_brightness`, `get/set_contrast`, `get/set_gamma`, `reset_image_correction`, `image_correction` — delegate to the persistent `CameraWidget` (single source of truth; survives stop/start since the widget is not recreated). |
| `SupportClasses/CameraCalibrationStore.py` | `get_image_correction` / `set_image_correction` — store `cameras[identity]["image_correction"] = {brightness, contrast, gamma}`; identity-keyed, per-machine, same pattern as µm/px. |
| `gui/pages/hardware_setup.py` | Per-slot **Image Correction** control strip beneath each camera's live preview (brightness / contrast / gamma sliders + Reset); wired to `CameraManager` setters + persisted to the store by device identity; `_restore_calibration_for_slot` also restores+applies stored correction. |

## Implementation Steps

1. [x] `CameraWidget`: `_contrast=1.0`, `set_contrast(0.1..3.0)`, `contrast`
   property, `image_correction()` dict, `reset_image_correction()`.
2. [x] `CameraWidget._grab_frame`: single `convertScaleAbs(alpha=contrast,
   beta=128-128*contrast+brightness)` so contrast pivots on mid-gray and
   brightness is an additive offset (reduces to old behavior at contrast=1.0).
3. [x] `CameraWidget` UI: contrast slider in the settings panel (between
   brightness and gamma); `set_*` now refresh the slider widgets if present.
4. [x] `CameraManager`: per-camera correction getters/setters delegating to the
   slot's `CameraWidget`; `reset_image_correction`; `image_correction`.
5. [x] `CameraCalibrationStore`: `get/set_image_correction` (nested dict under
   the identity entry; preserves µm/px + rotation siblings).
6. [x] `hardware_setup.py`: build the per-slot correction strip; wire sliders →
   `CameraManager` + store-by-identity; apply on restore.
7. [x] Tests: `tests/test_v75x_camera_image_correction.py`.

## Testing Notes

- `python -m unittest tests.test_v75x_camera_image_correction -v` (18 tests, green)
- Bench: start each camera on Hardware Setup → Cameras, drag brightness /
  contrast / gamma → live preview changes; Reset returns to neutral; restart →
  values restore for the same physical camera; needle detection / µm/px
  calibration still operate on the raw (uncorrected) frame.

## Issues & Decisions

- **Display-only.** Correction is applied to the displayed/`frame_captured`
  frame only, after the raw frame is cached for detection — consistent with the
  pre-existing brightness/gamma behavior. Vision/calibration must see real
  sensor data. (If a future need arises to correct the detection frame, that is
  a separate, deliberate change.)
- **Contrast pivot.** `out = contrast*(in-128)+128+brightness` keeps mid-gray
  fixed when raising contrast (a proper contrast control), and is identical to
  the old brightness-only path when contrast=1.0 (no regression).
- **Persistence keyed by device identity**, reusing `CameraCalibrationStore`
  (per-machine, survives hardware-setup-file loads) rather than `HardwareConfig`
  — same reasoning as the µm/px store.
- **Single source of truth.** The `CameraWidget` holds the live values; the
  manager delegates. The widget persists across stop/start/source-change, so
  the correction is not lost when a camera is restarted.

# MEBP v7.5.x — Camera-Side (Hardware) Controls + Resolution + Readout

## Objective

Let the operator change a camera's **firmware/SDK settings** — exposure (with an
auto-exposure toggle), analog gain, gamma, brightness, contrast — and the
**device capture resolution**, with a **terminal readout that proves the values
come from the camera**. Distinct from the software post-processing "Image
Correction" (`MEBP_v75x_CAMERA_IMAGE_CORRECTION.md`), which only touches
displayed frames.

Driven by the **BUC3D-1000C** microscope camera, which enumerates through the
**ToupTek SDK** as `C3CMOS10000KPA` (probed live).

## Hardware probe (ground truth)

`toupcam.dll` found at `C:\Program Files\ToupTek\ToupView\x64`. Opening the
device and reading every control back succeeded:

| Setting | Current | Range |
|---|---|---|
| Brightness | 0 | −64…64 (def 0) |
| Contrast | 0 | −100…100 (def 0) |
| Gamma | 100 | 20…180 (def 100) |
| Exposure | 70000 µs | 200…2,000,000 µs |
| Gain | 100 % | (1×) |
| Auto-exposure | ON | — |
| Resolution | eSize 0 = 3664×2748 | + 1832×1374, 912×686 |

Findings that shaped the design: (a) hardware controls were **not exposed** in
our ToupCam backend (only `set_auto_exposure` + `set_resolution_index`); (b) the
Microscope **Resolution** combo only set the µm/px *spec* — it never called
`put_eSize`, so it never reconfigured the camera; (c) there was **no readback**;
(d) `EnumV2` returns an empty resolution list on this SDK build — must query
`ResolutionNumber`/`get_Resolution` from the open handle instead.

## Design decisions (confirmed with user)

- **Full control, auto-exposure OFF on demand.** Manual exposure/gain only take
  effect with auto-exposure off; the dialog disables those sliders while auto is
  on. Camera already auto-picks a light preview resolution (≤1280 wide → 912×686).
- **UI = gear button on the microscope video → pop-out window.** A small ⚙ in
  the top-right of any *controllable* camera feed opens a modeless quick-settings
  dialog with a live preview.
- **Readout cannot lie.** Every displayed/logged value is fetched via SDK
  getters against the open handle; the payload always carries a `source` field
  (`toupcam`/`opencv`/`simulated`/`none`).
- **Persist per device identity** in `CameraCalibrationStore` (`hw_controls`),
  re-applied on camera start; that start also logs the read-back block.

## Files Modified

| File | Rationale |
|------|-----------|
| `gui/widgets/toupcam_backend.py` | Hardware getters/setters (`get/put_Brightness/Contrast/Gamma/ExpoTime/ExpoAGain`, `get_AutoExpoEnable`), `_bind`/`_get_scalar`/`_put_scalar`/`_get_triple` ctypes helpers (lazy, signature-missing-safe), `get_exposure_time_range`/`get_exposure_gain_range`, `get_eSize`, `get_resolution_list` (queries the open handle — works around empty EnumV2 list), `get_settings()` device readback, `HW_RANGES`. |
| `gui/widgets/camera_widget.py` | Cross-backend API: `hardware_capabilities()`, `get_hw_settings()` (source-labelled), `set_hw_auto_exposure/exposure_us/exposure_gain/gamma/brightness/contrast`, `set_capture_resolution()` (ToupCam→nearest eSize+restart; OpenCV→CAP_PROP_FRAME_*), `log_hw_settings()`. OpenCV fallback via CAP_PROP_*. |
| `gui/widgets/camera_manager.py` | Per-slot delegation of all the above + `log_hw_settings`. |
| `gui/widgets/camera_feed_view.py` | ⚙ gear overlay (top-right), visible only when `hardware_capabilities().controllable`; opens/reuses a modeless `CameraSettingsDialog`; new `enable_settings` kwarg (False for the dialog's own preview → no recursion); reposition/visibility hooks. |
| `gui/dialogs/camera_settings_dialog.py` | **NEW** modeless pop-out: reads caps+settings from the device, builds capability-gated controls (resolution combo / auto-exposure / exposure-ms / gain / gamma / brightness / contrast), live-applies + persists, "⟳ Read from camera" + "Defaults", monospace readout mirror, small live preview. |
| `SupportClasses/CameraCalibrationStore.py` | `get/set_hw_controls` — nested `hw_controls` in the identity entry (None-dropped), siblings (µm/px, image_correction) preserved. |
| `gui/pages/hardware_setup.py` | `camera_started → _on_camera_started_hw` (restore persisted controls + always log read-back); `_apply_hw_controls`; `_maybe_apply_resolution_to_device` so the spec Resolution combo also drives the live microscope device. |

## Implementation Steps

1. [x] ToupCam backend hardware getters/setters + resolution list + `get_settings`.
2. [x] CameraWidget cross-backend hardware API + `set_capture_resolution` + `log_hw_settings`.
3. [x] CameraManager delegation.
4. [x] CameraCalibrationStore `hw_controls`.
5. [x] `CameraSettingsDialog` (pop-out).
6. [x] CameraFeedView gear button + visibility.
7. [x] hardware_setup restore-on-start + readback log + Resolution-combo → device.
8. [x] Tests `tests/test_v75x_camera_hardware_controls.py` (17, green).
9. [~] Adversarial multi-dimension review workflow → apply confirmed findings.

## Testing Notes

- `python -m unittest tests.test_v75x_camera_hardware_controls -v` (17, green).
  Uses a `FakeToupCam` injected into a real `CameraWidget`.
- Real-HW bench: start the microscope; ⚙ appears top-right; pop-out shows real
  values; drag gamma/brightness/exposure (auto off) → live feed changes; change
  resolution → terminal logs the device-adopted mode; restart → values restore;
  every start prints a `[camera start] … source = toupcam` block.

## Issues & Decisions

- **Resolution change invalidates µm/px.** Changing capture resolution changes
  the effective µm/px (different binning), so `set_capture_resolution` logs a
  "calibration may need redoing" warning. (Future: auto-flag the calibration stale.)
- **`enable_settings` defaults True**, so the gear appears on *any* controllable
  camera feed (microscope = ToupCam; also OpenCV needle cams). Sim/stopped =
  no gear. Easy to scope to microscope-only later if desired.
- **OpenCV hardware control is best-effort** (UVC CAP_PROP_* is driver-specific
  and often a no-op); the readback shows reality so it can't silently lie.

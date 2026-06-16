# MEBP v7.5.x — Camera Calibration Start Buttons, Live View & Needle-Zero µm/px Fix

## Objective

Three hardware-surfaced camera-calibration fixes on the **Hardware Setup → Cameras**
sub-page and the **Calibration → Needle Location** workflow:

1. **Start buttons on the camera calibration page** — add a per-slot **Start/Stop**
   control in the *Camera Detection & Assignment* section so the user can power any
   assigned camera on/off without leaving the page (previously cameras only started
   inside the modal calibration dialogs).
2. **Live view while calibrating** — add a compact live `CameraFeedView` preview under
   each slot row so the user can verify the camera is showing reality *before and
   while* running a calibration. (The calibration dialogs themselves already show a
   live feed via `MeasurementCameraView` / `PixelCalibrationDialog`.)
3. **Needle-zero can't read the needle-camera µm/px** — the *Needle Location*
   (needle-zero) workflow and the *Plate Location* edge-fit read
   `getattr(cam, "_um_per_px", 0.0)` off the **CameraWidget**, which never carries that
   attribute, so both always saw `0.0` and reported *"camera µm/px or frame size not
   available"* — even right after the needle camera was calibrated in Hardware Setup.
   The canonical value lives on the shared `CameraManager` (`get_um_per_px`).

## Root Cause (issue 3)

- `gui/pages/calibration.py::_needle_loc_camera_info` (line ~1416) and the plate-location
  guard (line ~2033) read `getattr(<CameraWidget>, "_um_per_px", 0.0)`.
- `CameraWidget` has **no** `_um_per_px` attribute. µm/px is stored on
  `CameraManager._um_per_px[cam_idx]`, written by:
  - `HardwareSetupPage.set_calibrated_um_per_px` → `mgr.set_um_per_px` (needle/plate),
  - `ObjectiveCalibrationDialog` / `ObjectiveCalibrationCard` (microscope),
  - `calibration.py::_ctx_update_um_px_display` (microscope context panel).
- Established correct pattern: `gui/widgets/live_target_picker.py::_refresh_um_per_px`
  reads the `ObjectiveCalibrationStore` then falls back to `mgr.get_um_per_px`.
- Naively switching to `mgr.get_um_per_px` would regress the *uncalibrated* case:
  the manager seeds every slot to a **1.67** default, which would silently pass the
  `> 0` guard and use a bogus scale. So `CameraManager` gains an explicit
  *was-it-actually-set* flag (`is_um_per_px_calibrated`) to preserve the
  "fail → tell the user to calibrate" path.

## Files Modified

| File | Change |
|------|--------|
| `gui/widgets/camera_manager.py` | Track explicit calibration per slot: `_um_per_px_set` flag, set in `set_um_per_px`; new `is_um_per_px_calibrated(cam_idx)` accessor. |
| `gui/pages/calibration.py` | `_needle_loc_camera_info` + plate-location guard now read `mgr.get_um_per_px(cam_idx)` gated on `mgr.is_um_per_px_calibrated(cam_idx)` (else `0.0` → unchanged "calibrate first" message). |
| `gui/pages/hardware_setup.py` | Cameras sub-page *Detection & Assignment* rows gain a Start/Stop button + a collapsible live `CameraFeedView` preview; source-combo selection now drives `mgr.set_source`; preview/button state tracked via `camera_started`/`camera_stopped` signals. **Persistence (identity-keyed):** `set_calibrated_um_per_px` resolves the slot's device identity via `mgr.camera_identity` and writes `config.camera_calibrations[identity]` + fires `_on_config_changed` (save); `_restore_calibration_for_slot` / `_restore_all_calibrations` re-apply a stored value whenever a slot's source is assigned (`_on_live_cam_source_changed`, `_on_toggle_camera`) or after detect. |
| `SupportClasses/HardwareConfig.py` | New `camera_calibrations: dict[str, dict]` field — keyed by stable device identity (DirectShow path / `toupcam:<id>` / `simulated:<mode>`), each value `{"um_per_px": float, "name": str}`; `to_dict`/`from_dict` legacy-tolerant (missing→{}, bare-float coerced, malformed dropped). |
| `gui/widgets/camera_identity.py` | **New** — Windows DirectShow device enumeration (`FriendlyName` + `DevicePath` via pygrabber/comtypes). `enumerate_directshow_cameras`, `short_port_tag`, `label_for`, `identity_for_source`. Degrades to `[]`/index-fallback off-Windows or without deps. |
| `gui/widgets/camera_manager.py` (identity) | Caches the DirectShow enumeration on `detect_cameras`; new `camera_identity(cam_idx)` resolves a slot's `(identity_key, friendly_name)` from its assigned source. |
| `gui/widgets/camera_widget.py` | OpenCV cameras labeled with the friendly name + USB port tag (e.g. `"Teslong Camera (port 6&29d1719c&2)"`) instead of `"CV2: Camera N"`; `detect_cameras` + `start_with_index` open via `CAP_DSHOW` so OpenCV indices line up with the DirectShow enumeration used for identity. |

## Implementation Steps

- [x] `CameraManager`: add `_um_per_px_set` list (init `[False]*max`), set `True` in
      `set_um_per_px`, add `is_um_per_px_calibrated`.
- [x] `calibration.py::_needle_loc_camera_info`: read µm/px from the manager (calibrated-gated).
- [x] `calibration.py` plate-location: same manager-gated read.
- [x] `hardware_setup.py`: per-slot Start/Stop button + live preview holder in the
      assignment row build loop; lists `_live_cam_start_btns` / `_live_cam_preview_holders`
      / `_live_cam_previews`.
- [x] `hardware_setup.py`: `_on_toggle_camera`, `_on_live_cam_source_changed`,
      `_ensure_camera_previews`, `_refresh_camera_preview_state`.
- [x] `hardware_setup.py`: `set_camera_manager` wires `camera_started`/`camera_stopped`,
      builds previews, refreshes state; `_on_detect_live_cameras` refreshes state.
- [x] **Persistence (identity-keyed)** — new `gui/widgets/camera_identity.py`
      (DirectShow name + device path); `HardwareConfig.camera_calibrations` dict +
      serialization; `CameraManager.camera_identity` + cached enumeration; `CAP_DSHOW`
      + friendly labels in `camera_widget`; `set_calibrated_um_per_px` stores by identity
      + saves; `_restore_calibration_for_slot`/`_restore_all_calibrations` re-apply on
      source assignment / detect. So a calibration follows the physical camera+port
      across slot reassignments and restarts.

## Testing Notes

- `tests/test_v75x_camera_cal_liveview.py`: `CameraManager` calibration-flag semantics
  (default False, True after `set_um_per_px`, per-slot independence, OOB-safe);
  `HardwareConfig.camera_calibrations` round-trip / legacy / coercion; and
  `camera_identity` (port tag, label, **two identical cameras → distinct identities**,
  fallbacks). E2E (manual smoke): a calibration stored under camera A's identity is
  re-applied to whatever slot A is later assigned to, and is *not* applied to camera B.
- Manual (hardware): assign + Start a needle camera on Hardware Setup → Cameras, confirm
  the live preview shows the feed; calibrate µm/px; switch to Calibration → Needle
  Location, confirm the offset preview computes instead of "cannot compute".
- Manual (persistence): calibrate a needle camera, **restart the app**, confirm the
  Needle Cameras Setup card shows the calibrated value (not "— (not calibrated)") and the
  needle-zero offset preview still computes without recalibrating.

## Issues & Decisions

- **1.67 default trap** — chose an explicit `is_um_per_px_calibrated` flag over changing
  the `get_um_per_px` default sentinel (the 1.67 default is relied on by
  `live_target_picker` / `target_overlay_camera_view`).
- **No reparenting** — live previews use `CameraFeedView` (subscribes to
  `frame_captured`), so the same camera can be previewed here and mounted in the
  Calibration page's needle-location feeds simultaneously without stealing the widget.
- **Source wiring gap** — the assignment source combos were never connected to
  `mgr.set_source`; wired now so Start opens the *selected* source per slot.
- **Identity-keyed persistence (supersedes an initial slot-indexed draft)** — keyed by a
  stable **device identity** (DirectShow `DevicePath` = model + USB port) rather than slot
  index. Confirmed against real hardware: two physically-identical "Teslong Camera" units
  share name + VID/PID but get distinct device paths (`6&29d1719c&2` vs `7&b643b4c&0`), so
  the path disambiguates *which unit on which port* while still carrying the *type*. A
  calibration therefore follows the physical camera across slot reassignments and
  restarts. Moving a camera to a different USB port → new identity → recalibrate (by
  design). Index alignment relies on opening OpenCV with `CAP_DSHOW` so its indices match
  the DirectShow enumeration order.
- **Graceful degradation** — `camera_identity.py` returns `[]` (and `identity_for_source`
  falls back to `opencv:<idx>`) off-Windows or without pygrabber/comtypes, so the feature
  silently reverts to index-based identity rather than breaking detection.
- **Restore timing** — identity needs the slot's source assigned + cameras detected, so
  restore fires from `_on_live_cam_source_changed` / `_on_toggle_camera` (per slot) and
  `_restore_all_calibrations` after detect — not at bare `set_config` time (no source yet).
- **Microscope mirror is harmless** — `set_calibrated_um_per_px` also fires for the
  microscope (via the objective card's `um_per_px_committed`), so its current value is
  stored under the microscope camera's identity too. The authoritative microscope restore
  is still objectives.json (re-applied per-objective by the calibration page on load), so
  the mirror is redundant, not conflicting.
- **Save trigger** — `set_calibrated_um_per_px` calls `_on_config_changed` (guarded by
  `_restoring`) so *every* calibration entry point (needle dialog, objective card,
  calibration-page context-panel pixel cal) persists uniformly. `_rebuild_config` mutates
  `self._config` in place and never touches `camera_calibrations`, so the dict survives
  the rebuild. The app's save path is re-entrancy-guarded, so the objective path's extra
  save is benign.

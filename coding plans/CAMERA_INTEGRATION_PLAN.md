# BUC3D-1000C / ToupTek Camera Integration Plan

**Version:** v7.3-camera  
**Target:** Bestscope BUC3D-1000C (ToupTek C3CMOS10000KPA)  
**Status:** Camera confirmed working via ctypes + ToupView system DLL  

---

## Confirmed Hardware Details

| Property | Value |
|----------|-------|
| Camera   | C3CMOS10000KPA (Bestscope BUC3D-1000C) |
| USB ID   | VID_0547 PID_1377 |
| Max Res  | 3664 x 2748 |
| Interface | USB 3.0 |
| SDK      | ToupTek toupcam.dll via ToupView install |
| DLL Path | C:\Program Files\ToupTek\ToupView\x64\toupcam.dll |
| DLL Size | 29,144,576 bytes (29 MB) |
| API      | 17/17 functions confirmed working |

---

## Architecture

### New File: `gui/widgets/toupcam_backend.py`

A self-contained ToupTek camera wrapper that:
- Finds toupcam.dll (ToupView install → toupcam-master/x64/ → system PATH)
- Defines ctypes function signatures for the 10 functions we need
- Provides `ToupCamBackend` class with OpenCV-like API:
  - `enumerate()` → list of `{id, name}` dicts (class method)
  - `open(device_id)` → bool
  - `isOpened()` → bool
  - `read()` → (bool, numpy BGR frame)
  - `release()` → None
  - `set_resolution_index(i)` → None
  - `get_resolution()` → (w, h)
  - `set_auto_exposure(bool)` → None
- Thread-safe: callback writes to a locked buffer, `read()` copies it
- No dependency on NMGRL repo (uses system DLL directly)

### Patched File: `gui/widgets/camera_widget.py`

Changes:
1. Add `TOUPCAM_AVAILABLE` flag alongside `CV2_AVAILABLE`
2. New `CAMERA_AVAILABLE = CV2_AVAILABLE or TOUPCAM_AVAILABLE`
3. Modify `_populate_cameras()` → show placeholder if ANY backend available
4. Modify `refresh_cameras()` → detect OpenCV AND ToupCam cameras
   - Combo items store tuples: `("opencv", int_index)` or `("toupcam", str_device_id)`
5. Modify `start()` / `start_with_index()` → route to correct backend
6. Add `_start_toupcam(device_id)` method
7. Modify `_grab_frame()` → read from `self._capture` (OpenCV) or `self._toupcam` (ToupCam)
8. Modify `stop()` → release correct backend
9. Modify `take_snapshot()` → use correct backend
10. Widget now works even if ONLY ToupCam is available (no OpenCV needed)

### Patched File: `gui/pages/calibration.py`

Changes:
1. Import `CAMERA_AVAILABLE` (new flag) alongside `CV2_AVAILABLE`
2. Camera grid condition: `if CAMERA_AVAILABLE and CameraWidget is not None:`
3. The detect-cameras button handler checks both backends

---

## Session Breakdown

### Session 1 (this session): All three files
- `gui/widgets/toupcam_backend.py` — new file (delivered directly)
- `patches/v726_camera/patch_camera_integration.py` — patches camera_widget.py + calibration.py

### Session 2 (future, optional): Resolution & Exposure Controls
- Add resolution dropdown to CameraWidget context panel
- Add exposure time slider
- Add auto-exposure toggle
- Add white balance controls

### Session 3: Autocalibration (v7.3.0)
- See: `coding plans/Update plans/MEBP_v730_AUTOCALIBRATION_UPDATE.md`
- Well center auto-detection via HoughCircles
- Needle auto-detection & focus assist
- Parallel detection processing via QThread
- Camera config with micron/pixel calibration

---

## Key Design Decisions

1. **System DLL over NMGRL repo**: The NMGRL repo has broken Git LFS stubs.
   ToupView installs a working 29MB DLL. We find it at runtime.
   
2. **Fallback chain**: ToupView x64 → ToupView x86 → toupcam-master/x64/ → PATH

3. **combo data format**: `("backend_type", identifier)` tuples ensure clean routing

4. **Frame buffer threading**: ToupTek uses a callback from its own thread.
   We copy frames into a locked buffer; the QTimer-driven `_grab_frame()` 
   reads from that buffer on the GUI thread. No cross-thread Qt operations.

5. **Resolution default**: Use index 2+ (around 916×686) for live preview 
   instead of full 3664×2748. Full res only for snapshots.

# MEBP v7.3.3 → v7.3.4 Update Plan

## Objective

Move the µm/px (microns-per-pixel) calibration workflow into the per-camera settings panel on the Calibration page. Each camera slot gets an **objective selector** and a live µm/px readout (theoretical from sensor spec, calibrated from `objectives.json`). Running the empirical stage-motion calibration dialog saves the result to a persistent `objectives.json` file keyed by camera model + objective name, and also writes back into the active `HardwareConfig.camera_config`.

---

## Background / Design

The system already knows the sensor pixel size from the manufacturer (`CameraSpec.sensor_pixel_size_um`). The theoretical µm/px is:

```
effective_px_size = sensor_pixel_size × (max_res_width / active_res_width)
theoretical_um_per_px = effective_px_size / objective_nominal_magnification
```

The *real* objective magnification differs slightly from nominal. After an empirical stage-motion calibration we can back-calculate:

```
actual_magnification = effective_px_size / measured_um_per_px
```

These per-camera, per-objective calibrations are stored in `config/hardware/objectives.json` so they persist across sessions and hardware config changes.

---

## Files Modified

| File | Change |
|------|--------|
| `config/hardware/objectives.json` | **New** — standard objective list + per-camera calibration storage |
| `SupportClasses/ObjectiveCalibration.py` | **New** — load/save manager for objectives.json |
| `SupportClasses/MosaicCalibrator.py` | **New** — SVD Procrustes manual point-set registration |
| `SupportClasses/XYStage.py` | **Bug fix BF-2** — consume `R\r` ack in `move_stage_at_velocity()` |
| `gui/app.py` | **Bug fix BF-3** — safety status state guard; **Bug fix BF-5** — startup calibration push order |
| `gui/pages/calibration.py` | Extend each camera settings panel: add objective selector, theoretical/calibrated µm/px labels, FOV readout, Calibrate button; **Bug fix BF-4 + BF-5** — MosaicCalibrator integration, AffineCalibration bridge, auto-save |
| `gui/pages/dashboard.py` | **Bug fix BF-3** — connection/safety label state-change guards |

---

## Implementation Steps

- [x] Create `coding plans/Update plans/MEBP_v733_to_v734_UPDATE.md` (this file)
- [ ] Create `config/hardware/objectives.json` with standard objectives (1x–100x)
- [ ] Create `SupportClasses/ObjectiveCalibration.py`
  - `ObjectiveCalibrationStore` class: load, save, get_calibration, set_calibration
  - Lazy singleton accessor `get_store()`
- [ ] Extend camera settings panel loop in `calibration.py` (`_build_context_panel`)
  - Per-camera: objective QComboBox, theoretical µm/px label, calibrated µm/px label, FOV label, "Calibrate µm/px" button
  - Track in `_ctx_cam_obj_combos`, `_ctx_cam_theo_labels`, `_ctx_cam_cal_labels`, `_ctx_cam_fov_labels`
- [ ] Add handler methods to `calibration.py`
  - `_get_camera_spec_for_idx(idx)` — returns CameraSpec from hardware config
  - `_get_camera_model_for_idx(idx)` — returns model name string for objectives.json key
  - `_ctx_on_objective_changed(idx)` — populates labels, applies to CameraManager
  - `_ctx_update_um_px_display(idx)` — recomputes/reloads labels
  - `_ctx_calibrate_um_per_px(idx)` — launches PixelCalibrationDialog, on accept saves to objectives.json + hardware config
- [ ] In `set_hardware_config()` — call `_ctx_update_um_px_display(idx)` for all camera indices to reflect loaded camera spec and objective

---

## Status Tracking

| Step | Status |
|------|--------|
| Plan file | `[x]` done |
| objectives.json | `[x]` done |
| ObjectiveCalibration.py | `[x]` done |
| calibration.py — settings panel UI | `[x]` done |
| calibration.py — handler methods | `[x]` done |
| calibration.py — set_hardware_config hook | `[x]` done |
| BF-1: XY stage velocity stop fix | `[x]` done |
| BF-2: XY progressive lag — R-ack accumulation in VS | `[x]` done |
| BF-3: Per-tick setStyleSheet guards (app.py, dashboard.py) | `[x]` done |
| BF-4: MosaicCalibrator.py — create missing module | `[x]` done |
| BF-5: Calibration persistence + jog page startup race | `[x]` done |

---

## Testing Notes

1. Open Calibration page → expand a camera's Settings panel
2. Verify objective dropdown shows 1x–100x
3. With a BUC3D-1000C camera spec loaded in hardware config, verify theoretical µm/px is computed correctly for each objective
4. Run "Calibrate µm/px" — complete stage-motion calibration
5. Verify `config/hardware/objectives.json` is updated with measured value
6. Re-open settings panel — calibrated µm/px should appear in green
7. Switch objectives — theoretical and calibrated (if available) should update live
8. Reload app — calibrated value should persist (loaded from objectives.json)

---

## Bug Fixes

### BF-1 — XY stage does not stop when joystick returns to deadzone

**File:** `SupportClasses/StageController.py` — `XYJogHandler._jog_loop`

**Symptom:** After releasing the Xbox controller left stick, the XY stage continues moving at its last commanded velocity until the next joystick input.

**Root cause:** The ProScan XY stage operates in continuous velocity mode — once sent a velocity it keeps moving until explicitly commanded to zero (`VS 0,0`). The `_jog_loop` detected the moving→stopped transition (`_was_moving=True`, `is_moving=False`) and logged it, but immediately hit `continue` before ever calling `move_stage_at_velocity(0, 0)`.

**Fix:** In the `elif not is_moving and self._was_moving:` branch (fires exactly once per stop event, while `_was_moving` is still `True`), call `self.stage.move_stage_at_velocity(0, 0)` before `_was_moving` is updated and the loop returns to sleep.

**Note:** `ZPJogHandler` is unaffected — it uses segmented relative moves; the motor decelerates and stops naturally when no further moves are dispatched.

**Status:** `[x]` done

---

### BF-2 — Progressive XY position display lag during Xbox jogging

**File:** `SupportClasses/XYStage.py` — `move_stage_at_velocity()`

**Symptom:** For the first ~10 seconds of jogging the XY stage via Xbox controller, position display is fully responsive. After that, position updates progressively lag and eventually stop updating entirely. ZP stage position is unaffected throughout. Camera not required to reproduce.

**Root cause:** Every `VS,vx,vy` velocity command sent to the ProScan II/III stage generates a `R\r` acknowledgment byte. `move_stage_at_velocity()` wrote the command under `_serial_lock` but never read back the `R\r` response. At 10 velocity commands/sec during Xbox jogging, ~20 bytes/sec accumulated in the RX buffer. The `PositionPoller` background thread, which normally reads `P` position responses, then consumed these accumulated `R` bytes instead of actual position data — causing the position cache to stop updating and the display to freeze. This is the identical bug that was previously fixed in `move_stage_relative()`.

**Fix:** Inside `_serial_lock` in `move_stage_at_velocity()`, call `_read_response_cr(self.spo, timeout=0.05)` immediately after the write to consume the `R\r` ack — same pattern as `move_stage_relative()`.

**Confirmed via:** `config/controllers/proscan_ii.json` and `proscan_iii.json` both define `"set_velocity": {"cmd": "VS,{vx},{vy}", "response": "R"}`.

**Status:** `[x]` done

---

### BF-3 — Per-tick setStyleSheet calls causing CPU overhead

**Files:** `gui/app.py` — `_update_status()`, `gui/pages/dashboard.py` — `update_data()`

**Symptom:** `_update_status()` (called every 300ms) and dashboard `update_data()` (called every 50ms) both called `setStyleSheet()` on connection-status and safety labels on every tick regardless of whether the state had changed. Qt re-parses and re-evaluates CSS on every `setStyleSheet()` call, causing unnecessary CPU overhead.

**Fix:** Added state-change guards using `getattr(self, '_last_state', None) != current_state` pattern before each `setStyleSheet()` call. Stylesheet is only updated when the underlying state actually changes (XY connect/disconnect, ZP connect/disconnect, safety on/off).

**Also added:** A timing diagnostic to `_tick()` in `app.py` that logs a debug warning if `_update_status()` takes longer than 20ms, to aid future performance debugging.

**Status:** `[x]` done

---

### BF-4 — Well plate training fails with `ModuleNotFoundError: No module named 'SupportClasses.MosaicCalibrator'`

**File:** `gui/pages/calibration.py` — `_generate_well_positions()`, new `SupportClasses/MosaicCalibrator.py`

**Symptom:** When the user manually teaches 2+ well positions (e.g., A1 and A6) on the Calibration page and presses the training button, a `ModuleNotFoundError` is raised and the status label shows `Fit failed: No module named 'SupportClasses.MosaicCalibrator'`. The fallback 2-point alignment also fails because `_corner_well` (defaulting to H12) is not among the taught wells.

**Root cause:** `_generate_well_positions()` attempted to import `SupportClasses.MosaicCalibrator.MosaicCalibrator` which had never been created.

**Fix:** Created `SupportClasses/MosaicCalibrator.py` with the `MosaicCalibrator` class implementing SVD Procrustes similarity-transform registration:
- `add_point(pred_x, pred_y, meas_x, meas_y)` — accumulate point pairs
- `solve()` — compute scale + rotation + translation via SVD; requires ≥2 pairs
- `correct_positions(predicted_dict)` — apply transform to all predicted positions
- `rms_error_um` attribute — RMS residual after solve

Works with as few as 2 matched point pairs (unique similarity transform). With 3+ points becomes a least-squares fit.

**Status:** `[x]` done

---

### BF-5 — Manual well plate calibration not persistent across restarts

**Files:** `gui/pages/calibration.py` — `_generate_well_positions()`, `gui/app.py` — startup wiring

**Symptom:** After successfully training the well plate manually (MosaicCalibrator path), the calibrated positions correctly appear in both the calibration page plate view and the jog page well navigator. However, after restarting the application, the jog page reverts to approximate (geometry-predicted, yellow) well positions rather than the calibrated (green) ones.

**Root cause — save path:** After `MosaicCalibrator.solve()`, the code updated `_calibrated_positions` but never populated `self._three_well_calibration` (the `AffineCalibration` object). `_save_calibration()` only writes `mosaic_affine` when `_three_well_calibration` is non-None and non-identity, so the calibration was never persisted.

**Root cause — load path (startup race):** `_load_calibration()` is called from `_setup_ui()` during `CalibrationPage.__init__()`, which runs before `app.py` wires the `calibration_data_changed` signal to `jog_page.set_calibration_data`. When the signal fires during `__init__`, there are no listeners yet — the calibrated positions are emitted into the void. Subsequently, `app.py` calls `jog_page.load_startup_plate()` which sets approximate positions, permanently overwriting any calibration data the jog page might have received.

**Fix — save path:** After `mcal.solve()` succeeds, convert the `MosaicCalibrator` result to an `AffineCalibration` (mathematically equivalent with `center_um=(0,0)`) and store it in `self._three_well_calibration`. The existing `_save_calibration()` path then serializes it correctly as `mosaic_affine`. Also call `_save_calibration()` automatically on successful training — no manual Save required.

**Fix — load path:** In `app.py`, reorder startup so `load_startup_plate()` runs first (establishing the approximate baseline), then the signal is connected, then `set_calibration_data()` is immediately called once with the already-loaded calibration data from `get_calibration_data()`. This eliminates the race between `__init__` signal emission and signal connection.

**Status:** `[x]` done

---

## Issues & Decisions

- **Multi-camera config**: Currently `HardwareConfig.camera_config` is a single config (camera 0). For cameras 1–2, the objective UI reads from the same camera spec but tracks objective selection and calibration independently via `objectives.json`. Future work can add per-slot camera configs.
- **Camera model identification**: Uses `hardware_config.camera_config.camera_spec.name` for the objectives.json key. If no spec is loaded, defaults to `"unknown"` — calibrations will still be saved and retrieved under that key.
- **Override vs computed**: After calibration, the measured µm/px is stored in `objectives.json` only. The `hardware_config.camera_config.micron_per_pixel_override` is set to the measured value when the user accepts the calibration so downstream code picks it up immediately. It resets to `None` when the user switches objectives (reverting to computed from spec + magnification in objectives.json).
- **MosaicCalibrator vs AffineCalibration**: Both represent similarity transforms but use different parameterizations. `MosaicCalibrator` stores an explicit SVD rotation matrix + scale + translation. `AffineCalibration` stores rotation_deg + scale + translation + center. They are mathematically equivalent when `center_um=(0,0)`. The conversion is: `rotation_deg = degrees(arctan2(R[1,0], R[0,0]))`, same scale and translation. This conversion is done in `_generate_well_positions()` to bridge the two representations.
- **Well position coordinate system**: All calibrated positions are stored in absolute stage µm (origin at stage mechanical limit). The zero reference (`controller.zero_position`) is applied only when displaying positions to the user. This means calibrated positions remain valid even if the user moves the zero reference.

---

## BF-6 — Xbox Trigger Creep on Connection

**Files:** `SupportClasses/XboxController.py`, `SupportClasses/StageController.py`

**Symptom:** As soon as the Xbox controller connects, any pump axis assigned to an Xbox trigger (axis 4 or 5) begins creeping continuously before any trigger is pressed. On disconnect, the creep accelerates.

**Root cause (three interacting issues):**

1. **Heartbeat transient** — `xbox_polling_worker` calls `pygame.joystick.quit()/init()` every 3 seconds to detect disconnects. During reinit, Xbox XInput triggers transiently report `0.0` instead of their rest value of `-1.0`. With `offset = -1.0` applied: `(0.0 + 1.0) / 2.0 = 0.5` passes the 5% deadzone — a dispatch fires every 3 s.

2. **Unreliable rest value sampling** — The original multi-sample approach detected rest values at startup. Over Bluetooth all 5 samples could arrive during the transient window, computing an incorrect offset and span.

3. **Velocity not zeroed on disconnect** — On disconnect the jog handlers retained their last non-zero velocity, so motion continued (or accelerated).

**Fix:**
- **Platform-based trigger offsets**: Windows/Linux XInput triggers always rest at `-1.0`; hardcode `{4: -1.0, 5: -1.0}`. macOS triggers are already `0..1` — no normalization.
- **`_accum_suppress_until`**: After each heartbeat reinit, flush the axis accumulator and set `_accum_suppress_until = current_time + avg_interval + 0.15`. The accumulation section checks this timestamp and skips for the 250ms settle window.
- **Velocity zeroing on disconnect**: `XboxQueuePoller._poll_messages` now explicitly zeros `zp_jog.vel_z/vel_p1/vel_p2/vel_p3` and `xy_jog.vel_x/vel_y` under their respective locks on any disconnect/error event.

**Status:** `[x]` done

---

## BF-7 — Xbox Debug Mode

**Files:** `SupportClasses/XboxController.py`, `SupportClasses/StageController.py`, `gui/pages/dashboard.py`, `gui/pages/settings_page.py`

**Symptom:** Diagnosing Xbox issues required temporary print statements with no toggle to enable/disable verbose diagnostics.

**Fix:** Added `debug_mode: bool` flag threaded from Settings → Dashboard → `StageController.connect_xbox()` → `XboxQueuePoller` → `xbox_polling_worker` subprocess.

- **Settings page**: "Debug Mode" `QCheckBox` in the Xbox card. Persisted to `settings.xbox.debug_mode`. Tooltip notes reconnect is required to apply.
- **Gated log output** (only when `debug_mode=True`): startup trigger diagnostic (rest values, offsets, platform), periodic TRIG DIAG every 2s, per-dispatch axis/command/value log, ZPJog velocity change log (downgraded to `logger.debug` regardless).
- Always-on: connect/found messages.

**Status:** `[x]` done

---

## BF-8 — Calibration Page Click-to-Move

**File:** `gui/pages/calibration.py`

**Symptom:** No direct click-to-move interaction on the live camera feed in the calibration page. Users had to use jog controls to centre features manually.

**Fix:**
- Added `Click→Move: OFF / ON` checkable `QPushButton` in the Camera Controls panel (next to the Layout dropdown). Turns green when active.
- During feed view creation, each `CameraFeedView.clicked` signal is connected: `fv.clicked.connect(lambda px, py, idx=i: self._on_feed_clicked(idx, px, py))`.
- `_on_feed_clicked`: gets `img_w, img_h` from `feed_views[cam_idx].image_size`, calls `camera_manager.pixel_to_stage_offset(cam_idx, px_x, px_y, img_w, img_h)` → `(dx_um, dy_um)`, then `controller.move_xy_relative_um(dx_um, dy_um)`.
- Mode is off by default and not persisted.

**Status:** `[x]` done

---

## BF-9 — Safe Navigation Z-Wait Does Not Block XY Motion

**Files:** `SupportClasses/ZPStage.py`, `SupportClasses/StageController.py`

**Symptom:** When navigating to a calibration well, the Z stage began retracting to safe height but the XY stage started moving immediately without waiting for Z to arrive.

**Root cause (two interacting issues):**

1. **M114 returns commanded (planned) position** — After `G0 Z{safe_z}` is queued in the Marlin planner, an immediate `M114` query can return the target position before the stage has physically moved. `wait_for_z_arrival` saw "arrived" instantly.

2. **Serial race between `wait_for_z_arrival` and `PositionPoller`** — `send_data("M114")` and `receive_data()` are separate `_serial_lock` acquisitions in `ZPStage`. The `PositionPoller` background thread (polling every 300ms) could acquire the lock between them, consuming the M114 response. `receive_data()` returned empty; `_parse_position` failed to update `z_pos`; `get_current_position()` returned the stale `ZPStage.z_pos` instance variable (`0.0` initial). If that stale value satisfied the ≤0.1mm tolerance, `wait_for_z_arrival` returned `True` immediately.

**Fix:**

`ZPStage.flush_moves(timeout_s=15.0)`:
- Sends `M400` (Marlin "Wait for Moves to Finish").
- Reads `readline()` under `_serial_lock` (one line at a time) until a bare `"ok"` is received.
- M400's `"ok"` only arrives after all buffered moves have physically completed — unambiguous unlike M114.
- Simulation mode returns `True` immediately (`self.simulate` check).

`PositionPoller.suspend()` / `resume()`:
- `_suspended` flag. When `True`, `_poll_loop` skips all hardware queries and sleeps 50ms.
- Prevents the poller from racing on the ZP serial port during `safe_travel_to`.

`StageController.safe_travel_to()` rewrite:
- Wraps entire 3-step sequence in `_pos_poller.suspend()` / `finally: _pos_poller.resume()`.
- Z steps (retract + descend) use `zp_stage.flush_moves()` when available; fallback to `wait_for_z_arrival` for simulator.
- XY step retains `wait_for_xy_arrival` (Prior ProScan responds reliably to position queries; no M400 equivalent).

**Status:** `[x]` done

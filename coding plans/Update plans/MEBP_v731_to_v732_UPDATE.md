# MEBP v7.3.1 → v7.3.2 — Quality of Life Upgrades

**Objective:** Improve daily workflow with camera config persistence, enhanced jog controls, axis flip settings, calibration page restructuring, and live camera overlay on the print monitor.

**Branch:** `Version-7.3.2`
**Base:** `Version-7.3.1`
**Status:** Complete (Features 1-9 implemented; 5c.1-2 deferred)
**Date:** 2026-03-12

---

## Feature Summary

| # | Feature | Scope |
|---|---------|-------|
| 1 | Camera config auto-load/save | HardwareConfig, calibration.py, hardware_setup.py, app.py |
| 2 | Jog page well plate on startup (pre-calibration approx) | jog_control.py, jog_well_plate.py, WellPlate.py |
| 3 | Flip positive axis checkboxes (Z + each pump) | settings_page.py, Settings.py, StageController.py, ZPStage.py |
| 4 | Custom jog step sizes + absolute goto | jog_control.py |
| 5 | Calibration page restructure | calibration.py |
| 6 | Print monitor camera overlay on XY view | print_monitor.py, camera_widget.py |
| 7 | Settings page manual zero calibration with jog controls | settings_page.py, jog_button_array.py |
| 8 | Xbox controller stick zero calibration | XboxController.py, StageController.py, settings_page.py, dashboard.py, main.py |
| 9 | Safe travel Z-wait standardization | StageController.py, calibration.py |

---

## 1. Camera Config Auto-Load/Save on Setup

### Objective
Camera configuration (`CameraConfig`) is already part of `HardwareConfig` (added v7.3.0) but is only managed within the calibration page. It should be loaded from `settings.json` on startup and auto-saved when values change, matching the behavior of the rest of `HardwareConfig`.

### Current State
- `CameraConfig` is a dataclass in `SupportClasses/HardwareConfig.py` (line 81)
  - Fields: `camera_spec`, `objective_magnification`, `active_resolution`, `micron_per_pixel_override`, `camera_to_needle_offset_um`
  - Has `to_dict()` / `from_dict()` serialization
- `HardwareConfig.camera_config` field exists (line 341) and is serialized via `to_dict()`
- `HardwareConfig` is saved to `settings.json` under `"hardware_config"` key
- Camera settings are currently only set from calibration page context panel sliders/combos
- Changes to camera config in calibration are NOT auto-saved

### Implementation Steps

- [x] **1.1** Ensure `HardwareConfig.from_dict()` properly deserializes `camera_config` — VERIFIED: `from_dict()` calls `CameraConfig.from_dict()` at line 691
- [x] **1.2** Camera settings are configured in `hardware_setup.py` (Section 8: Camera Configuration). Changes trigger `_on_camera_changed()` → `_on_config_changed()` → `config_changed.emit()` — already propagates automatically
- [x] **1.3** `app.py::_on_hardware_config_changed()` calls `_save_hardware_config()` which serializes the entire `HardwareConfig` including `camera_config` via `to_dict()` — already works
- [x] **1.4** On startup, `_restore_hardware_config()` loads from settings.json → `HardwareConfig.from_dict()` → `_apply_config_to_ui()` restores camera combo/resolution/magnification/override — already works
- [x] **1.5** Calibration page reads camera config via `_get_camera_config()` from the propagated `_hardware_config` — already works
- **NOTE**: Feature 1 was already implemented in v7.3.0. No changes needed.

### Files Modified
| File | Change |
|------|--------|
| `gui/pages/calibration.py` | Emit config change on camera param edit; populate from loaded config |
| `gui/app.py` | Ensure camera_config persists in hardware_config save path |
| `SupportClasses/HardwareConfig.py` | Verify from_dict camera_config path (likely no change) |

---

## 2. Jog Page Well Plate on Startup (Pre-Calibration Approximation)

### Objective
Display a well plate on the jog page immediately at app startup using geometry-predicted positions (before calibration), so the user can fast-travel to approximate well locations. After calibration, update with precise positions.

### Current State
- `WellPlateNavigator` widget already exists on the jog page (v7.3.1, line 304–322)
- Currently only populated after `set_calibration_data()` is called from calibration page
- `WellPlate.get_well_position()` provides geometry-based positions from plate specs
- Predicted positions are computed in calibration page using plate center assumption

### Implementation Steps

- [x] **2.1** `load_startup_plate(settings)` on JogControlPage reads plate format from `calibration.plate_format` → `workspace.plate_format` → default 96, creates WellPlate, computes positions via `get_all_positions_from_plate_center(65000, 42500)` (stage center)
- [x] **2.2** `set_approximate_positions()` on WellPlateNavigator stores positions and marks wells as approximate
- [x] **2.3** Yellow (#f9e2af) fill + border for approximate wells; green for calibrated; tooltip shows "(approx)" suffix
- [x] **2.4** `set_well_positions()` clears `_approximate_wells` — calibrated data replaces approximate automatically
- [x] **2.5** Plate format already persisted in `calibration.plate_format` and `workspace.plate_format` by Settings.py defaults
- [x] **2.6** Uses existing plate format from settings (set in calibration page); no separate selector needed on jog page

### Files Modified
| File | Change |
|------|--------|
| `gui/pages/jog_control.py` | Add plate format selector; generate approx positions on startup |
| `gui/widgets/jog_well_plate.py` | Visual distinction for approx vs calibrated positions |
| `SupportClasses/Settings.py` | Persist last plate format (if not already) |

---

## 3. Flip Positive Axis Checkboxes (Z + Each Pump)

### Objective
Add a "Flip Positive Direction" checkbox for Z and each pump (P1, P2, P3) in the Settings page. This allows users to correct axis direction when hardware is wired with reversed polarity without changing firmware.

### Current State
- Settings page has safety limits for Z and pumps (line 548–616)
- No axis direction/flip settings exist
- `StageController` sends Z/pump commands through `ZPStageManager` or `ZPStageSimulator`
- Commands use G-code: `G1 Z10 F60` (Z), `G1 E5 F10` (pump via `T0`/`T1`/`T2` tool select)

### Implementation Steps

- [x] **3.1** Add `axis_flip` section to `Settings`: `{"Z": false, "P1": false, "P2": false, "P3": false}`
- [x] **3.2** In `settings_page.py`, add a new "Axis Direction" card with 4 checkboxes: Flip Z, Flip P1, Flip P2, Flip P3
- [x] **3.3** In `_apply_settings()`, save flip state to settings
- [x] **3.4** In `_load_from_controller()`, load flip state
- [x] **3.5** In `StageController`, apply flip multiplier (-1 or +1) when sending Z/pump moves
  - `move_z_relative(dist)` → `move_z_relative(dist * z_flip)`
  - `move_pump_relative(pump, dist)` → apply pump-specific flip
- [x] **3.6** Load flip settings in `main.py` and apply to controller on startup

### Files Modified
| File | Change |
|------|--------|
| `gui/pages/settings_page.py` | New "Axis Direction" card with 4 flip checkboxes |
| `SupportClasses/StageController.py` | Flip multiplier properties + apply in move methods |
| `main.py` | Load flip settings on startup |

---

## 4. Custom Jog Step Sizes + Absolute Goto

### Objective
Allow users to enter custom step sizes for XY, Z, and Pump jogging (not just preset combos). Add an "Absolute Go To" section where users can enter X, Y, Z coordinates and travel there (using safe travel by default).

### Current State
- XY steps: `[1, 5, 10, 50, 100, 500, 1000]` µm (combo box in context panel)
- Z steps: `[0.01, 0.05, 0.1, 0.5, 1.0, 5.0]` mm
- Pump steps: `[0.1, 0.5, 1.0, 5.0, 10.0, 50.0]` µL
- All use `QComboBox` with fixed options

### Implementation Steps

- [x] **4.1** Added "Custom…" entry to all three step size combos (XY, Z, Pump) — shows QDoubleSpinBox when selected
- [x] **4.2** Custom spinboxes appear below each combo when "Custom…" is selected: XY (0.1–50000 µm), Z (0.001–50 mm), Pump (0.001–100 mm/0.01–500 µL)
- [x] **4.3** Custom step sizes managed via spinbox values (session-persistent); `_refresh_pump_step_combo()` updated to include "Custom…" and adjust suffix/range for µL mode
- [x] **4.4** "Absolute Go To (zero-referenced)" card added to main content with X(µm), Y(µm), Z(mm) spinboxes, "Safe Travel" checkbox (default checked), "Go To" button
- [x] **4.5** `_absolute_goto()` uses `controller.safe_travel_to()` when checkbox is checked; falls back to direct `move_xy_absolute`/`move_z_absolute` when unchecked

### Files Modified
| File | Change |
|------|--------|
| `gui/pages/jog_control.py` | Editable step combos, absolute goto section |

---

## 5. Calibration Page Restructure

### Objective
Major layout restructure of the calibration page:
- **(a)** Move calibration steps from context panel (left box) to main content area
- **(b)** Allow user to choose how many camera feeds to display (1, 2, or 3)
- **(c)** Plate position view should match jog page well plate style; user can toggle between plate view and actual image overlay; show live needle position
- **(d)** Remove "Set Zero for Needle" (Step 1) — deprecate throughout codebase

### Current State
- Calibration steps (Zero Needle, Safe Z, Top Z, Auto-Calibrate, Z-Cal) are all in the context panel (left sidebar), starting at line 760
- Main content has: position readout + up to 3 camera feeds
- `_CalibrationPlateView` (QGraphicsView) shows plate in context panel
- Step 1 "Zero Needle" provides Set Zero / Go to Zero buttons

### 5a — Move Calibration Steps to Main Content

- [x] **5a.1** New `_build_wizard_steps(layout)` method creates all calibration step widgets in a "Calibration Wizard" card in main content. Layout: pos readout → camera feeds → plate view → wizard steps.
- [x] **5a.2** Removed all step widgets from `get_context_widget()` — only Camera Controls + Plate Config remain.
- [x] **5a.3** Context panel is now Camera Controls only (brightness, gamma, FPS, crosshair, detect) + Plate Configuration.

### 5b — Configurable Camera Feed Count

- [x] **5b.1** Camera count combo (1/2/3) added to camera card title row in main content.
- [x] **5b.2** `_on_cam_count_changed()` shows/hides cameras dynamically. Default: 1 camera shown.
- [ ] **5b.3** Persist camera count preference to settings (deferred — session-only for now).

### 5c — Enhanced Plate View

- [ ] **5c.1** Replace `_CalibrationPlateView` with `WellPlateNavigator` (deferred — existing view deeply integrated with needle tracking, well composites, calibration wells highlighting)
- [ ] **5c.2** View toggle between plate and image overlay (deferred)
- [x] **5c.3** Needle position updates via existing `_cal_plate_view.set_needle_xy()` (already works)
- [x] **5c.4** Real-time needle position polling via `_poll_cal_position()` (already works)

### 5d — Deprecate Needle Zero (Step 1)

- [x] **5d.1** Removed Step 1 "Zero Needle" section from calibration page UI (buttons, label, status)
- [x] **5d.2** Removed `_set_zero()` and `_goto_zero()` methods from calibration page. `zero_needle_pos` Xbox mapping + `_calibrate_zero` in StageController kept (used from jog page).
- [x] **5d.3** Steps renumbered: Safe Z (1A), Top Z (1B), Auto-Calibrate (1C), Z-Cal (2), Validate

### Files Modified
| File | Change |
|------|--------|
| `gui/pages/calibration.py` | Major restructure: steps to main content, configurable camera count, enhanced plate view, remove Step 1 |
| `gui/widgets/jog_well_plate.py` | Possible shared widget extraction for both pages |
| `INSTRUCTIONS.md` | Update calibration workflow docs |

---

## 6. Print Monitor Camera Overlay on XY View

### Objective
Add an option to overlay the live camera feed on the XY detail view in the print monitor, true to scale. Include zoom controls to fit-to-camera or zoom-out-to-full-plate.

### Current State
- `XYDetailView` is a custom `QWidget` with `paintEvent` that draws planned vs actual XY paths
- Camera feed exists as `CameraWidget` (OpenCV backend)
- No camera overlay on the print monitor
- The XY detail view has coordinate transforms: stage coords → widget pixel coords

### Implementation Steps

- [x] **6.1** "Camera" checkbox added to XY Detail legend bar. `_toggle_cam_overlay()` wires to `XYDetailView.set_camera_overlay()`.
- [x] **6.2** `_feed_camera_frame()` captures latest BGR frame from calibration page's active camera, converts to QImage RGB888, pushes to XY detail with `um_per_px` from `HardwareConfig.camera_config.micron_per_pixel`. Painted at 50% opacity centered on needle position.
- [x] **6.3** "Zoom Cam" button → `zoom_to_camera()` (fits camera FOV), "Zoom All" button → `zoom_to_plate()` (fits full trajectory).
- [x] **6.4** Frame updates tied to `on_status_update()` (~300ms / ~3 FPS) — matches position poll rate, not camera full rate.
- [x] **6.5** True-to-scale: camera frame sized by `(width * um_per_px / 1000, height * um_per_px / 1000)` mm, transformed through `_to_px()` coordinate system. Blue dashed border shows FOV boundary.

### Files Modified
| File | Change |
|------|--------|
| `gui/pages/print_monitor.py` | Camera overlay on XYDetailView, zoom controls |
| `gui/widgets/camera_widget.py` | Possibly add `get_latest_frame()` method for non-widget access |

---

## 7. Settings Page Manual Zero Calibration with Jog Controls

### Objective
Add a manual zero-position calibration section to the settings page context panel (left box) with embedded jog controls. This gives users a dedicated place to find the zero location for all axes (XY, Z, P1, P2, P3) using the same jog controls available on the jog page. This replaces the "Zero Needle" step that was removed from the calibration page.

### Current State
- Settings page context panel has: Quick Safety toggle, Simulation Mode status, Serial Ports, Apply/Reset buttons
- `JogButtonArray` widget exists (v7.3.1) — reusable jog pad with XY+Z buttons + step selectors, emits signals
- Settings page has access to `StageController` but no jog controls
- Zero position is set via `controller._calibrate_zero()` (sets all axes from current position)
- Individual axis zeroing not currently exposed in UI (only "Set Zero Here" on jog page which zeros all axes at once)

### Implementation Steps

- [x] **7.1** Add a "Manual Calibration" section to settings page context panel (left box):
  - Section label: "Manual Zero Calibration"
  - Position readout: X (µm), Y (µm), Z (mm), P1, P2, P3
  - Embedded `JogButtonArray(compact=True, show_pumps=True)` for XY+Z+pump jogging
- [x] **7.2** Add per-axis "Set Zero" buttons:
  - "Set XY Zero" — sets only X, Y zero from current position
  - "Set Z Zero" — sets only Z zero from current position
  - "Set P1 Zero" / "Set P2 Zero" / "Set P3 Zero" — sets individual pump zero
- [x] **7.3** Wire `JogButtonArray` signals to `StageController`:
  - `jog_xy_requested` → `controller.move_xy_relative_um(dx, dy)`
  - `jog_z_requested` → `controller.move_z_relative(dz)`
  - `jog_pump_requested` → `controller.move_pump_relative(pump, dist)`
  - `home_requested` → `controller.move_to_zero()`
- [x] **7.4** Pump jog buttons included via `JogButtonArray(show_pumps=True)` with mm step selector
- [x] **7.5** Position readout updates from `on_status_update()` timer (X/Y in µm, Z/P in mm)
- [x] **7.6** Zero-set buttons update `controller.zero_position` and save to settings.json
- [x] **7.7** Position readout shows live relative positions (current − zero reference)

### Files Modified
| File | Change |
|------|--------|
| `gui/pages/settings_page.py` | Add manual calibration section to context panel with JogButtonArray + pump buttons + per-axis zero buttons + position readout |
| `gui/widgets/jog_button_array.py` | May add pump jog signals if not already present |

---

## 8. Xbox Controller Stick Zero Calibration

### Objective
Fix stick drift by adding a calibration routine that samples stick center positions at rest and subtracts the measured offsets during polling. Users can run calibration from the Settings page.

### Current State
- Xbox sticks use a global deadzone of 0.2, symmetric around 0.0
- No per-axis center offset measurement exists
- Triggers have platform-aware normalization (Windows vs macOS rest values)
- Sticks with slight hardware drift send small nonzero values even when untouched

### Implementation Steps

- [x] **8.1** Add `calibrate_sticks()` function to `XboxController.py` — samples axes 0–3 for 2 seconds, returns center offsets
- [x] **8.2** Add `stick_offsets` parameter to `xbox_polling_worker()` — subtracts offsets from raw values before deadzone check
- [x] **8.3** Add `calibrate_xbox_sticks()` convenience method to `StageController`
- [x] **8.4** Pass stick offsets from `settings.json` to `connect_xbox()` in both GUI (`dashboard.py`) and headless (`main.py`) paths
- [x] **8.5** Add calibration UI to Settings page Xbox card: "Calibrate Sticks" button + "Clear" button + status label
- [x] **8.6** Save/load offsets in `settings.json` under `"xbox_stick_offsets"` section

### Files Modified
| File | Change |
|------|--------|
| `SupportClasses/XboxController.py` | New `calibrate_sticks()` function; `stick_offsets` param in worker |
| `SupportClasses/StageController.py` | Import `calibrate_sticks`; `stick_offsets` param in `connect_xbox()`; `calibrate_xbox_sticks()` method |
| `gui/pages/settings_page.py` | Calibrate/Clear buttons + status label in Xbox card |
| `gui/pages/dashboard.py` | Load stick offsets from settings and pass to `connect_xbox()` |
| `main.py` | Load stick offsets for headless mode; pass `settings` to `run_headless()` |

---

## 9. Safe Travel Z-Wait Standardization

### Objective
Ensure that all safe Z retract moves wait for Z to actually reach the safe height before starting XY moves, preventing needle crashes. Standardize on a single `safe_travel_to()` function with blocking waits.

### Implementation Steps

- [x] **9.1** Update `StageController.safe_travel_to()` to call `wait_for_z_arrival()` after Z retract, `wait_for_xy_arrival()` after XY move, and `wait_for_z_arrival()` after Z descent
- [x] **9.2** Refactor `calibration.py::_safe_navigate_to()` to delegate to `safe_travel_to()` instead of implementing its own 3-phase sequence
- [x] **9.3** Update existing `TestSafeTravelTo` tests to mock wait methods
- [x] **9.4** All tests pass (5/5)

### Files Modified
| File | Change |
|------|--------|
| `SupportClasses/StageController.py` | `safe_travel_to()` now blocks with `wait_for_z_arrival()` / `wait_for_xy_arrival()` |
| `gui/pages/calibration.py` | `_safe_navigate_to()` delegates to `safe_travel_to()` |
| `tests/test_v731_jog_navigation.py` | Mock `wait_for_z_arrival` / `wait_for_xy_arrival` in test fixtures |

---

## Implementation Order

Recommended build order (dependencies indicated):

1. **Feature 3** — Axis flip checkboxes (standalone, no dependencies)
2. **Feature 7** — Settings page manual calibration with jog controls (pairs naturally with Feature 3)
3. **Feature 8** — Xbox stick zero calibration
4. **Feature 9** — Safe travel Z-wait standardization
5. **Feature 1** — Camera config persistence (small scope, foundational for others)
6. **Feature 4** — Custom step sizes + absolute goto (standalone jog page enhancement)
7. **Feature 2** — Jog page well plate on startup (depends on plate format persistence from #1)
8. **Feature 5** — Calibration page restructure (largest scope, depends on #1 for camera config)
9. **Feature 6** — Print monitor camera overlay (depends on camera config from #1)

---

## Testing Notes

- [ ] Verify camera config round-trips through settings.json (save → restart → load)
- [ ] Verify jog page shows approximate well positions before calibration
- [ ] Verify axis flip correctly inverts Z and pump directions
- [ ] Verify custom step sizes persist across page switches
- [ ] Verify absolute goto uses safe travel by default
- [ ] Verify calibration page works with new layout (all steps accessible)
- [ ] Verify camera overlay is true-to-scale on XY view
- [ ] Verify needle zero removal doesn't break other functionality
- [ ] Verify settings page jog controls move all axes correctly
- [ ] Verify per-axis zero-set buttons update zero_position independently
- [ ] Verify zero positions persist to settings.json across restarts
- [ ] Verify Xbox stick calibration captures offsets and eliminates drift
- [ ] Verify stick calibration offsets persist and are loaded on next connect
- [ ] Verify safe_travel_to blocks until Z arrives before XY move (hardware or simulator)
- [ ] Run existing test suite (110 tests from v7.3.1 should still pass)

---

## Issues & Decisions

| # | Issue | Decision | Date |
|---|-------|----------|------|
| 1 | Where to store axis flip settings? | In `settings.json` under `"axis_flip"` section, not in `HardwareConfig` (axis flip is per-machine, not per-config) | 2026-03-12 |
| 2 | Should needle zero be fully removed or just hidden? | Remove from calibration page, keep the `_calibrate_zero` method in StageController and jog page "Set Zero" button. Xbox mapping `zero_needle_pos` remains available | 2026-03-12 |
| 3 | Shared well plate widget between jog and calibration? | Evaluate during implementation — may extract a common base widget or reuse `WellPlateNavigator` directly | 2026-03-12 |
| 4 | Where does manual zero calibration live now that it's removed from calibration page? | Settings page context panel (left box) — provides jog controls + per-axis zero-set buttons. This is the "hardware setup" location, matching the principle that zero calibration is per-machine, not per-print | 2026-03-12 |
| 5 | Xbox stick drift — how to fix? | Add stick center calibration: sample axes at rest for 2s, store offsets in settings.json, subtract before deadzone in polling worker. `calibrate_sticks()` runs with pygame directly (blocking, ~2s) | 2026-03-12 |
| 6 | Safe Z retract doesn't wait before XY | `safe_travel_to()` now uses `wait_for_z_arrival()` (blocking poll loop, 150ms interval, 15s timeout) before XY move, and `wait_for_xy_arrival()` before Z descent. Calibration page delegates to this standard function. Timeouts log warnings but proceed (fail-safe) | 2026-03-12 |

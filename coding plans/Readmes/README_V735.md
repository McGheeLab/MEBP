# MEBP v7.3.5 — Release Notes

## Overview

Version 7.3.5 delivers **critical safe Z navigation fixes**, a **click-to-move event filter fix**, an **XY unit system overhaul** (renamed `microsteps_per_micron` → `xy_position_scale`, corrected scale from 10 to 1), **ZP stage settings** (configurable feedrates + EEPROM save), and **simulation state persistence** for both ZP and XY simulators.

---

## Bug Fixes

### BF-1 — Safe Z: Serial Buffer Drain (CRITICAL)

Fixed a race condition where `flush_moves()` read stale "ok" responses from prior G-code commands instead of waiting for M400's physical-completion "ok". This caused XY movement to start while Z was still retracting, bending the needle.

| Layer | Mechanism |
|-------|-----------|
| Layer 1 | `ZPStage.flush_moves()` drains serial input buffer (`reset_input_buffer()`) before sending M400, all within a single `_serial_lock` acquisition |
| Layer 2 | `StageController.safe_travel_to()` polls actual Z position via M114 (`wait_for_z_arrival()`) after `flush_moves()` returns — XY starts only if BOTH layers confirm Z arrival |

**Files:** `SupportClasses/ZPStage.py`, `SupportClasses/StageController.py`

### BF-2 — Click-to-Move on Calibration Camera Feed

Fixed click-to-move not working on the calibration page camera feed. Mouse events were delivered to the child `QLabel` but not reliably propagated to the parent `CameraFeedView` across PySide6 versions.

**Fix:** Qt event filter installed on the `_display` QLabel intercepts `MouseButtonPress` events directly, performs widget-to-image coordinate conversion, and emits the `clicked` signal.

**Files:** `gui/widgets/camera_feed_view.py`

### BF-3 — XY Unit System Overhaul

Fixed XY positions being off by 10x (ProScan speaks µm natively, not microsteps). Renamed all "microsteps" terminology since the ProScan has real internal microsteps — calling the position readout scale "microsteps per micron" was misleading.

| Old | New |
|-----|-----|
| `microsteps_per_micron` | `xy_position_scale` |
| `DEFAULT_MICROSTEPS_PER_MICRON` | `DEFAULT_XY_POSITION_SCALE` |
| `steps_to_um()` / `um_to_steps()` | `stage_to_um()` / `um_to_stage()` |
| `set_microsteps_per_micron()` | `set_xy_position_scale()` |
| `get_microsteps_per_micron_from_protocol()` | `get_position_scale_from_protocol()` |
| `stage.microsteps_per_micron` (settings key) | `stage.xy_position_scale` |
| Default value: `10.0` | Default value: `1.0` |

**Files (15):** `gui/unit_helpers.py`, `gui/app.py`, `gui/pages/settings_page.py`, `gui/pages/jog_control.py`, `gui/pages/dashboard.py`, `gui/pages/print_setup.py`, `gui/pages/print_monitor.py`, `gui/pages/calibration.py`, `gui/pages/helper_functions.py`, `config/controllers/proscan_ii.json`, `config/controllers/proscan_iii.json`, `config/xy_diagnostic_profile.json`, `SupportClasses/SafetyLimits.py`, `SupportClasses/XYStageSimulator.py`, `tests/test_s2_jog_and_filebrowser.py`

---

## New Features

### F-1 — ZP Stage Settings Card

New "ZP Stage (Z-Needle / Pumps)" settings card on the Settings page:

| Setting | Default | Purpose |
|---------|---------|---------|
| Max Feedrate | 200 mm/min | Hardware speed limit (M203) |
| Safe Z Retract Feedrate | 200 mm/min | `safe_travel_to()` Step 1 — raise needle |
| Safe Z Insert Feedrate | 100 mm/min | `safe_travel_to()` Step 3 — lower needle (half speed for precision) |
| Default Jog Feedrate | 200 mm/min | Manual jog/step moves |
| Auto-Save Position | Off | Periodic M500 EEPROM save at watchdog interval |

**Files:** `gui/pages/settings_page.py`, `SupportClasses/StageController.py`, `main.py`

### F-2 — Watchdog Periodic Position Save (M500)

`ConnectionWatchdog` gains general-purpose `add_periodic(callback)` / `remove_periodic(callback)` hooks. `StageController._periodic_zp_position_save()` sends M500 to Marlin EEPROM at the watchdog interval when enabled. On crash/power loss, Marlin restores the last saved position on reset.

**Files:** `SupportClasses/SerialUtils.py`, `SupportClasses/StageController.py`

### F-3 — Simulation State Persistence

Both simulators now persist state to JSON files in `config/`, matching real hardware behavior:

**ZP Simulator** (`config/sim_zp_state.json`):
- M500 saves: position, steps/mm, max feedrate, max acceleration, speed factor, motor counts, mode flags
- M501 loads state from file; M503 reports settings in Marlin echo format
- M92/M203/M220 now actually parsed and stored (were no-ops)
- M400 implemented (checks axis convergence)
- Motor counts tracked during physics loop
- `readline()` method added for `flush_moves()` compatibility

**XY Simulator** (`config/sim_xy_state.json`):
- `stop()` saves position and settings (speed/accel/scurve, stage info)
- Startup loads previous position (matches real ProScan session persistence)

**Files:** `SupportClasses/ZPStageSimulator.py`, `SupportClasses/XYStageSimulator.py`

---

## Files Modified

| File | Changes |
|------|---------|
| `SupportClasses/ZPStage.py` | `flush_moves()` serial buffer drain before M400 |
| `SupportClasses/StageController.py` | Dual-layer Z verification, ZP feedrate settings, periodic M500 |
| `SupportClasses/SerialUtils.py` | `ConnectionWatchdog.add_periodic()` / `remove_periodic()` |
| `SupportClasses/ZPStageSimulator.py` | Full EEPROM simulation (M500/M501/M503/M92/M203/M220/M400), state persistence, motor counts, `readline()` |
| `SupportClasses/XYStageSimulator.py` | State persistence (`_save_state`/`_load_state`), `xy_position_scale` rename |
| `SupportClasses/SafetyLimits.py` | µm label terminology |
| `gui/unit_helpers.py` | Complete rename: `stage_to_um()`, `um_to_stage()`, `xy_position_scale`, `DEFAULT_XY_POSITION_SCALE` |
| `gui/app.py` | `_xy_position_scale` property, `_resolve_xy_position_scale()`, propagation rename |
| `gui/widgets/camera_feed_view.py` | Qt event filter for click-to-move on QLabel |
| `gui/pages/settings_page.py` | ZP Stage settings card, `xy_position_scale` rename |
| `gui/pages/jog_control.py` | `set_xy_position_scale()`, scale label fix |
| `gui/pages/dashboard.py` | `set_xy_position_scale()` rename |
| `gui/pages/calibration.py` | `set_xy_position_scale()` rename, local var `mpm` → `scale` |
| `gui/pages/print_setup.py` | `set_xy_position_scale()` rename |
| `gui/pages/print_monitor.py` | `set_xy_position_scale()` rename |
| `gui/pages/helper_functions.py` | `set_xy_position_scale()` stub rename |
| `config/controllers/proscan_ii.json` | `microsteps_per_micron` → `xy_position_scale` (1.0) |
| `config/controllers/proscan_iii.json` | `microsteps_per_micron` → `xy_position_scale` (1.0) |
| `config/xy_diagnostic_profile.json` | `microsteps_per_micron` → `xy_position_scale` (25) |
| `tests/test_s2_jog_and_filebrowser.py` | Full rename: `stage_units`, `xy_position_scale`, `set_xy_position_scale` |
| `main.py` | ZP stage settings wiring |

---

## Architecture Reference

See `coding plans/Architectures/ARCHITECTURE_V735.md` for the full system architecture.

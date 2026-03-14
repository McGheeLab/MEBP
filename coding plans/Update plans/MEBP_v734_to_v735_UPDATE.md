# MEBP v7.3.4 → v7.3.5 Update Plan

## Objective

Critical bug fixes for safe Z navigation and calibration page click-to-move.

---

## Bug Fixes

### BF-1 — Safe Z: flush_moves() reads stale "ok" responses (CRITICAL)

**Files:** `SupportClasses/ZPStage.py`, `SupportClasses/StageController.py`

**Symptom:** When navigating between wells, the Z axis begins retracting but XY movement starts immediately without waiting for Z to reach safe height. This bends and breaks the needle. The Z position readout confirms Z is still physically moving when XY starts.

**Root cause:** `ZPStageManager.send_data()` writes G-code commands (G90, G0, G91) to the serial port but never reads the "ok" response that Marlin sends for each command. These stale "ok" responses accumulate in the serial RX buffer. When `flush_moves()` sends M400 and calls `readline()` expecting M400's "ok" (which only arrives after physical motion completes), it immediately reads a stale "ok" from one of the prior commands and returns `True` — falsely indicating motion is complete.

Timeline:
1. `move_absolute()` sends G90 → Marlin responds "ok" (unread)
2. `move_absolute()` sends G0 Z{safe} → Marlin responds "ok" (unread)
3. `move_absolute()` sends G91 → Marlin responds "ok" (unread)
4. `flush_moves()` sends M400 → Marlin will respond "ok" only after physical completion
5. `flush_moves()` calls `readline()` → gets stale "ok" from step 1 → returns True immediately
6. XY move starts while Z is still retracting → needle crash

**Fix (two layers — belt and suspenders):**

1. **ZPStage.flush_moves()**: Drain the serial input buffer (`reset_input_buffer()`) and send M400 within a single `_serial_lock` acquisition. This ensures no stale data remains when we start waiting for M400's "ok".

2. **StageController.safe_travel_to()**: After `flush_moves()` returns, also poll the actual Z position via `wait_for_z_arrival()` (M114 queries) to verify physical arrival. XY will NOT start unless BOTH layers confirm Z is at safe height. On failure, the method returns `False` immediately — no "proceeding anyway."

**Status:** `[x]` done

---

### BF-2 — Click-to-move on calibration page camera feed

**Files:** `gui/widgets/camera_feed_view.py`

**Symptom:** Clicking on the live camera feed on the calibration page with Click→Move enabled does not move the stage.

**Root cause:** `CameraFeedView.mousePressEvent()` is defined on the parent QWidget, but mouse events are delivered to the child `QLabel` (`_display`). In PySide6, QLabel's default `mousePressEvent` should call `event.ignore()` which propagates to the parent, but this propagation is unreliable across PySide6 versions and when the QLabel has a pixmap set.

**Fix:** Install a Qt event filter on the `_display` QLabel in `CameraFeedView.__init__()`. The `eventFilter()` method intercepts `MouseButtonPress` events on the QLabel, performs the widget-to-image coordinate conversion, and emits the `clicked` signal directly. This bypasses any event propagation issues.

**Status:** `[x]` done

---

### BF-3 — XY Unit System: rename + fix scale factor

**Files (15):** `gui/unit_helpers.py`, `gui/app.py`, `gui/pages/settings_page.py`, `gui/pages/jog_control.py`, `gui/pages/dashboard.py`, `gui/pages/print_setup.py`, `gui/pages/print_monitor.py`, `gui/pages/calibration.py`, `gui/pages/helper_functions.py`, `config/controllers/proscan_ii.json`, `config/controllers/proscan_iii.json`, `config/xy_diagnostic_profile.json`, `SupportClasses/SafetyLimits.py`, `SupportClasses/XYStageSimulator.py`, `tests/test_s2_jog_and_filebrowser.py`

**Symptom:** XY positions off by 10x. Safety limits UI said "steps" instead of "µm". Code used misleading term "microsteps_per_micron" — the ProScan has real microsteps internally, but reports positions in µm.

**Root cause:** `microsteps_per_micron` was 10.0 but ProScan speaks µm natively (confirmed via v7.2.5 diagnostic). Term was also misleading.

**Fix (two parts):**

*Part 1 — Value fix:* Changed default from 10.0 to 1.0 everywhere.

*Part 2 — Terminology rename:* Removed all "microsteps" terminology since it's incorrect (the stage has real microsteps internally, but its position readout is in µm). Full rename:

| Old | New |
|-----|-----|
| `microsteps_per_micron` | `xy_position_scale` |
| `DEFAULT_MICROSTEPS_PER_MICRON` | `DEFAULT_XY_POSITION_SCALE` |
| `steps_to_um()` | `stage_to_um()` |
| `um_to_steps()` | `um_to_stage()` |
| `set_microsteps_per_micron()` | `set_xy_position_scale()` |
| `get_microsteps_per_micron_from_protocol()` | `get_position_scale_from_protocol()` |
| `_resolve_microsteps_per_micron()` | `_resolve_xy_position_scale()` |
| `stage.microsteps_per_micron` (settings key) | `stage.xy_position_scale` |
| `MICROSTEPS_PER_MICRON` (simulator) | `XY_POSITION_SCALE` |

**Status:** `[x]` done

---

## Implementation Steps

- [x] Create update plan (this file)
- [x] BF-1: Drain serial buffer in `flush_moves()` before sending M400
- [x] BF-2: Install event filter on CameraFeedView's QLabel child
- [x] BF-3: Fix XY scale factor (10→1) and rename microsteps_per_micron → xy_position_scale
- [x] F-3: Simulation state persistence (ZP EEPROM + XY position to JSON)
- [x] Update CLAUDE.md version
- [x] Create ARCHITECTURE_V735.md
- [x] Create README_V735.md
- [x] Update CLAUDE.md to reference ARCHITECTURE_V735.md

---

## Features

### F-1 — ZP Stage Settings Card

**Files:** `gui/pages/settings_page.py`, `SupportClasses/StageController.py`, `main.py`

**Description:** New "ZP Stage (Z-Needle / Pumps)" settings card on the Settings page with configurable feedrates and EEPROM position save toggle.

**Settings (persisted to settings.json under `zp_stage.*`):**
- **Max Feedrate** (mm/min) — Hardware speed limit, sent to Marlin via M203. Default: 200.
- **Safe Z Retract Feedrate** (mm/min) — Used by `safe_travel_to()` Step 1 (raise needle). Default: 200 (max speed).
- **Safe Z Insert Feedrate** (mm/min) — Used by `safe_travel_to()` Step 3 (lower needle). Default: 100 (half speed for precision).
- **Default Jog Feedrate** (mm/min) — Used for manual jog/step moves. Default: 200.
- **Auto-Save Position** (checkbox) — Periodically sends M500 to save Marlin position to EEPROM at the watchdog interval. Off by default.

**Apply behavior:**
- Max feedrate → `zp_stage.set_max_feedrate()` (M203 command)
- Jog feedrate → `zp_stage.feedrate` (used by `move_relative`)
- Retract/insert feedrates → `controller._zp_retract_feedrate` / `_zp_insert_feedrate`
- Auto-save → `controller._zp_auto_save_position`

**Status:** `[x]` done

---

### F-2 — Watchdog Periodic Position Save (M500)

**Files:** `SupportClasses/SerialUtils.py`, `SupportClasses/StageController.py`

**Description:** Periodically saves the ZP stage position to Marlin EEPROM via M500 at the watchdog interval (configurable, default 2s). On crash or power loss, Marlin restores the last saved position on reset.

**Implementation:**
- `ConnectionWatchdog` gains `add_periodic(callback)` / `remove_periodic(callback)` — general-purpose hooks called on every watchdog cycle.
- `StageController._periodic_zp_position_save()` — sends M500 when enabled, skips during `safe_travel_to` (poller suspended), skips in simulation mode.
- Enabled/disabled via the "Auto-Save Position" checkbox in the ZP Stage settings card.

**Status:** `[x]` done

---

### F-3 — Simulation Hardware State Persistence

**Files:** `SupportClasses/ZPStageSimulator.py`, `SupportClasses/XYStageSimulator.py`

**Description:** Both simulators now persist their state to JSON files in `config/`, making simulation consistent across sessions. On real hardware, the Marlin controller saves state to EEPROM (M500); the Prior ProScan maintains position until power loss. The simulators now behave the same way.

**ZP Simulator (`config/sim_zp_state.json`):**
- **M500**: Saves position, steps/mm (M92), max feedrate (M203), max acceleration (M201), speed factor (M220), motor counts, positioning mode, cold extrusion flag
- **M501**: Loads saved state from file
- **M503**: Reports current settings in Marlin format (echo lines)
- **M92/M203/M220**: Now actually parsed and stored (were no-ops before)
- **M400**: Implemented (checks if axes are at target)
- **Motor counts**: Now tracked during physics loop (`delta_mm * steps_per_mm`)
- **readline()**: Added for compatibility with `flush_moves()` serial interface
- **Startup**: Loads previous state from `sim_zp_state.json` if it exists

**XY Simulator (`config/sim_xy_state.json`):**
- **stop()**: Saves position and settings (speed/accel/scurve percentages, stage info)
- **Startup**: Loads position and settings from `sim_xy_state.json` if it exists
- Position persists across sessions (like real ProScan maintains position)

**State files are runtime-generated** — not committed to git.

**Status:** `[x]` done

---

## Testing Notes

### BF-1 — Safe Z
1. Connect real hardware (Z + XY stages)
2. Navigate needle into a well
3. Click a different well to trigger safe_travel_to()
4. Verify Z fully retracts BEFORE XY movement begins
5. Check logs for "flush_moves" timing
6. Verify Z retract uses the configured retract feedrate (check G0 F value in serial)

### BF-2 — Click-to-move
1. Open Calibration page with camera connected
2. Click "Click→Move: ON" toggle
3. Click on a feature in the live camera feed
4. Verify stage moves to center the clicked point
5. Check logs for "[CalibPage] click-to-move" debug messages

### BF-3 — XY Units
1. Launch app with ProScan connected
2. Check status bar XY position — should match physical position in µm (not 10x smaller)
3. Open Settings → Safety Limits → verify spinboxes say "µm" not "steps"
4. Open Jog page → move 100 µm → verify position display updates by ~100
5. Run `python -m pytest tests/test_s2_jog_and_filebrowser.py` — all assertions should pass with factor=1.0

### F-1 — ZP Stage Settings
1. Open Settings page → verify "ZP Stage" card appears after Controller Protocol
2. Change Max Feedrate → Apply → verify M203 is sent (check serial log or motor speed)
3. Change Retract Feedrate to 200, Insert Feedrate to 50 → navigate between wells → verify fast up, slow down
4. Close and reopen app → verify settings persist

### F-2 — Position Save
1. Enable "Auto-Save Position" in ZP Stage card → Apply
2. Move Z to a known position, wait for watchdog cycle
3. Check logs for "Periodic ZP position save (M500)"
4. Power-cycle the Marlin board → verify it starts at the saved position

---

### F-3 — Simulation State Persistence
1. Launch app in simulation mode
2. Jog Z down to 10mm, move a pump to 5mm
3. Close app — check `config/sim_zp_state.json` exists with correct positions
4. Relaunch — verify Z and pump positions restore from previous session
5. Verify M500 saves state (enable auto-save position, check log for "EEPROM saved")
6. Jog XY to (1000, 2000) µm → close app → check `config/sim_xy_state.json`
7. Relaunch → verify XY position restores

## Issues & Decisions

- **Serial buffer drain vs per-command response reading**: Chose buffer drain approach in `flush_moves()` rather than making every `send_data()` consume its response. The drain approach is a minimal, targeted fix. Making `send_data()` read responses would be a larger refactor affecting all G-code communication and could introduce new timing issues with commands that produce multi-line responses.
- **Event filter vs event propagation**: Chose event filter approach over relying on Qt event propagation because it's explicit and reliable across PySide6 versions. The event filter intercepts clicks at the source (QLabel) rather than depending on propagation behavior that may vary.
- **Periodic callbacks on ConnectionWatchdog**: Added a general-purpose `add_periodic()` mechanism rather than hardcoding ZP-specific logic in the watchdog. Any future periodic task can register the same way.
- **Insert feedrate default**: Set to half of max feedrate (100 mm/min) rather than max. Lowering the needle into a well benefits from slower approach to avoid splashing or overshooting.
- **xy_position_scale = 1.0 (identity)**: The `stage_to_um()` / `um_to_stage()` functions are retained even though they're now identity functions (scale=1.0). This preserves the API for future support of stages that report in different units without requiring another codebase-wide refactor. The scale factor is loaded from controller protocol JSON → settings → default, in priority order.
- **Terminology rename**: Renamed `microsteps_per_micron` → `xy_position_scale` throughout because the ProScan has real internal microsteps — calling the position readout scale "microsteps per micron" was misleading. The new name accurately describes what it is: a scale factor between the stage's position readout and µm.

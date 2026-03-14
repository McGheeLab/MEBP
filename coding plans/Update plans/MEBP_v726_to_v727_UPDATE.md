# MEBP v7.2.6 → v7.2.7 Update Plan

## Objective

Major feature and stability release: new calibration workflow with safe Z navigation, Xbox Bluetooth cross-platform support, print speed propagation fixes, position display accuracy, Helper Functions page, and simulator blocking behavior.

---

## Features

### F-1 — Calibration Workflow Overhaul

**Files:** `gui/pages/calibration.py`

**Description:** New calibration workflow with safe Z navigation, top Z teaching, 3-point Z-plane calibration, auto-navigate between wells, and clickable plate view.

**Key additions:**
- State attributes: `_safe_z`, `_top_z`, `_taught_third`
- `_safe_navigate_to()` helper: Z up → XY travel → Z down with threading + button disable
- Auto-estimate well positions from taught A1 + corner
- Clickable `_CalibrationPlateView` for direct well navigation
- `WellBottomDetector` integration for Z-plane calibration

**Status:** `[x]` done

---

### F-2 — Xbox Bluetooth Cross-Platform Support

**Files:** `SupportClasses/XboxController.py`, `SupportClasses/StageController.py`, `gui/pages/dashboard.py`, `current_button_mapping.json`

**Description:** Full Xbox controller support across macOS, Windows, and Linux over Bluetooth and USB.

**Key fixes:**
- OS-aware SDL environment variables (SDL_VIDEODRIVER=dummy for threads)
- Platform-specific trigger normalization (macOS 0..1 vs Windows/Linux -1..1)
- Xbox Series X Bluetooth button mapping (updated indices)
- D-pad-as-buttons support when `hats=0`
- `pygame.joystick.init()` instead of `pygame.init()` to prevent Cocoa SIGTRAP on macOS threads
- Reverted to `pygame.init()` with SDL_VIDEODRIVER=dummy (event system needed for polling)
- Speed decade increments (×10/÷10) with 300ms button debounce
- LT=retract/RT=extend trigger direction
- Faster XY/axis update interval (0.5s → 0.1s)
- Xbox status feedback on dashboard (disconnect status clearing)

**Status:** `[x]` done

---

### F-3 — Helper Functions Page

**Files:** `SupportClasses/ImagePathPlanner.py` (**New**), `gui/pages/helper_functions.py` (**New**), `gui/app.py`

**Description:** New page (index 6) for mask-based image-to-path planning. ImagePathPlanner converts images to rastering paths with proportional pump flow, multi-layer modes, and corner slowdown.

**Status:** `[x]` done

---

### F-4 — Print Speed Propagation Chain

**Files:** `SupportClasses/XYStage.py`, `SupportClasses/PrintManager.py`, `SupportClasses/TrajectoryPlanner.py`, `gui/pages/print_setup.py`

**Description:** Fixed the complete speed propagation chain from GUI settings to hardware.

**Key fixes:**
- `XYStage.set_speed_mm_s()` — new method for absolute speed setting (not percentage-based SMS)
- `PrintManager` uses `set_speed_mm_s` for trajectory, MOVE_XY, HOME_XY travel
- `PrintSettings` gains `print_speed_mm_s` and `travel_speed_mm_s` fields
- `print_feedrate` fixed (mm/min → mm/s conversion)
- SMS (Set Movement Speed) set before printing starts
- Trajectory timestamps rescaled to match GUI speed
- `TrajectoryPlanner` reads speeds from workspace, passes xy_travel_speed to `create_travel_move`
- Z default speed 2→5 mm/s, XY travel default 20→50 mm/s
- Travel speed synced from GUI to workspace

**Status:** `[x]` done

---

## Bug Fixes

### BF-1 — 10x position display error on Dashboard + Calibration

**Files:** `gui/pages/dashboard.py`, `gui/pages/calibration.py`

**Symptom:** Position values displayed at 1/10th of actual position.

**Root cause:** Controller reports positions in µm directly. GUI was dividing by `microsteps_per_micron` (10), producing 1/10th values.

**Fix:** Removed `/microsteps_per_micron` division from `Dashboard.update_data()` and `calibration.on_status_update()`. Display raw values directly.

**Status:** `[x]` done

---

### BF-2 — Calibration coordinate unit mismatches (10x errors)

**Files:** `gui/pages/calibration.py`

**Symptom:** Calibrated well positions off by 10x or mixed µm/mm in calculations.

**Root cause:** Multiple methods applied `steps_to_um` conversions on values already in µm. `_calculate_alignment` mixed µm and mm. `_set_zero` applied unnecessary conversion.

**Fix:** Removed spurious `steps_to_um` conversions from: `on_status_update`, `_set_zero`, `_record_a1`, `_record_corner`, `_goto_a1`, `_goto_corner`, `_load_calibration`, `_calculate_alignment`. Reset `_scale`/`_rotation` on A1 teach to prevent stale values.

**Status:** `[x]` done

---

### BF-3 — Calibration persistence + navigation bugs

**Files:** `gui/pages/calibration.py`

**Symptom:** Multiple AttributeErrors on startup. Navigation to wells fails.

**Root cause:** Missing attribute initialization in `__init__`. `_safe_navigate_to` didn't wait for motion. `_estimate_well_position_um` used stale `_scale` values.

**Fix:** Ensured all required attributes in `__init__`. Fixed `_safe_navigate_to` with proper waiting. Guarded `_estimate_well_position_um` against stale values. Guarded `_on_plate_changed`.

**Status:** `[x]` done

---

### BF-4 — Connection status dots not updating

**Files:** `gui/app.py`, `gui/pages/dashboard.py`, `SupportClasses/XYStage.py`, `SupportClasses/ZPStage.py`

**Symptom:** Connection status indicators stuck after state change.

**Root cause:** QSS stylesheet not re-evaluated on property change.

**Fix:** Added unpolish/polish QSS refresh cycle for status dots. Removed serial_lock from `get_current_position` reads (prevented starvation).

**Status:** `[x]` done

---

### BF-5 — ZPJogHandler._clamp_pump_flow AttributeError

**Files:** `SupportClasses/StageController.py`

**Symptom:** AttributeError for `_hardware_config` when clamping pump flow.

**Fix:** Safe `_hardware_config` access via `getattr`. Initialize in `ZPJogHandler.__init__`.

**Status:** `[x]` done

---

### BF-6 — XY velocity too slow during prints

**Files:** `SupportClasses/PrintManager.py`

**Symptom:** Print XY movements extremely slow compared to expected speed.

**Root cause:** `print_feedrate` passed as mm/min but consumed as mm/s. Redundant axis commands between moves.

**Fix:** Fixed mm/min → mm/s conversion. Set SMS before printing. Skipped redundant axis commands. Fixed `_wait_for_xy_settle`.

**Status:** `[x]` done

---

### BF-7 — Simulator G/GR commands don't block like real hardware

**Files:** `SupportClasses/XYStageSimulator.py`

**Symptom:** Simulator returns immediately from move commands while real Prior hardware blocks until arrival.

**Fix:** Added `_wait_for_idle()` helper. G and GR commands now block until simulated arrival, matching real ProScan behavior.

**Status:** `[x]` done

---

### BF-8 — app.py AST errors from patching

**Files:** `gui/app.py`

**Symptom:** Pages list formatting, menu button order, btn_map indices, page gating all broken.

**Fix:** Corrected pages list, titles, context_titles, and page gating. Fixed _connect_xbox thread mode. Restored dashboard from backup and re-applied patches.

**Status:** `[x]` done

---

## Implementation Steps

- [x] F-1: Calibration workflow (safe Z, top Z, 3-point Z-plane, auto-navigate, clickable plate)
- [x] F-2: Xbox Bluetooth (SDL env, triggers, D-pad, button mapping, debounce)
- [x] F-3: Helper Functions page (ImagePathPlanner, mask-based rastering)
- [x] F-4: Print speed chain (set_speed_mm_s, trajectory rescale, travel speeds)
- [x] BF-1: Remove 10x position display division
- [x] BF-2: Fix calibration coordinate unit mismatches
- [x] BF-3: Fix calibration persistence + navigation
- [x] BF-4: Fix connection status dots (QSS refresh)
- [x] BF-5: Fix ZPJogHandler pump flow clamping
- [x] BF-6: Fix XY print velocity (mm/min→mm/s, SMS)
- [x] BF-7: Simulator blocking for G/GR commands
- [x] BF-8: Fix app.py page registration errors

## Files Modified

| File | Change |
|------|--------|
| `gui/pages/calibration.py` | Workflow overhaul, unit fixes, persistence, navigation |
| `gui/pages/dashboard.py` | Position display fix, Xbox status, QSS refresh |
| `gui/pages/helper_functions.py` | **New** — Helper Functions page |
| `gui/app.py` | Page registration fix, QSS refresh, Xbox thread mode |
| `SupportClasses/XboxController.py` | Cross-platform SDL, triggers, D-pad, debounce |
| `SupportClasses/StageController.py` | Pump flow clamp init, speed debounce |
| `SupportClasses/XYStage.py` | set_speed_mm_s(), SMS fix |
| `SupportClasses/XYStageSimulator.py` | Blocking G/GR commands |
| `SupportClasses/PrintManager.py` | Speed propagation, velocity fix |
| `SupportClasses/TrajectoryPlanner.py` | Travel speed propagation |
| `SupportClasses/ImagePathPlanner.py` | **New** — mask-based image rastering |
| `gui/pages/print_setup.py` | Trajectory timestamp rescaling, GUI speed sync |
| `current_button_mapping.json` | Xbox Series X Bluetooth indices |

## Testing Notes

1. Calibration: teach A1 → navigate to corner → verify safe Z retract before XY move
2. Xbox: connect over Bluetooth → verify triggers work (LT retract, RT extend)
3. Xbox: disconnect → reconnect → verify auto-recovery
4. Print: run test print → verify XY speed matches settings
5. Dashboard: verify position display in µm (not 1/10th)
6. Simulator: G command → verify it blocks until arrival
7. Helper Functions: load image → generate path → verify raster pattern

## Issues & Decisions

- **pygame.init() vs joystick.init()**: `pygame.joystick.init()` alone avoids Cocoa SIGTRAP on macOS threads, but the event system (needed for joystick polling) requires `pygame.init()`. Resolved by setting `SDL_VIDEODRIVER=dummy` before `pygame.init()`.
- **Position display units**: ProScan reports µm directly. All intermediate `steps_to_um()` conversions were artifacts of an incorrect abstraction — the "microsteps" were always µm.
- **Simulator blocking**: Real ProScan blocks on G/GR until arrival. Making the simulator match prevents timing-dependent test failures and makes simulation behavior faithful.
- **Speed decade increments**: ×10/÷10 increments on Xbox bumpers give coarse speed control. 300ms debounce prevents accidental double-presses.

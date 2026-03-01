# MEBP v7.2 — Hardware Setup & µL Pump Control Upgrade

## Overview

This revision adds:
1. **New "Hardware Setup" page** (Page 0) — required before any other page works
2. **All pump motion in µL** throughout the entire project
3. **Save/Load hardware configurations** as JSON files

## Architecture

```
HardwareConfig (SupportClasses/HardwareConfig.py)
    └── PumpChannelConfig × 3
          ├── SyringeSpec (from PhysicalModels.py)  ← uL_to_mm() / mm_to_uL()
          ├── InkSpec
          ├── PrintingMode
          └── FluidColumn

HardwareSetupPage (gui/pages/hardware_setup.py)
    ├── Emits config_changed(HardwareConfig)
    ├── Emits config_validated(bool)
    └── Save/Load JSON files

MainWindow (gui/app.py)
    ├── Receives config signals
    ├── Gates pages 1-5 until valid
    └── Propagates HardwareConfig to all pages via set_hardware_config()

StageController
    ├── NEW: move_pump_uL(pump, volume_uL, rate_uL_s)
    ├── NEW: get_pump_position_uL(pump)
    └── NEW: extrude_uL(pump, volume_uL, rate_uL_s)

PrintManager
    ├── PrintSettings.pump_rate_uL_s (was pump_feedrate mm/min)
    ├── EXTRUDE cmd: amount_uL + rate_uL_s (was amount mm + feedrate mm/min)
    └── PRINT_PATH: flow_rate_uL_s (was dimensionless flow_rate)
```

## Conversion Chain

```
User enters: 5.0 µL at 0.25 µL/s
    ↓ PumpChannelConfig.uL_to_mm(5.0)
    ↓ SyringeSpec(100µL).mm_per_uL = 0.3  →  5.0 × 0.3 = 1.5 mm
    ↓ PumpChannelConfig.feedrate_uL_s_to_mm_min(0.25)
    ↓ 0.25 × 0.3 × 60 = 4.5 mm/min
    ↓ StageController.move_pump_relative("P1", 1.5, 4.5)
    ↓ Marlin G-code: G1 Y1.5 F4.5
```

---

## Files Delivered in This Session

| File | Status | Description |
|------|--------|-------------|
| `SupportClasses/HardwareConfig.py` | ✅ NEW | Central hardware config data model, save/load |
| `gui/pages/hardware_setup.py` | ✅ NEW | Hardware Setup GUI page (Page 0) |
| `gui/pages/jog_control.py` | ✅ MODIFIED | Pump jog in µL, HardwareConfig-aware |
| `gui/app_modifications.py` | ✅ GUIDE | Changes needed for app.py (documented) |
| `SupportClasses/stage_controller_additions.py` | ✅ GUIDE | New methods for StageController |
| `SupportClasses/print_manager_modifications.py` | ✅ GUIDE | Changes for PrintManager µL commands |

---

## Remaining Tasks for Next Chat Sessions

### Session 1: Core Integration (HIGHEST PRIORITY)

These tasks merge the new code into existing files:

- [ ] **T1.1** Merge `stage_controller_additions.py` into `StageController.py`
  - Add `from SupportClasses.HardwareConfig import HardwareConfig` import
  - Add `self._hardware_config = None` to `__init__()`
  - Add `set_hardware_config()`, `move_pump_uL()`, `get_pump_position_uL()`,
    `get_all_pump_positions_uL()`, `extrude_uL()` methods

- [ ] **T1.2** Merge `app_modifications.py` into `gui/app.py`
  - Add Hardware Setup page as page index 0
  - Shift all other page indices +1
  - Add `btn_hardware` button to sidebar (🔧 icon, first position)
  - Update `_on_menu_click()` button map with shifted indices
  - Add `_on_hardware_config_changed()`, `_on_hardware_validated()`,
    `_update_page_gating()` methods
  - Add `_restore_hardware_config()`, `_save_hardware_config()` methods
  - Update bottom bar pump readouts to show µL
  - Wire `config_changed` and `config_validated` signals from HardwareSetupPage

- [ ] **T1.3** Update `settings.json` DEFAULTS in `Settings.py`
  - Add `"hardware_config"` section to defaults
  - Remove old `"workspace"` section (superseded by HardwareConfig)

- [ ] **T1.4** Update `SupportClasses/__init__.py`
  - Add `HardwareConfig`, `PumpChannelConfig` to exports

### Session 2: PrintManager µL Migration

- [ ] **T2.1** Update `PrintSettings` dataclass
  - Add `pump_rate_uL_s`, `retract_amounts_uL`, `prime_amounts_uL`, `pump_rates_uL_s`
  - Keep legacy fields for backward compat
  - Update `from_dict()` to detect and handle both formats

- [ ] **T2.2** Update `_execute_command()` — EXTRUDE handler
  - Check for `amount_uL` first, fall back to `amount` (mm)
  - Use `controller.move_pump_uL()` for µL commands
  - Keep `move_pump_relative()` for legacy mm commands

- [ ] **T2.3** Update `_execute_command()` — PRINT_PATH handler
  - Use `flow_rate_uL_s` parameter
  - Calculate volume per segment using time × flow_rate_uL_s

- [ ] **T2.4** Update `build_well_plate_job()`
  - Generate EXTRUDE commands with `amount_uL` and `rate_uL_s`
  - Generate PRINT_PATH with `flow_rate_uL_s`
  - Use `settings.get_retract_uL()` and `settings.get_prime_uL()`

- [ ] **T2.5** Add `migrate_print_file_v71_to_v72()` to `load_print_file()`
  - Detect v7.1 format by checking for `amount` without `amount_uL`
  - Convert using HardwareConfig if available
  - Warn user if no conversion possible

- [ ] **T2.6** Update `export_gcode()` for µL-based amounts
  - Convert back to mm for G-code output (G-code is always mm)

- [ ] **T2.7** Update `PrintJob.to_dict()` / `from_dict()`
  - Add version field ("7.2")
  - Store HardwareConfig reference

### Session 3: Print Setup GUI Updates

- [ ] **T3.1** Update `gui/pages/print_setup.py`
  - Print settings panel: change pump fields to µL
  - Replace "Pump feedrate (mm/min)" with "Flow rate (µL/s)"
  - Replace "Retract (mm)" with "Retract (µL)"
  - Replace "Prime (mm)" with "Prime (µL)"
  - Per-pump rate overrides in µL/s

- [ ] **T3.2** Update print preview / 2D canvas
  - Show volume annotations in µL
  - Color-code by pump with syringe capacity indicators

- [ ] **T3.3** Update well plate job builder UI
  - Flow rate field in µL/s
  - Volume-per-well estimate display

- [ ] **T3.4** Add HardwareConfig awareness
  - `set_hardware_config()` method
  - Disable print execution if no valid config
  - Show configured pump info in context panel

### Session 4: Dashboard & Calibration Updates

- [ ] **T4.1** Update `gui/pages/dashboard.py`
  - Pump position readouts in µL (with syringe capacity bar)
  - Show "No syringe" instead of mm when unconfigured

- [ ] **T4.2** Update `gui/pages/calibration.py`
  - Add `set_hardware_config()` method
  - Use plate_format from HardwareConfig

- [ ] **T4.3** Update `gui/pages/print_monitor.py`
  - Syringe display widget shows µL dispensed / remaining
  - Flow rate display in µL/s
  - Trajectory view annotations in µL

### Session 5: SafetyLimits µL Integration

- [ ] **T5.1** Update `SafetyLimits`
  - Add pump limits in µL (computed from mm limits + syringe spec)
  - Add `clamp_pump_uL()` method
  - Add `clamp_pump_rate_uL_s()` method

- [ ] **T5.2** Update `gui/pages/settings_page.py`
  - Safety limits section: show pump limits in µL when syringe is configured
  - Dual display: "P1 max: 50 mm (166.7 µL with 100µL syringe)"

### Session 6: Xbox Controller µL Integration

- [ ] **T6.1** Update Xbox jog handlers in `StageController`
  - Pump jog via Xbox triggers/buttons should use µL steps
  - Speed scaling in µL/s instead of mm/min
  - Respect HardwareConfig for pump enable/disable

- [ ] **T6.2** Update Xbox mapping editor
  - Show pump step sizes in µL

### Session 7: Testing & Polish

- [ ] **T7.1** Write unit tests for HardwareConfig
  - Save/load round-trip
  - Validation logic
  - µL ↔ mm conversion accuracy
  - PumpChannelConfig serialization

- [ ] **T7.2** Write integration tests
  - Hardware Setup page → config propagation → page gating
  - Jog control µL commands
  - Print job with µL commands

- [ ] **T7.3** Update sample job files
  - Convert existing JSON print files to v7.2 format
  - Add sample hardware config files

- [ ] **T7.4** Update INSTRUCTIONS.md
  - New Hardware Setup section
  - Updated Jog Control section
  - Updated Print Setup section

- [ ] **T7.5** Update README.md and ARCHITECTURE.md
  - Add Hardware Setup page to page table
  - Update module diagram
  - Document µL conversion chain

---

## Migration Notes

### For Existing Users
- On first launch with v7.2, all pages except Hardware Setup and Settings are locked
- User must configure at least one pump with a syringe to unlock
- Existing print files (v7.1) are auto-detected and can be migrated
- Settings.json gains a `hardware_config` section on first save

### For Developers
- `move_pump_relative(pump, mm, feedrate_mm_min)` still works (for low-level/firmware use)
- `move_pump_uL(pump, µL, rate_µL_s)` is the preferred API going forward
- All GUI-facing code should use µL exclusively
- Only the StageController→ZPStage boundary converts to mm

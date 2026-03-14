# MEBP v7.1.2 → v7.2.0 Update Plan

## Objective

Major platform upgrade: new Hardware Setup page (Page 0) as a required prerequisite, uL-native pump control throughout the system, GUI migration from tab-based layout to PyDracula-style icon sidebar with Catppuccin Mocha theme, and central HardwareConfig data model.

---

## Features

### F-1 — Hardware Setup Page (Page 0)

**Files:** `gui/pages/hardware_setup.py` (**New**), `gui/app.py`, `SupportClasses/HardwareConfig.py` (**New**)

**Description:** New prerequisite page that must be completed before other pages unlock. Contains needle/plate/pump selectors, ink library CRUD, save/load hardware configurations as JSON files.

**Key components:**
- Needle gauge/length/channel selection
- Plate format selector (6-well through 384-well)
- Pump enable/disable with syringe selection per channel
- Ink library (add/edit/delete inks with color, viscosity, density)
- Save/Load hardware configs to `config/hardware/*.json`
- Hardware gating: pages 1-5 locked until valid hardware config

**Status:** `[x]` done

---

### F-2 — Central HardwareConfig Data Model

**Files:** `SupportClasses/HardwareConfig.py` (**New**), `SupportClasses/__init__.py`, `SupportClasses/Settings.py`

**Description:** Single source of truth for all hardware configuration. Replaces scattered per-page state with a unified model that propagates to all pages via signals.

**Key fields:** needle_config, plate_format, pumps (dict of PumpChannelConfig), ink_library, rosette_library, camera_config.

**Serialization:** `to_dict()` / `from_dict()` for JSON persistence. Version field for backward compatibility.

**Status:** `[x]` done

---

### F-3 — uL-Native Pump Control

**Files:** `SupportClasses/StageController.py`, `SupportClasses/PrintManager.py`, `gui/pages/jog_control.py`

**Description:** All user-facing pump values in microliters (uL). Conversion to mm happens only inside `StageController.move_pump_uL()`.

**New methods on StageController:**
- `set_hardware_config()` / `hardware_config` property
- `move_pump_uL(pump_id, volume_uL, direction)` — converts uL to mm using syringe cross-section
- `get_pump_position_uL(pump_id)` — converts mm position to uL
- `extrude_uL(pump_id, volume_uL, flow_rate_uL_s)` — timed extrusion

**PrintManager:** Updated all pump commands from mm to uL API.

**Status:** `[x]` done

---

### F-4 — PyDracula GUI Migration

**Files:** `gui/app.py` (829 lines), `gui/styles.py` (781 lines), `gui/ui_functions.py` (147 lines), all page files

**Description:** Complete GUI framework migration from tab-based layout to PyDracula-style icon sidebar with context panels.

**Key changes:**
- Catppuccin Mocha QSS theme with COLORS dict in `gui/styles.py`
- Left sidebar with page icons (48px column)
- Right-side context panel (300px) per page
- Page interface contract: `get_page_title()`, `get_context_widget()`, `on_status_update()`
- 5 pages migrated: Dashboard, Jog Control, Calibration, Print Setup, Settings

**Total:** ~6,048 lines across 15 files.

**Status:** `[x]` done

---

### F-5 — Console Log Widget

**Files:** `gui/widgets/console_log.py` (**New**), `gui/app.py`

**Description:** Bottom panel console log widget showing application logs. Connected to Python logging via `QtLogHandler`.

**Status:** `[x]` done

---

## Implementation Steps

- [x] Create HardwareConfig data model with PumpChannelConfig
- [x] Create Hardware Setup page with all UI sections
- [x] Add uL pump methods to StageController
- [x] Update PrintManager for uL commands
- [x] Migrate GUI framework to PyDracula sidebar layout
- [x] Implement Catppuccin Mocha theme (QSS)
- [x] Add context panel interface to all pages
- [x] Create ConsoleLogWidget with QtLogHandler
- [x] Update Settings.py with hardware_config defaults
- [x] Wire hardware gating (pages locked until HW setup complete)

## Files Modified

| File | Change |
|------|--------|
| `SupportClasses/HardwareConfig.py` | **New** — central hardware config model |
| `SupportClasses/StageController.py` | uL pump methods, hardware_config property |
| `SupportClasses/PrintManager.py` | uL command API migration |
| `SupportClasses/__init__.py` | Export HardwareConfig |
| `SupportClasses/Settings.py` | hardware_config defaults |
| `gui/app.py` | PyDracula sidebar, hardware gating, config propagation |
| `gui/styles.py` | Catppuccin Mocha theme, COLORS dict, style constants |
| `gui/ui_functions.py` | **New** — sidebar animation, toggle helpers |
| `gui/pages/hardware_setup.py` | **New** — Page 0 hardware configuration |
| `gui/pages/dashboard.py` | Migrated to new page interface |
| `gui/pages/jog_control.py` | Migrated + uL pump display |
| `gui/pages/calibration.py` | Migrated to new page interface |
| `gui/pages/print_setup.py` | Migrated to new page interface |
| `gui/pages/settings_page.py` | Migrated to new page interface |
| `gui/widgets/console_log.py` | **New** — console log with QtLogHandler |

## Testing Notes

1. Launch app — verify Hardware Setup (Page 0) is shown first
2. All other pages should be grayed/locked until HW config is complete
3. Configure pumps — verify uL values display correctly on Jog page
4. Save HW config — reload — verify all fields restore
5. Check Catppuccin theme renders correctly (dark backgrounds, accent colors)
6. Verify console log captures Python logging output

## Issues & Decisions

- **Hardware-first gating**: Pages 1-5 locked until Hardware Setup emits a valid config. This prevents null-reference errors throughout the pipeline.
- **uL-native convention**: All user-facing values in uL, conversion to mm only at the StageController boundary. This prevents unit confusion in the GUI layer.
- **PyDracula over tab-based**: Sidebar navigation allows context panels and gives a more professional look. Each page owns its context widget.
- **HardwareConfig as single source of truth**: Eliminates scattered state. Config propagates via `set_hardware_config()` signal chain through `app.py`.

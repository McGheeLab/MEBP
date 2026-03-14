# MEBP v7.2.3 → v7.2.4 Update Plan

## Objective

Comprehensive config propagation audit, XY jog step verification, centralized GUI styles, pump-ink-needle channel mapping, zoomable print preview with out-of-bounds detection, and print plan validation. Completed across 6 coding sessions.

---

## Features

### F-1 — Centralized GUI Style Constants (Session 1)

**Files:** `gui/styles.py`, all page files

**Description:** Added 4 centralized style constants to replace per-page `_group_style()` methods:
- `SECTION_TITLE_STYLE` — section header styling
- `CONTEXT_SECTION_LABEL_STYLE` — context panel labels
- `PAGE_HEADER_STYLE` — page title styling
- `CARD_FRAME_STYLE` — card container frames

All pages updated to use centralized constants.

**Status:** `[x]` done

---

### F-2 — Hardware Config Propagation Audit (Session 1)

**Files:** `gui/app.py`, `gui/pages/calibration.py`, `gui/pages/print_well_setup.py`, `gui/pages/print_objects.py`, `gui/pages/print_setup.py`

**Description:** Enhanced `_propagate_hardware_config()` in app.py with per-page debug logging and try/except error handling. Fixed propagation to calibration page (plate combo sync, WellPlate model rebuild), well setup (ink/rosette combos from HardwareConfig), and print objects (ink options, well diameter).

**Status:** `[x]` done

---

### F-3 — Jog Step Size Verification Display (Session 2)

**Files:** `gui/pages/jog_control.py`

**Description:** Added step verification UI to the jog control context panel:
- Conversion factor display showing current microsteps_per_micron
- Warning banner when using default factor (not calibrated)
- Last jog verification: requested step vs measured delta
- Pre-jog position tracking for delta calculation

**New attributes:** `_last_jog_step_um`, `_last_xy_before`, `_conversion_factor_set`

**Status:** `[x]` done

---

### F-4 — Hardware Config File Browser (Session 2)

**Files:** `gui/pages/hardware_setup.py`

**Description:** Added config file browser to Hardware Setup context panel:
- QListWidget showing `config/hardware/*.json` files with names from JSON
- Double-click to load, delete button with confirmation dialog
- Refresh button, active config highlighting
- Auto-refresh after save/load operations

**Status:** `[x]` done

---

### F-5 — Pump-Ink Assignment & Needle-Channel Mapping (Session 3)

**Files:** `SupportClasses/HardwareConfig.py`, `gui/pages/hardware_setup.py`

**Description:** Implemented exclusive pump-ink assignment and needle-channel-pump mapping.

**HardwareConfig additions:**
- Properties: `pump_ink_map`, `ink_pump_map`, `unassigned_inks`, `enabled_pump_ids`
- Field: `needle_channel_pump_map: dict[int, str]` (channel → pump ID)
- Methods: `set_channel_pump()`, `get_channel_pump()`, `clear_channel_map()`, `auto_assign_channels()`, `get_pump_for_ink()`, `get_channel_for_ink()`
- Validation: ink uniqueness, channel map completeness, validity, uniqueness
- Serialization version bumped to "7.2.4"

**Hardware Setup UI:**
- Reordered: Name → Plate → Inks → Pumps → Needle → Channel Map → Rosettes → Actions
- PumpChannelWidget: grayed-out excluded inks in combo
- Pump-ink summary label
- "Needle Channel Assignment" group with dynamic rows
- Channel map restore on config load

**Status:** `[x]` done

---

### F-6 — Zoomable Print Preview with Out-of-Bounds Detection (Session 4)

**Files:** `gui/widgets/well_preview.py` (**New**), `gui/pages/print_objects.py`, `gui/pages/print_well_setup.py`

**Description:** Replaced XZ/ZY projection views with a single large zoomable XY-only preview.

**New widget** `well_preview.py`:
- Classes: `WellPreviewScene`, `WellPreviewView`, `WellPreviewWidget`, `ObjectPath` dataclass
- Zoom 10%–1000%, pan via middle-click/Ctrl+click
- mm grid overlay, axis labels, well boundary circle
- Out-of-bounds detection (red/dashed border, objects flash red with ⚠ icon)
- `oob_detected` signal emits list of OOB object indices

**Print Objects:** Preview replaced with WellPreviewWidget. Added `_check_bounds()`, OOB flash timer (500ms).

**Print Well Setup:** Removed top splitter + XZ/ZY projections, replaced with full-width plate view.

**Status:** `[x]` done

---

### F-7 — Print Plan Validation & Send-to-Monitor Gate (Session 5)

**Files:** `gui/pages/print_well_setup.py`, `gui/pages/print_setup.py`

**Description:** Added plan of action UI section to well setup tab with validation gate before sending to monitor.

**Plan UI:** Plan summary display, "Generate Plan" button, plan step list.
**Validation:** `validate()` method checks well assignments, ink availability, plan completeness. "Send to Monitor" blocked until validation passes.

**Status:** `[x]` done

---

## Bug Fixes

### BF-1 — print_objects.py crashes on auto-save (Hotfix)

**Files:** `gui/pages/print_objects.py`

**Symptom:** `'str' object has no attribute 'objects'` crash on auto-save.

**Root cause:** `_load_print_file()` stored file path string in `_current_file` instead of using `file_manager.current` (the PrintFileData object).

**Fix:** Use `file_manager.current` as authoritative source. Added `hasattr(pf, 'objects')` guard in `_do_auto_save()`.

**Status:** `[x]` done

---

### BF-2 — print_well_setup.py crashes on get_zp_position() tuple

**Files:** `gui/pages/print_well_setup.py`

**Symptom:** `.get()` called on tuple return from `get_zp_position()`.

**Fix:** Changed from dict-style `.get()` to proper tuple index access.

**Status:** `[x]` done

---

### BF-3 — Plan not auto-generated on first well assignment

**Files:** `gui/pages/print_well_setup.py`

**Symptom:** "Send-to-Monitor" fails because plan is None.

**Root cause:** `_on_plan_auto_regen()` had a "plan is not None" guard that prevented generation on first assignment. `validate()` didn't auto-generate missing plan.

**Fix:** Removed guard; auto-generate plan if missing before validation.

**Status:** `[x]` done

---

## Implementation Steps

- [x] Session 1: Centralize GUI styles (4 constants)
- [x] Session 1: Audit and fix hardware config propagation to all pages
- [x] Session 2: Add jog step verification display
- [x] Session 2: Add config file browser to Hardware Setup
- [x] Session 3: Implement pump-ink assignment (exclusive)
- [x] Session 3: Implement needle-channel-pump mapping
- [x] Session 3: Update HardwareConfig serialization (v7.2.4)
- [x] Session 4: Create WellPreviewWidget (zoomable, pannable)
- [x] Session 4: Replace XZ/ZY projections with XY-only preview
- [x] Session 4: Add out-of-bounds detection with red flash
- [x] Session 5: Add print plan UI + validation
- [x] Session 5: Wire "Send to Monitor" validation gate
- [x] Session 6: Integration testing (60 tests, 0 failures)
- [x] Hotfixes: auto-save crash, tuple access, plan auto-generation
- [x] Create README_V724.md

## Files Modified

| File | Change |
|------|--------|
| `gui/styles.py` | 4 centralized style constants |
| `gui/app.py` | Config propagation audit with logging |
| `gui/pages/calibration.py` | `set_hardware_config()` — plate sync, WellPlate rebuild |
| `gui/pages/print_well_setup.py` | Ink/rosette refresh, plan UI, validation gate, projection removal |
| `gui/pages/print_objects.py` | WellPreviewWidget, OOB detection, auto-save fix |
| `gui/pages/print_setup.py` | Config forwarding to sub-tabs |
| `gui/pages/jog_control.py` | Step verification display, conversion factor warning |
| `gui/pages/hardware_setup.py` | Config file browser, UI reorder, channel map UI |
| `gui/widgets/well_preview.py` | **New** — zoomable XY preview with OOB detection |
| `SupportClasses/HardwareConfig.py` | pump_ink_map, channel mapping, validation, v7.2.4 serialization |

## Testing Notes

1. Change hardware config on Page 0 → verify propagation to calibration (plate), well setup (inks), print objects (ink combo)
2. Jog XY → context panel shows requested vs measured delta
3. Hardware Setup → context panel shows saved configs, double-click to load
4. Configure pumps with inks → verify exclusive assignment (each ink to one pump)
5. Set up needle channels → verify channel-pump mapping persists
6. Print Objects → zoom/pan preview → move object outside well → verify red flash
7. Well Setup → assign wells → generate plan → verify validation passes → send to monitor
8. 60 integration tests pass

## Issues & Decisions

- **Exclusive ink assignment**: Each ink can only be assigned to one pump. Prevents ambiguous routing during print execution.
- **Channel map rebuild**: Channel map UI dynamically rebuilds when needle channel count changes. Validates completeness before allowing print generation.
- **OOB flash timer**: 500ms flash cycle for out-of-bounds objects. Visual without being disruptive.
- **Validation gate**: "Send to Monitor" is blocked until all validation checks pass. Prevents partial/invalid jobs from reaching the execution engine.

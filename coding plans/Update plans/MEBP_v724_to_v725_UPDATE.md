# MEBP v7.2.4 → v7.2.5 Update Plan

## Objective

QoL improvements: fix config load pump restoration, XY motion unit fix (Prior expects µm not microsteps), pump µL jog controls, print generation button, print file list, well setup redesign with role-based workflow, and wellplate zoom.

---

## Bug Fixes

### BF-1 — Loading saved HW config doesn't update pump/needle assignment (CRITICAL)

**Files:** `gui/pages/hardware_setup.py`

**Symptom:** Loading a saved hardware config JSON didn't update pump widgets or auto-populate the needle-pump assignment.

**Root cause:** In `_apply_config_to_ui()`, pump widget `set_config()` was called before the ink library was fully refreshed. Ink names in the saved config couldn't be resolved, causing silent failures. Channel map restore ran before pump enable states were committed.

**Fix:** Restructured restore to enforce dependency order: ink library → pumps (with ink_names passed) → pump-ink exclusions → channel map → final signal emit. Added defensive check in `PumpChannelWidget.set_config()` for missing ink names.

**Status:** `[x]` done

---

### BF-2 — XY jog moves wrong distance (CRITICAL)

**Files:** `gui/pages/jog_control.py`, `SupportClasses/StageController.py`, `gui/app.py`

**Symptom:** User sets XY step to 50µm, stage moves 42.2µm Y and 39.5µm X (~80-85% of expected).

**Root cause:** Prior ProScan II/III controllers report positions in µm natively, but code multiplied by `microsteps_per_micron` (10.0) before sending commands. The actual hardware resolution was ~11.86 counts/µm, so 500 counts / 11.86 = 42.2µm.

**Fix:** Added `move_xy_relative_um()` to StageController that sends µm values directly to the Prior controller. Jog page sends µm without conversion. Position display reads raw values (already in µm). Added `_xy_units` attribute loaded from protocol.

**Status:** `[x]` done

---

### BF-3 — Hardware communication: ProScan detection and CR terminator (Hotfix)

**Files:** `config/controllers/proscan_ii.json`, `SupportClasses/XYStage.py`, `SupportClasses/XYStageSimulator.py`

**Symptom:** ProScan II not detected. Position polling limited to 1 Hz.

**Root cause:** Detection tokens `["ProScan","II"]` failed because real hardware responds with "E,4" to "V" command. Also, `readline()` blocked on ProScan II's CR terminator (not CRLF).

**Fix:** Use COMP as detection command. Replace `readline()` with `read_until(b'\r')` + 0.1s timeout. Enables ~83 Hz polling. Updated simulator timing from `proscan_hw_profile.json`.

**Status:** `[x]` done

---

## Features

### F-1 — Pump µL Jog Controls

**Files:** `gui/pages/jog_control.py`

**Description:** Pump axis inactive if no pump designated in HardwareConfig. When active, movements in µL (not mm) with configurable step sizes. Enable/disable per pump based on hardware config.

**Status:** `[x]` done

---

### F-2 — Generate Print Button

**Files:** `gui/pages/print_setup.py`

**Description:** "Generate Print" button on Print Settings tab. Validates configuration, generates execution plan, shows time estimate. Required before "Send to Monitor" is available.

**Status:** `[x]` done

---

### F-3 — Print File List & "Send to Available Prints"

**Files:** `gui/pages/print_objects.py`, `gui/pages/print_well_setup.py`, `gui/pages/print_setup.py`

**Description:** "Send to Available Prints" button on Print Objects tab populates well setup dropdown. Prints list with `prints_changed` signal wiring. Ink combo format shows "(InkName (P1))" to indicate pump assignment. Rosette combos populated from HW library.

**Status:** `[x]` done

---

### F-4 — Well Setup Redesign (Role-Based Workflow)

**Files:** `gui/pages/print_well_setup.py`

**Description:** Complete redesign of well setup with role-based workflow:
1. Select wells on plate view (click/drag)
2. Assign role (Print / Waste / Wash / Buffer / Ink)
3. Role-specific options appear (ink selection, rosette, print file, etc.)
4. Color-coded wells: ready (green border), incomplete (yellow), empty (gray)

**Layout:** Split layout with plate view LEFT, assignment summary RIGHT. Role assignment bar with role buttons. Role-specific options stack.

**Status:** `[x]` done

---

### F-5 — Wellplate Zoom Controls

**Files:** `gui/pages/print_well_setup.py`

**Description:** Home/Zoom+/Zoom- overlay buttons on the wellplate view for easier navigation of small wells on large plates.

**Status:** `[x]` done

---

### F-6 — Print Results Page

**Files:** `gui/pages/print_results.py`, `gui/app.py`

**Description:** New Print Results page (Page 6) for viewing completed print records. Settings page shifted to Page 7. Sidebar menu and button map updated.

**Status:** `[x]` done

---

### F-7 — µm Display Conversions

**Files:** `gui/pages/calibration.py`, `gui/pages/settings_page.py`, `gui/pages/print_monitor.py`, `gui/pages/print_setup.py`

**Description:** Position display conversions from microsteps to micrometers throughout the GUI. Taught points display in µm. Safety limits display in µm. Settings spinboxes show µm/µsteps units.

**Status:** `[x]` done

---

## Implementation Steps

- [x] Session 1: Fix config load pump restoration + XY motion unit fix
- [x] Session 2: Pump µL jog controls + Generate Print button
- [x] Session 3: Print file list + "Send to Available Prints" + signal wiring
- [x] Session 3 Fix: Verify multi-file validation + regex fixes
- [x] Session 4: Well setup redesign (role-based workflow + zoom)
- [x] Session 5: Well border colors + integration wiring
- [x] Session 6: Layout fixes (pump headers µL units, side-by-side summary)
- [x] Hotfix: ProScan detection + CR terminator fix
- [x] µm display conversions across all pages
- [x] Windows console encoding compatibility fix

## Files Modified

| File | Change |
|------|--------|
| `gui/pages/hardware_setup.py` | Config load restore order fix |
| `gui/pages/jog_control.py` | µm-direct XY, pump µL controls |
| `gui/pages/print_setup.py` | Generate Print button, print forwarding |
| `gui/pages/print_objects.py` | "Send to Available Prints", ink combo format |
| `gui/pages/print_well_setup.py` | Complete redesign: role workflow, zoom, colors |
| `gui/pages/print_results.py` | **New** — Print Results page |
| `gui/pages/calibration.py` | µm display conversion |
| `gui/pages/settings_page.py` | µm spinboxes, safety limits display |
| `gui/pages/print_monitor.py` | µm display, set_microsteps_per_micron |
| `gui/app.py` | Page registration (results page), µm propagation |
| `SupportClasses/StageController.py` | move_xy_relative_um(), µm API |
| `SupportClasses/XYStage.py` | CR terminator fix, read_until() |
| `SupportClasses/XYStageSimulator.py` | Timing from proscan_hw_profile.json |
| `config/controllers/proscan_ii.json` | COMP detection command |

## Testing Notes

1. Load saved HW config → verify pumps + channel map restore correctly
2. Jog XY 50µm → verify stage moves exactly 50µm (not 42µm)
3. Pump jog → verify movements in µL, inactive when no pump configured
4. Generate Print → verify plan summary appears
5. Print Objects → "Send to Available Prints" → well setup dropdown populated
6. Well setup → select wells → assign roles → verify color coding
7. Zoom in/out on plate view with overlay buttons

## Issues & Decisions

- **µm-direct for Prior**: ProScan reports µm natively. Sending µm directly eliminates the microstep conversion layer for Prior controllers. The `microsteps_per_micron` factor retained for potential non-Prior stages.
- **Role-based well setup**: Cleaner than the previous flat assignment list. Each role has specific options (ink wells need ink selection, print wells need print file, etc.).
- **COMP for ProScan II detection**: The `V` (version) command returns "E,4" error on ProScan II. COMP (compatibility) command reliably identifies both II and III models.
- **Windows encoding**: Added `sys.stdout.reconfigure(encoding='utf-8', errors='replace')` to handle Unicode box-drawing characters on Windows cp1252 consoles.

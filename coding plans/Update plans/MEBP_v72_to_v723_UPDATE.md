# MEBP v7.2.0 → v7.2.3 Update Plan

## Objective

UI/workflow overhaul: fix Hardware Setup auto-load and restore ordering, restructure Print Setup workspace to read-only hardware summary, move execution controls to Print Monitor, and redesign Print Objects with persistent file-based workflow.

---

## Features

### F-1 — Hardware Setup Auto-Load & Restore Fix

**Files:** `gui/pages/hardware_setup.py`, `gui/app.py`

**Description:** Overhauled `_apply_config_to_ui()` to restore fields in dependency order, fixing silent failures when loading saved configs.

**Restore order:**
1. Config name + notes
2. Ink library FIRST → `_refresh_ink_table()` + `_refresh_pump_ink_combos()`
3. Rosette library → `_refresh_rosette_table()`
4. Needle gauge + length + channels
5. Plate format
6. Each pump: enable → syringe → ink (now resolvable) → mode
7. Emit signals after full restore

**Also added:** Rosette library UI section (table + Add/Edit/Delete), reordered UI (Name → Needle → Plate → Ink Library → Pumps → Rosettes).

**Status:** `[x]` done

---

### F-2 — Workspace → Read-Only Hardware Summary

**Files:** `gui/pages/print_setup.py`

**Description:** Replaced all editable workspace controls with read-only labels showing HardwareConfig state. Added "Edit Hardware Setup ▶" button to navigate to Page 0. Bridge method builds WorkspaceConfig FROM HardwareConfig. Print settings moved to context panel only.

**Status:** `[x]` done

---

### F-3 — Execution Controls Moved to Print Monitor

**Files:** `gui/pages/print_setup.py`, `gui/pages/print_monitor.py`, `gui/app.py`

**Description:** Stripped execution controls from Print Setup. Print Monitor now owns Start/Pause/Abort buttons, progress bar, print queue, and job summary. PrintManager owned by `app.py`.

**Job handoff:** `job_ready(PrintJob)` signal → `app.py` → Monitor → Start.

**Status:** `[x]` done

---

### F-4 — Print Objects File-Centric Workflow

**Files:** `gui/pages/print_objects.py`

**Description:** Redesigned Print Objects tab with persistent JSON print files in `config/prints/`.

**New workflow:**
1. `[+ New Print]` → name → creates JSON file, resets workspace
2. Load existing prints from `config/prints/` dropdown
3. Add objects: type icons → parameters → live preview → "Add Object"
4. Auto-layout: ring/grid/hex/line arrangement of N objects
5. Auto-save on every change
6. Assignment to wells handled separately in Well Setup (Tab 3)

**Print file schema:** name, objects list (type, params, ink/pump, position), auto-layout config.

**Status:** `[x]` done

---

## Implementation Steps

- [x] Fix Hardware Setup `_apply_config_to_ui()` restore ordering
- [x] Add auto-load on startup (last used config)
- [x] Reorder Hardware Setup UI sections
- [x] Add Rosette Library UI section
- [x] Convert Workspace tab to read-only hardware summary
- [x] Move print settings to context panel only
- [x] Strip execution controls from Print Setup
- [x] Add Start/Pause/Abort + queue to Print Monitor
- [x] Wire job handoff signal chain (print_setup → app → monitor)
- [x] Create print file JSON schema
- [x] Implement file-centric Print Objects workflow
- [x] Add auto-layout (ring/grid/hex/line patterns)
- [x] Integration testing (52 tests)
- [x] Create README_V723.md

## Files Modified

| File | Change |
|------|--------|
| `gui/pages/hardware_setup.py` | Auto-load, restore ordering, rosette library, UI reorder |
| `gui/pages/print_setup.py` | Read-only workspace, stripped execution controls |
| `gui/pages/print_monitor.py` | Start/Pause/Abort, progress bar, print queue |
| `gui/pages/print_objects.py` | File-centric workflow, auto-layout, auto-save |
| `gui/app.py` | Job handoff signal wiring, PrintManager ownership |

## Testing Notes

1. Launch app → verify last-used hardware config auto-loads
2. Load a saved config → verify all fields restore correctly (especially ink library before pumps)
3. Print Setup workspace shows read-only hardware summary
4. "Edit Hardware Setup ▶" navigates to Page 0
5. Create new print file → add objects → close and reopen → verify persistence
6. Print Monitor has Start/Pause/Abort controls
7. "Send to Monitor" from Print Setup delivers job to Monitor page
8. 52 integration tests pass (TestCrossSessionDataFlow, TestEndToEndWorkflow, etc.)

## Issues & Decisions

- **Restore ordering**: Ink library must be populated BEFORE pump widgets attempt to resolve ink names. Without this, combo boxes can't find the ink and silently default to empty.
- **Workspace as read-only**: Eliminated duplicate state. HardwareConfig is the single source of truth; workspace just displays it.
- **PrintManager in app.py**: Keeps execution engine at the application level, accessible by both setup and monitor pages. Prevents circular dependencies.
- **File-centric print objects**: Persistent JSON files in `config/prints/` survive app restarts. Auto-save prevents lost work. Well assignment separated from object design for cleaner workflow.

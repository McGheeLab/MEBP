# MEBP v7.2.3 — UI/Workflow Overhaul Update Plan (Revised)

## Version: 7.2.3 | Date: March 2026
## Scope: Hardware Setup fixes, Print Setup restructure, Print Monitor integration, Print Objects workflow redesign

---

## 1. Executive Summary

| # | Issue | Pages Affected |
|---|-------|----------------|
| 1 | HW Setup: no auto-load, load skips pumps/syringes/inks, ink library below pumps, missing rosette library | `hardware_setup.py`, `app.py` |
| 2 | Print Setup Workspace disconnected from HardwareConfig; print settings duplicated | `print_setup.py`, `print_workspace.py` |
| 3 | Execution controls + queue in Print Setup instead of Print Monitor | `print_setup.py`, `print_monitor.py`, `app.py` |
| 4 | Print Objects: no persistent files, clunky multi-step creation, no auto-layout | `print_objects.py` |

---

## 2. Current State Analysis

### 2a. Hardware Setup (Page 0)
**Works**: Needle/plate/pump selectors, ink library CRUD, save/load JSON, signals
**Broken**: `_apply_config_to_ui()` doesn't restore pump ink assignments (ink library not populated before combos resolve). Ink library below pumps. No rosette library UI.

### 2b. Print Setup (Page 4)  
**Works**: 3-tab structure, cross-tab signals, `set_hardware_config()` forwarding
**Broken**: `WorkspaceTab` has its own `WorkspaceConfig` disconnected from `HardwareConfig`. Print settings duplicated in workspace AND context panel. Execution controls here instead of Monitor.

### 2c. Print Monitor (Page 5)
**Works**: Plate overview, trajectory view, syringe display, recording browser
**Missing**: No Start/queue/progress. Pause/Abort via indirect signals only.

### 2d. Print Objects (Tab 2)
**Works**: Object types, params, library, collections, projection canvas, CSV import, drag-drop
**Clunky**: Multi-step creation (type→params→library→collection), no persistent files, no auto-layout, no auto-save, assignment mixed with design.

---

## 3. Planned Changes

### 3.1 Hardware Setup Fixes

#### 3.1.1 Auto-Load + File-Load Fix
Overhaul `_apply_config_to_ui()` to restore in dependency order:
1. Config name + notes
2. Ink library FIRST → `_refresh_ink_table()` + `_refresh_pump_ink_combos()`
3. Rosette library → `_refresh_rosette_table()`
4. Needle gauge + length + channels
5. Plate format
6. Each pump: enable → syringe → ink (now resolvable) → mode
7. Emit signals after full restore

#### 3.1.2 Reorder UI
New order: Name → Needle → Plate → **Ink Library** (up) → **Pumps** (down) → **Rosettes** (new)

#### 3.1.3 Add Rosette Library Section
Table (Name, Sub-wells, Fits, Depth, Z-offset) + Add/Edit/Delete. Import `RosetteEditorDialog`.

---

### 3.2 Workspace → Read-Only Hardware Summary

#### 3.2.1 Read-Only Summary
Replace all editable controls with labels showing HardwareConfig state. "Edit Hardware Setup ▶" button navigates to Page 0. Bridge method builds `WorkspaceConfig` FROM `HardwareConfig`, emits `workspace_changed`.

#### 3.2.2 Print Settings in Context Panel Only
Remove settings from workspace tab. Context panel gets grouped per-pump controls.

---

### 3.3 Move Execution to Print Monitor

#### 3.3.1 Strip from Print Setup
Replace execution controls with: job summary + "Send to Monitor ▶" + save/load.

#### 3.3.2 Add to Print Monitor
Start/Pause/Abort + progress bar + print queue + job summary. `PrintManager` owned by `app.py`.

#### 3.3.3 Job Handoff
`job_ready(PrintJob)` signal → `app.py` → Monitor → Start.

---

### 3.4 Print Objects — File-Centric Workflow Redesign

#### 3.4.0 New Concept: Print Files
A **Print File** is a persistent JSON in `config/prints/` containing print name, objects list (type, params, ink/pump, position), and auto-layout config. Print files bridge Tab 2 (design) and Tab 3 (assign to wells).

#### 3.4.1 New Workflow
```
1. [+ New Print] → name it → creates file, resets workspace
2. Load existing prints from config/prints/ dropdown
3. Add Objects: type icons → parameters → live preview → "Add Object"
4. Auto-Layout: arrange N objects in ring/grid/hex/line patterns
5. Auto-save on every change
6. NO "Add to Print" button — assignment to wells is in Tab 3
```

#### 3.4.2 Print File JSON Schema
```json
{
  "version": "7.2.3",
  "print_name": "Scaffold_v1",
  "description": "",
  "created": "2026-03-02T10:30:00Z",
  "modified": "2026-03-02T11:15:00Z",
  "objects": [
    {
      "name": "Base Scaffold",
      "object_type": "cylinder_solid",
      "params": {"radius": 2.0, "height": 0.6, "layer_height": 0.2},
      "position": [0.0, 0.0, 0.0],
      "ink_assignments": {"P1": "Hydrogel A"},
      "color": "#a6e3a1",
      "num_layers": 3,
      "auto_layout": false
    }
  ],
  "auto_layout_config": null,
  "hardware_requirements": {
    "min_pumps": 2,
    "inks_used": ["Hydrogel A", "MSC Cells"],
    "needle_gauge_max": 22
  }
}
```

#### 3.4.3 New UI Layout
```
┌─────────────────────────────────────────────────────────────────┐
│ [+ New Print]  Name: [Scaffold_v1]  │ Saved: [▾] [Load][Dup]  │
│ [Delete] [Export]                    │ Status: ✅ Auto-saved    │
├──────────────────────┬──────────────────────────────────────────┤
│ Object Designer      │ Well Preview (L-shaped projection)       │
│                      │ ┌──────────────────────┬────────┐       │
│ Type Icons:          │ │                      │        │       │
│ [●][╱][◯][◎][▦][📄] │ │   XY Top-Down        │ ZY     │       │
│                      │ │   (large)            │ Side   │       │
│ Parameters:          │ │   Objects shown at   │        │       │
│ ┌──────────────────┐ │ │   positions, drag to │        │       │
│ │ Radius: [2.0] mm │ │ │   reposition         │        │       │
│ │ Height: [0.6] mm │ │ ├──────────────────────┴────────┤       │
│ │ Ink: [P1: Hydro] │ │ │ XZ Bottom View               │       │
│ │ Color: [■]       │ │ └───────────────────────────────┘       │
│ │ [Add Object ▼]   │ │                                         │
│ └──────────────────┘ │ [▶ Simulate]  Speed: [═══●════]        │
│                      ├─────────────────────────────────────────┤
│ Auto-Layout:         │ Objects in This Print:                   │
│ Pattern: [Ring ▾]    │ ┌─────────────────────────────────────┐ │
│ Count: [6]           │ │ 1. ● Base Scaffold P1 @ (0,0,0)    │ │
│ Radius: [2.0] mm    │ │ 2. ◯ Cell Ring P2 @ (0,0,0.65)     │ │
│ [Apply] [Clear]      │ │ [▲][▼][Edit][Dup][✕]               │ │
│                      │ └─────────────────────────────────────┘ │
│ [📂 Import CSV]      │ Summary: 3 obj | P1,P2 | ~45s | 2.3µL │
└──────────────────────┴─────────────────────────────────────────┘
```

#### 3.4.4 "New Print" Behavior
1. Prompt for name (default: "Print_001" auto-incrementing)
2. Create empty JSON in `config/prints/`
3. Reset workspace: clear objects, preview, auto-layout
4. Begin auto-save (debounced 500ms)

#### 3.4.5 Saved Prints Manager
- Dropdown of all `.json` in `config/prints/`
- Load / Duplicate / Delete / Export buttons
- Last active print remembered in `settings.json`

#### 3.4.6 Object Designer — Streamlined
- Type icon buttons (not dropdown): Dot ● Line ╱ Circle ◯ Spiral ◎ Grid ▦ CSV 📄
- Dynamic parameter panel per type
- Ink/pump integrated in panel
- Live preview (100ms debounce via `generate_object_trajectory()`)
- "Add Object" → appends to print → auto-save → preview updates
- Auto-naming: "Dot_1", "Circle_2", etc.

#### 3.4.7 Auto-Layout System (NEW)
| Pattern | Params |
|---------|--------|
| Ring | Count, Radius, Start Angle |
| Square Grid | Rows, Cols, Spacing |
| Hex Grid | Rows, Cols, Spacing |
| Line | Count, Start(x,y), End(x,y) |
| Concentric Rings | Ring count, Inner/Outer R |

Implementation: layout generators return `list[tuple[float, float]]` positions.
"Apply Layout" → N objects created with computed positions, tagged `auto_layout: true`.
"Clear Layout" → removes all tagged objects.

```python
def auto_layout_ring(count, radius_mm, start_angle_deg=0.0, center=(0,0)):
    return [(center[0] + radius_mm * cos(radians(start_angle_deg + i*360/count)),
             center[1] + radius_mm * sin(radians(start_angle_deg + i*360/count)))
            for i in range(count)]

def auto_layout_grid(rows, cols, spacing_mm, center=(0,0)):
    x0, y0 = center[0]-(cols-1)*spacing_mm/2, center[1]-(rows-1)*spacing_mm/2
    return [(x0+c*spacing_mm, y0+r*spacing_mm) for r in range(rows) for c in range(cols)]

def auto_layout_hex(rows, cols, spacing_mm, center=(0,0)):
    rh = spacing_mm * sqrt(3)/2
    x0, y0 = center[0]-(cols-1)*spacing_mm/2, center[1]-(rows-1)*rh/2
    return [(x0+c*spacing_mm+(spacing_mm/2 if r%2 else 0), y0+r*rh)
            for r in range(rows) for c in range(cols)]
```

#### 3.4.8 Objects List (Replaces Library + Collection)
Single list of all objects in current print. Select → load into designer for editing. Edit modifies in-place. Reorder by drag. Remove/Duplicate buttons. Auto-layout objects clearable as group.

#### 3.4.9 Auto-Save
Debounced 500ms on every change. Status: "✅ Saved" / "⏳ Saving...". Last active print in `settings.json`.

#### 3.4.10 Integration with Tab 3
`prints_changed` signal (replaces `collections_changed`) emits list of print file names. Tab 3's "Assign Print" dropdown populated from this.

---

## 4. Compatibility Matrix

| Interface | Status |
|-----------|--------|
| `HardwareConfig` serialization | ✅ No changes |
| `PrintManager` / `PrintQueue` API | ✅ Same, ownership moved |
| `WorkspaceConfig` | ⚠️ Built FROM HardwareConfig |
| `PrintSettings` dataclass | ✅ Same fields |
| `set_hardware_config()` on pages | ✅ Same interface |
| `PrintObject` / `PrintCollection` | ✅ Used internally |
| `generate_object_trajectory()` | ✅ Called by designer |
| `StageController` API | ✅ No changes |
| Print file JSON | 🆕 New v7.2.3 schema |

---

## 5. Session Breakdown

### Session 1: Hardware Setup Fixes (3-4 hrs)

- [ ] S1.1: Reorder `_setup_ui()`: Name → Needle → Plate → Ink Library → Pumps → Rosettes
- [ ] S1.2: Build rosette library UI (table + CRUD buttons + RosetteEditorDialog)
- [ ] S1.3: Overhaul `_apply_config_to_ui()` — dependency-ordered restore
- [ ] S1.4: Fix `PumpChannelWidget.set_config()` ink resolution
- [ ] S1.5: Test auto-load + file-load full restore

**Output**: `gui/pages/hardware_setup.py`

### Session 2: Workspace Read-Only + Settings (4-5 hrs)

- [ ] S2.1: Create `HardwareSummaryWidget` (read-only labels for all hardware)
- [ ] S2.2: Replace `WorkspaceTab` with summary + "Edit Hardware Setup" button
- [ ] S2.3: Implement `_hardware_config_to_workspace()` bridge
- [ ] S2.4: Enhance context panel: grouped per-pump settings
- [ ] S2.5: Remove execution controls from context (prep for S3)
- [ ] S2.6: Test workspace auto-updates from hardware changes

**Output**: `gui/pages/print_workspace.py`, `gui/pages/print_setup.py`

### Session 3: Move Execution to Monitor (5-6 hrs)

- [ ] S3.1: Add `job_ready` signal + "Send to Monitor ▶" to Print Setup
- [ ] S3.2: Remove PrintManager/Queue from Print Setup
- [ ] S3.3: Add Start/Pause/Abort + progress + queue to Print Monitor
- [ ] S3.4: Wire `app.py`: PrintManager ownership, job handoff, signal routing
- [ ] S3.5: Verify recording still works
- [ ] S3.6: Test full execution flow through Monitor

**Output**: `gui/pages/print_setup.py`, `gui/pages/print_monitor.py`, `gui/app.py`

### Session 4A: Print File Infrastructure (3-4 hrs)

- [ ] S4A.1: Create `PrintFileManager` (list/load/save/delete/duplicate prints)
- [ ] S4A.2: Print file JSON schema v7.2.3
- [ ] S4A.3: Build Print File Bar UI (New/Load/Dup/Delete + name + auto-save status)
- [ ] S4A.4: New main layout (splitter: designer|preview|objects list)
- [ ] S4A.5: "New Print" resets workspace, creates file
- [ ] S4A.6: Load/Duplicate/Delete implementations
- [ ] S4A.7: Auto-save with 500ms debounce
- [ ] S4A.8: Last active print in settings.json
- [ ] S4A.9: Replace `collections_changed` with `prints_changed` signal

**Output**: `gui/pages/print_objects.py` (infrastructure)

### Session 4B: Designer + Auto-Layout (3-4 hrs)

- [ ] S4B.1: Object type icon button row (QButtonGroup, 6 types)
- [ ] S4B.2: Dynamic parameters panel (QStackedWidget per type)
- [ ] S4B.3: Live preview (100ms debounce → generate_object_trajectory)
- [ ] S4B.4: "Add Object" → append to print → auto-save
- [ ] S4B.5: Objects list (select/edit/reorder/remove/duplicate)
- [ ] S4B.6: Edit-in-place (select → load params → "Update Object")
- [ ] S4B.7: Auto-layout UI (pattern selector + params + Apply/Clear)
- [ ] S4B.8: Layout generators (ring, grid, hex, line, concentric)
- [ ] S4B.9: Drag-to-reposition in XY preview
- [ ] S4B.10: CSV Import section
- [ ] S4B.11: Print summary widget
- [ ] S4B.12: Wire `prints_changed` signal to Tab 3

**Output**: `gui/pages/print_objects.py` (complete rewrite)

### Session 5: Integration Testing (2-3 hrs)

- [ ] S5.1: Full end-to-end workflow test
- [ ] S5.2: Hardware persistence across restarts
- [ ] S5.3: Page gating verification
- [ ] S5.4: Print recording verification
- [ ] S5.5: Edge cases (empty config, mid-session changes, deleted resources)
- [ ] S5.6: Run test suite
- [ ] S5.7: README_V723.md + sample print files

**Output**: Tests, docs, `config/prints/` samples

---

## 6. File Change Summary

| File | Session | Action | Lines |
|------|---------|--------|-------|
| `gui/pages/hardware_setup.py` | S1 | MODIFY | +150 |
| `gui/pages/print_workspace.py` | S2 | REWRITE | +200/-400 |
| `gui/pages/print_setup.py` | S2,S3 | MODIFY | +80/-200 |
| `gui/pages/print_monitor.py` | S3 | MODIFY | +300 |
| `gui/app.py` | S3 | MODIFY | +80 |
| `gui/pages/print_objects.py` | S4A,S4B | REWRITE | +800/-500 |
| `config/prints/` | S4A | NEW dir | — |
| `README_V723.md` | S5 | NEW | +120 |

**Total: ~1,730 added, ~1,100 removed, ~2,800 touched**

---

## 7. Schedule

| Session | Hours | Priority | Depends On |
|---------|-------|----------|------------|
| S1 | 3-4 | Critical | — |
| S2 | 4-5 | Critical | S1 |
| S3 | 5-6 | High | S2 |
| S4A | 3-4 | High | S2 |
| S4B | 3-4 | High | S4A |
| S5 | 2-3 | Critical | S3+S4B |

**Total: 20-26 hours across 6 sub-sessions**
S3 and S4A can run in parallel (different files).

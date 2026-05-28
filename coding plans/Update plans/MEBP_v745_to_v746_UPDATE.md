# MEBP v7.4.5 → v7.4.6 Update Plan

## Objective

Phase 2 of the parametric well-plate designer. v7.4.5 shipped the inline
sketcher (Select / Single Well / Grid Pattern tools + 8 base constraint
kinds). v7.4.6 lifts it from "demo" to "shipping feature":

1. **End-to-end propagation of custom plates** — Calibration, Jog, Print
   Setup, and Print execution all treat saved custom designs identically
   to built-in 96-well plates via `active_plate_key` / `WellPlate.load`.
2. **Group editing** — selecting any well within a grid or ring exposes
   the underlying pattern's parameters with an Apply button that
   rebuilds the group in place.
3. **Circle Pattern + Line + Construction Line tools** — drop a ring of
   N wells; place two-click reference geometry.
4. **Undo / Redo** — snapshot-based history (50-entry cap, lazy capture
   on first drag movement).
5. **Drag-preview overlay** while pattern tools are active.
6. **Additional constraint kinds** — point_on_line, parallel,
   perpendicular, equal_length, tangent_cc, symmetric_pp.
7. **Dimension markers** — lock glyphs on fixed wells, distance pills
   mid-segment on `distance_pp` constraints.
8. **Polish** — uniform 10-button toolbar with Phosphor SVG icons,
   selection-aware enable/disable, Line properties card.

Branch: continues on `Version-7.4.2`; bump to `Version-7.4.6` at
completion per CLAUDE.md checklist.

## Files Modified

| File | Rationale |
|------|-----------|
| `SupportClasses/PlateDesign.py` | New `add_circle_pattern(...)` factory; new `rebuild_group(group_id, new_params)` method for in-place pattern edits; `CONSTRAINT_KINDS` extended with `point_on_line`, `parallel`, `perpendicular`, `equal_length`, `tangent_cc`, `symmetric_pp`. |
| `SupportClasses/PlateSketchSolver.py` | New residuals for the six new constraint kinds. `_residual_count` updated for row accounting. |
| `SupportClasses/WellPlate.py` | `load(str)` strips a `"custom:"` prefix so round-trips like `WellPlate.load(plate.format)` work. |
| `SupportClasses/PhysicalModels.py` | `WorkspaceConfig.plate_name` field + `active_plate_key` property + JSON round-trip; `validate()` tolerates custom plate names. |
| `SupportClasses/WellSetup.py` | `WellSetupModel(plate_key: int \| str)` + `set_plate_format(key)` route through `WellPlate.load(...)`. |
| `gui/widgets/plate_designer_canvas.py` | `Tool` enum extended with `DRAW_CIRCLE_PATTERN`, `DRAW_LINE`, `DRAW_CONSTRUCTION_LINE`; new `LineItem(QGraphicsLineItem)`; `_handle_draw_line` / `_handle_draw_circle_pattern` handlers; snapshot-based undo/redo (`push_undo_snapshot` / `undo` / `redo` / `_restore_snapshot` + 50-entry cap); drag-preview overlay (`_update_*_preview` + `_clear_preview` + `leaveEvent`); `rebuild_group_now` / `add_constraint_explicit` public APIs; `_draw_constraint_markers` (lock glyphs + distance pills); `_entity_id_at` allows clicking Lines for selection. |
| `gui/pages/hardware/plate_designer.py` | Toolbar rebuilt with 10 uniform 44×44 QToolButtons + Phosphor SVG icons (cursor / circle-plus / grid / compass / line ×2 / lock / trash / undo / redo); selection-aware Lock/Delete; Save disabled on standards; Delete only enabled for custom plates; Line properties card; Group editor card (rows/cols/spacing for grids, count/radius for rings); Well+Line mixed-selection card with "+ Point on line" button; Tangent action in multi-well batch card; new keyboard shortcuts (S/W/G/C/L/Shift+L/K/Ctrl+Z/Ctrl+Shift+Z/Ctrl+Y). |
| `gui/widgets/icons.py` | New SVG icons: `cursor`, `grid`, `circle-plus`, `lock`, `compass`, `line`, `undo`, `redo`. |
| `gui/pages/calibration.py` | `set_hardware_config` reads `active_plate_key`, routes through `WellPlate.load`, derives rows/cols/corner_well from the loaded plate. |
| `gui/pages/print_workspace.py` | Bridge copies `plate_name` from HardwareConfig; plate-display panel renders custom plate dims + Ø range. |
| `gui/pages/print_well_setup.py` | `WellSetupModel` constructed from `workspace.active_plate_key`; `set_workspace` / `set_hardware_config` compare with `_same_plate_key` helper that normalizes int / "custom:foo" / "foo" encodings. |
| `tests/test_v745_canvas_history.py` (new in v7.4.5) | Layout-offset + undo/redo coverage. |
| `tests/test_v746_phase2.py` (new) | Group rebuild (grid + circle), constraint cleanup, line tool, construction-line endpoint fixing, point-on-line snap, parallel direction alignment, tangent_cc, equal_length, symmetric_pp, WorkspaceConfig round-trip. |
| `coding plans/Architectures/ARCHITECTURE_V745.md` (new) | v7.4.5 designer foundation delta against v742. |
| `coding plans/Architectures/ARCHITECTURE_V746.md` (new) | v7.4.6 Phase 2 delta against v745. |
| `CLAUDE.md` | Current-version bump to V7.4.6; v744→v745 and v745→v746 plans added to the *Existing Update Plans* table. |

## Implementation Steps

- [x] `WellSetup` + `WorkspaceConfig` + `print_well_setup` + `print_workspace` migrated to `active_plate_key`
- [x] `PlateDesign.rebuild_group` + group-editing card in the properties panel
- [x] Drag-preview overlay for Single Well / Grid / Circle Pattern tools
- [x] `Tool.DRAW_CIRCLE_PATTERN` + `_handle_draw_circle_pattern` + toolbar button
- [x] `Tool.DRAW_LINE` + `Tool.DRAW_CONSTRUCTION_LINE` + `LineItem` rendering + toolbar buttons
- [x] Solver residuals for `point_on_line` / `parallel` / `perpendicular` / `equal_length` / `tangent_cc` / `symmetric_pp`
- [x] Snapshot-based undo/redo (canvas push/undo/redo + lazy drag capture + 50-cap)
- [x] Dimension markers (`_draw_constraint_markers` — lock glyphs + distance pills)
- [x] Line properties card (length, endpoints, construction toggle, delete)
- [x] Tests: `tests/test_v746_phase2.py` (10 tests) + `tests/test_v745_canvas_history.py` (8 tests)
- [x] Architecture docs: ARCHITECTURE_V745.md + ARCHITECTURE_V746.md
- [x] CLAUDE.md current-version bump + update plans table
- [ ] Manual verification: full Plate sub-page golden path + downstream usage in Calibration / Print Setup
- [ ] Bump branch to `Version-7.4.6` (deferred until user signals "complete")

## Testing Notes

### Unit tests

```bash
QT_QPA_PLATFORM=offscreen python3 -m unittest \
  tests.test_v745_plate_design \
  tests.test_v745_plate_sketch_solver \
  tests.test_v745_canvas_history \
  tests.test_v746_phase2
```

All v745 + v746 tests green (42 tests). Full v730 + v74x + v744 + v745 +
v746 regression: **157 tests green**.

### Manual end-to-end (custom plate flowing through downstream pages)

```bash
python3 main.py
```

1. Hardware Setup → Plate sub-page → designer renders, picker shows 6 standards.
2. Pick `12-well` → 12 wells appear ANSI-positioned within the plate outline (A1 properly inset, not at the corner).
3. Switch to + Grid → click on the canvas → ghost preview appears → click to commit a new grid.
4. Click a well inside the grid → properties panel shows Well card + Group card (rows/cols/spacing/diameter spinboxes).
5. In Group card: change rows from 3 → 5, click Apply → wells refresh; status bar shows under-determined DOF.
6. Switch to + Ring (C) → click → 8 wells dropped on a 20mm radius.
7. Switch to Construction Line (Shift+L) → click two points → dashed teal line appears; endpoints rendered locked.
8. Ctrl-click both a well and the construction line → properties panel shows "+ Point on line" button → click → well snaps onto the line.
9. Ctrl+Z several times → previous states restore.
10. Save As "test-plate-v746" → file appears in `config/hardware/plates/user/`.
11. Navigate to Calibration page → log line "Calibration: plate synced to test-plate-v746 from HardwareConfig". A1 detection target uses custom plate's geometry.
12. Navigate to Print Setup → workspace panel shows the custom plate's row/col + Ø range.

## Issues & Decisions

- **Group editing is destructive.** `rebuild_group` removes the old member wells before recreating them, which drops any per-well constraints the user had added. This matches the SolidWorks-style "pattern feature regenerates" mental model. Users wanting to preserve constraints should avoid touching grid params.
- **Construction line endpoints fixed by default.** The line tool creates Points with `fixed=True` only when drawing a *construction* line; solid lines leave endpoints free. This makes construction lines act as immovable reference geometry, which is the user-expected SolidWorks behavior.
- **Tangent uses external tangency (d = r1+r2).** The implementation treats two Wells as externally-tangent circles. Internal tangency (d = |r1-r2|) was not added; would be a separate `tangent_cc_internal` kind if needed.
- **Symmetric requires 3 refs.** `symmetric_pp` takes `[p1_or_well, p2_or_well, axis_line]`. The axis must be a Line; the points/wells will be made reflections of each other across it. Two residuals: midpoint-on-line and `(p1-p2) ⟂ line`.
- **Snapshot-based undo/redo over QUndoStack.** Serializing `PlateDesign.to_dict()` was simpler than authoring per-action QUndoCommand subclasses, and stays correct across complex mutations like `rebuild_group` that touch many entities at once. Capacity (50) bounds memory; lazy capture during drag avoids polluting history with bare clicks.
- **Dimension markers auto-hide on dense plates.** When the design has >64 wells (96-well grids and beyond), constraint markers would clutter the canvas. The implementation skips marker drawing in that regime — the user can still see DOF status in the status bar.
- **Save/Save As semantics.** Save is disabled on bundled standards (use Save As to fork); enabled on custom plates only when dirty. Delete only available for custom plates. Matches user expectations for "save-edit-save" on documents.
- **Branch handling deferred.** Per CLAUDE.md's "Version Completion Checklist", the branch rename to `Version-7.4.6` happens after the user signals the update is complete. The current branch (`Version-7.4.2`) stages multiple in-progress versions (v7.4.3 / v7.4.4 / v7.4.5 / v7.4.6).

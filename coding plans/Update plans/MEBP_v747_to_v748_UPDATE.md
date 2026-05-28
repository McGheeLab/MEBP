# MEBP v7.4.7 → v7.4.8 Update Plan

## Objective

Make rosettes **first-class sub-wells** everywhere, plus add insert heights and standard inserts:

1. **Flatten rosettes into named sub-wells.** A well with a rosette is replaced at compile time by its sub-wells, named `A1.a`, `A1.b`, … Printing, ink assignment, and every workflow treat them as ordinary wells — no rosette-aware code downstream. Ink is assigned to `A1.a` in Print Setup exactly like any well.
2. **Insert heights.** Each sub-well (insert/tube) carries a **rim height** above the plate (e.g. Eppendorf tubes that stick up) and a prescribed **ink dispense Z**. Needle travel moves clear the tallest insert plate-wide; the dispense uses each sub-well's ink Z.
3. **Standard inserts.** A library of in-house insert designs that can be dropped into any well and **rotated** (rotation-only alignment) to match the physical part.

Branch: continues on `Version-7.4.2`; bump to `Version-7.4.8` at completion.

## Decisions (from user)

- Parent well with a rosette → **replaced by sub-wells only** (non-printable container; the parent is not a print/assignment target). Designer top view still shows the parent well with a rosette badge.
- Travel clearance → **plate-wide max-clearance floor** applied to every `safe_travel_to` (print, pick & place, jog). Each sub-well keeps its own dispense Z.
- Standard inserts → **rotation only** (no XY nudge), with a saved-insert library.

## Files Modified

| File | Rationale |
|------|-----------|
| `SupportClasses/WellPlate.py` | `WellInfo` gains `rim_height_mm`, `ink_z_mm`, `parent_well`, `is_subwell`. `WellPlate.max_rim_height_mm` property. |
| `SupportClasses/PlateDesign.py` | `Well` gains `rim_height_mm`, `ink_z_mm`, `rosette_rotation_deg`. `compile()` **flattens** each rosette well into sub-well `WellInfo`s named `<parent>.<sub>` at rotated offsets, inheriting parent row/col; parent itself is NOT emitted. Standard-insert library helpers (`save_insert`, `load_insert`, `list_inserts`). |
| `SupportClasses/StageController.py` | `_min_travel_z_mm` floor + `set_min_travel_z()`; `safe_travel_to` floors `safe_z_mm` against it so all callers clear tall inserts. |
| `SupportClasses/PrintManager.py` / `PrintTrajectoryPlanner.py` | Per-well ink dispense Z: use `WellInfo.ink_z_mm` (relative to plate top) when set, else the global `print_z_height`. |
| `SupportClasses/WellSetup.py` | Drop the now-dead `subwell_roles`/`subwell_inks`/`subwell_labels` lists + `attach_rosette` subwell init (rosettes are flattened, not attached). |
| `gui/app.py` (or controller hw-apply path) | Compute the clearance floor from the active plate's `max_rim_height_mm` + calibration top Z, push to `controller.set_min_travel_z`. |
| `gui/pages/hardware/plate_designer.py` | Rosette sub-well properties: rim-height + ink-Z spinboxes; parent-well rosette **rotation** field; "Save as standard insert" + "From standard insert…" in the rosette flow; default sub-well names = letters a/b/c. |
| `gui/widgets/plate_designer_canvas.py` | Rosette badge on parent wells that have a rosette (top view); letter naming for sub-wells in circular mode. |
| `tests/test_v748_*.py` | flatten naming/positions/rotation, heights carried to WellInfo, max_rim, travel-Z floor, dispense-Z, insert library round-trip. |
| `coding plans/Architectures/ARCHITECTURE_V748.md`, `CLAUDE.md` | version delta + bump. |

## Implementation Steps

- [x] Model: `WellInfo` + `Well` height fields; `Well.rosette_rotation_deg`; `WellPlate.max_rim_height_mm`
- [x] `compile()` flattening (sub-wells replace parent, rotation, heights, parent row/col, `<parent>.<sub>` names; case-insensitive keys)
- [x] Travel-Z floor in `StageController.safe_travel_to` + `set_min_travel_z()` + wiring in `app.py::_update_insert_clearance`
- [x] Per-well ink dispense Z in the trajectory planner (`_well_print_z`)
- [x] Designer UI: sub-well rim/ink-Z fields, rotation field, letter naming, rosette badge
- [x] Drill-in gesture: double-click a well → zoom into it and design sub-wells with the same designer chrome (`well_drill_requested` → `_on_edit_rosette`); tooltips hint it; empty drill-ins discard the rosette
- [x] Relocate the rosette designer to the **Rosette sub-page** (`PlateDesignerWidget(mode="rosette")`), separate from the layout-only **Plate sub-page** (`mode="plate"`). Both share one `PlateDesign`; `HardwareSetupPage` syncs on `sub_page_changed` (adopt on Rosette show, refresh on Plate show); rosette `save_requested`/`design_edited` route to the plate page. Circle Pattern / Grid in rosette mode re-letter sub-wells a, b, c… (`relabel_wells_as_letters`)
- [x] Interactive + configurable Circle Pattern: press snaps to centre → drag sets radius (live ring + "r = X.X mm" readout) → release places; "Circle Pattern options" card for **count / center well / well Ø**; `add_circle_pattern` / `rebuild_group` gained `center_well`
- [x] **Crash fix**: dimension distance lines no longer embed `QDoubleSpinBox` in the `QGraphicsScene` (segfaulted on key input). Dimensions render as read-only labels; editing moved to the properties panel (`set_constraint_value` for edge dims; group card for circle radius). Regression-guarded by `TestNoSceneEmbeddedInputWidgets`
- [x] Circle group options shown in the standard (no-selection) properties view after placement (not only when a well is clicked); **draggable radius endpoint** (`RadiusHandleItem`) resizes + rotates the pattern live (`PlateDesign.update_circle_layout`, reposition-in-place, names preserved)
- [x] Standard-insert library: save/load + drop-into-well picker (rotation-only)
- [~] `subwell_*` lists in `WellSetup`: left inert for back-compat (no longer populated since rosettes flatten — removing them would break deserialization of old saved setups for no functional gain)
- [x] Tests `tests/test_v748_rosette_flatten.py` (12 tests)
- [x] Docs (ARCHITECTURE_V748 + CLAUDE bump) + full regression (188 tests green)
- [ ] Manual launch verification blocked by an **unrelated** pre-existing broken edit in `gui/pages/print_objects.py` (undefined `_choose_csv_file` at line 925, 481 lines of uncommitted non-rosette edits in the working tree) — designer surface verified directly via `HardwareSetupPage`

## Testing Notes

```bash
QT_QPA_PLATFORM=offscreen python3 -m unittest tests.test_v748_rosette_flatten
QT_QPA_PLATFORM=offscreen python3 -m unittest \
  tests.test_v745_plate_design tests.test_v745_plate_sketch_solver \
  tests.test_v745_canvas_history tests.test_v746_phase2 \
  tests.test_v747_plate_designer  # regression
```

Manual (`python3 main.py`):
1. Design a 24-well plate, give A1 a "Ring of 6 + center" rosette with tube rim 30 mm + ink-Z; Save.
2. Print Setup → A1.a … A1.g appear as selectable wells; assign ink to A1.c; A1 itself is gone.
3. Run a sim print → travel Z clears 30 mm tube tops; dispense at A1.c uses its ink Z.
4. Save the rosette as a standard insert; drop it into B2 with rotation 30°; B2.a… positions rotate accordingly.

## Issues & Decisions

- **Flatten supersedes the v7.4.7 `RosetteInsert`-attach approach.** `compile()` no longer attaches a `RosetteInsert` to `WellInfo`; it emits the sub-wells directly. `RosetteInsert` remains only as a legacy/library convenience (the standard-insert library stores nested `PlateDesign`s).
- **Parent not emitted.** Per decision, a rosette well contributes only its sub-wells to the runtime `WellPlate`; nothing references the parent name downstream. The designer keeps the parent `Well` (with nested design) for editing + a top-view badge.
- **Naming.** Sub-well flattened name = `<parent>.<subwell.name>`; rosette sub-wells default to letters a, b, c… so the result is `A1.a`, `A1.b`. Row/col inherit the parent so row/column selection still groups them.
- **Heights.** `rim_height_mm` = mm the insert top sits ABOVE the plate top (clearance). `ink_z_mm` = dispense Z relative to plate top (None = use global print Z). Sign convention: larger Z = higher/further from plate.
- **Clearance floor is global + conservative.** `safe_travel_to` floors the retract Z at `plate_top + max_rim + margin`, so any path clears the tallest tube regardless of route. Per-well optimization deferred.
- **Standard inserts = saved nested designs.** A library under `config/hardware/inserts/`. Dropping one copies it into the well's `rosette_design` and records a rotation applied at flatten time. Rotation-only per decision.

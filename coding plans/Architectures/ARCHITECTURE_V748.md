# MEBP v7.4.8 — Architecture Reference (Delta)

**Multi-Extrusion Bioprinting Platform**
**Version 7.4.8 | May 2026**

> **Delta against [`ARCHITECTURE_V747.md`](ARCHITECTURE_V747.md).**
> Sections not mentioned here are unchanged.

---

## 1. What's New in v7.4.8

Rosettes become **first-class sub-wells** everywhere, gain **insert
heights**, and can be saved as reusable **standard inserts**:

1. **Flatten.** A well carrying a rosette is replaced at compile time by
   its sub-wells, named `A1.a`, `A1.b`, … Printing, ink assignment, and
   every workflow treat them as ordinary wells — no rosette-aware code
   downstream. The parent well is **not** a runtime target.
2. **Heights.** Each sub-well (insert/tube) carries a **rim height**
   above the plate (clearance) and a prescribed **ink dispense Z**.
   Needle travel clears the tallest insert plate-wide; print dispense
   uses each sub-well's ink Z.
3. **Standard inserts.** A library of saved insert designs that can be
   dropped into any well and **rotated** to match the physical part.

This **supersedes** the v7.4.7 design where `compile()` attached a
`RosetteInsert` to the parent `WellInfo`. `compile()` now emits sub-well
`WellInfo`s directly.

---

## 2. Backend Changes

### `SupportClasses/WellPlate.py`

```python
@dataclass
class WellInfo:
    ...
    rim_height_mm: float = 0.0       # insert/tube top ABOVE plate (clearance)
    ink_z_mm: float | None = None    # dispense Z rel. plate top; None = global
    is_subwell: bool = False         # flattened rosette sub-well
    parent_well: str | None = None   # e.g. "A1"

class WellPlate:
    @property
    def max_rim_height_mm(self) -> float: ...   # tallest insert across wells
```

`WellPlate.from_wells` now keys `_wells` by **upper-cased** name so
case-insensitive lookups resolve sub-wells like `A1.a` (the display name
on `WellInfo` is preserved).

### `SupportClasses/PlateDesign.py`

- `Well` gains `rosette_rotation_deg`, `rim_height_mm`, `ink_z_mm`
  (recursively serialized).
- **`compile()` flattens**: for each well with a non-empty
  `rosette_design`, emit one `WellInfo` per sub-well named
  `<parent>.<sub>`, positioned at the parent centre plus the sub-well
  offset rotated by `rosette_rotation_deg` (CCW-positive), inheriting the
  parent row/col, carrying `rim_height_mm` / `ink_z_mm`. The parent is
  dropped. An empty rosette degrades to an ordinary parent well.
- Standard-insert library: `INSERTS_DIR` +
  `list_standard_inserts()` / `load_standard_insert()` /
  `save_standard_insert()` (nested rosette `PlateDesign`s under
  `config/hardware/inserts/`).

### `SupportClasses/StageController.py`

- `_min_travel_z_mm` floor + `set_min_travel_z(z)`. `safe_travel_to`
  raises its `safe_z_mm` to at least this floor, so **every** safe travel
  (print, pick & place, jog) clears the tallest insert. Both values are
  zero-referenced mm; larger Z = higher.

### `SupportClasses/PrintTrajectoryPlanner.py`

- `_well_print_z(plate, name, settings)` returns `top_z + ink_z_mm` when
  the well prescribes an `ink_z_mm`, else the global `print_z_height`.
  `_travel_to_well` / `_lower_to_print` accept a per-well `print_z`.

### `gui/app.py`

- `INSERT_CLEARANCE_MARGIN_MM` + `_update_insert_clearance(cal_page)`:
  computes `plate_top_z + max_rim + margin` from the active plate +
  calibration and pushes it via `controller.set_min_travel_z`. Called
  whenever calibration data changes (`_push_cal_to_jog`).

---

## 3. UI Changes

### `gui/pages/hardware/plate_designer.py`

- Sub-well properties (rosette mode): **Rim height** + **Ink Z**
  (with an enable checkbox) spinboxes.
- Parent-well properties (top plate, has rosette): **Rosette rotation**
  spinbox; **Add/Replace from standard insert…** picker; **Save as
  standard insert…** in the breadcrumb bar.
- Rosette presets re-letter sub-wells a, b, c… (`_letter_name_subwells`)
  so flattened names read `A1.a`, `A1.b`. Free sub-wells placed with the
  Single Well tool in circular mode also get letter names.

### Plate sub-page vs Rosette sub-page (v7.4.8 relocation)

`PlateDesignerWidget(mode="plate" | "rosette")` now backs **two**
sub-pages:

- **Plate sub-page** (`mode="plate"`): plate layout only — wells, grids,
  constraints, dimensions. Double-click does nothing; the well card has
  no rosette controls.
- **Rosette sub-page** (`mode="rosette"`): mirrors the plate layout and is
  the home of the rosette designer. Plate-file chrome (picker / New /
  Save As / Delete / refresh) is hidden; a "Save plate" button delegates
  to the Plate page via `save_requested`. **Double-click a well → zoom in**
  (`well_drill_requested` → `_on_edit_rosette`) and design sub-wells with
  the same toolbar (Single Well / **Circle Pattern** for a circular
  layout / Grid / Lock / Undo …). The circular bore is the outline.

The two widgets **share one `PlateDesign` object**. `HardwareSetupPage`
syncs on `sub_page_changed`: showing the Rosette page calls
`_rosette_designer.adopt_design(_plate_designer.current_design())`;
returning to the Plate page calls `_plate_designer.refresh()` so rosette
badges appear. Rosette edits emit `design_edited`, which marks the plate
dirty; saving on either page persists the shared design (rosettes
included). New widget API: `current_design()`, `adopt_design()`,
`refresh()`.

### `gui/widgets/plate_designer_canvas.py`

- **Drill-in gesture**: `mouseDoubleClickEvent` on a top-level well emits
  `well_drill_requested(well_id)`. The widget connects it to
  `_on_edit_rosette` **only in `mode="rosette"`**, which zooms into the
  well (swaps to the circular nested design and fits the bore). Well
  tooltips hint "double-click to add/edit rosette".
- `_draw_rosette_badges`: a small ring of dots on wells that contain a
  rosette, so inserts are visible at a glance.
- Circular-mode naming: `_next_custom_name` yields letters; grid/circle
  placement + `rebuild_group_now` call `relabel_wells_as_letters()` so all
  sub-wells read a, b, c… and flatten to `A1.a`, `A1.b`.
- `_on_back_to_plate` discards an empty rosette so an accidental
  drill-in that places nothing leaves the well pristine.

### Circle Pattern tool — interactive + configurable (v7.4.8)

- **Press → drag → release**: `_circle_press` snaps the centre to the
  bore/plate centre by default (`_snap_circle_center`); dragging sets the
  radius live (`_draw_circle_drag_preview` shows the ghost ring + a radius
  dimension line + "r = X.X mm" readout); release `_commit_circle` places
  the ring. `_handle_draw_circle_pattern` remains as a click-to-drop
  convenience.
- **Options** via `_circle_params` (`set_circle_param`): **count**,
  **center well**, **well diameter**. Surfaced in a "Circle Pattern
  options" properties card while the tool is active
  (`_build_circle_options_card`).
- **Radius dimension + draggable handle**: placed circle groups render a
  radius line at the current start angle, a read-only "r X.XX" caption,
  and a **draggable endpoint** (`RadiusHandleItem`). Dragging it resizes
  (distance from centre → radius) **and** rotates (direction → start
  angle) the pattern live via `PlateDesign.update_circle_layout`
  (reposition-in-place, no rebuild, names preserved). Radius / count /
  start-angle / **center well** / diameter are also editable in the
  group's properties card.
- **Group options in the standard view**: after a circle pattern is laid
  down, its group card shows in the default (no-selection) properties
  panel — `_rebuild_properties_panel` lists a `_build_group_card` for each
  circle `Group`, so you don't have to click a well to tweak it.
  `_commit_circle` clears the selection so the card appears immediately.

### Crash fix — no input widgets embedded in the scene (v7.4.8)

Editable dimension fields were initially embedded in the `QGraphicsScene`
via `QGraphicsProxyWidget(QDoubleSpinBox)`. Typing into them **segfaulted**
(`QAbstractSpinBox::keyPressEvent` on a stale proxy, `EXC_BAD_ACCESS`).
Fixed by removing all scene-embedded input widgets:

- Edge-distance dimensions and the circle radius now render as **read-only
  `QGraphicsTextItem` captions** (`_add_dimension_label`, zoom-invariant).
- Editing happens in the **properties panel** (normal widget hierarchy):
  edge-distance values get an editable `QDoubleSpinBox` in the well's
  Constraints card → `PlateDesignerCanvas.set_constraint_value(cid, value)`;
  the circle radius is edited via the group card → `rebuild_group_now`.
- No `QGraphicsScene.addWidget` / proxy widgets remain in the canvas;
  `tests/test_v748_rosette_flatten.py::TestNoSceneEmbeddedInputWidgets`
  guards against regressions.
- `PlateDesign.add_circle_pattern` + `rebuild_group` gained a
  `center_well` parameter (the center well lists first so it letters as
  `a`).

---

## 4. Updated Core Design Principles

(Additions to the v7.4.7 list.)

30. **Rosettes are flattened, not special-cased.** The runtime
    `WellPlate` contains only ordinary wells; a rosette contributes named
    sub-wells (`A1.a` …) and the parent is dropped. No downstream code
    (print planner, ink assignment, workflows, pick & place) knows what a
    rosette is — they iterate wells by name as always.

31. **Insert clearance is global + conservative.** The travel-Z floor is
    `plate_top + tallest insert rim + margin`, applied to every
    `safe_travel_to`. Any route clears the tallest tube regardless of
    path. Per-well dispense Z is separate (`WellInfo.ink_z_mm`).

32. **Standard inserts are saved nested designs.** Dropping one copies it
    into the well's `rosette_design`; a per-well `rosette_rotation_deg`
    aligns it (rotation only). The library lives in
    `config/hardware/inserts/`.

---

## 5. Sections Unchanged

All earlier architecture sections not mentioned above apply verbatim.

# MEBP v7.4.6 — Architecture Reference (Delta)

**Multi-Extrusion Bioprinting Platform**
**Version 7.4.6 | May 2026**

> **Delta against [`ARCHITECTURE_V745.md`](ARCHITECTURE_V745.md).**
> Sections not mentioned here are unchanged.

---

## 1. What's New in v7.4.6

Phase 2 of the parametric well-plate designer. The v7.4.5 sketcher
shipped Select / Single Well / Grid Pattern tools and a base set of
8 constraint kinds. v7.4.6 lifts the designer from "demo" to "shipping
feature":

- Custom plates flow **end-to-end** through `WellSetup`, `WorkspaceConfig`,
  `print_well_setup`, and `print_workspace`. Print Setup now treats a
  saved custom design the same as a built-in 96-well plate.
- **Group editing** in the properties panel — selecting any well within
  a grid or ring exposes the underlying pattern's parameters
  (rows/cols/spacing/origin/diameter for grids; count/center/radius/
  diameter/start_angle for rings) with an Apply button that rebuilds
  the group's wells in place.
- **Circle Pattern** tool — drops N wells equally spaced on a ring.
- **Line / Construction Line** tools — two-click placement of reference
  geometry. Construction lines fix their endpoints by default so they
  act as immovable reference axes.
- **Undo / Redo** via a snapshot-based history stack (capacity 50,
  lazy capture on first drag movement, `Ctrl+Z` / `Ctrl+Shift+Z` /
  `Ctrl+Y` shortcuts plus toolbar buttons).
- **Drag-preview overlay** while pattern tools are active — ghost
  outlines of the placement before commit.
- **6 additional constraint kinds**: `point_on_line`, `parallel`,
  `perpendicular`, `equal_length`, `tangent_cc`, `symmetric_pp`.
- **Dimension markers** — distance constraint pills ("↔ 15.00") drawn
  at the segment midpoint; lock glyphs over fixed wells. Auto-hidden
  on dense plates (>64 wells) to keep the canvas readable.

---

## 2. Backend Updates

### `SupportClasses/PlateDesign.py`

```python
class PlateDesign:
    def add_circle_pattern(self, count, center_x, center_y,
                           radius, diameter, ...) -> Group: ...
    def rebuild_group(self, group_id, new_params: dict) -> Group: ...
```

`rebuild_group` drops a group's old member wells (plus orphan center
points and constraints touching them) and recreates wells per the
group's `pattern_kind`. Powers the properties-panel group editor.

`CONSTRAINT_KINDS` extended with: `point_on_line`, `parallel`,
`perpendicular`, `equal_length`, `tangent_cc`, `symmetric_pp`.

### `SupportClasses/PlateSketchSolver.py`

New residual implementations:

| Kind | Residual | Rows |
|---|---|---|
| `point_on_line` | unnormalized cross product `(P-A)×(B-A)` | 1 |
| `parallel` | direction-vector cross product | 1 |
| `perpendicular` | direction-vector dot product | 1 |
| `equal_length` | `|B1-A1|² - |B2-A2|²` (squared form) | 1 |
| `tangent_cc` | `|c1-c2|² - ((d1+d2)/2)²` (external tangent) | 1 |
| `symmetric_pp` | midpoint-on-line + (p1-p2) ⟂ line | 2 |

### `SupportClasses/WellPlate.py`

`WellPlate.load(str)` now accepts strings prefixed with `"custom:"`
(the encoding used internally by `WellPlate.format`) and strips the
prefix before resolving the file. This lets round-trips like
`WellPlate.load(plate.format)` work transparently.

### `SupportClasses/PhysicalModels.py`

`WorkspaceConfig` gains a `plate_name: str` field and an
`active_plate_key` property mirroring `HardwareConfig`. JSON round-trip
preserved.

### `SupportClasses/WellSetup.py`

`WellSetupModel.__init__` and `.set_plate_format` now accept
`int | str` (custom plate name). Internally route through
`WellPlate.load(...)` so the model can hold a custom plate.

---

## 3. UI Updates

### `gui/widgets/plate_designer_canvas.py`

- `Tool` enum extended with `DRAW_CIRCLE_PATTERN`, `DRAW_LINE`,
  `DRAW_CONSTRUCTION_LINE`.
- `LineItem(QGraphicsLineItem)` scene class for rendered lines (solid
  white for `Line`, dashed teal for construction lines).
- Drag-preview overlay (`_preview_items` list): ghost wells drawn at
  the cursor while `DRAW_SINGLE_WELL` / `DRAW_GRID` /
  `DRAW_CIRCLE_PATTERN` is active. Cleared on tool change and
  `leaveEvent`.
- Constraint markers (`_draw_constraint_markers`) render lock glyphs
  on fixed wells and "↔ X.XX" pills mid-segment on distance
  constraints. Auto-hidden when well count > 64.
- Snapshot-based undo/redo: `push_undo_snapshot()` is called by
  mutator methods (`_handle_draw_well`, `_handle_draw_grid`,
  `_handle_draw_circle_pattern`, `_handle_draw_line`,
  `delete_selection`, `lock_selection`, `add_constraint_now`,
  `add_constraint_explicit`, `rebuild_group_now`). Drag captures
  lazily on first mouseMove so bare clicks don't pollute history.
  Capacity capped at 50 snapshots (~1.5 MB peak).
- New public APIs: `push_undo_snapshot`, `can_undo`, `can_redo`,
  `undo`, `redo`, `rebuild_group_now`, `add_constraint_explicit`,
  `_entity_id_at` (for selecting Lines by click).

### `gui/pages/hardware/plate_designer.py`

- Toolbar rebuilt with 8 uniform 44×44 icon buttons:
  Select / +Well / +Grid / +Ring / Line / Construction Line /
  Lock / Delete / Undo / Redo. Each button carries an inline-SVG
  Phosphor-style icon (`cursor`, `circle-plus`, `grid`, `compass`,
  `line`, `lock`, `trash`, `undo`, `redo`).
- Lock and Delete are now selection-aware (disabled when nothing is
  selected); Undo/Redo reflect canvas stack state.
- Save is disabled on standard plates (use Save As to fork); Save
  enabled on custom plates only when dirty.
- Properties panel: per-selection cards now include:
  - Single Well + Group card (when the well is in a group)
  - Line / Construction Line card (length, endpoints, toggle, delete)
  - Well + Line mixed selection → "Point on line" button
  - Multi-well batch card adds "Tangent (touching circles)" action
- Keyboard shortcuts: `S` Select, `W` Well, `G` Grid, `C` Circle, `L`
  Line, `Shift+L` Construction Line, `K` Lock, `Ctrl+Z` Undo,
  `Ctrl+Shift+Z` / `Ctrl+Y` Redo, `Esc` cancel tool, `F` fit view,
  `Del` delete selection.
- `_format_status` renders DOF status (well-determined / under /
  inconsistent + top conflicting constraint ids).

### `gui/pages/print_workspace.py`

Bridge copies `plate_name` from `HardwareConfig` to `WorkspaceConfig`.
Plate-display panel renders custom plates with their actual
dimensions and Ø range.

### `gui/pages/print_well_setup.py`

`WellSetupModel` is constructed from `workspace.active_plate_key` (or
falls back to `plate_format` for older configs). `set_workspace` /
`set_hardware_config` compare using a `_same_plate_key` helper that
treats integer 96 / "custom:foo" / "foo" appropriately.

### `gui/pages/calibration.py`

`set_hardware_config` reads `config.active_plate_key` (fallback to
`plate_format`) and routes through `WellPlate.load(key)`. Rows/cols
and corner_well are derived from the loaded plate, not from
`PLATE_DEFINITIONS` — so custom plates calibrate correctly.

---

## 4. Updated Core Design Principles

(Additions to the v7.4.5 list.)

25. **Group params are editable post-placement.** A `Group` entity's
    `params` dict is the single source of truth for that pattern; the
    designer's group card writes to it and calls
    `PlateDesign.rebuild_group` to materialize the change. Constraints
    on the group's old wells are dropped (per the contract in the
    method's docstring).

26. **Construction geometry has fixed endpoints by default.**
    Construction lines act as reference geometry — `_handle_draw_line(
    construction=True)` sets `Point.fixed = True` on both endpoints
    so the solver won't drift the line when other constraints pull
    against it. Users can unlock via the line properties card.

27. **Undo/Redo uses serialized snapshots, not action records.**
    `PlateDesign.to_dict()` captures the full document state cheaply;
    `from_dict` restores. This avoids the bookkeeping of action
    objects and stays correct across complex mutations like
    `rebuild_group`. Capacity is bounded (50) and snapshots are
    lazily captured during drag.

---

## 5. Sections Unchanged

All earlier architecture sections not mentioned above apply verbatim.

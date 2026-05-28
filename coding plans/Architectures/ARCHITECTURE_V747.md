# MEBP v7.4.7 — Architecture Reference (Delta)

**Multi-Extrusion Bioprinting Platform**
**Version 7.4.7 | May 2026**

> **Delta against [`ARCHITECTURE_V746.md`](ARCHITECTURE_V746.md).**
> Sections not mentioned here are unchanged.

---

## 1. What's New in v7.4.7

Four follow-on capabilities for the parametric well-plate designer:

1. **Wheel-zoom toggle** — a "Wheel zoom" checkbox in the designer header
   disables scroll-wheel zoom (Fit + buttons still work).
2. **Fresh / blank plate** — a "New" button starts an empty ANSI-footprint
   plate to build from scratch.
3. **Edge-distance dimensions** — a Dimension tool places editable
   SolidWorks-style dimension lines from a well to the plate's left + top
   edges, with an inline numeric field at each midpoint.
4. **Rosette designer** — a multi-well insert *inside* a single well,
   authored with the same designer machinery (nested, in-place) and
   compiled to the existing `RosetteInsert`.

---

## 2. Backend Changes

### `SupportClasses/PlateDesign.py`

- **`Constraint.mode: str = "center"`** — `"center"` | `"edge"` reference
  mode for edge-distance dimensions. Serialized in `to_dict`/`from_dict`.
- **`CONSTRAINT_KINDS`** gains `dist_left_edge`, `dist_top_edge`.
- **`PlateOutline.kind`** gains `"circle"` + a `radius: float` field
  (rosette bore). Round-tripped.
- **`Well.rosette_design: Optional[PlateDesign]`** — a nested design
  (circular outline = well bore, sub-wells = wells, origin = well center).
  Recursively serialized in `_entity_to_dict`/`_entity_from_dict`.
- **`PlateDesign.blank(name, width, height, a1_offset_x, a1_offset_y)`** —
  rect outline + ground origin, no wells.
- **`PlateDesign.blank_rosette(bore_radius_mm, name)`** — circular outline,
  no sub-wells.
- **`PlateDesign.to_rosette_insert(name, well_format) -> RosetteInsert`** —
  converts sub-wells to the polar `RosetteSubWell` convention
  (0° = +Y via `θ = atan2(x, y)`); detects a center well at r≈0; sets
  `ring_radius_mm` to the median ring radius.
- **`PlateDesign.compile()`** — for each `Well` with a `rosette_design`,
  materializes a `RosetteInsert` onto the resulting `WellInfo.rosette_insert`
  so the existing render/print path sees it unchanged.

### `SupportClasses/PlateSketchSolver.py`

New residuals (edges derive from the footprint with A1 at scene origin):

| Kind | Residual | DOF |
|---|---|---|
| `dist_left_edge` | `(px − r − (−a1_offset_x)) − value` | 1 |
| `dist_top_edge`  | `(py − r − (−a1_offset_y)) − value` | 1 |

`r = well.diameter/2` when `mode == "edge"`, else 0. `_residual_count`
updated for both kinds.

### `SupportClasses/PhysicalModels.py`

No change — reuses `RosetteInsert` / `RosetteSubWell` (and the
`get_subwell_xy` 0°=+Y convention that `to_rosette_insert` matches).

---

## 3. UI Changes

### `gui/widgets/plate_designer_canvas.py`

- `_wheel_zoom_enabled` flag + `set_wheel_zoom_enabled`; `wheelEvent`
  early-returns when disabled.
- `Tool.DIMENSION` + `_handle_dimension` (click a well → adds
  `dist_left_edge` + `dist_top_edge` at the current measured distances).
- `set_dim_ref_mode("center"|"edge")` — reference for new dimensions.
- `_draw_dimensions` renders witness lines + end ticks per edge dimension,
  with an inline `QDoubleSpinBox` embedded via `QGraphicsProxyWidget`
  flagged `ItemIgnoresTransformations` (constant size during zoom). Editors
  tracked in `_dim_editors`, cleared on scene rebuild. `editingFinished`
  commits the value → re-solve → rebuild.
- Circular-outline render mode: when `outline.kind == "circle"`, the
  outline is an ellipse centered at the origin; `_draw_grid_headers` and
  `_draw_dimensions` are skipped (no rectangular edges in rosette mode).

### `gui/pages/hardware/plate_designer.py`

- Header: **New** button, **Wheel zoom** checkbox, **Dim ref**
  (Center/Edge) combo.
- Toolbar: **Dimension** tool button (`ruler` icon, shortcut `D`).
- `load_design(design, key, dirty)` mounts an in-memory design (used by
  New + rosette nesting); `load_plate` delegates to it.
- Save/Save As/Delete guards: Save As needs ≥1 well; blank plate (`key=""`)
  requires Save As to name it.
- **Rosette nesting** via `_edit_context: (parent_design, well_id)`:
  - "Add/Edit rosette…" button on the single-well card (top plate only).
  - Breadcrumb bar (◀ Back to plate + preset combo) shown while editing.
  - `_on_edit_rosette` swaps the canvas to the well's nested design
    (creating a blank circular one if absent); `_on_back_to_plate`
    restores the parent; `_on_rosette_preset_changed` rebuilds the nested
    design from a preset (Ring of N [+ center] via `add_circle_pattern`);
    `_on_remove_rosette` detaches it.
  - Top-level actions (New / Save / Save As / Delete / picker) disabled
    while in nested mode.
- `_format_constraint` renders the new edge-distance + v7.4.6 constraint
  kinds.

---

## 4. Updated Core Design Principles

(Additions to the v7.4.6 list.)

28. **Edge dimensions are dimension-driven constraints.** Plate edges are
    derived from `a1_offset_x/y` (A1 at scene origin), not Line entities.
    `dist_left_edge` + `dist_top_edge` together position a well
    parametrically and editably — unlike a static `fix`.

29. **Rosettes reuse the designer, nested.** A rosette is a `PlateDesign`
    with a circular outline edited in-place on the same canvas. It is
    stored richly on the parent `Well` (recursive JSON) and compiled to
    the legacy `RosetteInsert` at parent-compile time, so existing
    rendering (`well_plate_view.update_well_rosettes`) and future print
    integration consume it with no changes.

---

## 5. Sections Unchanged

All earlier architecture sections not mentioned above apply verbatim.

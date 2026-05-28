# MEBP v7.4.6 → v7.4.7 Update Plan

## Objective

Four follow-on capabilities for the parametric well-plate designer:

1. **Wheel-zoom toggle** — a header checkbox to disable scroll-wheel zoom on the canvas (some users don't want it).
2. **Fresh / blank plate** — a "New" button that starts an empty plate (ANSI footprint, no wells) to build from scratch.
3. **Edge-distance dimensions** — a Dimension tool: click a well → two SolidWorks-style dimension lines to the left + top plate edges, each with an inline editable distance field at its midpoint. References the well center by default; per-tool toggle to the well edge.
4. **Rosette designer** — a rosette is a multi-well insert inside a single well. Select a well → "Edit rosette" → the canvas swaps in-place to the well's circular interior (nested designer) with a "Back to plate" breadcrumb. Presets (Ring of N, +center) or fully custom. Compiles to the existing `RosetteInsert` so it flows into the existing well render path.

Branch: continues on `Version-7.4.2`; bump to `Version-7.4.7` at completion.

## Files Modified

| File | Rationale |
|------|-----------|
| `SupportClasses/PlateDesign.py` | `Constraint.mode` field (center/edge); `dist_left_edge` / `dist_top_edge` in `CONSTRAINT_KINDS`; `PlateOutline.kind="circle"` + `radius`; `Well.rosette_design` (nested `PlateDesign`, recursive serialization); `PlateDesign.blank()` factory; `to_rosette_insert()`; `compile()` attaches `WellInfo.rosette_insert` for wells with a rosette design. |
| `SupportClasses/PlateSketchSolver.py` | Residuals for `dist_left_edge` / `dist_top_edge` (center + edge reference); `_residual_count` updated. |
| `gui/widgets/plate_designer_canvas.py` | `_wheel_zoom_enabled` flag + `set_wheel_zoom_enabled`; `Tool.DIMENSION` + handler; dimension witness-lines + `QGraphicsProxyWidget` editable distance fields (`ItemIgnoresTransformations`); circular-outline render mode; nested-design swap support; skip grid headers in rosette mode. |
| `gui/pages/hardware/plate_designer.py` | Header: New button + Wheel-zoom checkbox; Dimension tool button + Center/Edge ref toggle; rosette breadcrumb bar + preset combo; "Edit rosette" on the single-well card; `load_design()` helper; blank/empty save guards; `_format_constraint` cases for edge dims. |
| `SupportClasses/PhysicalModels.py` | (no change) reuse `RosetteInsert` / `RosetteSubWell.create_standard` / `get_subwell_xy`. |
| `gui/widgets/well_plate_view.py` | (no change) existing `update_well_rosettes` consumes `WellInfo.rosette_insert`. |
| `tests/test_v747_plate_designer.py` (new) | edge constraints, blank plate, circular outline, rosette compile, nested round-trip, Constraint.mode round-trip. |
| `coding plans/Architectures/ARCHITECTURE_V747.md` (new) | delta against V746. |
| `CLAUDE.md` | version bump to V7.4.7 + plans-table entry. |

## Implementation Steps

- [x] Feature 1: wheel-zoom toggle (canvas flag + `wheelEvent` guard + header checkbox)
- [x] Feature 2: `PlateDesign.blank()` + `load_design()` + New button + dirty/empty guards
- [x] Feature 3: `Constraint.mode`; edge constraint kinds + solver residuals; `Tool.DIMENSION`; dimension-line + editable-field rendering; toolbar button + ref toggle; `_format_constraint`
- [x] Feature 4: `PlateOutline` circle mode; `Well.rosette_design` + recursive serialization; `to_rosette_insert()`; `compile()` attach; circular-outline canvas; nested navigation (breadcrumb + preset combo + Edit rosette)
- [x] Tests: `tests/test_v747_plate_designer.py` (16 tests)
- [x] Architecture doc `ARCHITECTURE_V747.md` + CLAUDE.md bump
- [x] Manual verification + full regression (176 tests green)

## Testing Notes

```bash
QT_QPA_PLATFORM=offscreen python3 -m unittest tests.test_v747_plate_designer
QT_QPA_PLATFORM=offscreen python3 -m unittest \
  tests.test_v745_plate_design tests.test_v745_plate_sketch_solver \
  tests.test_v745_canvas_history tests.test_v746_phase2  # regression
```

Manual (`python3 main.py` → Hardware Setup → Plate):
1. Uncheck "Wheel zoom" → wheel no longer zooms; re-check restores it.
2. "New" → blank ANSI plate, 0 wells; place a well; Save As.
3. Dimension tool → click a well → editable dim lines to left + top edges; type a value → well moves; flip Ref → measures to well edge.
4. Edit rosette → preset "Ring of 6 + center" → nested circular canvas; edit; Back to plate; rosette renders in Print Setup; Save + reopen preserves layout.

## Issues & Decisions

- **Edge dimensions are dimension-driven constraints.** `dist_left_edge` / `dist_top_edge` each remove 1 DOF; together they fully position a well parametrically (editable, unlike a static `fix`). Edges derive from `design.a1_offset_x/y` (A1 at scene origin 0,0), so no Line entities are needed for plate edges.
- **`Constraint.mode`** (`"center"` | `"edge"`) added to the ADT rather than encoding it in `kind`, keeping the kind list clean. Edge mode subtracts the well radius so the dimension measures to the well's near edge.
- **Inline edit field via `QGraphicsProxyWidget` + `ItemIgnoresTransformations`** — stays constant-size during zoom while anchored to the scene-space midpoint. Tracked in `_dim_editors`, cleared on scene rebuild.
- **Rosette = nested `PlateDesign` with a circular outline.** Reuses every designer tool/constraint. The nested design is stored on the parent `Well` (recursive JSON) for rich re-editing; `compile()` converts it to the existing `RosetteInsert` (polar subwells) so existing rendering — and future print integration — works unchanged.
- **In-place nested editing** (not a dialog) keeps one canvas and reuses undo/solve/markers. A breadcrumb bar + preset combo appear while `_edit_context` is set; the Dimension tool + grid headers are suppressed in rosette mode.
- **Presets reuse `add_circle_pattern` (+ center `add_well`)** rather than a new pattern generator; "Blank" rosette = circular outline only.

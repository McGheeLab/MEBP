# MEBP v7.5.x — Sketch page XZ side-profile view

## Objective
`changes_needed.md` item 4: "on the print sketch page, we should have an xz view
to the bottom which is resizeable with the xy view." Operator-confirmed content:
a **side-profile (elevation) of the drawn print itself** — print height, layer
stack, plate floor, and travel lifts — NOT the hardware Z widget.

## Files Modified
- **NEW `gui/widgets/sketch_profile_view.py`** — `SketchProfileView`, a custom-painted
  Z-vs-X elevation. `set_trajectory(traj, pump_states)` / `clear()`. Draws print
  segments solid (coloured per pump via `PUMP_HEX`), travel segments dashed/faded
  (from `pump_states` — a segment is travel when its leaving waypoint extrudes
  nothing), the plate floor (z=0) reference line, and labelled X/Z extents. Z is fit
  independently of X so thin layers stay visible; a "Z ×N exaggerated" hint flags it.
- `gui/pages/print_builder_sketch.py` — import `SketchProfileView`; in `_build_ui` the
  centre pane becomes a **vertical `QSplitter`** (canvas top ≈3 : profile bottom ≈1)
  so the XZ view is resizable against the XY canvas; `_recompute_preview` feeds the
  profile from the SAME `compile_to_trajectory` result it feeds the canvas (clears on
  empty/compile-error).

## Data source
`SupportClasses.SketchTrajectory.compile_to_trajectory` already returns the Nx7
trajectory (Z carries layers + `travel_clearance` lifts) AND a parallel `pump_states`
list — consumed directly, no extra compile.

## Implementation Steps
- [x] `SketchProfileView` widget (paint, set_trajectory/clear, travel detection)
- [x] Vertical splitter (canvas | profile) in the sketch page centre pane
- [x] Feed from `_recompute_preview` (both empty + populated branches)
- [x] Tests `tests/test_v75x_sketch_xz_profile_view.py` (8)

## Testing
`tests/test_v75x_sketch_xz_profile_view.py` — widget extents include the floor,
renders, clears, rejects degenerate trajectories, travel vs print classification,
page wiring (feeds + clears). Design-time only (no controller). All green.

## Issues & Decisions
- Independent X/Z scaling (not equal aspect) so 0.2 mm layers are visible against
  mm-scale widths; extents labelled + exaggeration hint so the distortion is explicit.
- Reused the existing compile pipeline rather than a second trajectory source.

# MEBP v7.5.x — Sketch page: single-well zoom + selectable well + needle-safe boundary

## Objective

Make the Print Builder → Sketch page well-aware so the operator draws inside a real
well, not on an unbounded grid:

1. **Default to a zoomed-in view of a single well** on entry.
2. **Select which well** the boundary is derived from (per-well diameter).
3. Draw an **inner "needle-safe" boundary** inset from the well wall by the **needle
   radius**, so the needle never contacts the well wall when printing. A sketch that
   extends past the safe ring raises a **non-blocking warning** (and a confirm on Send).

## Context / what already existed

The sketch canvas already authors shapes in **well-relative mm with (0,0) = well
center** and already drew a single dashed "reference well" outer circle
(`SketchCanvas.set_reference_well`) sized from the **max** well diameter, fitting it via
`fit_view()`. Missing: a per-well selector, the inner needle-radius boundary, and a
bounds check. Geometry confirmed via a multi-agent understanding workflow:
- `WellInfo.diameter` (mm) per well; `WellPlate.well_names`, `get_well_info(name)`,
  `get_all_wells()`; `WellPlate.load(config.active_plate_key)`.
- `NeedleSpec.od_mm` (outer diameter, mm). Inner safe Ø = `well_Ø − needle_Ø`
  (inset by the needle radius on each side); safe radius = inner Ø / 2.
- The well-bounds pattern `sqrt(x²+y²) > well_radius` already used by
  `print_objects.py::_check_bounds`.

## Files Modified

| File | Change |
|------|--------|
| `gui/widgets/sketch_canvas.py` | New `_safe_well_d` + `set_safe_boundary(diameter_mm)`; new `_draw_safe_boundary()` (red dashed inner ring + "needle-safe Ø" label) painted after the well-wall ring; outer label reworded to "well wall Ø". |
| `gui/pages/print_builder_sketch.py` | New persistent **Well boundary** card (well `QComboBox` + info + warning labels). `set_hardware_config` now loads the active plate, populates the well selector (default **A1**, else first), and applies the boundary. New helpers: `_load_plate`, `_populate_well_combo`, `_default_well_name`, `_on_well_changed`, `_well_diameter_for_selected`, `_apply_well_boundary` (sets both rings + `fit_view`), `_update_boundary_info`, `_max_radius_mm`, `_exceeds_safe_boundary`, `_update_bounds_warning`. New `showEvent` defaults to the zoomed single-well view when the sketch is empty. `_recompute_preview` flags out-of-bounds; `_send_to_print_setup` confirms (warn, allow) when out of bounds. Removed the superseded `_well_diameter_from_config`. |
| `tests/test_v75x_sketch_well_boundary.py` | New — 13 tests (canvas boundary storage/render, well selector + default A1, per-well derivation, needle/plate-absent fallbacks, safe-boundary check math, preview warning toggling). |

## Implementation Steps

- [x] Canvas: inner needle-safe ring (`set_safe_boundary` + `_draw_safe_boundary`).
- [x] Page: persistent well selector card; load plate; default well A1.
- [x] Derive both boundaries from the selected well + needle Ø; zoom to the well.
- [x] Default zoomed single-well view via `showEvent` (empty-sketch only).
- [x] Non-blocking out-of-bounds warning (preview label) + Send-time confirm.
- [x] Tests (13) + sketch suites green (34 total).

## Testing Notes

- `python -m unittest tests.test_v75x_sketch_well_boundary` → 13 passed.
- All sketch suites (`sketch_trajectory`, `sketch_print_visibility`,
  `sketch_well_boundary`) → 34 passed.
- Offscreen smoke: construct page → `set_hardware_config` (96-well + 22 g needle) →
  `showEvent` → render → well A1, well-wall Ø 6.35, needle-safe Ø 5.632, safe r 2.816.
- Manual bench (pending): open Print Builder → Sketch; confirm it opens zoomed on a
  single well with two dashed rings; change the well in the selector and confirm the
  rings + zoom update; draw past the inner ring → warning shows + Send asks to confirm.

## Issues & Decisions

- **Enforcement = warn, allow send** (user choice): draw both rings, warn in the status
  area when the compiled toolpath crosses the safe ring, confirm on Send, never block.
- **Inset = needle radius** exactly (`well_Ø − needle_Ø`), per the request — bead width
  is not added.
- **Boundary check covers all waypoints** (including in-well travel) via radial distance
  from origin, mirroring `print_objects._check_bounds`; conservative and consistent.
- **Per-well diameter** with fallbacks (uniform `well_diameter`, then max well diameter)
  so standard, custom, and rosette-flattened plates all work.
- `showEvent` re-fits only when the sketch is empty, so an in-progress sketch keeps the
  user's pan/zoom.

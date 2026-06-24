# MEBP v7.5.x — Sketch: lift-between-shapes knob + line-thickness preview

## Objective

Two Print Builder → Sketch usability additions:

1. **Lift the needle between shapes by X mm** — an editable clearance the needle
   lifts above the print to travel between shapes (and passes/layers).
2. **Show line thickness** — an optional preview toggle that draws the toolpath at the
   deposited bead width (≈ the needle Ø) so the operator sees how thick the printed
   lines will actually be.

## Context / what already existed

- The compiler `SketchTrajectory.compile_to_trajectory` already lifts to
  `z_travel = z_start + num_layers·layer_height + travel_clearance_mm` for every
  inter-path travel move, but `travel_clearance_mm` was **not exposed** in the print
  card (stuck at the 2.0 default). Feature 1 just surfaces it.
- The canvas `SketchCanvas._draw_toolpath` drew print runs at a fixed 1.6px pen. The
  deposited bead per pass equals `sketch.line_spacing_mm` (the page sets this to the
  needle OD), so Feature 2 strokes print runs at that world-mm width when toggled.

## Files Modified

| File | Change |
|------|--------|
| `gui/widgets/sketch_canvas.py` | New `_show_thickness` flag + `set_show_thickness(on)`; `_draw_toolpath` strokes **print** runs at `max(1px, line_spacing_mm·scale)` (α140, round cap/join) when on, else the thin 1.6px line. Travel runs stay thin/dashed. |
| `gui/pages/print_builder_sketch.py` | Toolbar adds a checkable **"line"** toggle (`_thickness_btn` → `_toggle_thickness` → `canvas.set_show_thickness`). Print card adds a **"Lift between shapes"** spin (0–40 mm) wired to `Sketch.travel_clearance_mm` via `_set_sketch`. |
| `tests/test_v75x_sketch_lift_and_thickness.py` | New — 6 tests (clearance raises travel Z + default/zero-clearance values; canvas toggle state + render at bead width; page toggle drives the canvas; lift setter writes the sketch). |

## Implementation Steps

- [x] Canvas: `set_show_thickness` + bead-width stroke in `_draw_toolpath`.
- [x] Page: toolbar thickness toggle + handler.
- [x] Page: "Lift between shapes" print-card field → `travel_clearance_mm`.
- [x] Tests (6) + sketch/quick-print suites green (56 total).

## Testing Notes

- `python -m unittest tests.test_v75x_sketch_lift_and_thickness` → 6 passed.
- All sketch + quick-print suites → 56 passed.
- Manual bench (pending): in Sketch, set "Lift between shapes" higher and confirm the
  needle clears printed material between shapes; toggle the thickness button and confirm
  the toolpath renders at the needle bead width.

## Issues & Decisions

- **Lift** reuses the existing `travel_clearance_mm` (the parameter the compiler already
  honors) rather than adding a competing knob — surfaced as "Lift between shapes". It
  governs all inter-path travel (between shapes, multi-pass offsets, and layers), which
  is the safe behavior (you want to lift between passes too).
- **Thickness** uses `line_spacing_mm` (the per-pass deposited bead ≈ needle Ø) as the
  stroke width. Multi-pass thick outlines are already separate adjacent passes in the
  toolpath, so each stroked at the bead width visually sums to the intended line width.
  Travel moves are never drawn thick (no material deposited). Toggle defaults off.

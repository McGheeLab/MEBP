# MEBP v7.5.x — Sketch retract-&-move points + back-trace a path

## Objective

Two operator-requested Print Builder → **Sketch** additions:

1. **Colour-coded retract-&-move point** — "add a color coded point to anywhere
   on the sketch that retracts and moves the needle." A placeable marker where
   the needle lifts (pen-up) and travels to that spot. It also **splits the
   toolpath into runs** (the delimiter feature 2 needs).

2. **Back-trace any path** — "take the continuous collection of lines between
   the retract points and move the needle back through with a user defined
   offset." Retrace a run **reversed**, offset in **Z-height** *and* **in-plane**
   (perpendicular to the path — "offsets the current continuous path"), with an
   **extrude-on-return toggle** (lay a second bead, or move-only).

Operator decisions (via AskUserQuestion + follow-up): offset = **Z height** AND
**in-plane** (both); return extrusion = **a toggle**.

## Files Modified

| File | Change |
|------|--------|
| `SupportClasses/SketchTrajectory.py` | `"travel"` added to `SHAPE_KINDS`. `SketchShape` gains `z_offset_mm` (per-shape print-height offset) + `no_print` (move-only pass), both serialized. `compile_to_trajectory`: a `travel` shape emits `lift_out(z_travel)` → move to its `cx,cy` at the retracted travel Z (deposits nothing, stays up); per-shape print Z = `max(0, layer_z + z_offset_mm)` (floored at the plate bottom so a negative back-trace offset can't punch through); `do_print = not no_print`; `z_travel` now also clears the tallest `z_offset`. New module fns `offset_polyline(pts, distance, closed)` (per-vertex averaged-normal parallel offset with clamped miter) + `backtrace_shape(shape, *, z_offset, xy_offset, print_on_return)` (reversed + offset copy; lines/polygons/regions reverse points + in-plane offset, circle/ellipse/rect grow by the offset). `_shape_paths`/`compute_fill_region` exclude `travel`. |
| `gui/widgets/sketch_canvas.py` | `Tool.TRAVEL` + `TRAVEL_HEX` (pink). `_place_travel(w)` (single-click create). `_run_indices(seed)` = maximal run of printing shapes bounded by travel points; `backtrace_run(seed, z_offset, xy_offset, print_on_return)` inserts the reversed offset copies **right after the run** (returns count). `travel` cases added to `_draw_shape` (pink diamond + up-chevron retract cue), `_hit`, `_shape_extent`, `_apply_move`, `_transform_shape` (position only, no size), `_snap_vertices`; routed in `mousePressEvent` + idle-hover snap preview. |
| `gui/pages/print_builder_sketch.py` | Retract tool button (`arrow-up` icon). Travel-point props card (X/Y + delete). `_build_backtrace_card` (Z offset / in-plane offset spins + "Extrude on return" checkbox + "Back-trace this path" button), shown for a single non-travel selection; `_backtrace_selected` calls `canvas.backtrace_run` with persisted `_bt_z_offset`/`_bt_xy_offset`/`_bt_extrude`. |

## Design decisions

- **Travel point = a shape in the list.** Print order = shape-list order, so a
  travel point delimits runs by position. Placing appends at the end (matches
  the incremental draw-a-shape-then-break workflow).
- **Per-shape Z offset (not a separate model).** The back-trace's height offset
  is stored on the copied shape (`z_offset_mm`), so it survives edit/save/reload
  and bakes correctly (the compiler folds it into the Z column). Legacy shapes
  default `z_offset_mm=0` → **byte-identical** compiled output.
- **In-plane offset = a parallel curve** (per-vertex averaged normal, clamped
  miter), not a translation — matches "offsets the current continuous path".
  Circle/ellipse/rect grow by the offset (concentric parallel).
- **Back-trace inserts right after the run** so the needle retraces immediately;
  a Z or in-plane offset means the retrace does not weld onto the forward bead
  (it travels/repositions), while offset 0 + extrude welds into a same-height
  double pass.
- **Plate-bottom floor:** effective per-shape Z is floored at 0 (plate bottom)
  so a negative Z offset degrades to "at the plate bottom", never below it.

## Testing Notes

- `python -m unittest tests.test_v75x_sketch_retract_and_backtrace` (43)
- Regression: sketch (retract/group-select/thickness/edit/lift/well-boundary/xz-profile/visibility) + print-library + quick-print (workflow/travel-split) + multi-object-seam + print-setup-routine = **193 green**.
- Adversarial review workflow (4 dims → verify).
- **Needs GUI verification on ME3B V1**: place a retract point (pink diamond),
  draw multiple runs, "Back-trace this path" with +Z and in-plane offsets and
  the extrude toggle, confirm the profile/preview and the baked print; edit-save
  round-trips the travel points + offsets.

## Status

- [x] Model: `travel` kind + `z_offset_mm`/`no_print` + serialization
- [x] Compiler: travel retract-&-move, per-shape Z (plate-bottom floored), move-only, z_travel clearance
- [x] Geometry: `offset_polyline` + `backtrace_shape`
- [x] Canvas: TRAVEL tool + marker + hit/move/transform/snap + `_run_indices`/`backtrace_run`
- [x] Page: retract tool button + travel props card + back-trace card + wiring
- [x] Tests (43) + regression (193)
- [x] Adversarial review (4 dims → verify) — 6 actionable fixes + 2 non-bugs locked in
- [x] CLAUDE.md table + memory

## Review findings (4-dim workflow → verify) — 21 confirmed, triaged

The verify pass confirmed 21/38. Seven were **"CONFIRMED correct"** (serialization
round-trip, travel compile, baked edit round-trip, plate-bottom floor, regions
drop travel, downstream consumers untouched, travel-only empty edge) — no action.
Six actionable fixes applied; two flagged HIGH were empirically shown to be
non-bugs and locked in with tests.

**Fixed:**
1. **Chained back-trace Z runaway** (F1, high): back-tracing a back-trace stacks
   `z_offset_mm` unbounded → `backtrace_shape` now clamps the accumulated offset
   to ±40 mm (the UI range).
2. **`offset_polyline` degenerates on duplicate vertices** (F2, med): duplicate
   consecutive points gave a zero-length segment → diverging normal → spike. Now
   dedups consecutive points up front.
3. **Negative-Z back-trace collision** (F9, med, safety): a return pass below the
   printed bead can contact it → `_backtrace_selected` now shows a non-blocking
   confirm when the Z offset is negative.
4. **`_transform_shape`** (F10, low): explicit "travel = position-only" no-op
   comment in the scale block (future-proofs the implicit skip).
5. **`fit_view` zoom blow-out** (F11, med): a tiny-span sketch (e.g. only
   clustered travel points) zoomed to absurdity → scale clamped to 200 (the
   wheel-zoom ceiling), a general fix.
6. **`_shape_extent_pts`** (F14, low): explicit `travel → []` defensive case.

**Non-bugs (verified empirically, locked with tests):**
- **Multi-layer serpentine + back-trace** (HIGH-flagged): the odd-layer path
  reversal only changes *travel direction*, not deposited geometry; per-layer Z
  is correct (fwd L0=0.2, bt L0=0.5, fwd L1=0.4, bt L1=0.7) and the footprint is
  identical → `test_multilayer_backtrace_z_levels`.
- **Travel point as first shape** (CRITICAL-flagged): the compiled first
  waypoint is at `z_travel` (retracted), and the executor's `build_well_plate_job`
  preamble retracts before travelling to the pattern start anyway → safe →
  `test_travel_first_shape_starts_retracted`.

**Declined (out of scope / not a regression / risky):** near-180° miter (current
clamp-to-1.0 avoids spikes, which is the safer choice); "path length" excluding
move-only (pre-existing convention — travel was always excluded); pump-index in
the weld check (pre-existing, each segment uses its own pump correctly);
`z_offset=0` welding (that IS a same-height retrace, the literal request); the
redundant final coincident waypoint after a trailing travel point (harmless).

## Issues & Decisions

- Travel points append to the shape list (print order = list order), matching the
  incremental "draw a shape, drop a break, draw the next" workflow.
- Back-trace grabs the whole run the *single* selected shape belongs to (bounded
  by travel points) and inserts the reversed copies right after it.
- In-plane offset is a true parallel curve (per-vertex normal), added on the
  operator's follow-up ("offsets the current continuous path"); circle/ellipse/
  rect grow by the offset (concentric).

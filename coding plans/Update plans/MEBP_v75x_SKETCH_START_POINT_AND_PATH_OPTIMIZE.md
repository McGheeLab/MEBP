# MEBP v7.5.x — Sketch: manual print start/stop control + path-optimize postprocess

## Objective

Two operator-requested Print Builder → **Sketch** additions to fix **print
continuity**:

> "On the sketch page there are many times that a shape is placed down and the
> start and stop location is standard, but this is not good for print
> continuity. We should (1) allow the user to control the start/stop location of
> any print object with snap to existing lines, and (2) add a postprocess step
> that optimizes the print path to minimize the number of discontinuities."

Each drawn shape prints from a *fixed default* start/stop — a circle at angle 0
(rightmost), a rect at a corner, a line at `points[0]`. When shapes sit near
each other the next shape's fixed start is often far from where the last ended,
so the compiler lifts the needle (pen-up), travels, and lowers again — an
avoidable *discontinuity* (bead break / needle-drag risk).

**(1) Manual start control:** any unfilled outline shape gains a draggable green
start marker; dragging it snaps to existing lines/vertices, so a shape can be
made to begin exactly where the previous one ended → the existing compiler weld
machinery fuses them into one continuous bead.

**(2) Auto-optimize:** a one-click "Optimize print path" reorders shapes and sets
each shape's start point/direction to minimize the number of pen-up travels
(**full reorder**, operator's choice). User-placed retract (`travel`) points stay
as fixed breaks; undoable.

The Sketch compiler already **welds** two paths into one continuous bead when the
next path's start is within `weld_tol` (≈ ½ bead) of where the last ended
(`compile_to_trajectory`, the `welded` block). So the whole feature reduces to
**controlling where each shape's path begins** — the weld machinery removes the
pen-up automatically.

## Files Modified

| File | Rationale |
|------|-----------|
| `SupportClasses/SketchTrajectory.py` | `SketchShape.start_point` field (+ conditional serialize → byte-identical legacy when `None`); `reorder_path_to_start(path, start_xy, closed)`; wrap `_shape_paths` → `_shape_paths_raw` (the wrapper reorders each outline pass to the start point); `optimize_print_order(sketch)` + helpers (`_shape_conn`, `_conn_entry_exit`, `_greedy_order`, `_optimize_segment`, `_weld_tol_for`); `count_discontinuities(sketch, needle, syringe)` |
| `gui/widgets/sketch_canvas.py` | Draggable green start marker: `_effective_start_world` / `_default_start_world` / `_resolve_start_on_shape` / `_start_marker_screen` / `_start_handle_at` / `_draw_start_marker`; `_press_select` checks the start handle before resize handles; `_apply_resize` `"start"` case; a start drag also fires `selection_changed` on release; `clear_start_point(idx)`; `optimize()` |
| `gui/pages/print_builder_sketch.py` | "Print start" props row (custom/default label + hint + "Reset start point" button) for outline shapes; "✨ Optimize print path" button in the Print-parameters card; `_reset_start_point` + `_optimize_path` handlers (status "N → M travel(s)") |
| `tests/test_v75x_sketch_start_point_and_optimize.py` | NEW — 33 headless + offscreen tests |
| `CLAUDE.md` | added this plan to the Existing Update Plans table |

## Implementation Steps

- [x] `SketchShape.start_point: tuple|None` + conditional `to_dict`/`from_dict`
- [x] `reorder_path_to_start` — open reverses to nearest endpoint; closed rolls
  ring to the nearest vertex and re-closes; deposited geometry unchanged
- [x] `_shape_paths` wrapper over `_shape_paths_raw`; reorders only unfilled
  outline kinds (`line/circle/ellipse/rect/polygon`); fills / `region` / `travel`
  never reordered
- [x] `optimize_print_order` — partition at `travel` markers (fixed breaks);
  greedy nearest-neighbour per segment with all-seed restarts (≤ 14 shapes);
  stamps `start_point` per shape; preserves all print params (returns a copy)
- [x] `count_discontinuities` — counts lift-to-max-Z groups − 1 (the ground truth
  the sketch tests assert via `n_lifts`; immune to zero-length weld-node segments)
- [x] Canvas start marker: draw + hit-test + `_apply_resize` `"start"` + snap via
  existing `_snap(raw, exclude=selected)`; `clear_start_point`; `optimize`
- [x] Page: "Print start" props row + Reset button; "Optimize print path" button
  + before/after status
- [x] Tests (33) green; existing sketch suites (154) + print-library / seam /
  quick-print-travel (40) green

## Testing Notes

Repo uses **unittest** (no pytest):

```
python -m unittest tests.test_v75x_sketch_start_point_and_optimize -v
```

Regression (all green):
```
python -m unittest tests.test_v75x_sketch_continuous_connected_lines \
  tests.test_v75x_sketch_trajectory tests.test_v75x_sketch_retract_and_backtrace \
  tests.test_v75x_sketch_edit_print tests.test_v75x_sketch_extrusion_thickness \
  tests.test_v75x_sketch_group_select tests.test_v75x_sketch_lift_and_thickness \
  tests.test_v75x_sketch_print_visibility tests.test_v75x_sketch_slow_lift \
  tests.test_v75x_sketch_well_boundary tests.test_v75x_sketch_xz_profile_view    # 154 OK
python -m unittest tests.test_v75x_print_library \
  tests.test_v75x_multi_object_print_seam tests.test_v75x_quick_print_travel_split  # 40 OK
```

Coverage: `reorder_path_to_start` (open reverse / closed roll+re-close / length
preserved / degenerate); `start_point` welds a rect onto a preceding line
(1 → 0 discontinuities) and never changes printed length; `start_point` is a
no-op on filled shapes; serialization round-trip + omitted-when-`None`;
`optimize_print_order` (scrambled chain 2 → 0, far-apart unchanged, travel breaks
preserved, single-shape no-op, params preserved, returns a copy);
`count_discontinuities` metric; canvas start-marker hit-test / press-routing /
`_apply_resize` / `clear_start_point` / `optimize` + undo; page `_optimize_path`
status + "Print start" props row + Reset.

**Needs real-HW / GUI verification on ME3B V1:** Print Builder → Sketch → draw a
line then a circle whose default start is far from the line's end → preview shows
a travel between them → drag the circle's green start marker onto the line's
endpoint (snaps) → the travel disappears (welds). Draw several shapes →
"Optimize print path" → status reports fewer travels, the toolpath/profile show
the chained path, retract points still split runs. Send to Print Setup / Save.

## Issues & Decisions

- **Full reorder chosen** (operator, over orientation-only / a toggle): the
  optimizer may change shape order to maximize chaining; it is undoable and
  reports before → after travels so the operator can Ctrl+Z if surprised.
- **`count_discontinuities` metric.** First cut counted printing runs from
  `pump_states`; it over-counted because the compiler emits a zero-length,
  pump-flat coincident waypoint at an exact weld node (the same behavior the
  Quick Print `_travel_mask` fix documented). Switched to counting lift-to-max-Z
  groups − 1, which matches the `n_lifts` ground truth the sketch tests assert.
- **Start-point resolution.** The stored `start_point` is the raw snapped anchor;
  the compiler resolves it to the nearest generated path vertex (closed) or the
  nearer endpoint (open). The canvas marker mirrors this (`_resolve_start_on_shape`
  = analytic nearest point on the outline) so the drawn marker lands on the shape
  after a drag. Continuity therefore depends on generated-vertex density vs
  `weld_tol` — a circle's nearest vertex to a neighbour node can be ≈ ¼ bead away,
  so a very tight `weld_tol` may not fuse a circle even when it visually touches
  (inherent to the existing weld mechanism, not new).
- **Fills / regions kept fixed** in the optimizer v1 (ordered but not
  reoriented — their raster start is fixed): the continuity win is chaining
  outline shapes, which is the operator's scenario. A reversible-region pass is a
  possible follow-up.
- **Retract (`travel`) points are hard breaks.** The optimizer partitions the
  shape list at them and optimizes each segment independently, preserving the
  operator's intentional pen-up breaks (and the back-trace runs they bound).
- **No safety-rule impact.** The Sketch page is design-time only (no motion); the
  change only reorders/reseats the *compiled* toolpath's entry points. Baked
  trajectories still bake through the same `save_trajectory_as_print_object` path
  and the same plate-bottom Z datum handling; the embedded vector `Sketch`
  (Library "Edit in Sketch") round-trips the new `start_point`.

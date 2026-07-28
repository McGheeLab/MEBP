# MEBP v7.5.x — Print Builder Sketch: Parametric Constraints

## Objective

Significantly upgrade the Print Builder → Sketch page with **parametric
modeling constraints** — tangent, join, coincident, parallel, horizontal,
vertical, concentric (operator-requested), plus perpendicular, equal
length/radius, distance & radius dimensions, and fix/lock (operator chose the
**full parametric set** via AskUserQuestion). Constraints solve **live while
dragging** (drag-ghost pinning, ~30 fps throttle, same feel as the plate
designer) and drawing with object-snap **auto-captures** coincident /
point-on constraints (toggleable) — both operator-chosen.

Constraints are **design-time only**: they position geometry in the editor;
the compiler (`compile_to_trajectory`), welds, optimizer semantics, and the
baked print output are unchanged.

## Architecture

Mirrors the proven `PlateSketchSolver` architecture (scipy
`least_squares`, LM/TRF, residuals per constraint kind, transient
`drag_ghost` pin at weight 1000, DOF report via Jacobian rank) — but operates
**directly on `SketchShape` DOFs** instead of Point entities:

| shape kind | free variables |
|---|---|
| line / polygon | every vertex (2N) |
| circle | cx, cy, radius |
| ellipse | cx, cy, rx, ry |
| rect | cx, cy, width, height |
| travel | cx, cy |
| region | **not constrainable** (baked raster) |

Only shapes **referenced by ≥ 1 constraint** enter the variable vector —
unconstrained sketches pay zero cost and every existing code path is
untouched (pay-for-play).

**Stable shape ids.** Constraints reference shapes by a new persistent
`SketchShape.id` (int, 0 = unassigned) + an **anchor** string:
`"center"`, `"p{k}"` (line/polygon vertex k), `"c0".."c3"` (rect corners
TL/TR/BR/BL), `"shape"` (whole entity), plus virtual drag anchors `"mid"`
(line) / `"centroid"` (polygon). Ids are assigned **lazily on first
constraint use** (`Sketch.ensure_shape_ids()`), so a sketch that never uses
constraints serializes **byte-identically to legacy** (id emitted only when
≠ 0; `constraints` key emitted only when non-empty). Index-based referencing
is impossible because `optimize_print_order` reorders the shape list.

**Constraint kinds + residuals** (weight-scaled rows, squared/smooth forms
matching `PlateSketchSolver`):

| kind | refs | rows | residual |
|---|---|---|---|
| `coincident` | 2 points | 2 | x1−x2, y1−y2 |
| `concentric` | 2 centers | 2 | same as coincident |
| `horizontal` | 2 points | 1 | y1−y2 |
| `vertical` | 2 points | 1 | x1−x2 |
| `parallel` | 2 lines | 1 | cross(d1, d2) |
| `perpendicular` | 2 lines | 1 | dot(d1, d2) |
| `equal_length` | 2 lines | 1 | ‖d1‖² − ‖d2‖² |
| `equal_radius` | 2 circles | 1 | r1 − r2 (radius IS a variable here, unlike the plate solver) |
| `tangent` (line↔circle) | line, circle | 1 | cross((C−A),(B−A))²/‖B−A‖² − r² |
| `tangent` (circle↔circle) | 2 circles | 1 | dist² − (r1±r2)²; `mode` = external/internal chosen from geometry at creation |
| `distance` | 2 points + value | 1 | dist² − value² |
| `radius` | circle + value | 1 | r − value |
| `point_on` | point + curve (line/circle) | 1 | cross (line) / dist²−r² (circle) |
| `fix` | 1 shape | 0 | shape DOFs excluded from variables (pinned) |
| `drag_ghost` | 1 anchor | 2 | transient, held by the solver (never in the model / serialized), weight 1000 |

Scope limits (documented): tangent/equal-radius/point-on restricted to
lines + circles (no ellipse tangency math); ellipse/rect participate via
center + rect corners; `region` excluded.

## Files Modified

| File | Change |
|---|---|
| `SupportClasses/SketchTrajectory.py` | `SketchShape.id` (conditional-emit), new `SketchConstraint` dataclass, `Sketch.constraints` + `_next_shape_id`/`_next_constraint_id` + helpers (`ensure_shape_ids`, `shape_by_id`, `add_constraint`, `remove_constraint`, `prune_constraints`, `constraints_referencing`), serialization (conditional-emit), `backtrace_shape` clears `id` on the copy |
| `SupportClasses/SketchConstraintSolver.py` | **NEW** — solver mirroring `PlateSketchSolver` (reuses its `SolveReport`/`DOFStatus`); variable packing over shape DOFs; residuals per table; drag lifecycle; write-back with min-size clamps |
| `gui/widgets/sketch_canvas.py` | constrained-move/endpoint drags → drag-ghost live solve (33 ms throttle); size-handle/typed/group edits → solve-after-edit; snap provenance (`_snap_hit`) + auto-capture (coincident on vertex snap, point_on on line/circle edge snap, toggleable); constraint glyph layer; `add_constraint_for_selection` resolution rules; delete cascade prunes constraints; `solve_report` signal |
| `gui/pages/print_builder_sketch.py` | new persistent **Constraints** card: DOF/status line, auto-capture toggle, constraint buttons (enabled per selection), constraint list rows (label + value spin for dims + delete, click→highlight shapes) |
| `tests/test_v75x_sketch_constraints.py` | **NEW** — backend solver tests, serialization/byte-identity guards, canvas + page offscreen tests |

## Implementation Steps

- [x] Explore existing sketch model/page + plate constraint solver (2 agents)
- [x] Operator decisions (AskUserQuestion): full set · live solve · auto-capture
- [x] Model: `SketchShape.id`, `SketchConstraint`, `Sketch.constraints` + helpers + serialization (byte-identical legacy)
- [x] `backtrace_shape` id hygiene; verify optimizer carries ids/constraints (deepcopy — it does)
- [x] Solver module with all residual kinds + drag lifecycle + DOF report
- [x] Canvas: solve-on-drag (ghost), solve-after-edit, snap provenance + auto-capture, glyphs, selection-based constraint creation, delete cascade
- [x] Page: Constraints card + wiring + DOF status
- [x] Tests: solver behavior, dims hold values, drag ghost, serialization round-trip, legacy byte-identity, prune/cascade, canvas auto-capture, page card build
- [x] Run sketch + print-library/quick-print adjacent suites
- [x] CLAUDE.md table row + finalize this plan

## Testing Notes

- `python -m unittest tests.test_v75x_sketch_constraints` — **43 tests green**
  (model/serialization 7, solver 15, canvas 17, page card 4 — incl. the
  legacy byte-identity guard, dedup, dry-run purity, rect-corner capture
  through the real commit path, and the inconsistent-→-conflicts report).
- Regression: **all 18 sketch suites** (`tests/test_v75x_sketch*`) —
  **300 tests green** (257 pre-existing + 43 new).
- Adjacent: `test_v75x_print_library` + `test_v75x_quick_print_multi_ink` +
  `test_v75x_quick_print_travel_split` — **44 green**.
- **Needs GUI verification on ME3B V1**: draw two lines → Join → drag one,
  the other follows live (~30 fps); H/V/parallel/tangent glyph badges render;
  distance dimension typed value holds during a drag; snap-drawing a line
  onto an endpoint auto-captures a Join (toggle off stops it); Lock pins a
  shape (dragging it moves nothing); conflicting dimensions show the red ⚠
  DOF state + red glyphs.
- **Needs GUI verification on ME3B V1**: draw two lines → Join → drag one,
  the other follows live; H/V/parallel/tangent glyphs; distance dimension
  typed value holds during drag; auto-capture on snap; DOF label states;
  conflict shows red.

## Issues & Decisions

- **Why not reuse `PlateDesign` entities**: restructuring `Sketch` into a
  Point/Line entity graph would break every existing sketch feature, test and
  saved print. Solving directly over shape DOFs keeps the model intact; only
  the solver skeleton + residual math is mirrored.
- **Stable ids over indices**: `optimize_print_order` reorders `shapes`, so
  index refs would silently corrupt constraints. Ids are lazy-assigned to
  preserve legacy byte-identity (regression-tested).
- **Optimizer safety**: verified `optimize_print_order` / the compiler only
  *reorder the list* and stamp `start_point` — shape `points` order is never
  mutated (reversal happens on the compiled path copy in
  `reorder_path_to_start`), so `p{k}` anchors stay valid.
- **`backtrace_shape` deep-copies** — the copy must get `id = 0` or two
  shapes would share an id and constraints would bind to whichever resolves
  first.
- **Ghost held by the solver, not the model** (unlike the plate solver which
  appends to `design.constraints`): an undo snapshot mid-drag can therefore
  never serialize a transient ghost.
- **No dedicated anchor-pick FSM in v1**: "Join" auto-resolves the nearest
  anchor pair between two selected shapes, and auto-capture covers
  drawing-time joins (drag an endpoint onto a vertex → coincident). A
  click-two-points pick mode can be added later if needed.
- **Group drags** solve once on release (not per-frame) in v1.
- Solve write-back clamps radii/sizes to ≥ 0.05 mm (matches canvas minimums)
  so a degenerate solve can't produce negative geometry.
- **Equal-radius as a residual, not a pre-pass**: unlike the plate solver
  (well diameter isn't a solver variable there), circle radius IS a free
  variable here, so `r1 − r2` is a plain residual row — no value-propagation
  pre-pass needed, and it composes with `radius`/`tangent` rows.
- **`fix` beats `drag_ghost`**: a fixed shape contributes no variables, so
  dragging it moves nothing (the ghost writes to constants) — correct lock
  semantics, verified by test.
- **Auto-capture dedup**: `_maybe_add_snap_constraint` refuses to add a
  constraint whose (kind, refs-set) already exists, so re-snapping the same
  endpoints never stacks duplicates.

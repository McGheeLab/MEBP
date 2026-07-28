# MEBP v7.5.x — Sketch: Closure Overlap + Draggable Start/End Markers

## Objective

Operator: "some shapes do not get closed due to technical limitations of the
type of printing we are doing; to ensure the deposited ink closes the shape we
want a start↔end overlap — a **distance-based** overlap or a **needle-diameter**
overlap, continuing along the shape's own path past the start. Also let us
control the shape's **start and end points by dragging markers along the
path**."

Two operator decisions (AskUserQuestion):
- **Closed shapes**: the END marker is an **overlap handle** — it slides along
  the path past the seam; how far past = the closure overlap. It stays in sync
  with the numeric overlap control.
- **Open shapes**: start + end markers **trim both ends** — drag them inward to
  print only a sub-segment of the drawn line/polyline (changes deposited
  geometry).

Builds on the existing `SketchShape.start_point` (draggable green seam marker)
and `overlap_closure` (bool → needle-radius overshoot). Design-time only — the
compiled trajectory of a shape *as configured* is what changes; the executor,
welds, and optimizer semantics are untouched.

## Model change (one source of truth)

`overlap_closure: bool` is RETIRED and folded into a single mode field (mirrors
the constraints-work discipline of not keeping two drifting sources):

| field | meaning |
|---|---|
| `start_point: tuple\|None` | seam (closed) / trim-start anchor (open). *(unchanged)* |
| `end_point: tuple\|None` | **NEW** — trim-end anchor (OPEN shapes only). |
| `overlap_mode: str = "none"` | **NEW** — `none` / `needle` / `distance` (CLOSED shapes). |
| `overlap_distance_mm: float = 0.0` | **NEW** — used when `overlap_mode == "distance"`. |

`SketchShape.overlap_amount_mm(needle_od, bead)`:
- `distance` → `overlap_distance_mm`
- `needle` → **one needle outer Ø** (`od`), fallback `bead` when Ø unknown
  (operator said "needle diameter" — note this is the full diameter; the
  retired `overlap_closure` used the needle *radius*).
- `none` → 0

**Serialization** — all conditional-emit → **legacy sketches byte-identical**:
`end_point` only when set; `overlap_mode` only when `!= "none"`;
`overlap_distance_mm` only when mode is `distance`. **Migration** in
`from_dict`: a legacy `overlap_closure: true` (with no `overlap_mode`) →
`overlap_mode = "needle"`.

## Compiler / geometry

- New arc-length helpers `_cumlen` / `_project_arclen` / `_point_at_arclen` /
  `_subpath_arclen` + **`trim_open_path(path, start_xy, end_xy)`** (projects both
  anchors onto the polyline and returns the sub-path between them; end unset →
  the far extreme → reduces EXACTLY to the legacy flip, byte-identical).
- `_shape_paths`: closed unfilled outline → `reorder_path_to_start` to the seam
  (unchanged); open unfilled outline with a start/end anchor → `trim_open_path`.
- Over-closure overshoot is now **per-shape** `shape.overlap_amount_mm(od, bead)`
  fed to the existing `extend_closed_path` (closed unfilled outlines only), in
  both `compile_to_trajectory` and `plan_print_sections`.

## Canvas (`sketch_canvas.py`)

- **End marker** (red flag, mirrors the green start flag): shown for a selected
  unfilled outline. Open shapes → always (trim end, default = far endpoint).
  Closed shapes → only when `overlap_mode != "none"` (the overlap handle), placed
  at the seam advanced `overlap` arc-length along the rolled ring.
- Drag (`_apply_resize` `"end"`): open → sets `end_point`; closed → projects the
  cursor onto the rolled ring, sets `overlap_mode="distance"` +
  `overlap_distance_mm = arclen-from-seam` (capped at one lap).
- `clear_end_point`; hit-test `_end_handle_at`; press routing added after the
  start-handle check.

## Page (`print_builder_sketch.py`)

- Closed loops: replace the "Overlap closure" checkbox with a **Closure overlap**
  combo (None / Needle Ø / Custom distance) + a distance spin (shown for
  Custom). Enabling it seeds a visible default so the handle appears.
- Open shapes: add an **End point** custom/default row + "Reset end point"
  (mirror the existing start-point row) with a hint to drag the red marker.

## Files Modified

- `SupportClasses/SketchTrajectory.py` — model, serialization/migration,
  arc-length helpers + `trim_open_path`, `_shape_paths`, compiler overshoot,
  `plan_print_sections`.
- `gui/widgets/sketch_canvas.py` — end marker draw/hit/drag, `clear_end_point`,
  rolled-ring arc-length.
- `gui/pages/print_builder_sketch.py` — closure-overlap combo + end-point row.
- `tests/test_v75x_sketch_overlap_sequence_needle_mode.py` — migrate to the new
  `overlap_mode` API.
- `tests/test_v75x_sketch_overlap_start_end_markers.py` — **NEW**.

## Implementation Steps

- [x] Explore existing start_point / overlap_closure / compiler
- [x] Operator decisions (AskUserQuestion): overlap-handle end marker · trim open shapes
- [x] Model + serialization/migration + `overlap_amount_mm`
- [x] Arc-length helpers + `trim_open_path` + `_shape_paths` + compiler + `plan_print_sections`
- [x] Canvas end marker (draw/hit/drag) + `clear_end_point`
- [x] Page closure-overlap combo + end-point row
- [x] Migrate existing overlap test; add new marker/overlap tests
- [x] Run sketch + adjacent suites; finalize docs + CLAUDE.md

## Testing Notes

- `python -m unittest tests.test_v75x_sketch_overlap_start_end_markers` —
  **25 green** (model/serialization/migration, arc-length + `trim_open_path`
  incl. byte-identical no-op & legacy flip, compiler overlap-extends /
  open-trim, canvas end-marker gating + drag semantics, page combo/rows).
- `tests.test_v75x_sketch_overlap_sequence_needle_mode` migrated to the new
  `overlap_mode` API — **33 green** (needle mode now = full Ø; distance mode;
  legacy `overlap_closure` migration).
- Regression: **all 19 sketch suites** (`tests/test_v75x_sketch*`) —
  **327 green**. Adjacent: print-library / quick-print-multi-ink /
  travel-split / sketch-edit / extrusion-volume-calc — **77 green**.
- **Needs GUI verification on ME3B V1**: on a circle, set Closure overlap →
  Needle Ø / Custom distance → the red ■ handle appears past the seam and
  dragging it changes the distance; the bead over-closes when printed. On a
  line, drag the green ▸ and red ■ inward to print only the middle segment.

## Issues & Decisions

- **Retire `overlap_closure`, migrate on load** — one source of truth beats two
  drifting flags; the feature is brand-new (v7.5.x) so no meaningful baked
  prints depend on the exact radius overshoot. Needle-mode is now the full
  diameter per the operator's words.
- **`trim_open_path` reduces to the legacy flip** when `end_point` is unset and
  the start lands on an endpoint (the optimizer's usage) → byte-identical for
  every existing open-shape sketch. Trimming only kicks in when the operator
  drags the markers inward.
- **Closed-shape end marker only appears once overlap is enabled** — avoids two
  flags stacked on the seam at zero overlap; the panel combo turns it on with a
  visible default, then dragging fine-tunes.
- Trimming an open shape changes deposited geometry (operator-chosen);
  overlap/closure remains a closed-shape concept.

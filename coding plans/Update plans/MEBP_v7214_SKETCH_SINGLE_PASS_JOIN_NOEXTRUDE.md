# MEBP v7.21.4 — Sketch: one pass per line, join to a print start/stop, no-extrude sections

## Objective

Three operator requests on the **Print Builder → Sketch** page, in one landing:

1. > *"on the print sketch page, do not auto calculate the need for multiple
   > lines to fill in the width of any line. We will set that line width and
   > just increase the extrusion modifier to match."*

   An outline now prints as **exactly one pass on the drawn geometry**. Width is
   reached by **flow** (the extrusion multiplier), never by synthesising extra
   parallel lines.

2. > *"on the print page, we want to be able to join a line to the start or stop
   > position of any existing print object."*

   A shape's **print start / print stop** are now snap targets, and a line drawn
   onto one is **re-ordered** so the two really print as a single continuous
   bead.

3. > *"we want to have the ability to do a no extrude on any section we want."*

   `no_print` — which existed, was honoured by the compiler, and had **no UI at
   all** (its only writer was the back-trace) — is exposed per shape, per
   multi-selection, and **per print section**.

---

## 1 — One pass per outline; width comes from the extrusion multiplier

### What it used to do

`_pass_offsets(line_width, bead)` expanded any outline whose `line_width_mm`
exceeded the fill pitch into `round(line_width / bead)` **adjacent parallel
passes**, centred on the drawn geometry:

```
2.0 mm width at a 0.4 mm pitch  →  5 passes at offsets [-0.8, -0.4, 0, +0.4, +0.8]
```

Three consequences, all of them surprising:

- **the printed geometry left the sketch** — the outermost pass sat half the
  declared width OUTSIDE the shape that was drawn (a `radius=5, width=1.2 mm`
  circle printed rings at r = 4.6 / 5.0 / 5.4);
- **length, volume and time jumped with the width** — 5× for the example above
  (an existing test asserted exactly that: `>3×` length AND volume at 2.0 mm vs
  0.4 mm);
- **width was a geometry instruction, not a flow one**, so the extrusion
  multiplier and the line width were two different, silently-competing ways to
  ask for a thicker bead.

### What it does now

`_pass_offsets` returns `[0.0]` unconditionally, and says why. `line_width_mm`
becomes a **declaration** — the target width — with the multiplier doing the
work.

**Why the function was kept rather than deleted:** every shape branch
(`line` / `circle` / `ellipse` / `rect`) iterates `offsets`, so keeping one
uniform "for each pass" shape means the whole change is one function, and
re-introducing multi-pass later (as an explicit, opt-in mode) is a change in
ONE place instead of five. `line_width`/`bead` are accepted and ignored.

**Fills are deliberately untouched.** The single-pass rule is about outline
WIDTH, not about rasters: a filled shape still lays as many lines as its pitch
requires (pinned by a guard-the-guard test, else this whole change could ship
with fills broken and the suite still green).

### Keeping `line_width_mm` honest instead of inert

With no compiler effect, the field risked becoming a dead setting — the trap
this project keeps recording. So the shape card now **prices** it:

> One pass is printed at this line — width comes from flow. 1.00× deposits
> ≈ 0.400 mm; this width needs 3.00×.   `[ Set extrusion to 3.00× to match ]`

One click sets `Sketch.extrusion_multiplier` so a single bead is as wide as the
declared width, and the shaded thickness band (already needle Ø × multiplier)
moves with it — so the operator can SEE the match. The button is disabled when
already matched or when the needed × falls outside the 0.05–5.0 range, and the
tooltip states the real limitation: **the multiplier is sketch-wide**, so shapes
with different line widths cannot all match at once.

⚠ **Deliberately NOT done:** drawing the band at the *declared* width. The band
shows the width the current flow actually predicts; painting the declared width
instead would assert a bead the flow may not back — exactly the confusion this
change removes.

---

## 2 — Join a line to an existing object's print start / stop

### The two halves

**(a) The snap targets.** New `SketchCanvas._print_point_targets()` yields every
non-filled shape's `_effective_start_world` / `_effective_end_world` — the SAME
points the green ▸ / red ■ markers are drawn at, i.e. where the compiler really
begins and ends that bead **after** a custom start point, a rolled seam or a
closure overlap. Not merely a geometric vertex: pinning a line's start to its
far endpoint swaps which end is the entry and which the exit, and the targets
follow.

They are consulted **first** in `_snap` and win ties against a coincident
vertex (the vertex/edge loops only replace on a *strictly* smaller distance),
because landing on the print point is what makes the two paths weld — the
compiler's weld tolerance is half a bead, so an exact snap always welds.

**(b) The order.** Coinciding points are necessary but **not sufficient**: the
compiler welds two paths only when they are ADJACENT in the shape list, and a
newly drawn line is appended at the END. So `_join_to_print_point` moves it:

| what landed where | insert | extra |
|---|---|---|
| press (p0) on *j*'s **STOP** | after *j* | — our path already starts there |
| release (p1) on *j*'s **STOP** | after *j* | the compiler's own pass-0 flip leads with the nearer endpoint |
| release (p1) on *j*'s **START** | before *j* | our default exit is p1, which is where *j* begins |
| press (p0) on *j*'s **START** | before *j* | **pin `start_point` to p1** so the line runs p1→p0 and ENDS there |

The last row is the only case the compiler cannot fix itself — its flip looks
*backwards*, at the previous shape's exit, so a line whose p0 sits at the next
shape's entry would print away from it. `_JOIN_RANK` prefers the two cases that
need no reversal when both endpoints landed on print points (a line bridging two
objects).

`new_idx` is always the last index, so `pop(new_idx)` cannot shift `j` — that is
what makes the insert arithmetic safe without re-scanning.

A snap onto anything else — a plain vertex, an edge, the grid — **re-orders
nothing**. Only a deliberate landing on a print marker does.

### Constraint capture

`_maybe_add_snap_constraint` now understands the two new hit kinds:

- the print point **is** a solver DOF (a line/polygon endpoint, a rect corner)
  → captured as an ordinary **coincident** constraint, so the join survives
  later edits;
- the print point is **derived** (a circle seam, a trimmed sub-segment end) →
  `anchor is None` → **capture nothing**. Writing a ref the solver cannot
  resolve would be worse than no constraint, and the geometric snap still welds.

`_snap_vertices` was refactored onto a new shared `_shape_anchor_points(sh)` (one
definition of a shape's exact snap points, reused by `_anchor_at_point`), emitting
the same tuples in the same order.

### Feedback

New `SketchCanvas.join_result` signal → the page's status line:
*"✓ Joined to the stop of circle — they print as one bead"*. The message names
the shape actually joined to, which is what pins the neighbour arithmetic.

---

## 3 — No extrude (move only) on any shape / selection / section

`SketchShape.no_print` was already in the model, serialized, and honoured by both
`compile_to_trajectory` and `plan_print_sections` — but the **only** production
writer was `backtrace_shape`. There was no way to ask for it.

New `SketchCanvas.set_no_print(indices, value)` — undoable, returns how many
shapes changed, **skips travel markers** (they have no path to print) — is
driven from three places:

- **the shape card** — "No extrude (move only — deposits nothing)";
- **the group card** — the same checkbox across a multi-selection, checked only
  when EVERY selected shape is already move-only, so a mixed selection reads as
  "printing" and one click makes all of it move-only;
- **each print-section header** in the Print-sequence card — an
  `extruding` / `no extrude` toggle. This is the literal ask: a section's shapes
  are already selectable from that card, so the toggle acts on exactly the
  section it sits on. Section body rows now read `▪ line #3 · no extrude`.

The needle still follows the path, welded to its neighbours, at print height —
only the pump does not advance. The preview needs no new code to show it: runs
are categorised from `pump_states`, so a move-only shape already renders as a
dashed grey travel run.

---

## Files Modified

| File | Why |
|---|---|
| `SupportClasses/SketchTrajectory.py` | `_pass_offsets` → single pass (+ the reasoning); `line_width_mm` re-documented as a target/declaration; four stale multi-pass comments corrected (volume model, weld tolerance, retrace gate, `plan_print_sections` docstring) |
| `gui/widgets/sketch_canvas.py` | `join_result` signal; `_print_point_targets`; `_anchor_at_point` + shared `_shape_anchor_points` (with `_snap_vertices` refactored onto it); print points first in `_snap`; print-hit handling in `_maybe_add_snap_constraint`; `_join_to_print_point` + `_JOIN_RANK` + the `_commit_draw` hook; `set_no_print` |
| `gui/pages/print_builder_sketch.py` | line-width tooltip + `_build_width_match_row` / `_match_extrusion_to`; per-shape and per-selection no-extrude checkboxes + `_set_no_print_selected`; per-section toggle + `_set_section_no_print` + body tag; `join_result` → `_on_join_result`; bead-info / extrusion / start-hint wording |
| `tests/test_v7214_sketch_single_pass_join_noextrude.py` | **NEW** — 36 tests |
| `tests/test_v75x_sketch_trajectory.py` | `test_thick_outline_multipass` → `test_thick_outline_is_still_a_single_pass` (the retired contract, inverted) |
| `tests/test_v75x_sketch_overlap_sequence_needle_mode.py` | `test_plan_uses_last_pass_end_for_thick_outline` → `test_plan_uses_the_drawn_radius_not_an_offset_pass` |
| `tests/test_v75x_sketch_constraints.py` | `test_snap_records_provenance` now expects `("printend", 0, "p1")` **and** asserts the coincident capture still happens |
| `tests/test_v75x_sketch_panel_rework.py` | section headers now carry a chevron **and** a checkable no-extrude toggle |

---

## Implementation Steps

- [x] `_pass_offsets` → one centred pass, with the reasoning and the "kept, not
      deleted" note
- [x] `line_width_mm` re-documented; stale multi-pass comments swept
- [x] Width-match row + one-click "set extrusion to match" on the shape card
- [x] `_print_point_targets` (+ anchor resolution) and print-point priority in
      `_snap`
- [x] Print-hit handling in `_maybe_add_snap_constraint` (anchor → coincident,
      derived → nothing)
- [x] `_join_to_print_point` + `_commit_draw` hook + `join_result` → status line
- [x] `set_no_print` + shape / selection / section UI + body tag
- [x] Four contract-changed tests updated (retired contracts replaced by their
      inverse, not deleted)
- [x] New suite (36) + mutation matrix (10/10)
- [x] Regression + `gui.app` import smoke + real-mouse end-to-end smoke
- [ ] **GUI verification on ME3B V1** (checklist below)

---

## Testing Notes

**New suite** `tests/test_v7214_sketch_single_pass_join_noextrude.py` — **36
tests**, offscreen, driving the PRODUCTION `SketchCanvas` / `SketchPage`
(a stand-in that agrees with the code proves nothing about it):

- single pass across line / circle / ellipse / rect at a 3 mm declared width;
  the printed radius stays exactly on the drawn 5.0 mm; width no longer changes
  length or volume; the multiplier is what scales deposition; **and a
  guard-the-guard that a fill still emits many lines**;
- the page prices the needed × and the button makes one bead that wide;
- print start/stop targets carry the right anchors (and **None** for a circle
  seam); a custom start point moves them; fills/regions/travel markers are not
  offered; the print point beats a coincident vertex;
- all four join cases re-order correctly and the plan collapses to one section,
  **plus** `test_without_the_reorder_it_would_NOT_have_welded` — the same
  geometry appended at the end stays three sections, so the join tests are
  really measuring the reorder and not a coincidence;
- an end-to-end test that asks the real `_snap` for the hit and hands it to the
  real commit path (so "the two halves are never wired together" cannot hide);
- no-extrude: set/clear/idempotent/undoable, travel markers untouched, the
  compiler traverses the path with a **flat pump column**, and the three UI
  surfaces.

**Mutation matrix — 10/10 CAUGHT**, sources verified restored byte-identically:
multi-pass restored (the original behaviour) · print points not snapped · the
join reorder removed · a derived point writing a bogus constraint · `no_print`
not undoable · the `start_point` pin dropped · **the joined-to neighbour index
off by one** · the section toggle removed · the width-match button removed · the
no-extrude checkbox removed.

⚠ **One of my own tests was too weak and the mutation run caught it first.** The
neighbour-index mutation (`shapes[dest+1]` / `shapes[dest-1]` swapped) initially
survived as an *assertion* — it was only caught by an incidental `IndexError` in
a different test, because the announce test asserted `"stop" in msg` and both
shapes in the fixture were lines, so naming the wrong one read the same. It now
uses a **circle** as the joined-to shape in both directions, so the message can
only pass by naming the right neighbour.

**Regression — all green:** every sketch suite **369** (333 pre-existing +
36 new, run together) · print-library / quick-print-multi-ink /
multi-object-seam / extrusion-volume-calc / print-setup-routine /
quick-print-travel-split **75** · `import gui.app` + a real `SketchPage` built
offscreen.

**Real-mouse smoke** (not just internals): a press/drag/release two hundredths
of a millimetre off a line's print stop produced a new line whose p0 is exactly
`(10.0, 0.0)`, inserted at index 1, welding into section `[0, 1]`, with a
`coincident` constraint captured — through `mousePressEvent` → `_snap` →
`_commit_draw`.

**Perf measured**, because `_snap` runs on every mouse move and a print STOP on
an overlap-closed loop has to compile that loop's path: **0.07 ms** per `_snap`
with no overlaps, **0.56 ms** at 20 overlap-closed circles, **5.36 ms** at a
pathological 200 — inside a 16 ms frame, and `paintEvent` already pays the same
cost for the markers it draws.

---

## Issues & Decisions

- **`v7.21.3` was already claimed** by concurrent work in this tree
  (`MEBP_v7213…` / `test_v7213_needle_capillary_input.py`, the pulled-capillary
  input freeze), so this landed as **v7.21.4**.
- **`_pass_offsets` kept, not deleted** — one uniform per-pass shape across five
  branches, and one place to change if an explicit multi-pass mode is ever
  wanted.
- **Retired contracts were replaced by their inverse, not deleted.** Both
  multi-pass tests now assert the NEW behaviour and say what they replace, so
  the old behaviour cannot creep back unnoticed.
- **The reorder is scoped to the LINE tool.** Snapping to a print point works
  for every tool (and for dragging an existing endpoint), but only a drawn line
  re-orders — that is what was asked for, and re-ordering on a drag would be
  surprising. Ordering elsewhere is still fixable with **✨ Optimize print
  path**.
- **A `printstart`/`printend` hit with no anchor captures no constraint** rather
  than inventing one. Absent is recoverable; an unresolvable constraint ref is
  not.
- **`line_width_mm` has no compiler consumer any more, by design** — the
  width-match row is what keeps it meaningful. If it is ever wanted as a real
  per-shape flow scale, that is a per-shape multiplier, not a return to
  multi-pass.
- ⚠ **Existing saved sketches change behaviour, deliberately.** Any sketch on
  disk whose `line_width_mm` exceeds its fill pitch was printing several passes
  and will now print ONE — thinner unless the extrusion multiplier is raised.
  That is the requested change, not a migration bug; there is no automatic
  back-fill because the old multiplier that would reproduce the old deposition
  is a per-shape quantity while the multiplier is sketch-wide, so guessing one
  would silently change what a saved print lays down. The shape card names the
  x needed, so re-matching is one click per sketch.
- **Not changed:** the polygon tool does not re-order (its commit is per-vertex);
  the shaded band still shows the predicted, not the declared, width; fills
  still raster at `line_spacing_mm`.

---

## Needs GUI verification on ME3B V1, IN ORDER

1. **The go/no-go:** draw a 0.4 mm line, then set its **Line width** to 2 mm —
   the canvas must still show **one** toolpath line on the geometry you drew,
   NOT five offset rings, and the reported length/volume must not jump.
2. Press **Set extrusion to N× to match**, confirm the shaded band grows to the
   declared width and the Print-parameters card shows the new ×.
3. Draw a second line and bring its end near the **red ■ stop marker** of the
   first — the yellow snap crosshair should grab it, and on release the status
   line must read *"Joined to the stop of line — they print as one bead"* with
   the **Print sequence** card showing **one** section, not two.
4. Repeat aiming at a **green ▸ start marker** — the new line must land *before*
   that shape in the sequence.
5. Join to a **circle's seam** and confirm it welds (no constraint is expected
   there — that is correct).
6. Snap to a plain vertex instead and confirm the order does **not** change.
7. Tick **No extrude** on one shape: the preview turns that run dashed/grey, the
   sequence body reads `· no extrude`, and the reported volume drops.
8. On the **Print sequence** card press a section's **extruding** toggle — the
   whole section goes dashed, the toggle reads **no extrude**, and pressing it
   again restores it.
9. Ctrl+A and use the selection card's **No extrude** to switch everything off
   and back on.
10. **Undo** covers all of it (each of steps 3, 7, 8 is one undo step).
11. Bake the print (**Send to Print Setup**) and confirm Quick Print's preview
    shows the same single-pass geometry and the move-only runs deposit nothing.

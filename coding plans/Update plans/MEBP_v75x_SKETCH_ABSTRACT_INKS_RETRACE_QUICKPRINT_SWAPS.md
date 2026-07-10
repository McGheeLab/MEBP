# MEBP v7.5.x — Sketch abstract inks + retrace-along-bead + Quick-Print ink mapping/swaps + panel rework

## Objective

Four operator-requested Print Builder → **Sketch** improvements (all landed together):

1. **Optimizer path flexibility — retrace along an existing bead.** Instead of
   always lifting (pen-up) to reach the next shape, when its start is a *brief
   length* back along an already-printed bead (same ink), keep the needle DOWN
   and retrace there. The retrace can **move faster** and/or **pause the pump**
   ("pause" = hold pressure, deposit ~nothing — NOT "stop", which triggers the
   quick-move pressure relief).
2. **Abstract inks.** A sketch assigns each shape an **abstract, user-named ink**
   (ordered list, unlimited count) — NOT a physical pump. The sketch tracks only
   the inks it needs; a pump is chosen later at print time.
3. **Quick-Print runtime ink mapping + full sequential ink swaps.** When a loaded
   sketch uses ≥2 abstract inks, the operator maps each → a configured ink; the
   run prints ink 1 fully → waste/wash/buffer → picks up ink 2 → prints, etc.
4. **Right context panel rework.** Collapsible, content-sized print sections; the
   whole panel scrolls as one unit; each section grows to fit its operation list;
   Save/Send pinned below.

## Design decisions (reconciled from two Plan-agent designs + AskUserQuestion)

- **Operator choices (AskUserQuestion):** unlimited named abstract inks · full
  sequential ink swaps in Quick Print · retrace *along the bead* (not a straight
  chord).
- **Ink identity = stable int ids.** `Sketch.inks: list[SketchInk{id,name,color}]`
  + monotonic `Sketch._next_ink_id` (never recycled). `SketchShape.ink_id`
  references by id (robust to reorder/delete).
- **Preview column vs weld gate decoupled.** The Nx7 trajectory has only 3 pump
  columns; sequential swaps mean one ink prints at a time, so NO trajectory-format
  change. The compiler puts a shape's displacement in column
  `ink_order_index(ink_id) % 3` (preview/volume only); the single-needle weld
  break keys on the true `ink_id` (`prev_print_ink`), so two inks colliding mod-3
  still break.
- **Retrace is same-ink, same-Z only** (a shape-to-shape connection).
- **Paused-pump retrace survives the bake→Quick-Print round trip via an epsilon.**
  A paused retrace advances the pump by a negligible fraction (`pump_creep`,
  floored `>1e-9`/segment) so Quick Print's `_travel_mask` (`dp<=1e-9 & moving` →
  travel) never re-classifies it as a pen-up (which would lift + relieve
  pressure). Zero change to `_travel_mask`.
- **Multi-ink Quick Print activates only for sketches using ≥2 abstract inks.**
  Non-sketch / single-abstract-ink prints keep the existing single-ink path.
- **Reuse, don't fork, the swap sequence.** Between-ink swap = existing
  `PickPlaceExecutor.run_post_clean()` (waste→wash→buffer); prep = `run_prep()`;
  pickup = `aspirate_ink()`; end = `run_print_cleanup()`. NOT the legacy
  `PrintPlanExecutor`/`InkSwapStrategy` (would reintroduce ZDIR bugs).

## Files Modified

| File | Change |
|------|--------|
| `SupportClasses/SketchTrajectory.py` | NEW `SketchInk`; `Sketch.inks`/`_next_ink_id` + `ensure_default_ink`/`ink_by_id`/`ink_order_index`/`add_ink`/`rebuild_inks_from_shapes` (+ `__post_init__`); `SketchShape.pump_index`→`ink_id` (migrate legacy `pump_index`→`ink_id`=pump+1); compiler preview column (`ink_order_index%3`) + `prev_print_ink` weld gate; `plan_print_sections` (`ink_id` + `pump_index` preview col + `retrace` break); optimizer by `ink_id`; NEW retrace: `Sketch.overlap_travel_*` fields + `SketchShape.retrace_from` (conditional serialize), `_retrace_polyline`, `move_to(speed, pump_creep)`, compiler `prev_print_path` + retrace emission (first-pass gated), `_stamp_segment_retraces` post-pass in `optimize_print_order` |
| `gui/widgets/sketch_canvas.py` | `set_active_pump`→`set_active_ink`(+`active_ink_id`/`_active_ink_color`); shape creation stamps `ink_id`+colour from the sketch ink; `set_sketch` resets the active ink |
| `gui/pages/print_builder_sketch.py` | NEW **Inks** manager card (add/rename/recolor/delete, reassign-on-delete); per-shape **Ink** picker (`_make_ink_combo`/`_set_ink`) replaces the P1/P2/P3 combo; `_ink_label_color` (reads sketch inks); **panel rework** — one outer `QScrollArea` + pinned Save/Send, sequence card sections are collapsible content-sized `QFrame`s (chevron `QToolButton` + clickable title `QPushButton` + per-shape op rows), per-section collapse state persisted; **Overlap travel** controls on the print card |
| `gui/pages/workflows/quick_print_workflow.py` | multi-ink detection (`_load_selected_sketch`/`_used_abstract_inks`/`_is_multi_ink`); mapping UI section + `_rebuild_ink_mapping_ui`/`_on_ink_map_changed`/`_validate_ink_map`/`_ink_color`; `_ink_groups`/`_group_segments`/`_subpaths_from_array` (refactor)/`_pickup_uL_for_length`/`_segments_length_mm`; `_start_multi_ink_run` + one-worker sequential-swap engine + `_run_group_job_blocking` (blocks on a `threading.Event`); `_PrintBridge.multi_done`+`_on_multi_done`; `_build_settings(pump=)`; `_on_print` multi-ink branch; abort + button-state + setup-status wired for the multi run |
| `tests/` | NEW `test_v75x_sketch_abstract_inks.py` (17), `test_v75x_sketch_panel_rework.py` (4), `test_v75x_sketch_overlap_travel.py` (18), `test_v75x_quick_print_multi_ink.py` (15); updated the `pump_index`→`ink_id` helpers/assertions in the existing sketch suites |
| `CLAUDE.md` | new row in the Existing Update Plans table |

## Implementation Steps

- [x] **Phase 1 — abstract inks** (model + compiler + canvas + page ink manager + tests; legacy `pump_index` migration verified against real `config/prints/*.json`)
- [x] **Phase 2 — panel rework** (one outer scroll, collapsible content-sized sections, pinned buttons + tests)
- [x] **Phase 3 — retrace-along-bead** (opt-in fields, `_retrace_polyline`, compiler emission + epsilon, optimizer post-pass stamping, `plan_print_sections` "retrace", print-card UI + tests incl. the `_travel_mask` round-trip guard)
- [x] **Phase 4 — Quick Print** (detect + mapping UI + validation + grouping + sequential-swap worker + `_build_settings(pump=)` + abort/button/status + tests)

## Testing Notes

Repo uses **unittest** (`QT_QPA_PLATFORM=offscreen` for GUI suites).
- `python -m unittest discover tests -p "test_v75x_sketch_*.py"` → **257 OK**
- `python -m unittest discover tests -p "test_v75x_quick_print_*.py"` → **137 OK**
- Adjacent suites (print-library / extrusion / multi-object-seam / print-setup /
  full-print / pump-relief-bead-model) → **83 OK**.

Coverage highlights: legacy migration (synthesized inks + identical
length/volume/discontinuities); preview column = `ink_order_index%3` while the
weld gate keys on `ink_id` (mod-3 collision still breaks); `_retrace_polyline`
arc-length/cap/None; retrace OFF = byte-identical, ON = 0 discontinuities + the
**round-trip guard** feeding a paused retrace through
`QuickPrintWorkflowPage._travel_mask` (one continuous run, no travel split);
multi-ink detection/grouping/mapping/validation/pump-resolution; the swap worker
call order (`prep` → per group `aspirate(pumpN)`+`job` → `swap` between →
`cleanup`, then `retract`); abort mid-run stops before the next group and ends at
safe Z; `return_home=False`; the calibrated ink well µm reaches `aspirate_ink`
byte-for-byte (no re-sign); single-ink/non-sketch unchanged.

**Needs GUI/HW verification on ME3B V1:** define 2–3 abstract inks + assign per
shape (panel colours by ink, sections collapse/scroll/grow, buttons reachable);
enable Overlap travel + Optimize → preview shows a stay-down retrace (no lift) and
on hardware the pump holds pressure (no relief) during it; Quick Print → load a
2-ink sketch → map inks → run → prints ink 1, swaps (waste/wash/buffer), picks up
+ prints ink 2, ends at safe Z.

## Issues & Decisions

- **Column semantics changed** (ink-order-based, not pump-id-based) → the two
  trajectory column tests in `test_v75x_sketch_trajectory.py` were updated to the
  new contract (a shape on the 2nd abstract ink routes to column index 4).
- **Retrace first-pass gate:** the compiler only retraces on a shape's FIRST pass
  (a shape-to-shape connection), not between a thick outline's own concentric
  passes.
- **`plan_print_sections` retrace = a light annotation:** a "move" break becomes
  a `retrace` break when the shape carries `retrace_from` and the feature is on
  (it does not re-derive the arc-length). `count_discontinuities` (physical lifts)
  correctly stays 0 for a retrace.
- **Multi-ink runtime scope (v1):** activates at ≥2 abstract inks. The between-ink
  swap = `run_post_clean` (waste→wash→buffer); prep once up front (if the Prep
  checkbox is on), cleanup once at end (if on). Per-group print via a discrete
  `PrintManager` the worker blocks on. **Deferred:** a multi-group *syringe-budget*
  pre-flight (the single-ink budget check does not run for multi-ink v1) and
  per-group flow-ceiling precision (`_build_settings` uses the currently-selected
  pump's ceiling as an approximation; the flow VALUE is bore-based and pump-
  independent). Documented; revisit if a real multi-ink run over-aspirates.
- **Abstract inks beyond 3** are allowed in the model and print fine via sequential
  swaps (one ink at a time), despite the trajectory having only 3 preview columns.

# MEBP v7.5.x — Sketch: extrusion-thickness preview + multi-select / group resize

Two operator-requested Print Builder → Sketch enhancements.

## Objective

**(1) Visualize print thickness, tunable by an extrusion multiplier.** Shade a
band around each printed line showing the deposited bead width. 1× = a bead the
width of the needle **inner diameter**; 0.1× / 2× scale it so the operator can
"see the effect." The multiplier also scales the deposited volume of the
compiled/baked print (it's a real extrusion parameter, not just cosmetic) — it
changes *how much* is laid, never *where* the needle goes.

**(2) Select all / lasso-select and resize the selection.** Select every shape
(Ctrl+A or a button), rubber-band ("lasso") marquee to select a subset, and
resize / move / scale the whole selection as a group.

## Files Modified

| File | Change |
|------|--------|
| `SupportClasses/SketchTrajectory.py` | `Sketch.extrusion_multiplier` (default 1.0, in `to_dict`/`from_dict`). `compile_to_trajectory` scales `vol_per_mm` (→ pump columns + `total_volume_uL`) by it, both syringe and no-syringe branches; XY geometry untouched. |
| `gui/widgets/sketch_canvas.py` | **Thickness:** `set_bead_width_mm()` + `_bead_width_mm`; `_draw_toolpath` draws a semi-transparent bead band (world-mm width, decoupled from `line_spacing`) + a crisp centerline on top. **Selection:** `self._selection: set[int]` (primary `_selected` = sole member or −1); `selected_indices`/`selection_count`/`select_all`/`clear_selection`/`_set_selection`; `delete_selected` deletes all; marquee mode (`_commit_marquee`, `_shape_in_rect`) + Ctrl/Shift toggle in `_press_select`; group move (`gmove`) + group resize (`gresize`) via `_transform_shape`/`_group_bbox`/`_group_handle_at`/`_begin_*`/`_apply_*`; `scale_selection()`; `_draw_selection_overlay` (group bbox + corner handle + marquee band); Ctrl+A / Esc-clears in `keyPressEvent`. Per-shape handles suppressed when 2+ selected. |
| `gui/pages/print_builder_sketch.py` | **Thickness:** "Extrusion" spinbox (0.05–5.0×) + live bead readout; `_needle_id_mm` captured in `set_hardware_config`; `_bead_ref_mm`/`_bead_width_mm`/`_apply_bead_width`/`_bead_info_text`/`_on_extrusion_changed`; thickness preview ON by default; multiplier recorded in the baked `extra_params`. **Selection:** `_rebuild_props` shows a group card (`_build_group_card`: count + numeric "Apply scale" + delete) for 2+; "Select all" button in the empty-state hint. |

## Design decisions

- **1× = needle inner Ø** (`NeedleSpec.id_mm`). No needle → fall back to the fill
  pitch (`line_spacing_mm`) so there's still a band.
- The extrusion multiplier scales **deposited volume**, not the toolpath. The
  shaded band is the honest visualization of that (`multiplier × ID`). This
  matches the canonical `extrusion_modifier` bead model used elsewhere.
- "Lasso" is implemented as a **rectangular rubber-band marquee** (bbox
  intersection select) — predictable and pairs naturally with an axis-aligned
  group bounding-box resize handle. (Freeform-polygon lasso is a possible
  follow-up if requested.)
- Group resize is **uniform** (preserves proportions; circles can't scale
  non-uniformly without becoming ellipses), anchored at the selection's
  top-left, dragging the bottom-right handle. Numeric "Apply scale" scales about
  the selection centre.
- Group move uses the **raw** cursor (no object-snap) — `_snap` can exclude only
  one index, so it would latch onto the shapes being dragged.

## Testing Notes

- `python -m unittest tests.test_v75x_sketch_extrusion_thickness` (12)
- `python -m unittest tests.test_v75x_sketch_group_select` (16)
- Regression: all sketch suites + canvas-history = **72 green**.
- Adversarial review workflow over the diff (3 dimensions → verify).
- **Needs GUI verification on ME3B V1**: draw shapes → shaded band tracks the
  extrusion spin; Ctrl+A / drag-a-box selects; drag the corner handle or Apply
  scale resizes the group; single-shape edit + draw/pan/fill still work.

## Status

- [x] Extrusion multiplier model + compiler
- [x] Canvas shaded band + centerline
- [x] Page control + needle-inner-Ø reference + baked provenance
- [x] Multi-select set + select-all + marquee + Ctrl/Shift toggle
- [x] Group move + group resize (handle) + numeric scale + group card
- [x] Group-move snap-latch fix (use raw cursor)
- [x] Tests (28 new) + regression (72 green)
- [x] Adversarial review workflow (3 dims → verify; 2 confirmed, both fixed)
- [x] Fix: shape-commit paths (`_commit_draw`/`_do_fill`/`_commit_polygon`) route through `_set_selection` (were desyncing `_selected` vs `_selection` → new shape drew unselected / no handles / stale highlight)
- [x] Fix: compiled volume bead reference = needle inner Ø (matches the shaded band's 1×; was `line_spacing`≈outer Ø)
- [x] CLAUDE.md table + memory

## Issues & Decisions

- Thickness toggle now defaults **ON** (the feature's whole point is to see
  thickness). Updated `test_v75x_sketch_lift_and_thickness` accordingly.
- Modal-in-offscreen test hazard: `_send_to_print_setup` pops a confirm dialog
  when a shape exceeds the needle-safe ring → patch `QMessageBox.warning` in
  tests that trigger it, else the offscreen modal blocks forever.

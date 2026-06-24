# MEBP v7.5.x — Ink Assignment (Reagent Locations) Well Colors

## Objective

On **Hardware Setup → Ink → Reagent Locations** the well-plate view must:

1. **Match the ink colors.** Each assigned well's **center fill** = the
   reagent's own `InkSpec.color` (the color set in the Ink editor "above"),
   not a generic role color.
2. **Show the type as the outline.** Each assigned well's **border/outline**
   = a color keyed by the reagent's `ink_type` (Ink / Wash / Waste / Buffer /
   Oil), so the operator reads the functional type at a glance while the fill
   reads the specific reagent.
3. **Stop the colors disappearing.** Reported bug: hovering a well and moving
   the mouse away wiped its color; the color only reappeared after switching
   pages and returning.

## Root Cause (bug 3)

`gui/widgets/well_plate_view.py` defines **two** methods named
`update_all_wells`. The *second* (≈ line 679, "v7.3.1-rolecolors") shadows the
first and was the one actually used. It painted the well with
`item.setBrush(QColor(role_color))` **directly**, never updating the item's
stored `_color`/`_role`. `WellGraphicsItem.hoverLeaveEvent` calls
`_apply_style()`, which repaints from the *stale* `self._color`
(`EMPTY_WELL_COLOR`) → the assigned color is wiped the moment the cursor
leaves the well. A full page re-show re-runs `_refresh_reagent_locations()`,
which is why the colors "came back" on tab switch.

## Files Modified

| File | Change |
|------|--------|
| `SupportClasses/PhysicalModels.py` | New `INK_TYPE_COLORS` map + `ink_type_border_color(ink_type)` helper (outline color keyed by reagent type; oil gets its own peach; service types reuse role colors; material types fall back to INK blue). |
| `gui/widgets/well_plate_view.py` | `WellGraphicsItem` gains a persisted `_border_color` + `set_appearance(fill, border, label)`; `_apply_style()` honors the stored border. Removed the buggy duplicate `update_all_wells` (canonical `set_role`-based one remains → hover-safe). New `WellPlateView.update_reagent_appearances({well: (fill, border, label)})`. |
| `gui/pages/hardware_setup.py` | `_refresh_loc_colors()` now passes the ink's own color as fill + `ink_type_border_color()` as border via `update_reagent_appearances()`. |

## Implementation Steps

- [x] Add `INK_TYPE_COLORS` + `ink_type_border_color()` to `PhysicalModels`.
- [x] `WellGraphicsItem`: persist `_border_color`; add `set_appearance`; honor stored border in `_apply_style`; reset `_border_color` in `set_role`/`set_color`.
- [x] Remove duplicate `update_all_wells`; add `update_reagent_appearances`.
- [x] Rewrite `_refresh_loc_colors` to use ink color (fill) + type color (border).
- [x] Tests: `tests/test_v75x_ink_location_well_colors.py`.

## Testing Notes

- `python -m pytest tests/test_v75x_ink_location_well_colors.py` — fill==ink
  color, border==type color, and the hover-leave round-trip preserves color.
- Regression: `tests/test_v75x_ink_location_assignments.py` (page smoke).
- **Needs real-HW verification on ME3B V1** (visual: assign reagents, confirm
  fill matches the ink swatch, outline matches the type, and the color stays
  put when the mouse leaves the well).

## Issues & Decisions

- **Oil has no `WellRole`** (it maps to `WellRole.INK`). To make Oil visually
  distinct in the outline (operator listed it as its own type), the outline
  color is keyed by `ink_type`, not by role. Material types
  (hydrogel/cells/media/granular/custom) fall back to the INK-blue outline.
- Kept the existing `update_all_wells` contract (`{well: (role, label)}`) for
  `print_well_setup.py`; the reagent view uses the new fill+border method.

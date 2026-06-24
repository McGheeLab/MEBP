# MEBP v7.5.x — Ink Well-Type vs Ink-Subtype split + pump ink-list filter

## Objective

Two operator-requested changes to the Ink / Pump setup:

1. **Pump setup must offer only INKS** — the "Inks" checklist on each pump
   (Hardware Setup → Pump) currently lists *every* reagent in the library,
   including the standard service reagents **waste, wash, buffer, oil**. A pump
   never aspirates from a service well, so those must be excluded; only
   printable inks should appear.

2. **Two-level reagent taxonomy (well type → ink subtype)** — split the single
   overloaded `InkSpec.ink_type` into:
   - **Well type** (the primary, behavior-driving classification):
     `ink`, `wash`, `buffer`, `waste`, `oil`.
   - **Ink subtype** (informational only, *only* for the `ink` well type):
     `granular material`, `fluorescent stains`, `cells`, `hydrogel monomer`,
     `media`, `cell removal reagents`, `ELISA Beads`, `Growth Factor Beads`.
     The subtype combo is **editable** (operator may add their own).

   Today `ink_type` conflates both: it holds well types *and* material types
   (`granular`/`cells`/`media`/`hydrogel`/`custom`). The fix keeps `ink_type`
   as the **well type** (so every service-well resolver, `well_role_for_ink_type`,
   and the reagent-location colors keep working byte-for-byte) and adds a new
   `ink_subtype` field carrying the material category.

## Operator decisions (confirmed)

- Subtype list = **editable combo pre-filled with the 8 presets**.
- Subtype scope = **`ink` well type only** (disabled/blank for service types).
- Existing files = **auto-migrate on load** (legacy material `ink_type` →
  `ink_type="ink"` + matching `ink_subtype`; service types unchanged).

## Key invariant preserved

`ink_type` continues to hold the four service strings `wash`/`waste`/`buffer`/`oil`
(and `ink`). This is load-bearing: `_reagent_prep.service_well_names`,
`PickAndPlaceManager` `__waste__/__oil__/__wash__/__buffer__` resolution,
`well_role_for_ink_type`, and `ink_type_border_color` all match on those
strings. **Nothing in the service-well / prep / cleanup path changes.**

The single behavioral branch on a *material* `ink_type` value is
`FlowPhysics.calculate_print_safety` → `has_cells = … or ink.ink_type == "cells"`.
Since loaded "cells" inks migrate to `ink_type="ink"` + `ink_subtype="cells"`,
that check is made subtype-aware (checks both — a superset, no regression).

## Files Modified

| File | Change |
|------|--------|
| `SupportClasses/PhysicalModels.py` | New `WELL_TYPES`, `SERVICE_WELL_TYPES`, `INK_SUBTYPES`, `is_service_reagent()`, `is_printable_ink_type()`, `split_legacy_ink_type()`; `InkSpec` gains `ink_subtype` field + subtype-aware `to_dict`/`from_dict` (migration in `from_dict` only). |
| `SupportClasses/FlowPhysics.py` | Cell-shear check also honors `ink_subtype == "cells"`. |
| `gui/pages/hardware_setup.py` | `InkEditorDialog`: type combo = `WELL_TYPES`; new editable Subtype combo (enabled only for `ink`); `get_ink()` writes `ink_subtype`. Ink table gains a Subtype column. New `_pump_ink_names()` (printable-only) used by `_refresh_pump_ink_exclusions` + `_apply_config_to_ui`. Reagent-location combo/summary show the subtype. |
| `gui/onboarding/wizard.py` | `_load_prefab_inks` routes through `InkSpec.from_dict` (applies migration). |
| `gui/onboarding/prefab_inks.json` | Starter inks rewritten to the well-type + subtype scheme. |
| `tests/test_v75x_ink_well_type_and_subtype.py` | NEW test suite. |

## Implementation Steps

1. `[x]` PhysicalModels: constants + helpers + `InkSpec.ink_subtype` + migration.
2. `[x]` FlowPhysics: subtype-aware cell detection.
3. `[x]` hardware_setup `InkEditorDialog`: well-type + editable subtype combo.
4. `[x]` hardware_setup ink table: Subtype column.
5. `[x]` hardware_setup pump list: `_pump_ink_names()` printable-only filter at both build sites.
6. `[x]` hardware_setup reagent locations: show subtype in combo/summary labels.
7. `[x]` wizard + prefab_inks.json migration.
8. `[x]` Tests.

## Testing Notes

- `tests/test_v75x_ink_well_type_and_subtype.py` — migration round-trips,
  printable filter, subtype persistence, FlowPhysics cell detection via subtype,
  offscreen `InkEditorDialog` build/get_ink, offscreen page pump-list filter.
- Regression: `test_v75x_ink_location_assignments`, `test_v75x_ink_location_well_colors`,
  `test_v75x_quick_print_pick_and_place`, `test_v75x_workflow_settings_popout`,
  flow-physics suite.

## Issues & Decisions

- **Migration in `from_dict` only** (not `__post_init__`): existing tests build
  `InkSpec(ink_type="hydrogel")` directly and assert the value; `__post_init__`
  migration would break them. Disk loads (the only place legacy material types
  enter at scale) all go through `from_dict`. The `InkEditorDialog` already
  produces the new scheme directly, so no migration is needed there.
- A pump previously assigned a service reagent will have it silently dropped on
  the next save (it no longer appears in the checklist) — desired behavior.

**Needs real-HW verification on ME3B V1** (visual: pump ink list shows only inks;
service-well prep/cleanup still resolves; cell inks still flag shear).

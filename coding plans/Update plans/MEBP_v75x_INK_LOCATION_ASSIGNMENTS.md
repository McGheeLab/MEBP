# MEBP v7.5.x — Ink / Reagent Location Assignments (Hardware Setup)

## Objective

Let the operator pin **reagents (inks) to physical plate locations** in
**Hardware Setup → Ink**, reusing the well-picking UX from the main printing
workflow. Each ink already carries a **type** (`InkSpec.ink_type`); the operator
now also picks **one or more wells** (including **rosette sub-wells** like
`A1.a`) where that reagent lives. These locations are **persisted in the hardware
config** and become the canonical "go-to" locations used across printing and
workflows — in particular they **auto-fill the per-print Well Setup** roles/inks.

Resolved design decisions (operator answers):

1. **Type = reuse `InkSpec.ink_type`.** No parallel role enum on the ink. The
   ink-type picker list is extended to include the purpose words the operator
   listed (`ink`, `wash`, `waste`, `oil`) alongside the existing material types
   (`hydrogel`, `cells`, `media`, `buffer`, `granular`, `custom`).
2. **Downstream = persist the map AND auto-fill Print Setup.** A reagent→wells
   map lives in `HardwareConfig`; the per-print `WellSetupModel` is seeded from
   it (non-destructively).
3. **Cardinality = one reagent → many wells; each well holds at most one
   reagent.** Re-assigning a well to a new reagent **moves** it off the old one.

Rosettes are made first on the Plate / Rosette designer pages; once the active
plate is built, `WellPlate.load()` flattens rosettes into sub-wells that appear
in the well-picker exactly like ordinary wells (no special-casing here).

---

## Files Modified

| File | Rationale |
|------|-----------|
| `SupportClasses/HardwareConfig.py` | New `ink_locations: dict[str, list[str]]` field + helpers (`assign_wells_to_ink`, `clear_ink_location`, `unassign_well`, `rename_ink_location`, `well_reagent_map`, `_prune_ink_locations`) + `to_dict`/`from_dict`. |
| `SupportClasses/PhysicalModels.py` | New module helper `well_role_for_ink_type(ink_type) -> WellRole` (maps wash/waste/buffer → those roles; everything else incl. oil + material types → INK reservoir). |
| `SupportClasses/WellSetup.py` | New module fn `seed_assignments_from_ink_locations(model, ink_locations, ink_library=None, destructive=False)` — testable headless auto-fill of a `WellSetupModel`. |
| `gui/pages/hardware_setup.py` | Extend `InkEditorDialog.INK_TYPES`; new **Reagent Locations** group in the Ink sub-page (a `WellPlateView` + ink selector + Assign/Clear + summary); prune `ink_locations` on ink remove/rename; refresh on config/plate change. |
| `gui/pages/print_well_setup.py` | `WellSetupTab.set_hardware_config` calls `seed_assignments_from_ink_locations` (non-destructive) then refreshes the view. |
| `tests/test_v75x_ink_location_assignments.py` | New test suite. |

---

## Implementation Steps

- [x] **Backend (HardwareConfig).** Add `ink_locations` field. Helpers enforce
      one-reagent-per-well (assigning wells to ink B removes them from any other
      ink). `well_reagent_map` is the reverse `well -> ink`. `_prune_ink_locations`
      drops entries for inks not in `ink_library` and de-dupes; called from
      `from_dict`. Serialize in `to_dict`/`from_dict` (skip empty lists).
- [x] **Type → role helper.** `well_role_for_ink_type` in `PhysicalModels.py`.
- [x] **Extend ink-type list.** `InkEditorDialog.INK_TYPES` gains
      `ink/wash/waste/oil` (purposes first, then material types).
- [x] **Seeding fn.** `seed_assignments_from_ink_locations` in `WellSetup.py`
      (non-destructive: only fills `WellRole.EMPTY` wells; sets `ink_name` for
      INK-role wells; skips wells absent from the plate). Returns changed wells.
- [x] **Reagent Locations UI.** New group in `self._sub_layouts["inks"]` below
      the Ink Library: ink selector combo + `WellPlateView` (active plate, colored
      by `well_reagent_map`) + Assign selected / Clear selected / Clear-all-for-ink
      + a summary line. Reloads the plate via `WellPlate.load(active_plate_key)`;
      refreshes on `_apply_config_to_ui`, plate-designer changes (guarded reload
      in `_on_config_changed`), and after edits.
- [x] **Prune on ink CRUD.** `_remove_ink` clears that ink's location; `_edit_ink`
      renames the location key when the ink is renamed.
- [x] **Auto-fill Print Setup.** `WellSetupTab.set_hardware_config` seeds + refreshes.
- [x] **Tests** + run affected suites.

## Status Tracking

**COMPLETE (pending real-HW verification on ME3B V1).** All steps `[x]`.

Tests: `tests/test_v75x_ink_location_assignments.py` (23) green — backend
model (round-trip / one-per-well / reverse map / clear / unassign / rename /
prune / string-tolerance), `well_role_for_ink_type` mapping, non-destructive
seeding (PRINT preserved, off-plate skipped, ink_name set), and an offscreen
Hardware Setup page-build smoke (widgets present, 24-well plate loaded, recolor +
summary). Regression: rosette-flatten / plate-design / calibration-revision /
freeform-warp / sketch-well-boundary / plate-designer / objective-cal /
camera-cal-store / camera-image-correction / hybrid-execution / print-setup-routine
= all green (170+ tests). Two extra offscreen smokes confirmed the live
`WellSetupTab` auto-fill and the `WellPlateView` select→Assign→move→clear UI path.
Pre-existing unrelated failures: `test_v726_print_execution` has 4 errors in its
own `_make_mock_plate` helper (`dict()` of 3-tuples) — present at HEAD, untouched
by this change.

## Testing Notes

- `tests/test_v75x_ink_location_assignments.py`:
  - `ink_locations` round-trips through `to_dict`/`from_dict`.
  - assigning a well to a second reagent moves it (one-per-well).
  - `well_reagent_map` reverse lookup.
  - `clear_ink_location` / `unassign_well`.
  - `from_dict` prunes locations for inks not in the library.
  - `well_role_for_ink_type` mapping (wash/waste/buffer/oil/ink/custom).
  - `seed_assignments_from_ink_locations` is non-destructive (a pre-set PRINT
    well is untouched; EMPTY wells receive role + ink_name; off-plate wells skipped).
  - offscreen Hardware Setup page-build smoke: the Reagent Locations widgets exist
    and `_refresh_loc_plate()` populates a 24-well plate.
- Regression: ink-library / pump-ink suites, well-setup, plate-mosaic.

**Needs real-HW verification on ME3B V1** (drive-to-reagent comes via the
existing calibrated Print Setup / workflow navigation; this change only supplies
the locations).

## Issues & Decisions

- **`oil` has no `WellRole`.** Per decision (1) we do NOT add a parallel role
  enum; `well_role_for_ink_type` maps `oil` (and all material types) to
  `WellRole.INK` (aspirate-from-here). The role color distinguishes
  wash/waste/buffer; oil/ink reservoirs share the INK color but keep their ink
  name + `ink_type`. A first-class `WellRole.OIL` can be added later if needed.
- **Auto-fill is non-destructive** (only `WellRole.EMPTY` wells). Repeated
  `set_hardware_config` calls re-seed any well the operator cleared back to
  EMPTY; acceptable for v1 (hardware locations are defaults, per-print overrides
  win while non-empty).

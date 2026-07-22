# MEBP v7.5.x — Rosette ink/reagent PICKUP resolves to the parent centre, not the sub-well

## Objective

Fix the operator-reported bug: on a Quick Print run with the pick-and-place
preamble, on a plate with a **rosette in well A2**, the needle picks up **buffer**
from its rosette sub-well correctly, but the **ink** pickup drives to the **centre
of the rosette** (the parent well) instead of the assigned **ink sub-well**. Make
the fix general (all reagents/workflows) and give the operator a hard reset for the
reagent landscape so stale membership can't accumulate silently.

## Root cause (confirmed from logs, config, and code — 2026-07-21)

- `hardware_config.ink_locations` maps `ink → list[well]` and is **append-only**
  (`HardwareConfig.assign_wells_to_ink`, `replace=False`, no plate-membership
  pruning).
- An ink pinned to `"A2"` **before** A2 became a rosette lingers as a stale index-`[0]`
  entry; assigning the real sub-well appends → **`"Inert GFP": ["A2", "A2.a"]`**
  (verified in `settings.json`).
- `PlateDesign.compile()` **drops the rosette parent** — the compiled plate has
  `A2.a/b/c` (`is_subwell=True`, `parent_well="A2"`) but **no bare "A2"**.
- Every reagent-pickup resolver took **`wells[0]`** → `"A2"`
  (`quick_print._ink_source_well`, its multi-ink twin, the shared
  `_reagent_prep.service_well_names`, the spheroid copy, and the cell reagent
  pickups).
- The calibrated `well_positions` map re-adds `"A2"` at the sub-well **centroid**
  (`calibration._ploc_open_single_well_mapping`; also `main_well_names`), so `"A2"`
  resolved to the centroid `(87234, 67824)` = mean of A2.a/b/c (matches the logged
  ink-aspirate XY). The Start-gate/status therefore passed silently and read "← A2".
- Buffer only worked because its list was a single correct sub-well.
- **Key asymmetry:** the Well-Setup *seed* path already validates against
  `plate.well_names` (excludes parents) and skips the dangling `"A2"`
  (`WellSetup.seed_assignments_from_ink_locations`) — the pickup resolvers validated
  against `well_positions` (which *includes* the re-added centroid).

## Files Modified

- **`gui/pages/workflows/_reagent_prep.py`** — new `resolve_pickup_well(wells, plate)`
  (prefer a real leaf well in `plate.well_names` over a flattened rosette parent;
  fall back to `wells[0]` → byte-identical legacy). `service_well_names` and
  `resolve_service_positions` gain an optional `plate` param and use the helper.
- **`gui/pages/workflows/quick_print_workflow.py`** — `_ink_source_well` and the
  multi-ink loop (`_validate_ink_map` gate + `_start_multi_ink_run`) use
  `resolve_pickup_well(..., self._plate)`; all `resolve_service_positions` /
  `service_well_names` calls pass `self._plate`.
- **`gui/pages/workflows/cell_targeting_workflow.py`** /
  **`gui/pages/workflows/cell_labeling_workflow.py`** — `_reagent_source_well` uses
  the helper; all service calls pass `self._plate`.
- **`gui/pages/workflows/spheroid_pickup_workflow.py`** — imports
  `resolve_pickup_well` and uses it in the private `_service_well_names` copy.
- **`SupportClasses/HardwareConfig.py`** — new static `_drop_redundant_parents`
  (drop bare parent `"A2"` when `"A2.<x>"` is in the same list — pure string rule,
  plate-independent) applied in `assign_wells_to_ink` and `_prune_ink_locations`
  (the latter runs on `from_dict`, so it's a one-time migration:
  `["A2","A2.a"] → ["A2.a"]`); new `clear_all_ink_locations()`.
- **`gui/pages/hardware_setup.py`** — new **"Clear all wells"** button (dangerBtn) in
  the Reagent Locations group + confirmed `_loc_clear_all` handler (hard reset of the
  whole `ink_locations` map — wipes stale/hidden entries for a fresh ink landscape).
- **`tests/test_v75x_rosette_ink_pickup_well.py`** — new (16 tests).
- `workflow_settings_dialog.py:836` (`service_well_names(hw_config)`) left as-is —
  read-only Locations panel, no compiled plate handy; harmless (falls back to legacy,
  and post-migration the stale parent is gone).

## Implementation Steps

- [x] 1. `resolve_pickup_well` helper + `plate=` on the two `_reagent_prep` service resolvers.
- [x] 2. Quick Print: ink single + multi resolvers use the helper; service calls pass `self._plate`.
- [x] 3. Cell Targeting + Cell Labeling: `_reagent_source_well` + service calls.
- [x] 4. Spheroid: private service copy uses the helper.
- [x] 5. `HardwareConfig._drop_redundant_parents` (assign + `_prune_ink_locations`/from_dict) + `clear_all_ink_locations()`.
- [x] 6. "Clear all wells" button + `_loc_clear_all` handler in Hardware Setup → Ink.
- [x] 7. New test file.
- [x] 8. Run new test + regression suites.
- [ ] 9. Real-HW verification on ME3B V3 (see below).

## Testing Notes

New `tests/test_v75x_rosette_ink_pickup_well.py` (16, all green): `resolve_pickup_well`
prefers the sub-well / falls back correctly / is byte-identical with no plate / leaves
a real "A2" on a non-rosette plate; `service_well_names` + `resolve_service_positions`
prefer the sub-well with a plate; `_drop_redundant_parents`, `assign_wells_to_ink`
self-clean, `_prune_ink_locations` migration, `clear_all_ink_locations`; and an
end-to-end `QuickPrintWorkflowPage` asserting `_ink_source_well()` → `"A1.a"` and
`_ink_source_pos()` == the sub-well µm (NOT the centroid) from a simulated stale
`["A1","A1.a"]` state.

Regression (all green): `test_v75x_quick_print_pick_and_place`, `test_v748_rosette_flatten`,
`test_v75x_plate_types`, `test_v75x_cell_targeting_removal`, `test_v75x_cell_labeling`
(160); `test_v75x_spheroid_pick_place_z`, `test_v75x_workflow_settings_popout`,
`test_v75x_ink_location_well_colors`, `test_v75x_quick_print_multi_ink` (78);
`test_v75x_ink_location_assignments` + colors (35) — incl. append/replace + serialization
round-trip, confirming the self-clean is a no-op on plain (non-sub-well) lists.

Run: `python -m unittest tests.test_v75x_rosette_ink_pickup_well`.

**Real-HW verification (ME3B V3 — the affected machine):**
- Quick Print "Inert GFP" with prep on → after "Prep complete" the ink-aspirate XY in
  `logs/app.log` is the **A2.a sub-well**, not `(87234, 67824)` (the centroid); buffer
  still correct; multi-ink swap picks each mapped sub-well.
- Hardware Setup → Ink: the current stale `["A2","A2.a"]` auto-migrates to `["A2.a"]` on
  next load/save; the new **"Clear all wells"** button wipes every assignment after a
  confirm.

## Issues & Decisions

- **Scope (operator-chosen): all reagents, not just Quick Print ink.** The `wells[0]`
  pattern is shared (service path + cell/spheroid), and buffer only worked by luck; a
  centralized helper fixes the whole class.
- **Self-clean is a plate-independent string rule** ("drop bare `X` when `X.<sub>` is in
  the same list") rather than a plate-membership prune, so it can't wrongly drop a well
  merely absent from the *current* plate in a multi-plate setup.
- **`resolve_pickup_well` keeps `wells[0]` fallback** (no plate / lone parent / nothing
  qualifies) so non-rosette behaviour is byte-identical and there is no regression.
- **"Clear all wells" (operator request):** a hard reset the automatic robustness can't
  substitute for — guarantees no residual stale members when setting up a new ink
  landscape, independent of plate load or the string heuristic.

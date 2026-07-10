# MEBP v7.5.x — Well-Type Presets for Rosette Wells

## Objective

When building a **rosette** (a multi-tube insert dropped into a plate well) the operator sets
each sub-well's physical geometry — and the safety-critical one is the **well height**: how far
the tube top rises above the plate, i.e. the height the needle must retract to (plus a safe
padding) before it can travel over the tube and come down inside it to sample ink.

Standard vessels recur — a **0.1 mL PCR tube** (~20 mm tall, 6 mm top Ø, 3 mm bottom Ø). This
update adds a **well-type preset library** so a sub-well's geometry can be stamped from a named
type in one click, with a built-in PCR-tube type shipped and user-savable custom types.

## What already existed (v7.4.8 — NOT changed here)

The needle-clearance mechanism the operator describes is already wired end-to-end:

`Well.rim_height_mm` → `WellPlate.max_rim_height_mm` → `gui/app.py::_update_insert_clearance`
(`plate_top_z + max_rim + INSERT_CLEARANCE_MARGIN_MM (3 mm)`) → `StageController.set_min_travel_z`
→ floors every `safe_travel_to` / `ensure_retracted_to` retract in the polarity-safe height frame.

The rim-height / ink-Z spinboxes were also already present in the rosette sub-well properties
card (`_edit_context is not None`). **This change reuses that path unchanged** — it only makes
`rim_height_mm` (and the sibling geometry) easy to set from a preset.

## Decisions (confirmed with the operator)

1. **Full preset library** — a well-type store with bundled built-ins + user-savable custom
   types (builtin/user shadow-by-id), mirroring `SupportClasses/PlateTypeStore.py`.
2. **Single diameter (no taper)** — a well type stores height + one diameter (the opening Ø) +
   depth. The tapered bottom Ø is deferred (needle-safe-at-depth is a later follow-up).
3. **Reuse rim-above-top semantics** — the type's height maps straight onto the existing
   `Well.rim_height_mm` ("height the tube top sits above the plate surface"). **No change to the
   safety/clearance math**; the 3 mm margin stays the padding.

Scope: the picker lives on the **rosette sub-well** card (`_edit_context is not None`).
Extending it to top-level plate wells is a trivial later add.

## Files Modified

| File | Rationale |
|------|-----------|
| `SupportClasses/WellTypeStore.py` (new) | Pure-JSON `WellType` dataclass + `WellTypeStore` (builtin/user shadow-by-id, atomic writes, lazy `get_store()` singleton). Zero GUI deps — close mirror of `PlateTypeStore`. |
| `config/hardware/well_types/builtin/pcr-tube-0.1ml.json` (new) | Shipped 0.1 mL PCR-tube preset (Ø 6, depth 20, rim 20, 100 µL). rim_height is a starting guess, tuned per rig. |
| `SupportClasses/PlateDesign.py` | `Well.well_type_id: Optional[str]` (design-time link to the preset); conditional serialize in `_entity_to_dict` (emit only when set → byte-identical legacy) + read in `_entity_from_dict`. `WellInfo`/`compile()` untouched (runtime uses the stamped geometry). |
| `gui/pages/hardware/plate_designer.py` | Well-type `QComboBox` + "Save as well type…" + "Delete" in `_build_well_card`'s rosette block; handlers `_apply_well_type` / `_save_current_as_well_type` / `_delete_selected_well_type`. Added `QTimer` import (defer the panel rebuild off the combo's own signal). |
| `tests/test_v75x_well_type_presets.py` (new) | 18 tests (below). |
| `CLAUDE.md` | Row added to the Existing Update Plans table. |

## Implementation Steps

- [x] `WellTypeStore.py` — `WellType(id, display_name, manufacturer, model, diameter_mm, well_depth_mm, rim_height_mm, ink_z_mm, volume_uL, builtin)` with `__post_init__` coercion, `to_dict`/`from_dict`, `label`; `WellTypeStore(builtin_dir, user_dir)` with `reload`/`get`/`all`/`save_user`/`delete_user`; `safe_id`; `get_store()` singleton.
- [x] Ship `pcr-tube-0.1ml.json` built-in.
- [x] `Well.well_type_id` + conditional serialize / deserialize.
- [x] Designer picker + 3 handlers. `_apply_well_type` stamps `diameter`/`well_depth_mm`/`rim_height_mm` (and `ink_z_mm` only when the type prescribes one), sets `well_type_id`, rebuilds scene, defers the panel rebuild via `QTimer.singleShot(0, …)`. `None` ("(custom)") clears the link and keeps the geometry.
- [x] Tests.
- [x] Docs (this plan + CLAUDE.md row).

## Testing Notes

`python -m unittest tests.test_v75x_well_type_presets -v` → **18 pass**:
- `WellType` to_dict/from_dict round-trip, JSON-string coercion, label (display-name / volume fallback).
- `WellTypeStore` (tmp dirs): built-in load, `save_user` shadows a built-in by id (user always `builtin=False`, persists across a fresh store), `delete_user` re-surfaces the shadowed built-in, `delete_user` of a no-user-file id → False, missing id → None, empty-id save rejected, `safe_id`.
- Shipped `pcr-tube-0.1ml` built-in geometry.
- `Well.well_type_id` serialization: None omits the key (byte-identical legacy), set value round-trips (entity + whole `PlateDesign`).
- Offscreen `PlateDesignerWidget` (rosette mode → drill into A1 → place a sub-well): the sub-well card contains a "(custom)" + "0.1 mL PCR tube" picker; `_apply_well_type` stamps the geometry; "(custom)" clears the link but keeps the geometry.

Regression: `python -m unittest tests.test_v748_rosette_flatten tests.test_v75x_plate_types tests.test_v747_plate_designer` → **80 pass** (the `CalibrationSnapshotStore: MagicMock` line is a pre-existing benign log).

**Needs real-HW/GUI verification on ME3B V1**: Hardware Setup → Rosette → add/edit a rosette →
select a sub-well → pick **0.1 mL PCR tube** (Diameter/Depth/Rim height fill in) → edit rim height
→ **Save as well type…** (custom type appears + persists across restart) → **Delete** removes a
user type. Build a plate with a tall rosette and confirm the needle retracts to
`plate_top + rim + 3 mm` before entering (the existing clearance path picks up the stamped rim
height unchanged).

## Issues & Decisions

- **Deferred panel rebuild.** `_apply_well_type` runs from the combo's `currentIndexChanged`;
  `_rebuild_properties_panel` destroys that combo, so the rebuild is deferred with
  `QTimer.singleShot(0, …)` to avoid deleting the signal's sender mid-emit. The save/delete
  handlers rebuild synchronously (button-driven, matching the existing `_on_drop_standard_insert`).
- **ink_z stamping is non-destructive.** A preset only overwrites the well's `ink_z_mm` when it
  *prescribes* one (`wt.ink_z_mm is not None`); a "no prescription" preset leaves a manually-set
  ink Z intact.
- **`well_type_id` is design-time only.** The runtime `WellInfo`/`compile()` path is untouched —
  it already receives the stamped geometry, so no runtime/safety behavior depends on the id.
- **Taper deferred.** Single-diameter approximation now (opening Ø); tapered bottom Ø +
  needle-safe-at-depth is a future follow-up, as is extending the picker to top-level plate wells.

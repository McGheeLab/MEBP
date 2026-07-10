# MEBP v7.5.x — Selectable Plate Types: per-type Z offsets + auto-loading mosaics

## Objective

On Hardware Setup, make plate **types** first-class and selectable. A given
well-count ships in materially different products — Corning glass-bottom, NEST
plastic-bottom, generic — that share the XY grid but differ in **bottom material
→ different needle Z references** and each has its **own full-plate mosaic
scan**. Two operator-facing outcomes:

1. The automatic Z-needle offsets (**Plate-Bottom, Fast-Move/Safe, Max, and
   Plate-Top Z**) now **inherit from the selected plate type** instead of a
   single global Hardware Setup → Device offset set. (Replace Z stays manual.)
2. Each plate type's mosaic / template / well-training data **auto-loads** when
   that type is selected.

Plus a **learn loop**: after teaching the real Z, save the measured offsets back
onto the plate type so the next selection auto-fills accurately.

## Design

A **plate type** is a thin OVERLAY on a base standard format:
- **geometry** inherited from the base format — the runtime `WellPlate.format`
  stays the base `int`, so every same-format guard / `PLATE_DEFINITIONS` lookup
  is unaffected;
- **identity** = the type id becomes `HardwareConfig.active_plate_key`. Because
  `MosaicStore` / `PlateTemplateStore` / `WellTrainingStore` are all keyed by
  that string, each type's data auto-segregates and auto-loads through the
  EXISTING reload triggers;
- **Z offsets** `{top, bottom, safe, max}` (mm below the needle-cam fiducial) —
  the guess source consumed by `StageController.estimate_plate_z_refs`.

`active_plate_key` precedence becomes **`plate_type_id` → `plate_name` →
`plate_format`**.

## Files Modified

| File | Change |
|------|--------|
| `SupportClasses/PlateTypeStore.py` | **NEW** — pure-JSON singleton store (mirror of `MosaicStore`). `PlateType` dataclass + built-in/user dirs; user entry shadows built-in by id (learn-loop writes a user override; built-in stays pristine). API `get/all/list_for_format/save_user/is_generic`. |
| `SupportClasses/HardwareConfig.py` | Added `plate_type_id: str=""`; `active_plate_key` precedence; serialize in `to_dict`/`from_dict`; `validate()` flags a dangling type id (lazy-imports the store). |
| `SupportClasses/WellPlate.py` | `load()` resolves a plate-type id → `from_format(base_format)` (optional `well_depth_mm` override; **format stays `int`**); lazy + guarded so an unknown id never crashes a caller. |
| `SupportClasses/StageController.py` | `_plate_z_offsets` + `set_plate_z_offsets` + `estimate_plate_z_refs` (returns `plate_max_z`) + `apply_z_convention` extended with **`max`**. New `_apply_active_plate_type_offsets(config)` called at the end of `set_hardware_config` adopts the active type's offsets (generic → leave device-profile offsets). Max Z still **never** written to `safety_limits.z_max`. |
| `gui/pages/calibration.py` | `_zoff_estimate_from_needle_cam` also pre-fills Max Z. New `_apply_plate_type_z_estimates(force=False)` (taught-wins: fills only `None` refs) hooked at the end of `set_hardware_config` (after `_load_calibration` so taught values survive). New learn-loop button + `_zoff_save_offsets_to_plate_type`. `_ploc_plate_key` + sub-well mapping key now derive from `active_plate_key`. |
| `gui/pages/jog_control.py` | `_jog_plate_key` prefers `active_plate_key` (fixes the type-collision: `self._plate.format` is the base int). |
| `gui/pages/hardware_setup.py` | Two-step **Plate Type** card (Format → product combo + offsets readout) above the designer; `_rebuild_config` sets `plate_type_id`; `set_hardware_config` loads the type's base-format geometry + syncs the card. Designer's own picker clears `plate_type_id`. |
| `config/hardware/plate_types/builtin/*.json` | **NEW** — seed products: 6/24/96-well × {glass, plastic} (z_offsets are best-guess until taught). Generic per format is the empty-id rule (no file). |
| `tests/test_v75x_plate_types.py` | **NEW** — 28 tests (store, config precedence/validate, `WellPlate.load`, controller max + offset adoption, identity key, auto-fill no-clobber, learn loop, two-step card). |

## Implementation Steps

- [x] 1. `PlateTypeStore` (built-in + user-shadow + save_user).
- [x] 2. `HardwareConfig.plate_type_id` + precedence + serialize + validate.
- [x] 3. `WellPlate.load` plate-type resolution (format stays int).
- [x] 4. `StageController` Z offsets `max` (set / estimate / apply_z_convention).
- [x] 5. `StageController.set_hardware_config` pushes active type offsets.
- [x] 6. Calibration estimate inherits max + no-clobber auto-fill.
- [x] 7. Mosaic/template/training keys via `active_plate_key` (jog + calibration + sub-well).
- [x] 8. Learn-loop "Save Z offsets to plate type" button.
- [x] 9. Two-step Plate Type UI + `_rebuild_config` + `set_hardware_config` sync.
- [x] 10. Seed 6 built-in plate-type JSONs.
- [x] 11. Tests + regressions.
- [x] 12. This plan doc + CLAUDE.md table entry.

## Testing Notes

`QT_QPA_PLATFORM=offscreen python -m unittest tests.test_v75x_plate_types` → 28
green. Regressions green: `test_v75x_plate_mosaic` (keys; the lone
`test_real_24_well_mosaic` 23/24 CV failure is the documented pre-existing one),
`test_v75x_cal_z_envelope_no_clobber` (z_max invariant), `test_v75x_last_known_calibration`,
`test_v75x_plate_centering`, `test_v75x_plate_orientation_convention`,
`test_v744_calibration_revision`, `test_v75x_printing_mode_calibrated_wells`,
`test_v75x_quick_print_workflow`, `test_v75x_ink_well_type_and_subtype`,
`test_v75x_z_axis_unified_setup`, `test_v75x_workflow_settings_popout`,
`test_v75x_camera_calibration_store`, `test_v75x_pump_plunger_setup`,
`test_v75x_pump_settle_and_prime_time`, `test_v75x_ink_location_*`.

**Needs real-HW verification on ME3B V1:** select Corning-24 → its mosaic loads
+ Z guesses fill from the type; teach the floor → "Save Z offsets to plate type"
updates the type (re-selecting auto-fills the learned values); switch to NEST-24
→ different mosaic + different Z guesses, XY calibration preserved (same base
format, same well names).

## Issues & Decisions

- **Geometry shared, identity distinct.** Keeping `WellPlate.format` as the base
  int (not the type id) leaves all same-format guards / `is_custom` / footprint
  lookups correct; the type id lives only in `active_plate_key`. This made the
  jog/calibration mosaic-key fix mandatory (they previously keyed off
  `self._plate.format`, which collides across types of one format).
- **Boot ordering.** `CalibrationPage.__init__` runs `_load_calibration()` before
  `_propagate_hardware_config`, so taught Z refs are already non-`None` when the
  auto-fill runs → the taught-wins (`None`-only) gate cannot clobber a restored
  calibration. The gate is unconditional (also safe on unrelated config changes).
- **Don't push the print-floor datum on select.** On a type change we push only
  `set_plate_z_offsets` (the guess source). The actual `set_plate_bottom_z/top_z`
  datum is still set by the explicit "Estimate plate Z" click / taught values via
  `app._update_print_floor_datum`, so print-floor behavior is byte-identical.
- **No invalidation banner for a type switch.** A same-format type change keeps
  XY calibration valid (preserved by the same-well-set guard) and auto-loads the
  right mosaic, so a banner would be a false nag. `plate_type_id` IS added to
  `validate()` (dangling-id check).
- **Max Z** is inherited as a print/calibration reference only — never written to
  `safety_limits.z_max` (preserves the v7.5.x inverted-envelope fix).
- **Built-in shadowing** means the learn loop writes a user override under
  `config/hardware/plate_types/user/` with the same id; the bundled built-in is
  never mutated and the selection key is unchanged.

## Addendum — per-plate-key calibration (the v1 limitation, now RESOLVED)

The first cut shared one taught calibration block per format in-session (only the
per-type *offsets* persisted). Follow-up: the **entire** taught calibration (XY
taught wells / warp / affine / Z references / Z-plane / reference markers) is now
**archived per `active_plate_key`** and auto-restored when a plate is selected, so
the calibration *belongs to the plate* (each type — Corning glass vs NEST plastic
vs generic — keeps its own map).

**Design (dual-section, minimal blast radius):** `settings.calibration` keeps
mirroring the **active** plate's flat block (so every existing flat reader —
`print_setup_legacy`, `app.py` snapshot restore, tests — is untouched), and a new
`settings.calibration_by_plate = {key: cal_data}` is the keyed archive. New
`CalibrationPage` helpers: `_calibration_key()` (= `active_plate_key` →
`plate.format`, mirrors `_ploc_plate_key`); `_calibration_archive()` (returns the
archive; **one-time migrates** a legacy flat block under its `plate_format`);
`_reset_calibration_state()` (clears taught state + labels); and
`_activate_calibration_for_key(key)` (loads the archived block or resets to
blank). `_save_calibration(key=None)` dual-writes the flat mirror (active only) +
`archive[key]`; the autosave keys by the active plate. On a plate switch
(`set_hardware_config` key change, and the in-page `_on_plate_changed` combo), the
**outgoing** plate's block is flushed under its old key *before* the geometry
rebuild (so `plate_format`/taught data are still the old plate's), then the
**incoming** plate's block is restored (or reset to blank) after. Composes with
the type-offset auto-fill: a restored plate keeps its taught Z; an uncalibrated
plate's `None` Z refs are filled from the type offsets. Tests:
`tests/test_v75x_plate_types.py::TestPerPlateCalibration` (4) +
`test_v75x_mosaic_orientation_remap` / `test_v75x_last_known_calibration`
unchanged-green (flat mirror preserved). **Needs real-HW verification on ME3B V1**
(teach plate A → switch to B → A's map gone from view; switch back to A → A's
taught wells + Z return; survives restart).

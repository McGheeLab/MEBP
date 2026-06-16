# MEBP v7.5.x — Last-Known Calibration Restore (needle + plate + Z)

## Objective

After a software restart with an **unchanged physical setup**, operators were
forced to redo the *entire* calibration. Restore that lost time: persist a
single **last-known-good calibration snapshot** bundling

- **needle zero** (`zero_position`: X/Y/Z + pump origins),
- **plate** (taught wells, freeform warp, 3-well affine, plate format),
- **Z** (Z-plane fit + Safe/Top/Max/Plate-Bottom/Replace reference heights),

plus a **hardware fingerprint** (device profile, axis map, steps/mm, plate
format, needle gauge) and a timestamp. On the next launch, if the live
calibration came up empty, offer to restore the snapshot as a unit (prompt,
mirroring the existing ZP position-restore prompt). Warn if the fingerprint
shows the setup changed.

**Scope (per user decision):** needle zero + plate + Z only. Camera µm/px and
objective µm/px keep their own identity-keyed stores
(`CameraCalibrationStore`, `ObjectiveCalibration`) and are out of scope here.

## Root-cause findings (why calibration was lost)

1. **Spurious plate-format clobber.** `CalibrationPage._on_plate_changed` clears
   *all* plate calibration (`_taught_a1`, `_plate_warp`, `_three_well_calibration`,
   predicted/calibrated positions, teach points) with **no same-format guard**.
   A programmatic/duplicate combo re-selection (e.g. while reflecting the loaded
   plate format at startup) wipes a just-loaded calibration; the debounced
   auto-save (`_save_calibration`, 500 ms after any `_emit_calibration_data_changed`)
   then persists the nulls over the good `settings.json → calibration` section.
   This matches the observed on-disk state (`taught_a1`/`taught_corner`/
   `taught_third`/`z_plane` all `null`, Z heights intact — the handler does not
   clear `_safe_z`/`_top_z`/`_max_z`/`_plate_bottom_z`).
2. **Needle zero only clean-shutdown-safe.** `zero_position` is persisted on an
   explicit Set Zero or `MainWindow.save_settings()` (clean shutdown). It is not
   bundled with the calibration, so a non-clean exit + a clobbered calibration
   leaves nothing coherent to fall back to.
3. **Three independent persistence paths**, no "restore everything" affordance.

## Design

- **`SupportClasses/CalibrationSnapshotStore.py`** (new, backend, zero GUI deps —
  modelled on `CameraCalibrationStore`): atomic JSON at
  `config/hardware/last_calibration.json`. Holds `version`, `saved_at`,
  `fingerprint`, `zero_position`, `calibration`. Static helpers
  `build_fingerprint(settings)` and `fingerprint_diff(saved, current)`.
  The snapshot is a **stable known-good checkpoint** — written only when a real
  plate calibration is present, and **never** overwritten by an empty state, so
  it is immune to the live-section clobber.
- **De-clobber** (`calibration.py`): add a same-format early-return guard to
  `_on_plate_changed` so re-selecting the current plate is a no-op (a *real*
  plate change still clears, which is correct — different plate ⇒ recalibrate).
- **Snapshot write** (`calibration.py::_save_calibration`): after persisting the
  live section, if `_has_plate_calibration()` is true, write the snapshot
  (zero_position + the same `cal_data` + fingerprint built from `self.settings`).
  Both the manual Save button and the debounced auto-save keep it fresh.
- **Startup restore prompt** (`app.py::_maybe_prompt_calibration_restore`,
  mirrors `_maybe_prompt_zp_position_restore`): fired once per session via
  `QTimer.singleShot` after construction. Only prompts when a snapshot exists
  **and** the live calibration is empty (`not cal_page._has_plate_calibration()`)
  — no nagging when calibration survived. On accept: apply `zero_position` to the
  controller (+ persist `zero_position` section), write the snapshot's
  `calibration` into the live section, `cal_page._load_calibration()` +
  `_emit_calibration_data_changed()` (which re-pushes to Jog/Workflows), refresh
  status. Shows a fingerprint-diff warning when the setup changed.
- **Shutdown flush** (`app.py::save_settings`): write a final snapshot when the
  calibration page holds meaningful data, so an in-progress (un-debounced) teach
  is captured on clean exit.

## Files Modified

| File | Rationale |
|------|-----------|
| `SupportClasses/CalibrationSnapshotStore.py` | **New** — durable last-known-good snapshot store + fingerprint helpers |
| `gui/pages/calibration.py` | Same-format guard in `_on_plate_changed`; `_has_plate_calibration()`; snapshot write in `_save_calibration` |
| `gui/app.py` | `_calibration_restore_prompted` flag; `_maybe_prompt_calibration_restore`; startup trigger; shutdown snapshot flush |
| `tests/test_v75x_last_known_calibration.py` | **New** — backend store + fingerprint + round-trip + de-clobber-guard tests |
| `CLAUDE.md` | Register this plan in the Existing Update Plans table |

## Implementation Steps

- [x] Investigate current persistence (needle/plate/Z), confirm on-disk null state, find clobber path
- [x] `CalibrationSnapshotStore.py` — load/save (atomic) + `build_fingerprint` + `fingerprint_diff`
- [x] `calibration.py` — `_on_plate_changed` same-format guard
- [x] `calibration.py` — `_has_plate_calibration()` helper
- [x] `calibration.py` — `_save_calibration` writes the snapshot when plate cal present
- [x] `app.py` — init flag + startup trigger + `_maybe_prompt_calibration_restore`
- [x] `app.py` — `save_settings` shutdown snapshot flush
- [x] Tests: `tests/test_v75x_last_known_calibration.py`
- [x] Register in `CLAUDE.md`

## Testing Notes

- `python -m unittest tests.test_v75x_last_known_calibration -v` — backend store
  round-trip, atomic overwrite, empty-state never clobbers a good snapshot,
  `build_fingerprint` reads the right settings keys, `fingerprint_diff` reports
  device/plate/needle/axis-map changes.
- GUI prompt + restore exercised manually (QMessageBox, mirrors the ZP
  position-restore prompt which is also manual-only in tests).
- Manual HW check: calibrate → restart → confirm the prompt offers restore and
  needle zero + plate + Z all come back; change plate format and confirm the
  prompt warns the setup changed.

## Issues & Decisions

- **Prompt-only-when-empty** (vs. every startup): chosen so a healthy restart
  (calibration auto-loaded fine) is never interrupted; the prompt targets the
  actual pain (calibration lost). The user picked "prompt on startup"; this is
  the least-naggy faithful reading. Switching to always-prompt is a one-line
  change if desired later.
- **Separate snapshot file** (vs. reusing `settings.json → calibration`): the
  snapshot must be a stable known-good checkpoint immune to the in-session
  live-state clobber, and must bundle `zero_position`; a dedicated file mirrors
  `CameraCalibrationStore`/`objectives.json` and keeps the live section as the
  working state.
- **Needle-zero-only loss** (plate cal intact but zero reset): not auto-prompted
  (live plate cal present ⇒ no prompt). Out of the common path; restoring would
  need a manual affordance — noted for a follow-up if it bites.
- **Marlin/ZP position** on restore: applying `zero_position` is a software
  reference only (no motion). The ZP firmware counter is a separate concern
  handled by the existing ZP position-restore prompt.

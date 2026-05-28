# MEBP — Microscope Objective Calibration Addendum

Scope: per-objective µm/px calibration for the microscope camera via a
slide-marking workflow. Coexists with the existing stage-motion
`PixelCalibrationDialog`. Author: 2026-05-22.

This is a v7.4.x addendum and does not bump the version number on its
own; it ships alongside whichever release the user lands in next. When
the release version is fixed (v7.4.3 / v7.4.4 / v7.5.0), rename this
file to `MEBP_v{from}_to_v{to}_OBJECTIVE_CALIBRATION.md`.

## Objective

Microscope objectives are sold with nominal magnifications (2×, 4×,
10×, 20×) but the real magnification can drift — a "4×" might actually
be 4.12×. The current µm/px pipeline uses *nominal* magnification, so
it inherits this error. Add a slide-marking workflow that empirically
calibrates µm/px per (camera, objective): user puts a slide with two
known marks under the scope, clicks two points on the live feed, drags
the endpoints to refine, enters the real-world distance, computes
µm/px, and persists it.

## Files Modified

| File | Rationale |
|------|-----------|
| `SupportClasses/HardwareConfig.py` | Add `CameraRole.MICROSCOPE`, `SINGLETON_CAMERA_ROLES`, `CameraConfig.current_objective_name`, singleton enforcement in `set_camera_role` |
| `gui/widgets/measurement_camera_view.py` | **New.** `CameraFeedView` subclass with two draggable measurement endpoints (frame-pixel coords, sub-pixel precision, snap radius 12 widget px) |
| `gui/dialogs/objective_calibration_dialog.py` | **New.** Modal dialog: live feed + endpoint placement + distance/unit form + Calculate / Accept |
| `gui/pages/hardware/objective_calibration_card.py` | **New.** Cameras sub-page card: microscope-camera combo, current-objective combo, per-objective status table, Calibrate / Clear buttons |
| `gui/pages/hardware_setup.py` | Mount the new card on the Cameras sub-page; preserve MICROSCOPE in `_rebuild_config`; include `current_objective_name` in the rebuilt `CameraConfig`; propagate `set_camera_manager` to the card; refresh the card in `_apply_config_to_ui` |
| `tests/test_v74x_objective_calibration.py` | **New.** Backend tests: MICROSCOPE round-trip, singleton enforcement, `current_objective_name` round-trip, `ObjectiveCalibrationStore` end-to-end |

No schema change to `config/hardware/objectives.json` — `ObjectiveCalibrationStore` already keys by `(camera_name, objective_name)`.

## Implementation Steps

- [x] `CameraRole.MICROSCOPE` enum + `SINGLETON_CAMERA_ROLES` constant
- [x] `CameraConfig.current_objective_name` field, `to_dict` / `from_dict` round-trip
- [x] `set_camera_role` clears prior holder of any singleton role before assignment
- [x] Backend tests pass (21 cases) — `tests/test_v74x_objective_calibration.py`
- [x] `MeasurementCameraView` with draggable endpoints, snap radius, sub-pixel storage, label overlay
- [x] `ObjectiveCalibrationDialog` with form + Calculate + Accept; writes `ObjectiveCalibrationStore` + `CameraManager.set_um_per_px`
- [x] `ObjectiveCalibrationCard` with microscope-camera combo, current-objective combo, status table, Calibrate / Clear buttons
- [x] Card mounted on `HardwareSetupPage` Cameras sub-page (after Live Camera Sources)
- [x] `_rebuild_config` preserves MICROSCOPE and includes `current_objective_name`
- [x] `_apply_config_to_ui` calls `card.apply_config(self._config)`
- [x] `set_camera_manager` propagates to the card
- [x] `calibration_changed` signal wired to `_on_config_changed`; `um_per_px_committed` wired to `set_calibrated_um_per_px`
- [ ] Manual end-to-end test with real microscope + stage micrometer (deferred to hardware bench)

## Testing Notes

Automated (backend, no GUI): `python3 -m unittest tests.test_v74x_objective_calibration`. All 21 cases pass:
- MICROSCOPE role exists, is in SINGLETON_CAMERA_ROLES, and round-trips through JSON
- Singleton enforcement applies to MICROSCOPE, NEEDLE_X, NEEDLE_Y, PLATE
- UNASSIGNED does not clear other slots; distinct singleton roles can coexist
- `current_objective_name` defaults to None, round-trips, and is backwards compatible with legacy CameraConfig dicts that omit it
- `ObjectiveCalibrationStore` set / get / clear / `all_calibrations_for_camera` round-trip against a temporary JSON file
- Pure-math sanity: 1 mm / 1000 px → 1 µm/px; 500 µm / hypot(300,400) px → 1 µm/px

GUI smoke test (offscreen Qt): `HardwareSetupPage` instantiates, the card is mounted, `current_objective_name` survives `_on_config_changed`.

Manual end-to-end (deferred):
1. Hardware Setup → Cameras: the "Microscope Objective Calibration" card is visible after Live Camera Sources.
2. With no microscope assigned, only the helper label and Microscope dropdown are visible.
3. Pick "Cam 1": the active controls reveal — Currently-installed combo, status table with all objectives showing "—", and the Calibrate / Clear buttons (disabled until a row is selected).
4. Pick "4x" in Currently-installed.
5. Click "Calibrate Selected Objective…": modal opens with the live feed on the left and the form on the right. Window title: "Calibrate Objective µm/px".
6. Click two points on a 1 mm stage-micrometer mark. The pixel-distance label updates. Drag an endpoint within ~12 widget-pixels and confirm it snaps.
7. Enter `1.0 mm`, click Calculate. The Computed µm/px and Effective Magnification labels update.
8. Click Accept Calibration. Modal closes; the row for "4x" now shows the calibrated value and today's date.
9. Switch Currently-installed to "10x" and back to "4x"; observe that the override spinbox on Live Camera Sources tracks the swap (via the `um_per_px_committed` → `set_calibrated_um_per_px` wire).
10. Restart the app and reload the config. Confirm: microscope slot, current objective, and the 4× calibration all persist.

## Revision: Three-section reorganization + custom objectives

After the initial card landed, the Cameras sub-page was restructured
into three explicit named sections and the standard-objectives list
was removed in favor of user-defined entries.

### Additional changes

- `SupportClasses/ObjectiveCalibration.py`
  - `STANDARD_OBJECTIVES = []` — no prepopulated list on first install.
    Existing `objectives.json` files keep their entries on load (the
    persisted "objectives" key wins over the new empty default).
  - New `add_objective(name, nominal_magnification) -> bool` (rejects
    duplicates) and `remove_objective(name) -> bool` (also strips
    matching calibration entries across all cameras). Both persist
    immediately.
- `gui/pages/hardware_setup.py` Cameras sub-page renamed sections:
  - "Camera Configuration" → **"Microscope Camera Setup"**.
    Adds a top-row "Microscope camera:" combo (`_cmb_microscope_slot`).
    Replaces the old hardcoded NIKON_TI2U objective combo with a
    read-only "Installed objective:" mirror populated from the user's
    library; the spec-computed pixel scale derives the magnification
    from the installed objective's nominal value rather than the
    removed combo.
  - "Live Camera Sources" → **"Needle Cameras Setup"**.
  - "Microscope Objective Calibration" → **"Objective Calibration
    Setup"**.
  - `_rebuild_config` derives `CameraConfig.objective_magnification`
    from `ObjectiveCalibrationStore.nominal_magnification(current_objective_name)`
    (falls back to the previous value when no installed objective is
    selected — keeps legacy configs working).
  - `_apply_config_to_ui` rehydrates the new microscope-slot combo
    and the installed-objective mirror.
  - New `_refresh_installed_objective_combo()` helper used after any
    card-driven change.
  - New `_on_microscope_slot_changed` handler writes the MICROSCOPE
    role and refreshes the card.
- `gui/pages/hardware/objective_calibration_card.py`
  - Removed the in-card "Microscope camera:" combo (that role now
    belongs to Section A).
  - Added an `_AddObjectiveDialog` (name + nominal magnification
    spinbox) and Add Objective / Remove Objective buttons.
  - First-added objective auto-installs (so the very first calibration
    isn't gated on a separate selection step).
  - Empty-library banner ("Add an objective below to start…") replaces
    the old per-state helper text when a microscope camera is assigned
    but no objectives exist yet.
- `tests/test_v74x_objective_calibration.py`
  - Old `test_objective_names_include_standards` replaced by:
    - `test_first_time_store_has_no_objectives`
    - `test_add_objective_persists`
    - `test_add_objective_rejects_duplicate_name`
    - `test_remove_objective_clears_calibrations`
    - `test_remove_objective_returns_false_when_missing`
  - Total: 25 tests passing (44 with v7.4.4 suite).

### Verification (revision)

Reproducible end-to-end via the smoke script (offscreen Qt + fresh
temp objectives.json):

1. State 1 — no microscope, no objectives: helper banner visible,
   active block + buttons hidden.
2. Assign Cam 1 in **Microscope Camera Setup**: helper hides; empty
   library banner appears in **Objective Calibration Setup**.
3. Click **Add Objective…**, fill in `My 4x` @ `4.0×`: empty banner
   hides; the installed-objective combo shows `My 4x`; the
   Section A mirror (`cam_objective_combo`) shows the same entry
   (disabled — driven by the card); `current_objective_name = "My 4x"`.
4. Rebuild config: `CameraConfig.objective_magnification == 4.0`
   (derived from the store, no longer from the removed Nikon list).
5. Add a second entry `Custom 10x` @ `10.5×`: first selection stays
   installed; user switches via the combo → `current_objective_name`
   updates; next rebuild yields `objective_magnification == 10.5`.
6. Click **Remove Objective** on a selected row: confirmation prompt,
   then store entry + all matching calibrations gone, combo + table
   refresh.
7. Section titles confirmed: `{Microscope Camera Setup, Needle Cameras Setup, Objective Calibration Setup}`.

## Revision 2: Detection at top + two-card needle section

Second pass on the Cameras sub-page, in response to user feedback:

1. Lift physical-camera detection + slot/role assignment out of the
   needle section and into a new top-of-page section so it reads
   first.
2. Reduce the needle section to exactly two cards — Needle X-view and
   Needle Y-view — each with its own independent calibrate button.
3. Drop the per-slot µm/px override spinbox and the per-slot
   magnification combo (both were hardcoded against the Nikon Ti2-U
   objective list, which contradicts the user-defined objectives
   model from revision 1).

### Final section order on Cameras sub-page

| # | Title | Owns |
|---|-------|------|
| 0 | Camera Detection & Assignment | "Detect Cameras" button + count, per-slot rows (3 slots × {Cam label, source combo, role combo}). Role combo now includes MICROSCOPE; singleton enforcement clears the role from any other slot on reassignment. |
| A | Microscope Camera Setup | Read-only "Microscope camera: Cam X" derived from role assignment in Section 0; camera spec / resolution / installed-objective mirror / pixel scale / override / FOV. |
| B | Needle Cameras Setup | Exactly two cards — Needle X-view and Needle Y-view — each showing the assigned slot, current µm/px, and a "Calibrate µm/px…" button that opens `PixelCalibrationDialog` against the role-resolved slot. PLATE is set via Section 0 but has no dedicated card here. |
| C | Objective Calibration Setup | Unchanged from revision 1 (user-defined objectives, status table, slide-marking calibrate). |

### File-level changes (rev 2)

- `gui/pages/hardware_setup.py`
  - Removed: `_cmb_microscope_slot` combo, `_on_microscope_slot_changed` handler, `_live_cam_umpx_spins`, `_live_cam_mag_combos`, `_on_live_cam_umpx_changed`, `_on_live_cam_mag_changed`, the global `_btn_calibrate_umpx` + `_on_calibrate_umpx`.
  - Added: `_lbl_microscope_assigned` (read-only readout in Section A), `_needle_cards` dict keyed by `CameraRole` (each entry has `assigned`, `umpx`, and `calibrate` widgets), `_refresh_role_derived_displays()` helper, `_on_calibrate_needle(role)` handler.
  - `_on_live_cam_role_changed` now (a) rehydrates *all* role combos after singleton enforcement so the UI matches the canonical config, (b) refreshes the role-derived displays, and (c) re-applies the Objective Calibration card before the standard `_on_config_changed` pipeline.
  - `_rebuild_config` no longer has MICROSCOPE-preservation logic; all roles round-trip through the per-slot role combo state.
  - `_apply_config_to_ui` calls `_refresh_role_derived_displays()` after the role combo restore loop.
  - `set_calibrated_um_per_px` rewritten to update `CameraManager.set_um_per_px` + the needle card display label (the obsolete spinbox is gone).
- No backend changes — all rev 2 changes live in the page module.

### Verification (rev 2)

Offscreen-Qt smoke test (in addition to the 44 backend unit tests):

1. The four top-level group titles appear in order: `Camera Detection & Assignment`, `Microscope Camera Setup`, `Needle Cameras Setup`, `Objective Calibration Setup`.
2. Detection section: 3 per-slot rows, each role combo offers 5 options including `MICROSCOPE`.
3. Needle section: exactly 2 cards (`NEEDLE_X`, `NEEDLE_Y`); both calibrate buttons disabled when no camera carries that role.
4. Setting Cam 1 = `MICROSCOPE` via the top section updates the Section A readout to "Cam 1".
5. Assigning Cam 2 = `NEEDLE_X` and Cam 3 = `NEEDLE_Y` enables both calibrate buttons and shows the slot in each card's "Assigned to" label.
6. Singleton enforcement: changing Cam 2 from `NEEDLE_X` to `NEEDLE_Y` clears `NEEDLE_Y` from Cam 3's role combo and updates the cards accordingly.
7. `HardwareConfig.to_dict()` round-trips the new role state cleanly (e.g. `camera_roles == ["microscope", "needle_y", "unassigned"]`).

## Issues & Decisions

**MICROSCOPE is owned by the new card, not the role combo.** The
v7.4.4 Live Camera Sources role combos expose UNASSIGNED, NEEDLE_X,
NEEDLE_Y, PLATE. MICROSCOPE is deliberately absent because the card
provides the richer UI for it. `_rebuild_config` skips the MICROSCOPE
slot when re-deriving roles from the combos, so changing a NEEDLE_X
combo (etc.) doesn't accidentally clobber the microscope assignment.

**Singleton enforcement is a behavior change for all four singleton
roles.** Previously `set_camera_role` could write the same role to two
slots; now it clears the prior holder. Legacy duplicates in saved
configs are not retroactively normalized — only the next user-driven
role change normalizes them. Test coverage included.

**Calibration store key.** The card uses
`hardware_config.camera_config.camera_spec.name` as the
`ObjectiveCalibrationStore` key, falling back to `camera_{idx}` when
no spec is set. This shares a key across slots whenever the user
hasn't differentiated their camera_specs — acceptable today (one
microscope per machine) and called out as a future-revisit if MEBP
ever grows per-slot camera specs.

**Sub-pixel precision.** Endpoints are stored as floats. Dragging
between integer click positions produces sub-pixel-accurate
coordinates because the widget→frame transform is float-based.

**Coexistence with `PixelCalibrationDialog`.** Both write to
`CameraManager.set_um_per_px`. The new dialog *also* writes the
calibrated value through to `HardwareSetupPage.set_calibrated_um_per_px`
via the card's `um_per_px_committed` signal, so the existing per-slot
override spinbox stays in sync. If the user later toggles that
spinbox by hand, the live value diverges from the store — by design,
the spinbox is the "live" value and the store is the historical record.

**Resolution staleness.** Stored calibrations record the resolution
captured at calibration time. If the active camera resolution
differs, the status table appends a " ⚠" to the date with a tooltip
explaining the mismatch. Recalibrating clears the warning.

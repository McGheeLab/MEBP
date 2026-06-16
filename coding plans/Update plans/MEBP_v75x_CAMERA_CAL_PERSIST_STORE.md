# MEBP v7.5.x — Camera calibration persists across restarts (per-machine store)

## Objective

Fix: **"I calibrate the cameras and then on the next start the calibrations don't load."**

The needle/plate camera µm/px (+ rotation) was stored in `HardwareConfig.camera_calibrations`
— i.e. *inside the swappable bioprinting setup*. Loading a saved hardware-setup file
(e.g. `Standard Bioprinting Setup3.json`, which has no camera calibration) replaced the
in-memory config, and the next auto-save then persisted the empty `{}` to `settings.json`,
**wiping the calibration**. (Confirmed: `settings.json` had `camera_calibrations: {}`
despite the calibration log showing values set, and the device identities were correct.)

A camera's µm/px + rotation is a property of the **physical camera on its USB port**, not
of a bioprinting setup — so it belongs in a **per-machine store**, exactly like the
microscope's per-objective µm/px lives in `objectives.json`.

## What changed

1. **New `SupportClasses/CameraCalibrationStore.py`** — a machine-level JSON store
   (`config/hardware/camera_calibrations.json`) keyed by stable device identity
   (DirectShow path = model + USB port). Holds `{um_per_px, rotation_deg, name, date}`
   per camera, plus a `role → identity` **assignments** map. Singleton via `get_store()`,
   mirrors `ObjectiveCalibration`.
2. **`hardware_setup` writes/reads the store, not the config** — `set_calibrated_um_per_px`
   persists to the store (and no longer triggers a full `_on_config_changed` rebuild);
   `_restore_calibration_for_slot` reads from the store. So a setup-file load can't wipe it.
3. **Auto-restore on the next start** — camera *sources* aren't persistent device handles,
   so the slot↔camera assignment was lost each session and the user had to re-pick. Now
   `_remember_assignment` records `role → device identity` whenever a source/role is set,
   and `_auto_assign_sources_from_store` (run after Detect) re-selects each slot's source
   by matching the stored identity — which fires the source-changed handler and restores
   that camera's µm/px + rotation automatically. New inverse helper
   `camera_identity.source_for_identity`.
4. **Removed the vestigial `HardwareConfig.camera_calibrations` field** (+ its
   to_dict/from_dict) so nothing looks like it persists camera calibration in the swappable
   config. A legacy key in an old saved config is ignored.

### Follow-up fix — auto-restore was gated on a *manual* Detect click

The store + auto-assign logic above was correct, but nothing ran camera
**detection** automatically on restart — `_auto_assign_sources_from_store` only
fires inside `_on_detect_live_cameras`, which was wired solely to the **Detect
Cameras** button. So on the next start the store had both the calibrations and
the `role → identity` assignments (confirmed in `camera_calibrations.json`), yet
nothing applied them until the user manually clicked Detect on the Cameras
sub-page. The user's expectation — *"assume the camera setup remains the same"* —
means the restore must be automatic.

Fix: `HardwareSetupPage` now runs the detect **once, automatically** when it
first becomes available/visible, *only if a setup was remembered*:

- New `_maybe_auto_detect_cameras()` — gated on `CameraCalibrationStore.all_assignments()`
  being non-empty (a first-ever run with nothing remembered still waits for a
  manual Detect — we don't probe cameras unprompted). One-shot via the
  `_auto_detect_done` guard; defers `_on_detect_live_cameras` through
  `QTimer.singleShot(0, …)` so the page paints before the blocking camera probe.
- Called from both `set_camera_manager` (fires at startup — the manager is wired
  before the window is shown, so the calibration is live for every page, not just
  Hardware Setup) and `showEvent` (belt-and-suspenders; idempotent via the guard).
- New `CameraCalibrationStore.all_assignments()` public accessor for the gate.

Hardware Setup is the startup landing page (hardware-first), so this effectively
restores the remembered needle-camera setup + µm/px on launch with no clicks.

## Files Modified

| File | Change |
|------|--------|
| `SupportClasses/CameraCalibrationStore.py` | **New** per-machine store (cameras + assignments), `get_store()` singleton. *(Follow-up: added `all_assignments()` accessor.)* |
| `SupportClasses/HardwareConfig.py` | Removed `camera_calibrations` field + serialization (moved to the store; legacy key ignored). |
| `gui/widgets/camera_identity.py` | New `source_for_identity()` (inverse of `identity_for_source`) to map a stored identity back to a live source for auto-assignment. |
| `gui/pages/hardware_setup.py` | `set_calibrated_um_per_px` → store; `_restore_calibration_for_slot` ← store; new `_remember_assignment` / `_auto_assign_sources_from_store`; wired into source-change, role-change, toggle, and detect. Tuple combo lookup via `itemData` (QComboBox.findData can't match tuples). *(Follow-up: `_maybe_auto_detect_cameras()` + `showEvent` + `_auto_detect_done` guard → one-time auto-detect at startup when a setup is remembered.)* |

## Testing Notes

- `tests/test_v75x_camera_calibration_store.py` (12): store set/get/clear, rotation
  preserved on µm/px-only update, assignments, **disk round-trip**, `source_for_identity`
  round-trip, an **integration test** that calibrates → loads a setup file (no wipe) →
  auto-restores into a fresh CameraManager in "session 2", and **`TestAutoDetectOnShow`**
  (2): a remembered setup schedules the detect exactly once (verified after a real
  `QApplication.processEvents()` drives the deferred `QTimer`), while no assignments → no
  auto-detect.
- Updated `test_v75x_camera_cal_liveview.py` / `test_v75x_camera_rotation.py` — the
  removed-field tests replaced with "legacy key ignored" assertions.
- Manual: calibrate needle cameras, restart, click **Detect Cameras** on Hardware Setup →
  Cameras → the sources auto-select and the µm/px (`… @ 45°`) reappears with no re-pick.

## Issues & Decisions

- **Root cause was persistence location, not the identity store** — identities were
  correct in the log; the data was wiped by a setup-file load + auto-save.
- **Per-machine store** mirrors the microscope's `objectives.json` model and is robust to
  swapping bioprinting setups.
- **Auto-assignment by identity** makes it "just load" after one Detect click (cameras must
  be enumerated regardless). `role → identity` is the key (roles persist in the config;
  identity is port-stable). Moving a camera to a different port = new identity → re-pick.
- **No migration needed** — the field existed only in this uncommitted work and the user's
  `settings.json` already showed `{}`. Existing users recalibrate once; it then sticks.
- **No `findData` for tuples** — source combo data are tuples; matched via `itemData`
  comparison (same quirk noted in the quick-print workflow).

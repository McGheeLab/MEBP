# MEBP v7.5.x — Z-Bottom Auto-Cal: manual first-spot seed + live microscope feed

## Objective

Make Z needle-bottom auto-calibration **start from a manually-calibrated first
spot** instead of the fragile `top_z − well_depth` estimate, and put a **live
microscope feed on the Needle Offset Calibration tab** so the operator can find
focus by eye for that first spot.

User-confirmed design (3 clarifying questions):

1. **Workflow** — *Manual first, auto rest (seeded).* The operator drives to the
   first calibration well, jogs Z to the well bottom while watching the live
   feed, and records it. Auto-cal then runs **only the remaining wells**, using
   the taught Z as the search seed.
2. **Approximate plate Z source** — *Derive from the manual first spot.* There is
   no separate "approximate plate Z" field; the single manually-taught first-well
   bottom **is** the seed for the rest.
3. **Live feed** — *Passive view only.* A live `CameraFeedView` with crosshair so
   the operator can watch the needle approach focus while jogging Z (via Xbox /
   Jog page, as with the existing manual Z-teach). No click-to-jog.

### Why this matters (root cause)

The legacy auto-Z (`_auto_z_tick`) computes the search region as
`estimated_bottom = top_z − well_depth` and then **steps Z *down* (subtract)** to
descend toward the plate. That stepping direction is hardcoded for a
conventional machine (`ZDIR = +1`, up = larger Z). On **ME3B V1 the needle
descends as zero-ref Z *increases*** (`ZDIR = −1`), so the legacy sweep walks the
needle *away* from the plate and the `top_z − well_depth` math is error-prone.
Anchoring the search to a human-confirmed bottom + making the sweep
**polarity-general** removes both failure modes.

## Files Modified

| File | Rationale |
|------|-----------|
| `gui/pages/calibration.py` | Add live microscope feed + manual first-spot teach UI to `_build_z_offset_tab`; seed state (`_zauto_seed_z`, `_zauto_first_well`); rework `_start_auto_z_cal` / `_auto_z_tick` to be polarity-general (height-above-reference parametrization) and to skip the manually-taught first well when seeded; tab-change hook to start the microscope feed; rebind the feed in `_refresh_ploc_view`. |
| `tests/test_v75x_z_autocal_manual_seed.py` | New — seeded search-region math (polarity-general), first-spot record/seed gating, skip-first-well in seeded mode. |
| `coding plans/Update plans/MEBP_v75x_Z_AUTOCAL_MANUAL_SEED.md` | This plan. |

## Implementation Steps

- [x] **1. Polarity-general sweep helpers.** Reuse `StageController.plate_relative_to_zref` (already polarity-general via `ZDIR`). Rework the coarse/fine sweep to walk a **height above a reference bottom** `h` (mm, + = away from plate / safe), converting each `h` to a zero-ref Z. "Toward plate" = decreasing `h`; floor = `h ≤ floor_h`.
- [x] **2. Seed state.** Add `_zauto_seed_z` (zero-ref mm bottom of first well, or `None`), `_zauto_first_well` (well name), `_zauto_approach_margin` (1.0 mm above bottom), `_zauto_tilt_margin` (0.5 mm below seed allowed for plate tilt).
- [x] **3. Live microscope feed.** Add `_zoff_live_view` (`CameraFeedView`, passive, crosshair) bound to the Microscope slot (`_ploc_live_cam_idx`), in a vertical splitter under the tab controls. `_zoff_ensure_live_camera()` mirrors `_ploc_ensure_live_camera`. Rebind in `_refresh_ploc_view` alongside `_ploc_live_view`. Start it on `_workflow_tabs.currentChanged` → Needle Offset tab.
- [x] **4. Manual first-spot UI.** New "Step 1 — Calibrate first spot manually" group on the tab: **Go to first well** (`_zauto_goto_first_well`, safe-navigate XY at safe Z) + **Record first-spot Z** (`_zauto_record_first_spot`, captures current zero-ref Z → `_zauto_seed_z` + `_auto_z_results[first]` + `_z_teach_points[first]`) + status label.
- [x] **5. Seeded auto-cal.** `_start_auto_z_cal`: when `_zauto_seed_z` is set, pre-seed `_auto_z_results[first]`, start at `well_idx = 1` (skip the taught first well), reference = seed; else legacy behavior (reference = `top_z − well_depth`). Both paths use the new `h`-parametrized sweep. "Run remaining wells" button on the tab requires the seed.
- [x] **6. Tests** — `tests/test_v75x_z_autocal_manual_seed.py`.

## Status Tracking

All steps `[x]`. Implementation confined to `gui/pages/calibration.py` (+ one
import of `plate_relative_to_zref`). `_auto_z_sweep_end` left in `__init__` as a
now-unused legacy attribute (harmless).

## Testing Notes

- `python -m unittest tests.test_v75x_z_autocal_manual_seed` -> 13 pass.
- Regression: `test_v731_integration` + `test_v731_jog_navigation` build the full
  `CalibrationPage` (incl. the reworked tab); these + the v7.5.x calibration
  modules (cal-z-envelope, plate-location ×2, last-known-calibration,
  camera-rotation) are green (148 tests).
- Full suite: 760 tests; 13 pre-existing failures in `test_v726_print_execution`
  / `test_v73_trajectory_planner` (MagicMock `<` float in
  `PrintTrajectoryPlanner._move_z`) + `test_sim_vs_hardware` flakes — confirmed
  pre-existing (fail identically with this change `git stash`ed) and have no
  import path to calibration.
- Headless: unbound `CalibrationPage` methods on duck-typed `SimpleNamespace`
  stubs (same pattern as `test_v75x_plate_location_manual_click_rim.py`);
  `QMessageBox` + `QTimer` patched so no Qt event loop is needed.
- Real-HW: on ME3B V1, drive to A1, jog Z to the well bottom watching the feed,
  Record; then Run remaining — confirm it descends *toward* the plate (increasing
  zero-ref Z) at the other wells and stops at the focus peak near the seed.

## Issues & Decisions

- **Legacy path kept for back-compat.** The Custom-tab "Auto Z-Cal" button still
  calls `_start_auto_z_cal`; with no seed it uses the `top_z − well_depth`
  reference. The sweep is now polarity-general for *both* paths, which also fixes
  the legacy ME3B wrong-direction walk as a side benefit.
- **No separate "approximate Z" field** — per user choice (3), the manual
  first-spot teach is the single source of the seed.

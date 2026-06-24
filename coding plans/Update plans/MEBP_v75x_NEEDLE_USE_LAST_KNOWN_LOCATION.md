# MEBP v7.5.x — Needle Location "Use last known location" button

## Objective

Add a **"✓ Use last known location"** button on the Calibration → **Needle
Location** tab so that, when the operator is sure nothing about the needle, its
mounting, or the side cameras has changed since the last session, they can
restore the saved needle reference **without re-doing the camera-based
Center & Save centering workflow**.

Scope (per operator decision): **needle reference only** — needle zero (the
workspace reference frame) + the needle-cam Z fiducial + the saved needle XY.
Plate / Z calibration are out of scope (they have their own restore — the
startup `MainWindow._maybe_prompt_calibration_restore` snapshot prompt).

## Files Modified

| File | Rationale |
|------|-----------|
| `gui/pages/calibration.py` | New `_needle_loc_btn_last_known` button in the Needle Location tab's goto row + new `_needle_loc_use_last_known()` handler. |
| `tests/test_v75x_needle_location_quick_move.py` | New `TestUseLastKnown` class (+ a richer `_RichMB` QMessageBox fake that supports the instance `addButton`/`exec`/`clickedButton` pattern; `setStyleSheet` no-op added to `_FakeLabel`). |

## Implementation

- The handler reads the three persisted values straight from `settings` (the
  same place the needle calibration writes them), so it is immune to whatever
  the live page state happens to be:
  - `device_profile.needle_loc_xy_um` — absolute Prior stage µm (the saved
    needle location, also drives the existing "⤵ Go to needle location").
  - `device_profile.needle_cam_z` — the needle-cam Z fiducial (user frame).
  - `zero_position` section — the needle zero / workspace reference frame
    (persisted on Set Zero and on clean shutdown).
- It is a **software-only** restore (NO stage motion) — it re-establishes the
  software reference; the operator can verify the needle is in view via the
  existing quick-move and re-center only if it looks off.
- Flow: gather → if nothing saved show an info dialog → otherwise a confirm
  dialog that explicitly warns "only if nothing changed" and lists what will be
  restored → on accept apply **needle zero first** (so the derived
  `needle_origin_um` uses the restored zero) → re-apply needle-cam Z to the
  controller (`set_needle_cam_z_user`) and update the Needle-Offset-tab label →
  adopt the saved XY + derive/display `needle_origin_um` (green, "(last known)")
  → `_emit_calibration_data_changed()` + `_needle_loc_update_goto_ui()` +
  a green confirmation banner.
- The button is always enabled and handles the empty case with a friendly
  message (rather than being greyed out), so the affordance is discoverable.

## Status Tracking

- [x] Add `_needle_loc_btn_last_known` to the goto row (`_build_needle_location_tab`)
- [x] Implement `_needle_loc_use_last_known()` (gather → confirm → apply)
- [x] Tests: `TestUseLastKnown` (5) — nothing-saved info, settings-None warn,
      full restore on accept, cancel = no-op, cam-Z-only suffices
- [x] Offscreen page-build smoke (button text/enabled, handler runs)
- [x] Regression: needle quick-move / needle-center-direction / autocal-tab
      suites green (48 tests)

## Testing Notes

```
python -m unittest tests.test_v75x_needle_location_quick_move   # 22 (incl. 5 new)
QT_QPA_PLATFORM=offscreen python -m unittest \
  tests.test_v75x_needle_center_direction_z \
  tests.test_v75x_plate_z_autocal_tab                            # green
```

Real-HW verification on ME3B V1: restart with an unchanged setup, open Needle
Location, click **Use last known location**, confirm; the needle origin /
needle-cam Z labels go green, then use **Go to needle location** to drive there
and confirm the needle re-enters both side views without re-centering.

## Issues & Decisions

- **Scope = needle reference only** (operator choice), distinct from the
  full-snapshot startup restore. The needle calibration does not itself set
  `zero_position` (that's the Jog page's Set Zero), but the restore re-applies
  the persisted zero so the whole needle reference is coherent as a unit.
- **No motion** — restoring a software reference must never move the stage;
  the existing "Go to needle location" quick-move remains the (separate)
  motion action.
- Source is `settings` (not `CalibrationSnapshotStore`) because the snapshot
  store is gated on a real *plate* calibration, whereas the needle reference
  values persist independently in `device_profile.*` / `zero_position`.

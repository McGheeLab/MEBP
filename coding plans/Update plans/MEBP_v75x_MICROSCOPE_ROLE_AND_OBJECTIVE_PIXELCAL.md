# MEBP v7.5.x — Drop PLATE camera role; objective calibration via stage-motion

## Objective

Two related camera cleanups requested by the user:

1. **No PLATE role — use MICROSCOPE.** The microscope camera *is* the camera that
   looks at the plate, so the separate `CameraRole.PLATE` ("overhead cam over the
   well plate") is redundant. Remove it and route every former PLATE consumer
   (the plate edge-fit auto-cal, the role combo, badges) to `CameraRole.MICROSCOPE`.

2. **Objective calibration uses the same stage-motion µm/px as the needle cameras.**
   Today objectives are calibrated with the slide-marking `ObjectiveCalibrationDialog`
   (mark a known distance on a stage micrometer). Switch to the stage-motion
   `PixelCalibrationDialog` (move the stage a known distance, phase-correlate the
   resulting image displacement / "flow") — identical to the needle-camera flow.
   The microscope looks at the plate, so moving the stage moves the plate image
   under the scope, giving the displacement to solve µm/px (and the in-plane
   rotation the live-target picker needs).

## Files Modified

| File | Rationale |
|------|-----------|
| `SupportClasses/HardwareConfig.py` | Remove `CameraRole.PLATE` from the enum + `SINGLETON_CAMERA_ROLES`; `from_dict` migrates a legacy `"plate"` role string → `MICROSCOPE` (otherwise it would silently fall back to UNASSIGNED). |
| `gui/pages/hardware_setup.py` | Drop the "Plate (overhead)" role-combo item; remove the `_role_badge_props` PLATE branch. |
| `gui/pages/calibration.py` | Plate edge-fit auto-cal reads `CameraRole.MICROSCOPE` (was PLATE); wording "Plate camera/role" → "Microscope camera/role". |
| `gui/pages/hardware/objective_calibration_card.py` | `Calibrate Selected…` launches the stage-motion `PixelCalibrationDialog` (was `ObjectiveCalibrationDialog`); preflight checks (microscope cam running + stage connected); stores the measured µm/px (+ rotation) per-objective via `ObjectiveCalibrationStore`; new `controller_getter`; updated wording. |
| `SupportClasses/ObjectiveCalibration.py` | `set_calibration` gains optional `rotation_deg`; stored in the entry (backwards compatible). |
| `tests/test_v744_calibration_revision.py` | PLATE → MICROSCOPE; add a legacy-`"plate"`→MICROSCOPE migration test. |
| `tests/test_v74x_objective_calibration.py` | Singleton-enforcement tests use MICROSCOPE/NEEDLE roles (drop PLATE); add rotation round-trip in the store test. |

## Implementation Steps

- [x] `HardwareConfig`: remove `CameraRole.PLATE` (enum + singleton set); migrate `"plate"`→`MICROSCOPE` in `from_dict`.
- [x] `hardware_setup`: remove PLATE combo item + badge branch; wire `controller_getter` into the card.
- [x] `calibration`: PLATE→MICROSCOPE for edge-fit auto-cal + wording.
- [x] `ObjectiveCalibration.set_calibration`: optional `rotation_deg`.
- [x] `objective_calibration_card`: switch to `PixelCalibrationDialog`, wire controller, store result per-objective (+rotation), preflight checks, wording.
- [x] Update both test files (PLATE→MICROSCOPE; legacy-plate migration test; rotation round-trip); full suite green apart from the same pre-existing failures (10 print/trajectory `MagicMock<float` errors; 3 sim state-persistence flakes).

## Testing Notes

- `CameraRole.PLATE` no longer exists; a config with `"camera_roles": [..., "plate"]`
  loads as MICROSCOPE on that slot (migration test).
- Objective calibration: with the microscope camera running and stage connected,
  `Calibrate Selected…` opens the stage-motion dialog; accepting stores the measured
  µm/px under `(camera_key, objective)` and pushes it live.

## Issues & Decisions

- The slide-marking `ObjectiveCalibrationDialog` + `MeasurementCameraView` are left
  in the tree but no longer wired from the card (deprecated). `MeasurementCameraView`
  is still imported by `live_target_picker`, so it stays.
- Microscope µm/px remains **per-objective** in `ObjectiveCalibrationStore` (not the
  identity-keyed `camera_calibrations`, which is per-camera and right for needles).
  Rotation is stored alongside the per-objective entry and pushed to the manager on
  objective swap so the live-target picker maps clicks correctly.

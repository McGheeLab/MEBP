# MEBP v7.5.x — Re-anchor moves mosaic + wells together, and calibrated camera-vs-stage rotation for the click mapping

## Objective

Two operator-requested plate-calibration changes:

1. **The mosaic is scanned once per plate TYPE, not per use.** When the plate is remounted (or the
   stage frame differs from when the scan was taken) the whole map is offset from reality. The
   operator's fix is **"Re-anchor map"**. Previously it shifted only the **well centres**, leaving the
   **mosaic image** in place → the two disagreed. Re-anchor must now move **the mosaic image AND the
   well centres as one unit**, and it must be **no-travel** (robust to a large remount offset).

2. **Account for the camera's orientation.** `CameraManager.pixel_to_stage_offset` mapped a live-view
   click to a stage offset with a plain identity mapping (image-right = stage +X, image-down = +Y),
   ignoring the camera's mounting rotation. If the microscope camera is mounted rotated relative to
   the stage axes (≈180°, but "not exactly" — any angle), a live-view click drives the correction the
   wrong way. The operator asked to **calibrate the camera's true rotation vs the stage axes** and
   apply it so live-view clicks map in the correct direction.

**Operator decisions:** re-anchor flow = **no-travel**; orientation = a **calibrated arbitrary
rotation angle** measured vs the stage (not a discrete flip); **correct the click mapping only** —
do not rotate the displayed live feed and do not touch the mosaic build / raw frames.

## Key design finding (makes both parts low-risk)

- `MosaicBuilder` places every tile purely by its stage position with **no per-frame rotation**
  (image col→stage +X, row→stage +Y). Rotating frames *at capture* would scramble the mosaic and
  break well-detection back-projection. **Therefore the rotation is applied ONLY to the click→stage
  mapping (`pixel_to_stage_offset`)** — the mosaic build is byte-identical, and re-anchor's
  translation shifts the mosaic overlay + wells together.
- The rotation **measurement + storage + restore + live-push** infrastructure already existed
  (`PixelCalibrationDialog` measures `result_rotation_deg` via stage-motion phase correlation;
  `ObjectiveCalibrationStore` / `CameraCalibrationStore` persist `rotation_deg`;
  `CameraManager.get/set_rotation_deg`; `hardware_setup._restore_calibration_for_slot` restores it).
  The **only** missing wire was applying the angle in `pixel_to_stage_offset`.

## The rotation math (sign-locked)

`PixelCalibrationDialog.plus_column_direction_deg` documents the model **content shift
`p = −scale·R(α)·m`** with the stored angle **`θ = −α`** and `scale = 1/µm_per_px`. Inverting it, the
stage move that centres a feature clicked at pixel offset `p` is:

```
(dx_um, dy_um) = µm_per_px · R(θ) · (dx_px, dy_px)      # standard CCW R(θ)
```

At `θ = 0` (or `None`, uncalibrated) this reduces **exactly** to the legacy identity
`(dx_px·µm_per_px, dy_px·µm_per_px)` — so aligned/uncalibrated cameras are byte-identical (no
regression). A round-trip unit test drives the real `plus_column_direction_deg` to derive `θ` and
proves a clicked feature is centred, locking the sign.

## Files Modified

| File | Change |
|------|--------|
| `gui/widgets/camera_manager.py` | `pixel_to_stage_offset` now applies the calibrated camera rotation `R(θ)` (getattr-guarded; identity at θ=0/None). Added `import math`. |
| `SupportClasses/CameraCalibrationStore.py` | New `get_rotation` / `set_rotation` — write ONLY `rotation_deg` (preserve `um_per_px`/`image_correction`/`hw_controls` siblings; clear on None). |
| `gui/pages/calibration.py` | Re-anchor converted to **no-travel** (`_ploc_start_reanchor` step order swapped to live→overview, Safe-Z travel gate dropped, `_ploc_safe_goto` no longer called); `_ploc_reanchor_live_click` (step 1: capture live feature_abs) and `_ploc_reanchor_overview` (step 2: E = live − map → apply) rewritten; overview click handlers gate on re-anchor stage (no queue pollution). New `_ploc_shift_mosaic_by(ex,ey)` shifts + persists the mosaic overlay; called from `_ploc_apply_global_translation` (wells + mosaic move by the same E); `_ploc_store_manual_align` refactored to reuse it. Button tooltip/labels updated. |
| `gui/pages/hardware/objective_calibration_card.py` | New **"Calibrate orientation…"** button + **"Camera rotation vs stage: θ°"** readout for the microscope slot: measures rotation via `PixelCalibrationDialog` (no µm/px recal), pushes to the manager, persists per camera identity via `CameraCalibrationStore.set_rotation`, and keeps the current objective's per-objective rotation consistent. µm/px calibration also refreshes the readout. |
| `gui/pages/hardware_setup.py` | `_restore_calibration_for_slot` now restores `rotation_deg` **independent of µm/px** (before the µm/px early-return), so an orientation-only calibration loads. |
| `SupportClasses/ObjectiveCalibration.py` | `set_calibration` no longer drops a stored `rotation_deg` on a µm/px-only update (review fix, pre-existing defect). |
| `tests/test_v75x_reanchor_mosaic_and_camera_orientation.py` | New — 21 tests (rotation math + sign-lock round-trip, store rotation-only write, per-objective rotation preservation, mosaic shift incl. malformed-extent guard, re-anchor no-travel flow). |
| `quickstart_guide/guide_content.json` | Documented the re-anchor + camera-orientation workflow. |

## Implementation Steps

- [x] `pixel_to_stage_offset` applies `R(θ)` (identity at 0/None); `import math`.
- [x] `CameraCalibrationStore.get_rotation` / `set_rotation` (sibling-preserving).
- [x] `calibration.py`: `_ploc_shift_mosaic_by` + call from `_ploc_apply_global_translation`; refactor `_ploc_store_manual_align`.
- [x] `calibration.py`: no-travel re-anchor (live→overview, drop travel + Safe-Z gate, guard overview clicks).
- [x] `objective_calibration_card.py`: orientation button + readout (persist per identity + per-objective sync + live push).
- [x] `hardware_setup.py`: rotation restore independent of µm/px.
- [x] New test file (21 tests, green).
- [x] Update plan + CLAUDE.md table.
- [x] Quickstart guide documentation.
- [x] Adversarial review findings addressed (see Issues & Decisions).
- [ ] **Real-HW verification on ME3B V1.**

## Testing Notes

- `python -m unittest tests.test_v75x_reanchor_mosaic_and_camera_orientation` → **21/21 OK**
  (incl. the review-fix regressions). `test_v74x_objective_calibration` +
  `test_v75x_camera_rotation` + `test_v75x_camera_calibration_store` → 65 OK after the
  ObjectiveCalibration store fix.
- Regression (green): `test_v75x_camera_rotation`, `test_v75x_camera_calibration_store`,
  `test_v75x_camera_image_correction`, `test_v75x_mosaic_orientation_remap`,
  `test_v75x_plate_orientation_convention`, `test_v75x_plate_location_manual_click_rim` (115 tests),
  `test_v75x_plate_mosaic` (115/116; the 1 failure `test_real_24_well_mosaic` is the **documented
  pre-existing CV failure**, unrelated — MosaicBuilder / detection are untouched).

**Real-HW verification (ME3B V1) — REQUIRED, still pending:**
1. Hardware Setup → Cameras → Objective Calibration → **"Calibrate orientation…"** → move the stage;
   confirm the readout shows a plausible angle (~180° on this rig, or the true mount angle).
2. In Plate Location, click a plate feature in the live view; confirm the reported stage offset points
   the right way. If mirrored/rotated wrong, flip the `R(θ)` sign (one line, isolated to
   `pixel_to_stage_offset` — negate `s`).
3. Offset the map (or remount the plate) → **Re-anchor map** → jog a feature into the live view, click
   it, then click the same feature on the mosaic overview; confirm the **mosaic image AND the wells
   shift together** onto the real plate on both Plate Location and the Jog page, and the stage does
   **not** move during the two picks. Restart → the shift persists.

## Issues & Decisions

- **Rotation applied at the click mapping, NOT at capture** — decided after verifying MosaicBuilder
  assumes an un-rotated pixel→stage convention; a capture-level flip/rotation would scramble the
  mosaic + break detection back-projection. Click-mapping-only keeps the mosaic build untouched.
- **Displayed live feed left un-rotated** (operator choice "correct mapping only"). For a re-anchor,
  the operator identifies a distinctive feature; a visual rotation of the feed is a decoupled future
  enhancement (would rotate the display + inverse-rotate the click to keep the `clicked` contract).
- **No-travel re-anchor** replaces the old travel-first flow (which only worked when the offset was
  < ~½ FOV). The E sign is unchanged (`live − map` == old `feature_abs − target`), so
  `_ploc_apply_global_translation` is reused verbatim.
- **Orientation stored two ways, consistently:** per camera identity (`CameraCalibrationStore`,
  restored on slot assignment) AND per objective (`ObjectiveCalibrationStore`, pushed on objective
  swap). The orientation button syncs the current objective's rotation when it has a µm/px cal so an
  objective swap can't push a stale rotation over the fresh one.
- **Changing orientation does NOT invalidate a scanned mosaic** — the rotation only changes the
  click→stage mapping, not the stored image (mosaic build never sees the rotation).
- **Adversarial review (4 dimensions → independent verify, 8 agents):** rotation-math and
  re-anchor-flow dimensions came back clean (all findings refuted/confirmed-correct; the R(θ) form,
  identity reduction, all 5 `pixel_to_stage_offset` callers single-apply, no plate-flip double-apply,
  state machine + frames + gates verified). Three confirmed defects, all fixed: **(1 HIGH)**
  `_ploc_shift_mosaic_by` indexed `ext[0..3]` without validating the cached extent shape → malformed
  extent = IndexError; now degrades to a logged no-op (+ test). **(2 HIGH)** after "Calibrate
  orientation", swapping to a different objective pushed that objective's STALE per-objective
  rotation over the fresh one (`_push_stored_um_per_px_to_manager`); the orientation button now syncs
  the fresh rotation into EVERY objective with a µm/px cal for the camera (mount rotation is a
  camera property, not per-objective). **(3 MED, pre-existing)** `ObjectiveCalibrationStore.
  set_calibration` rebuilt the entry dict and silently dropped a stored `rotation_deg` on a
  µm/px-only update; it now preserves the prior rotation when `rotation_deg=None` (mirrors
  `CameraCalibrationStore`; + 2 tests).

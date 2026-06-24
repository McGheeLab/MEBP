# MEBP v7.5.x — One Canonical Well-Plate Orientation

> **Addendum (mosaic remap):** after the orientation flip, the existing plate
> calibration + mosaic were stale (navigating to a well drove to the mirror
> well; a bogus warp had absorbed the 180° flip as a ~79 mm "correction"). The
> mosaic is ground truth (captured at trusted absolute stage positions), so
> every well is now RE-DERIVED from the mosaic's detected centres and RELABELLED
> for the current orientation, stored directly (no warp). See
> **"Addendum: re-derive wells from the saved mosaic"** at the end of this doc.

## Objective

Enforce a single well-plate orientation convention everywhere the software
references a well plate:

> **Well A1 displays TOP-LEFT; the stage's physical 0,0 is BOTTOM-RIGHT.**

The ME3B V1 Prior ProScan II stage has its origin at the bottom-right with
+X/+Y pointing toward the top-left, so the plate-local axes are **anti-aligned**
with the stage axes when A1 is physically top-left.

Fixes three operator-reported symptoms:
1. **Screens disagree visually** — the 180° display flip was a constant
   duplicated in two views and absent from others.
2. **Motion goes to the wrong / mirrored well** — geometric well→stage
   prediction assumed plate axes align with stage axes.
3. **No documented standard** — every new view/feature re-invented orientation.

## Core design decision

The per-machine orientation sign lives in the **plate-local → stage mapping
layer, never in the `WellPlate` data model.** The data model stays plate-local
(A1 at relative `(0,0)`, +col→+X, +row→+Y); the sign is applied only where a
well/plate-centre is converted to a stage coordinate by *geometry*. Positions
that are explicitly **taught/warped** are already absolute stage µm and are
never re-signed.

Single source of truth: **`plate_flip_180`** (per-machine, default `True`),
exposed two ways on `StageController`:
- `plate_flip_180() -> bool` for the display layer,
- `plate_axis_sign() -> (sx, sy)` = `(-1,-1)` when flipped, else `(1,1)`, for
  the geometry layer.

## Files Modified

### Single source of truth (Phase 1)
- `SupportClasses/StageController.py` — `DEFAULT_PLATE_FLIP_180`, `_plate_flip_180`,
  `plate_flip_180()`/`set_plate_flip_180()`/`plate_axis_sign()`,
  `apply_z_convention(plate_flip_180=)`.
- `gui/pages/hardware/device_profile.py` — `DeviceProfile.plate_flip_180`
  (to/from_dict, from/apply_settings).
- `config/hardware/devices/ME3B V1.json` — `"plate_flip_180": true`.
- `main.py` — pass `plate_flip_180` into the startup `apply_z_convention`.
- `gui/pages/hardware/stage_panel.py` — Z-Axis-Setup checkbox + persistence,
  restore in `_load_from_settings`.

### Display unification (Phase 2)
- `gui/widgets/jog_workspace_view.py` — `_flip_180` instance flag +
  `set_plate_flip_180`; `_apply_display_flip` + mosaic-overlay rotate read it.
- `gui/widgets/print_trajectory_monitor.py` — same.
- `gui/pages/print_monitor.py` — `PlateOverviewWidget` negates the zero-ref
  needle when flipped (wells are plate-local); pushed from `PrintMonitorPage`.
- Wiring: `gui/pages/jog_control.py`, `gui/pages/workflows/quick_print_workflow.py`.

### Geometry & calibration mapping (Phase 3)
- `SupportClasses/WellPlate.py` — optional `plate_axis_sign=(1,1)` on
  `get_all_positions_from_a1` / `get_a1_from_plate_center` /
  `get_all_positions_from_plate_center` / `get_well_area_bounds_um` /
  `get_well_area_bounds_from_a1_um` (bounds re-normalise min/max). Data model +
  `get_well_position` unchanged.
- `gui/pages/calibration.py` — `_plate_axis_sign()` helper; passed at every
  geometric prediction/seed/fallback site (incl. the hand-rolled
  `_predict_well_xy` / `_estimate_well_position_um` / manual go-to formulas);
  persistence orientation guard in `_save_calibration` (stamp) /
  `_load_calibration` (discard stale XY cal on mismatch → re-teach).
- `gui/pages/jog_control.py` — startup seed passes the sign.
- `SupportClasses/SimulatedCamera.py` — `_draw_wells` maps the well grid via the
  controller's sign so the sim plate matches the prediction.
- `SupportClasses/CalibrationSnapshotStore.py` — fingerprint carries orientation.

### Print-execution motion (Phase 4, needle-safety-critical)
- `SupportClasses/PrintManager.py` — `HybridPlanExecutor._plate_axis_sign()`
  applied to all `get_well_position` sites; `PrintSettings.plate_axis_sign`.
- `gui/pages/print_setup_legacy.py` — sign on `well_positions`; stamp settings.
- `SupportClasses/PrintTrajectoryPlanner.py` — `_well_xy` resolver (incl.
  service wells) reading `settings.plate_axis_sign`.
- `SupportClasses/PrintPlanOfAction.py` — `_signed_well_xy` helper at all sites.
- `gui/pages/workflows/quick_print_workflow.py` — stamp settings + sign the
  geometric fallback (calibrated well centre left as-is).

### Docs & tests
- `SupportClasses/WellPlate.py` docstring; `CLAUDE.md` safety rule #3 + table.
- `tests/test_v75x_plate_orientation_convention.py` (new, 17).
- `tests/test_v75x_plate_location_predict_well.py` — stub binds `_plate_axis_sign`.

## Implementation Steps

- [x] Phase 1 — `plate_flip_180` source of truth + plumbing + UI toggle.
- [x] Phase 2 — display flip driven by the setting in all stage-frame views.
- [x] Phase 3 — geometry sign at the mapping boundary + persistence guard.
- [x] Phase 4 — print-execution geometric well centres sign-mapped.
- [x] New test suite + affected-test fixes.
- [x] WellPlate docstring + CLAUDE.md convention rule + this plan.

## Testing Notes

- `python -m unittest tests.test_v75x_plate_orientation_convention` — 17 green
  (setting getters/round-trip, geometry sign + data-model invariance, warp
  no-double-apply, display-flip involution + origin-bottom-right, planner sign).
- Regression sweep green (188): `test_v75x_plate_centering`,
  `test_v75x_freeform_warp`, `test_v75x_plate_location_predict_well`,
  `test_v75x_last_known_calibration`, `test_v731_jog_navigation`,
  `test_v730_simulated_camera`, `test_v75x_quick_print_trajectory_view`,
  `test_v75x_z_axis_unified_setup`, `test_v745_plate_design`,
  `test_v748_rosette_flatten`.
- **Real-HW verification on ME3B V1 (Phases 3–4 change real motion):**
  1. Re-teach a full-plate calibration.
  2. "Go To" several named wells — needle reaches the correct *physical* well
     (not the mirror); Jog workspace + calibration + navigator + print monitor
     all show A1 top-left / stage origin bottom-right consistently.
  3. Run a small multi-well print — correct wells, ends at safe Z.
  4. Load a pre-orientation saved calibration — prompts a re-teach rather than
     driving to a mirrored position.

## Issues & Decisions

- **Sign locus = mapping layer, not the data model.** Flipping stored
  `well.x/well.y` in `WellPlate.__post_init__` was rejected: it breaks
  `get_well_position` invariants, custom/rosette plates, sketch/designer, and
  ~10 test files. The data model is plate-local and correct for the display
  convention; only the stage mapping is per-machine.
- **Default `plate_flip_180=True`** preserves today's ME3B rendering on a
  profile that predates the setting; the `WellPlate` geometry helpers default
  `plate_axis_sign=(1,1)` so any caller that doesn't pass it is byte-identical.
- **Warp not touched.** The sign is applied only to the *prediction*; the warp
  re-solves on sign-corrected predictions and reproduces measured wells exactly
  (test `TestWarpNoDoubleApply`). Mismatched-orientation saved calibration is
  invalidated on load (the predicted-leg frame would otherwise drive to garbage).
- **SimulatedCamera** reads the sign from its controller so the simulated plate
  faithfully mirrors ME3B and stays consistent with the prediction; only
  `_draw_wells` is signed (origin unchanged → no test churn).
- **Malformed sign degrades to aligned `(1,1)`** at every consumer (mock
  controllers in tests, defensive in production) — never crashes a print.
- **Camera live-overlay** (`TargetOverlayCameraView`) is OUT OF SCOPE —
  governed by physical camera mounting / per-camera `rotation_deg`.
- **`PrintTrajectoryPlanner`** still has the separately-documented `ZDIR=+1`
  Z-geometry limitation; this change makes its XY well mapping orientation-aware
  but does not resolve the Z polarity gap (use the discrete/hybrid path on ME3B).
- Pre-existing, unrelated test failures left as-is: `test_v726_print_execution`
  (`_make_mock_plate` `dict()` on 3-tuples) and `test_v73_trajectory_planner` /
  `test_hybrid_execution` (MagicMock settings z-math) — confirmed identical with
  these changes stashed.

---

## Addendum: re-derive wells from the saved mosaic (ground truth)

**Problem.** Flipping `plate_flip_180` to True mirrored the geometric well→stage
mapping, so the previously-taught calibration + mosaic became stale: navigating
to a well drove to the 180°-mirror well, and the saved `plate_warp` had absorbed
the flip as a degenerate ~79 mm / 179.58° "correction". The plate didn't move —
only which detected circle the software calls "A1" changed.

**Principle.** The full-plate mosaic is stitched at TRUSTED absolute stage
positions, so each detected well CENTRE is ground truth. Only the NAME→position
binding depends on orientation: with `plate_axis_sign == (-1,-1)`, **A1 = the
max-stageX / max-stageY detection** (it renders top-left under the 180° display
flip), columns increase toward −stageX, rows toward −stageY. The 24 positions
are fixed; the remap is a clean 180° relabel (verified: new A1 = old D6,
≤202 µm vs ~7720 µm tolerance).

**Files.**
- **NEW `SupportClasses/MosaicWellRemap.py`** (GUI-/hardware-free, NO motion):
  - `label_positions(positions, plate, plate_axis_sign)` — relabel fixed
    ground-truth centres by anchoring A1 at the orientation's bbox corner and
    nearest-matching the plate grid (mirrors `_ploc_fit_from_mosaic_detections`).
  - `detect_raw_positions(image, extent, scale, plate, global_shift)` — detect
    well centres in a saved mosaic (filled-disc blobs → grid-cull → Hough
    fallback) + back-project to absolute µm.
  - `detect_well_positions(...)` = `label_positions(detect_raw_positions(...))`.
- **`gui/pages/calibration.py`** — explicit-positions persistence + the action:
  - `_save_calibration`: writes a NEW `calibrated_positions` key (name→absolute
    µm) when positions are set directly with NO warp.
  - `_load_calibration`: a branch restores `_calibrated_positions` DIRECTLY from
    `calibrated_positions` and SUPPRESSES the warp/affine reconstruction (gated
    `not self._calibrated_positions`); the key is added to the orientation-guard
    pop-list (a future genuine flip discards it → re-teach).
  - `_has_plate_calibration` recognises `_calibrated_positions`.
  - `_ploc_rederive_from_saved_mosaic` + `_ploc_rederive_wrapper`: gather
    ground-truth centres (live `reference_markers`/`_ploc_well_results`, else
    re-detect the saved PNG; prefer the most complete), `label_positions` under
    the current sign, CLEAR the stale taught/warp/affine state, store the dict
    directly, set `taught_a1 = results['A1']`, save + emit. NO stage motion.
  - GUI: full-width button **"Re-derive wells from saved mosaic (ground truth)"**
    on the Plate Location → Mosaic card.
  - `_ploc_set_status` helper (renamed from a name that collided with the
    `_ploc_status` QLabel).
- **`tools_regen_plate_orientation.py`** (one-shot migration, already run):
  relabelled the existing 24 `reference_markers` and rewrote `settings.json` +
  `config/hardware/last_calibration.json` — set `calibrated_positions` +
  `taught_a1` (new A1 = (104915, 68044)), `plate_flip_180=true`, and DELETED the
  bogus `plate_warp` + `mosaic_affine`. Backups: `*.bak-orientremap`.

**Decisions.**
- Store positions DIRECTLY (no warp). A warp re-introduced the flip; the
  plausibility guard (scale 0.8–1.25×, |rot|<30°) wouldn't even accept a clean
  180°. The re-derive never routes through `_ploc_feed_affine`/`_manual_fit_xy`.
- Stored `reference_markers` (24 complete, validated) beat PNG re-detection
  (which misses the clipped max-corner well → 23/24, the documented
  `test_real_24_well_mosaic` CV limitation); the action prefers the most
  complete source.

**Tests.** `tests/test_v75x_mosaic_orientation_remap.py` (14): orientation-aware
relabel ((-1,-1)→A1 max corner, (1,1)→min corner, 180° rotation, permutation,
real ground-truth corners), `_ploc_rederive_from_saved_mosaic` (relabels + drops
warp + sets taught_a1; aligned keeps min corner; no-source is a safe no-op), and
explicit `calibrated_positions` round-trip (restored with no warp; dropped on an
orientation-stamp flip). Regression: plate-mosaic / template-reregister /
last-known-cal / predict-well / orientation-convention / jog-nav / freeform-warp
/ sim-camera / integration / centering green (only the documented
`test_real_24_well_mosaic` 23/24 CV failure remains, untouched).

**Verified end-to-end:** loading the regenerated `settings.json` reconstructs
24 wells directly (no warp/affine); navigation resolves A1→(104915,68044)
(top-left), A6→(8482,67970) (top-right), D6→(8595,10054) (bottom-right ≈ stage
origin). **Needs real-HW confirmation on ME3B V1** (go to a few named wells +
small multi-well print land on the correct physical wells).

### Follow-ups

- **Calibration plate-view dot bug (cause of "top-left well = D6"):**
  `_CalibrationPlateView` drew the predicted/calibrated dots positioned in
  absolute µm relative to `self._taught_a1` (with a latent µm/mm unit mix), so a
  `taught_a1` that desynced from the calibrated A1 (e.g. a Map-wells re-label put
  it on the min corner) placed the WRONG well's dot at the origin/top-left. Fixed
  by drawing the dots **by name on each well's plate-local grid cell** (A1 always
  top-left, same frame as the grey grid + A1/corner highlights) — robust to
  `taught_a1` desync and orientation. The regen also resyncs
  `taught_a1 == calibrated_positions["A1"]`.
- **Whole-plate mosaic scan defaults to the FULL reachable XY envelope:** new
  `_ploc_whole_plate_scan_bounds(positions, env)` returns the entire
  `safety_limits` XY envelope (so the whole XY space is mapped), falling back to
  the well-centre extent + margin only when no envelope is configured; the
  single-well (rosette) override is unchanged. The existing tile-count confirm
  dialog surfaces the larger scan size.

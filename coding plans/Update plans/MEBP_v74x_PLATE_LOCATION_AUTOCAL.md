# MEBP v7.4.x — Plate Location: Freeform Queuing + Adaptive Well Auto-Cal

Feature-scoped update to the **Calibration → Plate Location** tab
(`gui/pages/calibration.py`). Independent of the v7.4.7/v7.4.8
plate-*designer* track. Branch: continues on `Version-7.4.2`.

## Objective

Two capabilities on the Plate Location workflow:

1. **Freeform location queuing.** The tab was hard-locked to Snap
   (well) mode; the canvas's built-in Free/Snap toggle did nothing
   because `position_clicked` was never connected. Now the queue is
   heterogeneous — Snap clicks enqueue wells; Free clicks drop arbitrary
   XY points. On Run a freeform point is visited, the user manually
   centers it, and the confirmed position is fed into the XY affine as a
   `(predicted, measured)` calibration pair.

2. **Adaptive well auto-calibration.** For snapped wells, the
   single-frame known-radius arc fit is replaced by a strategy chosen
   from **well diameter vs. camera field of view**:
   - `full_circle` (well fits in view) → Hough/contour full-circle detect.
   - `partial_arc` (arc curves enough) → known-radius Kåsa arc fit.
   - `multi_edge` (rim ~flat in view) → drive to **8 rim points**, detect
     the local edge at each, fit a circle through the collected points.
   - **Fallback:** if the arc can't be found, the user is prompted to
     jog to **3 points on the rim** and Confirm each; a circle is fit
     through the 3 stage positions.

## Files Modified

| File | Rationale |
|------|-----------|
| `SupportClasses/VisionDetector.py` | New module-level helpers: `select_well_fit_strategy(well_d_um, fov_w, fov_h)` (FOV-ratio → `full_circle`/`partial_arc`/`multi_edge`, thresholds `WELL_FIT_FULL_CIRCLE_MAX_RATIO=0.85`, `WELL_FIT_PARTIAL_ARC_MAX_RATIO=3.0`); `detect_rim_point_near_center(frame)` (Canny → nearest central edge crossing, sub-pixel cluster average); `fit_circle_to_points(points)` (Kåsa via `NeedleDetector._fit_circle_kasa`, with a covariance-eigenvalue collinearity guard). |
| `gui/pages/calibration.py` | Plate Location tab: wired `position_clicked`; heterogeneous queue (well-name `str` \| `{"kind":"free",x,y,status}`); confirm-mode UI (Confirm/Skip/Cancel + step label); run state machine (`_ploc_advance`) with two pause kinds (`freeform`, `well_3pt`); adaptive well fit (`_ploc_auto_fit_well` → `_ploc_single_frame_fit` / `_ploc_multi_edge_fit`); manual 3-point fallback (`_ploc_begin_well_manual`, generic `_ploc_confirm`/`_ploc_skip`); **fixed `_ploc_feed_affine`** to populate `_xy_teach_points` (stage µm) so well fits actually drive `_manual_fit_xy`; extended `_manual_fit_xy` to also consume `_free_teach_pairs`. |

## Implementation Steps

- [x] Connect `position_clicked` + update banner (Free/Snap discoverability)
- [x] Heterogeneous queue model + freeform click handler (toggle-remove within 500 µm)
- [x] Confirm/Skip/Cancel UI + step label in the queue panel
- [x] Run state machine: wells synchronous, freeform + manual-3pt pause for input
- [x] Vision helpers: strategy selector, rim-point detector, circle-through-points fit
- [x] Adaptive auto-fit: full_circle → Hough/contour, partial_arc → arc fit, multi_edge → 8 rim points
- [x] 3-point manual fallback (jog + Confirm ×3) with radius/collinearity sanity checks
- [x] `_ploc_feed_affine` fix (populate `_xy_teach_points`) + `_manual_fit_xy` freeform pairs
- [x] Compile + headless smoke-test of vision helpers
- [ ] Manual in-app verification on hardware/simulator (pending user)
- [ ] Unit tests for vision helpers (`tests/`) — pending
- [ ] Architecture doc note + CLAUDE.md plans-table entry — pending user signal on version

## Testing Notes

Headless (pure-vision helpers — passing):

```bash
python3 -m py_compile gui/pages/calibration.py SupportClasses/VisionDetector.py
python3 - <<'PY'
from SupportClasses.VisionDetector import (
    select_well_fit_strategy, fit_circle_to_points, detect_rim_point_near_center)
# strategy thresholds, 3-pt circumcircle, 8-pt noisy fit, collinear→None,
# synthetic rim-edge detection — see commit smoke test.
PY
```

In-app (`python3 main.py` → Calibration → Plate Location):
1. Free/Snap toggle (top-left of plate view): Snap click adds a well; Free
   click drops a ⌖ freeform point; click near a freeform point removes it.
2. Run with mixed queue: wells auto-fit; freeform points pause → jog to
   center → Confirm. Cancel aborts; Skip leaves a point untaught.
3. Large well (well ≫ FOV): confirm `strategy=multi_edge` in the log; the
   stage tours 8 rim points; circle fit yields the center.
4. Force a failure (cover the rim / bad lighting): the run pauses asking
   for a 3-point jog-and-Confirm fit.

## Issues & Decisions

- **Existing `_ploc_feed_affine` was incompletely wired.** It set
  `_taught_a1/corner/third` but never populated `_xy_teach_points`, which
  is what `_manual_fit_xy` pairs against geometry — so queued-well runs
  produced *no* affine. It also double-added the zero offset (treated
  stage-µm results as zero-ref). Both fixed: results are written straight
  into `_xy_teach_points` as absolute stage µm. **Behavior change** for
  the existing well path — verify on a setup where `zero_position` x/y ≠ 0.
- **Freeform → affine as `(predicted, measured)` pairs.** The clicked
  zero-ref position (→ stage µm) is the predicted source; the confirmed
  center is the measured target. Appended to `_free_teach_pairs` and
  injected into the same Procrustes fit; cleared after each run so a
  later manual-tab refit doesn't double-count them.
- **Adaptive strategy keys on `well_diameter_um / min(fov_w, fov_h)`** —
  the FOV is read from the actual captured frame (`frame.shape * um_per_px`).
- **Multi-edge / 3-point fits are rejected** when the fitted radius
  deviates > 50 % from the plate's known well radius, or when points are
  (near-)collinear.
- **Known limitation:** the run executes synchronously on the GUI thread
  (matching the prior single-frame loop). A `multi_edge` sweep (8 moves +
  settle) briefly blocks the UI and can't be cancelled mid-sweep. A
  threaded executor is a future improvement.
- **Known limitation:** during a freeform/manual pause the prompt says
  "center in the plate camera," but the live feed lives on the Camera
  sub-tab — centering today means glancing there (or the eyepiece). An
  embedded preview on this tab is a candidate follow-up.

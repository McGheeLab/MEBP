# MEBP v7.5.x — Plate Location Manual Click-Rim Well Fit + Objective Confirmation

## Objective

On the Calibration page → **Plate Location** sub-tab, add an operator-driven
well-centering workflow on top of the existing automatic edge-fit:

1. **Live microscope view during the run.** A live feed is shown on the tab so the
   operator can watch the well throughout the run.
2. **Manual click-on-edge well fit (jog-driven).** For each queued well the stage
   drives ONCE to the predicted well center as a starting anchor; the operator then
   **jogs** (Xbox / Jog page) to bring each rim edge into the live view and **clicks
   the well edge**. Each click records the rim point at the stage's **current
   position** (so jog adjustments are honored) and never drives the stage. Once ≥3
   points are clicked around the rim the operator hits Confirm and they are circle-fit
   to the well center (reusing the known-radius circle fit). Manual is the default; the
   automatic edge-detection remains as a selectable mode.

   > **Revision (per follow-up):** the first cut drove the objective to the 8 predicted
   > rim points and auto-advanced after each click. Because the predicted points are
   > exactly what's being calibrated, they often didn't put an edge in view, and the
   > auto-advance teleported the stage away from where the operator had jogged. The flow
   > is now jog-driven: one anchor move, then free jog + click using the live position.
3. **Objective confirmation.** Before driving, the operator must confirm the
   microscope objective currently in use matches the one selected in the config
   (whose µm/px maps clicks → stage µm).

## Design / Data Flow

- **Click → stage µm** reuses the existing pipeline (no new transform math):
  `CameraFeedView.clicked(px_x, px_y)` → `CameraManager.pixel_to_stage_offset(cam_idx,
  px, py, w, h)` (center-offset × µm/px) → `+ StageController.get_xy_position(cached=
  False)` (absolute stage µm, `xy_position_scale=1`) = the clicked rim point in
  absolute stage µm — the same frame as `_predict_well_xy` / `move_xy_absolute_um`.
- **Jog + click**: `_ploc_begin_well_click_rim` drives once to the predicted well
  center (anchor), then waits. Each live-view click (`_ploc_on_live_view_click`) appends
  the current-position rim point and refreshes the count prompt
  (`_ploc_update_click_rim_prompt`) — it does **not** move the stage. The operator jogs
  to each rim edge themselves. `_ploc_finalize_click_rim` (on Confirm) circle-fits the
  collected points (with the existing `_PLOC_RADIUS_TOL` plausibility check) and records
  the center into `_ploc_well_results`, which feeds the affine/warp exactly like the auto
  and 3-point paths. `_PLOC_RIM_SAMPLES` (8) is now only the *suggested* click count
  shown in the prompt.
- **Objective**: `camera_config.current_objective_name` is the declared scope
  objective; selecting it pushes that objective's stored µm/px into `CameraManager`
  (`objective_calibration_card._push_stored_um_per_px_to_manager`). The gate shows the
  objective + the in-use µm/px and (best-effort) whether a stored per-objective
  calibration exists, then requires a Yes to proceed; blocks if no objective is set.

## Changes

All in `gui/pages/calibration.py`:

### `_build_plate_location_tab`
- New **Well-fit mode** combo (`_ploc_mode_combo`): "Manual — click well edge in live
  view" (default) / "Auto — camera edge-detect". `currentIndexChanged` →
  `_ploc_on_mode_changed`.
- New **Live microscope** `CameraFeedView` (`_ploc_live_view`, crosshair on) under the
  plate map in a vertical splitter (plate map 3 / live view 2); `clicked` →
  `_ploc_on_live_view_click`. Plus a `_ploc_live_hint` label.
- Updated banner text describing the modes.
- New state: `_ploc_fit_mode` (default "manual"), `_ploc_live_cam_idx`,
  `_ploc_click_points` (`_ploc_rim_targets`/`_ploc_rim_idx` retained but unused after the
  jog-driven revision).

### Run state machine
- `_ploc_advance`: a queued well in **manual** mode → `_ploc_begin_well_click_rim`
  (pause kind `"well_click_rim"`); **auto** mode unchanged (`_ploc_auto_fit_well` →
  3-point fallback).
- `_ploc_begin_well_click_rim`: drives once to the predicted well center (anchor), starts
  the live camera, shows the prompt; **no rim stepping**.
- `_ploc_on_live_view_click`: records the rim point at the current stage position and
  refreshes the count prompt (`_ploc_update_click_rim_prompt`) — does **not** move the
  stage.
- `_ploc_finalize_click_rim` (on Confirm): circle-fits the collected clicks, records the
  center, advances.
- `_ploc_confirm`: in `"well_click_rim"`, **Confirm** = fit the points clicked so far
  (≥3). `_ploc_skip`: in `"well_click_rim"`, **Skip** = leave this well uncalibrated and
  advance. (Freeform/`well_3pt` paths unchanged.)
- `_ploc_run_queue`: stores `_ploc_live_cam_idx = plate_cam_idx`, gates on
  `_ploc_confirm_objective(...)` (abort on decline), and starts the live camera in
  manual mode (`_ploc_ensure_live_camera`).
- `_ploc_cancel_run` / `_ploc_finish_run`: clear the new state + live overlay/hint.
- `_refresh_ploc_view`: rebinds `_ploc_live_view` to the Microscope slot when it changes
  (so the feed shows whenever that camera is running).
- New helpers: `_ploc_on_mode_changed`, `_ploc_ensure_live_camera`,
  `_ploc_confirm_objective`, `_ploc_update_click_rim_prompt`.

## Files Modified

| File | Change |
|------|--------|
| `gui/pages/calibration.py` | Live microscope view + well-fit mode toggle on Plate Location; manual click-rim well fit (`well_click_rim` pause kind); objective confirmation gate; live-camera start/rebind |
| `tests/test_v75x_plate_location_manual_click_rim.py` | New — 17 tests |

## Implementation Steps

- [x] Trace the click→stage-µm pipeline (`CameraFeedView`, `CameraManager.pixel_to_stage_offset`, `LiveTargetPicker`) and the objective wiring (`current_objective_name`, `ObjectiveCalibration` store)
- [x] Confirm `get_xy_position` frame (absolute µm) via the existing `well_3pt` path
- [x] Add mode toggle + live view to `_build_plate_location_tab` (vertical splitter)
- [x] Add the `well_click_rim` flow + mode dispatch in `_ploc_advance`
- [x] Route Confirm / Skip for the new pause kind
- [x] Add the objective-confirmation gate + live-camera start in `_ploc_run_queue`
- [x] State cleanup in cancel/finish; live-view rebind in `_refresh_ploc_view`
- [x] **Revision: make manual mode jog-driven** — anchor once, then free jog + click at
  the live position; removed the per-click teleport to predicted rim points
- [x] Tests + offscreen GUI build smoke

## Testing Notes

- `tests/test_v75x_plate_location_manual_click_rim.py` (17) — calls the real (unbound)
  methods with duck-typed stubs (`QMessageBox` patched; real
  `CameraManager.pixel_to_stage_offset`; real prompt-refresh bound onto the stub):
  - click conversion (`stage center + (px − center)·µm/px`), incl. negative offsets;
  - **a click records the point but does NOT move the stage** (no teleport);
  - **two clicks from two different (jogged) positions** both record relative to the
    position read at click time;
  - click no-ops outside a `well_click_rim` pause / when not running / before frames;
  - `_ploc_begin_well_click_rim` makes exactly one anchor move (to the predicted center);
  - finalize records the center from rim clicks (circle-fit), leaves the well
    uncalibrated for <3 clicks, and rejects an implausible radius;
  - Skip leaves the well uncalibrated and advances;
  - mode toggle changes only when idle; objective gate blocks with no objective,
    proceeds on Yes, aborts on No.
- Regression: `test_v75x_plate_location_predict_well`, `_well_radius_conversion`,
  `_plate_centering`, `_freeform_warp` (48 total) pass.
- Offscreen (`QT_QPA_PLATFORM=offscreen`) `CalibrationPage` build succeeds with the new
  widgets present and `_ploc_fit_mode == "manual"`.
- On hardware: select the scope objective in Hardware Setup → Cameras; queue wells in
  Snap mode; Run → confirm the objective dialog → for each well the stage drives to its
  center, then jog to bring each rim edge into the live view and click the edge (≥3
  around the rim); Confirm to fit, Skip to skip the well → the center is recorded and the
  map refines.

## Issues & Decisions

- **Manual default + auto toggle** (per user). Manual feeds the affine/warp identically
  to the auto and 3-point paths — only the rim-point *source* differs (operator clicks vs
  vision detection).
- **Jog-driven, not auto-tour** (revision per follow-up: "if I adjust the position using
  jog … it should take the current position into account"). The click already used the
  live position, but the first cut teleported the stage to the next predicted rim point
  after each click, fighting manual jogging — and the predicted points (the very thing
  being calibrated) often showed no edge. Now: one anchor move to the well center, then
  the operator jogs freely and each click records at the current position. `Skip` skips
  the whole well; `Confirm` fits with ≥3 collected points.
- **Objective gate applies to both modes** (auto also uses the objective's µm/px) and is
  a blocking confirmation; it blocks outright when no objective is selected, since the
  scope state is then unknown.
- **No new transform math**: click→stage µm and the circle fit reuse existing helpers,
  matching `LiveTargetPicker` and the `well_3pt` path.
- **Testing follows repo convention** (e.g. `MEBP_v75x_CAL_Z_ENVELOPE_NO_CLOBBER`):
  unbound methods + `SimpleNamespace` stubs rather than full headless page instantiation.

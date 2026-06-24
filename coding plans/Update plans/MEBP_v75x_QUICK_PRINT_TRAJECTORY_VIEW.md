# MEBP v7.5.x — Quick Print live trajectory + needle ghost-trail + camera view

## Objective

Give the Quick Print page visual feedback. Before this change it only showed a
text status line; the operator could not see what the toolpath looked like or
where the needle actually was during a run. Add, on the Quick Print page:

1. A **planned toolpath preview** (trajectory waypoint path) for the selected
   well + object combo.
2. A **live executed path** drawn over the plan as the print runs.
3. A **fading mauve ghost trail** at the needle — the same breadcrumb effect the
   Jog page overview uses.
4. A **live microscope camera feed** beside the trajectory view.

This is **display-only**. The print still runs through the unchanged
`_on_print` → `build_well_plate_job()` → `PrintManager` discrete path, so the
hardware safety invariants (Z-retract-before-XY-travel, Z polarity) are untouched.

User-confirmed design choices:
- **View scope:** zoomed/auto-fit to the selected well (waypoints visible), not a
  full-plate overview.
- **Overlays:** planned waypoint path **+** persistent executed polyline **+**
  short fading ghost trail.
- **Layout:** well-plate selector on the **bottom**; a **top row** of
  `[trajectory view | camera]`; **resizable splitter boundaries everywhere**.

## Files Modified

| File | Rationale |
|------|-----------|
| `gui/widgets/print_trajectory_monitor.py` | **NEW** `PrintTrajectoryMonitorView` — auto-fit top-down canvas drawing planned path + executed polyline + ghost trail + needle marker, all in zero-ref µm. |
| `gui/pages/workflows/quick_print_workflow.py` | `camera_manager` ctor param; splitter layout (`_build_main_area`); live microscope feed (start on show / stop on hide, only if we started it); planned-path recompute (`_refresh_planned_path`); live-needle push in `on_status_update` (`_push_live_position`); recording on/off at print start / terminal state. |
| `gui/pages/workflows_mode.py` | Pass the shared `camera_manager` to `QuickPrintWorkflowPage` (matching the spheroid branch). |
| `tests/test_v75x_quick_print_trajectory_view.py` | **NEW** 13 tests (widget + page wiring). |

## Coordinate contract (single frame: zero-ref µm)

Everything in the new view is **zero-referenced µm** so planned and live overlays
register:
- Planned waypoint = `((center_mm + path_point_mm) * 1000)`, where
  `center_mm = _well_center_zero_ref_mm(well)` (existing) and `path_point_mm`
  comes from the existing `_path_segments_for_selection()`.
- Live needle = `controller.get_xy_position(cached=True) − controller.zero_position`
  (identical to `jog_control.on_status_update`).
- The render is reflected 180° about the fitted bbox centre (`_FLIP_DISPLAY_180`,
  mirrors `JogWorkspaceView`) so orientation matches the Jog overview + camera;
  applied uniformly so all overlays stay aligned.

## Reuse

- Ghost-trail breadcrumb algorithm + `_um_to_px` / display-flip idiom copied from
  `gui/widgets/jog_workspace_view.py` (`set_position`, `_paint_breadcrumbs`,
  `_apply_display_flip`).
- Planned/executed pen styling follows `gui/widgets/trajectory_view.py`
  (green completed path) and `gui/widgets/sketch_canvas.py::_draw_toolpath`.
- Camera lifecycle mirrors `gui/pages/calibration.py::_ploc_ensure_live_camera`;
  microscope slot resolved via `HardwareConfig.camera_for_role(CameraRole.MICROSCOPE)`.
- Live position polled exactly like `jog_control.on_status_update`, off the
  existing ~300 ms MainWindow tick (no new timer). `PrintManager` is **not** the
  source — its `on_progress` is per-command, not per-waypoint XY.

## Implementation Steps

- [x] New `PrintTrajectoryMonitorView` widget (planned/executed/ghost/needle,
      auto-fit, well boundary, placeholder).
- [x] Quick Print: `camera_manager` ctor param + `_camera_view` state.
- [x] Quick Print: `_build_main_area()` vertical splitter (top
      `[trajectory | camera]` horizontal splitter, bottom well selector).
- [x] Quick Print: camera resolve/start/stop (`_resolve_microscope_cam_idx`,
      `_start_camera`, `_stop_camera`) on show/hide + rebind in
      `set_hardware_config`.
- [x] Quick Print: `_refresh_planned_path()` from well/object/size change,
      `set_calibration_data`, `set_hardware_config`, `showEvent`.
- [x] Quick Print: `_push_live_position()` in `on_status_update`.
- [x] Quick Print: `reset_live()` + `set_recording(True)` at print start;
      `set_recording(False)` on terminal state.
- [x] `workflows_mode.py`: pass `camera_manager` to the page.
- [x] Tests + regression.

## Testing Notes

- `python -m unittest tests.test_v75x_quick_print_trajectory_view` — 13 green
  (widget bbox/auto-fit, breadcrumb >1 µm gate + 24 cap, executed-only-when-
  recording, reset_live, needle-None clear, paint smoke; page planned-path
  placement in zero-ref µm, live-position push, no-camera-manager construction).
- Regression: `tests.test_v75x_quick_print_workflow` (16) green;
  `tests.test_v75x_sketch_print_visibility` + `tests.test_v75x_multi_object_print_seam`
  (20) green. `WorkflowsModePage` constructs with the camera manager fanned out
  to the Quick Print page (verified `_camera_manager is mgr`, `_traj_view` and
  `_camera_view` present).
- **Manual (real app):** open Quick Print → pick an object + well → planned path
  draws zoomed to the well, microscope feed shows beside it; start a print → the
  green executed polyline traces over the plan and the mauve ghost trail follows
  the needle; the splitter boundaries resize.

## Issues & Decisions

- **Why a new widget instead of reusing `JogWorkspaceView`:** the full-plate view
  renders a single well's path as a speck. The new view auto-fits to the path
  bbox so waypoints are visible, while borrowing the same breadcrumb/coord idiom.
- **Live path source = position poller, not `PrintManager`:** `on_progress` is
  per-command (a whole PRINT_PATH is one update) and carries no live XY, so the
  executed polyline + ghost are built from `get_xy_position(cached=True)` on the
  300 ms tick (same as the Jog page).
- **Camera stop on hide is gated on "we started it"** (`_camera_started_by_us` +
  `is_running` check) so visiting Quick Print briefly doesn't stop a microscope
  feed another page left running.
- **Minimum framed span** of 500 µm prevents a single-point/dot path from
  blowing the auto-fit scale up to infinity.

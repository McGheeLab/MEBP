# MEBP v7.4.3 → v7.4.4 Update Plan

## Objective

Revision the Calibration page so each calibration job has its own
workflow-oriented home, and split today's single misnamed "Needle
Calibration" tab (which is actually Z-offset work) into two distinct
flows.

The right-context `QTabWidget` returned by `CalibrationPage.get_right_context_widget()`
becomes four tabs:

1. **Needle Location** *(new)* — opens the two side-mounted needle
   cameras (orthogonal X-view / Y-view). User clicks the left and
   right edge of the needle in each view (4 clicks total). Software
   uses the per-camera µm/pixel to compute the stage offset that
   brings the needle pixel-center to the optical center of both
   cameras, then drives the stage and records `needle_origin_um` as
   the workspace reference (independent of stage zero, shared across
   plate loadouts).
2. **Z-Offset Calibration** *(rehouses today's Z logic)* — Safe Z,
   Top Z, focus-peak well-bottom auto-cal, per-well Z teach, Z-plane
   tilt fit. The math is unchanged; the tab just hosts the existing
   handlers in their natural home.
3. **Plate Location** *(new)* — single overhead camera (camera with
   `role == PLATE`). User clicks N>3 wells on the plate view; each
   click snaps to the nearest well centroid; system visits each well
   and fits the *known* well radius (per plate format) to the detected
   edge — partial arc or full circle depending on the objective
   (2× / 4× / 10×). Successful fits feed the existing XY affine map.
   Z calibration is out of scope.
4. **Custom** — preserves the camera grid display controls, well
   mark-left / mark-right edge widgets, vision overlays
   (`_toggle_needle_detect`, `_toggle_focus_assist`,
   `_toggle_needle_detect_relaxed`), validate `_goto_well`, and the
   calibration save/load buttons. Today's freeform behavior is
   preserved verbatim here.

Supporting changes:

- `CameraConfig` gets an explicit `role` field (`unassigned` /
  `needle_x` / `needle_y` / `plate`) so workflow tabs resolve cameras
  by role instead of asking the user to pick a camera index every
  session.
- The Hardware Setup → Cameras sub-page gets a role `QComboBox` per
  live-camera card. Objective and µm/pixel controls already live
  there; the role combo is the new addition.
- The left jog panel (`StandardJogContextPanel`, v7.4.3) persists
  across all four tabs.
- Sub-panels inside each workflow tab use `QSplitter` so the user can
  drag dividers to resize content within the main window (matches
  the project's pattern — no QDockWidget is used anywhere in the
  codebase).

Branch: `Version-7.4.2` (work proceeds on the current branch; bump to
`Version-7.4.4` at completion per CLAUDE.md checklist).

## Files Modified

| File | Rationale |
|------|-----------|
| `SupportClasses/HardwareConfig.py` | Add `CameraRole` enum + `role: CameraRole = UNASSIGNED` on `CameraConfig`; update `to_dict` / `from_dict`; add `HardwareConfig.camera_for_role(role) -> int \| None` helper. |
| `SupportClasses/VisionDetector.py` | Add `TwoCameraNeedleAligner` (takes 4 edge-click pixels + each camera's µm/pixel; returns `(dx_um, dy_um)` stage offset). Add `EdgeFitWellLocator.fit_partial_arc(frame, expected_radius_px, um_per_px, tolerance_pct)` for known-radius circle fits to partial well edges. |
| `gui/widgets/detection_worker.py` | Add `WELL_EDGE_FIT` mode for the Plate Location workflow. |
| `gui/widgets/camera_widget.py` | Add `pick_edges_requested` two-click mode for the Needle Location workflow; emits picked pixel coordinates back to the calibration page. |
| `gui/pages/hardware_setup.py` | Per camera mini-card: add a **Role** `QComboBox` (Unassigned / Needle X-view / Needle Y-view / Plate) wired to `CameraConfig.role`. Emit existing `hw_config_invalidated` signal on change. |
| `gui/pages/calibration.py` | Replace 3-tab `get_right_context_widget()` body with 4-tab workflow layout. Add new builders `_build_needle_location_tab`, `_build_z_offset_tab`, `_build_plate_location_tab`, `_build_custom_tab`. The existing handlers (`_set_safe_z`, `_set_top_z`, `_start_auto_z_cal`, `_zteach_record_z`, `_try_fit_z_plane`, `_manual_fit_xy`, `_toggle_needle_detect`, `_toggle_focus_assist`, `_toggle_needle_detect_relaxed`, `_goto_well`, `_save_calibration`, `_load_calibration`) are kept and reused unchanged. |
| `coding plans/Architectures/ARCHITECTURE_V744.md` *(new — at completion)* | Standard architecture-doc delta for the new version. |
| `CLAUDE.md` | New entry in *Existing Update Plans* table on completion; current-version bump. |

## Implementation Steps

- [x] Branch: continue on `Version-7.4.2` for now
- [x] Add `CameraRole` enum + per-index `camera_roles` list on `HardwareConfig` (serialize as string names, default `UNASSIGNED`). NOTE: stored on `HardwareConfig` instead of `CameraConfig` because there is only one `CameraConfig` instance but up to `MAX_LIVE_CAMERAS` live cameras.
- [x] Add `HardwareConfig.camera_for_role(role)` + `set_camera_role(idx, role)` helpers
- [x] Add `TwoCameraNeedleAligner` (+ `TwoCameraEdgePicks`) and `EdgeFitWellLocator` to `VisionDetector.py`
- [x] Add `WELL_EDGE_FIT` mode + `set_edge_fit_params()` to `DetectionWorker`
- [x] Add `pixel_clicked` signal + `set_edge_pick_mode()` on `CameraWidget` (event filter maps label-coords → frame-coords)
- [x] Add per-camera **Role** combo to Hardware Setup Cameras sub-page; persist via `HardwareConfig.camera_roles`
- [x] Restructure `CalibrationPage.get_right_context_widget()` to four tabs (Needle Location / Z-Offset / Plate Location / Custom)
- [x] Build `_build_needle_location_tab()` (dual-camera edge-click workflow with `CameraFeedView` per role)
- [x] Build `_build_z_offset_tab()` (Set Safe Z + Set Top Z + Run Z-Bottom Auto-Cal; pointer to Custom → Full Wizard for per-well teach)
- [x] Build `_build_plate_location_tab()` (click-snap N>3 wells, drive + edge-fit + feed `_manual_fit_xy`)
- [x] Build `_build_custom_tab()` (legacy Camera Setup + Full Wizard + Plate Calibration nested as sub-tabs)
- [x] Unit tests: camera role round-trip, two-camera aligner, edge-fit locator (19 new tests in `tests/test_v744_calibration_revision.py`)
- [x] Smoke-test in simulator (offscreen run — app launches; right context exposes the 4 new tabs; `to_dict` save no longer warns)
- [ ] Architecture doc + README archive + version bump (per CLAUDE.md checklist — at completion)

## Testing Notes

Manual test plan (run via `python main.py --simulate-xy --simulate-zp`):

1. **Hardware Setup → Cameras** — each live-camera mini-card now has a Role dropdown (default Unassigned). Assign cam 1 → Needle X-view, cam 2 → Needle Y-view, cam 3 → Plate. Save config.
2. **Calibration page tab order** — four tabs in this order: Needle Location, Z-Offset Calibration, Plate Location, Custom.
3. **Splitter behavior in each tab** — drag dividers to resize sub-panels; everything stays within the main window (no floating overlays).
4. **Needle Location workflow:**
   - Both side-camera feeds visible.
   - Step "Click needle edges in X-view" → click left, click right → status panel shows picked columns, midpoint, dy offset preview.
   - Step "Click needle edges in Y-view" → same.
   - Center & save → simulator moves; `needle_origin_um` persists.
   - Reset clears the picked points.
   - Missing role assignment → inline banner with link to Hardware Setup.
5. **Z-Offset workflow:** confirm Safe Z, Top Z, Z-Bottom Auto-Cal, manual Z teach, and Z-plane fit all behave exactly as in v7.4.2.
6. **Plate Location workflow:** click 4 wells on the plate view → each snaps to nearest centroid → Run visits each in `XYStageSimulator` → partial-arc fit reports success at simulated 2×/4×/10× objectives → affine map updates and refits via `_manual_fit_xy`.
7. **Custom tab:** camera tile/layout, mark-left/mark-right edges, vision overlays, validate go-to-well, save/load — all behave as in v7.4.2.
8. **Regression:** `python -m pytest tests/` passes with no new failures. Load a v7.4.2 settings.json — `role` defaults to `UNASSIGNED` without error.

## Issues & Decisions

- **µm/pixel calibration already lives in Hardware Setup → Cameras.**
  The plan's "relocate µm/px from Calibration to Hardware Setup" item
  is partially moot — the live-camera mini-cards in
  `gui/pages/hardware_setup.py` already host the µm/px spinbox,
  objective combo, and Calibrate-µm/px button. The Calibration page
  still has *duplicate* per-camera objective + µm/px controls in
  `_build_legacy_camera_section`. We leave them in place under the
  Custom tab so power users can adjust without leaving Calibration,
  but the Hardware Setup card remains the canonical source.
- **No new QDockWidget infrastructure.** The project uses
  `QSplitter` everywhere; consistency wins over the marginal
  flexibility of drag-rearrangeable dock widgets.
- **Needle Location is user-click driven, not vision driven.** The
  existing `NeedleDetector` auto-detect remains available in the
  Custom tab. The new workflow trades automation for predictability:
  the user picks the needle's visible edges, eliminating false
  detections when the µm/pixel scale is wrong or lighting is poor.
- **Z calibration deferred in Plate Location.** Per user direction:
  Plate Location only builds the XY map. Z lives in the Z-Offset tab.

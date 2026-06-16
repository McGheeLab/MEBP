# MEBP v7.5.x — Calculate-µm/px Live View + Per-Camera Rotation (45° mounting)

## Objective

Two requests on the stage-motion **Calculate µm/px** calibration (`PixelCalibrationDialog`,
used by Hardware Setup → Cameras → *Needle Cameras Setup* and the calibration-page
microscope context):

1. **Live view + detection overlay** — the dialog is currently a small form with no feed.
   Add a live `CameraFeedView` and, after each measurement, draw the phase-correlation
   **displacement vector** (arrow + magnitude + angle) on the feed so the operator can
   *see what the algorithm detected* and confirm the sample is textured / the motion is
   sane.

2. **Per-camera 45° rotation + direction selection** — the two needle cameras are 90°
   apart from each other but the *pair* sits at ~45° to the stage X/Y axes. Consequence:
   a stage move along the wrong direction drives the needle **along the camera's optical
   axis** → it just goes in/out of focus with almost no lateral image motion, so the
   phase-correlation displacement is tiny and µm/px is garbage. The operator must be able
   to **visualize the motion and pick the direction that gives real lateral motion**.
   - The dialog gains a **selectable move direction** (free angle + X / Y / ±45° presets)
     and overlays the detected displacement as a green arrow; a small/low-confidence
     arrow is flagged as "moving along the optical axis — try another direction."
   - The **accepted move direction** (the one giving clean lateral motion) *is* the
     camera's in-plane lateral stage direction; it's stored per camera (`result_rotation_deg`)
     and fed to `TwoCameraNeedleAligner` (whose orthogonal-mounting assumption is wrong at
     45°). The µm/px *scale* itself is rotation-invariant (`distance / magnitude`).
   - **Apply scope (user choice): needle centering only** — the live-view click→stage path
     (`pixel_to_stage_offset`) is intentionally left unchanged this pass.

## Rotation convention (locked)

- `rotation_deg`: the **absolute stage-frame angle** (deg CCW from +X) of the camera's
  lateral image axis — i.e. the move direction the operator accepts because it produced
  clean lateral motion. `None` = not measured.
- This is self-consistent with the legacy mapping: an orthogonal X-view camera's lateral
  axis *is* stage +Y (90°) and a Y-view camera's is stage +X (0°) — exactly what an
  operator would measure for an unrotated rig. So the value is fed **directly** as the
  aligner's column→stage angle (no `90 + ρ` offset).
- `TwoCameraNeedleAligner` takes **absolute** column→stage angles
  `angle_x_view_deg` / `angle_y_view_deg` (default `None` → legacy `90°` / `0°`) and solves
  the 2×2 system `U·n = s` (`U` rows = the two camera direction unit vectors,
  `s` = per-camera signed pixel offset × µm/px). Reduces exactly to the current code when
  angles are the legacy defaults. `x_sign` / `y_sign` (existing) absorb the recentering
  sign / handedness the operator validates on hardware.

## Files Modified

| File | Change |
|------|--------|
| `gui/widgets/camera_feed_view.py` | New general overlay: `set_overlay_vector(dx, dy, label)` draws an arrow from image center + label in `_render_frame` (used by the dialog to show the detected shift). |
| `gui/dialogs/pixel_calibration_dialog.py` | Two-column layout: live `CameraFeedView` (left) + controls (right). **Selectable move direction** (free-angle spinbox + X / Y / ±45° presets) replaces the X/Y-only combo; after measuring, overlays the displacement arrow and shows magnitude / detected angle / confidence; small or low-confidence motion is flagged as optical-axis (focus) motion → "try another direction". The accepted direction is exposed as `result_rotation_deg`. |
| `gui/widgets/camera_manager.py` | `_rotation_deg` per-slot list + `get_rotation_deg` / `set_rotation_deg`. |
| `SupportClasses/HardwareConfig.py` | `camera_calibrations` entry gains optional `rotation_deg`; `from_dict` tolerant. |
| `gui/pages/hardware_setup.py` | `set_calibrated_um_per_px(cam_idx, value, rotation_deg=None)` stores ρ (manager + identity store); `_on_calibrate_needle` passes `dlg.result_rotation_deg`; `_restore_calibration_for_slot` restores ρ; needle card shows ρ. |
| `SupportClasses/VisionDetector.py` | `TwoCameraNeedleAligner` generalized — `angle_x_view_deg` / `angle_y_view_deg` params + 2×2 solve; backward-compatible (legacy 90°/0° when unset). |
| `gui/pages/calibration.py` | `_needle_loc_camera_info` also returns the rotation (`mgr.get_rotation_deg`); `_needle_loc_compute_offset_um` passes each camera's absolute lateral angle to the aligner. **Cross-contamination fix:** `_ctx_update_um_px_display` pushed an objective-derived µm/px (from the *shared* microscope `camera_spec`) into the manager for **every** slot on each config change → it overwrote the needle cameras' just-set stage-motion calibration (calibrating Cam 2 stamped a microscope value onto Cam 1). The push is now gated to the slot tagged `MICROSCOPE`; needle/plate slots keep their measured value. |

## Implementation Steps

- [x] `CameraFeedView.set_overlay_vector` + arrow render.
- [x] `PixelCalibrationDialog`: live feed + arrow overlay; selectable move direction
      (free angle + X/Y/±45° presets); focus-vs-motion guidance; `result_rotation_deg`.
- [x] `CameraManager` rotation get/set.
- [x] `HardwareConfig.camera_calibrations` rotation_deg + serialization tolerance.
- [x] `hardware_setup`: store/restore rotation; pass it from the needle dialog; card readout.
- [x] `TwoCameraNeedleAligner` 2×2 generalization (backward-compatible).
- [x] `calibration.py` needle-loc: thread the rotation (absolute lateral angle) into the aligner.
- [x] Tests (`tests/test_v75x_camera_rotation.py`): aligner legacy-reduction + 45° solve +
      singular-raise; config rotation round-trip/tolerance; manager rotation get/set.

## Testing Notes

- `tests/test_v75x_camera_rotation.py`: `TwoCameraNeedleAligner` reduces to legacy with no
  angles / ρ=0; a 45° scenario recovers a known needle offset; singular (parallel) cameras
  raise. `HardwareConfig` ρ round-trip + tolerance. `CameraManager` ρ get/set + OOB-safe.
- Manual (hardware): Calculate µm/px on a needle camera — confirm the live view shows the
  feed and the post-measure arrow points along the stage move; confirm the measured camera
  rotation reads ~45°, edit it if needed, Accept; run Needle Location and confirm the
  centering move is correct at 45°.

## Issues & Decisions

- **Scale already rotation-invariant** — only the rotation (direction) is new; `um_per_px`
  math is unchanged.
- **Focus-vs-motion is the real problem** (user-raised) — at 45° a move along the camera's
  optical axis produces only focus change, not translation, so phase correlation returns a
  tiny vector. The dialog makes this *visible* (live feed + arrow + magnitude/confidence)
  and lets the operator sweep the move direction until the arrow is long → that direction
  is the camera's lateral axis. This replaces a pure "auto-measure angle" with
  "operator-selected, software-visualized" — which is what the user asked for and is more
  robust than trusting one blind phase-correlation angle.
- **Override = the direction control** — the editable move-direction angle *is* the manual
  override; presets pre-fill common angles; existing `x_sign`/`y_sign` absorb handedness /
  recentering sign the operator validates on hardware.
- **Apply scope = needle centering only** — per user choice; `pixel_to_stage_offset`
  (live-view click → move) deliberately untouched this pass.
- **Aligner stays pure math** — takes absolute angles and solves a 2×2; the role→nominal
  mapping (`90°`/`0°`) lives in the calibration page, so the aligner is role-agnostic and
  reduces cleanly to the legacy behavior.
- **Cross-contamination (field-reported)** — calibrating one needle camera appeared to
  change the other's calibration. Root cause was *not* the identity store (the log shows
  distinct device-path identities), but `_ctx_update_um_px_display` re-deriving a µm/px
  from the shared microscope `camera_spec` and pushing it to **all** slots on every
  config-change (each calibration triggers one via `_on_config_changed`). Gating the push
  to the `MICROSCOPE` slot fixes it; as a bonus it stops needle slots being marked
  "calibrated" with a bogus theoretical value at startup. Regression test:
  `TestNeedleCalibrationNotClobbered`.

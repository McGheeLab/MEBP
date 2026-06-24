# MEBP v7.5.x — Needle auto-center direction fix + Z centering on the crosshair

## Objective

Two needle-location (Calibration → Needle Location) issues surfaced on the
Teslong needle cameras:

1. **Wrong-direction auto-center.** After clicking the left/right edges of the
   needle in each side camera and pressing *Center & Save*, the stage moves the
   needle the **wrong way** (centering diverges instead of converging).

2. **No Z centering.** The workflow only centers the needle laterally (XY). The
   operator wants the **center-bottom of the needle tip to land on the camera
   crosshair** — i.e. also drive **Z** so the tip's vertical position matches
   the crosshair center, not just its horizontal position.

## Root cause (issue 1 — wrong direction)

The needle cameras are 90° apart but the pair sits at ~45° to the stage X/Y
axes, so v7.5.x (`MEBP_v75x_CAL_LIVEVIEW_AND_CAMERA_ROTATION.md`) added a
measured per-camera rotation fed to `TwoCameraNeedleAligner`
(`angle_x_view_deg` / `angle_y_view_deg`). The aligner convention (pinned by
`tests/test_v75x_camera_rotation.py::test_45deg_recovers_known_offset`) is:

> `s_i = n · û(angle_i)` where `n` is the needle's stage offset and `s_i` is the
> **signed column offset × µm/px**; the caller recenters with
> `move_xy_relative_um(+n)`. So `angle_i` must be **the stage direction along
> which displacing the needle increases its image column** (moves it rightward).

But `PixelCalibrationDialog` stored `result_rotation_deg = the operator's chosen
move-direction preset` (`gui/dialogs/pixel_calibration_dialog.py:416`). For a
~45°-mounted camera the operator picks whichever preset (e.g. **+45°** or
**−45°**) *looks* lateral — **both** produce "clean lateral motion", just in
opposite column directions. So the stored sign was **arbitrary** and not tied to
"+column", giving a ~50/50 chance of a 180°-flipped angle per camera → the
recenter move points away from center → "wrong direction". (The pre-rotation
orthogonal default of 90°/0° had its sign baked in correctly, which is why it
"used to work".)

### Fix

Derive the sign from the **measured** image displacement (`measure_pixel_
displacement` returns content shift, +dx = rightward). Moving the stage/cameras
by `+m=dist·(cosθ,sinθ)` changes the needle's position *relative to the optical
axis* by `−m` while its apparent column changes by `+dx`. Hence displacing the
needle by `+θ̂` changes its column by `−dx/dist`:

* `dx < 0` → `+θ̂` increases column → `angle_i = θ`
* `dx > 0` → `+θ̂` decreases column → `angle_i = θ + 180°`

So `pixel_calibration_dialog` now derives `result_rotation_deg` from the
**measured displacement direction** instead of the commanded preset. Re-running
the needle-camera µm/px calibration re-captures the correct angle.

### Refinement (the "needle lands in the left quadrant" residual)

The first cut only resolved the ±180° **sign** but still used the **commanded
preset** (0/45/90/−45°) as the camera's lateral axis. If the camera's true
lateral axis sits a few degrees off the nearest preset, that leftover tilt
biases the two-camera solve and the needle lands off-center (consistently to one
side). The two-camera solve is exact *iff* both `angle_i` match the real camera
axes — so a coarse preset value is not good enough.

Model the stage→image map as `scale·R(α)`. A commanded move `+m=D·(cosθ,sinθ)`
makes content shift by `p=−scale·R(α)·m`; with `φ=atan2(dy,dx)` the stage
direction that maps to image **+column** is

```
angle_i = θ − φ − 180°   (= −α, independent of the preset θ the operator picked)
```

`plus_column_direction_deg(commanded, dx, dy)` now returns this. On a purely
lateral move (`dy=0`) it reduces exactly to the ±180° sign rule, so the on-axis
behavior is unchanged; off-axis it removes the preset-quantization bias. Because
the result is independent of `θ`, two operators picking different presets for the
same camera store the same lateral axis.

> **HW-verification note:** the absolute sign also depends on
> `cv2.phaseCorrelate`'s reported direction. If after re-calibration the move is
> *consistently* 180° off on **both** cameras, flip the `dx > 0` test once (a
> single constant). The XY move is bounded + soft-limit clamped and the *Offset
> preview* shows ΔX/ΔY before the move, so a wrong direction is visible and
> recoverable, not a crash.

## Z centering (issue 2)

The four edge clicks are taken at the needle tip's **bottom corners** (existing
intent, see the needle-cam-Z comment at `calibration.py:1620`). Each click now
also records its **row** (`py`, already delivered by `CameraFeedView.clicked`).
Per the aligner's mounting model, each side camera's **rows map to stage Z**, so:

* per view, `tip_row = midpoint(left.py, right.py)`,
  `row_offset_px = tip_row − frame_height/2` (+ = tip below crosshair center);
* `vertical_um = row_offset_px × µm/px`;
* height-frame move = `+vertical_um` (tip below center → retract up), averaged
  over both views, applied via `move_z_user_relative(mm)` (height frame, up = +,
  **soft-limit + plate-floor clamped**).

Z centering is **opt-in** (a *Also center Z (tip → crosshair)* checkbox, default
checked) plus an *Invert Z* checkbox (for vertically-flipped camera mounts). The
*Offset preview* shows ΔX/ΔY/ΔZ. `_needle_loc_center_and_save` does the XY move,
then the Z move (when enabled), then captures the (now centered) Z as the
needle-cam fiducial — so the fiducial becomes the repeatable "tip on crosshair" Z.

> **HW-verification note:** the **vertical** sign is *not* captured by the µm/px
> calibration (that only measures lateral motion). The default assumes a
> non-flipped side camera (down-in-image = needle lower = retract up). If Z moves
> the wrong way, toggle *Invert Z*. The move is clamped by the Z soft-limit +
> print-floor, so it cannot drive past the configured envelope.

## Files Modified

| File | Change |
|------|--------|
| `gui/dialogs/pixel_calibration_dialog.py` | `result_rotation_deg` derived from measured `dx` sign (deterministic +column direction) instead of the raw commanded angle. |
| `gui/pages/calibration.py` | Capture click rows; `_needle_loc_compute_z_offset_um`; ΔZ in the preview; *Also center Z* + *Invert Z* checkboxes; Z move in `_needle_loc_center_and_save`; step prompts say "bottom-left/right corner". |
| `tests/test_v75x_needle_center_direction_z.py` | New tests. |

## Implementation Steps

- [x] Update plan doc (this file)
- [x] Dialog: derive rotation sign from measured `dx` (`plus_column_direction_deg`)
- [x] calibration.py: capture rows + Z compute + opt-in Z centering UI + move
- [x] Tests (rotation-sign derivation; Z-offset math; sign branches) — 11 tests
- [x] Run affected suites (`test_v75x_camera_rotation` + new test + click-rim/hw-controls smoke) green

## Testing Notes

- `test_45deg_recovers_known_offset` (existing) still passes — aligner math
  unchanged.
- New: a commanded +45° move that measures `dx>0` yields `result_rotation_deg`
  of −135° (i.e. +45+180 normalized), and `dx<0` yields +45°.
- New: Z-offset math — tip below crosshair → positive (up) height move; *Invert
  Z* negates it; averaging two views.
- **Bench:** re-run needle-camera µm/px calibration for both Teslong cameras
  (re-captures correct rotation sign), then verify *Center & Save* converges in
  XY and the tip lands on the crosshair in Z.

## Diagnostics (added after "needle lands at left-¼, stable" report)

Verified in code that click coords and the aligner's `frame_width` both come
from the **same raw capture frame**, and the `CameraFeedView` crosshair is drawn
at frame-center — so there is no click-scale / center-reference mismatch. A
stable off-center fixed point therefore implies a wrong per-camera angle/µm-px or
that the two cameras' joint optical center genuinely differs from where the
needle was placed. To pin it without further guessing:

- `CalibrationPage._needle_loc_log_diagnostics()` logs, on Center & Save, each
  view's `L/R` clicks, midpoint, frame width+center, **column offset (px & µm)**,
  µm/px, angle, the computed move, and XY before/after.
- `_needle_loc_offset_breakdown()` adds a live `X: col=±Npx @θ°  Y: col=±Npx @θ°`
  line to the on-screen Offset preview (a needle on the crosshair should read
  `col≈0` in BOTH views).

## Near-center deadband (added after "large offsets center correctly, but a hand-centered needle drifts to the left-¼")

A multi-agent investigation concluded a global sign inversion (recenter = −n) and
proposed negating the move. That was **rejected**: the operator confirmed that
**large misalignments converge correctly**, which a global sign error could not
do (it would diverge from every start). The arithmetic also reproduced the
operator's exact numbers, ruling out unit/resolution bugs. So the sign and scale
are correct; the only failure is **near-center**, where small click-noise offsets
get turned into real moves (and the 2-camera solve amplifies the per-view
difference), nudging a hand-centered needle off the crosshair.

Fix = a parameter-free **deadband**: `_needle_loc_already_centered()` returns True
when the frame center (crosshair) lies **between the two edge clicks in BOTH
views** — i.e. the needle already covers the optical center to within its own
width. `_needle_loc_center_and_save` then records the origin **without** an XY
move (Z centering + origin capture still run). Surfaced live in the preview ("On
crosshair (within needle width) — Save records origin without moving."). Large
offsets (crosshair outside the needle span) still move and converge as before.
Tests: `TestDeadband` (5).

> **Still to confirm on HW:** that large single-axis offsets cancel in ONE press
> (sign fully correct) — the bench experiment below. The deadband addresses the
> reported symptom regardless of the residual near-center dynamics.

## Issues & Decisions

- **Why not a blanket negate of the aligner output?** The pre-rotation
  orthogonal path worked with `+move`; a global negate would have broken it. The
  defect is specifically the *arbitrary sign of the measured per-camera angle*,
  so the fix belongs at capture time.
- **Z is opt-in + preview + invert + clamped** because the vertical sign is not
  calibrated and a wrong Z direction risks the needle. XY is bounded/clamped and
  preview'd, so it ships without an opt-in.

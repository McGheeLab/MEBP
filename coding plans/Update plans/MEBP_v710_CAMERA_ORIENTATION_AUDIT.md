# v7.10 — Camera orientation audit: one frame of reference everywhere, and a needle-camera calibration that uses the needle

## Objective

Operator, two requests in one message:

> *"do a full audit of the microscope camera live view and hardware calibration
> and mosaic calibration. All of these surfaces should agree on the same camera
> orientation and frame of reference for the stage etc. the live view should
> always be non mirrored looking to the user. Ensure that all of these align
> with each other and that everything is referencing the smallest number of
> variables possible so we ensure its all the same."*

> *"The needle cameras need the needle as a reference object to be able to
> calibrate stage motion to its rotation etc, so the needle must move on the
> needle cameras axis to be able to judge rotation. we should check z motion,
> and a 45 degree stage motion to ensure it stays within the camera frame. this
> 45 degree motion already works for calibration of the needle cams um to px so
> we can reuse that as well."*

---

## Part 1 — What the audit found

The **convention** was already right and already singular: every consumer applies
`A = R(θ)·diag(mx, my)` with `mx = −1` iff mirrored and `my = −1` iff flip_y —
`CameraFeedView.view_transform_coeffs`, `MosaicBuilder._orient_tile`,
`CameraManager.pixel_to_stage_offset` / `stage_offset_to_pixel`. That is one
convention and it is correct: for a mirrored camera `det A = −1`, so the display
transform reverses handedness and the operator sees an **un-mirrored** image.

What had diverged was everything *around* it — who answers "what is θ for this
camera", who is allowed to change it, and which surfaces bother to ask.

### F1 — Three implementations of the same precedence rule

| Implementation | Precedence |
|---|---|
| `MosaicCalibration._resolve_orientation` | store → manager → neutral |
| `CalibrationPage._ploc_microscope_frame_orientation` | store → manager → neutral, **hand-rolled again** |
| `CameraFeedView._sync_auto_orientation` | **manager only**, no store |

Three copies agreeing on the day they were written says nothing about three
copies agreeing after the next edit. Consolidated into one
`MosaicCalibration.resolve_camera_orientation`.

### F2 — 🐞 An objective switch silently re-oriented the live view

`ObjectiveCalibration` keeps a per-objective `rotation_deg`.
`MosaicCalibration`'s docstring is explicit that it must never be read — *"keeping
a second home for it is what let a stale per-objective value overwrite a freshly
measured one"* — and `MosaicCalibration` indeed never reads it. But **two other
call sites did**, and pushed it into the live `CameraManager` on every objective
change:

* `objective_calibration_card._push_stored_um_per_px_to_manager`
* `fluorescence_mosaic_workflow._on_objective_changed`

So switching objectives could move the live rotation while the
`CameraCalibrationStore` (ground truth) stayed put. The mosaic reads the store
and stayed **right**; the live view and `pixel_to_stage_offset` read the manager
and went **wrong** — the same camera, two orientations, and clicks mapping to the
wrong XY with nothing on screen to say which angle was in force.

The existing mitigation (`adopt_camera_rotation`, which syncs the fresh rotation
into *every* objective entry) only covers objectives that exist at the time; one
calibrated later still carries a stale copy. Rather than keep patching the
mirror, the **read** is gone. Rotation now has exactly one live source.

### F3 — 🐞 A measured rotation could be thrown away by a restart

The per-objective µm/px calibration (`_on_calibrate_clicked`) *also* measures a
rotation. It pushed that to the live manager and the per-objective copy, and
**never wrote the per-identity mount store**. Result: live view right, mosaic
keeps the old angle, and the next restart (`restore_calibration_from_store`)
reverts the measurement entirely — a calibration that appears to take and then
vanishes.

Fixed by one `commit_camera_rotation(cam_idx, θ)` that writes all three homes,
used by both of the card's calibration buttons.

### F4 — Six live views showed the camera's raw image

`auto_orient` defaults to `False` and had to be opted into per call site. Missing on:

1. **`hardware_setup` per-slot preview** — the worst one. This is the surface
   where the operator *sets* the mirror / flip-Y / rotation for a slot, and it
   was the only feed still showing raw pixels. Ticking "Mirrored view" changed
   the mosaic and every other page's feed while the preview six inches away did
   not move.
2. Needle Location needle feeds
3. Pump Compliance needle feeds
4. Calibration Custom-tab camera grid
5. `TargetOverlayCameraView` — the parameter **could not be passed at all**, so
   every pick/place live view (spheroid, cell targeting, cell labeling) rendered
   raw
6. `_ploc_live_view` — oriented, but by a *fourth* mechanism
   (`_ploc_apply_feed_orientation`, pushed only on camera bind / tab entry, so an
   orientation changed elsewhere left it behind)

All now `auto_orient=True`, which re-reads each frame and is therefore
self-healing. Two deliberate exemptions, each stated in the test:
`camera_rotation_align_dialog` (corrects the flips but deliberately *shows* the
rotation, since the rotation is what is being changed) and
`camera_settings_dialog` (previews the raw device output while device settings
are being changed).

### F5 — 🐞 Orienting the picker feed exposed a latent inverse bug

`_PickerCameraView._image_to_widget` mapped raw frame pixels to widget pixels
with a plain scale-and-offset, and its docstring justified that: *"this branch of
the hierarchy bypasses `_orient_qimage`, so there is no view transform to
invert"*. True only while the feed was raw. With it oriented, every target ring,
radius handle and hit test would land somewhere else on a rotated or mirrored
camera — the same class of defect as the v7.8 `to_px` identity-inverse bug this
method was written to fix. Now maps through `_view_true_xform`, making it a
genuine inverse of `_widget_to_image` on any camera.

### F6 — 🐞 A needle feed must NOT rotate by the residual (corrected after operator feedback)

The v7.5.x mount/roll split left the needle-cam roll measured, stored and
applied nowhere. My first cut applied it to the needle feeds. **That was wrong,
and the operator caught it on the bench:**

> *"there is a problem with how it tells me to change the rotation of the needle
> cameras. when i did this it kept shifting the live view to the rotation
> orientation. the entire point of this is that the live views do not rotate.
> they should be at fixed rotations on our screen corresponding to 0,90,180,270
> if needed. then the rotation needed is set and we try to match it."*

Correcting the residual in software is **self-defeating for a mount the operator
squares by hand**: the tilt they are trying to remove disappears from the view,
and as they turn the camera the correction follows them, so the picture never
appears to change — no feedback at all. It also resamples every frame for a few
degrees of correction, where a cardinal turn is a lossless fast path.

The rule is now split by *what the value is for*:

| | Display | Geometry (`pixel_to_stage_offset`, `_orient_tile`) |
|---|---|---|
| Flips (mirror / flip-Y) | applied — the view is always un-mirrored | applied |
| Rotation, needle cams | **nearest 0/90/180/270**, residual left visible | **exact θ** |
| Rotation, microscope | exact θ (unchanged — the operator reports this working well) | exact θ |

Implemented as `CameraFeedView(snap_rotation_to_cardinal=True)`, opt-in and
**off by default**, so the microscope path is untouched. A pinned test builds a
view *without* the parameter to catch a flipped default — a mutation that flipped
it survived the first version of that test, which is why it exists.

Side effect: this also removes the tilt hazard from the three orphaned Teslong
entries still holding the pre-v7.5.x conflated ±45°, since ±45 snaps to a
cardinal rather than tilting the view.

### F7 — `set_edge_pick_mode` has zero production callers

Only tests call it. Noted, not changed — the suppression branch is harmless and
removing it is its own change.

---

## Part 2 — The needle cameras

### The hardware fact that makes this work

The needle side cameras are bolted to the **XY stage**; the needle hangs from Z.
`plus_column_direction_deg`'s own docstring says it: *"Moving the **cameras** by
`+m` … makes stationary content shift"*. So:

* an **XY move** moves the CAMERA — the stationary needle *and the background*
  translate together;
* a **Z move** moves the NEEDLE and nothing else.

### 🐞 One leg is rank-one, and it over-estimated µm/px undetectably

A side camera only sees the component of a move perpendicular to its optical
axis. If the commanded direction is off the true lateral by `β`, the image moves
by `D·cos β` worth of pixels, so `u = D / |p|` comes out **too large by
`1/cos β`** — 6 % at 20°, **41 % at 45°**. Nothing in the single-leg flow could
detect this. The operator has been compensating by hand: clicking direction
presets and keeping "the longest arrow" is exactly a manual search for
`cos β → 1`.

A **Z** move has no such problem — lab-vertical is perpendicular to a horizontal
optical axis whatever the camera's azimuth. So `u_z` is the trustworthy scale,
and `u_lat / u_z = 1/cos β` **recovers the very error that used to be invisible**.

Two more things the Z leg settles, both of which were assumptions:

* **The roll** was inferred from the lateral leg, which requires knowing where
  lateral *is* — circular. Lab-vertical is a known direction, so the angle at
  which it lands on the sensor **is** the roll.
* **"Image rows map to stage Z"** is asserted in `TwoCameraNeedleAligner`'s
  docstring and the whole Z-centring path depends on it, but nothing checked it,
  and the **sign** was a manual *Invert Z* checkbox — a guess that, got wrong,
  drove the needle the wrong way vertically. Now measured per camera and stored
  as `z_row_sign`; absent means never measured, so the checkbox still governs an
  un-remeasured camera and nothing changes for it.

### The Z leg stands alone (corrected after operator feedback)

> *"we should not have to do the xy version of the calibration for the needle
> cam rotation correction, the z is independent and if we want to trust that
> more than the xy we should be able to use that exclusively."*

Correct, and my first cut had it backwards — the Z button was gated on the XY
leg, which made the trustworthy measurement depend on the one it supersedes.

The Z leg **fully determines** the µm/px, the sensor roll and the Z direction.
The XY leg adds exactly two things: the aligner's `column_dir_deg`, and the two
cross-checks that need a second vector (orthogonality and the foreshortening
ratio). So it is optional:

* `solve_needle_camera_axes` takes the Z leg as required and the lateral leg as
  optional; the lateral-derived fields are `None` and `has_lateral` says so.
* `refusal()` judges only what was measured — a Z-only solve is gated on the Z
  leg's own background-lock check, which is **not** relaxed.
* The commit path writes the roll, µm/px and `z_row_sign`, and leaves any
  previously measured `column_dir_deg` **untouched** rather than discarding the
  whole result.

A pinned test shows the same Z data being refused when a hopeless XY leg is
attached and accepted on its own — which is the whole point of being able to
drop it.

### NEW `SupportClasses/NeedleCameraCalibration.py` (pure — no Qt, no cv2 at import)

`solve_needle_camera_axes(z_um, z_dx/dy_px, [lateral_um, lateral_dx/dy_px])` →
`NeedleCameraAxes(um_per_px, um_per_px_lateral, roll_deg, z_row_sign,
orthogonality_err_deg, scale_ratio, off_lateral_deg, …)`.

**Verified by forward simulation** over roll × optical azimuth × handedness ×
preset (200 combinations): of the 130 geometrically usable ones, every single
one recovers **µm/px to 4.4e-16**, **roll to 0.0°** and **orthogonality to 0.0°**.
The other 70 are correctly **refused** — they are moves along the optical axis
(`β = 90°`, no lateral signal at all) or `β > 46°` where the scale would be
meaningless.

It **never raises**; a degenerate input returns an object whose `refusal()`
explains what went wrong. Four refusals, each for a real failure:

* **XY leg barely moved** — optical-axis move, pick another direction.
* **Z leg barely moved** — *"a Z move moves the NEEDLE and nothing else, so a
  near-zero reading means the tracker locked onto the stationary background"*.
  This is why the Z leg uses `select_trackable_patch` + `find_template` rather
  than the XY leg's whole-frame phase correlation: phase correlation would
  faithfully report the background's zero displacement and call it a
  measurement.
* **Legs not 90° apart** — square pixels on a rigid scene must be perpendicular,
  so the two legs are tracking different objects.
* **Scale ratio outside [0.85, 1.45]** — above ≈1.45 the XY move is >45° off
  lateral; below 1.0 is not physical.

### 🐞 In-frame pre-check — because the two failure modes are indistinguishable

"The move went along the optical axis" and "the feature left the sensor" arrive
as the **same symptom**: a tiny, low-confidence displacement. So the move is now
bounded *before* it is commanded (`in_frame_refusal`, ~25 % of the shorter frame
dimension), which makes a small reading mean the first — the one the operator can
actually fix. Silent when µm/px is unknown, since the first calibration has to be
allowed to run.

### Safety

The Z leg goes **UP first** (`move_z_user_relative(+dz)` — the height frame, so
polarity-safe on either `z_up_sign`) and returns in a `finally`. Up is away from
the plate. A calibration that descended a blind 200 µm could put a needle through
glass.

### Also fixed while here

The needle-cam µm/px commit dropped its **resolution stamp**
(`set_calibrated_um_per_px(..., resolution=None)`), so `effective_um_per_px` had
nothing to rescale from and the aligner used a value that was only right at one
capture resolution. Now stamped from the live frame.

---

## Files Modified

| File | Change |
|---|---|
| `SupportClasses/MosaicCalibration.py` | NEW public `resolve_camera_orientation` — the one implementation |
| `SupportClasses/NeedleCameraCalibration.py` | **NEW** — pure two-leg solver, refusals, in-frame guard |
| `SupportClasses/CameraCalibrationStore.py` | `get/set_z_row_sign` (absent = never measured) |
| `gui/dialogs/pixel_calibration_dialog.py` | `needle_mode` + Z leg + in-frame guard on both legs |
| `gui/pages/hardware_setup.py` | slot preview `auto_orient`; `needle_mode`; `_apply_slot_z_row_sign`; `_live_capture_resolution`; resolution stamp |
| `gui/pages/hardware/objective_calibration_card.py` | one `commit_camera_rotation`; stop pushing per-objective rotation |
| `gui/pages/workflows/fluorescence_mosaic_workflow.py` | stop pushing per-objective rotation |
| `gui/pages/calibration.py` | delegate to the shared resolver; 4 feeds `auto_orient`; `_needle_loc_z_sign` |
| `gui/widgets/target_overlay_camera_view.py` | forward `auto_orient`, default on |
| `gui/widgets/live_target_picker.py` | `_image_to_widget` maps through the view transform |
| `tests/test_v710_camera_orientation_audit.py` | **NEW** (38) |

## Implementation Steps

- [x] One shared orientation resolver; both other copies delegate
- [x] Per-objective rotation stops reaching the live manager
- [x] One rotation commit path writes manager + mount store + objective mirror
- [x] `auto_orient` on every human-facing feed; picker inverse fixed
- [x] Pure two-leg needle solver + forward-simulation verification
- [x] Z leg in the dialog (needle-locked tracking, safe up-first move)
- [x] Measured `z_row_sign` supersedes the Invert-Z checkbox
- [x] In-frame pre-check on both legs
- [x] Tests + mutation verification + regression

## Testing Notes

**50 new tests.** Several are pinned by **source structure rather than
behaviour**, deliberately: three copies of a precedence rule agree until they
don't, and a "does the mosaic still work" test cannot see F2 at all — the mosaic
reads the store, which stayed correct the whole time. `TestEveryFeedIsOriented`
walks the `CameraFeedView(...)` call sites so a **new** un-oriented feed is
caught, not just the ones already fixed; it self-guards by asserting it found at
least 8 call sites, since a matcher that finds nothing passes.

**23/23 mutations CAUGHT across two rounds** (each a real source edit, reverted in a `finally`):
per-objective rotation push restored ×2 · rotation commit stops persisting ·
plate-location page re-rolls its own precedence · slot preview back to raw ·
needle feed back to raw · target-overlay stops forwarding · picker ignores the
view transform · Z-leg µm/px no longer supersedes · roll sign flipped ·
background-lock check removed · orthogonality check removed · `z_row_sign`
hard-coded +1 · in-frame guard dropped · measured Z sign ignored.

⚠ **One of my own tests was too weak and the mutation caught it.** The in-frame
guard test asserted `"in_frame_refusal" in getsource(...)`, which passes on the
*import line* alone — so deleting the call and hard-coding "no refusal" survived.
Rewritten to find the `Call` node by AST, plus a second test that the refusal
gates an actual `return`. Both then caught it.

**Regression: 1,000+ green across 33 suites, run per-suite.** Camera/orientation
(unified-mosaic-cal, scale-fov, rotation-cal-and-monitor, rotation, needle-cam
mount-and-roll, square-up, objective-cal, camera-store, image-correction,
hw-controls, async-open), mosaic (orientation-adjust, mapping-orient, reanchor,
memory-perf, unreachable-travel, v731-mosaic, plate-mosaic, well-detection,
rosette-reanchor, startup-well-map, fluorescence, fluor-shift), picker/workflow
(target-actions, spheroid-page-integration, picker-scaling, spheroid-detection),
needle/bore (bore-wizard, optical-datum, bore-gate, bore-focus-roi,
bore-offset-cal, microscope-bore-sign), plus suite-hygiene, calibration-revision,
click-rim, needle-quick-move, and a `gui.app` import smoke.

Offscreen GUI smoke against the **real** `HardwareSetupPage`: 4 slot previews
built, all `auto_orient`, and a mirror+180° set on the manager is picked up by
the preview; `needle_mode` shows the Z group only for a needle slot and keeps its
button gated until the XY leg lands; `TargetOverlayCameraView` defaults on.

**Two pre-existing failures, both PROVED not mine:**
* `test_v75x_plate_mosaic::test_real_24_well_mosaic` (23 vs 24) — the legacy blob
  detector, untouched here. Re-run with my changes **stashed**: still fails.
  (The newer `PlateWellDetector` resolves all 24 on the same image.)
* `test_v79_cell_targeting_setup_page::test_the_real_saved_profile_reproduces_its_exact_volume`
  — the documented golden-file test over the operator's own re-saved
  `config/workflows/cell_targeting/__last__.json`.

`test_v75x_plate_mosaic` was run with `TestManualAlignPage` excluded — the
documented pre-existing hang that stops that suite terminating.

## Issues & Decisions

* **Kept store-first precedence** in the shared resolver rather than switching
  everything to the manager. With F2 fixed the two agree, and store-first is
  still correct when the manager has come up un-synced.
* **`adopt_camera_rotation` kept but demoted.** It was load-bearing only because
  of the push that is now gone. Retained as defence in depth: it keeps the
  per-objective copies consistent for an older build reading the same files.
* **`rotation_deg = −roll_deg`.** The stored value is the display rotation that
  renders the view level; the solver reports the sensor roll. Pinned by composing
  against the legacy `view_roll_from_displacement`, so the two-leg path cannot
  silently invert the display relative to every camera calibrated before it.
* **Lateral leg keeps phase correlation**; only the Z leg switched to template
  tracking. On an XY move everything moves together so either works, and changing
  the proven path would be gratuitous risk.
* **Not done:** removing the dead `set_edge_pick_mode` branch; folding
  `hardware_setup._apply_slot_rotation` into `commit_camera_rotation` (it already
  writes all three homes correctly, on a different class).

## Needs real-HW verification on ME3B V1, IN ORDER

1. **Cameras page** — tick *Mirrored view* on the microscope slot: the **slot
   preview itself** must flip immediately (it never did before). Untick it.
2. Confirm the live feed reads **un-mirrored** on the Andor (`mirrored: true`) —
   text/features the right way round.
3. **Switch objectives** (4× → 10× → 4×) and confirm the live view does **not**
   rotate and the mosaic still orients the same. Then restart and confirm the
   rotation survived.
4. Run the per-objective **Calculate µm/px** and confirm the rotation it measures
   is still there after a restart (it used to be discarded).
5. **Needle cameras — Z leg ON ITS OWN first.** Press *Measure Z leg* without
   touching the XY leg. Expect the needle to retract ~200 µm and return, and a
   result reporting roll + µm/px + Z direction, with a note that the mount
   direction was left alone.
6. **Confirm the needle view does NOT rotate** while you turn the camera in its
   mount: the picture must hold still at its quarter-turn while the reported
   residual changes. If the view swings to follow the correction, this
   regressed.
7. **Then, only if the aligner's mount direction needs re-measuring:** XY leg at
   ±45° → *Measure Z leg*. Expect the needle to retract ~200 µm and return. Check
   **`µm/px` differs from the XY-alone figure** — that gap is the foreshortening
   that used to be silent — and that the axes report ≈90° apart.
8. Deliberately point the XY move down the optical axis and confirm it is
   **refused** rather than producing a number.
9. Deliberately ask for a 2000 µm Z leg and confirm the **in-frame guard** refuses
   before any motion.
10. Confirm the needle now hangs **vertical** in both side views (the roll is
   finally applied) and that edge-picking still lands the needle on both
   crosshairs — clicks are inverted to raw coords, so the aligner math is
   unchanged.
11. Confirm the Z-centring drives the **right way** with *Invert Z* left alone —
   the measured sign now governs.
12. Pick/place pages: the live picker feed should now match the microscope feed's
    orientation, and target rings must still land where clicked.

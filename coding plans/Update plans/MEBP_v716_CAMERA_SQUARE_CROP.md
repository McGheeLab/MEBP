# MEBP v7.16 — Square sensor crop, applied at the frame source

## Objective

> *"it turns out that the image sensor is too wide, and we see some of the dark
> circles of the field of view. can we crop the image into a square in the camera
> calibration part of the camera detection settings page. this cropping should be
> assigned to all surfaces including mosaic, live view, image, recording etc."*
> — operator

A microscope's illuminated field is a **circle**. A sensor wider than that circle
images the unlit tube wall at the left and right edges. Those pixels:

* carry no specimen;
* drag the mosaic's flat-field estimate (v7.16 round 1) toward black;
* are still stepped over by the raster, so they cost tiles as well as quality.

This rig's Tucsen Libra 25 delivers **2600 × 2048** — a 1.27:1 sensor — so a
square crop discards a 276-px band on each side.

---

## Three things had to be true

### 1. Every surface sees the same pixels

There are exactly **three** points where a frame leaves `CameraWidget`, and they
are the reason this is applied at the source rather than per consumer:

| Egress point | Feeds |
|---|---|
| `_grab_frame` | the raw cache (`get_current_frame` → detection, mosaic, calibration, autofocus), the on-screen pixmap, and `frame_captured` → **video recording** |
| `capture_fresh_frame` | mosaic tiles, calibration grabs, the still-capture display path |
| `capture_raw_average` | **raw 16-bit stills** — reads the sensor buffer straight from the backend |

Missing any one of them is how a raw still ends up being the only image in the
app that still shows the vignetted edge. All three go through one
`CameraWidget._apply_crop`.

This is deliberately **unlike the mirror / rotation**, which are applied per
consumer precisely so the mosaic can re-blend against a *changing* orientation.
A crop cannot be undone after the fact — but it does not need to be: a vignetted
edge has no information to preserve. What it does need is for the raw cache, the
mosaic tile, the detection ROI and the click→stage map to agree about how many
pixels a frame has, and cropping once is the only way to get that for free.

### 2. µm/px must not move

**Cropping removes pixels; it does not change what a pixel spans.**

Four separate sites rescale a stored µm/px by frame width (`µm/px ∝ 1/width`).
That law describes the sensor being *sampled differently* — binning, a resolution
switch: the same optical field over more pixels. A crop changes the pixel count
without changing the pixel, so feeding a **delivered** width into any of those
ratios scales the answer by the crop fraction:

```
1.2710 × 2600 / 2048 = 1.6136 µm/px      ← +27 %, and in the WRONG direction
```

Too large means the mosaic believes its field is wider than it is and steps past
its own tiles — **seams**. This is the same failure mode as the 400-minute scan
(round 1), reached by a different route, so all four sites now resolve their
denominator through one helper:

`MosaicCalibration.capture_width_px(camera_manager, cam_idx, fallback)`

| Site | Was | Now |
|---|---|---|
| `CameraManager.effective_um_per_px` | `base × cal_w / live_w` | `… / capture_width` |
| `MosaicCalibration._resolve_scale` | `base × cal_w / live_res[0]` | `… / capture_width_px(...)` |
| `CalibrationPage._ploc_microscope_um_per_px` | `meas × cal_w / frame_w` | `… / capture_width_px(...)` |
| `PixelCalibrationDialog._expected_um_per_px` | both store legs on `frame_w` | both on `cap_w` |

With no crop configured, capture width **is** the delivered width, so every one
of these is byte-identical to before.

**Corollary — the stamp names a sensor mode, not a frame size.**
`_true_capture_resolution` (whose docstring already promised "the real pixels off
the sensor") and both dialogs' `result_resolution` now record the **pre-crop**
size. Two consequences:

* turning the crop on or off never invalidates an existing calibration — the
  ratio is 1.0 either way, verified by test;
* `ObjectiveCalibration.sensor_width_um` — the round-1 plausibility guard, whose
  whole premise is that `µm/px × magnification × width` is ONE fixed property of
  the camera — keeps working. Stamp two objectives at different crops and that
  invariant would report the camera's own objectives as disagreeing (pinned by
  test).

### 3. Choosing WHERE the crop sits must not move anything else

The lit circle is centred on the optical axis, not necessarily the sensor. So the
crop is positionable — and because ``pixel_to_stage_offset`` measures from the
frame centre, that displacement is **cancelled** in all three pixel-stage
conversions rather than left to shift every taught coordinate. See *Design
decisions* below.

---

## Design decisions

**Where the crop sits is choosable — and compensated.** (Round 2; operator:
*"i need to choose where the crop occurs in the frame"*.) The illuminated circle
is centred on the **optical axis**, which need not pass through the middle of the
sensor — a C-mount adapter or a slightly off-axis tube lens puts it elsewhere, and
then a centred square still clips one side. So `CameraCrop` gained
`offset_x` / `offset_y`.

That is not free, and it is the reason this section exists rather than just a UI
row. `pixel_to_stage_offset` maps a click by its offset from the frame **centre**,
so moving the crop moves the pixel that means *"the stage is here"* — and every
click, every overlay and every mosaic tile placement would follow it. Measured on
this rig: **up to ~1 mm** at a small crop, i.e. a silently relocated plate
calibration. Two honest choices existed:

1. treat the offset as a redefinition of the camera's reference point and warn
   that it invalidates a taught calibration (simple, but every nudge would force
   a re-teach — and nothing in this codebase measures the true principal point, so
   the "definition" was always arbitrary);
2. **cancel it**, so an offset crop maps to exactly the coordinates a full frame
   would and the operator can re-aim freely.

(2) shipped. Three places convert pixel↔stage and all three compensate:

| Place | Compensation |
|---|---|
| `pixel_to_stage_offset` | `+d` in RAW pixels, before the flips and rotation |
| `stage_offset_to_pixel` | `−d` at the mirroring point in the inverse chain |
| `MosaicBuilder` tile placement | `+d` mapped into the stage frame, at the two points a stage position enters the builder |

`d` comes from `CameraCrop.center_offset_px`, which is derived from the **achieved
rect** — never the requested fraction. At the clamp (a crop pushed against the
sensor edge) the two differ, and it is the applied one that has to be cancelled;
deriving it from `offset_x` drifts from reality exactly where it matters most
(mutation M15).

⚠ **The mosaic and the click path MUST use one convention.** Two conventions
consistently applied would each be fine; one of each is the bug. So
`MosaicCalibration.crop_offset_um` puts `d` through the *same*
`µm/px · R(θ) · diag(mx, my)` chain a click does, and a test asserts the two agree
at every rotation × flip combination. The mosaic side needed **no caller churn**:
`MosaicBuilder` gained one `tile_center_offset_um` applied inside `add_frame` /
`add_raster_frame`, wired from `build_mosaic_builder` — already "THE single place
builder arguments are chosen".

Verified: **0.000e+00 µm** disagreement between the cropped and uncropped mapping
of the same physical point, across 5 rotations × 4 flip combinations × 3 offsets ×
2 scales; forward/inverse round-trip to 2e-13 px; and a marked feature in a
synthetic mosaic back-projects to its **true** stage position (uncompensated it
would be off by the full offset — mutation M18).

⚠ **The canvas has to grow.** The scan bounds are the operator's region, not the
tiles' footprint, and at a small crop the displacement can *exceed* half a FOV —
so `_init_composite` pads by `|d|` as well. Only the canvas grows; the origin is
unchanged, so extent→stage back-projection is untouched (mutation M20).

**The crosshair follows the reference pixel.** It marks where the stage is
pointing, which is the frame middle only while the crop is centred. Left at the
geometric middle it would aim the operator at a point the stage is not on, so
`_draw_crosshair` uses `CameraCrop.reference_pixel`.

**A fraction, not pixels.** `scale` is a fraction of the sensor's short side, so
the crop describes a *physical region of the field* and survives a
capture-resolution change (2600 → 5200 when 2×2 binning is switched off keeps the
same physical square). Storing pixels would silently mean a different area after
that switch — the same class of stale-number bug as an unstamped µm/px
resolution. The default is 100 % (the largest square that fits) with a 10–100 %
spin, because the operator's symptom — *"we see some of the dark circles"* —
means the illuminated circle may be smaller than the short side, and no square
crop at all would fix that.

**The centre is preserved exactly.** `CameraManager.pixel_to_stage_offset` maps a
click by its offset from the frame **centre**, so an off-centre crop would put a
constant offset into every click→stage conversion — and into the mosaic
back-projection every well centre is derived from. `rect_for` matches the crop's
parity to the frame's, making the centre exact rather than up to half a pixel
off. Pinned across odd/even frame sizes and six scales; a mutation setting
`x0 = y0 = 0` is caught.

**A malformed stored crop means NO crop.** An unexpected crop is a silently wrong
field of view; no crop is merely the old behaviour. `CameraCrop.from_dict`
returns the neutral crop for anything unreadable.

**Per device identity, in `CameraCalibrationStore`.** The crop is a property of
the physical camera + optics on this machine, exactly like µm/px and the flips —
not of the swappable hardware setup. It follows the camera across slot
reassignment and is restored on load. A disabled crop **pops the key**, so a
camera that has never been cropped keeps a byte-identical entry and nothing is
migrated.

**Restore clears as well as sets.** A slot's `CameraWidget` outlives the source
assigned to it, so the crop restore runs **before** the no-entry early return and
pushes even when the store has nothing: reassigning a slot from a cropped camera
to an uncropped one must not leave the new camera cropped. (Mutation M10.)

---

## Files modified

| File | Why |
|---|---|
| **NEW** `SupportClasses/CameraCrop.py` | The pure model: `CameraCrop(mode, scale, offset_x, offset_y)`, `rect_for` / `size_for` / `apply` / `center_offset_px` / `reference_pixel` / `to_dict` / `from_dict` / `describe`. No Qt, no cv2 — numpy slicing only, so it works on 2-D uint16 raw and 3-D BGR alike. |
| `gui/widgets/camera_widget.py` | `_crop` + `_capture_size`; `set_crop` / `crop()` / `capture_size()` / `_apply_crop`; applied at all three egress points; `_draw_crosshair` follows the reference pixel. |
| `gui/widgets/camera_manager.py` | `get_crop` / `set_crop` (delegating to the widget, cache for slots without one), `capture_resolution`, `capture_width`, `crop_center_offset_px`; `effective_um_per_px` divides by the capture width; both pixel-stage maps cancel the crop displacement. |
| `SupportClasses/MosaicCalibration.py` | NEW `capture_width_px` (the one authority); `_resolve_scale` uses it; `MosaicCalibration.capture_resolution` + `crop_offset_px` + `is_cropped` + `crop_offset_um`; `was_rescaled` compares capture widths; `describe()` reports the crop; `build_mosaic_builder` passes `tile_center_offset_um`. |
| `SupportClasses/CameraCalibrationStore.py` | `get_crop` / `set_crop`, siblings preserved, disabled pops the key. |
| `gui/pages/hardware_setup.py` | Per-slot **⬛ Square crop** checkbox + size % spin + delivered-size readout; `_slot_crop_from_ui` / `_on_toggle_crop` / `_on_crop_scale_spin` / `_apply_slot_crop` / `_sync_crop_controls` / `_sync_crop_spin_enabled` / `_refresh_slot_crop_displays`; restore in `_restore_calibration_for_slot`; the Microscope Camera Setup FOV line reports the **delivered** field. |
| `gui/pages/hardware/objective_calibration_card.py` | `_true_capture_resolution` asks the manager for the pre-crop size first. |
| `gui/dialogs/pixel_calibration_dialog.py` | `_expected_um_per_px` resolves both store legs against the capture width. |
| `gui/dialogs/scale_fov_calibration_dialog.py` | `result_resolution` stamps the capture resolution (measure path and Verify path). |
| `gui/pages/calibration.py` | `_ploc_microscope_um_per_px` divides by the capture width. |
| `SupportClasses/MosaicBuilder.py` | `tile_center_offset_um` — added at the two points a stage position enters the builder, so no caller changes; canvas padded by the displacement. |
| **NEW** `SupportClasses/GuiWatchdog.py` | Round 3. GUI-thread stall watchdog: a QTimer heartbeat + a daemon watcher that dumps every thread's stack to `logs/freeze.log` when the event loop stops turning. Observation only — it never interrupts anything. |
| `main.py` | Arms the watchdog right after the QApplication exists. |
| **NEW** `tests/test_v716_camera_square_crop.py` | 75 tests. |
| **NEW** `tests/test_v716_gui_watchdog.py` | 9 tests. |
| `gui/widgets/camera_manager.py` | Round 3b. `_probe_tucam_bounded` + `_tucam_cache` + `TUCAM_PROBE_TIMEOUT_S` — a wedged Tucsen can no longer freeze detection. |
| **NEW** `tests/test_v716_tucam_probe_timeout.py` | 9 tests. |
| **NEW** `tools_reset_tucsen_camera.ps1` | Round 3c. Elevated USB re-enumeration + SDK verification; refuses while MEBP is running. |

---

## Implementation steps

- [x] Pure `CameraCrop` model (fraction-based, parity-exact centring, tolerant reader)
- [x] `CameraWidget`: crop at all three egress points + `capture_size()`
- [x] `CameraManager`: delegation + `capture_width` + crop-safe `effective_um_per_px`
- [x] `MosaicCalibration`: shared `capture_width_px`, `capture_resolution`, honest `was_rescaled`
- [x] The other two rescale sites (`_ploc_microscope_um_per_px`, `_expected_um_per_px`)
- [x] Calibration stamps record the capture resolution
- [x] Per-identity persistence + restore (including the clear-on-reassign case)
- [x] Per-slot UI on Hardware Setup → Cameras + delivered-size readout
- [x] Microscope Camera Setup FOV reports the delivered field
- [x] Tests + mutation verification
- [x] **Round 2** — choosable crop position (`offset_x` / `offset_y`), clamped
- [x] Compensation in all three pixel-stage conversions, one shared derivation
- [x] `MosaicBuilder.tile_center_offset_um` wired from `build_mosaic_builder`
- [x] Canvas padded for the displacement; crosshair follows the reference pixel
- [x] Offset controls + re-centre button + achieved-offset readout
- [x] **Round 3** — operator reported a freeze: debounced the per-step store write
- [x] GUI-thread stall watchdog so the next freeze leaves a stack trace
- [x] **Round 3b** — the watchdog named `TUCAM_Api_Init`; bounded the probe

---

## Testing notes

`tests/test_v716_camera_square_crop.py` — **67 green**, structured as the three
claims above:

* `TestCropGeometry` / `TestCropPersistenceModel` — the model, including the
  exact-centre property across odd/even sizes, and `test_it_crops_the_pixels_the_operator_asked_about`
  (a synthetic dark-band frame; the crop must remove exactly the band).
* `TestEveryFrameEgressPointCrops` — all three egress points, driving the
  **production** `CameraWidget` (a stand-in that crops would prove nothing about
  it). `test_the_display_grab_crops_the_cache_the_pixmap_and_the_signal` pins the
  raw cache and the recorder's `frame_captured` payload together, since they come
  off one array.
* `TestUmPerPxIsInvariantUnderCropping` — each rescale site separately, plus
  `test_a_real_resolution_change_still_rescales` (the crop must not disable the
  formula it shares) and `test_the_naive_answer_would_have_been_visibly_wrong`,
  which guards the guard: it asserts the delivered-width answer differs by >25 %,
  so the invariance tests cannot pass merely because nothing rescales at all.
* `TestClickToStageSurvivesTheCrop` — the same physical point addressed in the
  uncropped and the cropped frame resolves to the same stage offset, and the
  forward/inverse maps still round-trip at −90°.
* `TestTheCalibrationStampNamesTheSensorMode` — the stamp is pre-crop, and the
  `sensor_width_um` invariant refuses when one objective is stamped cropped.
* `TestCropPlacement` — the offset moves the box, is clamped inside the frame,
  and `center_offset_px` reports the ACHIEVED shift (at the clamp the request is
  1300 px and the applied shift is 276 — the compensated one).
* `TestAnOffsetCropIsCompensated` — the invariant over 5 rotations x 4 flip
  combinations x 3 offsets x 2 scales, the honest consequence
  (`test_the_cropped_frame_centre_is_no_longer_the_stage_position`), the
  forward/inverse round-trip, and
  `test_the_mosaic_and_the_click_path_agree_about_the_frame_centre` — the one that
  makes two conventions impossible. Plus
  `test_the_uncompensated_error_would_have_been_large`, a second guard-the-guard.
* `TestTheMosaicPlacesAnOffsetTileCorrectly` — back-projection to the TRUE stage
  position, the no-clipping check, and
  `test_zero_offset_is_byte_identical_to_omitting_the_argument`.
* `TestTheCrosshairMarksTheStagePosition` — probes the painted pixmap.
* `TestSlotRestore`, `TestCropPersistsPerCameraIdentity`,
  `TestTheCropControlOnTheCameraPage`.

**21/21 mutations CAUGHT**, sources restored hash-identical:

| # | Mutation | Caught by |
|---|---|---|
| M1 | `crop.apply` is a no-op | 5 tests |
| M2 | `capture_fresh_frame` does not crop | mosaic-tile path |
| M3 | `capture_raw_average` does not crop | raw-still path |
| M4 | the display grab does not crop | cache + recording |
| M5 | `effective_um_per_px` divides by the delivered width | **the 27 % bug** |
| M6 | `_resolve_scale` divides by the delivered width | FOV + resolve |
| M7 | `capture_width_px` always returns the fallback | 3 tests |
| M8 | the crop is not centred | click→stage |
| M9 | `_true_capture_resolution` reads the cropped frame shape | stamp test |
| M10 | restore skips clearing a leftover crop | slot reassignment |
| M11 | `from_dict` crops on junk | tolerant-reader test |
| M12 | the move bound uses the delivered width | dialog test |
| M13 | the offset is ignored in `rect_for` | 6 tests |
| M14 | the crop is not clamped into the frame | achieved-shift test |
| M15 | `center_offset_px` uses the REQUESTED offset | the invariant |
| M16 | `pixel_to_stage_offset` does not compensate | 4 tests |
| M17 | `stage_offset_to_pixel` does not compensate | round-trip |
| M18 | the mosaic ignores the tile displacement | **back-projection** |
| M19 | `crop_offset_um` skips the flips | mosaic-vs-click agreement |
| M20 | the canvas is not widened for the displacement | back-projection |
| M21 | the store drops the placement | reload test |
| M22 | the crosshair stays at the geometric middle | painted-pixel probe |

⚠ **Three of my own tests were too weak and mutations caught them first** — all
three the same failure mode this repo keeps recording (asserting on the model
instead of the behaviour):

* **M9** survived because nothing pinned that the calibration stamp is the
  pre-crop size — the exact gap that would have quietly broken the round-1
  `sensor_width_um` plausibility guard. A test was added rather than the gap
  recorded.
* **M19** survived because `CameraManager.get_mirrored` **delegates to the
  widget**, so setting only the manager's `_mirrored` cache left `flip_x` False on
  the click path while the calibration saw True. Half the flip matrix was dead and
  the two paths could never differ. Now set via `set_mirrored`, with the reason in
  the test.
* **M22** survived because the crosshair test asserted that `reference_pixel`
  moved — which says nothing about whether `_draw_crosshair` *used* it. Rewritten
  to probe the rendered pixmap for the painted column.

**Regression**, per batch:

* camera stores / hardware controls / image correction / async open / objective
  calibration / round-1 v7.16 / unified mosaic — **190 OK**
* orientation audit / mount square-up / rotation cal / scale-FOV / needle-cam
  mount / objective ladder — **261 OK**
* capture UI / recording / frame averaging / post-move frame / fluor capture /
  ND3 export / Tucsen / Andor — **262 OK**
* mosaic orientation adjust + remap / memory-perf / fluor shift / fluorescence /
  plate frame / unreachable travel — **149 OK**
* `test_v75x_plate_mosaic` **class-by-class, 89 OK** (`TestManualAlignPage`
  excluded per this repo's documented hang precedent)
* `gui.app` import smoke + offscreen builds of the real `HardwareSetupPage`
  (4 crop rows) and `CalibrationPage`

⚠ **One combined-batch flake, disclosed:** the first run of the capture batch
reported `test_v79_tucsen_libra_camera::test_the_refusal_is_logged_once_not_per_read`
failing ("expected one notice for 20 reads, got 0"). It does not reproduce — the
same batch is green on both re-runs, no pair of those suites reproduces it, and
every suite passes alone. That test asserts on `assertLogs` capture of the TUCam
backend's logger, which is sensitive to whatever logging configuration another
suite left behind; this change touches neither logging config nor the TUCam
backend. Matches this repo's documented intermittent cross-suite camera-probe
interference.

---

## Issues & decisions

**Why the crop is at the source and the mirror is not.** The mirror moved *out*
of the frame source in v7.10 so the mosaic could re-blend against a changing
orientation. That reasoning does not transfer: re-blending under a changed crop
is meaningless, because the pixels are gone. What does transfer is the cost of
the alternative — four consumers each deciding independently how big a frame is.

**µm/px semantics, settled explicitly.** Three options were considered:
(a) store the crop fraction beside every resolution stamp and rescale properly;
(b) treat a crop change as invalidating the calibration and force a re-measure;
(c) express the stamp in capture pixels and divide by capture widths.
(b) was rejected because it is not true — cropping genuinely does not change
µm/px, so forcing a re-measure would be needless and would train the operator to
re-run calibrations that were already correct. (a) duplicates state into four
sites. (c) is what shipped: one helper, no new stored state, and existing stamps
(written when no crop existed) are already capture widths, so **no migration**.

**The tile count goes UP, and that is the honest cost.** At 4× on the Tucsen the
FOV goes 3305 × 2603 µm → 2603 × 2603 µm, so a full-plate scan needs roughly 27 %
more tiles. That is the price of not stitching dark bands, and the Microscope
Camera Setup FOV line now states the delivered field (`— cropped to 2048×2048 of
2600×2048 px`) so the operator sees where it comes from rather than wondering why
the estimate moved.

**A square crop also makes the rotated-tile geometry exact.** The round-1
`_oriented_fov_um` work computes a rotated bounding box; for a square tile at
±90° that box equals the tile, so on this camera's ≈ −90° mount the raster steps
match the footprint with no bounding-box slack.

**Not done, deliberately:** a non-square (e.g. circular or arbitrary-rectangle)
crop. The operator asked for a square, a square is what makes the ±90° geometry
exact, and the size % already covers the "circle smaller than the short side"
case. `CameraCrop.mode` is an enum with room for another mode if a real need
appears.

---

---

## Round 3 — "now python freezes when I apply the crop"

### What the operator's own log shows

`logs/app.log`, session of 2026-08-11 (their rig, Tucsen on slot 3, `tucam:0`):

```
09:46:05,161  Camera crop saved: … offset_x: 0.01   ← one auto-repeat step
09:46:05,333  Camera crop saved: … offset_x: 0.02
09:46:05,448  Camera crop saved: … offset_x: 0.03
  … 40 more, 1 % → 23 % → 9 %, ~150 ms apart …
09:49:06,585  Camera 3 crop set to off (full sensor)
09:49:48,450  TUCam released
09:49:48,451  Camera 3: camera stopped
09:49:48,452  TUCam released                        ← two releases, one stop
09:50:23,858  TUCam Api_Init ok — 0 camera(s)       ← the camera did not come back
09:50:23,861  TUCam: index 0 out of range (0 camera(s))
```

The `[Tick]` line prints every 200 ticks, which dates the bad window precisely:
313 ms/tick before 09:48:22, **608 ms/tick** from there to 09:50:24. The event
loop was running at half speed for about two minutes — and that window contains
the camera stop and the failed reopen, not the crop drag.

### What was measured, and what it ruled out

| Suspicion | Measurement | Verdict |
|---|---|---|
| The crop makes the frame path expensive | full display path at 2600×2048: **40.6 ms** uncropped, **35.7 ms** square, **17.4 ms** at 55 % | **cheaper** with the crop — not it |
| `cv2.cvtColor` on a non-contiguous crop view | 1.6 ms vs 1.4 ms contiguous | not it |
| The store write is slow | `set_crop` ×40 on the real file: p50 **0.6 ms** | not it on this machine |
| `heightForWidth` + `QScrollArea` scrollbar oscillation | real `CameraFeedView` in a real scroll area, aspect 1.27:1 → 1:1, five heights across the scrollbar threshold: **0 resize events** in every case | not it |
| Double-crop (a cropped frame cropped again) | `capture_fresh_frame` reads the backend directly | not possible |
| Crop restore creating a second camera | `CameraManager._widget` is a pure lookup | not it |

A freeze could not be reproduced offscreen.

### What was fixed

**The store write is debounced.** The offset spins auto-repeat, and the log shows
forty `set_crop` calls in seven seconds from one held arrow. Each is a full JSON
serialise plus a file write on the GUI thread. It measures 0.6 ms *here* — but
that cost is not bounded by anything this application controls (an on-access
virus scanner sits directly in that path), and it is the shape the
image-correction sliders already avoid by persisting on `sliderReleased`.

* the **live push stays immediate** — aiming a crop is a visual task, so every
  step must reach the frame source; only the write is deferred (400 ms idle)
* a **checkbox or the ⌖ button persists immediately** — one discrete decision,
  nothing to coalesce, and turning the crop OFF is what an operator does when
  something looks wrong
* `hideEvent` flushes on navigation, `aboutToQuit` on close, so a pending write
  cannot be lost — a crop that survives the session but not the restart is worse
  than one that was never applied

This is **hygiene, not a proven cause.** It is stated that way because the
measurements above do not support calling it the freeze.

**The next freeze will leave a stack trace.** `faulthandler` has been armed since
v7.5.x, but it catches a *crash* and is blind to a *hang*: the process is alive,
the event loop simply stops turning, and nothing is written anywhere. NEW
`SupportClasses/GuiWatchdog.py` stamps a heartbeat from a 250 ms QTimer on the
GUI thread and watches it from a daemon thread; five seconds of silence dumps
**every** thread's Python stack to `logs/freeze.log`, once per stall, appending
so earlier freezes survive.

Verified end-to-end against a deliberately wedged GUI thread: the dump named the
exact blocking frame and the recovery was logged with its duration.

It **observes only**. A watchdog that tried to break a thread out of a blocking
driver call would be far more dangerous than the hang — this process holds serial
ports open to a stage carrying a needle over glass. A test asserts that no
interrupt/kill primitive appears in the class at all.

### Still open, and honestly not attributed

The Tucsen did not come back: two `TUCam released` lines for one stop, then
`Api_Init ok — 0 camera(s)`. `release()` is idempotent per backend object
(`_cleanup_api` is guarded by `_opened_api`), so two releases mean **two backend
objects** each holding an API reference — consistent with a second `open` having
happened, e.g. an async open superseded by a manual Start. That is in the TUCam
API lifecycle, not in the crop: this change touches no part of camera
start/stop/release (`git diff` on `camera_widget.py` shows no edit to any of
them), and the crop path only ever assigns a dataclass to a widget attribute.

Recorded rather than fixed on a guess. **If the freeze recurs, `logs/freeze.log`
will now say where it is.**

---

## Round 3b — the watchdog answered it the same morning

The operator restarted, reported *"now the tucsen camera doesnt load or start"*,
and `logs/freeze.log` already held the answer. Three stalls captured, the last
naming the frame directly:

```
GUI THREAD STALL — 2026-08-11 10:22:17 — idle 5.5s
  tucam_backend.py, line 433 in _api_init
  tucam_backend.py, line 531 in enumerate
  camera_widget.py, line 187 in detect_tucam_cameras
  camera_manager.py, line 224 in detect_cameras
  hardware_setup.py, line 5308 in _on_detect_live_cameras
  main.py, line 273 in run_gui
```

with the matching log window:

```
10:21:27,925  TUCam library loaded
10:21:31,687  GUI thread has not run for 5.9s — dumping all thread stacks
10:21:57,058  TUCam Api_Init ok — 0 camera(s)          ← 29.1 s inside one call
10:21:57,835  GUI thread recovered after ~31.1s
```

**`TUCAM_Api_Init` blocks for ~30 seconds on the GUI thread when the camera is
present but not claimable, then answers "0 cameras".** Both operator reports are
that one fact: the freeze IS camera detection, and the camera "not starting" is
the same call returning nothing. It runs at startup *and* on every press of
Detect, so the application froze twice in the first minute of the session.

Confirmed against the rig's actual state: `Get-PnpDevice` reports the Libra 25
`Status: OK, Problem: CM_PROB_NONE` — healthy driver, healthy USB — while a
fresh out-of-app `TUCamBackend.open("0")` still gets `0 camera(s)`. The device
is wedged below the SDK, consistent with the unclean teardown recorded above
(two `TUCam released` for one stop). Recovery is a USB re-enumeration:
unplug/replug, or elevated
`Disable-PnpDevice`/`Enable-PnpDevice` on `USB\VID_5453&PID_E437\...`.

### The fix

`CameraManager._probe_tucam_bounded` — the enumeration runs on a daemon thread
and is waited on for `TUCAM_PROBE_TIMEOUT_S` (4 s). The GUI can no longer be
held hostage by a blocking C call with no cancellation.

Two details that make it safe rather than merely fast:

* **a late answer is kept, not discarded** — it lands in `_tucam_cache` and the
  next detection uses it, so a merely SLOW SDK costs one round, not the camera
* the thread is a **daemon** — it may still be inside a 30-second C call at
  shutdown, and a non-daemon thread would hold the process open for it

The warning names the state and the remedy instead of leaving a silent freeze:
*"Tucsen enumeration did not answer in 4s — the camera is present to Windows but
the SDK cannot claim it … unplug and replug the camera to re-enumerate it."*

**Measured on the wedged rig:** `detect_cameras()` now returns in **6.8 s** (4 s
of it the bounded Tucsen wait) instead of ~30 s, and still finds both Teslongs
and the simulator.

⚠ **Not caused by the crop, and not fixed by removing it.** `_api_init` /
`enumerate` / `detect_cameras` are untouched by the crop work; what the crop
work added was the watchdog that found this. The freeze predates it and would
have recurred on any startup with the camera in this state.

**Also cleared:** two `python -m unittest` processes hung since the previous
evening (the repo's documented cross-suite camera-probe hang) were still holding
resources and were killed. They were not the blocker — the fresh probe still
saw 0 cameras — but they are exactly the kind of stale claim that makes a camera
un-openable, and worth checking first next time.

### Round 3c — the numbers that settle what "wedged" means

Every `TUCAM_Api_Init` in the operator's log, timed against the "library loaded"
line beside it:

| When | Duration | Result |
|---|---|---|
| 2026-08-10 20:02 | **2.07 s** | 1 camera |
| 2026-08-11 08:51 | **2.11 s** | 1 camera |
| 2026-08-11 09:39 | **2.07 s** | 1 camera |
| 2026-08-11 10:21 | **29.13 s** | 0 cameras |
| 2026-08-11 10:34 | **29.13 s** | 0 cameras |
| out-of-app probe, nothing else running | **29.2 s** | 0 cameras |

A healthy init answers in ~2 s. 29 s IS the failure — the SDK scanning for a
device that will not answer. That fixes `TUCAM_PROBE_TIMEOUT_S = 4.0 s` as an
evidence-based choice: ~2× headroom over healthy, and 7× shorter than wedged.

**Three things this rules out.**

*The crop.* The camera ran normally for **three minutes** after the crop was
applied (09:46:22 → 09:49:06, nothing in the log but ticks). It broke on a
Stop → Start, at 09:49:48.

*The double release.* An identical pair of `TUCam released` lines at 08:54:15 was
followed 2.0 s later by `Api_Init ok — 1 camera(s)`. Two releases for one stop
is untidy but is not the trigger.

*Anything holding the device.* Probed with **no MEBP process running at all**
(and the two hung test processes killed): still 29.2 s, still 0 cameras.

What is left is an intermittent Tucsen stop/start fragility: sometimes the
device does not come back, and once it is in that state only a USB
re-enumeration recovers it. `DEVPKEY_Device_LastArrivalDate` on the rig still
read **8/10 12:43 PM** — i.e. the device had not re-enumerated once since before
it broke, so the recommended unplug/replug had not yet actually happened.

### The recovery tool

NEW **`tools_reset_tucsen_camera.ps1`** — the software equivalent of unplugging
the camera, following the convention of the two existing install scripts:

* refuses politely (with the exact command to re-run) unless elevated — PnP
  device control requires it and UAC cannot be answered non-interactively
* **refuses while MEBP is running**, because a reset underneath a process that
  still holds `TUCam.dll` is wasted
* finds the device by VID rather than a hard-coded instance id
* prints `LastArrivalDate` before and after, so the operator can SEE that the
  device really re-enumerated
* then asks **the SDK** how many cameras it sees — Windows reporting `Status:
  OK` is exactly what it reported throughout the fault, so the OS is not a
  trustworthy oracle here

Pure ASCII on purpose (PowerShell 5.1 reads a BOM-less `.ps1` as ANSI; a UTF-8
dash breaks the parse — recorded in the v7.9 Tucsen work after it cost a
session).

---

## Needs GUI/HW verification on ME3B V1, IN ORDER

1. Hardware Setup → Cameras: tick **⬛ Square crop** on the Tucsen slot. The live
   preview should immediately lose the dark side bands, and the readout should
   name the delivered box and its position.
2. If the dark arc sits on ONE side, nudge **Offset X / Y** until the lit circle
   is centred in the picture; if the corners are dark all round, dial the size %
   down instead. **⌖** re-centres.
3. Watch the crosshair move with the offset — it marks where the stage is
   pointing, and it must stay on the same physical feature as you nudge.
4. **The offset must not move anything:** put the needle on a feature, note the
   XY, then change the offset by 5 % and confirm a click on that same feature
   still drives to the same XY (this is the compensation; a shift here is a bug).
5. **Confirm µm/px did NOT change** — the Microscope Camera Setup µm/px line
   should be identical before and after ticking the box; only the *Field of view*
   line should change (and should say it was cropped).
6. Restart. The crop must come back on, still on the Tucsen and only the Tucsen.
7. Take a still (display) and a **raw** still — both must be square. Record a
   short video — it must be square too. This is the "all surfaces" check, and the
   raw still is the one that would have been missed.
8. Run a small mosaic. Expect **more tiles than before** and **no dark seams**;
   the stitch should be visibly cleaner where tiles overlap.
9. Click a feature in the live view and drive to it — the needle must land on it.
   A constant offset here would mean the crop is not centred.
10. Re-run Mosaic & Camera Calibration with the crop on and confirm the stored
   resolution stamp still reads **2600 × 2048** (the sensor mode), not
   2048 × 2048.

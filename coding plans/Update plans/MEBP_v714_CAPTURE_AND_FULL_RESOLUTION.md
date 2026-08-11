# MEBP v7.14 — Image/video capture + full-resolution mosaics

## Objective

Two operator asks that share one capability:

1. *"For the mosaic, or any other situation where we will want max resolution, we could
   feasibly take a single image at 2048×2048 at a time where the live view is still lower.
   So we just time the change in resolution with a capture event."*
2. *"We should also make a capture microscope image button at the bottom of the page. We
   should also make a record video button."* — with settings for **raw or display** (stills
   *and* video), **user-specified frame rate and quality**, a **destination picker** and a
   **name picker with optional metadata (date, channel, objective "10x NA 0.3")**, and
   *"if we can save this information in the image meta data that would be great."*

Before this, there was no usable operator capture (`CameraWidget.take_snapshot` reads
`self._capture` directly, so it is OpenCV-only and silently does nothing on the Zyla or
Libra) and **no camera video recording at all** — the "walkthrough recorder" saves GUI
screenshots for tutorials. Every mosaic also captured at whatever the live preview
resolution happened to be.

**Operator decisions (AskUserQuestion + follow-ups):** buttons on **every live feed** ·
raw-vs-display **selectable in settings** for both stills and video · **frame rate and
quality user-specified** · **destination + name picker with metadata tokens**, embedded in
the file where possible.

---

## Verified environment facts (measured this session — do not re-derive)

- **Pillow 12.3.0** — `PngImagePlugin.PngInfo` for PNG `tEXt`; TIFF `ImageDescription`
  (**ImageJ reads it**); uint16 saves natively as mode `I;16`.
- **cv2 5.0.0** — `mp4v` / `XVID` / `MJPG` writers open. ⚠ **`avc1` reports
  `isOpened() == True` while the OpenH264 DLL fails to load**, so it writes a file that
  never plays. Excluded from the fourcc chain **by name, with a test**, because it looks
  like the obvious quality upgrade.
- `CameraManager.set_capture_resolution` emits no signal and writes no config/store — safe
  for a momentary switch. On the Zyla it stops the stream, joins the reader, fails pending
  averaged captures, re-ROIs, re-syncs frame rate and restarts (~hundreds of ms).
- **Mosaic canvas costs 35 bytes/pixel** (float64 composite + float64 weights + uint8
  cache, from `MosaicBuilder._init_composite`): 3000 px ≈ **315 MB**, 6000 px ≈ **1.26 GB**.
- 🐞 **`capture_fresh_frame` had no `tucam` branch** — `_read_once` fell through to
  `self._capture`, which is None on a Tucsen, so it returned None on the Libra and every
  caller (mosaic tiles, calibration grabs, the new capture button) silently got nothing.
  Pre-existing; fixed here.

---

## Part A — "Scan at full camera resolution"

### The honest economics (drives the design)

Binning changes pixel count, **not field of view**, so a full-resolution scan visits the
**same tiles** — it costs ~4× per-tile transfer, not 4× tiles. But the deliverable is the
stitched **canvas**, and `target_px` (default 3000) is what tiles are resized into:

| Scan | Canvas µm/px @ 3000 px | Camera µm/px @10× |
|---|---|---|
| Single well (~15 mm) | ~5.0 | 1.30 binned / **0.65 full** |
| Full plate (~110 mm) | ~37 | 1.30 binned |

**Full resolution only pays off when the canvas is raised with it**, and on a full-plate
scan the canvas is already ~28× coarser than the *binned* camera — 2048 buys essentially
nothing there. So the toggle raises the canvas with it (capped at 4500 px ≈ 709 MB) and the
dialog states the memory live, rather than silently quadrupling RAM.

### Implementation

- **NEW `SupportClasses/CaptureResolution.py`** (pure, duck-typed manager; shared with the
  capture button): `max_resolution` · `switch_to_max` → the previous resolution to restore
  · `restore` (never raises) · `canvas_px_for_full_res` · `estimated_canvas_mb` ·
  `describe_switch`. `switch_to_max` **reads back** and returns None when the device did
  not actually move — an accepted-but-ignored switch must not leave the caller believing it
  is at full resolution.
- `MosaicCalibration.SCAN_DEFAULTS` + dataclass + `_scan_params` gain `full_res_scan`
  (default False). A *scan* parameter — `build_mosaic_builder` is untouched.
- `mosaic_settings_dialog`: checkbox in Camera timing + a **live canvas/MB note** that
  warns past 700 MB.
- **Plate / rosette / single-well** (`calibration.py`): all three route through
  `_ploc_start_mosaic_scan_impl`, so ONE hook — placed after the camera-start block and
  **before the frame grab whose `fw/fh` size the whole calibration**. Restore lives in
  `_ploc_mosaic_cleanup_ui()`, the chokepoint all five exits already use (objective
  declined / tiles declined / finished / failed / cancelled), **plus** a new call in
  `_shutdown_mosaic_worker` — the one exit it missed.
- **Fluorescence**: switched **after** the tile-count confirm (the tile count is identical
  at either resolution, so the number the operator approved stays true, and a declined
  dialog never leaves the camera switched), then the raster plan is re-derived because the
  frame size and µm/px both change while their product, the FOV, does not. Restore is
  **paired with `_restore_entry_exposure`** — both are run-scoped camera state snapshotted
  at start, so all four of its exits are covered without four new call sites.
- **Documented hazard:** if the camera is (re)started mid-scan,
  `hardware_setup._maybe_apply_resolution_to_device` drives it back to `active_resolution`.
  Logged at switch time, not assumed away.

### 🐞🐞 A2 — the post-move frame guarantee was NOT real, and full resolution is exactly what breaks it

Operator question that found it: *"we need to ensure the camera has grabbed the
frame before the xy stage moves to the next coordinate. how do we ensure that
the image doesnt come out blurry"*.

**Ordering was never the problem.** The scan loop is one worker thread and the
grab call blocks, so the next `move_xy_absolute_um` structurally cannot be
issued until the frame is in hand. No test is written for that; it would only
restate the control flow.

**The freshness guarantee was the problem, and it was illusory.**
`CameraWidget._frame_seq` was incremented by the **display timer** on every
tick, while the SDK backends' `read()` is documented *"Non-blocking after the
first frame"* — it returns the **same cached frame** as often as it is asked.
So `frame_count_value()` counted timer ticks, not sensor frames, and the
workers' "wait for 3 fresh frames" measured nothing about the camera.

The timer is fixed at **15 fps**. While the camera outran it every tick really
was a new frame and the guard worked *by luck*. At **2048×2048 with a 200 ms
exposure the Zyla delivers ~5 fps**: the timer grabs the same frame three times
in 200 ms, the wait is satisfied, and the tile stitched is the one **exposed
during the stage move**. Mutation-confirmed: 10 ticks with zero new frames
advanced the counter by 10.

A second, independent failure: `fresh_timeout_s` was a flat **2.5 s**. Three
frames at a 1 s exposure take >3 s, so the wait expired — and the old code then
fell through to `get_current_frame()` **unconditionally**, stitching a stale
tile at the new canvas position. That is the same "duplicate image at the wrong
spot" corruption the arrival check already guards against, except silent.

**Fixes** (all three mutation-verified):

1. **Count sensor frames.** `frames_acquired()` on all three SDK backends,
   incremented at the *single* place a new frame lands — pinned by an **AST**
   test that the increment sits in the same block that assigns `self._frame`,
   so a new reader path cannot reintroduce the blur. `_frame_seq` now advances
   only when that counter moved.
   ⚠ **The gate lives in `_grab_frame`, not in `frame_count_value()`** — reading
   the backend counter directly would let it advance while the display timer is
   stopped (a hidden page), telling a worker "new frame" while `_current_frame`
   still held the old one. The counter and the buffer must move together.
   OpenCV/simulated keep the unconditional increment: `read()` genuinely
   advances the FIFO there, so every grab *is* a new frame.
2. **NEW pure `SupportClasses/CaptureTiming.py`** — timeout sized from the
   camera's own frame period (`frame_rate` if reported, else `exposure_us` as a
   *lower* bound plus readout and margin). It may only ever **lengthen** the
   wait, so no rig that works today can start dropping tiles. `MIN_FRESH_FRAMES
   = 2`, because the first new frame after the settle may have *started*
   integrating before the stage stopped; only the one after it is guaranteed
   clean. Out-of-band values are rejected rather than used to size a nonsense
   timeout.
3. **A timeout drops the tile.** Both workers return `None`, which the callers
   already handle (warn, count, abort after 8 in a row). In fluorescence the
   refusal is placed **before** `capture_raw_average`, not after — an averaged
   stack begun mid-move is smeared in every frame.

**Consequence handled, not left:** `camera_rotation_align_dialog`'s
`FEED_WATCHDOG_S = 1.5` reported "feed stopped — restart it" purely from this
counter. Now that it tracks sensor frames, a legitimate 2 s exposure would
accuse a healthy camera, so the threshold scales to `max(1.5 s, 3 × frame
period)`.

**The `settle_ms` knob is unchanged and still needed** — it is the *mechanical*
settle (stage vibration). Frame counting handles the *exposure* straddle. They
are different problems.

### A3 — average N frames per tile, *when they agree*

Operator: *"we should average the three frames it grabs to get an even better
image as long as the three frames look like each other."*

⚠ **Correcting the premise it was built on:** the scan did **not** grab three
frames and keep them. It waited for three to *arrive* and read **one** — a
freshness gate, the rest discarded. There is also no "trigger to grab": the
camera **free-runs** continuously and the display timer samples it; what starts
after arrival is the *counting*. That matters here, because **the first new
frame after the settle may have straddled the move** (A2), so averaging *those*
three would fold the smeared one straight back in. The frames averaged are the
ones **after** the gate.

**Averaging was already implemented — and already wrong.** `avg_frames` shipped
in the shared scan settings, but only the fluorescence page consumed it, and it
averaged **blindly**; the plate / rosette / single-well scans ignored the
setting entirely and always took one frame. So this adds the guard the operator
asked for and extends averaging to the other three scans.

**🐞 The naive similarity test does not work, and that is measured, not
asserted.** Consecutive frames never look identical — shot noise guarantees a
difference, and at low signal the difference is *entirely* noise, which is
exactly what averaging removes. A normalised-difference metric was built first
and inverts: on a dim, low-contrast scene **noise alone scored 0.0707** while a
real 3 px shift on a bright scene scored **0.0236**. No fixed threshold can
separate those. Pinned by `test_absolute_difference_cannot_work`, which fails
if the naive form ever stops inverting — so the scale-free statistic cannot be
"simplified away" without re-deriving it.

**What works:** noise is spatially uncorrelated, structure is not. Block-average
the difference by `B`; noise falls as `1/B`, structure does not fall at all, so
`ratio = B · mean|blockmean(Δ)| / mean|Δ|` is ~1.0 for pure noise and rises
toward `B` as the difference becomes structural — **independent of signal level
and contrast**, which is what the first attempt lacked.

Measured over brightness 200–20000 counts × contrast 0.3–1.0 × sizes
64² – 2048², two independent runs:

| case | ratio |
|---|---|
| noise only | **1.00 – 1.05** (2048²: 1.004 – 1.015) |
| 1 px shift | 1.6 – 4.8 |
| 3 px shift | 2.8 – 7.5 |
| 10 % brightness step | 3.5 – 8.0 |
| three different scenes | = `B` (the theoretical max) |
| flat featureless field | 1.00 either way |

Hence **threshold 1.5** — a 1.43× margin over the worst noise-only case while
still catching a single-pixel shift. **The block size is adaptive**
(`min_dim // 32`, a constant ~32×32 grid) because the statistic's *spread*
depends on how many blocks it averages: a **fixed** block size measured **1.44
at 128²**, dangerously near the threshold, so real scans would have randomly
refused to average.

**Two honest limits, both benign and documented in the module:** on a dim,
low-contrast scene a 1 px shift scores ~1.05 and is missed — it is also barely
present, so the blur it adds is small next to the noise removed; and on a flat
featureless field motion is undetectable *and* harmless.

**On disagreement the tile falls back to the FIRST frame, it is not dropped.**
A single post-move frame is exactly what the scan produced before averaging
existed, so the fallback can never be worse than the previous behaviour —
whereas refusing the tile would make scans fail where they currently succeed.

The guard also covers a hazard nobody asked about: the Andor's **per-frame
display auto-scale**. Averaging frames that were each mapped differently is
meaningless, and it registers as a brightness step (3.5–8.0), so it is refused
rather than silently producing a wrong tile.

---

## Part B — Capture subsystem

### Pure core (`SupportClasses/`, no Qt — pinned by a structural test)

| Module | Responsibility |
|---|---|
| `CaptureSpec.py` | `CAPTURE_DEFAULTS`, `merged_settings`, `resolve_output_dir`, filename templating, **collision-safe `open_unique`** |
| `CaptureOrientation.py` | `orient_array` — the numpy twin of the display transform |
| `CaptureMetadata.py` | `collect` / `to_tokens` / `to_text_pairs` / `to_json` / `to_imagej_description` |
| `CaptureImageWriter.py` | PNG tEXt+iTXt · TIFF ImageDescription + resolution tags · sidecar |
| `CaptureVideoWriter.py` | verified fourcc chain · `plan_frame_repeats` · raw time-lapse |
| `CaptureContext.py` | process-wide `Settings`/controller registry + shutdown finalizer |

Decisions worth keeping:

- **The frame rate is the PLAYBACK rate.** `plan_frame_repeats` writes extra copies when the
  camera runs slower than the requested rate and drops frames when it runs faster, so a
  minute of recording is a minute of video. Without it a 4 fps camera in a 15 fps file
  plays at near-4× slow motion while the operator believes they recorded real time. The
  dialog says so in words.
- **`open_unique` uses `O_CREAT | O_EXCL`** and returns *the path it actually got*. The
  sanitiser is deliberately many-to-one ("10x NA 0.3" and "10x_NA_0.3" collapse), so two
  distinct captures can render one stem — they become `…_002`, never an overwrite, and
  there is no exists()-then-write window. Everything downstream (sidecar, embedded
  `filename`, log line) uses the returned path, never a re-rendered template.
- **PNG `tEXt` is latin-1 only.** Every µm/px note contains `µ`; non-latin-1 values go
  through `iTXt` (UTF-8). This repo already has a cp1252 encoding scar.
- **Raw 16-bit requires TIFF.** Pillow's 16-bit PNG byte order is surprising and ImageJ
  reads 16-bit TIFF natively, so the combination is prevented in the dialog **and** refused
  by the writer rather than silently rewritten.
- **TIFF carries the resolution tags** (pixels-per-cm from µm/px, unit 3) so **ImageJ's Set
  Scale is automatic** — the operator's real downstream workflow. Passed as a single float;
  a pair makes Pillow write a malformed two-entry tag that warns on read.
- **A JSON sidecar is always written** (`foo.png.json`, not `foo.json`, so a PNG and a TIFF
  captured in the same second do not fight): greppable, survives re-saving in ImageJ, and
  for video it is the only metadata path.
- **µm/px is stamped at the CAPTURED width**, not the live width — a full-resolution still
  has a different scale, and a wrong scale bar is worse than none.
- **A raw request that cannot be served is REFUSED**, never quietly given a display frame:
  an auto-scaled 8-bit frame is not the same measurement, and a file that looks right and
  is not quantitative is the worse outcome.
- **Raw video ships honestly as time-lapse.** `capture_raw_average` is a one-shot request
  serviced by the backend's reader thread; looping it captures some frames and misses the
  rest. `RawFrameSequenceWriter` writes numbered 16-bit TIFFs + a manifest with **real
  per-frame timestamps**, and nothing claims the sequence is continuous. (A true raw tap
  would put a new consumer on the SDK reader thread that services the live feed — a
  bench-verification problem, deliberately not done here.)
- **Zero-frame / zero-byte finalize DELETES the file** and reports; leaving a 0-byte `.mp4`
  that looks like a recording is worse than the failure.

### Qt side

- `gui/widgets/capture_controller.py` — one orchestrator so ~18 feeds do not each
  reimplement it. Blocking work (raw capture, stream restart, encoding) on daemon threads,
  results back through queued signals. Recording ingress connects to the **camera's**
  `frame_captured`, not the view's `_on_frame` (which early-returns when hidden), so a
  recording survives the operator navigating away.
- `gui/dialogs/capture_settings_dialog.py` — destination + Browse, filename templates with
  a **live rendered preview** and a token legend, still source/format/averaging/full-res,
  video source/fps/quality/container/limits with a **live MB-per-minute estimate**,
  metadata embed + sidecar + operator/notes. **Nothing persists until OK.**
- `CameraFeedView` — `enable_capture` ctor flag (default True); 📷 and ⏺ overlay buttons in
  the existing gear style; `_position_settings_btn` generalised to lay the visible buttons
  out right-to-left. **The gear is first in that order, so a feed without capture buttons
  puts it exactly where it has always been** (pinned by a test). Recording shows a red
  `⏹ 0:14` pill that re-flows the row each second; a saved capture flashes its filename.
  Right-click either button for the settings dialog.
- `app.py` registers the capture context; `CameraManager.shutdown` finalizes live
  recordings **before** stopping cameras (an AST test pins that order).

---

## Files modified

`SupportClasses/`: **NEW** CaptureResolution · CaptureSpec · CaptureOrientation ·
CaptureMetadata · CaptureImageWriter · CaptureVideoWriter · CaptureContext; edited
MosaicCalibration.
`gui/`: **NEW** widgets/capture_controller.py · dialogs/capture_settings_dialog.py; edited
widgets/camera_feed_view.py · widgets/camera_widget.py (tucam gap) ·
widgets/camera_manager.py (shutdown finalize) · dialogs/mosaic_settings_dialog.py ·
pages/calibration.py · pages/workflows/fluorescence_mosaic_workflow.py · app.py.

## Testing

**NEW `test_v714_full_res_mosaic.py` (29)** — switch/restore/no-op/refused/deaf-camera,
canvas math and the square-law memory, settings plumbing, dialog round-trip + warning, and
that **every** scan exit restores (cleanup chokepoint, shutdown, idempotence, stale-canvas
clearing) on both pages.
**NEW `test_v714_capture_core.py` (47)** — templating and the many-to-one collision, a
concurrent `open_unique` race, `bool("false")`, env override; **orientation parity against
the real `_orient_qimage` across all 16 cardinal combinations**, with a guard proving the
corner signature distinguishes all 8 dihedral results; metadata from failing sources,
µm/px at the captured width; PNG/TIFF round-trip, `µ` survival, bit-exact 16-bit, ImageJ
resolution tags, sidecar naming; pacing table, verified open, avc1 exclusion, size-lock,
zero-frame deletion, manifest timestamps.
**NEW `test_v714_capture_ui.py` (28)** — button presence/opt-out/visibility, **gear
position unchanged**, no overlap, record pill reflow; display and raw capture end-to-end,
busy guard, **raw-unavailable refusal writing nothing**, full-res switch **restoring even
when the grab raises**, context provider into filename + metadata, orientation applied;
recording start/stop, non-blocking ingress, shutdown finalize + AST order; dialog
round-trip, raw-forces-TIFF, preview, unknown tokens, Cancel persists nothing.
**NEW `test_v714_post_move_frame.py` (34)** — the freshness guarantee (A2). Pure
timing math (frame-rate precedence, exposure as a lower bound, out-of-band rejection,
never-shorter-than-configured, cap, two-frame floor, graceful degradation); the counter
driven through the **production `CameraWidget._grab_frame`** with a backend shaped like
the real ones (non-blocking `read()` + a separate sensor counter) — repeated grabs of one
frame must not advance, the counter must stay **paired** with `get_current_frame()`, a
backend without the accessor must not freeze at zero; an **AST** check that every backend
increments in the same block that stores `_frame`; both workers dropping the tile on
timeout, fluorescence refusing **before** the averaged read; and the feed watchdog
following the exposure.

**NEW `test_v714_frame_averaging.py` (28)** — A3. The statistic driven over the same
range it was designed on (noise floor at every size with **8 draws each**, 1/3/8 px
shifts, brightness step, different scenes pegging at `B`); degenerate inputs (flat field,
identical buffers, shape change, colour); `frames_agree` comparing against the **first**
so a slow drift cannot pass as three small steps; √N noise reduction verified against the
noise-free scene; uint8 accumulator overflow; the raw-path guard refusing with a reason
and still failing shape-change first; the plate worker averaging, falling back on motion,
`avg_frames=1` byte-identical to the old single-frame path, and an **AST** check that the
setting is actually forwarded (otherwise the dialog control is inert).

**4/4 mutations CAUGHT** for A3: guard always agrees (7 failures) · **fixed block size
instead of adaptive** · the `avg_frames` setting not forwarded · the raw path returning
the mean regardless of agreement.

⚠ **Two of my own tests were too weak and were caught by their own subject:**
`test_averaging_combines_frames` used three *uniform* frames at 100/104/108 — a constant
offset is pure structure, so the guard correctly refused and the test failed; the fixture
now differs only by spatially-uncorrelated noise, as real frames do.
`test_noise_floor_is_stable_at_every_frame_size` used a **single seed** and passed even
with the fixed-block mutation it exists to catch — one draw is not evidence about a
distribution, so it now takes the worst of 8 and fails at 1.411.

**3/3 mutations CAUGHT** for A2, each a real source edit reverted afterwards:
count timer ticks instead of sensor frames (`11 != 1` — the bug reproduced exactly) ·
flat timeout ignoring the frame period (`2.5 not greater than 2.5`) ·
stitch whatever is buffered on timeout (a stale array returned where `None` is required).

**Two real bugs the new tests caught in my own code:** the rotation direction was
**backwards** (Qt renders clockwise; the parity test found it), and a **failed raw capture
skipped the resolution restore** because the early return sat outside the `try/finally`.

**3 mutations confirmed CAUGHT**: reverse `np.rot90` sign · drop `O_EXCL` · remove the
restore from the mosaic cleanup chokepoint.

**Regression** per-suite green (A2 pass): v713+v714 (196) · tucsen + andor-display (100) ·
mosaic worker + settings + unreachable + fluorescence (55) · camera-mount-square-up (64) ·
camera liveview/hw-controls/image-correction/async-open (59) · orientation-audit +
unified-mosaic + orientation-adjust (106) · picker-scaling + survey-tab (61) ·
`gui.app` import smoke.

**Regression** per-suite green: v713 camera (121) · v714 (104) · fluorescence + tucsen (31)
· unified-mosaic + camera-hw-controls + hygiene (69) · plate-mosaic **110, 1 failure = the
documented pre-existing `test_real_24_well_mosaic` blob-detector CV failure (23 vs 24)**,
`TestManualAlignPage` excluded per its documented hang · `gui.app` import smoke.

**Two legitimate test updates:** `test_v713_fluor_capture`'s exposure stub now models the
paired resolution restore; `test_v75x_plate_mosaic::test_dialog_round_trip_and_defaults`
built its `custom` dict by hand and so broke on every new setting (v7.13's `avg_frames` had
already broken it) — now built from the defaults with every value overridden, and asserting
that it exercises them all.

**⚠ Not from this work:** `test_v75x_fluorescence_mosaic::test_set_raster_grid_paints` fails
on `well.x` inside `jog_well_plate._well_layout` → `plate_layout.wells_from_plate`. That
call is a `+` line in **another session's uncommitted v7.12 plate work** (HEAD's
`jog_well_plate.py` contains no such call); neither file is touched by this change.

---

## Bench verification (ME3B V1) — IN ORDER

0b. **A3 — averaging.** Set *Average frames per tile* = 3 and re-run the same well.
   Tiles should be visibly cleaner (√3 ≈ 1.7× less noise) and the scan ~2 frame-times
   per tile slower. Then deliberately provoke a refusal — nudge the bench, or drop the
   settle to 0 ms — and confirm the log says *"averaging skipped for this tile"* with a
   structure ratio, and that the tile is **sharp, not blurred**. On the Zyla also check
   averaging works with display auto-scale **off**; with it on, every tile should refuse
   (each frame is mapped differently — that refusal is correct).
0. **A2 first — the tiles must be sharp before anything else is trusted.** Set a long
   exposure (≥500 ms) at 2048×2048 and run a small single-well scan. Every tile must be
   crisp; a smeared tile means a frame was exposed during the move. Then watch the log:
   dropped tiles say so by name (*"no new camera frame within N s"*) instead of appearing
   as a blurred stitch. Sanity-check the *rate*: at a long exposure the scan is slower
   per tile than at preview — that is the fix working, not a stall.
1. **Capture a still on the Zyla** with the default display settings: the file lands in
   `captures/<date>/`, the toast names it, and the image matches the screen orientation.
2. **Open that file in ImageJ** and confirm the embedded objective/exposure/stage metadata
   (Image ▸ Show Info) — and for a TIFF, that **Set Scale is already populated** from µm/px.
3. **Capture a raw 16-bit TIFF** (settings ▸ Images ▸ Raw) and confirm ImageJ reads it as
   16-bit with the counts intact; then tick **full sensor resolution** and confirm the feed
   blips once and comes back at the preview resolution.
4. **Capture on the Libra** — this is the `capture_fresh_frame` tucam fix; before it, the
   button would have silently produced nothing.
5. **Record ~30 s**, stop, and play the file **outside the app**. Check the duration matches
   the wall clock (that is `plan_frame_repeats` doing its job) and the toast's frame count
   looks sane. Repeat with a long exposure so the camera runs well under the target rate.
6. **Navigate to another page mid-recording** and back — recording must continue.
7. **Close the app mid-recording** — the file must be finalized and playable, not truncated.
8. **Full-resolution mosaic**: scan one well with the toggle ON and the canvas note read
   first, then the same well binned; compare sharpness and confirm the RAM figure the
   dialog predicted. Confirm the live preview resolution is back where it started.
9. **Full-plate scan with the toggle ON** — expect it to be slower with little visible gain
   (the canvas is the limit); this is the case where the toggle is not worth it.
10. Cancel a scan mid-run and confirm the preview resolution is restored.

## Deferred / flagged

- **True raw video** (a frame sink on the backend reader thread) — deliberately not done;
  it puts a new consumer on the thread that services the live feed.
- **`still_full_res` defaults OFF** — it interrupts the stream on every press.
- **Overlay buttons are anchored to the label, not the pixmap**, so on a letterboxed feed
  they float over the dark bar. That is the existing gear behaviour; changing it moves the
  gear and belongs in its own change.
- `take_snapshot`'s OpenCV-only path is left in place (its button still works on a webcam);
  redirecting it to the new controller is a small follow-up.

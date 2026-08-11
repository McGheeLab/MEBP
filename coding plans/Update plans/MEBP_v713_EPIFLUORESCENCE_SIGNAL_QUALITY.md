# MEBP v7.13 — Epifluorescence Signal Quality (Zyla sensor features · raw stats · fluorescence AF + averaging)

## Objective

Boost epifluorescence image quality through software: (1) wire the Andor Zyla's unwired
low-noise SDK3 features with sane defaults + dialog controls; (2) retain raw 16-bit frame
statistics so exposure can be pushed without clipping (histogram, % clipped, live SATURATED
badge); (3) upgrade the fluorescence mosaic — per-channel exposure, raw-frame averaging per
tile with frozen per-channel display levels, per-tile autofocus (checkerboard sweeps +
plane-prediction on every tile) producing a per-well **critical sample surface** artifact
(NOT the plate bottom — cells often sit above the well bottom, e.g. on hydrogel); (4)
optional denoise + background subtraction, non-destructive; (5) the Tucsen (Libra) gets the
identical raw-stats/averaging treatment; (6) a retroactive post-processing pass over
already-captured mosaics.

Operator decisions (2026-08-07):
- Zyla SDK tier: best defaults + dialog controls. Saturation tier: histogram + readout + badge.
- Fluorescence tier: per-channel exposure, frame averaging, per-tile AF. **Correction:** the
  AF map is the SAMPLE SURFACE, never installed as the plate-bottom datum (v7.11 wizard owns
  that); AF sweeps hit a **checkerboard lattice**; Z step/range user-assigned (0 = DOF-auto);
  post-scan surface model plane / linear / spline with confidence highest inside the well
  boundary; **cell targeting consumes the surface now** (per-target removal Z, gated).
- Post-processing: denoise + background subtraction, off by default. Deconvolution and
  brightfield-LED auto-off explicitly out of scope.
- Bench feedback (same day): *"the signal is so much better"* → follow-ups: software
  gain/brightness/contrast readable; Tucsen parity; retroactive denoise. All landed.

## Files Modified

| File | Rationale |
|---|---|
| `gui/widgets/mono_display.py` | `compute_raw_frame_stats` (shared Andor/Tucsen seam), shared `RawAverageRequest`, public aliases |
| `gui/widgets/andor_backend.py` | `ANDOR_SENSOR_FEATURES` table + probe/defaults/get/set (token-matched enums, stop→apply→restart live fallback), BitDepth-aware clip level, reader-loop raw stats + temperature, `capture_raw_average` |
| `gui/widgets/tucam_backend.py` | same treatment: `_service_raw_plane` (ucDepth-aware clip), `get_raw_frame_stats`, `capture_raw_average`, pending-average fail on stream stop |
| `gui/widgets/camera_widget.py` | caps advertising (sensor features + `andor_raw_stats` gate on both mono backends), `set_hw_andor_feature`, `reset_andor_sensor_defaults`, `get_raw_frame_stats`, `capture_raw_average` |
| `gui/widgets/camera_manager.py` | fan-outs for the four new methods |
| `gui/widgets/hw_controls_snapshot.py` (NEW) | ONE persisted-key list shared by both write sites — closes the documented bulk-save gap (which dropped the andor_* display keys) structurally |
| `gui/dialogs/camera_settings_dialog.py` | sensor rows (cooling / readout rate / gain mode / noise filter / blemish), Signal (raw sensor) group w/ histogram + 500 ms timer, software-correction readout line, snapshot-based persistence |
| `gui/pages/hardware_setup.py` | `_apply_hw_controls` sensor block (order: gain_mode → readout_rate → bools → auto_scale → levels), bulk save via snapshot |
| `gui/widgets/raw_histogram_widget.py` (NEW) | log-y raw histogram, clip bins tinted red |
| `gui/widgets/camera_feed_view.py` | SATURATED badge (raw clipped_frac ≥ 0.5%, polled as lock snapshot) |
| `gui/pages/workflows/fluorescence_mosaic_workflow.py` | `_ChannelPromptDialog` (live-applying per-channel exposure), worker probe (well centre: coarse AF + frozen levels) + `_capture_tile` averaged path + lease/entry-focus handling in finally, AF hooks, `_FocusSurveyDialog` (report-only), popout sections (Channels / Autofocus / Post-processing + retroactive apply), focus-survey persistence at run end |
| `SupportClasses/FluorescenceMosaicStore.py` | channel metadata (`display_lo/hi`, `avg_frames`, live `exposure_us`), `attach_processed` + processed-image preference in overlays (detection keeps raw), `set_focus_survey`/`get_focus_survey`/`set_surface_model` |
| `SupportClasses/MosaicCalibration.py` + `gui/dialogs/mosaic_settings_dialog.py` | `avg_frames` knob (shared camera-timing group) |
| `SupportClasses/MosaicFocusTracker.py` (NEW) | checkerboard lattice scheduling (grid-indexed, serpentine-proof), running plane fit, refusal/outlier isolation, `PlanePredictor` replay |
| `SupportClasses/TileAutofocus.py` (NEW) | lease-safe focus executor: micro-sweeps + coarse ladder, readback-verified moves, stale-drop = loud abort |
| `SupportClasses/SampleSurface.py` (NEW) | plane / linear (Delaunay) / spline (thin-plate RBF) surface models; confidence = well ∩ sample hull |
| `SupportClasses/FluorescencePostProcess.py` (NEW) | median/gaussian denoise + rolling-ball background subtraction (downscaled opening for big kernels) |
| `SupportClasses/PickAndPlaceManager.py` | `PickPlaceTarget.pick_z_zref_mm` (conditional-emit), `_execute_cell_removal` per-target override |
| `gui/pages/workflows/cell_targeting_workflow.py` | "Removal Z from measured sample surface" option + `_surface_z_resolver` (full gate chain: survey → verified datum → above-plate-bottom sanity; per-target low-confidence fallback) |

## Implementation Steps — ALL DONE

- [x] Camera tier: stats/aliases · sensor features · clip level · reader-loop stats · averaged capture · plumbing · snapshot + both write sites · restore order · histogram + dialog · badge · tests
- [x] Fluor Stage A (per-channel exposure) · B (averaging + frozen levels, probe at well centre) · C (post-processing, save-time, non-destructive) · D (checkerboard AF, user step/range, lease + restore in finally) · E (sample-surface artifact + survey dialog, **no plate-datum writes**) · F (cell targeting per-target surface Z)
- [x] Follow-ups: software-correction readout line · Tucsen raw-stats/averaging parity · retroactive "Apply to captured channels now"
- [x] Tests + regression + `gui.app` import smoke

## Testing Notes

New suites (all hardware-free): `test_v713_andor_sensor_features` (41),
`test_v713_andor_raw_stats` (18), `test_v713_andor_raw_average` (18),
`test_v713_fluor_postprocess` (14), `test_v713_mosaic_focus_tracker` (14),
`test_v713_tile_autofocus` (11), `test_v713_sample_surface` (14),
`test_v713_fluor_capture` (17), `test_v713_cell_targeting_surface_z` (12),
`test_v713_tucam_raw_parity` (14) — **173 combined-run green**, plus additive
`FakeAndorCam` extensions and the 6-arg `finished_ok` update in
`test_v75x_fluorescence_mosaic` (35 green).

Highlights: exact clipped fractions at 12/16-bit clip levels (`>=`-at-clip pinned);
uint16-overflow-proof exact averaging (values near 60000); the REAL reader loops produce
stats AND the display frame; stop→apply→restart sequence pinned by op ordering; the
`hw_controls_snapshot` key-list test documents + closes the bulk-save gap; restore-order
recording test; checkerboard-vs-sequence scheduling mutation guard; refused/outlier samples
never enter the surface fit; **end-to-end AF over a synthetic tilted sample surface through
the real worker recovers the plane slopes**; microscope lease acquire/release pairing on
success/abort/refusal + entry-focus restore; surface resolver gate chain incl. the
below-plate-bottom refusal and focal-sign physics; raw-vs-processed image preference
(detection reads raw, overlays prefer processed; a fresh save drops stale processed copies).

Regression green (per-suite/batch): andor ×2 (45), tucsen (74), camera hw-controls /
image-correction / calibration-store (146 batch), fluorescence mosaic (35), fluor shift (15),
target actions (48), per-bore cell targeting (50), workflow settings popout (35), spheroid
survey tab (49), suite hygiene, `gui.app` import smoke.

## Bench Verification Checklist (ME3B V1, Zyla + Ti-E) — IN ORDER

1. Zyla open-time log lists each sensor feature applied (runtime enum strings) or skip
   reason; `[camera start]` readback shows 16-bit low-noise gain, 216 MHz, filters on,
   cooling on; a changed gain mode persists across restart (persisted wins over defaults).
   ✅ Operator reports the signal improvement is real on the bench (2026-08-07).
2. Readout-rate toggle changes dark-frame noise in the histogram; a 12-bit gain mode drops
   histogram range/clip to ~4095 and the SATURATED badge fires at 4095 not 65535; cooling
   trends to "Stabilised".
3. **SpuriousNoiseFilter verified by consequence** on known sub-resolution puncta — if it
   eats them, flip the shipped default to OFF (one table line).
4. Deliberate overexposure lights the badge at ~0.5 % clipped; clears on reduction.
5. `avg_frames=8` on a dim well: visible SNR gain, no tile-to-tile brightness steps,
   `display_lo/hi` recorded; webcam slot unchanged. Repeat once on the Libra (same knob,
   same treatment).
6. Per-channel exposure: live view responds in the prompt dialog; metadata matches readback;
   entry exposure restored after run AND abort.
7. AF: mis-focus ~200 µm → probe coarse solve recovers; focus follows tilt; checkerboard
   pattern in log; abort → focus restored + lease released (jog microscope card immediately
   usable); empty corner tiles refused without dragging the fit; channel 2 replays without
   sweeps.
8. Survey dialog: tilt/residual/height-above-plate-bottom plausible; spline vs linear vs
   plane on a hydrogel well; datum-refusal paths show reasons; **no plate-bottom write**.
9. Cell targeting with surface-Z: removal lands at the surface (above glass on hydrogel);
   low-confidence target falls back; needle never below the plate-bottom floor.
10. Post-processing: raw PNG unchanged; `_proc.png` preferred in overlays; disable reverts;
    "Apply to captured channels now" retro-processes an old mosaic.

## Issues & Decisions

- **Sample surface ≠ plate bottom (operator correction, mid-design):** the AF map measures
  where the CELLS are — often above the well bottom (hydrogel) — so it is persisted as its
  own per-well artifact with plane/linear/spline evaluation and is NEVER installed into the
  plate-bottom datum. The whole-plate tilt comparison in the survey dialog is diagnostic
  text only.
- **Checkerboard, not every-Nth:** the raster is serpentine, so sequence-index scheduling
  stripes; sweeps are keyed on the tile's (col, row) grid index with staggered lattice rows.
- **Probe at the well centre:** tile 1 of the raster is a bounding-square corner = empty
  glass on a circular well; freezing display levels or seeding AF there would calibrate on
  background. The probe visit gets the retract-gated `safe_travel_to`; tile 0 then uses a
  plain move.
- **Never fake-average display frames:** each display frame is independently autoscaled, so
  their mean is not quantitative. Cameras without raw support fall back to the single-frame
  path byte-identically, logged once.
- **The bulk "Save Camera Settings" gap** (dropped `andor_auto_scale/lo/hi` since v7.9) is
  fixed as a side effect of `hw_controls_snapshot` — both write sites now share one key list,
  pinned by an AST test (calls checked, not substrings — the v7.10/v7.11 weak-guard lesson).
- **Zyla gain/brightness/contrast:** the Zyla has no ISP; its hardware rows stay hidden BY
  DESIGN (readout-cannot-lie). The always-existing SOFTWARE correction is now printed in the
  readout as `software corr = … (display-only)` so those values are readable on every camera.
- **Tucsen parity by construction:** raw stats + averaging use the SAME
  `mono_display.RawAverageRequest` and the same `andor_raw_stats` capability gate, so the
  dialog Signal section, the SATURATED badge and the mosaic's averaged tiles light up on the
  Libra with zero consumer changes. Clip level comes from each frame's own `ucDepth`.
- **Signal signature change:** `_SingleWellMosaicWorker.finished_ok` grew a 6th (meta) arg;
  the one existing connector was updated (grep confirmed no others).
- A concurrent session added `_bore_markers` to `camera_feed_view.py` mid-implementation;
  changes coexist (saturation badge untouched).

## Deferred / follow-ups

- Deconvolution (needs a measured PSF) and brightfield-LED auto-off during fluorescence
  channels (not selected).
- Per-channel parfocal (chromatic) focus offsets for 20×+ objectives (map replay is within
  DOF at 4×/10×).
- Per-tile mono16 flat-fielding (the raw path now exists end-to-end; noted in
  `FluorescencePostProcess`'s docstring).
- `SensorCooling` may be read-only on USB Zylas — tolerated; consider hiding if the bench
  shows it immutable.

---

# v7.13.x follow-up (2026-08-07) — Zyla bench fixes + one-shot signal optimizer

Bench reports: **(1)** 2048×2048 mode drops frames and the feed sometimes goes dead;
**(2)** exposure is not settable — "it resets to a small number"; **(3)** wanted: the
software computes the camera parameters for the best histogram, sets them, and they then
STAY FIXED (no per-frame changes).

## Root cause (verified in pylablib source, `AndorSDK3.py:449-466`)

**pylablib's `set_exposure()` pins FrameRate at its MAXIMUM** (`set_frame_period(0)`)
*before* writing ExposureTime, and the SDK truncates the request against
`ExposureTime.max ≈ 1/FrameRate` — ask for 500 ms, get ~10–38 ms. That is bug (2), and
MEBP never touched `FrameRate` anywhere (repo grep: zero hits). The same pinned-at-max
FrameRate exceeds the USB3 link's `MaxInterfaceTransferRate` at full 2048×2048 Mono16,
overflowing the camera's INTERNAL buffer — bug (1). Compounding the dead feed: the reader
loop only re-armed when `acquisition_in_progress()` admitted False, and a wedged USB
stream keeps claiming True. Secondary: `get_exposure_time_range` read the attribute
without `update_properties=True` (stale cached max → the dialog spin clamped keystrokes
instantly — the visible "reset"); the dialog never re-synced the spin to the achieved
value; `_apply_hw_controls` restored exposure BEFORE gain mode/readout rate (wrong
constraint set).

## Fixes

1. **`andor_backend.put_exposure_time` bypasses pylablib's poisoned `set_exposure`
   entirely** (`_apply_exposure_s`): lower FrameRate first via `set_frame_period(t×1.02)`
   when the request doesn't fit the current frame period, clamp against the LIVE
   ExposureTime range (`_sdk_attr` requests `update_properties=True` with a TypeError
   fallback for older builds), write the attribute directly, then `_sync_frame_rate()`.
   Returns True only when the achieved value is within ~2 % of the request; a live-write
   refusal falls back to the proven stop→apply→`_start_stream()` shape (stream ALWAYS
   restarted).
2. **`_sync_frame_rate()` caps FrameRate to min(FrameRate.max, MaxInterfaceTransferRate ×
   0.97)** — Andor's own guidance for USB Zylas. FrameRate.max is dynamic (folds in the
   current exposure/ROI/readout), so re-raising it can never clamp the exposure back
   down (pinned by a coupled-stub test that RAISES on any out-of-range write). Called at
   open (pre-acquisition), resolution change, readout-rate/gain-mode change, and after
   every exposure set. `get_settings()` now reports `frame_rate` +
   `max_interface_transfer_rate` so the constraint is bench-visible.
3. **Reader-loop forced re-arm**: after `_stall_threshold_fails()` consecutive
   wait-failures (exposure-aware — max(10 s, 4× exposure), so a 5 s exposure never
   spuriously re-arms) the stream is re-armed even while `acquisition_in_progress()`
   claims True, with exponential backoff (×2 per forced re-arm, reset on any real frame).
4. **`get_exposure_time_range` reports the ACHIEVABLE max** — min(30 s spec cap,
   1/FrameRate.min) — not ExposureTime.max at the current frame rate, so the dialog spin
   stops clamping keystrokes to ~one frame period.
5. **Dialog truth-sync** (`camera_settings_dialog._on_exposure_changed`): after the set,
   the spin re-syncs to the camera's achieved value under blockSignals — it never shows
   an exposure the camera isn't running. (Persistence was already truthful — `_persist`
   reads back from the device — only the UI lied.) Readout gains a
   `frame rate = X fps (link max Y)` line.
6. **Restore order fixed** (`hardware_setup._apply_hw_controls`): sensor features (gain
   mode → readout rate → bools) now apply BEFORE the exposure restore, so the saved
   exposure lands against the constraint set it was saved under; display levels stay
   LAST. Pinned order updated in the order test. No migration for previously-persisted
   clamped exposures (operator-visible value; one re-set persists the truth).
7. **⚡ Optimize signal (one-shot)** — `mono_display.plan_auto_exposure_step` (pure:
   linear-scale to P99.9 ≈ 70 % of the TRUE clip level; a clipped frame is CENSORED so it
   HALVES, never scales; terminal notes at range ends) +
   `mono_display.run_signal_optimize(mgr, cam_idx, freeze_display=)` (blocking,
   worker-thread-only: set → settle 1.5× exposure → `capture_raw_average(2)` →
   full-precision `np.percentile` → step, ≤6 iterations; on convergence turns the
   per-frame display auto-scale OFF and pins fixed black/white levels from the final
   frame — auto first, levels after, the restore-order invariant). Refuses when hardware
   auto-exposure is on (would fight it). Surfaced as a **"⚡ Optimize signal" button** in
   the settings dialog's Signal group (daemon worker + queued signal; result persisted
   via the shared snapshot) and an **"Auto" button in the fluorescence
   `_ChannelPromptDialog`** (exposure only — `freeze_display=False`, since the mosaic
   freezes its own capture levels from the probe; the found value writes into the ms
   spin, which the accept path already records into channel metadata). Both gated on the
   `andor_raw_stats` capability, so the Tucsen gets them too.

## Tests

NEW `tests/test_v713_exposure_framerate.py` (**21**) — the coupled `_FrCam` stub models
the real SDK3 physics (`ExposureTime.max = 1/FrameRate`, `FrameRate.max = min(sensor,
1/exposure)`, STALE attr limits unless `update_properties`, out-of-range writes RAISE),
including a test that REPRODUCES the bug through pylablib's own path; 500 ms/5 s stick;
short-after-long re-raises to the link cap; `cam.set_exposure` never called; unachievable
request returns False with the truth readable; range ≥ 1/FrameRate.min capped at 30 s;
link-cap after sync/resolution/readout (+ AST check that `open()` CALLS
`_sync_frame_rate` — a substring test passes on an import line); wedged-stream forced
re-arm fires in the [threshold, threshold+10-fail-check-granularity] window, scales with
exposure, and backs off ×2; dialog spin re-syncs to a clamped value. NEW
`tests/test_v713_signal_optimize.py` (**23**) — pure step math (converged/halve/weak/
quadruple/12-bit clip target), freeze recipe, and the driver against a linear-scene fake
manager (converges ≤6 steps to 63–77 % of clip; saturated start halves first; dim pins at
max + "weak"; freeze order auto→lo→hi; `freeze_display=False` touches no scaling;
hardware-auto-exposure refusal touches NOTHING; no-frames refusal never freezes blind);
dialog + channel-prompt gating. **3 mutations confirmed CAUGHT** (frame-period lowering
removed → exposure collapses to exactly 1/48 fps, 5 failures · forced-re-arm branch
removed → feed stays dead, 2 failures · display freeze removed → 2 failures). Pinned
restore-order test extended: bools → **exposure** → auto_scale → lo → hi.

Regression green per-suite: v713 andor features/raw-stats/raw-average (77) ·
zyla+display-scaling (45) · camera-hw-controls (22) · tucam parity + libra (90) ·
fluor-capture + fluorescence-mosaic (52) · camera-store + image-correction (48) ·
suite-hygiene (10) · remaining v713 suites (130 incl. the two new) · `gui.app` import
smoke.

## Bench verification (ME3B V1, Zyla) — IN ORDER

1. 2048×2048 16-bit: dialog readout shows `frame rate ≤ link max`; feed streams ≥10 min
   with no stutter and never goes dead; a USB wiggle mid-stream recovers with the
   "forcing a re-arm" log line.
2. Set exposure 500 ms → the spin STAYS at 500 ms, the feed visibly slows to ~2 fps,
   readout + persisted value read 500 ms; restart restores it.
3. A 5 s exposure runs without spurious re-arm log lines.
4. "⚡ Optimize signal" on a fluorescent well: converges in a few steps, the histogram
   P99.9 lands ~70 % of clip, no SATURATED badge, the display stops flickering per frame,
   and the result line matches the histogram. Repeat on the Tucsen (and confirm it
   refuses while the Libra's hardware auto-exposure is on).
5. Fluorescence channel prompt "Auto" fills the ms spin; the recorded channel
   `exposure_us` matches the readback.

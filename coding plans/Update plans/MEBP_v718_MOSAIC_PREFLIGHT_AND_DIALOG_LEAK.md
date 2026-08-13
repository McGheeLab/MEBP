# MEBP v7.18 — the mosaic "error code", and the app getting laggy over time

## Objective

Operator: *"two problems: 1. There was an error code when doing the mosaic
builder and prevented me from calibrating the objective 2. The software became
laggy/slow over time."*

Both root-caused from the operator's own `logs/app.log` — **and they share a
mechanism.**

## ⭐ What the log said

### 1. The mosaic error

Every tile was dropped, from the very first one:

```
10:39:09 WARNING Mosaic worker: no new camera frame within 2.5 s after the move (needed 3)
10:39:09 WARNING Mosaic worker: no fresh frame at tile 1 (1 consecutive)
…
10:39:32 WARNING Mosaic worker: no fresh frame at tile 8 (8 consecutive)
        → "camera stopped delivering frames (8 tiles in a row) — scan aborted"
10:41:43 INFO    Mosaic & camera calibration cancelled — nothing saved.
```

Twice in a row (10:39 and 10:40), which is why the objective could not be
calibrated. The measured µm/px (1.8974 @ 2600×2048) was fine — it was the
*verify mosaic* that failed.

**The arithmetic pins the cause.** The operator's saved `hw_controls` record
`'exposure_us': 299999.4` — a **300 ms exposure ≈ 2.4 fps**, so three frames need
~1.3 s of integration alone. `CaptureTiming` exists to size the wait from exactly
that, and would have asked for ~4.1 s… but the log says **2.5 s**, the flat
configured default. So `resolve_grab_timing` could not read the exposure
(`get_hw_settings()` → `{"source": "none"}`) and silently fell back to a timeout
shorter than the physics.

### 2. The lag

The app already logs a widget census. Over one session:

| time | tick# | widgets |
|---|---|---|
| 09:36 | 200 | 8218 |
| 10:23 | 8600 | 8405 |
| 10:30 | 9800 | 8609 |
| 10:31 | 10000 | 8789 |
| 10:35 | 10800 | **9072** |

**+871 widgets in one hour, in steps of ~82–204** — and the 300 ms status tick
stretched from ~63 s per 200 ticks to **203 s**, i.e. the event loop running at a
third of its rate. `_update_status` itself stayed at 1–2 ms, so the loop was
being starved by something else.

**Cause: parented dialogs are never released.** `dlg = SomeDialog(parent=self);
dlg.exec()` gives Qt ownership to the parent, so the dialog survives `exec()`
with its ENTIRE tree — every spin box, and every live `CameraFeedView` still
connected to `frame_captured`. **23 `exec()` sites across `gui/`, essentially
none disposing.** Each leaked camera dialog keeps converting and scaling a pixmap
on the GUI thread for every frame, forever. The operator opened the Scale/FOV and
mosaic dialogs repeatedly (four "Cleared stale mosaic FOV" lines), which is
exactly the ~200-widget steps.

### ⭐ Why these are ONE problem

`frame_count_value()` — the counter the mosaic waits on — is advanced by the
widget's **display timer on the GUI thread** (deliberately: the counter and the
frame buffer must move together). A GUI thread starved to a third of its rate
therefore publishes frames at a third of the rate, so "3 fresh frames in 2.5 s"
becomes unachievable. **The leak degrades the very clock the mosaic measures
freshness with.**

## Files Modified

- **`gui/widgets/components.py`** — NEW `exec_dialog(dlg)`: execs and guarantees
  `deleteLater()` in a `finally`. Deletion is **deferred, never immediate**,
  because callers legitimately read `dlg.result_um_per_px` / `dlg.values()` after
  close, and those touch live child widgets; `deleteLater` runs on the next
  event-loop pass, which cannot happen until the handler returns.
- **`gui/pages/hardware/objective_calibration_card.py`** — all 5 dialogs disposed
  (add-objective, pixel-calibration ×2, mosaic-confirm, Scale/FOV).
- **`gui/dialogs/scale_fov_calibration_dialog.py`** — the nested verify-mosaic
  dialog disposed (it owns a mosaic builder AND a live feed).
- **`gui/widgets/camera_feed_view.py`** — the capture-settings dialogs disposed.
- **`gui/pages/hardware/microscope_setup_panel.py`** — the diagnostics and
  write-support dialogs disposed.
- **`gui/pages/calibration.py`** — `_MosaicScanWorker` gains
  `_measure_frame_period()`, `_preflight_camera()`, `_measured_period_s`,
  `_last_wait_timeout_s`; `run()` preflights BEFORE any stage motion; the
  per-tile timeout uses the measured rate when the backend cannot report one; the
  abort message now names the measured fps, the applied timeout and what to
  change.
- **NEW `tests/test_v718_mosaic_preflight_and_dialog_leak.py`** — 13 tests.
- `tests/test_v716_tucsen_mosaic_fov_and_intensity.py` — one legitimate update
  (see below).

## Design notes

- **Measure, don't trust the SDK.** The preflight times the *actual* delivery
  rate through the same display-gated counter the scan uses, so a camera whose
  backend cannot report exposure still gets a correctly-sized timeout. A
  *reported* exposure still wins — the measured value is a fallback only, so a
  rig that works today is bit-for-bit unaffected (pinned).
- **Refuse before moving.** A frozen feed used to cost 8 moves and ~25 s and then
  report "camera stopped delivering frames" — a symptom, not a cause. The
  preflight refuses in ~2 s.
- **The two failure modes need different messages,** so the preflight
  distinguishes them by cross-checking the backend's own `frames_acquired()`:
  counter frozen but sensor streaming ⇒ *"the camera is streaming but its live
  view is not updating… make sure the preview is visible and started"* (a
  software state); both frozen ⇒ *"not delivering frames… check the exposure"*.
- `exec_dialog` is **only** for a dialog constructed for that one call; a
  cached/reused modeless dialog must not be passed to it. The camera settings
  dialog was checked and is freshly built per open.

## Testing Notes

**582 green** across the mosaic, capture-timing, camera-calibration and
microscope suites (`test_v718_*`, `v75x_nikon_ti_microscope`,
`v75x_unified_mosaic_calibration`, `v716_camera_square_crop`,
`v716_tucsen_mosaic_fov_and_intensity`, `v714_capture_core/ui`,
`v714_full_res_mosaic`, `v731_mosaic`, `v75x_mosaic_unreachable_travel`,
`v75x_mosaic_memory_and_overlay_perf`, `v713_mosaic_plate_frame`,
`v75x_reanchor_mosaic_and_camera_orientation`, `v711_objective_ladder`,
`test_suite_hygiene`), plus a `gui.app` import and offscreen builds of the real
`HardwareSetupPage` (3706 widgets) and `CalibrationPage` (1225).

**Demonstrated against a real page:** 10 dialog open/close cycles now net
**−5** widgets; the old undisposed pattern leaks **+520**.

**6/6 mutations CAUGHT:** `exec_dialog` stops disposing · disposal moved outside
the `finally` · preflight always passes (stage moves on a dead camera) · preflight
cannot distinguish a frozen feed from a dead camera · the measured period is
ignored (the flat 2.5 s, i.e. the original bug) · the measured period overrides a
reported exposure.

⚠ **One of my own tests was too weak and a mutation caught it.**
`test_a_reported_exposure_still_wins` used a sentinel measured period of
**99.0 s**, which exceeds `CaptureTiming.MAX_PERIOD_S` (60) and is therefore
discarded — so the test could not observe the `period is None` guard being
removed, and that mutation survived. Changed to 3.0 s (plausible, and would give
~23 s instead of ~4 s). Same failure mode this repo keeps recording: a sentinel
outside the validated range makes a mutation a no-op.

⚠ **A `TestGuardTheGuard`-style test came first:**
`test_the_leak_this_fixes_is_real` reproduces the operator's growth with a bare
`exec()` before the fix is asserted, so the disposal test cannot pass merely
because nothing leaks in a headless harness.

**One legitimate existing-test update.**
`test_v716_..._tucsen...::test_a_before_snapshot_is_taken_ahead_of_the_dialog`
asserted `src.index("dlg.exec()")` — a source-string match on the exact line this
change rewrites. Its *invariant* (the manager snapshot must precede the verify
dialog, which is the 400-minute-scan guard) is untouched and still asserted; the
matcher is now wrapper-agnostic so a cosmetic call change cannot break it again.

`test_v75x_plate_mosaic` was excluded — it contains the `TestManualAlignPage`
hang this repo documents as pre-existing.

## Needs verification on ME3B V1

1. **Re-run Mosaic & Camera Calibration on the Tucsen at 4×.** The verify mosaic
   should now build instead of aborting. Expect a log line
   `Mosaic preflight: camera delivering ~2.4 fps (~413 ms/frame)`.
2. If it still refuses, the message now says which of the two causes it is —
   send that line rather than "error code".
3. **The lag:** work for an hour as before (open Scale/FOV, the mosaic dialog,
   camera settings a few times each), then check `logs/app.log` for the `[Tick]`
   lines: `widgets=` should stay roughly flat instead of climbing ~870/hour, and
   the tick interval should stay near 63 s per 200 ticks.
4. If the exposure is genuinely 300 ms for a reason, nothing more is needed — the
   timeout now sizes itself. If it is 300 ms only because auto-exposure left it
   there, lowering it will also make the scan much faster (25 tiles × ~1.3 s of
   integration each).

## Not done (deliberately)

- **The remaining ~14 undisposed `exec()` sites** outside this workflow
  (`plate_library`, `print_setup_legacy`, `live_target_picker`,
  `fluorescence_mosaic_workflow`, `cell_targeting_setup_panel`,
  `hardware_setup` ×5, `calibration` ×2). `exec_dialog` is in place and they are
  a mechanical change, but each needs checking that its dialog is not cached and
  reused before disposal is safe — and the measured leak was in the camera path
  fixed here. Worth a follow-up sweep with the AST test extended to cover them.
- The **startup XY serial-scan stall** also visible in `logs/freeze.log`
  (`_connect_xy` → `_auto_detect_controller`, ~5 s at launch). Pre-existing, at
  connect time only, and unrelated to "over time".

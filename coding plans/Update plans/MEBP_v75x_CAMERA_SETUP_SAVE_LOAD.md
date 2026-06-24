# MEBP v7.5.x — Save / Load the full camera setup (+ start on load & startup)

## Objective

Let the operator persist the **entire** camera setup as this machine's default
and bring it back in one action — including **starting** the cameras. Two buttons
at the top of Hardware Setup → Cameras: **Save Camera Settings** and **Load
Cameras**. On startup, the last-used setup is reloaded automatically *and the
cameras are started*, matching how every other section restores.

## Background — what already persisted

A lot of camera state was already machine-level (identity-keyed) in
`CameraCalibrationStore` / `config/hardware/camera_calibrations.json`:
µm/px + rotation, image correction (brightness/contrast/gamma), camera-side
hardware controls (exposure/gain/gamma/brightness/contrast/resolution), and the
role→identity assignments. Startup already auto-**detected** and re-applied all
of that (`_maybe_auto_detect_cameras` → `_on_detect_live_cameras` →
`_auto_assign_sources_from_store` + `_restore_all_calibrations` +
`_on_camera_started_hw`). **The gaps:** nothing recorded *which cameras were
running*, nothing **started** them on load/startup, and there was no single
"save the whole setup / load it all" action.

## Changes

### `SupportClasses/CameraCalibrationStore.py`
- New per-identity **`autostart`** flag (sibling of `um_per_px` / `hw_controls`):
  `get_autostart`, `set_autostart`, `any_autostart`. Records whether a physical
  camera was running when the setup was last saved.

### `gui/pages/hardware_setup.py`
- **Top of the Cameras page** (Camera Detection & Assignment group): a button row
  — **Save Camera Settings** (`save` icon, accent) + **Load Cameras** (`play`
  icon) + a status badge.
- `_on_save_camera_settings()` — for each slot with an assigned source:
  re-affirm role→identity (`_remember_assignment`), persist image correction
  (`_persist_correction`), persist camera-side hardware controls **while running**
  (`get_hw_settings` → `store.set_hw_controls`, same shape as the settings
  dialog), and `store.set_autostart(identity, is_running)`. (µm/px + rotation
  already persist when calibrated, so they're left as-is.)
- `_on_load_cameras()` — `_on_detect_live_cameras()` (detect + restore every
  section) then `_start_saved_cameras()`.
- `_start_saved_cameras()` — start every assigned-but-stopped slot whose camera
  is flagged `autostart`; returns the count. Skips already-running slots.
- `_auto_load_cameras()` — startup variant (detect + restore + start), scheduled
  by `_maybe_auto_detect_cameras` (was `_on_detect_live_cameras`, which never
  started cameras). Startup gate widened to fire on `all_assignments()` **or**
  `any_autostart()`.
- `_set_cam_setup_status()` — drives the status badge.

## Files Modified

| File | Change |
|------|--------|
| `SupportClasses/CameraCalibrationStore.py` | `autostart` flag get/set + `any_autostart`. |
| `gui/pages/hardware_setup.py` | Save/Load buttons; save-snapshot, load+start, startup auto-load+start. |
| `tests/test_v75x_camera_calibration_store.py` | `autostart` round-trip + save/load-start page tests. |

## Implementation Steps

- [x] Store: `autostart` get/set/any
- [x] Save button + `_on_save_camera_settings`
- [x] Load button + `_on_load_cameras` + `_start_saved_cameras`
- [x] Startup auto-load+start (`_auto_load_cameras`, widened gate)
- [x] Tests (store round-trip + page save/start) — 17 in file, 67 across camera suites green

## Testing Notes

- `TestStore.test_autostart_round_trip` — flag persists, is a sibling that
  doesn't disturb µm/px, clears.
- `TestSaveLoadCameraSetup` — Save flags only running cameras (+ remembers
  roles); `_start_saved_cameras` starts only flagged + stopped slots; round-trip
  (save 2 running → new session both stopped → load starts both).
- Existing `TestAutoDetectOnShow` still green (the scheduled call now routes
  through `_auto_load_cameras`, which calls the stubbed detect + a no-op start).
- **Bench:** start the cameras you want, set their settings, click **Save Camera
  Settings**; restart the app → those cameras should come up running with their
  settings. **Load Cameras** repeats it on demand.

## Adversarial review (6-agent, findings verified against code)

- **[HIGH, fixed] Stale identity in `_start_saved_cameras`.** `camera_identity(i)`
  was read *before* `set_source(i, data)`, so the autostart check could use the
  manager's lagging selected source (the detect "restore-prev" branch sets the
  combo with signals blocked, so `set_source` never fired). Fixed: sync the
  source first, then resolve identity. Regression test
  `test_set_source_runs_before_identity_check`.
- **[HIGH, fixed in the restore path] `_apply_hw_controls` mis-handled non-bool /
  unknown auto-exposure.** OpenCV/DShow cams (the Teslong needle cams) report a
  raw `CAP_PROP` value (e.g. `-1.0`) or nothing for auto-exposure; `bool(-1.0)`
  would force auto ON. Now only a *clean bool* drives auto-exposure, and manual
  exposure/gain restore unless auto is explicitly `True` — so "reload exactly"
  holds for OpenCV cams without clobbering a real auto state. (bool/None cases
  behave exactly as before; 39 hw-control tests green.)
- **[MEDIUM, left as-is] resolution `None` placeholder.** Save writes
  `resolution: None` when unavailable; the store drops `None` and restore uses
  `if res:` — currently safe, and identical to `camera_settings_dialog._persist`,
  so kept for consistency.

## Follow-up — implicit "last-used" for the microscope (no Save click)

Operator request: *"The Microscope Camera setup, we should apply the last used
camera automatically (Camera spec = BUC3D-1000C, resolution = 916×686, but these
can change)."* The explicit Save flow above already covers this **after one Save
click**; the operator wanted it fully implicit for the microscope — it should
"just remember whatever you last used". Two additions, both scoped to the
`MICROSCOPE` role:

### `gui/pages/hardware_setup.py`
- **Resolution applied to the device on every microscope start** (`_on_camera_started_hw`):
  when no per-camera `hw_controls.resolution` was restored, apply the persisted
  spec-combo resolution (`camera_config.active_resolution`, which already
  auto-saves with the normal hardware config) via `_maybe_apply_resolution_to_device()`.
  Previously the spec-combo resolution reached the device *only* on a manual combo
  change, so a freshly-started microscope came up at native resolution even though
  the combo read 916×686. `hw_controls.resolution`, when present, still takes
  precedence (it set the device earlier in the method).
- **Implicit autostart flagging** — `_on_camera_started_hw` now `set_autostart(identity, True)`
  for the microscope on *any* start, and `_on_toggle_camera` `set_autostart(identity, False)`
  on a **deliberate user stop**. So the moment the operator runs the microscope
  it's remembered for next launch, and a deliberate stop forgets it. App shutdown
  stops via `stop_all` (not the toggle handler), so it never clears the flag.
  Combined with the `any_autostart()` startup gate, the microscope auto-detects,
  restores, and **starts at its last-used resolution** on every launch with no
  Save click. (First-ever launch still needs the operator to start it once — the
  flag is *learned from use*, not seeded.)

The camera **spec** (BUC3D-1000C) needs no new wiring: it persists in
`camera_config.camera_spec` and `set_config` already restores it to the combo at
startup; the resolution apply above is what makes the device match.

## Follow-up — start the cameras in the background (no startup freeze)

The startup auto-load (`_auto_load_cameras`) detected + started cameras **on the
GUI thread** via `QTimer.singleShot(0, …)`, so the window froze for several
seconds right as it appeared. Worse, `CameraManager.detect_cameras()` called
`refresh_cameras()` on **every** camera widget and each one re-ran the slow
OpenCV device probe (opening/releasing `cv2.VideoCapture` on indices 0–3) — i.e.
~3× redundant probing (≈12 device opens) blocking the UI.

Fix — probe once, off the GUI thread, then finish on it:

### `gui/widgets/camera_widget.py`
- `refresh_cameras(probe=None)` — accepts a pre-computed inventory
  (`{"opencv": [indices], "dshow": [...], "toupcam": [...]}`) and populates the
  combo from it instead of probing. `None` keeps the old self-probe (lazy
  `start()` path).

### `gui/widgets/camera_manager.py`
- `detect_cameras(opencv_indices=None)` — builds the inventory **once**
  (OpenCV indices from the arg or a sync probe; DirectShow/COM + ToupCam enum
  always here on the GUI thread — fast, and COM stays on the main thread) and
  shares it with every widget. Kills the redundant per-widget re-probe.
- `detect_cameras_async(on_complete)` — runs the slow OpenCV index probe on a
  daemon thread, then calls `on_complete(indices)` (None on failure) from that
  thread.

### `gui/pages/hardware_setup.py`
- New `_cameras_probed = Signal(object)` connected to `_finish_auto_load`; the
  worker's `on_complete` is `self._cameras_probed.emit`, so the result marshals
  to the GUI thread via a **queued** connection (Qt's cross-thread-safe path).
- `_auto_load_cameras()` now calls `mgr.detect_cameras_async(self._cameras_probed.emit)`
  and returns immediately (window paints + stays responsive); `_finish_auto_load(indices)`
  runs on the GUI thread → `_on_detect_live_cameras(indices)` (populate from the
  pre-probe, no reprobe) + `_start_saved_cameras()`. Synchronous fallback if the
  manager lacks `detect_cameras_async`.
- `_on_detect_live_cameras(opencv_indices=None)` passes the indices through to
  `detect_cameras`; a non-list arg (e.g. the bool from `QPushButton.clicked`) is
  treated as "no pre-probe" → sync detect. The manual **Detect Cameras** /
  **Load Cameras** buttons stay synchronous (deliberate user action), but now
  also benefit from the probe-once optimization.

COM/DirectShow enumeration deliberately stays on the GUI thread (comtypes COM is
apartment-sensitive on worker threads, and a wrong-thread `[]` would make
identities fall back to `opencv:<idx>` and miss the stored `dshow:…` keys). Only
the OpenCV `VideoCapture` probe — the dominant blocking cost, and thread-safe
(OpenCV manages its own COM in C++) — moves off-thread.

Net: the window loads and is interactive immediately; camera detection runs
concurrently on a worker thread; cameras come online a moment later (the flagged
microscope opens on the GUI thread — one ~1 s open, after the window is up).

### Tests (`tests/test_v75x_camera_calibration_store.py`)
- `TestBackgroundStartupProbe.test_auto_load_backgrounds_probe_then_finishes_on_gui`
  — pre-probed indices flow through `_finish_auto_load` → detect + start.
- `TestBackgroundStartupProbe.test_manager_detect_async_runs_off_thread_and_returns_indices`
  — the real async probe completes on a worker thread (not MainThread) and
  returns a list/None, never raising on the caller.
- `TestAutoDetectOnShow` updated to stub `_auto_load_cameras` (what
  `_maybe_auto_detect_cameras` now schedules). Full file = 26 green; camera +
  calibration suites = 105 green.

## Follow-up — "Camera spec" dropdown not auto-selected on launch (KEY/name)

Operator: *"each time I start I have to manually select the BUC3D-1000C camera
from the dropdown."* Diagnosed via a 4-agent workflow (spec-combo / source-combo
/ reset-audit / synthesis); root cause confirmed end-to-end.

The dropdown in question is the **Camera spec** combo (`self.camera_combo`), not
the source dropdown. The microscope *source* dropdown already auto-selects
correctly — the ToupCam enumerates with a stable USB-path id
(`\\?\usb#vid_0547&pid_1377#5&1abff3a4&0&13`, `displayname` `C3CMOS10000KPA`)
that matches the stored `toupcam:…` assignment.

**Bug:** the spec combo is populated `addItem(name, name)` where `name` is the
catalog **KEY** (`"BUC3D-1000C"`), so each item's *data* is the short key. But
`set_config` restored it via `self.camera_combo.findData(cam_cfg.camera_spec.name)`
— and `camera_spec.name` is the long display name
(`"Bestscope BUC3D-1000C (ToupTek C3CMOS10000KPA)"`). `findData` matches data
exactly, so the long name never matched the short-key data → `cidx == -1` →
`setCurrentIndex` never fired → the dropdown sat on "None" every launch. The bug
was latent until BUC3D-1000C — the first catalog entry whose key ≠ name (earlier
entries where key == name coincidentally matched). The **save** path was already
correct (`_rebuild_config` reads `currentData()` = the KEY and looks it up).

**Fix** (`gui/pages/hardware_setup.py`, camera-config restore block): resolve the
catalog KEY from the saved `camera_spec.name` before `findData` — find the key
whose `spec.name` matches, with a legacy fallback for configs that saved the key
itself; unknown spec → `cidx == -1` → combo stays unset (unchanged safe
behaviour). Population (intentionally KEY-as-data), `_on_camera_changed`, and the
save path are untouched — the fix aligns *restore* to the existing contract.

Tests (`TestSpecComboRestore`, 4): auto-selects BUC3D-1000C (the direct
regression — fails pre-fix); legacy name==key still resolves; unknown spec stays
"None"; save round-trips back to the same long-name spec. Resolution restore is
independent (`preview_resolutions`) and asserted unaffected. Full store file =
30 green; camera + calibration suites = 135 green.

### Tests (`tests/test_v75x_camera_calibration_store.py`)
- `TestMicroscopeResolutionOnStart` (3): spec-combo resolution applied to the
  device on microscope start; `hw_controls.resolution` takes precedence; a
  non-microscope start applies nothing.
- `TestMicroscopeResolutionOnStart.test_microscope_start_flags_autostart` /
  `test_non_microscope_start_does_not_flag_autostart`: start flags autostart for
  the microscope only.
- `TestMicroscopeAutostartCleared`: a deliberate user stop clears the flag.
- Full `test_v75x_camera_calibration_store` = 24 green; camera suites
  (hardware-controls/image-correction/liveview/rotation) = 59 green.

**Needs real-HW verification on ME3B V1** (BUC3D-1000C via ToupTek SDK).

## Issues & Decisions

- **Load/startup start the *flagged* cameras (what was running at Save), not all
  assigned** — this is "exactly as we had it". Save while running → they come
  back; Save while stopped → they stay off.
- **Implicit microscope autostart is *learned from use*, not seeded** — the first
  launch after this ships won't auto-start the microscope (no flag yet); starting
  it once sets the flag and every launch after auto-starts it. Avoids presuming
  the operator wants a camera powered on before they've ever run it.
- **Implicit autostart is microscope-scoped** (per the operator's request). The
  needle cameras keep restoring their µm/px on detect but don't auto-start unless
  flagged via explicit Save — trivially extensible to all roles by dropping the
  `MICROSCOPE` gate if wanted.
- **Reused the existing identity-keyed store** rather than a new file, so the new
  flag travels with the physical camera/USB port like the rest of its settings.
- Startup auto-start is deferred via `QTimer.singleShot(0, …)` so the page paints
  before the blocking detect/start probe; gated so a first-ever run never probes.

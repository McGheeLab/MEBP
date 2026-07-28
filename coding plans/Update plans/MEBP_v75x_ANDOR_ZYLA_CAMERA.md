# MEBP v7.5.x — ANDOR Zyla-4.2P-USB3 Microscope Camera Backend

## Objective

Add support for the **ANDOR ZYLA-4.2P-USB3-2V3** — a scientific **sCMOS** camera driven by
the **Andor SDK3** runtime — as a selectable **microscope** camera, alongside the existing
OpenCV/DirectShow and ToupTek backends. The camera delivers **2048×2048 mono 16-bit** frames;
the whole app pipeline expects 8-bit **BGR**. The backend converts to BGR8 internally so the
rest of the pipeline (`_grab_frame`, `VisionDetector`, all display/consumer paths) needs no
changes.

Outcome: operator plugs in the Zyla → Hardware Setup → Cameras → **Detect** shows
"Andor: Zyla …" → assign the **Microscope** role → smooth binned, auto-scaled live feed →
tune exposure via the ⚙ settings dialog → calibrate objective µm/px → every microscope
consumer (mosaic scan, plate-location click-rim, live-target picker, fluorescence mosaic, jog
view) works unchanged.

## Decisions (confirmed with operator)

1. **SDK binding: `pylablib`** (`Andor.AndorSDK3Camera`) — handles SDK3 queue/wait-buffer
   acquisition, AOI/binning, pixel-encoding→numpy, exposure/ROI. Adds a Python dependency +
   PyInstaller hidden-import. Far lower risk than hand-rolled ctypes for a complex sCMOS.
2. **Live feed = binned preview + per-frame auto-scale.** Default the live readout to a binned
   mode (auto-pick ≤1280 wide → 2×2 = 1024²) for a smooth USB3 feed; map mono-16→8 by scaling
   to a per-frame 1–99 percentile range (good for dim fluorescence). Full-res 2048² selectable
   via the resolution combo. Backend emits **BGR8**.
3. **DLLs are in-repo** at `DLLs/zyla dlls/` (confirmed: `atcore.dll` + all SDK3 companions +
   `atutility.dll`). Discovery pins this folder first, then Andor SDK3 / Solis Program-Files
   dirs / PATH.

## Design principle

`AndorBackend` mirrors `ToupCamBackend`'s duck-typed surface and delivers **8-bit BGR** frames.
Everything Zyla-specific (SDK3 acquisition, mono-16→8 auto-scale + GRAY2BGR, binning) is
contained in `gui/widgets/andor_backend.py`. The rest of the app just gets a new `"andor"`
dispatch branch and a new `andor:<serial>` device identity. Hardware controls the Zyla lacks
(gamma/brightness/contrast/gain/auto-exposure) are reported `None` so the settings dialog
auto-hides them; only **exposure** is exposed. Software image-correction (display-only) still
works.

## Files Modified / Created

| File | Change |
|------|--------|
| `gui/widgets/andor_backend.py` | **NEW** — pylablib-backed backend; guarded lazy import, `_find_andor_dll_dir()`, `ANDOR_AVAILABLE`, `AndorBackend` (ToupCam-compatible surface), mono-16→BGR8 auto-scale, daemon reader thread. |
| `gui/widgets/camera_widget.py` | Guarded import + `ANDOR_AVAILABLE` fold-in; `detect_andor_cameras()`; `refresh_cameras` combo item; `start()` route + `_start_andor()`; `stop()` release; `_grab_frame`/`capture_fresh_frame` read branch; `"andor"` branch in `hardware_capabilities`/`get_hw_settings`/`set_hw_*`/`set_capture_resolution`/`log_hw_settings`. |
| `gui/widgets/camera_manager.py` | `detect_cameras()` — guarded `detect_andor_cameras`, add `"andor"` to `probe`. |
| `gui/widgets/camera_identity.py` | `identity_for_source`/`source_for_identity` — `andor:<serial>` branch. |
| `gui/dialogs/camera_settings_dialog.py` | `_render_readout` gate widened to include `"andor"`. |
| `gui/pages/hardware_setup.py` | `_on_save_camera_settings` hw-controls gate widened; Detect tooltip mentions Andor. |
| `config/hardware/cameras.json` | Add Zyla `CameraSpec` (6.5 µm pixel, 2048² max, binned previews). |
| `requirements.txt` | Document `pylablib` as an OPTIONAL manual install (NOT an active line — see numba/numpy note below). |
| `MEBP.spec` | Conditionally `collect_all('pylablib')` + bundle `DLLs/zyla dlls/`. |
| `tests/test_v75x_andor_zyla_camera.py` | **NEW** — `FakeAndorCam` injection, mono-16→BGR8 conversion, caps/settings/resolution delegation, identity round-trip, availability guard. |

## Implementation Steps

- [x] Create this update-plan doc.
- [x] `gui/widgets/andor_backend.py` — new backend.
- [x] `gui/widgets/camera_widget.py` — flags, detect, refresh, start/stop, grab, hw branches.
- [x] `gui/widgets/camera_manager.py` — andor probe.
- [x] `gui/widgets/camera_identity.py` — andor identity.
- [x] `gui/dialogs/camera_settings_dialog.py` + `gui/pages/hardware_setup.py` — widen gates + tooltip.
- [x] `config/hardware/cameras.json` — Zyla spec.
- [x] `requirements.txt` + `MEBP.spec` — pylablib + DLL bundling.
- [x] `tests/test_v75x_andor_zyla_camera.py` — headless tests.
- [x] Run new + regression camera suites headless.

## Testing Notes

**Headless (no hardware / pylablib):**
- `python -m pytest tests/test_v75x_andor_zyla_camera.py -v`
- Regression: `tests/test_v75x_camera_hardware_controls.py`, `tests/test_v75x_camera_image_correction.py`,
  `tests/test_v75x_camera_calibration_store.py`, `tests/test_v730_simulated_camera.py`.
- Importing `gui.widgets.andor_backend` without pylablib/DLLs must not raise; `ANDOR_AVAILABLE`
  falsy; app launches with the Andor backend simply absent.

**Real hardware — CONFIRMED WORKING recipe WITHOUT the Andor installer (2026-07-23, ME3B rig,
Zyla serial VSC-07863).** The operator had only the SDK3 `at*.dll` files (no Andor installer).
Two extra pieces were needed beyond copying those DLLs, and both are now handled in-repo:

1. **USB driver.** Copying DLLs does NOT install the camera's kernel driver — Windows left the
   ZYLA (`USB\VID_136E&PID_0014\VSC-07863`) at **Code 28** ("drivers not installed"). Fixed by
   binding a generic **WinUSB** driver to the device with **Zadig** (https://zadig.akeo.ie):
   Options → List All Devices → select ZYLA (136E/0014) → WinUSB → Install/Replace Driver. No
   Andor Solis/SDK installer required. (libusbK would likely also work; WinUSB is confirmed.)
2. **Missing libusb backend DLL.** Andor's USB plugins delay-load an external backend:
   `atusb_libusb10.dll` needs `libusb-1.0.dll`, `atusb_libusb.dll` needs `libusb0.dll` — the
   pasted DLL set had NEITHER, so atcore loaded the plugin but enumerated 0 cameras even with a
   driver bound. Fixed by vendoring a 64-bit **`libusb-1.0.dll`** into `DLLs/zyla dlls/` (sourced
   from the freely-redistributable `libusb-package` PyPI wheel, LGPL; ABI-stable libusb-1.0). It
   pairs with the WinUSB driver. `MEBP.spec` already bundles the whole `DLLs/zyla dlls/` folder,
   so it ships with the exe.

With both in place: `Andor.get_cameras_number_SDK3()` → 1; `AndorBackend.enumerate()` returns
`{id: 'VSC-07863', displayname: 'ZYLA4.2PUSB32V3', …}`; `open()` + `read()` deliver a
(1024×1024×3) uint8 BGR frame (mono-16→8 auto-scale spanning 0–255). `enumerate()` also logs an
actionable hint (check power/driver) when the SDK loads but finds 0 cameras.

Symptom→cause quick-map for the next person: Device Manager Code 28 = no driver (run Zadig);
SDK count 0 with driver OK = missing `libusb-1.0.dll` in the DLL folder.

**Live-feed freeze fixed (2026-07-23).** After the camera streamed, the feed froze in the GUI
with no error logged. Root causes (the app log showed it jumping to 2048×2048 on every start):
(1) the resolution combo defaulted to the FIRST `preview_resolutions` entry, which was 2048² —
full res is heavy for a live feed; (2) pylablib's default 100-frame ring buffer at 2048² mono-16
is **~840 MB**, which thrashes RAM and stalls the GUI; (3) the reader thread swallowed
`wait_for_frame` errors silently (`except: continue`), so any stall looked like a clean freeze;
(4) a persisted `hw_controls.resolution = [2048,2048]` for `andor:VSC-07863` forced full res on
start regardless of the combo default. Fixes:
- `cameras.json`: Zyla `preview_resolutions` reordered **binned 1024² first** (default preview).
- `andor_backend._buffer_nframes()`: ring buffer sized by BYTES (~256 MB cap, min 10) instead of
  pylablib's fixed 100 → full-res ring drops 840 MB → ~268 MB; `_start_stream`/`set_resolution_index`
  use it.
- `andor_backend._reader_loop`: throttled logging + **self-heal** — if `acquisition_in_progress()`
  is False (e.g. transient overflow) it re-arms acquisition instead of spinning silently.
- `andor_backend._mono_to_bgr8`: auto-scale percentile computed on a **decimated sample** (fast).
- Corrected the persisted `andor:VSC-07863` hw-control resolution `[2048,2048] → [1024,1024]`.
Verified on hardware: binned 1024² default streams smoothly; full 2048² also streams with the
capped buffer.

**Real hardware (ME3B rig; camera plugged; DLLs present; DRIVER installed; `pip install pylablib`):**
1. Hardware Setup → Cameras → **Detect** → "Andor: Zyla 4.2 (…)" appears in a slot's source combo.
2. Assign that slot the **Microscope** role → **Start** → smooth binned, auto-scaled live feed
   (not black on a dim scene).
3. ⚙ settings → Exposure slider changes brightness; gamma/brightness/contrast HW controls
   hidden; resolution combo switches binning; values persist per `andor:<serial>` across restart.
4. Objective Calibration card → calibrate µm/px live → stored under the CameraSpec name;
   live-target-picker click maps correctly.
5. Mosaic scan / plate-location click-rim / fluorescence mosaic driven by the Zyla feed.
6. Unplug mid-session → graceful (no crash); replug + Detect recovers.

## Issues & Decisions

- **Gain / cooling / trigger deferred.** The Zyla's gain is an enum (`SimplePreAmpGainControl`),
  not a numeric %, so v1 exposes exposure only; sensor cooling + trigger modes are out of scope.
- **pylablib is an optional dependency** — everything is lazy/guarded exactly like the ToupCam
  backend, so dev/CI without pylablib or the SDK is unaffected. Tests inject a `FakeAndorCam`
  and never load the real SDK.
- **`atmcd64d.dll`** in the DLL folder is the older SDK2 DLL (iXon/CCD) — unused by the SDK3
  Zyla path, harmless.
- **PyInstaller** — `collect_all('pylablib')` is wrapped in try/except in the spec so the build
  still works when pylablib isn't installed on the build machine.
- **numba + numpy pin (decided by operator 2026-07-22).** `pip install pylablib` pulls `numba`
  (a declared pylablib dep) + `llvmlite`, and numba caps `numpy<2.5` — so it downgraded numpy
  2.5.1 → 2.4.6. **Operator chose to keep numba/llvmlite** (the JIT acceleration is worth it;
  the 2.5.1→2.4.6 step is immaterial to MEBP's features). Consequences applied:
  - `requirements.txt`: numpy pin relaxed `2.5.1 → 2.4.6` (numba-compatible), and `pylablib` +
    `numba` + `llvmlite` are active Windows-only deps — so `pip install -r requirements.txt`
    resolves cleanly (previously the 2.5.1 pin vs numba's `<2.5` was unsatisfiable).
  - `MEBP.spec`: `numba` / `llvmlite` / `pandas` removed from the excludes (pylablib imports all
    three eagerly, so the bundled Zyla path needs them; ~150 MB added to the bundle).
  - Verified: numba imports + JITs, pylablib+Andor import, `ANDOR_AVAILABLE` True, full test
    suite green on numpy 2.4.6.

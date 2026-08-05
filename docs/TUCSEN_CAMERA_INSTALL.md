# Installing a Tucsen camera (Libra / Dhyana / Aries / FL) on a new machine

**Audience: Claude, working on a second MEBP machine.** The operator has already
done this once on ME3B (Libra 25) and the software side is in git. Your job here is the
*machine-specific* half: get the camera enumerating, then **verify the SDK mapping on this
camera** rather than assuming it matches.

Read this whole file before running anything. It is ordered, and step 5 is the one that
matters most.

Full background and the reasoning behind every design choice:
`coding plans/Update plans/MEBP_v79_TUCSEN_LIBRA_CAMERA.md`.

---

## 0. What you need, and what you cannot do yourself

| Need | Notes |
|---|---|
| The repo, pulled | The backend and the vendored SDK (`DLLs/tucsen dlls/`, ~27 MB) come with git. **No code changes should be needed.** |
| The vendor bundle | e.g. `Dhyana&FL&Libra16-18-22-25&Aries16_20251120.zip`, usually on a USB stick with the camera. Contains the driver installer. Locate it and note the path. |
| An **elevated** PowerShell | ⛔ **You cannot do this step.** Driver installation needs UAC, and you cannot answer a UAC prompt non-interactively. You must hand the command to the operator. Do not try to elevate yourself — you will hang. |

**Never** conclude "the software is broken" before finishing step 4. On the first machine the
camera was invisible purely because the driver was not installed, and the SDK reported that
correctly.

---

## 1. Confirm the software side is present

```bash
python -c "from gui.widgets.tucam_backend import TUCAM_AVAILABLE, _find_tucam_dll; print('DLL:', _find_tucam_dll()); print('available:', bool(TUCAM_AVAILABLE))"
```

Expect the vendored path and `True`:

```
DLL: C:\dev\MEBP\DLLs\tucsen dlls\TUCam.dll
available: True
```

If the DLL is `None`, the vendored SDK did not come through git — check
`git ls-files "DLLs/tucsen dlls/"` lists ~10 DLLs including `TUCam.dll` (~17 MB). Without it
the backend is inert and nothing else here will work.

Also run the suite once; it needs no camera:

```bash
python -m unittest tests.test_v79_tucsen_libra_camera
```

Expect **OK** (74 tests, ~2 s). A failure here is a software problem — stop and report it
rather than blaming the hardware.

---

## 2. Record the hardware state BEFORE installing anything

This is what lets you tell "driver missing" apart from "camera not plugged in", and it takes
seconds.

```powershell
Get-PnpDevice -PresentOnly | Where-Object { $_.InstanceId -match '^USB\\VID' -and ($_.Class -match 'Camera|Image|USBDevice' -or $_.Status -ne 'OK') } | Select-Object Status, Class, FriendlyName, InstanceId | Format-Table -AutoSize
```

A Tucsen camera appears as class **Image** with **VID `5453`**, e.g.:

```
OK   Image   Libra 25   USB\VID_5453&PID_E437\5&3d36519&0&5
```

### Two red herrings — do not chase these

* **`USB\VID_0000&PID_0001` "Unknown USB Device (Port Reset Failed)", Code 43.**
  `VID_0000` means Windows could not read the device descriptors **at all** — USB
  enumeration failed outright. **No driver install fixes this.** It is power, cable, or
  port. On ME3B such a device existed *and was not the camera*. Check its
  `FirstInstallDate`; if it predates the camera's arrival, it is unrelated.
* **A generic UVC camera** (class Camera, service `usbvideo`, a name like "HD USB Camera").
  Tucsen scientific cameras are **not** UVC. If you see one, it is a different device.

Also check whether a driver is already present:

```powershell
Get-ItemProperty 'HKLM:\SOFTWARE\Microsoft\Windows\CurrentVersion\Uninstall\*','HKLM:\SOFTWARE\WOW6432Node\Microsoft\Windows\CurrentVersion\Uninstall\*' -ErrorAction SilentlyContinue | Where-Object { $_.DisplayName -match 'TUCam|Tucsen' } | Select-Object DisplayName, DisplayVersion
```

---

## 3. Ask the SDK directly (the ground truth)

```bash
python -c "from gui.widgets.tucam_backend import TUCamBackend; print(TUCamBackend.enumerate())"
```

* `[]` → the SDK loaded but sees no camera. Continue to step 4 (install the driver).
* `[{'id': '0', 'displayname': '...'}]` → **already working**, skip to step 5.

This is authoritative: the SDK returning SUCCESS with zero cameras means the software is
fine and the device is not reachable.

---

## 4. Driver install — hand this to the operator

Give them this, with the real bundle path, and tell them it must be an **elevated**
PowerShell ("Run as administrator"):

```
powershell -ExecutionPolicy Bypass -File C:\dev\MEBP\tools_install_tucsen_sdk.ps1 -Bundle "<path to the vendor zip>"
```

Add `-WithApp` only if they want Tucsen's own TUCam viewer (~360 MB) for sanity-checking
outside MEBP. **MEBP does not need it** — the app uses the vendored DLL.

The script refuses politely if not elevated, verifies the installer's Authenticode signature
(expect **Valid**, `CN=FUZHOU TUCSEN PHOTONICS CO., LTD.`) and SHA256, installs, then reports
how many cameras the SDK can see. A reboot is recommended afterwards.

Expected result: driver `Windows Driver Package - Tucsen (TUUSB3) Image 2.1.6.1`
(`oem81.inf` / `tuusb3.inf`), signed by *Microsoft Windows Hardware Compatibility
Publisher* — a current WHQL signature, so **no HVCI / Memory-Integrity problem** of the kind
the Nikon Ti driver hit on this rig. If the device nevertheless fails to start, check the
CodeIntegrity event log **before** assuming a software fault.

Then repeat step 3. It should now report a camera.

---

## 5. ⭐ VERIFY THE PROPERTY MAP ON THIS CAMERA — do not skip

**This is the most important step, and the reason this document exists.**

The `TUIDP_*` / `TUIDC_*` constants in `tucam_backend.py` were measured on a **Libra 25**.
A different Tucsen model can lay them out differently. On the first machine the
pre-measurement guesses were wrong in a way that was **not** benign: exposure pointed at
the *temperature* property, so "set exposure" would have written a cooling setpoint.

Critically: **the `GetAttr` safety gate does not catch this.** It protects against an ID the
camera does not *implement*. It cannot protect against an ID that exists but *means
something else*. Only measuring does.

```bash
python -c "from gui.widgets.tucam_backend import TUCamBackend; c=TUCamBackend(); c.open('0'); print(c.diagnostics()); c.release()"
```

Compare against what the Libra 25 reported:

| id | expected meaning | Libra 25 range | default |
|---|---|---|---|
| 0 | GLOBALGAIN (a **mode**, not a %) | 0..3 | 2 |
| **1** | **EXPOSURETM — milliseconds** | 0.0063..5.76e6 | 5.2336 |
| 3 | BLACKLEVEL | 0..255 | 8 |
| 4 | TEMPERATURE | 500..1000 | 500 |
| 8 | GAMMA | 1..255 | 100 |
| 9 | CONTRAST | 0..255 | 128 |
| 10 / 11 | LFT / RGT levels (14-bit) | 0..16382 / 1..16383 | |

`diagnostics()` labels every ID it recognises, so this is a direct comparison. Reference
output from the ME3B Libra 25 (call `read()` once first so the frame line is populated):

```
model:      Libra 25
resolution: 2600x2048  (index 1)
res list:   [(5200, 4096), (2600, 2048)]
res capa:   0
frame:      channels=1 elem_bytes=2

PROPERTIES (id: min .. max, default = current)
    0: 0 .. 3, dft=2 step=1 = 2.0  GLOBALGAIN (gain mode)
    1: 0.00630556 .. 5.75512e+06, dft=5.23361 step=0.00630556 = 5.23  EXPOSURETM  <-- exposure, ms
    3: 0 .. 255, dft=8 step=1 = 8.0  BLACKLEVEL
    4: 500 .. 1000, dft=500 step=1 = 0.125  TEMPERATURE
    8: 1 .. 255, dft=100 step=1 = 100.0  GAMMA
    9: 0 .. 255, dft=128 step=1 = 128.0  CONTRAST
   10: 0 .. 16382, dft=0 step=1 = 0.0  LFTLEVELS
   11: 1 .. 16383, dft=16383 step=1 = 16383.0  RGTLEVELS
   16: 0 .. 3, dft=1 step=1 = 1.0
   42: 0 .. 4990, dft=10 step=10 = 10.0
   43: 500 .. 1000, dft=500 step=1 = 500.0

CAPABILITIES (id: min .. max, default = current)
    0: 0 .. 1, dft=1 = 1  [0=5200x4096(Resolution), 1=2600x2048(Sensitive)]  RESOLUTION
    1: 0 .. 0, dft=0 = 0  [0=High]  PIXELCLOCK
    2: 16 .. 16, dft=16 = 0  BITOFDEPTH
    3: 0 .. 1, dft=0 = 0  ATEXPOSURE  <-- auto-exposure
    4: 0 .. 1, dft=0 = 0  HORIZONTAL (mirror)
    5: 0 .. 1, dft=0 = 0  VERTICAL (flip)
    8: 0 .. 3, dft=0 = 0  ATLEVELS (auto levels, NOT auto-exposure)
   ...
   37: 0 .. 1, dft=0 = 0  [0=1x1Normal, 1=2x2Bin_Sum]
```

Note property 4 (TEMPERATURE) reading **0.125**, outside its own declared 500..1000 range —
that is why `get_temperature()` returns `None` unless the value is in range, rather than
printing a fabricated °C. Unlabelled ids (16, 42, 43, and capabilities 10/15/19/23/25/26/30)
are unidentified and unused by the app; that is expected.

**If the layout matches, you are done verifying — proceed.**

**If it differs**, do not edit constants based on the range shapes alone. Confirm by
consequence, the way it was done originally:

* **Exposure** — set a candidate property and measure the **frame interval**. A real
  exposure property changes it. On the Libra 25: 10 ms → 0.035 s, 300 ms → 0.228 s. A
  readback alone looks perfectly healthy on a wrong-but-writable property, which is exactly
  how the original error survived.
* **Auto-exposure** — enable a candidate capability and watch whether the exposure property
  starts **self-adjusting**. On the Libra 25, capability 3 drove it 20 → 44.5 → 145.8 → 200 ms
  while capability 8 left it pinned. A `0..3` range is a hint it is *not* the boolean you want.

Then update the constants **and** the table in
`coding plans/Update plans/MEBP_v79_TUCSEN_LIBRA_CAMERA.md`, and check whether
`tests/test_v79_tucsen_libra_camera.py::TestHardwareConfirmedMapping` needs to become
model-aware rather than asserting one layout. Report the difference to the operator — a
second model with a different map is a design decision, not a silent edit.

---

## 6. Verify frames and exposure

```bash
python -c "
import time
from gui.widgets.tucam_backend import TUCamBackend
c = TUCamBackend(); print('open ->', c.open('0'))
print('model      :', c._model)
print('resolution :', c.get_resolution(), c.get_resolution_list())
print('exp range  :', c.get_exposure_time_range())
t0 = time.time(); f = None
while time.time() - t0 < 15:
    ok, f = c.read()
    if ok and f is not None: break
    time.sleep(0.2)
print('frame      :', None if f is None else (f.shape, str(f.dtype)))
print('channels/elem:', c._last_channels, c._last_elem_bytes)
print('put 25000us  ->', c.put_exposure_time(25000), '| read', round(c.get_exposure_time(), 1))
c.release()
"
```

Check:

* A frame arrives, shape `(H, W, 3)`, dtype `uint8`.
* `channels` is 1 (mono) or 3 (colour) — **whichever it is, is fine**; the decoder reads the
  format from the frame rather than assuming. Record it.
* Exposure readback lands near what you asked (quantisation to the sensor line time is
  normal — 25000 → 25001.5 µs on the Libra 25). A result **1000× off** means the ms↔µs
  scale is wrong for this model; fix `_exposure_scale_us`, which exists as a single function
  for exactly this reason.
* The image is **not sheared or offset**. Shearing means the row-pitch handling is wrong for
  this sensor; a shifted first row means the header offset is.

---

## 7. Verify end-to-end in the app

```bash
QT_QPA_PLATFORM=offscreen python -c "
from PySide6.QtWidgets import QApplication
app = QApplication.instance() or QApplication([])
from gui.widgets.camera_manager import CameraManager
mgr = CameraManager(); mgr.detect_cameras(opencv_indices=[])
print('sources:', [t for t, _ in mgr.available_sources])
"
```

You should see a **`Tucsen: <model>`** entry. If not, the camera is not being detected —
go back to step 3; it is not a UI problem.

Then in the real app: **Hardware Setup → Cameras → Detect Cameras**, assign the Tucsen
source to a slot, Start, and confirm a live image.

> The source list is only rebuilt by **Detect Cameras** (or at startup). A camera plugged in
> after launch will not appear until you re-detect. On the first machine the operator
> reasonably reported "I don't see an option for the tucam" when the camera simply was not
> attached yet.

---

## 8. Calibration — operator's job, at the scope

Not something you can do headlessly.

1. Assign the camera the **MICROSCOPE** role and calibrate **µm/px + orientation** in
   Hardware Setup → Cameras.
2. MICROSCOPE is a **singleton** role: assigning the Tucsen displaces whatever held it
   (on ME3B, the ANDOR Zyla).
3. Calibration is keyed by **device identity**, so each camera keeps its own µm/px — confirm
   by switching back and forth that neither is lost.
4. Changing capture resolution changes µm/px. If you switch to the full-resolution mode
   (e.g. 5200×4096 on a Libra 25), **recalibrate**. The code logs a warning when you do.

---

## Troubleshooting

| Symptom | Meaning | Action |
|---|---|---|
| `enumerate()` → `[]` but DLL loads | SDK fine, camera not reachable | Driver (step 4), then cable/port/power |
| No Tucsen option in the app | Camera not detected, or list is stale | Step 3, then **Detect Cameras** |
| Device shows `VID_0000&PID_0001`, Code 43 | USB enumeration failed outright | Power / cable / rear USB3 port. **Not** a driver or software issue |
| Device present but Code 28 | Driver not installed | Step 4 |
| Device present, driver installed, still Code 39/failed start | Possibly HVCI | Check CodeIntegrity events. The Tucsen driver is WHQL-signed so this is *not* expected |
| Live view goes **black** after a restart | The redundant-write bug is back | See below — this should already be fixed |
| Exposure changes do nothing | Exposure ID wrong for this model | Step 5 |
| Exposure off by ~1000× | ms↔µs scale wrong | `_exposure_scale_us` |
| Image sheared / skewed | Row pitch (`uiWidthStep`) mishandled | Check `_decode_frame` against this sensor |

### The black-preview bug, for context

On the Libra 25, writing a capability the value it **already holds** is *not* a no-op — it
reset exposure to the sensor minimum (6.3 µs). Because `hardware_setup._apply_hw_controls`
restores `auto_exposure` from persisted settings on every camera start, and that value was
read *from* the camera, it normally matches — so the preview went black on **every startup**.

`_capa_set` now skips any write that would not change the value. If you ever see a black
preview after a restart, verify that guard is intact (`tests/…::test_redundant_capability_write_is_not_sent`)
before looking anywhere else.

---

## Things not to do

* **Don't** edit the `TUIDP_*` / `TUIDC_*` constants from range shapes alone. Confirm by
  consequence (step 5). The original error came from a plausible-looking mapping.
* **Don't** trust a readback as proof a property is what you think. A wrong-but-writable
  property reads back perfectly.
* **Don't** add a `subprocess` call to the camera-detection path. It runs on the GUI thread;
  the model name deliberately uses `winreg` (~0.1 ms) instead of PowerShell (~1 s) for this
  reason.
* **Don't** report success on the strength of the unit tests alone. They run against a fake
  SDK and pass with no camera attached — by design.
* **Don't** install the 360 MB TUCam application unless the operator asks. The vendored DLL
  is what MEBP loads.

## When you are done, report

The model name, `channels`/`elem_bytes` (mono vs colour), the resolution list, the exposure
range, whether the property map matched the Libra 25, and anything you changed. If the map
differed, say so prominently — it affects whether the constants can stay shared between
machines.

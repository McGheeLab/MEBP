# MEBP v7.9.x — Tucsen Libra 25 camera support (TUCam SDK backend)

**Operator request (2026-08-04):** *"I have another camera to install. Its the Libra 25
and it should be connected to the computer now. and here is the zip file that came with
it."* — `E:\Dhyana&FL&Libra16-18-22-25&Aries16_20251120.zip`

**Operator decisions (AskUserQuestion, 2 questions):**
1. **Install approach = "Both"** — install the vendor driver + the TUCam application, AND
   vendor `TUCam.dll` into the repo so the app never depends on the install staying
   present.
2. **Role = "an alternate microscope camera, and I'm evaluating it for this system"** — so
   it must be selectable in the MICROSCOPE role *alongside* the current microscope camera
   (the ANDOR Zyla), and switching between them must not destroy either one's calibration.

---

## ✅ STATUS: HARDWARE-VERIFIED AND WORKING (2026-08-04)

The camera came up once the driver was installed, and everything below was then verified
live on the real Libra 25.

| Check | Result |
|---|---|
| Device present? | **Yes.** `Libra 25` on `USB\VID_5453&PID_E437`, class Image, Status **OK**. |
| Driver installed? | **Yes.** `Windows Driver Package - Tucsen (TUUSB3) Image 2.1.6.1` (`oem81.inf` / `tuusb3.inf`), signed by *Microsoft Windows Hardware Compatibility Publisher* — so none of the HVCI / Memory-Integrity trouble the Nikon Ti driver hit. Plus `Tucam Camera Driver V2.1.6.1` and `TUCam_SDK 2.0.8.0`. |
| SDK sees it? | **Yes.** `TUCAM_Api_Init` → SUCCESS, `uiCamCount = 1`. |
| Live frames? | **Yes.** `2600x2048`, **mono 16-bit** (`channels=1, elem_bytes=2`), decoded to BGR8 `(2048, 2600, 3)` uint8. |
| End-to-end in the app? | **Yes.** Detected as source **"Tucsen: Libra 25"**, identity `tucam:0`, opened through `CameraManager`, frames reached the widget's raw cache, exposure set and read back (15000 → 15000.9 µs), clean stop. |

Being mono confirms the shared-display-math decision: the Libra and the Zyla are *both*
mono, so they render through one identical conversion and an A/B compares sensors rather
than our code.

### The earlier "not reachable" state, for the record

Before the driver install this was blocked, and it was correctly diagnosed as **not** a
software fault: no Tucsen device on the bus, no driver installed, and the genuine SDK
returning SUCCESS with `uiCamCount = 0`. Two red herrings were ruled out then and are worth
keeping written down, because both are still present on this machine:

* **`USB\VID_0000&PID_0001` "Port Reset Failed" (Code 43)** on root-hub port 6 — a device
  whose descriptors Windows cannot read at all, i.e. failed USB enumeration, which no
  driver install fixes. It predates this camera (`FirstInstallDate` 07/31) and is **not**
  the Libra.
* **"HD USB Camera" `VID_32E4&PID_0577`** — a generic **UVC** device on Microsoft's
  `usbvideo` driver, installed 07/24. Tucsen scientific cameras are not UVC.

The Libra's real identity, now known: **VID `0x5453`** (which the SDK also reports as info
id 2 = `21587`) and PID `0xE437`.

---

## ⭐ WHAT HARDWARE VERIFICATION CHANGED — the guessed mapping was WRONG

This is the part that could not be done without the camera. It found **two real defects and
one hardware quirk**, and all three were caught by *measuring a consequence* rather than by
trusting a readback.

### 1. 🐞 Exposure was pointed at the TEMPERATURE property

The pre-bench table had `TUIDP_EXPOSURETM = 4`. On this camera **property 4 is
TEMPERATURE** (declared range 500..1000), so "set exposure" would have written a cooling
setpoint and exposure would never have changed.

**The `GetAttr` gate did NOT catch this.** Property 4 genuinely exists, so it passed
validation. That gate protects against an ID a model does not *implement*; it cannot
protect against an ID that means something *else*. Only reading real values did.

The enum is sequential from 0. Measured on this camera:

| id | meaning | range | default | notes |
|---|---|---|---|---|
| 0 | GLOBALGAIN | 0..3 | 2 | a gain **mode** selector, not a percentage |
| 1 | **EXPOSURETM** | 0.0063..5.76e6 **ms** | 5.2336 | ← the real exposure |
| 3 | BLACKLEVEL | 0..255 | 8 | |
| 4 | TEMPERATURE | 500..1000 | 500 | **reads 0.375** — outside its own declared range |
| 8 | GAMMA | 1..255 | 100 | |
| 9 | CONTRAST | 0..255 | 128 | |
| 10 | LFTLEVELS | 0..16382 | 0 | 14-bit black point |
| 11 | RGTLEVELS | 1..16383 | 16383 | 14-bit white point |

BRIGHTNESS (2) is **absent** on this model and is correctly not advertised: the app offers
`exposure_us`, `exposure_gain_pct`, `gamma`, `contrast`, `auto_exposure` and the three mono
display-scale controls, with no dead brightness slider.

**Exposure = property 1 in milliseconds was proven by a physical consequence:** commanding
10 ms gave a 0.035 s frame interval, 300 ms gave 0.228 s. A readback alone would have looked
perfectly healthy on a wrong-but-writable property.

Because the SDK contradicts itself on temperature (declares 500..1000, reads 0.375),
`get_temperature()` now returns **None** unless the value lies inside the range the SDK
itself declared — reporting "unknown" beats printing a fabricated °C.

### 2. 🐞 Auto-exposure was pointed at auto-LEVELS

`TUIDC_ATEXPOSURE` was 8, whose range is **0..3** — which should have been the tell for
something boolean. Proven the same way: enabling capability **3** made the exposure property
self-adjust (20.0 → 44.5 → 145.8 → 200.0 ms, converging on a target), while enabling
capability 8 left it pinned at 20.001 ms. Capability 8 is auto-**levels**, hence 0..3.

Measured capabilities: `0` RESOLUTION `[0=5200x4096(Resolution), 1=2600x2048(Sensitive)]` ·
`1` PIXELCLOCK `[0=High]` · `2` BITOFDEPTH `16..16` · **`3` ATEXPOSURE `0..1`** ·
`4` HORIZONTAL · `5` VERTICAL · `8` ATLEVELS `0..3` · `37` binning
`[0=1x1Normal, 1=2x2Bin_Sum]`.

Two things the empirical resolution discovery got right on real hardware: it found
capability 0 from that capability's own `WxH` value-texts (so the guessed
`TUIDC_RESOLUTION` was never relied on), and it did **not** mistake capability 37's
`1x1Normal` / `2x2Bin_Sum` binning labels for resolutions — the `\d{2,6}`
minimum-two-digit requirement is what prevents that, and there is now a test for it.

### 3. 🐞🐞 A redundant capability write BLACKS OUT the preview

The worst of the three, and only findable on hardware. Writing a capability the value it
**already holds** is not a no-op on this camera:

```
put exposure 30000us                          -> exposure = 30001.8 us   auto=False
set_auto_exposure(False)   # it was ALREADY False
                                              -> exposure =     6.3 us   <-- sensor minimum
```

A genuine transition (True → False) preserves exposure correctly. Only the **redundant**
write is destructive.

**This was reachable on a completely ordinary path.**
`hardware_setup._apply_hw_controls` restores `auto_exposure` from the persisted
`hw_controls` every time a camera starts — and that stored value was itself read back *from
the camera*, so it normally **equals** the current value. The live preview would have gone
black on every startup, and the cause would have looked like anything but a redundant write.

Fixed in `_capa_set`: a write that would not change the value is skipped entirely. That is
semantically correct regardless (setting a value to what it already is means nothing), so it
guards **every** capability rather than special-casing auto-exposure. Verified on hardware:
redundant writes now preserve exposure, real transitions still work, and
`set_auto_exposure(get_auto_exposure())` — the exact startup-restore shape — is a no-op.

### 4. The model name has to come from the OS

This SDK build returns **no model text at all**: every string info id came back empty via
both the open handle and `GetInfoEx`-by-index, and only numeric fields are populated (info
id 2 = `21587` = the VID, id 3 = `58423` = the PID). Windows, however, names the device
exactly "Libra 25".

So `_read_model` tries the SDK's text ids first (a future model may provide them) and
otherwise reads the name from the registry: `DeviceDesc` is stored as an INF reference
(`@oem81.inf,%vid_5453&pid_e437.devicedesc%;Libra 25`) whose display text follows the final
`;`. Read via `winreg`, **not** by shelling out to PowerShell — camera detection runs on the
GUI thread, and a subprocess there is the freeze class this repo has fixed several times
already. Measured **0.0001 s** for the registry read against roughly a second for a
PowerShell spawn, and it is cached per process. Label-only: a camera's identity never
depends on it.

*(A first attempt did use PowerShell, which announced itself immediately — the test suite
went from 2.4 s to 22 s, and fake-SDK tests started inheriting the real camera's name. The
tests now pin the lookup so results never depend on what is plugged in.)*

---

## Objective

Add a Tucsen (TUCam SDK) camera backend so a Libra 25 can be assigned to a CameraWidget
slot and used in the MICROSCOPE role, as an A/B alternative to the ANDOR Zyla, without
disturbing any existing camera.

## Files modified

| File | Change |
|---|---|
| `DLLs/tucsen dlls/` **(new, 27 MB, 10 DLLs)** | Vendored SDK: `TUCam.dll` v2.0.8.0 + the three `tuimgcv_*` libs it imports + `msvcp120`/`msvcr120`/`vcomp120` (VC++ 2013 runtime) + `MultiCam`/`phxlx64`/`clallserial` (CameraLink, lazily loaded). Contents chosen by reading `TUCam.dll`'s PE import table, not by copying the whole kit. Mirrors the existing tracked `DLLs/zyla dlls/`. |
| `gui/widgets/mono_display.py` **(new)** | `_auto_levels` + `_mono_to_bgr8` + `LEVEL_MAX`, extracted **verbatim** from `andor_backend`. Pure, GUI-free, no SDK. |
| `gui/widgets/andor_backend.py` | Those two functions replaced by a re-export from `mono_display`. Behaviour unchanged; `andor_backend._mono_to_bgr8` still resolves (existing tests reference it by that path). |
| `gui/widgets/tucam_backend.py` **(new, ~1000 lines)** | The backend. |
| `gui/widgets/camera_widget.py` | Guarded import + `TUCAM_AVAILABLE` in `CAMERA_AVAILABLE`; `detect_tucam_cameras()`; `_tucam` slot state; combo entry `"Tucsen: <model>"`; sync `_start_tucam()` + async open branch + adopt + release; `stop()` releases the stream; `hardware_capabilities` / `get_hw_settings` / `log_hw_settings` / all six `set_hw_*` delegates; new `_mono_display_backend()`. Also de-duplicated `set_capture_resolution` and `_grab_frame`, whose ToupCam and Andor branches were byte-identical copies. |
| `gui/widgets/camera_manager.py` | `detect_tucam_cameras` folded into the shared probe inventory (guarded, like Andor). |
| `gui/widgets/camera_identity.py` | `tucam:<index>` identity, both directions. |
| `gui/dialogs/camera_settings_dialog.py` | `tucam` accepted as a controllable source; reports display scale, sensor temperature and the live frame format. |
| `gui/pages/hardware_setup.py` | `tucam` added to the `hw_controls` persistence gate; Detect-Cameras tooltip mentions Tucsen. |
| `tools_install_tucsen_sdk.ps1` **(new)** | Elevated driver (+ optional app) install with verification. |
| `tests/test_v79_tucsen_libra_camera.py` **(new, 74 tests)** | The suite. |

## Implementation steps

- [x] Inspect the vendor bundle **without installing** (96 entries: driver exe, 360 MB TUCam app, Micro-Manager adapter, manuals, CAD).
- [x] Establish hardware/driver reality first (device enumeration, driver presence, real-SDK camera count) before writing code.
- [x] Confirm the SDK supports "Libra 25" from the DLL's own model string table.
- [x] Determine the API surface from Tucsen's own Micro-Manager adapter (`mmgr_dal_Tucsen_x64.dll`) rather than guessing.
- [x] Vendor the SDK working set (from `TUCam.dll`'s PE imports).
- [x] Extract the shared mono→BGR8 display math; confirm the 45 Andor tests still pass.
- [x] Write `tucam_backend.py`; verify against the **real** DLL.
- [x] Wire into widget / manager / identity / settings dialog / hardware setup.
- [x] Write the test suite; mutation-verify the load-bearing behaviours.
- [x] Write the elevated install script; verify it parses and refuses politely unelevated.
- [x] **Driver installed; verified end-to-end on the real Libra 25.**
- [x] **Correct the property/capability mapping from measured values** (see above).
- [x] Targeted regression + `gui.app` import smoke.
- [ ] Calibrate µm/px + orientation for the MICROSCOPE role, and confirm a Libra↔Zyla swap loses neither calibration (operator, at the scope).

---

## Design decisions

### 1. The SDK's own answers are the authority, not our table

The property/capability IDs came from the published TUCam API, and **they were wrong** (see
above). The structures that limited the damage, and that should be kept:

* Every property access is gated on **`TUCAM_Prop_GetAttr`** first, so an ID a model does
  not implement degrades to "control unavailable" rather than to a wrong read/write.
  `hardware_capabilities()` therefore advertises only controls this camera actually has.
  **Caveat learned the hard way:** this cannot catch an ID that exists but *means something
  else* — which is exactly what happened with exposure→temperature. Only measuring does.
* Setters **clamp** into the SDK's declared range. Right for a continuous camera property —
  deliberately asymmetric with the microscope turret, which must *refuse* an out-of-range
  slot (a mistyped discrete selector silently moving is the worse failure).
* `diagnostics()` sweeps every property + capability ID and dumps real
  min/max/default/current plus decoded value-texts. **Run it first on any new Tucsen model.**
  It is what turned this from a guess into a measurement in one session.

### 2. Resolutions are discovered, not assumed

`_discover_resolutions()` sweeps capability IDs and keeps the one whose selectable values'
**own text labels** parse as `WxH` via `TUCAM_Capa_GetValueText`. Self-verifying, and on
real hardware it found capability 0 and correctly ignored capability 37's binning labels.
Falls back to the single frame-reported size if nothing qualifies, so the camera still
streams.

### 3. Pixel format is read from the frame, never assumed

`_decode_frame` takes `ucChannels`, `ucElemBytes`, `ucDepth`, `usWidth`, `usHeight`,
`uiWidthStep` and `usHeader` from the descriptor the SDK just filled in — so mono-16,
RGB-24 and BGRA-32 all decode, and it did not matter that the Libra 25's mono-ness was
unknown when the code was written. Two details are easy to get wrong and are
mutation-tested: padded row pitch (a wrong stride *shears* the image) and the in-band
header offset.

### 4. Exposure: ms in the SDK, µs in this app

One conversion point (`_exposure_scale_us`), with the reported range derived from the SDK's
own min/max. Confirmed on hardware.

### 5. One mono→BGR8 conversion, shared with the Andor backend

The operator A/Bs the Libra against the Zyla, and **both turned out to be mono**. Converting
a mono sensor to 8-bit for display is a *display* decision (per-frame percentile auto-scale
vs. fixed levels) that materially changes how bright and contrasty a camera looks. Two
private copies could drift, and the comparison would then measure our display code instead
of the sensors. So `_auto_levels` / `_mono_to_bgr8` moved to `gui/widgets/mono_display.py`
and both backends import it; `andor_backend` re-exports them under the original names, so
its behaviour and every existing reference are unchanged (45 Andor tests green after the
move). A test pins that all three modules resolve to the *same function object*.

### 6. `andor_*` control keys reused, deliberately

The mono display-scaling controls travel under the historical keys `andor_auto_scale` /
`andor_scale_lo` / `andor_scale_hi`. Those are what the settings dialog gates on and what
`hw_controls` persists, and the Andor suite pins them across ~15 assertions — renaming would
churn a hardware-verified suite for a cosmetic win. So the Tucsen backend reuses that proven
plumbing, `get_settings()` emits **both** the correctly-named `mono_*` keys and the
`andor_*` aliases, and `camera_widget._mono_display_backend()` resolves whichever mono
backend is live so the three delegates don't duplicate a branch per camera. Retiring the
aliases is a follow-up.

### 7. `pText` is `POINTER(c_char)`, not `c_char_p`

Found while testing. The two are ABI-identical for a C `char*`, but ctypes coerces a
`c_char_p` **field** to an immutable `bytes` on read — discarding the pointer and making the
out-parameter unusable and unverifiable. `POINTER(c_char)` keeps it a real pointer, which is
both truer to the C signature and what lets the fake exercise the real string-out path.

### 8. `TUCAM_Api_Init` is refcounted

It is process-global: `enumerate()` and every `open()` share it. Uninit'ing while another
camera streams would kill that stream — the same non-refcounted-global hazard this repo hit
with the position poller's suspend flag and, in v7.9, with `_print_floor_active`. A test
opens two cameras and asserts `Api_Uninit` fires only on the **last** release.

### 9. Teardown order

`Buf_AbortWait` → **join the reader** → `Cap_Stop` → `Buf_Release` → `Dev_Close` → API
release. Releasing the buffer while the reader is still inside `WaitForFrame` would hand the
SDK a freed pointer.

### 10. Only the driver is actually required

MEBP loads the **vendored** `TUCam.dll`, so the 360 MB TUCam application is optional — a
vendor reference viewer for sanity-checking outside the app. The install script installs the
driver by default and the app only with `-WithApp`.

### 11. The install script is pure ASCII on purpose

Windows PowerShell 5.1 reads a BOM-less `.ps1` as ANSI, and a UTF-8 em-dash decodes as
CP1252 ending in `0x94` = a RIGHT DOUBLE QUOTE — which terminated a string and broke the
parse. (The existing Nikon script survives its own non-ASCII only because all of it sits in
comments, which the parser skips.)

---

## Testing notes

**New:** `tests/test_v79_tucsen_libra_camera.py` — **74 tests**, all green in ~2.3 s. A
`FakeTUCamLib` stands in for `TUCam.dll` at the ctypes boundary, so **the production backend
runs unmodified**: device open, buffer alloc, capture start, the reader thread,
`Buf_WaitForFrame`, the frame decode and the mono→BGR8 conversion are all real code.

Coverage: availability / DLL discovery / graceful degradation with no SDK · enumeration +
model resolution + API-ref hygiene on refusal · stream lifecycle, teardown order, reader
join, double-release · frame decode (mono16→BGR8, **padded row pitch**, **in-band header
offset**, colour 24-bit, 4-channel alpha drop, geometry from the frame, `read()` returns a
copy) · exposure µs↔ms both directions + range conversion + garbage rejection · property
gating (unsupported ID reads `None` and writes nothing; clamping; auto-exposure;
diagnostics lists only implemented IDs) · resolution discovery / auto-pick / explicit index
/ stream restart / no-capability fallback · display scaling (shared-function identity,
default on, freeze-on-toggle-off, clamping + ordering, levels change the rendered image) ·
settings readback + both key namings · API refcounting · identity round-trip · **source-list
visibility** (the operator-visible "is there a Tucsen option?" chain) · CameraWidget wiring
· and a `TestHardwareConfirmedMapping` class pinning every fact measured on the real camera.

**Mutations confirmed CAUGHT** (each reverted → the suite fails):
row pitch ignored · `usHeader` offset dropped · exposure ×1000 conversion removed ·
`GetAttr` gate removed · API refcount broken · Tucsen combo entry deleted ·
**exposure ID reverted to 4 (the temperature bug)** ·
**the redundant-capability-write guard removed (the black-preview bug)**.

**Regression, all green, run per-suite:** simulated-camera (23) · objective-cal (26) ·
andor-display-scaling (24) · andor-zyla (21) · camera-async-open (8) · cal-liveview (10) ·
camera-cal-store (30) · camera-hw-controls (22) · image-correction (18) · rotation (9) ·
rotation-cal-and-monitor (43) · scale-fov (55) · mapping-orient (13) · reanchor (21) ·
picker-scaling (12) · calibration-revision (20) · suite-hygiene (8) · tucsen (74)
= **437 green**, plus `import gui.app` OK.

**Not a failure:** `tests/test_toupcam_chain.py` reports "NO TESTS RAN" — it is a hand-run
diagnostic script with no `TestCase` class, pre-existing and untouched here.

---

## Remaining bench work (operator, at the scope)

1. **Calibrate µm/px + orientation for the MICROSCOPE role** via Hardware Setup → Cameras.
   MICROSCOPE is a singleton role, so assigning the Libra displaces the Zyla from it.
2. **Confirm a Libra↔Zyla swap loses neither calibration.** Calibration is keyed by device
   identity (`tucam:0` vs `andor:<serial>`), so each should keep its own µm/px — that
   property is what makes the A/B evaluation workable and is worth confirming explicitly.
3. **A/B the two cameras** in both display-scaling modes; the shared conversion means any
   difference you see should be the sensors, not our code.
4. Decide whether the **5200x4096 full-resolution mode** is wanted for mosaics (the preview
   auto-picks 2600x2048; changing capture resolution invalidates µm/px, and the code logs
   that).

## Known gaps / follow-ups

* **`tucam:<index>` identity is positional, not port-stable.** The SDK opens by device
  *index*, unlike the DirectShow path which encodes hub/port. Fine for one Tucsen camera;
  with two, calibrations could swap if enumeration order changed. The SDK exposes VID
  (`0x5453`) and PID (`0xE437`) but **no per-unit serial** was found, so a genuinely stable
  key would have to come from the OS device instance path. Worth doing only if a second
  Tucsen camera is added.
* **`hardware_setup`'s bulk `hw_controls` save omits the display-scale keys** (it saves
  exposure/gain/gamma/brightness/contrast/resolution). Those keys *are* persisted by the
  camera settings dialog's own save path and *are* restored by `_apply_hw_controls`.
  Pre-existing for the Zyla, not introduced here.
* **Retire the `andor_*` control-key aliases** in favour of `mono_*` when there is a reason
  to touch that plumbing (decision 6).
* Property IDs 5/6/7 (SHARPNESS / NOISELEVEL / HDR_KVALUE) and 10/11 (LFT/RGT levels) are
  named from the sequential enum but **not individually confirmed**, since this camera
  either lacks them or the app does not use them. `diagnostics()` will show them.
* The optional TUCam application's installer type is unconfirmed; the script tries a silent
  install and falls back to telling the operator to run it interactively rather than
  reporting a false failure.
* **Process note, disclosed:** the first attempt to update this document used
  `open(path, "w")` and then raised mid-write on a bad unicode escape — which left the file
  **truncated to zero bytes**, because `"w"` truncates before the write. It was untracked, so
  there was no git copy and it had to be rewritten from scratch. Build the content first,
  then write.

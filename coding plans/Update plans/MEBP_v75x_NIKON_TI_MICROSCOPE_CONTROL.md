# MEBP v7.5.x — Nikon Ti Eclipse body control (filter cubes / focus / objectives)

## Objective

Connect the Nikon Ti Eclipse to the GUI and give the operator **manual** control
of its three motorized devices from the jog panel:

1. **Filter cubes** — assign a name to each cassette slot, see which cube is
   currently in the light path, and switch.
2. **Focus (Z)** — raise and lower the focal plane.
3. **Objectives** — switch the nosepiece and see which objective is in.

Explicitly scoped by the operator: *"I want to keep this all separated and
manual for now, but later we will integrate these motors into all of the
workflows and calibrations."* So **nothing** in the print, workflow, or
calibration paths drives these turrets — this lands the hardware layer and the
manual surface, with the integration points deliberately left for a later pass.

---

## Design

### Why a backend abstraction (and not a direct SDK call)

The Ti body is not a serial device with an ASCII protocol like the Prior stage or
the Marlin board — it is driven either through Nikon's own **Ti SDK COM
automation object** or through **Micro-Manager's `NikonTI` device adapter**.
Which one is available depends on what is installed on the rig. So the driver is
a swappable backend:

| Backend | Identifier | Notes |
|---|---|---|
| Simulated | `simulated` | Always available. Models both turrets + a focus axis so the whole UI is exercisable and testable with no hardware. |
| Nikon Ti SDK | `nikon_ti` | `comtypes` → `Nikon.LvMic.NikonTi` (or `…NikonTi2`). **Mapping unverified against hardware** — see below. |
| Micro-Manager | `micromanager` | `pymmcore` + an MM configuration. Recommended when MM is installed: the adapter is long-proven against Ti bodies, state devices report their own labels, and its focus device is **µm-native** (no unit factor to get wrong). |

### ⚠ The Nikon SDK device mapping is UNVERIFIED

`NikonTiSdkBackend` is written from the SDK's documented shape — each device is a
COM object with a `Position` property whose `Value` is the 1-based turret index;
the Z drive counts in device units. It has **never been run against a real
body**. Two mitigations rather than a silent guess:

* Every access **probes** several attribute spellings (`FilterBlockCassette1` /
  `FilterBlockCassette` / …, `Position.Value` / `Position`) and raises a *named*
  `MicroscopeError` instead of failing obscurely.
* `diagnostics()` dumps the live COM object model — every device and property the
  body actually exposes, plus each current position. Running it once at the
  bench (Microscope Setup → **Diagnostics…**) is enough to finish or correct the
  mapping in a single pass, with no guess-and-recompile loop.

**`z_units_per_um` is a per-machine setting, not a constant** (default 100, i.e.
a 10 nm Ti Z step). It scales *every* focus move, so it is bench-verifiable
(command 100 µm, measure it) and correctable without a code change — the same
reasoning that makes plate orientation and Z polarity per-machine settings
elsewhere in this codebase.

### Why one dedicated worker thread

`MicroscopeController` serializes **every** backend call onto a single worker
thread, for two independent reasons:

1. **COM apartment affinity** — a COM object must be used on the thread that
   created it. One thread for connect *and* all subsequent calls satisfies this
   by construction (the same affinity concern already noted for the ToupCam /
   Andor camera backends).
2. **Never block the Qt event loop** — a turret rotation is a multi-hundred-
   millisecond blocking call. This codebase has had to fix exactly that class of
   freeze three times (`PUMP_JOG_OFF_GUI_THREAD`, `STAGE_JOG_OFF_GUI_THREAD`,
   `JOG_TRAVEL_OFF_GUI_THREAD`). Doing it right the first time here.

The widget never calls hardware directly and never registers a cross-thread
callback: it **polls an immutable `MicroscopeState` snapshot** on its own timer.
That removes the listener-outlives-the-widget hazard entirely (a callback firing
into a deleted Qt object), and means a wedged turret can never stall a paint.

Two ordering details that would otherwise produce a stuck UI:

* `busy=True` is published **while holding the queue lock**, so the worker's
  `busy=False` can never land first and leave the panel showing a move that
  already finished.
* Queued ops older than `STALE_OP_S` (20 s) are dropped rather than executed —
  a turret move whose button was clicked 30 s ago is not what the operator wants
  to happen now.

### Why a separate store

Filter-cube and objective assignments live in a **per-machine**
`MicroscopeConfigStore` (`config/hardware/microscope.json`), not on
`HardwareConfig`. `HardwareConfig` is the *swappable print setup* — loading a
setup file from another machine must never carry another rig's cube assignments,
the same reason `CameraCalibrationStore` was split out in
`MEBP_v75x_CAMERA_CAL_PERSIST_STORE.md`.

Slot numbering is **1-based everywhere**, matching the numbers engraved on the
turret, so what the operator reads on the microscope is what the UI shows. The
turret only ever reports a *position number* — "slot 3 holds the mCherry cube" is
knowledge only the operator has, and remembering it is the store's whole purpose.

---

## Files Modified

| File | Change |
|---|---|
| `SupportClasses/MicroscopeControl.py` | **NEW** — GUI-free. `MicroscopeError`, frozen `MicroscopeState`, the `MicroscopeBackend` contract + `SimulatedMicroscopeBackend` / `NikonTiSdkBackend` / `MicroManagerBackend`, `build_backend`, the thread-serialized `MicroscopeController`, and the `get_microscope()` / `shutdown_microscope()` singleton. |
| `SupportClasses/MicroscopeConfigStore.py` | **NEW** — per-machine JSON store: driver selection + driver knobs, filter-cube / objective slot assignments, slot counts, focus step / direction / soft limits. Atomic write, `$MEBP_MICROSCOPE_CONFIG_DIR` override, `get_store()` singleton. |
| `gui/widgets/microscope_panel.py` | **NEW** — `MicroscopePanel`: status + Connect + ⚙, cube combo, objective combo, focus read-out, ▼/step/▲ jog, absolute go-to. Polls the cached state; stops polling while hidden. |
| `gui/dialogs/microscope_settings_dialog.py` | **NEW** — driver selection (stacked per-driver options), slot-assignment grids, focus preferences, **Diagnostics…**. Nothing persists until OK. |
| `gui/widgets/standard_jog_context.py` | Adds the `Card("Microscope")` (between Illumination and Hardware Info) + tick forwarding — so it rides the "Jog" pill everywhere that panel appears. |
| `gui/widgets/context_sections.py` | `MicroscopeSection` + `register_section("microscope", …)` so it can also be dropped into a Custom panel. |
| `gui/app.py` | `closeEvent` releases the body (COM / Micro-Manager) and stops the worker thread — best-effort, so a wedged turret cannot block app close. |
| `tests/test_v75x_nikon_ti_microscope.py` | **NEW** — 56 tests. |

---

## Implementation Steps

- [x] `MicroscopeConfigStore` — assignments, slot counts, focus prefs, atomic persistence
- [x] `MicroscopeBackend` contract + `SimulatedMicroscopeBackend`
- [x] `NikonTiSdkBackend` (probed mapping + `diagnostics()` + unit scaling)
- [x] `MicroManagerBackend` (pymmcore, µm-native focus, state labels)
- [x] `MicroscopeController` — worker thread, cached state, soft-limit clamp, singleton
- [x] `MicroscopePanel` — cubes / focus / objectives
- [x] `MicroscopeSettingsDialog` — driver + assignments + focus prefs + diagnostics
- [x] Wire into `StandardJogContextPanel` and the Custom-panel catalog
- [x] App-shutdown release
- [x] Tests + regression
- [x] **Bench verification on the real Ti-E — 22/22** (turrets, read-back,
      focus read, out-of-range refusal)
- [ ] Focus **movement** on hardware — opt-in `--focus`, needs the objective clear
- [ ] *Deferred by design:* integration into workflows and calibrations

---

## Testing Notes

`tests/test_v75x_nikon_ti_microscope.py` — **56 tests, all green** (0.2 s):

* **Store** — 1-based slots, blank clears, names beyond the slot count are hidden
  but not destroyed, bulk replace, focus prefs incl. swapped-limit normalisation,
  on-disk round-trip, unknown-key / bad-backend tolerance.
* **Simulated backend** — connect gating, range checks, travel clamp.
* **Nikon SDK plumbing** — `Position.Value` and flat-`Position` read/write, a
  clear error when the property is missing, unknown-device handling, and the
  device-units → µm scaling in both directions (the number that scales every
  focus move).
* **Micro-Manager backend** — refuses cleanly with no configuration; reads are
  safe while disconnected.
* **Controller** — state population, turret + focus ops, operator soft-limit
  clamping, errors reported (never raised) and cleared by the next success,
  disconnect clearing positions, reconnect replacing the old backend, and the
  threaded path draining in order with `busy` clearing.
* **Panel** — slot names rendered from the store, selection drives the turret,
  the hardware's own position rendered *without* re-commanding it, focus step +
  direction convention, absolute go-to, inert while disconnected, connect toggle,
  error surfacing.
* **Setup dialog** — loads current config, OK commits, **Cancel leaves the store
  untouched**, typed names survive a slot-count change, opt-in soft limits,
  Micro-Manager config required.
* **Jog panel** — hosts the card and forwards ticks.

Regression, all green: context-panel / illumination / responsive-context /
jog-navigation (**80**), workflow-settings / common-print-settings /
jog-pump-fill / axis-speed ×2 / xz-custom-z (**122**). Full `gui.app` import +
threaded connect → state → shutdown smoke passes.

### Needs real-hardware verification on the Nikon Ti, in this order

1. **Driver** — Microscope Setup → pick `Nikon Ti — Nikon SDK` (or Micro-Manager
   with the rig's `.cfg`) → Connect. The status line should go green.
2. **Diagnostics…** *before touching anything else* — confirm the report names a
   nosepiece, a filter cassette and a Z drive. If any is missing, the report
   lists what this body *does* expose and the aliases in `_TI_DEVICE_ALIASES`
   can be corrected against it.
3. **Read-back** — the two combos should show the slot the turrets are physically
   in. Rotate a turret by hand/NIS and confirm the panel follows within ~1 s.
4. **Switching** — select another cube and another objective; the body should
   move and the read-back settle on the requested slot.
5. **Focus scale** — command a **100 µm** move and measure it. If it is off by a
   constant factor, that factor is `z_units_per_um` (Microscope Setup). This is
   the single most important check on the SDK path.
6. **Focus direction** — if ▲ moves the focal plane the wrong way, clear
   "*Up* increases the focus position" (no recalibration needed).
7. **Soft limits** — optionally set a focus range and confirm the clamp holds.
8. **Responsiveness** — switch a turret while a camera feed is live; the UI must
   stay responsive throughout the rotation (this is what the worker thread buys).

---

---

## Hardware test session — 2026-07-30 (BLOCKED: no driver on this PC)

Attempted the full feature test against the real body. **It cannot be driven
from this computer yet** — not a software fault, a missing driver/SDK.

Evidence gathered, in order:

| Check | Result |
|---|---|
| Nikon COM ProgIDs in `HKEY_CLASSES_ROOT` | **none registered** |
| `CreateObject("Nikon.LvMic.NikonTi")` / `…NikonTi2` (+3 other spellings) | `WinError -2147221005` **Invalid class string** |
| Nikon USB device present? | **YES** — `USB\VID_04B0&PID_7832` ("HUB-A"); VID 04B0 is Nikon Corp |
| …its driver state | **`CM_PROB_FAILED_INSTALL` — Code 28, "The drivers for this device are not installed"** |
| Nikon software installed | only **NIS-Elements *Viewer*** — an ND2 file viewer, no hardware control |
| Micro-Manager install | not found |
| `pymmcore` | not installed |
| Ti SDK DLLs on disk | none |

So the microscope **is physically plugged in and enumerating**, but Windows has
no driver bound to it and no control API is installed. `NikonTiSdkBackend`
refused cleanly with the actionable message it was designed to give — no crash,
no hang — which is the one thing this session *did* verify about the real path.

### To unblock (any one of these)

1. **Nikon Ti SDK / NIS-Elements** (full, not Viewer) — installs the driver and
   registers the `Nikon.LvMic.NikonTi` COM object the `nikon_ti` backend needs.
2. **Micro-Manager** + `pip install pymmcore` — then point the `micromanager`
   backend at a `.cfg` that loads the body. Preferred if available: its adapter
   is proven and its focus device is µm-native.

Either way the driver for `VID_04B0&PID_7832` must install cleanly first —
Device Manager should stop showing Code 28.

### What WAS verified

Every feature was exercised end-to-end through the **real widget and
controller** against the simulated backend — the same code path the hardware
will take, with only the bottom driver layer substituted:

`tools_microscope_hw_check.py simulated --focus` → **26/26 pass**, and a wider
scripted exercise → **35/35 pass**: cube assignment + persistence across a
restart, connect, per-slot cube switching with read-back, per-position objective
switching with read-back, focus up/down at two step sizes, absolute go-to,
direction-convention inversion, operator soft-limit clamping, an impossible slot
reported without moving the turret or crashing, and disconnect disabling the
controls.

This proves the store, controller, threading, widget and state machine. It does
**not** prove the SDK device mapping or `z_units_per_um` — those remain
hardware-gated, which is exactly what `diagnostics()` and the checklist above
exist to close.

### 🐞 Bug found and fixed by testing

**Focus jog buttons silently dropped clicks.** `_render` disabled ▲/▼ whenever
`state.busy` was set, so a second click landing while the first move was still
in flight hit a disabled button and vanished. Caught by the exercise script:
two ▼ clicks after one ▲ netted 0 µm instead of −10 µm.

Repeated small focus steps are *the* most common microscope interaction, and
queued focus deltas are additive and each bounded by the step size — so dropping
the operator's click is strictly worse than queueing it. ▲/▼ now stay live
during a move; absolute go-to and the two turret combos remain busy-gated
(those are not additive, and re-entering a selection mid-rotation is confusing).
Pinned by `test_focus_jog_stays_live_during_a_move` +
`test_repeated_jogs_accumulate`.

Also confirmed deliberate, after the exercise flagged it: a **background refresh
does not clear a previous error**. It fires every ~1 s, so clearing there would
wipe the message before the operator could read it; the next *successful
command* clears it. Pinned by `test_background_refresh_keeps_the_error_readable`.

### Micro-Manager route investigated — 2026-07-30 (does NOT unblock)

Operator asked to install Micro-Manager + `pymmcore` to fix the above.
**`pymmcore` was installed; the full Micro-Manager install was NOT, because the
evidence says it cannot fix this blocker.**

Micro-Manager's own documentation for the adapter is explicit — *"This adapter
uses the driver and API supplied by Nikon"* — and requires Nikon's **Ti Setup
Tool** (drivers + TiControl) from Nikon Healthcare's Software Developer Toolkit
site (registration required), which installs
`C:\Program Files\Nikon\Shared\Bin\NikonTi.dll` and the driver directory
`C:\Program Files\Nikon\Shared\Drivers` that Windows must be pointed at to clear
Code 28. **The MM adapter wraps Nikon's driver; it does not replace it.**

Verified on this machine: `C:\Program Files\Nikon` does not exist, there is no
`NikonTi*.dll` anywhere on `C:`, and no `Shared\Drivers`. So the MM path is
blocked by the *same* missing component as the SDK path. Installing
Micro-Manager (≈200 MB, system-wide) would have changed nothing about
reachability — so it was not installed. `pymmcore-plus` (the pip-based
device-adapter installer) was also declined: it pulls **40 packages** including
botocore, s3fs, aiohttp and tensorstore onto a lab control machine.

**`pymmcore` 12.5.0.75.0 was installed** (777 kB, numpy-only dependency;
confirmed the pinned `numpy==2.4.6` is untouched, so the numba/Andor cap holds)
and added to `requirements.txt` as optional. It is a prerequisite for the MM
route and made the next two findings possible.

#### ✅ Two correctness wins from having real `pymmcore` present

1. **API surface verified.** All 11 `CMMCore` methods `MicroManagerBackend`
   calls — `setDeviceAdapterSearchPaths`, `loadSystemConfiguration`,
   `getLoadedDevices`, `reset`, `getNumberOfStates`, `getState`, `setState`,
   `waitForDevice`, `getStateLabels`, `getPosition`, `setPosition` — exist on
   the real object with compatible arity. A typo would otherwise have surfaced
   mid-session at the bench. Failure paths also confirmed clean: an empty config
   path and a nonexistent `.cfg` both raise `MicroscopeError` with a readable
   message, and reads while disconnected return `None`/`0`/`()` rather than
   raising.

2. **🐞 Wrong device-name defaults corrected.** The adapter's published device
   names are `TINosePiece`, `TIFilterBlock1` and `TIZDrive` (all hanging off the
   `TIScope` hub) — **not** the `Nosepiece` / `FilterBlockCassette1` / `ZDrive`
   that had been guessed from the COM SDK's property names, which are a
   different namespace. Every default now matches the documentation, so the MM
   backend should bind first try. Still operator-editable, since a configuration
   may relabel devices.

### ⭐ Ti SDK redistributable found — and it CORRECTED the ProgID (2026-07-30)

Operator pointed out that micro-manager.org publishes the **redistributable**
Nikon Ti SDK, which sidesteps Nikon's registration wall:
`TiSDKRedist64-4.4.1.714.zip` (SHA256
`706500BA50451EAC61E552C2B7AB08FE335B66BA001C48B1B833F98D0A818950`, 1.2 MB).
Correct kit for this rig — it is a **Ti-E** (first-generation Ti), not a Ti2.

Downloaded and inspected **without installing** (`msiexec /a` administrative
extract). The payload is exactly what was missing:

* `Nikon\Shared\Bin\NikonTi.dll` **v4.4.1.714** ("Instruments Company, Nikon
  Corporation") — the DLL both the COM backend and Micro-Manager's adapter need,
  and per the MSI's `SelfReg` table it **self-registers as a COM server**.
* `Nikon\Shared\Drivers\micusb.inf/.sys/.cat` — **the USB driver**, i.e. the fix
  for the body's Code 28 state.
* `Nikon\Shared\Data\Ti\ObjectiveNames.txt` (199 entries),
  `FilterBlockNames.txt` (34: DAPI, UV-2A, V-2A, …) — Nikon's own name tables.
  A future nicety: offer these as slot-assignment suggestions.

#### 🐞 THE PROGID WAS WRONG — corrected from the DLL itself, no hardware needed

Reading the COM type information straight out of `NikonTi.dll` settled the one
thing `diagnostics()` was built to discover at the bench:

* The DLL publishes **61 `Nikon.TiScope.*` classes**, including
  **`Nikon.TiScope.NikonTi`** (the root), plus `Nosepiece`,
  `FilterBlockCassette1`, `ZDrive`, `XDrive`, `YDrive`, `PFS`, `DiaLamp`,
  `EpiShutter`, `LightPathDrive`, `PiezoZDrive`…
* The string **`LvMic` does not occur anywhere in the payload.**

So `Nikon.LvMic.NikonTi` — the ProgID this backend was written against, taken
from Micro-Manager-era notes — is **not** what a Ti/Ti-E class SDK registers.
`TI_PROG_IDS` now tries **`Nikon.TiScope.NikonTi` first**, keeping the `LvMic`
entries only as fallbacks for other SDK generations. Pinned by
`test_progid_order_matches_the_shipped_sdk`.

**The device aliases were right**: `Nosepiece` / `FilterBlockCassette1` /
`ZDrive` are the SDK's own class names, so the probing layer needed no change.
Pinned by `test_device_aliases_match_the_sdk_class_names`.

This is the single highest-value finding of the whole exercise — it is precisely
the unknown flagged as "unverified" at design time, resolved statically.

#### Install requires elevation → `tools_install_nikon_ti_sdk.ps1`

A quiet install from a non-elevated shell returns **1603** with
`HKEY_LOCAL_MACHINE\...\Installer\Rollback\Scripts` access denied — a
per-machine MSI plus a driver install needs Administrator, and elevation cannot
be requested from a non-interactive session without a UAC prompt that would hang
it. So the sequence is packaged as a repo-root script that:

1. refuses politely unless elevated (verified),
2. downloads (or reuses) the zip and **checks it against the recorded SHA256**,
3. installs the MSI quietly, accepting 0 and 3010,
4. `pnputil /add-driver … /install` for `micusb.inf` — the Code 28 fix,
5. verifies: DLL version, `Nikon.TiScope.NikonTi` registered (listing any
   `Nikon.TiScope*` keys found if not), and the `VID_04B0` device's status.

```powershell
# right-click PowerShell -> Run as administrator
powershell -ExecutionPolicy Bypass -File .\tools_install_nikon_ti_sdk.ps1
python tools_microscope_hw_check.py nikon_ti
```

#### ⚠ Code-signing caveat, disclosed

| Component | Signature |
|---|---|
| `TiSDKRedist64-4.4.1.714.msi` | **unsigned** |
| `NikonTi.dll`, `TiReg.dll` | **unsigned** (user-mode; not enforced) |
| `micusb.cat` (driver catalog) | **Valid**, `CN=Nikon Corporation` — but **SHA-1**, cert valid only 2011-10-24 → 2012-10-24, counter-timestamped by Symantec (which is why it still verifies) |

Provenance is the official micro-manager.org HTTPS media link. The user-mode
COM registration is unaffected by signing policy, so the **SDK half should
install regardless**. The risk is step 4: Windows 11 (build 26200 here) may
refuse to load a 2011 SHA-1-signed kernel driver. If it does, the SDK will
register but the body stays unreachable, and the fallback is Nikon's current Ti
Setup Tool. The script reports this explicitly rather than failing silently.

### ⭐⭐ SDK INSTALLED — real-hardware session (2026-07-30)

`tools_install_nikon_ti_sdk.ps1` run elevated. Everything installed:

* MSI exit 0, `NikonTi.dll` v4.4.1.714 present.
* `pnputil` added the driver and **bound it**:
  *"Driver package installed on device: USB\VID_04B0&PID_7832\00002"*,
  published as `oem72.inf`. The device is now correctly identified as
  **"Nikon USB Microscope"** (class Image, service `micusb`).
* **`Nikon.TiScope.NikonTi` registered** — the corrected ProgID connects, and
  the scope exposed exactly the predicted device tree (`Nosepiece`,
  `FilterBlockCassette1/2`, `ZDrive`, `XDrive`, `YDrive`, `PFS`, …).

#### 🔴 THE REMAINING BLOCKER IS HVCI — my SHA-1 theory was WRONG

The device moved from Code 28 to **Code 39 (`CM_PROB_DRIVER_FAILED_LOAD`)**.
The signature was *not* the reason. CodeIntegrity event **3111** names it:

> Code Integrity determined that a process (System) attempted to load
> `\Windows\System32\drivers\micusb.sys` that is **not compatible with
> hypervisor enforcement**. Failure bitmap 0x2. Status 0xC0000220.

Confirmed by `Win32_DeviceGuard`: `SecurityServicesRunning = 2`, i.e.
**HVCI / Memory Integrity is running**. A 2012 driver that maps
writable-and-executable memory cannot load under it.

**Fix: Windows Security → Device security → Core isolation → Memory integrity →
Off → reboot.** That is a targeted setting; Secure Boot and driver-signature
enforcement stay ON. Downgrading to redist 4.4.1.672 would not help — it ships
an *older* driver (2.0.2.1 vs 2.0.5.1).

Recorded because it is a trap: the old-driver + Code 39 combination *looks* like
a signature problem, and the fix for a signature problem (disabling signature
enforcement) would not have worked here.

#### ⭐ The SDK object model — mapped, and it corrected two more things

Introspecting the live COM object settled how the SDK actually works. **No
device property returns a number.** Every one returns an `IMipParameter`:

| Field | Meaning |
|---|---|
| `RawValue` | **the number** (this is what to read/write) |
| `RangeLowerLimit` / `RangeHigherLimit` / `RangeIncrement` | real bounds |
| `Unit` | physical unit — **`'um'` on the Z drive** |
| `DisplayString` | human text — `IsMounted` reports **"Device not available"** |
| `IsReadOnly`, `DataType`, `Name` | metadata |

Reading the wrapper itself is what produced the raw pointer bytes that crashed
`int()` on hardware. The plumbing now dereferences `RawValue`, and turret reads
**work**: filter and nosepiece both report `RawValue=1, range 1..6`.

**🐞 A 100× focus error, avoided.** The Z drive declares **`Unit='um'`** and a
range of **0…400000** — it is **µm-native**, so the correct scale is **1.0
units/µm, not the configured 100**. Every focus move would have been 100× too
small. `focus_units_per_um()` now **reads the SDK's declared unit** (µm / nm /
mm) and only falls back to the configured number if none is declared; the
setting is relabelled "fallback" accordingly. This retires the item flagged as
*"the single most important check on the SDK path"* — it no longer needs
measuring at the bench. `focus_limits_um()` likewise returns the declared
0–400000 µm travel instead of nothing.

**Operator-readable errors.** A raw `COMError` used to dump a full Python
traceback per failed operation. `_com_message()` now extracts the SDK's own
words, so the log reads **`No available instruments. [Nikon.TiScope.Nosepiece.1]`**
instead of 20 lines of traceback — and `IsMounted`'s "Device not available" is
surfaced verbatim.

Everything above is pinned by tests using a `_TiDevice`/`_MipParam` pair modelled
on the real interface; the two earlier tests that encoded the *guessed*
`Position.Value` model were deleted rather than patched, since they asserted an
object model that does not exist.

### ✅ HARDWARE VERIFIED — 22/22 after disabling Memory Integrity (2026-07-30)

Memory Integrity off + reboot → `micusb.sys` loads → the body is live.
`tools_microscope_hw_check.py nikon_ti`: **22 passed, 0 failed**.

| Feature | Result on the real Ti-E |
|---|---|
| Connect | `Nikon.TiScope.NikonTi`, no error |
| Filter cubes | **all 6 slots** switched, each confirmed by read-back; start slot restored |
| Objectives | **all 6 positions** switched, each confirmed by read-back; started at 2, restored to 2 |
| Focus read | **20045.0 µm**, scale auto-resolved to **1.0 units/µm**, travel 0–400000 µm |
| Out-of-range | refused, turret unmoved |

Focus **moves** remain the one unverified feature — they are opt-in (`--focus`)
because the drive moves the objective toward the specimen. Run
`python tools_microscope_hw_check.py nikon_ti --focus` with the objective clear.

#### 🐞 SAFETY BUG FOUND BY THE HARDWARE RUN — the SDK silently clamps

Asked for filter slot **999**, the Ti SDK **moved the turret to slot 6 and
reported SUCCESS**. No error, no exception — the operation looked fine and the
wrong cube was in the light path. For a *discrete selector* that is the worst
possible failure mode: a mistyped, stale or off-by-one slot number would quietly
image through the wrong filter while the software reported the move as good.

This is exactly the gap a simulator cannot expose — `SimulatedMicroscopeBackend`
had always raised for an out-of-range slot, so the behaviour looked correct
right up until real hardware disagreed.

Fixed in `_set_turret`: both turrets now validate the requested index against
the SDK's **own declared** `RangeLowerLimit`/`RangeHigherLimit` and **refuse**
before writing. Pinned by `test_out_of_range_turret_index_is_refused` (asserts
the device is never written) and `test_in_range_turret_index_still_moves`.

**Deliberately asymmetric:** the focus drive **clamps** to its declared travel
instead of refusing, because it is a continuous axis where running to the end of
travel during a jog is normal operation, not a mistake. Pinned by
`test_focus_clamps_rather_than_refusing`.

### ⭐ Hardware Setup → Microscope tab, and "Read from microscope"

Operator: *"lets make a hardware configuration tab for the microscope so I can
setup the objectives and filter cubes."*

**The body knows its own optics.** Probing the live SDK found two collections
that make typing names largely unnecessary:

* `FilterBlockCassette1.FilterBlocks` → per slot: `Name` (**DAPI**, **FITC**,
  **TxRed**, **Cy5**) and `Code` (0 = empty, name `-----`).
* `Nosepiece.Objectives` → per position: `Name`/`ProductCode` (**MRH20040**),
  `NumericalAperture`, `WorkingDistance`, and `Magnification` — which is an
  **index into the SDK's own table**, not a magnification. Verified against
  `ObjectiveMagnifications.txt` *and* cross-checked on hardware: codes 5/7/9 →
  **4x/10x/20x**, consistent with the NA/WD those same objectives report. An
  empty position has `Code == 0` and every other field raises
  *"No database code is associated with this optical element."*

New `MountedOptic` + `mounted_filters()` / `mounted_objectives()` on the backend
contract (`()` = the driver cannot report), implemented for the Nikon backend
and for the simulator (so the page is exercisable with no scope). Surfaced on
`MicroscopeState` and populated on connect + by `refresh_mounted()` — kept OFF
the ~1 s poll, because optics only change when someone physically swaps them.

**NEW `gui/pages/hardware/microscope_setup_panel.py` — `MicroscopeSetupPanel`**,
the **one** microscope setup surface:

* Connection: driver + per-driver options, live status (cube slot / objective
  position / moving), Diagnostics… — *no Connect button; see below*
* Filter cubes and Objectives: a row per slot showing **# · what the body
  reports · your name · Go**, a slot-count spin, and **↓ Read from microscope**
  which fills the editable names from the hardware. Per-row **Go** rotates the
  turret so the operator can see what is in a slot; the current slot is
  outlined. Go and Read are gated on a live connection.
* Focus: live position with the declared travel, jog step, direction, soft limits.
* Nothing reaches the store until **Save** (or the dialog's OK).

Registered as Hardware Setup sub-page **"Microscope"** (after Cameras). The
Plate tab's icon moves `microscope` → `grid` — a plate *is* a grid of wells, and
that frees the microscope icon for the actual microscope.

**Anti-divergence:** `MicroscopeSettingsDialog` was rewritten as a thin wrapper
that embeds the same panel (OK = `commit()`, Cancel = discard). Two
independently written surfaces onto one set of settings is precisely the failure
recorded in `MEBP_v75x_UNIFIED_MOSAIC_CALIBRATION.md`.

Also: an unnamed slot in the jog card now falls back to **the body's own name**
before showing "(empty)" — operator name wins, hardware name next.

**Verified on the real Ti-E**: one click filled cubes `DAPI/FITC/TxRed/Cy5` and
objectives `4x/10x/20x` (with `MRH20040 · NA 0.13 · WD 16.4 mm` shown alongside),
empty slots correctly marked, focus `22,564.00 µm (travel 0 – 400,000 µm)`, and
Save round-tripped to the store.

### 🐞 Two operator-reported bugs, 2026-07-30 (both fixed)

**1. Focus moves were 40× too small.** *"I was trying to move up 1000 µm but it
only moved 10 microns."* My earlier "auto-detect" read
`ZDrive.Position.Unit == 'um'` and concluded the drive was µm-native
(factor 1.0). **`Unit` describes the DISPLAY value, not `RawValue`.** The SDK's
actual model is:

```
display_value [in Unit] = RawValue × DisplayScale
```

Measured: `DisplayScale = 0.025`, `Unit = 'um'` ⇒ **0.025 µm (25 nm) per raw
unit ⇒ 40 units/µm**. Confirmed three independent ways —
`RawValue 30844` → `DisplayString '771.100 um'` (30844 × 0.025 = 771.100,
exact); `ZDrive.Resolution = 25` with the ZDrive's *device* `Unit = 'nm'`; and
the declared range `400000` → `'10000.000 um'` = **10 mm**, the Ti's real focus
travel. **400 mm — what the old reading implied — should have been the tell.**

`focus_units_per_um()` now computes `1 / (DisplayScale × unit_to_um(Unit))`,
falling back to the configured value only when no `DisplayScale` is exposed.
`focus_limits_um()` uses the same conversion, so the travel now reads
0–10000 µm instead of 0–400000.

**Verified on hardware:** our reading `771.175 µm` matches the SDK's own
`DisplayString '771.175 um'` exactly; commanded ±10 µm and ±5 µm each moved
±10.000 / ±5.000 µm; zero net drift over four moves. Pinned by
`test_focus_scale_uses_displayscale_not_the_unit_alone`,
`test_focus_move_of_1000um_writes_the_right_raw_delta` (the operator's exact
case: 1000 µm → raw 40000) and `test_focus_travel_converts_through_displayscale`.

Two earlier tests that asserted the Unit-alone model were **deleted, not
patched** — they encoded the bug.

**2. The background poll closed an open drop-down.** *"every time it reads it
cancels the dropdown box I have opened."* The jog card polls the body every
~1 s; the refresh sets `busy=True`, `_render` then disabled the combo, and a
disabled combo snaps its list shut under the cursor. `_rebuild_slot_combos()`
would have cleared it outright.

Fixed in `gui/widgets/microscope_panel.py`: `_tick()` skips polling entirely
while any drop-down is open, and `_render()` never disables, re-indexes or
rebuilds a combo whose popup is showing — while a *closed* combo still tracks
the hardware. Pinned by
`test_background_poll_is_paused_while_a_dropdown_is_open` and
`test_open_combo_keeps_its_selection_during_a_render`.

### New: `tools_microscope_hw_check.py`

Repo-root bench tool (same convention as `tools_render_geometry_panel.py`) that
runs this whole exercise through the real widget against any backend:

```
python tools_microscope_hw_check.py nikon_ti           # turrets only
python tools_microscope_hw_check.py nikon_ti --focus   # + focus moves
```

⚠ **Focus moves are opt-in.** The focus drive moves the objective toward the
specimen, so a blind move with a dish loaded can drive it into the glass.
Without `--focus` the axis is read-only; with it, excursions are limited to
`--step` µm (default 5) and the start position is always restored. It prints
`diagnostics()` *before* commanding anything, uses the operator's real saved
configuration on a hardware backend, and never rewrites their slot assignments.

---

## Issues & Decisions

* **Manual only, by request.** No print / workflow / calibration path calls into
  this module. The objective turret in particular is *not* wired to
  `ObjectiveCalibrationStore` or `camera_config.current_objective_name`: linking
  them means deciding whether rotating the nosepiece should re-point the µm/px
  calibration (and what happens mid-print), which is exactly the integration
  work being deferred. The store's objective names are free text so they can be
  typed to match the existing objective names ahead of that pass.
* **SDK mapping unverified — disclosed, not hidden.** Rather than assert a
  guessed COM mapping works, the backend probes, fails with named errors, and
  ships `diagnostics()` to close the loop in one bench session. Micro-Manager is
  offered as the already-proven alternative.
* **`z_units_per_um` is a setting, not a constant** — it scales every focus move,
  so it must be correctable from the UI.
* **Focus direction is a setting** — this codebase has repeatedly been bitten by
  hard-coded axis polarity (`ZDIR`, `plate_flip_180`, `z_up_sign`). The
  convention is a checkbox from day one.
* **Widget polls, it does not subscribe.** Avoids a worker-thread callback firing
  into a destroyed Qt object, at the cost of ≤400 ms of render latency.
* **Hardware polling stops while the card is hidden**; the connection itself is
  left alone because the panel is instantiated on several pages and they share
  one body.
* **Singleton controller** — `StandardJogContextPanel` is built on Jog,
  Calibration and every jog-capable workflow; all instances must observe and
  command the same body over one connection.
* **No collision-safety coupling.** The focus drive moves the *objective*, below
  the plate — it is not the needle Z and does not participate in the
  retract-before-XY invariants. It has its own optional soft limits.
* **Bug found in my own test, worth recording:** `QMessageBox.warning` inside the
  dialog's refusal path blocks *forever* under offscreen Qt — the modal must be
  patched in headless tests.

---

## Connect moved to the Connect Hardware card (2026-07-31)

Operator: *"move the microscope connect button to the connection area for xy
stage, zp stage"*.

The body is hardware, so it is opened where hardware is opened. A **Microscope**
row now sits with XY / Z + Pumps / Xbox in the `Connect Hardware` card
(`gui/pages/hardware/control_panel.py::_build_connect_group`), with the same
three controls the stage rows have — **Connect · Simulate · Disconnect** — plus a
live status badge.

* **Connect** uses the driver **saved** on Hardware Setup → Microscope. Nothing
  about the driver is decided here; this row only opens and closes the link.
* **Simulate** connects the software body via `connect("simulated")`, which takes
  a backend override *without writing it to the store* — a simulator run can
  never silently rewrite the operator's real driver choice (pinned by test).
* The badge resolves **asynchronously**. `MicroscopeController.connect()` queues
  work onto the COM-owning worker thread, so the click only shows
  "Connecting…"; the outcome lands on a later `on_status_update()` tick via
  `_sync_microscope_badge()`. `busy && !connected` is the in-flight signal —
  read from the controller's own state rather than matching the badge's text.

Two placement details that follow from the microscope being its **own
singleton**, not part of `StageController`:

* `_sync_microscope_badge()` is called from `on_status_update()` **before** the
  `self._controller is None` early-return, so the badge is live even on a panel
  with no stage controller injected.
* Everything is guarded on `hasattr(self, 'badge_scope')` — the calibration
  variant builds this panel with `show_connect=False` and has no connect card
  at all (regression-tested).

**Removed** from `MicroscopeSetupPanel`: its own Connect button and
`_toggle_connect`. That method also wrote `backend` + `prog_id` to the store on
every click so an unsaved driver choice would be used — a hidden write that is
now gone. The panel still *reports* the live link (its **Go** and **↓ Read from
microscope** need it) and carries a hint naming the two places to connect, plus
the consequence: **a driver change takes effect on the next connect after Save.**

**Not** removed: the jog card's own Connect button. `StandardJogContextPanel`
builds `HardwareControlPanel(show_connect=False)`, so the jog pill has no Connect
Hardware card — taking the card's button away as well would leave the jog
surface with no way to connect at all.

Tests: `TestConnectHardwareRow` (6) — the row sits in the same card as the stage
rows, connect/disconnect round-trip, Simulate leaves the saved driver alone, a
failed connect reaches the badge, the tick works with no stage controller, and
`show_connect=False` still builds and ticks. Suite **101 green**; regression
context-panel / jog / device-page / camera / hardware-setup **273 green**.

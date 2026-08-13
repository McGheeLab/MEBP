# MEBP v7.17 — EpiShutter / DiaLamp / LightPathDrive + shutter↔cassette interlock

## Objective

The Nikon Ti SDK exposes 61 `Nikon.TiScope.*` classes. v7.5.x wired **three** of
them (`FilterBlockCassette1`, `Nosepiece`, `ZDrive`). Three more that this rig
physically has are exposed and driven by nothing:

| Device | What it is | Why we want it |
|---|---|---|
| `EpiShutter` | epi-fluorescence (excitation) shutter | keeps excitation off the sample except while acquiring |
| `DiaLamp` | transmitted-light (diascopic) lamp | brightfield illumination on/off + level, currently set by hand on the body |
| `LightPathDrive` | eyepiece / camera-port selector | the camera sees nothing unless this is on the right port |

Plus the operator-requested behaviour that motivated the ask:

> **EpiShutter / DiaLamp / LightPathDrive API, and whether shutter can be
> interlocked with the cassette move.**

**The interlock is the substantive half.** Rotating the filter cassette with the
excitation shutter open sweeps the excitation beam across every cube that passes
through the light path: the sample is flashed with out-of-band excitation
(photobleaching, and phototoxicity on a live prep), and any camera integrating at
that moment gets a bright artefact frame. Closing the shutter for the duration of
the rotation is what NIS-Elements does and what our multi-channel fluorescence
mosaic will need once cube changes are automated (today it still
[prompts the operator to switch the cube by hand](../../gui/pages/workflows/fluorescence_mosaic_workflow.py)).

Scope is the same as the rest of `MicroscopeControl.py`: **manual control plus the
interlock.** No print / workflow / calibration path is changed.

---

## Files Modified

| File | Why |
|---|---|
| `SupportClasses/MicroscopeControl.py` | backend contract + all three backends + controller ops/state + the interlock |
| `SupportClasses/MicroscopeConfigStore.py` | `filter_shutter_interlock`, `epi_shutter_invert`, three `mm_*_device` names |
| `gui/widgets/microscope_panel.py` | the manual surface: shutter toggle, lamp on/off + level, light-path combo |
| `gui/pages/hardware/microscope_setup_panel.py` | the two policy checkboxes + MM device names + live readout |
| `tests/test_v717_microscope_illumination_and_interlock.py` | NEW |

---

## Implementation Steps

- [x] **1. Store.** `filter_shutter_interlock` (default **False**) and
      `epi_shutter_invert` (default False) + accessors; three `mm_*_device`
      defaults threaded through `backend_kwargs()`.
- [x] **2. Backend contract.** `has_epi_shutter` / `epi_shutter_open` /
      `set_epi_shutter`; `has_dia_lamp` / `dia_lamp_on` / `set_dia_lamp_on` /
      `dia_lamp_intensity` / `set_dia_lamp_intensity` /
      `dia_lamp_intensity_range`; `light_path_count` / `get_light_path` /
      `set_light_path` / `light_path_names`. Base class returns
      absent/`None`/raises, so a body without the accessory is normal.
- [x] **3. Simulated backend** models all three, so the panel and the interlock
      are exercisable and testable with no hardware. Constructor flags
      (`epi_shutter=False`, `dia_lamp=False`, `light_path_slots=0`) make the
      *body-without-this-device* path testable too — a simulator that always has
      everything would have hidden it.
- [x] **4. Nikon Ti backend** — probed, never assumed, per the existing pattern.
      Own `_value_param`/`_write_value`/`_value_range` helpers because the
      value-carrying property is **not** always `Position` on these devices (a
      lamp level and a shutter state are both commonly `Value`); the
      hardware-verified turret/focus paths are left byte-identical.
- [x] **5. Micro-Manager backend** — `setShutterOpen`/`getShutterOpen` for the
      two shutters, a probed `Intensity`/`Voltage`/`Level` property for the lamp,
      the existing state-device helpers for the light path.
- [x] **6. Controller** — new state fields, four new ops, and the interlock
      inside `set_filter`.
- [x] **7. GUI** — jog card controls; setup-page checkboxes + MM device rows +
      a live "what is fitted and what state is it in" line.
- [x] **8. Tests**, mutation checks, regression run.

---

## Issues & Decisions

### D1 — The interlock lives in `MicroscopeController.set_filter`, not in a new method

`set_filter` is the **one chokepoint** every cassette move reaches: the jog
card's combo, the Hardware Setup slot table's `Go` button, and anything added
later. A separate `set_filter_interlocked()` would have left the `Go` button
unprotected — the "one writer" lesson this file keeps re-recording.

### D2 — It must be ONE queued op

The controller is a process-wide singleton and the jog card polls it ~1 Hz. Close
→ move → restore is wrapped in a single `_submit`ted function so no other
surface's op can interleave between the close and the move. (Three separate ops
would be three queue entries with a `refresh` free to land in between.)

### D3 — Restore the PREVIOUS state, never "open"

If the shutter was already closed the interlock must leave it closed. Reopening a
deliberately-darkened light path would illuminate a sample the operator had gone
dark on. Same class as the v7.16 "restore CLEARS as well as sets" hazard.

### D4 — Restore in a `finally`, and surface a failed restore

A cassette move that fails mid-rotation must still reopen the shutter, or the
operator's next acquisition is black with nothing on screen explaining why. A
failure of the *restore itself* is reported as the operation's error — a shutter
stuck closed is exactly the thing you need told about.

### D5 — Interlock defaults **OFF**

Two reasons, and only the second is about caution:

1. **The shutter's open/closed encoding is UNVERIFIED on hardware.** If it is
   inverted on this body, an interlock would *open* the shutter for the duration
   of the rotation — precisely the damage it exists to prevent. Enabling it by
   default would ship that risk.
2. Precedent in this very store: `parfocal_auto_apply` defaults False because
   *"silently moving focus on a turret change is a surprise until the operator has
   seen the measurement."* Same shape.

The bench checklist below therefore verifies the shutter direction **first**, and
only then turns the interlock on.

### D6 — Shutter encoding: derived, then refined, then overridable

Not hardcoded. In order:

1. The SDK's **declared range** gives two codes; lower = closed, upper = open
   (the Ti convention).
2. That is then **refined by the live `DisplayString`** — a shutter reports
   "Open"/"Closed", so the current raw value's own text tells us which code means
   which, and the other follows by elimination. This is a real measurement, not a
   guess, whenever the SDK populates the string.
3. `epi_shutter_invert` in the store is the operator override, so a body that
   disagrees is a checkbox, not a code change — the same treatment as
   `focus_up_is_positive` and `plate_flip_180`.

### D7 — Light path REFUSES an out-of-range index; lamp intensity CLAMPS

Straight from the hardware-verified v7.5.x finding that the Ti SDK **silently
clamped filter slot 999 to 6 and reported success**. A light path is a discrete
selector, so the same refusal applies — it routes through the existing
`_set_turret`. Lamp intensity is a continuous level where hitting the end of the
declared range is ordinary, so it clamps, exactly as `set_focus_um` does.

### D8 — Lamp intensity is reported in the SDK's OWN units, not a fake percentage

`dia_lamp_intensity_range()` returns the declared range and the UI spans it.
Rescaling to 0–100 % would invent a number: the declared range may be volts or an
arbitrary index, and a percentage of an unknown quantity is the "fabricated value
that looks measured" trap.

### D9 — Nothing about illumination is persisted

Lamp level and shutter state are live illumination, not calibration. Restoring a
saved lamp level on connect would turn a lamp on unexpectedly.

### D10 — All three are polled

The Ti-E has **physical buttons and a knob on the body** for the dia lamp and the
shutter, so the operator can change them without the software. Polling is the only
way the panel reflects reality.

### D11 — No operator labels for light-path positions this pass

The obvious symmetry would be store-backed labels like the filter cubes. Skipped:
adding a store key with no production writer is the dead-code trap this project
has repeatedly recorded (v7.9's `prep_bores`, the longest-bore descend). The combo
shows the SDK's own reported name, else `port N`. Labels can be added later with
their own UI and no schema change.

### D12 — Switching the light path away from the camera blacks out every frame

A real operational hazard: a mosaic scan would then stitch black tiles and read as
an exposure or focus fault. **Not** given a new guard, because one already exists —
a scan or calibration holding the controller's exclusivity lease makes the panel's
light-path op fail fast with *"microscope is reserved by …"*. The panel tooltip
says so.

### D13 — The interlock is applied to the cassette only, not the nosepiece

A nosepiece rotation also sweeps the beam, and `_with_shutter_closed()` is written
generically so extending it is one line. Left out because the operator asked about
the cassette, and an objective change during fluorescence is a rarer gesture worth
its own decision.

---

## Testing Notes

`tests/test_v717_microscope_illumination_and_interlock.py`. The load-bearing tests
drive the **real** `MicroscopeController` and assert the shutter state *at the
moment* `set_filter` executes (a test subclass of the simulated backend records
it), because asserting on the final state cannot distinguish "closed during the
move" from "never touched".

**74 tests, all green, no skips.** ⚠ One of them was originally written to
`skipTest` when offscreen Qt would not give a spin box focus — which would have
left the "don't overwrite a level being typed" guard unverified on every machine
that runs the suite headless. Rewritten to force `hasFocus` in both directions
instead.

**6/6 mutations confirmed CAUGHT**, sources restored byte-identical:

| Mutation | Caught by |
|---|---|
| 1. interlock skips the close | `…closed_during_the_move…` + 2 others |
| 2. restore forces "open" instead of the previous state | `test_an_already_closed_shutter_stays_closed` |
| 3. restore moved out of the `finally` | `test_a_failed_move_still_reopens_the_shutter` + 1 |
| 4. interlock runs even when the setting is off | `test_off_by_default_the_shutter_is_never_touched` |
| 5. light path clamps instead of refusing | `test_out_of_range_is_refused_not_clamped` |
| 6. shutter codes hardcoded, ignoring range + `DisplayString` | `test_codes_are_refined_by_the_reported_state_text` + 1 |

**Regression: 706 green**, run per-batch —
nikon-ti + this suite + suite-hygiene **185** ·
context-panel / illumination-LED / responsive-context / jog-navigation /
workflow-settings **130** ·
microscope-focus-state-bug / objective-ladder / lablink-job / lablink-page **129** ·
v7.11 focus-curve / focus-needle-datum / focus-sweep-planner /
host-accessor-contract / objective-optics / optical-datum-placement /
plate-bottom-optical + wizard + worker / plate-level-holdout-focal + math +
wizard + worker **262** — plus a `gui.app` import smoke and a build of the
**real** `HardwareSetupPage`, switching to its Microscope tab and confirming the
new group renders (`excitation shutter: not fitted · dia lamp: not fitted ·
light path: not fitted`, correct with nothing connected).

---

## ⭐ REAL-HARDWARE SESSION 2026-08-12 — Ti-E on ME3B V1

Preconditions confirmed first: `Nikon USB Microscope` **Status OK /
CM_PROB_NONE**, Memory Integrity **off** (`SecurityServicesRunning = 0`),
`NikonTi.dll` **v4.4.1.714**, `Nikon.TiScope.NikonTi` registered.

**All three devices resolved** — `EpiShutter`, `DiaLamp`, `LightPathDrive` (plus
`DiaShutter`, `PFS`, `PiezoZDrive`, `CondenserCassette`, `ExcitationFilterWheel`,
`BarrierFilterWheel`, `FiberIlluminator`, `TIRF`, `Analyzer`, `XDrive`, `YDrive`
on the scope object). The read-only probe then overturned **three** of the
assumptions this work shipped with.

### 🔴 F1 — "the attribute resolved" is NOT presence, and it silently disarmed the interlock

The SDK publishes a COM object for **every device it knows about, fitted or
not**. On this body:

| | `IsMounted` | `IsOpened` |
|---|---|---|
| `EpiShutter` | `0` — *"Device not available"* | `-1` — *"Status Unknown"* |
| `DiaShutter` | `0` — *"Device not available"* | `-1` |
| `DiaLamp` | `1` — *"Device mounted"* | — |
| `LightPathDrive` | `1` — *"Device mounted"* | — |

**There is no epi shutter on this microscope**, yet `has_epi_shutter()` returned
**True**. The consequence was not cosmetic: `epi_shutter_open()` fell through to
the generic `Value` (1, in a declared 1–2 range), decoded it as *closed*, and the
interlock therefore concluded *"already closed — nothing to protect"* and
**silently did nothing** while the UI showed a shutter row. An operator would
enable the interlock, see no error, and believe the sample was protected.

Fixed with `_accessory_present()` consulting `IsMounted` (absent = present, since
a device that does not report its mount state is still usable), and `-1 /
'Status Unknown'` now reads as **unknown, never as a state**.

⚠ **My simulator always had every accessory mounted, which is precisely why no
test could see this** — the recurring "a stub that agrees with the code proves
nothing" lesson. The fake now takes `mounted=False`.

### 🔴 F2 — `Value` is not the shutter state; there are named members instead

`IEpiShutter` publishes **`IsOpened`**, **`Open()`**, **`Close()`** — and
**`IsInterlockEnabled`**, i.e. *the SDK has its own interlock notion*.
`IDiaLamp` publishes **`IsOn`**, **`On()`**, **`Off()`**, `Increase()`,
`Decrease()`, `MeasuredVoltage`, `IsControlled`.

So the riskiest thing in this change — the derived open/closed encoding, and the
`epi_shutter_invert` flag that existed to rescue it — is **unnecessary wherever
`IsOpened`/`Open()`/`Close()` exist**: a named action cannot be got backwards.
Those are now preferred, with the derived codes kept only as the fallback for an
SDK generation that lacks them. `IsInterlockEnabled` is surfaced read-only in
Diagnostics (it reads `-1` here, the shutter being unfitted) and is a good
question for Nikon.

⚠ `dir()` on a comtypes pointer **lies in both directions**: `ControlVoltage`
appears in it and raises `AttributeError` on access, while `Position` is absent
from `DiaLamp`'s listing. Probing must be by `getattr`, which is what the code
does.

### 🔴 F3 — the dia lamp has a control-mode gate that refuses everything

`IsControlled` = **`0 'MainMode'`** (the body's front-panel knob owns the lamp)
⇒ the SDK refuses every write: `0xE01004BB` for the level, `0xE01004BE` for the
switch. **`1 'RemoteMode'`** ⇒ writes are accepted. This is the state the rig sits
in, so it is the first thing an operator hits, and a bare HRESULT says nothing
about the knob in front of them.

Now checked **before** writing, with a message that names MainMode and the
remedy; `dia_lamp_remote()` / `set_dia_lamp_remote()` added, plus a state field
so the panel disables the lamp controls and explains why. **Taking Remote control
is a separate explicit action, never a side effect of moving a slider** — it takes
the lamp away from the person standing at the microscope.

### ⚠ F4 — the lamp LEVEL does not stick (disclosed, not fixed)

In RemoteMode a level write is **accepted**, `Value` reads back the number
written, and then it **reverts to the knob's setting (1) within about a second**;
`MeasuredVoltage` never moves off `0.5` at all, so the brightness does not appear
to change. Identical via `set_dia_lamp_intensity()` and via a direct parameter
write; `Increase()` also does nothing. **On/off does work.**

⚠ **I initially got this wrong and a re-measure caught it.** One probe sampled at
0.4 s intervals, saw `1 → 6 → 12` hold, and I recorded "level control works". A
focused re-run with reads either side of the revert showed the value latching for
under a second and then snapping back. The claim was withdrawn rather than
shipped.

Left writing rather than refusing: the write is correct and harmless, may hold on
another body, and the ~1 s poll makes a level that does not stick visibly bounce
back on screen instead of being silently swallowed. What governs it is a question
for Nikon.

### 🐞 F5 — the first `Position` read after connect is STALE (found while checking my own restore)

Chasing an apparent inconsistency — the raw dump read the light path as `1`
while `get_light_path()` read `3` in the same run — turned up a separate defect,
and it is **the access that primes it, not elapsed time**:

* reading `Position` as the *first* COM access on a device returns a default:
  the drive physically at 3 answered **1**, and kept answering 1 for as long as
  nothing else on that device was touched (6 reads over 0.9 s);
* one access to any other property (`IsMounted`) makes the very next `Position`
  read return the true 3. Reproduced across two fresh connections.

Every position getter is primed **today only by accident**: they all call
`_require_mounted` first. So is `dia_lamp_intensity` in `_read_all`, because
`has_dia_lamp` happens to run ahead of it — reorder those two lines and the first
state published after connect silently carries a wrong value. NEW
`_prime_devices()` called from `connect()` makes it deliberate: six cheap reads,
no sleep. **Verified on hardware — first-access `Position` now reads 3 on three
consecutive fresh connections.**

⚠ This is also the reason I briefly believed my own test had *moved* the
operator's light path. It had not: the drive was at 3 throughout. Worth recording
because the false alarm came from trusting a single read.

### Exercised through the real `MicroscopeController` — **20/22**

Temp store seeded from the operator's real settings so
`config/hardware/microscope.json` was never written (verified untouched); every
original value recorded up front and restored in a `finally` (verified: cube 3,
lamp on at 1, light path 3 — all back). No focus moves.

| | Result |
|---|---|
| connect, read every accessory | PASS |
| light path 3 → 1, read back | PASS |
| **light path 99 REFUSED, and the drive did not move** | PASS |
| restore light path | PASS |
| **lamp write in MainMode refused, naming MainMode** | PASS |
| take Remote control, read back | PASS |
| lamp on/off toggled and read back | PASS |
| lamp level set / clamped | accepted, **reverts** — F4 |
| **cassette 3 → 1 with the interlock ENABLED and no shutter fitted → plain move** | PASS |
| every original restored | PASS |

**The interlock's protective path could NOT be verified on this body — there is
no epi shutter to close.** What was verified is its *degradation*: with the
interlock enabled, a real cassette rotation succeeded, touched no shutter, and
reported no error. The close/reopen ordering remains covered only by the
simulator tests.

## Still needs real-HW verification, IN ORDER

Everything above was done on 2026-08-12. What remains needs hardware this body
does not currently have, or an operator standing at the microscope.

1. **Fit an epi shutter, then verify the interlock's protective path** — the whole
   point of the feature, and unverifiable here. In order: Diagnostics… must show
   `epi_shutter FITTED -> True` and an `IsOpened` that is not *Status Unknown* →
   toggle *Excitation* and **look at the body, not the screen** (with `IsOpened`
   and `Open()`/`Close()` present the encoding is no longer derived, so
   *Shutter reads inverted* should never be needed — if it is, something else is
   wrong) → **then** enable the interlock and confirm the shutter closes for a
   cube rotation and reopens; that an already-closed shutter stays closed; and
   that Hardware Setup's per-slot `Go` behaves identically → force a failure
   mid-rotation and confirm the shutter still reopens and the error names it.
2. **Confirm which light-path position feeds the camera.** All four switch
   correctly; only an operator can say which one is the camera port (the live
   feed going black on the eyepiece position is expected, not a fault).
3. **The dia lamp level (F4).** Ask Nikon what governs `DiaLamp.Value` in
   RemoteMode, or confirm at the body that the knob overrides it. If it is
   genuinely knob-only, consider making the level read-only in the UI rather than
   offering a control that bounces back.
4. **Watch the readout follow the body's own controls** — turn the lamp knob and
   press the front-panel buttons and confirm the panel tracks them (this is what
   the ~1 s poll exists for; it could not be checked without someone at the rig).
5. Restart and confirm the interlock/invert flags persisted and the lamp level was
   **not** restored.

---

## Status

- [x] Implementation
- [x] Tests green (**96**, no skips)
- [x] Mutation checks (**10/10**, including the four bugs the hardware found)
- [x] Regression (**370** across the affected suites, + a `gui.app` import smoke
      and a real `HardwareSetupPage` build)
- [x] **Real-HW session on the Ti-E (2026-08-12)** — 20/22; found and fixed
      F1, F2, F3, F5; disclosed F4. Rig left exactly as found (light path 3,
      cube 3, objective 1) and `config/hardware/microscope.json` never written.
- [ ] Interlock's protective path — **blocked: no epi shutter is fitted on this
      body**

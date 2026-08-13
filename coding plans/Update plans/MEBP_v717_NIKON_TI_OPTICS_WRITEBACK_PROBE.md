# MEBP v7.17 — Can the Nikon Ti-E be TOLD what optics are fitted? **No. Answered on hardware.**

## Objective

Operator: *"I need a way to write to the NikonTi-E what objectives, and filter
cubes are on it. If I change it in the software I want the microscope display
to be correct."*

Then, as the decisive test: *"as a test I want you to write to the microscope
that filter cube 4 is CY5."*

## ⭐ THE ANSWER (real Ti-E, SDK 4.4.1.714, 2026-08-12)

**The Ti's optics database is READ-ONLY through this SDK.** Asked to set filter
slot 4's `Name` to `CY5`, the SDK answered in its own words:

```
Database entry cannot be modified. [Nikon.TiScope.FilterBlock.1]
```

Nothing changed on the body; the re-read afterwards was byte-identical.

**⚠⚠ AND THE TYPELIB CLAIMED THE OPPOSITE.** The read-only inspection on that
same body reported:

```
{'filter': {'Name': True, 'Code': True}, 'objective': {'Name': True, 'Code': True}}
```

i.e. every identity field declares a `propput`. **A declared setter is a FALSE
POSITIVE on this body** — only an actual attempt settles it. That is now
recorded in three places (`probe_optic_write_support`, `set_optic_name`, and the
panel's own report) precisely so the optimistic reading cannot be quoted later
as an answer.

### Round 2 — every other write path tested, on the operator's prompt

*"pymmcore might be able to help make these changes, I know micromanager can do
it."* Worth taking seriously, and it produced the real evidence chain. Read the
COM **type information** out of `NikonTi.dll` (74 interfaces) rather than
guessing, and it looked genuinely promising:

* `_IElementBase.Name` — *"**Gets or sets** the abbreviated name"* (propput
  present!)
* `_IElementBase.CanModify` — *"Determines if properties such as 'Name' **can be
  modified** for this optical element (read-only)"*
* `IFilterBlock` — writable `ExcitationFilterCode`, `DichroicMirrorCode`,
  `BarrierFilterCode`, `Composition` (*"Gets or sets the codes for the optical
  elements in the filter block"*)
* *"The number of **factory-defined** filter blocks / objectives"* — implying a
  factory vs **user-defined** split
* `Nikon.TiScope.Database` / `IDatabase`, whose `FilterBlocks` collection holds
  **309** entries and exposes **`Add`** and **`Remove`**

So the hypothesis was: declare slot 4 by its optical make-up — `Cy5` is
catalogue code 25 = excitation 20 / dichroic 11 / barrier 19 per
`FilterCodes.txt`. Tested on the EMPTY slot 4 (no motion-safety coupling),
originals recorded, restore in a `finally`:

```
ExcitationFilterCode <- 20: REFUSED (Database entry cannot be modified.)
DichroicMirrorCode   <- 11: REFUSED (Database entry cannot be modified.)
BarrierFilterCode    <- 19: REFUSED (Database entry cannot be modified.)
==> body identification CHANGED: False        (nothing written, nothing to restore)
```

And `CanModify` reads **0 on all six slots**, filled and empty alike. The
`Database.Add` route is a dead end for a different reason: it edits the
**catalogue**, while the live slot's `Code` is documented read-only and reports
0 — so a new catalogue entry cannot be bound to a slot. **The chain breaks at
the sensing step, not the catalogue step.**

⚠ **And pymmcore/Micro-Manager cannot differ.** MM's `NikonTI` adapter *wraps*
this same `NikonTi.dll` ("This adapter uses the driver and API supplied by
Nikon") and would take the identical refusal from the identical code path — the
message itself lives in Nikon's `MipDeviceMsg.dll`, beneath any wrapper. Also
confirmed: **no MM device adapter (`mmgr_dal_NikonTI.dll`) exists anywhere on
this machine**, so MM cannot currently reach the body at all. What MM genuinely
offers is `defineStateLabel` — **host-side** labels for turret positions kept in
MM's own configuration, i.e. the same kind of thing `MicroscopeConfigStore`
already provides, and equally invisible to the body's display.

**Improvement kept from this round:** `set_optic_name` now reads Nikon's own
`CanModify` flag and refuses with a plain explanation *before* attempting any
write, instead of surfacing a COM error. Verified on hardware.

### Round 3 — *"I physically put a Cy5 cube in, and it is not auto reading it"*

Exhausted the remaining software levers, all refused or inert:

| Attempt | Result |
|---|---|
| `Name` write, unlocked | `Database entry cannot be modified.` |
| `Name` write, **holding `scope.LockDevice(5000)`** | same refusal — **not a locking issue** |
| `ExcitationFilterCode` / `DichroicMirrorCode` / `BarrierFilterCode`, unlocked | all refused |
| the same three **while locked** | all refused |
| `Code` | documented read-only; reads 0 |
| `CanModify` | **0 on all six slots**, filled and empty alike |
| `Database.FilterBlocks.Add` (309-entry catalogue) | edits the CATALOGUE; can't bind to a slot, since slot `Code` is read-only |
| `scope.ShowWindow(0, 0)` | returns OK but only creates internal ATL message windows (`MyTiEventWindow`, class `CNikonTi`) — **not a setup UI** |
| pymmcore / Micro-Manager | wraps this same `NikonTi.dll`; also no `mmgr_dal_NikonTI.dll` on this machine |

**⭐ ROOT CAUSE, from the one field that differs.** `SupportFirmVersion` reads
`'v1.00'` on slots 1–3 (the named ones) and **`None`** on slots 4–6. Nikon's own
`FilterBlockNames.txt` stores each entry as `"DAPI,v1.00"` — a name *plus a
version*. So the body holds a **persistent per-slot registration**, not a live
reading of the cube: slots 1–3 were registered once (by whoever commissioned the
scope), 4–6 never were. **The Ti-E's filter turret has no cube-identification
sensor, so there is nothing to "auto read"** — which is why fitting a cube
changes nothing, and why the SDK calls it a *database entry*.

Body: `SystemType` = `TiE+HubA`; cassette 1 `IsMounted='Device mounted'`,
`MountedPosition='Lower'`; cassette 2 `'Device not available'` (so not a
wrong-cassette case).

**No workflow impact.** The turret still rotates to position 4 and the fitted
Cy5 cube still images; only the body's own label is absent. MEBP already carries
`"4": "Cy5"` with 640/685 nm, which is what every app surface and the LabLink
sidecar use. Registering it in the body needs Nikon's own tooling (Ti Setup Tool
/ NIS-Elements / a Nikon engineer) — the same route that registered slots 1–3.

**What the physical display actually follows:** a `Code` the body *senses* from
the fitted optic, resolved through Nikon's own catalogues in
`C:\Program Files\Nikon\Shared\Data\Ti`. Cross-validated against this rig's live
readings — the body reported codes **4 / 15 / 23**, and `FilterBlockNames.txt`
(0-based) row 4/15/23 is **DAPI / FITC / TxRed**. Exact match. **`Cy5` is code
25.** (`FilterCodes.txt` further decomposes code 25 → excitation 20, dichroic 11,
barrier 19.)

**Slot 4 reports `Code == 0` — the body sees nothing coded there**, so there is
no database entry to rename even in principle. Either the slot is empty or it
holds an optic the body cannot identify. The operator's app-side config already
carries `"4": "Cy5"` (plus its 640/685 nm wavelengths), which is the correct home
for that knowledge and is what every MEBP surface and the LabLink sidecar use.

**Consequence for the original request:** the app cannot make the Ti-E's own
display follow a rename. To change what the *body* displays, the fitted optic's
hardware code has to change (a coded Nikon cube/objective), or Nikon's own
configuration tooling has to write that database — not this SDK. The bundled
`C:\Program Files\Nikon\TiSDK\Help\NIKONTI_E.chm` is the vendor reference if
that path is ever pursued (`hh.exe -decompile` does not run non-interactively;
open it by hand).

## Investigation trail

`NikonTiSdkBackend` only ever **read** the optics database
(`mounted_filters()` / `mounted_objectives()`); the only writes in the module
were `Position`/`RawValue` on the **motion** devices. `MicroscopeConfigStore` is
confirmed entirely local (every mutator ends in an atomic JSON write, zero COM).

The pre-existing comment *"Code 0 = nothing fitted; every other field then
raises 'No database code is associated with this optical element'"* already
pointed at a catalogue lookup rather than free text — which is exactly what the
hardware then confirmed, from both directions.

## Files Modified

- `SupportClasses/MicroscopeControl.py`
  - `MicroscopeBackend.probe_optic_write_support()` — **READ-ONLY** contract,
    returns `{device: {field: True|False|None}}`; docstring carries the
    false-positive warning.
  - `MicroscopeBackend.set_optic_name()` — contract; refuses by default.
  - `NikonTiSdkBackend._declared_writability()` (pure introspection of the
    comtypes wrapper's `property` descriptors), `probe_optic_write_support()`,
    `_first_optic()`, `_OPTIC_IDENTITY_FIELDS`, `_OPTIC_COLLECTIONS`.
  - `NikonTiSdkBackend.set_optic_name()` — the one genuine write: single
    target, **`Name` only, never `Code`**, read-back **verified**, refuses a
    declared-read-only field up front, surfaces the SDK's own words. Docstring
    records the full hardware finding.
  - `MicroscopeState.optic_write_support`; `_Op.result`;
    `MicroscopeController._submit_op()` (extracted from `_submit`),
    `.probe_optic_write_support()`, `.set_optic_name()`; `disconnect()` clears
    the new state field.
- `gui/pages/hardware/microscope_setup_panel.py` — **"🔎 Check write
  support…"** button; `_probe_write_support()` (surfaces `op.error` instead of
  rendering a stale result as a hardware answer); `_format_write_support()` +
  `_WRITE_VERDICT_TEXT` with three verdicts, the "writable" one carrying the
  hardware-verified false-positive warning.
- `tools_microscope_hw_check.py` — runs the read-only check (writes nothing).
- `tests/test_v75x_nikon_ti_microscope.py` — +26 tests and 4 new comtypes-shaped
  fakes (`_ReadOnlyOptic`, `_WritableNameOptic`, `_IgnoringNameOptic`,
  `_RefusingOptic`, `_RecordingOptic`).

## Implementation Steps

- [x] Audit for an existing write path (none) and for prior evidence either way
      (none).
- [x] Read-only writability inspection + the single verified write.
- [x] Controller ops; `_submit_op` so an op's `fn` can close over its own op.
- [x] Panel + bench-tool surfaces.
- [x] Unit tests — **247 green** across `test_v710_microscope_bore_sign`,
      `test_v710_needle_bore_wizard`, `test_v711_microscope_focus_state_bug`,
      `test_v711_objective_ladder`, `test_v75x_nikon_ti_microscope`,
      `test_test_suite_hygiene`, plus a clean `gui.app` import.
- [x] **Ran the operator's CY5 test on the real body** → refused, as above.
- [x] Recorded the finding in code (×3 sites), tests and this doc.
- [ ] Nothing further is pending in software. If the body's own display must
      change, that is a hardware/vendor-tooling task: fit a coded optic, or use
      Nikon's configuration tool to write the database this SDK will not.

## Testing Notes

`python -m unittest tests.test_v710_microscope_bore_sign
tests.test_v710_needle_bore_wizard tests.test_v711_microscope_focus_state_bug
tests.test_v711_objective_ladder tests.test_v75x_nikon_ti_microscope
tests.test_test_suite_hygiene -q` → **249 green**.

**6/6 mutations CAUGHT** (verified by monkeypatching the production methods, no
source edits):

| Mutation | Result |
|---|---|
| probe writes a field's own value back again (the original design) | CAUGHT |
| `_declared_writability` always returns True | CAUGHT |
| the declared-read-only refusal in `set_optic_name` is dropped | CAUGHT |
| `set_optic_name` trusts the write instead of reading back | CAUGHT |
| a rename also writes `Code` | CAUGHT |
| the pre-refactor `_submit` closure bug (see below) | CAUGHT |

## Issues & Decisions

- 🔴 **The first cut's safety argument was wrong, and this is the most
  important entry here.** It settled writability by writing each field's **own
  current value back to itself**, claiming that "can never change what the body
  reports." `tucam_backend._capa_set` is a hardware-verified counterexample in
  this very repo: *"writing a capability the value it ALREADY holds is not a
  harmless no-op on this camera"* — a redundant `auto_exposure` write reset the
  exposure to the sensor minimum and blacked out the preview. Worse, the probe
  included **`Code`**, which on the nosepiece resolves the NA and working
  distance that bound a focus sweep (`ObjectiveLadder`, plate-bed levelling) —
  so a corrupted one is a **collision hazard**, not a cosmetic bug. Replaced
  with pure type-information inspection; pinned by
  `test_write_support_check_writes_nothing_at_all`, which asserts against a
  **recorded** write list so a reintroduced write cannot hide inside a broad
  `except`.
- 🐞 **A closure bug I introduced and caught before shipping.** `set_optic_name`
  needs its `fn` to report a result back onto its own `_Op`. Written the obvious
  way (`op = self._submit("…", _do)` with `_do` referencing `op`), the
  **`threaded=False`** path — the one the entire suite uses — executes `_do`
  *inside* `_submit`, before `op` is bound, dying with a `NameError` swallowed
  into `op.error`. Fixed by extracting `_submit_op(op)` so the op is fully built
  first; pinned by inline **and** threaded tests, and mutation-confirmed.
- ⚠ **One of my own tests was wrong and the code was right.**
  `test_set_optic_name_never_touches_code` failed because `_RecordingOptic`
  declared `Name` read-only, so `set_optic_name` correctly refused *before*
  writing — the test's premise, not the behaviour, was broken. The fake now
  declares a setter so the write is actually reached.
- **Rejected:** adding a "push name to the body" button to the panel. The answer
  is now known to be *no* for this body, so a button that always fails is worse
  than none. `set_optic_name` stays in the backend — it is what produced the
  answer and is the one call that can settle another body or SDK generation.
- **Rejected:** having `SimulatedMicroscopeBackend` report a fake "supported"
  result to exercise the UI branch. That would misrepresent what is known about
  hardware; the simulator inherits the honest default and the "writable" branch
  is tested against hand-built dicts.
- Probed `Name` + `Code` only — the two fields that plausibly correspond to what
  a human reads on the LCD. NA / WD / Magnification are optical properties
  nobody renames.

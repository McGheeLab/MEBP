# MEBP v7.17.1 — three GUI-thread stalls and the redundant I/O behind them

## Objective

Fix three defects surfaced by a single operator log (2026-08-13, ME3B_01). All
three are the same shape: work that must not sit on the Qt event loop was
sitting on it. None of them was reported as a bug — they were found because
v7.16's `GuiWatchdog` started writing stalls down, and because the log itself
was being rotated at an implausible rate.

⚠ The logs that produced this analysis were deleted mid-session (they are
gitignored runtime files, not recoverable). Every number below was taken from
them before that, or re-derived from the source afterwards.

---

## The three defects

### 1. Camera settings: a store write per slider tick

`gui/dialogs/camera_settings_dialog.py::_make_slider` wires every slider on
`valueChanged` → handler → `_persist()`, and `_persist` did a full
`CameraCalibrationStore` write: JSON serialise + atomic file replace, **on the
GUI thread**, at the rate the mouse moves.

Measured in the operator's log: one gamma drag (84 → 128) wrote
`camera_calibrations.json` **~50 times between 14:26:51 and 14:27:00**.

This is the identical failure the v7.16 crop-offset spins had (40 writes in
7 s → freeze report), and it takes the identical remedy — which had simply
never been applied to this dialog.

**Fix.** The live push to the camera stays **immediate** (aiming a control is a
visual task; the preview must follow the slider). Only the WRITE is debounced,
400 ms, matching `hardware_setup._flush_crop_persist`. Flushed on
`hideEvent` / `closeEvent` / `aboutToQuit`, so a pending write cannot be lost by
closing the dialog mid-drag.

`_persist()` is **debounced by default**, and the seven discrete decisions
(resolution, auto-exposure, Andor auto-scale, sensor bool, sensor enum, signal
optimize, Defaults) pass `now=True`. Defaulting this way is deliberate: a call
site that forgets to ask for an immediate write merely lands 400 ms later and
is still flushed on teardown, whereas defaulting to immediate would let a
future slider silently reintroduce the storm.

### 2. `PrintFileManager.load` rescans the whole prints directory

`load(name)` resolves a display name by globbing `prints_dir` and
**JSON-parsing every file** to match on `metadata.name`. One call is O(number
of prints) in file opens. The Quick Print readiness / planned-path refreshes
call it per tick, via `_path_segments_for_selection()`.

Measured: **302 loads of `Four_Circles_1` in one session** over 39 prints —
roughly **12,000 file opens** on the GUI thread — and each one then re-read and
re-parsed the same CSV through `import_csv_trajectory`. Two INFO lines per load
is what churned five 8 MB log files in a day.

**Fix.** Two caches, both keyed on the file's own `(st_mtime_ns, st_size)` so an
edit on disk invalidates itself:

- `PrintFileManager._load_cache` stores the **validated + migrated raw dict**.
  A repeat load costs one `stat()` — no scan, no parse, no validation, no
  migration. A fresh `PrintFileData` is still built from a **deep copy** every
  call, because `PrintFileData.from_dict` assigns `pf.objects = data["objects"]`
  **by reference** and the print pages mutate what they are given.
- `TrajectoryPlanner._CSV_CACHE` stores the parsed array and hands out a
  **copy**, so no consumer's in-place edit can rewrite the cached trajectory.
  An invalid file (non-monotonic time) is never cached — it must raise, and be
  re-reported, every time.

### 3. Calibration travel froze the whole application

`CalibrationPage._safe_navigate_to` ran `safe_travel_to` (raise Z → M400 +
`wait_for_z_arrival` → XY → `wait_for_xy_arrival` → optional descent) directly
on the GUI thread. On ME3B V1 a real retract is 11–14 s.

Measured: a **10.0 s stall** at 14:26:24, and **29 stalls** in `freeze.log`
across 2026-08-11 → 08-13, all 5–7 s. Every camera feed is a `QTimer`, so they
all froze with it.

**Fix.** New module-level `run_safe_travel_responsive(page, …)` in
`gui/pages/calibration.py`. The travel is byte-identical — same call, same
arguments, same retract-before-XY ordering, same abort-if-unconfirmed
semantics. Only *where it runs* changes: the move goes onto the existing
`SafeTravelWorker` daemon thread while a nested `QEventLoop` keeps this
thread's event queue turning.

The synchronous `bool` contract is preserved deliberately rather than
restructuring into callbacks: **twelve** call sites branch on it, and
`_on_park` arms a measurement session on it.

⚠ **The modal wait dialog is load-bearing, not cosmetic.** A frozen GUI
accidentally provided a safety property — the operator physically could not
issue a second motion command mid-travel. Spinning the event loop hands that
ability back, and a Z jog issued while the stage is travelling XY is exactly
the crash this page's safe-travel rules exist to prevent. Application-modal
input blocking restores the property while still letting QTimer repaints
through. It appears only after 400 ms, because a travel that starts already
retracted is a sub-second no-op that must not flash a dialog. There is
deliberately **no Cancel button**: `safe_travel_to` has no mid-flight abort, so
a Cancel that left the stage moving would be a lie about the machine's state.
A `_nav_travel_in_flight` guard is the second line of defence.

⚠ **It is a FREE FUNCTION, not a method.** The calibration suites drive these
handlers through `SimpleNamespace` pages carrying only the attributes a handler
touches; a new bound helper breaks every such call site. This repo already paid
for that lesson in v7.12 (four click-rim tests). A first cut here was a method
and broke `test_v791_calibration_and_layout_fixes::TestSafeNavigateNeverFallsBackToZero`
— caught by regression, then converted. Qt parentage is applied only when the
page really is a `QWidget`.

---

### 4. 🔴 STARTUP CRASH — `MicroscopeState` is missing its illumination fields

Reported after the above landed: **the application would not start at all.**

```
HardwareSetupPage() → MicroscopeSetupPanel() → load() → _refresh_live()
  → _illumination_summary(state)
AttributeError: 'MicroscopeState' object has no attribute 'epi_shutter_present'
```

Page 0 is built unconditionally in `MainWindow._create_pages`, so this is a
hard startup failure, not a degraded tab.

**This is NOT a consequence of the accidental branch switch.** Verified:

```
git log --all -S "epi_shutter_present" -- SupportClasses/MicroscopeControl.py
    → no commits, on any branch
git grep -l "epi_shutter_present" HEAD
    → gui/pages/hardware/microscope_setup_panel.py
      tests/test_v717_microscope_illumination_and_interlock.py
```

The v7.17 illumination work was **partially committed**: the GUI panel and its
96-test suite landed, the `SupportClasses/MicroscopeControl.py` backend never
did — on `7.17.1`, `Version-7.17.0`, or either remote. The panel has therefore
been referencing fields that have never existed in any commit.

**Fix (scoped to restoring startup).** Added the eleven fields the panel reads
to `MicroscopeState`, with defaults that mean *not fitted / not known*:
`epi_shutter_present/_open`, `dia_lamp_present/_on/_intensity/_min/_max/_remote`,
`light_path_position/_count`, `native_light_path_names`.

⚠ The value fields default to **`None`, not `False`**. The panel renders
absent / unknown / known as three distinct readings, and for a shutter that is
the difference between "the sample is protected" and "we do not know". A
comfortable default would be a fabricated measurement.

⚠ **NOTHING POPULATES THESE.** The panel now honestly reports *"not fitted"*
for all three accessories. That is correct for the epi shutter on this Ti-E
(the 2026-08-12 session found `IsMounted=0`), but the dia lamp and light path
ARE fitted and will read as absent until a backend is wired.

**Deliberately NOT done: reconstructing the illumination backend.** The missing
surface is ~20 methods —

`has_epi_shutter` · `epi_shutter_open` · `set_epi_shutter` · `_shutter_codes` ·
`epi_shutter_interlock_enabled` · `has_dia_lamp` · `dia_lamp_on` ·
`set_dia_lamp_on` · `dia_lamp_intensity` · `set_dia_lamp_intensity` ·
`dia_lamp_intensity_range` · `dia_lamp_remote` · `set_dia_lamp_remote` ·
`get_light_path` · `set_light_path` · `light_path_count` · `light_path_names` ·
`_prime_devices` · `_state_from_text` · the `set_filter` shutter interlock

— including a **sample-safety interlock** that closes the excitation shutter
across a cassette rotation, and a COM device-priming fix. Rebuilding that from
a prose description, unverified against a body whose lamp is currently in
front-panel MainMode and which has no epi shutter fitted, is a separate task
that needs the hardware. `tests/test_v717_microscope_illumination_and_interlock.py`
(96 tests, currently 85 failing) is a precise executable specification for it.

### 5. 🔴 "The software started to get very slow" — a stale Prior ack corrupts every position read

Reported 2026-08-13 15:51. The Prior acks **every** command with a bare `R`,
including the `SMS`/`SAS` setters. From the operator's log:

```
15:51:42  set_velocity: 10000 µm/s → SMS 20%      → R
15:51:42  XY relative move: (5000, 0)             → R
15:51:43  set_velocity: 10000 µm/s → SMS 20%      → R
15:51:43  XY relative move: (5000, 0)             → R
15:51:43  Failed to parse XY position: Expected 3 values, got 1: R
```

`get_current_position` already calls `reset_input_buffer()` before querying
(the v7.5.0 hotfix), **but a flush can only discard bytes that have already
ARRIVED.** An ack still in flight when the flush runs lands immediately after
it, so `P` goes out and the first line back is that ack, not the position.

The consequence is the reported slowness, not a cosmetic log line: a failed
parse returns `(None, None, None)`, which clears the poller's
`_last_position_read_ok`, makes `wait_for_xy_arrival` poll garbage until it
times out, and leaves every cached-position reader stale.

**Fix.** New `_read_position_line()` reads replies after `P` until one is not a
bare ack. `_is_bare_ack` exact-matches the protocol's own
`ack_success_token` on the stripped line — **exact, not prefix**, because
`:A` alone is a Ludl ack while `:A 399 321` is a Ludl *position*.

⚠ **Deliberately a READ-side fix.** `_send_protocol_command` carries an
explicit v7.5.x investigation note: draining the ack after each WRITE was tried
on real hardware and made things *worse* — the Prior stopped answering `P` at
all. Nothing about the write path changes here. This only declines to mistake
an ack for a position, which is the one direction that note never tried.

Bounded two ways so a controller answering nothing but acks can never spin the
poll thread: a 0.5 s wall-clock budget and `_MAX_STALE_ACKS = 4`. An empty
reply is **not** treated as an ack, so a silent port still costs exactly one
read and is still reported as "no answer" — the existing liveness/disconnect
handling is untouched.

### ⚠ NOT FIXED — XY detection blocks the GUI thread at startup

The same log shows the other half:

```
15:51:20 → 15:51:26   three protocols × ports × {115200, 38400, 9600}
15:51:24 [E] GuiWatchdog: GUI thread has not run for 5.4s
15:51:27 [W] GuiWatchdog: recovered after ~8.0s
```

Auto-detection walks every protocol × port × baud with a 1–2 s probe each, on
the GUI thread, so startup freezes for ~6–8 s. Two contributing facts worth
noting: the stage is at **9600**, the last baud tried, and `proscan_iii_h117`
is the last protocol tried — so the successful combination is the most
expensive one to reach.

Not fixed here because `connect_stages` is called both from `main.py` before
the window exists and from the Connect buttons; making it asynchronous changes
connect semantics for every caller and deserves its own verified pass.
Cheap partial mitigations if the freeze is what matters: persist the
last-good `(protocol, port, baud)` and try it first, and/or order
`baud_rate_candidates` with 9600 first for this machine.

### 6. Advanced Z references: open by default, resizable, plate-type round trip on the picker

Operator: *"I need to be able to expand up advanced plate z references to be
much taller. in fact this should not be hidden by default"*, then *"on the
references picker I need another button for assign these to the plate type for
auto application next time. and a button for apply plate offsets based on plate
top position from that assignment."*

**Layout.** The panel started collapsed and, once opened, shared the wizard
column 50/50 (`wiz` and `adv_content` were both added at `stretch=1`). What it
holds is the reference-height picker, the live XZ side view used to *drive* Z,
the needle-cam estimate group and the learn loop — so half a column left the XZ
view a sliver. `wiz` and the panel are now the two halves of a **vertical
splitter** (`_needle_loc_adv_split`), so the division is dragged rather than
hardcoded and the panel can be taken nearly full height; `setChildrenCollapsible(False)`
plus a `s(340)` minimum keeps it from vanishing. Default is now **open**. The
toggle remains for reclaiming space, and re-opening explicitly re-sizes the pane
so it can never reappear as a sliver.

**Assign button.** `_zoff_save_offsets_to_plate_type` already existed — it was
simply in a group further down *inside the panel that defaulted to hidden*,
which is why the round trip was undiscoverable. Surfaced as
`_zoff_btn_assign_to_type` directly under the reference rows.

**Apply-from-top button** is new. NEW `StageController.plate_z_refs_from_top()`:
stored offsets are mm below the needle-cam fiducial, so anchoring on a taught
top cancels the fiducial and keeps only the plate's own feature spacing —

    user_z(k) = user_z(top) − (offset[k] − offset["top"])

Once a plate type carries offsets, touching off only the *top* of a fresh plate
of that type places the rest, absorbing that plate's thickness and seating.

⚠ **The derived bottom is pushed with `source="estimated"`**, so
`print_floor_datum_zref()` withholds it from the print-floor clamp. An
estimated floor is unsafe in BOTH directions — too high blocks the very
touch-off that would measure it, too low passes a Z that punches through the
glass while the caller believes it is guarded (the v7.17 reasoning).
Plate Top is never overwritten: it is the input.

⚠ **Returns `None` rather than zeros when nothing is stored** — every delta
would be zero, reporting the well floor as exactly the plate's top surface.

NOTE — the bore wizard's step 2 already uses the same identity for one pair
(`_plate_top_bottom_distance` = `bottom − top`). That yields a scalar to seed a
typed spin; this maps every reference into the zero-ref frame. Kept separate
deliberately: folding the hardware-verified v7.13 path into this would change a
tested descent-planning input for no functional gain. Cross-referenced in both
docstrings so they cannot drift unnoticed.

---

## Files Modified

| File | Change |
|---|---|
| `gui/dialogs/camera_settings_dialog.py` | Debounced `_persist(now=False)` + `_flush_persist`; 7 discrete sites pass `now=True`; flush on hide/close/quit; `bool()` hardening on `res_ok` |
| `SupportClasses/PrintFileManager.py` | `_stat_key()` helper; `_load_cache`; fast path in `load()` |
| `SupportClasses/TrajectoryPlanner.py` | `_CSV_CACHE` + copy-on-return in `import_csv_trajectory` |
| `gui/pages/calibration.py` | New free `run_safe_travel_responsive`; `_safe_navigate_to` routes through it |
| `SupportClasses/MicroscopeControl.py` | 11 illumination-accessory fields on `MicroscopeState` — fixes the startup crash |
| `SupportClasses/XYStage.py` | `_is_bare_ack` / `_read_position_line` / `_MAX_STALE_ACKS`; `get_current_position` skips stale acks |
| `tests/test_v7171_xy_stale_ack_position_read.py` | NEW — 12 tests, 5/5 mutations caught |
| `SupportClasses/StageController.py` | NEW `plate_z_refs_from_top()` |
| `gui/pages/calibration.py` | Advanced Z panel open by default + vertical splitter; `_zoff_btn_assign_to_type` / `_zoff_btn_apply_from_top` / `_zoff_apply_offsets_from_top` |
| `tests/test_v7171_z_reference_panel_and_plate_offsets.py` | NEW — 14 tests, 7/7 mutations caught |
| `tests/test_v713_needle_loc_tab_layout.py` | `..._collapsed_by_default` → `..._OPEN_by_default` (contract deliberately inverted) |
| `tests/test_v7171_gui_responsiveness_and_caching.py` | NEW — 25 tests |
| `tests/test_v75x_camera_hardware_controls.py` | `test_live_apply_and_persist` updated for the genuinely-changed timing |

---

## Implementation Steps

- [x] Debounce camera-settings persistence; classify continuous vs discrete
- [x] Flush on hide / close / aboutToQuit
- [x] `PrintFileManager` load cache with deep-copy hand-out
- [x] CSV parse cache with copy-on-return
- [x] Calibration travel off the GUI thread behind a modal wait
- [x] Convert to a free function after regression caught the stub breakage
- [x] 25 new tests; 10/10 mutations caught
- [x] Regression parity vs a clean `git worktree` at HEAD

---

## Testing Notes

`tests/test_v7171_gui_responsiveness_and_caching.py` — **25 tests, green**.

**10/10 mutations CAUGHT**, sources restored byte-identical:
debounce removed · live push debounced too · hide-flush removed · load cache
removed · load cache never invalidated · load cache hands out a shared object ·
CSV cache removed · CSV cache returns by reference · travel back on the GUI
thread · re-entrancy guard removed.

⚠ **Three of my own tests were too weak and mutations caught them first** — all
the same mistake, and one this file keeps recording:

- The shared-object test mutated `metadata`, which `from_dict` **rebuilds**;
  only `objects` is assigned by reference. It also compared the *first*
  (uncached) load against a cache hit — those differ even with the copy
  removed. Rewritten to compare **two cache hits** and mutate `objects`.
- The CSV corruption test had the same first-call flaw: the first call returns
  the freshly parsed array, not the cached one.
- The freeze test drove the **helper** directly, so reverting `_safe_navigate_to`
  to the blocking call survived it. Rewritten to drive the production entry
  point (`test_the_PRODUCTION_entry_point_is_the_responsive_one`).

⚠ **PROCESS INCIDENT — a test wrote into the operator's live calibration file.**
An earlier revision let `_flush_persist` reach the real
`CameraCalibrationStore` singleton and wrote a bogus `fake:0` camera into
`config/hardware/camera_calibrations.json`. The addition was purely additive (no
real camera entry was altered), was removed surgically, and the file verified
byte-identical to HEAD (`git diff` empty, JSON re-parsed). The suite now
redirects `get_store` to a temp dir **before** constructing the dialog, with
`test_this_test_cannot_touch_the_real_machine_store` pinning it. Note that
`CameraCalibrationStore` has **no env-var override** for test isolation, unlike
the other per-machine stores — worth adding.

Regression, per batch, compared against a clean `git worktree` at HEAD:

- print-library / migration / trajectory-regen / sketch-edit / sketch-trajectory
  / quick-print-trajectory-view / suite-hygiene — **86 green**
- camera hardware-controls / calibration-store / image-correction / square-crop
  / cal-liveview — **26 errors, EXACTLY the same 26 as HEAD** (`comm` diff
  empty). All are `MicroscopeState.epi_shutter_present`, which does not exist in
  `SupportClasses/MicroscopeControl.py` on this branch — the v7.17 illumination
  work is not present here.
- z-retract / plate-location-click-rim / v791-calibration / v717-step2 /
  needle-bore-wizard — **3 errors, identical to HEAD**
- `test_v73_trajectory_planner::TestFullPlanGeneration` — 6 errors, the
  MagicMock `<` in `PrintTrajectoryPlanner._move_z` already documented in
  CLAUDE.md as pre-existing; that file is not in this diff.
- `gui.app` import smoke — OK.
- **Startup**: a real `MainWindow(controller, settings)` on simulated stages now
  constructs cleanly (exit 0). Before the `MicroscopeState` fix this aborted in
  `_create_pages` → page 0.
- After the fix the camera batch above went from **26 errors to 0 — 155 tests
  green** (17 more tests than before, because suites that died in `setUp` now
  actually run). `test_v75x_nikon_ti_microscope` — **129 green**.
- `test_v717_microscope_illumination_and_interlock` — **96 tests, 85 failing**.
  Expected and pre-existing: it specifies the backend that was never committed
  (see defect 4). It has never passed in this repo's history.

---

## Issues & Decisions

1. **Debounce default.** Debounced-by-default with explicit `now=True` opt-in,
   so a forgotten call site degrades to a 400 ms delay rather than a lost write.
2. **Cache invalidation by `(mtime_ns, size)`**, not an explicit `clear()` on
   save/delete/rename. Content identity self-invalidates and cannot be forgotten
   by a future mutating method.
3. **Deep copy on cache hit** rather than returning a shared object. The parse +
   validation + migration is what was expensive; the copy is not.
4. **Synchronous contract kept** for `_safe_navigate_to`. Restructuring twelve
   motion-safety call sites into callbacks is a much larger change that wants
   hardware verification of its own.
5. **Modal block chosen over leaving the UI live.** Stated above — it replaces a
   safety property the freeze was providing by accident.
6. **`bool()` on `res_ok`** — not reachable today (`hardware_capabilities`
   always sets the key) but `setVisible(None)` raises, so a future backend
   omitting it would crash `reload()`.

---

## Needs GUI/HW verification on ME3B V1, IN ORDER

1. Drag the camera gamma/brightness/contrast sliders and confirm **the preview
   follows the slider with no lag** — the live push must still be immediate.
2. `logs/app.log` shows **one** `Camera hardware controls saved` per drag, not
   one per tick. Close the dialog mid-drag and confirm the value **survives a
   restart** (the flush).
3. Toggle auto-exposure / change resolution / press Defaults — each still writes
   **immediately**.
4. Open Quick Print, change well/object/settings repeatedly, and confirm
   `Loaded: …` / `Imported CSV: …` appear **once**, not per tick.
5. Edit a print file on disk while the app is running and confirm the change is
   picked up (cache invalidation).
6. **The safety one.** With the needle DOWN in a well, click a well on the
   Plate Location tab: a "Stage travel" dialog appears after ~0.4 s, **the
   camera feeds keep updating**, and — while it is up — confirm you **cannot**
   jog Z or start another travel. Watch the needle: it must retract before XY
   moves, exactly as before.
7. With the needle already retracted, the same click must **not** flash a
   dialog (sub-second no-op).
8. Confirm no new `GUI thread has not run for …` lines in `logs/freeze.log`
   during calibration navigation.

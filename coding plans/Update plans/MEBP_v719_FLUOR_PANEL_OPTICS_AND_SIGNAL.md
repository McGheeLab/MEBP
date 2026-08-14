# MEBP v7.19 — Fluorescence mosaic: a dedicated left panel, real optics control, per-cube signal recipes, tile-major scan

## Objective

Seven operator requests on the Fluorescence Mosaic workflow and its calibration surface:

1. Selecting an objective should **actually move the nosepiece** when the microscope is connected.
2. With several cubes selected, offer **two acquisition orders** — a full scan per colour, or every colour per tile.
3. An **exposure slider per cube**, live-applying, editable only for the cube in the light path; the
   camera taken **out of auto-exposure and auto white/black** on this workflow, with a **toggle
   between "camera defaults" and "fluorescence mosaic settings"**, defaulting to the latter.
4. The **objective-calibration table populated from the objectives defined on the Microscope page**,
   knowing which is in the path.
5. A **histogram** so the effect of an exposure change is visible.
6. Exposure alone is not enough — **open the other signal controls as sliders too**.
7. **All of these settings live on a left context panel dedicated to this workflow page.**

Operator's words: *"when i'm in fluorescence mosaic workflow, and i select a different objective at
the top, it should change the objective if the microscope is connected … when i select multiple
filter cubes at the top it should have two modes in the mosaic full scan of each color or scan of
each color per tile … under each filter cube i want an exposure slider … the camera needs to be set
out of autoexposure mode and auto white black balance mode … a toggle button for camera default
settings and camera fluorescent mosaic settings … in the microscope camera objective calibration the
objectives in the table should be auto populated based on the objectives defined in the microscope
page"*, then *"i also would like a histogram just above the well selection section"* / *"is exposure
enough to make a good image, should we also allow a way to open other options as sliders"*, then
*"lets make all of these settings on the left context menu for the fluorescence mosaic workflow.
this is a special left context panel just for this workflow page."*

---

## What the code actually did before this update (verified, not assumed)

- `_on_objective_changed` was a **software-only relabel**: it wrote
  `camera_config.current_objective_name` and pushed µm/px. It never rotated the turret and never read
  the body's position. Picking `20x` on a body sitting at `4x` silently rescaled every tile.
- **`SupportClasses/OpticsService.py` existed, fully specified and tested, with ZERO production
  callers.** `ensure_filter` / `ensure_objective` already handle the lease (re-entrant per thread;
  release only a lease they took), read-back verification, stale-drop-aborts, focus retreat before
  rotation, and parfocal. The missing piece was the GUI adapter its own docstring names —
  `gui/widgets/optics_ensure.py` — which did not exist.
- The page's **module docstring claimed cubes are switched automatically via `ensure_filter`. That
  code did not exist.** Every channel got an unconditional modal, labelled with
  `fms.channel_number(channel)` — a **deprecated ordinal the store's own comments say is not a turret
  slot**. On this rig it printed "mCherry channel (3)" for a TxRed cube and "Bright Field channel
  (5)" for an empty slot. `fms.channel_slot()` does the real resolution and had no callers.
- Channel iteration was a **GUI-thread recursion**, one worker per channel — structurally
  channel-major only.
- The **objective-calibration card and the Microscope page kept two unrelated objective lists**,
  joined by nothing. They already join *by string*: `ObjectiveLadder.resolve_ladder` hands the
  panel's name verbatim to the calibration store and `_resolve_key` folds case, so `4X`↔`4x` already
  resolves.
- **The fluorescence page returned `None` from `get_context_widget()`**, so it had no left box.
  `WorkflowsModePage.get_context_widget()` already delegates to the active workflow,
  `sub_page_changed` already fires on workflow open/close, and `ContextPanelHost.set_native_label()`
  already relabels the native pill — so a workflow-specific panel is a supported shape, not a new
  subsystem.
- **Genuine API gap:** the Tucsen's hardware auto-levels (`TUIDC_ATLEVELS = 8`) existed only as a
  constant and a diagnostics label — no getter or setter anywhere. Software display auto-scale was
  fully wired.

Stages C–E are **Stages 3–5 of `MEBP_v718_OPTICS_SINGLE_SOURCE_OF_TRUTH.md`**, designed and approved
and never implemented.

---

## Decisions (AskUserQuestion ×2, 2026-08-14)

| | Decision |
|---|---|
| Scan order | **Tile-major default**; channel-major stays as the option. |
| Camera preset | Auto-exposure **OFF** · hardware auto-levels + display auto-scale **OFF** · gamma/contrast/brightness neutral. |
| Objective combo | Connected + drivable ⇒ the pick **drives the turret**, refusing and reverting on failure. Disconnected or a manual turret ⇒ **trust the operator**, label only. |
| Objective list | **Derived from Hardware Setup → Microscope**, in turret order. |
| Signal sliders | Exposure · **analog gain** · **frame averaging** · **display black/white levels**. Binning excluded. |
| Recipe scope | **Per channel** — each cube keeps its own exposure/gain/averaging/levels. |
| ⚡Auto button | **Exposure only, unchanged** (`run_signal_optimize` stays as shipped). |

> **Why there is no illumination slider.** Lamp intensity is the biggest signal lever, and on this
> body it is not software-controllable: the v7.17 hardware session found `DiaLamp` accepts a level
> write then reverts to the front-panel knob within ~1 s, and no epi shutter is fitted. A control
> that silently does nothing is worse than no control.
>
> **Why binning is excluded.** 2×2 sum is 4× signal for free, but it changes µm/px — which
> invalidates the objective calibration *and* the mosaic tile spacing, the exact class of error that
> produced the v7.16 400-minute scan. It belongs behind a deliberate recalibration, not a slider.

---

## Files Modified

### New

| File | Rationale |
|---|---|
| `gui/widgets/fluorescence_controls_panel.py` | `FluorescenceControlsPanel` — the single home for every control. ONE class, two mount points (left context host when standalone, inline when embedded), so the two surfaces cannot diverge. |
| `gui/widgets/optics_ensure.py` | The GUI-thread adapter `OpticsService`'s own docstring names and which did not exist. `OpticsService` blocks on `op.done.wait`, so it is worker-thread-only. |
| `gui/widgets/signal_slider.py` | Compact log-scale slider + truthful readout for exposure / gain / averaging / display levels. |

### Modified

| File | Change |
|---|---|
| `gui/pages/workflows/fluorescence_mosaic_workflow.py` | all stages; docstring correction |
| `gui/pages/workflows_mode.py` | `context_label()` delegate beside the existing `get_context_widget()` |
| `gui/app.py` | `_refresh_left_context` consults `context_label()` before the page-class-name map |
| `gui/widgets/hw_controls_snapshot.py` | `apply_hw_controls` (the ordered applier) + `fluorescence_preset` + `auto_levels` key |
| `gui/pages/hardware_setup.py` | `_apply_hw_controls` delegates to the shared applier |
| `gui/widgets/tucam_backend.py` | `get_auto_levels` / `set_auto_levels` over `TUIDC_ATLEVELS` |
| `gui/widgets/camera_widget.py`, `camera_manager.py` | auto-levels fan-out + caps advertisement |
| `gui/pages/hardware/objective_calibration_card.py` | table derived from the nosepiece; `write_to_config` raw-name fix |
| `SupportClasses/MosaicBuilder.py` | public `apply_registration_from` |
| `SupportClasses/FluorescenceMosaicStore.py` | `cube_slot` / `cube_label` / `gain_pct`; docstring correction |

### Reused, not rewritten

`OpticsService.ensure_filter/ensure_objective` · `OpticsRegistry.resolve_slots/find_slot/optic_at` ·
`ObjectiveLadder.ladder_gate` · `FluorescenceMosaicStore.channel_slot` ·
`MicroscopeConfigStore.set_optic_alias` · `RawHistogramWidget` + `get_raw_frame_stats` ·
`mono_display.run_signal_optimize` · `MosaicBuilder._reblend_at_positions` ·
`ContextPanelHost.set_native_widget/set_native_label` ·
`hw_controls_snapshot.PERSISTED_HW_CONTROL_KEYS` · `responsive.container_scale`.

---

## Implementation Steps

### Stage A — the dedicated left context panel

- `[x]` `FluorescenceControlsPanel(QWidget)`, built lazily, owned by the page, driven by the page's timers
- `[x]` Standalone: `get_context_widget()` returns it; embedded: returns **`None`** and the same
  instance mounts inline instead (the host owns the box in the embedded case)
- `[x]` `context_label()` → `"Signal"`; delegated by `WorkflowsModePage`; consulted by
  `_refresh_left_context` before its page-class-name map
- `[x]` Narrow-width behaviour: `minimumSizeHint()` override, `responsive.container_scale`
- `[x]` Histogram at the top of the panel (see "Issues & Decisions" for the placement call)
- `[x]` Disclosure state via `isHidden()`, never `isVisible()`

### Stage B — camera: the preset, the recipe, the histogram (no motion)

- `[x]` B1 `tucam_backend.get_auto_levels` / `set_auto_levels` **through `_capa_set`**; widget +
  manager fan-out; caps entry gated on a non-None getter; `"auto_levels"` in
  `PERSISTED_HW_CONTROL_KEYS`
- `[x]` B2 `hw_controls_snapshot.apply_hw_controls(mgr, cam_idx, hw)` owning the load-bearing order;
  `hardware_setup._apply_hw_controls` delegates; `fluorescence_preset(caps)`
- `[x]` B3 preset toggle; entry snapshot on `showEvent` (subsuming `_entry_exposure_us`); restore on
  hide / toggle / run end; refused while scanning; nothing written to `CameraCalibrationStore`
- `[x]` B4 per-channel recipe `{exposure_us, gain_pct, avg_frames, display_lo, display_hi}`;
  `signal_slider`; 300 ms debounce then **re-read the achieved value**; `WorkflowSettingsStore`
  persistence with the panel and the popout as two views of one value; `avg_frames` per channel
  seeded from `mosaic_scan.avg_frames`; manual levels beat the probe freeze; `save_channel(gain_pct=)`

### Stage C — the optics are driven

- `[x]` C1 `optics_ensure.ensure_optics_or_prompt(...)`
- `[x]` C2 objective combo drives the nosepiece; reverts on refusal; label-only when not drivable;
  refused while scanning; combo populated from nosepiece slots
- `[x]` C3 `channel_number` → `channel_slot` in the prompt; `ensure_filter` skips the modal; alias
  binding offered on refusal; active channel read from the body; `save_channel(cube_slot=, cube_label=)`;
  both module docstrings corrected

### Stage D — objective-calibration table from the Microscope page

- `[x]` Rows from `OpticsRegistry.resolve_slots(kind=OBJECTIVE)` in turret order; live position marked
- `[ ]` **Selection drives the turret through C1 — NOT DONE.** ⚠ I marked this
  `[x]` in the first pass and it was never written; the correction is recorded
  rather than quietly amended, because a false `[x]` is worse than a missing
  one. `ensure_objective` refuses without `glass_focus_um` and refuses outright
  on `needle_retracted=False`, and **this card can supply neither**: it receives
  a `HardwareConfig`, not calibration data, and there is no controller-side safe
  Z accessor (`StageController` has `needle_at_or_above`/`ensure_retracted_to`,
  both of which take the safe Z as an ARGUMENT). Driving from here would
  therefore pass `needle_retracted=None` — a weaker safety posture than the
  fluorescence panel, which does judge it — so it is left undone and reported
  rather than shipped hopefully. See "🐞 v7.19.1" below for what the card does
  instead.
- `[x]` Manual add/remove only as a fallback; "Remove" means `clear_calibration`
- `[x]` 🐞 `write_to_config` writes `currentData()`, not the decorated `currentText()`

### 🐞 v7.19.1 — the "← in path" marker went stale (bench, 2026-08-14)

Operator: *"i just switched objectives while monitoring the microscope camera
objective calibration section, it did not change the objective in path"*, then
*"the physical objective changed, but didnt update the in path"* — and, on the
general point, *"that means that the reading and changing of any objective and
filter cube is now suspect on all pages"*.

**The defect.** Stage D resolved the live nosepiece position in exactly two
places — `apply_config` (page build / config push) and inside
`_on_objective_changed` — and the combo's decoration was rebuilt **only** by the
former. There was no timer anywhere in the card. But this card is not the only
thing that moves the nosepiece: `microscope_panel` (jog card),
`microscope_setup_panel` (Hardware Setup → Microscope), the v7.19 fluorescence
panel, `plate_level_wizard` and the operator's own hand all do. So the moment the
turret was driven from anywhere else the marker froze — and a frozen "← in path"
marker does not merely fail to update, it goes on making a POSITIVE and false
claim about the hardware.

**Fixed** with a 1 s poll (`_LIGHT_PATH_MS`, matching `microscope_panel` and the
controller's own ~1 Hz sampling) started on `showEvent` and stopped on
`hideEvent`, re-decorating only when the position actually changes. Three
properties, each a distinct way to get this wrong:

- **Display only.** The poll never writes `current_objective_name` and never
  pushes µm/px. Adopting a polled position as the declared objective would be a
  background write of the key the whole µm/px chain hangs off — the mutation the
  v7.18 optics design names as *"the one most likely to be proposed in good
  faith"*. Pinned by a test that drives the poll and asserts both stayed put.
- **It never yanks an open drop-down shut.** Rebuilding a combo clears it, which
  closes its popup under the cursor — the v7.5.x Nikon Ti defect (*"every time it
  reads it cancels the dropdown box I have opened"*). Guarded with the same
  `view().isVisible()` check `microscope_panel._popup_open` uses; a CLOSED combo
  still tracks.
- **A disagreement is STATED.** The card is where the objective is *declared*,
  and the declared name is what µm/px is keyed by. When the body has a different
  objective in the path, the note now reads *"⚠ 20x is in the light path — µm/px
  is being taken from 4X"* instead of a green *"Calibrated: 1.8968 µm/px"* tick
  over a number that is not forming the image. That disagreement IS the scale
  error this whole area exists to prevent.

⚠ **My own first test was wrong and offscreen Qt caught it**: it forced the popup
with `view().setVisible(True)`, which cannot work — `isVisible()` is False while
any ancestor is unshown (the trap CLAUDE.md already records), so the guard read
"closed" and the test failed against correct code. Driven through the real
`showPopup()`/`hidePopup()` instead, which is what the production check sees.

### 🔴 v7.19.1 ROUND 2 — the audit the operator asked for, and the root finding

Operator, on the general point: *"that means that the reading and changing of any
objective and filter cube is now suspect on all pages"*. An audit of every GUI
surface that reads or writes turret/cassette state says: **yes, and the reason is
one fact underneath all of them.**

**🔴 `MicroscopeController` DOES NOT POLL ITSELF.** `state()` returns a cached
snapshot updated only by `_read_all()` after an op, or by an explicit
`refresh()`; its worker loop waits on a *command queue* and reads nothing on its
own. Repo-wide, `refresh()` had exactly **two** callers — `microscope_panel._tick`
(only while that panel is on screen) and `OpticsService._ensure` (only around a
switch we command).

So four surfaces "poll" a snapshot that, on their own pages, **nobody refreshes**:
the objective-calibration card, `microscope_setup_panel`'s live status + green
current-slot border, the v7.19 fluorescence panel when standalone, and by
extension anything reading `optic_at(state, …)`. And the failure is *asymmetric*
in the worst way: an app-driven switch propagates (`set_objective` calls
`_read_all` afterwards), so **the staleness is invisible in exactly the case a
bench test is most likely to try**, and only shows up for a turret turned BY
HAND. My round-1 fix above added a 1 s poll to the card and would have been
inert for that case.

Fixed with ONE shared `optics_ensure.request_state_refresh()` rather than a
fourth copy of the policy — three refusals, each a real cost, each pinned:
**never under a busy or LEASED body** (`STALE_OP_S` drops a queued op silently,
so the one displaced could be the one that mattered), **never faster than 1 s
process-wide** (several panels can be alive at once; a per-widget throttle
multiplies hardware reads by the number of visible surfaces), and **never
blocking** (`refresh()` returns an op handle; the callers are timers and read the
cache on their next tick). Wired into the card, the fluorescence panel and
`microscope_setup_panel`.

**🔴 TWO DEFECTS IN MY OWN v7.19 CODE, both found by the audit:**

1. **A CONFIG LOAD was rotating the nosepiece.** `_refresh_objectives` ends by
   re-applying the restored objective and is called from `set_hardware_config`.
   That was harmless while `_on_objective_changed` was a software relabel — v7.19
   gave it a second job (`_drive_objective` → `ensure_optics_async`), so loading a
   saved hardware setup commanded a turret rotation nobody asked for. It now
   calls `_adopt_objective`, which is exactly the half that line ever wanted:
   record the name, push its µm/px, touch no hardware. The handler's own
   docstring claim — *"Only this last, operator-initiated GUI-thread path writes
   `current_objective_name`"* — was false because of this call site.
2. **A REFUSED rotation left µm/px on the objective that never arrived.**
   `ensure_optics_async` is asynchronous, so `_adopt_objective` has already
   written `current_objective_name` and pushed µm/px into `CameraManager` by the
   time a refusal lands. Reverting only the COMBO left the screen right and the
   numbers wrong — the worst arrangement, because nothing on screen disagrees
   with anything else. `_revert_objective_combo` now returns the objective the
   body actually has and the refusal path re-adopts it.

Also fixed: the fluorescence panel re-indexed its objective combo every second
**with no popup guard**, moving the highlight under the operator's cursor
mid-selection — the defect `microscope_panel` has guarded since v7.5.x and this
panel copied none of.

**Reported, NOT fixed** (pre-existing, outside what was reported, each its own
change): `hardware_setup`'s disabled *"Installed objective"* combo is sourced
entirely from the declared config field and never reads the body, so its label
is a claim it cannot support; `calibration.py`'s per-slot context panel pushes
µm/px from a combo populated by the calibration *library* with no relationship to
the turret at all; `needle_bore_wizard` files a measurement made through the
LIVE objective under the DECLARED objective's key; `live_target_picker` derives
its scale from the declared name with no live read. `ObjectiveLadder.ladder_gate`
is the one place a declared-vs-live disagreement is a hard block, and only the
plate-level wizard consults it.

Tests round 2: +7 `test_v719_objective_table_from_microscope` · +6
`test_v719_fluor_optics_drive` · +2 `test_v719_fluor_context_panel`;
**8/8 further mutations CAUGHT** (card polls a cache nobody refreshes · throttle
removed · polls under another's lease · polls a busy body · **config load rotates
the nosepiece again** · **a refused rotation leaves µm/px on the wrong
objective** · panel re-indexes an open drop-down · panel polls a stale cache).
Regression **508 green** across the v7.19 + optics + microscope + fluorescence
suites, +96 across context-panel/fluor-capture/signal-optimize/print-queue, a
`gui.app` import smoke, and a real offscreen `HardwareSetupPage` through all 11
tabs. Two `plate_builder_ui::TestLearnLoopSavesToADesign` failures PROVED
pre-existing in a `git worktree` at HEAD (never `git stash` in this repo).

### 🐞 v7.19.3 — the panel's values were never written to disk

Operator: *"the values assigned here should be persistant on this page"*.

**They were being written into exactly the right widgets — and nothing ever
wrote the FILE.** Every recipe control is a registered `WorkflowSettingsDialog`
field, so the round trip looked complete; but `save_last()` is reached only from
that dialog's own `hideEvent`/`closeEvent`, and the page hides the popout only
`if self._settings_dialog.isVisible()`. So the save ran **only for an operator
who opened the ⚙ popout and closed it again** — and the entire point of the
v7.19 left panel is that they never need to. Every slider change was lost on
restart. Fixed with a 500 ms debounced save scheduled from `set_channel_recipe`
and flushed on `hideEvent` **after** the panel's own slider flush (reversing
those two loses the last edit every time).

Debounced rather than written per commit because ⚡Auto and a channel switch
commit several values in a burst, and the v7.16 camera-crop incident is the
recorded cost of a JSON write per widget step.

**Two panel controls were not in the profile at all.** Scan order and the
camera-preset toggle were plain page attributes with hardcoded defaults, so they
reset to *tile* / *fluorescence* on every launch however well the recipe saved.
Both now go through `register_external` — the v7.7 hook for exactly this (a
widget has one parent, so a control promoted onto the panel cannot also live in
a settings section). Registered where the panel EXISTS, not in
`_build_settings_dialog`, which runs before it and must stay that way.

**🐞 And that exposed a gap in the shared store:** `widget_value` /
`set_widget_value` / `connect_widget_changed` recognised only `QCheckBox`.
`register_external` accepts any widget, so the preset toggle — a checkable
`QPushButton`, because it reads *"📷 Fluorescence" / "📷 Camera defaults"*,
which a checkbox cannot — was accepted, silently read back as `None`, and never
saved. Widened to `isinstance(w, QAbstractButton) and w.isCheckable()`:
`QCheckBox` IS a `QAbstractButton` and is always checkable, so this is
behaviour-identical for every existing field, and a non-checkable button still
falls through to "unrecognised" (pinned both ways — widening it to every button
would start persisting action buttons).

**Also removed a mirror:** `scan_order()` read a page-level `_scan_order`
attribute updated by the panel's change handler. Once the combo is a restored
settings field there are two ways it can move — an operator pick and `apply()` —
and only one was guaranteed to update the mirror. It now reads the widget.

Tests: +6 `test_v719_fluor_signal_recipe` (a real `WorkflowSettingsStore` round
trip through a fresh page = "restart") · +3 `test_v75x_workflow_settings_popout`;
**7/7 mutations CAUGHT**. ⚠ **The most important one SURVIVED the first run**:
my round-trip test called `_flush_recipe_save()` by hand, so it passed with
`set_channel_recipe` scheduling nothing — i.e. with the reported bug still
shipping. It now asserts the timer is running and fires it, never calling the
flush directly. Same weakness that let the v7.18 setpoint keeper ship green.
⚠ Also: `workflow_settings_dialog.py` is **CRLF**, so `\n`-anchored multi-line
patches silently matched nothing — the trap already recorded for the v7.20
mutation harness. Regression **542 green** across every settings-dialog consumer
(Quick Print queue, print calibrator, all workflow popouts) plus a `gui.app`
import, and an end-to-end check that `exposure_ms_fitc: 250.0` and
`scan_order: channel` are on disk and reload into a fresh page.

### 🐞 v7.19.2 — the objective was never being read, and the signal strip is one row per control

Operator: *"flourescence mosaic is not autodetecting the objective and the
current filter. I changed both and it did not update"* · *"it should have a
highligh box around the current filter on the signal pannel"* · *"there should be
one slider for exposure gain averaging whie black"* · *"if we move the sliders
around they get applied to the active filter"* · *"as we build the mosic these
signals automatically update"*.

**🔴 The objective half of `panel_optics_state` was never a hardware read.** The
cube half genuinely was (`filter_position` → `_cube_slots`), but the objective
returned `_current_objective_name()` — i.e. `camera_config
.current_objective_name`, the app's OWN declared value. So the panel compared its
combo against a copy of itself: it could not detect a nosepiece anyone else had
moved, and it never would have, at any poll rate. New `_live_objective_name()`
resolves through `optic_at(state, objective_position, OBJECTIVE)` and falls back
to the declared name only when there is no body.

**Detecting it is not enough, so it is ADOPTED.** µm/px is keyed by NAME, so
noticing the objective changed and going on scaling every tile by the old one's
value is the silent scale error this area exists to prevent. New
`on_panel_objective_detected` declares the name and pushes its µm/px — and
deliberately does **not** drive (the turret is already there; commanding it would
be an unasked-for move) and **refuses mid-scan** (the raster is planned once
against one µm/px and replayed for every channel). ⚠ This is the one poll-driven
write of `current_objective_name`, and it is legitimate for a stated reason: what
v7.18 forbids is a background write of a DERIVED or guessed value; this is a
verified read-back of what is physically fitted, and the hardware is the
authority on that.

**One slider strip, not one per cube.** The first cut gave every cube its own
exposure slider and hid gain/averaging/black/white behind a "▸ More signal"
disclosure. That is worse on both counts — N−1 of those sliders were permanently
disabled decoration (only the fitted cube's may drive the camera), and the four
controls you reach for while judging a histogram were one click away and out of
sight. Now: five rows, always visible, re-rendered from whichever channel is in
the light path. They remain a per-channel RECIPE — what changed is that there is
one view of it instead of N.

**What replaces "which slider is enabled" as the guard** against editing the
wrong cube is that the strip NAMES the cube it writes to (`Signal — FITC`) and
the live pill gets a **highlight box**. The green `◉` alone was too quiet for the
one fact every slider below depends on.

**Following a running mosaic came almost free** — the active channel is read from
`filter_position`, and in tile-major the worker changes the cube at every tile —
but only because `OpticsService` refreshes the cache around its own switches. The
shared freshener refuses under a lease and a scan holds one, so without that the
strip would have frozen for the whole run. The sliders stay **read-only** while
scanning: following the run is a readout, and editing mid-run would change the
camera under a raster already planned around the old value.

Tests: +2 `test_v719_objective_table_from_microscope` · +6
`test_v719_fluor_optics_drive` · +7 `test_v719_fluor_context_panel`, and five
existing tests REWRITTEN for the genuinely-changed contract rather than deleted
(`_exposure[ch].isEnabled()` and the whole disclosure suite describe a UI that no
longer exists). **8/8 mutations CAUGHT** — objective read back from the declared
name (= the reported bug) · a detected objective shown but never adopted ·
detection adopting mid-scan · detection also driving the turret · no highlight
box · the strip not naming its cube · the strip not re-rendering when the
cassette moves · sliders live during a scan. ⚠ **One of those mutations SURVIVED
the first run** and was right to: I had tested the page's `on_panel_objective_detected`
and the panel's snap-the-combo, but nothing asserted the panel actually CALLS the
page — the wiring between two tested halves. A test for it was added rather than
the mutation retargeted away. Regression **423 green**, `gui.app` imports, and
the panel renders through a real `QScrollArea` at 340/200/130 px with no
horizontal scrollbar (compact at 130).

Tests round 1: +6 in `test_v719_objective_table_from_microscope.py` (**17** in the file);
**6/6 mutations CAUGHT** — timer never connected (= the reported bug) · timer
never started · poll keeps running off-screen · popup guard removed · the poll
adopting the polled objective as declared · the mismatch never stated. The
behavioural test fires through `_path_timer.timeout`, **never** by calling
`_poll_light_path()` by hand, because calling it directly would pass with the
timer completely unwired — the exact weakness that let the v7.18 setpoint keeper
ship broken with green tests; an AST pin on the `connect` call backs it up.

### Stage E — tile-major acquisition

- `[x]` E1 worker takes `channels: list[ChannelPlan]` + `order`; `channel_done` per channel;
  `OpticsService` constructed on the worker thread (re-entrant lease); levels frozen per channel at the probe
- `[x]` E2 register the reference channel only; `MosaicBuilder.apply_registration_from(reference)`
- `[x]` E3 tile-major requires every channel drivable, refuses by name otherwise; confirm dialog
  prices rotations, time and memory; `free_accumulators()` per channel as it is saved

---

## Testing Notes

New suites, **113 tests, all green**: `test_v719_fluor_context_panel` (17) ·
`test_v719_fluor_camera_preset` (21) · `test_v719_fluor_signal_recipe` (21) ·
`test_v719_fluor_optics_drive` (24) · `test_v719_tile_major_scan` (19) ·
`test_v719_objective_table_from_microscope` (11).

**11/11 mutations CAUGHT**, every source restored SHA256-identically and the baseline re-confirmed
green afterwards: preset skips auto-levels · a raw `TUCAM_Capa_SetValue` instead of `_capa_set` (the
write that blacks out the Libra) · the entry camera state never restored · the slider showing the
REQUEST rather than what the camera achieved · an objective change allowed mid-scan · the combo not
reverted on refusal · the prompt labelled with the deprecated ordinal again · each channel registered
INDEPENDENTLY in tile-major · a refused cube captured through whatever is fitted · the embedded
instance handing over its panel (stealing the host's box) · `write_to_config` storing the decorated
display text again.

⚠ **Run the mutation harness with `PYTHONDONTWRITEBYTECODE=1` / `python -B`.** Python invalidates a
`.pyc` on (mtime, SIZE), so a same-length mutation restored inside one mtime tick lets the
interpreter reuse the MUTATED bytecode — a red baseline against a verified-identical file. Recorded
in the v7.18 objective-catalogue work and hit again here.

🐞 **A harness bug worth recording:** the first restore pass used `Path.write_text`, which translates
`\n` → `\r\n` on Windows, so a file created with LF endings came back byte-DIFFERENT and the SHA
assertion fired. Content was intact (and the file is now CRLF like the rest of the repo), but a
harness that cannot restore is worse than no harness — it does binary I/O now.

### One test in an existing suite was updated, not deleted

`test_v713_andor_sensor_features::test_key_list_complete` pins the exact persisted key set, which
legitimately gained `auto_levels`. A sibling was ADDED asserting the two channels stay
distinguishable (`auto_levels` = the sensor's own black/white points, `andor_auto_scale` = the
software display mapping) — collapsing them would make a mosaic's raw data depend on a display
preference.

⚠ **One of my own tests was too weak and I caught it before shipping.** The "a manager without
`set_hw_auto_levels` is skipped" test subclassed the recording manager and `del`-ed the attribute —
which just finds the base class's method again, so it passed while proving nothing. Rewritten around
a standalone class that genuinely lacks the method.

**Regression (per-suite):** `test_v75x_fluorescence_mosaic` · `test_v713_fluor_capture` ·
`test_v713_signal_optimize` · `test_v713_exposure_framerate` · `test_v713_andor_raw_stats` ·
`test_v718_optics_service` · `test_v718_optics_registry` · `test_v718_objective_name_resolution` ·
`test_v718_objective_catalogue` · `test_v718_filter_cube_catalogue` · `test_v711_objective_ladder` ·
`test_v75x_nikon_ti_microscope` · `test_v74x_objective_calibration` ·
`test_v75x_camera_hardware_controls` · `test_v79_tucsen_libra_camera` ·
`test_v75x_andor_display_scaling` · `test_v75x_context_panel` · `test_test_suite_hygiene`, plus a
`gui.app` import smoke and offscreen builds of the real `HardwareSetupPage` and
`FluorescenceMosaicWorkflowPage`.

⚠ `test_v75x_fluorescence_mosaic::test_channel_numbers` asserts the hardcoded ordinal map. It is
still correct and passes unchanged — the ordinal itself did not change, only who is allowed to use
it, which the new `TestThePromptStopsPrintingAWrongFact` class pins by AST.

### Regression, per batch — all green except four PRE-EXISTING failures

| Batch | Result |
|---|---|
| The six new v7.19 suites | **113 green** |
| fluorescence + optics (`v75x_fluorescence_mosaic`, `v713_fluor_capture`, `v713_fluor_postprocess`, `v78_fluor_mosaic_shift`, `v718_optics_service`, `v718_optics_registry`, `v718_objective_name_resolution`, `v718_objective_catalogue`, `v718_filter_cube_catalogue`, `v711_objective_ladder`, `v75x_nikon_ti_microscope`, `v74x_objective_calibration`) | **444 green** |
| camera stack (`v79_tucsen_libra_camera`, `v75x_camera_hardware_controls`, `v75x_andor_display_scaling`, `v713_andor_sensor_features`, `v713_andor_raw_stats`, `v713_exposure_framerate`, `v713_signal_optimize`, `v713_tucam_raw_parity`, `v75x_camera_calibration_store`) | 276 run, **4 pre-existing failures** |
| mosaic + context panel + hygiene (`v75x_context_panel`, `v731_mosaic`, `v75x_unified_mosaic_calibration`, `v716_tucsen_mosaic_fov_and_intensity`, `test_suite_hygiene`) | **143 green** |

Plus a `gui.app` import smoke and an offscreen `HardwareSetupPage` built and switched through all 11
sub-pages.

**The four failures were PROVED not ours** in a clean `git worktree` at HEAD (never `git stash` in
this repo), where they fail identically: `v79_tucsen_libra_camera::test_sdk_config_dir_is_gitignored`
(the `.gitignore` went structural in v7.17 and no longer carries that literal line) ·
`v75x_andor_display_scaling::test_persists_per_identity` ·
`v713_andor_sensor_features::test_dialog_persist_routes_through_snapshot` ·
`v713_tucam_raw_parity::test_avg_request_serviced_by_plane_path`.

⚠ **A concurrent session was editing this tree throughout.** `SupportClasses/StageController.py`
(`is_position_poller_suspended`), `SupportClasses/PrintQueue.py` and `gui/widgets/jog_well_plate.py`
carry another v7.19 change (a Quick Print held print queue) that this work neither made nor touched.
The two coexist — different files and different areas — and the regression above ran with both
present.

---

## Needs bench verification on ME3B_01, IN ORDER

1. Open Workflows → Fluorescence Mosaic: **the left box appears** with a "Signal" pill; go back to
   the picker and it disappears. Drag the splitter narrow — nothing clips.
2. The camera visibly stops auto-adjusting; toggle **Camera defaults ⇄ Fluorescence** and confirm
   the preview changes both ways; leave the page and confirm the camera returns to how it was.
3. Move the active cube's exposure slider → the histogram moves and the readout **stays where you
   put it** (it must not snap back — the v7.13 Zyla symptom). A non-active cube's slider is disabled.
4. Open **More signal**: gain, averaging and black/white each visibly change the histogram; % clipped
   tracks; ⚡Auto still converges on exposure alone.
5. Click `◉` on FITC → **watch the cassette**, not the screen. The active pill follows. Turn the
   cassette by hand and confirm the pill follows that too.
6. Pick a different objective → **the nosepiece rotates**, focus retreating first. Disconnect the
   microscope and confirm the pick is accepted as a label with a log line saying so.
7. Hardware Setup → Cameras: the table lists the nosepiece objectives in turret order with the live
   one marked; calibrate from a `4X` row and confirm `objectives.json` still holds one `4x` entry,
   updated — **no new sibling key**.
8. A 2-channel tile-major scan on a small well: the confirm dialog states rotation count, time and
   memory; the run completes; **both channels overlay in register**; each `save_channel` records the
   cube actually used and its recipe.
9. Re-run the same well channel-major and compare — the only real check that tile-major's shared
   registration earns its cost.
10. Select `mCherry` (unbound on this rig) → tile-major refuses **by name**, offers the binding, and
    after binding to TxRed the scan runs.

---

## Issues & Decisions

- 🐞 **A real bug the first panel test caught: the page had TWO `get_context_widget` definitions.**
  The new one was added with the panel bridge; a second, later `return None` further down the class
  silently won, so the left box would never have appeared and nothing anywhere would have errored.
  The dead definition is gone and a comment records why it must not come back.
- **Histogram placement resolves two instructions against each other.** Request 5 asked for it "just
  above the well selection"; request 7 then moved the settings to the left panel. A histogram is only
  useful adjacent to the slider that moves it, so it sits at the top of the panel. Flagged at plan
  review; revisit if the literal placement is wanted.
- 🐞 **The exposure slider's default range was in the wrong units and a smoke test caught it.**
  `lo`/`hi` are the CONTROL's units (µs, what the camera stack uses everywhere) while `scale` only
  changes the READOUT, so a `hi=10000` meant to be "10 s" capped the slider at 10 µs and a stored
  20 ms recipe rendered as 10 ms. Now `50 … 10_000_000` µs until the camera's own declared range
  replaces it.
- **The decorated table label forced a second fix.** Once the objective column shows
  `"2. 10X  ← in path"`, `_selected_objective_name` (which read the cell TEXT) would have returned
  the decoration — and that string is the key every calibration read and write is filed under, so it
  would have looked up a calibration that does not exist and then created one under the decorated
  name. The raw name now lives in `Qt.UserRole`, in both the table and the combo.
- **Narrow-width behaviour was verified through a real `QScrollArea`, not a bare widget.** A
  parentless widget takes its minimum from its own layout, so `resize()` appeared to do nothing and
  compact mode looked broken. Inside the host shape the panel scrunches 338 → 98 px with **no
  horizontal scrollbar at any width** and goes compact at 130 px.
- **⚠ The redundant-write trap is reachable on the ordinary path.** `tucam_backend.py`'s hardware
  finding is that writing a capability the value it already holds **collapses exposure to the 6.3 µs
  sensor minimum** on the Libra 25 — the live image goes black. The preset is written on *every* page
  entry, so it must go through `_capa_set`'s equal-value guard, never a raw SDK write.
- **Display-path vs data-path, stated in the code.** Display auto-scale and gamma/contrast/brightness
  are display corrections: tiles come from `capture_raw_average` with frozen levels, so on a
  raw-capable camera those three change no captured pixel. They matter because a flickering,
  gamma-warped preview makes judging exposure impossible, and because the single-frame fallback does
  go through them. Hardware `auto_levels` is different — it moves the sensor's own black/white points
  and **does** change the raw data.
- **The automatic path must not write `current_objective_name`.** The v7.18 document flags this as
  *"the mutation most likely to be proposed in good faith"*. It stays a GUI-thread write from an
  explicit operator act — never poll-driven, never from a worker. Following a hand-rotated turret is
  therefore display-only here; the single-writer `OpticsMirror` remains future work.
- **No data migration.** `ObjectiveCalibration._resolve_key` already folds case, so the panel's `4X`
  resolves the existing `4x` calibration and `set_calibration` writes *into* the existing
  case-variant key rather than creating a sibling. The v7.18 rule stands: normalize at lookup, never
  rewrite a measurement's key.
- **Tile-major's honest costs**, priced in the confirm dialog because the operator is choosing them:
  `N_tiles × N_channels` cube rotations (a 400-tile well × 3 cubes ≈ 1200 rotations ≈ 20–40 min of
  rotation alone), and N live composites (~200 MB of float64 accumulators per channel at the default
  `target_px` 2500).
- **Disclosed, not fixed:** one focus per tile means all channels share it. Per-channel chromatic
  (parfocal) focus offsets remain the existing documented v7.13 deferral — within DOF at 4×/10×, not
  at 20×+.

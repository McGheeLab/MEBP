# MEBP v7.18 — Incubator Integration (page + Hardware Setup tab)

## Objective

Integrate the standalone two-zone incubator heater tool
(`C:\Users\mcghe\OneDrive\Desktop\tools\incubator`, previously `tools/incubator/`
in-repo) into the MEBP application:

1. a new **Incubator** GUI page (Workflows tile, 🌡️) hosting the full control
   surface — zone cards, trend plot, sensor table, Stability/PID/Autotune/
   Calibration/Firmware/Console tabs, ALL-OFF / E-STOP bar;
2. a new **Hardware Setup → Incubator** tab for the incubator settings
   (transport, port hints, zone naming/enable, setpoint ceiling, ramp/dither
   prefs), following the Microscope-tab precedent;
3. improve the code while porting (worker-thread connect/detect, shared-ZP
   transport, port arbitration both directions, per-machine config store).

## The one fact that shapes everything

**The incubator heaters live on the SAME SKR Mini E3 V3 board as the ZP stage**
(Zone A = bed output HB/THB, Zone B = hotend HE0/THO — see the tool's
`FIRMWARE_NOTES.md`). The standalone tool required disconnecting ZP in the app
before connecting. Integrated, the app holds that port for the whole session,
so the incubator gets a **shared transport that rides the live
`ZPStageManager` connection** (`_txn` under `_serial_lock`) as its default,
with the standalone **dedicated-port** and **simulator** transports kept for a
future second board and for no-hardware use.

### Shared-transport consequences (each encoded in code, not prose)

- **M105's data rides ON the `ok` line** (`ok T:.. B:..`) and
  `ZPStage._read_until_ok` discards the ok line — new opt-in
  `include_ok_line` (default False = byte-identical) + a public
  `ZPStageManager.transact()` wrapper so the incubator never calls privates.
- **Long board-side operations are REFUSED in shared mode** (PID autotune
  M303, board-side waits M190/M109): they would hold `_serial_lock` for
  minutes → the position poller starves → false "ZP disconnected" cascade.
  The refusal names the alternative (dedicated connection / manual PID /
  host-side hold, which is the default anyway).
- **No autoreport in shared mode** (M155 pushes unsolicited lines that would
  interleave with M114/M400 parsing) — M105 polling only, through atomic
  transactions.
- **Polling yields to prints**: the shared link takes a `poll_gate` callable;
  wired to the position poller's suspension flag so incubator M105s never
  add jitter inside a PRINT_PATH burst. While gated, staleness warnings are
  suppressed (paused ≠ dead).
- **A heater fault kills the whole board** (Marlin `kill()`), motion included.
  Error lines eaten by our own transactions are parsed and latched; error
  lines eaten by the poller's M114 are lost to us — the stale-data message
  therefore names the halted-board possibility. Documented limitation.
- **EEPROM/M502 affect the WHOLE board** (steps/mm etc. live there too) —
  confirm dialogs say so in shared mode.
- **Emergency stop (M112) halts motion too** — the confirm dialog names it.

### Port arbitration (both directions)

- `ZPStageManager` gains `exclude_ports=` (it had NONE — a ZP scan would
  open + DTR-reset a dedicated incubator board mid-hold, resetting Marlin =
  heaters silently OFF). Filter applied in `_initialise_serial` before
  `_try_open_marlin`, same shape as `XYStage._find_with_protocol`.
- `StageController.connect_stages` passes the incubator's dedicated port
  (live via `peek_incubator()`, persisted via `IncubatorConfigStore`) into
  both the ZP and XY exclusion sets.
- The incubator's `detect_marlin()` / dedicated connect gains
  `exclude_ports=`; wired to {zp_connected_port, _preferred_zp_port, live XY
  port}. Trying to open the ZP-owned port dedicated → refusal pointing at
  shared mode.

## Files Modified

### New backend — `SupportClasses/incubator/` (vendored package + new modules)
| File | Provenance |
|---|---|
| `__init__.py` | new (docstring only — no eager imports) |
| `zones.py`, `marlin_gcode.py`, `sensors.py`, `safety.py`, `stability.py`, `ramp.py`, `probe.py`, `marlin_link.py`, `fake_marlin.py`, `telemetry.py` | vendored verbatim (proven by the tool's 89-check selftest) |
| `calibration.py` | vendored; store path → `config/hardware/incubator_calibration.json` (+ `MEBP_INCUBATOR_CAL_DIR` env override) |
| `device_config.py` | vendored; `detect_marlin(exclude_ports=)`, `ranked_ports(exclude=)` |
| `serial_helpers.py` | vendored from `_serial_helpers.py`; prefers a plain `SupportClasses.SerialUtils` import in-app, path-load fallback kept |
| `controller.py` | vendored + transport seam: `connect(..., link_factory=)`, `connect_shared(zp_getter, poll_gate=)`, `supports_board_waits` / `supports_autotune` gates, ceiling from config store |
| `zp_shared_link.py` | **new** — MarlinLink-compatible transport over the live ZPStageManager |
| `config_store.py` | **new** — `IncubatorConfigStore` (clone of MicroscopeConfigStore; `config/hardware/incubator.json`, `MEBP_INCUBATOR_CONFIG_DIR`) + `get_store()`/`reset_store()` |
| `service.py` | **new** — `get_incubator()` / `peek_incubator()` / `shutdown_incubator()` singleton (microscope/LabLink pattern) |

### Modified backend
| File | Change |
|---|---|
| `SupportClasses/ZPStage.py` | `exclude_ports=` ctor + filter in `_initialise_serial`; `include_ok_line=` on `_txn`/`_read_until_ok` (default False); public `transact()`; public `priority_write()` (bounded-lock raw write, quickstop's two-path pattern; quickstop itself untouched) |
| `SupportClasses/StageController.py` | incubator port folded into `xy_exclude`; new `zp_exclude` passed to `ZPStageManager` |

### New GUI
| File | Content |
|---|---|
| `gui/widgets/incubator_widgets.py` | `_Pill`→`StatePill`, `Banner`, `DutyBar`, `TempTrendPlot`, `ZoneCard`, `HeaterBridge` (ported from the tool's gui.py, house theme/scaling) |
| `gui/pages/workflows/incubator_workflow.py` | `IncubatorWorkflowPage` — the page; connect/detect on worker threads with busy-guards; transport from the config store |
| `gui/pages/hardware/incubator_panel.py` | `IncubatorSetupPanel(store=None, controller=None, *, show_save=True)` — settings only, `load()`/`commit()` single-save |

### Modified GUI
| File | Change |
|---|---|
| `gui/pages/workflow_picker.py` | 🌡️ `incubator` tile |
| `gui/pages/workflows_mode.py` | import + dispatch elif + `incubator_page` property |
| `gui/pages/hardware_setup.py` | Incubator sub-page (microscope-block pattern), registered after Microscope |
| `gui/pages/hardware/control_panel.py` | ROUND 2: **no Incubator row** — the ZP Connect IS the incubator connect (same board); a comment marks where an ESP32-board row would return |
| `gui/widgets/icons.py` | `incubator` icon (thermometer SVG) |
| `gui/app.py` | closeEvent: heaters-off prompt when heating + `shutdown_incubator()` between microscope and controller shutdown |
| `gui/widgets/context_sections.py` | ROUND 3: `IncubatorSection` + `register_section("incubator", …)` — 🌡️ zone readout on the Custom jog panel (peek-only) |
| `config/hardware/incubator.json` + `.gitignore` | ROUND 3: this rig's config (Zone B disabled — no usable heater on HE0), and the per-machine file + its calibration sibling are now ignored |

### Tests
| File | Coverage |
|---|---|
| `tests/test_v718_incubator_backend.py` | controller against FakeMarlinLink through production paths (connect/probe/setpoint/ramp/dither/fault/rescan/EEPROM) |
| `tests/test_v718_incubator_shared_link.py` | ZPSharedLink vs fake ZPStageManager: M105 ok-line data, refusals (autotune/M190/sim-ZP), poll gate, disconnect propagation, ZPStage.transact/include_ok_line/priority_write, exclude_ports |
| `tests/test_v718_incubator_gui.py` | offscreen page build + tile registration + bridge wiring + connect-off-GUI-thread + HW tab + config store round-trip + connect-card row |

## Implementation Steps

- [x] Read + map the standalone tool and MEBP integration points
- [x] Decide architecture (shared/dedicated/sim transports; Workflows tile; HW tab; store)
- [x] Vendor backend into `SupportClasses/incubator/`
- [x] `zp_shared_link.py` + controller transport seam + refusal gates
- [x] `config_store.py` + `service.py`
- [x] ZPStage: `transact`/`include_terminal_line`/`allow_identity_lines`/`priority_write`/`exclude_ports`; StageController exclusions
- [x] GUI widgets + `IncubatorWorkflowPage` + tile registration
- [x] `IncubatorSetupPanel` + Hardware Setup tab (+ bump tab-count test 10→11)
- [x] Connect-card row + app.py shutdown/prompt
- [x] Tests + targeted regression suites
- [x] Adversarial review + mutation checks
- [x] ROUND 3: refused-command latch (bench fault) + jog-panel readout section
- [x] Finalize docs (this file + CLAUDE.md table)

## Testing Notes

**NEW: 107 tests across 3 files, all green** (44 backend · 31 shared-link ·
32 GUI, after the round-2 rework, the round-3 refusal/readout work and the
round-4 setpoint keeper).

- `test_v718_incubator_backend.py` (28) — production controller against the
  thermal simulator (transport bookkeeping, ceiling can-only-lower, zone
  labels reaching the sensor source, integer setpoints, dither arming,
  watchdog-safe ramp cap), calibration store under the env override, config
  store validation (junk transports refused, ceiling/ramp clamped ON LOAD as
  well as on set, malformed file → blank), service singleton
  (peek-never-constructs, store ceiling applied at construction), and the
  port-exclusion matrix — including the load-bearing
  `test_detect_never_opens_an_excluded_port` (a fake `serial` module records
  every open; the excluded port must have ZERO opens, because the open
  itself is the DTR reset). **Round 3 adds `TestRefusedHeaterCommand` (8)** —
  the bench fault: a refusal latches with the board's own reply text, is
  logged at WARNING, banners ONCE not per retry, a timeout/reset does NOT
  latch (link problems have their own owners), an accepted command or any
  fresh operator action clears it, and — the load-bearing pair — a refused
  setpoint **stops the dither re-assertion and stays stopped**, and a
  refused ramp rung stops the ramp. **Round 4 adds `TestSetpointKeeper`
  (10)** — a board that forgets its target gets it put back (logged, and the
  operator told), one lagging sample does not trigger and the counter resets,
  a stale reading cannot trigger it, a latched fault and a REFUSED zone both
  stop it, the spoken warning caps at three while the count and the commands
  keep going — plus the two that guard the guard: an AST pin that
  `_publish_sample` still calls the keeper, and a behavioural test that lets
  the live 1 Hz sampler do it.
- `test_v718_incubator_shared_link.py` (31) — ZPStage's new hooks driven
  through the REAL `_read_until_ok` against scripted serial fakes:
  `test_m105_data_would_be_LOST_without_the_terminal_line` (both halves —
  the naive path provably drops the temps), the M115/identity-lines pair
  (below), byte-identity of the default `_txn`, priority_write under lock
  AND raw-on-contention, ZP scan exclusion; then `ZPSharedLink` semantics
  (refusals, timeout clamp, poll gate, close-never-touches-the-port,
  disconnect-once) and `connect_shared` end-to-end against a fake Marlin ZP
  (probe, M155 S0, M140 flow, autotune refusal, M190 degrade, poll yield,
  heaters-off on disconnect, M112 out-of-band), plus
  `StageController._incubator_reserved_ports` (serial-transport-only,
  ZP-preferred-port never excluded, live-session port).
- `test_v718_incubator_gui.py` (23, offscreen) — tile registration (AST
  dispatch check), the AUTO session (the trigger must return in <0.5 s —
  the GUI-freeze class; re-entry guard; **the status tick itself starts
  the session** — the wiring that replaces the connect button; shared
  transport waits for the ZP board with the hint naming the real connect
  and NO banner spam; a live fake ZP is joined automatically; a simulator
  session is retired on transport switch), zone-card confirm path with
  QMessageBox patched (modals block forever offscreen — the documented
  trap), serial-controls-hidden-unless-serial, store→card prefs,
  revisit-keeps-typed-setpoint, panel load/commit round-trip +
  nothing-written-before-Save + live ceiling push, HardwareSetupPage tab
  resolving BY NAME, **the Connect card's incubator row PINNED ABSENT**
  (operator decision; the tick must not even touch the incubator service),
  and an AST check that `closeEvent` calls `shutdown_incubator` BEFORE
  `controller.shutdown()`. **Round 3 adds `TestZoneCardRefusalDisplay` (3)**
  — the REFUSED pill + persistent card message carrying the command, the
  board's reply AND the physical heater port (a sensor fault still outranks
  it on the pill, but the refusal text stays readable) — **and
  `TestIncubatorJogPanelSection` (6)**: registered in the catalog, honest
  "no session → open Workflows → Incubator" pointer, live temp → target ·
  duty with a state word, a REFUSED zone reading REFUSED, a store-disabled
  zone's row hidden, and `test_the_section_never_starts_a_session` pinning
  peek-only.

**Mutations: 15/15 CAUGHT** (each guard reverted, its test failed, source
restored and re-verified green): ZP scan exclusion removed · ok-line data
dropped again · identity-lines narrowing removed (M115 reads as reset) ·
shared-mode autotune refusal removed · poll gate ignored · shared timeout
clamp removed · board-side-wait degrade removed · dedicated connect to an
excluded port allowed · **the status tick no longer starts the session
(round 2 — the wiring that replaces the connect button)** · **the
simulator auto-retire removed (round 2)** · **the dither back on the
unchecked sender = the 48-minute bench bug (round 3)** · **the jog-panel
readout constructing the service instead of peeking (round 3)** · **the
keeper unwired from the sample loop = the forgotten-setpoint bug, caught by
BOTH the AST pin and the behavioural test (round 4)** · **the keeper's
two-sample debounce removed (round 4)** · **a REFUSED zone re-asserted
(round 4)**.

**Regression: ~516 green across the touched-area suites, run per batch** —
zp-serial-flow-control/close-during-read/watchdog-abort/jog-clamp/
position-override/suite-hygiene 58 · hard-abort/ludl-xystage/dtr/auto-reconnect
76 · plate-builder-ui/phase0-hygiene 135 · nikon-ti 101 (covers the
Connect-card and HW-page shape) · full-print/stress-test workflows 41
(constructs the real WorkflowsModePage, now incl. the Incubator page) ·
timing-calibration/fluorescence 57 · illumination/per-pump-rate 48 — plus a
`gui.app` import smoke and an offscreen page-connect walk-through.
**Five pre-existing failures PROVED not ours by stashing this change's
edits and reproducing them identically**: `test_v75x_zp_dtr_no_reset::
TestOpenStillConnects` ×2 and `test_v75x_zp_auto_reconnect_and_fast_z::
test_no_z_max_keeps_defaults` (both documented in CLAUDE.md), and
`test_v712_plate_builder_ui::TestLearnLoopSavesToADesign` ×2 (the plate-type
`max` z-offset — from other uncommitted working-tree WIP, untouched here).

## Issues & Decisions

- **⭐ ROUND 6d — BOARD REPLACED, AND THE ONE SOFTWARE GAP IT LEFT.**
  The replacement SKR Mini E3 V3 (COM3, serial `2061328F4231`) settles round
  6c: **the identical test that killed the old board in 1.00 s ran 59.4 s at
  duty 127/127 with zero link drops.** The operator confirms the pads now
  heat. Diagnosis closed — the fault was the old board's bed output.

  **What that left is a firmware/UX gap, not an electrical one.** Both boards
  halted with `Heating Failed` -> `kill()` when a setpoint was commanded far
  above the current temperature, because Marlin arms its heat-up watchdog and
  a slow thermal load cannot satisfy it. `ramp.py` already derives the exact
  arm threshold FROM MARLIN'S OWN SOURCE (`HeaterWatch::restart`: the watch
  is only scheduled above `current + INCREASE + HYSTERESIS + 1`, ~6 C) — and
  **`watchdog_arm_threshold_c()` had ZERO production callers**, the dead-
  function trap this file keeps recording. So the Ramp button was safe and
  the Set button beside it was a board-killer, with nothing saying so.
  `preview_setpoint` now reports `watchdog_risk` / `watchdog_arm_gap_c` /
  `current_c` (additive — every existing key untouched, pinned), judged
  **only from a live reading**: a risk we cannot substantiate would train the
  operator to click through the warning. The zone card offers the Ramp
  (default) / Set anyway / Cancel.

  ⚠ **The offer is ONE overridable seam (`ZoneCard._ask_watchdog`), not a
  bare modal** — and that is not tidiness: the first cut put a
  `QMessageBox(...).exec()` straight into `_on_set` and **hung an existing
  test**, because offscreen a modal blocks forever and this suite patches
  only `QMessageBox.question`. A hidden modal turns every future test that
  presses Set into a hang; an AST test now pins both that `_on_set` asks
  through the seam and that it opens no modal itself.
  `test_set_target_through_the_card_with_confirm` was updated for the
  genuinely-changed contract (it takes "Set anyway", since it is about the
  direct set).

  🐞 **A cancel now outranks a teardown failure.** A staircase cancelled
  mid-rung could report `died_at=("dropped", "'M140 S0' not accepted")` —
  the command failed *because* we were stopping, and blaming the board for
  the operator's own cancel is a false diagnosis. Surfaced as an
  intermittent test failure and fixed at all three failure sites.
  ⚠ Two suite-interaction flakes disclosed and fixed in the TESTS, not by
  weakening them: late in a full run the simulated link can take the full 8 s
  transaction timeout, which exceeded a 3 s rung, so no sample was ever
  published — the window was widened, the assertion is unchanged (3/3 in
  isolation before, 2/2 full-suite repeats after).

  Regression: staircase-in-app 32 · incubator GUI 37 · backend 44 · staircase
  33 · shared link 33 · hygiene 10, all green per suite, plus a `gui.app`
  import smoke; **3/3 mutations CAUGHT** (risk never reported = the board
  killer goes through unremarked · risk claimed without a live reading ·
  threshold ignored so every setpoint warns).

  **Still open, and hardware not software:** the pad gained only 2.7 C/min at
  28 C at FULL duty during the HE0 soak and had not reached 37 C — whether
  this heater can hold 37 C in the real enclosure is a power/insulation
  question independent of everything above.

- **🔴🔴 ROUND 6c — BENCH 2026-08-17 EVENING: THE FAULT IS THE BOARD'S BED
  OUTPUT, PROVED BY A CONTROLLED SUBSTITUTION.** Driven directly over COM4
  with MEBP closed, so the app, the ZP poller and the shared link were all
  out of the picture. **The control experiment is the whole argument:** a
  target BELOW ambient is accepted by Marlin but never switches the MOSFET,
  a target ABOVE ambient does — same command, same code path, same load,
  and only the switching differs.

  | test | MOSFET switches? | result |
  |---|---|---|
  | HB `M140 S10` | no | survives indefinitely, duty 0 |
  | HB `M140 S37` | yes | **port dies in 1.00 s** |
  | HE0 `M104 S37` | yes | **survives 60 s at duty 127**, +6.2 C |

  So: the command path is innocent (the board takes `M140` and runs), the
  **pad** is good (4 ohm; +10.5 C/min on HE0), and the **supply** is good
  (it carried the identical ~3 A on HE0 for a minute — measured heating
  rates HB +11.8 vs HE0 +10.5 C/min over comparable windows, i.e. the same
  load). What remains is the HB output itself: fine at rest, collapsing the
  12 V rail within ~4 ms of being driven. It degraded across the day —
  18 s of carrying the load in the morning, then failure at 3 % duty, then
  at 1 s — and **an unplug/replug did not fix it**, ruling out a loose
  terminal. Board replaced.

  ⚠ **A DUTY LEVEL CANNOT HELP, and my own round-6 rationale was WRONG
  about why.** Marlin's bed heating is TIME-PROPORTIONED soft PWM: the
  MOSFET is fully ON during each pulse, so the peak current at 3 % is
  identical to 100 % and only the pulse WIDTH changes. The staircase died
  at the 5 % rung — one ~4 ms full-current pulse was already too much. The
  "pid mode limits PEAK current" claim is corrected in
  `power_staircase.py`, and the verdict now says so.

  ⚠ **`@:`/`B@:` IS NOT EVIDENCE OF CURRENT** — the second correction, and
  it came from a real mistake: three HE0 runs were made with the heater
  UNPLUGGED (the operator said so afterwards), and Marlin reported duty
  **127/127 for 20 s** into that open circuit, with no heat and no sag. The
  staircase would have printed *"survived 100%"* — telling the operator
  their supply carries full power when nothing was connected, the exact
  false-all-clear class already guarded on the other side. NEW
  `rung_open_circuit()` (sustained high duty + no temperature rise) fills a
  NEW `StaircaseOutcome.no_heat`, wired into BOTH the bench tool and the
  controller (a pure check nothing populates is the dead-field trap), and
  the verdict leads with **"NOTHING WAS DRAWING … the link was never
  loaded"**. Pinned by a fake that reports full duty and delivers no heat.

  ⚠ **THE HOTEND CHANNEL IS THE WRONG HOME FOR A SLOW LOAD, and that is
  firmware, not electrical.** With the pad on HE0 and the probe moved to
  THO the board was rock solid at full duty — and Marlin still called
  `Heating Failed` / `kill()` at 60 s, because its hotend heat-up watchdog
  wants +2 C per 20 s and this pad delivered +3.59 / +1.86 / +0.91 across
  three successive windows (peak 28.0 C of a 37 C target). The BED channel
  exists for exactly this: its watch window is 60 s, not 20 s. Operator
  note that settles it — on the real rig the probe is NOT bonded to the
  pad, so every window gets smaller still. Staying on HE0 would require
  `WATCH_TEMP_PERIOD` 20->60 and `WATCH_TEMP_INCREASE` 2->1 in
  `Configuration_adv.h` (compile-time; no G-code reaches them).

  ⚠ **Open question, disclosed:** at FULL duty the pad gained only
  2.7 C/min at 28 C and was still short of 37 C. That is a heater-power /
  insulation question independent of every fault above, and it may mean
  this pad cannot hold 37 C in open air whichever output drives it.

  ⚠ **The replacement board enumerates on a DIFFERENT port and serial**
  (COM4 `2044328F4231` -> COM3 `2061328F4231`); `ME3B_2.json` still carries
  `zp_stage.last_port: COM4`, which the app's probe corrects on the next
  successful connect. Tools used: `tools_incubator_heater_diagnostic.py
  --staircase` plus per-test transcripts under `logs/hwtest/`
  (`he0_soak*.txt`, `he0_hold37.*`, `hb_retest.*`, `hb_newboard.*`).

- **⭐ ROUND 6b — THE STAIRCASE IS AN OPTION IN THE APP** (operator: *"I
  want these to be options in the incubator workflow somewhere"*). The bench
  tool stays (it is the confound-free measurement, app closed, no ZP poller
  traffic); this adds the same engine as a **Power staircase tab** on the
  Incubator page, between Autotune and Sensor calibration. **ONE HOME FOR THE
  MEASUREMENT:** NEW pure `SupportClasses/incubator/power_staircase.py` owns
  the arithmetic *and the interpretation* — `plan_staircase`,
  `p_gain_for_duty`, `rung_delivered`, `rung_overshot`, `StaircaseOutcome`
  and `verdict_lines` — and the tool now IMPORTS them (verified `is`-identical
  in both surfaces), because two copies of a verdict is how two surfaces come
  to disagree about the same measurement. `DUTY_FULL` **aliases**
  `marlin_gcode.HEATER_PWM_FULL_SCALE` rather than re-declaring 127, which
  already had a home feeding the duty bar and the trend plot. **THREE HAZARDS
  THE BENCH VERSION DOES NOT HAVE, each mutation-pinned. (1) The setpoint
  keeps ONE writer:** a staircase drives the target directly, so the round-4
  keeper and the round-3 dither would re-assert the operator's hold on top of
  every rung and the measurement would be of the two fighting, not of the
  supply — NEW `ZoneRuntime.diagnostic_active` makes both stand off, and the
  tests guard-the-guard by proving the keeper DOES re-assert without it.
  **(2) It refuses while a print owns the channel** (`_poll_permitted()`
  False): the test is *expected* to reset the board, and on the shared link
  that board runs Z and the pumps — resetting it mid-print is a crash, not a
  diagnostic. The confirmation names that consequence (retract the needle;
  the Z position must be re-declared after a reset) and defaults to No.
  **(3) `mode="pid"` rewrites the bed PID**, so it REFUSES to start without a
  copy of the gains it can put back, restores them on every exit path, and
  **never sends M500** (pinned) — a diagnostic's pure-P gains must not become
  permanent. **NOT refused on the shared link, unlike autotune, and the
  difference is structural rather than a judgement call:** `M303` answers its
  `ok` only when the whole tune ends — minutes to hours in ONE transaction
  holding the motion board's lock — while the staircase is a handful of
  ordinary short commands spaced a second apart, and it READS temperature and
  duty from the existing 1 Hz sample stream instead of polling itself. For
  the same reason it runs on **its own thread, not the command worker**: that
  worker also services the poll, so occupying it would starve the very
  stream the run reads. 🐞 **Two defects my own tests found. (a)** The verdict
  trusted the REQUESTED percentage, so a bed already at the ceiling (duty 0,
  no error to drive) reported *"Highest level SURVIVED: 100%"* — a false
  all-clear that would send the operator hunting a software fault that is not
  there; a rung now counts only if the MEASURED duty reached 60 % of the
  request, with `NOTHING WAS PROVEN` when none did, and a pre-check refuses
  to start within 1 °C of the ceiling. **(b)** A temperature abort DISCARDED
  the rung it had just measured and then printed "nothing was proven" over a
  log showing 99 % duty — the link carried that duty right up to the abort,
  so it is recorded and the note says it is a lower bound. 🐞 **A third
  found by reading the test log:** `_sc_apply` returned a bare bool, so an
  UNANSWERED `M304` was reported as *refused* — round 3's rule is that only
  an answered "no" is a verdict about the zone; a timeout is a link problem,
  and calling it a refusal sends the operator after the wiring when the port
  had simply gone. It now returns the failure KIND (`refused`/`dropped`).
  ⚠ **NEW `rung_overshot` warning, and the simulator is why it exists:** a
  firmware that ignores the gains runs every rung at full power, so an
  operator who asked for 10 % would have applied 100 % — the verdict now says
  **THE LEVEL WAS NOT LIMITED** and names the rungs, verified firing against
  the thermal simulator (which does exactly that). Tests: NEW
  `tests/test_v718_staircase_in_app.py` (**24**) + `test_v718_heater_staircase`
  (**27**); **8/8 further mutations CAUGHT** (keeper stops standing off = the
  two-writers bug · dither ditto · runs during a print · pid starts with no
  gains to restore · the diagnostic flag left set, which would disable the
  keeper forever · a timeout called a refusal · the confirmation removed · a
  refused start leaving the tab's buttons dead). Regression **175 green**
  (incubator backend 44 · shared link 31 · GUI 37 · staircase 51 · hygiene)
  + a `gui.app` import smoke + a full staircase driven through the REAL page
  against the simulator (buttons gate, rows stream at 1 Hz, verdict renders,
  flag clears, PID restored). **Bench:** Incubator -> Power staircase, pick
  peak-limited, Run, and send me the report (there is a Copy button).

- **🔴🔴 ROUND 6 — BENCH 2026-08-17: "THE HEATER WON'T START" IS THE BOARD
  LEAVING USB, NOT THE CONTROL PATH** (operator: *"the sensors work and it
  plots the sensor temp … when I try to start the heater … maybe it should
  start with a low voltage and come up. maybe we should have a manual mode
  so i can debug."*). Root-caused from the operator's OWN telemetry — round
  4's `log_on_connect` change is what made this readable, and it paid for
  itself the first time it was needed. **The command path is CORRECT and the
  heater WORKS:** every session logs `setpoint requested 37.0 commanded 37`
  with no `command_refused`, and the bed physically climbed **22.3 → 33.3 °C
  in 18 s at duty 100/127**. What fails is the link: in `app.log` the first
  `WriteFile failed (PermissionError(13, …, 22))` — Windows
  `ERROR_BAD_COMMAND`, the device gone mid-write — lands **0.45 s after
  `M140 S37`** in the 16:06 session and **0.7 s after** it in the 16:09 one,
  and the correlation is EXCLUSIVE: **0** write errors in the 35 s between
  ZP connecting and the heater command, **0** across the 4-minute
  heater-off window, then 50–104 per minute once it is on. Then the
  documented cascade: link dies → poller-liveness declares ZP disconnected
  → auto-reconnect opens the port → **DTR resets Marlin, clearing the
  target** → round 4's keeper re-asserts → it heats for seconds → dies
  again, which is why the card looks alive while nothing sustains. ⚠ **A
  real hazard, disclosed:** in the 16:14 session the bed rose 22.3 → 33.3 °C
  **while the board reported target 0.0 and duty 0.0** — during the
  brownout/reconnect windows the heater was live and the software was blind
  to it. No software soft-start makes that safe; it is why the electrical
  fault is the first thing to fix. **Operator-supplied fact that narrowed
  it:** VIN is **12 V** into a 12 V heater, so it is NOT the 24 V-rail
  overvoltage case (which would have been 4× rated power) — leaving supply
  capacity, inrush, or coupling. **DELIVERED: `--staircase` on
  `tools_incubator_heater_diagnostic.py`** (operator chose the bench tool
  over GUI work first — the round-4 precedent, and it removes the ZP
  poller's ~3 Hz M114 traffic as a confound). It steps bed power up a rung
  at a time and reports the highest level the USB link survives. **The
  mechanism is the interesting part: Marlin has NO set-bed-PWM G-code** —
  `M140` sets a target and the firmware picks the duty — so `--mode pid`
  reduces the bed PID to pure proportional (`M304 P<k> I0 D0`), where
  `pid_output = P·error` clamped to `MAX_BED_POWER` and `B@ = pid_output>>1`,
  making the duty a chosen function of a known error; `p_gain_for_duty`
  inverts that for a starting gain and one measured trim corrects it. `I` is
  zeroed because integral windup would drift the duty off the level under
  test; `D` because it chases sensor noise. **`--mode pwm` is the
  differentiator, not a spare:** host-side slow PWM gates FULL-power bursts,
  limiting average while every burst is full current — so pid surviving
  where pwm dies at the same percentage proves the failure is peak/inrush,
  which is exactly the condition under which an app-side duty cap would be a
  real fix rather than a mask. **🔴 THE HONESTY RULE, and my own first cut
  got it wrong:** the verdict initially trusted the REQUESTED percentage, so
  a bed already at the ceiling (duty 0, no error to drive) reported
  *"Highest level SURVIVED: 100%"* — a false all-clear that would send the
  operator hunting a software fault that is not there. A rung now counts
  only if the MEASURED duty reached 60 % of the request; otherwise it is
  reported **not delivered / inconclusive**, `NOTHING WAS PROVEN` when no
  rung loaded the link at all, and a pre-check refuses to start when the bed
  is already within 1 °C of the ceiling. That also catches round 4's
  upstream case (a board answering normally with duty stuck at 0). Both
  defects were found by my own tests failing, not by review. Other
  properties: the run **stops at the first killing rung** (every later
  "result" would be a lie once the port is dead); **`silent` is never
  conflated with `dropped`** (port-open-and-quiet is `kill()` needing a
  power cycle; port-gone is a supply fault — different remedies); the bed
  PID is **restored in every path and never written to EEPROM** (no `M500`,
  so a power cycle also restores it), and `--mode pid` **REFUSES** when
  `M503` reports no `M304` rather than leave gains it cannot put back;
  `--mode pwm` never touches the PID at all. Tests: NEW
  `tests/test_v718_heater_staircase.py` (**26**) driving the real
  `staircase_test()` against a fake Marlin that browns out at a chosen duty
  — a bench tool that crashes when it is needed is worthless, which is the
  lesson already recorded for this file's first draft dying on an emoji
  under cp1252, so the ASCII-purity check is pinned by test both on the
  source bytes and on the rendered report encoding to cp1252. **6/6
  mutations CAUGHT** (delivered-gate removed = the false all-clear ·
  ceiling pre-check removed · PID restore removed · silent conflated with
  dropped · staircase continues past a dead link · pwm mode rewriting the
  PID). Regression **117 green** (staircase + incubator backend + GUI +
  suite hygiene). **NOT built, deliberately:** the app-side soft start and
  the GUI manual mode the operator suggested. Both are the right tools *if*
  the staircase says the failure is peak current; if it dies at a low rung
  the answer is heater resistance or supply capacity and a software cap
  would only mask an uncontrolled heater. **Bench, in order:** close MEBP →
  `python tools_incubator_heater_diagnostic.py --staircase` → note the
  highest surviving rung → if it survives 100 %, re-run with the app open to
  implicate the shared-port traffic; if it dies below 100 %, re-run
  `--mode pwm` at that rung to separate peak from average; then measure the
  heater resistance with everything powered off (at 12 V: `P = 144/R`,
  `I = 12/R`) and compare against the PSU and the board's bed-output rating.

- **⭐ ROUND 5 — the trend plot is resizable** (operator: *"the temperature
  trend plot in the incubator is too wide, it needs to be resizable as
  well."*). The upper area was a plain `QHBoxLayout` whose left pane —
  the zone-card scroll — was `setFixedWidth(s(372))`, so the right column
  absorbed **every** spare pixel: on a wide screen the plot rendered as a
  very wide, very short strip with no handle anywhere to change it. The
  sensor table underneath was `setFixedHeight(s(124))`, pinning the
  vertical split too. Both fixed sizes became **floors**
  (`setMinimumWidth` / `setMinimumHeight`) and both boundaries became
  `QSplitter`s: horizontal (zone cards | right column) so width can be
  handed back to the cards, vertical (trend | sensor table) so plot height
  trades against table rows — which also means the sensor table can grow
  at all, which it never could. Three details: **neither pane may collapse
  to zero** (`setChildrenCollapsible(False)` — a pane dragged away is a
  control surface the operator cannot find again); the plot gains a
  **minimum WIDTH** beside its existing minimum height, since it now lives
  between drag handles and the axis labels stop being readable well before
  a pane reaches zero (measured non-binding on the page's minimum — the
  trend group's control row already demands 616 px, so this changed no
  layout minimum); and the horizontal stretch stays 0 : 1, i.e. window
  growth still goes to the plot as before, so nothing about the default
  look changed except that it can now be dragged. ⚠ **Honest limit:** on a
  short window the trend and sensor panes sit AT their combined minimum,
  so the vertical handle has no slack and does not move — verified: at
  1600×950 it is inert, at 1600×1300 it drags freely. The dividers are not
  persisted (the page is built once, so a drag survives navigation but not
  a restart); writing layout state into the incubator store would race the
  Hardware Setup tab's `load()`/`commit()` contract, which owns that file.
  Tests: NEW `TestTrendPlotIsResizable` (**5** in
  `test_v718_incubator_gui.py`, now 37) asserting the **drag**, not the
  widget types — a splitter already at its minimums is still unmovable, so
  each test moves a divider and measures the plot; **5/5 mutations CAUGHT**
  (cards column fixed-width again = the original bug · sensor table
  fixed-height again · either pane made collapsible · the plot's width
  floor removed). Regression **124 green** (incubator gui + backend +
  shared link + suite hygiene) and a `gui.app` import smoke.
  `TempTrendPlot` has exactly one consumer, so the new floor has no other
  blast radius.

- **🔴🔴 ROUND 4 — BENCH SESSION 2026-08-13: THE HARDWARE WAS NEVER THE
  PROBLEM, AND THE HOLD HAD NO KEEPER.** Operator, after round 3: *"the
  heater is still not heating up, i have plugged the heater film directly
  into the heat bed connection and it doesnt work"* + *"the thermistor is
  attached directly to the heater"*, then *"i want you to run this"*.
  **NEW `tools_incubator_heater_diagnostic.py`** (repo root, the
  `tools_microscope_hw_check.py` convention) settles the question the GUI
  cannot: Marlin reports its OWN bed PWM in every M105 (`B@:`, 0-127), which
  splits the fault in half — *duty > 0 with a flat temperature* is
  electrical (VIN/element/crimp/MOSFET), *duty stuck at 0* is upstream
  (target not applied, bed disabled, latched fault). It also prints every
  port with its VID:PID, M115, M503, and **M122 as an indirect VIN check**
  (the TMC2209s run off VIN, not USB, so a board with no supply answers M105
  cheerfully while its drivers report nothing). Never moves an axis, caps
  the target, aborts above 45 °C, and sends `M140 S0` in a `finally`
  including on Ctrl-C. **MEASURED ON ME3B V1 (COM4):** `M304 P41.78 I7.32
  D158.93` present (PIDTEMPBED on), M122 answered with full per-driver
  registers (VIN present), bed sensor 21.5 °C, hotend the documented `-15.0`
  open circuit. Then `M140 S31`: **duty 104/127 at t=3 s, 21.9 → 33.9 °C in
  ten seconds (~72 °C/min)**. Then `M140 S37` for three minutes: **37 °C
  reached in ~7 s and held 36.7-38.0 °C (±0.65) at 8-24% steady duty, no
  thermal halt, no USB drop, every poll answered.** So the film, the wiring,
  the MOSFET, VIN and even the stock Ender-3 bed PID are all FINE — and my
  own round-3 theory that Marlin's runaway watchdog was killing the board
  was **wrong**, disproved by the very soak meant to reproduce it.
  **THE ACTUAL DEFECT, found by tracing `ramp.py` against this fast heater:**
  the ramp is sound (24 → 28 → 31 → 34 → 37, done in ~10 s), but it
  **finishes the moment it has commanded the final target** — its own comment
  reads *"from this point the firmware owns the hold"*. That is true right up
  until the firmware **forgets**. A Marlin reset clears every heater target
  to 0, and **this board resets routinely: opening its serial port pulses
  DTR, so every ZP auto-reconnect reboots it** — the operator's app.log shows
  three reconnects between 19:07 and 19:10, and even says
  *"ZP auto-reconnect succeeded — re-declare the Z position (Marlin reset on
  reopen)"*. **The Z position is re-declared after that reset. The heater
  target was not, by anyone.** Net effect and exact symptom: the page shows a
  37 °C hold, the board sits at target 0, nothing heats. NEW
  `_service_setpoint_keeper()` on the 1 Hz sample loop compares the board's
  OWN reported target (M105 carries it) against `commanded_c` and puts it
  back. Four properties: **two disagreeing samples, never one** (a sample
  taken between issuing a command and the board applying it legitimately
  lags, so acting on one would put a command on the wire at every setpoint
  change); **a stale reading can never trigger it** (judging from a reading
  the board never sent would re-command every poll while the link is down);
  **a REFUSED zone is never re-asserted** — the round-3 rule outranks the
  keeper, since a refusal is a verdict, not a target to retry; and the
  **count is surfaced on the zone card** (*"board forgot the setpoint x3,
  restored"*) with the status spoken only the first three times, because a
  board reset-looping is a fault in its own right but must not bury every
  other message. ⚠ **My own tests were too weak and a mutation caught it:**
  every keeper test called `_service_setpoint_keeper()` DIRECTLY, so all of
  them passed with the call removed from `_publish_sample` — i.e. they would
  have passed with the exact fault still shipping. Fixed with an AST pin on
  the call site plus a behavioural test that lets the live sampler do it.
  ⚠ **The diagnostic's first draft died with a `UnicodeEncodeError`** on a
  decorative emoji, because this console is cp1252 — a bench tool that
  crashes when it is needed is worthless, so its output is now pure ASCII
  (the trap already recorded for `tools_install_tucsen_sdk.ps1`).
  **Also corrected: `FIRMWARE_NOTES.md` says the board is COM6. It is COM4**
  — COM6 is the FTDI Prior XY (`0403:6001`); the board is the STM32 native
  USB CDC (`0483:5740`). The app finds it by probing, so this misled only
  humans. And `log_on_connect` is now **true** in this rig's config so the
  next failure leaves a temperature record (`logs/incubator/*.jsonl`) —
  its absence is why round 3 had to reason from serial traces alone.
- **🔴 ROUND 3 — BENCH FAULT: "the heater is not getting hot, ensure you are
  using the right port".** The port was right and the software was silent
  about the real problem. `logs/zp_serial.log` from the 2026-08-12 session
  answers it outright: **`M104 S37 → error`, repeated every ~minute from
  16:59 to 17:47** — the firmware ANSWERED, and the answer was *no*.
  `M105` alongside it returned `ok` throughout, so the link was healthy the
  whole time; the shared transport, the probe (`fw=Marlin bugfix-2.0.x
  bed=PID hotend=PID`) and the ramp (`[ramp bed] Ramping bed from 23.3 °C to
  37.0 °C`) all worked. **`M104` is the HOTEND command** — Zone B, heater
  output HE0 — and on this rig the heater is the **heat bed** (HB, `M140`,
  Zone A, which is what `zones.py` has always mapped). So the operator was
  driving the zone with no heater behind it, and would have been told
  instantly if the software had looked at the reply.
  **Three separate defects, all fixed:**
  **(1) THE SOFTWARE NEVER READ THE ANSWER.** Every heater command went
  through the bare `_send`, whose `Transaction` — carrying `ok`, `rejected`
  and the board's own `Error:` text — was DISCARDED at every call site
  (`set_target`, `_ramp_command`, `_service_dither`, `heater_off`). A
  refused command therefore produced no status, no log line, no card state.
  NEW `_send_zone_cmd(zone_id, cmd)` checks it: an answered "no" latches
  `ZoneRuntime.refused` (command → the board's reply), logs at WARNING,
  writes a `command_refused` telemetry event, and banners ONCE. Only
  `txn.rejected` latches — a timeout or a board reset is a LINK problem
  that the stale-data warning and reset detection already own, so latching
  on those would move a diagnosis to the wrong place.
  **(2) THE HOST KEPT RE-ASSERTING IT.** The fine-setpoint dither and the
  ramp re-command the target every period, so the refusal repeated ~48
  times with nothing accumulating anywhere. A latched refusal now stops the
  dither and the ramp — *"refused" is a verdict about this zone, and
  retrying it every minute forever is exactly how it stayed invisible*.
  Pinned by `test_the_refusal_STOPS_the_dither_reassertion`, which also
  forces the next flip and asserts NO further send.
  **(3) A REFUSED `heater_off` MATTERS MOST**, so it goes through the same
  checked path: an operator who presses OFF and is refused believes a live
  heater is cold. Any fresh operator action (set / ramp / fine / off) clears
  the latch, so the verdict is always about the current attempt.
  **The card now says which port it drives:** the HB/THB · HE0/THO header
  gained a tooltip naming heater-output vs thermistor and pointing at the
  other zone's card, and the refusal message names `heater_connector`
  explicitly — *"is my heater on the right port"* was the operator's actual
  question, so the answer belongs in the failure text rather than in a doc.
  **Per-machine config written for this rig:** `config/hardware/
  incubator.json` disables Zone B (an open-circuit thermistor AND no heater),
  so the page and the new readout stop offering a phantom zone; the file is
  now gitignored beside `incubator_calibration.json` — which zone has a
  heater is wiring, i.e. per-rig, the CAMERA_CAL_PERSIST_STORE rule.
  ⚠ **What this does NOT do:** it cannot make Zone B heat, and it does not
  guess that a refused `M104` means "use `M140`" — the operator plugs the
  heater into a physical connector and only they know which. The software's
  job here is to say *the board refused this, this zone is not heating, and
  this is the port it drives*.
- **ROUND 3 — 🌡️ Incubator readout on the Custom jog panel** (operator
  request). A registered section (`context_sections.py`, the illumination /
  microscope drop-in pattern), rendering per zone: temperature → target ·
  duty plus a state word (heating / cooling / at target / off / stale /
  SENSOR / REFUSED). Deliberately a **READOUT, not a control** — setpoints
  keep ONE writer (the Incubator page), the lesson this file already records
  for `set_filter`. Deliberately **peek-only**: it calls `peek_incubator()`,
  never `get_incubator()`/`connect_from_store()`, so session policy stays in
  one place (the page, which follows the ZP board by itself) and a jog panel
  can never start a heater session as a side effect of being visible;
  pinned by a mutation. With no session it names where to start one rather
  than showing a dead card, and it hides a store-disabled zone's row. Its
  own render is throttled to 1 Hz (the controller samples at 1 Hz — the
  ~300 ms tick would just be repaint noise) and wrapped so a readout can
  never take the status tick down.
- **⭐ ROUND 2 — operator: *"The incubator is attached to the same board as
  the zp stage, it does not need a separate connect button yet. Later we
  plan to make a separate board via esp32 that monitors temperatures, but
  this is future looking."*** The first cut gave the incubator a Connect
  Hardware card row + a page-level transport combo + Connect button — a
  second connect surface for a board the ZP row already connects. Reworked:
  the Connect-card row is REMOVED (a comment marks where an ESP32-board row
  would return), and the page's session is now **automatic** — NEW
  `_ensure_session()` runs from `showEvent` and the ~300 ms status tick
  (throttled to one attempt per 3 s so a down ZP board is not hammered with
  probes): saved transport "shared" joins the ZP link the moment the ZP
  board is really connected; "simulate" starts the thermal simulator on
  page entry; only the dedicated-serial transport (the future ESP32 box)
  shows manual Port/Detect/Connect controls, hidden otherwise. The page's
  transport combo is gone — Hardware Setup → Incubator is the ONE place the
  transport is chosen. A connected page shows a status pill + a hint naming
  how it is connected; a waiting page names the REAL connect ("connect the
  ZP board on Hardware Setup → Device / the Connect Hardware card").
  Details that keep the automation honest: a failed AUTO attempt never
  raises a banner (the hint covers it; a banner per 3 s retry would spam —
  only a manual serial attempt earns one); a stale SIMULATOR session is
  auto-retired when the transport is switched away (the one transport with
  no real heaters behind it), while a shared/serial session is never torn
  down automatically (disconnecting would not stop the firmware's hold,
  only blind us to it). Mutations M9 (tick no longer starts the session)
  and M10 (auto-retire removed) CAUGHT; the connect-card tests now PIN the
  row's absence so a reversion cannot creep back.
- **Workflows tile, not a sidebar page** — 2-file registration vs 4-way index
  sync + hardware-setup gating; Stress Test / Timing Calibration precedent for
  operational-diagnostic pages. `open_workflow("incubator")` covers routing.
- **Per-machine `IncubatorConfigStore`, not `HardwareConfig`** — heater wiring
  and ceilings are rig properties; a setup file from another machine must not
  carry them (CAMERA_CAL_PERSIST_STORE lesson).
- **Autotune/M190 refused in shared mode** rather than degraded: an M303 holds
  the board's command queue for up to an hour; through the shared lock that
  would starve the poller into a false ZP disconnect. Refusing with the
  remedy named beats a silent hang. (M190 is a graceful degrade rather than a
  refusal — the non-blocking form plus the host-side hold is the DEFAULT
  design anyway, so the outcome is identical and the status line says so.)
- **🐞 REAL BUG FOUND BY THE NEW TESTS, fixed in ZPStage:** a mid-session
  M115 (the shared-transport firmware probe) was classified as a BOARD RESET
  by `_read_until_ok` — its reply legitimately contains
  `FIRMWARE_NAME`/`Marlin`, which are two of the three `_RESET_MARKERS`
  (they assume M115 is only ever sent during `_try_open_marlin`, outside
  that reader). The misread failed the probe AND falsely latched
  `_board_reset_detected`, the flag the ZP position-restore flow trusts.
  New `allow_identity_lines` (default False = byte-identical for every
  existing caller) narrows reset detection to a literal `start` line for
  exactly that transaction; a genuine mid-probe reset still announces
  itself with `start` first. Pinned both ways
  (`test_m115_reply_needs_allow_identity_lines`,
  `test_a_genuine_reset_is_still_caught_during_an_identity_txn`).
- **Fixed while porting: connect/detect off the GUI thread.** The standalone
  window called `controller.connect()` (0.35 s banner wait + multi-second
  probe) and `detect_board()` (0.6 s per port per baud) synchronously on the
  GUI thread — the freeze class the GuiWatchdog exists to catch. Both now
  run on daemon workers with busy-guards, resumed via queued Qt signals.
- **Fixed post-review: revisiting the page no longer clobbers the typed
  setpoint.** `showEvent` re-applies store prefs; re-seeding the preset
  while connected would overwrite the operator's value on every navigation
  — presets now seed only while idle (pinned by test).
- **`priority_write` is a deliberate twin of `quickstop`, not a refactor of
  it** — quickstop is hardware-verified abort code; the small duplication is
  cheaper than risking it.
- **The `import SupportClasses.ZPStage as x` trap** (hit in a test):
  `SupportClasses/__init__` rebinds the `ZPStage` attribute, so that import
  form yields the CLASS; use `importlib.import_module`.
- **Adversarial-review workflow could not run** — all six reviewer agents
  failed on a session usage limit and returned nothing; an empty findings
  list from errored agents is NOT a clean bill. The review was done inline
  instead (threading/lifecycle, lock ordering, fanout tolerance, exclusion
  tracing); the mutation matrix and the 80 tests are what back the quality
  claim.
- **Known edge, disclosed:** `disconnect()` joins the command worker with a
  2 s timeout (the standalone tool's behaviour). A reconnect inside that
  window while a ≥2 s transaction is draining can briefly leave two workers
  competing for one queue — harmless on the shared link (its own lock
  serializes) and bounded by the 20 s shared-transaction clamp, but a
  dedicated-port MarlinLink could interleave two transactions in that
  window. Not fixed here: it requires reworking the standalone teardown
  contract, and the exposure window is a deliberate rapid
  disconnect-reconnect race.
- **Shared-mode fault-message limitation, disclosed:** after a Marlin
  `kill()` the error text is consumed by whichever reader is active — 
  usually the position poller's M114, not the incubator's M105 — so the
  incubator may only see timeouts + the stale-data warning (which names the
  halted-board possibility) rather than the specific MINTEMP/runaway text.
  The board's own protection is unaffected; this is a reporting gap only.
- **Vendored modules stay verbatim where possible** — they carry the tool's
  hardware-verified reasoning (duty 0-127 scale, watchdog-safe ramp math,
  probe-below-current-temp trick) and its 89-check selftest heritage. The
  only vendored-file edits: store paths → `config/hardware/` +
  `logs/incubator/` with env overrides resolved at CONSTRUCTION (test
  isolation), `serial_helpers` prefers the app's own `SerialUtils` module
  instance, `device_config`/`controller` grew the exclusion plumbing, and
  `controller` the transport seam.

## Needs real-HW verification on ME3B V1, IN ORDER

1. **Shared transport first, nothing else is trusted until it passes:**
   connect the ZP board, open Workflows → Incubator → the page joins the
   board BY ITSELF within ~3 s (no connect click anywhere — that is the
   round-2 design), the Firmware tab fills from the probe, and
   `logs/app.log` shows **no** "ZP board RESET detected" line (the
   M115/identity fix — a false reset here would also poison the ZP
   position-restore prompt). Also: open the page BEFORE connecting ZP —
   the hint must point at the ZP connect, and the page must join
   automatically once ZP comes up.
2. Temperatures update ~2 s cadence; **jog Z and a pump while watching** —
   both keep working and the readouts stay live (lock interleaving).
3. **Zone A — the heat bed (HB, `M140`) — is the heater on this rig.** Drive
   **Zone A** to 37 °C **via Ramp** (stock firmware watchdog windows are
   still the open risk — FIRMWARE_NOTES.md); watch for a false
   thermal-runaway halt, and watch the duty bar move — that is the proof the
   command was accepted, not merely sent. Zone B is disabled in this rig's
   `incubator.json` (no heater on HE0, open-circuit thermistor); if it is
   ever re-enabled it should still be blocked by the sensor message.
3a. **Round-4 check — the one the whole session was about.** Set Zone A to
   37 °C in the app and confirm it *reaches and holds* it: the bench proved
   the hardware does 21.5 → 37 °C in ~7 s and holds ±0.65 °C for minutes, so
   anything slower or flatter in the app is a software difference, not the
   heater. Then **provoke the failure that was fixed**: while it is holding,
   disconnect and reconnect the ZP board (or just let an auto-reconnect
   happen). Marlin resets on port open and forgets the target, so the card
   should show *"board forgot the setpoint x1, restored"*, `app.log` should
   carry a `re-asserting (board reset #1)` WARNING, and **the temperature
   should not sag** — before this fix the hold silently died there.
3b. **Round-3 regression check, deliberately provoking the bench fault:**
   temporarily re-enable Zone B on Hardware Setup → Incubator and try to set
   it. Expected: the card turns **REFUSED** with the board's own `Error:`
   text and the words *"is really wired to HE0"*, `logs/app.log` carries a
   `firmware REFUSED 'M104 S37'` WARNING, and the retry **stops** — the old
   behaviour was a healthy-looking card and a refused command re-sent every
   minute forever. Then disable Zone B again.
4. **Run a small print while holding** — the incubator sensor table may go
   "stale" during PRINT_PATH bursts (polling yields, by design, without a
   board-gone-quiet warning) and recovers after; the print's pacing is
   unchanged.
5. Heater OFF / ALL HEATERS OFF; then disconnect ZP → the incubator shows
   link lost and says the board keeps holding its setpoint.
6. Hardware Setup → Incubator: lower the ceiling to 45 °C, Save → the
   page's setpoint spins clamp immediately (no reconnect needed).
7. Connect Hardware card: confirm there is NO incubator row (round-2
   decision — the ZP row is the connect), and that ZP Disconnect makes the
   Incubator page report the link lost + rejoin on ZP reconnect.
8. Close the app while heating → the prompt appears; choose "turn off" →
   `logs/zp_serial.log` shows `M140 S0`/`M104 S0` acked BEFORE the port
   closes; choose "leave running" → the hold survives the app exit.
9. Autotune in shared mode → the refusal names the manual-PID and
   dedicated-connection alternatives.
10. (Future second board only) dedicated transport: Detect must skip the
    ZP and XY ports, and a ZP reconnect scan must skip the incubator's
    port both live and from the saved hint.
11. **Round 3 — the jog-panel readout:** on the left context panel switch to
    **Custom → ＋ Add section → 🌡️ Incubator (zone temps)**. With no session
    it must point at Workflows → Incubator (and must NOT start one — the
    Incubator page stays disconnected until you open it); with Zone A
    holding it tracks `24.6 → 37 °C · 42%` and reads *heating* then *at
    target*; only Zone A's row appears (Zone B is disabled in this rig's
    config); the section survives a restart via the saved panel layout.

---

## ⭐⭐⭐⭐ ROUND 5 — THE HOLD OSCILLATES: PID RETUNED FROM THE LOG, HELD HOST-SIDE, NO EEPROM

Operator, 2026-08-18: *"I just ran the incubator, and we can see that there is
oscillation in the bed temperature around 37 degrees. From this signal can we
tune the PID to fit in a tighter temperature range. I want 37.4 C +- 0.3 C as
the expectation."* Then, on the remedy: *"I dont want to flash the board unless
i really have to."*

### What the signal actually says

Source: `logs/incubator/incu_20260818_091055_hold.jsonl` (1 Hz, 8154 s).
Settled window t=3880-7997 s:

* period **588 s** (autocorrelation r = 0.93 — a strong, sustained limit cycle,
  not drift);
* peak-to-peak **1.15-2.06 °C**, sd 0.42 °C;
* duty **railing at 0 % or 100 % for 55 % of samples**, mean 57 %.

A duty that spends half its life on the rails is not regulating, it is
bang-banging.

**The confounder was cleared before any tuning conclusion was drawn.** The run
logs seven `setpoint_reasserted` events, and a board that keeps forgetting its
target would produce exactly this sawtooth by a completely different mechanism
(round 4). All seven are at t=137-1075 s, i.e. **during the ramp**; from
t=1507 s to t=8144 s the target sits at 37.0 with no reassertion and no change.
So the oscillation is a genuine control limit cycle.

### Plant identification, and why the obvious method fails

Regressing dT/dt against T over the log gives nonsense (ambient 7.4 °C and
31.7 °C on two attempts, against a log that *starts* at 21.3 °C with the heater
off): the samples come from a closed loop where duty and temperature are
correlated through the controller, and the full-power bursts are too short for
the block to reach its quasi-steady rate. Classic closed-loop identification
bias — recorded here because the first two attempts looked plausible.

What is trustworthy is three measurements, and the model is fitted to exactly
those three:

1. **DC gain** — 57.0 % duty holds 36.90 °C against 21.3 °C ambient ⇒
   K = 27.4 °C per unit duty;
2. **|G(jω_u)| = 0.979 °C/unit-duty** and **∠G(jω_u) = −136.5°**, taken as the
   fundamental Fourier components of duty and temperature at the limit-cycle
   frequency. In a self-excited limit cycle u→y *is* the plant, so this is a
   direct frequency-response measurement (relay-feedback identification), and
   the oscillation (amp 0.57 °C) is far above the noise (σ 0.077 °C).

Solving gives **K = 27.4 °C/unit-duty, τ = 2614 s (43.6 min), θ = 79 s**,
θ/τ = 0.030 — strongly lag-dominant. Ceiling at 100 % duty ≈ 48.7 °C; 37.4 °C
needs ≈ 59 % duty.

**The model is validated by reproducing the fault.** Simulating the closed loop
with the stock gains against this plant gives period **594 s vs 588 measured**,
duty **57 % vs 57 %**, saturation **58 % vs 55 %**, mean **36.91 vs 36.90 °C**.
A model that reproduces the observed limit cycle to 1 % on period is a model
worth designing against.

### Root cause

Stock Ender-3 bed gains `M304 P41.78 I7.32 D158.93` ⇒ integral time
Ti = Kp/Ki = **5.7 s**, against a water block whose time constant is **2614 s**.
**Integral action 458× too fast.** Those gains are correct for a thin aluminium
printer bed; the water block is a different plant by two orders of magnitude.
Nothing was wrong with the heater, the wiring or the firmware.

### The retune

SIMC (Skogestad) with τ_c = 2θ: Kc = τ/(K(τ_c+θ)), Ti = min(τ, 4(τ_c+θ)).

**`M304 P102.40 I0.11 D0.00`** (Ti = 931 s), phase margin 62°.

| | true band | reported band | −3 °C ambient step |
|---|---|---|---|
| stock P41.78 I7.32 D158.93 | ±0.65 °C | — | — |
| **P102.40 I0.11 D0.00** | **±0.015 °C** | ±0.28 °C | 0.32 °C, 23 min |

Ladder, all simulated against the identified plant: `P153.6 I0.24` (τ_c=θ,
PM 50°) recovers from a disturbance in 10 min instead of 23; `P76.8 I0.06`
(τ_c=3θ, PM 68°) is the most robust but a 3 °C ambient step costs 0.59 °C.
Stable across ±40 % error in K or τ individually and θ×3; only the compound
worst case (K+40 % **and** τ−40 % **and** θ×2 simultaneously) destabilises the
τ_c=2θ choice, and the model uncertainty is far smaller than that because it
was fitted to a validated limit cycle.

**D = 0 is deliberate, not an omission.** Swept 0/200/500/1000: derivative gave
*no* overshoot reduction (+0.62 → +0.68 °C) while duty chatter went from 3 % to
20 % against the 0.077 °C sensor noise. With θ/τ = 0.03 there is almost no dead
time for D to anticipate.

**Ki = 0.11, not 0.108.** `ZoneSpec.set_pid` formats M304 to two decimals (and
tests pin that exact string), so the designed 0.108 would be silently truncated.
0.11 was simulated and is indistinguishable — Ti 931 s vs 948 s, identical band,
identical overshoot. Choosing a value that survives the format beats widening
the format.

### The remedy is host-side, and that is not merely a preference

`M304` is a runtime command and `M500` writes EEPROM — **neither is a firmware
flash**, and that was said plainly. But EEPROM is avoidable *and worth
avoiding*: **a Marlin reset reverts the running gains to EEPROM**, and this
board resets whenever its port is opened (DTR). That is the very mechanism
behind round 4's forgotten setpoint. RAM-only gains would therefore snap back
to the stock oscillating values on the next ZP reconnect **with nothing
explaining why** — the 588 s limit cycle returning silently.

So the gains are held in the per-machine store and re-asserted the same way the
setpoint is:

* `config_store` gains a per-zone `pid` block + `pid_apply_on_connect`, with
  `parse_pid` **refusing** anything malformed/absurd rather than clamping — a
  clamped gain is a *different controller than the one that was tuned*, applied
  silently to a live heater. Absent is recoverable (the board keeps its own
  gains); wrong is not. `Kp = 0` reads as "do not manage this zone".
* `service.apply_store_config` *declares* them onto the controller
  (`set_configured_pid`) — the controller deliberately does not read the store,
  the same split the ceiling and zone labels already use.
* `controller.apply_configured_pid()` pushes them on connect (unforced: the
  probe has just read the board's real gains, so an already-matching push
  correctly no-ops) and **forced** whenever the setpoint keeper catches a reset.
* ⚠ **The `force` flag is load-bearing.** After a reset the board is back on
  EEPROM gains but `rt.pid` still holds the *pre-reset* value, so an unforced
  push would compare-equal, skip, and leave the stock gains in charge. Pinned
  by `test_FORCE_pushes_even_when_the_cached_gains_match` and by a mutation.
* The Hardware Setup → Incubator tab grows Kp/Ki/Kd per zone and the
  re-assert checkbox, on the existing `load()`/`commit()` contract (nothing
  written before Save, ONE `store.save()`), with `commit()` also pushing to a
  LIVE controller so Save takes effect without a reconnect.

### 🐞 Fixed en route: the ramp faked seven board resets

`commanded_c` is updated the instant the ramp decides, while the G-code is still
queued for the worker — so the board legitimately still reports the *previous*
step for a sample or two, and the keeper's two-sample debounce was not enough.
Every ramp step logged a false *"the board forgot its setpoint"*. Harmless to
the hold, but it inflates the counter whose entire purpose is to make a
genuinely rebooting board visible. New `REASSERT_GRACE_S = 6.0` measured from
`_commanded_at`; pinned both ways (a fresh command is given time to land; a real
reset is still caught once the grace expires).

### ⚠ Disclosed, not fixed

* **Warm-up overshoots ≈ +0.8 °C for ~20 min.** Slower integral action means
  the integrator winds up over the hour-long climb, and Marlin's bed PID has
  only a crude clamp (`MAX_BED_POWER/Ki`), no true anti-windup. Swept the ramp
  lead 0.3-3.0 °C: it barely helps (+0.82 → +0.40 °C at a 0.3 °C lead, at the
  cost of taking 102 min instead of 39 to reach target). What *does* work is
  soaking below target for ≥ τ: 36.4 °C for 80 min then step to 37.4 gives
  +0.13 °C and zero time out of band. In practice: warm up before loading cells.
* **Sensor noise, not control error, is the binding limit on ±0.3 °C.**
  σ = 0.077 °C, so the *reported* value scatters ±0.28 °C even with the block
  dead steady at ±0.015 °C. 100 % of samples land inside ±0.3, but with almost
  no margin. A 10 s smoother on the displayed value would give ±0.07 °C;
  deliberately NOT added here, because a display filter also masks genuine
  short excursions and that is a separate decision from tuning the loop.
* **The gains are inferred, not read back.** The limit-cycle match plus
  CLAUDE.md say the board is on `P41.78 I7.32 D158.93`; yesterday's power
  staircase wrote `M304 P… I0 D0` but never `M500`, and today's session
  DTR-reset the board, so EEPROM should be untouched. Confirm with `M503`.

### Tests

NEW `tests/test_v718_incubator_pid_tuning.py` (**25**): store round-trip and
refusal matrix, apply-on-connect, the forced reset re-assert, diagnostic and
`pid_available` stand-off, a refused push not claiming success, the ramp grace
both ways, the service wiring, an AST pin that connect and the keeper both call
`apply_configured_pid`, and the REAL Hardware Setup panel round-tripping gains
(including Kp=0 clearing them and nothing being written before Save).

Two existing keeper tests in `test_v718_incubator_backend.py` were **updated,
not loosened**: a detected reset now submits two things (setpoint *and* PID
re-assert), so counting raw submissions conflated them; they now count
`_send_zone_cmd` specifically and a new helper exposes the PID re-asserts.

### Needs real-HW verification on ME3B V1, IN ORDER

1. `M503` on the Firmware/Console tab — confirm the bed line really reads
   `P41.78 I7.32 D158.93` before changing anything.
2. Connect ZP, open Workflows → Incubator. `logs/app.log` should carry
   `incubator bed: applied PID Kp=102.40 Ki=0.11 Kd=0.00 (connect)` and the PID
   tab should show the new values. **No `M500` anywhere.**
3. Ramp Zone A to **37.4 °C**. Expect ~39 min to target and a warm-up peak
   around 38.2 °C, then settling.
4. **The payoff:** once settled, watch for ≥ 30 min. The duty should sit around
   57-60 % and *stop railing*; the 588 s swing should be gone. Judge the block,
   not the number — ±0.28 °C of the reported scatter is thermistor noise.
5. **Provoke the reset path:** while holding, reconnect the ZP board. Expect
   *"board forgot the setpoint"* **and** a second `applied PID … (board reset)`
   line, and the hold must not resume oscillating.
6. Confirm the ramp no longer logs false reassertions — the counter should stay
   at 0 through a whole ramp.
7. Hardware Setup → Incubator: change Kp, Save, and confirm it applies live
   (`… (settings saved)` in the log) without a reconnect.
8. If any residual oscillation remains, step down the ladder to
   `P76.8 I0.06`; if disturbance recovery is too slow, step up to
   `P153.6 I0.24`.

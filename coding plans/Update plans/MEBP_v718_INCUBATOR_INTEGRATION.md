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

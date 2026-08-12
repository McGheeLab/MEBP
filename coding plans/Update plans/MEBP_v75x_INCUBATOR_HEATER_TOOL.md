# MEBP v7.5.x — Standalone two-zone incubator heater tool

## Objective

Give the ZP stage board (BigTreeTech SKR Mini E3 V3, Marlin) a fully controllable
**two-zone incubator heater** UI for cell culture at ~37 °C, covering live
temperature, setpoints, the board's PID constants, sensor calibration, heater
health, and full manual control.

Delivered as a **completely separate test module** under `tools/incubator/` at the
operator's explicit request ("as a full separate test module, do not code this
into main"). It opens its **own** serial connection and is not wired into
`main.py`, `gui/app.py`, or any `SupportClasses` class.

## Hardware

| Zone | Heater | Sensor | Marlin slot | Commands |
|---|---|---|---|---|
| **A** — water block | polyimide film heater 10 × 89 mm on `HB` | thermistor on `THB` | `TEMP_SENSOR_BED` | `M140`/`M190`, `M304`, `M303 E-1` |
| **B** — stage area | heater on `HE0` | thermistor on `THO` | `TEMP_SENSOR_0` | `M104`/`M109`, `M301`, `M303 E0` |

Zones are **independently setpointed, each closing its own Marlin PID loop**
(operator's choice), which preserves Marlin's per-heater thermal protection and
lets zones with different losses be trimmed separately.

**Thermistors only in this phase.** An RTD/MAX31865 reference was considered and
deferred; additional distributed sensors will arrive later on a **separate sensor
box** with its own connection, because this board exposes only the two analog
temperature inputs above and both are consumed by the control zones.

## Physics that drove the design

A ~10–25 W film heater with ~8.9 cm² contact area against a large water-filled
aluminium block gives a thermal time constant of order an hour with very little
power headroom. Consequences baked into the tool:

1. **Stock Marlin will halt the board.** `THERMAL_PROTECTION_BED` /
   `THERMAL_PROTECTION_HOTENDS` demand a ~2 °C rise within 40–60 s; this rig
   cannot, so Marlin declares thermal runaway, calls `kill()`, and stops
   answering G-code. Documented as the #1 expected bring-up failure with a
   firmware fix (widen the windows, never disable the protection).
2. **Bed PID is disabled in stock Marlin** (`PIDTEMPBED` off ⇒ bang-bang), while
   hotend PID is on by default. Detected per zone and reported honestly; PID
   fields read "unavailable", never zeros.
3. **Marlin setpoints are integer °C** (`celsius_t` = `int16_t`), so 37.5 °C
   cannot be commanded. Surfaced explicitly, verified empirically at connect, and
   worked around with an optional host-side setpoint dither that the block's
   thermal mass low-passes into a smooth fractional hold.
4. **Board-side waits are avoided.** `M190`/`M109` could block the command queue
   for the best part of an hour on this rig, during which a queued "heater off"
   could not get through, so the default is non-blocking `M140`/`M104` plus
   host-side arrival detection. Board-side wait remains an explicit opt-in.
5. **Heater adequacy is measured, not assumed** — steady-state duty at hold is
   the fraction of heater power the losses consume, so it answers "is the heater
   big enough?" directly.

## Files added (all new)

Everything under `tools/incubator/`:

| File | Role |
|---|---|
| `run.py` | entry point (`python tools/incubator/run.py`) |
| `marlin_gcode.py` | pure parsing/building; generic `KEY:value[/target]` frame parser |
| `zones.py` | `ZoneSpec` + `BED_ZONE`/`HOTEND_ZONE` — the only place the zones differ |
| `sensors.py` | `SensorChannel`/`SensorSource`/`MarlinSensorSource`/`SensorHub` |
| `marlin_link.py` | reader thread, rolling-deadline `ok` handshake, out-of-band priority writes |
| `fake_marlin.py` | simulated board, two thermal models, time acceleration, fault injection |
| `probe.py` | connect-time firmware capability + setpoint-resolution detection |
| `controller.py` | `IncubatorController` — framework-agnostic facade (no Qt) |
| `stability.py` | rate, ETA, overshoot, settle, ripple, steady duty, time constant |
| `calibration.py` | host-side per-channel sensor correction |
| `safety.py` | setpoint ceiling, fault latch, generic divergence check |
| `telemetry.py` | JSONL run logger to `logs/incubator/` |
| `device_config.py` | read-only COM-port hints from `config/hardware/devices/*.json` |
| `_serial_helpers.py` | loads `SerialUtils.py` **by path**, bypassing the app package |
| `gui.py` | `IncubatorWindow`, Qt bridge, `_TempTrendPlot` with a real °C axis |
| `selftest.py` | 96-check end-to-end verification against the simulator |
| `README.md`, `FIRMWARE_NOTES.md` | usage/safety, and the Marlin config checklist |

**Only file modified outside that directory:** `.gitignore` (ignore
`tools/incubator/_data/` and `logs/incubator/*.jsonl`, following the existing
`logs/prints/*.jsonl` precedent).

## Reuse

- `gui/styles.py::build_theme` + `COLORS`, `gui/scaling.py::s/sf/scaled_font_size`
  — safe to import (`gui/__init__.py` is docstring-only). `QApplication` is
  constructed before the first `s()` call, since `scale_factor()` memoises.
- Plot idiom from `gui/pages/print_results.py::ErrorTimeSeriesWidget` (numeric
  axis + gridlines) and `gui/pages/workflows/timing_calibration_workflow.py`
  (`s()`-scaled padding, empty-state early return). No matplotlib/pyqtgraph.
- Telemetry idiom from `SupportClasses/PrintExecutionLogger.py` (path from
  `__file__`, `%Y%m%d_%H%M%S`, per-line flush, idempotent stop, swallow-all).
- Simulator idiom from `SupportClasses/ZPStageSimulator.py` (JSON-file EEPROM).
- Entry-point idiom from `tests/find_xy_stage.py` (probe upward for
  `SupportClasses/`).
- `SupportClasses/SerialUtils.py` logic, loaded by **file path** — see Issues.

## Implementation steps

- [x] Package skeleton (`tools/__init__.py`, `tools/incubator/__init__.py`) + `.gitignore`
- [x] `marlin_gcode.py` — generic frame parser, line classifier, fault/PID/autotune parsing
- [x] `zones.py` — two zones, one place for their command differences
- [x] `sensors.py` — multi-channel hub (the phase-2 seam)
- [x] `marlin_link.py` — reader thread, `send_and_wait`, `send_priority`
- [x] `fake_marlin.py` — physics-based simulated board
- [x] `stability.py`, `calibration.py`, `safety.py`, `telemetry.py`, `device_config.py`
- [x] `probe.py` — connect-time capability detection
- [x] `controller.py` — zone-parameterised facade
- [x] `gui.py` — window, bridge, trend plot
- [x] `run.py`, `README.md`, `FIRMWARE_NOTES.md`
- [x] `selftest.py` — 96 checks, all passing (incl. offscreen GUI build)
- [x] `ramp.py` — watchdog-safe staircase setpoint ramp (+ controller/GUI wiring)
- [x] Firmware rebuilt with widened thermal windows (`C:\Users\mcghe\MarlinIncubator`)
- [x] **Sensor hot-plug re-check** — `↻ Rescan sensors` (see below)
- [x] **Duty scale fix — 127, not 255** (see below)
- [ ] **Real-hardware verification on ME3B (see Testing notes)**

### Duty scale fix — `@:`/`B@:` are 0-127, not 0-255

Operator report: *"the heater never goes past 50% power … is there a way we can
increase this?"* — **nothing was limiting it; the readout was halving it.**

`SensorField.power_pct` divided the `M105` duty field by 255. Marlin's soft-PWM
period is **127 ticks** (`if (pwm_count_tmp >= 127)` in `Temperature::isr()`,
`temperature.cpp:3575`) and every writer stores `control_value >> 1` to match:

```cpp
temp_bed.soft_pwm_amount = (int)get_pid_output_bed() >> 1;   // PID      :1644
temp_bed.soft_pwm_amount = MAX_BED_POWER >> 1;               // bang-bang:1652
```

`M105` echoes `soft_pwm_amount` verbatim through `getHeaterPower()`. The
bang-bang line is the decisive proof: that branch *means* "heater fully on", and
with the stock `MAX_BED_POWER 255` it writes **127**. So a saturated heater
reports `B@:127`, which ÷255 renders as **49.8 % — exactly the reported symptom.**

**Why this mattered more than a cosmetic error.** The halved value fed the duty
bar, the trend plot's duty trace, the JSONL telemetry, and
`stability.steady_duty_pct` — the metric this tool exists to produce, and the one
the README tells the operator to read as "the fraction of heater power your losses
consume". Understating it by 2× invites precisely the wrong conclusion: *"only 50 %
duty, so there is plenty of headroom"* when the heater is flat out and the real
answer is insulation or more watts. It also corroborates the thermal-runaway
report — a heater already at 100 % that still cannot make 2 °C/min is exactly the
false-trip case the widened windows address.

Confirmed there is **no** power limit to raise: `MAX_BED_POWER 255` ("255=full
current") and `MIN_BED_POWER` commented out in the Ender-3 / SKR Mini E3 V3 config.

- `marlin_gcode.py` — new `HEATER_PWM_FULL_SCALE = 127.0` carrying the source
  citation; `power_pct` scales against it. One chokepoint, so one fix corrected
  every consumer.
- `fake_marlin.py` — `_temp_report` now emits `duty >> 1`, i.e. Marlin's reporting
  scale rather than the model's internal 0-255. Emitting the raw value is what let
  the simulator agree with a host that was wrong by 2×; a faithful fake would have
  caught this.
- Tests pin the scale directly (`B@:127` → 100 %, `B@:114` → 89.8 %, clamping) plus
  an end-to-end check that a *saturated* zone reads ~100 % and that the simulator
  emits `B@:127` at full power.

**Not yet confirmed on the bench**, because confirming it means running the heater
to saturation. The source evidence is conclusive and version-stable (the `>> 1` /
127-tick scheme is long-standing, so it holds for the bugfix-2.0.x build too), and
the first real ramp will show it for free: watch for `B@:` topping out at 127.

### Sensor hot-plug re-check (`↻ Rescan sensors`)

Operator report: *"the temp sensors need a reset button so if I plug one in after
I started it can refresh the board state and know it has the sensor."*

**Cause.** `FirmwareProbe` decides once, at connect, whether each zone's sensor is
usable, and that verdict is cached on `ZoneRuntime.sensor_ok` — which *gates
heating* in `set_target` / `start_ramp` (deliberately: driving a heater with a
broken sensor is how thermal runaway happens). Plug a thermistor in afterwards and
Marlin reports it on the very next `M105`, because it samples every configured ADC
continuously; the **firmware** needs no reset, only our cached verdict does. But
with no way to refresh it, the tool kept refusing to heat until the operator
thought to disconnect and reconnect — indistinguishable from a broken tool.

**Fix.** One `M105`, re-evaluated. Deliberately narrow: no heater is touched, no
setpoint is written, and the setpoint-resolution probe is **not** repeated (that
one commands a target, which a button labelled "re-check sensors" has no business
doing). PID availability is left alone, being compile-time.

- `probe.py` — the plausibility test moved out of `_probe_sensors` into
  `evaluate_zone_sensor(spec, frame)` so the re-read applies **exactly** the same
  rule; two copies of a threshold is how a rescan ends up disagreeing with the
  connect probe about whether it is safe to heat. Plus `read_temp_frame()`,
  `SensorRescan` (recovered / lost / still_bad / **not_configured**, with a
  `summary()`), and `rescan_sensors(link, report)` which updates the report in
  place.
- `controller.py` — `rescan_sensors()` (queued to the command worker, so it cannot
  race the serial port) → `_do_rescan_sensors()`: refreshes `sensor_ok` /
  `sensor_fault`, **resets the StabilityTracker of any recovered zone** (its
  history spans the jump from an open circuit to a real reading, which would
  otherwise be reported as a wild rate of change and a huge ripple), logs a
  `sensor_rescan` telemetry event, and re-fires the probe callback — that is what
  actually refreshes the zone cards, firmware tab and banner, since all three
  render from the report.
- `gui.py` — `↻ Rescan sensors` on the sensor table, plus `↻ Re-check this
  sensor` on any zone card blocked by its sensor (shown via `_show_blocked(...,
  rescan=True)`, hidden once READY). New `_probe_flagged` flag lets a later probe
  take *its own* banner down once resolved without clearing a banner some other
  event owns.
- `fake_marlin.py` — `ZoneState.sensor_configured` + `configure_sensor()` models
  `TEMP_SENSOR_x 0` by omitting the zone's fields from `M105` entirely.

**Two failures that look alike and need opposite responses**, now reported
separately: a *configured but unplugged* sensor still reports (open circuit,
≈ −15 °C) and is a wiring fix; a sensor *absent from `M105`* means the firmware was
built with `TEMP_SENSOR_x 0` and wiring one in changes nothing until it is
reflashed. A **halted** board is reported as needing a power-cycle rather than
retried, because every fault we latch on is one that makes Marlin `kill()` and
stop answering G-code.

## Testing notes

`python tools/incubator/selftest.py --gui` → **146 passed, 0 failed**. It drives the
production code path against the simulator, covering: the generic parser
(hotend-absent frames, extra sensor letters, `E:`/`W:` wait tokens, `ok`-prefixed
vs bare autoreport), probe detection, both zones converging independently,
setpoint quantisation/calibration/ceiling, dither, autotune success **and** the
`timeout`/`Bad extruder number` failures, bang-bang degradation, EEPROM
round-trip, fault latching (thermal runaway + detached-thermistor MINTEMP),
out-of-band `M108`/`M112` during a long transaction, the phase-2 sensor seam,
JSONL telemetry, stability maths on synthetic data, import decoupling, the
hung-board staleness warning, the watchdog-safe ramp (asserting **every** step
stays under Marlin's arming threshold), the sensor hot-plug re-check, and an
offscreen build of the full window.

The sensor-rescan section is worth calling out because its load-bearing assertion
is not "the button ran" but **"the heating gate follows the re-check"**: detach →
rescan → `set_target` is refused and says why; re-attach → rescan → `set_target` is
accepted, with no reconnect. The GUI section then asserts the per-zone button
appears exactly when a zone is blocked, the setpoint controls disable and
re-enable with it, and the banner stops claiming a sensor problem once resolved
(it correctly still shows the *remaining* firmware limitations, so "banner hidden"
would have been the wrong assertion).

One incidental confirmation from writing these tests: calling
`MarlinLink.send_and_wait` from anywhere other than the command worker thread
races for the `ok` and intermittently returns nothing. The first draft of the test
did exactly that and failed; `controller.rescan_sensors()` queues to the worker,
so the production path is unaffected. Worth remembering when adding future
diagnostics — the deterministic-stub approach the test now uses is the way to
exercise parse/classification logic.

**Needs real-HW verification on ME3B** — ordered checklist at the end of
`tools/incubator/FIRMWARE_NOTES.md`. In brief: confirm each heater's voltage
matches the board supply *before powering*; check the `M115` banner + capability
probe; confirm both sensors appear and track (hand-warm each in turn — a swapped
THB/THO would silently control the wrong zone and software cannot detect it);
verify `M140 S0`/`M104 S0` and the fault-latch path; then attempt a 37 °C hold and
watch for a false thermal-runaway halt, which decides whether the firmware
changes must happen before anything else is meaningful.

## Issues & decisions

- **`SupportClasses/__init__.py` eagerly imports the whole hardware layer**
  (`ZPStageManager`, `StageController`, `PrintManager`, multiprocessing
  `XboxController`, and the XY simulator, which reads a profile off disk at import
  time). A plain `from SupportClasses.SerialUtils import …` therefore pulled all
  of it into a standalone diagnostic tool — contradicting the decoupling goal and
  risking breakage from unrelated modules or a missing optional dependency. Fixed
  with `_serial_helpers.py`, which loads `SerialUtils.py` via
  `importlib.util.spec_from_file_location` (skipping the package `__init__`) and
  falls back to plain pyserial. The self-test asserts neither `SupportClasses` nor
  `ZPStageManager` is imported.
- **Bug found by the self-test:** `TelemetryLogger.log_event("fault", kind=…)`
  collided with the method's own `kind` parameter → `TypeError` raised on the
  reader thread and swallowed at debug level, so faults latched but never reached
  the UI. The parameter is now **positional-only** (`event: str, /`), which makes
  that collision impossible for all present and future call sites, and the
  reader's consumer-exception handler now warns for the first few occurrences
  instead of hiding them at debug level.
- **Stability metrics reworked:** requiring an unbroken in-band streak made the
  duty and ripple readouts blink to "—" on every small PID excursion, i.e. exactly
  when they matter. Now a separate "first reached band" marker (not reset by later
  excursions) anchors a trailing window, with a tolerance band and a
  majority-near-target test so a ramp still cannot contaminate hold statistics.
- **GUI seeding:** a window built against an already-connected controller never
  received the probe result (it fires during `connect()`), leaving the Firmware
  tab and capability labels blank. `IncubatorWindow.__init__` now seeds from
  existing controller state.
- **Silently hung board:** the reader detects an *unplugged* device (reads raise)
  but not a board that hangs or stops autoreporting with a valid port handle.
  Added a data-staleness check that warns and re-polls, since otherwise the
  operator faces a frozen readout with no explanation.
- **`M112` is not a soft stop** — it calls `kill()` and needs a power-cycle. It is
  visually separated and labelled as such; `M140 S0`/`M104 S0` is the normal stop.
- **`M108` is the correct cancel** for both a board-side wait and a running
  autotune (needs `EMERGENCY_PARSER`); sent out-of-band via `send_priority`.
- **Safety scope stated plainly in the UI:** the firmware owns the control loop
  and keeps heating if the host goes away. Host-side guardrails (50 °C ceiling,
  confirmations, fault latch, close-time heater-off prompt) reduce operator error;
  they are not a substitute for `THERMAL_PROTECTION_*`, which stays enabled.
- **Phase-2 finding (recorded for later):** `TEMP_SENSOR_BED = -5` (MAX31865
  PT100/PT1000) *is* supported by Marlin, but on stable **2.1.2.8** it does not
  compile — `temperature.cpp` references `MAX31865_SENSOR_OHMS_BED` /
  `MAX31865_CALIBRATION_OHMS_BED` which stock configs never define and no
  `SanityCheck` guards. Present on `bugfix-2.1.x`. Also: prefer **PT1000 over
  PT100** (10× less lead-resistance error), and use `TEMP_SENSOR_CHAMBER` while
  characterising rather than `TEMP_SENSOR_REDUNDANT`, which would halt the board
  over the very offset you are trying to measure.

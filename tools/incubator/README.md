# Incubator heater bring-up tool (standalone)

A self-contained control and diagnostic UI for a **two-zone incubator stage** held
at ~37 °C, driven by the existing ZP stage board (BigTreeTech SKR Mini E3 V3
running Marlin).

**This is not part of the MEBP application.** It lives entirely under
`tools/incubator/`, opens its own serial connection, and nothing in `main.py`,
`gui/app.py` or `SupportClasses/` was modified to accommodate it (the only file
touched outside this directory is `.gitignore`).

It does reuse existing code, read-only:

* `gui/styles.py` + `gui/scaling.py` for the theme and DPI scaling — safe to
  import directly, since `gui/__init__.py` is a docstring with no imports.
* `SupportClasses/SerialUtils.py` for port enumeration and friendly error
  messages — but loaded **by file path** (`_serial_helpers.py`) rather than as
  `SupportClasses.SerialUtils`, because `SupportClasses/__init__.py` eagerly
  imports the entire hardware layer (`ZPStageManager`, `StageController`,
  `PrintManager`, the multiprocessing `XboxController`) and reads a profile off
  disk as a side effect. Loading the single module keeps the shared logic without
  the baggage, and the self-test asserts that neither the package nor
  `ZPStageManager` is ever imported.

Disconnect detection is the reader thread noticing failed reads, plus a
data-staleness check that catches the nastier case: a board that has hung or
stopped reporting while its port handle stays perfectly valid.

```
python tools/incubator/run.py                    # normal use
python tools/incubator/run.py --simulate         # no hardware needed
python tools/incubator/selftest.py --gui         # verify everything works
```

---

## Hardware it expects

| Zone | Heater output | Sensor input | Controlled as | G-code |
|---|---|---|---|---|
| **A** — water block | `HB` (bed) | `THB` | Marlin bed | `M140`/`M190`, `M304`, `M303 E-1` |
| **B** — stage area | `HE0` (hotend) | `THO` | Marlin hotend 0 | `M104`/`M109`, `M301`, `M303 E0` |

Each zone is independently setpointed and closes **its own Marlin PID loop** on
its own thermistor, which keeps the firmware's per-heater thermal protection
intact and lets zones with different heat losses be trimmed separately.

Zone A is a polyimide film heater (10 × 89 mm) bonded to a large aluminium block
filled with water.

---

## Read before you plug anything in

* **Check heater voltage against the board supply.** A 12 V film heater on a 24 V
  output dissipates 4× its rating and will fail. Software cannot protect you here.
* **Only one program can own a COM port.** If the main MEBP app is connected to
  the ZP board, this tool cannot open it, and vice versa. The tool says so
  explicitly instead of showing a generic serial error.
* **Stock Marlin will fight this rig.** Its thermal-runaway watchdog expects a
  printer bed that heats fast, and it disables bed PID by default. See
  [`FIRMWARE_NOTES.md`](FIRMWARE_NOTES.md) — the tool detects both at connect time
  and names the exact settings, but cannot fix them.
* **The firmware owns the control loop, not this tool.** Closing the window or
  unplugging USB does **not** stop heating; Marlin keeps holding its last
  setpoint. On close the tool offers to switch both heaters off, and it says
  plainly that this is a courtesy rather than a safety mechanism.

---

## What the tool does

**Connection** — port list pre-filled from the app's cached ZP port
(`config/hardware/devices/*.json` → `zp_stage.last_port`, read-only), 38400 baud
by default (the same rate `ZPStage.py` uses for this board).

**Firmware probe on every connect.** Rather than guessing which build options are
enabled, it asks: `M115` for identity and capabilities, one `M105` to see which
sensor fields actually exist, `M503` to see whether each zone has PID, and a
deliberate fractional-setpoint test to confirm quantisation. Anything missing is
reported with the specific `Configuration.h` option to change.

**Per-zone control** — large readout, setpoint with a **50 °C hard ceiling**,
presets, Heater OFF, live duty, and each zone's PID and autotune.

**↻ Rescan sensors** — re-reads the sensors and re-decides which zones may be
heated, without reconnecting. Needed because the probe's verdict *gates heating*:
plug a thermistor in after starting and Marlin reports it on the very next
`M105`, but a cached verdict would keep refusing to heat and look like a bug. One
button on the sensor table, plus one on any zone card that is blocked by its
sensor. It sends a single `M105` — no heater is touched and no setpoint changes.

It also distinguishes two failures that look alike but need opposite responses: a
**configured but unplugged** sensor still reports (as an open circuit, ≈ −15 °C)
and is a wiring fix, whereas a sensor **absent from `M105` entirely** means the
firmware was built with `TEMP_SENSOR_x 0` and no amount of plugging in will help.
A halted board is reported as such rather than retried, since every fault that
latches is one that makes Marlin `kill()` and stop answering.

**Stability & health** — the numbers that actually tell you whether the heater
works: rate of change in °C/min, time to setpoint, overshoot, settle time, steady
ripple ±, thermal time constant, and most usefully **steady-state duty %**. At a
stable hold, duty *is* the fraction of heater power your losses consume, so a
figure near 100 % means the heater cannot hold that setpoint — insulate the block
or fit more power.

> **Duty scale.** `M105`'s `@:`/`B@:` fields run **0-127, not 0-255.** Marlin's
> soft-PWM period is 127 ticks and every writer stores `control_value >> 1`, so
> `B@:127` is 100 %. The raw console shows those raw numbers; the percentages
> everywhere else are scaled against 127. Worth knowing before you conclude a
> heater has headroom it does not have — this tool divided by 255 at first and
> displayed a heater at full power as "50 %".

**Trend plot** — both zones' temperature and target on a real °C axis with duty as
a secondary trace, over a selectable window up to 6 hours (this rig's dynamics
play out over tens of minutes).

**Sensor calibration** — correcting what the board *reports* against a trusted
external thermometer. Applied on the host at the display and
setpoint-translation layer only; the firmware's thermistor tables are never
touched. This is a different thing from PID autotune, which tunes the control
loop, and the UI keeps them clearly separate.

**Raw console** — every line sent and received, plus a G-code entry field.

**Run logging** — JSONL to `logs/incubator/`, one line per sample with every
channel, flushed immediately so a crash still leaves the data.

---

## Two things worth understanding

### Marlin setpoints are whole degrees

Marlin stores temperature targets as `celsius_t` (`int16_t`), so **37.5 °C cannot
be commanded**. Current temperature is a float, so readback is fractional — only
the target is quantised. The tool:

* shows the whole chain (requested → calibration-corrected → integer commanded →
  predicted real result) so the rounding is visible rather than silently absorbed,
  and confirms the quantisation empirically at connect time;
* offers a **fine setpoint** option that alternates the integer target either side
  of your value on a slow duty ratio. The block's thermal mass is enormous
  compared with a minute, so it low-passes that into a smooth fractional hold.
  This works *because* of the physics of this particular rig; it would be a poor
  idea on a fast, low-mass load.

### Board-side waits are avoided by default

`M190`/`M109` block Marlin's command queue until the target is reached. On this
rig that could be most of an hour, during which a queued "heater off" could not
get through. So the tool sends non-blocking `M140`/`M104` and decides
"arrived / stable" on the host from the temperature stream. Board-side waits are
still available as an explicit option, cancellable with `M108`.

Emergency commands (`M112`, `M108`, `M410`) bypass the command queue entirely,
which is correct because Marlin's `EMERGENCY_PARSER` reads them straight out of
the serial buffer. Note **`M112` halts the board** — it needs a power-cycle
afterwards and is not a soft stop; use *Heater OFF* for normal use.

---

## Simulation

`--simulate` runs against a built-in fake Marlin board with a real first-order
thermal model per zone, following the same convention as the repo's
`XYStageSimulator` / `ZPStageSimulator`. It reproduces the awkward cases:
integer-quantised setpoints, bang-bang firmware, autotune failures, thermal
runaway, a detached thermistor, and EEPROM persistence.

Because the real rig's time constant is of order an hour, the simulator has a
**time-acceleration factor** (default 300×) that speeds up the clock without
changing the physics or the protocol.

`selftest.py` drives the whole production code path against it — 89 checks
covering the parser, probe, both zones converging independently, setpoint
translation, dither, autotune success and failure, bang-bang degradation, EEPROM
round-trip, fault latching, the out-of-band priority path, telemetry, and an
offscreen build of the full window.

---

## Layout

| File | Role |
|---|---|
| `run.py` | entry point |
| `marlin_gcode.py` | pure line parsing / command building, no I/O |
| `zones.py` | the two zones and their command families — the only place they differ |
| `sensors.py` | multi-channel sensor model (`SensorHub`) |
| `marlin_link.py` | serial transport, reader thread, `ok` handshake, priority writes |
| `fake_marlin.py` | simulated board + thermal models |
| `probe.py` | connect-time firmware capability detection |
| `controller.py` | framework-agnostic facade (no Qt) |
| `stability.py` | derived metrics |
| `calibration.py` | host-side sensor correction |
| `safety.py` | setpoint ceiling, fault latch, divergence check |
| `telemetry.py` | JSONL run logger |
| `device_config.py` | read-only COM-port hints |
| `gui.py` | the window |
| `_serial_helpers.py` | loads `SerialUtils.py` by path, bypassing the app package |
| `selftest.py` | end-to-end verification |

Local state (simulated EEPROM, calibration) goes to `tools/incubator/_data/`,
which is git-ignored along with `logs/incubator/*.jsonl`.

### Planned: external sensor box

Additional sensors distributed through the incubator will arrive on a separate
box with its own connection, since this board's two analog temperature inputs are
both used by the control zones. The sensor layer is already multi-source
(`SensorSource` / `SensorHub`), so that becomes one new file and the extra
channels appear in the UI table automatically — `StaticSensorSource` in
`sensors.py` is a working template, and the self-test exercises that seam.

---

## Status

Verified against the simulator only. **Everything involving real hardware still
needs bench verification** — see the ordered checklist at the end of
[`FIRMWARE_NOTES.md`](FIRMWARE_NOTES.md). In particular, whether the stock
thermal-runaway windows false-trip on this rig is the first thing to find out,
because until that is settled nothing else can be trusted.

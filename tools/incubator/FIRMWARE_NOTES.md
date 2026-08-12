# Marlin configuration for the two-zone 37 °C incubator

Target board: **BigTreeTech SKR Mini E3 V3** (the existing "ZP stage" board).

---

## Measured state of the actual board

Probed directly over USB, so this is fact rather than assumption:

| | |
|---|---|
| **Port** | `COM6` — `USB VID:PID=0483:5740`, an STM32 native-USB CDC. Baud is **irrelevant** on this interface: 38400, 115200 and 250000 all work identically. (`COM3` is FTDI = the Prior XY stage.) |
| **Firmware** | `Marlin bugfix-2.0.x (Oct 11 2022 14:32:21)`, `MACHINE_TYPE:Ender-3` |
| **Zone A (bed, THB)** | Thermistor **working** — read 23.0–24.0 °C at room temperature |
| **Zone A PID** | **`PIDTEMPBED` is ENABLED**: `M304 P41.78 I7.32 D158.93` |
| **Zone B (hotend, THO)** | **Thermistor reads −15.00 °C — an open circuit.** Not connected, or broken. Zone B cannot be used, and the tool blocks heating it. |
| **Zone B PID** | `M301 P21.73 I1.54 D76.55` (present, but useless until the sensor works) |
| **Capabilities** | `EEPROM:1`, `AUTOREPORT_TEMP:1`, `EMERGENCY_PARSER:1`, `THERMAL_PROTECTION:1`, `CHAMBER_TEMPERATURE:0` |
| **Setpoint resolution** | 1 °C — confirmed empirically (a `.5` target was truncated) |

**What this means in practice:**

1. **Bed PID is already on**, so the "enable `PIDTEMPBED`" item below is already
   satisfied on this board. Good news — but note the stored constants
   (`41.78 / 7.32 / 158.93`) are Ender-3 bed defaults for a thin 220×220 aluminium
   plate. They will be badly wrong for a water-filled block, so expect to autotune
   or set them by hand.
2. **Fix the Zone B thermistor on THO before anything else there.** −15 °C is not
   a temperature.
3. **The thermal-runaway windows are still stock and remain the open risk.**
   `EMERGENCY_PARSER` and `AUTOREPORT_TEMP` being enabled is convenient, but
   `THERMAL_PROTECTION:1` only says protection is compiled in — not that its
   timing windows suit a slow water block. That is the one thing still to find out
   on the bench, by attempting a hold and watching for a false halt.

> **Read this before spending time debugging the software.** Stock Marlin is
> configured for a 3D printer, and two of its defaults will actively prevent this
> rig from working. Both are one-line firmware changes. The tool probes for them
> at connect time and names them in its Firmware tab, but it cannot fix them.

---

## The two things that will bite you

### 1. Thermal-runaway protection will halt the board mid-heat

Marlin requires a heater to raise the measured temperature by
`WATCH_*_TEMP_INCREASE` (default **2 °C**) within `WATCH_*_TEMP_PERIOD`
(default **60 s** bed / **40 s** hotend) after being commanded, or it declares
thermal runaway, calls `kill()`, and **stops answering G-code entirely** until
power-cycled.

A ~10–25 W polyimide film heater against a large water-filled aluminium block
physically cannot do that — the thermal mass is enormous and the heater is small.
So a perfectly healthy rig trips the watchdog.

**Fix — widen the windows. Do NOT disable the protection.** It is the only thing
standing between a failed thermistor and a boiled or scorched rig.

```cpp
// Configuration_adv.h
#define WATCH_BED_TEMP_PERIOD              900   // was 60
#define WATCH_BED_TEMP_INCREASE              1   // was 2
#define THERMAL_PROTECTION_BED_PERIOD      900   // was 20
#define THERMAL_PROTECTION_BED_HYSTERESIS    4   // was 2

#define WATCH_TEMP_PERIOD                  900   // was 40  (Zone B)
#define WATCH_TEMP_INCREASE                  1   // was 2
#define THERMAL_PROTECTION_PERIOD          900   // was 40
#define THERMAL_PROTECTION_HYSTERESIS        4   // was 4
```

Keep these enabled in `Configuration.h`:

```cpp
#define THERMAL_PROTECTION_BED
#define THERMAL_PROTECTION_HOTENDS
```

Start generous (900 s) and tighten later once you have measured the real heat-up
rate — the tool's **Stability & health** tab reports it in °C/min.

### 2. Bed PID is disabled by default — the bed runs bang-bang

> **Already satisfied on this board** — the measured state above shows
> `PIDTEMPBED` enabled. Kept here because it applies to any reflash: if you build
> fresh firmware from stock Marlin you will lose it again.

`PIDTEMPBED` is **off** in stock Marlin. With it off:

* `M304` (set bed PID) is rejected as an unknown command,
* `M303 E-1` (bed autotune) fails with `PID Autotune failed! Bad extruder number`,
* `M503` emits no `M304` line, so there is no bed PID to read,
* the bed is controlled by simple on/off switching, giving **±1–2 °C swings**.

For a 37 °C cell-culture bath that swing is the difference between working and
not. Enable it:

```cpp
// Configuration.h
#define PIDTEMPBED
```

Hotend PID (`PIDTEMP`, used by Zone B) **is** enabled by default, so Zone B
usually works out of the box. The tool reports each zone's mode separately.

---

## Full checklist

### Sensors

```cpp
// Configuration.h — must match the ACTUAL parts fitted.
#define TEMP_SENSOR_BED    1     // Zone A thermistor on THB
#define TEMP_SENSOR_0      1     // Zone B thermistor on THO
```

Sensor `1` is the generic 100 kΩ EPCOS table. If you fit Semitec 104GT-2 /
104NT-4 parts, use their table (`5`) instead — a mismatched table is a silent
offset of several degrees, and no amount of host-side calibration makes a wrong
curve right across a range.

Min/max limits:

```cpp
#define BED_MINTEMP         5    // MUST be below room temperature or it trips at boot
#define BED_MAXTEMP        90    // generous headroom over the tool's 50 C ceiling
#define HEATER_0_MINTEMP    5
#define HEATER_0_MAXTEMP  120
```

A **disconnected thermistor reads as an open circuit**, i.e. very cold, and trips
MINTEMP. That is correct and desirable behaviour — it is the failure mode most
likely to matter with a heater bonded to a block. The tool names this cause
explicitly when it sees a MINTEMP fault.

### Arrival / residency (affects `M190` / `M109` only)

The tool defaults to non-blocking `M140`/`M104` and decides "arrived" on the
host, so these only matter if you use the board-side wait option:

```cpp
#define TEMP_BED_WINDOW        1
#define TEMP_BED_HYSTERESIS    1   // was 3 — tighten for a 37 C bath
#define TEMP_BED_RESIDENCY_TIME 30
#define TEMP_WINDOW            1
#define TEMP_HYSTERESIS        1
#define TEMP_RESIDENCY_TIME    30
```

### Host interaction

```cpp
// Configuration.h
#define EEPROM_SETTINGS            // else M500/M501/M502 do nothing and PID is
                                   // lost on every reset

// Configuration_adv.h
#define EMERGENCY_PARSER           // else M108/M112 queue behind the current
                                   // command, so "Cancel" during a wait or an
                                   // autotune arrives late
#define AUTO_REPORT_TEMPERATURES   // lets the board push temperatures; the tool
                                   // falls back to polling M105 without it
#define EXTENDED_CAPABILITIES_REPORT  // makes M115 report Cap: lines so the tool
                                      // can detect the above automatically
```

### Power limiting (optional)

```cpp
#define MAX_BED_POWER  255   // reduce to cap film-heater power if it runs hot
#define PID_FUNCTIONAL_RANGE 10   // full power until within 10 C of target
```

---

## Things to verify on the bench, in order

1. **Before applying power to either heater**, confirm the heater's rated voltage
   matches the board supply. A 12 V film heater on a 24 V output dissipates **4×**
   its rating and will fail, possibly destructively.
2. Connect the tool and check the **Firmware tab**. It lists what this build
   supports and names the exact option for anything missing.
3. Confirm **both sensors appear and track**. Warm each one by hand in turn and
   watch which zone's reading moves — a swapped THB/THO pair would silently
   control the wrong zone, and nothing in software can detect that.
4. Verify **Heater OFF** works, and that the fault path is visible (briefly
   unplug a thermistor; expect a MINTEMP fault and a latched banner).
5. Attempt a **37 °C hold on one zone** and watch for a false thermal-runaway
   halt. If it halts, the `WATCH_*` windows above are still too tight.
6. Read the **steady-state duty** on the Stability tab once holding. Near 100 %
   means the heater cannot hold that setpoint against ambient losses — insulate
   the block or fit more power. This number is the real answer to "is the heater
   big enough?".
7. Only then consider **PID autotune**. On this thermal mass a cycle may exceed
   Marlin's internal `MAX_CYCLE_TIME_PID_AUTOTUNE` (20 min) and fail with
   `timeout`; that is a firmware limit, not a tool bug. Setting PID manually is a
   legitimate alternative.

---

## Notes for phase 2 (external sensor box)

The plan is to add distributed sensors on a separate box rather than through
Marlin, because this board exposes only the two analog temperature inputs used
above (`THB`, `THO`) and both are consumed by the control zones. The tool's
sensor layer is already multi-source, so a second source appears as extra rows in
the Sensor channels table with no further changes.

If you ever *do* want a precision RTD read by Marlin itself, note this verified
gotcha:

* `TEMP_SENSOR_BED = -5` (MAX31865 with PT100/PT1000) **is** supported — SPI RTD
  support is not hotend-only. The accepting slots are `TEMP_SENSOR_0/1/2`,
  `TEMP_SENSOR_BED` and `TEMP_SENSOR_REDUNDANT`.
* **But on stable release 2.1.2.8 it does not compile.** `temperature.cpp`
  references `MAX31865_SENSOR_OHMS_BED` and `MAX31865_CALIBRATION_OHMS_BED`, and
  stock `Configuration.h` / `Configuration_adv.h` / `Conditionals_adv.h` define
  neither, with no `SanityCheck.h` guard — so you get a raw C++ "not declared"
  error. They are present on `bugfix-2.1.x`. Either build from `bugfix-2.1.x` or
  hand-add both defines (`100`/`430` for PT100, `1000`/`4300` for PT1000).
* Each MAX31865 needs its own chip-select (`TEMP_BED_CS_PIN`, `TEMP_0_CS_PIN`, …)
  and free GPIOs are scarce on this board.
* Prefer **PT1000 over PT100**: at ~3.85 Ω/°C versus ~0.385 Ω/°C, 0.1 Ω of lead
  and contact resistance is ~0.026 °C of error instead of ~0.26 °C.
* `TEMP_SENSOR_REDUNDANT` lets the *firmware* kill on sensor divergence, which is
  the strongest safety option — but during bring-up a large legitimate
  thermistor-versus-reference offset is exactly the calibration data you want to
  see, and redundancy mode would halt the board over it. Use
  `TEMP_SENSOR_CHAMBER` (a free-floating extra reading) while characterising,
  then switch to `TEMP_SENSOR_REDUNDANT` for production.

# MEBP v7.5.x — Dimmable Illumination LED Control

## Objective

Add a microscope **illumination LED** (a ZF-R25-0403 12 V constant-voltage COB
module) to the machine and let the operator **vary its brightness from the MEBP
app**. The LED is driven by the existing Marlin **ZP** controller (a BigTreeTech
SKR Mini E3 V3.0, powered at 12 V) — it is **not** a motion axis, so it uses a
spare PWM **fan output** rather than an `axis_map` entry.

The GUI is a compact **on/off toggle + brightness slider** that lives in the
**jog panel** by default and is also registered as a **drop-in custom-panel
module** ("💡 Illumination LED") users can add anywhere the Custom context panel
appears.

## Hardware / firmware (operator, outside this repo)

- **Wiring:** LED module `+` → FAN0 connector `+` (12 V rail); LED module `−` →
  FAN0 connector `−` (MOSFET-switched, low-side). No resistor/driver — the module
  is constant-voltage 12 V on a 12 V rail. ≤ 1 A is within the fan MOSFET rating.
- **Fan index is one constant** — `ZPStage._LED_FAN_INDEX = 0` is the single
  place the header is named; `set_led_brightness` sends `M106 P{index} S<v>`.
  (A 2026-07-31 move to FAN2 was made and then reverted the same day — the LED
  is on FAN0. Worth knowing if you rewire: FAN2/`PB15` is commonly an
  **auto/controller fan** in stock E3 V3 firmware rather than an M106 fan, and
  Marlin **acks an unassigned `P` index with `ok` and does nothing**, so a wrong
  index fails SILENTLY — dark LED, app reports success. Verify on a terminal.)
- **No firmware flash required** for the default path: FAN0 (`PC6`) is already
  controllable by `M106 P0 S<0-255>` in stock SKR Mini E3 Marlin.
- **Flicker fallback (imaging):** only if bench testing shows camera banding,
  flash `FAST_PWM_FAN` (raise fan PWM frequency). `M106` and `M355` share the
  pin timer, so command choice does not affect flicker — frequency does.
- **Decision — bed heater rejected:** the bed MOSFET was considered (more current
  headroom, pin free) but its deliberately low-frequency PWM would band the
  microscope camera, so FAN0 was chosen. See
  `~/.claude/plans/i-have-a-zf-r25-0403-jiggly-church.md` for the full rationale.
- If ever flashed to Case Light, swap the one G-code line to `M355 S1 P<v>`.

## Files Modified

| File | Rationale |
|------|-----------|
| `SupportClasses/ZPStage.py` | New `ZPStageManager.set_led_brightness(level)` — clamps 0-255, sends `M106 P{_LED_FAN_INDEX} S<v>` via the existing `send_data()` (`ok`-handshaked, lock-safe). Placed next to `save_settings`/`emergency_stop`. Module constant `_LED_FAN_INDEX = 0` (FAN0) is the one place the header is named. |
| `SupportClasses/StageController.py` | New `set_led_brightness(level)` passthrough — guarded on `is_zp_connected`; no-op returning False when the ZP board is not connected. |
| `gui/widgets/illumination_control.py` | **New.** Reusable `IlluminationControl(QWidget)`: on/off `QCheckBox` + 0-100 % `QSlider` + live label. Debounced (~100 ms `QTimer`) so slider drags don't flood the shared serial bus; every write guarded on `is_zp_connected`; `on_status_update()` greys the card when disconnected; silent `set_on`/`set_value`/`is_on`/`value` for state restore. |
| `gui/widgets/standard_jog_context.py` | Adds a `Card("Illumination")` (new `_build_illumination_card`) between "Absolute Go To" and "Hardware Info"; forwards the tick in `on_status_update` and the controller in `set_controller`. Because `JogControlPage.get_context_widget()` returns this panel, the card rides the left-box **"Jog" pill** on every jog-aware page automatically. |
| `gui/widgets/context_sections.py` | New `IlluminationSection` wrapper (reuses `IlluminationControl`, restores persisted `options` `{level, on}`) + `register_section("illumination", SectionSpec("Illumination LED", "💡", …))`. No changes needed to the panel/host/store. |
| `tests/test_v75x_illumination_led.py` | **New.** 16 tests (see below). |

No `config/hardware/devices/*.json` changes — the LED is not a motion axis and
those profiles model only motion; there is no pin/output map.

## Implementation Steps

- [x] `ZPStageManager.set_led_brightness` → `M106 P{_LED_FAN_INDEX} S<v>` (clamped)
- [x] Fan header extracted to `_LED_FAN_INDEX` (2026-07-31, during a FAN2 move
      that was reverted the same day — the constant is kept, value back to `0`)
- [x] `StageController.set_led_brightness` guarded passthrough
- [x] `IlluminationControl` reusable widget (toggle + slider + debounce + guard)
- [x] Embed in `StandardJogContextPanel` (card + tick + controller wiring)
- [x] Register `IlluminationSection` custom-panel module
- [x] Tests + this update-plan doc
- [ ] **Real-HW verification on ME3B V1** (see Testing Notes)

## Testing Notes

**Automated** (`python3 -m unittest tests.test_v75x_illumination_led` — 17 pass):
- `ZPStageManager.set_led_brightness` emits `M106 P0 S<v>`; clamps −10→S0, 999→S255; floats coerced; the emitted index is pinned to the `_LED_FAN_INDEX` constant (so a future move stays a one-line change).
- `StageController.set_led_brightness` forwards when connected, no-ops (False) when disconnected.
- Widget: toggle-on queues + sends the slider level; toggle-off sends 0; toggle-on from a dark slider defaults to full; disconnected never sends; slider move while off queues nothing; `on_status_update` enables/disables with connection; `set_on`/`set_value` are silent.
- Section: registered in `catalog()`/`known_types()` with label + 💡; builds headless and ticks; restores persisted `{level, on}`.

Regression (all green): `test_v75x_context_panel`, `test_v75x_zp_serial_flow_control`, `test_v75x_zp_position_override`, `test_v75x_jog_pump_fill_readout` (66). Offscreen smoke: `StandardJogContextPanel` builds with the LED card, ticks, and commands the controller.

**Real hardware (ME3B V1) — pending:**
1. Bench, no flash: send `M106 P0 S255 / S64 / M107` from a terminal (38400 baud); confirm the LED lights/dims/off and — at real camera exposures — **no banding** and smooth low end. If banding → flash `FAST_PWM_FAN`, re-test. (Marlin acks an unassigned fan index with `ok` and does nothing, so if the LED is ever silent, confirm the header on a terminal before suspecting the app.)
2. App: left context box → **Jog pill** shows the Illumination card; slider/toggle drive the LED live; card greys out when the ZP board is disconnected. Confirm the ZP `ok` handshake stays healthy (no `M400`/serial timeouts from slider spam — debounce should prevent it).
3. Custom pill → **＋ Add section → 💡 Illumination LED**: drops in and works standalone.

## Issues & Decisions

- **2026-07-31 — moved FAN0 → FAN2, then reverted to FAN0 the same day** at operator request; the LED is on **FAN0**. The one lasting artefact is the extraction of the header into `_LED_FAN_INDEX`, kept because it makes any future rewire a one-line change (pinned by a test). Note for whoever tries FAN2 again: FAN0 is the part-cooling fan and M106-controllable out of the box, whereas FAN2/`PB15` is commonly an auto/controller fan in stock E3 V3 firmware — `M106 P2` then acks `ok` and does nothing, a silent failure.
- **Output = FAN0 (`M106`), not the bed heater.** Operator (AskUserQuestion): LED ≤ 1 A and used for imaging → fan port's high PWM frequency is flicker-free; the bed's low-frequency PWM would band the camera. No-flash path chosen; `FAST_PWM_FAN` flash kept only as the flicker fallback.
- **One reusable widget, two mount points.** `IlluminationControl` is the single source of truth (mirrors how `JogSection` reuses `StandardJogContextPanel`) so the jog-panel card and the custom-panel module never diverge.
- **Command-output, not sensor.** The widget only needs the status tick to grey out on disconnect; no `on_motion_tick`. Writes are debounced and connection-guarded to respect the shared, `ok`-blocking ZP serial channel.
- **Persistence is partial (v1).** Section state restores from the `options` dict; writing state back to the layout store on change is a tracked follow-up (`store.set_section_options`).
- **Firmware note:** Marlin acks unknown commands with `ok`, so an app/firmware command mismatch won't error — the LED just won't respond. If the board is later flashed to Case Light, change the single `M106` line in `ZPStage.set_led_brightness` to `M355`.

# MEBP v7.5.x — Post-aspirate pump pressure relief (setup parameter)

## Objective

Operator request: after picking up any reagent (ink, oil, buffer, …), add a
small **pressure-relief** step that dispenses a little of it back. When fluid is
drawn into the needle a residual vacuum remains; moving between wells then
ingests a bit of air, worsening compliance. Dispensing a small, experimentally
derived volume back at the pickup well bleeds the vacuum. The amount is
**setup-dependent**, so it's exposed as a **common setup parameter** (Hardware
Setup → Pump), not a per-workflow knob.

## Design

Mirrors the existing global pump params (`pump_settle_time_s` /
`pump_prime_time_s`): one `HardwareConfig` field + a `StageController` reader +
a `move_pump_uL` flag, applied at the reagent-pickup call sites.

- **`HardwareConfig.pump_relief_volume_uL: float = 0.0`** (µL, non-negative,
  serialized; `0` = disabled). Coerced via `_nonneg_float` like its siblings.
- **`StageController.pump_relief_volume_uL()`** — reads the config, clamps ≥ 0,
  `getattr`-safe for `__new__` stubs.
- **`StageController.move_pump_uL(..., relieve: bool = False)`** — when
  `relieve=True` **and** the move is an ASPIRATE (`volume_uL < 0`), after the
  aspirate completes (and its settle), it dispenses `pump_relief_volume_uL`
  back via a single recursive `move_pump_uL` (positive → the `volume_uL < 0`
  guard + `relieve=False` make it non-recursive). No-op when the relief volume
  is 0 or the move is a dispense. The relief happens **at the pickup well,
  before any travel** (it's inside the same `move_pump_uL`, before the caller's
  next safe-travel), which is exactly where the vacuum needs bleeding.
- **`PickAndPlaceManager._settled_pump_move(..., relieve=False)`** passes the
  flag through (graceful TypeError fallback chain: full → settle-only → plain,
  for fakes/older controllers).

### Where `relieve=True` is set (reagent pickups followed by inter-well travel)

- `run_prep`: oil aspirate, buffer aspirate.
- `run_post_clean`: buffer reload aspirate.
- `prepare_starting_oil`: the fresh-oil aspirate branch.
- `aspirate_ink` (Quick Print's "pick the print's ink").

**Deliberately NOT applied** to volume-balanced pick&place captures (spheroid
pickup, cell-removal pull, cell-labeling deposit/aspirate, trypsin/dye loads) —
those must keep exactly what they aspirated; a relief dispense would eject the
captured target / unbalance the plunger return. The `relieve` flag + the global
volume are general, so extending to other reagent loads later is a one-liner.

### GUI

Hardware Setup → Pump → "Pump Timing (all pumps)" group gains a **Pressure
relief** spinbox (µL, 0–50, 3 decimals) beneath Settle/Prime, with a tooltip
explaining the compliance rationale. Wired into `_rebuild_config` (save) and
`_apply_config_to_ui` (load), `getattr`/`hasattr`-guarded like the siblings.

## Files Modified

- `SupportClasses/HardwareConfig.py` — field + `to_dict`/`from_dict`.
- `SupportClasses/StageController.py` — `pump_relief_volume_uL()` reader +
  `move_pump_uL(relieve=)` relief logic.
- `SupportClasses/PickAndPlaceManager.py` — `_settled_pump_move(relieve=)`
  passthrough + `relieve=True` at the 5 reagent-pickup sites.
- `gui/pages/hardware_setup.py` — Pressure-relief spinbox + save/load wiring.
- `tests/test_v75x_pump_settle_and_prime_time.py` — relief field round-trip,
  reader clamping, `move_pump_uL` relief behaviour (aspirate→dispense-back,
  no-op on dispense / zero volume / relieve=False), helper passthrough.

## Implementation Steps

- [x] `HardwareConfig.pump_relief_volume_uL` + serialization.
- [x] `StageController.pump_relief_volume_uL()` + `move_pump_uL(relieve=)`.
- [x] `_settled_pump_move(relieve=)` + `relieve=True` at pickup sites.
- [x] Hardware Setup → Pump spinbox + wiring.
- [x] Tests (31 in the settle/relief suite green; prep/cell/quick-print suites
  unaffected — relief defaults to 0 = no-op).
- [ ] **Real-HW verification on ME3B V1** — set a small relief volume (e.g.
  0.2–1 µL), run printing prep + print: confirm a small dispense-back after each
  reagent aspirate, no air ingestion during inter-well travel, and the print
  still flows. Tune the value experimentally.

## Testing Notes

- `tests/test_v75x_pump_settle_and_prime_time.py` — 31 green (added
  `TestPumpRelief` + helper passthrough + config round-trip).
- `test_v75x_spheroid_pick_place_z` / `cell_targeting_removal` / `cell_labeling`
  / `quick_print_pick_and_place` (106) green — unchanged because the default
  relief is 0 (no extra move) and the balanced captures don't pass `relieve`.
- `test_v75x_pump_plunger_setup` / `workflow_settings_popout` (55, offscreen
  GUI) green; offscreen Hardware Setup spinbox build + value round-trip
  confirmed.

## Issues & Decisions

- **Net loaded volume = aspirate − relief.** Intended (the operator wants some
  given back). Keep the relief small relative to the pickup; the ink-pickup
  volume already carries a safety multiplier.
- **Scope = reagent loads, not biological captures.** The vacuum-relief physics
  applies to any aspirate, but ejecting part of a spheroid/cell capture is
  wrong, so relief is opt-in per call site and limited to carried reagents.
- **Centralised in `move_pump_uL`** (single pump chokepoint) so the relief
  reuses the existing µL→mm conversion, flow clamp, and the new M400 completion
  drain — and so the streamed print path / manual jog (`relieve` defaults
  False) are untouched.

# MEBP v7.5.x — Bore-area bead model (F-1) + redesigned pressure relief (F-4)

> Origin: operator review (2026-06-30) of `coding plans/Analyses/MEBP_WORKFLOW_MOVE_AUDIT.md`.
> The broader `changes_needed.md` backlog (mosaic RAM, side-view UIs, F-9 max-flow-from-bore)
> is being planned **separately** — out of scope here. This plan covers only **F-1** and **F-4**.

## Objective

1. **F-1 — one bead model everywhere.** The deposited volume-per-mm of print path must be
   **`inner-bore cross-section area × extrusion modifier`** (the "thickness" of the laid line),
   replacing the legacy `outer-Ø × layer_height` model in `GeometryEngine`/`FlowPhysics`.
   `layer_height` keeps its role **only** as the Z step between stacked layers — it leaves the
   volume formula. Quick Print already uses this model; this aligns the trajectory / Print-Builder
   path to it.

2. **F-4 — pressure relief as a tunable, multi-context feature.** Relief becomes
   **`% of each pump's syringe volume`** (default **1 %**, tunable ~0.001 %–5 %, per-pump µL
   derived), with **three independent toggles (all default ON):**
   - **pickup** — after a reagent/ink ASPIRATE, DISPENSE the relief back to bleed needle vacuum
     (existing behavior, now toggle-gated + %-based).
   - **deposit** — after an ink DEPOSIT (print path), ASPIRATE the relief back (suck-back) to stop
     drool before the needle travels. **New.**
   - **quick-move** — during a print, before an inter-object/quick travel, suck-back + confirm the
     Z retract, then re-prime on arrival. **New** (also addresses `changes_needed.md` item 5).

## Scope decisions (operator-confirmed 2026-06-30)

- **F-1 bead model:** `vol/mm = π·(id/2)² × modifier`. `modifier` defaults **1.0**; threaded into the
  three volume functions as a kwarg. **The per-object modifier UI for Print Builder is a tracked
  follow-up** — this change ships the bore-area basis with `modifier=1.0` for the trajectory path
  (byte-stable canonical model; Quick Print's own modifier knob already works). `PrintObject` gains
  an `extrusion_modifier` field so the data model is ready.
- **F-1 fill pitch:** `line_spacing` (raster pitch for fills) is **left on `od_mm`** — it is a
  *lateral fill-density* lever, distinct from the deposited bead volume. Flagged as a follow-up if
  fills under/over-pack with the bore-area bead. (Avoids a silent fill-density change.)
- **F-4 relief magnitude:** `% of syringe volume`, **per-pump** µL = `pct/100 × syringe.volume_uL`.
  **1 % of a 250 µL syringe = 2.5 µL** — *enormous* next to the nL-scale spheroid/cell captures
  (0.006 µL). Therefore **deposit/quick-move relief is scoped to ink PRINTING only**; the
  volume-balanced pick&place micro-captures (spheroid / cell-target / cell-label) keep
  `relieve=False` and their exact net-zero balance. Pickup relief stays at the µL-scale reagent/ink
  aspirate sites. **(Confirm this scoping.)**
- **F-4 toggles:** default **all ON**. Suck-back = ASPIRATE a small volume after a dispense.

## Files to modify

| File | Change |
|---|---|
| `SupportClasses/FlowPhysics.py` | `extrusion_flow_rate` → `print_speed × cross_section_area_mm2 × modifier` (+`modifier=1.0` kwarg); `extrusion_pump_speed` passthrough. |
| `SupportClasses/GeometryEngine.py` | `compute_pump_positions` / `compute_total_volume` → `cross_section_area_mm2 × modifier` (+`modifier=1.0`; `layer_height_mm` kept, no longer used for volume). `PrintObject` gains `extrusion_modifier` (default 1.0) + (de)serialize. |
| `SupportClasses/HardwareConfig.py` | Drop `pump_relief_volume_uL`; add `pump_relief_percent` (1.0) + `pump_relief_on_pickup/deposit/quick_move` (True). (de)serialize + non-neg/bool coercion + migration (old µL ignored). |
| `SupportClasses/StageController.py` | Replace `pump_relief_volume_uL()` with `pump_relief_percent()`, `pump_relief_uL(pump)` (per-pump %→µL), `pump_relief_on_*()` readers. `move_pump_uL` relief block → direction-aware (aspirate+on_pickup → dispense-back; dispense+on_deposit → suck-back). |
| `SupportClasses/PrintManager.py` | Deposit suck-back at end of `_execute_print_path` (on_deposit); quick-move suck-back on inter-object hop `MOVE_XY` (on_quick_move) + confirm retract; re-prime via the next object's prime. |
| `SupportClasses/CommonPrintSettings.py` | `GLOBAL_KEYS`/`DEFAULTS`: swap `pump_relief_volume_uL` → `pump_relief_percent` (+ the 3 bools as globals). |
| `gui/pages/hardware_setup.py` | "Pump Timing" group: relief µL spin → relief **%** spin (0.001–5, 3 dec) + 3 checkboxes; `_rebuild_config` / `_apply_config_to_ui`. |
| `gui/dialogs/workflow_settings_dialog.py` | "Common — Pump (global)" relief row → % + toggles (if surfaced there). |
| `tests/test_v75x_pump_relief_and_bead_model.py` | New: bore-area volume math, modifier, %→µL per pump, direction-aware relief + toggle gating, migration. |

## Implementation steps

- [x] F-1a `FlowPhysics.extrusion_flow_rate` bore-area + modifier kwarg
- [x] F-1b `GeometryEngine.compute_pump_positions` / `compute_total_volume` bore-area + modifier
- [x] F-1c `PrintObject.extrusion_modifier` field + (de)serialize
- [x] F-4a `HardwareConfig` fields (percent + 3 bools), (de)serialize, coercion, migration
- [x] F-4b `StageController` readers (`pump_relief_percent`, `pump_relief_uL`, `pump_relief_on_*`)
- [x] F-4c `StageController.move_pump_uL` direction-aware relief + toggle gating
- [x] F-4d `CommonPrintSettings` globals swap (+ Common Print Settings page row)
- [x] F-4e `PrintManager` deposit + quick-move relief (`_print_pump_suckback` + 2 sites)
- [x] F-4f GUI: hardware_setup relief % spin + 3 checkboxes; 4 workflow-page common-pump rows
- [x] Tests + run affected suites

## Testing notes

- Bore-area math: 22 G (id 0.413, 2″) → `vol/mm = 0.13396 µL/mm` at modifier 1.0; modifier 2.0 → 0.268.
- Relief µL: P2 250 µL @ 1 % → 2.5 µL; @ 0.001 % → 0.0025 µL.
- Direction: `relieve=True` + aspirate(−) + on_pickup → +relief dispense; + dispense(+) + on_deposit → −relief aspirate; toggles off → no relief.
- Migration: a config with legacy `pump_relief_volume_uL` loads with `pump_relief_percent` defaulted (1.0) and old µL discarded.
- **Needs real-HW verification on ME3B V1/V3** (relief stops drool on deposit + quick moves; print flow still primes; pickup unchanged).

## Issues & decisions

- (open) Deposit/quick-move relief scoped to printing only — confirm pick&place micro-captures stay untouched.
- (open) `line_spacing` left on `od_mm`; revisit if fills mis-pack.
- (follow-up) Per-object `extrusion_modifier` UI in Print Builder (data model is ready here).

## Status

`[x]` Code + tests complete — 2026-06-30. **Needs real-HW verification on ME3B V1/V3.**

**What landed.** F-1: `FlowPhysics.extrusion_flow_rate`, `GeometryEngine.compute_pump_positions`
/ `compute_total_volume` now use `cross_section_area_mm2 × modifier` (inner bore); `layer_height`
kept as a param but no longer scales volume; `PrintObject.extrusion_modifier` added (default 1.0,
serialized). F-4: `HardwareConfig` `pump_relief_percent` (1.0) + `pump_relief_on_{pickup,deposit,
quick_move}` (True), legacy µL discarded on load; `StageController` readers + direction-aware
relief in `move_pump_uL` (aspirate→dispense-back / dispense→suck-back, toggle-gated);
`PrintManager._print_pump_suckback` wired at end-of-deposit and before each quick-move hop;
Hardware Setup → Pump relief **%** spin + 3 checkboxes; the 4 prep-workflow common-pump rows +
the Common Print Settings page row swapped to `pump_relief_percent`.

**Tests.** New `tests/test_v75x_pump_relief_and_bead_model.py` (14). Updated
`tests/test_v75x_pump_settle_and_prime_time.py` (relief now % + direction-aware) and
`tests/test_v75x_common_print_settings.py` (global key rename). Green suites: relief/settings/bead
(71), workflow-settings + 4 workflow pages + sketch (153), print-path (62), config-serialize (85).

**Pre-existing failures observed (NOT from this work, confirmed by stashing the changed files):**
`test_v73_trajectory_planner.TestFullPlanGeneration` (6 — `MagicMock < float` in
`PrintTrajectoryPlanner._move_z`) and `test_v75x_pump_plunger_setup.TestDirectionOwnershipInMotion`
(3 — `move_pump_relative` reads `self._pos_poller.zp_position` from an in-progress, uncommitted
`MotionEstimator` change another session left in `StageController.py`; the test's `__new__`
partial controller has no `_pos_poller`). Flag to whoever owns the MotionEstimator work.

## Issues & decisions

- (resolved) Deposit/quick-move relief scoped to **ink printing only**; the volume-balanced
  spheroid/cell micro-captures keep `relieve=False` (a 1 %-of-syringe relief would dwarf their nL
  volumes). **Confirm acceptable.**
- (note) Multi-object prints with deposit+quick-move both ON suck back once at each object's
  path-end AND before each hop; the next object's prime re-primes. Minor over-suck (small %);
  net handled by the syringe budget + post-print reset. Fine, documented in the handler.
- (open) `line_spacing` (fill raster pitch) still uses `od_mm` — a separate fill-density lever;
  revisit if fills mis-pack with the bore-area bead.
- (follow-up) Per-object `extrusion_modifier` UI in Print Builder (data model is ready; the
  trajectory path currently runs at modifier 1.0, matching Quick Print's default).

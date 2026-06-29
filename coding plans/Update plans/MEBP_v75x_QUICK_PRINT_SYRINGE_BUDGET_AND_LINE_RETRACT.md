# MEBP v7.5.x — Quick Print syringe budget pre-flight, per-line retract, buffer-needle count, reset-to-initial cleanup

## Objective

Four operator-requested changes, all centred on the single oil-backed syringe used
for a Quick Print run:

1. **Pre-print syringe-budget pre-flight (Quick Print only).** Before a print
   starts, compute **every** pump move the run will make (needle prep
   aspirate/dispense → ink pickup → print prime + path dispense + retract → post-print
   cleanup) and simulate the plunger fill against the calibrated syringe envelope
   `[empty = 0, full = capacity]`. If the run cannot fit no matter the starting
   fill (`span > capacity`), **tell the operator it won't work** and block. If it
   *can* fit with a different starting fill, **offer to waste (or aspirate) that
   amount of oil first** to reach a feasible start, and proceed on confirm.

2. **"Buffer needles" on all prep workflows, default 1.** Add the buffer-pickup
   count to Quick Print's needle prep (it had none) and change the default from
   **4 → 1** everywhere (executor + Spheroid / Cell-Targeting / Cell-Labeling).

3. **Quick Print per-line retract + fast moves.** Expose a configurable
   **retract height after each print line** (the lift between separate
   strokes/sub-paths) plus a **quick-move Z speed** (the lift up + lower down)
   and a **quick-move XY speed** (the inter-line travel).

4. **Post-print cleanup → "reset to initial condition".** Replace Quick Print's
   "Waste ×needle / Oil refill ×needle" inputs with a single
   **"Reset syringe to initial condition"** choice: waste the unprinted
   ink + buffer (computed live) + a small oil flush margin, then top the oil back
   up so the plunger returns to its pre-run (initial) position.

## Decisions (operator-confirmed)

- **Buffer default = 1 everywhere** (incl. the cell workflows that previously
  defaulted to 4).
- **"Print line" = between strokes/sub-paths** (the existing per-segment hop in
  `build_well_plate_job`); a continuous fill (meander/circle) prints as one pass.
- **Syringe-budget pre-flight = Quick Print only** (the only path where the full
  prep + ink + print + cleanup pump sequence is known).

## Polarity fix surfaced during design (Issues & Decisions)

`run_print_cleanup`'s oil reset passed `reset = baseline − current` (both read via
`get_pump_position_uL`) straight to `move_pump_uL`. But `move_pump_uL` applies
`pump_dir_sign` internally while `get_pump_position_uL` does **not**, so a
`move_pump_uL` of `V µL` changes `get_pump_position_uL` by `pump_dir_sign × V`.
Returning the plunger to `baseline` therefore requires
`V = pump_dir_sign × (baseline − current)` — the existing code omitted the
`pump_dir_sign` factor and is only correct when `pump_dir_sign == +1`
(on a pump calibrated to `pump_dir_sign == −1` it drives the plunger the **wrong
way**, doubling the error). Fix: new polarity-correct helper
`StageController.pump_volume_to_reach_uL(pump, target_position_uL)` (and
`move_pump_to_position_uL`), used by the new reset-to-initial path; the legacy
`reset = baseline − current` is also routed through the `pump_dir_sign` factor
(a no-op when `+1`, a correction when `−1`). Polarity-safe on ME3B V1.

## Files Modified

| File | Change |
|------|--------|
| `SupportClasses/StageController.py` | New module fn `compute_pump_budget(moves_uL, start_fill_uL, capacity_uL, *, tol_uL)` (pure) + `StageController.simulate_pump_budget(pump, moves_uL, *, start_fill_uL)`; `pump_volume_to_reach_uL` / `move_pump_to_position_uL` (polarity-correct plunger-to-position). |
| `SupportClasses/PickAndPlaceManager.py` | `buffer_needles` default `4.0 → 1.0`. New cleanup fields `cleanup_reset_to_initial: bool`, `cleanup_oil_margin_uL`. `run_print_cleanup` reset-to-initial branch (waste = live leftover + margin; oil reset via `pump_volume_to_reach_uL`). New executor pre-step `prepare_starting_oil(volume_uL, *, dispense_to_waste)` for the Feature-1 remedy. |
| `SupportClasses/PrintManager.py` | `PrintSettings`: `line_move_z_speed_mm_s`, `line_move_xy_speed_mm_s` (0 = use default). `build_well_plate_job` tags the inter-segment hop `MOVE_XY` with `xy_speed_mm_s` + `retract_feedrate_mm_min` and the following lower `MOVE_Z` with `feedrate_mm_min` from the line speeds. `MOVE_XY` / `MOVE_Z` handlers + `_retract_for_travel` honor the per-command overrides. |
| `gui/pages/workflows/quick_print_workflow.py` | Buffer-needles spin (default 1) wired into the prep executor. Line-retract height + line Z/XY speed spins → `PrintSettings`. Cleanup section → "Reset syringe to initial condition" + "Oil flush margin (× needle)". `_on_print`: assemble the full pump-move list, call `simulate_pump_budget`, block on infeasible, offer waste/add-oil on shiftable; thread the chosen starting-oil adjust into the preamble worker. |
| `gui/pages/workflows/spheroid_pickup_workflow.py` | Buffer-needles spin + `sec.add` default `4.0 → 1.0`. |
| `gui/pages/workflows/cell_targeting_workflow.py` | Buffer-needles spin + `sec.add` default `4.0 → 1.0`. |
| `gui/pages/workflows/cell_labeling_workflow.py` | Buffer-needles spin + `sec.add` default `4.0 → 1.0`. |
| `tests/test_v75x_quick_print_syringe_budget.py` | New — budget math, polarity reset, line-retract plan, reset-to-initial cleanup, buffer default. |

## Implementation Steps

- [ ] Feature 2 — buffer default 1 (executor + 3 cell pages) + Quick Print buffer spin wired.
- [ ] Feature 3 — `PrintSettings` line speeds; `build_well_plate_job` hop tagging; handler overrides; Quick Print UI + `_build_settings`.
- [ ] Feature 4 — polarity-correct `pump_volume_to_reach_uL`/`move_pump_to_position_uL`; `run_print_cleanup` reset-to-initial; Quick Print cleanup UI + ctx.
- [ ] Feature 1 — `compute_pump_budget` + `simulate_pump_budget`; `prepare_starting_oil`; Quick Print `_on_print` pre-flight + dialogs + worker wiring.
- [ ] Tests for all four.
- [ ] Adversarial review + run affected suites.

## Budget model (Feature 1)

Plunger fill µL: `0 = empty (fully dispensed)`, `capacity = full (fully aspirated)`.
Move sign (as passed to `move_pump_uL`): `+ = dispense` (lowers fill),
`− = aspirate` (raises fill). The shiftable budget covers **prep + ink + print**
(the aspiration peak risk); the post-print cleanup returns the plunger to baseline
and only dips by the oil-flush margin, so its trough (`baseline − margin`) is
checked separately.

- Simulate `fill += −move` over the ordered moves; track `peak`, `trough`.
- `ok` iff `trough ≥ 0` and `peak ≤ capacity`.
- Else `span = peak − trough`: if `span > capacity` → **infeasible** ("won't
  work"; reduce print size / ink pickup / prep volumes). Otherwise a feasible
  start exists — shift the whole trajectory by `Δ` (span is shift-invariant):
  overflow (`peak > capacity`) → **waste oil** `Δ = −(peak − capacity)`;
  underflow (`trough < 0`) → **aspirate oil** `Δ = −trough`.

## Testing Notes

- Pure `compute_pump_budget`: feasible, overflow→waste, underflow→add, span>cap→infeasible, shift-invariance.
- `pump_volume_to_reach_uL` returns the `pump_dir_sign`-correct signed volume for both polarities.
- `run_print_cleanup` reset-to-initial: waste = leftover + margin; oil reset returns to baseline (both polarities).
- `build_well_plate_job` tags the 2nd+ object hop `MOVE_XY`/`MOVE_Z` with line speeds; single-object plan unchanged.
- Buffer default is 1 on the executor and all four pages.
- Offscreen Quick Print page build smoke (settings dialog with the new sections).

**Needs real-HW verification on ME3B V1** (run a multi-object Quick Print: per-line
lift at the set height/speeds; an oversized run is blocked or offered an oil-waste;
reset-to-initial returns the plunger to its pre-run position).

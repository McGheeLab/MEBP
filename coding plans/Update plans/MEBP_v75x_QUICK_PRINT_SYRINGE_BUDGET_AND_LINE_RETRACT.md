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
   and a **quick-move XY speed** (the inter-line travel). The two speeds are
   entered as a **% of each stage's calibrated max** (Z = per-axis Z max
   feedrate ÷60; XY = `_xy_max_mm_s`), resolved to mm/s in `_build_settings`;
   `PrintSettings.line_move_*_speed_mm_s` stays in mm/s so the backend is
   unchanged. Defaults 100 %.

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

- [x] Feature 2 — buffer default 1 (executor + 3 cell pages) + Quick Print buffer spin wired.
- [x] Feature 3 — `PrintSettings` line speeds; `build_well_plate_job` hop tagging; handler overrides; Quick Print UI + `_build_settings`. Line speeds entered as **% of stage max** (Z = per-axis Z max ÷60; XY = `_xy_max_mm_s`), resolved to mm/s in `_build_settings`.
- [x] Feature 4 — polarity-correct reset (nested `_vol_to_baseline` in `run_print_cleanup` + `StageController.pump_volume_to_reach_uL`/`move_pump_to_position_uL`); reset-to-initial cleanup; Quick Print cleanup UI + ctx.
- [x] Feature 1 — `compute_pump_budget` + `simulate_pump_budget`; `prepare_starting_oil`; Quick Print `_on_print` pre-flight + `_check_syringe_budget`/`_offer_oil_remedy` dialogs + worker `starting_oil` wiring.
- [x] Tests — `tests/test_v75x_quick_print_syringe_budget.py` (19) + `TestLineMoveSpeedPercent` (2) in `test_v75x_quick_print_pick_and_place.py`.
- [x] Affected suites green (243 incl. quick-print/z-retract/print-always-safe-z/multi-object/cell-targeting/cell-labeling/spheroid/workflow-settings/pump-settle/pump-plunger). NOTE: `TestPickupVolume.test_dot_floored_to_prime` + `TestPrintSpeedPercent.test_scales_both_speed_and_flow` are **pre-existing**, order-dependent failures (the modified hardware-config `pump_prime_time_s`), confirmed by stashing this change — NOT caused here.
- [~] Adversarial-review workflow could not run (session token limit); replaced by a focused manual review of the budget math, polarity reset (both `pump_dir_sign`), and the GUI budget-gate/worker wiring.

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

## Addendum — auto-calculated flow + ink padding (operator follow-on)

Quick Print's **Flow @100% is now AUTO-calculated**, not a manual value:
- The deposited bead is modelled as a cylinder of the needle's **inner-bore
  cross-section** (`NeedleSpec.cross_section_area_mm2 = π·(id/2)²`) run along the
  print **length** (total XY path). Volume/mm = bore area, so the flow that keeps
  up with the stage is `area × speed`; at 100% speed (= calibrated XY max),
  `flow@100% = area × xy_max × extrusion_modifier`
  (`QuickPrintWorkflowPage._auto_flow_100_uL_s`). Falls back to a small constant
  when no needle is configured.
- New **"Extrusion modifier (×)"** (default 1.0) — line-thickness multiplier on
  the auto flow; scales BOTH the pump flow (thicker bead) and, automatically, the
  ink pickup. Replaces the editable "Flow @100%" spin.
- New **"Ink padding (µL)"** (default 0) — extra ink aspirated beyond the
  computed print volume so the needle never runs dry; additive to the pickup,
  flushed to waste in the reset-to-initial cleanup. Replaces the "Pickup safety
  (×)" multiplier.

**Bug fix — "prep prints oil" (operator report).** With prep ON the print
dispensed clear oil/buffer instead of ink; with prep OFF it printed ink. Root
cause: removing the old `pickup_safety` multiplier (the operator's profile had it
at **5×**, which had been masking the problem with a huge ink excess — logs show a
43 µL pickup for a ~9 µL deposit) and defaulting the new padding to 0 made the
pickup ≈ the deposit. On this machine **one needle of bore ≈ 6.8 µL** (logs:
buffer aspirate `-6.805 µL`), so a deposit-sized pickup barely fills the bore and
leaves the buffer right at the tip — a needle prep RESETS the needle to oil +
buffer with **no residual ink**, so the run relies solely on the pickup and the
print exhausts the thin plug into the buffer/oil. (Prep OFF leaves residual ink
from prior pickups, which masked it.) **Fix:** `_compute_pickup_volume_uL` now
keeps a full **needle-bore dead-volume reserve** of ink behind the deposit
(`dispensed + prime + needle_dead_volume + padding`, via new
`_needle_dead_volume_uL` = `NeedleSpec.internal_volume_uL`), so the bore stays
ink-filled and the buffer/oil never reaches the tip; the operator's padding is
extra on top. The reserve is real ink the syringe-budget pre-flight already
accounts for (larger pickup move), and it is wasted in the reset-to-initial
cleanup. **Needs real-HW re-verification on ME3B V1** (prep → print should now
deposit ink end-to-end).
- The budget pre-flight (`_print_dispense_volume_uL`, `_check_syringe_budget`)
  already runs off the resolved flow + pickup, so the modifier and padding flow
  through to the syringe-budget simulation unchanged.

**Test isolation fix:** the QuickPrint page tests (`test_v75x_quick_print_*`,
`test_v75x_print_setup_routine`) did not isolate `MEBP_WORKFLOW_SETTINGS_DIR`, so
the page's settings dialog restored the **operator's real saved Quick Print
profile** (`config/workflows/quick_print/__last__.json`: prep off, preflow 2.0,
…) over the code defaults — the true root of the formerly "pre-existing"
`TestPickupVolume`/`TestPrintSpeedPercent` failures and an order-dependent
`TestPrepDefault` flake. Fixed by pointing the store at a per-module temp dir at
import time (mirrors `test_v75x_workflow_settings_popout`); the operator's profile
is left untouched.

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

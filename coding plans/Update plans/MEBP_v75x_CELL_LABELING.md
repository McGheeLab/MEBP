# MEBP v7.5.x — Cell Labeling / Staining Workflow

## Objective

Add a new **Cell Labeling** workflow (enables the previously-stubbed
`cell_labeling` tile). It is a deliberate sibling of **Cell Targeting &
Removal**, with the operator-requested differences:

1. The deposit **and** the aspirate are both **slow** (a stain is delivered and
   withdrawn gently).
2. The headline knob is the **stain incubation time** — how long the stain is
   allowed to develop on the cells.
3. There is **no placement**: the operator only selects the *regions to stain*
   and the *stain reagent*. After incubation the recovered stain is dumped to
   the **waste** well (there is no place target).

Per region the needle: loads the stain → travels to the region (just off the
plate bottom) → SLOWLY deposits a small column (needle inner area × depth) →
incubates for the user-defined time → SLOWLY aspirates a multiple (default 2×)
of the deposited volume back up → travels to the waste well and dispenses it.
The pump is volume-balanced per region (load − deposit − aspirate +
waste-dispense = 0), so it never drifts. An optional needle **prep** brackets
the loop at the start (waste → oil → wash → buffer) and an optional **clean** at
the end (waste → wash → buffer), reusing the shared `PickPlaceExecutor` prep /
post-clean routines.

Operator decisions (confirmed up front):
- **Post-incubation:** aspirate the stain back up and **dump it to the waste
  well** (keeps the pump balanced; discards excess dye between regions).
- **Aspirate volume:** a **multiple** of the deposited volume (default 2×,
  configurable).

## Files Modified

| File | Change |
|------|--------|
| `SupportClasses/PickAndPlaceManager.py` | New `OperationType.CELL_LABELING`; new `CellLabelingConfig` dataclass (`compute_deposit_volume_uL` / `compute_aspirate_volume_uL` + `to_dict`); added to the `PickPlaceOperation.config` union; dispatch branch + new `_execute_cell_labeling` handler (load → region → slow deposit → incubate → slow aspirate → waste dump; pump volume-balanced; `pick_z_mm` = label height, `waste_well_pos` / `service_z_mm` = the always-required waste dump). |
| `gui/widgets/live_target_picker.py` | New `pick_only` constructor kwarg: hides the Place section + Mode toggle and forces every click to the Pick list. `_update_pairing_status` shows an "N region(s) selected" message in pick-only mode. Existing (paired pick/place) behaviour is unchanged when the flag is omitted. |
| `gui/pages/workflows/cell_labeling_workflow.py` | **NEW** — `CellLabelingWorkflowPage`, modelled on `CellTargetingWorkflowPage`. `pick_only` picker; "⚙ Settings" popout with sections **Stain incubation** (the headline dwell), **Region height**, **Pump & stain reagent**, **Stain deposit / removal** (deposit depth, aspirate ×, slow deposit/aspirate flows), **Needle prep / clean**, **Motion & timeouts**, + read-only locations. Start gate: ≥1 region, bore, Safe Z, ZP connected, calibrated plate bottom (label/reagent/service Z), stain reagent assigned + calibrated, and a **waste well** assigned + calibrated (always — the dump is core, not part of optional prep/clean). |
| `gui/pages/workflows/workflow_picker.py` | `cell_labeling` tile `enabled=True` + new description; docstring updated. |
| `gui/pages/workflows_mode.py` | Import + instantiate `CellLabelingWorkflowPage` for the `cell_labeling` tile. |
| `tests/test_v75x_cell_labeling.py` | **NEW** — 23 tests (config math, executor pump/travel sequence + balance + slow speeds + waste-dump + ends-at-safe-Z + ZP-disconnect refuse + multi-region, prep/clean bracketing, `pick_only` picker, offscreen page build + gating). |

## Implementation Steps

- [x] Backend: `OperationType.CELL_LABELING`, `CellLabelingConfig`, dispatch, `_execute_cell_labeling`.
- [x] `LiveTargetPicker.pick_only` mode.
- [x] `cell_labeling_workflow.py` page.
- [x] Wire into `workflow_picker.py` + `workflows_mode.py`.
- [x] Tests + run affected suites.
- [x] Update plan + CLAUDE.md table entry.

## Design Notes / Decisions

- **No place, waste is the sink.** With no placement target, the natural
  equivalent of Cell Targeting's "dispense at place" is "dispense to waste". The
  waste well is therefore ALWAYS required (set on the executor independently of
  the optional prep/clean), and gated in the GUI even when prep/clean are off.
  The service dip Z (`_service_z`) is used for the waste dump, so it is **not**
  disabled when prep/clean are toggled off.
- **Both flows slow.** Loading and the post-incubation aspirate use
  `aspirate_speed_uL_s`; the deposit and the waste dispense use
  `deposit_speed_uL_s`. Both default to 0.5 µL/s.
- **Reused, not reinvented.** The page reuses the shared `LiveTargetPicker`,
  `WorkspaceTargetView`, `XZSideView`, `StandardJogContextPanel`,
  `WorkflowSettingsDialog`, the `_reagent_prep` service-well helpers, and the
  `PickPlaceExecutor` prep / post-clean / safe-Z primitives. The label height
  reuses the executor's `pick_z_mm` field (same contract as the spheroid /
  cell-removal handlers).
- **Intra-well clean waste.** When clean-after is on, the op's final waste dump
  leaves the needle at the waste well, so the clean's first (waste) step runs as
  an intra-well move rather than a full safe-Z travel — correct and expected.

## Testing Notes

- `python -m unittest tests.test_v75x_cell_labeling` — 23 tests green.
- Regression: `test_v75x_cell_targeting_removal`, `test_v75x_spheroid_pick_place_z`,
  `test_v75x_quick_print_pick_and_place`, `test_v75x_spheroid_picker_scaling`,
  `test_v75x_workflow_settings_popout` — all green (149).
- Offscreen smoke: `WorkflowsModePage` builds and selects the `cell_labeling`
  page (title "Cell Labeling").

## Needs real-HW verification on ME3B V1

Per region on real hardware: stain loads from its reagent well, the needle
lowers to the label Z at the picked region, deposits slowly, holds for the
incubation time, aspirates ×-multiple slowly, then dumps to waste; the run ends
at Safe Z; prep/clean bracket correctly; a missing waste-well / uncalibrated
plate-bottom is refused with a clear message.

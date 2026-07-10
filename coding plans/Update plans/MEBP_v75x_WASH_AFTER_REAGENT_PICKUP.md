# MEBP v7.5.x — Wash needle after reagent pickup (before deposit)

## Objective

For the **Cell Targeting & Removal** and **Cell Labeling** workflows, rinse the
needle **exterior** at the wash well **immediately after aspirating the reagent
from its reagent well and before travelling to deposit it** onto the cells.

Operator intent (verbatim): *"we will need to wash the needle before we go
deposit"* → clarified: *"cell labeling"* (not immuno) + *"we just want to wash
after picking up trypsan"*.

The aspirated reagent (trypsin for removal / stain for labeling) stays inside
the needle **bore** — held by suction. When the needle dips into the reagent
well to aspirate, its **outer surface** picks up a film of reagent. Without a
wash, that exterior film reaches the cells too, so the delivered dose is not
just the metered push/deposit column. A gentle rinse at the wash well removes
the exterior film so only the intended volume is delivered.

This is **not** a wash of the extracted-cells payload, and **not** a wash before
the final placement of extracted cells — it is a wash right after reagent
pickup.

## Scope decisions (operator-confirmed)

- **Workflows:** Cell Targeting & Removal + Cell Labeling. Immuno is still a
  stub (no page) — untouched.
- **Timing:** after the reagent-load step, before the travel to the deposit
  (the push at the removal location / the deposit at the stain region).
- **Wash mechanics:** reuse the existing `PickAndPlaceManager._do_wash`
  (dip → Z jiggle → random XY jiggle × cycles), driven by the SAME wash
  config already exposed in each workflow's Settings (cycles / Z amp / XY amp /
  settle). No new wash knobs.
- **Toggle:** a new "Wash needle after reagent pickup" checkbox, **default ON**
  (the operator said "we will need to"). Persists in the workflow settings
  profile.
- **NOT applied** to the spheroid workflow or Quick Print.

## Files Modified

| File | Change |
|------|--------|
| `SupportClasses/PickAndPlaceManager.py` | New executor state `wash_after_pickup` (default False); new `_wash_needle(op, label)` helper (safe-travel to `__wash__` → dip to service Z → `_do_wash`; no-op if wash well unconfigured; abort-aware); inserted after the reagent-load step in `_execute_cell_removal` and `_execute_cell_labeling`. |
| `gui/pages/workflows/cell_targeting_workflow.py` | "Wash needle after reagent pickup" checkbox (default ON) in the Needle prep/clean section; `_on_prep_toggled` enables wash mechanics when it's on (even with prep/clean off); `_on_start` resolves the wash well + service Z + wash config and wires `executor.wash_after_pickup` when on; `_refresh_prep_status` reflects the wash-only case. |
| `gui/pages/workflows/cell_labeling_workflow.py` | Same checkbox ("after stain pickup" wording) + gating + wiring. Waste well + service Z already resolved unconditionally there; only the wash well is the added gate. |
| `tests/test_v75x_wash_after_reagent_pickup.py` | New test suite (executor + offscreen page). |

## Implementation Steps

- [x] Create this plan doc.
- [x] Backend: `wash_after_pickup` state + `_wash_needle` helper.
- [x] Backend: insert wash step in `_execute_cell_removal` (after load, before removal travel).
- [x] Backend: insert wash step in `_execute_cell_labeling` (after load, before region travel).
- [x] GUI: Cell Targeting page — checkbox + toggle-enable + start-gate + wiring + status.
- [x] GUI: Cell Labeling page — checkbox + toggle-enable + start-gate + wiring + status.
- [x] Tests: new `tests/test_v75x_wash_after_reagent_pickup.py`.
- [x] Run affected suites green.

## Testing Notes

`tests/test_v75x_wash_after_reagent_pickup.py` drives the REAL `PickPlaceExecutor`
against the recording fake controller (mirrors the sibling suites):

- Removal + labeling: with `wash_after_pickup=True` the travel sequence gains a
  wash-well leg between the reagent load and the deposit travel; the reagent
  bore's pump volumes are UNCHANGED (still load → push/deposit → …).
- Default OFF: no extra wash leg.
- Graceful no-op when `wash_well_pos` is unset.
- Offscreen page smoke: checkbox exists, defaults ON, persists via the settings
  dialog registry.

Result: `tests/test_v75x_wash_after_reagent_pickup.py` (10) + the sibling
`test_v75x_cell_targeting_removal` / `test_v75x_cell_labeling` suites = **57
green**; spheroid / workflow-settings-popout / common-print-settings = **73
green** (shared executor + settings dialog unaffected — the change is additive,
`wash_after_pickup` defaults False on the executor).

Real-HW verification on ME3B V1 still pending: pick up trypsin/stain → the
needle dips at the wash well and jiggles → travels to the cell and deposits.

## Issues & Decisions

- **Where the wash lands:** after the reagent load, before the deposit travel.
  For cell removal there are two "deposits" (push trypsin at the cell, dispense
  extracted cells at the placement); the operator's "wash after picking up
  trypsin" is the FIRST — so the wash sits between load-reagent and
  travel-to-removal, NOT before the final cell placement.
- **Wash-only gating:** when prep/clean are both OFF but wash-after-pickup is
  ON, only the wash well (+ service dip Z) is required — not the full
  waste/oil/buffer set the prep/clean cycle needs.
- **Reuses `_do_wash`:** no gentle-vs-full distinction added. The reagent is
  safely inside the bore (suction), so the existing jiggle wash is fine; the
  cycles/amplitudes are already operator-tunable in Settings.

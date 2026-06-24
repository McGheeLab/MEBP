# MEBP v7.5.x — Cell Targeting & Removal Workflow

## Objective

Add a new **Cell Targeting & Removal** workflow (enables the previously-stubbed
`cell_targeting` tile), built on the Spheroid Pick & Place scaffold. The operator
loads the needle with a **cell-release reagent** (e.g. trypsin) assigned to a
reagent well, then for each picked target the needle trypsinizes the cells *in
place* and extracts them to a chosen placement location.

Operator-specified sequence:

1. **Standard needle prep** (waste → oil → wash → buffer) — the same prep used by
   Quick Print and Spheroid Pick & Place. Once, before the loop.
2. **Load the needle with the cell-release reagent** (trypsin), drawn from its
   reagent well.
3. **Travel to the cell-removal location** (x, y) and lower to a Z **0.1 mm off
   the plate bottom** (configurable).
4. **Slowly push in a small amount** of reagent = **needle inner area × push
   depth** (push depth default 0.1 mm).
5. **Wait** a user-defined incubation time.
6. **Quickly pull up 2×** the volume of reagent used (configurable multiplier).
7. **Move to the placing location** (user-defined) and dispense the cells.
8. **Needle waste, wash, and reset** (waste → wash → buffer). Once, after the loop.

Steps **1** and **8** bracket the whole loop (run once each); steps **2–7** run per
picked removal→placement pair. The reagent well + the four service wells are
inherited from Hardware Setup → Ink (Reagent Locations); all Z heights are
entered as a height above the calibrated plate bottom and resolved
polarity-safely via `StageController.print_height_to_zref`.

## Files Modified

| File | Rationale |
|------|-----------|
| `SupportClasses/PickAndPlaceManager.py` | New `OperationType.CELL_TARGET_REMOVAL`; new `CellRemovalConfig` dataclass (push volume from needle bore area × depth, ×multiplier pull, slow/fast speeds, removal/place Z offsets); new executor fields (`reagent_well_pos`/`reagent_dip_z_mm`, `do_post_clean`/`post_expel_needles`); `__reagent__` special well; `_execute_cell_removal` handler + dispatch; `run_post_clean()` (waste → wash → buffer); `execute_queue` runs post-clean once after the loop. |
| `gui/pages/workflows/cell_targeting_workflow.py` | **NEW** workflow page — clone of the Spheroid scaffold (LiveTargetPicker + WorkspaceTargetView + XZSideView + StandardJogContextPanel + executor bridge) with a cell-removal config row, a cell-release reagent picker (mirrors Quick Print's ink combo), prep + "clean after" rows, the start gate, and executor wiring. |
| `gui/pages/workflows/workflow_picker.py` | Enabled the `cell_targeting` tile (`enabled=True`) + updated description/docstring. |
| `gui/pages/workflows_mode.py` | Import + instantiate `CellTargetingWorkflowPage` for the `cell_targeting` tile. |
| `tests/test_v75x_cell_targeting_removal.py` | **NEW** — 21 tests (config math, executor sequence/balance/speeds/Z, prep+clean bracketing, ZP-disconnect refusal, offscreen page build + gating). |

## Implementation Steps

- [x] Backend: `OperationType.CELL_TARGET_REMOVAL`.
- [x] Backend: `CellRemovalConfig` (+ `compute_release_volume_uL` / `compute_extract_volume_uL`).
- [x] Backend: executor `reagent_well_pos` / `reagent_dip_z_mm` + `__reagent__` resolution.
- [x] Backend: `_execute_cell_removal` (load → travel → slow push → dwell → fast pull → travel → dispense; volume-balanced).
- [x] Backend: `do_post_clean` / `post_expel_needles` + `run_post_clean()` (waste → wash → buffer), called once after the loop in `execute_queue`.
- [x] GUI: `cell_targeting_workflow.py` page (config / reagent / prep+clean rows, gates, executor wiring).
- [x] Wire: enable the tile + instantiate in the mode container.
- [x] Tests + offscreen page smoke.

## Key Design Decisions

- **Reuse, don't fork.** The whole pick/place scaffold (live picker, workspace,
  XZ, context panel, executor bridge), the prep routine (`run_prep`), the safe-Z
  travel primitives (`safe_travel_to` / `ensure_retracted_to`), the µL-native
  pump moves (`move_pump_uL(rate_uL_s=…)`), and the service-well resolution
  (`_reagent_prep` helpers) are reused unchanged. Only the per-op handler +
  post-clean are new. Spheroid Pick & Place is untouched.
- **Push volume = needle bore column.** "A small amount … needle inner area
  times 0.1 mm" → `NeedleSpec.cross_section_area_mm² × push_depth_mm` (1 mm³ = 1
  µL). The GUI computes it (and stamps `release_volume_uL` for display + as a
  backstop); the executor recomputes from the live needle when `hw_config` has
  one (single source of truth = the needle).
- **Pump stays volume-balanced** over each op: `−push (load) + push (release)
  − pull (extract) + pull (dispense) = 0`, so the plunger never drifts across a
  multi-target run.
- **Slow push / fast pull** are independent flow rates (`push_speed_uL_s`,
  `pull_speed_uL_s`); the final cell dispense uses the gentle push speed.
- **Removal/place heights** reuse the executor's existing `pick_z_mm`/`place_z_mm`
  fields (zero-ref mm) — the GUI resolves the height-above-bottom offsets through
  `print_height_to_zref`, polarity-correct on ME3B V1 (`ZDIR=-1`).
- **"Reset" = reload buffer.** Post-clean ends by drawing buffer (like the prep
  tail) so the needle finishes in a known clean, conditioned, buffer-loaded
  state. Reuses the same service wells / needle volume as the prep.
- **Reagent is required** (you can't trypsinize without it); prep + clean default
  ON but can be turned off. The Start gate checks: balanced pick/place pairs,
  Safe Z, ZP connected, calibrated plate bottom (removal/place/reagent/service
  dip Z), needle inner Ø (push volume), reagent assigned + calibrated, and (when
  prep/clean on) the four service wells + needle volume.

## Safety

Honors the critical retract-before-XY-travel invariant: every cross-position move
(reagent load, removal, placement, all prep/clean service hops) routes through
`_safe_move_to` → `StageController.safe_travel_to` (retract → confirm → XY →
lower). The picker emits empty-`well_name` targets, so each is a full safe-Z
travel (never the 1 mm intra-well shortcut). A mid-run ZP drop raises
`AbortException` (refuses to drag an unretracted needle). `execute_queue`'s
`finally` always ends the run at the safe Z via `_retract_to_safe_z`.

## Testing Notes

`tests/test_v75x_cell_targeting_removal.py` (21, all green):

- **Config:** push = bore area × depth (with needle), fallback to stamped value,
  pull = ×multiplier.
- **Executor:** load/push/pull/dispense pump order + signs; volume balance (Σ ≈
  0); slow-push/fast-pull/gentle-dispense rates; needle-overrides-config; three
  full safe travels with distinct removal/place/reagent dip Z; reagent well hit
  first; ends at safe Z; ZP-disconnect refusal; no-reagent-well graceful skip.
- **Prep + clean:** prep(4) → op(3) → clean(3) travel order = 10; clean-only;
  clean pump volumes (expel +1, buffer −4 needles); clean skipped when off;
  missing service well in clean aborts + still retracts.
- **Page (offscreen):** constructs; `_push_pull_uL` from needle; `_current_config`
  reflects spins; Start refuses when pick/place counts are unbalanced.

Adjacent suites green: `test_v75x_spheroid_pick_place_z`,
`test_v75x_quick_print_workflow`, `test_v75x_quick_print_pick_and_place`,
`test_v75x_spheroid_picker_scaling` (79).

**Needs real-HW verification on ME3B V1.**

## Issues & Decisions

- Removal/place heights reuse `pick_z_mm`/`place_z_mm` rather than adding new
  executor fields — same contract as the spheroid handler (None → `operating_z_mm`).
- Trypsin is loaded **per op** (not once before the loop) so each
  removal→placement pair is self-contained and the pump ends balanced; with one
  picked target this is identical to the operator's single-cell description.

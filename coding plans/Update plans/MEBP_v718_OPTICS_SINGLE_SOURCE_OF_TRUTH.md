# MEBP v7.18 — One optics truth + automatic objective / filter-cube adjustment

## Objective

Make "which objective and which filter cube is in the light path" **one fact, read from the
body**, and give every workflow a shared Qt-free way to **adjust** the optics automatically —
refusing loudly rather than reporting a switch that did not happen.

Operator: *"Multiple workflows throughout the code ask for objectives and filter cubes. All of
these areas should know what the current filter cube is, and what the current objective is. If in
a workflow it needs to change filter cubes or objectives, it should do so automatically."*

The hardware write path **already exists and is hardware-verified** —
`MicroscopeController.set_filter(pos)` / `set_objective(pos)` are public, thread-safe (queued onto
one COM-affinity worker thread), lease-aware, and implemented on all three backends. What is
missing is name→slot resolution, a filter equivalent of `resolve_ladder`, a shared `ensure` helper
with read-back verification, and the wiring.

### Operator decisions (AskUserQuestion, 2026-08-12)

1. **Cubes switch silently (logged); objective changes are announced**, retract focus away from
   the plate first, and apply the parfocal offset when one is taught.
2. **The label-only objective combos become live controls**, *and* the app follows a hand-rotated
   turret — *"we can change objectives from the microscope and it knows that and adapts."*
3. **Channel → cube is an explicit per-machine binding** in `microscope.json`, auto-seeded by
   case-insensitive name match. Unmatched (`mCherry`, `Bright Field`) stays unbound and falls back
   to the manual prompt — never guessed.
4. **The per-channel "Set filter" modal is skipped** when that channel already has a remembered
   exposure; shown only for a channel with no exposure yet, or when "pause between channels" is ticked.

---

## Phase 0 blockers — fixed before any new module (no motion)

| # | Defect | Consequence | Status |
|---|---|---|---|
| **B1** | Case-sensitive objective-name lookup. `config/hardware/objectives.json` calibration keys are `"4x"/"10x"/"20x"`; `config/hardware/microscope.json` nosepiece labels are `{"1":"4X","2":"10X","3":"20x"}`. `ObjectiveCalibration.get_calibration` is an exact, case-sensitive nested `dict.get`. | `ObjectiveLadder.resolve_ladder` returns `calibrated=False` for nosepiece positions 1 and 2 with *"no µm/px calibration for '4X'"*, and `ladder_gate` compares with case-sensitive `!=`. **The plate-bed-leveling objective ladder is unusable for 4X and 10X on this rig, purely because of letter case** — and its message sends the operator to re-run a calibration they already have (measured 2026-08-11). | `[ ]` |
| **B2** | `FluorescenceMosaicStore.CHANNEL_NUMBERS` hardcodes DAPI→1 … `Bright Field`→5, never reconciled against `filter_labels()` or `native_filter_names`. | A second owner of the slot map. This rig's slot 3 is **TxRed**, not mCherry; slot 5 is **unnamed**. Both wrong facts are printed to the operator as if read from the body. | `[ ]` |
| **B3** | `fluorescence_mosaic_workflow.py:2894` reads `getattr(optic, "name", "")`, but `MountedOptic` has **`label`**, not `name` (sibling lines correctly read `magnification` / `working_distance_mm`). | Silent, inside a `getattr` default: the autofocus `ObjectiveOptics.label` **always** falls back to `self._scan_objective` and never carries the body's own label. | `[ ]` |
| **B4** | Module docstring lines 5-8 assert *"There is no filter-wheel hardware, so the operator switches the physical filter"*; the pre-run confirm promises a prompt. | Now false — the Ti's `FilterBlockCassette1` was hardware-verified switching all 6 slots with read-back. The code documents the opposite of what it will do. | `[ ]` |
| **B5** | `plate_level_wizard._restore` restores **objective before focus** (lines 368-389). | If the entry objective is the 20x (WD ~1 mm) and the run ended at a 4x-legal focus height, restore rotates the 20x in **at that height**. Reorder to focus-safe → objective → focus-entry. | `[ ]` |

---

## Files Modified

### New — pure, Qt-free (`SupportClasses/`)

| File | Rationale |
|---|---|
| `OpticsRegistry.py` | The ONE join of live turret state + operator slot labels + body-reported names, for **both** turrets, plus the reverse `name → slot` lookup that exists nowhere today. Stdlib only, so it can be imported from anywhere with no cycle risk. |
| `OpticsService.py` | The shared idempotent `ensure_filter` / `ensure_objective`, shaped after the existing `ensure_retracted_to` precedent. Lifts `wait_for_op` / `restore` out of `plate_level_wizard`. |

### New — thin GUI

| File | Rationale |
|---|---|
| `gui/widgets/optics_mirror.py` | `OpticsMirror(QObject)` — the SINGLE writer of `CameraConfig.current_objective_name`, on the GUI thread, with lease + run-freeze interlocks. |
| `gui/widgets/optics_ensure.py` | `ensure_optics_or_prompt(...)` — runs one `EnsureResult` off the GUI thread with a busy label; on refusal shows the existing manual prompt with `why_not` prepended. |

### Modified

| File | Change |
|---|---|
| `SupportClasses/ObjectiveCalibration.py` | B1: `_resolve_key`; `set_calibration` reuses an existing case-variant key; `add_objective` refuses normalized duplicates |
| `SupportClasses/ObjectiveLadder.py` | `resolve_ladder` re-expressed on `resolve_slots`; `_optic_at` promoted to the registry; `ladder_gate` normalized + alias-aware |
| `SupportClasses/MicroscopeConfigStore.py` | `optic_aliases` / `set_optic_alias` / `clear_optic_alias`; real `focus_min_um` / `focus_max_um` (both `null` today ⇒ `_clamp_focus` is a no-op) |
| `SupportClasses/FocusSweepPlanner.py` | new `plan_turret_change(...)` beside `plan_sweep` / `plan_handoff` / `wd_bounded_half_range_um` |
| `SupportClasses/FluorescenceMosaicStore.py` | B2; `save_channel` gains `cube_slot` / `cube_label` |
| `gui/pages/workflows/fluorescence_mosaic_workflow.py` | B3, B4; `_prompt_next_channel` → `ensure_filter` first; per-channel objective verification; objective combo demoted; shared µm/px helper |
| `gui/pages/hardware/objective_calibration_card.py` | selection drives the turret; stops writing the name; µm/px push helper extracted |
| `gui/pages/calibration.py` | `_ploc_confirm_objective` (both call sites) reads the body and auto-corrects; per-camera "Obj:" combo becomes a live control |
| `gui/widgets/plate_level_wizard.py` | B5; `_scope_op`→`wait_for_op`, `_restore`→`service.restore`, per-rung switch → `ensure_objective`, "Adjust" button |
| `gui/widgets/microscope_panel.py`, `gui/pages/hardware/microscope_setup_panel.py` | → `ensure_position`; lease owner shown + combos disabled while leased; "Check names" + Fix; alias editor |
| `gui/widgets/live_target_picker.py` | re-resolve on `objective_changed`; disagreement banner + click gate |
| `gui/app.py` | own + tear down the single `OpticsMirror` |
| `CLAUDE.md` | replace the "deliberately NOT linked" paragraph with the resolved decision |

---

## Implementation Steps

### Stage 0 — name resolution + the read path. **NO MOTION.** ✅ COMPLETE

- `[x]` B1 — `ObjectiveCalibration`: `_resolve_key` (exact → unique normalized → None, logging both keys on a non-unique match); `get_calibration` / `clear_calibration` / `nominal_magnification` / `remove_objective` route through it; `set_calibration` reuses an existing case-variant key rather than creating a sibling; `add_objective` refuses a normalized duplicate
- `[x]` `SupportClasses/OpticsRegistry.py` — `normalize_optic_name`, `OpticSlot`, `SlotMatch`, `OpticsSnapshot`, `resolve_slots`/`resolve_filters`/`resolve_objectives`, `find_slot`, `optic_at`, `snapshot`
- `[x]` `MicroscopeConfigStore` — `optic_aliases` / `set_optic_alias` / `clear_optic_alias` (+ `optic_aliases` in `_blank()` and a `_clean_aliases` pass in `_load()`); `set_optic_alias` refuses a target that is not a named slot AND refuses a case-only "alias" (normalization already covers that, so one would be a no-op that reads as configured)
- `[x]` `ObjectiveLadder` — `resolve_ladder` onto `resolve_slots` (signature, `LadderRung` shape and every refusal sentence unchanged); `_optic_at` delegates to the shared walk; `ladder_gate` compares normalized and honours an `aliases=` mapping
- `[x]` B2 — `CHANNEL_NUMBERS` → `CHANNEL_ORDINALS` with the legacy name kept as the *same object* (on-disk `.nd3` sidecars carry it and `LabLinkJob` orders by it); new `channel_ordinal()`; new `channel_slot()` doing the real registry-backed resolution, `kind=FILTER` explicit so an empty-cassette refusal names the right turret
- `[x]` B3 — `getattr(optic, "name", "")` → `label`, routed through the shared `optic_at`
- `[x]` B4 — module docstring corrected to state that the cube IS drivable and the prompt is now the fallback
- `[x]` B5 — `_restore` reordered to focus → objective → **verify by read-back**
- `[x]` Tests: `test_v718_objective_name_resolution.py` (25), `test_v718_optics_registry.py` (34), `test_v718_gui_free_core.py` (11) — **70 new, all passing**
- `[ ]` Read-only reporting of the two records on `microscope_panel`, `live_target_picker` and the calibration dialog captions → **deferred to Stage 5**, where those surfaces are edited anyway; it is presentation, and keeping Stage 0 motion-free and store-only kept the blast radius small

**Gate — met.** `test_v711_objective_ladder` passes **unchanged** (38 tests, no edits to that
file). Verified against this rig's real stores: `4X` → 1.896833, `10X` → 0.752684, `20x` →
0.370371 µm/px; `40x` and `""` still refuse; nothing moved. Regression **793 green** across
Stage-0-adjacent suites plus a `gui.app` import smoke.

**Mutation check: 11/11 CAUGHT**, every source restored byte-identically (SHA256-verified) and the
baseline re-confirmed green afterwards — case-sensitive `dict.get` restored · ambiguity guessing
instead of refusing · `set_calibration` creating a case sibling · `add_objective` accepting a
case-only duplicate · `ladder_gate` comparing case-sensitively · a fuzzy substring tier in
`find_slot` · accepting a slot the body reports empty · `set_optic_alias` accepting a bogus target ·
the workflow reading `optic.name` again · Qt imported into the pure core · `resolve_ladder`
re-implementing the walk.

### Stage 1 — the verified-switch primitive ✅ COMPLETE (with the Stage-2 guard pulled forward)

- `[x]` `SupportClasses/OpticsService.py` — `EnsureResult`, `wait_for_op`, `OpticsService` with `ensure_filter` / `ensure_objective` / `ensure_position` / `capture_entry` / `restore`
- `[x]` Lease hygiene on `microscope_panel`: it no longer **polls** under someone else's lease (a lease-blocked op fails fast *without* publishing an error, so a refused poll is invisible and the card would render off the lease holder's own reads then silently go stale), both combos are disabled with a tooltip naming the owner, and the status line reads `◐ reserved by plate level`
- `[x]` Tests: `test_v718_optics_service.py` (**51**), `test_v718_turret_focus_safety.py` (**21**)

**⭐ `plan_turret_change` was pulled forward from Stage 2 deliberately.** The staged plan had
`ensure_objective` land in Stage 1 and its collision guard in Stage 2, which would have left a
callable turret-mover with no guard for one stage. Since the guard is pure code and needs no bench
time, shipping them together means there is never a window in which the app can rotate a nosepiece
without proving the rotation is safe. The remaining Stage 2 items are operator/bench actions, not
code: real `focus_min_um`/`focus_max_um` in `microscope.json`, and measuring the parfocal offsets.

**⚠ DEVIATION — `plate_level_wizard._scope_op` was NOT re-pointed onto `wait_for_op`.** Its wording
is materially better (the stale case names the two surfaces that actually contend for the body) and
17 existing tests pin that text, so delegating would mean re-deriving the case in the wizard purely
to restore the phrasing — more code, no more safety. What mattered was the DECISION being shared,
so `TestTheWizardAndTheServiceCannotDrift` pins that the two agree on all six outcomes (clean /
stale / reserved / driver error / timeout / no-op), and `_scope_op` now carries a pointer to
`wait_for_op` for new callers. Its `_restore` is likewise kept — `service.restore` also restores the
filter, which this workflow never touches.

**Mutation check: 14/15 CAUGHT**, sources restored byte-identically, baseline green after —
`wait_for_op` ignoring `.error` · a stale drop retried instead of aborted · the position read-back
dropped · the freshness refresh skipped · releasing a lease it did not take · the needle gate
removed · the focus retreat skipped · parfocal using `offset[to]` absolute · the guard bounding by
the SOURCE objective's WD · a fabricated budget for an unknown WD · a missing glass datum assumed to
be the current focus · the wizard's restore rotating before the focus · the panel polling under
another lease · the panel's combos left live while leased.

⚠ **Two mutations initially SURVIVED and were real test gaps, now closed** — both the failure mode
this file keeps recording (covering the new code and not the old): the freshness refresh had no test
because every fixture was already fresh, so a **hand-rotated turret** case was added (the exact
scenario the service exists for); and B5's ordering fix was covered only in `OpticsService.restore`,
not in the wizard's own `_restore`, so reverting it there passed everything.

⚠ **One mutation survives legitimately and is documented rather than "fixed":** the raise-only
belt-and-braces branch in `plan_turret_change` **cannot fire**. Reaching it requires the focus to be
closer to the specimen than the budget allows, and the retreat target is then always further away.
Measured over 400 000 randomised (polarity, WD, glass, focus, limits) combinations: 121 265 reached
that point, the condition held **0 times**. It is kept as free insurance against a future change to
the target computation, with a comment saying so — the property test is what actually protects the
invariant. Claiming a test for an unreachable branch would be the dishonest option.

### Stage 1b — objectives are CLASSIFIED, with a picker ✅ COMPLETE

Operator: *"the objectives should all be well classified with the focal depth … lets fill out a
list of common objectives from nikon that we can apply and have a drop down for selection"*, naming
the three fitted on ME3B_01: **4x/0.13 Plan Fluor WD 17.1**, **10x/0.30 Plan Fluor DIC L/N1
∞/0.17 WD 16.0**, **CFI Plan Apo VC 20x/0.75 air**.

**Why this is a safety change, not a convenience.** Until now NA and working distance came ONLY from
the body, which knows nothing but the product code programmed into its nosepiece — routinely blank,
and sometimes a different variant of the same nominal name. Both figures do real work:

| | body reported (pre-v7.18) | actual part | consequence |
|---|---|---|---|
| 4x | NA 0.13, WD 16.4 | 0.13, **17.1** | DOF unchanged at 47.1 µm; budget ±4100 → ±4275 µm |
| 10x | NA 0.30, WD **4.0** | 0.30, **16.0** | DOF unchanged at 8.6 µm; budget **4× too tight** |
| 20x | NA **0.45**, WD 1.0 | **0.75**, 1.0 | **DOF 3.54 → 1.47 µm — the sweep step was 2.4× too coarse**, i.e. "a real peak can hide between samples" |

- `[x]` NEW `SupportClasses/ObjectiveCatalogue.py` — mirrors `FilterCubeStore` exactly (shared path
  so a part list syncs across rigs, `builtin/` + `user/` with user shadowing by id, provenance on
  every figure, `safe_id`/`clean_*` that never raise). No fuzzy name tier: `"20x"` names a
  magnification, not a part, and this rig's 20x/0.75 vs a Plan Achromat 20x/0.40 differ by **13× in
  working distance**.
- `[x]` NEW `config/hardware/ME3B_general/objective_types/builtin/nikon_cfi.json` — **27 entries**
  across CFI Achromat / Plan Achromat / Plan Fluor / S Plan Fluor ELWD / Plan Apo λ / Plan Apo VC.
  The operator's three carry `datasheet` provenance; **every other entry is `nominal`** and the UI
  says to confirm against the engraving. No product code is ever guessed — a wrong one silently
  invalidates a parfocal offset. ELWD entries record the **shortest** collar position, because WD is
  a collision bound and software cannot read the collar.
- `[x]` `MicroscopeConfigStore.objective_specs` / `objective_spec_for` / `set_objective_spec` /
  `set_all_objective_specs` + `_clean_objective_specs`. Keyed by NAME (like `filter_optics`) so the
  optics travel with the lens between positions. **Unit slips are REFUSED, not clamped** —
  `working_distance_mm=17100` would authorise a 17-metre focus excursion, and an NA of 75 is a
  percentage. An entry with neither NA nor WD is not stored, because the cleaner drops exactly that
  on reload and accepting it would look saved and vanish.
- `[x]` `OpticsRegistry._merge_objective_spec` — **the spec wins on NA** (the operator knows the
  part), but **working distance takes the SHORTER of the two on a disagreement**, which is a
  different rule on purpose: WD decides whether a rotation is allowed, so the fail-safe direction is
  the smaller number regardless of which source is more trustworthy. The disagreement is reported,
  never swallowed. One escape hatch, requiring a deliberate act: a `measured` provenance wins
  outright, so a body reporting a wrong code cannot permanently cap a correct objective.
- `[x]` `immersion_n` threaded through `OpticSlot` → `LadderRung` → `ObjectiveOptics`, so an
  oil/water objective's DOF is no longer computed as if it were dry (~1.5× error).
- `[x]` UI: a **separate, non-editable "Objective type" column** plus NA / WD / Source, on
  Hardware Setup → Microscope.
- `[x]` Tests: `test_v718_objective_catalogue.py` (**49**). **12/12 mutations CAUGHT.**

⚠ **THE PICKER MUST NOT SHARE A WIDGET WITH THE NAME, and my first cut did.** Mirroring the filter
cube's editable-combo-as-name-field renamed the objective on every pick — Qt rewrites an editable
combo's line-edit text whenever an item is selected, which no care in the handler can undo. That
name is the key for the objective's µm/px calibration in `ObjectiveCalibration`, its parfocal offset
and its own spec, so a pick would have **orphaned the operator's measurements** — precisely the
class of change this document forbids. Caught by driving the real panel offscreen and asserting the
label survived. Name and part are separate facts and now have separate widgets; the name column is
untouched free text, and only a BLANK slot is filled in as a convenience.

⚠ **A legitimate test update in the concurrent filter-cube work:**
`test_v718_filter_cube_catalogue::test_objectives_stay_plain_text` asserted the objective row has no
combo at all — true when the catalogue was a cube-only concept. Rewritten (not deleted) to assert
the surviving, stronger requirement: the name widget is a plain `QLineEdit`, the picker is a
separate non-editable widget, and no filter-cube band widgets leaked onto the nosepiece rows.

⚠ **My own purity guard caught a real design slip.** `OpticsRegistry` imported `ObjectiveCatalogue`
for the immersion table, breaking the "imports nothing from the repo" invariant that lets
`ObjectiveCalibration` and `MicroscopeConfigStore` both import it. The table moved DOWN into
`OpticsRegistry` and is re-exported from the catalogue, with a test pinning both names to one object.

⚠ **Process hazard worth recording — a same-length mutation poisoned the bytecode cache.** Python
invalidates a `.pyc` on (source mtime, source SIZE), and swapping two operands
(`na = spec_na if spec_na else body_na` ⇄ the reverse) is byte-length **identical**. Restoring inside
the same mtime tick let the interpreter reuse the MUTATED bytecode, producing a red baseline against
a SHA-verified-identical source file — genuinely confusing to chase. Mutation harnesses here must
run with `python -B` / `PYTHONDONTWRITEBYTECODE=1`.

⚠ **One mutation was INERT and had to be redone.** An `if False: pass` inserted above a guard
changes nothing, so its survival proved nothing; re-expressed as deleting the guard's `return`, it
was caught immediately.

Regression **585 green** across the optics, plate-level, bore-wizard, mosaic and LabLink suites,
plus the full 222-test v7.18 sweep and a `gui.app` import smoke.

### Stage 2 — remaining items are BENCH work, not code
- `[ ]` Real focus soft limits in `microscope.json` (both `null` today ⇒ `_clamp_focus` is a no-op)
- `[ ]` Measure + install parfocal offsets, then enable `parfocal_auto_apply`
- `[ ]` **Assign the three catalogue entries** on Hardware Setup → Microscope and confirm the 10x
  working-distance conflict appears (the body says 4.0 mm, the part is 16.0 mm) — then settle which
  is right rather than letting the fail-safe cap it forever

### Stage 3 — cube resolution + the channel recipe
- `[ ]` Aliases UI; `refresh_mounted()` at run start; refuse on zero / ambiguous / `present=False`
- `[ ]` Channel recipe (cube + LED + exposure) as one ordered, darker-first transaction
- `[ ]` Tests: `test_v718_filter_cube_resolution.py`

### Stage 4 — the single writer
- `[ ]` `OpticsMirror` + lease and run-freeze interlocks + shared µm/px push helper
- `[ ]` Tests: `test_v718_single_writer.py`

### Stage 5 — the surfaces
- `[ ]` Fluorescence unattended; `_ploc_confirm_objective` auto-correct; combos as live controls; picker banner/gate; "Check names" + Fix
- `[ ]` Tests: `test_v718_fluor_mosaic_optics.py`, `test_v718_needle_gate.py`

---

## Testing Notes

House style: `unittest`, offscreen Qt, hand-written module-level `_Fake*` hardware classes,
env-var/tmpdir store isolation, test names as sentences. **Every test that matters drives the
production function** — `resolve_ladder`, `ladder_gate`, `OpticsService.ensure_*`,
`plan_turret_change`, `PlateLevelSurveyWorker` — and at least one per module goes through the
**real `MicroscopeController`** over a fake *backend*, so the `_Op` / `.error` / `STALE_OP_S` /
`_read_all` contract is exercised rather than modelled.

`_ScriptedTiBackend` reports **this rig's actual data** (objectives 4x/10x/20x with WD 16.4 … 1.0;
filters DAPI/FITC/TxRed/Cy5 with slots 5 and 6 `present=False`) and injects `refuse_on=`,
**`ack_without_moving=`** (the case `.error` cannot catch), `slow_by_s=25.0` (drives the real stale
path), `clamp_focus_at=`. The config store is seeded with the **disagreeing** labels
`{"1":"4X","2":"10X","3":"20x"}` against keys `4x/10x/20x` — that seed is what makes the
regression real rather than hypothetical.

⚠ `tests/test_v75x_fluorescence_mosaic.py::test_channel_numbers` asserts the hardcoded map
including `mCherry→3` and `Bright Field→5`. **Rewrite** it into "the display hint is not a hardware
mapping" — do not delete it; silently dropping an assertion is how a guarantee disappears.

**Mutations each key test must catch:** drop the read-back · stale-drop retried instead of aborted ·
`"reserved by"` treated as benign · `wait_idle()` substituted for per-op `.error` · revert to the
case-sensitive `dict.get` · guess on ambiguity · a live-manager µm/px fallback for an uncalibrated
objective · rotate before retreating · bound by the **source** objective's WD (±4100 vs ±250) · a
fabricated default for unknown WD · `offset[to]` instead of `offset[to] − offset[from]` ·
`product_code` ignored · restore objective-then-focus · fall back to `CHANNEL_NUMBERS` ·
fuzzy-match mCherry→TxRed · accept `present=False` · skip `refresh_mounted()` · LED raised before
the cube · entry LED/exposure restored only on success · objective verified once instead of per
channel · **the automatic switch writes `current_objective_name` "so every surface knows"** (the
mutation most likely to be proposed in good faith) · needle/print gate dropped.

Regression per stage: `test_v711_objective_ladder` · `test_v711_plate_level_worker` ·
`test_v711_plate_level_wizard` · `test_v711_focus_sweep_planner` · `test_v711_objective_optics` ·
`test_v75x_nikon_ti_microscope` · `test_v75x_fluorescence_mosaic` · `test_v75x_illumination_led` ·
`test_v74x_objective_calibration` · `test_v75x_camera_scale_fov_and_registration` · plus a
`gui.app` import smoke and an offscreen `HardwareSetupPage` / `CalibrationPage` build.

---

## Issues & Decisions

### B0 — a machine-bucket stranding seen mid-session, which then RESOLVED ITSELF. Observation only; no code change, and I did not cause or fix it.

While verifying B1 I found both optics stores sitting in `config/hardware/unassigned/` while
`config/machine_id.txt` said **`ME3B_01`** — so at that moment the app saw an **empty** objective
library, no calibrated cameras and no slot labels at all, with the operator's real 2026-08-11
measurements invisible. Recorded here because it is a real failure mode and the reason it is
possible is still in the code.

A later probe found `unassigned/` gone and every file consolidated into `ME3B_01/` with
**unchanged mtimes** (`microscope.json` 1018 B @ 11:34:35, `objectives.json` 935 B @ 8/11 16:27) —
i.e. moved, not rewritten — plus a fresh `calibration_status.json` write at 13:28. So the app (or
`tools_migrate_machine_config.py`) ran during the session and consolidated it. **Not attributable
to this work:** nothing here touches `MachineConfig`, and the only paths my probes exercised were
the flat legacy root (absent) and explicit paths. Current state, verified:

```
machine_id          : ME3B_01          objectives/microscope default paths: exists= True
objective library   : ['4x', '10x', '20x']      calibrated cameras : ['tucam:0']
objective labels    : {1: '4X', 2: '10X', 3: '20x'}
filter labels       : {1: 'DAPI', 2: 'FITC', 3: 'TxRed', 4: 'Cy5'}
```

**This is what makes B1 live rather than historical:** on the ACTIVE path the nosepiece labels are
`4X`/`10X` while the calibration keys are `4x`/`10x`.

⚠ The underlying gap is unfixed and worth its own change: `MachineConfig._migrate_once` only ever
adopts from the legacy **flat** `config/hardware/<name>` location, never from another machine
bucket, so a store that lands in `unassigned/` before an id is set is stranded until something
moves it by hand. Deliberately **not** "fixed" by teaching it to adopt from `unassigned/` — on a
checkout where that bucket came from a *different* physical rig, silent adoption would import
another machine's calibration, exactly the collision `MachineConfig` exists to prevent. A visible
"your config is in the wrong bucket" prompt is the right shape, and is not this update's scope.


- **Normalize at lookup only; do NOT migrate the stores.** The objective name also keys
  `parfocal_offsets_um`, the fluorescence `_align_key`, mosaic alignment records and `.nd3`
  sidecars already on disk. Renaming a calibration key is the v7.16 class of change that destroyed
  a good calibration. A "Check names" report offers to rewrite the *slot label* in
  `microscope.json` — the cheap, per-machine, non-measurement store — never the other direction.
- **Refuse rather than guess on names.** Tiers are `exact` → `normalized` → operator `alias` →
  body `native`, each of which must resolve to exactly ONE slot. Never substring, prefix,
  edit-distance or magnification-digit parsing. `NikonTiSdkBackend._set_turret` already documents
  the SDK clamping slot 999 → 6 and **reporting success**; a fuzzy name match is that failure one
  layer up, and a TxRed image filed as mCherry is a result nothing downstream can detect.
- **`done` is not success, and `.error` cannot see an ack-without-moving.** Every op checks
  `.error`, `dropped (stale)` aborts rather than retries, and the position is **verified by
  read-back**. `wait_idle()` must never substitute — it returns True on a drained queue of failed ops.
- **Parfocal offsets are reference-relative:** the delta is `offset[to] − offset[from]`, not
  `offset[to]`. Getting this wrong is a full-magnitude focus error that still looks plausible.
- **The objective is the collision hazard, not the needle.** The focus drive raises the objective
  toward the plate; rotation does not move Z, so a height comfortable under the 4x (WD 16.4 mm) can
  be inside a 20x's front lens (~1 mm) at the instant of rotation, with no move to abort. Focus
  retreats first, bounded by the **target** objective's `0.25 × WD`.
- **The service never commands the stage** — travel stays with `safe_travel_to` /
  `ensure_retracted_to`, and it never calls `set_print_floor_active` (the plate-level precedent:
  arming a refcount it cannot need risks an unbalanced decrement against a concurrent print).
- **Filter-only automation in the fluorescence workflow.** The run freezes `_scan_positions` /
  `_scan_um_per_px` once and replays them for every channel, so a mid-run *objective* change leaves
  tile spacing not matching FOV — the v7.16 seams / 400-minute-scan class. The objective is frozen
  and verified per channel, never commanded.
- **Simulated is not real.** A simulated switch reported as real is a fabricated fact, so
  `simulated=True` reaches the operator and a real capture still prompts.

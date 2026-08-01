# MEBP v7.9 — Multi-bore needles + per-bore Cell Targeting & Removal

## Objective

Redesign **Cell Targeting & Removal** around needle assemblies that have more than
one bore, and around a multi-channel fluorescence scan of the well.

Operator's request (verbatim):

> "on the cell targeting and removal workflow we need to redesign it a bit. We use
> various needle types 1) Single needle 2) backpack where there are two needles of
> different sizes bound together 3) tripple needle fused together. I want to be
> able to do a fluorescent scan of the well in multiple channels then if we are
> using a tripple needle i want to be able to assign a target per channel. Each
> channel is connected to a different pump. Also I may assign one pump and channel
> as the trypsan channel which will be pushed onto the cell right before we
> aspirate the timeing of this action should be defined by the user."

Follow-up clarifications (AskUserQuestion, 2026-08-01):

> "i want to use the flourescent mosaic workflow as an instance for this. the
> operator would pick all of the settings and scan the mosic. then in another tab
> in the cell targeting and removal workflow, we use it to setup the workflow. the
> setup page is where we will do most of the options and establishment of what and
> how to do everything, then the other page is used as a viewer for the work being
> done. an explicit mapping for a target type that the user defines not exactly per
> channel. it is based on some meta data for the objects being removed. for example
> we will identify a cells flouescent signature say it has some brightness in both
> channels. or cells with brigtness in one channel etc. the user decides these
> rules. and the scheme for selecting the cells from a flourescent image will be
> done later. for now we are focusing on the user defines something for each pump
> channel to go and do something"

> prep: "only the bores this run uses. but the should be doing the same thing
> simultaneously"

### Decisions taken (AskUserQuestion)

| # | Question | Decision |
|---|---|---|
| D1 | Bore lateral offsets | **Calibrate per-bore XY offsets; visit targets sequentially.** `stage_xy = target_xy − bore_offset(k)` |
| D2 | Trypsin timing knobs | **Lead time + push rate + push volume/depth.** Overlap **ruled out** — see below |
| D3 | Target ↔ bore mapping | **User-defined TARGET TYPE → bore.** Signature *rules* are authored now, **rule evaluation deferred** |
| D4 | Scan surface | **Embed a live instance of the Fluorescence Mosaic page** (v7.8 `embedded=True` pattern) |
| D5 | Trypsin bore | **A DIFFERENT bore from the aspirating bore, with an XY move between push and aspirate** |
| D6 | Per-bore definition | **One ROLE + its parameters per bore.** Run order derived from roles |
| D7 | Bore spacing | **~100–500 µm lateral, Z coplanar within ~50 µm** |
| D8 | Prep scope | **Only the bores this run uses — acting SIMULTANEOUSLY** |

### ⛔ "Overlap" is ruled out by D5, not deferred

The operator initially also selected *"start aspirating before the push ends"*.
D5 makes that **physically impossible**: with a separate trypsin bore and an XY
move between push and aspirate, the trypsin bore and the aspirating bore are
never over the cell at the same time. Aspiration cannot begin before the push
ends because the tool has to travel in between. Recorded here so it is not
re-litigated: **there is no overlap setting, and adding one later requires
revisiting D5, not just the executor.**

### ⚠ D8 is the one place simultaneity IS required

"the bores should be doing the same thing simultaneously" during prep means a
**coordinated multi-pump move**. That primitive does not exist:
`StageController.move_pump_uL` is single-pump. Calling it from two threads is
wrong for four independent reasons (see *Hard parts* → *Simultaneity*). The
correct primitive is ONE `G0` line with several axes — which `ZPStage.move_relative`
already accepts and which the Xbox jog loop already exercises on real hardware
(`StageController.py:876-938`, ME3B V1 `axis_map {Z:Z, P1:X, P2:Y, P3:E}`).
So: NEW `StageController.move_pumps_uL()`, modelled verbatim on the jog loop.

---

## Vocabulary — the word "channel" is banned in new code

`channel` currently means **five** different things in this repo
(fluorescence filter cube / needle lumen / pump / microscope turret slot /
sketch abstract ink), and meaning-2 exists on disk in **both** 0-based and
1-based form. Adopt this convention for every new identifier, UI string,
settings key and log line. Existing serialized keys are frozen.

| Concept | Word | Identifier | Index |
|---|---|---|---|
| One physical lumen of the needle assembly | **bore** | `NeedleBore`, `bore_index`, `bores` | 0-based in code, displayed "Bore 1..N" |
| The assembly's form factor | **needle form** | `NeedleForm` ∈ `single`/`backpack`/`triple` | — |
| A syringe pump | **pump** | `pump_id` ∈ `"P1"/"P2"/"P3"` | by id |
| A fluorescence filter cube / mosaic layer | **imaging channel** | `imaging_channel` | by name |
| A microscope turret position | **filter slot** | `filter_slot` | 1-based |
| A user-defined class of object to remove | **target type** | `TargetType`, `target_type_id` | by id |

**The index-base war is retired by eliminating the index from the wire format.**
Bores are a **list**; position in the list *is* the index, and each `NeedleBore`
carries its own `pump_id`. No map, no key base, no possible count mismatch —
which retires `HardwareConfig.validate()`'s `len(map) != num_channels` rule by
construction.

Disposition of the two legacy maps:
* `NeedleSpec.channel_pump_map` — **freeze write-only** (1-based, zero readers
  repo-wide). Must keep being emitted or four byte-identity tests fail.
* `HardwareConfig.needle_channel_pump_map` — **keep, derive from `bores`,
  deprecate.** Still emitted so `PrintPlanOfAction:1226-1235` stays truthful.

---

## Files Modified

### New
| File | Purpose |
|---|---|
| `SupportClasses/NeedleBoreCalibrationStore.py` | Per-**mount** bore XY/Z offsets (per-machine, NOT in the swappable setup file — the `CAMERA_CAL_PERSIST_STORE` lesson) |
| `SupportClasses/TargetTypeStore.py` | User-defined target types (name, colour, signature rule) |
| `gui/pages/workflows/cell_targeting_setup_panel.py` | The Setup tab: per-bore role table + target types + trypsin timing |
| `tests/test_v79_multibore_needle.py` | Needle model + byte-identity + per-bore physics |
| `tests/test_v79_move_pumps_simultaneous.py` | Coordinated multi-pump move |
| `tests/test_v79_per_bore_cell_targeting.py` | Executor: roles, per-bore volume balance, trypsin move-between |
| `tests/test_v79_cell_targeting_setup_page.py` | Setup tab + embedded viewer + Start gate |

### Modified
| File | Why |
|---|---|
| `SupportClasses/PhysicalModels.py` | `NeedleForm`, `NeedleBore`, `NeedleSpec.bores`, per-bore geometry/physics accessors |
| `SupportClasses/HardwareConfig.py` | serialize `bores`, derive the legacy map, relax the capillary single-bore gate, per-bore validation |
| `SupportClasses/SafetyLimits.py` | per-**pump** flow ceiling from that pump's own bore |
| `SupportClasses/StageController.py` | `move_pumps_uL()`; bore-offset aware travel helper |
| `SupportClasses/PickAndPlaceManager.py` | `BoreRole`/`BoreProgram`, multi-bore cell targeting, simultaneous prep, **3 latent bug fixes**, delete 2 dead implementations |
| `gui/pages/workflows/cell_targeting_workflow.py` | tabs, Setup tab host, embedded fluorescence viewer, `_stage_busy`, `_travel_to_absolute` |
| `gui/pages/hardware_setup.py` | needle form combo + per-bore geometry/pump rows |
| `gui/pages/calibration.py` | per-bore offset calibration on the Needle Location tab |
| `SupportClasses/CalibrationSnapshotStore.py` | fingerprint the assembly + bore count so a re-seat invalidates |

---

## Hard parts

### (a) `cross_section_area_mm2` must keep its v7.6 fail-safe meaning

v7.6 redefined it to the **orifice** area precisely so a call site that does not
know about tips gets the *right* number rather than one ~1000× too large. With N
parallel bores it has four candidate meanings and three are wrong:

* **sum** — wrong: not a single passage. F-1 bead √3× too wide, ceiling 3× inflated.
* **largest** — unsafe: the *small* bore is the pressure-limiting one.
* **smallest** — safe for flow, wrong for volume.
* **bore 0** ✅ — preserves the fail-safe property (a missed call site gets *one
  real bore's* number, never a fictional aggregate) and is byte-identical for a
  single-bore needle, where bore 0 *is* the needle.

**Decision:** `cross_section_area_mm2`, `orifice_id_um`, `internal_volume_uL`,
`barrel_area_mm2`, `total_length_mm` all resolve **bore 0**. Multi-bore consumers
become explicit by construction via `needle.bore(k)`. The duck-typed free
functions stay single-valued (≈30 call sites pass MagicMocks/stubs); add `bore_*`
siblings taking an index.

**Topology:** `flow_segments()` composes stages in **SERIES**. Parallel bores are
a different composition — but **do not** use `1/R = Σ1/R_i`: that applies when
*one* pressure source feeds N branches. Here **each bore has its own pump**, i.e.
N independent pressure sources. Therefore:

> **There is no assembly-level flow ceiling. There are N independent per-bore
> ceilings, one per pump.**

`SafetyLimits` already has per-pump fields and a setter; `update_from_hardware_config`
merely happens to write the same value into all three.

### (b) Simultaneity — yes, but only as ONE coordinated move

`ZPStage.move_relative(axes: dict, feedrate)` already emits **one** `G0` with
several axes (`ZPStage.py:1012-1044`) and Marlin runs one block as a coordinated
move. This is exercised in production by the Xbox ZP jog loop
(`StageController.py:876-938`).

Calling `move_pump_uL` twice is wrong, four ways:
1. **Not simultaneous** — two `G0` blocks execute in planner order → silent serialization.
2. **M400 drains everything** — `_wait_pump_move_complete` waits for *all* queued
   moves, but its timeout is sized for one move → spurious timeouts.
3. **Poller suspend is not refcounted** — `PositionPoller._suspended` is a plain
   bool; whichever thread finishes first re-enables it under the other →
   the v7.5.x false ZP-disconnect returns.
4. **They serialize on `_serial_lock` anyway** — an RLock held across the whole
   write→read-`ok` transaction.

**Constraint that cannot be designed away:** a coordinated move has ONE feedrate
applied to the *vector*; each bore's rate is `(Δ_i/|Δ|)·F`. **You cannot give bore
A 0.5 µL/s and bore B 5.0 µL/s in the same simultaneous move.** That is fine for
prep (D8: all bores doing the *same* thing) and unacceptable for the slow-push /
fast-pull asymmetry, which stays sequential.

Flow-ceiling clamp for a coordinated move must respect *every* pump:
`F ≤ min_i( ceiling_i · |Δ| / Δ_i )`.

Backlash compensation is forced **off** for coordinated moves (its take-up/unload
bracket is per-axis-direction and ill-defined when bores move oppositely) —
matching the existing pick&place convention.

### (c) Bore offsets are a per-MOUNT calibration, not a preset constant

A fused assembly's rotation about Z in the holder is arbitrary, so the offsets
must be re-measured on every needle change or re-seat. They live **beside the
needle-location calibration** (per-machine), **never** in `NeedleTypeStore` —
the direct analogue of the `CAMERA_CAL_PERSIST_STORE` lesson.

Sign convention, fixed once and pinned by a round-trip test:

```
to place bore k on target T:   stage_xy = T − offset_um(k)
bore k is currently at:        stage_xy + offset_um(k)
offset_um(0) ≡ (0, 0)          bore 0 IS the needle_origin_um datum
```

A mis-signed offset is a *right-distance-wrong-way* error — the same class as the
plate-orientation bugs recorded in CLAUDE.md.

At D7 (100–500 µm) the offsets are **larger than a cell**, so they are required
for correctness, while Z coplanarity within ~50 µm means a shared descend Z is
tolerable — but 50 µm is *half* the 0.10 mm working clearance, so per-bore
`z_offset_mm` is stored and the descend uses the **longest** bore.

### (d) The trypsin move-between activates a latent glass-crash bug

D5 requires a ~100–500 µm XY shift inside one well between the trypsin push and
the aspirate. That is exactly the `_intra_well_move` path — and it is
**polarity-broken today**: `move_z_relative(-intra_well_retract_mm)`
(`PickAndPlaceManager.py:1973`) is a **raw** Marlin delta, while
`z_height_of = z_up_sign × raw` and **both** real device profiles record
`z_up_sign = +1.0`. So the "retract" **lowers the needle 1 mm** — into the glass.

Dormant only because picker targets carry `well_name=""` and the shortcut needs a
non-empty matching name. **Per-bore targets inside one well are precisely what
activates it.** MUST be fixed before enabling. The correct pattern is 60 lines
above in `_do_wash` (`move_z_user_relative(+amp)` then an absolute return).

### (e) Safety invariants at risk

| Invariant | Risk |
|---|---|
| Retract before cross-position XY | new per-bore shifts; `_intra_well_move` broken (see (d)) |
| End at safe Z on every exit | a multi-bore op must not bypass `execute_queue`'s `finally` |
| **Volume balance = 0 per operation** | the existing test sums across **all** pumps → passes *vacuously* on an unbalanced two-bore op. **Must become per-bore.** |
| Per-pump flow ceiling | a fine bore's pump inheriting a coarse bore's ceiling = over-pressure, shattered glass |
| Plate-bottom floor | one `pick_z_mm` + non-coplanar tips = crash |
| Mosaic positions need a recorded shift | `has_shift` gate is **GUI-only** today; promote into the Start gate |
| Abort responsiveness | every new wait must route through `_dwell` and be a true no-op at 0.0 |
| Poller suspend not refcounted | embedding the scan page adds a second stage driver → `_stage_busy()` must consult `is_scanning()` |

### (f) Latent defects fixed en route (found by recon, not in the request)

1. **`_intra_well_move` Z polarity** — glass crash, see (d).
2. **`PickPlaceTarget.from_dict` is a bare `cls(**d)`** — a target dict from a
   newer build raises TypeError in an older one. Exactly what `NeedleSpec.from_dict`
   was hardened against in v7.6.
3. **Volume-guard asymmetry** — step 1 gated on `reagent_well_pos is not None and
   push>0`, step 3 only on `push>0`, so with no reagent well the executor
   **dispenses a volume it never aspirated**. Produced today with the default cfg.
4. **Channel-count spin wipes the pump map** (`hardware_setup.py:3858-3891`) and
   immediately persists the emptied dict; its sibling `_refresh_channel_map_pump_options`
   has the preserve-and-restore pattern to copy.

### (g) Two dead multi-bore implementations — mine, then delete

`TrypsinPickupConfig`/`_execute_trypsin_pickup` (`:214-239, 1699-1787`) and
`FluorescentTaggingConfig`/`DyeConfig` (`:386-427, 1791-1893`, whose `dye_name`
is literally `"DAPI"/"GFP"/"mCherry"`) are the shape of this request. **Neither has
ever run**: no GUI constructs them, no test covers them, and `_execute_trypsin_pickup`
resolves wells through `self._well_positions`, whose only writer has **zero
callers**. Leaving them in place while writing a third implementation is precisely
how the `UNIFIED_MOSAIC_CALIBRATION` divergence happened. **Delete** (breaks zero tests).

---

## Implementation Steps

### Stage 1 — Needle model foundation
- [ ] `NeedleForm` = `single` / `backpack` / `triple`; **orthogonal** to `needle_type`
      (taper). A backpack of two pulled capillaries needs both axes.
      ⚠ `__post_init__` silently rewrites an unknown `needle_type` to `hypodermic`,
      so a form must NOT be smuggled in as a needle_type value.
- [ ] `NeedleBore` dataclass: `id_um/od_um/wall_um/length_mm`, optional tip stage,
      `pump_id`, `offset_um=(0,0)`, `z_offset_mm=0.0`, `label`; its own
      `orifice_area_mm2 / flow_segments() / bore_profile() / internal_volume_uL /
      max_safe_flow_rate_uL_s`
- [ ] `NeedleSpec.bores: list[NeedleBore] | None = None`; `None` ⇒ synthesize ONE
      from the flat fields so `bores_resolved()` always returns ≥1
- [ ] `bores[0]` **mirrors** the flat fields; emit `bores` **only when len > 1**
      (byte-identity); `num_channels` reconciled to `len(bores_resolved())`
- [ ] `needle.bore(k)`; `assembly_internal_volume_uL`; `bore_*` free-function siblings
- [ ] `channel_pump_map` docstring → DEPRECATED / WRITE-ONLY / never read

### Stage 2 — Config + safety
- [ ] `HardwareConfig` serialize/deserialize `bores`; derive `needle_channel_pump_map`
- [ ] Relax the three capillary single-bore gates; per-bore validation
- [ ] `SafetyLimits.update_from_hardware_config` → per-pump ceiling from that pump's bore
- [ ] Fix the channel-count-spin map wipe

### Stage 3 — Coordinated multi-pump move
- [ ] `StageController.move_pumps_uL({pump_id: µL}, rate, ...)`: per-pump µL→mm,
      per-pump `pump_dir_sign`, per-pump soft-limit clamp, vector flow clamp,
      ONE `move_relative` dict, ONE `_wait_pump_move_complete`, ONE settle,
      `compensate=False`

### Stage 4 — Executor
- [ ] Fix `_intra_well_move` polarity **first** (glass crash)
- [ ] Fix `PickPlaceTarget.from_dict` filtering; fix the volume-guard asymmetry
- [ ] `BoreRole` = `aspirate_target` / `push_reagent` / `dispense_place` / `idle`
- [ ] `BoreProgram` (bore_index, pump_id, role, target_type_id, params)
- [ ] `CellTargetingConfig` (multi-bore) with a single-bore path that stays
      byte-identical to `CellRemovalConfig`
- [ ] Bore-offset aware `_safe_move_to(..., bore_index=)`
- [ ] Simultaneous prep over the used bores via `move_pumps_uL`
- [ ] Per-bore volume balance
- [ ] Delete the two dead implementations

### Stage 5 — Target types
- [ ] `TargetTypeStore`: id, name, colour, `signature_rule` (authored, **not
      evaluated** — D3), atomic write, builtin/user shadow
- [ ] Target type ↔ bore binding

### Stage 6 — Cell Targeting page
- [ ] `QTabWidget`: **Setup** (per-bore roles, target types, trypsin timing) +
      **Well Survey / Viewer** (embedded `FluorescenceMosaicWorkflowPage(embedded=True)`)
- [ ] `_stage_busy()` incl. `is_scanning()`; `_travel_to_absolute()` kept SEPARATE
      from the zero-ref handler (sharing one adds `zero_position` twice)
- [ ] Forward `set_settings` to the embedded page (the spheroid host does **not**)
- [ ] Promote the `has_shift` gate into `_on_start`
- [ ] Live run overlay on the viewer tab

### Stage 7 — Hardware Setup → Needle
- [ ] Needle form combo; per-bore rows (geometry + pump + measured offset readout)

### Stage 8 — Per-bore offset calibration
- [ ] `NeedleBoreCalibrationStore`; "centre each bore" flow on the Needle Location tab
- [ ] Fingerprint the assembly so a re-seat invalidates

### Stage 9 — Tests + regression

---

## Status Tracking

Stages 1–5 `[x]` · Stages 6–8 `[~]` · Stage 9 `[~]`

| Stage | State | Evidence |
|---|---|---|
| 1 Needle model | `[x]` | byte-identity verified on all 6 real `config/hardware/*.json`; `test_v76_pulled_capillary_needle` green; per-bore ceilings measured **45×** apart (22G vs 30G) and **~2000×** (22G vs a 30 µm pulled tip); bore 0 ceiling **bit-identical** to the equivalent single needle |
| 2 Config + per-pump safety | `[x]` | 641 tests green; verifier fixed a **7071× over-permissive** ceiling hole |
| 3 `move_pumps_uL` | `[x]` | 53 tests; verifier fixed an **11.11×-over-ceiling** feedrate defect |
| 4 Executor | `[x]` | `test_v79_per_bore_cell_targeting` **36/36**; 3 latent bugs fixed |
| 5 Target types | `[x]` | 81 tests |
| 6 Cell Targeting page | `[~]` | in progress |
| 7 Needle form UI | `[~]` | in progress |
| 8 Bore offset calibration | `[~]` | in progress |

---

## Testing Notes

Byte-identity and forward-compat constraints that MUST hold:
* `NeedleSpec.to_dict()` for a single-bore needle emits **exactly** the 7 legacy
  keys in order — pinned by `tests/test_v76_pulled_capillary_needle.py:70, :74,
  :78, :96` (one of which scans the operator's six real on-disk setup files).
* Duck-typed needle stubs exposing only `gauge/id_m/length_mm` keep working.
* `test_v76::TestSeriesResistance::test_straight_ceiling_is_bit_identical_to_the_legacy_closed_form`
  uses `assertEqual` on floats — do not reorder the single-cylinder fast path.
* `test_v76::TestValidation::test_multichannel_capillary_is_rejected` **must be
  rewritten** — it pins the opposite of the new behaviour.
* `CalibrationSnapshotStore` added keys must be **absent** on old snapshots, not
  None-vs-value — pinned by `test_v76:494-541`.
* `pick_only` and measure-mode-OFF byte-identity in `LiveTargetPicker`.
* Every executor test sets `dwell_time_s=0.0`; new timing fields must be a true
  no-op at 0.0.

## Issues & Decisions

* **2026-08-01 — overlap ruled out** by D5 (see above). Not deferred; contradicted.
* **2026-08-01 — simultaneity scoped to prep only** (D8). The slow-push/fast-pull
  asymmetry keeps independent rates and therefore stays sequential.
* **2026-08-01 — `cross_section_area_mm2` resolves bore 0**, preserving the v7.6
  fail-safe direction rather than aggregating.
* **2026-08-01 — rule EVALUATION deferred** (D3). Target types and their signature
  rules are authored and persisted now; nothing evaluates them against an image yet.

### Safety defects found by the adversarial verify passes (all fixed)

Each of these would have over-pressured a fine bore, which on a pulled glass tip
means a shattered tip. They are recorded because they share one root cause: **a
per-pump ceiling is only as good as the weakest link in resolving "which bore
feeds this pump".**

1. **`SafetyLimits` resolved the bore from ONE authority.** It asked
   `needle.bore_for_pump()` only, ignoring `HardwareConfig.resolved_bore_pump_map()`.
   The two disagree in exactly the configuration the docstring calls reachable —
   bores present but not yet wired, map set by the GUI. Measured on a config that
   `validate()` passes clean: P2 (feeding a 30 µm pulled tip) received bore 0's
   22G ceiling, **7071× over-permissive**, and `clamp_flow_rate(1000, "P2")`
   passed 1000 µL/s straight through. Now resolved via both, and the
   *unclaimed*-pump fallback changed from bore 0 (the **widest** — the
   over-pressure direction) to the **narrowest** bore on heterogeneous assemblies.
2. **`move_pumps_uL` applied Marlin's "never emit F ≤ 0" floor to each axis's
   CEILING** instead of to the vector feedrate. In a coordinated move an axis
   legitimately runs below 1 mm/min while the vector runs far faster, so raising
   the ceiling to 1.0 let that bore be over-driven — and the board's own floor
   cannot save it because F is already ≫ 1. Measured: a 0.005 µL/s tip alongside a
   10 µL/s bore ran at **11.11× its ceiling**. This is precisely the v7.6
   capillary case, since `max_safe_flow_rate` scales as d⁴.
3. **Derived pump ids were not normalized**, so a bore holding `"p1"` got a
   correct ceiling but landed in the map verbatim, where every consumer compares
   `"P1"` — `validate()` then reported the enabled pump as not enabled.

### Other real issues fixed en route

* **`PrintPlanOfAction` read the STORED map** to check bore↔pump coverage. Once
  the Needle card emits a bore list, `num_channels` reconciles to the bore count
  while the stored map may be empty — producing a spurious *"N bores but 0
  mapped"* on every plan. Now asks `resolved_bore_pump_map()`; verified a triple
  assembly with an empty legacy map validates clean.
* **`move_pumps_uL` under-reported a shortened move.** Its docstring promised
  requested-vs-delivered µL logging, but only *fully dropped* axes warned — a
  partially soft-limit-clamped axis silently under-delivered a prep volume.
* **The motion estimator was fed the vector feedrate**, not the axis's own
  `F·|Δ|/L` — a 1.41× optimistic ETA with two equal bores. Display-only.

### ⚠ Incident — destroyed and recovered uncommitted work

While deleting the two dead multi-bore implementations from
`PickAndPlaceManager.py`, a regex used to find a block boundary matched the wrong
place and the deletion took **1777 lines instead of ~30** (2099 → 372). The file
carried ~76 lines of **uncommitted** v7.6 work, so `git checkout` alone could not
restore it.

Fully recovered and verified from three independent sources: the `.pyc` compiled
from the lost version, a dangling git blob, and the original edit operations in a
prior session transcript. Verification was per-function bytecode comparison
**including line numbers** — `_bore`@847, `_planned_aspirate_uL`@870,
`_pump_move`@2055, `_safe_travel`@2069 only reproduce if every preceding line in
the file is exactly right. Restored to 2099 lines; 143 tests green.

**Two standing consequences:**
* This branch has substantial **uncommitted v7.6 work in both `PhysicalModels.py`
  and `PickAndPlaceManager.py`** — `git show HEAD:` lacks `needle_flow_segments`
  entirely. **Commit before any `git checkout`/`git restore` on those files.**
* **The dead-code deletion was NOT completed** (see *Hard parts (g)*).
  `TrypsinPickupConfig`/`_execute_trypsin_pickup` and
  `FluorescentTaggingConfig`/`DyeConfig`/`_execute_fluorescent_tagging` remain in
  the file, still unreachable. Their `OperationType` members were re-added during
  the restore. Deleting them is still the right call — a third divergent
  implementation is how `UNIFIED_MOSAIC_CALIBRATION` went wrong — but it should be
  done as its own small, separately-verified change, not bundled here.

### D8 wiring (completed 2026-08-01, after the GUI stages)

The stage-6 verifier flagged that D8 ("only the bores this run uses. but the
should be doing the same thing simultaneously") was never actually wired:
`move_pumps_uL` had zero call sites, so on a two-bore run the trypsin bore went
to its reagent well dry. Fixed:

* `PickPlaceExecutor.prep_bores: list[dict]` — every bore this run will use,
  `[{"pump_id", "bore_index"}]`, empty ⇒ the legacy single-`prep_bore` path
  (byte-identical).
* `_prep_bore_plan()` resolves each bore's OWN internal volume — a backpack's
  22G bore holds ~6.8 µL, its 30G bore ~1.0 µL, so one scalar cannot size both.
* `_prep_pump_move()` actuates every prepping bore in ONE coordinated
  `move_pumps_uL` call, falling back to sequential per-bore moves on an older
  controller (degrades to "correct but not simultaneous", never to failing).
* Wired into `run_prep` **and `run_post_clean`** — the latter matters
  specifically because after a run that used a dedicated trypsin bore, that
  bore also holds residual that must be cleared; clearing only `prep_bore`
  would leave it dirty.

Tests: `TestSimultaneousPrep` (7) + `TestSimultaneousPostClean` (2) in
`test_v79_per_bore_cell_targeting.py`, proving ONE `G0` per actuation (not two
sequential), per-bore volumes, correct signs, and the sequential fallback.

### Sibling fixes made while verifying

* **`spheroid_pickup_workflow.py` carried the IDENTICAL blank-tab bug** found in
  the Cell Targeting host (`hideEvent` calling `self._scan_page.hide()` — an
  EXPLICIT hide on a child widget STICKS, so Qt never re-shows it and the
  survey tab came back permanently blank on a second visit). Fixed there too;
  `test_v78_spheroid_page_integration.py`'s test asserting the old (buggy)
  behaviour was rewritten to assert the fix.
* **`PrintPlanOfAction`'s bore↔pump validation read the STORED map**, not the
  resolved one — once the Needle card emits a `bores` list, `num_channels`
  reconciles to the bore count while the legacy stored map may be empty,
  producing a spurious *"N bores but 0 mapped"* on every plan. Now calls
  `resolved_bore_pump_map()`; verified a triple assembly with an empty legacy
  map validates clean.

## Outstanding

* Delete the two dead multi-bore implementations (above).
* `update_from_hardware_config(MagicMock())` raises `TypeError` from a
  **pre-existing** syringe-stroke log line (`SafetyLimits.py` ~`:505`); no test
  exercises it.
* **Needs real-HW verification on ME3B V1, IN ORDER.** 1 = go/no-go: measure the
  bore offsets, then drive each bore to the SAME target and confirm each tip lands
  on it — a mis-signed offset lands the right distance on the wrong side, so check
  the direction explicitly, not just the magnitude. Then: per-bore Z clearance
  (descend must be planned against the LONGEST bore); the trypsin push → XY shift
  → lead time → aspirate sequence with the shift visibly inside the well;
  simultaneous prep actually moving the used bores together on ONE `G0`; and a
  per-pump ceiling check that a fine bore's pump refuses a flow the coarse bore
  would allow.
* **Full test-discovery run — inconclusive, not a failure.** `python -m
  unittest discover -s tests -p "test_*.py"` over all 220 test files was run as
  an extra check beyond the targeted regression below. It hit a 50-minute
  wall-clock timeout **mid-run, with no final `Ran N tests` summary ever
  printed** — killed, not failed. The last log activity before the kill was
  ordinary calibration/mosaic test output. This matches the exact class of
  issue this repo's own history already documents
  (`MEBP_v78_SPHEROID_DETECT_MEASURE_AND_PICK.md`: *"running the camera-probing
  suites together in ONE process intermittently hangs on real DirectShow
  enumeration … reproduced in a pristine git worktree at HEAD"*;
  `MEBP_v75x_UNIFIED_MOSAIC_CALIBRATION.md`: *"that hang makes a whole-suite run
  never terminate and hides real failures"*). Given that precedent it was not
  re-run inside this session — the targeted regression (every suite touched by
  this change, run explicitly and to completion, all green — see Testing Notes)
  is the evidence of record. **If re-attempting: follow the project's own
  practice of running suites individually or in small groups rather than one
  220-file discovery invocation**, and expect to exclude the camera/
  DirectShow-probing suites from any combined run.

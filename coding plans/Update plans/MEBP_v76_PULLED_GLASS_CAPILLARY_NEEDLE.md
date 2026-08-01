# MEBP v7.6 — pulled glass capillary needles (two-stage needle geometry)

## Objective

Operator: *"on the needle hardware setup page we need the option of pulled glass
capillary needles. these needles should have an inner diameter and length for the bulk
needle then a tip diameter and length where we pull the tip to a certain diameter. These
two pieces of information are used in different ways: 1) calculation of max flow rate (no
longer a constant diameter and length), 2) print line bead diameter, 3) spheroid size we
can pick up, 4) the needle volume for print ink pickup."*

`NeedleSpec` modelled one straight cylinder — a gauge from `needles.json` plus a length
combo — and ~30 call sites across flow physics, print geometry, pick & place and the
workflow GUIs read that assumption. A pulled capillary is a **bulk barrel** (D1, L1)
feeding a **pulled tip** (D2, L2), where D2 is typically 10–100 µm against a ~1 mm
barrel, so every one of the four consumers above was wrong for it.

## Decisions (AskUserQuestion)

| Decision | Choice |
|---|---|
| Tip shape | **Both, selectable per needle** — `tip_profile` ∈ {cylinder, cone}, default cylinder |
| Needle volume | **Total (barrel+tip) for prep multiples; tip-only for the Quick Print ink reserve** |
| Spheroid too big for the tip | **Warn, never block** |
| Preset storage | **Free-form spin boxes + a savable `NeedleTypeStore` preset library** |

**Convention:** total needle = barrel + tip, additive. `length_inches` keeps meaning the
**barrel** (a capillary stores `mm / 25.4`) so `length_mm` and every barrel-length
consumer work untouched; `total_length_mm = length_mm + tip_length_mm`.

## Files Modified

| File | Why |
|---|---|
| `SupportClasses/PhysicalModels.py` | `NeedleSpec` two-stage fields + orifice/volume/segment properties; new `FlowSegment`, `BoreProfile`; getattr-tolerant free functions; conditional `to_dict` / filtering `from_dict` |
| `SupportClasses/FlowPhysics.py` | series-resistance `max_safe_flow_rate_uL_s` + `needle_pressure_drop_Pa` / `needle_hydraulic_resistance` / `limiting_flow_segment`; orifice-based Reynolds, wall shear, clogging |
| `SupportClasses/GeometryEngine.py` | bead volume + line spacing → orifice |
| `SupportClasses/SketchTrajectory.py` | bead volume + closure overlap → orifice |
| `SupportClasses/PickAndPlaceManager.py` | `bore_profile` + piecewise `_lift_for_uL` / `_uL_for_lift`; bore-column volumes → orifice |
| `SupportClasses/SpheroidSinkCalibrationStore.py` | tip provenance on `set_curve`; corrected docstring premise |
| `SupportClasses/HardwareConfig.py` | capillary `validate()` branch; `__repr__` no longer prints `NoneG` |
| `SupportClasses/CalibrationSnapshotStore.py` | `needle_type` / `needle_bore_um` / `needle_tip_length_mm` fingerprint dimensions + the added-key guard |
| **`SupportClasses/NeedleTypeStore.py`** | NEW — preset library (mirrors `WellTypeStore`) |
| **`config/hardware/needle_types/builtin/*.json`** | NEW — 3 bundled pull recipes |
| `gui/pages/hardware_setup.py` | needle-type combo, capillary card, preset row, rebuild/restore, max-flow label |
| `gui/pages/workflows/quick_print_workflow.py` | orifice area, orifice OD marker, `ink_reserve_volume_uL`, reserve warning |
| `gui/pages/print_builder_sketch.py` | bead width + line spacing → orifice |
| `gui/pages/workflows/spheroid_pickup_workflow.py` | `_bore_profile()`, executor injection, advisory fit warning |
| `gui/pages/workflows/spheroid_sink_calibration.py` | orifice bore + tip provenance in the saved curve |
| `gui/pages/workflows/cell_targeting_workflow.py`, `cell_labeling_workflow.py` | orifice OD + total length for the needle visuals |
| `gui/dialogs/workflow_settings_dialog.py`, `gui/pages/print_workspace.py` | two-stage needle readouts |
| `gui/pages/jog_control.py`, `gui/widgets/standard_jog_context.py`, `gui/widgets/context_sections.py` | **latent bugfixes** (see below) |
| `gui/pages/calibration.py` | deleted a dead needle-banner block |
| `gui/onboarding/wizard.py` | catalog-aliasing bugfix + capillary-aware first-run gate |

## The core design decision

Barrel dimensions keep their existing names (`id_um`, `od_um`, `length_mm`, …) because
those are the numbers the operator types and every legacy readout prints. But
**`cross_section_area_mm2` was REDEFINED to be the ORIFICE area**, with a new explicit
`barrel_area_mm2` for the one internal consumer that means the barrel.

That is deliberate and it is the fail-safe direction. Every external reader of
`cross_section_area_mm2` — the bead model in FlowPhysics / GeometryEngine /
SketchTrajectory / Quick Print, the bore-column volumes in PickAndPlaceManager, and the
four workflow `_bore_area_mm2()` helpers — means *"the bore the material passes through"*,
which is the tip. Had the legacy name kept barrel meaning, any call site missed in the
sweep would be silently wrong by ~1000× on a capillary. This way a missed site gets the
right number and the few that genuinely want the barrel say `barrel_`. For a straight
needle the two are identical, so nothing existing moves.

`orifice_*` aliases exist for readable new code, and duck-typing-tolerant free functions
(`needle_orifice_area_mm2`, `needle_flow_segments`, `needle_bore_profile`, …) are used at
every consumer because several call sites pass needle-LIKE stubs exposing only
`gauge`/`id_m`/`length_mm`.

## Implementation Steps

- [x] **Phase 0** — data model: constants, `FlowSegment`, `BoreProfile`, six new
  `NeedleSpec` fields with `__post_init__` normalisation, orifice/volume/segment
  properties, `spheroid_pickup_detail`, `summary_line`, conditional `to_dict`, filtering
  `from_dict`, free functions, `_particle_ratio` dedupe
- [x] **Phase 0 gate** — entire existing suite green with ZERO test edits
- [x] **Phase 1** — series resistance with a bit-exact single-cylinder fast path;
  Reynolds / wall shear / clogging switched to the orifice; `limiting_segment` +
  `tip_pressure_fraction` on `FlowSafetyResult`
- [x] **Phase 2** — bead + OD consumers switched to the orifice
- [x] **Phase 3** — piecewise sink timing, `bore_profile` injection, advisory fit check,
  tip provenance on the sink curve, tip-only ink reserve
- [x] **Phase 4** — Needle page UI + `NeedleTypeStore` + 3 bundled presets
- [x] **Phase 5** — validation, fingerprint, readouts, latent bugfixes
- [x] **Tests** — `tests/test_v76_pulled_capillary_needle.py` (69)

## Backward compatibility (the hard constraint)

A straight hypodermic needle is **byte-identical in serialization and float-identical in
physics** to pre-v7.6:

* `to_dict` emits exactly the seven legacy keys in their original order; a new key appears
  only when it differs from its default. Regression-locked against the real on-disk
  payload (including the JSON string key in `channel_pump_map`) and against every
  `config/hardware/*.json` needle block.
* `max_safe_flow_rate_uL_s` keeps a **single-cylinder fast path that evaluates the original
  expression in the original operand order**. This is not an optimisation — it is the
  byte-identity guarantee. `(P·π·d⁴)/(128·µ·L)` and `P/((128µ/π)(L/d⁴))` differ in the last
  ULP, and the ceiling is stored and compared downstream (`SafetyLimits.set_max_flow_rate`,
  Quick Print's speed chain, print records). The test asserts `assertEqual`, not
  `assertAlmostEqual`.
* `internal_volume_uL`, `cross_section_area_mm2` and `ink_reserve_volume_uL` all reduce to
  their pre-v7.6 values when there is no tip stage.
* `from_dict` now FILTERS unknown keys instead of `cls(**data)`. The old form raised
  `TypeError` on any unrecognised key, and since `settings["hardware_config"]` is a full
  `to_dict()` mirror restored inside a broad `except` in `gui/app.py`, an older build
  reading a capillary settings.json would have silently lost the entire hardware config.

## Testing Notes

`tests/test_v76_pulled_capillary_needle.py` — **69 tests, all passing, no skips**:

* **Serialization identity** — legacy payload round-trips byte-identically; exactly the
  seven legacy keys; every on-disk setup needle block round-trips; unknown keys tolerated;
  capillary round-trip omits unset optional fields.
* **Geometry / volumes** — orifice is the tip when pulled and the bore otherwise; volume =
  barrel + tip; ink reserve unchanged for a straight needle and tip-only for a capillary;
  `total_length_mm`; OD ratio fallback; non-positive tip dims collapse to no-tip.
* **Physics** — straight ceiling bit-identical to the legacy closed form; duck-typed stub
  resolves; the cone resistance factor reduces to the cylinder form at d1 == d2 (validates
  the 1/3 and the `d1²+d1·d2+d2²` numerator); the tip is reported as the limiting stage at
  >99% of the resistance; ceiling monotone in tip Ø; cylinder ≤ cone (conservatism);
  clogging classifies a 200 µm spheroid against a 30 µm tip as JAMMING.
* **BoreProfile** — exact inverses across the knee, continuity at `V == A2·L2`, lift
  flattens past the tip, single-area profile reproduces the legacy `V/A`.
* **Executor** — no bore ⇒ carrier volume with no wait (contract preserved); legacy scalar
  still drives the conversion; the two-stage profile plans far less volume than the barrel
  area would.
* **Feasibility** — too-large / tight / past-tip all warn, and nothing is ever severity
  `error` (operator decision: warn, never block).
* **Validation + fingerprint** — a capillary does not trip "No needle gauge selected" while
  a gauge-less hypodermic still does; tip-wider-than-barrel and multi-channel rejected;
  a dimension added after a snapshot produces NO diff (otherwise every existing user gets a
  bogus "needle bore: None → 210" on first launch); two capillaries with different tips do
  differ; a tip-length change is flagged.
* **NeedleTypeStore** — user shadows builtin, delete resurfaces the builtin, atomic write
  leaves no `.tmp`, `safe_id` sanitises, `matches` detects divergence, bundled builtins
  load, and an AST scan asserts the module stays GUI-free.
* **Hardware Setup round-trip** — straight needle unchanged; `length_inches=0.5` no longer
  coerced to 1.0"; capillary geometry + tip profile survive; type combo toggles visibility
  and clamps channels; info label shows both stages; max-flow label names the limiting
  stage; preset selection stamps geometry, provenance survives an exact match and is
  dropped once the geometry diverges.
* **Readout bugfix regressions** — all three fail on pre-v7.6 code.

Existing suites re-run green (bead model, extrusion volume, needle max flow, common axis
speed source, spheroid sink timing, sketch extrusion, two-param overlays, last-known
calibration, context panel, workflow settings, cell targeting/labeling, spheroid pick &
place).

## Issues & Decisions

**The tip-only ink reserve needed a guard.** The operator chose tip-only for the Quick
Print dead volume, but a 30 µm × 5 mm tip holds ~3.5 nL — far below one pump step — so a
literal tip-only reserve would put the buffer plug at the orifice the instant a print
starts. `ink_reserve_volume_uL` is therefore `tip_volume_uL` **when a tip stage exists,
else the full bore volume**, which keeps every existing straight needle at exactly today's
~3.4 µL, and Quick Print warns (`_reserve_warning`) when the resolved reserve falls below
`_MIN_MEANINGFUL_RESERVE_UL` so the operator raises the ink padding instead. The other
three reserve terms (dispensed + prime + padding) are unaffected.

**Cylinder is the default tip profile** because it matches how a pull is specified and
measured, it is conservative on pressure (higher resistance for the same D2/L2, so the
flow ceiling errs low — the safe direction for glass that shatters), and it makes lift ↔
volume exactly piecewise-linear rather than cubic. A tapered tip is represented in the
sink model by its **equal-volume cylinder area** — exact in volume, monotone, closed-form
invertible; the exact frustum inverse is a cubic, not worth the numerics for a timing
estimate. That is the single approximation in the sink model, documented in `BoreProfile`.

**The sink-curve docstring premise was false for a capillary.** It claimed the curve is a
property of spheroid + fluid so a needle change only warns. Wall drag differs by orders of
magnitude between a 250 µm tip and a 1 mm barrel and the relation becomes piecewise at L2,
so the docstring was corrected and `tip_id_um` / `tip_length_mm` / `tip_area_mm2` /
`tip_profile` / `needle_type` are now stored with the curve (written only when supplied, so
a straight-needle save produces the identical JSON entry it always did).

**Deliberately NOT switched to the tip: the optical µm/px calibration**
(`calibration.py:~14589`). It *measures* the needle silhouette and divides a known OD by the
detected pixel width. A 30 µm tip is ~20–40 px, near the detector's floor, and the detector
will lock onto the high-contrast in-focus barrel shoulder anyway — so it stays on the barrel
OD (no behaviour change). A 30× error there would corrupt the whole optical calibration
invisibly. An explicit reference-feature selector is the follow-up, not a heuristic.

**Real numeric change by design:** `CellRemovalConfig.compute_release_volume_uL` and
`CellLabelingConfig.compute_deposit_volume_uL` now use the tip area, so a 0.1 mm push
through a 30 µm tip is ~0.07 nL — below one pump microstep. Straight needles are unaffected.

**Three latent bugs fixed in the blast radius** — each read `outer_diameter_um` /
`inner_diameter_um`, names `NeedleSpec` has never had:
* `jog_control.py:524` — inside a swallowing `except`, so the jog workspace and XZ side
  view have **never** drawn a real needle diameter (always the default).
* `standard_jog_context.py` and `context_sections.py` — silently dropped OD and ID, so the
  needle readout on Jog, Calibration and all four pick & place workflows only ever showed
  the gauge. Both now route through the shared `NeedleSpec.summary_line()`.

Fixed here rather than separately because they are 2-line changes on the identical lines
this work already touches, and shipping capillary support while that panel stays blind
would mean jogging a 30 µm tip near glass with no tip geometry displayed.

**Also fixed while in the area:** `calibration.py` built text for `ctx_lbl_needle_info`, a
widget created **nowhere in the repo**, so the `hasattr` guard was always False and the text
was discarded — deleted, with a pointer to `StandardJogContextPanel` as the single owner.
`wizard.py:386` assigned the catalog's own `NeedleSpec` instance into `config.needle`, so a
later edit mutated the shared catalog entry — now copies. `should_show_onboarding` gained an
`hardware_config.needle` gate so a capillary-first operator (who never writes
`workspace.needle_gauge`) isn't shown the wizard on every launch. And
`_apply_config_to_ui` no longer silently coerces a `length_inches` outside {1.0, 1.5, 2.0}
to 1.0" on the next save.

**Two pre-existing failures on this branch, confirmed NOT from this change** (reproduced
with the change stashed): `test_v75x_pump_relief_and_bead_model` ×2 (suck-back, from
uncommitted `PrintManager.py` WIP) and `test_v75x_print_setup_routine` ×2
(`_speed_pct_spin`, replaced by the v7.6 two-param rework). A pre-existing cross-suite hang
when several Qt workflow suites run in one process also reproduces on the baseline.

## Needs real-HW verification on ME3B V1

The model changes nothing until a capillary is configured, so verify in this order:

1. With the existing 27G needle, confirm the max-flow readout, a Quick Print bead and a prep
   run are numerically unchanged.
2. Configure a capillary → the max-flow ceiling drops by orders of magnitude (a 580 µm
   blank pulled to 30 µm over 5 mm reads ~0.8 µL/s against ~49000 µL/s for the barrel
   alone) and Quick Print's speed auto-reduces to respect it.
3. Run a short print → the bead matches the tip, not the barrel.
4. Re-run the spheroid sink staircase → the lift preview stays linear through the L2 knee.
5. Edit the tip length → the calibration-changed prompt fires.

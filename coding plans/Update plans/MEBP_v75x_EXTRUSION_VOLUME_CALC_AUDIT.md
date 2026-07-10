# MEBP v7.5.x — Extrusion / print-volume calculation audit & fixes

## Objective

Operator report: printing with a **27 G** needle, the extrusion (print volume)
has to be **ramped to ~7×** the default to get anything to print reasonably.
Requested a full check of the print-volume calculations for issues.

**Audit verdict: the core arithmetic is correct — no units bug or hidden factor.**
The whole Quick Print chain deposits exactly `bore_area × modifier` µL per mm of
travel, dimensionally sound end-to-end
(`flow = pct·area·xy_max·mod`, `speed = pct·xy_max` → `vol/mm = flow/speed =
area·mod`, then `move_pump_uL` converts µL→plunger-mm cleanly). The
Hagen–Poiseuille flow ceiling for a 27 G at water viscosity is ~376 µL/s — far
above any print flow — so the safety clamp is **not** silently limiting flow.

Needing a large multiplier is expected (the 1× model = a filament the exact
diameter of the bore = the theoretical minimum; a visible/spreading bead + prime
+ compliance losses need more). But the audit surfaced four genuine issues that
*compound* the need for a large multiplier, three of which this update fixes
(operator chose A + C + E; D folded into A):

| # | Finding | Fix |
|---|---------|-----|
| **A** | **Two inconsistent bead models.** Canonical F-1 (FlowPhysics / GeometryEngine / Quick Print) = `π·(id/2)² × modifier`, layer-height-independent. Sketch compiler = `id_mm × layer_height × mult` (a rectangle, layer-height-dependent). Same needle+path extrudes differently by path, and the Sketch under-extrudes for thin layers. CLAUDE.md F-1 claimed "ONE bead model everywhere" but the Sketch compiler was never migrated. | Migrate `SketchTrajectory.compile_to_trajectory` to the canonical **bore-area × mult** model (needle present); keep `bead × layer_height × mult` only as the no-needle preview fallback. |
| **C** | **Sub-threshold pump moves dropped and NOT accumulated.** `_execute_print_path` skips a segment whose `volume_uL ≤ 0.001` (at 27 G 1× that's every segment < 0.029 mm), and `ZPStage.move_relative` drops `<1e-4 mm` plunger deltas. Neither carries the remainder forward → cumulative under-extrusion on finely-sampled paths, worst at 1×. Cranking the modifier pushes segments over both thresholds → part of why "ramping up" makes it print. | Accumulate a `_pending_pump_uL` residual across segments; emit once it crosses the emit threshold; flush the tail at path end. Pacing/logging keep using the nominal per-segment volume (timing unchanged). |
| **D** | **Stale docstring.** `GeometryEngine` module docstring still says "Volume conservation: flow_rate = travel_speed × needle_OD × layer_height" — the pre-F-1 outer-Ø model. | Update to the inner-bore-area × modifier model (folded into A). |
| **E** | **Modifier has no effect on the parametric/trajectory path.** `PrintObject.extrusion_modifier` exists but `generate_object_trajectory` never reads it (calls `compute_pump_positions`/`compute_total_volume` at the 1.0 default), and there is no UI. So Print Builder parametric objects / Full Print run **locked at 1×** — un-printable for the same reason, with no knob. | Make `generate_object_trajectory` read `obj.extrusion_modifier` and thread it through the internal builders; add an "Extrusion ×" control to the Print Builder object designer, captured per-object. |

## Files Modified

- `SupportClasses/GeometryEngine.py` — (A/D) module docstring; (E) `generate_object_trajectory` reads `obj.extrusion_modifier` and threads it through `_gen_2d_trajectory` / `_gen_2d_fill_then_outline` / `_gen_multilayer` and the 8 shell/solid generators → `compute_pump_positions`, plus the final `compute_total_volume`. All new params default `1.0` (byte-identical legacy).
- `SupportClasses/SketchTrajectory.py` — (A) `compile_to_trajectory` volume model → canonical bore-area × mult when a needle is supplied; `bead × layer_height × mult` fallback only when no needle.
- `SupportClasses/PrintManager.py` — (C) `_execute_print_path` accumulates a `_pending_pump_uL` residual + flushes at path end; new `_PATH_PUMP_EMIT_MIN_UL` constant.
- `gui/pages/print_objects.py` — (E) "Extrusion ×" spin in the object designer; captured into `params["extrusion_modifier"]`; `_build_print_object` sets `obj.extrusion_modifier`; direct `PrintObject` commit sets the field.
- `tests/test_v75x_extrusion_volume_calc.py` — NEW: drop-gate accumulation, Sketch bore-area unification, parametric-path modifier threading.
- `tests/test_v75x_sketch_extrusion_thickness.py` — update the one test that asserted the OLD `id × layer_height` Sketch volume to the unified bore-area model.

## Implementation Steps

- [x] Audit the full chain (needle spec → FlowPhysics/GeometryEngine → Sketch → Quick Print → PrintManager `_execute_print_path` → `move_pump_uL`/`move_pump_relative` → `ZPStage.move_relative`; SafetyLimits flow ceiling; syringe conversions). Verdict: arithmetic correct; findings A–E.
- [x] (A/D) GeometryEngine docstring + thread `extrusion_modifier` through the builder chain (E) — new params default 1.0.
- [x] (A) SketchTrajectory bore-area unification + comment.
- [x] (C) PrintManager `_execute_print_path` residual accumulation + tail flush + constant.
- [x] (E) print_objects.py "Extrusion ×" UI + capture/restore + `_build_print_object` + commit-path field.
- [x] Tests: new `test_v75x_extrusion_volume_calc.py`; update `test_v75x_sketch_extrusion_thickness.py`.
- [x] Run affected suites green.

## Testing Notes

- `tests/test_v75x_extrusion_volume_calc.py` — (C) a finely-sampled path at 1× dispenses the full expected volume (no silent loss) and matches a coarse path of the same length; (A) Sketch volume == `length × π(id/2)² × mult`, layer-height-independent, matches `compute_total_volume`; (E) a parametric object's baked pump column scales with `extrusion_modifier`.
- Real-HW on ME3B V1: with the unified model, the 27 G print needs a smaller multiplier than before (fine-path drops fixed + parametric path now honors the modifier). Re-tune the Quick Print "Extrusion ×" / new Print Builder "Extrusion ×" against the physical bead.

## Issues & Decisions

- **Why not just raise the 1× baseline?** The bore-area model is the physically-correct *conservation* baseline (filament = bore). A visible bead needs a deliberate operator multiplier; baking a fudge factor into the baseline would hide the real geometry. Keep 1× = bore area, make the multiplier work everywhere and stop losing volume.
- **Sketch preview band unchanged.** The shaded thickness band is a display WIDTH (`id × mult`); the baked VOLUME is now bore AREA (`π(id/2)² × mult`). Width vs area differ by design (same as Quick Print, which shows no band). Band stays inner-Ø-based; only the baked volume changed.
- **`outer_ring_volume_uL` fixed target preserved.** Threading (not post-scaling) the modifier means the fill gets the modifier while an explicit outer-ring µL stays the exact absolute target the operator set.
- **Modifier stored in `params`** in the GUI to avoid churn across the many `_build_print_object` call sites; `_build_print_object` lifts it onto the real `obj.extrusion_modifier` field, which `generate_object_trajectory` reads.

# MEBP v7.9.x — Jog Step Sliders (µL-native pump steps, snap-to-setpoint, custom range)

## Objective

Operator (2026-08-07): *"we also need lower total volume for the pumps, lets
make a slider bar for each axis that we want to move by with setpoints close
to 0.001, 0.005, 0.01, 0.05, 0.1, 0.5, 1, 5, 10 uL. there should be a check
box for snap to nearest, and custom range for the slider. These settings
should be available via a collapsible settings section which is collapsed by
default."*

### Root cause of "we need lower total volume for the pumps"

The jog tiles' pump step was **a percentage of the syringe volume**
(`_PUMP_PERCENT_MAGNITUDES = 0.1 / 0.5 / 1 / 5 / 10 %`), converted at click
time by `StageController.pump_pct_to_uL`. On a 250 µL syringe the *smallest*
selectable step was 0.1 % = **0.25 µL** — 250× larger than the 0.001 µL the
operator needs, and the ladder's meaning changed with the syringe. A % ladder
structurally cannot express a sub-µL dose, so the step had to become a
**µL volume**, not a finer set of percentages.

## Files Modified

| File | Change |
|---|---|
| `gui/widgets/jog_button_array.py` | The five preset magnitude buttons per axis (`_StepGroup`) are replaced by NEW `_StepSlider` — a **log-scale slider** with two models: snap ON (one position per setpoint, ticks under each) and snap OFF (1000 log-interpolated positions, 3 s.f.). Ladders: `_XYZ_STEP_SETPOINTS_UM` (1…10000 µm) and NEW **`_PUMP_STEP_SETPOINTS_UL`** = the operator's list verbatim. The value box doubles as the live readout and still accepts any typed value — including outside the range, where only the slider position clamps. NEW collapsible **"Step slider settings"** disclosure (COLLAPSED by default) holding the snap checkbox + a per-axis min/max range. NEW `step_settings_changed` signal + `step_settings()`/`apply_step_settings()` for persistence, and public `set_step_snap`/`set_step_range`/`step_snap`/`step_settings_expanded`. `pump_step_is_percent` → **`pump_step_is_uL`**. |
| `gui/pages/hardware/control_panel.py` | `_on_jog_pump` takes the emitted distance as a **signed µL volume** on the %/µL pages (no `pump_pct_to_uL` scaling), guarded by the array's own declared unit. NEW `_persist_step_settings` / `_seed_step_settings` wired into `refresh_speed_limits` (so `set_controller` / `set_settings` / `showEvent` all re-seed) and the array's change signal. |
| `SupportClasses/StageController.py` | NEW `set_jog_step_settings` / `get_jog_step_settings` — the shared cross-tile store (same rationale as `_pump_jog_pct`). |
| `gui/pages/settings_page.py` | 🐞 `_ctx_jog_pump` routed the array's distance into `move_pump_relative`, which expects **plunger mm** — with the old % array a "1 %" step commanded a **1 mm plunger move**; the units never matched. Now `move_pump_uL`, matching the array's declared unit. |
| `tests/test_v75x_responsive_context_panel.py`, `tests/test_v75x_common_axis_speed_source.py` | Updated for the slider registries + the µL contract. |
| `tests/test_v79x_jog_step_sliders.py` | NEW — 16 tests. |

## Implementation Steps

- [x] `_StepSlider` (log scale, snap/free models, typed-value override)
- [x] µL setpoint ladder for pumps; µm ladder for XY/Z
- [x] Collapsible settings section (snap + per-axis range), collapsed by default
- [x] µL-native pump jog through `control_panel`; `settings_page` unit bug fixed
- [x] Cross-tile + restart persistence via the controller store + settings
- [x] Tests + 4 mutation checks
- [ ] Bench verification on ME3B V1 (below)

## Testing Notes

`tests/test_v79x_jog_step_sliders.py` (**16 green**): one slider per axis and
no preset buttons · the pump ladder equals the requested µL list verbatim ·
**walking the slider end-to-end yields exactly `0.001 … 10 µL`** (snap ON) ·
snap OFF is a continuous log sweep whose midpoint is the geometric mean and
is *not* a ladder value · a custom range narrows the ladder · an invalid
range (`min ≥ max`, ≤ 0, junk, None) is **refused, not clamped**, and the
edits snap back · a typed value outside the range is used verbatim ·
collapsed by default + toggles · the snap checkbox and the programmatic API
stay in sync · Hardware Setup's mm jog unchanged · a pump click emits signed
µL · settings shared across two panels on one controller · restart seeds from
settings and repopulates the controller · the panel dispenses the slider's µL
**without** calling `pump_pct_to_uL` · the `settings_page` handler routes to
`move_pump_uL` (AST, not a substring — the recorded trap where an
`in getsource()` check passed on a comment) · controller store returns a copy
and ignores non-dicts.

**4 mutations confirmed CAUGHT** (each a real source edit, reverted after):
treat the step as a % again → the two µL passthrough tests fail; expanded by
default → the collapse test fails; clamp instead of refuse an inverted range
→ the refusal test fails; drop `_seed_step_settings` from `refresh_speed_limits`
→ both sharing tests fail.

Regression, run per-suite, all green: jog-step-sliders (16) ·
per-pump-jog-flow (14) · responsive-context-panel (15) ·
common-axis-speed-source (18) · axis-max-speed-inputs (6) ·
jog-direction-z-up-sign (20) · context-panel (21) · jog-navigation (28) ·
stage-jog-off-gui-thread (3) · jog-pump-fill-readout (16) ·
pump-plunger-setup (27) · jog-motion-interpolation (29) ·
xbox-axis-speed-percent (20) · per-pump-max-rate (9) · illumination-led (31) ·
suite-hygiene (10). Plus a `gui.app` import smoke and offscreen builds of the
real `StandardJogContextPanel` (3 sliders, collapsed, µL step, three flow
rows), the real `HardwareSetupPage` (mm step, three max rows), and the
no-pump calibration Z-teach variant (XY/Z only; a stored `P` entry is
tolerated), each resized 120 px ↔ 520 px.

### Needs real-HW/GUI verification on ME3B V1

1. Jog pill → each axis shows a slider; drag the P slider to its far left and
   confirm the box reads **0.001 µL**; each detent steps the ladder exactly.
2. Aspirate/dispense at 0.001 µL and confirm the plunger moves (this is below
   one microstep on some syringes — if nothing moves, that is the firmware
   floor, not the UI; raise to 0.005/0.01).
3. Expand "Step slider settings", untick snap → the slider sweeps
   continuously; set a custom range (e.g. 0.001–0.05) → the ladder narrows.
4. Type an inverted range (min > max) → it is refused and the boxes snap back.
5. Set a step on the Jog page, switch to Calibration → same step; restart →
   still there.
6. Hardware Setup's pump ▲/▼ still jog in mm and its per-pump max rows are
   unchanged.

## Issues & Decisions

- **The pump step became a µL VOLUME, not a finer % ladder.** A % of syringe
  volume cannot express 0.001 µL usefully (0.0004 % of a 250 µL syringe) and
  its meaning drifts with the syringe. `jog_pump_requested`'s distance is now
  signed µL on the %/µL pages (unchanged mm on Hardware Setup), and the array
  declares which via `pump_step_is_uL` — the panel reads that flag rather
  than assuming, so a future embedder cannot silently dose 100× wrong.
- **`_pump_step_is_uL` (the old, unused ctor field) is gone**; it was set but
  never read (`set_pump_step_mode` had zero callers), and keeping a
  same-named-but-different-meaning flag beside the new `pump_step_is_uL`
  property would be the exact "two owners of one fact" trap this repo keeps
  hitting.
- **Snap ON is the default** and is a *display/selection* model only — the
  step VALUE stays authoritative across a snap toggle, so flipping the
  checkbox never silently re-rounds a value the operator typed.
- **An invalid range is refused, not clamped** — a silently clamped range
  leaves the slider disagreeing with the boxes that describe it (the same
  reasoning as the v7.9 bore-offset plausibility gates: a clamped input is a
  wrong setting that looks right).
- **`isHidden()`, not `isVisible()`, for the disclosure state.** `isVisible()`
  is False while any ancestor is unshown, so a toggle keyed on it refuses to
  open on a page that has not been displayed yet — and reads as "collapsed"
  to anything inspecting it. (CLAUDE.md records `isVisible() or not
  isHidden()` as a tautology to avoid; exactly one is chosen here, with the
  reason in the code.)
- **Settings persist and are shared, the step value included.** Same
  reasoning as the per-pump jog % landed alongside: nine pages each build
  their own jog panel, so the state lives on the controller and is re-seeded
  on every show. The step value is included because "lower total volume"
  should stay chosen — not reset to 1 µL on the next page.
- **`settings_page._ctx_jog_pump` was already wrong before this change** (a %
  fed to a mm API). It is fixed rather than left, since this change alters the
  unit flowing into it and leaving it would turn a latent mismatch into a
  live one.

# MEBP v7.21.1 — XY max speed is set in the hardware config, and it is authoritative

## Objective

Operator: *"the stage attached to this computer has a max speed of 5.5 mm/s. for some
reason during the most recent flourescent mosaic image (and print), the XY stage speed
defaulted to 50 mm/s — even when the hardware setup page set that value as something
else."* Then: *"in the hardware config for the xy stage give me the ability to set max
speed. in the workflow xy-zp calibration this max speed should be authoritative."*

Give the XY stage a real **Max speed** field in the hardware config and make that one
number the authority for every XY speed decision: the mm/s↔SMS-% conversion, the 100%
anchor for jog/print percentages, safe-travel speed, and the XY↔ZP timing calibration
workflow.

---

## Root cause (from the operator's own log)

```
set_speed_mm_s: 50.0 mm/s = 50000 µm/s = SMS 100% (max=50000 µm/s)
```

Two independent defects stacked. `settings.json` held `safety_limits.max_xy_speed =
5000.0` µm/s; neither defect consulted it.

**1. `safe_travel_to` hardcoded 50 mm/s and NO caller overrode it.**
`StageController.safe_travel_to(..., fast_xy_speed_mm_s: float = 50.0)` →
`xy_stage.set_speed_mm_s(50.0)`. All 12 call sites were checked (fluorescence mosaic,
Quick Print, pick & place, calibration navigation, plate-level wizard, stress test, sink
calibration, XY auto-cal) — **none** passes the argument. `StageController.
get_max_xy_speed_um_s()`, whose own docstring calls itself the single anchor "every
surface reads", was never called by travel.

**Why the whole mosaic ran that way:** only the *first* raster tile goes through
`safe_travel_to`; tiles 2..36 call `move_xy_absolute_um` with no speed at all. Prior SMS
is **modal**, so every subsequent tile inherited SMS 100%.

**2. The mm/s↔SMS denominator was a nominal 50000.**
`XYStage._max_speed_um_s()` resolves: per-machine override → sim → protocol
`parameters.max_speed` → 50000 literal. `config/controllers/proscan_iii_h117.json`
declares `"max_speed": 50000`, and `config/hardware/ME3B_01/print_timing_calibration.json`
holds only `{"version":"1.0","enabled":true}` — no measurement — so the connect-time
seeding was a no-op. `50.0 / 50000 → SMS 100%`.

**3. There was no UI to set it at all.** `spin_max_xy_speed` has existed since v7.4.2
with a comment saying it "is reparented into the XY Calibration section below". It never
was — it was created and **never added to any layout**. From v7.4.2 to v7.21.0 the XY max
speed was unsettable. (`DeviceProfile.xy_max_speed_um_s` likewise existed with a docstring
describing exactly this role and had **zero production readers or writers**.)

**Second-order:** with a 50000 denominator against a true 5500 top speed, every mm/s
request is scaled ~9× wrong *in the opposite direction* — a print asking
`travel_speed_mm_s = 10.0` sends SMS 20% ≈ 1.1 mm/s while the open-loop pacing sleeps as
if it were doing 10. This is the failure already recorded in
`MEBP_v75x_XY_ZP_TIMING_CALIBRATION.md` Rev 9, which flagged the unverified 50000 and was
never closed on this rig.

---

## Decisions (AskUserQuestion ×2)

1. **Measurement reports + explicit Apply.** The timing tool's "Measure top speed" no
   longer overwrites the configured value; it reports, compares against what is
   configured, and arms an **Apply to hardware config** button. (Alternatives: keep
   auto-applying; read-only measurement.)
2. **Fix `safe_travel_to` in this change.** Default → `None` → resolves from the
   configured max, so all 12 callers inherit it.

---

## The one design fact everything rests on

**"Never declared" must stay distinguishable from "declared 10000".**

`SafetyLimits.max_xy_speed` carries a **10000 µm/s dataclass default**. The declared top
speed becomes the **denominator** of `SMS% = requested ÷ top`, so declaring a value
*below* the stage's real top speed makes every commanded speed run proportionally
**FASTER** than asked. Feeding a policy default into that denominator would therefore be
actively dangerous on a machine whose real top speed is higher.

So there are two resolvers with deliberately different precedence:

| Resolver | Precedence | Used for |
|---|---|---|
| `declared_xy_top_speed_um_s()` | explicit declaration → legacy timing-store measurement → **None** | the mm/s↔SMS denominator (a physical claim; `safety_limits` cannot make one because it has a default) |
| `get_max_xy_speed_um_s()` | explicit declaration → `safety_limits.max_xy_speed` → legacy store → conservative constant | the 100% anchor for jog %, print %, and travel speed |

The v7.5.x rule that `safety_limits` outranks the stored measurement is **preserved** —
editing the safety value must still propagate over a stale measurement.

An undeclared machine keeps the protocol `max_speed` for its denominator, i.e. exactly
pre-v7.21.1 behaviour.

---

## Files Modified

| File | Change |
|---|---|
| `SupportClasses/StageController.py` | NEW `_positive_number` module helper; `_declared_xy_top_speed_um_s` field; NEW `_explicit_xy_top_speed_um_s` / `declared_xy_top_speed_um_s` / `set_xy_top_speed_um_s` / `_push_xy_top_speed_to_stage`; `get_max_xy_speed_um_s` prefers the declaration; connect-time seeding routed through the one resolver; `apply_device_settings(xy_max_speed_um_s=)`; `safe_travel_to(fast_xy_speed_mm_s=None)` resolves the configured max + one-shot undeclared warning |
| `gui/pages/hardware/stage_panel.py` | **Max speed** spin + live mm/s hint added to the XY Stage Calibration section (`spin_max_xy_speed` finally parented); `_refresh_xy_max_speed_hint`; load prefers `device_profile.xy_max_speed_um_s`; both Save paths write it; apply routes through `set_xy_top_speed_um_s` |
| `gui/app.py` | startup `apply_device_settings(xy_max_speed_um_s=…)` so the declaration is in force before the first move |
| `gui/pages/workflows/timing_calibration_workflow.py` | worker stops writing the store/stage; NEW `_configured_xy_top_speed_um_s` / `_offer_measured_xy_top_speed` / `_on_apply_measured_speed`; **Apply to hardware config** button; `_apply_measured_xy_top_speed` routes through the controller's one setter and persists both keys |
| `tests/test_v7211_xy_max_speed_authoritative.py` | NEW (16) |
| `tests/test_v75x_print_timing_calibration.py` | contract updated (see below) |

---

## Implementation Steps

- [x] Root-cause from the operator's log; audit all 12 `safe_travel_to` call sites
- [x] `_positive_number` guard + declaration field + resolvers + one setter
- [x] `safe_travel_to` resolves the configured max; explicit arg still wins
- [x] Max speed field parented into XY Stage Calibration, with mm/s hint
- [x] Persist to `safety_limits.max_xy_speed` **and** `device_profile.xy_max_speed_um_s`
- [x] Startup restore via `apply_device_settings`
- [x] Timing workflow: measure → report → explicit Apply
- [x] Tests + mutation matrix + regression

---

## Issues & Decisions

**⚠ `float()` is not a numeric check — a MagicMock answers 1.0.** The first cut coerced
with `float(...)` in a `try`. `MagicMock` implements `__float__`, so a partially-stubbed
controller (this repo's standard GUI-test pattern) injected a **1 µm/s** "top speed" that
every speed command then scaled against — and the existing
`test_v75x_common_axis_speed_source` caught it, reporting `1.0 != 20000.0` and a rendered
`= 0 µm/s`. Replaced with an `isinstance` guard in `_positive_number`. This is a real
robustness fix, not a test accommodation: the coercing form would let a stubbed test pass
while production scaled every speed by ~5000×.

**⚠ A precedence regression I introduced and the suite caught.** Bundling the timing-store
measurement into `declared_xy_top_speed_um_s()` and checking that first made the store
outrank `safety_limits.max_xy_speed` — the opposite of "the hardware config is
authoritative", and it broke
`test_v75x_common_axis_speed_source::test_safety_limit_is_the_single_source`. Split into
`_explicit_xy_top_speed_um_s` (declaration only) vs the public bundled resolver, so the
two precedence chains are stated once each.

**⚠ DISCLOSED — an undeclared machine now travels SLOWER until the value is set.** With no
declaration, travel is commanded at the safety anchor (e.g. 5 mm/s) while the stage still
converts against the protocol's 50000 → SMS 10% → ~0.55 mm/s actual. That is the *safe*
direction (slow, never fast) and it is self-announcing, but it looks like a fault, so
`safe_travel_to` logs a one-shot warning naming the remedy. **On ME3B_01 this applies
until Max speed is set to 5500 µm/s and saved.**

**Deliberately NOT auto-migrated:** `safety_limits.max_xy_speed` is *not* adopted as the
declaration on upgrade. On a rig where that value is a policy cap below the physical top
speed, adopting it would make every command run faster than asked — the exact hazard
`_positive_number` and the two-resolver split exist to prevent. Declaring is one explicit
click.

**Test contract changed, old test deleted not renamed:**
`test_v75x_print_timing_calibration::test_measures_stores_and_applies` asserted that the
measurement writes the store and the live stage. That contract no longer exists, so it
was replaced by `test_measures_and_reports_without_writing_anything` +
`test_apply_button_is_what_makes_it_authoritative`, with the unchanged safety property
(`ends retracted`) kept as its own test.

---

## Testing Notes

**NEW `tests/test_v7211_xy_max_speed_authoritative.py` — 16, all green.** Drives the
production `StageController` and the **real** `StageHardwarePanel` (a stand-in that agrees
with the code proves nothing about it).

Load-bearing tests:
- `test_undeclared_reports_none_even_though_safety_has_a_default` + `..._never_touches_the_stage_denominator` — the distinction the whole design rests on.
- `test_the_old_hardcoded_value_would_have_been_visibly_wrong` — guards the guard: asserts the resolved travel speed is nowhere near 50 mm/s, so the positive test cannot pass merely because nothing sets a speed at all.
- `TestNoCallerHardcodesTravelSpeed` — AST walk over `SupportClasses/` + `gui/` failing on any `safe_travel_to(..., fast_xy_speed_mm_s=…)`, self-guarded by asserting it found >5 call sites (a matcher that finds nothing passes).
- `TestTheFieldIsActuallyOnScreen` — the field is **parented, not hidden, and inside the `xy_cal` section**. Asserting the attribute merely *exists* would have passed for the three versions the widget was headless. Uses `isHidden()` not `isVisible()` (an offscreen panel's ancestors are never shown) and pins section **identity**, since `ReorderableSection` blanks the QGroupBox title.

**Mutation matrix — 3/3 CAUGHT**, anchors verified against the live source first (and
against CRLF, this file's line ending — a `\n` anchor silently matches nothing):
1. travel speed back to the hardcoded `50.0` → 3 failures (the original bug)
2. declaration falls back to the safety default → 3 failures (the faster-than-asked hazard)
3. setter stops pushing the SMS denominator → 1 failure

Sources restored byte-identical after each.

**Regression, run per batch:**
- 138 green: `v7211` · `v731_jog_navigation` · `v75x_axis_max_speed_inputs` · `v75x_common_axis_speed_source` · `v75x_device_page_section_layout` · `v75x_z_retract_before_xy_travel` · `v720_no_descent_before_xy_arrival` · `v75x_jog_travel_off_gui_thread`
- 301 across the XY speed/timing/protocol suites (`xy_speed_conversion`, `xy_auto_calibration`, `xy_challenge_panel`, `xy_challenge_metrics`, `xy_dead_time`, `xy_envelope_absolute`, `xy_feed_plan`, `xy_path_simulator`, `xy_stage_model`, `print_timing_calibration`, `timing_encoder_detector`, `xbox_axis_speed_percent`, `ludl_xy_prior_regression`, `v7181_xy_fast_connect`, `v7171_xy_stale_ack`, suite hygiene)
- 201 green: travel consumers (`pump_plunger_setup`, `quick_print_travel_split`, `mosaic_unreachable_travel`, `fluorescence_mosaic`, `spheroid_pick_place_z`, `illumination_led`, `v721_section_promotion`)
- `gui.app` import smoke

**9 pre-existing failures PROVED not ours** in a `git worktree` at committed HEAD (never
`git stash` in this repo): the documented `_note_move_estimate_xy_rel` errors in
`test_v75x_xy_envelope_absolute`. They reproduce identically at HEAD.

---

## Needs HW verification on ME3B_01, IN ORDER

1. **Set it first — nothing else is trusted until this is done.** Hardware Setup → Device →
   XY Stage Calibration → **Max speed = 5500 µm/s** (hint should read `= 5.50 mm/s`) →
   Apply/Save. Before this, expect the one-shot *"no declared top speed"* warning in
   `logs/app.log` and slower-than-commanded travel.
2. Travel to a well and confirm the log now reads `set_speed_mm_s: 5.5 mm/s … SMS 100%
   (max=5500 µm/s)` — **not** `50.0 mm/s … max=50000`.
3. **The real check is a stopwatch, not the log:** command a known long travel and confirm
   the achieved speed matches the commanded mm/s. A commanded 2.75 mm/s that arrives in
   half the expected time means the declared top speed is too low.
4. Restart → the field still reads 5500 and the log still shows `max=5500`.
5. Re-run the fluorescence mosaic: the first tile and **every** subsequent tile travel at
   the configured speed (SMS is modal — a wrong first tile poisons the whole scan).
6. Workflows → XY↔ZP Timing Calibration → **Measure top speed**: it must **report** and
   compare against 5.50 mm/s and change nothing; then press **Apply to hardware config**
   and confirm Hardware Setup shows the measured value.
7. Print speed % and jog % re-anchor to the new max (a 50% jog should be ~2.75 mm/s).

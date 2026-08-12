# MEBP v7.17 — Bore offset selection needs no stage motion

## Objective

Operator: *"needle bore selection should not move the xy stage, I should be able
to put the needle down anywhere i want and just select the offset from the
microscope camera."*

They are describing what the measurement already IS, and the code already said
so — the module docstring calls the park "convenience, not calibration" and a
`Start measuring` button existed for a hand-jogged stage. What contradicted it
was the **gate**: one shared `gate()` served step 3 (a zero-motion measurement)
and step 4 (a real descent toward the glass), so step 3 refused until the
**Plate Bottom Z** and **Fast Move (Safe) Z** were taught — two heights that
exist only to compute and reach the optional park. An operator who had the
needle exactly where they wanted it was sent back to step 2 to teach travel
heights before being allowed to click two dots in one camera frame.

## Why the requirements were wrong for step 3

The bore offsets are `offset_um(k) = pto(P_k) − pto(P_0)` — differences between
clicks in ONE frame. The stage position cancels exactly, so *where* the needle
sits is irrelevant; only that it does not move between clicks, which the
existing drift guard (`MAX_STAGE_DRIFT_UM`) already enforces. Nothing in that
chain needs a plate bottom or a safe Z.

Note the gate never protected the needle either: manual jogging happens on the
Jog panel / Xbox, outside this wizard. The gate only ever disabled the park
button and the click session, so relaxing it removes no protection.

## Files Modified

| File | Change |
|---|---|
| `gui/widgets/needle_bore_wizard.py` | Split the gate; park gated separately; button states + tooltips; floor advisory; text and docstring |
| `tests/test_v710_needle_bore_wizard.py` | Two coupled gate assertions replaced by the split's contract (+2 classes) |

## Implementation Steps

- [x] `gate(STEP_BORES)` keeps only what turns a click into a distance: bore
      store, XY connected (a *read*, for the drift guard), needle configured,
      microscope role assigned, µm/px calibrated.
- [x] **`STEP_TOUCHOFF` keeps both reference heights** — step 4 genuinely
      descends toward the glass. The two steps no longer share one gate.
- [x] New `park_gate()` = the measurement gate **plus** plate bottom + safe Z +
      a saved needle location + a `_safe_navigate_to` host. `_on_park` uses it.
- [x] Every park refusal names the no-motion alternative, so a blocked park
      cannot read as "step 3 is blocked".
- [x] New `_render_step3_buttons()` disables each button with its first reason as
      the tooltip (the `_commit_blockers` pattern — never click to find out). A
      blocked park leaves **Start measuring here** enabled.
- [x] `Start measuring here` promoted to the primary action (`successBtn`, first
      row); the park moved below and labelled "Optional park".
- [x] Instruction/status/docstring rewritten to lead with "put the needle
      anywhere every bore is visible".
- [x] New `_floor_advisory()` — see Issues & Decisions.

## Issues & Decisions

**The relaxed gate exposed one honest hazard, disclosed rather than blocked.**
Recording each bore's focus Z means jogging the needle DOWN by hand. Step 3 arms
the plate-bottom clamp (`_arm_floor(True)`), but
`StageController._apply_print_floor_raw` returns early when
`_plate_bottom_z_zref is None` — so with no taught plate bottom the clamp is a
**no-op**, and the operator may believe it is protecting them. Blocking the
measurement over this would reintroduce exactly the bug being fixed, and staying
silent would imply protection that is not armed; so `_floor_advisory()` says so
inline and disappears once the datum exists.

**⚠ One of my own tests was too weak and I caught it before shipping.** Asserting
on `park_gate()` alone says nothing about whether `_on_park` *calls* it: with the
split gate, `_on_park` reading `gate()` would still find a plate bottom (so
`_survey_target_zref` returns a number) and a destination, and would travel with
no safe Z to retract to. Added
`test_the_park_itself_refuses_without_a_safe_z`, which drives `_on_park` and
asserts no travel was commanded — mutation M2 confirms it catches that wiring.

**⚠ A bulk rename of the button label missed one string, and a test caught it.**
The label appeared in a refusal message split across a line break (`"'Start "` +
`"measuring'.)"`), so a whole-word replace skipped it and one park refusal named
a button that no longer exists. Fixed, plus a grep for stale refs.

## Testing Notes

`tests/test_v710_needle_bore_wizard.py` — **73 pass** (was 71; two coupled gate
assertions replaced, 15 added across `TestMeasuringNeedsNoMotionPrerequisites`
and `TestParkGate`). The load-bearing one is
`test_a_session_starts_and_clicks_land_with_no_reference_heights`: no plate
bottom, no safe Z, no saved needle location — a two-bore offset still measures
to 320.0 µm with `nav == []` and `z_moves == []`.

**4/4 mutations CAUGHT**, sources restored:

| Mutation | Result |
|---|---|
| Plate-bottom/safe-Z checks back in the shared `gate()` (the original bug) | 7 failures |
| `_on_park` uses `gate()` instead of `park_gate()` | 3 failures |
| `_floor_advisory()` always returns `""` | 1 failure |
| Start button gated on `park_gate()` | 1 failure |

Regression **285 green** across needle-bore-wizard / microscope-bore-sign /
bore-gate-live-refresh / bore-focus-ROI / print-floor-refcount /
optical-needle-datum / bore-dot-overlay / needle-loc-tab-layout (189) and
host-accessor-contract / focus-needle-datum / optical-datum-placement /
plate-bottom-optical / bore-offset-calibration / bore-safety-remediation /
multibore-needle (96), plus a `gui.app` import smoke.

**Offscreen smoke against the real `CalibrationPage`** (not a stand-in — a
stand-in whose gate agrees with the wizard proves nothing): with
`_plate_bottom_z is None` and `_safe_z is None`, the measure gate returns
`(True, '')`, **Start measuring here** is enabled, the session arms anchored at
the live stage position, the park is disabled naming the alternative, and the
floor advisory is shown.

**One pre-existing failure PROVED not ours:**
`test_v79_needle_form_ui::test_every_on_disk_setup_round_trips_through_the_page`
("only 1 on-disk setup(s) exercised", wants ≥ 6) — it globs
`config/hardware/*.json` and this working tree has one. Reproduced identically
with this change stashed.

## Needs GUI verification on ME3B V1

1. Open the bore wizard on a rig with **no** Plate Bottom / Fast Move Z taught
   and go to step 3 — it must be usable, not refuse.
2. Jog the needle anywhere the bores are visible in the microscope, press
   **Start measuring here**, click each bore's tip — offsets appear, and the
   stage must not move at any point.
3. The park button reads disabled with a tooltip naming what to teach *and* that
   it is not needed to measure.
4. Confirm the ⚠ inactive-clamp advisory appears, and disappears once the plate
   bottom is taught.
5. Teach both heights, then confirm the park still works as before (retract →
   XY → lower).
6. **The measurement is unchanged:** drive each bore to the same target and
   check direction as well as distance — a flipped sign lands at twice the
   spacing on the wrong side (per `MEBP_v710_NEEDLE_BORE_MICROSCOPE_WIZARD.md`).
7. Step 4 must still refuse without both reference heights — it really descends.

# MEBP v7.21.7 — Hold the needle in the liquid after aspirating

## Objective

Operator report (Quick Print):

> *"whenever the needle aspirates a liquid (e.g. oil or ink), the Z is moving up
> before the pump is done aspirating. This results in the needle being outside of
> the liquid and into the air. When the needle is the air, it aspirates air --
> this is bad. To remedy this problem, I would like to add more time in between
> the time the needle aspirates liquid and moves Z"*

Add a configurable **hold, with the needle still submerged**, between a reagent
aspirate finishing and the retract-and-travel that follows.

---

## Root cause — the plunger was already synchronous; the FLUID was not

This is worth stating precisely, because the obvious diagnosis ("the code
doesn't wait for the pump") is **wrong**, and acting on it would have produced a
fix that changes nothing:

* every pick/place pump actuation goes through `_settled_pump_move` →
  `StageController.move_pump_uL(settle=True)`;
* that path calls `_finish_pump_submove(block=True)` →
  `_wait_pump_move_complete`, which **drains the move from Marlin's planner with
  M400** (timeout scaled to the estimated move duration, poller suspended);
* so when `aspirate_ink` / `run_prep` returns, the **plunger** is provably
  finished. Then `pump_settle_time_s` dwells on top of that.

What is *not* finished is the **fluid**. The column between plunger and tip is
compliant (a fine bore, a viscous ink, a long tube), so liquid keeps being drawn
in for a while after the plunger has stopped. The next step is a
`_safe_move_to*` → `safe_travel_to`, whose **first action is the Z retract** —
which lifts the tip out of the well inside exactly that window. The tail of the
aspirate is then air.

`pump_settle_time_s` was the closest existing knob but is the wrong instrument:
it brackets **every** discrete pump move (before *and* after, dispenses
included, the print prime included), so buying seconds of in-liquid hold with it
would slow every unrelated actuation on the machine by the same amount.

Hence a separate, purpose-named value.

---

## Design

**One new global**, resolved in ONE place, inherited by every workflow.

```
HardwareConfig.pump_post_aspirate_dwell_s        (default 2.0 s, serialized)
        ↓
StageController.pump_post_aspirate_dwell_s()     (clamped ≥ 0)
        ↓
PickPlaceExecutor.post_aspirate_dwell_s = None   → _post_aspirate_dwell_s()
        ↓
        _settle_in_liquid(op, "ink" | "oil" | "buffer")
```

`post_aspirate_dwell_s = None` on the executor means **inherit the global**.
That is what makes this a one-line-per-surface change: no workflow page has to
wire the field, so Quick Print, spheroid pick & place, cell targeting and cell
labeling all pick it up by construction, and none of them can drift.

### Where the hold fires

Every point where the needle is **in a reagent well, has just ASPIRATED, and a
travel follows**:

| Site | Step |
|---|---|
| `aspirate_ink` | ink pickup (after the aspirate, and after the tip-prime dispense-back when priming is on) |
| `run_prep` | step 2 aspirate oil · step 4 aspirate buffer |
| `run_post_clean` | step 3 buffer reload |
| `run_print_cleanup` | step 3 oil reset — **only when the signed reset is an aspirate** |
| `prepare_starting_oil` | the `dispense_to_waste=False` oil top-up |

Deliberately **not** applied to: any dispense, the streamed print path, the
print prime, or manual jog.

### Decisions worth recording

* **Default 2.0 s, not 0.** A knob defaulting to 0 leaves the reported defect in
  place until the operator finds the setting. An absent key in an existing saved
  setup also takes 2.0 — a setup written before this existed was written by code
  carrying the defect, so inheriting the fix is the intended migration. The cost
  is ~2 s × a handful of reagent pickups per run, against a failure mode (an
  air-filled needle that prints nothing while the run reports success) that
  costs a whole run.
* **One hold in `aspirate_ink`, at the END of the in-well pump sequence.** With
  tip-priming on the sequence is `aspirate(vol+prime)` → `dispense(prime)` back
  into the same well with **no Z move in between**, so a single hold immediately
  before the needle actually leaves is what the operator asked for, and it does
  not double the wait when priming is on. It sits inside the `try`, so a
  granular-ink orbit keeps stirring through the hold and is still stopped by the
  `finally`.
* **The cleanup oil reset is gated on DIRECTION, not on the step.** That reset is
  signed — it can dispense or aspirate depending on where the plunger ended up —
  so `if oil_uL < 0` is the correct gate; gating on "it is the oil step" would
  hold pointlessly half the time.
* **Abort-aware.** The hold routes through the existing `_dwell`, so an Abort
  during a multi-second hold raises immediately instead of sitting it out.
* **0 s is byte-identical to legacy** — including the sub-step / dwell-tick
  chatter, which is skipped entirely rather than emitted with a 0 duration.

---

## Files Modified

| File | Change |
|---|---|
| `SupportClasses/HardwareConfig.py` | new `pump_post_aspirate_dwell_s` field + `to_dict` + `from_dict` (junk/negative tolerant; absent ⇒ 2.0) |
| `SupportClasses/StageController.py` | new `pump_post_aspirate_dwell_s()` reader, documented against `pump_settle_time_s` so the two are not conflated later |
| `SupportClasses/PickAndPlaceManager.py` | executor field `post_aspirate_dwell_s`; `_post_aspirate_dwell_s()` resolver; `_settle_in_liquid()`; calls at all six aspirate sites |
| `SupportClasses/CommonPrintSettings.py` | registered in `GLOBAL_KEYS` / `GLOBAL_DEFAULTS` |
| `gui/pages/hardware_setup.py` | **Hold in liquid** spin in *Pump Timing (all pumps)* + capture + restore |
| `gui/pages/workflows/common_print_settings_workflow.py` | matching row on the Common Print Settings page |
| `gui/pages/workflows/{quick_print,spheroid_pickup,cell_targeting,cell_labeling}_workflow.py` | linked row in each *Common — Pump (global)* settings section |
| `gui/pages/workflows/quick_print_workflow.py` | `_hold_in_liquid_s()` + both confirm dialogs name the hold |
| `tests/test_v7217_hold_in_liquid_after_aspirate.py` | **NEW** — 28 tests |

---

## Implementation Steps

- [x] `HardwareConfig` field + serialization + tolerant load
- [x] `StageController.pump_post_aspirate_dwell_s()`
- [x] Executor field, resolver, `_settle_in_liquid`
- [x] Wire `aspirate_ink`
- [x] Wire `run_prep` (oil + buffer)
- [x] Wire `run_post_clean` (buffer)
- [x] Wire `run_print_cleanup` (oil reset, direction-gated)
- [x] Wire `prepare_starting_oil` (top-up branch only)
- [x] Register as a shared global (`CommonPrintSettings`)
- [x] Hardware Setup → Pump row
- [x] Common Print Settings page row
- [x] All four workflow popout rows
- [x] Quick Print confirm dialogs state the hold
- [x] Tests + mutation matrix

---

## Testing Notes

`tests/test_v7217_hold_in_liquid_after_aspirate.py` — **28 tests, green.**

The fixture records every travel / pump / wash / hold onto **one ordered event
list**, because the defect is purely an **ordering** defect: a fixture that can
only count holds would pass with the hold on the wrong side of the retract, i.e.
with the bug still shipping. The ordering tests therefore assert the hold sits
**immediately after the aspirate and before the next travel**, not merely that
one happened.

The controller fake implements `move_pump_uL` rather than the test overriding
`_pump_move` — the prep's `compensate=None` aspirates go through the
module-level `_settled_pump_move` instead, so an override would have left
exactly the reagent aspirates this change is about un-recorded.

**Mutation matrix: 11/11 CAUGHT**, sources verified restored byte-identical.

| Mutation | Result |
|---|---|
| ink pickup never holds | CAUGHT |
| prep oil aspirate not marked as a liquid | CAUGHT |
| prep buffer aspirate not marked as a liquid | CAUGHT |
| hold moved BEFORE the pump move (right dwell, wrong side) | CAUGHT |
| executor stops inheriting the global | CAUGHT |
| a DISPENSE is held too | CAUGHT |
| cleanup oil reset never holds | CAUGHT |
| starting-oil top-up never holds | CAUGHT |
| controller reader aliased onto the settle dwell | CAUGHT |
| absent key no longer inherits the fix | CAUGHT |
| zero no longer disables the hold | CAUGHT |

The harness asserts the **baseline is GREEN before it starts** and restores in a
`finally` (the failure mode already recorded in this project's history: a run
scored against an already-red suite manufactures false "CAUGHT" verdicts).

**Regression, all green:**

* pump settle/prime · common print settings · workflow settings popout (123)
* Quick Print pick&place · multi-ink · spheroid · cell targeting · cell labeling (121)
* multibore · per-bore cell targeting · bore safety ×2 · simultaneous pumps · suite hygiene (228)
* v7.19 plate queue · v7.20 print calibrator · v7.21 section promotion (288)
* sketch→Quick-Print extrusion · capillary input · pump plunger setup (89)

Plus a `gui.app` import smoke and an **offscreen build of the REAL
`HardwareSetupPage`** round-tripping 6.50 s in / 3.25 s out, and the **REAL
`QuickPrintWorkflowPage`** reading the same global through `CommonPrintSettings`.

**One pre-existing failure, PROVED not ours:**
`test_v75x_pump_relief_and_bead_model::TestPrintPathSuckback` ×2 — it drives
`PrintManager._print_pump_suckback` (a file **not in this diff**) as a `__new__`
partial; the uncommitted `PrintManager.py` WIP in this tree routes that method
through `_pump_uL`, which reads `self._abort_flag` that the partial never sets,
so the exception-safe wrapper swallows it and records 0 moves. Already recorded
against the v7.6 entry in `CLAUDE.md` as a known pre-existing failure from that
same WIP.

---

## Issues & Decisions

* **A first cut broke an abort opportunity and a test caught it.** Routing the
  hold through `run_prep`'s / `run_post_clean`'s inner `actuate()` used
  `else: return` on the "nothing was actuated" branch — which also skipped the
  trailing `_check_abort()` that ran there before v7.21.7. Replaced with a
  `moved` flag so the abort check stays **unconditional**; pinned by
  `test_prep_still_checks_abort_when_nothing_is_actuated`.
* **`_post_aspirate_dwell_s` reads its own field with `getattr`.** Several suites
  drive `PickPlaceExecutor` as a `__new__` partial whose `__init__` never ran, so
  a bare attribute read raises there — 12 pre-existing tests went red until this
  was fixed. Missing ⇒ inherit, same as `None`.
* ⚠ **Shell heredocs collapse escapes in this environment** (already recorded in
  `CLAUDE.md`) and it bit again: an unquoted heredoc command-substituted the
  backticks in a Sphinx `:meth:` reference, silently emptying two docstring
  cross-references. Escape-heavy source was written with the file tools and
  patched through a line-ending-preserving helper; `SupportClasses/*` is CRLF
  while `gui/*` is mixed, and a naive text-mode rewrite converts a whole file.

---

## Needs verification on ME3B V1, IN ORDER

1. **Go/no-go:** Hardware Setup → Pump shows **Hold in liquid** at **2.00 s** on
   this rig's existing setup (the absent-key migration), and *Settle time* is
   unchanged at its saved value.
2. Run a Quick Print with prep on. The confirm dialog names the hold. Then
   **watch the needle**: after the oil aspirate it should sit still in the oil
   for ~2 s before Z lifts — and the same at the buffer well and at the ink well.
3. **The actual check — inspect the needle after the ink pickup.** No air gap at
   the tip. If there still is one, raise the value (it is a compliance
   time-constant, so a fine bore or a viscous ink can legitimately need 5–10 s)
   and repeat until the air is gone; that number is this rig's answer.
4. Set it to **0** and confirm the old behaviour returns exactly (needle lifts
   immediately after each aspirate) — that is the proof the hold is what changed
   things, not something else.
5. Confirm a **dispense** is NOT held: the prep's oil-to-waste step should lift
   immediately, and the print itself should start with no added delay.
6. **Abort during a hold** ends the run promptly at Safe Z rather than waiting
   the dwell out.
7. Restart and confirm the value persists; check it also shows on Workflows →
   Common Print Settings and in the other workflows' ⚙ popouts (one value
   everywhere).

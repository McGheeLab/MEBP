# MEBP v7.9.1 — jog panels: per-pump rate coverage + the Live Position row

**Branch:** `Version-7.9.1`

Two operator reports, both about features a plan doc already claimed were done
(`MEBP_v79x_PER_PUMP_JOG_FLOW_AND_POSITION_READOUT.md`). Both claims were
**partly** true, which is why they read as fixed and behaved as broken.

---

## Objective

| Operator's words | Verdict |
|---|---|
| *"on all the pump jog pannels we need independent control of rate for each pump"* | Real, **but not a phantom feature** — it exists and works on nine surfaces; two bypass it. |
| *"under the jog pannel the live position pannel the current position of the axis is overtop of the units"* | Real. Two size settings fighting each other. |

---

## 1. Per-pump rate — real, with two gaps

Audited every pump-jog surface rather than trusting the plan doc, because this
repo has form for features described as done and never wired (v7.9's
`prep_bores` had no production writer; the longest-bore descend had no
consumer). This one is genuinely implemented:

**Already per-pump ✔** — `HardwareControlPanel`, in both modes:
* `%` mode (`control_panel.py:646-665`) builds one `spin_p_pct_pumps[pid]` row
  per pump, anchored by `_pump_speed_anchor_for` →
  `StageController.get_max_pump_feedrate_for(pump)` — that pump's own
  needle/syringe-derived ceiling.
* `max` mode (Hardware Setup, mm) builds `spin_p_max_pumps` per pump.
* State lives on the **controller** (`set_pump_jog_pct` / `get_pump_jog_pcts`,
  persisted `jog.pump_jog_pct`), which is what stops the nine tiles diverging.
* Traced end to end: clicking P2 uses **P2's own spin × P2's own ceiling**.

That covers the Jog page, Calibration, Hardware Setup, the Custom context
panel, and all six workflow pages. **No change needed there.**

### Gap A — the Settings-page context panel sent NO rate at all

`settings_page.py::_ctx_jog_pump` called `move_pump_uL(pump, volume_uL)` with
`rate_uL_s` left `None`, so all three pumps ran at the controller's internal
default while the same pump on every other tile ran at its configured flow.
This panel has no speed widget of its own — the v7.9.x step-slider rework
touched this very method to fix a mm/µL units mismatch and simply never added
the rate argument.

Fixed by reading the **shared controller state** rather than inventing a
widget: `pct = get_pump_jog_pcts()[pump]`, `ceiling =
get_max_pump_feedrate_for(pump)` — the same resolution
`HardwareControlPanel._on_jog_pump` performs, so a pump now jogs identically
wherever it is jogged from. An unset percentage or any failure returns `None`
and keeps the legacy default: a wrong rate is worse than the old behaviour.

### Gap B — the Xbox anchored all three pumps to one ceiling

`ZPJogHandler._pump_vel_mm_s` applied per-pump *syringe geometry* but a single
`p_speed_max`. Since bore ceilings differ by orders of magnitude (v7.9 measured
~2000× between a 22G bore and a 30 µm pulled tip), a full stick deflection drove
a fine bore at a coarse bore's rate.

The ceiling is now per-pump, read from `SafetyLimits.get_max_flow_rate` — the
same source `get_max_pump_feedrate_for` uses, so the Xbox and the tiles resolve
from one authority. Measured with realistic ceilings (8.0 / 0.5 / 2.0 µL/s), a
full deflection now yields three different speeds holding the ceilings' ratio.

**The Xbox %-ladder stays SHARED, deliberately.** It is one physical control for
the whole "p" group; per-pump percentages would leave its buttons with no
defined meaning. Independence comes from the ceiling, which is the
safety-relevant half. No limits configured → falls back to the legacy scalars
and jogs exactly as before.

### Deliberately NOT changed

* **Per-pump STEP SIZE.** `jog_button_array.py` builds one `_p_step` group for
  P1/P2/P3, so the step is shared *even where the rate is per-pump*. The
  operator asked for **rate**; if they also want each pump to move a different
  *amount* per click, that is a separate, genuinely-missing feature and should
  be scoped on its own.
* `stage_panel.py::_on_jog_array_pump` — dead code (no `JogButtonArray` is built
  there any more) that calls `move_pump_relative` with no feedrate. Zero
  connections today, so no runtime effect; left in place and recorded here
  rather than removed as a drive-by.

---

## 2. The Live Position row — the number drew on top of the units

`PositionValueLabel` (v7.9.x) set **both** `QSizePolicy.Ignored` horizontally
**and** a `minimumWidth` that tracks its text. Those fight:

* `Ignored` tells the **layout** to size the grid column without regard to this
  widget.
* `setMinimumWidth` forces the **widget** to stay that wide regardless.

Measured on the real panel: the value label was **108 px wide inside a ~5 px
cell** (`val x=435 w=108`, `unit x=440`), so the right-aligned number was
painted ~100 px on top of the unit label — at *every* width from 540 px down to
200 px, not just narrow ones.

Fixed by making the policy `Preferred`, so the layout actually reserves the
width. The **bar** keeps `Ignored` + a 10 px minimum and remains the element
that yields, which is what lets a narrow panel scrunch without clipping the
number. Verified: a clean +5 px gap at 540/380/300/260/200/160 px, no clipping,
bar still shrinks. `PositionReadoutCard` (the Custom panel) shares the class and
is fixed with it (+8 px gap down to 100 px).

⚠ The original docstring's reasoning — *"the policy stays Ignored so the layout
reads the minimum, never the raw text sizeHint"* — is the bug, stated as intent.
The comment now says so, because `Ignored` looks deliberate.

⚠ **My first measurement was wrong and nearly hid this.** Comparing text width
against the label's own box showed "OK" at every width, because the *box* was
fine — it was the box versus its *cell* that was broken. Only measuring the
widgets' actual geometries against each other exposed it.

---

## Files Modified

| File | Why |
|---|---|
| `gui/pages/settings_page.py` | Gap A: resolve and pass a per-pump `rate_uL_s`. |
| `SupportClasses/StageController.py` | Gap B: `_xbox_pump_ceiling_uL_s` / `_xbox_pump_pct`; `_pump_vel_mm_s` uses the per-pump ceiling. |
| `gui/pages/hardware/control_panel.py` | `PositionValueLabel` policy `Ignored` → `Preferred`. |
| `tests/test_v791_per_pump_rate_and_position_row.py` | NEW — 17 tests. |

---

## Testing Notes

### Mutation verification — **5/5 CAUGHT**

| # | Mutation | Result |
|---|---|---|
| R1 | Settings panel sends no rate again | CAUGHT |
| R2 | Settings panel uses one shared ceiling | CAUGHT |
| R3 | Xbox back to the shared ceiling | CAUGHT |
| R4 | Position label back to `Ignored` policy | CAUGHT |
| R5 | Minimum width stops tracking the text | CAUGHT |

### Suites

`tests/test_v791_per_pump_rate_and_position_row.py` — **17 green**.

Regression, all green: per-pump-jog-flow · jog-step-sliders · axis-max-speed ·
common-axis-speed · xbox-axis-speed-percent · xbox-input-pipeline ·
jog-pump-fill · context-panel · responsive-context-panel · jog-navigation ·
pump-plunger-setup · jog-direction-z-up-sign (**237**), plus stage-jog-off-thread
· illumination · nikon-ti · xz-custom-z · the v7.9.1 calibration suite ·
suite-hygiene (**233**), plus a `gui.app` import smoke.

### Needs GUI / HW verification on ME3B V1

1. **Live Position**: the number must sit clear of its unit on the Jog page and
   in the Custom panel, at full width and when the pane is dragged narrow.
2. Set P1 and P2 to visibly different jog flows on any tile; jog each and
   confirm they move at different speeds.
3. Open **Settings**, jog a pump from its context panel, and confirm it now
   moves at that pump's configured flow rather than the old fixed default.
4. **Xbox**: with two pumps whose bores differ, hold the stick fully on each and
   confirm the finer bore runs slower. The %-ladder buttons still scale both.
5. Confirm the % set on one page still shows on every other page and survives a
   restart (unchanged behaviour — worth re-checking since the Settings panel now
   reads that same state).

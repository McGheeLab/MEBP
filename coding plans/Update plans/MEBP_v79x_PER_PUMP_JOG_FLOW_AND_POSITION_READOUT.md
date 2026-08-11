# MEBP v7.9.x — Per-Pump Jog Flow Rate on Every Jog Tile + Position Readout Text Priority

## Objective

Two operator reports (2026-08-07):

1. *"on all jog tiles, each pump should get its own flow rate. Currently this
   only exists on the hardware setup page which is a special version of the
   jog controls."*
2. *"the current position of each axis has the scroll bar being too long so
   we cant see the text of the position since it overlays other text."*

### Root causes

**(1) One shared pump % anchored to the FASTEST pump.** The shared jog panel
(`HardwareControlPanel` in percent mode — every jog tile via
`StandardJogContextPanel`) had exactly ONE pump jog-speed spin (`spin_p_pct`)
whose 100% anchor was `StageController.get_max_pump_feedrate()` =
**`max(get_max_flow_rate(pid) for pid in configured)`** — the fastest pump's
ceiling. So P1/P2/P3 all jogged at the same absolute µL/s regardless of
syringe/bore, and a % chosen for a coarse pump over-anchored a fine pump by
the ratio of their ceilings (the safety clamp downstream prevents damage, but
the UI could not express a per-pump flow at all). Hardware Setup's per-pump
rows (`spin_p_max_pumps`) exist only under `speed_as_max=True` and edit the
per-pump mm/min *ceiling*, not a jog rate on the tiles.

**(2) The bar had width priority over the number.** Each Live-Position row is
`axis | PositionBar (the row's only stretch column) | value | unit`. The value
label sat behind a fixed tiny floor (`s(20)`) with an `Ignored` size policy
(a v7.5.x narrow-panel measure), so at ordinary widths the bar absorbed all
slack and the number was clipped to its low-order digits, visually running
into the unit label — the operator's "scroll bar too long / text overlays
other text". Duplicated with different floors in the Custom-panel
`PositionReadoutCard` (bar kept `Expanding` + `s(60)` min there).

## Files Modified

| File | Change |
|---|---|
| `SupportClasses/StageController.py` | NEW `get_max_pump_feedrate_for(pump)` (one pump's OWN needle/syringe-derived flow ceiling, µL/s; 10 µL/s default when no limit computed; legacy no-config delegates to `_pump_jog_max_native`) + NEW per-pump jog-% store `_pump_jog_pct` with `set_pump_jog_pct` / `get_pump_jog_pcts` — held on the controller (not the panels) so the nine jog tiles share ONE state (the illumination-LED lesson). |
| `gui/pages/hardware/control_panel.py` | Percent mode builds **one flow row per pump** (`spin_p_pct_pumps` / `lbl_p_resolved_pumps` / `row_p_pct_pumps`, sub-1% capable, unconfigured pumps hidden — mirrors the max-mode rows); NEW `_pump_speed_anchor_for(pump)` (µL/s via the per-pump resolver on %/µL pages; per-pump mm/min override on the mm path; falls back to the shared anchor on older stubs); `_on_jog_pump` uses the CLICKED pump's own % × its own anchor in both branches; `_on_pump_pct_changed` pushes the controller store + persists settings `jog.pump_jog_pct`; `_seed_pump_jog_pcts` (controller → settings [pushed back onto the controller] → shared 'p' % → default). NEW `PositionValueLabel` — min width tracks its text (re-synced on `setText` + `FontChange`, floor `s(20)`, cap `s(160)`), used by the Live Position grid; the bar keeps `Ignored` + `s(10)` min so it is the element that yields. |
| `gui/widgets/context_sections.py` | `PositionReadoutCard` matched to the control-panel copy: bar → `Ignored` + `s(10)` min, value label → `PositionValueLabel` (was a fixed `s(70)` floor next to an `Expanding` `s(60)`-min bar, which overflowed narrow panes instead). |
| `tests/test_v75x_axis_max_speed_inputs.py` | Test 1 updated: the sub-1% floor is asserted on each per-pump spin. |
| `tests/test_v75x_per_pump_jog_flow.py` | NEW — 14 tests (see Testing Notes). |

## Implementation Steps

- [x] `StageController.get_max_pump_feedrate_for` + per-pump jog-% store
- [x] Per-pump flow rows in `HardwareControlPanel` percent mode (build, resolve, hide-unconfigured, seed, persist)
- [x] `_on_jog_pump` routes the clicked pump's own % × its own ceiling (µL branch AND the legacy mm branch)
- [x] `PositionValueLabel` + Live Position grid rework (control panel)
- [x] `PositionReadoutCard` matched (Custom panel)
- [x] Tests: new suite (14) + updated axis-max test; mutations verified
- [ ] Bench verification on ME3B V1 (below)

## Testing Notes

New `tests/test_v75x_per_pump_jog_flow.py` (**14 green**): one row per pump ·
resolved labels per-pump · a jog uses the clicked pump's own % × its OWN
ceiling (P1 and P2 at different %/ceilings in one test — catches
"always read P1's spin" and "shared fastest-pump anchor" mutations) · edit
pushes the controller store + persists settings, and a SECOND panel on the
same controller seeds the same values · restart re-seeds from settings and
repopulates the controller · shared 'p' % remains the fallback seed
(pre-per-pump installs) · unconfigured rows hide · controller anchor
per-pump / 10 µL/s default / legacy delegate · pct-store round-trip returns a
copy + rejects non-numeric · `PositionValueLabel` min width tracks text and
font changes, capped · both position surfaces have a yielding bar
(`Ignored`, ≤ `s(10)` min) and text-tracking value labels.

**2 mutations confirmed CAUGHT** (each a real source edit, reverted after):
`_pump_speed_anchor_for(pump)` → shared `_pump_speed_anchor()` fails the
per-pump jog test; `PositionValueLabel._sync_min_width` → the legacy fixed
`s(20)` floor fails all 3 position tests.

Regression, run per-suite, all green: axis-max-speed-inputs (6) ·
common-axis-speed-source (18) · per-pump-max-rate (9) ·
stage-jog-off-gui-thread (3) · responsive-context-panel (15) ·
context-panel (21) · jog-pump-fill-readout (16) · xbox-axis-speed-percent
(20) · jog-motion-interpolation (29) · pump-plunger-setup (27) ·
jog-navigation (28) · nikon-ti-microscope (101) · test-suite-hygiene (10) ·
plus a `gui.app` import smoke.

### Needs real-HW/GUI verification on ME3B V1

1. Open the Jog pill on any workflow page — three flow rows P1/P2/P3 (only
   configured pumps shown), each with its own resolved µL/s.
2. Set P1 and P2 to different % — aspirate/dispense clicks on each pump run
   at visibly different rates matching the resolved readouts.
3. Change a % on the Jog page, switch to Calibration → the same value shows
   there (shared controller state).
4. Restart → the per-pump values are restored (`jog.pump_jog_pct`).
5. Live Position: at normal panel width the full number is readable (no
   clipping into the unit); at a very narrow panel the BAR shrinks first and
   the shrunken font keeps the number fitting.

## Issues & Decisions

- **Per-pump state lives on the controller, not the panels** — nine pages
  each build their own `StandardJogContextPanel`; per-panel state silently
  diverges (the v7.5.x illumination-LED bug, verbatim). Panels re-seed from
  `get_pump_jog_pcts()` on every show; settings are the restart persistence
  and are pushed back onto the controller at seed time.
- **The shared 'p' group % is kept, untouched** — it remains the Xbox
  pump-jog ladder value (`set_jog_speed_pct('p', …)` / the controller ladder)
  and the seed fallback for pre-per-pump installs. Panel spins no longer
  write it; the Xbox page is the surface for that value.
- **Anchor = the pump's own `SafetyLimits.get_max_flow_rate(pid)`** (the
  needle/syringe-derived safe ceiling), not `pump_max_feedrate_uL_s` — it is
  the same family of value the old shared anchor maxed over, so 100% keeps
  meaning "this pump's safe ceiling"; the flow clamp downstream is unchanged.
- **Legacy mm branch** (percent mode + raw ▲/▼ arrows — tests only in
  production) now also resolves per-pump via
  `get_pump_max_feedrate_mm_min(pid)` so its behaviour matches the per-pump
  model instead of keeping a third semantic.
- **Position fix inverts the yield priority instead of capping the bar** —
  the value label claims exactly its current text width (re-synced on text
  and font changes so the responsive font scaler keeps narrow panels
  fitting), and the bar, which paints fine at any width, is the element that
  shrinks. A cap on the bar would have left the label clipped at mid widths.
- `PositionReadoutCard`'s old `s(70)`-floor label + `Expanding` `s(60)`-min
  bar made that card's minimum ~180 px — above the 100 px context-pane
  minimum, so it overflowed/clipped instead of scrunching; it now matches the
  control-panel geometry exactly.

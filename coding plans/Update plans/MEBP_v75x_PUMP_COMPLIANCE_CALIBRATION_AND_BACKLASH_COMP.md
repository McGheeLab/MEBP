# MEBP v7.5.x — Pump Compliance Calibration + Backlash Compensation

## Objective

Give each pump a **measured compliance value `c` (µL, per pump)** and use it to
compensate for drivetrain/syringe/tubing flex ("backlash") at every pump
start/stop, so small-volume flows are accurate and the tip is pressure-neutral
during stage travel.

Operator request (2026-07-09):

1. Express "pressure relief" in **µL, per pump** (retire the `% of syringe`
   model) and surface it on the global **Common Print Settings** page.
2. A guided **per-pump calibration** on the **Needle Location** tab: dispense in
   0.1 µL steps until a droplet forms, aspirate `A` until it's pulled back in,
   store `c = A/2`.
3. **Backlash compensation** at every pump start/stop — *take-up on reversal +
   unload on stop* — with an on/off **toggle in the pump jog panel** ("no
   residual pressure in the system during moves").
4. A **Quick Start Guide** section explaining the physics + procedure.

## The physics

Pulling the plunger back loads the drivetrain flex one way; to then push, you
must first overcome that flex before fluid moves, then load the flex the other
way. `A` (the aspirate to pull a formed droplet back in) is the full backlash
span; `c = A/2` is the µL needed to remove residual flex in one direction.

**Comp cycle:** bracket a fluid move `V` with take-up `+c` (fluid direction,
loads flex) → fluid `V` → unload `−c` (pressure-neutral). Net plunger = `V`;
net fluid ≈ `V`; residual pressure ≈ 0.

## Design decisions (operator-confirmed)

- **Per-pump only** relief µL (no scalar global, no `%`). Shown as a P1/P2/P3
  table on Common Print Settings. `c` = calibrated compliance.
- **Backlash comp = take-up on reversal + unload on stop (both)**, single global
  enable toggle in the pump jog panel. Per-pump `c=0` ⇒ no-op for that pump.
- **Retire** the v7.5.x `pump_relief_percent` model (% spin + 3
  `pump_relief_on_*` toggles + the `relieve=` path). The new comp subsumes the
  old pickup/deposit relief (dispense-back after aspirate / suck-back after
  dispense = the "unload" half).
- **Safety invariant:** volume-balanced spheroid/cell micro-captures stay
  net-zero — they pass `compensate=False` (a µL-scale `c` would dwarf their nL
  captures). Mirrors the old `relieve=False` exemption.
- **Auto-comp is gated on `settle=True`** so the streamed print path
  (`move_pump_uL(settle=False)` / per-segment `move_pump_relative`) never
  brackets. Print path keeps its existing prime (take-up) + `_print_pump_suckback`
  (unload) hooks, re-gated on the new toggle.
- **Calibration UI = inline card on the Needle Location tab** (reuses the side
  needle cameras; no XY/Z travel). Pump steps run on a daemon worker so the live
  feed doesn't freeze. Calibration steps run with `compensate=False`.

## Files Modified

| File | Change |
|---|---|
| `gui/pages/hardware/device_profile.py` | `pump_compliance_uL` dict + `backlash_comp_enabled` bool + serialize/settings-bridge |
| `SupportClasses/StageController.py` | per-pump relief state + readers/setters; `apply_pump_relief`; backlash toggle; comp engine in `move_pump_uL(compensate=)`; retire `pump_relief_percent`/`_flag`/`relieve` block |
| `SupportClasses/HardwareConfig.py` | remove `pump_relief_percent` + 3 toggles; ignore legacy keys on load |
| `main.py` | restore `pump_compliance_uL` + `backlash_comp_enabled` at startup |
| `SupportClasses/PrintManager.py` | re-gate `_print_pump_suckback` on `backlash_comp_enabled()` |
| `SupportClasses/PickAndPlaceManager.py` | `_settled_pump_move` `relieve=`→`compensate=`; captures `compensate=False`; drop `relieve=True` at reagent pickups |
| `gui/pages/calibration.py` | compliance-calibration card on Needle Location tab + worker + persist |
| `gui/pages/hardware/control_panel.py` | jog take-up/unload hooks in `_on_jog_pump` |
| `SupportClasses/StageController.py` (`ZPJogHandler`) | Xbox continuous-jog start/stop comp |
| `gui/widgets/pump_rack.py` / `gui/pages/jog_control.py` | "Backlash compensation" toggle |
| `gui/pages/workflows/common_print_settings_workflow.py` | per-pump relief µL table + enable checkbox; drop % row |
| `SupportClasses/CommonPrintSettings.py` | drop `pump_relief_percent` global |
| `gui/pages/hardware_setup.py` | remove % spin + 3 checkboxes |
| `quickstart_guide/{guide_content.json,build_guide.py}` | new guide section |
| `tests/test_v75x_pump_compliance_and_backlash.py` | new test suite |

## Implementation Steps

- [x] DeviceProfile: `pump_compliance_uL` + `backlash_comp_enabled` + bridge
- [x] StageController: per-pump relief state/readers/setters + `apply_pump_relief`
- [x] StageController: backlash toggle + comp engine in `move_pump_uL(compensate=)`
- [x] HardwareConfig: remove % relief + toggles; ignore legacy keys
- [x] main.py: startup restore
- [x] PrintManager: re-gate suck-back on `backlash_comp_enabled` + EXTRUDE `compensate=False`
- [x] PickAndPlaceManager: `relieve`→`compensate`; captures `compensate=False`; reagent sites `compensate=None`
- [x] calibration.py: Needle Location calibration card + worker + persist
- [x] control_panel: jog-click comp hook (`_on_jog_pump`)
- [x] jog_control: backlash toggle (Pumps card)
- [x] common_print_settings_workflow: per-pump relief table + enable checkbox
- [x] CommonPrintSettings: drop % global
- [x] hardware_setup: remove % spin + checkboxes (→ note)
- [x] 4 workflow pages: drop the `pump_relief_percent` common rows
- [x] quickstart guide section + regenerate PDF
- [x] Tests + run affected suites
- [ ] **Xbox continuous-jog comp — deferred follow-up** (real-time velocity loop;
      per-increment bracket is wrong, start/stop injection is risky). On-screen
      click-jog comp + discrete-actuation comp cover the operator's ask.

## Testing Notes

- New suite `tests/test_v75x_pump_compliance_and_backlash.py`: DeviceProfile
  round-trip (dict + settings bridge); controller relief state (set/get/all/apply
  + toggle); comp engine (take-up `+c` → fluid `V` → unload `−c`, net = `V`;
  `compensate=False`/toggle-off/`c=0` no-op; `settle=False` never auto-comps;
  `compensate=True` forces on); HardwareConfig legacy keys discarded; offscreen
  Needle-Location card (builds, `c = A/2`, Save → controller + settings).
- Updated `test_v75x_pump_settle_and_prime_time.py` (relief % → per-pump µL comp;
  `relieve`→`compensate`), `test_v75x_pump_relief_and_bead_model.py` (suck-back
  now gated on `backlash_comp_enabled`), `test_v75x_common_print_settings.py`
  (dropped `pump_relief_percent` global → use `pump_prime_time_s`).
- Green suites run: compliance/settle/relief-bead/common-settings (91),
  spheroid+quick-print pick&place (62), cell-targeting/cell-labeling/
  spheroid-disengage/quick-print-travel-split/pump-plunger/print-setup-routine
  (109), workflow-settings-popout + full-print (55), print-path/manager/simple
  (24), needle-max-flow(Pump page)/plate-types/gentle-descent (60), jog-nav/
  jog-fill/jog-travel (48).
- **Needs real-HW verification on ME3B V1** (calibrate c per pump on Needle
  Location; enable the jog toggle; confirm take-up-on-start + unload-on-stop and
  that small volumes flow / no drool during travel; spheroid pickup still exact).

## Issues & Decisions

- (decision) Reader stays `pump_relief_uL(pump)` (used by PrintManager; matches
  operator "pressure relief value" vocabulary); the stored per-pump value is the
  physical compliance `c`.
- (decision) Print-path comp is conservative — reuses prime + `_print_pump_suckback`
  at path boundaries; no new per-segment motion. Full trajectory take-up/unload
  is a follow-up if needed.
- (decision) Auto-comp is gated on `settle=True` so the streamed print path and
  manual-jog passthrough are never auto-bracketed; the PrintManager discrete
  EXTRUDE/prime passes `compensate=False`; pick&place captures pass
  `compensate=False`; reagent pickups pass `compensate=None` (auto per toggle).
- (deferred) Xbox continuous-jog comp — real-time velocity loop; on-screen
  click-jog + discrete-actuation comp cover the ask.

## Addendum (2026-07-09b — calibration-UX refactor: own tab + 5-step reset-able flow + Needle Offset merge)

Operator feedback after using the shipped feature: it works, but they want to be
able to **reset/retry** the measurement and a **cleaner guided flow**, the
calibration on **its own tab**, and the old **Needle Offset Calibration** tab
**merged into Needle Location** ("basically the same concept"). This is a
UI-layer refactor only — the comp engine, `move_pump_uL(compensate=)`,
`apply_pump_relief`, the jog toggle, the Common Print Settings table, and the
persistence keys are **unchanged**.

- **Own tab.** New `CalibrationPage._build_pump_compliance_tab` — a vertical
  splitter: live **NEEDLE_X / NEEDLE_Y side-camera** views on top (`_compcal_cam_slots`
  + `_compcal_refresh_cameras` / `_compcal_ensure_live_cameras`, mirroring the
  needle-location binding; view-only, no edge-pick clicks) and the guided
  controls (the relocated `_build_compliance_cal_group`, now inside a scroll
  area) below. Registered at **index 1** (`_compcal_tab_index = 1`), right after
  Needle Location. `_on_workflow_tab_changed` starts the needle cameras when the
  tab is shown.
- **5-step reset-able flow.** Added `_compcal_phase` ∈ {`prep`, `measure`,
  `done`} + `_compcal_update_phase_ui` (guidance note + per-phase button
  enable-states; pump selector locks once measuring). New **`_compcal_start`**
  (step ② — zeroes the counters, `prep → measure`; no pump move). The old "Drop
  back in ✓ (compute c)" button is now **"④ Finish (compute c)"** — it requires
  the `measure` phase, computes `c = A/2`, and moves to `done`. `_compcal_on_step_done`
  now only feeds `A` **after Start** (measure phase; a stray dispense subtracts,
  net floored at 0); prep-phase moves only update an informational free-dispense
  tally. **`_compcal_reset`** returns to `prep` and clears the computed value.
  The manual override spin + Save stay live throughout (pure-manual path
  preserved). The daemon-thread step pattern + `_CompCalBridge` + `compensate=False`
  (measure the *uncompensated* compliance) are unchanged.
- **Merge.** `_build_z_offset_tab` renamed to **`_build_z_offset_content`** (same
  scrollable widget: reference-Z-heights grid + `_zoff_xz_view` + estimate +
  save-to-type + go-to-Replace-Z — every widget/handler name identical).
  `_build_needle_location_tab` now wraps the needle-centring column and that
  content in an **outer horizontal `QSplitter`** (centre in the side cameras
  left, set reference-Z on the XZ elevation right). The standalone tab +
  `_zoff_tab_index` are removed. Net tab count is unchanged (−1 Offset, +1
  Compliance), so `_zauto_tab_index` stays **3**.

**Adversarial review** (independent read of the final code) → 2 confirmed
gating gaps fixed: (1) the Aspirate button was always enabled — now gated to the
`measure` phase (aspirate only counts after Start; Dispense stays live in
`prep`+`measure` for droplet-forming + net self-correction); (2) Save was always
enabled — now enabled in `done` **or** when the manual spin holds a value > 0
(preserves the manual-override path while blocking an accidental bare-0 save; the
manual spin's `valueChanged` refreshes the gating). Non-bugs verified clean: the
busy→phase-UI ordering, the net-A sign/floor logic, `_zoff_live_view` sole
ownership by the autocal tab, and no lingering `_zoff_tab_index`/`_build_z_offset_tab`
references.

**Tests.** `test_v75x_pump_compliance_and_backlash.py`: `TestComplianceCalCard`
reworked for the phased flow (start-zeroes, measure-net-A, prep-does-not-feed-A,
finish-computes-half, finish-before-start-noop, reset-returns-to-prep) + new
`TestCalibrationTabRestructure` (5 tabs / no Offset title / `_compcal_tab_index==1`
/ `_zauto_tab_index==3` / no `_zoff_tab_index` / Z-refs merged onto the page /
compliance camera slots). `test_v75x_plate_z_autocal_tab.py` updated to the new
tab order + camera triggers. Green: compliance (28), plate-z-autocal-tab (9),
calibration-revision / plate-location-click-rim / plate-z-autocal-per-well /
needle-location-quick-move / needle-offset-z-side-view / plate-location-z-side-view
/ reanchor-mosaic / single-well-mosaic (all green). Pre-existing unrelated fail:
`test_v75x_plate_mosaic::test_real_24_well_mosaic` (documented CV). Guide section
`pump_compliance_calibration` rewritten for the own-tab + 5-step + Reset flow; PDF
regenerated.

## Addendum (2026-07-24) — calibration steps are RAW moves (no settle, no comp)

Operator: "when doing the calibrate compliance, do not use the settle time or
the compensation when doing the pump motion." The step move already passed
`compensate=False` (the calibration measures the compliance, so it must not
apply comp itself) but still ran `settle=True`, which brackets the move with
the global `pump_settle_time_s` dwell. `_compcal_step_move`'s worker now calls
`move_pump_uL(..., settle=False, compensate=False)` — a raw plunger step, the
true measurand. Because `settle=False` also skips `move_pump_uL`'s internal
completion block, the worker drains the move itself via
`StageController._wait_pump_move_complete(est_s)` (M400, poller suspended;
open-loop sleep fallback when unavailable) before emitting the done signal, so
the busy-guard still holds until the pump physically stops. The kwargs
`TypeError` fallback (older/fake controller) is scoped to the move call only so
a wait-helper signature mismatch can't re-issue (double) the move. New test
`test_step_move_is_raw_no_settle_no_comp` locks the kwargs + the worker-side
completion wait; also fixed the stale `test_tabs_restructured` expectation
(the concurrent Rosettes tab at index 3 → `_zauto_tab_index == 4`). Suite: 31
green.

## Status

`[~]` Code + tests complete — 2026-07-09 (calibration-UX refactor 2026-07-09b;
raw-step addendum 2026-07-24).
**Needs real-HW verification on ME3B V1** (Pump Compliance tab: droplet visible in
the live side view; dispense → Start (zeroes) → aspirate → Finish (c=A/2) → Save;
Reset retries; saved µL shows on Common Print Settings and the jog backlash toggle
applies it; Needle Location tab also shows the reference-Z heights; step moves
run with no settle dwell and no comp bracketing).

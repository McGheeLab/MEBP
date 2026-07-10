# MEBP v7.5.x — One common per-axis speed source + needle-derived pump flow + %/µL pump units

## Objective

Make every page inherit per-axis SPEED from a single common source (no more independent
per-page derivation), express jog speed as a **% of the calibrated max everywhere**, make the
pump max speed a **needle-derived flow rate** (µL/s, gauge + length), and switch pump
move/jog units so **Hardware Setup uses mm** (raw plunger, for calibration) while **all other
pages use % of syringe volume input with a µL + fill-% readout**.

Confirmed product decisions:
1. Jog speed = % of the single per-axis calibrated max on every page.
2. Pump max flow ceiling = `FlowPhysics.max_safe_flow_rate_uL_s(needle, ink, ΔP)` with a fixed
   **reference viscosity** (water, ink-independent) — replaces the `GAUGE_MAX_FLOW` table.
3. Non-Hardware-Setup pump moves: input %, show µL (+ current fill µL and %).
4. Timing-Calibration "Measure top speed" **writes the single XY value**.

## Files Modified

- `SupportClasses/StageController.py` — speed resolvers, µL↔% pump helpers, redirect internal
  reads, `notify_speed_limits_changed`/`on_speed_limits_changed`.
- `SupportClasses/SafetyLimits.py` — needle-derived per-pump flow ceiling + `REFERENCE_VISCOSITY_CP`.
- `gui/pages/hardware/control_panel.py` — %-of-max speed UI, %/µL pump jog + readout, refresh slot.
- `gui/widgets/jog_button_array.py` — percent pump-step mode (driven off `pump_action_labels`).
- `gui/widgets/standard_jog_context.py` — `refresh_speed_limits` forwarder + needle-flow readout.
- `gui/pages/workflows/quick_print_workflow.py` — `_xy_max_mm_s`/`_z_max_mm_s` → resolvers.
- `gui/pages/workflows/timing_calibration_workflow.py` — write the one XY value + broadcast.
- `gui/pages/hardware/stage_panel.py` — per-axis-Z emit, resolver-preferring Z read, pump tooltip.
- `gui/app.py` — extend `safety_limits_changed` fan-out + `controller.on_speed_limits_changed`.
- `tests/test_v75x_common_axis_speed_source.py` — new.

## Implementation Steps

- [x] Part 1 — StageController resolvers (`get_max_xy_speed_um_s`, `get_max_z_feedrate_mm_min`,
  `get_max_pump_feedrate`) + `notify_speed_limits_changed`/`on_speed_limits_changed`; redirect
  `refresh_jog_speed_limits` + `_refresh_zp_move_feedrates`.
- [x] Part 3 — `SafetyLimits.update_from_hardware_config` needle-derived flow ceiling + `REFERENCE_VISCOSITY_CP`.
- [x] Part 4a — StageController `pump_effective_capacity_uL`/`pump_pct_to_uL`/`pump_uL_to_pct`/`pump_fill_pct`.
- [x] Part 2 — Control Panel %-of-max speed spinboxes + resolved labels + rewired discrete jog + shared jog-%.
- [x] Part 4b — `jog_button_array` percent pump-step mode + control_panel %/µL pump jog + µL/fill-% readout.
- [x] Part 1/3 reads — Quick Print (resilient: resolver else legacy inline) + stage_panel Z read → resolvers;
  needle-flow read-only readout in the shared hardware-info card. (`max_pump_feedrate` spin is a headless
  shadow — no visible tooltip to change; kept as the legacy mm-jog/no-needle fallback.)
- [x] Part 5 — `refresh_speed_limits` slots + app.py fan-out + per-axis-Z emit + Measure-top-speed writes the one value.
- [x] Tests — new `tests/test_v75x_common_axis_speed_source.py` (18) + updated `test_v75x_quick_print_pick_and_place.py`.
- [x] Adversarial review of the diff (safety invariants untouched).

## Status Tracking

Implementation complete; resolver precedence settled on **safety_limits.max_xy_speed primary** (the one
editable value; the timing tool's measurement WRITES it) rather than measured-preferred — this is what makes
"set the speed → all pages inherit" true. The XYStage SMS denominator (`_max_speed_um_s`) stays seeded from
the timing store at connect (physics), separate from the anchor.

## Testing Notes

`python -m pytest tests/test_v75x_common_axis_speed_source.py tests/test_v75x_xbox_axis_speed_percent.py
tests/test_v75x_print_timing_calibration.py tests/test_v75x_xy_speed_conversion.py
tests/test_v75x_quick_print_pick_and_place.py tests/test_v75x_pump_plunger_setup.py
tests/test_v731_jog_navigation.py -q`

Real-HW (ME3B V1): set XY max on one page → reflected on all; jog % matches resolved speed on
Control Panel + Xbox; Hardware Setup pump jog = mm; other pages = % step → correct µL + fill %;
needle gauge/length change → pump max flow (µL/s) updates; "Measure top speed" propagates.

## Test results

Green (unittest): `test_v75x_common_axis_speed_source` (18, incl. offscreen Control-Panel
mode smoke), `xbox_axis_speed_percent`, `print_timing_calibration`, `xy_speed_conversion`,
`pump_plunger_setup`, `test_v731_jog_navigation`, `quick_print_pick_and_place` (46),
`quick_print_workflow`, `print_setup_routine`, `workflow_settings_popout`, `spheroid_pick_place_z`,
`cell_targeting_removal`, `cell_labeling`, `xbox_input_pipeline`, `z_retract_before_xy_travel`,
`print_always_safe_z`, `printing_mode_calibrated_wells`, `zp_feedrate_inheritance_fix`,
`simple_print_manager`.

**Pre-existing, NOT from this change:** `test_v75x_pump_settle_and_prime_time` (7) +
`test_v75x_common_print_settings` (3) fail on `HardwareConfig.pump_relief_volume_uL` — the working
tree has a concurrent in-flight refactor replacing the absolute `pump_relief_volume_uL` with
`pump_relief_percent`, which those (older) tests haven't caught up to. None of these symbols are
touched by this change.

## Issues & Decisions

- Reference viscosity = water (1.0 cP): least viscous → most permissive geometry-only ceiling;
  per-ink/per-move flow safety still applies elsewhere. Constant lives in `SafetyLimits.py`
  (avoids a circular import into `FlowPhysics`).
- `pump_action_labels` (existing flag on `HardwareControlPanel`) is the mm-vs-%/µL discriminator —
  one shared widget, no per-page edits for the unit switch.
- `PrintTimingCalibrationStore.xy_max_speed` stays (restart persistence + phase-lag model) but is
  no longer a direct speed READ path; the resolver routes through `xy_stage._max_speed_um_s()`.
- Workflow process-volume/flow parameters stay native µL (scientific dosing, not "pump moves").
- Legacy `max_pump_feedrate` (mm/min) kept as Hardware-Setup mm-jog speed + no-needle fallback.

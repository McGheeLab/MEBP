# MEBP v7.5.x — needle-derived max flow: display + flow-bounded print speed

## Objective
`changes_needed.md` item 6: "we need a setting for max pump flow rate given the
needle setup… too high… causing air to build up… This max flow rate calculation
should be shown in the settings. Also the max print speed should be calculated given
that we must obey this limit. Also, when picking up or depositing ink/materials,
pump flow rates should be settings the user has control over." Operator chose
**physics calc + hard per-pump clamp, user-overridable**.

## State at start (existing WIP — already done)
- `SafetyLimits.update_from_hardware_config` computes the per-pump ceiling via
  `FlowPhysics.max_safe_flow_rate_uL_s(needle, water-ref, DEFAULT_PRESSURE_LIMIT_PA)`
  (Hagen–Poiseuille, `REFERENCE_VISCOSITY_CP=1.0`) and `StageController.move_pump_uL`
  hard-clamps every pump move to it → the **safety** (air-ingestion) goal was met.
- `StageController.get_max_xy_speed_um_s` / `get_max_z_feedrate_mm_min` speed
  resolvers + Quick Print delegation, and per-workflow pickup/deposit rate spinboxes
  (spheroid pickup/release, cell-targeting push/pull, cell-labeling deposit/aspirate)
  already existed.

## Added here (the remaining gaps)
- **(a) Hardware Setup → Pump readout.** `gui/pages/hardware_setup.py`: new
  `_pump_maxflow_lbl` in the Pump Timing group + `_refresh_max_flow_display()` that
  recomputes the ceiling from the configured needle (same FlowPhysics + reference
  viscosity as SafetyLimits) and shows "Max safe pump flow (NNG, water ref): X.XX
  µL/s — hard-capped on all pumps; bounds the max print speed". Refreshed on every
  `_on_config_changed` (after rebuild) and on `_apply_config_to_ui`.
- **(b) Flow-bounded print speed.** `gui/pages/workflows/quick_print_workflow.py`:
  - `_max_pump_flow_uL_s()` reads `safety_limits.get_max_flow_rate(pump)`, **guarded**
    against a bare MagicMock (`__float__`→1.0) by requiring a real numeric return.
  - `_flow_limited_xy_max_mm_s()` = `min(xy_max, max_flow / (bore_area × modifier))`.
  - `_auto_flow_100_uL_s` + `_resolved_print_kinematics` use the flow-limited XY max
    for BOTH speed and flow → flow@100% never exceeds the ceiling and the bead width
    (volume/mm = area × modifier) stays correct (no silent under-extrusion).

## Implementation Steps
- [x] Hardware Setup max-flow label + `_refresh_max_flow_display` (+ refresh hooks)
- [x] `_max_pump_flow_uL_s` (MagicMock-guarded) + `_flow_limited_xy_max_mm_s`
- [x] Bind both speed + flow anchors to the flow-limited XY max
- [x] Tests `tests/test_v75x_needle_max_flow_and_pickup_rates.py` (6)

## Testing
`tests/test_v75x_needle_max_flow_and_pickup_rates.py` — MagicMock guard → 0 (no bogus
1 µL/s limit); a real ceiling at ½ the unbounded flow@100% halves the XY max so
flow@100% lands exactly on the ceiling; a huge ceiling never raises the speed;
Hardware Setup label shows µL/s for a needle and "—" with no bore. quick-print
pick&place / seam / pump suites stay green.

## Issues & Decisions
- The hard clamp (safety) was already present; this layer makes the limit VISIBLE and
  keeps the bead correct by reducing SPEED rather than letting flow be clamped.
- Pickup/deposit rate settings (item 6's third clause) were already exposed by the
  existing workflow settings dialogs; no change needed.

## Needs real-HW verification on ME3B V1
Set a fine-gauge needle → displayed max drops; an over-fast flow is clamped (log
line); pickup no longer ingests air; Quick Print speed auto-reduces when flow-bound.

# MEBP v7.5.x — Pick & Place prep routine (oil → wash → buffer) + needle-volume unit

## Objective

Add the operator-specified needle-conditioning **prep** that runs once before the
pick & place loop, with service-well locations **inherited from Hardware Setup**:

**Prep (once):** 1) safe-travel→waste, expel 1 needle of oil · 2) →oil, draw 1
needle of oil · 3) →wash, wash the needle · 4) →buffer, draw 4 needles of buffer.

**Loop (unchanged):** safe-travel→pick, pick spheroid; safe-travel→place, place;
repeat over the picked pairs.

Decisions (operator-confirmed):
- **Service-well locations** are inherited from **Hardware Setup → Ink → Reagent
  Locations** — a well is matched to a role by the assigned ink's `ink_type`
  (`oil`/`wash`/`waste`/`buffer`).
- **"1 needle's worth"** = the needle's **inner-bore cylinder volume**
  `π·(id/2)²·length` (µL).
- **Wash** = dip into the wash well, jiggle Z up/down, and jiggle random XY
  vectors about the well centre, then recentre.

## Implementation

- **`SupportClasses/PhysicalModels.py`** — `NeedleSpec.internal_volume_uL`
  property = `cross_section_area_mm2 × length_mm` (inner bore, 1 mm³ = 1 µL).
- **`SupportClasses/PickAndPlaceManager.py`**
  - `PickPlaceExecutor` gains `oil_well_pos` (+ `__oil__` in
    `_resolve_well_position`), and prep config: `do_prep`, `prep_bore`,
    `needle_volume_uL`, `oil_needles` (1), `buffer_needles` (4),
    `prep_rate_uL_s`, `service_z_mm`, and wash params (`wash_cycles`,
    `wash_z_amplitude_mm`, `wash_xy_amplitude_um`, `wash_dwell_s`).
  - New `run_prep()` — the 4-step sequence built from `_safe_move_to_well(key,
    target_z_mm=service_z)` (full safe-Z travel) + `move_pump_uL(bore, ±n·unit,
    rate_uL_s=...)` (sign: + dispense/expel, − aspirate/draw, matching the
    spheroid handler). Abort-aware; raises if a required service well is
    unresolved (GUI gates first; this is a backstop).
  - New `_do_wash()` — dip already done; per cycle: `move_z_user_relative(+amp)`
    then `(−amp)` (height-frame, lifts before descending), then a random XY
    nudge about the well centre; recentres at the end. Intra-well agitation (no
    retract between jiggles — the within-well exemption).
  - `_safe_move_to_well(well_name, target_z_mm=None)` now forwards the dip Z and
    returns True/False (resolved or not).
  - `execute_queue` runs `run_prep()` once (inside the try, before the loop) when
    `do_prep` is set; the existing `finally: _retract_to_safe_z()` still
    guarantees end-at-safe-Z.
- **`gui/pages/workflows/spheroid_pickup_workflow.py`**
  - New prep config row: "Prep needle (waste → oil → wash → buffer)" checkbox
    (default on), Service Z (↑ bottom) spin, Wash cycles spin, and a live status
    label (`✓ 1 needle = X µL · waste=… oil=… wash=… buffer=…`, or a ⚠ telling
    the operator what to assign/calibrate).
  - `_needle_volume_uL()`, `_service_well_names()` (ink_locations × ink_type →
    role→well), `_resolve_service_positions()` (role→absolute µm via the
    calibrated `_well_positions`), `_refresh_prep_status()` (called from
    `set_hardware_config` / `set_calibration_data`).
  - `_on_start` resolves the prep inputs, **gates** on the needle geometry, all 4
    service wells (assigned + calibrated), and plate-bottom Z (clear message
    otherwise), then sets the executor's prep fields + `*_well_pos`.
  - Fixed a latent bug: the needle outline used `outer_diameter_mm` (never a real
    `NeedleSpec` attr) → now `od_mm`, so the workspace/XZ needle actually draws.

## Files Modified

| File | Change |
|------|--------|
| `SupportClasses/PhysicalModels.py` | `NeedleSpec.internal_volume_uL` |
| `SupportClasses/PickAndPlaceManager.py` | oil slot + `__oil__`; prep config; `run_prep`; `_do_wash`; `_safe_move_to_well(target_z_mm)`; `execute_queue` prep call |
| `gui/pages/workflows/spheroid_pickup_workflow.py` | prep row + resolution helpers + gate + executor wiring; needle-viz `od_mm` fix |
| `tests/test_v75x_spheroid_pick_place_z.py` | +`TestNeedleVolume`, +`TestPrepRoutine` (now 16 in file) |

## Testing Notes

- `tests/test_v75x_spheroid_pick_place_z.py`: needle inner-bore volume; prep runs
  before the loop in waste→oil→wash→buffer order at the service dip Z; pump
  volumes (+1, −1, −4 needles); wash jiggles (lift-first) + recentre; prep
  disabled = loop only; missing service well aborts + still retracts; prep
  refuses when ZP disconnected.
- Offscreen page smoke: needle volume + `_service_well_names` (ink_type → role) +
  `_resolve_service_positions` (missing when uncalibrated, resolved after) +
  status label.
- Spheroid picker-scaling, jog-navigation, z-retract suites green.
- **Needs real-HW verification on ME3B V1**: assign oil/wash/waste/buffer inks to
  wells in Hardware Setup → Ink → Reagent Locations, calibrate the plate, then run
  — confirm the prep sequence and volumes, the wash agitation stays in the well,
  and the loop picks/places as before.

## Issues & Decisions

- **Oil has no `WellRole`** (`well_role_for_ink_type` collapses it to INK), so the
  role match is on the ink's `ink_type` **string** (`oil/wash/waste/buffer`),
  with a fallback to an ink literally named one of those.
- **Frames**: `_well_positions` and the service `*_well_pos` are ABSOLUTE stage
  µm — the frame `safe_travel_to` expects; the wash recentre wait converts to
  zero-ref mm using `controller.zero_position`.
- **Wash XY amplitude** is a fixed default (200 µm); not yet derived from the
  wash-well radius. Safe for typical reservoir wells; revisit if a small wash
  well is used.
- Prep is **opt-in** (default on) so the workflow degrades to the plain
  pick/place loop when service wells aren't set up.

## Verification (adversarial, 2 lenses — both claim_holds: true)

A safety lens and a correctness lens reviewed the prep/wash code; no
reachable defect was found. Two LOW latent items were hardened:
- **Wash Z jiggle drift (latent):** the net-zero jiggle relied on the lift never
  being soft-limit-clamped. The descend now returns to the **exact** dip Z via
  an absolute move (`move_z_absolute(service_z, from_zero_ref=True)`) instead of
  a symmetric `-amp`, so a clamped lift can never walk the tip toward the plate
  over cycles. Falls back to the symmetric relative descend if the dip Z is
  unknown / no absolute Z move.
- **Status/gate mismatch (UX):** `_refresh_prep_status` now also checks that the
  service dip Z resolves (plate bottom calibrated), so the status never reads ✓
  while Start would still block.

Confirmed safe by the review: every prep service move retracts+confirms before
XY (inter-well `safe_travel_to`, two-layer Z confirm, abort-on-fail); ZP-
disconnect refuses (executor guard + `safe_travel_to` guard + GUI gate); wash
lifts before descending (polarity-correct), XY jiggle bounded + envelope-clamped
+ recentred; abort checks throughout; always ends at safe Z (finally); correct
µm/mm units; correct pump signs (+expel / −draw) and volumes (1/1/4 needles);
needle volume uses the inner bore.

Left as-is (implausible / cosmetic): a 30 s recentre-wait timeout only if a wash
well centre sits within the jiggle radius of the hard XY limit; duplicate inks of
the same `ink_type` → first-assigned well wins (deterministic).

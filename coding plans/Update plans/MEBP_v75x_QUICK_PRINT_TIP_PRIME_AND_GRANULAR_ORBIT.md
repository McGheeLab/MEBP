# MEBP v7.5.x — Quick Print: bioink tip-prime pickup + granular anti-clog orbit

## Objective

Three operator-requested additions to the **Quick Print** bioink **pickup** step
(the only workflow that does a fresh per-print ink pickup, via
`PickPlaceExecutor.aspirate_ink`):

1. **Tip prime.** Aspirate an **extra X µL** beyond the print volume, then
   **dispense the same X back into the ink well**. This advances ink to the very
   tip and purges the air gap, so the ink is ready to deposit the moment
   printing starts. Net retained volume is unchanged.
2. **Skip compliance compensation for the pickup step only** while prime mode is
   active — the drivetrain/syringe compliance is already primed by the
   dispense-back, so bracketing the pickup aspirate with the backlash take-up /
   unload (`move_pump_uL` comp) would fight it. Prep / service / print pump moves
   keep their normal compensation.
3. **Granular anti-clog orbit.** When picking up an ink whose
   `InkSpec.ink_subtype == "granular material"`, orbit the needle in a small
   **circle (default 1 mm dia, configurable dia + speed)** *while* aspirating so
   the granules behave more fluid-like and don't clog the bore. Auto-applies to
   granular inks; a **manual override** forces it for any ink.

**Decisions (operator, AskUserQuestion):** Quick Print only · extra amount = a
**global µL** setting · circle diameter + speed **configurable**.

## Design

All three funnel through the single ink-pickup primitive
`PickPlaceExecutor.aspirate_ink` — both Quick Print pickup paths (multi-ink
sequential-swap + single-print) already call it. No new motion/coordinate math;
reuses existing primitives (`_settled_pump_move`, `move_xy_absolute_um`,
`safe_travel_to`).

- **Prime + compliance:** `aspirate_ink` gains `prime_uL` / `prime_rate_uL_s`.
  When `prime_uL > 0` the whole pickup step uses `compensate=False`; it aspirates
  `-(vol + prime_uL)` then dispenses `+prime_uL` back into the well. Prime off ⇒
  a single `-vol` aspirate at `compensate=None` (byte-identical legacy). Mirrors
  the spheroid "disengage lead + release dispense-back" pattern.
- **Orbit:** `aspirate_ink` gains `orbit` / `orbit_diameter_mm` /
  `orbit_speed_mm_s`. New `_orbit_xy` runs a daemon thread that steps `N=24`
  points/rev around the ink-well centre via `move_xy_absolute_um` (the
  `_ploc_multi_edge_fit` template), paced by tangential speed, cycling until a
  stop event OR `_abort_flag`. The pickup pump moves run on the main thread (XY /
  Prior and pump / ZP-Marlin are independent buses); a `finally` stops the
  orbit, re-centres, and `wait_for_xy_arrival`. **Safety:** in-well motion with
  the needle already at dip Z — exempt from retract-before-XY (CLAUDE.md rule 1,
  same class as the wash jiggle); end-at-safe-Z still guaranteed by the caller's
  `_retract_to_safe_z()`.
- **GUI:** the ⚙ Settings **"Ink pickup"** section gains `_ink_prime_check`
  (off) + `_ink_prime_spin` (µL), `_orbit_check` (on) + `_orbit_all_check` (off)
  + `_orbit_dia_spin` (mm) + `_orbit_speed_spin` (mm/s), all registered via
  `sec.add(...)`/`sec.add_check(...)` so they round-trip in `WorkflowSettingsStore`
  profiles. New `_ink_pickup_kwargs(ink_name)` resolves prime + granular-orbit
  kwargs on the GUI thread (granular from the ink's subtype); both call sites
  pass them (multi-ink via the group dict `pickup_kwargs`, single via a captured
  local). Both confirm dialogs note when prime / orbit are active.

## Files Modified

| File | Change |
|---|---|
| `SupportClasses/PickAndPlaceManager.py` | `aspirate_ink` + `prime_uL`/`prime_rate_uL_s`/`orbit`/`orbit_diameter_mm`/`orbit_speed_mm_s`; prime aspirate-then-dispense-back; `compensate=False` when priming; new `_orbit_xy` / `_stop_orbit` helpers + concurrent orbit wrap |
| `gui/pages/workflows/quick_print_workflow.py` | 6 new pickup widgets + registration; new `_ink_pickup_kwargs`; granular resolution + kwargs at both `aspirate_ink` call sites; confirm-dialog notes |
| `tests/test_v75x_quick_print_tip_prime_and_granular_orbit.py` | new suite (13) |
| `tests/test_v75x_quick_print_multi_ink.py`, `tests/test_v75x_quick_print_pick_and_place.py` | fake `aspirate_ink` signatures updated to the grown real API |

## Implementation Steps

- [x] `aspirate_ink`: prime (extra aspirate + dispense-back) + `compensate=False` while priming
- [x] `_orbit_xy` / `_stop_orbit` concurrent granular orbit + wrap
- [x] Quick Print settings widgets + persistence keys
- [x] `_ink_pickup_kwargs` + wire both call sites (GUI-thread resolution)
- [x] Confirm-dialog notes (multi-ink + single)
- [x] Tests + fake-signature updates; run affected suites

## Testing Notes

- New suite `tests/test_v75x_quick_print_tip_prime_and_granular_orbit.py` (13),
  real executor vs recording fake controller: prime off ⇒ single `-vol`
  `compensate=None`; prime on ⇒ `-(vol+X)` then `+X`, both `compensate=False`,
  net `-vol`; prime rate honored; travel-only (vol≤0) no pump; prime-only pure
  cycle; orbit off ⇒ no XY moves; orbit on ⇒ blocking-aspirate has a background
  circle of the set diameter around the well, re-centres + waits after; orbit
  thread honors `_abort_flag`; zero-diameter no-op; `_ink_pickup_kwargs` granular
  auto / force-all / disabled-beats-granular / prime-volume.
- Green suites: quick-print pick&place / multi-ink / travel-split, spheroid
  pick-place-z / disengage, pump compliance+backlash, pump settle+prime,
  workflow-settings-popout, quick-print-workflow (239 across the runs).
- **Needs real-HW verification on ME3B V1:** pick a bioink with prime on → ink
  advances to the tip with no air gap, X returns to the well, and the pickup
  aspirate runs with no take-up/unload bracketing; a granular-subtype ink orbits
  a ~1 mm circle while aspirating with no clogging; run ends at safe Z.

## Issues & Decisions

- (decision) Prime is a **global µL** setting; compliance-skip is coupled to
  prime being on (the operator's stated rationale — the dispense-back primes the
  compliance), owned inside `aspirate_ink` (callers don't touch compliance).
- (decision) Orbit auto-detects `ink_subtype == "granular material"`; a
  force-all override covers non-granular inks that still clog.
- (decision) Orbit uses discrete point-stepping on a daemon thread (proven
  `_ploc_multi_edge_fit` template, abort-friendly) rather than a continuous
  velocity vector (no ramped-vector helper exists; XY/pump buses are independent
  so concurrent motion is safe).
- (safety) Orbit is in-well (needle already lowered) → retract-before-XY exempt;
  1 mm circle ≪ well diameter; caller's `finally` still ends at safe Z.
- (note) Widget-backed kwargs are resolved on the GUI thread and passed into the
  off-thread worker (no cross-thread Qt reads).

## Status

`[~]` Code + tests complete. **Needs real-HW verification on ME3B V1.** On
operator confirmation, run the CLAUDE.md version-completion checklist
(architecture doc, finalize this plan, README, push).

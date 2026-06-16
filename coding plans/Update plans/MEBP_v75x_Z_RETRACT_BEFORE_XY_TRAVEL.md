# MEBP v7.5.x — Guarantee Z Retract Before Cross-Position XY Travel

## Objective

**CRITICAL SAFETY FIX.** Any time the stage performs an XY move to a *different*
location, the Z needle must first retract to the safe / "move" / travel Z height
(and the code must WAIT for Z to arrive before XY starts) — **except** when moving
*within the same well* during a print/workflow (the print pattern itself), where Z
intentionally stays at print height.

Two reported failures motivated this:

1. **Jog page well click drives straight to the well without retracting Z.**
   Root cause: the click-to-travel gate `if current_z >= self._safe_z - 0.05:`
   *skips* the retract when the needle is "at/above" safe Z — but this comparison
   is **polarity-wrong** on ME3B V1 (`StageController.ZDIR = -1`, needle descends
   as raw Z increases). At a print/low position `current_z` is numerically *higher*
   than the retracted `safe_z`, so the gate skips the retract exactly when the
   needle is DOWN. Duplicated in three places (jog_control, standard_jog_context,
   spheroid_pickup_workflow).

2. **Quick Print returns to (0,0) at the end without retracting Z.**
   Discrete plan relies on a *separate* `TRAVEL_UP` command preceding `HOME_XY`;
   the `MOVE_XY`/`HOME_XY` handlers do a bare `move_xy_absolute` with no self-retract
   and no wait for the Z move to finish. The Quick-Print `travel_z_height` fallback
   of `5.0` when Safe Z is unset is also not a genuine retract on `ZDIR=-1`.

## Design

**Single principle: a "retract for travel" must never lower the needle.** It only
ever raises the needle (in the *height* frame) to at least the travel/safe height;
if the needle is already at/above that height, it is a no-op. This is polarity-safe
by construction — a bad/low travel target can never drive a crash.

1. **StageController — polarity-aware helpers + a guaranteed travel-retract:**
   - `z_height_of(raw_zref)` → `ZDIR * raw_zref` (height frame).
   - `needle_at_or_above(current_raw_zref, ref_raw_zref, tol)` → polarity-aware test.
   - `ensure_retracted_to(safe_z_raw_zref, ...)` → if the needle is below `safe_z`
     in the height frame, raise to `safe_z` (retract feedrate) + `flush_moves` +
     `wait_for_z_arrival`; **never descends**. Honors the `_min_travel_z_mm` floor.
     Returns True when the needle is confirmed at/above the height.

2. **Gold standard already exists** — `safe_travel_to` (raise→wait→XY→wait→lower)
   and `PickAndPlaceManager._safe_move_to`. Route the broken call sites through them.

3. **GUI click-to-travel gates (3 sites)** — remove the polarity-wrong "skip retract"
   shortcut. When a Safe Z is known, **always** `safe_travel_to(...)`. Keep the
   "No Safe Z configured" prompt for the unconfigured case.

4. **PrintManager discrete travel commands** — `MOVE_XY` and `HOME_XY` call
   `ensure_retracted_to(travel_z_height)` (never-descend, with wait) **before** the
   XY move (defense-in-depth; independent of the plan's separate `TRAVEL_UP`).
   `PRINT_PATH` segments are within-well → unchanged (exempt).

5. **Quick Print fallback** — when Safe Z is unset, derive a real retract target from
   available Z references instead of the bare `5.0`.

## Files Modified

| File | Change |
|------|--------|
| `SupportClasses/StageController.py` | New `z_height_of` / `needle_at_or_above` / `ensure_retracted_to` helpers (polarity-aware, never-descend). |
| `SupportClasses/PrintManager.py` | `MOVE_XY` + `HOME_XY` handlers ensure travel-Z retract (+ wait) before the XY move. |
| `gui/pages/jog_control.py` | `_on_workspace_position_clicked` — always `safe_travel_to` when Safe Z known. |
| `gui/widgets/standard_jog_context.py` | `_on_workspace_position_clicked` — same fix. |
| `gui/pages/workflows/spheroid_pickup_workflow.py` | `_on_workspace_position_clicked` — same fix. |
| `gui/pages/workflows/quick_print_workflow.py` | Safer `travel_z_height` fallback when Safe Z unset. |
| `CLAUDE.md` | Add the critical "always retract Z before cross-position XY travel" note. |
| `coding plans/Architectures/` | Note in the architecture doc. |
| memory | `z-retract-before-cross-position-xy-travel.md`. |
| `tests/test_v75x_z_retract_before_xy_travel.py` | New regression tests. |

## Implementation Steps

- [x] Audit all 68 XY-move call sites (parallel workflow) + adversarially verify
- [x] Add StageController helpers (`z_height_of`, `needle_at_or_above`,
      `default_travel_z`, `ensure_retracted_to`); fix `safe_travel_to` floor to
      the height frame
- [x] Fix the 2 polarity-wrong GUI click gates (`jog_control`,
      `spheroid_pickup_workflow`) → always `safe_travel_to` when Safe Z known
- [x] PrintManager `MOVE_XY`/`HOME_XY` self-retract before XY (`_retract_for_travel`)
- [x] HybridPlanExecutor `TRAVEL_XY` → blocking `move_z(travel_z)` before XY
- [x] Quick Print safer travel-Z fallback (`default_travel_z`, not raw `5.0`)
- [x] Calibration `_jog_xy_home` / `_goto_a1` / `_goto_corner` / `_goto_well`
      fallback → `_safe_navigate_to`; one-time pre-scan retract in `_ploc_run_queue`
- [x] `PickAndPlaceManager._intra_well_move` XY µm/mm scaling bugfix
- [x] `PrintTrajectoryPlanner` ZDIR=+1 limitation documented + runtime warning
- [x] Critical note in CLAUDE.md + memory
- [x] Regression tests (`tests/test_v75x_z_retract_before_xy_travel.py`, 22) — green
- [ ] (Follow-up) Full ZDIR-polarity pass of `PrintTrajectoryPlanner` Z geometry

## Status Tracking

**Complete** (except the documented `PrintTrajectoryPlanner` follow-up). All edited
files byte-compile; 22 new tests pass; full suite shows only 13 pre-existing
failures in unrelated/stale test modules (`test_v73_trajectory_planner`,
`test_v726_print_execution`, `test_sim_vs_hardware`) — none in files this change
touched.

### Audit summary (68 call sites)

| Class | Count | Disposition |
|-------|-------|-------------|
| cross_position | 29 | The unsafe ones fixed (see steps); the rest already used `safe_travel_to` / blocking `move_z`. |
| within_well | 8 | Exempt (print pattern). Unchanged. |
| manual_jog | 10 | Exempt (user-controlled). Unchanged. |
| bypass_intended | 5 | Operator-confirmed / device-setup. Unchanged. |
| calibration_scan | 10 | Covered by the one-time `_ploc_run_queue` retract + `_safe_navigate_to`; sub-FOV nudges exempt. |
| unknown | 6 | Generic primitives (`move_xy_absolute*`, `move_z_*`) — caller's responsibility. |

## Testing Notes

- Unit: `ensure_retracted_to` never descends (both `ZDIR=±1`); raises when below.
- Unit: GUI gate always routes to `safe_travel_to` when Safe Z known (mock controller).
- Unit: PrintManager `MOVE_XY`/`HOME_XY` call the retract before `move_xy_absolute`.
- Regression: existing `safe_travel_to` / jog-navigation / quick-print tests stay green.

## Issues & Decisions

- **Manual incremental jog (arrow keys, jog buttons, Xbox, nudge) is EXEMPT** — it is
  user-controlled motion with the needle wherever the user placed it; auto-retracting
  every nudge would make jogging unusable. The rule targets *point-to-point travel to
  a new target* (well click, go-to, return-home, move-to-print-start).
- **Within-well print-pattern moves are EXEMPT** (`PRINT_PATH` segments) — Z is
  intentionally at print height there.
- **Never-descend** chosen over absolute-move so a misconfigured/low travel target can
  never cause a crash-down; worst case it is a no-op.

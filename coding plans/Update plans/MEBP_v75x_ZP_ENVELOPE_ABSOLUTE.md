# MEBP v7.5.x — ZP Safety Envelope → Absolute Marlin Frame

## Objective

Fix the reported bug: **after calibration, ZP (Z + pump) jog moves on the Jog
page and via the Xbox controller get wrongly clamped**, even when the move is
physically valid. (Hardware Setup jog escaped this only because it passes
`bypass_safety=True`.) XY is unaffected.

## Diagnosis

This is the ZP twin of the XY needle-zero clamp bug fixed in
`MEBP_v75x_XY_ENVELOPE_ABSOLUTE.md`. The Z/pump safety envelope was stored
**zero-referenced** (`z_min/z_max`, `p*_min/p*_max` in mm relative to
`zero_position`). Every ZP clamp path compared the **zero-referenced**
destination `(raw + delta − zero_position)` against those limits:

- `StageController.move_z_relative`, `move_pump_relative`, `move_z_absolute`
- `ZPJogHandler._clamp_delta` (the Xbox velocity jog)

When calibration re-anchors the needle/pump zero — `reset_z_zero` / Set Z Zero
records the *current* raw as the new `zero_position["Z"]` — the stored limits
(recorded relative to a *different* zero) no longer match the working range.
After a re-zero the needle sits near zero-ref `0`, but a stored envelope like
ME3B V1's `[10, 60]` excludes it, so every jog clamps (it even forces the needle
toward `10`). The clamp code was *correct* (zero-ref vs zero-ref); the
envelope/zero relationship went stale.

## Decision (user)

Make the Z/pump envelope **absolute Marlin raw mm** — fixed mechanical extents
that do not move when the operator does Set Z Zero / needle-zero. Clamps compare
the **absolute** destination against the absolute bounds. This mirrors the XY
fix and matches the operator's model: the Z axis can travel its full mechanical
range regardless of where the needle zero is calibrated.

### Asymmetry vs XY (and the one caveat)

The Prior (XY) keeps its absolute frame while powered; Marlin (ZP) loses its
frame on power cycle. So an absolute ZP envelope is only aligned with reality
once the raw frame is established — i.e. it **relies on the ZP last-known
position restore** (`MEBP_v75x_ZP_POSITION_OVERRIDE.md` addendum) or a fresh
calibration on power-up. Documented; the manual Override / Sync Axis Position
card remains the fallback.

## Frame model (after this change)

- `get_zp_position()` → physical Marlin raw mm (unchanged).
- `zero_position["Z"|"P*"]` → raw mm designated as the display/print zero.
- `SafetyLimits.z_min/z_max/p*_min/p*_max` → **absolute Marlin raw mm** (was
  zero-ref).
- Clamp paths compare the **absolute** destination against the absolute bounds.
- User-facing readouts stay **zero-ref** (`raw − zero`).
- `XZSideView` draws the Z scale from the envelope but is fed a zero-ref Z
  position, so it subtracts the current Z zero (`set_zero_offset_z`) at draw
  time — pixel-identical in the common case (zero = 0), correct after a re-zero.

### Migration

Existing persisted Z/pump limits are **reinterpreted as absolute** (no numeric
transform). Re-record each limit with one click (Hardware Setup → Stage →
Set Min/Max, or Settings → Set from Current — both now store the raw reading).

## Files Modified

| File | Change |
|------|--------|
| `SupportClasses/SafetyLimits.py` | Docstrings: Z/pump limits are absolute Marlin raw mm. `clamp_z`/`clamp_pump` bodies unchanged. |
| `SupportClasses/StageController.py` | Clamp the **absolute** destination in `move_z_relative`, `move_z_absolute` (now also clamps the `from_zero_ref=False` raw path — a safety improvement), `move_pump_relative`, and `ZPJogHandler._clamp_delta`. |
| `gui/pages/hardware/stage_panel.py` | `_record_limit` stores absolute raw for Z/pumps (drop `− zero`); removed now-dead `zero` local. |
| `gui/pages/settings_page.py` | `_set_z_from_current` / `_set_pump_from_current` store absolute raw. |
| `gui/widgets/xz_side_view.py` | New `_zero_off_z` + `set_zero_offset_z`; new `_z_safety_bounds()` chokepoint (absolute → zero-ref) used by `_z_bounds_mm` and both scrollbar methods. |
| `gui/pages/jog_control.py` | Push `set_zero_offset_z(zero["Z"])` in `on_status_update`. |
| `gui/pages/workflows/spheroid_pickup_workflow.py` | Push `set_zero_offset_z` next to `set_zero_offset_x`. |
| `tests/test_v75x_zp_envelope_absolute.py` | **New** (7). |

## Implementation Steps

- [x] SafetyLimits docstrings (Z/pump absolute).
- [x] `move_z_relative` / `move_pump_relative`: clamp `cur + delta` (absolute); `delta = clamped − cur`.
- [x] `move_z_absolute`: clamp absolute `position` in both `from_zero_ref` paths.
- [x] `ZPJogHandler._clamp_delta`: `clamp_fn(cur + delta) − cur` (drop zero ref).
- [x] `_record_limit` (Z/pumps) + `settings_page` setters store absolute raw.
- [x] `XZSideView` `set_zero_offset_z` + `_z_safety_bounds()` chokepoint.
- [x] Push Z zero offset from `jog_control` + spheroid workflow.
- [x] Tests (7) + full sim suite (no new failures; the 10 MagicMock planner errors + 3 sim physics/state flakes are the pre-existing baseline, present before this change).

## Testing Notes

`python -m unittest tests.test_v75x_zp_envelope_absolute`

Asserts: after a mid-travel Set Z Zero, a small Z/pump jog is **not** clamped
(absolute destination within bounds); a jog past the absolute envelope **is**
clamped; the clamp is independent of `zero_position`; the ME3B V1 axis_map reads
the real Z slot.

Manual (HW): calibrate (Set Z Zero), then jog Z and pumps on the Jog page and
via Xbox — full mechanical range is reachable without spurious clamping; the
XZSideView envelope still frames the needle correctly.

## Issues & Decisions

- **Crash protection vs absolute frame:** the absolute Z lower limit only
  protects against a plate crash if the raw frame is correct. The ZP
  position-restore (prompted on connect) re-establishes it; a fresh
  calibration also does. If neither has happened, re-record / re-zero first.
  The needle-zero + `set_min_travel_z` (print-path retract floor) remain the
  print-time crash guards.
- **`move_z_absolute` now clamps the raw path too** — previously
  `from_zero_ref=False` skipped the soft limit. Tightening, consistent with the
  XY change.

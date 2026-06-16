# MEBP v7.5.x — Axis-Map Jog & Jog-Speed Fixes

## Objective

Fix two reported bugs, both surfacing on the **ME3B V1** machine (which uses a
**non-default `axis_map`**: `Z→Z`, `P1→X`, `P2→Y`, `P3→E`):

1. **Jog Z direction / safety bypass.** On the Jog page (and Calibration /
   Settings — anywhere safety limits are enforced) the Z jog "seemed to always
   go the same direction (down)" and **bypassed the Z soft-limit**. It worked on
   the Hardware Setup page only because that panel jogs with `bypass_safety=True`.
2. **Jog speed defaults.** The Hardware Setup / Jog Stages speed spinboxes used
   hardcoded defaults (XY 2000 µm/s, Z 600 mm/min, P 200 mm/min) that ignored the
   calibrated maxima — the Z default of 600 even **exceeded** the ME3B V1
   `max_z_feedrate` of 500. They should default to **½ of the calibrated max**.

## Root Cause (Bug 1)

`StageController.get_zp_position()` returns the ZP tuple in **physical Marlin
order `(X, Y, Z, E)`**. Logical axes (`Z`, `P1`, `P2`, `P3`) map to physical
slots via the live `axis_map`. Several call sites hardcoded `pos[0]` (or a
positional unpack) for logical **Z**, assuming the legacy default map
(`Z→X = index 0`). Under ME3B V1, `index 0` is the **P1 pump**, so:

- `move_z_relative`'s clamp read P1's position, clamped it against the Z
  envelope `[0,60]`, then applied the bogus delta to the real Z motor → no real
  Z-limit protection, and a constant-direction move whenever P1 sat outside
  `[0,60]`.
- The Z **zero references** (`_calibrate_zero`, `reset_z_zero`) were recorded
  from the wrong slot, so all zero-ref Z math was offset.

The v7.4.2 helpers `_axis_index()` / `_axis_letter()` already resolve
logical→physical correctly (and `reset_pump_zero` / `zero_axis` /
`zp_logical_value` already use them). The fix routes the missed Z/jog sites
through the same helpers. Under the **default** map the change is a strict no-op.

## Files Modified

| File | Change |
|------|--------|
| `SupportClasses/StageController.py` | `move_z_relative` clamp, `_calibrate_zero`, `reset_z_zero`, `wait_for_z_arrival`, and `ZPJogHandler._jog_loop` (Xbox jog) now resolve logical axes via `_axis_index` / `_axis_letter` instead of hardcoded indices/letters. New `get_zp_position_logical_tuple()` helper; the three `position_logger.record(zp_pos=…)` sites now log logical order. |
| `SupportClasses/PrintManager.py` | The four `_execute_loop` `pos_logger.record(zp_pos=…)` calls (print_start/progress/end/error) now pass `get_zp_position_logical_tuple()` so the position log is correctly labeled. |
| `gui/pages/settings_page.py` | `_set_z_from_current` reads Z via `zp_logical_value` (keeps the "set Z limit from current" button consistent with the corrected Z zero). |
| `gui/pages/hardware/control_panel.py` | `HardwareControlPanel` seeds jog speeds to ½ of the calibrated max and caps each spinbox at that max (`_apply_speed_defaults_from_settings`, called from `set_settings`). Seeds once; preserves a manual edit. |
| `tests/test_v75x_axis_map_jog.py` | New deterministic regression tests for the axis-map jog/clamp/zero behaviour + logical-ordered position logging. |

### Bug 2 — speed sources (keys read in `control_panel.py`)

- XY (µm/s): `safety_limits.max_xy_speed` (ME3B V1 = 10000) → default 5000.
- Z (mm/min): `device_profile.per_axis_max_feedrate["Z"]` (ME3B V1 = 500),
  fallback `safety_limits.max_z_feedrate` → default 250.
- Pump (mm/min): `safety_limits.max_pump_feedrate` (ME3B V1 = 200) → default 100.

## Implementation Steps

- [x] `move_z_relative`: read `pos[_axis_index("Z")]`; both `new_z` and the
      back-calc use the same indexed value; guard `idx None / short tuple / None slot`.
- [x] `_calibrate_zero`: set each `zero_position[logical]` via `_axis_index`.
- [x] `reset_z_zero`: resolve Z slot via `_axis_index`; drop the `pos[0]` sentinel.
- [x] `wait_for_z_arrival`: poll Z via `_axis_index` slot.
- [x] `ZPJogHandler._jog_loop`: clamp each logical axis via `_axis_index`; build
      the `move_relative` dict via `_axis_letter` (not hardcoded X/Y/Z/E).
- [x] `settings_page._set_z_from_current`: use `zp_logical_value(pos, "Z")`.
- [x] `control_panel`: half-of-max seeding + spinbox cap, guarded against
      clobbering manual edits (`editingFinished` tracking + seed-once flag).
- [x] Logical-ordered position logging: add `get_zp_position_logical_tuple()`;
      route all 7 `zp_pos=` log sites (PrintManager ×4 + StageController ×3) through it.
- [x] Regression tests + run; byte-compile changed files.

## Testing Notes

- `tests/test_v75x_axis_map_jog.py` — 8 tests, all pass:
  - ME3B V1 map: Z up sends +, Z down sends − (no longer constant-direction);
  - soft-limit clamps the **real** Z (index 2), not the P1 pump (index 0);
  - below-min does not invert direction;
  - default map is identity (no behaviour change);
  - `_calibrate_zero` records Z's zero from physical index 2 (P1 from index 0);
  - `get_zp_position_logical_tuple` returns `(Z,P1,P2,P3)` logical order (ME3B V1
    + default map), and `reset_z_zero` logs the real Z as `z` (not the P1 pump).
- Changed files byte-compile.
- Pre-existing failures in `tests/test_v731_*` (11 errors) were verified to fail
  **identically with my edits stashed** — they reference the pre-v7.4.3
  `JogControlPage` API / MagicMock fixtures and lack `_pos_poller`; not caused by
  this change.
- Manual GUI verification recommended on the ME3B V1 hardware: Jog page Z ▲/▼
  move in the correct direction and stop at the Z limits; speed spinboxes show
  5000 / 250 / 100 and cap at 10000 / 500 / 200.

## Issues & Decisions

- **Fix the trio together.** `move_z_relative` + the two Z-zero setters must be
  corrected in lockstep; fixing only the read while leaving the zero in the P1
  frame would mix frames. `move_z_absolute` consumes the same `zero_position["Z"]`
  and becomes correct automatically once the zero is in the Z frame.
- **Scope.** Extended beyond the two reported bugs to the whole same-root-cause
  family on the live machine: the Xbox jog loop (`ZPJogHandler`) drove the wrong
  motor under ME3B V1, and `wait_for_z_arrival` gates needle descent in
  `safe_travel_to` — both safety-relevant.
- **Resolved (was deferred) — diagnostic position-log mislabel.**
  `PositionLogger.record`'s `zp_pos` contract is logical `(z, p1, p2, p3)`, but
  callers passed the physical `get_zp_position()` tuple `(X, Y, Z, E)`, so under
  ME3B V1 the position **log** recorded P1's value as "z" and the real Z as "p2"
  (diagnostic only — no motion effect). Fixed by adding
  `StageController.get_zp_position_logical_tuple(cached)` (resolves each slot via
  the live axis_map) and routing all 7 `zp_pos=` log sites through it:
  `PrintManager._execute_loop` (print_start/progress/end/error) +
  `_calibrate_zero` / `reset_pump_zero` / `reset_z_zero`. `PositionLogger` keeps
  its logical-order contract unchanged (it has no axis_map). New tests in
  `tests/test_v75x_axis_map_jog.py::TestLogicalPositionLogging`.

## Stale-test refresh (`test_v731_*`)

While verifying, the `tests/test_v731_jog_navigation.py` + `tests/test_v731_integration.py`
suites had **11 pre-existing errors** (failed identically before this change).
All were stale tests, not production bugs — refreshed to the current API:

- **`TestSafeTravelTo` (5)** — fixture built the controller via `__new__`, missing
  `_pos_poller` / `_zp_retract_feedrate` / `_zp_insert_feedrate` that the v7.4.x
  `safe_travel_to` reads; assertions also predated the per-step `feedrate_mm_min`
  kwarg. Added the attributes and updated the call-signature assertions.
- **`_set_safe_z` signal tests (2)** — `_set_safe_z` now reads Z via the v7.4.2
  logical accessor `controller.zp_logical_value(zp, "Z")`; the mocked controllers
  didn't stub it, so `_safe_z` became a `MagicMock` and `:.2f` raised. Stubbed
  `zp_logical_value`.
- **Jog-page navigator / fast-travel (4)** — the v7.3.1 `WellPlateNavigator` +
  `_on_well_nav_click(well_name)` were replaced by the v7.4.3 coordinate-based
  `JogWorkspaceView` + `_on_workspace_fast_travel_requested(x_um_zr, y_um_zr)`.
  Rewrote the navigator-state test against `_workspace_view`, rewrote fast-travel
  + no-safe-z tests against the new handler (patching `gui.pages.jog_control.QMessageBox`),
  and repurposed the removed "uncalibrated well" gate test into a
  "fast travel requires XY connected" gate (`test_jog_page_fast_travel_requires_xy_connected`).

Result: both suites green (77 tests), plus the 8 axis-map / logging tests — 85 total.

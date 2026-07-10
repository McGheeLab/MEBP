# MEBP v7.5.x — Gentle Z: slow lift-off + slow descent, standard for all workflows + user-controllable

## Objective
Operator: (1) "Does Quick Print obey the slow lift-off we included? it seemed to
not be doing this." (2) "On re-entry we also want the last 1 mm of travel into
the position to be slower." (3) "Make this slow lift and slow descent standard
for all workflows. The user should also have control over this value."

So: ease the needle IN and OUT of every print/work position — the first N mm of a
**lift** out of a print and the last N mm of a **descent** back into position both
run slowly (the rest is fast) — for **every workflow**, with a **user-editable**
distance + speed.

## Findings (from an empirical + adversarially-verified investigation)
- **The slow LIFT already works in Quick Print** (and everywhere). Every lift
  funnels through `StageController.ensure_retracted_to` / `safe_travel_to` →
  `_retract_z_slow_then_fast` (first `_retract_slow_dist_mm` at
  `_retract_slow_feedrate`, default 1 mm @ 60 mm/min). An empirical Z-move trace of
  a discrete Quick Print job confirmed the job-start `TRAVEL_UP`, the inter-line
  `MOVE_XY(hop_z)`, and the final `TRAVEL_UP` all lift slow-then-fast.
  - **Why it *looked* like it wasn't:** for a single continuous shape (the common
    case) there are **no mid-print lifts** — the only slow lift is the final
    retract *after* printing finishes, so nothing gentle is visible *during* the
    print. (Also: Quick Print drops a sketch's own `lift_slow_*` and used the
    controller defaults — now the operator can tune those defaults directly.)
- **The DESCENT was NOT gentle.** Both re-entry descents (`safe_travel_to` step 3
  and the discrete `MOVE_Z` handler) were a single `move_z_absolute` at the insert
  feedrate — no slow final segment. This is the gap the change fills.

## Files Modified
- `SupportClasses/StageController.py`
  - `__init__`: new `_descend_slow_dist_mm` (1.0) / `_descend_slow_feedrate` (60.0).
  - New **emit-only** `_descend_z_moves_only(cur_zref, target_zref, fast_fr)` — the
    polarity-safe descent twin of `_retract_z_slow_then_fast`: descend fast to
    `target_h + slow_dist` (HEIGHT frame), then the final `slow_dist` at the slow
    feedrate; a short descent runs entirely slow; an ascent / unknown-cur /
    slow-dist-0 degrades to a single move; **never** slows an ascent. Does NOT
    M400/wait — the callers keep their own confirm (no double-wait).
  - New `emit_descent_moves(target_zref, fast_fr)` — reads the CACHED Z (no serial
    round-trip → safe outside a poller-suspend window) and delegates.
  - New `set_descend_slow_final(dist, feedrate)` setter (mirror of
    `set_retract_slow_lift`).
  - `safe_travel_to` step 3: emit the descent via `_descend_z_moves_only` (cur read
    guarded on slow-dist>0, inside the existing poller-suspend window); the two
    existing confirm layers (`flush_moves` M400 + `wait_for_z_arrival` → `ok=False`)
    are kept verbatim.
  - `set_hardware_config`: push the config's gentle-Z (one distance + one
    mm/s speed) into **both** `set_retract_slow_lift` and `set_descend_slow_final`.
- `SupportClasses/PrintManager.py`
  - Discrete `MOVE_Z` handler: emit the descent via `ctrl.emit_descent_moves` on a
    real controller (gated by `isinstance(getattr(ctrl,"_descend_slow_dist_mm",None),
    (int,float))`), else the legacy single `move_z_absolute`. The handler's own
    poller-suspend + M400 + `wait_for_z_arrival` + **abort-before-extrusion** block
    is untouched.
- `SupportClasses/HardwareConfig.py`: `gentle_z_slow_dist_mm` (1.0) +
  `gentle_z_slow_speed_mm_s` (1.0) fields + `to_dict`/`from_dict`.
- `SupportClasses/CommonPrintSettings.py`: both keys added to `GLOBAL_KEYS` +
  `GLOBAL_DEFAULTS` (proxied to HardwareConfig — the single source of truth).
- `gui/pages/workflows/common_print_settings_workflow.py`: new "Gentle Z near the
  plate (global)" card with **Slow zone distance** (mm; 0 disables) + **Slow speed**
  (mm/s) rows.

## How it works / why it's safe
- **Standard for all workflows:** every cross-position travel already funnels
  through `safe_travel_to` (pick-and-place, cell targeting, cell labeling,
  calibration navigation, needle-location goto) or the discrete `MOVE_Z` (Quick
  Print / discrete prints). Wiring the descent into those two primitives + the
  already-present slow lift = gentle in/out for all of them. Intra-well jiggles
  (`_intra_well_move`, `_do_wash`) and operator-gated calibration/jog descents are
  intentionally left alone.
- **Emit-only + isinstance guard (from the adversarial review):** the descent
  helper emits moves but does not confirm, so `safe_travel_to`/`MOVE_Z` keep their
  single confirm layer (no double-wait), and the `MOVE_Z` abort-before-extrusion
  safety keeps firing. A MagicMock / older controller has no numeric
  `_descend_slow_dist_mm` → the `isinstance` gate falls to the legacy single
  `move_z_absolute`, so the existing print-setup-routine / feedrate-inheritance
  tests (which assert `move_z_absolute.assert_called_once()` + abort on
  `wait_for_z_arrival=False`) are unaffected.
- **Polarity-safe (ZDIR / z_up_sign):** all comparisons are in the HEIGHT frame
  via `z_height_of` / `z_up_sign` (`inter_zref = inter_h * z_up_sign()`), the exact
  mirror of the retract helper; verified for both `z_up_sign = +1` and `-1`.
- **User control:** one distance + one mm/s speed on the Common Print Settings page
  drive BOTH directions (their defaults are identical). A live edit → HardwareConfig
  attr → `app.py::_on_common_setting_changed` → `_propagate_hardware_config` →
  `controller.set_hardware_config` → re-pushed to the two setters. `dist = 0`
  disables the easing (single-speed / legacy).

## Implementation Steps
- [x] `_descend_z_moves_only` + `emit_descent_moves` + `set_descend_slow_final` + `__init__` fields
- [x] Wire `safe_travel_to` step 3 (emit-only, keep confirm layers)
- [x] Wire discrete `MOVE_Z` handler (isinstance guard, keep confirm/abort)
- [x] HardwareConfig fields + serialize
- [x] CommonPrintSettings globals
- [x] Common Print Settings page "Gentle Z" card
- [x] Tests `tests/test_v75x_gentle_descent_slow_final.py` (18)

## Testing
`tests/test_v75x_gentle_descent_slow_final.py` (18): primitive fast-then-slow /
ascent-single / short-descent-all-slow / both polarities / slow-fr fallback;
`emit_descent_moves` cached read; `safe_travel_to` 4-Z-move descent + disabled
single-move; setter; config round-trip + CommonPrintSettings proxy +
`set_hardware_config` push (incl. disable); `MOVE_Z` real→2-moves+confirm,
mock→single move. Regression batch green (279): gentle-retract, zp-feedrate-
inheritance, print-setup-routine, print-always-safe-z, z-retract-before-xy,
quick-print-travel-split, quick-print-pick-and-place, common-print-settings,
pump-settle-and-prime, simple-print-manager, multi-object-seam, spheroid-pick-
place-z, cell-targeting, cell-labeling. Common Print Settings page builds offscreen
and the gentle-Z rows edit→model→HardwareConfig round-trip.

## Issues & Decisions
- **Shared distance + speed for both directions** (not 4 separate knobs): the user
  asked for control over "this value"; lift and descent defaults are identical, so
  one distance + one speed is the simplest coherent model and reproduces the
  current behavior exactly. Split later if a machine needs asymmetric lift/descent.
- **Emit-only helper** rather than a confirming primitive at the `MOVE_Z` call
  (the adversarial reviewer showed a confirming controller method would break 5
  existing tests and, worse, silently stop the abort-before-extrusion from firing
  because the confirm bool would come from a method the tests can't influence).
- **`safe_travel_to` step 3** reads the current Z with `cached=False` (accurate;
  the poller is already suspended for the sequence); the `MOVE_Z` path reads
  `cached=True` (no serial, no race — the poller is not yet suspended there). A
  stale/unknown cache degrades safely to a single move (no gentle final mm for that
  one hop-lower); the long first-object approach — the important one — always
  reads a fresh retracted Z and eases in.
- The **hop-lower** inherits the easing automatically (same `MOVE_Z` handler): its
  FAST per-line feedrate drives the fast leg; the final mm always runs at the slow
  descent feedrate.

## Needs real-HW verification on ME3B V1
- Quick Print a multi-shape print → needle eases OUT at each inter-shape lift and
  eases IN (slow last mm) on each re-entry; final retract eases out.
- A pick-and-place / cell workflow → each inter-well touch-down eases in the last
  mm (via `safe_travel_to` step 3).
- Common Print Settings → set Slow zone distance / Slow speed → both lift and
  descent honor it; distance 0 → single-speed; value persists across restart.

---

## Addendum (2026-07-01) — fix: gentle slow descent falsely aborted the print ("M400 timed out → board stuck")

**Symptom (operator):** a Quick Print aborted right after picking up ink —
`flush_moves: M400 timed out after 10.0s` → `Z move to print height 13.400 mm not
confirmed (board stuck or disconnected) — aborting before extrusion`. The operator
suspected the ink pickup (">10s → M400 timing issue"). The `>10s` intuition was
right; the culprit was **this feature's slow descent**, not the ink pump.

**Root cause (proven from `logs/prints/…154312…A5.jsonl` + `logs/zp_serial.log`):**
the operator had lowered the gentle-Z slow speed to **0.1 mm/s (6 mm/min)**. The
`MOVE_Z` descent emitted the two-segment gentle re-entry:
`G0 Z-50.44 F300` (fast ~20 mm ≈ 4 s) then `G0 Z-51.44 F6` (**final 1 mm @ 6 mm/min
= 10 s**) → ~14 s total. But the `MOVE_Z` handler confirmed with a **FIXED
`flush_moves(timeout_s=10.0)`** — so the M400 expired at 10 s while a perfectly
healthy slow descent was still finishing → `_z_confirmed=False` → the
abort-before-extrusion guard fired. The pump (p2) was idle/settled by then (the ink
aspirate had already drained via `move_pump_uL(settle=True)`); the descent itself
was the >10 s move. This is the **Z twin** of the pump-move-bleeds-into-M400 bug
(`MEBP_v75x_PREP_BUFFER_PUMP_DRAIN_BEFORE_TRAVEL.md`): a legitimately long,
deterministic move exceeding a fixed confirmation timeout.

**Fix — the confirmation timeout is now SIZED to the descent's estimated duration**
(fast leg + slow leg), floored at the caller's baseline, capped at 120 s:
- New `StageController.estimate_gentle_z_time_s(target_zref_mm, fast_fr, *, cur_zref_mm=None)`
  — pure geometry mirror of `_descend_z_moves_only` (slow LAST mm on a descent) /
  `_retract_z_slow_then_fast` (slow FIRST mm on a lift): picks the direction-
  appropriate `slow_dist`/`slow_fr`, returns `(fast/fast_fr + slow/slow_fr)·60` s.
  Reads cached Z when `cur_zref_mm` not given; returns 0.0 on unknown Z (caller
  floors anyway). New module constants `_GENTLE_Z_CONFIRM_MARGIN_S=5.0`,
  `_GENTLE_Z_CONFIRM_CAP_S=120.0`.
- `PrintManager` `MOVE_Z` handler: `_confirm_to = min(max(10.0, est+5), 120)` used
  for BOTH `flush_moves` and `wait_for_z_arrival` (was hard-coded `10.0`). Bare
  mock/older controller → `float()` raises → falls back to `10.0` (existing tests
  unaffected).
- `StageController.safe_travel_to` step 3 (descent confirm) and
  `_retract_z_slow_then_fast` (the slow-LIFT twin, so a low slow-speed can't
  falsely time out a lift either) both bump their `flush_moves`/`wait_for_z_arrival`
  timeouts by the same estimate (`max(baseline, est+margin)`, capped).

`SimplePrintManager` is unaffected — its `_confirmed_descent` does a single-speed
`move_z_absolute` (no gentle final mm), so its descent never has a slow leg.

**Files:** `SupportClasses/StageController.py` (constants + `estimate_gentle_z_time_s`
+ `_retract_z_slow_then_fast` + `safe_travel_to` step 3), `SupportClasses/PrintManager.py`
(`MOVE_Z` handler). Tests: `tests/test_v75x_gentle_descent_slow_final.py`
(`TestGentleZConfirmTimeoutIsDurationAware`, 5 new; 23 total green); gentle-retract
/ z-retract / always-safe-z / spheroid-z / quick-print-pick&place / zp-feedrate /
print-setup-routine / pump-settle suites green (187+).

**Needs real-HW verification on ME3B V1:** with the gentle-Z slow speed set low
(e.g. 0.1 mm/s), Quick Print a print → the descent eases in over the last mm and
the print STARTS (no false "board stuck" abort); confirm `zp_serial.log` shows the
`G0 Z… F6` slow leg followed by an M400 that returns `ok` (not `timeout`).

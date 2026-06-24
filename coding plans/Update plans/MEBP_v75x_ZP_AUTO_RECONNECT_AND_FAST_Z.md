# MEBP v7.5.x — faster Z moves + ZP auto-reconnect (sustained-load hardening)

## Context

After the print-path planner-buffer fix (`MEBP_v75x_PRINT_PATH_PLANNER_BARRIER.md`)
a *single* print runs clean (verified on HW: descent confirmed, path acked
653/653, completed). But a **sustained back-to-back stress loop** still trips the
USB-layer `WriteFile ERROR_BAD_COMMAND` drop, and the trace showed the Z moves
**slowing first** — `M400 ok 6.1 s → 6.5 s → timeout 10 s` — before the drop,
consistent with the "only the Z stepper gets hot" report (Z driver heating →
slower moves) plus the recurring USB drop. Two hardening changes (user-chosen):

## Changes

| File | Change |
|------|--------|
| `SupportClasses/StageController.py` | **Faster Z:** new `_refresh_zp_move_feedrates()` derives the Z move feedrates from the configured Z max — retract (UP, always safe) = the full Z max; insert (DOWN to print) = 0.6× — so a ~21 mm Z move finishes in ~2.5 s instead of ~6 s (less motor on-time/heat, completes within the M400 flush timeout). Called from `apply_device_settings`; no-op fallback keeps the conservative 200/100 defaults if no Z max is configured. **Auto-reconnect:** an UNEXPECTED ZP loss (`_handle_disconnect("ZP")`) now spawns a background `_zp_reconnect_loop` (`_schedule_zp_reconnect`) that retries `connect_zp(simulate=False)` up to `_zp_reconnect_attempts` (6) every `_zp_reconnect_delay_s` (2 s) — the CH340 re-enumerates on the same COM port, so the link self-heals without an app restart. On success `connect_stages` fires `on_connect("ZP")` → the GUI's position-restore prompt (reopening DTR-resets Marlin → position lost). Gated: real HW only + `auto_reconnect_zp` (default True) + not `_shutting_down`; getattr-safe so it never triggers from `__new__` test controllers; idempotent (`_zp_reconnecting`). `shutdown()` sets `_shutting_down` so a reconnect can't fight teardown. |
| `SupportClasses/PrintManager.py` | The print `MOVE_Z` descent now passes `feedrate_mm_min=ctrl._zp_insert_feedrate` (a controlled, deterministic speed) instead of inheriting whatever feedrate the previous command left set. |
| `tests/test_v75x_zp_auto_reconnect_and_fast_z.py` | **New** (10): feedrate derivation (Z max / per-axis priority / no-max fallback); MOVE_Z passes the insert feedrate; auto-reconnect scheduled on real ZP loss, NOT when simulated/disabled; reconnect loop stops on success, gives up after max, bails when shutting down. |

## Testing Notes

- `python -m unittest tests.test_v75x_zp_auto_reconnect_and_fast_z` → 10 OK.
- Full `test_v75x*` sweep → 775/776 (the 1 failure is the pre-existing CV
  `test_real_24_well_mosaic`, unrelated — `MosaicBuilder`/`VisionDetector`
  already modified in the working tree).
- **Real-HW (the point):**
  1. A **single** real print should be solid (the planner-barrier fix already
     made it so). Confirm.
  2. Under the **stress loop**, Z moves should now be ~2.5 s not ~6 s (shorter
     busy windows; `logs/zp_serial.log` M400s drop from ~6 s toward ~2.5 s).
  3. If the USB drop still happens, the app now **auto-reconnects** (watch
     `app.log` for "ZP auto-reconnect: attempt …" → "succeeded") and prompts to
     re-declare Z position. A print in progress aborts (can't resume — Marlin
     reset on the reopen).

## Issues & Decisions

- The auto-reconnect restores the LINK; it does not resume the aborted print
  (Marlin loses position on the DTR reset that reopening triggers). The operator
  re-declares Z and restarts. `_maybe_prompt_zp_position_restore` is once/session
  — if it already fired, re-declare manually (minor follow-up: re-arm it on an
  auto-reconnect).
- Faster Z is a **mitigation** for the Z-driver heating, not a cure. If the
  motor still overheats, the firmware levers are: a Marlin idle-stepper timeout
  (M84 S<sec>) so Z de-energizes between moves, and Z run/hold current (M906) —
  config, not app code.
- The USB drop itself is still a USB-layer event (power management / re-enum);
  disabling Windows USB selective suspend remains recommended. Auto-reconnect is
  the software resilience that makes a transient drop survivable.

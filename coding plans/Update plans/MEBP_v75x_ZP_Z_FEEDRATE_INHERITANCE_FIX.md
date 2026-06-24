# MEBP v7.5.x — ROOT CAUSE: Z moves inherited the pump's slow modal feedrate

## Objective

Stop the recurring "ZP board drops on the Z retract / only Z gets hot / works
flawlessly in jog+travel but fails almost immediately in a print." This is the
**actual root cause** behind the whole multi-session ZP-disconnect saga — and it
is **pure software** (as the user insisted), not the USB/electrical layer.

## The decisive evidence (2026-06-18 15:04 stress-print run)

`logs/zp_serial.log` (tail) + `logs/app.log`:

```
+85.863s  G0 Z18.4400 F300        ok            ← descent (explicit F — fine, 4.4 s)
+97.863s  M400        timeout 11999.9ms rx=6 busy=6 no-ok   ← Marlin stuck "busy", never "ok"
15:04:14  ANOMALY M114 timeout 11991ms rx=5 busy=5          ← even M114 now returns only "busy"
15:04:32  ANOMALY G0…Y read_error / write_error  WriteFile failed (ERROR_BAD_COMMAND)  ← board off USB
```

and the **retract** emitted just before it:

```
+80.999s  G0 Z39.5900   ok    ← NO feedrate!   (and again at +81.545)
```

The pump (P2 → Marlin **Y** axis on ME3B V1) streams as `G0 F1.7999… Y0.0016`
— **F1.8 mm/min**. Z and the pump are the **same Marlin board**, sharing
Marlin's *modal* feedrate (`F` persists across commands until changed).

So a Z move emitted **without an explicit `F`** inherited **F1.8 mm/min**. A
~21 mm full-travel Z retract at 1.8 mm/min = **~12 MINUTES**. Marlin sits
`busy:processing` executing it; the host's `M400` blocks the whole time →
"timeout"; the Z motor creeps under load and overheats ("only Z gets hot"); and
after minutes in that state the CH340 ultimately drops off USB
(`ERROR_BAD_COMMAND` = surprise device removal).

**Why only the print:** jog, "Go To", and `safe_travel_to` always set their own
`F`. Only the **print** interleaves slow per-segment pump moves (F1.8) right
before the Z retract/travel moves, poisoning the shared modal feedrate. That is
exactly why "all the motions work flawlessly, but the print fails immediately."

## Root cause at the emission site

`ZPStageManager.move_absolute` (ZPStage.py):
```python
feed_str = f" F{feedrate_mm_min:.0f}" if feedrate_mm_min else ""   # ← omits F when None
```
…and the many `move_z_absolute(...)` callers (TRAVEL_UP, end-of-print retract,
hybrid/trajectory waypoints, VelocityExecutor) pass **no** feedrate. So they
emitted bare `G0 Z…` lines that inherited the pump's F1.8. (`move_relative` was
already safe — it always emits an `F`, falling back to `self.feedrate`.)

## Changes

| File | Change |
|------|--------|
| `SupportClasses/ZPStage.py` | `move_absolute` now **always** emits an explicit `F` — falls back to the configured default feedrate (`self.feedrate`, 200 mm/min) instead of omitting `F` and inheriting Marlin's last (pump) modal feedrate. Mirrors `move_relative`. Defensive chokepoint covering every absolute-move caller. |
| `SupportClasses/StageController.py` | `move_z_absolute`: when `feedrate_mm_min is None`, resolve a proper **Z** feedrate (`_zp_retract_feedrate` → `safety_limits.max_z_feedrate` → 200) before delegating, so a bare Z move gets a real Z speed (not the generic default and never the pump's). Fixes all bare `move_z_absolute(...)` sites at once. |
| `SupportClasses/PrintManager.py` | `TRAVEL_UP` passes the explicit **retract** feedrate; `TRAVEL_DOWN` passes the **insert** feedrate. `MOVE_Z` now **aborts the print** (raises → clean ERROR + safe-Z retract) if the needle can't be confirmed at print Z (M400 timeout or arrival fail) instead of dry-dragging at the wrong Z — the failed run showed the descent un-confirmed (Z frozen at 39.59) yet the print continued. |
| `gui/pages/workflows/stress_test_workflow.py` | `_print_burst` returns a stop signal; a print that ends in ERROR (or a start error / link-down / operator stop) **stops the whole stress test** instead of launching the next print onto a degraded board. The run loop honors it. Added per-print "starting…/complete" logging (fixes the "says complete before starting" confusion). |
| `tests/test_v75x_zp_feedrate_inheritance_fix.py` | **New (12):** move_absolute always emits F (incl. the exact pump-then-bare-Z regression); move_z_absolute resolves retract→Zmax→200; TRAVEL_UP/DOWN feedrates; MOVE_Z aborts on M400 timeout / arrival fail, not on success. |

## Testing Notes

- `python -m unittest tests.test_v75x_zp_feedrate_inheritance_fix` → 12 OK.
- ZP/print/stress suites (8 files) → 85 OK.
- Full `test_v75x*` sweep → 787/788 (the 1 failure is the pre-existing,
  unrelated CV `test_real_24_well_mosaic`, 23 vs 24 circles).
- **Real-HW (the point), on ME3B V1:**
  1. Run a single print and a stress print. In `logs/zp_serial.log`, every
     `G0 Z…` must now carry an explicit `F` (e.g. `F500` retract / `F300`
     descent) — there should be **no bare `G0 Z…`** lines, and **no `M400`
     that sits `busy` for minutes**. The Z retract/descent should complete in
     ~2.5–4 s.
  2. The Z motor should run far less / stay cooler (no multi-minute crawls).
  3. If a Z move still can't be confirmed, the print now aborts and retracts
     (logged `move_z_abort_unconfirmed`) rather than printing at the wrong Z.

## Issues & Decisions

- This supersedes the "faster Z + auto-reconnect" framing as a *thermal/USB*
  problem (`MEBP_v75x_ZP_AUTO_RECONNECT_AND_FAST_Z.md`): the Z heat and the
  drop were **downstream of** the 12-minute crawl, which was a feedrate-
  inheritance bug. The faster-Z feedrate derivation is still useful (it gives
  retract/descent their explicit fast `F`), and auto-reconnect remains the
  fallback for a genuine transient USB drop.
- Auto-reconnect "failing" in the 15:04 log was expected: the operator
  **physically unplugged** the board, so no probe could find it. For a
  transient drop the CH340 re-enumerates (found by VID:PID on any COM number).
- The USB-layer `ERROR_BAD_COMMAND` is still a surprise-removal event, but with
  the crawl gone the board should never enter the prolonged stuck/hot state
  that precedes it. Disabling Windows USB selective suspend remains advisable
  defense-in-depth.

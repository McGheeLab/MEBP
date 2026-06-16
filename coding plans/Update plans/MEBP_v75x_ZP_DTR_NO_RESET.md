# MEBP v7.5.x — ZP Serial DTR/RTS No-Reset on Connect & Disconnect

## Objective

**Safety fix.** Closing the application caused every active axis on the ZP
stage (Z + pumps P1/P2/P3) to run to its mechanical extents — "chaos" in the
user's words, and a hardware-damage risk.

This implements the **"Layer 6 (DTR-no-reset probe)"** that was explicitly
deferred in `MEBP_v750_ZP_RECONNECT_HOTFIX.md` ("deferred pending real-HW
logs").

## Root Cause

The shutdown path issues **no motion command**:

```
MainWindow.closeEvent → controller.shutdown() → disconnect_zp()
    → ZPStage.stop() → serial.close()
```

Verified end-to-end: no `G28`/home, no `G0`/`G1`, no quickstop anywhere in
stop/disconnect. The jog handlers and watchdog only stop threads and send
`M500` (EEPROM save). The **only** board-touching action on shutdown is
`serial.close()`.

The motion is therefore a **hardware side effect of the Arduino/Marlin
auto-reset**: on CH340/Arduino-class boards the **DTR** line (and on CH340,
**RTS**) is capacitively coupled to the MCU `RESET` pin. The default
`serial.Serial(port, …)` **asserts DTR on open** (the reset the code already
documents at `ZPStage.py` `_try_open_marlin`/`_initialise_serial`) and the OS
**de-asserts DTR on close**, so **both connect and disconnect reboot the
board**. On reboot Marlin de-energizes the steppers — a gravity-loaded Z
drops and backdrivable/spring-loaded pump axes drift, so each axis runs to
its mechanical extent — and, if the firmware auto-homes on boot, drives them
there under power. (Observed symptom from user: unsure free-fall vs. driven;
this fix addresses both because it removes the reboot entirely.)

## Fix (revised after real-HW feedback)

**First attempt (reverted):** open the port with DTR/RTS pinned low for the
whole session so neither open nor close emits a reset edge. → **Broke
connection on the ME3B V1 board.** That board *requires* the open-time DTR
reset edge to start talking (with the lines held low, the M115 probe got no
answer). So suppressing the open reset is not viable here.

**Current approach:** leave the **open path at the default (resetting)
behavior** — the board connects exactly as before — and mitigate the
**close-time** reset only, in `ZPStage.stop()`:

- `ZPStage.stop()` (non-sim branch): set `dtr = False`/`rts = False`
  **before** `serial.close()`, so the OS close doesn't generate the
  resetting edge. This runs only at shutdown (after the session is done), so
  it **cannot affect connectivity**.

Whether the de-assert actually avoids the reset is **board/driver-polarity
dependent**. If it does not, the **reliable cure is hardware**: disable the
controller's auto-reset (cut the `RST-EN` jumper/trace, or fit a ~10 µF cap
from `RESET` to `GND`). That keeps serial data working (board still connects)
while removing the reset on both open and close.

## Files Modified

| File | Change |
|------|--------|
| `SupportClasses/ZPStage.py` | `stop()`: de-assert DTR/RTS before `close()` (close-time reset mitigation). `_try_open_marlin`: docstring note only — open path reverted to the default `serial.Serial(port, baud, timeout)` after the pin-low-before-open attempt broke connection. |
| `tests/test_v75x_zp_dtr_no_reset.py` | New — 7 tests (stop() de-asserts before close, stop() no-op w/o serial, stop() simulate delegates, Marlin probe still returns handle, port configured, non-Marlin → None+close, open-failure → None). |
| `coding plans/Update plans/MEBP_v75x_ZP_DTR_NO_RESET.md` | This plan. |

## Implementation Steps

- [x] Trace shutdown path; confirm no motion command is issued (only `serial.close()`).
- [x] Identify DTR/RTS auto-reset as the mechanism (corroborated by existing code comments + deferred Layer 6 note).
- [x] ~~`_try_open_marlin`: deferred-open with DTR/RTS pinned low~~ — **reverted**: broke connection on real HW (board needs the open reset).
- [x] `stop()`: de-assert DTR/RTS before `close()` (close-time-only mitigation; connectivity-safe).
- [x] Update `tests/test_v75x_zp_dtr_no_reset.py` to the close-time-only behavior (7 tests).
- [x] Verify no regression in `test_v75x_zp_position_override` / `test_v75x_zp_position_restore` (22 tests pass).
- [ ] **Real-hardware verification:** (a) board reconnects (regression check), (b) does close still slam the axes?

## Testing Notes

Sim/unit:
```
python -m unittest tests.test_v75x_zp_dtr_no_reset -v          # 7 pass
python -m unittest tests.test_v75x_zp_position_override \
                   tests.test_v75x_zp_position_restore          # 22 pass
```

**Real hardware (must be done on the ME3B V1 board):**
1. **Regression first:** launch → confirm the ZP board connects normally.
2. Jog Z + a pump to a mid-travel position; close the app.
3. **If axes hold:** the `stop()` de-assert worked on this board — done.
4. **If axes still run to extents:** software can't fix it on this board
   (open reset is required, same DTR line) → apply the hardware auto-reset
   disable (cut `RST-EN` / 10 µF `RESET`→`GND`).

## Issues & Decisions

- **Cannot separate open-reset from close-reset purely in software on this
  board.** The board needs the DTR reset edge on open to connect, and that is
  the same physical line that resets on close. Pinning the line low for the
  session (which would kill both) kills connection. So the software lever is
  limited to a close-time de-assert whose effect is polarity-dependent.
- **The reliable fix is hardware.** Disabling the controller's auto-reset
  (cut `RST-EN` / 10 µF `RESET`→`GND`) removes the reset on open AND close
  while leaving serial data intact — the board still connects and no longer
  reboots (so steppers stay energized / no auto-home) on shutdown.
- **`stop()` mitigation is strictly safe.** It runs only at shutdown and
  never touches the open path, so worst case it is a no-op equal to the
  original bug — it cannot regress connectivity.
- **Did not pre-park / disable steppers before close.** That does not help —
  if the reset is what moves the axes, parking first then closing still fires
  the reset on `close()`. Preventing the reset is the only effective software
  lever.
- **Left the open-path reset comments intact only where still accurate.** The
  "only one Arduino DTR reset" line in the old docstring was removed (now zero
  resets).
- **Module-shadowing gotcha in tests:** `import SupportClasses.ZPStage as m`
  resolves to the `ZPStageManager` *class* (re-exported by
  `SupportClasses/__init__.py`), not the module. The test pulls the real
  module via `importlib.import_module(ZPStageManager.__module__)` to
  monkeypatch its `serial`.

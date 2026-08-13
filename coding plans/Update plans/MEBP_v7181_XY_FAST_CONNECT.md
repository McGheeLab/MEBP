# MEBP v7.18.1 — XY connect stops re-scanning every protocol on every connect

## Objective

Operator report: *"every connection of the xy stage does this … we used to have it
so it was super fast at connecting"*, with a 7-second GUI freeze on each connect:

```
16:22:49 [I] StageController: XY stage connected (REAL)
16:22:47 [E] GuiWatchdog: GUI thread has not run for 5.7s — dumping all thread stacks
16:22:49 [D] XYStage: Failed to parse XY position: Expected 3 values, got 1: D
16:22:50 [W] GuiWatchdog: GUI thread recovered after ~7.0s
```

Make an XY connect on a known-good rig a single probe (~0.8 s, no watchdog
trip) instead of a full protocol × baud × port sweep, and stop the first
position read from eating the tail of the detection reply.

## Root cause — measured, not inferred

`settings.json` has `controller.controller_json = "auto"`, so **every** connect
runs `_auto_detect_controller`: each protocol JSON × each of its candidate
bauds × each COM port. From the operator's own `logs/app.log`:

| when | probe | result |
|---|---|---|
| 16:22:42.33 | COM3 @ 9600 — Ludl MAC 5000 | miss |
| 16:22:43.03 | COM3 @ 38400 — Prior ProScan II | miss |
| 16:22:43.92 | COM3 @ 38400 — Prior ProScan III | miss |
| 16:22:45.25 | COM3 @ 115200 — H117 | miss |
| 16:22:46.76 | COM3 @ 38400 — H117 | miss |
| 16:22:48.27 | COM3 @ 9600 — H117 | **MATCH** |
| 16:22:49.09 | XY stage connected | **6.76 s total** |

Only **one** candidate port exists (COM4 is correctly excluded as the ZP
board). The entire 6.76 s re-proves five combinations that had already been
ruled out on the previous run — and it runs on the GUI thread, which is why
`GuiWatchdog` fired every time. Per probe the cost is structural: 0.35 s
DTR-toggle + settle, 0.3–0.6 s wake, then up to two 0.3 s reads.

The ZP stage has had the cure since v7.4.2 (`preferred_port`, "try the
previously-good port first"); **XY never got the equivalent** — `XYStageManager`
had no preferred/port parameter at all. `controller.auto_detect_result` exists
in settings but is cosmetic (read only by the Settings page for display).

Second, smaller defect — the `got 1: D` warning: detection matches on the
**first line** of the H117's multi-line `STAGE` reply
(`STAGE = H117P1N4/F\rTYPE = 25\rMICROSTEPS = …`) and returns immediately. The
remaining lines are still arriving at 9600 baud, so `get_current_position`'s
`reset_input_buffer()` cannot remove them — it flushes, *then* the trailing
bytes land, and the first position read parses a fragment.

## Files Modified

| File | Why |
|---|---|
| `SupportClasses/XYStage.py` | `preferred=` hint; `_try_preferred` fast path; `_find_with_protocol(only_port, only_baud)` + hinted-port-first ordering; `_claim_port` (one place a match becomes the session); `_drain_quiet`; `connected_port`/`connected_baud`/`detection_hint` |
| `SupportClasses/StageController.py` | `_preferred_xy_hint`, `set_preferred_xy_hint()`, `xy_connection_hint`; passes `preferred=` into `XYStageManager`; adopts the live result after connect |
| `main.py` | restore `xy_stage.last_good` at startup; persist it in the headless path |
| `gui/pages/hardware/control_panel.py` | persist `xy_stage.last_good` after a real XY connect (mirrors the ZP row) |
| `gui/pages/hardware/stage_panel.py` | same, on the Device sub-page |
| `tests/test_v7181_xy_fast_connect.py` | NEW — 21 tests |

## Design decisions

- **The hint is `{protocol, port, baud}`, not just a port.** Port alone would
  still leave the protocol × baud sweep — 5 of the 6 wasted probes here were
  *the same port*.
- **A miss ALWAYS falls back to the full scan.** The hint can make a connect
  faster; it must never make one fail. Moved cable, swapped controller, Prior's
  documented baud reversion, deleted protocol JSON — every one of those misses
  cleanly and rescans (each pinned by a test).
- **Even on a full scan the hinted port is ordered first**, so a hint whose
  *baud* went stale still saves every other port's probe time.
- **Nothing is persisted unless something actually answered.** `detection_hint`
  returns `None` without a `_connected_port`, so a failed scan can never write a
  hint that sends the next launch at nothing. (The case that matters is an
  *explicit* `controller_json`: the protocol is loaded before detection, so a
  protocol-is-None check alone would not catch it.)
- **The excluded-port rule keeps exactly ONE enforcement point.** A first cut
  added an early-out in `_try_preferred` as well; that is two homes for the rule
  that opening a port DTR-resets the board behind it. Removed — `_find_with_protocol`
  is the single gate. (This was caught by a mutation surviving; see below.)
- **The hint survives `disconnect_xy`**, unlike the ZP one. The ZP hint is
  cleared because a Marlin board re-enumerates to a different COM after a power
  cycle; the Prior is an FTDI adapter with a stable assignment, a miss is cheap,
  and keeping it is what makes an in-session reconnect fast.
- `StageController` adopts the winning triple immediately after connect, so a
  USB drop / re-click is already fast before the GUI persists anything.

## Implementation Steps

- [x] `_drain_quiet` + call it from `_claim_port`
- [x] `preferred=` ctor param, `_connected_port`/`_connected_baud` tracking
- [x] `connected_port` / `connected_baud` / `detection_hint` properties
- [x] `_try_preferred` fast path in `_find_controller`
- [x] `only_port` / `only_baud` restriction + hinted-port-first ordering
- [x] `StageController` hint plumbing (set / pass / adopt)
- [x] `main.py` restore + headless persist
- [x] both GUI connect buttons persist the hint
- [x] tests + mutation matrix

## Testing Notes

`tests/test_v7181_xy_fast_connect.py` — **21 green**, driving the production
`XYStageManager` against a fake serial injected at the ctypes/pyserial boundary
(`SupportClasses.XYStage.serial`), so the real detection loop runs.

Load-bearing test: with a good hint the fake records **exactly one port open**
(`[("COM3", 9600)]`). A companion "guard the guard" test asserts the un-hinted
sweep opens ≥ 5, so the one-open assertion cannot pass because nothing probes
at all.

**6/6 mutations CAUGHT** (each a real source edit, reverted in a `finally`):
fast-connect call removed (= the original bug) · trailing-reply drain removed
(= the `got 1: D` warning) · a hint miss made fatal instead of falling back ·
`detection_hint` published with nothing found · `preferred=` not passed to
`XYStageManager` (AST pin) · the excluded-port guard defeated.

⚠ **Two of my own tests were too weak and the mutation run caught them first** —
the trap this repo keeps recording:
- The excluded-port mutation **SURVIVED**: I had removed a *redundant* outer
  guard while the real one in `_find_with_protocol` still blocked the open, so
  the mutation changed no behaviour and proved nothing. Fixed by deleting the
  redundant copy (one rule, one home) and re-running the mutation against the
  real gate, which it now catches.
- The `detection_hint` mutation **SURVIVED**: my "nothing found" test used
  auto-detect, where `_protocol` is `None` anyway, so the `_connected_port`
  check was never exercised. Added a case with an explicit protocol loaded plus
  an assertion that a protocol really *is* loaded, so it cannot pass vacuously.

Regression, run under `.venv`: jog-navigation / context-panel / pump-plunger /
axis-max-speed / common-axis-speed / prior-multi-baud **108 green**, plus a
`gui.app` import smoke.

Pre-existing failures **PROVED not ours** in a `git worktree` at HEAD (never
`git stash` in this repo — see the v7.17 process incident): the same 10 errors
(`_note_move_estimate_xy_rel` on a `SimpleNamespace` in the ludl/envelope/
speed-conversion suites) and the same 3 failures + 1 error in the ZP batch
(`zp_dtr_no_reset::TestOpenStillConnects` ×2, `zp_auto_reconnect::
test_no_z_max_keeps_defaults` — all already documented in CLAUDE.md).

## Needs real-HW verification on ME3B_01, IN ORDER

1. **First** connect after the update is still the slow full scan (nothing is
   cached yet) — expected, once.
2. `settings.json` gains `xy_stage.last_good`
   = `{protocol: …proscan_iii_h117.json, port: "COM3", baud: 9600}`.
3. **The payoff:** Disconnect → Connect. `app.log` should show a single
   `XY fast-connect: … on COM3 @ 9600 baud in 0.8s (skipped the scan)` and
   **no** `GuiWatchdog` stall.
4. Restart the app — the same fast connect from cold.
5. **No** `Failed to parse XY position: Expected 3 values, got 1: D` on the
   first reads after connect.
6. **Self-heal check:** move the Prior to a different USB port (or edit the
   stored port to a wrong one) and confirm it still connects, logging
   `XY fast-connect miss … falling back to the full scan`, and that the new
   port is then cached.
7. Confirm COM4 (the ZP/Marlin board) is never opened by the XY scan — the ZP
   must not reset when XY connects.

## Issues & Decisions

- **Not done: moving the connect off the GUI thread.** The freeze is a real
  second defect — `connect_xy` is called synchronously from the button handler,
  so even a fast connect blocks the event loop. At ~0.8 s that is below the
  5 s watchdog and below the "feels frozen" bar, and backgrounding a serial
  connect touches the badge/state machine on both connect surfaces. Recorded as
  its own change rather than folded in here.
- **Not done: writing `controller.controller_json` to the detected protocol.**
  Pinning it would also skip the sweep, but it changes what the operator sees
  selected on the Settings page and would silently stop auto-detect from ever
  reconsidering. The hint is advisory and self-healing; a pinned protocol is not.

# MEBP v7.5.x — Click-to-travel runs OFF the GUI thread (fix "program freezes when jogging to another well with the needle down")

## Objective

Fix the operator-reported freeze: **"When the needle is DOWN in a well and I jog
to another well, the program freezes; but if I hit *Move to Safe Z* first, it
doesn't freeze."**

Root cause (confirmed by a 4-lens investigation + adversarial verify, and the
live `logs/app.log`): the Jog page's click-to-travel handler called
`StageController.safe_travel_to(...)` **synchronously on the Qt GUI thread**.
`safe_travel_to` step 1 retracts Z to safe height and *waits for confirmation*
(`flush_moves` M400 + `wait_for_z_arrival`). On ME3B V1 that retract-and-confirm
takes **11–14 s** (logged), and up to ~60–120 s when the board momentarily
stalls (`flush_moves: M400 timed out after 10.0s`). For that entire time the Qt
event loop is blocked → "Not Responding" = the freeze.

- **Why "Move to Safe Z first" avoids it:** that button (`_on_go_to_z_requested`)
  is fire-and-forget (`move_z_absolute` queues the move and returns immediately),
  so the GUI stays live while the needle rises. The subsequent well-click then
  runs `safe_travel_to` with the needle already up → step-1 retract is a fast
  near-no-op → no freeze.
- **Not a broken safety check.** The retract-before-XY safety logic is correct
  and unchanged; it was simply run on the GUI thread. Not a true deadlock
  (`_serial_lock` is a reentrant `RLock`, no cycle) — a long, in-practice-bounded
  block (pathologically unbounded only on a never-completing move).

Fix (operator chose **Full fix + serial hardening**):
1. Run every Jog-page / workflow-page **click-to-travel** `safe_travel_to` on a
   daemon worker thread with a Qt-Signal bridge (the pattern the Quick Print
   pre-position and the pick-&-place executors already use), plus a **busy-guard**
   so rapid re-clicks are ignored (not queued behind each other on the shared
   serial channel).
2. **Serial hardening:** give `ZPStage._read_until_ok` an absolute wall-clock cap
   so a never-completing move (whose Marlin `busy` keep-alives used to reset the
   wait window forever) fails after a ceiling instead of hanging the worker
   thread — and, under `_serial_lock`, every other ZP thread — indefinitely.

## Files Modified

| File | Change |
|------|--------|
| `gui/widgets/safe_travel_worker.py` | **NEW.** `SafeTravelWorker(QObject)` — runs `controller.safe_travel_to(*args, **kwargs)` on a daemon `threading.Thread`; emits `finished(bool)` (queued to the GUI thread since the QObject lives there); busy-guard (`start()` returns `False` while a travel is in flight — the click is ignored, not queued); `_clear_busy` connected to `finished` toggles the flag on the GUI thread. |
| `gui/pages/jog_control.py` | `_on_workspace_position_clicked` + `_on_workspace_fast_travel_requested` dispatch through `self._travel_worker.start(...)` instead of calling `safe_travel_to` inline. New `_set_travelling` (WaitCursor on the workspace view) + `_on_travel_finished` (restore cursor, warn on `ok=False`). Worker created + wired in `__init__`. |
| `gui/widgets/standard_jog_context.py` | Absolute **Go To** `_absolute_goto` safe-travel branch dispatches through `self._travel_worker.start(...)`; the Go button shows **"Travelling…"** + is disabled until `_on_travel_finished` re-enables it. Worker + `_btn_go` stored. The no-Safe-Z fire-and-forget branch is unchanged. |
| `gui/pages/workflows/spheroid_pickup_workflow.py` | Both click handlers dispatch through `self._travel_worker.start(...)`; worker created after `self._bridge`; `_on_travel_finished` added (distinct from the executor's existing `_on_finished`). |
| `gui/pages/workflows/cell_targeting_workflow.py` | Same treatment. |
| `gui/pages/workflows/cell_labeling_workflow.py` | Same treatment. |
| `gui/pages/workflows/*` (guard) | Each workflow page gained `_travel_blocked_by_run()` — both click handlers now refuse a manual travel while an executor run is active (`_exec_thread` alive), showing "Busy running — abort first to move manually." Closes the pre-existing hazard the review flagged: a click during a run would launch a 2nd `safe_travel_to` and its `finally→resume()` could un-suspend the non-refcounted `PositionPoller` mid-run. The Jog page owns no executor and needs no guard. |
| `SupportClasses/ZPStage.py` | New `READ_OK_HARD_CAP_S = 180.0` class constant; `_read_until_ok` computes `hard_deadline = now + max(ok_timeout, cap)` and returns outcome `hard_timeout` at the top of the loop once exceeded, so `busy`/live-line deadline resets can no longer extend the wait forever. `flush_moves` (its own bounded loop) is unaffected. |
| `tests/test_v75x_jog_travel_off_gui_thread.py` | **NEW** (8). `SafeTravelWorker` off-thread execution, busy-guard ignores re-click, exception→`False`, timeout→`False`, missing-controller refused; `_read_until_ok` busy-forever → `hard_timeout` (bounded), `ok` fast return, silence uses normal timeout not the cap. |
| `tests/test_v75x_z_retract_before_xy_travel.py` | Updated the two click-to-travel assertions to attach a real `SafeTravelWorker` and **join the worker thread** before asserting `safe_travel_to` was called (the call is now async; the intent — retract, not skipped — is unchanged). Added an offscreen `QApplication` in `setUpModule`. |

## Implementation Steps

- [x] Root-cause investigation (4 lenses + adversarial verify): GUI-thread block, not a deadlock; 3 synchronous entry points on the Jog page + the same latent bug in the workflow-page click handlers.
- [x] New `SafeTravelWorker` helper (daemon thread + `finished` Signal + busy-guard).
- [x] `ZPStage._read_until_ok` absolute wall-clock cap (`READ_OK_HARD_CAP_S`).
- [x] Jog page: dispatch both travel handlers off-thread + WaitCursor cue + finished slot.
- [x] `StandardJogContextPanel`: dispatch Absolute-Go-To off-thread + "Travelling…" button state.
- [x] Spheroid / Cell Targeting / Cell Labeling: dispatch both click handlers off-thread.
- [x] Tests (new suite + updated z-retract suite); import smoke for all modified modules.
- [x] Adversarial code review of the diff (thread-safety / call-sites / hard-cap / residual concurrency).
- [ ] **Real-HW verification on ME3B V1** (see Testing Notes).
- [ ] Version completion checklist (architecture doc / README / push) when the version is declared complete.

## Testing Notes

Automated (all green):

```
python -m unittest tests.test_v75x_jog_travel_off_gui_thread            # 8
python -m unittest tests.test_v75x_z_retract_before_xy_travel           # 22
python -m unittest tests.test_v731_jog_navigation \
    tests.test_v75x_workflow_settings_popout \
    tests.test_v75x_zp_serial_flow_control \
    tests.test_v75x_zp_close_during_read_crash \
    tests.test_v75x_zp_jog_clamp_freeze                                  # 121 combined
python -m unittest tests.test_v75x_gentle_descent_slow_final \
    tests.test_v75x_print_always_safe_z \
    tests.test_v75x_spheroid_pick_place_z \
    tests.test_v75x_cell_targeting_removal tests.test_v75x_cell_labeling \
    tests.test_v75x_stress_test_workflow                                 # green
```

Real-HW verification on ME3B V1 (the fix is a threading change; behavior of the
move itself is unchanged):

1. **The bug:** lower the needle into a well (print/pick Z), then click a
   different well on the **Jog** page. The GUI must stay **responsive** during the
   retract+travel (cursor shows busy over the workspace; live camera keeps
   painting) — no "Not Responding". The needle retracts to safe Z, travels, and
   the needle marker jumps to the destination when the (poller-suspended) move
   completes.
2. **Re-click while travelling** is ignored (no second travel queues).
3. **Absolute Go To** with *Safe Travel* checked and the needle down: Go button
   reads "Travelling…" + disabled, GUI live, re-enables on completion.
4. Repeat on **Spheroid / Cell Targeting / Cell Labeling** workspace clicks.
5. Confirm normal prints/travel still behave (the 180 s `_read_until_ok` cap
   never trips in normal operation — an `ok` returns in ms on a healthy board).

## Issues & Decisions

- **Not a broken safety check.** The retract-before-XY invariant is preserved
  verbatim; only *where the blocking wait runs* changed (worker thread vs GUI
  thread). `safe_travel_to` args are identical.
- **Busy-guard = ignore, not queue.** Rapid re-clicks during a travel are
  dropped. Chosen over queuing because queuing N serialized multi-second travels
  behind each other on the shared serial channel is worse UX and would still
  starve the poller.
- **Shared helper vs per-page bridge.** Used one reusable `SafeTravelWorker`
  (instead of extending each page's existing `_ExecutorBridge`) so the travel
  path is independent of the executor-run bridge and the pattern is identical in
  all 5 call sites.
- **`READ_OK_HARD_CAP_S = 180 s` (deliberately generous).** An `ok` means
  "admitted to the planner buffer" (ms on a healthy board); even a saturated
  planner admits far under 180 s, so only a truly stuck/crawling move reaches the
  cap. Since the GUI is now responsive (worker thread), being conservative here
  (no false abort of a legitimate slow operation) is preferred over a tighter
  ceiling. `flush_moves` (M400 motion-complete wait) has its own separate bound.
- **Pre-existing, unrelated failure:** `test_v75x_zp_auto_reconnect_and_fast_z::
  test_no_z_max_keeps_defaults` fails in the current working tree because of the
  large uncommitted WIP in `SupportClasses/StageController.py` (a new
  `get_max_z_feedrate_mm_min` resolver — not present at HEAD, not touched by this
  change). Confirmed independent of this fix.
- **Adversarial review (4 lenses → verify): fix confirmed correct + thread-safe.**
  Two findings, both applied: (1) *medium* — the 3 workflow pages didn't gate
  click-to-travel against a running executor (pre-existing, not a regression —
  the old inline call had the same overlap). Fixed with `_travel_blocked_by_run()`
  (see Files Modified). The deeper lever — making `PositionPoller.suspend/resume`
  reference-counted so overlapping suspenders compose — is a **tracked follow-up**;
  the guard prevents the overlap that would trigger it. (2) *nit* — a stuck-but-
  chatty board (streaming non-`busy` "live" lines) logged `timeout` instead of
  `hard_timeout` at the cap; fixed by leaving the live-line `deadline` uncapped so
  the top-of-loop hard check is the single exit for both stuck paths (trace-label
  only; both already returned `ok=False`, bounded).
- **Residual risks (documented, orthogonal):** (a) the `ConnectionWatchdog` stays
  active during Jog travel (unchanged from before); (b) simultaneous Xbox jog +
  worker travel serialize on the reentrant `_serial_lock` (no deadlock); (c)
  non-reference-counted `PositionPoller` suspend/resume (follow-up above).

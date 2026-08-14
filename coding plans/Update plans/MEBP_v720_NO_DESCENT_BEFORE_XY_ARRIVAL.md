# MEBP v7.20 — Every move is verified before the next axis moves

**Status:** code complete, tested, mutation-verified (15/15). **Needs real-HW
verification on ME3B V1.**

---

## 1. Objective

Operator report:

> "the needle started moving down before the stage arrived at the print location.
> this broke the needle."

then, after the first fix:

> "every move needs to be verified before making the next axis move for critical
> paths so we dont break needles."

Round 1 fixed the specific ordering that broke (XY → descent). Round 2 audited
**every** motion call site on the critical paths and made the confirmation
impossible to ignore.

### The invariant, and where it actually matters

**Z and the pumps are the same board.** They are all axes of the one Marlin
controller (`ZPStage`, mapped via `axis_map`), whose planner executes queued
moves *sequentially* — so Z-then-pump and pump-then-Z ordering is guaranteed by
the firmware and needs no host-side confirmation.

**XY is a different board** (the Prior ProScan). It moves *genuinely
concurrently* with everything on the Marlin board.

So the axis pair that can actually break a needle is exactly **XY ↔ ZP**, and
the invariant reduces to two rules, both now enforced at the primitive:

1. Before any **XY** move: the Z retract must be *confirmed* (else the needle is
   dragged across the plate).
2. Before any **Z descent**: the XY arrival must be *confirmed* (else the needle
   is lowered at the wrong place).

Every fix below is one of those two rules at a site that was missing it.

---

## 2. Root cause

**One defect, replicated at five sites: the XY-arrival confirmation was computed
and then discarded.**

The discrete plan that Quick Print (and every well-plate print) executes is:

```
TRAVEL_UP  →  MOVE_XY  →  MOVE_Z  →  [prime]  →  PRINT_PATH
             (travel)     (DESCENT to print height)
```

`MOVE_XY` called `_wait_for_xy_settle(...)` and ignored the result — **because
there was no result to ignore**: that method `return`s bare on all three of its
exit paths (arrived / aborted / timed out), so it always evaluated to `None`. No
caller *could* distinguish "the stage is there" from "we gave up waiting". The
plan therefore advanced to `MOVE_Z` and drove the needle down at whatever
position the stage had actually reached.

The method's own timeout branch already said so:

```python
# v7.5.x exec log: a settle TIMEOUT silently continues execution —
# capture it; this is a prime suspect for desync bugs.
```

Logging was added. The guard was not.

### Why XY fails to arrive in the first place

Both mechanisms are already documented in this repo:

* **Stale-ack position reads.** `XYStage._read_position_line`'s docstring spells
  out the consequence: *"A failed parse returns `(None,None,None)` … makes
  `wait_for_xy_arrival` poll garbage until it times out."* v7.17.1 reduced how
  often that happens; it never changed what happens afterwards.
* **A genuinely slow stage.** The ME3B V1 Prior measures ~2.5× slower than
  commanded on short accel-dominated segments, and the flat `timeout=10.0` on
  `MOVE_XY` is not distance-aware — a long traverse can outlast it while moving
  perfectly well.

### The asymmetry that made it invisible

`safe_travel_to` **already aborted** when the Z *retract* could not be confirmed:

```python
logger.error("safe_travel_to: Z retract not confirmed … ABORTING, will not start XY move")
return False
```

…but for the XY *arrival* it did the opposite:

```python
logger.warning("safe_travel_to: XY arrival timed out — proceeding with Z descent anyway")
ok = False
```

One unconfirmed axis aborted; the other one lowered the needle. `MOVE_Z` also
already raised on an unconfirmed descent — so the descent was guarded on Z and
unguarded on XY, which reads as "well protected" at a glance.

### The five sites

| # | Site | Old behaviour |
|---|---|---|
| 1 | `PrintManager._wait_for_xy_settle` | returned `None` on every path — unusable as a gate |
| 2 | `PrintManager` `MOVE_XY` handler | discarded the wait, fell through to `MOVE_Z` |
| 3 | `StageController.safe_travel_to` | *"proceeding with Z descent anyway"* |
| 4 | `HybridPlanExecutor._execute_print_step` | warned, then `move_z(print_z)` |
| 5 | `DirectCommandExecutor.travel_to_well` | warned, then Phase 3 lowered into the well |
| 6 | `SimplePrintManager._confirmed_xy` | named for the confirmation it discarded |

`PickAndPlaceManager._intra_well_move` was **already correct** (it raises
`AbortException` on an unconfirmed in-well XY move) — that path is the model the
others now follow.

---

## 2b. Round 2 — the full audit

Enumerating every motion call on the critical paths turned up **the same discard
at eleven more sites**, all of the form "compute a confirmation, ignore it, move
the next axis".

| Site | Was |
|---|---|
| `DirectCommandExecutor.raise_z` ×4 callers (`TRAVEL_XY`, `MOVE_SAFE_Z`, `RETURN_HOME`, `travel_to_well`) | returned `False`; **every caller then moved XY** — the drag hazard |
| `DirectCommandExecutor.move_z` ×5 callers (well entry, approach, raise-from-well) | returned `False`, discarded |
| `DirectCommandExecutor.move_xy` ×3 callers | returned `False`, discarded |
| `HybridPlanExecutor._execute_service` | discarded `travel_to_well`'s result, then **dispensed** — into whatever well the stage was actually over |
| `PickAndPlaceManager._safe_move_to` | discarded `safe_travel_to`'s verdict, then descended / aspirated / dispensed |
| `TRAVEL_UP` (fallback branch) | `move_z_absolute` + `time.sleep(0.5)` — no verification |
| `TRAVEL_DOWN` | `move_z_absolute` + `time.sleep(0.3)` — **a DESCENT**, unverified |
| `MOVE_Z_REL` | `move_z_relative` + `time.sleep(0.3)` — unverified |

### The structural fix

Patching eleven call sites is how this defect got here in the first place — the
result was *available* at every one of them and simply not used. So instead:

**`DirectCommandExecutor.move_xy` / `move_z` / `raise_z` now RAISE
`MoveNotConfirmedError` instead of returning a status a caller may ignore.**

That single change closes all twelve call sites at once and makes a future one
safe by default. Two broad `except Exception` handlers (`_execute_print_step`,
`execute`'s `TRAVEL_XY`) were re-ordered to let it through — they exist for
lookup/config faults, and swallowing a *motion* fault there would log it and
carry on to the next well, which is how the needle gets broken on **that** one.

The three sleep-based Z moves gained `_confirm_z_or_raise` (M400 + position
poll, duration-scaled), degrading to the legacy fixed settle only where
verification is genuinely impossible (ZP not connected, older controller, mock)
— absent is recoverable, wrong is not.

`PickAndPlaceManager._safe_move_to` now acts on the verdict, with the same
`is False` convention `_wait_xy_arrival_um` already documents (a stub that
cannot report is not a failure). This removes the last instance of the
asymmetry: its intra-well sibling already aborted, and the inter-well travel —
the needle crossing the whole plate — must not use the weaker policy.

---

## 3. Files modified

| File | Change |
|---|---|
| `SupportClasses/PrintManager.py` | new `MoveNotConfirmedError`; the three `DirectCommandExecutor` primitives raise it; `_wait_for_xy_settle` returns a verdict; new `_xy_settle_timeout_s` (distance-scaled); `MOVE_XY` gates on it and records `_last_xy_target`; new `_assert_xy_at_confirmed_target` called by `MOVE_Z` **and** `TRAVEL_DOWN`; new `_confirm_z_or_raise` used by `TRAVEL_UP`/`TRAVEL_DOWN`/`MOVE_Z_REL`; `PRINT_PATH` and `start()` clear the record; `HOME_XY` tracks it; both broad handlers re-raise; new `_MOVE_Z_XY_PRECONDITION_TOL_MM` |
| `SupportClasses/StageController.py` | `safe_travel_to` returns `False` instead of descending after an unconfirmed arrival |
| `SupportClasses/SimplePrintManager.py` | `_confirmed_xy` returns its verdict; `MOVE_XY` raises rather than handing off |
| `SupportClasses/PickAndPlaceManager.py` | `_safe_move_to` raises `AbortException` on a refused/unconfirmed inter-well travel |
| `tests/test_v720_no_descent_before_xy_arrival.py` | **NEW** — 34 tests |

---

## 4. Implementation

- [x] `_wait_for_xy_settle` → `True` only on confirmed arrival; `False` on
      timeout **and on abort** (an interrupted wait is not an arrival); `True`
      when there is no XY stage (unchanged no-stage behaviour).
- [x] `MOVE_XY` raises `RuntimeError` when unconfirmed. This reuses the contract
      `MOVE_Z` already had: `_execute_loop` catches → state `ERROR` → its
      `finally` runs `_retract_to_safe_z`. **The needle is still retracted when
      this fires** (MOVE_XY retracts first), so the stop is in the safe state.
- [x] **Distance-scaled timeout** (`_xy_settle_timeout_s`): `2·dist/speed + 10 s`,
      clamped `[10, 45]`. Failing closed only helps if healthy long moves still
      pass — a flat 10 s against a 2.5×-slow stage would have traded a broken
      needle for spurious aborts.
- [x] **Second, independent gate at the point of danger**
      (`_assert_xy_at_confirmed_target`, called by `MOVE_Z` *and* `TRAVEL_DOWN`).
      Redundant on a well-formed plan, deliberately: the original failure had
      exactly one enforcement point and it was discarded. Tolerance is loose
      (0.5 mm) because precision is `MOVE_XY`'s job — this catches gross
      mis-positioning (stall, soft-limit clamp, a plan that skipped the travel),
      not settle jitter.
- [x] `PRINT_PATH` **clears** `_last_xy_target` — the path drives XY across the
      well, so a later descent compared against the pre-path target would be a
      *false* abort. Clearing (not updating) is the honest answer: after an
      open-loop path there is no confirmed position. Every real plan puts a
      confirmed `MOVE_XY` before its next `MOVE_Z`, so no protection is lost.
- [x] `start()` clears it too — a reused `PrintManager` must not check this job's
      first descent against the previous job's position.
- [x] `safe_travel_to` returns `False` before the descent; needle left retracted.
- [x] Hybrid `_execute_print_step` **skips the well** (needle at safe Z; the next
      iteration re-retracts). `travel_to_well` returns `False` — every caller
      already treats that as a failed step.
- [x] `HOME_XY` deliberately does **not** raise: it is the end-of-plan return,
      nothing descends after it, and failing a finished print at its last step
      would be worse than useless. It logs and clears the record.

### Deliberately NOT changed

* **In-well `PRINT_PATH` moves.** Per CLAUDE.md these are exempt — Z is
  intentionally at print height for the pattern itself. Their settle waits stay
  advisory.
* **Manual jog.** The operator owns Z there.
* **The `_last_position_read_ok` gap on `XYStage`.** `ZPStage` has this
  read-validity flag and the XY side does not, so an XY arrival wait cannot tell
  a garbled read from a genuine one — it only sees "not arrived yet" and times
  out. Failing closed on the timeout is correct regardless, so this change does
  not depend on it; adding the flag is a separate, testable improvement.
* **The wash jiggle** (`PickAndPlaceManager._do_wash`) lifts / lowers / jogs XY
  on fixed dwells rather than confirmations. Left as-is: its own comment already
  reasons this out — a service well is millimetres across, so a missed confirm
  there is not a collision risk, unlike the in-well shift, which aborts. Z-vs-Z
  ordering within it is guaranteed by the Marlin planner anyway.
* **~12 `CalibrationPage._safe_navigate_to` call sites ignore its bool.**
  Disclosed, not fixed. The *needle* is already safe: `safe_travel_to` now
  refuses the descent itself, so the worst case is a caller believing a travel
  succeeded (a calibration-accuracy issue, not a crash). `_safe_navigate_to`
  already logs an error and returns `False`. Fixing the callers means editing a
  very large GUI file for no additional needle protection, so it is recorded
  rather than bundled in here.
* **Pump-move confirmation on the streamed print path.** `move_pump_uL(settle=True)`
  already drains via M400 for every discrete actuation. The per-segment
  emissions inside `PRINT_PATH` stay non-blocking by design (confirming each
  would destroy print quality), and they cannot break a needle: they are pump
  moves on the same sequential board, with Z held at print height.

---

## 5. Testing

`tests/test_v720_no_descent_before_xy_arrival.py` — **34 tests, all green.**

Every test asserts on the **motion commands actually issued**, not on a return
value; and "descend" is evaluated in the polarity-safe HEIGHT frame (`ZDIR = -1`
on ME3B V1, where a *larger* raw Z is physically lower). Every gate is paired
with a healthy-path test, so a gate that fires when it shouldn't fails too.

**15/15 mutations CAUGHT**, all four sources restored byte-identical:

| Mutation | Result |
|---|---|
| M1 settle helper returns nothing again (**the original defect**) | CAUGHT |
| M2 `MOVE_XY` gate removed | CAUGHT |
| M3 `safe_travel_to` descends after a timeout (**the original defect**) | CAUGHT |
| M4 `SimplePrintManager` gate removed | CAUGHT |
| M5 descent's own precondition disarmed | CAUGHT |
| M6 `PRINT_PATH` no longer clears the record (false aborts) | CAUGHT |
| M7 `move_xy` returns an ignorable status instead of raising | CAUGHT |
| M8 `raise_z` ditto (**the drag hazard**) | CAUGHT |
| M9 `move_z` ditto | CAUGHT |
| M10 print-step swallows the fault in its broad handler | CAUGHT |
| M11 `TRAVEL_XY` swallows the fault in its broad handler | CAUGHT |
| M12 `TRAVEL_UP` ignores its retract confirmation | CAUGHT |
| M13 `TRAVEL_DOWN` back to a blind sleep after a **descent** | CAUGHT |
| M14 `MOVE_Z_REL` back to a blind sleep | CAUGHT |
| M15 pick&place discards `safe_travel_to`'s verdict | CAUGHT |

⚠ **The mutation runs earned their keep four times — three of my own tests were
too weak and one of my mutations was a no-op:**

* **M11 SURVIVED** — the AST test pinned that the specific handler comes *first*
  but not that its body actually re-raises, so replacing `raise` with `pass`
  passed. Now it also requires a bare `ast.Raise` in the handler.
* **The hybrid print step** was protected by code no test exercised — I had
  tested `travel_to_well` and assumed it covered that path. `TestHybridPrintStep`
  exists because a mutation survived.
* **A substring check proved nothing** — `src.index("except Exception")` matched
  an unrelated *inner* handler, so the ordering was effectively untested. Rewritten
  over the AST.
* **A bad mutation** added `ok = False` but left the `return False` in place, so
  it changed nothing. A mutation that does not alter behaviour proves nothing.

⚠ **A guard-the-guard test caught a bug I introduced.** My first `MOVE_XY` edit
deleted the `ctrl.move_xy_absolute(...)` call itself — the stage would never have
moved. `test_confirmed_xy_proceeds_normally` failed immediately.

### Regression

**474 green** in one consolidated batch (new suite · z-retract-before-XY ·
always-safe-Z · jog-navigation · hybrid-execution · simple-print-manager ·
multi-object-seam · confirmed-segments · preposition-async · gentle-descent ·
bore-safety ×2 · cell-targeting · cell-labeling · spheroid ×2 · print-calibrator ·
print-setup-routine · travel-split).

**11 pre-existing failures PROVED not ours by a controlled A/B**: the same suites
were run twice, once with the current tree and once with **all gates reverted
simultaneously**, and produced *byte-identical* failure lists (re-run after
round 2 — still identical).

> A `git worktree` at HEAD would **not** have been a valid control here — this
> working tree already carried other sessions' v7.19/v7.21 WIP before this
> session began, so HEAD is a different tree, not a control. Reverting only this
> change in place is.

Pre-existing failures: `test_v76_hard_abort` ×4 and
`test_v75x_quick_print_workflow::test_settings_use_safe_z_and_flow` (the v7.19
plate-queue WIP in `quick_print_workflow.py`), `test_all_set_is_clean` +
`test_enabled_when_ready` ("Stage motion not characterised" / stale readiness —
both already recorded in CLAUDE.md), and 4 `max_z` badge tests in
`test_v791_calibration_and_layout_fixes` / a `test_v711` import error (Max Z was
retired by v7.17). None of these touch a file this change modifies.

⚠ **PROCESS HAZARD, DISCLOSED — the tree is being edited concurrently.** During
this work six workflow files under `gui/pages/workflows/` were modified inside a
three-minute window by another session, and one batch run failed on a transient
`NameError: PromotedSectionsPanel` in `spheroid_pickup_workflow.py` (a **v7.21**
reference, newer than this change) that passed on re-run once the other session
settled. All v7.20 markers were re-verified present afterwards. **Anyone
continuing here should re-verify these three files before editing them.**

---

## 6. Issues & decisions

0. **Raise, don't return.** The single most important decision. Every one of the
   ~17 sites had the confirmation *available* and ignored it; a status that can
   be ignored eventually is. Raising at the primitive fixed twelve call sites at
   once and makes the next one safe by default. The cost is that a stage which
   genuinely cannot confirm now stops the run — which is the point.
1. **Fail closed, not fail soft.** An unconfirmed arrival now stops the print.
   The alternative — descending anyway — is what broke the needle. The stop
   always leaves the needle retracted, which is the recoverable outcome.
2. **Two gates, not one.** The descent verifies for itself even though `MOVE_XY`
   already refuses to hand off. The whole failure was a single enforcement point
   being discarded.
3. **Timeout scaled to distance.** Otherwise this change would trade a broken
   needle for an unusable machine on long traverses.
4. **`PRINT_PATH` clears rather than updates the record** — after an open-loop
   path we genuinely do not have a confirmed position, and inventing one would
   be the same class of mistake as the original bug.
5. **`HOME_XY` logs rather than raises** — nothing descends after it.

---

## 7. Needs real-HW verification on ME3B V1, IN ORDER

1. **A normal Quick Print still runs end to end.** This is the go/no-go: the
   gates must not fire on healthy hardware. Watch for `settle_wait ok=true` in
   `logs/prints/*.jsonl`.
2. **A long full-plate traverse still prints** (the distance-scaled timeout). If
   a legitimate long move now aborts, raise the cap in `_xy_settle_timeout_s`
   rather than removing the gate.
3. **Provoke the fault:** with the needle retracted and clear of the plate, pull
   the Prior's USB (or command a target outside the reachable envelope) during
   the travel to the print start. Expect: the print **stops with the needle
   retracted**, state `ERROR`, and `move_xy_abort_unconfirmed` in the exec log —
   **and the needle never moves down**.
4. **Multi-object print in one well** — the hop path (`MOVE_XY{hop_z}` →
   `MOVE_Z`) must behave identically, with no false aborts between objects.
5. **A multi-well print** — confirm no false abort at any well transition, and
   that `PRINT_PATH` clearing the record does not trip the descent gate.
6. **Calibration / pick-and-place travel** (they share `safe_travel_to`): a
   "Go To" well and a spheroid pick must still lower normally.
7. **A full pick-and-place run** (prep → ink → wash → waste): each service well
   must still be entered and dispensed into normally. `_safe_move_to` now aborts
   the whole run on an unconfirmed travel, so a false trip here would be
   immediately obvious.
8. **A hybrid-mode print** (`execution.mode = "hybrid"`), which exercises the
   `DirectCommandExecutor` primitives that now raise — this is the path with the
   most changed call sites and the least test coverage before this change.
9. Confirm `logs/prints/*.jsonl` shows no `move_z_abort_xy_moved`,
   `*_unconfirmed`, or `move_xy_abort_unconfirmed` on healthy runs.

⚠ **If a gate false-trips on the bench, do not remove it.** Each one names its
target and timeout in the message and the exec log. Raise the relevant timeout
(`_xy_settle_timeout_s`'s cap, or `z_timeout_s`) — a too-short timeout is the
expected cause, and the numbers are tuned from simulation, not from this rig.

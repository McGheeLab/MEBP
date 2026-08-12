# MEBP v7.17.0 — Python crashes when a mosaic scan COMPLETES

## Objective

Operator: *"I just finished the workflow->mosaic builder. it finished it
correctly, then after it was complete python crashed."*

Then, on being told it was the already-fixed zoom bug: *"no that is unreliable,
the program crashed when it finished the mosaic, so there is a problem with
that. look through to find the actual solution."*

**The operator was right and my first answer was wrong.** This documents the
real cause.

---

## ⚠ My first diagnosis was wrong — recorded because the reasoning error matters

I attributed this to `MEBP_v717_JOG_VIEW_ZOOM_CRASH.md` (the unbounded mosaic
`scaled()`), because `logs/crash.log` contained access violations whose dump
listed `jog_workspace_view.py, line 868 in _paint_mosaic_overlay` — the pre-fix
`scaled()` line — and because the crashing process had started at 09:20, before
that fix was committed at 11:06.

Every one of those facts is true, and the conclusion was still wrong:

- **The GUI thread was a BYSTANDER in that dump.** faulthandler marks the
  faulting thread `Current thread`; the mosaic-paint frame is listed under plain
  `Thread`. It was merely painting when another thread faulted.
- **`crash.log` is append-only across 361 dumps from many sessions** and, when
  several threads fault at once, its writes physically interleave (there are
  lines in it like `Windows fatal exception: access violationWindows fatal
  exception:`). Attribution from it alone is unreliable — exactly the operator's
  word for it.
- **The zoom explanation required an assumption I never verified** — that the
  view happened to be zoomed in. `_zoom` is not persisted anywhere and starts at
  1.0, and at zoom 1 the requested pixmap is ~999×672. I filled that gap with a
  plausible story instead of a measurement.

Lesson for this file's own genre: a stack frame in a multi-thread crash dump is
evidence of *presence*, not of *guilt*, and "the process predates the fix" is not
evidence that the fix was for *this* crash.

---

## Root cause (reproduced deterministically — 5 crashes in 5 runs)

**A completed worker's QThread is garbage-collected while its `run()` is still
executing.**

Every background worker in this app shares two properties:

1. it emits its completion signal (`finished_ok` / `done` / `failed`) from
   **inside `run()`**, not from `QThread.finished`; and
2. it is constructed with **no Qt parent**, so the page attribute holding it is
   the ONLY strong reference and **Python owns the C++ object**.

`_SingleWellMosaicWorker.run()` emits `finished_ok` and then, in its `finally`,
does real work on the worker thread:

```python
self.finished_ok.emit(composite.copy(), extent, scale, frames, shift, meta)
except ...
finally:
    self._restore_focus(entry_focus)     # a Nikon Ti COM focus MOVE
    self._scope.release(self._LEASE)
    self._controller.resume_position_poller()
```

The connection is queued (worker → GUI), so the GUI thread runs
`_on_channel_finished`, whose **first line** was:

```python
self._worker = None      # ← drops the LAST reference to a running QThread
```

Python's GC then runs `~QThread()` under the live thread. That is undefined
behaviour, and here it is fatal.

**Measured in an isolated reproduction of exactly that shape: 5 crashes out of
5, exit code `0xC0000409`** (the CRT fail-fast / `abort()` signature) **with no
traceback, no stderr and no Qt warning** — i.e. "python just closed". Holding the
reference instead survived 5/5.

### Why this fits the report exactly, where the zoom theory did not

- The scan **completes and is saved first** (`FluorescenceMosaicStore: saved
  nest-plastic-24/B1/mCherry` at 11:51:25,800) and the crash follows — "it
  finished it correctly, then after it was complete python crashed".
- It is tied to **completion**, not to any zoom/mouse action.
- No Python traceback, because it is a C++ destructor race.
- It explains why `MicroscopeControl.mounted_filters` appears in the faulting
  dump: the dying worker's `finally` was inside a Nikon Ti COM call while the
  microscope's own worker was concurrently polling COM, and `0x8001010d`
  (`RPC_E_CANTCALLOUT_ININPUTSYNCCALL`) is a COM apartment error, not a paint
  error.
- It is a race, so it does not fire every single time — matching "unreliable".

---

## The fix

NEW **`gui/worker_retirement.py`** — `retire_worker(worker)` keeps the object
alive until `QThread.finished` has been delivered **on the GUI thread**, then
releases it.

Applied as, at every site that releases a worker in response to a
`run()`-emitted signal:

```python
retire_worker(self._worker)
self._worker = None
```

Design points that are load-bearing:

- **Non-blocking, deliberately.** The obvious alternative, `worker.wait()`,
  would freeze the UI for the length of that microscope focus move and trip the
  GUI watchdog. Cancel/teardown paths that genuinely must synchronise already
  use `stop()` + `wait(timeout)` and are left alone — `_ploc_mosaic_cancel` and
  `plate_level_wizard` were already correct, which is how we know the rule was
  understood in one place and simply not applied in the completion paths.
- **The `finished` connection must be QUEUED.** `finished` is emitted *by* the
  worker thread, so a direct connection would drop the last reference — and run
  `~QThread()` — from inside the thread's own emission: the same crash by
  another route. The reaper is a `QObject` created on the GUI thread, which
  makes Qt's AutoConnection a queued one.
- **Qt emits `finished()` just BEFORE it sets `isFinished()`**, so a purge run
  synchronously in that slot can still see the thread as running. It purges
  again one event-loop turn later.
- **Every uncertain branch keeps the reference.** Holding a finished worker a
  turn too long costs a few bytes; releasing one a turn too early aborts the
  process. The error is one-directional and the code errs on holding.

### Sites fixed (6, in 3 files)

| File | Slot | Worker |
|---|---|---|
| `fluorescence_mosaic_workflow.py` | `_on_channel_finished` | `_SingleWellMosaicWorker` ← **the reported crash** |
| `fluorescence_mosaic_workflow.py` | `_on_channel_failed` | same |
| `calibration.py` | `_ploc_on_mosaic_finished` | `_MosaicScanWorker` (plate mosaic scan) |
| `calibration.py` | `_ploc_on_mosaic_failed` | same |
| `calibration.py` | `_ploc_on_auto_reanchor_done` | `_AutoReanchorWorker` |
| `calibration.py` | `_ploc_on_auto_reanchor_failed` | same |
| `spheroid_survey_panel.py` | `_on_detect_done` / `_on_detect_failed` | `_DetectWorker` |

⚠ **The last three pairs were found by the structural test, not by reading.** I
had fixed the fluorescence page and the plate-mosaic scan by hand and believed
that was the lot; the AST guard immediately reported
`_ploc_on_auto_reanchor_done/_failed` as well. The auto-reanchor worker drives
the *stage* in `run()`, so it had the same defect on a path that moves hardware.

`calibration.py::_ploc_on_mosaic_finished` deserves a note: it already captured
the worker into a local (`_worker = self._ploc_mosaic_worker`) before nulling the
attribute, which looks like lifetime care but only defers the drop to the end of
that method — and in the common case (no unreachable tiles, so no modal dialog)
that is microseconds later.

---

## Files Modified

| File | Rationale |
|---|---|
| `gui/worker_retirement.py` | NEW — the shared safe-release helper. |
| `gui/pages/workflows/fluorescence_mosaic_workflow.py` | Retire the worker in both completion slots (the reported crash). |
| `gui/pages/calibration.py` | Same for the plate-mosaic scan and auto-reanchor workers. |
| `gui/widgets/spheroid_survey_panel.py` | Same for the detection worker. |
| `tests/test_v717_worker_retirement.py` | NEW — 9 tests. |
| `coding plans/Update plans/MEBP_v717_MOSAIC_COMPLETION_CRASH.md` | NEW — this document. |

---

## Implementation Steps

- [x] Reject the first (zoom) attribution and re-derive from the code
- [x] Identify the emit-from-`run()` + no-parent + drop-in-slot pattern
- [x] Reproduce the crash in isolation (5/5, `0xC0000409`)
- [x] Add `retire_worker` and apply it at every completion slot
- [x] Verify the reproduction now survives 5/5 through the real helper
- [x] Structural test to catch NEW instances (which found 2 more sites)
- [x] Mutation-verify
- [x] Regression suites
- [ ] **Bench verification on ME3B V1 (below)**

---

## Testing Notes

`tests/test_v717_worker_retirement.py` — **9 tests, green.**

- Behaviour of `retire_worker`: held while the thread runs, released after it
  finishes, `None`-safe, idempotent, safe when retired after finishing, and
  **does not block the caller** (a test asserts <100 ms, so it cannot silently
  degrade into `wait()`).
- **End-to-end in a SUBPROCESS**, because the failure mode is a hard abort that
  would otherwise kill the test runner: the unfixed pattern must exit non-zero
  (`test_the_bug_really_does_abort_the_process` — guard the guard, so the sibling
  test is not vacuous) and the fixed pattern must exit 0.
- **`TestEveryWorkerReleaseIsSafe`** walks the GUI tree by AST: any function that
  assigns `None` to a `*_worker` attribute must either call `retire_worker` or
  `wait`, unless it is a constructor/`_build*` initialiser. It self-guards by
  asserting it examined ≥5 functions, so a broken matcher cannot pass. **This is
  the test that found the auto-reanchor sites.**
- `TestProductionSitesAreWired` pins the six specific slots by name.

**4/4 mutations CAUGHT** (sources hash-verified restored after each):

| # | Mutation | Result |
|---|---|---|
| M1 | the fluorescence completion slot stops retiring (the original bug) | CAUGHT |
| M2 | the plate-mosaic completion slot stops retiring | CAUGHT |
| M3 | `retire_worker` degrades to a no-op | CAUGHT — and the runner itself aborted with `0xC0000409` |
| M4 | the reaper releases a still-running worker | CAUGHT — ditto |

M3/M4 aborting the test process is the loudest possible signal, and it is also
why the primary end-to-end proof lives in a subprocess.

Regression, per-suite: worker-retirement 9 · jog-view-zoom 16 ·
fluorescence-mosaic 35 · spheroid-survey-tab 49 · fluor-mosaic-shift 11 ·
suite-hygiene 10 · mosaic-plate-frame 54 · spheroid-detection 43 ·
plate-mosaic 110 (class-by-class, `TestManualAlignPage` excluded per precedent) —
plus a `gui.app` import smoke confirming the helper is wired into all three
modules.

Two pre-existing failures PROVED not ours (both fail identically with these
changes stashed): `test_v75x_rosette_tab_auto_reanchor::test_tab_order_and_indices`
(the v7.11 "Plate Bed Level" tab rename, already recorded twice in CLAUDE.md) and
`test_v75x_plate_mosaic::TestFilledWellDetector::test_real_24_well_mosaic` (the
legacy blob detector).

⚠ **A harness bug of mine, disclosed:** my first class-by-class run of
`test_v75x_plate_mosaic` reported all 31 classes failing, which looked like an
import regression from this change. It was Windows `\r` left on the class names
by my own shell loop; with it stripped, 30/31 pass.

### Needs bench verification on ME3B V1

1. **Run a fluorescence mosaic scan to completion — the app must still be alive
   afterwards.** That is the whole fix. Do it two or three times: the old failure
   was a race.
2. Run a multi-channel scan so several channels complete back-to-back (each
   completion retires a worker).
3. Abort a scan mid-way — the failure path retires too.
4. Run a **plate-location mosaic scan** to completion, and an **auto re-anchor**
   (that worker moves the stage in `run()`).
5. Run the spheroid survey detector.
6. Check `logs/crash.log` gains no new dumps across all of the above.

---

## Issues & Decisions

- **Fixed the whole defect class, not just the reported instance.** The operator's
  complaint was that the behaviour is *unreliable*; leaving three more known
  instances of a crash-on-completion bug in the mosaic/calibration paths would
  have kept it unreliable.
- **Did not use `wait()`** in completion slots — see above. Cancel paths keep it.
- **Did not put the helper in `SupportClasses/`**: it needs Qt, and CLAUDE.md
  requires those modules to have zero GUI dependencies.
- 🐞 **Found but deliberately NOT fixed — reported instead:**
  `MicroscopeControl._read_all` calls `backend.filter_names()` /
  `objective_names()` on its **~1 s poll** (line 1638), and each of those walks
  the whole COM collection (`FilterBlocks.Count`, then `Item`/`Code`/`Name` per
  slot) — roughly 40 COM round-trips per second per turret. The `include_mounted`
  flag guards `mounted_filters`/`mounted_objectives` for exactly this reason
  ("optics only change when physically swapped", per the v7.5.x plan), but
  `filter_names`/`objective_names` reach the same COM walk unguarded, so that
  intent is defeated. This is very likely the source of the recurring
  `0x8001010d` COM noise in `crash.log`. It is a behaviour change to microscope
  polling and deserves its own verified pass rather than being folded into a
  crash fix.
- **The v7.17 zoom fix remains valid and unrelated.** That was a separately
  reproduced segfault (9.66 GB `scaled()` request at `_ZOOM_MAX`); it simply was
  not the cause of *this* crash. Two real bugs, one file apart.

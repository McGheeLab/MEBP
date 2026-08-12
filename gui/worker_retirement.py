"""worker_retirement.py — release a finished ``QThread`` worker SAFELY.

⚠ THE RULE THIS ENFORCES: **never drop the last Python reference to a QThread
while its ``run()`` is still executing.**

Every background worker in this app (``_SingleWellMosaicWorker``,
``_MosaicScanWorker``, ``_DetectWorker``, …) has the same two properties:

1. it emits its completion signal (``finished_ok`` / ``done`` / ``failed``)
   from **inside** ``run()``, not from ``QThread.finished``; and
2. it is constructed with **no Qt parent**, so the page attribute holding it
   (``self._worker``) is the ONLY strong reference and *Python* owns the C++
   object.

So a completion slot that does the obvious thing::

    def _on_channel_finished(self, ...):
        self._worker = None          # ← drops the last reference

hands the QThread to Python's garbage collector while ``run()`` is still
unwinding. In the fluorescence mosaic worker that ``finally`` block restores the
microscope focus (a real Nikon Ti COM move), releases the scope lease and
resumes the position poller — hundreds of milliseconds of work *after* the
signal was emitted. ``~QThread()`` then runs under the live thread, which is
undefined behaviour.

Measured, in an isolated reproduction of exactly that shape: **5 crashes out of
5**, exit code ``0xC0000409`` (the CRT fail-fast / ``abort()`` signature) with
**no traceback, no stderr and no Qt warning** — i.e. "python just closed".
Holding the reference until the thread finished survived 5/5.

``retire_worker`` keeps the object alive until ``QThread.finished`` has been
delivered on the GUI thread. It is deliberately **non-blocking**: the obvious
alternative, ``worker.wait()``, would freeze the UI for the length of that
focus move (and trip the GUI watchdog). Where a caller genuinely must
synchronise — cancel/teardown paths — ``stop()`` + ``wait(timeout)`` is still
the right tool and those paths already use it.

Erring is one-directional here: holding a finished worker one event-loop turn
too long costs a few bytes, while releasing one turn too early aborts the
process. Every uncertain branch below therefore keeps the reference.
"""

from __future__ import annotations

import logging
from typing import Any

from PySide6.QtCore import QObject, QTimer

logger = logging.getLogger(__name__)


class _Reaper(QObject):
    """Owns retired workers until their threads have actually exited.

    Lives on the GUI thread (it is created from a GUI-thread slot), which is
    what makes the ``finished`` connection below a QUEUED one.
    """

    def __init__(self) -> None:
        super().__init__()
        self._alive: list[Any] = []

    def keep(self, worker: Any) -> None:
        self._purge()
        if worker is None or any(w is worker for w in self._alive):
            return
        self._alive.append(worker)
        try:
            # AutoConnection + a receiver whose thread affinity is the GUI
            # thread ⇒ QUEUED. That matters: ``finished`` is emitted BY the
            # worker thread, so a direct connection would drop the last
            # reference — and run ~QThread() — from inside the thread's own
            # finished emission, which is the very crash this module exists to
            # prevent.
            worker.finished.connect(self._on_finished)
        except Exception:                                  # pragma: no cover
            logger.debug("retire_worker: could not connect finished", exc_info=True)
        # A worker that finished before we got here never emits again.
        self._purge()

    def _on_finished(self) -> None:
        # Qt emits finished() just BEFORE it sets isFinished(), so a purge run
        # synchronously here can legitimately still see the thread as running.
        # Purge now (usually enough) and once more next event-loop turn, by
        # which point isFinished() is settled.
        self._purge()
        try:
            QTimer.singleShot(0, self._purge)
        except Exception:                                  # pragma: no cover
            pass

    def _purge(self) -> None:
        keep: list[Any] = []
        for w in self._alive:
            try:
                done = bool(w.isFinished())
            except Exception:
                # Unknown state ⇒ assume it is safe to let go; a wrapper whose
                # C++ side is already gone cannot be destroyed again.
                done = True
            if not done:
                keep.append(w)
        self._alive = keep

    def pending_count(self) -> int:
        """How many retired workers are still being held (tests/diagnostics)."""
        return len(self._alive)


_reaper: _Reaper | None = None


def _get_reaper() -> _Reaper:
    # Created lazily, from a GUI-thread slot, so the QObject's thread affinity
    # is the GUI thread (and so there is no QObject at import time, before the
    # QApplication exists).
    global _reaper
    if _reaper is None:
        _reaper = _Reaper()
    return _reaper


def retire_worker(worker: Any) -> None:
    """Hand ``worker`` over to be released once its thread has exited.

    Call this **immediately before** clearing the attribute that holds it::

        retire_worker(self._worker)
        self._worker = None

    Safe to call with ``None``, with an already-finished worker, or twice with
    the same worker.
    """
    if worker is None:
        return
    try:
        _get_reaper().keep(worker)
    except Exception:                                      # pragma: no cover
        # Never let bookkeeping break a completion handler. Leaking the
        # reference is the SAFE failure: the worker simply outlives its page.
        logger.debug("retire_worker failed", exc_info=True)


def pending_worker_count() -> int:
    """Retired workers still held (tests/diagnostics)."""
    return _reaper.pending_count() if _reaper is not None else 0


def _reset_for_tests() -> None:
    """Drop the module singleton (tests only)."""
    global _reaper
    _reaper = None

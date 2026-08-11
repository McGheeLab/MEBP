"""GUI-thread stall watchdog — turns "the app froze" into a stack trace.

WHY THIS EXISTS
---------------
``faulthandler`` (enabled in ``main.py``) catches a hard crash: a segfault or a
fatal error prints every thread's traceback. It catches NOTHING when the
process is alive but the Qt event loop has stopped turning — which is what an
operator actually reports as "python freezes". A hang leaves no trail at all,
so diagnosing one means reproducing it, and a freeze that only happens on the
rig (a specific camera, a specific driver, a specific timing) may not reproduce
anywhere else.

This closes that gap. A QTimer on the GUI thread stamps a heartbeat; a daemon
thread watches the stamp, and when it goes stale past ``stall_s`` it dumps EVERY
thread's Python stack to ``logs/freeze.log``. The wedged GUI thread is in that
dump, with the exact frame it is stuck in.

WHAT IT DELIBERATELY DOES NOT DO
--------------------------------
It does not interrupt, kill or recover anything. A watchdog that tries to break
a stuck thread out of a blocking SDK call is far more dangerous than the hang
(this application is holding serial ports open to a stage carrying a needle over
glass). It only observes and records.

It also cannot see a stall inside C code that never releases the GIL — the
watcher thread would not run either. In that case the dump is simply absent,
which is itself informative: the freeze is below Python.

COST
----
One 250 ms QTimer (a timestamp assignment) and one thread waking each second.
Both are cheap enough to leave on permanently, which matters — a diagnostic you
have to remember to enable is one you will not have enabled when it matters.
"""

from __future__ import annotations

import faulthandler
import logging
import os
import threading
import time
from typing import Optional

logger = logging.getLogger(__name__)

#: How long the event loop may stop turning before it counts as a stall. A
#: blocking read or a big relayout can legitimately take a second or two; five
#: seconds is past anything this application does on the GUI thread by design.
DEFAULT_STALL_S = 5.0

#: How often the heartbeat is stamped. Must be well under DEFAULT_STALL_S so a
#: stall is detected promptly, and cheap enough to be invisible.
BEAT_MS = 250


class GuiThreadWatchdog:
    """Watches the Qt event loop and dumps all stacks when it stops turning."""

    def __init__(self, stall_s: float = DEFAULT_STALL_S,
                 path: str = os.path.join("logs", "freeze.log")):
        self.stall_s = float(stall_s)
        self.path = path
        self._beat = time.monotonic()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._timer = None
        self._dumped_at: Optional[float] = None

    # ── GUI side ──────────────────────────────────────────────────

    def _stamp(self) -> None:
        self._beat = time.monotonic()

    def start(self) -> bool:
        """Install the heartbeat timer + watcher thread. Call AFTER the
        QApplication exists (the timer must live on the GUI thread)."""
        try:
            from PySide6.QtCore import QTimer
        except Exception as exc:                      # headless / no Qt
            logger.debug(f"GUI watchdog not started (no Qt): {exc}")
            return False
        if self._thread is not None:
            return True
        try:
            self._timer = QTimer()
            self._timer.setInterval(BEAT_MS)
            self._timer.timeout.connect(self._stamp)
            self._timer.start()
        except Exception as exc:
            logger.debug(f"GUI watchdog timer failed: {exc}")
            return False
        self._beat = time.monotonic()
        self._thread = threading.Thread(
            target=self._watch, name="GuiWatchdog", daemon=True)
        self._thread.start()
        logger.info(
            f"GUI watchdog armed — a stall over {self.stall_s:.0f}s dumps every "
            f"thread's stack to {self.path}")
        return True

    def stop(self) -> None:
        self._stop.set()
        try:
            if self._timer is not None:
                self._timer.stop()
        except Exception:
            pass

    # ── Watcher side ──────────────────────────────────────────────

    def _watch(self) -> None:
        while not self._stop.wait(1.0):
            self._check_once()

    def _check_once(self) -> bool:
        """One watch cycle. Returns True if this cycle dumped.

        Split out from the loop so the decision can be tested directly — a
        diagnostic that only runs during a freeze is one you cannot check by
        using the application.
        """
        try:
            idle = time.monotonic() - self._beat
            if idle > self.stall_s:
                # One dump per stall, not one per second — a 60-second freeze
                # must not produce sixty tracebacks to read through.
                if self._dumped_at is None:
                    self._dumped_at = time.monotonic()
                    self._dump(idle)
                    return True
            elif self._dumped_at is not None:
                stalled = time.monotonic() - self._dumped_at
                logger.warning(
                    f"GUI thread recovered after ~{stalled + self.stall_s:.1f}s "
                    f"— stacks captured in {self.path}")
                self._dumped_at = None
        except Exception:
            pass                                      # never kill the watchdog
        return False

    def _dump(self, idle_s: float) -> None:
        stamp = time.strftime("%Y-%m-%d %H:%M:%S")
        logger.error(
            f"GUI thread has not run for {idle_s:.1f}s — dumping all thread "
            f"stacks to {self.path}")
        try:
            os.makedirs(os.path.dirname(self.path) or ".", exist_ok=True)
            # Opened per dump and closed straight after: this file is what the
            # operator sends after a freeze, and a buffered handle held open by
            # a wedged process may never be flushed.
            with open(self.path, "a", encoding="utf-8") as f:
                f.write(f"\n{'=' * 70}\n")
                f.write(f"GUI THREAD STALL — {stamp} — idle {idle_s:.1f}s\n")
                f.write(f"{'=' * 70}\n")
                f.flush()
                faulthandler.dump_traceback(file=f, all_threads=True)
                f.flush()
        except Exception as exc:
            logger.error(f"GUI watchdog: could not write {self.path}: {exc}")


_WATCHDOG: Optional[GuiThreadWatchdog] = None


def start_watchdog(stall_s: float = DEFAULT_STALL_S) -> Optional[GuiThreadWatchdog]:
    """Start (once) the process-wide GUI-thread watchdog."""
    global _WATCHDOG
    if _WATCHDOG is None:
        wd = GuiThreadWatchdog(stall_s=stall_s)
        if wd.start():
            _WATCHDOG = wd
    return _WATCHDOG


def stop_watchdog() -> None:
    global _WATCHDOG
    if _WATCHDOG is not None:
        _WATCHDOG.stop()
        _WATCHDOG = None

"""v7.16 — the GUI-thread stall watchdog.

An operator reports "python freezes". ``faulthandler`` catches a CRASH and is
blind to a HANG, so a freeze that only happens on the rig used to leave nothing
behind at all. These tests pin the watchdog's decisions, because a diagnostic
that only runs during a freeze cannot be checked by using the application.
"""

import os
import re
import sys
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from SupportClasses.GuiWatchdog import GuiThreadWatchdog     # noqa: E402


class TestItDumpsWhenTheEventLoopStops(unittest.TestCase):

    def setUp(self):
        import tempfile
        self._tmp = tempfile.TemporaryDirectory()
        self.path = os.path.join(self._tmp.name, "freeze.log")
        self.wd = GuiThreadWatchdog(stall_s=0.2, path=self.path)

    def tearDown(self):
        self._tmp.cleanup()

    def test_a_beating_heart_dumps_nothing(self):
        self.wd._stamp()
        self.assertFalse(self.wd._check_once())
        self.assertFalse(os.path.exists(self.path))

    def test_a_stalled_heart_dumps_every_thread(self):
        """all_threads is the whole point: the watcher's own stack is useless —
        the one that matters is the wedged thread, which is a DIFFERENT thread."""
        import threading
        running = threading.Event()
        release = threading.Event()

        def _a_thread_stuck_somewhere():
            running.set()
            release.wait(5.0)

        t = threading.Thread(target=_a_thread_stuck_somewhere,
                             name="PretendGuiThread", daemon=True)
        t.start()
        self.assertTrue(running.wait(2.0))
        try:
            self.wd._beat = time.monotonic() - 5.0
            self.assertTrue(self.wd._check_once())
        finally:
            release.set()
            t.join(2.0)
        text = Path(self.path).read_text(encoding="utf-8", errors="replace")
        self.assertIn("GUI THREAD STALL", text)
        self.assertIn("idle 5.0s", text)
        # The OTHER thread's frame — proof the dump is not just the watcher's.
        self.assertIn("_a_thread_stuck_somewhere", text)

    def test_one_dump_per_stall_not_one_per_second(self):
        """A sixty-second freeze must not produce sixty tracebacks to read."""
        self.wd._beat = time.monotonic() - 5.0
        self.assertTrue(self.wd._check_once())
        for _ in range(10):
            self.assertFalse(self.wd._check_once())
        text = Path(self.path).read_text(encoding="utf-8", errors="replace")
        self.assertEqual(len(re.findall("GUI THREAD STALL", text)), 1)

    def test_recovery_rearms_it_for_the_next_stall(self):
        self.wd._beat = time.monotonic() - 5.0
        self.wd._check_once()
        self.wd._stamp()                      # event loop turns again
        self.wd._check_once()
        self.assertIsNone(self.wd._dumped_at)
        self.wd._beat = time.monotonic() - 5.0
        self.assertTrue(self.wd._check_once())
        text = Path(self.path).read_text(encoding="utf-8", errors="replace")
        self.assertEqual(len(re.findall("GUI THREAD STALL", text)), 2)

    def test_it_appends_so_earlier_freezes_survive(self):
        """The operator sends this file after the fact — an overwriting log
        would keep only the last freeze of the session."""
        for _ in range(2):
            self.wd._beat = time.monotonic() - 5.0
            self.wd._check_once()
            self.wd._stamp()
            self.wd._check_once()
        text = Path(self.path).read_text(encoding="utf-8", errors="replace")
        self.assertEqual(len(re.findall("GUI THREAD STALL", text)), 2)

    def test_an_unwritable_path_never_raises(self):
        """It watches a wedged application; it must not become a second fault."""
        wd = GuiThreadWatchdog(stall_s=0.1,
                               path=os.path.join(self._tmp.name, "x", "\0bad"))
        wd._beat = time.monotonic() - 5.0
        wd._check_once()                       # must not raise

    def test_it_never_tries_to_interrupt_the_stuck_thread(self):
        """Deliberate: this process holds serial ports open to a stage carrying
        a needle over glass. Breaking a thread out of a blocking driver call is
        far more dangerous than the hang. Observation only."""
        import inspect
        src = inspect.getsource(GuiThreadWatchdog)
        for weapon in ("_thread.interrupt_main", "os._exit", "os.kill",
                       "signal.pthread_kill", "ctypes"):
            self.assertNotIn(weapon, src)


class TestItIsCheapEnoughToLeaveOn(unittest.TestCase):

    def test_the_heartbeat_is_only_a_timestamp(self):
        wd = GuiThreadWatchdog()
        t0 = time.perf_counter()
        for _ in range(10000):
            wd._stamp()
        per_beat_us = (time.perf_counter() - t0) / 10000 * 1e6
        self.assertLess(per_beat_us, 50.0)

    def test_the_beat_interval_is_well_under_the_stall_threshold(self):
        """Otherwise a healthy application would look stalled."""
        from SupportClasses.GuiWatchdog import BEAT_MS, DEFAULT_STALL_S
        self.assertLess(BEAT_MS / 1000.0, DEFAULT_STALL_S / 4.0)


if __name__ == "__main__":
    unittest.main()

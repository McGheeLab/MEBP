"""test_v76_hard_abort.py — the v7.6 hard abort: kill ALL motion with bounded
latency, then retract the needle raise-only.

Before v7.6, `PrintManager.abort()` set flags only: XY kept moving to its
commanded point, queued Z/pump moves drained to completion, the "safety" Z move
was NOT raise-only (it could descend), and it was issued on the CALLING thread,
so an abort during an in-flight M400 froze the GUI for 10–180 s.

No hardware; fake serial + fake stages throughout.
"""

import threading
import time
import unittest
from unittest.mock import MagicMock

from SupportClasses.ZPStage import ZPStageManager


# ── Fakes ─────────────────────────────────────────────────────────────

class _SilentSerial:
    """A serial port that accepts writes and never answers — reproduces the
    'board busy/silent during an M400' case that used to block for 10–180 s."""

    def __init__(self):
        self.writes = []
        self.is_open = True

    def write(self, data):
        self.writes.append(bytes(data))
        return len(data)

    def flush(self):
        pass

    def reset_input_buffer(self):
        pass

    def reset_output_buffer(self):
        pass

    def readline(self):
        time.sleep(0.02)        # models the read timeout
        return b""

    def close(self):
        self.is_open = False


def _zp_with_serial(ser):
    """A ZPStageManager wired to a fake port without touching real hardware."""
    zp = ZPStageManager.__new__(ZPStageManager)
    zp.serial = ser
    zp.simulate = False
    zp._serial_lock = threading.RLock()
    zp._last_position_read_ok = True
    zp.emergency_parser = True
    zp.x_pos = zp.y_pos = zp.z_pos = zp.e_pos = 0.0
    return zp


class TestFlushMovesAbortAware(unittest.TestCase):
    def test_returns_promptly_when_abort_event_set(self):
        zp = _zp_with_serial(_SilentSerial())
        ev = threading.Event()
        ev.set()
        t0 = time.monotonic()
        ok = zp.flush_moves(timeout_s=30.0, abort_event=ev)
        elapsed = time.monotonic() - t0
        self.assertFalse(ok)
        self.assertLess(elapsed, 0.5, "abort must not wait out the timeout")

    def test_abort_set_mid_wait(self):
        zp = _zp_with_serial(_SilentSerial())
        ev = threading.Event()
        threading.Timer(0.15, ev.set).start()
        t0 = time.monotonic()
        ok = zp.flush_moves(timeout_s=30.0, abort_event=ev)
        elapsed = time.monotonic() - t0
        self.assertFalse(ok)
        self.assertLess(elapsed, 1.0)

    def test_without_event_still_waits_out_the_timeout(self):
        """Legacy behaviour is byte-identical when no event is passed."""
        zp = _zp_with_serial(_SilentSerial())
        t0 = time.monotonic()
        ok = zp.flush_moves(timeout_s=0.4)
        elapsed = time.monotonic() - t0
        self.assertFalse(ok)
        self.assertGreaterEqual(elapsed, 0.35)


class TestQuickstop(unittest.TestCase):
    def test_uncontended_writes_m410_under_lock(self):
        ser = _SilentSerial()
        zp = _zp_with_serial(ser)
        self.assertTrue(zp.quickstop())
        self.assertEqual(ser.writes, [b"M410\n"])

    def test_contended_raw_writes_without_deadlocking(self):
        """The whole point: an in-flight M400 holds `_serial_lock` for up to
        180 s — quickstop must still get M410 out, bounded."""
        ser = _SilentSerial()
        zp = _zp_with_serial(ser)
        holder_ready = threading.Event()
        release = threading.Event()

        def _hold():
            with zp._serial_lock:
                holder_ready.set()
                release.wait(5.0)

        h = threading.Thread(target=_hold, daemon=True)
        h.start()
        holder_ready.wait(2.0)
        t0 = time.monotonic()
        ok = zp.quickstop(lock_timeout_s=0.2)
        elapsed = time.monotonic() - t0
        release.set()
        h.join(2.0)
        self.assertTrue(ok)
        self.assertLess(elapsed, 1.0)
        # leading newline terminates any partial line on the wire
        self.assertEqual(ser.writes, [b"\nM410\n"])

    def test_simulated_and_disconnected_are_noops(self):
        zp = _zp_with_serial(_SilentSerial())
        zp.simulate = True
        self.assertTrue(zp.quickstop())
        zp.simulate = False
        zp.serial = None
        self.assertTrue(zp.quickstop())

    def test_write_failure_reported(self):
        class Boom(_SilentSerial):
            def write(self, data):
                raise OSError("port gone")
        zp = _zp_with_serial(Boom())
        self.assertFalse(zp.quickstop())

    def test_resync_reads_twice(self):
        zp = _zp_with_serial(_SilentSerial())
        calls = []
        zp.get_current_position = lambda: (calls.append(1), (1.0, 2.0, 3.0, 4.0))[1]
        out = zp.resync_position()
        self.assertEqual(len(calls), 2)     # stray-ok absorber + clean read
        self.assertEqual(out, (1.0, 2.0, 3.0, 4.0))

    def test_resync_survives_read_errors(self):
        zp = _zp_with_serial(_SilentSerial())

        def _boom():
            raise OSError("nope")
        zp.get_current_position = _boom
        self.assertEqual(zp.resync_position(), (0.0, 0.0, 0.0, 0.0))


class TestAbortAllMotion(unittest.TestCase):
    def _ctrl(self):
        from SupportClasses.StageController import StageController
        c = StageController.__new__(StageController)
        c.xy_stage = MagicMock()
        c.zp_stage = MagicMock()
        c.zp_stage.quickstop.return_value = True
        c.zp_stage.emergency_parser = True
        return c

    def test_order_and_result(self):
        c = self._ctrl()
        order = []
        c.xy_stage.stop_stage.side_effect = lambda: order.append("stop_stage")
        c.xy_stage.move_stage_at_velocity.side_effect = \
            lambda vx, vy: order.append(("vs", vx, vy))
        c.zp_stage.quickstop.side_effect = lambda: (order.append("quickstop"),
                                                    True)[1]
        c.zp_stage.resync_position.side_effect = lambda: order.append("resync")
        res = c.abort_all_motion("unit-test")
        self.assertEqual(order, ["stop_stage", ("vs", 0.0, 0.0),
                                 "quickstop", "resync"])
        self.assertTrue(res["xy_stopped"])
        self.assertTrue(res["vs_zeroed"])
        self.assertTrue(res["zp_quickstop"])
        self.assertTrue(res["resync_ok"])
        self.assertEqual(res["reason"], "unit-test")

    def test_never_raises_when_everything_fails(self):
        c = self._ctrl()
        c.xy_stage.stop_stage.side_effect = OSError("x")
        c.xy_stage.move_stage_at_velocity.side_effect = OSError("y")
        c.zp_stage.quickstop.side_effect = OSError("z")
        res = c.abort_all_motion("broken")      # must not raise
        self.assertFalse(res["xy_stopped"])
        self.assertFalse(res["zp_quickstop"])

    def test_zp_only_and_xy_only_rigs(self):
        from SupportClasses.StageController import StageController
        c = StageController.__new__(StageController)
        c.xy_stage = None
        c.zp_stage = MagicMock()
        c.zp_stage.quickstop.return_value = True
        self.assertTrue(c.abort_all_motion("zp-only")["zp_quickstop"])
        c2 = StageController.__new__(StageController)
        c2.xy_stage = MagicMock()
        c2.zp_stage = None
        self.assertTrue(c2.abort_all_motion("xy-only")["xy_stopped"])


class TestPrintManagerAbort(unittest.TestCase):
    def _pm(self, *, thread=None, travel_z=5.0):
        from SupportClasses.PrintManager import (
            PrintManager, PrintJob, PrintSettings, PrintState)
        pm = PrintManager.__new__(PrintManager)
        pm.state = PrintState.RUNNING
        pm._abort_flag = threading.Event()
        pm._pause_event = threading.Event()
        pm._trajectory_executor = None
        pm.exec_logger = None
        pm._current_step = 3
        pm._active_pump = "P1"
        pm.on_state_changed = None
        pm._thread = thread
        pm.controller = MagicMock()
        pm.controller.is_zp_connected = True
        pm.controller.abort_all_motion.return_value = {"zp_quickstop": True}
        settings = PrintSettings()
        settings.travel_z_height = travel_z
        pm.job = PrintJob(name="t", commands=[], settings=settings)
        pm._stop_recorder = lambda *_a, **_k: None
        pm._record_history = lambda *_a, **_k: None
        return pm

    def test_abort_returns_immediately_and_kills_motion(self):
        from SupportClasses.PrintManager import PrintState
        # a print thread parked in a blocking wait, like a real M400
        parked = threading.Event()

        def _parked():
            parked.wait(5.0)
        t = threading.Thread(target=_parked, daemon=True)
        t.start()
        pm = self._pm(thread=t)
        t0 = time.monotonic()
        pm.abort()
        elapsed = time.monotonic() - t0
        self.assertLess(elapsed, 0.2, "abort() must not block the caller")
        self.assertEqual(pm.state, PrintState.ABORTED)
        self.assertTrue(pm._abort_flag.is_set())
        # the worker kills motion asynchronously
        for _ in range(50):
            if pm.controller.abort_all_motion.called:
                break
            time.sleep(0.02)
        pm.controller.abort_all_motion.assert_called_once()
        parked.set()
        t.join(2.0)

    def test_no_descending_z_move_on_abort(self):
        """Regression: the old abort issued move_z_absolute(travel_z) on the
        calling thread, which DESCENDED when the needle was already above the
        travel height. The retract of record is now the print thread's finally
        (raise-only ensure_retracted_to)."""
        pm = self._pm(thread=None)
        pm.abort()
        for _ in range(50):
            if pm.controller.abort_all_motion.called:
                break
            time.sleep(0.02)
        pm.controller.move_z_absolute.assert_not_called()

    def test_backstop_retract_when_thread_never_unwinds(self):
        stuck = threading.Event()

        def _stuck():
            stuck.wait(10.0)
        t = threading.Thread(target=_stuck, daemon=True)
        t.start()
        pm = self._pm(thread=t)
        pm._ABORT_UNWIND_S = 0.2          # keep the test fast
        calls = []
        pm._retract_to_safe_z = lambda ctx=None: calls.append(ctx)
        pm.abort()
        for _ in range(100):
            if calls:
                break
            time.sleep(0.02)
        self.assertEqual(calls, ["abort_backstop"])
        stuck.set()
        t.join(2.0)

    def test_no_backstop_when_thread_unwinds(self):
        t = threading.Thread(target=lambda: None, daemon=True)
        t.start()
        t.join()
        pm = self._pm(thread=t)
        pm._ABORT_UNWIND_S = 0.5
        calls = []
        pm._retract_to_safe_z = lambda ctx=None: calls.append(ctx)
        pm.abort()
        time.sleep(0.4)
        self.assertEqual(calls, [], "finally does the retract, not the worker")

    def test_abort_ignored_when_not_running(self):
        from SupportClasses.PrintManager import PrintState
        pm = self._pm()
        pm.state = PrintState.COMPLETED
        pm.abort()
        pm.controller.abort_all_motion.assert_not_called()


class TestAbortAwareWaits(unittest.TestCase):
    def _ctrl(self):
        from SupportClasses.StageController import StageController
        c = StageController.__new__(StageController)
        c.zp_stage = MagicMock()
        c.zero_position = {"Z": 0.0}
        c._pos_poller = MagicMock()
        return c

    def test_wait_for_z_arrival_early_out(self):
        c = self._ctrl()
        # is_zp_connected is a read-only property → satisfy it via the stage
        # (simulate=True short-circuits it to True)
        c.zp_stage.simulate = True
        c.get_zp_position = lambda cached=True: (0.0, 0.0, 99.0, 0.0)
        ev = threading.Event()
        ev.set()
        t0 = time.monotonic()
        ok = c.wait_for_z_arrival(0.0, timeout_s=30.0, abort_event=ev)
        self.assertFalse(ok)
        self.assertLess(time.monotonic() - t0, 0.5)

    def test_wait_pump_move_complete_forwards_event(self):
        c = self._ctrl()
        seen = {}

        def _flush(timeout_s=15.0, abort_event=None):
            seen["ev"] = abort_event
            return False
        c.zp_stage.flush_moves = _flush
        c.suspend_position_poller = lambda: None
        c.resume_position_poller = lambda: None
        ev = threading.Event()
        c._wait_pump_move_complete(1.0, abort_event=ev)
        self.assertIs(seen["ev"], ev)

    def test_wait_pump_move_complete_tolerates_old_flush(self):
        c = self._ctrl()
        c.zp_stage.flush_moves = lambda timeout_s=15.0: True   # no kwarg
        c.suspend_position_poller = lambda: None
        c.resume_position_poller = lambda: None
        self.assertTrue(c._wait_pump_move_complete(
            1.0, abort_event=threading.Event()))

    def test_finish_pump_submove_skips_dwell_when_aborting(self):
        c = self._ctrl()
        ev = threading.Event()
        ev.set()
        slept = []
        from unittest.mock import patch
        with patch("SupportClasses.StageController.time.sleep",
                   side_effect=lambda s: slept.append(s)):
            c._finish_pump_submove(1.0, 1.0, block=True, dwell_s=5.0,
                                   abort_event=ev)
        self.assertEqual(slept, [], "no drain wait, no settle dwell on abort")


class TestSafeTravelAbort(unittest.TestCase):
    def test_no_motion_when_already_aborting(self):
        from SupportClasses.StageController import StageController
        c = StageController.__new__(StageController)
        c.xy_stage = MagicMock()
        c.zp_stage = MagicMock()
        c._pos_poller = MagicMock()
        c._min_travel_z_mm = None
        ev = threading.Event()
        ev.set()
        ok = c.safe_travel_to(1000.0, 1000.0, safe_z_mm=5.0, abort_event=ev)
        self.assertFalse(ok)
        c.xy_stage.assert_not_called()


class TestQuickPrintAbortRoutes(unittest.TestCase):
    """All three abort routes must kill physical motion, not just set flags."""

    def _page(self):
        from gui.pages.workflows.quick_print_workflow import (
            QuickPrintWorkflowPage)
        page = QuickPrintWorkflowPage.__new__(QuickPrintWorkflowPage)
        page._controller = MagicMock()
        page._status = MagicMock()
        page._multi_thread = None
        page._active_executor = None
        page._pm = None
        page._multi_abort_requested = False
        page._preflight_abort_requested = False
        page._update_button_state = lambda: None
        return page

    def _wait_called(self, page):
        for _ in range(100):
            if page._controller.abort_all_motion.called:
                return True
            time.sleep(0.02)
        return False

    def test_plain_print_route(self):
        page = self._page()
        page._pm = MagicMock()
        page._on_abort()
        self.assertTrue(self._wait_called(page))
        page._pm.abort.assert_called_once()

    def test_preflight_route(self):
        page = self._page()
        ex = MagicMock()
        ex._abort_flag = threading.Event()
        page._active_executor = ex
        page._on_abort()
        self.assertTrue(self._wait_called(page))
        self.assertTrue(ex._abort_flag.is_set())
        self.assertTrue(page._preflight_abort_requested)

    def test_multi_ink_route(self):
        page = self._page()
        t = threading.Thread(target=lambda: time.sleep(0.5), daemon=True)
        t.start()
        page._multi_thread = t
        ex = MagicMock()
        ex._abort_flag = threading.Event()
        page._active_executor = ex
        page._pm = MagicMock()
        page._on_abort()
        self.assertTrue(self._wait_called(page))
        self.assertTrue(page._multi_abort_requested)
        self.assertTrue(ex._abort_flag.is_set())
        t.join(2.0)

    def test_no_controller_support_is_harmless(self):
        page = self._page()
        page._controller = object()       # no abort_all_motion attribute
        page._pm = MagicMock()
        page._on_abort()                  # must not raise
        page._pm.abort.assert_called_once()


class TestPickPlaceExecutorAbortWiring(unittest.TestCase):
    def test_pump_move_forwards_the_flag(self):
        from SupportClasses.PickAndPlaceManager import PickPlaceExecutor
        ex = PickPlaceExecutor.__new__(PickPlaceExecutor)
        ex._abort_flag = threading.Event()
        ex.controller = MagicMock()
        seen = {}

        def _move(pump, vol, rate_uL_s=None, **kw):
            seen.update(kw)
        ex.controller.move_pump_uL = _move
        ex._pump_move("P1", -5.0, 1.0)
        self.assertIs(seen.get("abort_event"), ex._abort_flag)

    def test_safe_travel_forwards_and_degrades(self):
        from SupportClasses.PickAndPlaceManager import PickPlaceExecutor
        ex = PickPlaceExecutor.__new__(PickPlaceExecutor)
        ex._abort_flag = threading.Event()
        ex.controller = MagicMock()
        seen = {}
        ex.controller.safe_travel_to = lambda **kw: seen.update(kw) or True
        ex._safe_travel(target_x_um=1.0, target_y_um=2.0, safe_z_mm=5.0)
        self.assertIs(seen.get("abort_event"), ex._abort_flag)

        # older controller without the kwarg → falls back cleanly
        def _old(**kw):
            if "abort_event" in kw:
                raise TypeError("unexpected keyword")
            return True
        ex.controller.safe_travel_to = _old
        self.assertTrue(ex._safe_travel(target_x_um=1.0, target_y_um=2.0,
                                        safe_z_mm=5.0))


if __name__ == "__main__":
    unittest.main()

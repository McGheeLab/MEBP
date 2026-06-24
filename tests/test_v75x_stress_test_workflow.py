"""test_v75x_stress_test_workflow.py — ZP Stress Test workflow + comm telemetry.

The Stress Test workflow loops the command patterns that historically dropped
the ZP board (retract→XY travel hops, dense Z/pump jog bursts, real prints) and
reports the flow-control telemetry, so the operator can bench-validate the
synchronous `ok` fix on the real ME3B V1 board. A run PASSES only with zero
disconnects, zero board resets, and zero un-acked commands.

Covers: page build + registration, the safety gate (no Safe Z → refuse), the
stress primitives drive the controller through the safe APIs (travel never
lowers the needle; jog retracts first), the PASS/FAIL verdict, and the
ZPStageManager comm counters that feed the report.
"""

import sys
import threading
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from PySide6.QtWidgets import QApplication

from SupportClasses.ZPStage import ZPStageManager
from SupportClasses.PrintManager import CommandType
from gui.pages.workflows.stress_test_workflow import (
    StressTestWorkflowPage, _StressConfig,
)


class _FakePM:
    """Records the built job without spawning the real executor thread."""

    instances: list = []

    def __init__(self, controller):
        self.controller = controller
        self.job = None
        self.state = None
        self.exec_logger = None
        self.on_progress = None
        self.on_state_changed = None
        _FakePM.instances.append(self)

    def load_job(self, job):
        self.job = job

    def start(self):
        pass

    def abort(self):
        pass


def _cfg(**kw):
    base = dict(cycles=2, do_travel=False, do_jog=False, do_print=False,
               travel_hops=3, jog_moves=4, jog_amp_mm=1.0,
               prints_per_cycle=1, print_flow_uL_s=0.0,
               print_descend=True, print_height_mm=0.5, jog_pumps=False,
               do_xy=False, xy_amp_um=500.0,
               do_descend=False, descend_depth_mm=5.0, do_dispense=False,
               component_reps=3)
    base.update(kw)
    return _StressConfig(**base)


class _FakeZP:
    def __init__(self, sticky=False):
        self.c = {"cmd": 12, "ok": 12, "ok_fail": 0, "reset": 0}
        # sticky=True models failures that accrue *during* the run (a real
        # board's counters climb as commands fail), so the start-of-run
        # reset_comm_counters() doesn't wipe the scenario.
        self.sticky = sticky

    def get_comm_counters(self):
        return dict(self.c)

    def reset_comm_counters(self):
        if not self.sticky:
            self.c = {"cmd": 0, "ok": 0, "ok_fail": 0, "reset": 0}

    def flush_moves(self, timeout_s=10.0):
        return True


class _FakeCtrl:
    def __init__(self):
        self.zp_stage = _FakeZP()
        self.is_xy_connected = True
        self.is_zp_connected = True
        self.zero_position = {"x": 0.0, "y": 0.0, "Z": 0.0}
        self.safety_limits = SimpleNamespace(
            xy_min_x=0.0, xy_min_y=0.0, xy_max_x=100000.0, xy_max_y=80000.0)
        self.travel_calls = []
        self.retracts = []
        self.zjogs = []
        self.xy_moves = []
        self.z_abs_moves = []
        self.pump_uL_calls = []
        self._xy_lock = threading.Lock()
        self.drop_after_travel = False

    def default_plate_center_um(self):
        return (50000.0, 40000.0)

    # Z convention helpers (z_up_sign = +1 stand-in: height == zero-ref).
    def zref_to_user_z(self, z):
        return z

    def user_z_to_zref(self, u):
        return u

    def move_z_absolute(self, z, from_zero_ref=True, **k):
        self.z_abs_moves.append(z)

    def wait_for_z_arrival(self, z, timeout_s=10.0, **k):
        return True

    def move_pump_uL(self, pump, uL, rate, **k):
        self.pump_uL_calls.append((pump, uL, rate))

    def suspend_position_poller(self):
        pass

    def resume_position_poller(self):
        pass

    def safe_travel_to(self, x, y, safe_z_mm=None, target_z_mm="UNSET",
                       **kw):
        self.travel_calls.append((x, y, safe_z_mm, target_z_mm))
        if self.drop_after_travel:
            self.is_zp_connected = False
        return True

    def ensure_retracted_to(self, z, *a, **k):
        self.retracts.append(z)
        return True

    def move_z_user_relative(self, d, *a, **k):
        self.zjogs.append(d)

    def move_pump_relative(self, p, d, *a, **k):
        pass

    def move_xy_relative_um(self, dx, dy, *a, **k):
        with self._xy_lock:
            self.xy_moves.append((dx, dy))

    def print_z_dir(self):
        return -1.0

    def print_height_to_zref(self, h):
        return None


class _Base(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls._app = QApplication.instance() or QApplication(sys.argv)

    def _page(self, ctrl=None):
        page = StressTestWorkflowPage(ctrl or _FakeCtrl(), settings=None)
        page._safe_z = 5.0
        return page


class TestRegistrationAndBuild(_Base):
    def test_registered_enabled_in_picker(self):
        from gui.pages.workflows.workflow_picker import WORKFLOWS
        tile = next((t for t in WORKFLOWS if t.workflow_id == "stress_test"),
                    None)
        self.assertIsNotNone(tile)
        self.assertTrue(tile.enabled)

    def test_page_builds_and_titles(self):
        page = self._page()
        self.assertEqual(page.get_page_title(), "ZP Stress Test")
        self.assertEqual(page.get_sub_page_title(), "ZP Stress Test")


class TestSafetyGate(_Base):
    def test_refuses_without_safe_z(self):
        ctrl = _FakeCtrl()
        page = self._page(ctrl)
        page._safe_z = None
        page._on_start()
        self.assertIsNone(page._thread)
        self.assertIn("safe z", page._status.text().lower())

    def test_refuses_when_disconnected(self):
        ctrl = _FakeCtrl()
        ctrl.is_zp_connected = False
        page = self._page(ctrl)
        page._on_start()
        self.assertIsNone(page._thread)

    def test_refuses_with_no_mode_selected(self):
        ctrl = _FakeCtrl()
        page = self._page(ctrl)
        page._chk_travel.setChecked(False)
        page._chk_jog.setChecked(False)
        page._chk_print.setChecked(False)
        page._chk_xy.setChecked(False)
        page._on_start()
        self.assertIsNone(page._thread)
        self.assertIn("at least one", page._status.text().lower())


class TestStressPrimitives(_Base):
    def test_travel_burst_never_lowers_and_clamps(self):
        ctrl = _FakeCtrl()
        page = self._page(ctrl)
        page._travel_burst(_cfg(travel_hops=4), lambda: True)
        self.assertEqual(len(ctrl.travel_calls), 4)
        for x, y, safe_z, target_z in ctrl.travel_calls:
            self.assertIsNone(target_z)            # never lower the needle
            self.assertEqual(safe_z, 5.0)
            # clamped inside the envelope (with margin)
            self.assertGreaterEqual(x, 2000.0)
            self.assertLessEqual(x, 100000.0 - 2000.0)
            self.assertGreaterEqual(y, 2000.0)
            self.assertLessEqual(y, 80000.0 - 2000.0)

    def test_jog_burst_retracts_first_then_oscillates(self):
        ctrl = _FakeCtrl()
        page = self._page(ctrl)
        page._jog_burst(_cfg(do_jog=True, jog_moves=5, jog_amp_mm=1.0),
                        lambda: True)
        self.assertEqual(ctrl.retracts[0], 5.0)        # retract to Safe Z first
        # 5 moves × (down + up) height-frame jogs, net zero per pair.
        self.assertEqual(len(ctrl.zjogs), 10)
        self.assertAlmostEqual(sum(ctrl.zjogs), 0.0)
        self.assertIn(-1.0, ctrl.zjogs)
        self.assertIn(1.0, ctrl.zjogs)

    def test_stop_halts_travel_burst(self):
        ctrl = _FakeCtrl()
        page = self._page(ctrl)
        page._stop.set()
        page._travel_burst(_cfg(travel_hops=10), lambda: True)
        self.assertEqual(len(ctrl.travel_calls), 0)

    def test_concurrent_xy_runs_during_jog(self):
        # do_xy → the XY (Prior) channel is driven concurrently with the ZP
        # jog: net-zero relative moves, and Z still oscillates.
        ctrl = _FakeCtrl()
        page = self._page(ctrl)
        page._jog_burst(_cfg(do_jog=True, do_xy=True, jog_moves=20,
                             xy_amp_um=500.0), lambda: True)
        self.assertEqual(len(ctrl.zjogs), 40)            # ZP still jogged
        self.assertGreater(len(ctrl.xy_moves), 0)        # XY moved concurrently
        # XY moves are net-zero (sum ≈ 0 since they cycle ±a in x and y).
        sx = sum(m[0] for m in ctrl.xy_moves)
        sy = sum(m[1] for m in ctrl.xy_moves)
        self.assertLessEqual(abs(sx), 500.0)
        self.assertLessEqual(abs(sy), 500.0)

    def test_xy_only_drives_xy_without_zp_jog(self):
        # do_xy without do_jog → XY still moves, ZP is not jogged.
        ctrl = _FakeCtrl()
        page = self._page(ctrl)
        page._jog_burst(_cfg(do_jog=False, do_xy=True, jog_moves=30),
                        lambda: True)
        self.assertEqual(len(ctrl.zjogs), 0)
        self.assertGreater(len(ctrl.xy_moves), 0)

    def test_descend_burst_goes_down_then_up_and_confirms(self):
        # Component isolation: descend Z (down from safe) then retract (up),
        # each confirmed — reproducing the print MOVE_Z in isolation.
        ctrl = _FakeCtrl()
        page = self._page(ctrl)         # safe_z = 5.0
        page._descend_burst(_cfg(do_descend=True, descend_depth_mm=2.0,
                                 component_reps=3), lambda: True)
        # Each rep: move to descent target (3.0 = 5.0 − 2.0) then back to 5.0.
        self.assertIn(3.0, ctrl.z_abs_moves)   # descended below safe
        self.assertIn(5.0, ctrl.z_abs_moves)   # retracted to safe
        self.assertEqual(len(ctrl.z_abs_moves), 6)  # 3 reps × (down + up)

    def test_dispense_burst_is_net_zero_pump(self):
        ctrl = _FakeCtrl()
        page = self._page(ctrl)
        page._dispense_burst(_cfg(do_dispense=True, component_reps=4),
                            lambda: True)
        self.assertEqual(len(ctrl.pump_uL_calls), 8)  # 4 reps × (+ / −)
        self.assertAlmostEqual(sum(c[1] for c in ctrl.pump_uL_calls), 0.0)


class TestPrintDescent(_Base):
    """The full print stress now descends Z into position (real print)."""

    def setUp(self):
        _FakePM.instances.clear()

    def _ctrl(self):
        ctrl = _FakeCtrl()
        ctrl.print_height_to_zref = lambda h: 18.0  # calibrated descent target
        return ctrl

    def _move_z_targets(self, job):
        return [c.params.get("z") for c in job.commands
                if c.type == CommandType.MOVE_Z]

    def test_print_descends_to_print_height(self):
        ctrl = self._ctrl()
        page = self._page(ctrl)            # safe_z = 5.0
        page._well_positions = {"A1": (8855.0, 10856.0)}
        with patch("gui.pages.workflows.stress_test_workflow.PrintManager",
                   _FakePM):
            page._print_burst(
                _cfg(do_print=True, print_descend=True, print_height_mm=0.5,
                     prints_per_cycle=1), lambda: True)
        self.assertTrue(_FakePM.instances)
        job = _FakePM.instances[-1].job
        # MOVE_Z descends to the real print height (18.0), NOT the safe Z (5.0).
        self.assertIn(18.0, self._move_z_targets(job))

    def test_print_no_descend_stays_at_safe_z(self):
        ctrl = self._ctrl()
        page = self._page(ctrl)
        page._well_positions = {"A1": (8855.0, 10856.0)}
        with patch("gui.pages.workflows.stress_test_workflow.PrintManager",
                   _FakePM):
            page._print_burst(
                _cfg(do_print=True, print_descend=False, prints_per_cycle=1),
                lambda: True)
        job = _FakePM.instances[-1].job
        # No descent → MOVE_Z stays at safe Z (5.0), nothing at 18.0.
        self.assertNotIn(18.0, self._move_z_targets(job))
        self.assertIn(5.0, self._move_z_targets(job))


class TestRunVerdict(_Base):
    def _run_sync(self, page, cfg):
        # _run uses queued bridge signals; same-thread emit delivers directly.
        page._run(cfg)
        self._app.processEvents()

    def test_clean_run_passes(self):
        ctrl = _FakeCtrl()
        page = self._page(ctrl)
        self._run_sync(page, _cfg(cycles=2, do_travel=True, travel_hops=2))
        self.assertIn("PASS", page._banner.text())
        # finally-block always retracts to Safe Z.
        self.assertIn(5.0, ctrl.retracts)

    def test_disconnect_during_run_fails(self):
        ctrl = _FakeCtrl()
        ctrl.drop_after_travel = True  # board "drops" on the first hop
        page = self._page(ctrl)
        self._run_sync(page, _cfg(cycles=3, do_travel=True, travel_hops=2))
        self.assertIn("FAIL", page._banner.text())
        self.assertIn("disconnect", page._banner.text().lower())

    def test_unacked_command_fails(self):
        ctrl = _FakeCtrl()
        ctrl.zp_stage = _FakeZP(sticky=True)
        ctrl.zp_stage.c = {"cmd": 100, "ok": 98, "ok_fail": 2, "reset": 0}
        page = self._page(ctrl)
        self._run_sync(page, _cfg(cycles=1, do_jog=True, jog_moves=2))
        self.assertIn("FAIL", page._banner.text())

    def test_safety_stop_halts_driving_on_sustained_ack_failures(self):
        # SAFETY: a live-but-failing link (ok_fail climbing, no disconnect) must
        # halt the run BEFORE the jog phase — so the Z motor isn't driven (and
        # overheated) on a degraded board.
        ctrl = _FakeCtrl()
        ctrl.zp_stage = _FakeZP(sticky=True)
        ctrl.zp_stage.c = {"cmd": 50, "ok": 45, "ok_fail": 5, "reset": 0}
        page = self._page(ctrl)
        self._run_sync(page, _cfg(cycles=5, do_travel=True, do_jog=True,
                                  jog_moves=100))
        self.assertIn("FAIL", page._banner.text())
        # Degraded link tripped at the first cycle gate → no Z jogging happened.
        self.assertEqual(len(ctrl.zjogs), 0)


# ── ZPStageManager comm telemetry (feeds the report) ─────────────────

class _ScriptedSerial:
    def __init__(self, loop_ok=True):
        self.is_open = True
        self.loop_ok = loop_ok
        self.writes = []

    def write(self, b):
        self.writes.append(b.decode("utf-8", errors="replace").strip())

    def flush(self):
        pass

    def readline(self):
        return b"ok\n" if self.loop_ok else b""

    def close(self):
        self.is_open = False


def _bare_zp(serial):
    zp = ZPStageManager.__new__(ZPStageManager)
    zp.serial = serial
    zp.simulate = False
    zp._serial_lock = threading.RLock()
    zp.feedrate = 1000.0
    zp._last_position_read_ok = True
    zp._board_reset_detected = False
    zp.cmd_count = zp.ok_count = zp.ok_fail_count = zp.reset_count = 0
    return zp


class TestCommCounters(unittest.TestCase):
    def test_counts_acked_commands(self):
        zp = _bare_zp(_ScriptedSerial(loop_ok=True))
        zp.send_data("G0 X1")
        zp.send_data("G0 X2")
        c = zp.get_comm_counters()
        self.assertEqual(c["cmd"], 2)
        self.assertEqual(c["ok"], 2)
        self.assertEqual(c["ok_fail"], 0)

    def test_counts_unacked_on_silence(self):
        zp = _bare_zp(_ScriptedSerial(loop_ok=False))
        zp.DEFAULT_OK_TIMEOUT_S = 0.2
        zp.send_data("G0 X1")
        c = zp.get_comm_counters()
        self.assertEqual(c["cmd"], 1)
        self.assertEqual(c["ok_fail"], 1)

    def test_reset_counters_zeroes(self):
        zp = _bare_zp(_ScriptedSerial(loop_ok=True))
        zp.send_data("G0 X1")
        zp.reset_comm_counters()
        self.assertEqual(zp.get_comm_counters(),
                         {"cmd": 0, "ok": 0, "ok_fail": 0, "reset": 0})


if __name__ == "__main__":
    unittest.main()

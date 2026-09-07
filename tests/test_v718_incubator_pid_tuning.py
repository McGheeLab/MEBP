"""
test_v718_incubator_pid_tuning.py — host-held PID gains, re-asserted without EEPROM.

Why this exists
---------------
2026-08-18 bench run (``logs/incubator/incu_20260818_091055_hold.jsonl``):
Zone A held 37 C with the duty railing 0<->100 % for 55 % of samples and the
temperature swinging +/-0.6 C on a 588 s period. The heater was fine. The board
was running the stock Ender-3 bed gains, whose integral time is
41.78/7.32 = 5.7 s, against a water block whose time constant is 2614 s --
integral action 458x too fast. A plant identified from that log
(K=27.4 C/unit-duty, tau=2614 s, theta=79 s) reproduces the observed limit
cycle to within 1 % on period, which is what validates the retune.

The gains therefore live in the per-machine config store and are pushed with
``M304``. They are NOT saved with ``M500``: a Marlin reset reverts the running
gains to EEPROM, and this board resets whenever its port is opened (DTR) --
the same mechanism that made it forget its setpoint. So the host re-asserts
them, exactly like the setpoint keeper, and no EEPROM write is required.
"""

from __future__ import annotations

import ast
import os
import tempfile
import unittest
from types import SimpleNamespace


def _isolate_env():
    td = tempfile.mkdtemp(prefix="incu_pid_")
    os.environ["MEBP_INCUBATOR_CONFIG_DIR"] = td
    os.environ["MEBP_INCUBATOR_CAL_DIR"] = td
    os.environ["MEBP_INCUBATOR_SIM_DIR"] = td
    os.environ["MEBP_INCUBATOR_LOG"] = "0"
    return td


TUNED = {"kp": 102.40, "ki": 0.11, "kd": 0.0}


# ── store ────────────────────────────────────────────────────────

class TestStoreHoldsTheGains(unittest.TestCase):

    def setUp(self):
        self.tmp = _isolate_env()
        from SupportClasses.incubator.config_store import reset_store
        reset_store()

    def _store(self):
        from SupportClasses.incubator.config_store import IncubatorConfigStore
        return IncubatorConfigStore(os.path.join(self.tmp, "incubator.json"))

    def test_gains_round_trip_through_disk(self):
        st = self._store()
        st.set_zone("bed", pid=TUNED)
        self.assertEqual(TUNED, self._store().zone_pid("bed"))

    def test_apply_on_connect_defaults_true(self):
        self.assertTrue(self._store().get("pid_apply_on_connect"))

    def test_a_zone_with_no_gains_reads_none(self):
        self.assertIsNone(self._store().zone_pid("hotend"))

    def test_editing_a_label_does_not_disturb_the_gains(self):
        st = self._store()
        st.set_zone("bed", pid=TUNED)
        st.set_zone("bed", label="Water block")
        self.assertEqual(TUNED, st.zone_pid("bed"))

    def test_explicit_none_clears_them(self):
        st = self._store()
        st.set_zone("bed", pid=TUNED)
        st.set_zone("bed", pid=None)
        self.assertIsNone(st.zone_pid("bed"))

    def test_malformed_gains_are_REFUSED_not_repaired(self):
        """A clamped gain is a different controller than the one that was
        tuned, applied silently to a live heater. Absent is recoverable
        (the board keeps its own gains); wrong is not."""
        from SupportClasses.incubator.config_store import parse_pid
        for bad in ({"kp": 10.0, "ki": 1.0},                # incomplete
                    {"kp": -1.0, "ki": 1.0, "kd": 1.0},     # negative
                    {"kp": 1e9, "ki": 1.0, "kd": 1.0},      # absurd
                    {"kp": float("nan"), "ki": 1.0, "kd": 1.0},
                    {"kp": float("inf"), "ki": 1.0, "kd": 1.0},
                    {"kp": "x", "ki": 1.0, "kd": 1.0},
                    "not a dict", None):
            self.assertIsNone(parse_pid(bad), f"{bad!r} should be refused")

    def test_zero_kp_is_not_a_controller(self):
        from SupportClasses.incubator.config_store import parse_pid
        self.assertIsNone(parse_pid({"kp": 0.0, "ki": 1.0, "kd": 1.0}))

    def test_a_junk_file_leaves_the_board_alone(self):
        path = os.path.join(self.tmp, "incubator.json")
        with open(path, "w", encoding="utf-8") as f:
            f.write('{"zones": {"bed": {"pid": {"kp": "banana"}}}}')
        self.assertIsNone(self._store().zone_pid("bed"))


# ── controller ───────────────────────────────────────────────────

class _Ctrl(unittest.TestCase):

    def setUp(self):
        self.tmp = _isolate_env()
        from SupportClasses.incubator.config_store import reset_store
        from SupportClasses.incubator.service import reset_service
        reset_store()
        reset_service()
        from SupportClasses.incubator.controller import IncubatorController
        self.ctrl = IncubatorController()
        self.assertTrue(self.ctrl.connect(simulate=True, sim_time_scale=600))
        self.sent = []

        def _fake_send(cmd, **kw):
            self.sent.append(cmd)
            return SimpleNamespace(ok=True, rejected=False, lines=[], text="")

        self.ctrl._send = _fake_send
        for zid in ("bed", "hotend"):
            self.ctrl.zone_runtime(zid).pid_available = True

    def tearDown(self):
        try:
            self.ctrl.disconnect()
        except Exception:
            pass

    def _m304(self):
        return [c for c in self.sent if c.startswith("M304")]


class TestApplyingConfiguredGains(_Ctrl):

    def test_gains_are_pushed_with_M304_and_no_M500(self):
        self.ctrl.set_configured_pid({"bed": TUNED})
        self.ctrl._do_apply_configured_pid("connect", False)
        self.assertEqual(["M304 P102.40 I0.11 D0.00"], self._m304())
        self.assertNotIn("M500", self.sent,
                         "the tuned gains must not require an EEPROM write")

    def test_the_running_gains_are_updated_so_the_UI_reflects_them(self):
        self.ctrl.set_configured_pid({"bed": TUNED})
        self.ctrl._do_apply_configured_pid("connect", False)
        pid = self.ctrl.zone_runtime("bed").pid
        self.assertAlmostEqual(102.40, pid.kp, places=2)
        self.assertAlmostEqual(0.11, pid.ki, places=2)

    def test_gains_that_already_match_are_not_re_pushed(self):
        from SupportClasses.incubator.marlin_gcode import PidValues
        self.ctrl.zone_runtime("bed").pid = PidValues(kp=102.40, ki=0.11,
                                                      kd=0.0)
        self.ctrl.set_configured_pid({"bed": TUNED})
        self.ctrl._do_apply_configured_pid("connect", False)
        self.assertEqual([], self._m304())

    def test_FORCE_pushes_even_when_the_cached_gains_match(self):
        """The load-bearing one. After a board reset the board is back on its
        EEPROM gains, but rt.pid still holds the pre-reset value -- so an
        unforced push is skipped and the zone silently resumes oscillating."""
        from SupportClasses.incubator.marlin_gcode import PidValues
        self.ctrl.zone_runtime("bed").pid = PidValues(kp=102.40, ki=0.11,
                                                      kd=0.0)
        self.ctrl.set_configured_pid({"bed": TUNED})
        self.ctrl._do_apply_configured_pid("board reset", True)
        self.assertEqual(["M304 P102.40 I0.11 D0.00"], self._m304())

    def test_a_zone_with_no_configured_gains_is_left_alone(self):
        self.ctrl.set_configured_pid({"bed": None, "hotend": None})
        self.ctrl._do_apply_configured_pid("connect", False)
        self.assertEqual([], self._m304())

    def test_apply_on_connect_off_suppresses_the_push(self):
        """Assert on the SUBMISSION, not on the wire.

        apply_configured_pid() queues work for the controller thread rather
        than sending inline, so checking _m304() straight afterwards races the
        worker and passes whether the gate exists or not -- it did, and a
        mutation removing the gate survived because of it.
        """
        self.ctrl.set_configured_pid({"bed": TUNED}, apply_on_connect=False)
        calls = []
        self.ctrl.submit = lambda fn, *a, **k: calls.append(fn)
        self.ctrl.apply_configured_pid(reason="connect")
        self.assertEqual([], calls,
                         "the gate must stop the push being queued at all")

    def test_and_with_the_gate_ON_the_push_IS_queued(self):
        """Guard the guard: the test above must be able to fail."""
        self.ctrl.set_configured_pid({"bed": TUNED}, apply_on_connect=True)
        calls = []
        self.ctrl.submit = lambda fn, *a, **k: calls.append(fn)
        self.ctrl.apply_configured_pid(reason="connect")
        self.assertEqual(
            ["_do_apply_configured_pid"],
            [getattr(f, "__name__", "") for f in calls])

    def test_a_running_diagnostic_owns_the_gains(self):
        """A power staircase drops the loop to pure proportional and restores
        it itself; re-asserting on top would corrupt the measurement."""
        self.ctrl.zone_runtime("bed").diagnostic_active = True
        self.ctrl.set_configured_pid({"bed": TUNED})
        self.ctrl._do_apply_configured_pid("connect", False)
        self.assertEqual([], self._m304())

    def test_a_zone_without_firmware_PID_is_skipped(self):
        self.ctrl.zone_runtime("bed").pid_available = False
        self.ctrl.set_configured_pid({"bed": TUNED})
        self.ctrl._do_apply_configured_pid("connect", False)
        self.assertEqual([], self._m304())

    def test_a_refused_push_does_not_claim_the_gains_are_running(self):
        from SupportClasses.incubator.marlin_gcode import PidValues
        rt = self.ctrl.zone_runtime("bed")
        rt.pid = PidValues(kp=41.78, ki=7.32, kd=158.93)
        self.ctrl._send = lambda cmd, **kw: SimpleNamespace(
            ok=False, rejected=True, lines=[], text="Error:")
        self.ctrl.set_configured_pid({"bed": TUNED})
        self.ctrl._do_apply_configured_pid("connect", False)
        self.assertAlmostEqual(
            41.78, rt.pid.kp, places=2,
            msg="a failed push must not be reported as applied")


class TestResetReAssertsTheGains(_Ctrl):

    def test_a_detected_reset_re_asserts_the_PID_forced(self):
        rt = self.ctrl.zone_runtime("bed")
        rt.requested_c = 37.4
        rt.commanded_c = 37
        rt.refused = ""
        rt._target_mismatch = 0
        rt._commanded_at = 0.0
        self.ctrl.hub.marlin_channel = lambda key: SimpleNamespace(
            value_c=30.0, target_c=0.0, power_pct=0.0, stale=False)
        calls = []
        self.ctrl.submit = lambda fn, *a, **k: calls.append((fn, a, k))
        self.ctrl._service_setpoint_keeper()
        self.ctrl._service_setpoint_keeper()
        pid_calls = [c for c in calls
                     if getattr(c[0], "__name__", "")
                     == "_do_apply_configured_pid"]
        self.assertEqual(1, len(pid_calls),
                         "a reset must re-assert the gains, not just the target")
        self.assertTrue(pid_calls[0][1][1],
                        "the reset re-assert must be FORCED -- rt.pid still "
                        "holds the pre-reset value, so an unforced push is "
                        "skipped and the stock gains stay in charge")


class TestRampNoLongerFakesAReset(_Ctrl):
    """The 2026-08-18 run logged 7 'the board forgot its setpoint' events, all
    during the ramp. commanded_c is updated the instant the ramp decides, while
    the command is still queued -- so the board legitimately still reports the
    previous step. That inflates the counter which exists to make a genuinely
    rebooting board visible."""

    def _arm(self, age_s):
        import time as _t
        rt = self.ctrl.zone_runtime("bed")
        rt.requested_c = 37.4
        rt.commanded_c = 37
        rt.refused = ""
        rt._target_mismatch = 0
        rt.reasserts = 0
        rt._commanded_at = _t.monotonic() - age_s
        self.ctrl.hub.marlin_channel = lambda key: SimpleNamespace(
            value_c=30.0, target_c=34.0, power_pct=0.0, stale=False)
        calls = []
        self.ctrl.submit = lambda fn, *a, **k: calls.append((fn, a, k))
        return rt, calls

    def test_a_just_commanded_setpoint_is_given_time_to_land(self):
        rt, calls = self._arm(age_s=0.0)
        for _ in range(6):
            self.ctrl._service_setpoint_keeper()
        self.assertEqual([], calls)
        self.assertEqual(0, rt.reasserts,
                         "a ramp step must not count as a board reset")

    def test_but_a_real_reset_is_still_caught_once_the_grace_expires(self):
        rt, _calls = self._arm(age_s=self.ctrl.REASSERT_GRACE_S + 1.0)
        self.ctrl._service_setpoint_keeper()
        self.ctrl._service_setpoint_keeper()
        self.assertEqual(1, rt.reasserts,
                         "the grace window must not disarm the keeper")


# ── wiring ───────────────────────────────────────────────────────

class TestWiring(unittest.TestCase):

    def setUp(self):
        self.tmp = _isolate_env()
        from SupportClasses.incubator.config_store import reset_store
        reset_store()

    def test_service_pushes_the_stored_gains_onto_the_controller(self):
        from SupportClasses.incubator.config_store import get_store
        from SupportClasses.incubator.service import apply_store_config
        get_store().set_zone("bed", pid=TUNED)
        seen = {}
        ctrl = SimpleNamespace(
            set_max_setpoint_c=lambda v: None,
            set_zone_labels=lambda v: None,
            set_configured_pid=lambda m, **kw: seen.update(
                {"map": m, "kw": kw}),
        )
        apply_store_config(ctrl)
        self.assertEqual(TUNED, seen["map"]["bed"])
        self.assertTrue(seen["kw"]["apply_on_connect"])

    def test_the_keeper_and_connect_both_apply_the_configured_gains(self):
        """AST, not behaviour: the connect path is long, and gains that are
        merely *declared* would leave the board on its stock ones."""
        with open("SupportClasses/incubator/controller.py",
                  encoding="utf-8") as fh:
            src = fh.read()
        tree = ast.parse(src)
        fn = next(n for n in ast.walk(tree)
                  if isinstance(n, ast.FunctionDef)
                  and n.name == "_service_setpoint_keeper")
        keeper_calls = {ast.unparse(n.func) for n in ast.walk(fn)
                        if isinstance(n, ast.Call)}
        self.assertIn("self.apply_configured_pid", keeper_calls,
                      "a Marlin reset clears the PID too -- the keeper must "
                      "re-assert the gains, not only the setpoint")
        self.assertEqual(
            1, src.count('self.apply_configured_pid(reason="connect")'),
            "connect must push the configured gains exactly once")


class TestHardwareSetupPanel(unittest.TestCase):

    def setUp(self):
        self.tmp = _isolate_env()
        from SupportClasses.incubator.config_store import reset_store
        reset_store()
        os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
        from PySide6.QtWidgets import QApplication
        self.app = QApplication.instance() or QApplication([])

    def _panel(self):
        from SupportClasses.incubator.config_store import IncubatorConfigStore
        from gui.pages.hardware.incubator_panel import IncubatorSetupPanel
        st = IncubatorConfigStore(os.path.join(self.tmp, "incubator.json"))
        return IncubatorSetupPanel(store=st, controller=lambda: None), st

    def test_gains_round_trip_through_the_real_panel(self):
        pan, st = self._panel()
        st.set_zone("bed", pid=TUNED)
        pan.load()
        row = pan._zone_rows["bed"]
        self.assertAlmostEqual(102.40, row["kp"].value(), places=2)
        self.assertAlmostEqual(0.11, row["ki"].value(), places=2)
        row["kp"].setValue(76.80)
        self.assertTrue(pan.commit())
        self.assertAlmostEqual(76.80, st.zone_pid("bed")["kp"], places=2)

    def test_kp_zero_stores_no_gains_rather_than_a_P_less_controller(self):
        pan, st = self._panel()
        st.set_zone("bed", pid=TUNED)
        pan.load()
        pan._zone_rows["bed"]["kp"].setValue(0.0)
        pan.commit()
        self.assertIsNone(st.zone_pid("bed"))

    def test_nothing_is_written_before_save(self):
        pan, st = self._panel()
        pan.load()
        pan._zone_rows["bed"]["kp"].setValue(55.0)
        self.assertIsNone(st.zone_pid("bed"),
                          "the panel wrote before commit()")


if __name__ == "__main__":
    unittest.main()

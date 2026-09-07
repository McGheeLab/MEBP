"""
test_v718_staircase_in_app.py -- the power staircase inside the app.

v7.18 round 6. The bench tool answers "how much bed power can this rig
carry" with the app closed; the operator asked for the same options on the
Incubator page. Running it in-app adds three hazards the bench version does
not have, and these pin all three:

  1. The setpoint has ONE writer. A staircase drives the target directly,
     so the keeper (round 4) and the dither (round 3) must stand off, or
     the measurement is of the two fighting rather than of the supply.
  2. On the shared transport the board it may reset also runs Z and the
     pumps -- so it refuses outright while a print owns the channel.
  3. mode="pid" rewrites the bed PID, so it refuses to start without a copy
     of the gains it can put back, and restores them on every exit path.
"""

from __future__ import annotations

import os
import tempfile
import time
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")


def _isolate_env():
    td = tempfile.mkdtemp(prefix="incu_sc_")
    os.environ["MEBP_INCUBATOR_CONFIG_DIR"] = td
    os.environ["MEBP_INCUBATOR_CAL_DIR"] = td
    os.environ["MEBP_INCUBATOR_SIM_DIR"] = td
    os.environ["MEBP_INCUBATOR_LOG"] = "0"
    return td


class _SimCtrl(unittest.TestCase):

    def setUp(self):
        _isolate_env()
        from SupportClasses.incubator.config_store import reset_store
        from SupportClasses.incubator.service import reset_service
        reset_store()
        reset_service()
        from SupportClasses.incubator.controller import IncubatorController
        self.ctrl = IncubatorController()
        self.assertTrue(self.ctrl.connect(simulate=True, sim_time_scale=600))

    def tearDown(self):
        try:
            self.ctrl.cancel_power_staircase()
            self.ctrl.disconnect()
        except Exception:
            pass

    def _wait_done(self, timeout=45.0):
        t0 = time.monotonic()
        while self.ctrl.staircase_running() and time.monotonic() - t0 < timeout:
            time.sleep(0.05)
        self.assertFalse(self.ctrl.staircase_running(), "staircase never ended")

    def _pid_ready(self):
        self.ctrl.query_pid()
        t0 = time.monotonic()
        while (self.ctrl.zone_runtime("bed").pid is None
               and time.monotonic() - t0 < 10.0):
            time.sleep(0.05)
        return self.ctrl.zone_runtime("bed").pid


# ── one writer: the keeper and the dither stand off ────────────────

class TestOneWriter(_SimCtrl):

    def _fake_channel(self, target_c, temp=25.0):
        from types import SimpleNamespace
        return SimpleNamespace(value_c=temp, target_c=target_c,
                               power_pct=0.0, stale=False)

    def _arm_disagreement(self):
        """Zone A holds 37; the board claims 0 -- the keeper's trigger."""
        rt = self.ctrl.zone_runtime("bed")
        rt.requested_c, rt.commanded_c = 37.0, 37
        rt.refused, rt.reasserts, rt._target_mismatch = "", 0, 0
        self.ctrl.hub.marlin_channel = lambda key: self._fake_channel(0.0)
        sent = []
        self.ctrl.submit = lambda fn, *a, **k: sent.append((fn, a, k))
        return rt, sent

    def test_the_keeper_would_normally_re_assert(self):
        """Guard the guard: without the flag the keeper DOES act, so the
        test below cannot pass just because nothing ever re-asserts."""
        rt, sent = self._arm_disagreement()
        rt.diagnostic_active = False
        self.ctrl._service_setpoint_keeper()
        self.ctrl._service_setpoint_keeper()      # two-sample debounce
        # A detected reset re-asserts the PID gains as well as the setpoint
        # (a Marlin reset clears both), so count the setpoint command rather
        # than every submission.
        setpoints = [e for e in sent
                     if getattr(e[0], "__name__", "") == "_send_zone_cmd"]
        self.assertEqual(1, len(setpoints))
        self.assertEqual("M140 S37", setpoints[0][1][1])

    def test_the_keeper_stands_off_during_a_staircase(self):
        rt, sent = self._arm_disagreement()
        rt.diagnostic_active = True
        for _ in range(5):
            self.ctrl._service_setpoint_keeper()
        self.assertEqual([], sent,
                         "the keeper fought the staircase for the setpoint")

    def test_the_dither_stands_off_during_a_staircase(self):
        rt = self.ctrl.zone_runtime("bed")
        rt.dither_enabled = True
        rt.dither_target_c = 37.5
        rt._dither_next_flip = 0.0
        sent = []
        self.ctrl.submit = lambda fn, *a, **k: sent.append((fn, a, k))
        rt.diagnostic_active = True
        self.ctrl._service_dither()
        self.assertEqual([], sent, "the dither fought the staircase")
        rt.diagnostic_active = False
        self.ctrl._service_dither()
        self.assertEqual(1, len(sent), "guard-the-guard: dither never fires")

    def test_the_flag_is_cleared_when_the_run_ends(self):
        self._pid_ready()
        self.assertTrue(self.ctrl.start_power_staircase(
            "bed", rungs="25", rung_seconds=2.0, ceiling_c=45,
            abort_above_c=90, mode="pid"))
        self.assertTrue(self.ctrl.zone_runtime("bed").diagnostic_active)
        self._wait_done()
        self.assertFalse(self.ctrl.zone_runtime("bed").diagnostic_active,
                         "a stuck flag would disable the keeper forever")


# ── refusals ───────────────────────────────────────────────────────

class TestRefusals(_SimCtrl):

    def test_refuses_while_a_print_owns_the_channel(self):
        """Resetting the motion board mid-print is a crash, not a test."""
        self.ctrl._poll_permitted = lambda: False
        msgs = []
        self.ctrl.on_status(msgs.append)
        self.assertFalse(self.ctrl.start_power_staircase(
            "bed", rungs="25", ceiling_c=45))
        self.assertTrue(any("print" in m.lower() for m in msgs), msgs)

    def test_pid_mode_refuses_without_gains_it_can_restore(self):
        rt = self.ctrl.zone_runtime("bed")
        rt.pid = None
        rt.pid_available = True
        msgs = []
        self.ctrl.on_status(msgs.append)
        self.assertFalse(self.ctrl.start_power_staircase(
            "bed", rungs="25", ceiling_c=45, mode="pid"))
        self.assertTrue(any("PID" in m for m in msgs), msgs)

    def test_pwm_mode_runs_without_any_pid(self):
        """It never touches the gains, so it must not inherit the refusal."""
        rt = self.ctrl.zone_runtime("bed")
        rt.pid = None
        self.assertTrue(self.ctrl.start_power_staircase(
            "bed", rungs="25", rung_seconds=2.0, ceiling_c=45,
            abort_above_c=90, mode="pwm"))
        self._wait_done()

    def test_refuses_a_zone_whose_sensor_is_faulty(self):
        rt = self.ctrl.zone_runtime("bed")
        rt.sensor_ok = False
        rt.sensor_fault = "open circuit."
        self.assertFalse(self.ctrl.start_power_staircase(
            "bed", rungs="25", ceiling_c=45, mode="pwm"))

    def test_refuses_junk_rungs_without_commanding_anything(self):
        self.assertFalse(self.ctrl.start_power_staircase(
            "bed", rungs="0,999", ceiling_c=45, mode="pwm"))
        self.assertFalse(self.ctrl.staircase_running())

    def test_will_not_start_twice(self):
        self.assertTrue(self.ctrl.start_power_staircase(
            "bed", rungs="25,50", rung_seconds=4.0, ceiling_c=45,
            abort_above_c=90, mode="pwm"))
        self.assertFalse(self.ctrl.start_power_staircase(
            "bed", rungs="25", ceiling_c=45, mode="pwm"))
        self.ctrl.cancel_power_staircase()
        self._wait_done()


# ── the run leaves the board as it found it ────────────────────────

class TestRunLeavesNoTrace(_SimCtrl):

    def test_pid_is_restored_and_the_heater_is_off(self):
        orig = self._pid_ready()
        self.assertIsNotNone(orig)
        sent = []
        real = self.ctrl._send
        self.ctrl._send = lambda cmd, **k: (sent.append(cmd), real(cmd, **k))[1]
        self.assertTrue(self.ctrl.start_power_staircase(
            "bed", rungs="25", rung_seconds=2.0, ceiling_c=45,
            abort_above_c=90, mode="pid"))
        self._wait_done()
        time.sleep(0.5)
        joined = " | ".join(sent)
        self.assertIn(f"M304 P{orig.kp:.2f}", joined.replace(",", "."))
        self.assertEqual(0, self.ctrl.zone_runtime("bed").commanded_c,
                         "the heater was left commanded after the run")

    def test_nothing_is_ever_saved_to_eeprom(self):
        """M500 would make a diagnostic's pure-P gains permanent."""
        self._pid_ready()
        sent = []
        real = self.ctrl._send
        self.ctrl._send = lambda cmd, **k: (sent.append(cmd), real(cmd, **k))[1]
        self.assertTrue(self.ctrl.start_power_staircase(
            "bed", rungs="25", rung_seconds=2.0, ceiling_c=45,
            abort_above_c=90, mode="pid"))
        self._wait_done()
        self.assertEqual([], [c for c in sent if c.strip().upper().startswith("M500")])

    def test_cancel_ends_the_run_and_says_so(self):
        from SupportClasses.incubator.power_staircase import verdict_lines
        done = []
        self.ctrl.on_staircase_done(lambda z, o: done.append(o))
        self.assertTrue(self.ctrl.start_power_staircase(
            "bed", rungs="25,50,75,100", rung_seconds=30.0, ceiling_c=45,
            abort_above_c=90, mode="pwm"))
        time.sleep(1.0)
        self.ctrl.cancel_power_staircase()
        self._wait_done()
        self.assertTrue(done)
        self.assertTrue(done[0].cancelled)
        self.assertIn("Stopped by the operator",
                      "\n".join(verdict_lines(done[0])))

    def test_an_unanswered_command_is_not_reported_as_a_refusal(self):
        """Round 3's rule: only an ANSWERED no is a verdict about the zone.
        Calling a timeout 'refused' sends the operator after the wiring when
        the port had simply gone."""
        from types import SimpleNamespace
        self.ctrl._send_zone_cmd = lambda *a, **k: SimpleNamespace(
            ok=False, rejected=False, error_text="")
        self.assertEqual("dropped", self.ctrl._sc_apply("bed", "M140 S40"))

    def test_an_answered_no_is_reported_as_a_refusal(self):
        from types import SimpleNamespace
        self.ctrl._send_zone_cmd = lambda *a, **k: SimpleNamespace(
            ok=False, rejected=True, error_text="Error: no bed")
        self.assertEqual("refused", self.ctrl._sc_apply("bed", "M140 S40"))

    def test_an_accepted_command_reports_no_failure(self):
        from types import SimpleNamespace
        self.ctrl._send_zone_cmd = lambda *a, **k: SimpleNamespace(
            ok=True, rejected=False, error_text="")
        self.assertEqual("", self.ctrl._sc_apply("bed", "M140 S40"))

    def test_rows_are_published_while_it_runs(self):
        rows = []
        self.ctrl.on_staircase_row(rows.append)
        # Longer than one transaction timeout (8 s): late in a full suite the
        # simulated link can take that long for a single command, and a rung
        # shorter than that can end before the 1 Hz sampler ever runs. The
        # assertion is unchanged -- only the window it is given.
        self.assertTrue(self.ctrl.start_power_staircase(
            "bed", rungs="50", rung_seconds=12.0, ceiling_c=45,
            abort_above_c=90, mode="pwm"))
        self._wait_done(timeout=60.0)
        self.assertTrue(rows, "no live rows reached the UI")
        for key in ("zone", "pct", "elapsed_s", "temp_c", "duty", "duty_pct"):
            self.assertIn(key, rows[0])


# ── the page ───────────────────────────────────────────────────────

class TestWatchdogArmGuard(_SimCtrl):
    """Bench 2026-08-17: a direct M140 S37 from ~21 C armed Marlin's heat-up
    watchdog and ended in Heating Failed -> kill() at 60 s, on BOTH boards.
    That is exactly what the zone card's Set button sends, and on the shared
    link the halt takes Z and the pumps with it. ramp.py already derived the
    arm threshold from Marlin's own HeaterWatch::restart; it just had no
    production caller."""

    def _fake_channel(self, temp_c, stale=False):
        from types import SimpleNamespace
        return SimpleNamespace(value_c=temp_c, target_c=0.0,
                               power_pct=0.0, stale=stale)

    def _at(self, temp_c, stale=False):
        self.ctrl.hub.marlin_channel = lambda key: self._fake_channel(
            temp_c, stale)

    def test_a_big_jump_is_flagged(self):
        self._at(21.0)
        self.assertTrue(
            self.ctrl.preview_setpoint("bed", 37.0)["watchdog_risk"])

    def test_a_step_below_the_arm_threshold_is_not(self):
        """The whole point of the ramp: under the threshold the watchdog is
        never scheduled, so a small step must not raise a warning."""
        self._at(21.0)
        self.assertFalse(
            self.ctrl.preview_setpoint("bed", 24.0)["watchdog_risk"])

    def test_the_threshold_matches_marlins_own_arithmetic(self):
        from SupportClasses.incubator.ramp import watchdog_arm_threshold_c
        self._at(21.0)
        plan = self.ctrl.preview_setpoint("bed", 37.0)
        self.assertAlmostEqual(watchdog_arm_threshold_c(),
                               plan["watchdog_arm_gap_c"])
        gap = plan["watchdog_arm_gap_c"]
        self.assertTrue(
            self.ctrl.preview_setpoint("bed", 21.0+gap+0.5)["watchdog_risk"])
        self.assertFalse(
            self.ctrl.preview_setpoint("bed", 21.0+gap-0.5)["watchdog_risk"])

    def test_no_reading_means_no_claim(self):
        """A risk we cannot substantiate would train the operator to click
        through the warning."""
        self._at(21.0, stale=True)
        self.assertFalse(
            self.ctrl.preview_setpoint("bed", 37.0)["watchdog_risk"])
        self.ctrl.hub.marlin_channel = lambda key: None
        self.assertFalse(
            self.ctrl.preview_setpoint("bed", 37.0)["watchdog_risk"])

    def test_heater_off_is_never_flagged(self):
        self._at(21.0)
        self.assertFalse(self.ctrl.preview_setpoint("bed", 0.0)["watchdog_risk"])

    def test_the_rest_of_the_preview_is_unchanged(self):
        """Existing callers read these keys; the guard must be additive."""
        self._at(21.0)
        plan = self.ctrl.preview_setpoint("bed", 37.0)
        for key in ("check", "requested_c", "raw_wanted_c", "commanded_c",
                    "predicted_real_c", "calibrated", "quantisation_error_c"):
            self.assertIn(key, plan)


class TestWatchdogDialogSeam(unittest.TestCase):
    """The offer must be ONE overridable seam, not a bare modal inside
    _on_set: offscreen a modal blocks forever, so a hidden one turns every
    future test that presses Set into a hang (it did exactly that here)."""

    def test_the_card_exposes_the_seam(self):
        from gui.widgets.incubator_widgets import ZoneCard
        self.assertTrue(hasattr(ZoneCard, "_ask_watchdog"))

    def test_on_set_routes_through_it_and_honours_each_answer(self):
        import ast, inspect
        from gui.widgets.incubator_widgets import ZoneCard
        src = inspect.getsource(ZoneCard._on_set)
        tree = ast.parse(src.lstrip().replace("def _on_set", "def f", 1))
        names = {n.attr for n in ast.walk(tree)
                 if isinstance(n, ast.Attribute)}
        self.assertIn("_ask_watchdog", names,
                      "_on_set no longer asks before a board-killing set")
        # and no raw modal was left behind in _on_set itself
        self.assertNotIn("exec", names,
                         "_on_set opens a modal directly -- offscreen that "
                         "hangs every test that presses Set")


class TestStaircaseTab(unittest.TestCase):

    def setUp(self):
        _isolate_env()
        from SupportClasses.incubator.config_store import reset_store
        from SupportClasses.incubator.service import reset_service
        reset_store()
        reset_service()
        from PySide6.QtWidgets import QApplication
        self.app = QApplication.instance() or QApplication([])
        from gui.pages.workflows.incubator_workflow import (
            IncubatorWorkflowPage,
        )
        self.page = IncubatorWorkflowPage()

    def tearDown(self):
        try:
            self.page.ctrl.cancel_power_staircase()
            if self.page.ctrl.connected:
                self.page.ctrl.disconnect()
        except Exception:
            pass
        self.page.deleteLater()
        self.app.processEvents()

    def _tab_titles(self):
        from PySide6.QtWidgets import QTabWidget
        tabs = self.page.findChild(QTabWidget)
        return [tabs.tabText(i) for i in range(tabs.count())]

    def test_the_tab_exists_with_its_controls(self):
        self.assertIn("Power staircase", self._tab_titles())
        self.assertEqual("pid", self.page._sc_mode.itemData(0))
        self.assertEqual("pwm", self.page._sc_mode.itemData(1))
        self.assertTrue(self.page._sc_rungs.text())
        self.assertFalse(self.page._sc_stop.isEnabled())

    def test_the_ceiling_cannot_exceed_the_configured_maximum(self):
        self.assertLessEqual(self.page._sc_ceiling.maximum(),
                             float(self.page.ctrl.MAX_SETPOINT_C))

    def test_declining_the_confirmation_starts_nothing(self):
        """It can reset the motion board -- it must never run un-asked."""
        from PySide6.QtWidgets import QMessageBox
        calls = []
        self.page.ctrl.start_power_staircase = (
            lambda *a, **k: calls.append(a) or True)
        orig = QMessageBox.question
        QMessageBox.question = staticmethod(lambda *a, **k: QMessageBox.No)
        try:
            self.page._sc_start.click()
        finally:
            QMessageBox.question = orig
        self.assertEqual([], calls)

    def test_junk_rungs_never_reach_the_controller(self):
        from PySide6.QtWidgets import QMessageBox
        calls = []
        self.page.ctrl.start_power_staircase = (
            lambda *a, **k: calls.append(a) or True)
        orig_q, orig_w = QMessageBox.question, QMessageBox.warning
        QMessageBox.question = staticmethod(lambda *a, **k: QMessageBox.Yes)
        QMessageBox.warning = staticmethod(lambda *a, **k: None)
        try:
            self.page._sc_rungs.setText("nonsense")
            self.page._sc_start.click()
        finally:
            QMessageBox.question, QMessageBox.warning = orig_q, orig_w
        self.assertEqual([], calls)

    def test_a_refused_start_re_enables_the_buttons(self):
        """Otherwise a refusal leaves the tab permanently dead."""
        from PySide6.QtWidgets import QMessageBox
        self.page.ctrl.start_power_staircase = lambda *a, **k: False
        orig = QMessageBox.question
        QMessageBox.question = staticmethod(lambda *a, **k: QMessageBox.Yes)
        try:
            self.page._sc_start.click()
        finally:
            QMessageBox.question = orig
        self.assertTrue(self.page._sc_start.isEnabled())
        self.assertFalse(self.page._sc_stop.isEnabled())
        self.assertIn("refused", self.page._sc_log.toPlainText())

    def test_the_verdict_is_rendered_from_the_shared_formatter(self):
        from SupportClasses.incubator.power_staircase import (
            StaircaseOutcome, verdict_lines,
        )
        out = StaircaseOutcome(survived=[(25, 32)],
                               died_at=(50, 64, "dropped", "OSError: x"))
        self.page._on_staircase_done("bed", out)
        text = self.page._sc_log.toPlainText()
        for line in verdict_lines(out):
            if line:
                self.assertIn(line, text)

    def test_a_live_row_is_rendered(self):
        self.page._on_staircase_row({
            "zone": "bed", "pct": 25, "elapsed_s": 1.0, "temp_c": 22.4,
            "target_c": 40.0, "duty": 32, "duty_pct": 25.2})
        text = self.page._sc_log.toPlainText()
        self.assertIn("25", text)
        self.assertIn("22.40", text)


if __name__ == "__main__":
    unittest.main()

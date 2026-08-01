"""test_v75x_pid_from_dead_time.py — repeatable cross-track PID gains.

The ZN relay tuner gave a different answer every run (observed on ME3B V1: kp
walked 4.938 → 6.216 → 6.763 → 7.052 across consecutive presses). Root causes:

  1. the relay switch had NO HYSTERESIS, so near the line the sign chattered on
     µm encoder quantisation and manufactured spurious tiny half-cycles;
  2. the amplitude was the MEAN over half-cycles, which those spurious cycles
     dragged down — and since Ku = 4d/(πa), an under-measured amplitude INFLATES
     the gain;
  3. one run, no repeat, and no check that the answer was even physically
     possible.

The fix rests on a closed-form fact: from perpendicular velocity command to
cross-track position the plant is a PURE INTEGRATOR WITH TRANSPORT DELAY, so

    Ku = π/(2L)      Tu = 4L        L = dead_time + loop_period

which makes the gains derivable from the measured dead time — exactly repeatable,
no oscillation to mis-measure — and gives a hard ceiling any relay result must
respect. The operator's own relay run corroborates the model: it measured
Tu = 0.391 s against the predicted 0.396 s (~1 %); only Ku was wrong.
"""

import math
import unittest

from SupportClasses import VelocityControl as VC


#: The operator's real measured machine state (2026-07-28).
ME3B_DEAD_S = 0.0672
ME3B_LOOP_MS = 31.75


class TestPlantUltimateGain(unittest.TestCase):
    def test_matches_the_closed_form(self):
        for dead, loop in ((0.0672, 31.75), (0.045, 31.75), (0.2872, 31.75),
                           (0.010, 5.0)):
            L = dead + loop / 1000.0
            ku, tu = VC.plant_ultimate_gain(dead, loop)
            self.assertAlmostEqual(ku, math.pi / (2 * L), places=9)
            self.assertAlmostEqual(tu, 4 * L, places=9)

    def test_shorter_dead_time_allows_more_gain(self):
        slow, _ = VC.plant_ultimate_gain(0.2872, 31.75)
        fast, _ = VC.plant_ultimate_gain(0.0672, 31.75)
        self.assertGreater(fast, slow)

    def test_unmeasured_returns_zero(self):
        self.assertEqual(VC.plant_ultimate_gain(0.0, 0.0), (0.0, 0.0))
        self.assertEqual(VC.plant_ultimate_gain(None, None), (0.0, 0.0))

    def test_pins_this_machines_numbers(self):
        ku, tu = VC.plant_ultimate_gain(ME3B_DEAD_S, ME3B_LOOP_MS)
        self.assertAlmostEqual(ku, 15.87, delta=0.02)
        self.assertAlmostEqual(tu, 0.396, delta=0.002)


class TestGainsFromDeadTime(unittest.TestCase):
    def test_repeatable_by_construction(self):
        """The property the relay tuner lacked: same inputs → same answer."""
        a = VC.pid_gains_from_dead_time(ME3B_DEAD_S, ME3B_LOOP_MS)
        b = VC.pid_gains_from_dead_time(ME3B_DEAD_S, ME3B_LOOP_MS)
        self.assertEqual(a, b)

    def test_gain_is_below_the_stability_limit(self):
        g = VC.pid_gains_from_dead_time(ME3B_DEAD_S, ME3B_LOOP_MS)
        self.assertLess(g["kp"], g["kp_stability_limit"])
        self.assertAlmostEqual(g["kp"], 0.33 * g["ku"], places=9)

    def test_pins_this_machines_gain(self):
        g = VC.pid_gains_from_dead_time(ME3B_DEAD_S, ME3B_LOOP_MS, use_kd=False)
        self.assertAlmostEqual(g["kp"], 5.239, delta=0.01)
        self.assertEqual(g["kd"], 0.0)

    def test_the_old_kp_grid_could_not_reach_the_right_answer(self):
        """The descent grid topped out at 4.0 while the correct gain is ~5.24, so
        the optimiser was structurally unable to find it."""
        correct = VC.pid_gains_from_dead_time(ME3B_DEAD_S, ME3B_LOOP_MS)["kp"]
        old_grid = (0.0, 0.5, 1.0, 2.0, 4.0)
        self.assertGreater(correct, max(old_grid))
        from gui.dialogs import xy_challenge_dialog as d
        new_grid = dict(d.AUTOTUNE["velocity"])["pid_kp"]
        self.assertGreater(max(new_grid), correct)      # now straddles it
        self.assertLess(min(new_grid), correct)

    def test_ki_is_always_zero(self):
        """The plant is already an integrator: P alone has zero steady-state
        error, and a second integrator is a classic limit-cycle source."""
        g = VC.pid_gains_from_dead_time(ME3B_DEAD_S, ME3B_LOOP_MS)
        self.assertEqual(g["ki"], 0.0)
        self.assertIn("ki_theoretical", g)             # reported, not applied

    def test_gain_margin_divides_down(self):
        base = VC.pid_gains_from_dead_time(ME3B_DEAD_S, ME3B_LOOP_MS)["kp"]
        half = VC.pid_gains_from_dead_time(ME3B_DEAD_S, ME3B_LOOP_MS,
                                           gain_margin=2.0)["kp"]
        self.assertAlmostEqual(half, base / 2.0, places=9)
        # A margin below 1 must not AMPLIFY the gain.
        self.assertAlmostEqual(
            VC.pid_gains_from_dead_time(ME3B_DEAD_S, ME3B_LOOP_MS,
                                        gain_margin=0.1)["kp"], base, places=9)

    def test_methods_are_ordered_by_aggressiveness(self):
        kw = dict(control_loop_ms=ME3B_LOOP_MS)
        gentle = VC.pid_gains_from_dead_time(ME3B_DEAD_S, method="no_overshoot",
                                             **kw)["kp"]
        mid = VC.pid_gains_from_dead_time(ME3B_DEAD_S, method="some_overshoot",
                                          **kw)["kp"]
        hot = VC.pid_gains_from_dead_time(ME3B_DEAD_S, method="classic",
                                          **kw)["kp"]
        self.assertLess(gentle, mid)
        self.assertLess(mid, hot)

    def test_use_kd_toggle(self):
        with_d = VC.pid_gains_from_dead_time(ME3B_DEAD_S, ME3B_LOOP_MS,
                                              use_kd=True)
        without = VC.pid_gains_from_dead_time(ME3B_DEAD_S, ME3B_LOOP_MS,
                                              use_kd=False)
        self.assertGreater(with_d["kd"], 0.0)
        self.assertEqual(without["kd"], 0.0)
        self.assertAlmostEqual(with_d["kp"], without["kp"], places=9)

    def test_unmeasured_dead_time_yields_no_gains(self):
        g = VC.pid_gains_from_dead_time(0.0, 0.0)
        self.assertEqual(g["kp"], 0.0)
        self.assertEqual(g["kd"], 0.0)


class TestRelaySanity(unittest.TestCase):
    def test_rejects_the_operators_actual_relay_result(self):
        """kp 6.216 / kd 0.8098 inverts to Ku 18.84, Tu 0.391 s. Tu matches the
        prediction to ~1 % — but Ku is 1.19× the physical maximum, which is the
        fingerprint of an under-measured amplitude."""
        ku = 6.216 / 0.33
        tu = 0.8098 / (0.11 * ku)
        chk = VC.relay_sanity(ku, tu, ME3B_DEAD_S, ME3B_LOOP_MS)
        self.assertFalse(chk["ok"])
        self.assertGreater(chk["ku_ratio"], 1.15)
        self.assertAlmostEqual(chk["tu_ratio"], 1.0, delta=0.05)  # Tu was fine
        self.assertIn("under-measured", chk["reason"])

    def test_accepts_a_physically_consistent_measurement(self):
        ku, tu = VC.plant_ultimate_gain(ME3B_DEAD_S, ME3B_LOOP_MS)
        chk = VC.relay_sanity(ku * 0.9, tu * 1.05, ME3B_DEAD_S, ME3B_LOOP_MS)
        self.assertTrue(chk["ok"], msg=chk["reason"])

    def test_rejects_a_wildly_wrong_period(self):
        ku, tu = VC.plant_ultimate_gain(ME3B_DEAD_S, ME3B_LOOP_MS)
        chk = VC.relay_sanity(ku * 0.8, tu * 0.1, ME3B_DEAD_S, ME3B_LOOP_MS)
        self.assertFalse(chk["ok"])
        self.assertIn("Tu", chk["reason"])

    def test_degenerate_and_unmeasured_inputs(self):
        self.assertFalse(VC.relay_sanity(0.0, 0.0, ME3B_DEAD_S,
                                         ME3B_LOOP_MS)["ok"])
        chk = VC.relay_sanity(10.0, 0.4, 0.0, 0.0)
        self.assertFalse(chk["ok"])
        self.assertIn("unmeasured", chk["reason"])

    def test_noise_chatter_inflates_ku_and_the_gate_catches_it(self):
        """Simulate the original defect: a true 0.5 mm limit cycle polluted by
        spurious ~5 µm half-cycles. Averaging drags the amplitude down ~10×, so
        Ku comes out ~10× too big — and the gate rejects it."""
        d = 1.8
        true_a = 0.5
        amps = [true_a] * 4 + [0.005] * 36        # 4 real cycles, 36 noise
        mean_a = sum(amps) / len(amps)
        median_a = sorted(amps)[len(amps) // 2]
        ku_mean, _ = VC.relay_ultimate_gain(d, mean_a, 0.4)
        ku_true, _ = VC.relay_ultimate_gain(d, true_a, 0.4)
        self.assertGreater(ku_mean, 5.0 * ku_true)     # the inflation
        self.assertFalse(VC.relay_sanity(ku_mean, 0.4, ME3B_DEAD_S,
                                         ME3B_LOOP_MS)["ok"])
        # The median is ALSO wrong here (noise dominates the count) — which is why
        # the hysteresis band that stops the chatter is the real fix, with the
        # median and the gate as defence in depth.
        self.assertLess(median_a, true_a)


class TestAnalyticWorkerIsRepeatable(unittest.TestCase):
    """End-to-end through the dialog: pressing it twice must give one answer."""

    def test_worker_stores_the_same_gains_every_time(self):
        import os
        os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
        from PySide6.QtWidgets import QApplication
        QApplication.instance() or QApplication([])
        from types import SimpleNamespace
        from tests.support.store_fixture import use_temp_store
        from gui.dialogs.xy_challenge_dialog import XYChallengeDialog, _Bridge

        store = use_temp_store(self)
        store.set_control_loop_ms(ME3B_LOOP_MS)
        store.set_velocity_dead_time_s(ME3B_DEAD_S, n=5)

        dlg = XYChallengeDialog.__new__(XYChallengeDialog)
        dlg._store = store
        dlg._bridge = _Bridge()
        results = []
        dlg._bridge.finished.connect(lambda ok, m: results.append((ok, m)))

        dlg._worker_pid_analytic()
        first = store.get_mode_params("velocity")["pid_kp"]
        dlg._worker_pid_analytic()
        second = store.get_mode_params("velocity")["pid_kp"]

        self.assertEqual(first, second)
        self.assertAlmostEqual(first, 5.239, delta=0.01)
        self.assertEqual(store.get_mode_params("velocity")["pid_kd"], 0.0)
        self.assertTrue(all(ok for ok, _m in results))

    def test_refuses_without_a_measured_dead_time(self):
        import os
        os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
        from PySide6.QtWidgets import QApplication
        QApplication.instance() or QApplication([])
        from tests.support.store_fixture import use_temp_store
        from gui.dialogs.xy_challenge_dialog import XYChallengeDialog, _Bridge

        store = use_temp_store(self)          # nothing measured
        dlg = XYChallengeDialog.__new__(XYChallengeDialog)
        dlg._store = store
        dlg._bridge = _Bridge()
        out = []
        dlg._bridge.finished.connect(lambda ok, m: out.append((ok, m)))
        dlg._worker_pid_analytic()
        self.assertTrue(out)
        self.assertFalse(out[0][0])
        self.assertIn("Measure the dead time", out[0][1])


if __name__ == "__main__":
    unittest.main()

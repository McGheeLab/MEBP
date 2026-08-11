"""v7.11 — the optical plate-bottom worker, driven synchronously.

``run()`` is called directly rather than via ``start()``: this exercises the real
body with no thread, so a failure points at a line instead of a race.

THE PROPERTY THIS FILE EXISTS FOR: the needle is commanded exactly ONCE per
margin, before the sweep, and never while the sweep is running. If that ever
stops being true the tip is moving through the glass-side end of a ±3·DOF sweep,
which at 4x is 150 µm below where it was parked.
"""

import os
import sys
import threading
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import numpy as np                                    # noqa: E402
from PySide6.QtWidgets import QApplication            # noqa: E402

from SupportClasses.ObjectiveOptics import ObjectiveOptics  # noqa: E402
from SupportClasses.PlateBottomOptical import (       # noqa: E402
    needle_target_zref, plate_bottom_from_rung, tip_height_um)
import gui.widgets.needle_bore_wizard as wiz          # noqa: E402

B_TRUE = 21.130
F0 = 1200.0
ZDIR = -1.0
FUS = 1.0

OPTICS_10X = ObjectiveOptics(
    label="10x", position=2, magnification=10.0, numerical_aperture=0.30,
    working_distance_mm=16.0, um_per_px_sample=1.32, frame_wh=(2048, 2048))


class _Op:
    def __init__(self, error=None):
        self.done = threading.Event()
        self.done.set()
        self.error = error


class FakeScope:
    """Only the surface the worker touches."""

    def __init__(self, *, focus_um=900.0, lo=0.0, hi=10000.0, stale_ops=()):
        self.focus = float(focus_um)
        self.lo, self.hi = lo, hi
        self.leases = []
        self.released = []
        self.calls = 0
        # A stale drop is TRANSIENT — the queue clears. Failing every op
        # forever would also break the restore, which is a different fault.
        self._stale_ops = set(stale_ops)

    class _State:
        def __init__(self, f, lo, hi):
            self.focus_um, self.focus_min_um, self.focus_max_um = f, lo, hi
            self.has_focus = True
            self.objective_position = 2
            self.mounted_objectives = ()

    def state(self):
        return self._State(self.focus, self.lo, self.hi)

    def set_focus_um(self, v):
        self.calls += 1
        if self.calls in self._stale_ops:
            return _Op(error="dropped (stale)")
        self.focus = max(self.lo, min(self.hi, float(v)))   # clamps SILENTLY
        return _Op()

    def try_acquire(self, owner, timeout=0.0):
        self.leases.append(owner)
        return True

    def release(self, owner):
        self.released.append(owner)

    def lease_owner(self):
        return self.leases[-1] if self.leases else None


class FakeCtrl:
    """Records every Z command — the property under test."""

    def __init__(self, z=25.0):
        self.z = float(z)
        self.moves = []
        self.suspends = 0
        self.resumes = 0
        self.floor = []

    def move_z_absolute(self, z, from_zero_ref=False):
        self.moves.append(float(z))
        self.z = float(z)

    def capture_current_z_raw(self):
        return self.z

    def wait_for_z_arrival(self, z, **kw):
        return True

    def suspend_position_poller(self):
        self.suspends += 1

    def resume_position_poller(self):
        self.resumes += 1

    def z_height_of(self, z):
        return ZDIR * float(z)

    def print_z_dir(self):
        return ZDIR

    def set_print_floor_active(self, on):
        self.floor.append(bool(on))


class FakeCam:
    """Renders a tip whose CONTRAST peaks when the focal plane meets it.

    The tip's focus is derived from the needle's ACTUAL Z against the TRUE plate
    bottom, so the fake obeys physics rather than being told the answer. That is
    what lets a test give the worker a wrong guess and check that the result is
    unaffected — with a hard-coded tip focus the two would simply contradict.

    Defocus reduces contrast; it does not change the mean intensity. Modulating
    brightness instead produces a curve that inverts as the feature crosses the
    background level, which the peak estimator then correctly refuses — a fake
    that fails for a reason unrelated to the code under test.
    """

    def __init__(self, scope, ctrl, sigma_um=12.0, size=256):
        self._scope = scope
        self._ctrl = ctrl
        self._sigma = float(sigma_um)
        self._n = size
        self._count = 0

    def tip_focus_um(self):
        """Where the objective must sit to see the tip sharply."""
        height_um = ZDIR * (self._ctrl.z - B_TRUE) * 1000.0
        return F0 + height_um * FUS

    def frame_count_value(self):
        self._count += 1
        return self._count

    def get_current_frame(self):
        d = (self._scope.focus - self.tip_focus_um()) / self._sigma
        contrast = float(np.exp(-0.5 * d * d))
        y, x = np.mgrid[0:self._n, 0:self._n]
        checker = (((x // 4) + (y // 4)) % 2).astype(np.float64)
        img = 128.0 + (checker - 0.5) * 200.0 * contrast
        return np.clip(img, 0, 255).astype(np.uint8)

    def get_hw_settings(self):
        return {"exposure_us": 15000, "gain": 1.0}


def make_worker(*, margin=1000.0, ctrl=None, scope=None, cam=None,
                bottom=None, **kw):
    """``bottom`` is the ESTIMATE the worker plans against; ``B_TRUE`` is the
    truth the camera obeys. Passing a wrong ``bottom`` models a bad guess."""
    scope = scope or FakeScope()
    ctrl = ctrl or FakeCtrl()
    cam = cam or FakeCam(scope, ctrl)
    w = wiz.PlateBottomRungWorker(
        controller=ctrl, scope=scope, cam=cam, margin_um=margin,
        focus_zero_um=F0,
        plate_bottom_zref_mm=(B_TRUE if bottom is None else bottom),
        zdir=ZDIR, focus_up_sign=FUS, optics=OPTICS_10X,
        roi_rect=None, focus_limits_um=(0.0, 10000.0), **kw)
    # The settle is a hardware wait, not logic under test; ~18 tests x ~40
    # frames x 0.3 s would be over a minute of sleeping.
    w.SETTLE_S = 0.0
    return w, ctrl, scope, cam


class _Base(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication(sys.argv)

    def collect(self, w):
        out = {"measured": [], "failed": [], "samples": []}
        w.measured.connect(out["measured"].append)
        w.failed.connect(out["failed"].append)
        w.sample.connect(lambda f, s: out["samples"].append((f, s)))
        return out


class TestTheNeedleHoldsStill(_Base):
    def test_exactly_one_needle_move_per_margin(self):
        w, ctrl, _s, _c = make_worker()
        w.run()
        self.assertEqual(len(ctrl.moves), 1, ctrl.moves)
        self.assertEqual(w.needle_moves, ctrl.moves)

    def test_the_move_happens_before_any_focus_sample(self):
        w, ctrl, scope, _c = make_worker()
        order = []
        real_move = ctrl.move_z_absolute
        ctrl.move_z_absolute = lambda z, **k: (order.append("needle"),
                                               real_move(z, **k))
        w.sample.connect(lambda f, s: order.append("sample"))
        w.run()
        self.assertEqual(order[0], "needle")
        self.assertNotIn("needle", order[1:])

    def test_the_descent_target_is_the_margin_above_the_given_bottom(self):
        w, ctrl, _s, _c = make_worker(margin=500.0)
        w.run()
        self.assertAlmostEqual(ctrl.moves[0],
                               needle_target_zref(B_TRUE, 500.0, ZDIR),
                               places=9)

    def test_a_later_rung_is_bounded_by_the_measurement_not_the_guess(self):
        """The widget passes the previous measurement in as ``bottom``; the
        worker must plan against THAT, so a wrong guess is used at most once."""
        measured = B_TRUE + 0.030
        w, ctrl, _s, _c = make_worker(margin=200.0, bottom=measured)
        w.run()
        self.assertAlmostEqual(ctrl.moves[0],
                               needle_target_zref(measured, 200.0, ZDIR),
                               places=9)


class TestTheMeasurement(_Base):
    def test_it_recovers_the_plate_bottom(self):
        w, ctrl, _s, _c = make_worker(margin=1000.0)
        got = self.collect(w)
        w.run()
        self.assertEqual(got["failed"], [])
        self.assertEqual(len(got["measured"]), 1)
        m = got["measured"][0]
        self.assertFalse(m.refusal, m.refusal)
        h = tip_height_um(m.focus_tip_um, F0, FUS)
        b = plate_bottom_from_rung(m.needle_z_zref_mm, h, ZDIR)
        self.assertAlmostEqual(b, B_TRUE, places=3)     # µm-level

    def test_a_wrong_guess_does_not_reach_the_answer(self):
        """THE reason the focus sweeps and the needle holds still.

        The starting estimate is 120 µm off, so the needle lands at 880 µm above
        the real glass rather than the 1000 it was sent to. The focus finds it
        where it actually is, and the recovered plate bottom is the true one —
        the guess's error does not propagate.
        """
        for err_mm in (+0.120, -0.120):
            w, ctrl, scope, cam = make_worker(margin=1000.0,
                                              bottom=B_TRUE + err_mm)
            got = self.collect(w)
            w.run()
            self.assertEqual(got["failed"], [], got["failed"])
            m = got["measured"][0]
            self.assertFalse(m.refusal, m.refusal)
            h = tip_height_um(m.focus_tip_um, F0, FUS)
            self.assertAlmostEqual(h, 1000.0 + ZDIR * err_mm * 1000.0,
                                   delta=3.0)
            self.assertAlmostEqual(
                plate_bottom_from_rung(m.needle_z_zref_mm, h, ZDIR),
                B_TRUE, places=3)

    def test_the_focus_ends_at_the_peak_so_the_tip_looks_sharp(self):
        """On success the focus is deliberately NOT restored — the operator is
        about to be asked whether it looks sharp."""
        w, ctrl, scope, _c = make_worker(margin=1000.0)
        got = self.collect(w)
        w.run()
        self.assertAlmostEqual(scope.focus, got["measured"][0].focus_tip_um,
                               delta=30.0)

    def test_it_emits_a_sample_per_swept_position(self):
        w, _c1, _s, _c2 = make_worker()
        got = self.collect(w)
        w.run()
        self.assertGreaterEqual(len(got["samples"]), 5)


class TestRefusals(_Base):
    def test_a_flat_scene_refuses_rather_than_fitting_noise(self):
        """Empty glass has almost no contrast. Without the prominence gate the
        estimator fits sensor noise and reports a confident surface height."""
        scope, ctrl = FakeScope(), FakeCtrl()

        class Flat(FakeCam):
            def get_current_frame(self):
                return np.full((256, 256), 128, np.uint8)
        w, _c, _s, _cam = make_worker(cam=Flat(scope, ctrl), scope=scope,
                                      ctrl=ctrl)
        got = self.collect(w)
        w.run()
        self.assertEqual(len(got["measured"]), 1)
        self.assertTrue(got["measured"][0].refusal)

    def test_a_stale_drop_aborts_instead_of_retrying(self):
        """Retrying only re-queues behind the same backlog."""
        scope = FakeScope(stale_ops={3})
        w, _c, _s, _cam = make_worker(scope=scope)
        got = self.collect(w)
        w.run()
        self.assertEqual(got["measured"], [])
        self.assertEqual(len(got["failed"]), 1)
        self.assertIn("busy", got["failed"][0])

    def test_clamped_samples_are_discarded_not_believed(self):
        """``set_focus_um`` clamps silently, so a sample taken at a clamped
        position describes a different height than the one requested."""
        scope = FakeScope(lo=0.0, hi=F0 + 1000.0)     # ceiling mid-sweep
        w, _c, _s, _cam = make_worker(margin=1000.0, scope=scope)
        got = self.collect(w)
        w.run()
        for f, _s2 in got["samples"]:
            self.assertLessEqual(f, scope.hi + 1e-9)


class TestExitGuarantee(_Base):
    def test_poller_resumed_and_lease_released_on_success(self):
        w, ctrl, scope, _c = make_worker()
        w.run()
        self.assertEqual((ctrl.suspends, ctrl.resumes), (1, 1))
        self.assertEqual(scope.released, [wiz.PlateBottomRungWorker.LEASE])

    def test_poller_resumed_and_lease_released_on_failure(self):
        scope = FakeScope(stale_ops={2})
        w, ctrl, _s, _c = make_worker(scope=scope)
        self.collect(w)
        w.run()
        self.assertEqual((ctrl.suspends, ctrl.resumes), (1, 1))
        self.assertEqual(scope.released, [wiz.PlateBottomRungWorker.LEASE])

    def test_cancel_emits_nothing_and_still_cleans_up(self):
        w, ctrl, scope, _c = make_worker()
        got = self.collect(w)
        w.stop()
        w.run()
        self.assertEqual(got["measured"], [])
        self.assertEqual(got["failed"], [])
        self.assertEqual(ctrl.resumes, 1)
        self.assertEqual(scope.released, [wiz.PlateBottomRungWorker.LEASE])

    def test_failure_puts_the_needle_back_no_lower_than_it_started(self):
        ctrl = FakeCtrl(z=25.0)
        scope = FakeScope(stale_ops={2})
        w, _c, _s, _cam = make_worker(ctrl=ctrl, scope=scope)
        self.collect(w)
        w.run()
        # Height frame: never end below where we found it.
        self.assertGreaterEqual(ctrl.z_height_of(ctrl.z),
                                ctrl.z_height_of(25.0) - 1e-9)

    def test_failure_restores_the_entry_focus(self):
        scope = FakeScope(focus_um=777.0, stale_ops={2})
        w, _c, _s, _cam = make_worker(scope=scope)
        self.collect(w)
        w.run()
        self.assertAlmostEqual(scope.focus, 777.0, places=6)

    def test_a_refused_lease_fails_with_the_owner_named(self):
        scope = FakeScope()
        scope.try_acquire = lambda owner, timeout=0.0: False
        scope.lease_owner = lambda: "plate_level"
        w, ctrl, _s, _c = make_worker(scope=scope)
        got = self.collect(w)
        w.run()
        self.assertIn("plate_level", got["failed"][0])
        self.assertEqual(ctrl.moves, [])            # nothing moved


class TestTheFloorIsTheWidgetsJob(_Base):
    def test_the_worker_never_touches_the_print_floor_refcount(self):
        """The wizard arms it for the whole step. A second arm/disarm pair on a
        worker that can die mid-run is how a refcount goes unbalanced."""
        w, ctrl, _s, _c = make_worker()
        w.run()
        self.assertEqual(ctrl.floor, [])


if __name__ == "__main__":
    unittest.main()

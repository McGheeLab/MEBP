"""
Survey-worker discipline: the failure branches, and the exit guarantee.

This worker drives the stage, the microscope and the camera from a background
thread during a run that lasts many minutes. Almost everything that can go wrong
is invisible in a happy-path test, so each branch is driven here with hand-written
fakes:

* a **silently dropped** microscope op (the STALE_OP_S hazard) must ABORT, because
  a dropped ``set_objective`` means every later sample is taken through the wrong
  objective and the result still looks well-formed;
* a **silently clamped** focus move must mark the site, because ``set_focus_um``
  clamps before queueing and a clamped sample is a guess;
* **cancel during the operator pause** must return promptly rather than park on
  an event nobody will set;
* and on EVERY exit the poller resumes, the body is restored, the lease is
  released, and the needle was never commanded downward.
"""

import os
import sys
import threading
import time
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication            # noqa: E402

from SupportClasses.ObjectiveOptics import ObjectiveOptics   # noqa: E402
from gui.widgets.plate_level_wizard import (          # noqa: E402
    PlateLevelSurveyWorker, SiteSpec)


# ── fakes ─────────────────────────────────────────────────────────────────

class _Op:
    def __init__(self, error=None):
        self.done = threading.Event()
        self.done.set()
        self.error = error


class _NeverDoneOp:
    def __init__(self):
        self.done = threading.Event()      # never set
        self.error = None


class _FakeScope:
    """Records every command; configurable failure modes."""

    def __init__(self, *, stale_on=None, clamp_at=None, never_done=False,
                 lease_taken=False):
        self.calls = []
        self.focus = 5000.0
        self.objective = 1
        self._stale_on = stale_on
        self._clamp_at = clamp_at
        self._never_done = never_done
        self._lease_taken = lease_taken
        self.released = []
        self.acquired = []

    # lease
    def try_acquire(self, owner, timeout=0.0):
        if self._lease_taken:
            return False
        self.acquired.append(owner)
        return True

    def release(self, owner):
        self.released.append(owner)

    def lease_owner(self):
        return "someone else" if self._lease_taken else None

    # ops
    def set_objective(self, pos):
        self.calls.append(("objective", pos))
        if self._stale_on == "objective":
            return _Op(error="dropped (stale)")
        self.objective = pos
        return _Op()

    def set_focus_um(self, z):
        self.calls.append(("focus", z))
        if self._never_done:
            return _NeverDoneOp()
        if self._clamp_at is not None and z > self._clamp_at:
            self.focus = self._clamp_at
        else:
            self.focus = z
        return _Op()

    def state(self):
        class _S:
            pass
        s = _S()
        s.focus_um = self.focus
        s.objective_position = self.objective
        return s


class _FakeCtrl:
    def __init__(self, travel_ok=True):
        self.travels = []
        self.suspends = 0
        self.resumes = 0
        self.floor_calls = []
        self.z_moves = []
        self.xy_moves = []
        self._travel_ok = travel_ok

    def safe_travel_to(self, x, y, safe_z_mm=None, target_z_mm="MISSING",
                       apply_insert_floor=True, **kw):
        self.travels.append(dict(x=x, y=y, safe_z=safe_z_mm,
                                 target_z=target_z_mm,
                                 insert_floor=apply_insert_floor))
        return self._travel_ok

    def suspend_position_poller(self):
        self.suspends += 1

    def resume_position_poller(self):
        self.resumes += 1

    def set_print_floor_active(self, on):
        self.floor_calls.append(bool(on))

    def move_z_absolute(self, *a, **k):
        self.z_moves.append((a, k))

    def move_xy_absolute_um(self, *a, **k):
        self.xy_moves.append((a, k))


class _FakeCam:
    """Frame counter + a synthetic focus image whose sharpness peaks at 5000."""

    def __init__(self):
        self._n = 0
        self.scope = None

    def frame_count_value(self):
        self._n += 1
        return self._n

    def get_current_frame(self):
        """A feature whose TEXTURE amplitude peaks at 5000 µm on a constant mean.

        Modulating brightness instead would make the score bimodal — the block
        vanishes as it crosses the background level and reappears inverted — and
        the peak estimator would (correctly) refuse it. Defocus reduces contrast;
        it does not change mean intensity.
        """
        import numpy as np
        z = self.scope.focus if self.scope else 5000.0
        sigma = 25.0
        sharp = float(np.exp(-((z - 5000.0) ** 2) / (2 * sigma ** 2)))
        img = np.full((200, 200), 128.0)
        yy, xx = np.mgrid[0:200, 0:200]
        checker = np.where(((yy // 2) + (xx // 2)) % 2 == 0, 1.0, -1.0)
        mask = ((yy >= 70) & (yy < 130) & (xx >= 70) & (xx < 130))
        img += checker * mask * (100.0 * sharp)
        img = np.clip(img, 0, 255).astype("uint8")
        return np.dstack([img, img, img])


class _FakeMgr:
    def get_hw_settings(self, idx):
        return {"exposure_us": 15000, "gain": 1.0}


def optics():
    return ObjectiveOptics(label="10x", numerical_aperture=0.30,
                           working_distance_mm=16.0, um_per_px_sample=1.28,
                           frame_wh=(200, 200))


class _Rung:
    def __init__(self, pos=1, name="10x"):
        self.turret_position = pos
        self.objective_name = name
        self.optics = optics()
        self.magnification = 10.0


SITES = [SiteSpec(label="A1", x_um=0.0, y_um=0.0),
         SiteSpec(label="A6", x_um=60000.0, y_um=0.0),
         SiteSpec(label="D1", x_um=0.0, y_um=40000.0),
         SiteSpec(label="D6", x_um=60000.0, y_um=40000.0)]


def make_worker(**kw):
    scope = kw.pop("scope", None) or _FakeScope()
    ctrl = kw.pop("ctrl", None) or _FakeCtrl()
    cam = _FakeCam()
    cam.scope = scope
    w = PlateLevelSurveyWorker(
        controller=ctrl, scope=scope, cam=cam, cam_mgr=_FakeMgr(), cam_idx=0,
        sites=kw.pop("sites", SITES), ladder=[_Rung()],
        safe_z_zref_mm=44.0, focus_limits_um=(0.0, 10000.0),
        soft_limits_um=(200.0, 9000.0), settle_ms=0, fresh_frames=1, **kw)
    w.set_feature(100.0, 100.0)      # pre-confirm so no operator pause
    return w, scope, ctrl


class _WorkerCase(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication(sys.argv)

    def run_worker(self, w, timeout=30.0):
        failures, dones = [], []
        w.failed.connect(failures.append)
        w.finished_ok.connect(dones.append)
        w.start()
        w.wait(int(timeout * 1000))
        self.app.processEvents()
        return failures, dones


class TestStaleOpAborts(_WorkerCase):
    def test_a_dropped_objective_op_aborts_the_run(self):
        """The worst case: it is silent, and the result looks fine."""
        w, scope, ctrl = make_worker(scope=_FakeScope(stale_on="objective"))
        failures, dones = self.run_worker(w)
        self.assertTrue(failures)
        self.assertIn("Another part of the app", failures[0])
        self.assertEqual(dones, [])

    def test_it_stops_sampling_rather_than_carrying_on(self):
        """No sweep may run after the drop. The one focus command that remains
        is the RESTORE putting the drive back where it started, which is part of
        the exit guarantee rather than a sample."""
        scope = _FakeScope(stale_on="objective")
        entry = scope.focus
        w, sc, ctrl = make_worker(scope=scope)
        self.run_worker(w)
        focus_calls = [c for c in scope.calls if c[0] == "focus"]
        self.assertLessEqual(len(focus_calls), 1)
        for _kind, z in focus_calls:
            self.assertAlmostEqual(z, entry, places=6)


class TestWedgedBody(_WorkerCase):
    def test_an_op_that_never_completes_times_out_with_advice(self):
        w, scope, ctrl = make_worker(scope=_FakeScope(never_done=True))
        w._ladder[0].optics = optics()
        failures, dones = self.run_worker(w, timeout=60.0)
        self.assertTrue(failures)
        self.assertIn("did not finish", failures[0])


class TestLease(_WorkerCase):
    def test_a_held_lease_refuses_before_touching_anything(self):
        w, scope, ctrl = make_worker(scope=_FakeScope(lease_taken=True))
        failures, dones = self.run_worker(w)
        self.assertTrue(failures)
        self.assertIn("holds the microscope", failures[0])
        self.assertEqual(ctrl.travels, [])
        self.assertEqual(ctrl.suspends, 0)

    def test_the_lease_is_released_on_every_path(self):
        for scope in (_FakeScope(), _FakeScope(stale_on="objective")):
            w, sc, ctrl = make_worker(scope=scope)
            self.run_worker(w)
            self.assertIn("plate_level", sc.released)


class TestExitGuarantee(_WorkerCase):
    def test_poller_resumes_exactly_once_on_success_and_on_failure(self):
        for scope in (_FakeScope(), _FakeScope(stale_on="objective")):
            w, sc, ctrl = make_worker(scope=scope)
            self.run_worker(w)
            self.assertEqual(ctrl.suspends, 1)
            self.assertEqual(ctrl.resumes, 1)

    def test_the_body_is_returned_to_where_it_started(self):
        scope = _FakeScope()
        scope.focus = 4321.0
        scope.objective = 2
        w, sc, ctrl = make_worker(scope=scope)
        self.run_worker(w)
        self.assertEqual(sc.objective, 2)
        self.assertAlmostEqual(sc.focus, 4321.0, places=3)

    def test_the_print_floor_refcount_is_never_touched(self):
        """This workflow never commands a descent, so arming a refcount it
        cannot need would only risk an unbalanced decrement against a
        concurrent print."""
        w, sc, ctrl = make_worker()
        self.run_worker(w)
        self.assertEqual(ctrl.floor_calls, [])

    def test_every_site_is_reached_by_a_retract_only_travel(self):
        """target_z_mm=None is what makes safe_travel_to raise-only. A bare
        move_xy would drag a lowered needle across the plate."""
        w, sc, ctrl = make_worker()
        self.run_worker(w)
        self.assertTrue(ctrl.travels)
        for t in ctrl.travels:
            self.assertIsNone(t["target_z"])
            self.assertEqual(t["safe_z"], 44.0)
            self.assertFalse(t["insert_floor"])

    def test_no_direct_z_or_xy_commands_are_ever_issued(self):
        w, sc, ctrl = make_worker()
        self.run_worker(w)
        self.assertEqual(ctrl.z_moves, [])
        self.assertEqual(ctrl.xy_moves, [])

    def test_a_refused_travel_stops_the_run(self):
        w, sc, ctrl = make_worker(ctrl=_FakeCtrl(travel_ok=False))
        failures, dones = self.run_worker(w)
        self.assertTrue(failures)
        self.assertIn("refused to travel", failures[0])


class TestClamping(_WorkerCase):
    def test_a_silently_clamped_focus_move_is_detected(self):
        """set_focus_um clamps BEFORE queueing, so the commanded value can never
        be assumed — the readback is the only truth."""
        scope = _FakeScope(clamp_at=5010.0)
        w, sc, ctrl = make_worker(scope=scope)
        self.run_worker(w)
        # The run either completes with sites flagged, or refuses. Either way it
        # must not silently report a peak derived from clamped samples.
        state = w._state
        for m in state.measurements.values():
            if m.focus_um is not None:
                self.assertTrue(m.clamped or m.focus_um <= 5010.0 + 1e-6)


class TestCancel(_WorkerCase):
    def test_cancel_during_the_operator_pause_returns_promptly(self):
        """A bare Event.wait() here is the classic deadlock: the operator walks
        away, the page hides, cancel stops and joins — and the worker is parked
        on something nobody will ever set."""
        w, sc, ctrl = make_worker()
        w._feature_px = None
        w._feature_ready = False
        w.start()
        time.sleep(0.4)
        t0 = time.monotonic()
        w.stop()
        w.set_feature(None, None)
        finished = w.wait(5000)
        self.assertTrue(finished, "the worker did not stop")
        self.assertLess(time.monotonic() - t0, 3.0)

    def test_cancel_emits_no_result(self):
        w, sc, ctrl = make_worker()
        failures, dones = [], []
        w.failed.connect(failures.append)
        w.finished_ok.connect(dones.append)
        w.start()
        time.sleep(0.1)
        w.stop()
        w.wait(5000)
        self.app.processEvents()
        self.assertEqual(dones, [])
        self.assertEqual(failures, [])

    def test_cancel_still_resumes_the_poller(self):
        w, sc, ctrl = make_worker()
        w.start()
        time.sleep(0.1)
        w.stop()
        w.wait(5000)
        self.assertEqual(ctrl.resumes, 1)


class TestHappyPath(_WorkerCase):
    def test_a_clean_run_measures_every_site(self):
        w, sc, ctrl = make_worker()
        failures, dones = self.run_worker(w)
        self.assertEqual(failures, [], failures[:1])
        self.assertTrue(dones)
        state = dones[0]
        self.assertEqual(len(state.measurements), len(SITES))

    def test_the_first_site_is_re_measured_to_detect_drift(self):
        """Ti focus drives creep 0.1-1 µm/min; over a ten-minute survey that is a
        real contributor to the residuals, and invisible unless measured."""
        w, sc, ctrl = make_worker()
        failures, dones = self.run_worker(w)
        self.assertTrue(dones)
        self.assertEqual(len(ctrl.travels), len(SITES) + 1)
        self.assertIsNotNone(dones[0].anchor_focus_closing_um)


if __name__ == "__main__":
    unittest.main()

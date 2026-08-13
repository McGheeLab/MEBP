"""test_v718_mosaic_preflight_and_dialog_leak.py — two operator reports.

Operator: *"1. There was an error code when doing the mosaic builder and
prevented me from calibrating the objective 2. The software became laggy/slow
over time."*

BOTH root-caused from the operator's own ``logs/app.log``, and they share a
cause — the GUI thread being progressively starved by leaked dialogs, which also
starves the display timer that advances the mosaic's fresh-frame counter.

1. **The mosaic error.** Every tile was dropped with *"no new camera frame
   within 2.5 s after the move (needed 3)"*, 8 in a row, then *"camera stopped
   delivering frames — scan aborted"*. The Tucsen runs a **300 ms exposure**
   (``'exposure_us': 299999.4`` in the operator's saved ``hw_controls``) ≈ 2.4
   fps, so three frames need ~1.3 s; the log's **2.5 s** proves the adaptive
   timeout never engaged, i.e. ``resolve_grab_timing`` could not read the
   exposure and fell back to the flat configured value.
2. **The lag.** Widget count over one session: **8201 → 9072**, in steps of
   ~82–204, while the 300 ms status tick stretched from ~63 s per 200 ticks to
   **203 s**. Dialogs are created ``parent=self`` and ``exec()``d with no
   deletion, so each one stays resident with its whole tree — live camera feed
   views included, still doing per-frame GUI work.
"""

import os
import time
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtCore import QEvent, QObject
from PySide6.QtWidgets import QApplication, QDialog, QLabel, QVBoxLayout, QWidget

from SupportClasses.CaptureTiming import fresh_frame_timeout_s

_app = QApplication.instance() or QApplication([])


def _flush_deletes():
    """Run the deferred-delete pass deleteLater() schedules."""
    _app.sendPostedEvents(None, QEvent.DeferredDelete)
    _app.processEvents()


# ── 1. The dialog leak (the lag) ────────────────────────────────────

class _HeavyDialog(QDialog):
    """Stands in for a calibration dialog: a non-trivial widget tree."""

    N_CHILDREN = 40

    def __init__(self, parent=None):
        super().__init__(parent)
        lay = QVBoxLayout(self)
        for i in range(self.N_CHILDREN):
            lay.addWidget(QLabel(f"row {i}", self))

    def exec(self):            # never actually block a test
        return QDialog.DialogCode.Accepted


class TestExecDialogReleasesTheTree(unittest.TestCase):
    def setUp(self):
        self.host = QWidget()
        self.addCleanup(self.host.deleteLater)

    def _count(self):
        return len(self.host.findChildren(QObject))

    def test_the_leak_this_fixes_is_real(self):
        """⚠ GUARD THE GUARD. If a bare exec() did not leak, the fix below
        would be proving nothing. Reproduce the operator's growth first."""
        base = self._count()
        for _ in range(5):
            dlg = _HeavyDialog(parent=self.host)
            dlg.exec()                      # the shipped pattern: no disposal
        _flush_deletes()
        grew = self._count() - base
        self.assertGreater(
            grew, 5 * _HeavyDialog.N_CHILDREN,
            "a parented dialog should survive exec() — if this fails the "
            "premise of the fix is wrong, not the fix")

    def test_exec_dialog_returns_the_tree_to_baseline(self):
        from gui.widgets.components import exec_dialog

        base = self._count()
        for _ in range(5):
            dlg = _HeavyDialog(parent=self.host)
            self.assertEqual(exec_dialog(dlg), QDialog.DialogCode.Accepted)
        _flush_deletes()
        self.assertEqual(self._count(), base)

    def test_the_result_is_still_readable_after_exec(self):
        """Deletion MUST be deferred: callers read dlg.result_* / dlg.values()
        after exec() returns, and those touch live child widgets."""
        from gui.widgets.components import exec_dialog

        class _WithResult(_HeavyDialog):
            def values(self):
                return self.findChildren(QLabel)[0].text()

        dlg = _WithResult(parent=self.host)
        exec_dialog(dlg)
        self.assertEqual(dlg.values(), "row 0")   # not yet destroyed
        _flush_deletes()

    def test_a_raising_dialog_is_still_released(self):
        from gui.widgets.components import exec_dialog

        class _Boom(_HeavyDialog):
            def exec(self):
                raise RuntimeError("dialog blew up")

        base = self._count()
        dlg = _Boom(parent=self.host)
        with self.assertRaises(RuntimeError):
            exec_dialog(dlg)
        _flush_deletes()
        self.assertEqual(self._count(), base)


class TestHotSitesUseIt(unittest.TestCase):
    """The measured leak was in the camera-calibration path; pin those sites.

    AST, not a substring: a comment mentioning exec_dialog would satisfy `in
    source`, which is a trap this repo has already been bitten by.
    """

    def _bare_exec_calls(self, module_path, allowed=()):
        import ast
        with open(module_path, encoding="utf-8") as fh:
            tree = ast.parse(fh.read())
        bad = []
        for node in ast.walk(tree):
            if not isinstance(node, ast.Call):
                continue
            fn = node.func
            if (isinstance(fn, ast.Attribute) and fn.attr == "exec"
                    and isinstance(fn.value, ast.Name)
                    and fn.value.id not in allowed):
                bad.append((fn.value.id, node.lineno))
        return bad

    def test_objective_calibration_card_disposes_every_dialog(self):
        import gui.pages.hardware.objective_calibration_card as mod
        self.assertEqual(self._bare_exec_calls(mod.__file__), [])

    def test_scale_fov_dialog_disposes_its_verify_mosaic(self):
        import gui.dialogs.scale_fov_calibration_dialog as mod
        self.assertEqual(self._bare_exec_calls(mod.__file__), [])


# ── 2. The mosaic preflight (the error code) ────────────────────────

class _FakeCam:
    """Display-gated frame counter, like CameraWidget.frame_count_value().

    ``fps`` is what the DISPLAY timer publishes; ``backend_fps`` is what the
    sensor delivers. They differ in exactly the case that broke this rig.
    """

    def __init__(self, fps=10.0, backend_fps=None, hw=None):
        self._fps = float(fps)
        self._backend_fps = (self._fps if backend_fps is None
                             else float(backend_fps))
        self._t0 = time.time()
        self._hw = hw

    def frame_count_value(self):
        if self._fps <= 0:
            return 0
        return int((time.time() - self._t0) * self._fps)

    def frames_acquired(self):
        if self._backend_fps <= 0:
            return 0
        return int((time.time() - self._t0) * self._backend_fps)

    def get_current_frame(self):
        return object()

    def get_hw_settings(self):
        return dict(self._hw) if self._hw else {"source": "none"}


def _worker(cam, **kw):
    """A worker with only what the preflight/grab path touches."""
    from gui.pages.calibration import _MosaicScanWorker
    w = _MosaicScanWorker.__new__(_MosaicScanWorker)
    w._cam = cam
    w._stop = False
    w._settle_ms = 0
    w._fresh_frames = kw.get("fresh_frames", 3)
    w._fresh_timeout_s = kw.get("fresh_timeout_s", 2.5)
    w._avg_frames = 1
    w._measured_period_s = None
    w._last_wait_timeout_s = None
    return w


class TestPreflight(unittest.TestCase):
    def test_a_healthy_camera_passes_and_its_rate_is_measured(self):
        w = _worker(_FakeCam(fps=20.0))
        self.assertIsNone(w._preflight_camera())
        self.assertIsNotNone(w._measured_period_s)
        self.assertAlmostEqual(w._measured_period_s, 0.05, delta=0.04)

    def test_a_dead_feed_that_is_still_streaming_names_the_live_view(self):
        """⚠ THE OPERATOR'S CASE. frame_count_value() is advanced by the display
        timer, so a stopped/hidden feed freezes it while the sensor streams on.
        That is a software state and the message must say so — not blame the
        camera."""
        w = _worker(_FakeCam(fps=0.0, backend_fps=20.0))
        w._PREFLIGHT_WAIT_S = 0.4
        why = w._preflight_camera()
        self.assertIsNotNone(why)
        self.assertIn("live view is not updating", why)
        self.assertIn("preview", why)

    def test_a_truly_silent_camera_says_so(self):
        w = _worker(_FakeCam(fps=0.0, backend_fps=0.0))
        w._PREFLIGHT_WAIT_S = 0.4
        why = w._preflight_camera()
        self.assertIsNotNone(why)
        self.assertIn("not delivering frames", why)

    def test_preflight_refuses_before_any_stage_motion(self):
        """The whole point: fail in ~2 s with a reason, not after 8 moves."""
        moves = []

        class _Ctrl:
            def suspend_position_poller(self):
                pass

            def resume_position_poller(self):
                pass

            def safe_travel_to(self, *a, **k):
                moves.append(a)
                return True

        from gui.pages.calibration import _MosaicScanWorker
        w = _worker(_FakeCam(fps=0.0, backend_fps=0.0))
        w._PREFLIGHT_WAIT_S = 0.3
        w._controller = _Ctrl()
        w._positions = [(0, 0), (100, 0), (200, 0)]
        w.unreachable_skipped = 0
        failures = []
        w.failed = type("S", (), {"emit": staticmethod(failures.append)})()
        w.progress = type("S", (), {"emit": staticmethod(lambda *a: None)})()
        _MosaicScanWorker.run(w)
        self.assertEqual(moves, [], "the stage must not move on a refusal")
        self.assertEqual(len(failures), 1)
        self.assertIn("cannot start the scan", failures[0])


class TestTimeoutUsesTheMeasuredRate(unittest.TestCase):
    def test_the_operators_numbers_reproduce_the_bug(self):
        """300 ms exposure ⇒ ~2.4 fps. Three frames cannot land in 2.5 s with
        any margin, and the flat timeout is what the log shows was used."""
        period = (0.3 + 0.030) * 1.25          # CaptureTiming's own estimate
        self.assertAlmostEqual(period, 0.4125, places=4)
        self.assertGreater(3 * period, 1.2)
        # With the rate known the timeout becomes generous instead of 2.5 s.
        self.assertGreater(fresh_frame_timeout_s(3, period, 2.5), 4.0)

    def test_measured_period_extends_the_timeout_when_the_sdk_is_silent(self):
        """get_hw_settings() reports nothing usable — exactly what produced the
        flat 2.5 s — so the MEASURED rate must size the wait instead."""
        cam = _FakeCam(fps=2.4)                # no hw settings => source none
        w = _worker(cam, fresh_frames=3, fresh_timeout_s=2.5)
        w._measured_period_s = 1.0 / 2.4
        frame = w._grab_post_move_frame()
        self.assertIsNotNone(frame, "three frames at 2.4 fps must be waited for")
        self.assertGreater(w._last_wait_timeout_s, 2.5)

    def test_a_reported_exposure_still_wins(self):
        """When the backend CAN report, nothing changes — the measured value is
        only a fallback, so a rig that works today is unaffected.

        ⚠ The sentinel must be INSIDE CaptureTiming's plausible band. A first
        cut used 99.0 s, which exceeds MAX_PERIOD_S (60) and is discarded — so
        the test could not see the guard being removed at all, and a mutation
        deleting it survived."""
        cam = _FakeCam(fps=20.0, hw={"source": "tucam", "exposure_us": 300000.0})
        w = _worker(cam, fresh_frames=3, fresh_timeout_s=2.5)
        w._measured_period_s = 3.0             # plausible, and would give ~23 s
        self.assertIsNotNone(w._grab_post_move_frame())
        # The reported 300 ms exposure gives ~4.1 s; the measured 3.0 s would
        # give ~23.5 s. Anything above 10 means the fallback wrongly took over.
        self.assertLess(w._last_wait_timeout_s, 10.0)


if __name__ == "__main__":
    unittest.main()

"""Tests for the Quick Print simulation overlay + realtime progress (v7.5.x):
PrintTrajectoryMonitorView.set_predicted_path / progress tracking, and the
QuickPrintWorkflowPage prediction bridge plumbing. Offscreen Qt, no hardware.
"""

import os
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication

_app = QApplication.instance() or QApplication([])

from gui.widgets.print_trajectory_monitor import PrintTrajectoryMonitorView


def _square_um(size_um=2000.0):
    h = size_um / 2.0
    return [(-h, -h), (h, -h), (h, h), (-h, h), (-h, -h)]


class TestPredictedOverlay(unittest.TestCase):
    def test_set_predicted_path_and_note(self):
        v = PrintTrajectoryMonitorView()
        v.set_planned_path([_square_um()])
        v.set_predicted_path([_square_um(1900.0)], "sim: p95 84 µm")
        self.assertEqual(len(v._predicted), 1)
        self.assertEqual(v._predicted_note, "sim: p95 84 µm")
        v.set_predicted_path(None)
        self.assertEqual(v._predicted, [])
        self.assertEqual(v._predicted_note, "")

    def test_predicted_included_in_bbox(self):
        v = PrintTrajectoryMonitorView()
        v.set_planned_path([_square_um(1000.0)])
        x0, y0, x1, y1 = v._bbox
        v.set_predicted_path([_square_um(4000.0)])
        nx0, ny0, nx1, ny1 = v._bbox
        self.assertLess(nx0, x0)
        self.assertGreater(nx1, x1)

    def test_single_point_predicted_segments_dropped(self):
        v = PrintTrajectoryMonitorView()
        v.set_predicted_path([[(0.0, 0.0)], _square_um()])
        self.assertEqual(len(v._predicted), 1)


class TestLiveProgress(unittest.TestCase):
    def _view(self):
        v = PrintTrajectoryMonitorView()
        # two 1 mm strokes with a travel between them
        v.set_planned_path([[(0.0, 0.0), (1000.0, 0.0)],
                            [(0.0, 500.0), (1000.0, 500.0)]])
        v.set_recording(True)
        return v

    def test_progress_advances_monotonically_to_one(self):
        v = self._view()
        self.assertEqual(v.progress_fraction(), 0.0)
        fracs = []
        for x in range(0, 1001, 100):        # stroke 1
            v.set_position(float(x), 0.0)
            fracs.append(v.progress_fraction())
        self.assertAlmostEqual(fracs[-1], 0.5, delta=0.02)
        for x in range(0, 1001, 100):        # stroke 2 (after the travel)
            v.set_position(float(x), 500.0)
            fracs.append(v.progress_fraction())
        self.assertAlmostEqual(fracs[-1], 1.0, delta=0.02)
        self.assertEqual(fracs, sorted(fracs))    # monotone

    def test_deviation_tracked(self):
        v = self._view()
        v.set_position(0.0, 0.0)
        v.set_position(500.0, 40.0)          # 40 µm off the stroke
        self.assertAlmostEqual(v._prog["dev_um"], 40.0, delta=1.0)

    def test_reset_live_resets_progress(self):
        v = self._view()
        for x in range(0, 1001, 100):
            v.set_position(float(x), 0.0)
        self.assertGreater(v.progress_fraction(), 0.4)
        v.reset_live()
        self.assertEqual(v.progress_fraction(), 0.0)

    def test_no_plan_no_progress(self):
        v = PrintTrajectoryMonitorView()
        v.set_recording(True)
        v.set_position(10.0, 10.0)
        self.assertIsNone(v.progress_fraction())


class TestQuickPrintBridgePlumbing(unittest.TestCase):
    def test_bridge_has_predicted_signal_and_gen_guard(self):
        from gui.pages.workflows.quick_print_workflow import (
            QuickPrintWorkflowPage, _PrintBridge)
        self.assertTrue(hasattr(_PrintBridge, "predicted"))

        page = QuickPrintWorkflowPage.__new__(QuickPrintWorkflowPage)
        calls = []

        class FakeView:
            # v7.7: the view also takes the prediction as a NUMBER so the live
            # readout can show it beside the measured deviation mid-print.
            def set_predicted_path(self, segs, note="", predicted_p95_um=None):
                calls.append((segs, note, predicted_p95_um))

            def set_overlay(self, *_a, **_k):
                pass                     # v7.6: applied after each prediction
        page._traj_view = FakeView()
        page._overlay_group = None       # v7.6: no pills → overlay "none"
        page._pred_gen = 3
        page._on_predicted(2, [[(0, 0), (1, 1)]], "stale")   # old generation
        self.assertEqual(calls, [])
        page._on_predicted(3, [[(0, 0), (1, 1)]], "fresh")
        self.assertEqual(calls, [([[(0, 0), (1, 1)]], "fresh", None)])

    def test_predicted_p95_is_forwarded_and_kept_out_of_the_overlays(self):
        """v7.7: the worker rides the predicted p95 in the overlay payload under
        a reserved key. It must reach the view AND be removed before the payload
        is stored, or the overlay pills would see a bogus channel."""
        from gui.pages.workflows.quick_print_workflow import \
            QuickPrintWorkflowPage
        page = QuickPrintWorkflowPage.__new__(QuickPrintWorkflowPage)
        got = {}

        class FakeView:
            def set_predicted_path(self, segs, note="", predicted_p95_um=None):
                got["p95"] = predicted_p95_um

            def set_overlay(self, *_a, **_k):
                pass
        page._traj_view = FakeView()
        page._overlay_group = None
        page._pred_gen = 1
        payload = {"speed": {"values": [[1.0]], "unit": "mm/s",
                             "vmin": 1.0, "vmax": 1.0},
                   "_pred_p95_um": 7.3}
        page._on_predicted(1, [[(0, 0), (1, 1)]], "note", payload)
        self.assertAlmostEqual(got["p95"], 7.3)
        self.assertNotIn("_pred_p95_um", page._pred_overlays)
        self.assertIn("speed", page._pred_overlays)


if __name__ == "__main__":
    unittest.main()

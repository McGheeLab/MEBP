"""v7.5.x tests — Sketch page XZ side-profile view (changes_needed item 4).

The Print Builder Sketch page gains a resizable XZ side-profile (elevation) of
the drawn print, driven by the same compiled trajectory + pump_states the 2D
preview uses. These tests cover the standalone ``SketchProfileView`` widget and
its wiring into ``SketchPage``.
"""
import os
import sys
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import numpy as np
from PySide6.QtWidgets import QApplication

from gui.widgets.sketch_profile_view import SketchProfileView
from SupportClasses.SketchTrajectory import (
    Sketch, SketchShape, compile_to_trajectory,
)


class _QtBase(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls._app = QApplication.instance() or QApplication(sys.argv)


class TestSketchProfileView(_QtBase):
    def _two_shape_traj(self, layers=2):
        sk = Sketch(num_layers=layers)
        sk.shapes = [SketchShape(kind="circle", cx=-4, cy=0, radius=2, filled=True),
                     SketchShape(kind="circle", cx=4, cy=0, radius=2, filled=True)]
        return compile_to_trajectory(sk)

    def test_empty_by_default(self):
        v = SketchProfileView()
        self.assertIsNone(v._traj)
        v.resize(400, 160)
        v.grab()  # must paint the empty placeholder without error

    def test_set_trajectory_extents_include_floor(self):
        v = SketchProfileView()
        res = self._two_shape_traj(layers=2)
        v.set_trajectory(res.trajectory, res.pump_states)
        self.assertIsNotNone(v._traj)
        # z extent always includes the plate floor (z=0) and the travel lift.
        self.assertLessEqual(v._zmin, 0.0)
        self.assertGreater(v._zmax, 0.0)
        self.assertLess(v._xmin, v._xmax)

    def test_renders_with_data(self):
        v = SketchProfileView()
        res = self._two_shape_traj()
        v.set_trajectory(res.trajectory, res.pump_states)
        v.resize(420, 180)
        pm = v.grab()
        self.assertEqual(pm.width(), 420)

    def test_clear(self):
        v = SketchProfileView()
        res = self._two_shape_traj()
        v.set_trajectory(res.trajectory, res.pump_states)
        v.clear()
        self.assertIsNone(v._traj)

    def test_rejects_degenerate_trajectory(self):
        v = SketchProfileView()
        v.set_trajectory(np.zeros((1, 7)))   # < 2 rows
        self.assertIsNone(v._traj)
        v.set_trajectory(np.zeros((5, 2)))   # < 3 cols (no Z)
        self.assertIsNone(v._traj)

    def test_seg_is_print_uses_pump_states(self):
        v = SketchProfileView()
        # 3 waypoints; segment 0 prints, segment 1 travels.
        traj = np.array([[0, 0, 0.2, 0, 0, 0, 0],
                         [1, 0, 0.2, 1, 0, 0, 1],
                         [2, 0, 2.0, 1, 0, 0, 2]], dtype=float)
        v.set_trajectory(traj, [[1.0, 0, 0], [0.0, 0, 0], [0, 0, 0]])
        self.assertTrue(v._seg_is_print(0))
        self.assertFalse(v._seg_is_print(1))


class TestSketchPageWiring(_QtBase):
    def test_page_has_profile_view_and_feeds_it(self):
        from gui.pages.print_builder_sketch import SketchPage
        page = SketchPage()
        self.assertTrue(hasattr(page, "_profile_view"))
        sk = Sketch(num_layers=1)
        sk.shapes = [SketchShape(kind="circle", cx=0, cy=0, radius=3, filled=False)]
        page._canvas.set_sketch(sk)
        page._recompute_preview()
        self.assertIsNotNone(page._profile_view._traj)

    def test_empty_sketch_clears_profile(self):
        from gui.pages.print_builder_sketch import SketchPage
        page = SketchPage()
        page._canvas.set_sketch(Sketch())  # empty
        page._recompute_preview()
        self.assertIsNone(page._profile_view._traj)


if __name__ == "__main__":
    unittest.main()

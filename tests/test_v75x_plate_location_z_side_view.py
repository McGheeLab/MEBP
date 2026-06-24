"""
v7.5.x — Z side view on the Calibration → Plate Location tab.

Adds an ``XZSideView`` next to the well-layout map so the operator can
retract the needle up to safe travel manually (click the green "Safe"
badge → drives Z to the Fast Move height via ``move_z_absolute``).

These tests build a real ``CalibrationPage`` offscreen and verify:
  * the side view is present on the Plate Location tab;
  * its ``go_to_z_requested`` badge routes to ``move_z_absolute`` in the
    zero-ref move frame (so the move contract / soft limits hold);
  * a disconnected ZP stage is a no-op (no motion);
  * captured Z references (Safe badge) propagate into the view;
  * the periodic status tick feeds the view without raising.
"""

import os
import sys
import unittest
from unittest.mock import MagicMock

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication  # noqa: E402

from gui.pages.calibration import CalibrationPage  # noqa: E402
from gui.widgets.xz_side_view import XZSideView  # noqa: E402


class _ZSideViewBase(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls._app = QApplication.instance() or QApplication(sys.argv)

    def _make_page(self, zp_connected=True):
        ctrl = MagicMock()
        ctrl.is_xy_connected = False
        ctrl.is_zp_connected = zp_connected
        ctrl.zero_position = {"x": 0, "y": 0, "Z": 0}
        ctrl.get_xy_position.return_value = (1000.0, 2000.0)
        ctrl.get_zp_position.return_value = (5.0, 0.0, 0.0)
        ctrl.zp_logical_value.return_value = 5.0
        ctrl.z_up_sign.return_value = -1.0
        ctrl.raw_to_user_z.side_effect = lambda v: -v
        self._moves = []
        ctrl.move_z_absolute.side_effect = (
            lambda z, from_zero_ref=False: self._moves.append((z, from_zero_ref)))
        return CalibrationPage(ctrl, settings=None)


class TestZSideViewPresence(_ZSideViewBase):
    def test_side_view_built_on_plate_location_tab(self):
        page = self._make_page()
        self.assertTrue(hasattr(page, "_ploc_xz_view"))
        self.assertIsInstance(page._ploc_xz_view, XZSideView)

    def test_display_sign_follows_controller_z_up_sign(self):
        page = self._make_page()
        # set_z_display_sign normalises to ±1; ME3B V1 z_up_sign = -1.
        self.assertEqual(page._ploc_xz_view._z_disp_sign, -1.0)


class TestGoToZBadge(_ZSideViewBase):
    def test_badge_routes_to_move_z_absolute_zero_ref(self):
        page = self._make_page(zp_connected=True)
        page._ploc_xz_view.go_to_z_requested.emit(8.0)
        self.assertEqual(self._moves, [(8.0, True)])

    def test_disconnected_zp_is_noop(self):
        page = self._make_page(zp_connected=False)
        page._ploc_xz_view.go_to_z_requested.emit(8.0)
        self.assertEqual(self._moves, [])


class TestReferencePropagation(_ZSideViewBase):
    def test_safe_z_reference_pushed_to_view(self):
        page = self._make_page()
        page._safe_z = 8.0
        page._refresh_ploc_view()
        self.assertEqual(page._ploc_xz_view._z_refs.get("fast_move_z"), 8.0)

    def test_status_tick_feeds_position_without_error(self):
        page = self._make_page()
        # Should not raise; pushes live Z (5.0 raw, zero-ref) into the view.
        page.on_status_update()
        self.assertEqual(page._ploc_xz_view._z_mm, 5.0)


if __name__ == "__main__":
    unittest.main()

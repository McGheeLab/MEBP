"""v7.5.x tests — Z side view on the Needle Offset Calibration reference-Z card
(changes_needed item 3).

Mirrors the Plate Location Z side view: an XZSideView beside the "Reference Z
heights" capture grid so the operator can move Z quickly (the green "Safe" badge
retracts to Fast Move Z; other badges drive to each reference). Reuses the
existing ``_on_ploc_go_to_z`` handler.
"""
import os
import sys
import unittest
from unittest.mock import MagicMock

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication

from gui.pages.calibration import CalibrationPage


class _Base(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls._app = QApplication.instance() or QApplication(sys.argv)

    def _page(self, zp_connected=True):
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


class TestNeedleOffsetZSideView(_Base):
    def test_view_present(self):
        page = self._page()
        self.assertTrue(hasattr(page, "_zoff_xz_view"))
        self.assertIsNotNone(page._zoff_xz_view)

    def test_status_update_feeds_view(self):
        page = self._page()
        page.on_status_update()   # must push live position without raising

    def test_safe_badge_moves_z_zero_ref(self):
        page = self._page(zp_connected=True)
        page._zoff_xz_view.go_to_z_requested.emit(40.0)
        self.assertTrue(self._moves)
        z, from_zero_ref = self._moves[-1]
        self.assertEqual(z, 40.0)
        self.assertTrue(from_zero_ref)

    def test_go_to_z_noop_when_zp_disconnected(self):
        page = self._page(zp_connected=False)
        page._zoff_xz_view.go_to_z_requested.emit(40.0)
        self.assertEqual(self._moves, [])

    def test_view_receives_z_references(self):
        page = self._page()
        # get_z_references returns the 5-key dict the badges render from.
        refs = page.get_z_references()
        self.assertIsInstance(refs, dict)
        # Feeding the live view with references must not raise.
        page.on_status_update()


if __name__ == "__main__":
    unittest.main()

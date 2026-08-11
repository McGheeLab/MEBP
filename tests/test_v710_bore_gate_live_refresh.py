"""
v7.10 — the bore-offset gate is re-evaluated live (BUG A).

THE OPERATOR'S REPORT
---------------------
"it has a warning to connect the xy stage, but i can confirm the xy stage is
connected."

ROOT CAUSE — and what it is NOT. ``StageController.is_xy_connected`` is a proper
``@property`` returning ``xy_stage is not None``, used bare, so there is no
bound-method-truthiness bug. ``CalibrationPage.controller`` is assigned once in
``__init__`` from the same long-lived controller the main window polls, so it is
not stale either. Both plausible culprits are innocent.

The actual cause is *when* the gate runs. ``_bore_cal_refresh`` had exactly four
callers — page build, ``set_hardware_config``, after a capture, after a clear —
and none of them fire when the XY stage connects. The calibration page is
constructed at startup, BEFORE the operator connects the hardware, so the gate
evaluated once against a disconnected stage, painted the yellow refusal, called
``setEnabled(False)`` on every row button, and nothing ever re-ran it. The tab
stayed dead for the whole session; the only accidental cure was re-saving a
hardware config, which happens to call ``set_hardware_config``.

THE FIX is a cheap change key checked on the existing ~300 ms page tick, so the
rows rebuild on a real transition and not 3× a second.
"""

import os
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock

from PySide6.QtWidgets import QPushButton

from SupportClasses.PhysicalModels import NeedleBore, NeedleSpec
from gui.pages.calibration import CalibrationPage


def _backpack() -> NeedleSpec:
    """Two fused bores on different pumps — the smallest genuine multi-bore."""
    return NeedleSpec(
        needle_form="backpack",
        bores=[
            NeedleBore(id_um=413, od_um=718, length_mm=50.8, pump_id="P1"),
            NeedleBore(id_um=159, od_um=305, length_mm=50.8, pump_id="P2"),
        ],
    )


class _PageBase(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
        import sys
        from PySide6.QtWidgets import QApplication
        cls._app = QApplication.instance() or QApplication(sys.argv)

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory(prefix="mebp_bore_gate_")
        self.addCleanup(self._tmp.cleanup)
        self._prev = os.environ.get("MEBP_NEEDLE_BORE_CAL_PATH")
        os.environ["MEBP_NEEDLE_BORE_CAL_PATH"] = str(
            Path(self._tmp.name) / "bore.json")
        import SupportClasses.NeedleBoreCalibrationStore as mod
        self._prev_singleton = mod._store
        mod._store = None

        def _restore():
            mod._store = self._prev_singleton
            if self._prev is None:
                os.environ.pop("MEBP_NEEDLE_BORE_CAL_PATH", None)
            else:
                os.environ["MEBP_NEEDLE_BORE_CAL_PATH"] = self._prev
        self.addCleanup(_restore)

    def _page(self, *, xy_connected: bool):
        """The real page, built the way the app builds it at startup."""
        ctrl = MagicMock()
        ctrl.is_xy_connected = xy_connected
        ctrl.is_zp_connected = False
        ctrl.zero_position = {"x": 0, "y": 0, "Z": 0}
        ctrl.get_xy_position.return_value = (1000.0, 2000.0)
        ctrl.z_up_sign.return_value = 1.0
        # on_status_update formats these, so a bare MagicMock would raise on
        # __format__ — give the readout path real numbers.
        ctrl.get_zp_position.return_value = (0.0, 0.0, 0.0, 0.0)
        ctrl.zp_logical_value.return_value = 0.0
        ctrl.raw_to_user_z.return_value = 0.0
        page = CalibrationPage(ctrl, settings=None)
        page._hardware_config = SimpleNamespace(needle=_backpack())
        # Headless builds have no cameras, so stand in for two live, calibrated
        # needle cams; otherwise THAT refusal (correctly) wins the status line
        # and we would not be testing the XY clause at all.
        page._needle_loc_camera_info = lambda role: (1.67, 640, 45.0)
        return page

    @staticmethod
    def _row_buttons(page):
        return [b for w in page._bore_cal_row_widgets
                for b in w.findChildren(QPushButton)]


class TestStaleWarningClears(_PageBase):

    def test_the_operator_sequence_build_disconnected_then_connect(self):
        """THE BUG, end to end.

        Build the page with XY down (what happens at app startup), then connect
        the stage, then let one tick run. The refusal must clear and the buttons
        must enable — WITHOUT a set_hardware_config push, which was the only
        thing that used to cure it.
        """
        page = self._page(xy_connected=False)
        page._bore_cal_refresh()
        self.assertIn("XY stage", page._bore_cal_status.text())
        self.assertFalse(any(b.isEnabled() for b in self._row_buttons(page)))

        page.controller.is_xy_connected = True      # operator connects
        page._bore_cal_tick()                       # one ordinary page tick

        self.assertNotIn("XY stage", page._bore_cal_status.text())
        buttons = self._row_buttons(page)
        self.assertEqual(len(buttons), 2)
        self.assertTrue(all(b.isEnabled() for b in buttons),
                        "row buttons still disabled after the stage connected")

    def test_the_tick_is_reached_from_on_status_update(self):
        """The fix is worthless if the tick handler never calls it."""
        page = self._page(xy_connected=False)
        page._bore_cal_refresh()
        page.controller.is_xy_connected = True
        page.on_status_update()
        self.assertNotIn("XY stage", page._bore_cal_status.text())

    def test_disconnecting_re_arms_the_refusal(self):
        """Symmetric: the gate must go back to refusing, not latch open."""
        page = self._page(xy_connected=True)
        page._bore_cal_refresh()
        self.assertTrue(all(b.isEnabled() for b in self._row_buttons(page)))
        page.controller.is_xy_connected = False
        page._bore_cal_tick()
        self.assertIn("XY stage", page._bore_cal_status.text())
        self.assertFalse(any(b.isEnabled() for b in self._row_buttons(page)))

    def test_cameras_coming_up_also_clears_their_refusal(self):
        """The gate froze wholesale, not just on the XY clause — the camera
        clause reads a live frame and is equally unset at build time."""
        page = self._page(xy_connected=True)
        page._needle_loc_camera_info = lambda role: None
        page._bore_cal_refresh()
        self.assertIn("Needle cam 1", page._bore_cal_status.text())
        page._needle_loc_camera_info = lambda role: (1.67, 640, 45.0)
        page._bore_cal_tick()
        self.assertNotIn("Needle cam", page._bore_cal_status.text())

    def test_arriving_on_the_tab_re_evaluates(self):
        page = self._page(xy_connected=False)
        page._bore_cal_refresh()
        page.controller.is_xy_connected = True
        page._on_workflow_tab_changed(page._needle_loc_tab_index)
        self.assertNotIn("XY stage", page._bore_cal_status.text())


class TestThrottling(_PageBase):
    """3 Hz is fine for a tuple compare, not for tearing down widgets."""

    def test_a_static_rig_never_rebuilds_rows(self):
        page = self._page(xy_connected=True)
        page._bore_cal_refresh()
        calls = []
        page._bore_cal_refresh = lambda: calls.append(1)
        for _ in range(200):
            page._bore_cal_tick()
        self.assertEqual(calls, [],
                         "the tick rebuilt rows with nothing changed")

    def test_one_change_causes_exactly_one_rebuild(self):
        page = self._page(xy_connected=False)
        page._bore_cal_refresh()
        calls = []
        page._bore_cal_refresh = lambda: calls.append(1)
        for _ in range(5):
            page._bore_cal_tick()
        self.assertEqual(calls, [])
        page.controller.is_xy_connected = True
        for _ in range(5):
            page._bore_cal_tick()
        self.assertEqual(len(calls), 1,
                         "a single transition must rebuild exactly once")

    def test_an_explicit_refresh_does_not_cause_a_redundant_rebuild(self):
        """_bore_cal_refresh re-syncs the key, so a capture/clear/config push
        does not leave the tick one rebuild behind."""
        page = self._page(xy_connected=True)
        page._bore_cal_refresh()
        calls = []
        real = page._bore_cal_refresh
        page._bore_cal_refresh = lambda: (calls.append(1), real())[1]
        page._bore_cal_tick()
        self.assertEqual(calls, [])

    def test_a_new_measurement_is_picked_up(self):
        """The key includes the store, so a capture made elsewhere shows up."""
        import SupportClasses.NeedleBoreCalibrationStore as mod
        page = self._page(xy_connected=True)
        page._bore_cal_refresh()
        calls = []
        page._bore_cal_refresh = lambda: calls.append(1)
        needle = page._hardware_config.needle
        mod.get_store().set_bore(0, (0.0, 0.0), needle=needle)
        page._bore_cal_tick()
        self.assertEqual(len(calls), 1)


class TestGateKeyIsCheap(_PageBase):

    def test_key_is_hashable_and_comparable(self):
        page = self._page(xy_connected=True)
        k = page._bore_cal_gate_key()
        self.assertIsInstance(k, tuple)
        hash(k)
        self.assertEqual(k, page._bore_cal_gate_key())

    def test_key_reads_no_camera_frames(self):
        """It samples the frame WIDTH via _needle_loc_camera_info, never a
        frame — a per-tick pixel copy would be a real cost."""
        seen = []
        page = self._page(xy_connected=True)
        page._needle_loc_camera_info = (
            lambda role: (seen.append(role), (1.67, 640, 45.0))[1])
        page._bore_cal_gate_key()
        self.assertEqual(len(seen), 2)      # exactly the two needle cams

    def test_key_survives_a_missing_controller(self):
        page = self._page(xy_connected=True)
        page.controller = None
        page._bore_cal_gate_key()           # must not raise

    def test_tick_survives_a_raising_gate(self):
        page = self._page(xy_connected=True)

        def _boom(role):
            raise RuntimeError("camera exploded")
        page._needle_loc_camera_info = _boom
        page._bore_cal_tick()               # must not raise


if __name__ == "__main__":
    unittest.main()

"""
test_v714_full_res_mosaic.py — momentary full-resolution capture.

The camera runs its live preview binned (a full 2048² mono-16 stream saturates
the USB link), but a mosaic tile wants every sensor pixel. Binning does not
change the FIELD OF VIEW, so a full-resolution scan visits the SAME tiles —
which is what makes a momentary switch cheap.

Covered: the pure switch/restore/canvas helpers; that the setting plumbs
through MosaicCalibration; and — the part that actually matters on hardware —
that EVERY scan exit path restores the preview resolution.
"""

import os
import sys
import unittest
from types import SimpleNamespace

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from SupportClasses.CaptureResolution import (
    CANVAS_BYTES_PER_PX, DEFAULT_CANVAS_CAP_PX, available_resolutions,
    canvas_px_for_full_res, current_resolution, describe_switch,
    estimated_canvas_mb, max_resolution, restore, switch_to_max)


class _FakeMgr:
    """Duck-typed camera manager: a resolution list + the current one."""

    def __init__(self, resolutions=((256, 256), (1024, 1024), (2048, 2048)),
                 current=(1024, 1024), refuse=False, deaf=False):
        self.resolutions = [tuple(r) for r in resolutions]
        self.current = tuple(current) if current else None
        self.refuse = refuse          # set_capture_resolution raises
        self.deaf = deaf              # accepts the call, ignores it
        self.calls = []

    def get_hw_settings(self, idx):
        return {"resolution": self.current,
                "resolutions": list(self.resolutions)}

    def set_capture_resolution(self, idx, w, h):
        self.calls.append((idx, w, h))
        if self.refuse:
            raise RuntimeError("device busy")
        if not self.deaf:
            self.current = (int(w), int(h))
        return self.current


class TestResolutionQueries(unittest.TestCase):
    def test_sorted_ascending_and_max(self):
        mgr = _FakeMgr(resolutions=((2048, 2048), (256, 256), (1024, 1024)))
        self.assertEqual(available_resolutions(mgr, 0)[-1], (2048, 2048))
        self.assertEqual(max_resolution(mgr, 0), (2048, 2048))

    def test_broken_entries_ignored(self):
        mgr = _FakeMgr(resolutions=((1024, 1024),))
        mgr.resolutions = [(1024, 1024), None, (0, 0), ("a", "b")]
        self.assertEqual(available_resolutions(mgr, 0), [(1024, 1024)])

    def test_unreadable_manager_is_empty_not_raising(self):
        class _Boom:
            def get_hw_settings(self, idx):
                raise RuntimeError("no camera")
        self.assertEqual(available_resolutions(_Boom(), 0), [])
        self.assertIsNone(max_resolution(_Boom(), 0))
        self.assertIsNone(current_resolution(None, 0))


class TestSwitchRestore(unittest.TestCase):
    def test_switch_returns_previous_and_moves_device(self):
        mgr = _FakeMgr()
        prev = switch_to_max(mgr, 0)
        self.assertEqual(prev, (1024, 1024))
        self.assertEqual(mgr.current, (2048, 2048))

    def test_restore_puts_it_back(self):
        mgr = _FakeMgr()
        prev = switch_to_max(mgr, 0)
        self.assertTrue(restore(mgr, 0, prev))
        self.assertEqual(mgr.current, (1024, 1024))

    def test_already_max_is_a_noop(self):
        mgr = _FakeMgr(current=(2048, 2048))
        self.assertIsNone(switch_to_max(mgr, 0))
        self.assertEqual(mgr.calls, [])

    def test_single_resolution_camera_is_a_noop(self):
        mgr = _FakeMgr(resolutions=((640, 480),), current=(640, 480))
        self.assertIsNone(switch_to_max(mgr, 0))
        self.assertEqual(mgr.calls, [])

    def test_refused_switch_reports_nothing_to_restore(self):
        """A camera that raises must not leave the caller believing it is at
        full resolution — and must not blow up the scan."""
        mgr = _FakeMgr(refuse=True)
        self.assertIsNone(switch_to_max(mgr, 0))

    def test_deaf_camera_reports_nothing_to_restore(self):
        """Accepted-but-ignored is the dangerous case: without the read-back
        check the caller would think it switched."""
        mgr = _FakeMgr(deaf=True)
        self.assertIsNone(switch_to_max(mgr, 0))

    def test_restore_never_raises(self):
        self.assertFalse(restore(None, 0, (1024, 1024)))
        self.assertFalse(restore(_FakeMgr(refuse=True), 0, (1024, 1024)))
        self.assertFalse(restore(_FakeMgr(), 0, None))


class TestCanvasMath(unittest.TestCase):
    def test_canvas_scales_with_linear_pixel_ratio(self):
        self.assertEqual(
            canvas_px_for_full_res(3000, (1024, 1024), (2048, 2048),
                                   cap_px=10000), 6000)

    def test_canvas_capped(self):
        self.assertEqual(
            canvas_px_for_full_res(3000, (1024, 1024), (2048, 2048),
                                   cap_px=4500), 4500)

    def test_no_gain_leaves_canvas_alone(self):
        self.assertEqual(
            canvas_px_for_full_res(3000, (2048, 2048), (2048, 2048)), 3000)
        self.assertEqual(canvas_px_for_full_res(3000, None, None), 3000)

    def test_memory_grows_with_the_square(self):
        """The reason the canvas is capped rather than simply doubled."""
        one = estimated_canvas_mb(3000)
        two = estimated_canvas_mb(6000)
        self.assertAlmostEqual(two / one, 4.0, places=3)
        # ...and matches the real MosaicBuilder allocation.
        self.assertAlmostEqual(
            one, 3000 * 3000 * CANVAS_BYTES_PER_PX / (1024 * 1024), places=3)

    def test_default_cap_is_under_a_gigabyte(self):
        self.assertLess(estimated_canvas_mb(DEFAULT_CANVAS_CAP_PX), 1024.0)

    def test_describe_mentions_both_resolutions(self):
        txt = describe_switch((1024, 1024), (2048, 2048), 6000)
        self.assertIn("2048x2048", txt)
        self.assertIn("1024x1024", txt)
        self.assertIn("restored", txt)


class TestSettingsPlumbing(unittest.TestCase):
    def test_default_is_off(self):
        from SupportClasses.MosaicCalibration import SCAN_DEFAULTS
        from gui.dialogs.mosaic_settings_dialog import MOSAIC_SCAN_DEFAULTS
        self.assertIs(SCAN_DEFAULTS["full_res_scan"], False)
        self.assertIs(MOSAIC_SCAN_DEFAULTS["full_res_scan"], False)

    def test_resolve_carries_the_flag(self):
        from SupportClasses.MosaicCalibration import resolve
        cal = resolve(live_resolution=(2048, 2048),
                      scan_settings={"full_res_scan": True})
        self.assertTrue(cal.full_res_scan)
        cal2 = resolve(live_resolution=(1024, 1024), scan_settings={})
        self.assertFalse(cal2.full_res_scan)

    def test_dialog_round_trips_the_key(self):
        from PySide6.QtWidgets import QApplication
        _app = QApplication.instance() or QApplication(sys.argv)
        from gui.dialogs.mosaic_settings_dialog import MosaicScanSettingsDialog
        dlg = MosaicScanSettingsDialog({"full_res_scan": True})
        self.assertTrue(dlg.values()["full_res_scan"])
        self.assertIn("MB", dlg._full_res_note.text())     # cost stated
        dlg.set_values({"full_res_scan": False})
        self.assertFalse(dlg.values()["full_res_scan"])
        self.assertEqual(dlg._full_res_note.text(), "")

    def test_dialog_warns_on_a_huge_canvas(self):
        from PySide6.QtWidgets import QApplication
        _app = QApplication.instance() or QApplication(sys.argv)
        from gui.dialogs.mosaic_settings_dialog import MosaicScanSettingsDialog
        dlg = MosaicScanSettingsDialog(
            {"full_res_scan": True, "target_px": 8000})
        self.assertIn("⚠", dlg._full_res_note.text())


# ── Every scan exit restores the preview resolution ──────────────────

class _PageStub:
    """Minimal stand-in carrying only what the two helpers touch."""

    def __init__(self, mgr, settings):
        self._camera_manager = mgr
        self._mosaic_settings = settings
        self._ploc_live_cam_idx = 0
        self._ploc_full_res_prev = None
        self._ploc_full_res_canvas = None


class TestCalibrationPageRestore(unittest.TestCase):
    def _stub(self, on=True):
        from gui.pages.calibration import CalibrationPage
        mgr = _FakeMgr()
        page = _PageStub(mgr, {"full_res_scan": on, "target_px": 3000})
        page._ploc_apply_full_res = (
            lambda idx: CalibrationPage._ploc_apply_full_res(page, idx))
        page._ploc_restore_full_res = (
            lambda: CalibrationPage._ploc_restore_full_res(page))
        return page, mgr

    def test_switch_then_restore(self):
        page, mgr = self._stub()
        page._ploc_apply_full_res(0)
        self.assertEqual(mgr.current, (2048, 2048))
        self.assertEqual(page._ploc_full_res_canvas, 4500)   # capped
        page._ploc_restore_full_res()
        self.assertEqual(mgr.current, (1024, 1024))

    def test_off_does_nothing(self):
        page, mgr = self._stub(on=False)
        page._ploc_apply_full_res(0)
        self.assertEqual(mgr.calls, [])
        self.assertIsNone(page._ploc_full_res_canvas)

    def test_restore_is_idempotent(self):
        page, mgr = self._stub()
        page._ploc_apply_full_res(0)
        page._ploc_restore_full_res()
        page._ploc_restore_full_res()          # a second exit path firing
        self.assertEqual(len(mgr.calls), 2)    # switch + ONE restore
        self.assertEqual(mgr.current, (1024, 1024))

    def test_stale_canvas_cleared_when_toggled_off(self):
        """A canvas left from a previous full-res run must not silently raise
        the memory of the next ordinary scan."""
        page, _mgr = self._stub()
        page._ploc_apply_full_res(0)
        page._ploc_restore_full_res()
        page._mosaic_settings["full_res_scan"] = False
        page._ploc_apply_full_res(0)
        self.assertIsNone(page._ploc_full_res_canvas)

    def test_cleanup_ui_restores(self):
        """_ploc_mosaic_cleanup_ui is the chokepoint all five scan exits use —
        objective declined / tiles declined / finished / failed / cancelled.
        Mutation guard: drop the restore call and the camera is stranded at
        full resolution after every scan."""
        from gui.pages.calibration import CalibrationPage
        page, mgr = self._stub()
        page._ploc_apply_full_res(0)
        btn = SimpleNamespace(setEnabled=lambda v: None,
                              setVisible=lambda v: None)
        page._ploc_btn_run = btn
        page._ploc_btn_clear = btn
        page._ploc_btn_mosaic = btn
        page._ploc_btn_cancel = btn
        CalibrationPage._ploc_mosaic_cleanup_ui(page)
        self.assertEqual(mgr.current, (1024, 1024))

    def test_shutdown_restores(self):
        """Closing the app mid-scan is the ONE exit that bypasses the
        cleanup chokepoint."""
        from gui.pages.calibration import CalibrationPage
        page, mgr = self._stub()
        page._ploc_apply_full_res(0)
        page._ploc_mosaic_running = True
        page._ploc_mosaic_worker = None
        CalibrationPage._shutdown_mosaic_worker(page)
        self.assertEqual(mgr.current, (1024, 1024))


class TestFluorescencePageRestore(unittest.TestCase):
    def _stub(self, on=True):
        from gui.pages.workflows.fluorescence_mosaic_workflow import (
            FluorescenceMosaicWorkflowPage as P)
        mgr = _FakeMgr()
        page = SimpleNamespace(
            _camera_manager=mgr,
            _scan_settings=lambda: {"full_res_scan": on},
            _target_px=SimpleNamespace(value=lambda: 2000),
            _resolve_microscope_cam_idx=lambda: 0,
            _full_res_prev=None, _full_res_canvas=None,
            _entry_exposure_us=None)
        page._apply_full_res = lambda: P._apply_full_res(page)
        page._restore_full_res = lambda: P._restore_full_res(page)
        page._restore_entry_exposure = (
            lambda: P._restore_entry_exposure(page))
        return page, mgr

    def test_switch_and_canvas(self):
        page, mgr = self._stub()
        self.assertTrue(page._apply_full_res())
        self.assertEqual(mgr.current, (2048, 2048))
        self.assertEqual(page._full_res_canvas, 4000)   # 2000 × 2, under cap

    def test_entry_restore_also_restores_resolution(self):
        """The four exits that already restore the exposure (finish, abort,
        operator cancel, failure) must also restore the resolution — pairing
        them is what guarantees coverage without four new call sites."""
        page, mgr = self._stub()
        page._apply_full_res()
        page._restore_entry_exposure()
        self.assertEqual(mgr.current, (1024, 1024))

    def test_off_does_nothing(self):
        page, mgr = self._stub(on=False)
        self.assertFalse(page._apply_full_res())
        self.assertEqual(mgr.calls, [])


if __name__ == "__main__":
    unittest.main()

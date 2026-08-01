"""
test_v75x_andor_display_scaling.py — Zyla display auto-scale option + manual levels.

v7.5.x: The Andor Zyla is a mono-16 sCMOS with NO ISP auto-gain — what looked
like the camera "adjusting its gain to the current frame" was the backend's
per-frame 1–99 percentile auto-scale in the mono16→BGR8 display conversion
(andor_backend._mono_to_bgr8). This change makes that behaviour a controllable
option surfaced in the camera settings dialog:

- "Auto display scaling (per-frame)" checkbox — ON reproduces the historical
  per-frame normalize; OFF freezes the mapping at fixed black/white levels.
- "Black level" / "White level" sliders (raw sensor counts 0..65535), active
  in manual mode; turning auto OFF seeds them from the last auto frame so the
  image freezes at its current appearance.
- Persisted per device identity (hw_controls) and restored on camera start.

No SDK / pylablib required: AndorBackend's display-scale state is pure Python,
and the widget/dialog paths run against the FakeAndorCam mirror.
"""

import os
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import numpy as np
from PySide6.QtWidgets import QApplication

_app = QApplication.instance() or QApplication(sys.argv)

from gui.widgets import andor_backend
from gui.widgets.andor_backend import AndorBackend
from gui.widgets.camera_widget import CAMERA_AVAILABLE
from gui.widgets.camera_manager import CameraManager

from tests.test_v75x_andor_zyla_camera import FakeAndorCam, _andor_widget


# ── Backend state machine (no SDK needed — pure Python state) ─────────

class TestBackendDisplayScaleState(unittest.TestCase):
    def test_defaults_auto_on_full_levels(self):
        be = AndorBackend()
        self.assertTrue(be.get_display_auto_scale())
        self.assertEqual(be.get_display_levels(), (0, 65535))

    def test_toggle_off_freezes_last_auto_levels(self):
        be = AndorBackend()
        be._last_auto_levels = (210.4, 5980.6)   # as the reader would record
        be.set_display_auto_scale(False)
        self.assertFalse(be.get_display_auto_scale())
        self.assertEqual(be.get_display_levels(), (210, 5981))

    def test_toggle_off_without_auto_history_keeps_levels(self):
        be = AndorBackend()
        be.put_display_black(100)
        be.put_display_white(2000)
        be._last_auto_levels = None
        be.set_display_auto_scale(False)
        self.assertEqual(be.get_display_levels(), (100, 2000))

    def test_retoggle_off_does_not_reseed_manual_levels(self):
        # Once in manual mode, calling set(False) again must not clobber the
        # operator's levels with stale auto levels.
        be = AndorBackend()
        be._last_auto_levels = (50.0, 900.0)
        be.set_display_auto_scale(False)
        be.put_display_black(10)
        be.put_display_white(300)
        be.set_display_auto_scale(False)
        self.assertEqual(be.get_display_levels(), (10, 300))

    def test_levels_clamped_and_ordered(self):
        be = AndorBackend()
        be.put_display_black(-50)
        self.assertEqual(be.get_display_levels()[0], 0)
        be.put_display_white(999999)
        self.assertEqual(be.get_display_levels()[1], 65535)
        # black pushed above white drags white up (hi > lo invariant)
        be.put_display_white(500)
        be.put_display_black(600)
        lo, hi = be.get_display_levels()
        self.assertLess(lo, hi)
        # white pushed below black drags black down
        be.put_display_white(100)
        lo, hi = be.get_display_levels()
        self.assertLess(lo, hi)

    def test_put_levels_reject_garbage(self):
        be = AndorBackend()
        self.assertFalse(be.put_display_black("not-a-number"))
        self.assertFalse(be.put_display_white(None))

    def test_get_settings_carries_display_scale(self):
        be = AndorBackend()
        be.set_display_auto_scale(False)
        be.put_display_black(120)
        be.put_display_white(4000)
        st = be.get_settings()
        self.assertFalse(st["andor_auto_scale"])
        self.assertEqual(st["andor_scale_lo"], 120)
        self.assertEqual(st["andor_scale_hi"], 4000)


# ── Fixed-level conversion math ───────────────────────────────────────

class TestManualLevelConversion(unittest.TestCase):
    def test_fixed_levels_do_not_chase_the_scene(self):
        # The regression this feature exists for: with FIXED levels, a dim
        # frame stays dim instead of being auto-stretched to full range.
        dim = np.random.randint(200, 800, size=(64, 64)).astype(np.uint16)
        out_auto = andor_backend._mono_to_bgr8(dim)                  # auto
        out_fixed = andor_backend._mono_to_bgr8(dim, levels=(0, 65535))
        self.assertGreater(int(out_auto.max()), 200)     # auto stretches
        self.assertLess(int(out_fixed.max()), 10)        # fixed stays dim

    def test_fixed_levels_map_deterministically(self):
        # Same raw value → same display value regardless of frame content.
        a = np.full((16, 16), 1000, dtype=np.uint16)
        b = a.copy()
        b[0, 0] = 60000  # different scene content
        la = andor_backend._mono_to_bgr8(a, levels=(0, 2000))
        lb = andor_backend._mono_to_bgr8(b, levels=(0, 2000))
        self.assertEqual(int(la[8, 8, 0]), int(lb[8, 8, 0]))
        self.assertEqual(int(la[8, 8, 0]), 127)  # 1000/2000 ≈ mid-gray

    def test_fixed_levels_clip_ends(self):
        f = np.array([[0, 500, 1000, 2000, 65535]], dtype=np.uint16)
        out = andor_backend._mono_to_bgr8(f, levels=(500, 1000))
        vals = out[0, :, 0].tolist()
        self.assertEqual(vals[0], 0)       # below black → 0
        self.assertEqual(vals[1], 0)       # black level → 0
        self.assertEqual(vals[2], 255)     # white level → 255
        self.assertEqual(vals[3], 255)     # above white → clipped
        self.assertEqual(vals[4], 255)

    def test_degenerate_levels_no_divide_by_zero(self):
        f = np.full((8, 8), 500, dtype=np.uint16)
        out = andor_backend._mono_to_bgr8(f, levels=(500, 500))
        self.assertEqual(out.shape, (8, 8, 3))

    def test_auto_levels_helper(self):
        f = np.linspace(0, 65535, 64 * 64, dtype=np.uint16).reshape(64, 64)
        lo, hi = andor_backend._auto_levels(f)
        self.assertLess(lo, hi)
        self.assertGreater(hi, 60000)


# ── CameraWidget + CameraManager plumbing ─────────────────────────────

@unittest.skipUnless(CAMERA_AVAILABLE, "camera backend unavailable")
class TestWidgetAndManagerPlumbing(unittest.TestCase):
    def test_capabilities_advertise_display_scaling(self):
        _mgr, cam = _andor_widget()
        ctrls = cam.hardware_capabilities()["controls"]
        self.assertIn("andor_auto_scale", ctrls)
        self.assertIn("andor_scale_lo", ctrls)
        self.assertIn("andor_scale_hi", ctrls)
        self.assertEqual(ctrls["andor_scale_hi"]["range"], (0, 65535, 65535))

    def test_widget_setters_delegate(self):
        _mgr, cam = _andor_widget()
        self.assertTrue(cam.set_hw_andor_auto_scale(False))
        self.assertFalse(cam._andor.get_display_auto_scale())
        self.assertTrue(cam.set_hw_andor_scale_lo(150))
        self.assertTrue(cam.set_hw_andor_scale_hi(9000))
        self.assertEqual(cam._andor.get_display_levels(), (150, 9000))

    def test_widget_setters_false_on_non_andor(self):
        mgr = CameraManager(max_cameras=1)
        cam = mgr.cameras[0]   # not running / no andor backend
        self.assertFalse(cam.set_hw_andor_auto_scale(True))
        self.assertFalse(cam.set_hw_andor_scale_lo(0))
        self.assertFalse(cam.set_hw_andor_scale_hi(100))

    def test_manager_delegates(self):
        mgr, cam = _andor_widget()
        self.assertTrue(mgr.set_hw_andor_auto_scale(0, False))
        self.assertTrue(mgr.set_hw_andor_scale_lo(0, 42))
        self.assertTrue(mgr.set_hw_andor_scale_hi(0, 4200))
        st = mgr.get_hw_settings(0)
        self.assertFalse(st["andor_auto_scale"])
        self.assertEqual(st["andor_scale_lo"], 42)
        self.assertEqual(st["andor_scale_hi"], 4200)

    def test_log_readout_names_display_scale(self):
        _mgr, cam = _andor_widget()
        _st, text = cam.log_hw_settings()
        self.assertIn("display scale", text)
        self.assertIn("auto (per-frame)", text)


# ── Settings dialog ───────────────────────────────────────────────────

@unittest.skipUnless(CAMERA_AVAILABLE, "camera backend unavailable")
class TestSettingsDialog(unittest.TestCase):
    def setUp(self):
        import SupportClasses.CameraCalibrationStore as CCS
        self._ccs = CCS
        self._orig_store = getattr(CCS, "_store", None)
        self.tmp = Path(tempfile.mkdtemp()) / "cam_cal.json"
        CCS._store = CCS.CameraCalibrationStore(self.tmp)

    def tearDown(self):
        self._ccs._store = self._orig_store

    def _dialog(self):
        from gui.dialogs.camera_settings_dialog import CameraSettingsDialog
        mgr, cam = _andor_widget()
        mgr.camera_identity = lambda i: ("andor:SN-ZYLA-001", "Zyla")
        dlg = CameraSettingsDialog(
            mgr, 0, identity_getter=lambda: ("andor:SN-ZYLA-001", "Zyla"))
        return dlg, mgr, cam

    def test_rows_visible_for_andor(self):
        dlg, _mgr, _cam = self._dialog()
        self.assertTrue(dlg._ascale_chk.isVisibleTo(dlg))
        self.assertTrue(dlg._blk_sld.isVisibleTo(dlg))
        self.assertTrue(dlg._wht_sld.isVisibleTo(dlg))
        # auto on (default) → the checkbox reflects it, sliders disabled
        self.assertTrue(dlg._ascale_chk.isChecked())
        self.assertFalse(dlg._blk_sld.isEnabled())
        self.assertFalse(dlg._wht_sld.isEnabled())

    def test_toggle_manual_enables_sliders_and_applies(self):
        dlg, _mgr, cam = self._dialog()
        dlg._ascale_chk.setChecked(False)
        self.assertFalse(cam._andor.get_display_auto_scale())
        self.assertTrue(dlg._blk_sld.isEnabled())
        self.assertTrue(dlg._wht_sld.isEnabled())
        dlg._blk_sld.setValue(300)
        dlg._wht_sld.setValue(7000)
        self.assertEqual(cam._andor.get_display_levels(), (300, 7000))

    def test_persists_per_identity(self):
        dlg, _mgr, _cam = self._dialog()
        dlg._ascale_chk.setChecked(False)
        dlg._blk_sld.setValue(250)
        from SupportClasses.CameraCalibrationStore import get_store
        hw = get_store().get_hw_controls("andor:SN-ZYLA-001")
        self.assertIsNotNone(hw)
        self.assertIs(hw["andor_auto_scale"], False)
        self.assertEqual(hw["andor_scale_lo"], 250)

    def test_rows_hidden_for_non_andor(self):
        # A dialog over a slot with no controllable backend hides the rows.
        from gui.dialogs.camera_settings_dialog import CameraSettingsDialog
        mgr = CameraManager(max_cameras=1)
        dlg = CameraSettingsDialog(mgr, 0)
        self.assertFalse(dlg._ascale_chk.isVisibleTo(dlg))
        self.assertFalse(dlg._blk_sld.isVisibleTo(dlg))

    def test_defaults_button_restores_auto(self):
        dlg, _mgr, cam = self._dialog()
        dlg._ascale_chk.setChecked(False)
        self.assertFalse(cam._andor.get_display_auto_scale())
        dlg._on_defaults_clicked()
        self.assertTrue(cam._andor.get_display_auto_scale())
        self.assertTrue(dlg._ascale_chk.isChecked())


# ── hardware_setup restore ────────────────────────────────────────────

@unittest.skipUnless(CAMERA_AVAILABLE, "camera backend unavailable")
class TestApplyHwControlsRestore(unittest.TestCase):
    def _apply(self, mgr, hw):
        from gui.pages.hardware_setup import HardwareSetupPage
        fake_self = SimpleNamespace(_camera_manager=mgr,
                                    _config=SimpleNamespace(
                                        camera_for_role=lambda role: -1))
        HardwareSetupPage._apply_hw_controls(fake_self, 0, hw)

    def test_restores_manual_levels_after_auto_flag(self):
        mgr, cam = _andor_widget()
        # Seed as if a previous auto frame existed — the stored levels must
        # still win over the freeze-seed (order: auto first, then levels).
        cam._andor._lo, cam._andor._hi = 0, 65535
        self._apply(mgr, {"andor_auto_scale": False,
                          "andor_scale_lo": 111, "andor_scale_hi": 8888})
        self.assertFalse(cam._andor.get_display_auto_scale())
        self.assertEqual(cam._andor.get_display_levels(), (111, 8888))

    def test_restore_ignores_missing_keys(self):
        mgr, cam = _andor_widget()
        self._apply(mgr, {"exposure_us": 12345})   # legacy blob, no andor keys
        self.assertTrue(cam._andor.get_display_auto_scale())   # untouched
        self.assertEqual(cam._andor.get_exposure_time(), 12345)


if __name__ == "__main__":
    unittest.main()

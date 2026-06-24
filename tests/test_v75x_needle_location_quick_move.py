"""
v7.5.x — Needle Location quick-move button.

A "Go to needle location" button on the Calibration → Needle Location tab drives
the stage to the saved approximate needle position (retract Z → XY → lower to the
needle-cam Z) so the needle re-enters both side cameras, ready to re-center. The
position is captured on Center & Save and via a manual "Set current as location"
button, persisted in the device profile (absolute Prior stage µm).

These tests call the real (unbound) CalibrationPage methods with duck-typed stubs
so no Qt widget / camera / event loop is needed. `QMessageBox` is patched so the
dialog paths don't require a QApplication.
"""

import unittest
from types import SimpleNamespace

from gui.pages.calibration import CalibrationPage
import gui.pages.calibration as calmod
from gui.pages.hardware.device_profile import DeviceProfile


class _FakeMB:
    """Stand-in for QMessageBox in headless tests."""
    Yes = 1
    No = 0
    info = 0
    warned = 0
    critical_n = 0

    @classmethod
    def reset(cls):
        cls.info = cls.warned = cls.critical_n = 0

    @classmethod
    def information(cls, *a, **k):
        cls.info += 1

    @classmethod
    def warning(cls, *a, **k):
        cls.warned += 1

    @classmethod
    def critical(cls, *a, **k):
        cls.critical_n += 1


class _FakeBtn:
    def __init__(self):
        self.enabled = None

    def setEnabled(self, v):
        self.enabled = v

    def repaint(self):
        pass


class _FakeLabel:
    def __init__(self):
        self.text = ""

    def setText(self, t):
        self.text = t

    def setStyleSheet(self, *a):
        pass


class _FakeSettings:
    def __init__(self):
        self.store = {}
        self.saved = 0

    def get(self, key):
        return self.store.get(key)

    def get_section(self, key):
        return self.store.get(key, {})

    def set(self, key, value):
        self.store[key] = value

    def set_section(self, key, value):
        self.store[key] = value

    def save(self):
        self.saved += 1


def _bind(stub, *names):
    """Bind the named real CalibrationPage methods onto a stub."""
    for n in names:
        setattr(stub, n, getattr(CalibrationPage, n).__get__(stub))


class _MBPatch(unittest.TestCase):
    def setUp(self):
        _FakeMB.reset()
        self._orig_mb = calmod.QMessageBox
        calmod.QMessageBox = _FakeMB

    def tearDown(self):
        calmod.QMessageBox = self._orig_mb


class TestStoreAndUI(_MBPatch):
    def _stub(self, **over):
        stub = SimpleNamespace(
            settings=_FakeSettings(),
            _needle_loc_xy_um=None,
            _needle_loc_btn_goto=_FakeBtn(),
            _needle_loc_goto_label=_FakeLabel(),
            controller=SimpleNamespace(zero_position={"x": 0.0, "y": 0.0}),
        )
        _bind(stub, "_needle_loc_store_xy", "_needle_loc_update_goto_ui")
        for k, v in over.items():
            setattr(stub, k, v)
        return stub

    def test_store_persists_absolute_xy_and_enables_button(self):
        stub = self._stub()
        stub._needle_loc_store_xy(11719.0, 10059.0)
        self.assertEqual(stub._needle_loc_xy_um, (11719.0, 10059.0))
        self.assertEqual(
            stub.settings.store["device_profile.needle_loc_xy_um"],
            [11719.0, 10059.0])
        self.assertGreaterEqual(stub.settings.saved, 1)
        self.assertIs(stub._needle_loc_btn_goto.enabled, True)

    def test_label_shows_zero_referenced_um(self):
        stub = self._stub(
            controller=SimpleNamespace(zero_position={"x": 1000.0, "y": 2000.0}))
        stub._needle_loc_store_xy(11000.0, 12000.0)
        # zero-ref = (10000, 10000)
        self.assertIn("10,000", stub._needle_loc_goto_label.text)

    def test_update_ui_disables_button_when_unset(self):
        stub = self._stub()
        stub._needle_loc_update_goto_ui()
        self.assertIs(stub._needle_loc_btn_goto.enabled, False)
        self.assertIn("not set", stub._needle_loc_goto_label.text)

    def test_update_ui_is_noop_before_button_built(self):
        # No button attribute yet (tab not built) → must not raise.
        stub = SimpleNamespace(_needle_loc_xy_um=(1.0, 2.0))
        _bind(stub, "_needle_loc_update_goto_ui")
        stub._needle_loc_update_goto_ui()  # should simply return


class TestSetCurrent(_MBPatch):
    def _stub(self, xy, **over):
        stub = SimpleNamespace(
            controller=SimpleNamespace(
                get_xy_position=lambda cached=False: xy,
                zero_position={"x": 0.0, "y": 0.0}),
            settings=_FakeSettings(),
            _needle_loc_xy_um=None,
            _needle_loc_btn_goto=_FakeBtn(),
            _needle_loc_goto_label=_FakeLabel(),
        )
        _bind(stub, "_needle_loc_set_current",
              "_needle_loc_store_xy", "_needle_loc_update_goto_ui")
        for k, v in over.items():
            setattr(stub, k, v)
        return stub

    def test_captures_current_stage_xy(self):
        stub = self._stub((1234.0, 5678.0, 0.0))
        stub._needle_loc_set_current()
        self.assertEqual(stub._needle_loc_xy_um, (1234.0, 5678.0))
        self.assertEqual(
            stub.settings.store["device_profile.needle_loc_xy_um"],
            [1234.0, 5678.0])

    def test_warns_and_skips_when_no_controller(self):
        stub = self._stub((1.0, 2.0, 0.0), controller=None)
        stub._needle_loc_set_current()
        self.assertEqual(_FakeMB.warned, 1)
        self.assertIsNone(stub._needle_loc_xy_um)

    def test_warns_when_position_unreadable(self):
        stub = self._stub((None, None))
        stub._needle_loc_set_current()
        self.assertEqual(_FakeMB.warned, 1)
        self.assertIsNone(stub._needle_loc_xy_um)


class TestGoto(_MBPatch):
    def _stub(self, **over):
        calls = []
        stub = SimpleNamespace(
            _needle_loc_xy_um=(11719.0, 10059.0),
            _safe_z=5.0,
            _needle_loc_btn_goto=_FakeBtn(),
            _needle_loc_btn_set=_FakeBtn(),
            _needle_loc_goto_label=_FakeLabel(),
            controller=SimpleNamespace(
                is_zp_connected=True,
                zero_position={"x": 0.0, "y": 0.0, "Z": 20.27},
                get_needle_cam_z_user=lambda: 29.81,
                user_z_to_zref=lambda u: u / 1.0),  # z_up_sign = +1
            _safe_navigate_to=(
                lambda x, y, target_z_mm=None, lower_z=True:
                calls.append((x, y, target_z_mm, lower_z))),
        )
        stub._calls = calls
        _bind(stub, "_needle_loc_goto", "_needle_loc_update_goto_ui")
        for k, v in over.items():
            setattr(stub, k, v)
        return stub

    def test_safe_travels_to_saved_xy_and_needle_cam_z(self):
        stub = self._stub()
        stub._needle_loc_goto()
        self.assertEqual(stub._calls, [(11719.0, 10059.0, 29.81, True)])

    def test_stays_at_safe_z_when_no_needle_cam_z(self):
        stub = self._stub(controller=SimpleNamespace(
            is_zp_connected=True,
            zero_position={"x": 0.0, "y": 0.0, "Z": 20.27},
            get_needle_cam_z_user=lambda: None,
            user_z_to_zref=lambda u: u))
        stub._needle_loc_goto()
        self.assertEqual(stub._calls, [(11719.0, 10059.0, None, False)])

    def test_blocked_when_zp_connected_and_no_safe_z(self):
        stub = self._stub(_safe_z=None)
        stub._needle_loc_goto()
        self.assertEqual(stub._calls, [])
        self.assertEqual(_FakeMB.warned, 1)

    def test_allowed_when_zp_disconnected_and_no_safe_z(self):
        # Without ZP the retract gate doesn't apply (XY-only rig).
        stub = self._stub(
            _safe_z=None,
            controller=SimpleNamespace(
                is_zp_connected=False,
                zero_position={"x": 0.0, "y": 0.0},
                get_needle_cam_z_user=lambda: None,
                user_z_to_zref=lambda u: u))
        stub._needle_loc_goto()
        self.assertEqual(len(stub._calls), 1)

    def test_info_when_no_saved_location(self):
        stub = self._stub(_needle_loc_xy_um=None)
        stub._needle_loc_goto()
        self.assertEqual(stub._calls, [])
        self.assertEqual(_FakeMB.info, 1)

    def test_buttons_reenabled_after_move(self):
        stub = self._stub()
        stub._needle_loc_goto()
        self.assertIs(stub._needle_loc_btn_set.enabled, True)
        self.assertIs(stub._needle_loc_btn_goto.enabled, True)


class _RichMB:
    """QMessageBox fake supporting BOTH the classmethod (information/warning)
    and instance (addButton/exec/clickedButton) patterns used by
    ``_needle_loc_use_last_known``."""

    class Icon:
        Question = "Q"
        Warning = "W"

    class ButtonRole:
        AcceptRole = "accept"
        RejectRole = "reject"

    info = 0
    warned = 0
    click_role = "accept"  # which role exec() will "click"

    @classmethod
    def reset(cls):
        cls.info = cls.warned = 0
        cls.click_role = "accept"

    @classmethod
    def information(cls, *a, **k):
        cls.info += 1

    @classmethod
    def warning(cls, *a, **k):
        cls.warned += 1

    def __init__(self, *a, **k):
        self._buttons = []
        self._clicked = None

    def setIcon(self, *a):
        pass

    def setWindowTitle(self, *a):
        pass

    def setText(self, *a):
        pass

    def setInformativeText(self, *a):
        pass

    def setDefaultButton(self, *a):
        pass

    def addButton(self, label, role):
        obj = object()
        self._buttons.append((obj, role))
        return obj

    def exec(self):
        for obj, role in self._buttons:
            if role == type(self).click_role:
                self._clicked = obj
                return
        self._clicked = None

    def clickedButton(self):
        return self._clicked


class TestUseLastKnown(unittest.TestCase):
    """Restore the last-known needle reference without re-centering."""

    def setUp(self):
        _RichMB.reset()
        self._orig_mb = calmod.QMessageBox
        calmod.QMessageBox = _RichMB

    def tearDown(self):
        calmod.QMessageBox = self._orig_mb

    def _stub(self, store=None, zero=None, **over):
        cam_z_calls = []
        ctrl = SimpleNamespace(
            zero_position=dict(zero or {"x": 0.0, "y": 0.0}),
            set_needle_cam_z_user=lambda v: cam_z_calls.append(v),
        )
        s = _FakeSettings()
        s.store.update(store or {})
        stub = SimpleNamespace(
            settings=s,
            controller=ctrl,
            _xy_position_scale=1.0,
            _needle_loc_xy_um=None,
            _needle_origin_um=None,
            _needle_loc_origin_label=_FakeLabel(),
            _needle_loc_banner=_FakeLabel(),
            _needle_loc_btn_goto=_FakeBtn(),
            _needle_loc_goto_label=_FakeLabel(),
            _zoff_lbl_needle_cam=_FakeLabel(),
            _emit_calibration_data_changed=lambda: None,
        )
        stub._cam_z_calls = cam_z_calls
        _bind(stub, "_needle_loc_use_last_known", "_needle_loc_update_goto_ui")
        for k, v in over.items():
            setattr(stub, k, v)
        return stub

    def test_info_when_nothing_saved(self):
        stub = self._stub(store={})
        stub._needle_loc_use_last_known()
        self.assertEqual(_RichMB.info, 1)
        self.assertIsNone(stub._needle_loc_xy_um)
        self.assertEqual(stub._cam_z_calls, [])

    def test_warns_when_settings_unavailable(self):
        stub = self._stub()
        stub.settings = None
        stub._needle_loc_use_last_known()
        self.assertEqual(_RichMB.warned, 1)

    def test_restores_xy_camz_and_zero_on_accept(self):
        stub = self._stub(store={
            "device_profile.needle_loc_xy_um": [11000.0, 12000.0],
            "device_profile.needle_cam_z": 29.81,
            "zero_position": {"x": 1000.0, "y": 2000.0, "Z": 5.0},
        })
        stub._needle_loc_use_last_known()
        # Needle zero restored.
        self.assertEqual(stub.controller.zero_position["x"], 1000.0)
        self.assertEqual(stub.controller.zero_position["y"], 2000.0)
        # Needle-cam Z re-applied.
        self.assertEqual(stub._cam_z_calls, [29.81])
        # Saved XY adopted + origin derived against the RESTORED zero.
        self.assertEqual(stub._needle_loc_xy_um, (11000.0, 12000.0))
        self.assertEqual(stub._needle_origin_um, (10000.0, 10000.0))
        self.assertIn("last known", stub._needle_loc_origin_label.text)
        self.assertIn("✓", stub._needle_loc_banner.text)

    def test_cancel_makes_no_changes(self):
        _RichMB.click_role = "reject"
        stub = self._stub(store={
            "device_profile.needle_loc_xy_um": [11000.0, 12000.0],
            "device_profile.needle_cam_z": 29.81,
        })
        stub._needle_loc_use_last_known()
        self.assertIsNone(stub._needle_loc_xy_um)
        self.assertIsNone(stub._needle_origin_um)
        self.assertEqual(stub._cam_z_calls, [])

    def test_cam_z_only_is_enough_to_proceed(self):
        # No saved XY, but a needle-cam Z exists → still offered + restored.
        stub = self._stub(store={"device_profile.needle_cam_z": 30.0})
        stub._needle_loc_use_last_known()
        self.assertEqual(_RichMB.info, 0)
        self.assertEqual(stub._cam_z_calls, [30.0])
        self.assertIsNone(stub._needle_loc_xy_um)


class TestDeviceProfileRoundTrip(unittest.TestCase):
    def test_to_from_dict(self):
        p = DeviceProfile(profile_name="T", needle_loc_xy_um=[100.0, 200.0])
        d = p.to_dict()
        self.assertEqual(d["needle_loc_xy_um"], [100.0, 200.0])
        self.assertEqual(
            DeviceProfile.from_dict(d).needle_loc_xy_um, [100.0, 200.0])

    def test_apply_to_settings(self):
        s = _FakeSettings()
        DeviceProfile(needle_loc_xy_um=[7.0, 8.0]).apply_to_settings(s)
        self.assertEqual(
            s.store["device_profile.needle_loc_xy_um"], [7.0, 8.0])

    def test_apply_skips_when_none(self):
        s = _FakeSettings()
        DeviceProfile(needle_loc_xy_um=None).apply_to_settings(s)
        self.assertNotIn("device_profile.needle_loc_xy_um", s.store)

    def test_from_settings(self):
        s = _FakeSettings()
        s.store["device_profile.needle_loc_xy_um"] = [5.0, 6.0]
        self.assertEqual(
            DeviceProfile.from_settings(s).needle_loc_xy_um, [5.0, 6.0])


if __name__ == "__main__":
    unittest.main()

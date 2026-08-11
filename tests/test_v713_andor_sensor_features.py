"""
test_v713_andor_sensor_features.py — Zyla sensor-quality features (v7.13).

Covers the sensor feature layer added for the epifluorescence signal-quality
upgrade: the ANDOR_SENSOR_FEATURES table, token-based enum matching (never
hardcoded SDK strings), open-time low-noise defaults, the live-set
stop→apply→restart fallback, stale-persisted-enum refusal, capability
advertising / widget / manager delegation, the shared hw_controls_snapshot
(one key list for BOTH persistence sites — closes the documented bulk-save
gap), the _apply_hw_controls restore order, and the settings-dialog rows.

No SDK / pylablib required: the pure helpers and the real AndorBackend's
feature layer run against a stub camera object; widget/dialog paths run
against the FakeAndorCam mirror.
"""

import ast
import inspect
import os
import sys
import tempfile
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import numpy as np
from PySide6.QtWidgets import QApplication

_app = QApplication.instance() or QApplication(sys.argv)

from gui.widgets import andor_backend
from gui.widgets.andor_backend import (
    ANDOR_SENSOR_FEATURES, ANDOR_SENSOR_FEATURE_KEYS, AndorBackend,
    _match_enum_value, _parse_bit_depth)
from gui.widgets.camera_widget import CAMERA_AVAILABLE
from gui.widgets.camera_manager import CameraManager
from gui.widgets.hw_controls_snapshot import (
    PERSISTED_HW_CONTROL_KEYS, hw_controls_snapshot)

from tests.test_v75x_andor_zyla_camera import FakeAndorCam, _andor_widget


# ── Stub pylablib camera (real AndorBackend, no SDK) ──────────────────

class _StubAttr:
    def __init__(self, values=None):
        self.values = values


class _StubCam:
    """Just enough of pylablib's camera surface for the feature layer +
    reader loop. Frames are staged through a semaphore so tests control
    exactly which frames the reader consumes."""

    def __init__(self, features=None, locked_live=(), always_locked=()):
        # features: sdk_name -> {"values": [...]|None, "value": current}
        self.features = dict(features or {})
        self.sets = []          # (name, value)
        self.ops = []           # "stop" / "start" / "set:<Name>"
        self.acquiring = True
        self.locked_live = set(locked_live)      # NOTWRITABLE while acquiring
        self.always_locked = set(always_locked)  # always raises
        self._frames = []
        self._sem = threading.Semaphore(0)
        self.closed = False

    # attributes
    def get_attribute(self, name):
        if name not in self.features:
            raise KeyError(name)
        return _StubAttr(self.features[name].get("values"))

    def get_attribute_value(self, name):
        if name not in self.features:
            raise KeyError(name)
        return self.features[name].get("value")

    def set_attribute_value(self, name, value):
        if name in self.always_locked:
            raise RuntimeError("feature not writable")
        if name not in self.features:
            raise KeyError(name)
        if self.acquiring and name in self.locked_live:
            raise RuntimeError("NOTWRITABLE while acquiring")
        self.features[name]["value"] = value
        self.sets.append((name, value))
        self.ops.append(f"set:{name}")

    # exposure / roi
    def get_exposure(self):
        return 0.03

    def set_exposure(self, s):
        pass

    def set_roi(self, *a, **k):
        pass

    def get_roi(self):
        return (0, 64, 0, 64, 1, 1)

    # acquisition
    def start_acquisition(self, mode="sequence", nframes=10):
        self.acquiring = True
        self.ops.append("start")

    def stop_acquisition(self):
        self.acquiring = False
        self.ops.append("stop")

    def acquisition_in_progress(self):
        return self.acquiring

    # frames (semaphore-gated so tests control delivery)
    def stage_frame(self, frame):
        self._frames.append(frame)
        self._sem.release()

    def wait_for_frame(self, timeout=0.5):
        if not self._sem.acquire(timeout=min(float(timeout), 0.05)):
            raise TimeoutError("no frame staged")

    def read_newest_image(self):
        return self._frames.pop(0) if self._frames else None

    def close(self):
        self.closed = True


_GAIN_VALUES = [
    "11-bit (high well capacity)",
    "11-bit (low noise)",
    "12-bit (low noise)",
    "16-bit (low noise & high well capacity)",
]
_RATE_VALUES = ["100 MHz", "216 MHz", "280 MHz", "540 MHz"]


def _full_features(gain_value=None, rate_value=None):
    return {
        "SensorCooling": {"values": None, "value": False},
        "PixelReadoutRate": {"values": list(_RATE_VALUES),
                             "value": rate_value or "540 MHz"},
        "SimplePreAmpGainControl": {"values": list(_GAIN_VALUES),
                                    "value": gain_value or _GAIN_VALUES[0]},
        "SpuriousNoiseFilter": {"values": None, "value": False},
        "StaticBlemishCorrection": {"values": None, "value": False},
    }


def _backend(stub):
    be = AndorBackend()
    be._cam = stub
    be._probe_sensor_features()
    return be


# ── Pure helpers ──────────────────────────────────────────────────────

class TestMatchEnumValue(unittest.TestCase):
    def test_picks_216_rate(self):
        self.assertEqual(_match_enum_value(_RATE_VALUES, ("216",)), "216 MHz")

    def test_gain_mode_needs_all_tokens(self):
        # "low noise" alone would match "11-bit (low noise)" — the 16-bit
        # token must ALSO match (mutation guard: all-tokens → any-token).
        got = _match_enum_value(_GAIN_VALUES, ("16-bit", "low noise"))
        self.assertEqual(got, "16-bit (low noise & high well capacity)")

    def test_case_insensitive(self):
        self.assertEqual(
            _match_enum_value(["216 MHZ - Lowest Noise"], ("216",)),
            "216 MHZ - Lowest Noise")

    def test_no_match_returns_none(self):
        self.assertIsNone(_match_enum_value(["100 MHz", "540 MHz"], ("216",)))

    def test_empty_inputs(self):
        self.assertIsNone(_match_enum_value([], ("216",)))
        self.assertIsNone(_match_enum_value(_RATE_VALUES, ()))
        self.assertIsNone(_match_enum_value(None, ("216",)))


class TestParseBitDepth(unittest.TestCase):
    def test_12_bit(self):
        self.assertEqual(_parse_bit_depth("12 Bit"), 4095)

    def test_16_bit(self):
        self.assertEqual(_parse_bit_depth("16 Bit"), 65535)

    def test_gain_mode_string(self):
        self.assertEqual(
            _parse_bit_depth("16-bit (low noise & high well capacity)"), 65535)
        self.assertEqual(_parse_bit_depth("12-bit (low noise)"), 4095)

    def test_garbage(self):
        self.assertIsNone(_parse_bit_depth("garbage"))
        self.assertIsNone(_parse_bit_depth(None))
        self.assertIsNone(_parse_bit_depth("AOI 2048"))  # 2048 not a bit depth


# ── Probe + open-time defaults (real backend, stub cam) ──────────────

class TestProbeAndDefaults(unittest.TestCase):
    def test_probe_registers_only_present_features(self):
        stub = _StubCam(features={
            "SensorCooling": {"values": None, "value": True},
            "PixelReadoutRate": {"values": list(_RATE_VALUES),
                                 "value": "540 MHz"},
        })
        be = _backend(stub)
        specs = be.sensor_feature_specs()
        self.assertIn("andor_sensor_cooling", specs)
        self.assertIn("andor_readout_rate", specs)
        self.assertNotIn("andor_gain_mode", specs)      # not on this camera
        self.assertEqual(specs["andor_readout_rate"]["values"], _RATE_VALUES)

    def test_enum_without_values_is_hidden(self):
        stub = _StubCam(features={
            "PixelReadoutRate": {"values": None, "value": "540 MHz"},
        })
        be = _backend(stub)
        self.assertNotIn("andor_readout_rate", be.sensor_feature_specs())

    def test_defaults_applied_with_runtime_strings(self):
        stub = _StubCam(features=_full_features())
        be = _backend(stub)
        be._apply_sensor_defaults()
        applied = dict(stub.sets)
        self.assertIs(applied["SensorCooling"], True)
        self.assertEqual(applied["PixelReadoutRate"], "216 MHz")
        self.assertEqual(applied["SimplePreAmpGainControl"],
                         "16-bit (low noise & high well capacity)")
        self.assertIs(applied["SpuriousNoiseFilter"], True)
        self.assertIs(applied["StaticBlemishCorrection"], True)

    def test_default_skipped_when_no_token_match(self):
        feats = _full_features()
        feats["PixelReadoutRate"]["values"] = ["100 MHz", "540 MHz"]
        stub = _StubCam(features=feats)
        be = _backend(stub)
        be._apply_sensor_defaults()
        self.assertNotIn("PixelReadoutRate", dict(stub.sets))
        # The others still applied.
        self.assertIn("SensorCooling", dict(stub.sets))

    def test_raising_feature_does_not_stop_the_rest(self):
        stub = _StubCam(features=_full_features(),
                        always_locked={"SensorCooling"})
        be = _backend(stub)
        be._apply_sensor_defaults()          # must not raise
        applied = dict(stub.sets)
        self.assertNotIn("SensorCooling", applied)
        self.assertEqual(applied["PixelReadoutRate"], "216 MHz")


# ── set_sensor_feature semantics ──────────────────────────────────────

class TestSetSensorFeature(unittest.TestCase):
    def _stop_backend(self, be):
        be._running = False
        reader = be._reader
        if reader is not None and reader.is_alive():
            reader.join(timeout=2.0)

    def test_live_refusal_falls_back_to_stop_apply_restart(self):
        stub = _StubCam(features=_full_features(),
                        locked_live={"PixelReadoutRate"})
        be = _backend(stub)
        be._running = True                    # pretend the stream is live
        try:
            ok = be.set_sensor_feature("andor_readout_rate", "216 MHz")
            self.assertTrue(ok)
            # Sequence: stop → set → start (mutation guard: deleting the
            # restart leaves a dead feed and fails this).
            i_stop = stub.ops.index("stop")
            i_set = stub.ops.index("set:PixelReadoutRate")
            i_start = stub.ops.index("start")
            self.assertLess(i_stop, i_set)
            self.assertLess(i_set, i_start)
            self.assertEqual(be.get_sensor_feature("andor_readout_rate"),
                             "216 MHz")
        finally:
            self._stop_backend(be)

    def test_stream_restarts_even_when_set_fails_stopped(self):
        stub = _StubCam(features=_full_features(),
                        always_locked={"PixelReadoutRate"})
        be = _backend(stub)
        be._running = True
        try:
            ok = be.set_sensor_feature("andor_readout_rate", "216 MHz")
            self.assertFalse(ok)
            # A refused setting must not cost the live feed.
            self.assertIn("start", stub.ops)
            self.assertTrue(be._running)
        finally:
            self._stop_backend(be)

    def test_stale_persisted_enum_refused(self):
        stub = _StubCam(features=_full_features())
        be = _backend(stub)
        ok = be.set_sensor_feature("andor_gain_mode",
                                   "12-bit (a different SDK's spelling)")
        self.assertFalse(ok)
        self.assertNotIn("SimplePreAmpGainControl", dict(stub.sets))

    def test_unknown_key_refused(self):
        be = _backend(_StubCam(features=_full_features()))
        self.assertFalse(be.set_sensor_feature("andor_bogus", True))

    def test_bool_set_applies_live(self):
        stub = _StubCam(features=_full_features())
        be = _backend(stub)
        self.assertTrue(be.set_sensor_feature("andor_noise_filter", False))
        self.assertIs(dict(stub.sets)["SpuriousNoiseFilter"], False)


# ── Clip level tracking ───────────────────────────────────────────────

class TestClipLevel(unittest.TestCase):
    def test_bit_depth_feature_wins(self):
        feats = _full_features(gain_value="16-bit (low noise & high well capacity)")
        feats["BitDepth"] = {"values": ["11 Bit", "12 Bit", "16 Bit"],
                             "value": "12 Bit"}
        be = _backend(_StubCam(features=feats))
        self.assertEqual(be._read_clip_level(), 4095)

    def test_gain_mode_fallback(self):
        be = _backend(_StubCam(features=_full_features(
            gain_value="12-bit (low noise)")))
        self.assertEqual(be._read_clip_level(), 4095)

    def test_final_fallback_full_16bit(self):
        be = _backend(_StubCam(features={}))
        self.assertEqual(be._read_clip_level(), 65535)

    def test_gain_mode_change_refreshes_clip_level(self):
        feats = _full_features(gain_value="16-bit (low noise & high well capacity)")
        stub = _StubCam(features=feats)
        be = _backend(stub)
        be._clip_level = be._read_clip_level()
        self.assertEqual(be._clip_level, 65535)
        self.assertTrue(be.set_sensor_feature("andor_gain_mode",
                                              "12-bit (low noise)"))
        self.assertEqual(be._clip_level, 4095)


# ── Capability / delegation (FakeAndorCam through real widget/manager) ─

@unittest.skipUnless(CAMERA_AVAILABLE, "camera backend unavailable")
class TestCapsAndDelegates(unittest.TestCase):
    def test_caps_advertise_sensor_features_and_raw_stats(self):
        _mgr, cam = _andor_widget()
        ctrls = cam.hardware_capabilities()["controls"]
        for key in ANDOR_SENSOR_FEATURE_KEYS:
            self.assertIn(key, ctrls, key)
        self.assertEqual(ctrls["andor_readout_rate"]["kind"], "enum")
        self.assertTrue(ctrls["andor_readout_rate"]["values"])
        self.assertEqual(ctrls["andor_sensor_cooling"]["kind"], "bool")
        self.assertIn("andor_raw_stats", ctrls)

    def test_widget_delegate_sets_feature(self):
        _mgr, cam = _andor_widget()
        self.assertTrue(cam.set_hw_andor_feature("andor_readout_rate",
                                                 "280 MHz"))
        self.assertEqual(cam._andor.get_sensor_feature("andor_readout_rate"),
                         "280 MHz")

    def test_manager_fanout(self):
        mgr, cam = _andor_widget()
        self.assertTrue(mgr.set_hw_andor_feature(0, "andor_noise_filter",
                                                 False))
        self.assertIs(cam._andor.get_sensor_feature("andor_noise_filter"),
                      False)
        self.assertTrue(mgr.reset_andor_sensor_defaults(0))
        self.assertIs(cam._andor.get_sensor_feature("andor_noise_filter"),
                      True)

    def test_non_andor_slot_returns_false(self):
        mgr = CameraManager(max_cameras=1)
        self.assertFalse(mgr.set_hw_andor_feature(0, "andor_noise_filter",
                                                  True))
        self.assertFalse(mgr.reset_andor_sensor_defaults(0))

    def test_get_settings_carries_sensor_keys(self):
        _mgr, cam = _andor_widget()
        st = cam.get_hw_settings()
        for key in ANDOR_SENSOR_FEATURE_KEYS:
            self.assertIn(key, st, key)
        self.assertEqual(st["raw_clip_level"], 65535)


# ── Persistence: ONE snapshot for both write sites ────────────────────

class TestHwControlsSnapshot(unittest.TestCase):
    def test_key_list_complete(self):
        # The full persisted set — including the three andor display-scale
        # keys the bulk save used to drop, and the five v7.13 sensor keys.
        expected = {
            "auto_exposure", "exposure_us", "exposure_gain_pct",
            "gamma", "brightness", "contrast",
            "andor_auto_scale", "andor_scale_lo", "andor_scale_hi",
            "andor_sensor_cooling", "andor_readout_rate", "andor_gain_mode",
            "andor_noise_filter", "andor_blemish_correction",
        }
        self.assertEqual(set(PERSISTED_HW_CONTROL_KEYS), expected)

    def test_snapshot_from_readback(self):
        st = {"exposure_us": 5000, "andor_gain_mode": "16-bit (x)",
              "resolution": (1024, 1024), "unrelated": "dropped"}
        snap = hw_controls_snapshot(st)
        self.assertEqual(snap["exposure_us"], 5000)
        self.assertEqual(snap["andor_gain_mode"], "16-bit (x)")
        self.assertEqual(snap["resolution"], [1024, 1024])
        self.assertNotIn("unrelated", snap)
        self.assertIsNone(snap["gamma"])     # absent → None (store drops it)

    def _calls_snapshot(self, func) -> bool:
        """AST check: does ``func`` CALL hw_controls_snapshot? (A substring
        match would pass on an import line alone — the documented v7.10/v7.11
        weak-guard trap.)"""
        src = inspect.getsource(func)
        # Dedent for ast
        import textwrap
        tree = ast.parse(textwrap.dedent(src))
        for node in ast.walk(tree):
            if isinstance(node, ast.Call):
                f = node.func
                name = getattr(f, "id", getattr(f, "attr", ""))
                if name == "hw_controls_snapshot":
                    return True
        return False

    def test_dialog_persist_routes_through_snapshot(self):
        from gui.dialogs.camera_settings_dialog import CameraSettingsDialog
        self.assertTrue(self._calls_snapshot(CameraSettingsDialog._persist))

    def test_bulk_save_routes_through_snapshot(self):
        from gui.pages.hardware_setup import HardwareSetupPage
        self.assertTrue(
            self._calls_snapshot(HardwareSetupPage._on_save_camera_settings))


# ── Restore order in _apply_hw_controls ───────────────────────────────

class _RecordingMgr:
    """Fake manager recording every hw call in order."""

    def __init__(self):
        self.calls = []

    def set_hw_auto_exposure(self, i, v):
        self.calls.append(("auto_exposure", v))
        return True

    def set_hw_exposure_us(self, i, v):
        self.calls.append(("exposure_us", v))
        return True

    def set_hw_exposure_gain(self, i, v):
        self.calls.append(("exposure_gain", v))
        return True

    def set_hw_gamma(self, i, v):
        self.calls.append(("gamma", v))
        return True

    def set_hw_brightness(self, i, v):
        self.calls.append(("brightness", v))
        return True

    def set_hw_contrast(self, i, v):
        self.calls.append(("contrast", v))
        return True

    def set_hw_andor_feature(self, i, key, v):
        self.calls.append((key, v))
        return True

    def set_hw_andor_auto_scale(self, i, v):
        self.calls.append(("andor_auto_scale", v))
        return True

    def set_hw_andor_scale_lo(self, i, v):
        self.calls.append(("andor_scale_lo", v))
        return True

    def set_hw_andor_scale_hi(self, i, v):
        self.calls.append(("andor_scale_hi", v))
        return True

    def set_capture_resolution(self, i, w, h):
        self.calls.append(("resolution", (w, h)))
        return (w, h)


class TestApplyHwControlsOrder(unittest.TestCase):
    def _apply(self, mgr, hw):
        from gui.pages.hardware_setup import HardwareSetupPage
        fake_self = SimpleNamespace(
            _camera_manager=mgr,
            _config=SimpleNamespace(camera_for_role=lambda role: -1))
        HardwareSetupPage._apply_hw_controls(fake_self, 0, hw)

    def test_gain_mode_then_rate_then_bools_then_exposure_then_display(self):
        mgr = _RecordingMgr()
        self._apply(mgr, {
            "exposure_us": 500_000,
            "andor_gain_mode": "16-bit (low noise & high well capacity)",
            "andor_readout_rate": "216 MHz",
            "andor_sensor_cooling": True,
            "andor_noise_filter": True,
            "andor_blemish_correction": False,
            "andor_auto_scale": False,
            "andor_scale_lo": 100,
            "andor_scale_hi": 9000,
        })
        keys = [k for k, _v in mgr.calls]
        # Pinned order: gain mode constrains bit depth + legal readout rates;
        # the achievable EXPOSURE range depends on the readout/gain constraint
        # set, so the saved exposure must be applied AFTER the sensor features
        # (v7.13.x — it used to run first, against open-time defaults); the
        # manual display levels are raw counts whose meaning depends on bit
        # depth, so they must land LAST (after the auto-scale flag).
        self.assertLess(keys.index("andor_gain_mode"),
                        keys.index("andor_readout_rate"))
        self.assertLess(keys.index("andor_readout_rate"),
                        keys.index("andor_sensor_cooling"))
        self.assertLess(keys.index("andor_blemish_correction"),
                        keys.index("exposure_us"))
        self.assertLess(keys.index("exposure_us"),
                        keys.index("andor_auto_scale"))
        self.assertLess(keys.index("andor_auto_scale"),
                        keys.index("andor_scale_lo"))
        self.assertLess(keys.index("andor_scale_lo"),
                        keys.index("andor_scale_hi"))

    def test_non_string_enum_values_skipped(self):
        mgr = _RecordingMgr()
        self._apply(mgr, {"andor_gain_mode": 12, "andor_readout_rate": None,
                          "andor_noise_filter": "yes"})
        keys = [k for k, _v in mgr.calls]
        self.assertNotIn("andor_gain_mode", keys)
        self.assertNotIn("andor_readout_rate", keys)
        self.assertNotIn("andor_noise_filter", keys)   # non-bool skipped


# ── Settings dialog rows ──────────────────────────────────────────────

@unittest.skipUnless(CAMERA_AVAILABLE, "camera backend unavailable")
class TestDialogSensorRows(unittest.TestCase):
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

    def test_rows_visible_and_populated(self):
        dlg, _mgr, cam = self._dialog()
        for key, chk in dlg._sensor_checks.items():
            self.assertTrue(chk.isVisibleTo(dlg), key)
        for key, (lbl, combo) in dlg._sensor_combos.items():
            self.assertTrue(combo.isVisibleTo(dlg), key)
            self.assertGreater(combo.count(), 1, key)
        # Current values reflected
        combo = dlg._sensor_combos["andor_gain_mode"][1]
        self.assertEqual(combo.currentText(),
                         "16-bit (low noise & high well capacity)")
        self.assertTrue(dlg._sensor_checks["andor_sensor_cooling"].isChecked())

    def test_rows_hidden_without_backend(self):
        from gui.dialogs.camera_settings_dialog import CameraSettingsDialog
        mgr = CameraManager(max_cameras=1)
        dlg = CameraSettingsDialog(mgr, 0)
        for key, chk in dlg._sensor_checks.items():
            self.assertFalse(chk.isVisibleTo(dlg), key)
        for key, (_lbl, combo) in dlg._sensor_combos.items():
            self.assertFalse(combo.isVisibleTo(dlg), key)
        self.assertFalse(dlg._signal_group.isVisibleTo(dlg))

    def test_combo_change_applies_and_persists(self):
        dlg, _mgr, cam = self._dialog()
        combo = dlg._sensor_combos["andor_readout_rate"][1]
        idx = combo.findText("280 MHz")
        self.assertGreaterEqual(idx, 0)
        combo.setCurrentIndex(idx)
        self.assertEqual(cam._andor.get_sensor_feature("andor_readout_rate"),
                         "280 MHz")
        from SupportClasses.CameraCalibrationStore import get_store
        hw = get_store().get_hw_controls("andor:SN-ZYLA-001")
        self.assertEqual(hw["andor_readout_rate"], "280 MHz")

    def test_checkbox_applies_and_persists(self):
        dlg, _mgr, cam = self._dialog()
        dlg._sensor_checks["andor_noise_filter"].setChecked(False)
        self.assertIs(cam._andor.get_sensor_feature("andor_noise_filter"),
                      False)
        from SupportClasses.CameraCalibrationStore import get_store
        hw = get_store().get_hw_controls("andor:SN-ZYLA-001")
        self.assertIs(hw["andor_noise_filter"], False)

    def test_defaults_button_restores_sensor_defaults(self):
        dlg, _mgr, cam = self._dialog()
        combo = dlg._sensor_combos["andor_readout_rate"][1]
        combo.setCurrentIndex(combo.findText("540 MHz - fastest readout"))
        self.assertEqual(cam._andor.get_sensor_feature("andor_readout_rate"),
                         "540 MHz - fastest readout")
        dlg._on_defaults_clicked()
        self.assertEqual(cam._andor.get_sensor_feature("andor_readout_rate"),
                         "216 MHz - lowest noise")


# ── Feature table sanity ──────────────────────────────────────────────

class TestFeatureTable(unittest.TestCase):
    def test_keys_and_kinds(self):
        keys = [row[0] for row in ANDOR_SENSOR_FEATURES]
        self.assertEqual(keys, list(ANDOR_SENSOR_FEATURE_KEYS))
        for key, sdk, kind, default in ANDOR_SENSOR_FEATURES:
            self.assertTrue(key.startswith("andor_"), key)
            self.assertIn(kind, ("bool", "enum"))
            if kind == "enum":
                self.assertIsInstance(default, tuple,
                                      f"{key}: enum defaults must be match-"
                                      "token tuples, never literal strings")
            else:
                self.assertIsInstance(default, bool)

    def test_every_persisted_sensor_key_is_in_the_table(self):
        table_keys = set(ANDOR_SENSOR_FEATURE_KEYS)
        persisted_sensor = {k for k in PERSISTED_HW_CONTROL_KEYS
                            if k.startswith("andor_") and
                            k not in ("andor_auto_scale", "andor_scale_lo",
                                      "andor_scale_hi")}
        self.assertEqual(table_keys, persisted_sensor)


if __name__ == "__main__":
    unittest.main()

"""
v7.19 — the fluorescence camera preset: what it contains, in what order it is
applied, and that leaving the page puts the camera back.

Operator: *"the camera needs to be set out of autoexposure mode and auto white
black balance mode when doing these mosaics … a toggle button for camera default
settings and camera fluorescent mosaic settings and by default the camera should
switch to fluorescent mosaic settings while on that workflow."*

THE ONE THAT MATTERS MOST is ``TestTheRedundantWriteGuardIsUsed``. The Libra 25
collapses its exposure to the 6.3 µs sensor minimum when a capability is written
the value it already holds (hardware finding, ``tucam_backend._capa_set``), and
this preset is applied on EVERY page entry — so the no-change case is the common
one. A raw SDK write here would black out the live image every time the operator
opened the workflow.
"""

from __future__ import annotations

import unittest
from unittest import mock

from gui.widgets.hw_controls_snapshot import (
    PERSISTED_HW_CONTROL_KEYS, apply_hw_controls, fluorescence_preset,
    hw_controls_snapshot)


def _caps(*names, ranges=None):
    ranges = ranges or {}
    return {"controls": {n: {"range": ranges.get(n)} for n in names}}


class _RecordingMgr:
    """Records every hw call in order, like the v7.13 restore-order fake."""

    def __init__(self, readback=None):
        self.calls = []
        self._readback = readback or {}

    def get_hw_settings(self, _i):
        return dict(self._readback)

    def set_hw_auto_exposure(self, _i, v):
        self.calls.append(("auto_exposure", v))
        return True

    def set_hw_auto_levels(self, _i, v):
        self.calls.append(("auto_levels", v))
        return True

    def set_hw_exposure_us(self, _i, v):
        self.calls.append(("exposure_us", v))
        return True

    def set_hw_exposure_gain(self, _i, v):
        self.calls.append(("exposure_gain_pct", v))
        return True

    def set_hw_gamma(self, _i, v):
        self.calls.append(("gamma", v))
        return True

    def set_hw_brightness(self, _i, v):
        self.calls.append(("brightness", v))
        return True

    def set_hw_contrast(self, _i, v):
        self.calls.append(("contrast", v))
        return True

    def set_hw_andor_auto_scale(self, _i, v):
        self.calls.append(("andor_auto_scale", v))
        return True

    def set_hw_andor_scale_lo(self, _i, v):
        self.calls.append(("andor_scale_lo", v))
        return True

    def set_hw_andor_scale_hi(self, _i, v):
        self.calls.append(("andor_scale_hi", v))
        return True

    def set_capture_resolution(self, _i, w, h):
        self.calls.append(("resolution", (w, h)))
        return (w, h)

    def keys(self):
        return [k for k, _v in self.calls]


class TestWhatThePresetContains(unittest.TestCase):
    def test_it_stops_the_camera_deciding_for_itself(self):
        p = fluorescence_preset(_caps("auto_exposure", "auto_levels",
                                      "andor_auto_scale"))
        self.assertIs(p["auto_exposure"], False)
        self.assertIs(p["auto_levels"], False)
        self.assertIs(p["andor_auto_scale"], False)

    def test_auto_levels_is_not_forgotten(self):
        """The operator asked for "auto white black balance" specifically, and
        it is the only one of the three that changes RAW pixel values."""
        self.assertIn("auto_levels",
                      fluorescence_preset(_caps("auto_levels")))

    def test_it_neutralises_the_software_corrections(self):
        p = fluorescence_preset(_caps(
            "gamma", "contrast", "brightness",
            ranges={"gamma": (1, 255, 100), "contrast": (0, 255, 128)}))
        # The camera's OWN declared default beats our guess at "neutral":
        # the Tucsen states gamma as an integer centred on 100, not 1.0.
        self.assertEqual(p["gamma"], 100)
        self.assertEqual(p["contrast"], 128)
        # No declared default → the documented neutral.
        self.assertEqual(p["brightness"], 0.0)

    def test_a_camera_without_a_control_is_never_written(self):
        """A webcam slot must get a no-op preset, not fabricated writes."""
        self.assertEqual(fluorescence_preset({"controls": {}}), {})
        self.assertEqual(fluorescence_preset({}), {})
        p = fluorescence_preset(_caps("auto_exposure"))
        self.assertEqual(set(p), {"auto_exposure"})

    def test_every_preset_key_is_persistable(self):
        """Otherwise the entry snapshot could not restore what the preset
        changed — the camera would keep a preset value after leaving."""
        p = fluorescence_preset(_caps(
            "auto_exposure", "auto_levels", "andor_auto_scale",
            "gamma", "contrast", "brightness"))
        for key in p:
            self.assertIn(key, PERSISTED_HW_CONTROL_KEYS, key)


class TestTheApplyOrder(unittest.TestCase):
    """The order is load-bearing; see apply_hw_controls' comments."""

    def test_auto_flags_precede_the_values_they_would_clobber(self):
        mgr = _RecordingMgr()
        apply_hw_controls(mgr, 0, {
            "auto_exposure": False, "auto_levels": False,
            "andor_gain_mode": "16-bit", "andor_readout_rate": "216 MHz",
            "exposure_us": 500_000, "gamma": 100,
            "andor_auto_scale": False, "andor_scale_lo": 100,
            "andor_scale_hi": 9000,
        })
        k = mgr.keys()
        # Turning display auto-scale off SEEDS the levels from the last auto
        # frame, so stored levels must be applied after the flag to win.
        self.assertLess(k.index("andor_auto_scale"), k.index("andor_scale_lo"))
        self.assertLess(k.index("andor_scale_lo"), k.index("andor_scale_hi"))
        # Exposure is applied against the constraint set it was saved under.
        self.assertLess(k.index("exposure_us"), k.index("andor_auto_scale"))
        # The "stop deciding for itself" pair comes first of all.
        self.assertLess(k.index("auto_exposure"), k.index("exposure_us"))
        self.assertLess(k.index("auto_levels"), k.index("exposure_us"))

    def test_a_manager_without_auto_levels_is_not_called(self):
        """Only the Tucsen implements it; every other backend must be skipped
        silently rather than raising or being handed a write that does nothing.
        """
        class Old:
            """A pre-v7.19 manager: no set_hw_auto_levels at all.

            Written standalone rather than subclassing _RecordingMgr and
            deleting the attribute — a `del` on the subclass just finds the
            base class's method again, which is how the first version of this
            test passed while proving nothing.
            """

            def __init__(self):
                self.calls = []

            def set_hw_gamma(self, _i, v):
                self.calls.append(("gamma", v))
                return True

            def keys(self):
                return [k for k, _v in self.calls]

        mgr = Old()
        self.assertFalse(hasattr(mgr, "set_hw_auto_levels"))
        apply_hw_controls(mgr, 0, {"auto_levels": False, "gamma": 1.0})
        self.assertNotIn("auto_levels", mgr.keys())
        self.assertIn("gamma", mgr.keys())

    def test_a_non_boolean_auto_value_is_ignored(self):
        """OpenCV reports -1.0; bool() would wrongly force auto ON."""
        mgr = _RecordingMgr()
        apply_hw_controls(mgr, 0, {"auto_exposure": -1.0, "auto_levels": 3})
        self.assertNotIn("auto_exposure", mgr.keys())
        self.assertNotIn("auto_levels", mgr.keys())

    def test_skip_resolution_is_honoured(self):
        """The microscope slot's resolution has one source of truth."""
        mgr = _RecordingMgr()
        apply_hw_controls(mgr, 0, {"resolution": [2600, 2048]},
                          skip_resolution=True)
        self.assertNotIn("resolution", mgr.keys())
        apply_hw_controls(mgr, 0, {"resolution": [2600, 2048]})
        self.assertIn("resolution", mgr.keys())

    def test_hardware_setup_still_routes_through_the_shared_applier(self):
        """Two hand-written appliers is how the two persisted key lists drifted
        (the whole reason hw_controls_snapshot exists)."""
        import ast
        import inspect
        import textwrap
        from gui.pages.hardware_setup import HardwareSetupPage
        src = textwrap.dedent(
            inspect.getsource(HardwareSetupPage._apply_hw_controls))
        names = {getattr(n.func, "id", getattr(n.func, "attr", ""))
                 for n in ast.walk(ast.parse(src)) if isinstance(n, ast.Call)}
        self.assertIn("apply_hw_controls", names)


class TestTheRedundantWriteGuardIsUsed(unittest.TestCase):
    """Applying the preset twice must not re-write an unchanged capability.

    ⚠ On the Libra 25 a redundant capability write resets the exposure to the
    6.3 µs sensor minimum — the live image goes black. The preset runs on every
    page entry, so this is the ordinary path, not an edge case.
    """

    def _backend(self):
        from gui.widgets import tucam_backend as tb
        be = tb.TUCamBackend.__new__(tb.TUCamBackend)
        be._lib = mock.MagicMock()
        be._handle = 1
        self.writes = []
        state = {tb.TUIDC_ATEXPOSURE: 1, tb.TUIDC_ATLEVELS: 1}

        def _set(_h, cid, val):
            cid = int(getattr(cid, "value", cid))
            val = int(getattr(val, "value", val))
            self.writes.append((cid, val))
            state[cid] = val
            return tb.TUCAMRET_SUCCESS

        be._lib.TUCAM_Capa_SetValue.side_effect = _set
        be._capa_attr = lambda cid: object() if cid in state else None
        be._capa_get = lambda cid: state.get(cid)
        return be, tb

    def test_set_auto_levels_goes_through_capa_set(self):
        be, tb = self._backend()
        self.assertTrue(be.set_auto_levels(False))
        self.assertEqual(self.writes, [(tb.TUIDC_ATLEVELS, 0)])

    def test_a_second_identical_apply_writes_NOTHING(self):
        be, tb = self._backend()
        be.set_auto_levels(False)
        be.set_auto_exposure(False)
        before = list(self.writes)
        be.set_auto_levels(False)      # the second page entry
        be.set_auto_exposure(False)
        self.assertEqual(self.writes, before,
                         "a redundant capability write blacks out the Libra 25")

    def test_an_unsupported_capability_never_reaches_the_sdk(self):
        be, tb = self._backend()
        be._capa_attr = lambda cid: None
        self.assertFalse(be.set_auto_levels(False))
        self.assertEqual(self.writes, [])

    def test_get_auto_levels_reports_unknown_as_none(self):
        be, _tb = self._backend()
        be._capa_get = lambda cid: None
        self.assertIsNone(be.get_auto_levels())


class TestTheCameraIsPutBack(unittest.TestCase):
    """Entry snapshot → preset → restore, on the real page methods."""

    def _page(self):
        import os
        import tempfile
        os.environ.setdefault("MEBP_WORKFLOW_SETTINGS_DIR", tempfile.mkdtemp())
        try:
            from PySide6.QtWidgets import QApplication
        except Exception:                                    # pragma: no cover
            self.skipTest("PySide6 not available")
        QApplication.instance() or QApplication([])

        entry = {
            "auto_exposure": True, "auto_levels": True,
            "andor_auto_scale": True, "gamma": 140, "exposure_us": 20000.0,
        }
        mgr = _RecordingMgr(readback=entry)
        mgr.hardware_capabilities = lambda _i: _caps(
            "auto_exposure", "auto_levels", "andor_auto_scale", "gamma",
            ranges={"gamma": (1, 255, 100)})

        class SL:
            xy_min_x = xy_min_y = 0.0
            xy_max_x, xy_max_y = 120000.0, 80000.0

        class Ctrl:
            safety_limits = SL()
            is_zp_connected = False
            zero_position = {"x": 0.0, "y": 0.0, "Z": 0.0}

        from gui.pages.workflows.fluorescence_mosaic_workflow import (
            FluorescenceMosaicWorkflowPage)
        pg = FluorescenceMosaicWorkflowPage(
            controller=Ctrl(), settings=object(), camera_manager=None)
        pg._camera_manager = mgr
        return pg, mgr, entry

    def test_the_snapshot_is_taken_before_the_preset(self):
        """Snapshotting after would make "Camera defaults" restore the preset.
        """
        pg, _mgr, entry = self._page()
        pg._capture_entry_hw()
        pg._apply_camera_preset()
        self.assertIs(pg._entry_hw["auto_exposure"], True)
        self.assertIs(pg._entry_hw["auto_levels"], True)

    def test_the_snapshot_is_taken_once_per_visit(self):
        """Otherwise toggling back and forth slowly turns the operator's own
        settings INTO the preset."""
        pg, mgr, _entry = self._page()
        pg._capture_entry_hw()
        pg._apply_camera_preset()
        mgr._readback = {"auto_exposure": False, "auto_levels": False}
        pg._capture_entry_hw()
        self.assertIs(pg._entry_hw["auto_exposure"], True)

    def test_the_preset_turns_both_autos_off(self):
        pg, mgr, _entry = self._page()
        pg._capture_entry_hw()
        pg._apply_camera_preset()
        self.assertIn(("auto_exposure", False), mgr.calls)
        self.assertIn(("auto_levels", False), mgr.calls)
        self.assertIn(("andor_auto_scale", False), mgr.calls)

    def test_toggling_to_defaults_restores_the_entry_state(self):
        pg, mgr, _entry = self._page()
        pg._capture_entry_hw()
        pg._apply_camera_preset()
        mgr.calls.clear()
        pg.on_panel_camera_preset_changed(False)
        self.assertIn(("auto_exposure", True), mgr.calls)
        self.assertIn(("auto_levels", True), mgr.calls)
        self.assertIn(("gamma", 140), mgr.calls)

    def test_toggling_back_re_applies_the_preset(self):
        pg, mgr, _entry = self._page()
        pg._capture_entry_hw()
        pg.on_panel_camera_preset_changed(False)
        mgr.calls.clear()
        pg.on_panel_camera_preset_changed(True)
        self.assertIn(("auto_exposure", False), mgr.calls)

    def test_a_scan_refuses_the_toggle(self):
        pg, mgr, _entry = self._page()
        pg._capture_entry_hw()
        pg.is_scanning = lambda: True
        mgr.calls.clear()
        pg.on_panel_camera_preset_changed(False)
        self.assertEqual(mgr.calls, [])

    def test_nothing_is_written_to_the_calibration_store(self):
        """The preset is a mode this workflow runs IN, not a change to the
        camera's saved configuration."""
        import ast
        import inspect
        import textwrap
        from gui.pages.workflows.fluorescence_mosaic_workflow import (
            FluorescenceMosaicWorkflowPage as P)
        for fn in (P._apply_camera_preset, P._restore_entry_hw,
                   P._capture_entry_hw):
            src = textwrap.dedent(inspect.getsource(fn))
            names = {getattr(n.func, "id", getattr(n.func, "attr", ""))
                     for n in ast.walk(ast.parse(src))
                     if isinstance(n, ast.Call)}
            self.assertNotIn("set_hw_controls", names, fn.__name__)


if __name__ == "__main__":
    unittest.main()

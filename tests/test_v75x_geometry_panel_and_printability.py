"""test_v75x_geometry_panel_and_printability.py — GUI smoke + behaviour tests
for the Geometry Panel dialog and the Sketch printability check (v7.5.x).

Offscreen Qt; no hardware. Simulated runs use a fast fake stage model so the
panel's worker path is exercised end-to-end.
"""

import json
import os
import tempfile
import time
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication

from SupportClasses import XYChallenge as XC
from SupportClasses import XYPathSimulator as PS
from tests.support.store_fixture import use_temp_store

_app = QApplication.instance() or QApplication([])


def _stamp_measured(store):
    """Give the temp store a complete set of measured characteristics
    (the real ME3B V1 numbers)."""
    store.set_velocity_dead_time_s(0.067, n=5, spread_s=0.0005, tau_s=0.027)
    store.set_xy_max_speed_um_s(5945.6)
    store.set_control_loop_ms(31.75)


class TestGeometryPanelDialog(unittest.TestCase):
    def setUp(self):
        self.store = use_temp_store(self)
        self._tmp_logs = tempfile.mkdtemp()
        os.environ["MEBP_CHALLENGE_LOG_DIR"] = self._tmp_logs
        self.addCleanup(os.environ.pop, "MEBP_CHALLENGE_LOG_DIR", None)

    def _dlg(self, controller=None, safe_z=None):
        from gui.dialogs.xy_geometry_panel_dialog import GeometryPanelDialog
        return GeometryPanelDialog(controller, safe_z=safe_z)

    def test_builds_without_measurement_and_disables_sim(self):
        dlg = self._dlg()
        self.assertFalse(dlg._sim_btn.isEnabled())
        self.assertIn("NOT measured", dlg._char_lbl.text())
        dlg.close()

    def test_sim_enabled_when_measured_and_hw_gated(self):
        _stamp_measured(self.store)
        dlg = self._dlg(controller=None)
        self.assertTrue(dlg._sim_btn.isEnabled())
        self.assertFalse(dlg._hw_btn.isEnabled())   # no controller
        dlg.close()

    def test_hw_gate_requires_safe_z_when_zp_connected(self):
        _stamp_measured(self.store)

        class Ctrl:
            is_xy_connected = True
            is_zp_connected = True
        dlg = self._dlg(controller=Ctrl(), safe_z=None)
        self.assertFalse(dlg._hw_btn.isEnabled())
        dlg.close()
        dlg2 = self._dlg(controller=Ctrl(), safe_z=44.0)
        self.assertTrue(dlg2._hw_btn.isEnabled())
        dlg2.close()

    def test_simulated_panel_fills_cells_and_saves(self):
        _stamp_measured(self.store)
        dlg = self._dlg()
        dlg._sizes.setText("2")
        dlg._speed.setValue(3.0)
        dlg._start("simulated")
        t0 = time.monotonic()
        while dlg._thread is not None and time.monotonic() - t0 < 120:
            _app.processEvents()
            time.sleep(0.02)
        _app.processEvents()
        self.assertIsNone(dlg._thread)
        self.assertEqual(len(dlg._records),
                         len(dlg.SHAPES))          # one size × all shapes
        filled = [c for c in dlg._cells.values() if c.sim is not None]
        self.assertEqual(len(filled), len(dlg.SHAPES))
        self.assertIn("saved", dlg._status.text())
        dlg.close()

    def test_load_panel_data_normalises_world_coords(self):
        _stamp_measured(self.store)
        dlg = self._dlg()
        # a hardware-script-style record: world-frame coords around (58, 37)
        ideal = [[58.0 + p[0], 37.0 + p[1]]
                 for p in XC.make_shape("Square", 2.0, 0.5)]
        data = {"cells": [{
            "shape": "Square", "size_mm": 2.0, "status": "ok",
            "ideal": ideal, "samples": [list(p) + [0.1] for p in ideal],
            "actual": {"p95_um": 50.0, "rms_um": 20.0, "max_um": 60.0,
                       "dither_ratio": 1.0, "completion_frac": 1.0,
                       "corner_p95_um": 40.0, "wall_s": 3.0},
            "actual_verdict": "marginal", "actual_reason": "arrived"}]}
        dlg.load_panel_data(data)
        cell = dlg._cells[("Square", 2.0)]
        self.assertIsNotNone(cell.hw)
        xs = [p[0] for p in cell.ideal]
        self.assertLess(max(abs(min(xs)), abs(max(xs))), 5.0)  # re-centred
        self.assertIn("marginal", dlg._labels[("Square", 2.0)].text())
        dlg.close()

    def test_parse_sizes_garbage_safe(self):
        dlg = self._dlg()
        dlg._sizes.setText("2, bogus; 5,  1000")
        self.assertEqual(dlg._parse_sizes(), [2.0, 5.0])
        dlg._sizes.setText("")
        self.assertEqual(dlg._parse_sizes(), [2.0, 5.0, 10.0])
        dlg.close()


class TestPrintabilityDialog(unittest.TestCase):
    def setUp(self):
        self.store = use_temp_store(self)

    def _traj(self, pts, start_pump=0.0):
        rows = []
        p = start_pump
        for i, (x, y) in enumerate(pts):
            if i:
                p += 0.01
            rows.append([x, y, 0.2, p, 0.0, 0.0, 0.0])
        return rows

    def _wait(self, dlg, timeout=120):
        t0 = time.monotonic()
        while dlg._thread is not None and time.monotonic() - t0 < timeout:
            _app.processEvents()
            time.sleep(0.02)
        _app.processEvents()

    def test_unmeasured_machine_shows_guidance(self):
        from gui.dialogs.sketch_printability_dialog import (
            SketchPrintabilityDialog)
        dlg = SketchPrintabilityDialog(self._traj([(0, 0), (5, 0)]),
                                       store=self.store)
        self._wait(dlg)
        self.assertIn("one-click XY calibration", dlg._banner.text())
        dlg.close()

    def test_gentle_path_passes_tight_star_fails(self):
        _stamp_measured(self.store)
        from gui.dialogs.sketch_printability_dialog import (
            SketchPrintabilityDialog)
        line = self._traj([(i * 1.0, 0.0) for i in range(11)])
        dlg = SketchPrintabilityDialog(line, store=self.store)
        self._wait(dlg)
        self.assertIsNotNone(dlg._last_check)
        self.assertTrue(dlg._banner.text().startswith(("✅", "⚠")))

        star = self._traj(XC.make_shape("Star", 2.0, 0.3))
        dlg2 = SketchPrintabilityDialog(star, store=self.store)
        self._wait(dlg2)
        self.assertIsNotNone(dlg2._last_check)
        self.assertEqual(dlg2._last_check["verdict"], "fail")
        self.assertIn("✖", dlg2._banner.text())
        dlg2.close()
        dlg.close()

    def test_speed_change_resimulates(self):
        _stamp_measured(self.store)
        from gui.dialogs.sketch_printability_dialog import (
            SketchPrintabilityDialog)
        dlg = SketchPrintabilityDialog(
            self._traj([(i * 1.0, 0.0) for i in range(6)]), store=self.store)
        self._wait(dlg)
        first = dlg._last_check
        dlg._speed.setValue(1.0)
        self._wait(dlg)
        self.assertIsNotNone(dlg._last_check)
        self.assertIsNot(dlg._last_check, first)
        dlg.close()


class TestSketchPageHook(unittest.TestCase):
    def test_page_has_printability_button(self):
        """The Sketch page exposes the check without needing a full app."""
        import inspect
        from gui.pages import print_builder_sketch as mod
        src = inspect.getsource(mod)
        self.assertIn("_check_printability", src)
        self.assertIn("SketchPrintabilityDialog", src)


class TestChallengeDialogHook(unittest.TestCase):
    def test_dialog_has_geometry_button(self):
        import inspect
        from gui.dialogs import xy_challenge_dialog as mod
        src = inspect.getsource(mod)
        self.assertIn("_open_geometry_panel", src)
        self.assertIn("GeometryPanelDialog", src)


if __name__ == "__main__":
    unittest.main()

"""
test_v713_cell_targeting_surface_z.py — removal Z from the sample surface.

Cell Targeting & Removal can now take its per-target removal height from the
fluorescence mosaic's measured SAMPLE SURFACE (cells often sit above the well
bottom, e.g. on hydrogel). Pins:
- PickPlaceTarget's optional ``pick_z_zref_mm`` override (conditional-emit
  serialization; forward-compatible from_dict);
- the executor consulting the override in _execute_cell_removal (AST-checked,
  the repo's guard style — a substring match would pass on a comment);
- the page's _surface_z_resolver: the full gate chain (survey present,
  focus↔needle datum present, plate bottom taught, surface not below the
  glass), the offset arithmetic through the polarity-safe height frame, and
  the per-target low-confidence fallback.
"""

import ast
import inspect
import os
import sys
import tempfile
import textwrap
import unittest
from pathlib import Path
from types import SimpleNamespace

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication

_app = QApplication.instance() or QApplication(sys.argv)

from SupportClasses.PickAndPlaceManager import PickPlaceTarget

PLATE_BOTTOM_ZREF = 20.0     # fake controller's plate bottom (zref mm)


class TestPickPlaceTargetOverride(unittest.TestCase):
    def _t(self, **kw):
        return PickPlaceTarget(target_id="T1", x_um=100.0, y_um=200.0,
                               well_name="B2", **kw)

    def test_default_none_and_not_serialized(self):
        t = self._t()
        self.assertIsNone(t.pick_z_zref_mm)
        self.assertNotIn("pick_z_zref_mm", t.to_dict())   # byte-identity

    def test_round_trip_when_set(self):
        t = self._t(pick_z_zref_mm=20.51)
        d = t.to_dict()
        self.assertEqual(d["pick_z_zref_mm"], 20.51)
        back = PickPlaceTarget.from_dict(d)
        self.assertEqual(back.pick_z_zref_mm, 20.51)

    def test_legacy_dict_loads_with_none(self):
        d = self._t().to_dict()
        d.pop("pick_z_zref_mm", None)
        self.assertIsNone(PickPlaceTarget.from_dict(d).pick_z_zref_mm)


class TestExecutorConsultsOverride(unittest.TestCase):
    def test_execute_cell_removal_reads_the_override(self):
        # AST, not substring: the v7.10/v7.11 lesson is that a source-text
        # match passes on an import line or a comment alone.
        from SupportClasses.PickAndPlaceManager import PickPlaceExecutor
        src = textwrap.dedent(
            inspect.getsource(PickPlaceExecutor._execute_cell_removal))
        tree = ast.parse(src)
        found = False
        for node in ast.walk(tree):
            if isinstance(node, ast.Call):
                f = node.func
                if getattr(f, "id", "") == "getattr" and len(node.args) >= 2:
                    arg = node.args[1]
                    if (isinstance(arg, ast.Constant)
                            and arg.value == "pick_z_zref_mm"):
                        found = True
        self.assertTrue(found,
                        "_execute_cell_removal no longer consults the "
                        "per-target pick_z_zref_mm override")


class TestSurfaceZResolver(unittest.TestCase):
    """Unbound _surface_z_resolver against a fake page (repo pattern)."""

    def setUp(self):
        import SupportClasses.FluorescenceMosaicStore as fms
        import SupportClasses.PlateFocusDatumStore as pfd
        self._fms, self._pfd = fms, pfd
        self._orig_fms = fms._store_singleton
        self._orig_pfd = pfd._store
        tmp = Path(tempfile.mkdtemp())
        fms._store_singleton = fms.FluorescenceMosaicStore(tmp / "fluor.json")
        pfd._store = pfd.PlateFocusDatumStore(tmp / "datum.json")

    def tearDown(self):
        self._fms._store_singleton = self._orig_fms
        self._pfd._store = self._orig_pfd

    # ── fixtures ──────────────────────────────────────────────────

    def _ctrl(self):
        # Simple polarity-safe height frame around the fake plate bottom.
        return SimpleNamespace(
            zref_to_print_height=lambda z: float(z) - PLATE_BOTTOM_ZREF,
            print_height_to_zref=lambda h: PLATE_BOTTOM_ZREF + float(h),
            get_plate_bottom_z=lambda: PLATE_BOTTOM_ZREF)

    def _page(self, offset_um=10.0, scan_well="B2"):
        scan_page = SimpleNamespace(
            _scan_well=scan_well,
            _plate_key=lambda: "plateX",
            _camera_key=lambda: "cam1",
            _well_center_um=lambda w: (0.0, 0.0),
            _well_diameter_mm=lambda w: 12.0)
        return SimpleNamespace(
            _scan_page=scan_page,
            _controller=self._ctrl(),
            _surface_offset=SimpleNamespace(value=lambda: offset_um))

    def _survey(self, focus_um=1500.0, tilt=0.0):
        samples = [{"x_um": float(x), "y_um": float(y),
                    "focus_um": float(focus_um + tilt * x)}
                   for x in (-2000, 0, 2000) for y in (-2000, 0, 2000)]
        self._fms.get_store().set_focus_survey(
            "plateX", "B2", samples,
            summary={"well_center_um": (0.0, 0.0),
                     "well_radius_um": 6000.0})

    def _datum(self, offset_mm=19.0, sign=1):
        # needle_zref = sign × focus_um/1000 + offset → 1500 µm ⇒ 20.5 mm
        # = 0.5 mm above the 20.0 plate bottom.
        self._pfd.get_store().save("cam1", "", "plateX",
                                   focal_sign=sign, offset_mm=offset_mm)

    def _resolve(self, page):
        from gui.pages.workflows.cell_targeting_workflow import (
            CellTargetingWorkflowPage)
        return CellTargetingWorkflowPage._surface_z_resolver(page)

    # ── the gate chain ────────────────────────────────────────────

    def test_refuses_without_survey(self):
        resolver, why = self._resolve(self._page())
        self.assertIsNone(resolver)
        self.assertIn("no focus survey", why)

    def test_refuses_without_datum(self):
        self._survey()
        resolver, why = self._resolve(self._page())
        self.assertIsNone(resolver)
        self.assertIn("datum", why)

    def test_refuses_when_surface_below_plate_bottom(self):
        # Datum places the surface 0.5 mm BELOW the taught bottom: the datum
        # or the bottom is wrong — the whole mode must refuse, not aim there.
        self._survey(focus_um=1500.0)
        self._datum(offset_mm=18.0)      # 1.5 + 18.0 = 19.5 < 20.0
        resolver, why = self._resolve(self._page())
        self.assertIsNone(resolver)
        self.assertIn("BELOW the plate bottom", why)

    def test_refuses_without_scanned_well(self):
        page = self._page()
        page._scan_page = None
        resolver, why = self._resolve(page)
        self.assertIsNone(resolver)

    # ── the happy path ────────────────────────────────────────────

    def test_resolves_surface_plus_offset(self):
        self._survey(focus_um=1500.0)    # surface = 0.5 mm above bottom
        self._datum(offset_mm=19.0)
        resolver, why = self._resolve(self._page(offset_um=100.0))
        self.assertIsNotNone(resolver, why)
        z = resolver(0.0, 0.0)
        # 20.0 (bottom) + 0.5 (surface) + 0.1 (offset) = 20.6 zref mm
        self.assertAlmostEqual(z, 20.6, places=6)

    def test_tilted_surface_varies_per_target(self):
        self._survey(focus_um=1500.0, tilt=0.05)   # +0.05 µm per µm of x
        self._datum(offset_mm=19.0)
        resolver, _ = self._resolve(self._page(offset_um=0.0))
        z0 = resolver(0.0, 0.0)
        z1 = resolver(1000.0, 0.0)
        self.assertAlmostEqual(z1 - z0, 0.05, places=4)

    def test_low_confidence_target_falls_back_to_none(self):
        self._survey()
        self._datum()
        resolver, _ = self._resolve(self._page())
        # Outside the well radius (6000) → low confidence → per-target None.
        self.assertIsNone(resolver(50000.0, 0.0))

    def test_focal_sign_matters(self):
        # Mutation guard: a flipped focal sign puts the surface far below the
        # bottom → whole-mode refusal, never a silently-mirrored height.
        self._survey(focus_um=1500.0)
        self._datum(offset_mm=22.0, sign=-1)   # −1.5 + 22.0 = 20.5 OK
        resolver, why = self._resolve(self._page(offset_um=0.0))
        self.assertIsNotNone(resolver, why)
        self.assertAlmostEqual(resolver(0.0, 0.0), 20.5, places=6)
        self._datum(offset_mm=22.0, sign=1)    # +1.5 + 22.0 = 23.5 → 3.5 up?
        resolver, why = self._resolve(self._page(offset_um=0.0))
        # Still above bottom (3.5 mm up) → allowed; the direction test is the
        # sign=-1 case above landing exactly where physics says.
        self.assertIsNotNone(resolver)


if __name__ == "__main__":
    unittest.main()

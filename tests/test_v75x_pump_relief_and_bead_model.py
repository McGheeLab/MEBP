"""
v7.5.x — F-1 (one bead model: inner-bore area × modifier) + F-4 print-path relief.

F-1: the deposited volume-per-mm of print path is the inner-bore cross-section
(× an extrusion modifier), NOT the legacy outer-Ø × layer_height. Covers
FlowPhysics.extrusion_flow_rate, GeometryEngine.compute_pump_positions /
compute_total_volume, and the PrintObject.extrusion_modifier data field.

F-4 (print path): PrintManager._print_pump_suckback performs the deposit /
quick-move suck-back, gated on the per-context relief toggles. (The
HardwareConfig fields, StageController readers, direction-aware move_pump_uL,
and serialization/migration are covered in test_v75x_pump_settle_and_prime_time.)
"""

import math
import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import unittest

import numpy as np

from SupportClasses.PhysicalModels import NeedleSpec, SyringeSpec
from SupportClasses import FlowPhysics
from SupportClasses import GeometryEngine
from SupportClasses.GeometryEngine import (
    PrintObject, compute_pump_positions, compute_total_volume,
)


def _needle_22g():
    # 22 G, 2" — id 413 µm → bore area π·(0.2065)² ≈ 0.13396 mm²
    return NeedleSpec(gauge=22, od_um=718, id_um=413, wall_um=152,
                      length_inches=2.0)


def _syringe_250():
    return SyringeSpec(volume_uL=250, stroke_length_mm=30.0, barrel_id_mm=3.256)


# ════════════════════════════════════════════════════════════════════
#  F-1 — bead model = inner-bore area × modifier
# ════════════════════════════════════════════════════════════════════

class TestBoreAreaBeadModel(unittest.TestCase):

    def test_cross_section_is_inner_bore(self):
        n = _needle_22g()
        self.assertAlmostEqual(n.cross_section_area_mm2,
                               math.pi * (0.413 / 2) ** 2, places=5)

    def test_extrusion_flow_rate_uses_bore_area_not_od_layer(self):
        n = _needle_22g()
        # bore-area model: speed × area × modifier (layer_height is ignored).
        got = FlowPhysics.extrusion_flow_rate(5.0, n, 0.2)
        self.assertAlmostEqual(got, 5.0 * n.cross_section_area_mm2, places=6)
        # Explicitly NOT the legacy od × layer_height model.
        self.assertNotAlmostEqual(got, 5.0 * n.od_mm * 0.2, places=6)

    def test_extrusion_flow_rate_modifier_scales(self):
        n = _needle_22g()
        base = FlowPhysics.extrusion_flow_rate(5.0, n, extrusion_modifier=1.0)
        dbl = FlowPhysics.extrusion_flow_rate(5.0, n, extrusion_modifier=2.0)
        self.assertAlmostEqual(dbl, 2.0 * base, places=6)

    def test_compute_total_volume_bore_area(self):
        n = _needle_22g()
        vol = compute_total_volume(10.0, n)           # 10 mm path, modifier 1
        self.assertAlmostEqual(vol, 10.0 * n.cross_section_area_mm2, places=6)
        vol2 = compute_total_volume(10.0, n, extrusion_modifier=2.0)
        self.assertAlmostEqual(vol2, 2.0 * vol, places=6)

    def test_compute_pump_positions_bore_area(self):
        n = _needle_22g()
        s = _syringe_250()
        dists = np.array([0.0, 10.0])
        pos = compute_pump_positions(dists, n, s)     # modifier 1
        # volume/mm = bore area; pump mm = volume × mm_per_uL (30/250 = 0.12)
        expected = 10.0 * n.cross_section_area_mm2 * s.mm_per_uL
        self.assertAlmostEqual(pos[-1], expected, places=6)

    def test_layer_height_no_longer_changes_volume(self):
        n = _needle_22g()
        a = compute_total_volume(10.0, n, 0.1)
        b = compute_total_volume(10.0, n, 5.0)        # different layer_height
        self.assertAlmostEqual(a, b, places=9)        # identical — lh ignored

    def test_print_object_modifier_round_trips(self):
        obj = PrintObject(name="x", object_type="circle",
                          extrusion_modifier=1.75)
        out = PrintObject.from_dict(obj.to_dict())
        self.assertAlmostEqual(out.extrusion_modifier, 1.75)

    def test_print_object_modifier_defaults_to_one(self):
        out = PrintObject.from_dict({"name": "y", "object_type": "circle"})
        self.assertAlmostEqual(out.extrusion_modifier, 1.0)


# ════════════════════════════════════════════════════════════════════
#  F-4 — print-path suck-back (deposit / quick-move)
# ════════════════════════════════════════════════════════════════════

class _FakeReliefCtrl:
    """Records move_pump_uL calls; exposes the v7.5.x compliance readers.

    The print-path suck-back (deposit / quick-move) is the "unload" half of
    backlash compensation — gated on the single ``backlash_comp_enabled`` toggle,
    volume = per-pump ``pump_relief_uL``."""

    def __init__(self, relief_uL=0.5, enabled=True):
        self._relief = relief_uL
        self._enabled = enabled
        self.moves = []

    def pump_relief_uL(self, pump):
        return self._relief

    def backlash_comp_enabled(self):
        return self._enabled

    def move_pump_uL(self, pump, vol, rate_uL_s=None, **kw):
        self.moves.append((pump, vol))


class TestPrintPathSuckback(unittest.TestCase):

    def _pm(self, ctrl):
        from SupportClasses.PrintManager import PrintManager
        pm = PrintManager.__new__(PrintManager)
        pm.controller = ctrl
        pm._active_pump = "P2"
        pm.exec_logger = None
        return pm

    def test_deposit_suckback_aspirates_relief(self):
        ctrl = _FakeReliefCtrl(relief_uL=0.5, enabled=True)
        pm = self._pm(ctrl)
        pm._print_pump_suckback("deposit", "P2")
        self.assertEqual(len(ctrl.moves), 1)
        self.assertEqual(ctrl.moves[0][0], "P2")
        self.assertAlmostEqual(ctrl.moves[0][1], -0.5)   # ASPIRATE (suck-back)

    def test_deposit_suckback_skipped_when_disabled(self):
        ctrl = _FakeReliefCtrl(relief_uL=0.5, enabled=False)
        pm = self._pm(ctrl)
        pm._print_pump_suckback("deposit", "P2")
        self.assertEqual(len(ctrl.moves), 0)

    def test_quick_move_suckback(self):
        ctrl = _FakeReliefCtrl(relief_uL=0.3, enabled=True)
        pm = self._pm(ctrl)
        pm._print_pump_suckback("quick_move")          # uses _active_pump
        self.assertEqual(len(ctrl.moves), 1)
        self.assertAlmostEqual(ctrl.moves[0][1], -0.3)

    def test_quick_move_skipped_when_disabled(self):
        ctrl = _FakeReliefCtrl(relief_uL=0.3, enabled=False)
        pm = self._pm(ctrl)
        pm._print_pump_suckback("quick_move")
        self.assertEqual(len(ctrl.moves), 0)

    def test_no_suckback_when_relief_zero(self):
        ctrl = _FakeReliefCtrl(relief_uL=0.0, enabled=True)
        pm = self._pm(ctrl)
        pm._print_pump_suckback("deposit", "P2")
        self.assertEqual(len(ctrl.moves), 0)

    def test_older_controller_without_readers_is_noop(self):
        class _Bare:
            def __init__(self): self.moves = []
            def move_pump_uL(self, *a, **k): self.moves.append(a)
        ctrl = _Bare()
        pm = self._pm(ctrl)
        pm._print_pump_suckback("deposit", "P2")       # no relief readers → skip
        self.assertEqual(len(ctrl.moves), 0)


if __name__ == "__main__":
    unittest.main()

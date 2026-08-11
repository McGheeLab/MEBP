"""
Turret position ↔ objective name ↔ µm/px.

The load-bearing test here is the one asserting a MISSING calibration REFUSES
rather than falling back. A fallback to the live camera manager's µm/px, or to
``current_objective_name``, is exactly how a stale value came to shadow a
calibration the operator had just measured. One source, refuse otherwise.
"""

import os
import sys
import unittest
from types import SimpleNamespace

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from SupportClasses.ObjectiveLadder import (        # noqa: E402
    LadderRung, ladder_gate, parfocal_offsets_um, resolve_ladder,
    um_per_px_at_resolution,
)


class _Cfg:
    def __init__(self, labels):
        self._labels = dict(labels)

    def objective_labels(self):
        return dict(self._labels)


class _ObjStore:
    def __init__(self, cals):
        self._cals = cals

    def get_calibration(self, camera, objective):
        return (self._cals.get(camera) or {}).get(objective)


def optic(pos, label, code="MRH", mag=None, na=None, wd=None, present=True):
    return SimpleNamespace(position=pos, present=present, label=label,
                           code=code, magnification=mag,
                           numerical_aperture=na, working_distance_mm=wd)


def state(count=3, optics=None, objective_position=1):
    return SimpleNamespace(objective_count=count,
                           mounted_objectives=tuple(optics or ()),
                           objective_position=objective_position)


CALS = {"BUC3D": {"4x": {"measured_um_per_px": 3.227, "resolution": [1024, 1024]},
                  "10x": {"measured_um_per_px": 1.320, "resolution": [2048, 2048],
                          "rotation_deg": 12.5}}}


class TestRescale(unittest.TestCase):
    def test_scales_inversely_with_frame_width(self):
        cal = {"measured_um_per_px": 3.227, "resolution": [1024, 1024]}
        self.assertAlmostEqual(um_per_px_at_resolution(cal, 2048),
                               3.227 / 2.0, places=9)

    def test_same_resolution_is_unchanged(self):
        cal = {"measured_um_per_px": 3.227, "resolution": [1024, 1024]}
        self.assertAlmostEqual(um_per_px_at_resolution(cal, 1024), 3.227)

    def test_unstamped_calibration_is_used_as_is_not_invented(self):
        """It CANNOT be rescaled; inventing a scale would be worse than using it
        unchanged, and the caller is told nothing was applied."""
        cal = {"measured_um_per_px": 3.227}
        self.assertAlmostEqual(um_per_px_at_resolution(cal, 2048), 3.227)

    def test_missing_or_zero_is_none(self):
        self.assertIsNone(um_per_px_at_resolution(None, 2048))
        self.assertIsNone(um_per_px_at_resolution({"measured_um_per_px": 0.0},
                                                  2048))


class TestResolveLadder(unittest.TestCase):
    def test_named_and_calibrated_slot_resolves(self):
        rungs = resolve_ladder(
            scope_state=state(optics=[optic(1, "4x", mag=4.0, na=0.13, wd=16.4)]),
            config_store=_Cfg({1: "4x"}), objective_store=_ObjStore(CALS),
            camera_name="BUC3D", live_resolution=(1024, 1024), positions=[1])
        self.assertTrue(rungs[0].calibrated)
        self.assertAlmostEqual(rungs[0].um_per_px, 3.227)
        self.assertAlmostEqual(rungs[0].numerical_aperture, 0.13)

    def test_unnamed_slot_is_not_selectable_and_says_where_to_fix_it(self):
        rungs = resolve_ladder(scope_state=state(), config_store=_Cfg({}),
                               objective_store=_ObjStore(CALS),
                               camera_name="BUC3D", positions=[2])
        self.assertFalse(rungs[0].selectable)
        self.assertIn("Read from", rungs[0].why_not)

    def test_uncalibrated_slot_REFUSES_and_never_falls_back(self):
        """The 'I literally just calibrated it' regression: no fallback to the
        live manager, no fallback to current_objective_name."""
        rungs = resolve_ladder(
            scope_state=state(optics=[optic(3, "20x", mag=20.0, na=0.45, wd=1.0)]),
            config_store=_Cfg({3: "20x"}), objective_store=_ObjStore(CALS),
            camera_name="BUC3D", positions=[3])
        self.assertFalse(rungs[0].calibrated)
        self.assertIsNone(rungs[0].um_per_px)
        self.assertIn("no µm/px calibration", rungs[0].why_not)
        self.assertIn("20x", rungs[0].why_not)
        self.assertIn("BUC3D", rungs[0].why_not)

    def test_a_different_camera_does_not_inherit_a_calibration(self):
        rungs = resolve_ladder(
            scope_state=state(optics=[optic(1, "4x", mag=4.0, na=0.13, wd=16.4)]),
            config_store=_Cfg({1: "4x"}), objective_store=_ObjStore(CALS),
            camera_name="OTHERCAM", positions=[1])
        self.assertFalse(rungs[0].calibrated)

    def test_rotation_is_carried_through(self):
        rungs = resolve_ladder(
            scope_state=state(optics=[optic(2, "10x", mag=10.0, na=0.3, wd=16.0)]),
            config_store=_Cfg({2: "10x"}), objective_store=_ObjStore(CALS),
            camera_name="BUC3D", live_resolution=(2048, 2048), positions=[2])
        self.assertAlmostEqual(rungs[0].rotation_deg, 12.5)


class TestLadderGate(unittest.TestCase):
    def _rung(self, pos, name, mag, cal=True):
        return LadderRung(turret_position=pos, objective_name=name,
                          magnification=mag, um_per_px=1.0 if cal else None,
                          calibrated=cal,
                          why_not="" if cal else "no µm/px calibration")

    def test_empty_ladder_refuses(self):
        ok, why = ladder_gate([])
        self.assertFalse(ok)
        self.assertIn("at least one", why)

    def test_uncalibrated_rung_blocks_with_its_own_reason(self):
        ok, why = ladder_gate([self._rung(1, "4x", 4.0),
                               self._rung(3, "20x", 20.0, cal=False)])
        self.assertFalse(ok)
        self.assertIn("no µm/px calibration", why)

    def test_descending_magnification_is_refused(self):
        ok, why = ladder_gate([self._rung(3, "20x", 20.0),
                               self._rung(1, "4x", 4.0)])
        self.assertFalse(ok)
        self.assertIn("magnification increasing", why)

    def test_the_two_records_of_which_objective_are_compared(self):
        """The first time in this codebase that current_objective_name and the
        live turret position are ever checked against each other. Every µm/px in
        a run comes from whichever one wins."""
        ok, why = ladder_gate([self._rung(1, "4x", 4.0),
                               self._rung(2, "10x", 10.0)],
                              current_objective_name="20x",
                              live_turret_position=1)
        self.assertFalse(ok)
        self.assertIn("20x", why)
        self.assertIn("4x", why)

    def test_agreement_passes(self):
        ok, why = ladder_gate([self._rung(1, "4x", 4.0),
                               self._rung(2, "10x", 10.0)],
                              current_objective_name="4x",
                              live_turret_position=1)
        self.assertTrue(ok, why)


class TestParfocalOffsets(unittest.TestCase):
    def test_reference_is_exactly_zero(self):
        offs, spread, ref = parfocal_offsets_um(
            {"4x": [1000.0, 1010.0], "10x": [871.6, 881.6]}, reference="4x")
        self.assertEqual(ref, "4x")
        self.assertAlmostEqual(offs["4x"], 0.0)

    def test_offset_is_the_median_difference_across_sites(self):
        offs, _s, _r = parfocal_offsets_um(
            {"4x": [1000.0, 1010.0, 1020.0],
             "10x": [871.6, 881.6, 891.6]}, reference="4x")
        self.assertAlmostEqual(offs["10x"], -128.4, places=6)

    def test_spread_is_reported_as_a_quality_check(self):
        """A large spread means drift, or that the sites were not measuring the
        same thing — it is free evidence, so it is surfaced."""
        _o, spread, _r = parfocal_offsets_um(
            {"4x": [1000.0, 1000.0], "10x": [900.0, 700.0]}, reference="4x")
        self.assertAlmostEqual(spread["10x"], 200.0)

    def test_empty_input_is_safe(self):
        self.assertEqual(parfocal_offsets_um({}), ({}, {}, ""))


if __name__ == "__main__":
    unittest.main()

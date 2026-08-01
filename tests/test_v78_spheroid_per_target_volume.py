"""test_v78_spheroid_per_target_volume.py — per-spheroid aspirate volume + the
"needle ID >= 1.5x spheroid" advisory.

Uses the partial-page pattern (``Cls.__new__(Cls)`` + duck-typed stubs) from
``tests/test_v75x_spheroid_picker_scaling.py``: the real unbound methods are
called against hand-built widget stand-ins, so no Qt, no camera and no stage are
needed. The load-bearing assertion is the NEGATIVE one — with per-spheroid sizing
off, or every ``size_um`` still 0, the built queue must be byte-identical to the
pre-v7.8 single-config queue.
"""

from __future__ import annotations

import math
import unittest
from types import SimpleNamespace

from SupportClasses.PhysicalModels import NeedleSpec
from SupportClasses.PickAndPlaceManager import (
    PickPlaceTarget, SpheroidPickupConfig)
from gui.pages.workflows.spheroid_pickup_workflow import (
    SpheroidPickupWorkflowPage as Page)


class _Spin:
    def __init__(self, value):
        self._v = value

    def value(self):
        return self._v

    def setValue(self, v):
        self._v = v


class _Check:
    def __init__(self, on):
        self._on = bool(on)

    def isChecked(self):
        return self._on

    def setChecked(self, on):
        self._on = bool(on)


class _Label:
    def __init__(self):
        self.text = ""

    def setText(self, t):
        self.text = t

    def setStyleSheet(self, _s):
        pass


def _page(*, diameter=200.0, safety=1.5, per_target=True, clearance=1.5,
          needle=None, picks=(), release_enabled=False, release_vol=0.0):
    """A partial page carrying only what the volume/advisory path reads."""
    p = Page.__new__(Page)
    p._diameter = _Spin(diameter)
    p._safety = _Spin(safety)
    p._per_target_volume = _Check(per_target)
    p._clearance = _Spin(clearance)
    p._bore = SimpleNamespace(currentText=lambda: "P1", count=lambda: 1)
    p._pick_flow = _Spin(1.0)
    p._place_flow = _Spin(1.0)
    p._pick_z = _Spin(0.10)
    p._place_z = _Spin(0.50)
    p._pick_dwell = _Spin(0.0)
    p._place_dwell = _Spin(0.0)
    p._disengage_enabled = _Check(False)
    p._disengage_vol = _Spin(0.0)
    p._disengage_rate = _Spin(2.0)
    p._sink_timing_enabled = _Check(False)
    p._travel_margin = _Spin(1.0)
    p._release_enabled = _Check(release_enabled)
    p._release_vol = _Spin(release_vol)
    p._status = _Label()
    p._fit_summary = _Label()
    p._volume_label = _Label()
    p._hw_config = SimpleNamespace(needle=needle) if needle else None
    p._picker = SimpleNamespace(picks=lambda: list(picks))
    return p


def _needle(id_um=300.0):
    return NeedleSpec(gauge=22, od_um=id_um * 1.7, id_um=id_um, wall_um=50.0)


def _target(tid, size_um=0.0):
    return PickPlaceTarget(target_id=tid, x_um=0.0, y_um=0.0, well_name="",
                           size_um=size_um)


def _sphere_uL(d_um, safety=1.5):
    r = d_um / 2.0
    return ((4.0 / 3.0) * math.pi * r ** 3) / 1e9 * safety


class TestEffectiveDiameter(unittest.TestCase):
    def test_measured_diameter_used_when_enabled(self):
        p = _page()
        self.assertEqual(p._effective_diameter_um(_target("P1", 312.0)), 312.0)

    def test_unmeasured_target_reports_zero(self):
        p = _page()
        self.assertEqual(p._effective_diameter_um(_target("P1", 0.0)), 0.0)

    def test_disabled_checkbox_ignores_a_measured_diameter(self):
        p = _page(per_target=False)
        self.assertEqual(p._effective_diameter_um(_target("P1", 312.0)), 0.0)

    def test_absurd_value_is_clamped(self):
        """size_um serialized for years without ever being written, so a
        restored/edited value must not drive a wild aspirate."""
        p = _page()
        self.assertEqual(p._effective_diameter_um(_target("P1", 1e9)),
                         Page._MAX_TARGET_DIAMETER_UM)

    def test_garbage_value_is_ignored(self):
        p = _page()
        self.assertEqual(
            p._effective_diameter_um(SimpleNamespace(size_um="big")), 0.0)
        self.assertEqual(p._effective_diameter_um(SimpleNamespace()), 0.0)

    def test_negative_value_is_ignored(self):
        p = _page()
        self.assertEqual(p._effective_diameter_um(_target("P1", -5.0)), 0.0)


class TestPerPairConfig(unittest.TestCase):
    def test_measured_pick_gets_its_own_diameter(self):
        p = _page(diameter=200.0)
        base = p._current_config()
        cfg = p._config_for_pick(base, _target("P1", 320.0))
        self.assertIsNot(cfg, base)
        self.assertAlmostEqual(cfg.spheroid_diameter_um, 320.0)
        # Everything else is carried over verbatim.
        self.assertAlmostEqual(cfg.safety_factor, base.safety_factor)
        self.assertEqual(cfg.pickup_bore, base.pickup_bore)
        self.assertAlmostEqual(cfg.pick_z_offset_mm, base.pick_z_offset_mm)

    def test_unmeasured_pick_reuses_the_SAME_config_object(self):
        """The identity that keeps the queue byte-identical to pre-v7.8."""
        p = _page()
        base = p._current_config()
        self.assertIs(p._config_for_pick(base, _target("P1", 0.0)), base)

    def test_matching_diameter_reuses_the_same_object(self):
        p = _page(diameter=200.0)
        base = p._current_config()
        self.assertIs(p._config_for_pick(base, _target("P1", 200.0)), base)

    def test_disabled_reuses_the_same_object_for_every_pick(self):
        p = _page(per_target=False)
        base = p._current_config()
        for d in (0.0, 150.0, 320.0):
            self.assertIs(p._config_for_pick(base, _target("P1", d)), base)

    def test_volume_scales_as_diameter_cubed(self):
        p = _page(diameter=200.0, safety=1.5)
        base = p._current_config()
        v200 = p._config_for_pick(base, _target("A", 0.0)).compute_volume_uL()
        v150 = p._config_for_pick(base, _target("B", 150.0)).compute_volume_uL()
        v320 = p._config_for_pick(base, _target("C", 320.0)).compute_volume_uL()
        self.assertAlmostEqual(v200, _sphere_uL(200.0), places=9)
        self.assertAlmostEqual(v150, _sphere_uL(150.0), places=9)
        self.assertAlmostEqual(v320, _sphere_uL(320.0), places=9)
        # Cube law: (320/150)^3 ≈ 9.7
        self.assertAlmostEqual(v320 / v150, (320.0 / 150.0) ** 3, places=6)

    def test_safety_factor_is_always_applied(self):
        """The operator's explicit requirement."""
        for safety in (0.01, 0.5, 1.0, 2.5, 10.0):
            p = _page(diameter=200.0, safety=safety)
            base = p._current_config()
            cfg = p._config_for_pick(base, _target("A", 250.0))
            self.assertAlmostEqual(cfg.safety_factor, safety)
            self.assertAlmostEqual(cfg.compute_volume_uL(),
                                   _sphere_uL(250.0, safety), places=9)

    def test_safety_factor_range_extremes_accepted(self):
        for safety in (0.01, 10.0):
            p = _page(safety=safety)
            self.assertAlmostEqual(p._current_config().safety_factor, safety)


class TestNeedleClearanceRule(unittest.TestCase):
    """``spheroid_pickup_detail``'s ratio is orifice_id/spheroid_d, so a
    clearance of 1.5 IS "needle ID at least 1.5x the spheroid"."""

    def test_tight_at_1_5x_when_ratio_is_1_25(self):
        p = _page(diameter=200.0, needle=_needle(250.0), clearance=1.5)
        detail = p._spheroid_fit_detail(p._current_config())
        self.assertEqual(detail["status"], "tight")
        self.assertEqual(detail["severity"], "warning")
        self.assertAlmostEqual(detail["ratio"], 1.25, places=6)

    def test_ok_when_the_orifice_clears_1_5x(self):
        p = _page(diameter=200.0, needle=_needle(350.0), clearance=1.5)
        detail = p._spheroid_fit_detail(p._current_config())
        self.assertEqual(detail["status"], "ok")
        self.assertEqual(detail["severity"], "info")

    def test_default_1_2_clearance_would_have_passed_the_same_spheroid(self):
        """Shows the setting is what changed the verdict, not the geometry."""
        p = _page(diameter=200.0, needle=_needle(250.0), clearance=1.2)
        self.assertEqual(p._spheroid_fit_detail(p._current_config())["status"],
                         "ok")

    def test_too_large_when_the_spheroid_exceeds_the_bore(self):
        p = _page(diameter=400.0, needle=_needle(300.0), clearance=1.5)
        detail = p._spheroid_fit_detail(p._current_config())
        self.assertEqual(detail["status"], "too_large")
        self.assertEqual(detail["severity"], "warning")

    def test_no_severity_is_ever_error(self):
        """Re-pins the advisory-only contract with the new clearance in play."""
        for d, bore in ((50.0, 300.0), (200.0, 250.0), (400.0, 300.0),
                        (1000.0, 300.0)):
            p = _page(diameter=d, needle=_needle(bore), clearance=1.5)
            detail = p._spheroid_fit_detail(p._current_config())
            self.assertNotEqual(detail["severity"], "error")

    def test_missing_needle_returns_none(self):
        p = _page(diameter=200.0, needle=None)
        self.assertIsNone(p._spheroid_fit_detail(p._current_config()))

    def test_stub_needle_without_the_kwarg_still_works(self):
        """A duck-typed needle predating the clearance kwarg must not crash."""
        calls = []

        def old_detail(d_um, volume_uL=None):
            calls.append((d_um, volume_uL))
            return {"status": "ok", "severity": "info", "ratio": 2.0,
                    "message": "fine"}

        p = _page(diameter=200.0,
                  needle=SimpleNamespace(spheroid_pickup_detail=old_detail))
        detail = p._spheroid_fit_detail(p._current_config())
        self.assertEqual(detail["status"], "ok")
        self.assertEqual(len(calls), 1)

    def test_clearance_defaults_to_1_5_without_the_widget(self):
        p = _page()
        del p._clearance
        self.assertAlmostEqual(p._needle_clearance(), 1.5)

    def test_zero_clearance_falls_back(self):
        p = _page(clearance=0.0)
        self.assertAlmostEqual(p._needle_clearance(), 1.5)


class TestPerTargetBadges(unittest.TestCase):
    def test_badge_names_the_problem_and_the_ratio(self):
        p = _page(needle=_needle(250.0), clearance=1.5)
        badge, msg = p._fit_badge(200.0)
        self.assertIn("tight", badge)
        self.assertIn("1.25", badge)
        self.assertTrue(msg)

    def test_no_badge_for_a_comfortable_spheroid(self):
        p = _page(needle=_needle(350.0), clearance=1.5)
        self.assertEqual(p._fit_badge(200.0), ("", ""))

    def test_no_badge_for_an_unmeasured_target(self):
        p = _page(needle=_needle(250.0))
        self.assertEqual(p._fit_badge(0.0), ("", ""))

    def test_too_large_badge(self):
        p = _page(needle=_needle(300.0), clearance=1.5)
        badge, _ = p._fit_badge(400.0)
        self.assertIn("too large", badge)


class TestRunLevelWarnings(unittest.TestCase):
    def test_warns_per_config_and_counts_them(self):
        p = _page(needle=_needle(250.0), clearance=1.5)
        base = p._current_config()
        cfgs = [p._config_for_pick(base, _target("A", 200.0)),
                p._config_for_pick(base, _target("B", 400.0)),
                p._config_for_pick(base, _target("C", 60.0))]
        p._warn_spheroid_fits(cfgs)
        self.assertIn("⚠", p._status.text)
        self.assertIn("of 3 picks", p._status.text)

    def test_silent_when_every_spheroid_fits(self):
        p = _page(needle=_needle(2000.0), clearance=1.5)
        base = p._current_config()
        p._warn_spheroid_fits([base])
        self.assertEqual(p._status.text, "")

    def test_single_config_shim_still_works(self):
        """The pre-v7.8 entry point kept so the existing surface survives."""
        p = _page(diameter=200.0, needle=_needle(250.0), clearance=1.5)
        p._warn_spheroid_fit(p._current_config())
        self.assertIn("⚠", p._status.text)
        # One pick → no "N of M" suffix.
        self.assertNotIn("of 1 picks", p._status.text)

    def test_never_raises_without_a_status_label(self):
        p = _page(needle=_needle(250.0))
        del p._status
        p._warn_spheroid_fits([p._current_config()])   # must not raise


class TestReleaseResidualWarning(unittest.TestCase):
    def test_warns_when_retention_exceeds_the_needle(self):
        needle = _needle(300.0)
        p = _page(needle=needle, release_enabled=True, release_vol=0.0001)
        p._hw_config = SimpleNamespace(needle=needle)
        base = p._current_config()
        cfgs = [p._config_for_pick(base, _target(f"P{i}", 900.0))
                for i in range(20)]
        p._warn_release_residual(cfgs)
        self.assertIn("retained", p._status.text)

    def test_silent_when_release_is_disabled(self):
        p = _page(needle=_needle(300.0), release_enabled=False)
        base = p._current_config()
        p._warn_release_residual([base])
        self.assertEqual(p._status.text, "")

    def test_silent_when_release_volume_is_zero(self):
        p = _page(needle=_needle(300.0), release_enabled=True, release_vol=0.0)
        base = p._current_config()
        p._warn_release_residual([base])
        self.assertEqual(p._status.text, "")


class TestFitSummary(unittest.TestCase):
    def test_states_the_limit_in_microns(self):
        p = _page(needle=_needle(300.0), clearance=1.5)
        p._refresh_fit_summary()
        self.assertIn("300 µm", p._fit_summary.text)
        self.assertIn("200 µm", p._fit_summary.text)   # 300 / 1.5

    def test_names_the_offending_picks(self):
        picks = [_target("P001", 100.0), _target("P002", 400.0),
                 _target("P003", 350.0)]
        p = _page(needle=_needle(300.0), clearance=1.5, picks=picks)
        p._refresh_fit_summary()
        self.assertIn("2 of 3", p._fit_summary.text)
        self.assertIn("P002", p._fit_summary.text)
        self.assertIn("P003", p._fit_summary.text)
        self.assertNotIn("P001", p._fit_summary.text)

    def test_unknown_bore_says_so(self):
        p = _page(needle=None)
        p._refresh_fit_summary()
        self.assertIn("unknown", p._fit_summary.text)


class TestVolumeLabel(unittest.TestCase):
    def test_flags_a_sub_unity_safety_factor(self):
        p = _page(safety=0.5, needle=_needle(300.0))
        p._refresh_volume_label()
        self.assertIn("⚠", p._volume_label.text)

    def test_notes_the_fallback_role_when_per_target_is_on(self):
        p = _page(per_target=True, needle=_needle(300.0))
        p._refresh_volume_label()
        self.assertIn("unmeasured", p._volume_label.text)

    def test_plain_when_per_target_is_off(self):
        p = _page(per_target=False, safety=1.5, needle=_needle(300.0))
        p._refresh_volume_label()
        self.assertNotIn("unmeasured", p._volume_label.text)
        self.assertNotIn("⚠", p._volume_label.text)


class TestQueueByteIdentity(unittest.TestCase):
    """The regression guard: nothing about the built queue may change unless the
    operator opted in AND a diameter was actually measured."""

    def _queue_cfgs(self, page, picks):
        base = page._current_config()
        return [page._config_for_pick(base, pk) for pk in picks]

    def test_all_unmeasured_gives_one_shared_config(self):
        picks = [_target("P1"), _target("P2"), _target("P3")]
        p = _page()
        cfgs = self._queue_cfgs(p, picks)
        self.assertEqual(len({id(c) for c in cfgs}), 1)

    def test_opt_out_gives_one_shared_config_even_when_measured(self):
        picks = [_target("P1", 150.0), _target("P2", 320.0)]
        p = _page(per_target=False)
        cfgs = self._queue_cfgs(p, picks)
        self.assertEqual(len({id(c) for c in cfgs}), 1)
        for c in cfgs:
            self.assertAlmostEqual(c.spheroid_diameter_um, 200.0)

    def test_opt_in_with_measurements_gives_distinct_configs(self):
        picks = [_target("P1"), _target("P2", 150.0), _target("P3", 320.0)]
        p = _page()
        cfgs = self._queue_cfgs(p, picks)
        self.assertAlmostEqual(cfgs[0].spheroid_diameter_um, 200.0)
        self.assertAlmostEqual(cfgs[1].spheroid_diameter_um, 150.0)
        self.assertAlmostEqual(cfgs[2].spheroid_diameter_um, 320.0)
        # The unmeasured pick shares the base object; the measured ones don't.
        self.assertIsNot(cfgs[1], cfgs[0])
        self.assertIsNot(cfgs[2], cfgs[0])
        self.assertIsNot(cfgs[1], cfgs[2])

    def test_config_is_a_real_SpheroidPickupConfig(self):
        p = _page()
        cfg = p._config_for_pick(p._current_config(), _target("P1", 250.0))
        self.assertIsInstance(cfg, SpheroidPickupConfig)


if __name__ == "__main__":
    unittest.main()

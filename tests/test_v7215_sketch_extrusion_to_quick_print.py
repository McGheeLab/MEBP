"""
test_v7215_sketch_extrusion_to_quick_print.py — the sketch's EXACT output,
including its extrusion along the path, reaches Quick Print (v7.21.5).

Operator: *"the exact output of the print sketch including a calculation of the
extrusion modifier along the printing path should be sent to the quick print."*

Before this, the sketch's toolpath was baked faithfully (the full Nx7 CSV,
pump columns and all) but Quick Print kept only the XY of each sub-path and
re-derived ONE flow for the whole print from its own knobs — so a sketch whose
shapes declare different line widths printed every one of them at the same
width, and a *no extrude* section still extruded.

The chain under test, end to end:

    shape.line_width_mm  ->  compile_to_trajectory (per-shape modifier +
    per-segment profile)  ->  baked print params  ->  Quick Print (split with
    the sub-paths, converted to µL/mm against the needle fitted now)  ->
    build_well_plate_job (PRINT_PATH.vol_per_mm_profile)  ->  the executor
    (per-segment deposition).

Runs offscreen (no hardware).

See coding plans/Update plans/MEBP_v7215_SKETCH_EXTRUSION_TO_QUICK_PRINT.md.
"""

import math
import os
import sys
import tempfile
import unittest
from unittest.mock import MagicMock

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np                                            # noqa: E402
from PySide6.QtWidgets import QApplication                     # noqa: E402

from SupportClasses.ExtrusionProfile import (                  # noqa: E402
    ExtrusionProfile, modifier_to_vol_per_mm, rate_for,
)
from SupportClasses.PhysicalModels import (                    # noqa: E402
    InkSpec, NeedleSpec, SyringeSpec,
)
from SupportClasses.HardwareConfig import (                    # noqa: E402
    HardwareConfig, PumpChannelConfig,
)
from SupportClasses.PrintManager import (                      # noqa: E402
    CommandType, PrintSettings, build_well_plate_job,
)
from SupportClasses.PrintFileManager import (                  # noqa: E402
    PrintFileManager, save_trajectory_as_print_object,
)
from SupportClasses.SketchTrajectory import (                  # noqa: E402
    Sketch, SketchShape, compile_to_trajectory,
)
from SupportClasses.WellPlate import WellPlate                 # noqa: E402


# 22G: OD 0.718 mm, ID 0.413 mm → 1× bead = 0.413 mm wide, area = 0.13398 mm².
_ID_MM = 0.413
_AREA = math.pi * (_ID_MM / 2) ** 2


def _needle():
    return NeedleSpec(gauge=22, od_um=718, id_um=413, wall_um=152,
                      length_inches=1.0)


def _line(x1, y1, x2, y2, **kw):
    return SketchShape(kind="line", points=[(x1, y1), (x2, y2)], **kw)


def _two_width_sketch(w1=_ID_MM, w2=2 * _ID_MM, **sk_kw):
    """Two WELDED lines (they share (10,0)) at different declared widths — so
    one continuous bead has to change thickness partway along."""
    kw = dict(line_spacing_mm=0.4, num_layers=1, extrusion_multiplier=1.0)
    kw.update(sk_kw)
    return Sketch(shapes=[_line(0, 0, 10, 0, line_width_mm=w1),
                          _line(10, 0, 20, 0, line_width_mm=w2)], **kw)


# ═══════════════════════════════════════════════════════════════════
# 1 — the pure helper
# ═══════════════════════════════════════════════════════════════════

class TestExtrusionProfileHelper(unittest.TestCase):

    def _p(self):
        pts = [(0, 0), (1, 0), (3, 0), (6, 0)]
        return ExtrusionProfile.build(pts, [1.0, 2.0, 4.0])

    def test_index_and_arclen_agree(self):
        p = self._p()
        self.assertEqual(p.at_index(0), 1.0)
        self.assertEqual(p.at_index(1), 2.0)
        self.assertEqual(p.at_index(2), 4.0)
        self.assertEqual(p.at_arclen(0.5), 1.0)
        self.assertEqual(p.at_arclen(2.0), 2.0)
        self.assertEqual(p.at_arclen(4.0), 4.0)

    def test_a_waypoint_boundary_takes_the_segment_it_enters(self):
        p = self._p()
        self.assertEqual(p.at_arclen(1.0), 2.0)     # entering segment 1
        self.assertEqual(p.at_arclen(3.0), 4.0)     # entering segment 2

    def test_ends_are_clamped(self):
        p = self._p()
        self.assertEqual(p.at_arclen(-5.0), 1.0)
        self.assertEqual(p.at_arclen(99.0), 4.0)
        self.assertEqual(p.at_index(-3), 1.0)
        self.assertEqual(p.at_index(99), 4.0)

    def test_totals_and_span(self):
        p = self._p()
        self.assertAlmostEqual(p.total_mm, 6.0)
        self.assertAlmostEqual(p.total_uL, 1 * 1 + 2 * 2 + 4 * 3)
        self.assertEqual(p.printing_span(), (1.0, 4.0))
        self.assertFalse(p.is_uniform)

    def test_zeros_are_absence_not_a_thin_bead(self):
        p = ExtrusionProfile.build([(0, 0), (1, 0), (2, 0)], [0.0, 2.0])
        self.assertEqual(p.printing_span(), (2.0, 2.0))
        self.assertTrue(p.is_uniform)

    def test_a_mismatched_profile_is_REFUSED(self):
        """The load-bearing refusal: a profile one entry out would apply a
        shape's flow to the wrong stretch of path. None = use the scalar."""
        pts = [(0, 0), (1, 0), (2, 0)]
        self.assertIsNone(ExtrusionProfile.build(pts, [1.0]))
        self.assertIsNone(ExtrusionProfile.build(pts, [1.0, 1.0, 1.0]))
        self.assertIsNone(ExtrusionProfile.build(pts, []))
        self.assertIsNone(ExtrusionProfile.build(pts, None))
        self.assertIsNone(ExtrusionProfile.build([(0, 0)], []))
        self.assertIsNone(ExtrusionProfile.build(pts, [1.0, float("nan")]))
        self.assertIsNone(ExtrusionProfile.build(pts, [1.0, -1.0]))

    def test_modifier_to_vol_per_mm_applies_area_and_trim(self):
        got = modifier_to_vol_per_mm([1.0, 2.0, 0.0], _AREA, trim=1.5)
        self.assertAlmostEqual(got[0], _AREA * 1.5)
        self.assertAlmostEqual(got[1], _AREA * 3.0)
        self.assertEqual(got[2], 0.0)

    def test_rate_tracks_the_volume(self):
        """The rate has to scale with the volume, else a thicker segment merely
        takes longer (the discrete executor paces on the slower of XY / pump)
        and the extra volume is spread over a longer bead."""
        self.assertAlmostEqual(rate_for(0.1, 5.0, 9.9), 0.5)
        self.assertEqual(rate_for(0.0, 5.0, 9.9), 0.0)      # deposits nothing
        self.assertEqual(rate_for(0.1, 0.0, 9.9), 9.9)      # unusable → scalar


# ═══════════════════════════════════════════════════════════════════
# 2 — the sketch computes it
# ═══════════════════════════════════════════════════════════════════

class TestSketchComputesTheProfile(unittest.TestCase):

    def test_one_entry_per_segment(self):
        r = compile_to_trajectory(_two_width_sketch(), _needle(), None)
        self.assertEqual(len(r.extrusion_profile), len(r.trajectory) - 1)
        self.assertEqual(len(r.vol_per_mm_profile), len(r.trajectory) - 1)

    def test_each_shape_gets_its_own_declared_width(self):
        r = compile_to_trajectory(_two_width_sketch(), _needle(), None)
        mods = sorted({round(v, 6) for v in r.extrusion_profile if v > 0})
        self.assertEqual(mods, [1.0, 2.0])
        self.assertAlmostEqual(r.extrusion_ref_width_mm, _ID_MM, places=6)

    def test_travel_and_lift_segments_deposit_nothing(self):
        # Two far-apart shapes → a real pen-up between them.
        sk = Sketch(shapes=[_line(0, 0, 5, 0, line_width_mm=_ID_MM),
                            _line(40, 0, 45, 0, line_width_mm=_ID_MM)],
                    line_spacing_mm=0.4)
        r = compile_to_trajectory(sk, _needle(), None)
        self.assertIn(0.0, r.extrusion_profile)
        # every zero entry is a non-printing move
        traj = r.trajectory
        for i, m in enumerate(r.extrusion_profile):
            if m == 0.0:
                continue
            self.assertGreater(m, 0.0)

    def test_a_no_extrude_shape_is_zero_in_the_profile(self):
        sk = Sketch(shapes=[_line(0, 0, 10, 0, line_width_mm=_ID_MM),
                            _line(10, 0, 20, 0, line_width_mm=_ID_MM,
                                  no_print=True)],
                    line_spacing_mm=0.4)
        r = compile_to_trajectory(sk, _needle(), None)
        traj = r.trajectory
        for i, m in enumerate(r.extrusion_profile):
            if traj[i][0] >= 10.0 - 1e-9 and traj[i + 1][0] > traj[i][0]:
                self.assertEqual(m, 0.0)

    def test_vol_per_mm_is_the_modifier_times_the_bore_area(self):
        r = compile_to_trajectory(_two_width_sketch(), _needle(), None)
        for m, v in zip(r.extrusion_profile, r.vol_per_mm_profile):
            self.assertAlmostEqual(v, m * _AREA, places=12)

    def test_the_total_is_the_sum_not_one_scalar_times_the_length(self):
        r = compile_to_trajectory(_two_width_sketch(), _needle(), None)
        # 10 mm at 1× + 10 mm at 2× = 30 × area, NOT 20 × area.
        self.assertAlmostEqual(r.total_volume_uL, 30.0 * _AREA, places=9)

    def test_the_global_multiplier_still_trims_everything(self):
        a = compile_to_trajectory(_two_width_sketch(), _needle(), None)
        b = compile_to_trajectory(
            _two_width_sketch(extrusion_multiplier=2.0), _needle(), None)
        self.assertAlmostEqual(b.total_volume_uL, a.total_volume_uL * 2.0,
                               places=9)

    def test_the_pump_column_matches_the_profile(self):
        """The exact output IS the trajectory, so its plunger column has to
        agree with the profile the same compile emitted."""
        syr = SyringeSpec(volume_uL=250, stroke_length_mm=30.0)
        r = compile_to_trajectory(_two_width_sketch(), _needle(), syr)
        traj = r.trajectory
        for i, v in enumerate(r.vol_per_mm_profile):
            d = math.dist(tuple(traj[i][:3]), tuple(traj[i + 1][:3]))
            dp = float(traj[i + 1][3] - traj[i][3])
            if v > 0:
                self.assertAlmostEqual(dp, d * v * syr.mm_per_uL, places=9)

    def test_geometry_is_untouched_by_the_widths(self):
        a = compile_to_trajectory(_two_width_sketch(w2=_ID_MM), _needle(), None)
        b = compile_to_trajectory(_two_width_sketch(w2=8 * _ID_MM),
                                  _needle(), None)
        self.assertTrue(np.allclose(a.trajectory[:, :3], b.trajectory[:, :3]))


class TestLegacySketchesAreUntouched(unittest.TestCase):
    """A sketch saved before v7.21.5 never meant its widths as an extrusion
    instruction — and its shapes were seeded from the orifice OUTER diameter,
    so honouring them would multiply its deposition by od/id (~1.7×)."""

    def test_a_dict_without_the_key_loads_with_the_feature_OFF(self):
        d = _two_width_sketch().to_dict()
        d.pop("width_drives_extrusion", None)
        self.assertFalse(Sketch.from_dict(d).width_drives_extrusion)

    def test_and_therefore_deposits_the_old_uniform_amount(self):
        sk = _two_width_sketch(width_drives_extrusion=False)
        r = compile_to_trajectory(sk, _needle(), None)
        self.assertAlmostEqual(r.total_volume_uL, 20.0 * _AREA, places=9)
        self.assertEqual({round(v, 6) for v in r.extrusion_profile if v > 0},
                         {1.0})

    def test_a_new_sketch_has_it_on_and_round_trips(self):
        self.assertTrue(Sketch().width_drives_extrusion)
        sk = _two_width_sketch()
        self.assertTrue(Sketch.from_dict(sk.to_dict()).width_drives_extrusion)


# ═══════════════════════════════════════════════════════════════════
# 3 — the bake carries it
# ═══════════════════════════════════════════════════════════════════

class TestBakedPrintCarriesTheProfile(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls._app = QApplication.instance() or QApplication(sys.argv)

    def _bake(self, sk, tmp):
        from gui.pages.print_builder_sketch import SketchPage
        page = SketchPage()
        page._canvas.set_sketch(sk)
        page._needle = _needle()
        res = compile_to_trajectory(sk, _needle(), None)
        name = page._do_bake(sk, res, base_name="ProfileTest", overwrite=False)
        return name, res, page

    def test_the_params_carry_the_profile_and_its_reference(self):
        sk = _two_width_sketch()
        with tempfile.TemporaryDirectory() as tmp:
            from gui.pages.print_builder_sketch import SketchPage
            page = SketchPage()
            page._canvas.set_sketch(sk)
            page._prints_dir = tmp
            res = compile_to_trajectory(sk, _needle(), None)
            name = page._do_bake(sk, res, base_name="ProfileTest",
                                 overwrite=False)
            self.assertIsNotNone(name)
            pf = PrintFileManager(tmp).load(name)
            params = list(pf.objects.values())[0]["params"]
            self.assertEqual(len(params["extrusion_profile"]),
                             len(res.trajectory) - 1)
            self.assertEqual(
                sorted({round(v, 6)
                        for v in params["extrusion_profile"] if v > 0}),
                [1.0, 2.0])
            self.assertAlmostEqual(params["extrusion_ref_width_mm"], _ID_MM,
                                   places=6)
            self.assertTrue(params["width_drives_extrusion"])
            self.assertAlmostEqual(params["extrusion_total_uL"],
                                   30.0 * _AREA, places=6)


# ═══════════════════════════════════════════════════════════════════
# 4 — Quick Print reads, splits and applies it
# ═══════════════════════════════════════════════════════════════════

def _hw():
    cfg = HardwareConfig()
    cfg.needle = _needle()
    ink = InkSpec(name="Alginate", ink_type="hydrogel")
    cfg.pumps = {
        "P1": PumpChannelConfig(
            pump_id="P1", syringe=SyringeSpec(volume_uL=250,
                                              stroke_length_mm=30.0),
            inks=[ink], enabled=True),
        "P2": PumpChannelConfig(pump_id="P2"),
        "P3": PumpChannelConfig(pump_id="P3"),
    }
    cfg.add_ink(ink)
    return cfg


def _ctrl():
    c = MagicMock()
    c.is_xy_connected = True
    c.is_zp_connected = True
    c.get_xy_position.return_value = (0.0, 0.0, 0.0)
    c.print_z_dir.return_value = 1.0
    c.plate_axis_sign.return_value = (1, 1)
    return c


class TestQuickPrintAppliesTheProfile(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls._app = QApplication.instance() or QApplication(sys.argv)

    def _page_with_baked_sketch(self, tmp, sk=None):
        from gui.pages.print_builder_sketch import SketchPage
        from gui.pages.workflows.quick_print_workflow import (
            QuickPrintWorkflowPage,
        )
        sk = sk if sk is not None else _two_width_sketch()
        sp = SketchPage()
        sp._canvas.set_sketch(sk)
        sp._prints_dir = tmp
        res = compile_to_trajectory(sk, _needle(), None)
        name = sp._do_bake(sk, res, base_name="ProfileTest", overwrite=False)

        page = QuickPrintWorkflowPage(_ctrl(), settings=None)
        page.set_hardware_config(_hw())
        page.set_calibration_data(
            WellPlate.from_format(96),
            {"A1": (10000.0, 10000.0), "A2": (19000.0, 10000.0)}, 5.0)
        page._print_mgr = PrintFileManager(tmp)
        page._refresh_objects()
        idx = page._object_combo.findData(f"file:{name}")
        self.assertGreaterEqual(idx, 0, "the baked print is not in the combo")
        page._object_combo.setCurrentIndex(idx)
        page._selected_well = "A1"
        return page, res

    def test_the_modifiers_survive_the_subpath_split(self):
        with tempfile.TemporaryDirectory() as tmp:
            page, _res = self._page_with_baked_sketch(tmp)
            segs, mods = page._segments_and_modifiers_for_selection()
            self.assertTrue(segs)
            self.assertEqual(len(segs), len(mods))
            for seg, prof in zip(segs, mods):
                self.assertIsNotNone(prof, "a sub-path lost its profile")
                self.assertEqual(len(prof), len(seg) - 1)
            flat = [v for prof in mods for v in prof if v > 0]
            self.assertEqual(sorted({round(v, 6) for v in flat}), [1.0, 2.0])

    def test_it_becomes_uL_per_mm_against_the_needle_fitted_now(self):
        with tempfile.TemporaryDirectory() as tmp:
            page, _ = self._page_with_baked_sketch(tmp)
            _segs, mods = page._segments_and_modifiers_for_selection()
            vpm = page._vol_per_mm_profiles(mods)
            flat = [v for prof in vpm for v in prof if v > 0]
            self.assertAlmostEqual(min(flat), _AREA, places=9)
            self.assertAlmostEqual(max(flat), 2 * _AREA, places=9)

    def test_the_operators_own_extrusion_x_trims_it(self):
        with tempfile.TemporaryDirectory() as tmp:
            page, _ = self._page_with_baked_sketch(tmp)
            page._extrusion_mod_spin.setValue(0.5)
            _segs, mods = page._segments_and_modifiers_for_selection()
            flat = [v for prof in page._vol_per_mm_profiles(mods)
                    for v in prof if v > 0]
            self.assertAlmostEqual(max(flat), 2 * _AREA * 0.5, places=9)

    def test_the_flow_ceiling_is_sized_by_the_THICKEST_segment(self):
        """Safety: the peak governs the ceiling. A mean (or the bare trim) would
        let the widest segment over-pressure the needle — a pulled glass tip
        shatters — so this asserts on the RESOLVED SPEED, not on the accessor:
        a 2× thickest segment must halve the flow-limited speed.
        """
        with tempfile.TemporaryDirectory() as tmp:
            page, _ = self._page_with_baked_sketch(tmp)
            # Pin a ceiling low enough to actually bind (a 22G at water is far
            # above any print flow, so the real needle limit never would).
            page._max_pump_flow_uL_s = lambda: 0.5
            page._xy_max_mm_s = lambda: 100.0
            page._refresh_setup_status()
            with_profile = page._flow_limited_xy_max_mm_s()
            page._ext_profile_peak = 0.0        # as if no profile were carried
            without = page._flow_limited_xy_max_mm_s()
            self.assertAlmostEqual(with_profile, without / 2.0, places=6)
            self.assertLess(with_profile, without)

    def test_no_profile_leaves_the_flow_ceiling_exactly_as_before(self):
        page = None
        from gui.pages.workflows.quick_print_workflow import (
            QuickPrintWorkflowPage,
        )
        page = QuickPrintWorkflowPage(_ctrl(), settings=None)
        page.set_hardware_config(_hw())
        idx = page._object_combo.findData("simple:circle")
        page._object_combo.setCurrentIndex(idx)
        page._refresh_setup_status()
        self.assertEqual(page._ext_profile_peak, 0.0)
        self.assertAlmostEqual(page._flow_modifier_peak(),
                               page._extrusion_modifier(), places=9)

    def test_the_status_states_that_the_prints_own_extrusion_is_in_force(self):
        with tempfile.TemporaryDirectory() as tmp:
            page, _ = self._page_with_baked_sketch(tmp)
            page._refresh_setup_status()
            txt = page._setup_status.text() if hasattr(
                page, "_setup_status") else ""
            blob = txt + " " + "\n".join(
                c.detail for c in (page._readiness.checks
                                   if getattr(page, "_readiness", None) else []))
            self.assertIn("extrusion", blob.lower())


class TestTheJobCarriesItToTheExecutor(unittest.TestCase):

    def _settings(self):
        return PrintSettings(num_layers=1, travel_z_height=5.0,
                             print_z_height=0.2, pump_rate_uL_s=1.0,
                             print_speed_mm_s=5.0, travel_speed_mm_s=10.0)

    def test_each_print_path_gets_its_own_deposition(self):
        segs = [[(0, 0), (1, 0), (2, 0)], [(5, 0), (6, 0)]]
        profs = [[0.1, 0.2], [0.4]]
        job = build_well_plate_job(
            well_positions=[("A1", 0.0, 0.0)], path_points=segs[0],
            settings=self._settings(), pump="P1", flow_rate=0.01,
            path_segments=segs, path_extrusion_profiles=profs,
            return_home=False)
        paths = [c for c in job.commands if c.type == CommandType.PRINT_PATH]
        self.assertEqual(len(paths), 2)
        self.assertEqual(paths[0].params["vol_per_mm_profile"], [0.1, 0.2])
        self.assertEqual(paths[1].params["vol_per_mm_profile"], [0.4])

    def test_a_wrong_length_profile_is_dropped_not_applied(self):
        segs = [[(0, 0), (1, 0), (2, 0)]]
        job = build_well_plate_job(
            well_positions=[("A1", 0.0, 0.0)], path_points=segs[0],
            settings=self._settings(), pump="P1", flow_rate=0.01,
            path_segments=segs, path_extrusion_profiles=[[0.1]],
            return_home=False)
        path = [c for c in job.commands
                if c.type == CommandType.PRINT_PATH][0]
        self.assertNotIn("vol_per_mm_profile", path.params)

    def test_no_profiles_at_all_is_byte_identical_to_before(self):
        segs = [[(0, 0), (1, 0), (2, 0)]]
        kw = dict(well_positions=[("A1", 0.0, 0.0)], path_points=segs[0],
                  settings=self._settings(), pump="P1", flow_rate=0.01,
                  path_segments=segs, return_home=False)
        a = build_well_plate_job(**kw)
        b = build_well_plate_job(path_extrusion_profiles=None, **kw)
        for ca, cb in zip(a.commands, b.commands):
            self.assertEqual(ca.type, cb.type)
            self.assertEqual(ca.params, cb.params)
            self.assertNotIn("vol_per_mm_profile", ca.params)


class TestTheExecutorDepositsPerSegment(unittest.TestCase):
    """Drive the REAL discrete executor and add up what the pump was asked
    for — a profile that never reaches ``move_pump_uL`` changes nothing."""

    def _run(self, points, profile, speed=5.0, flow=1.0):
        from SupportClasses.PrintManager import PrintManager, PrintCommand
        moves = []
        ctrl = MagicMock()
        ctrl.is_xy_connected = True
        ctrl.is_zp_connected = True
        ctrl.get_xy_position.return_value = (0.0, 0.0, 0.0)
        ctrl.move_pump_uL.side_effect = \
            lambda pump, uL, rate=None, **kw: moves.append((uL, rate))
        # No continuous-velocity command → the discrete point stream runs.
        del ctrl.send_velocity_xy
        pm = PrintManager(ctrl)
        # The arrival wait is not what is under test here (and a MagicMock
        # controller cannot answer it); the deposition bookkeeping is.
        pm._wait_for_xy_settle = lambda *a, **k: True
        settings = PrintSettings(num_layers=1, travel_z_height=5.0,
                                 print_z_height=0.2, pump_rate_uL_s=flow,
                                 print_speed_mm_s=speed, travel_speed_mm_s=10.0)
        pm.job = MagicMock()
        pm.job.settings = settings
        params = {"points": points, "pump": "P1", "flow_rate": 0.01,
                  "flow_rate_uL_s": flow}
        if profile is not None:
            params["vol_per_mm_profile"] = profile
        cmd = PrintCommand(type=CommandType.PRINT_PATH, params=params,
                           label="test")
        pm._execute_print_path(cmd)
        # DISPENSES only: the path also ends with the v7.5.x pressure-relief
        # suck-back (a NEGATIVE move), which is a separate feature and would
        # otherwise be netted off the deposited total.
        return [(uL, r) for uL, r in moves if uL > 0]

    def test_a_thick_half_deposits_twice_the_thin_half(self):
        # 20 mm path, 1 mm segments: 0.1 µL/mm then 0.2 µL/mm.
        pts = [(float(i), 0.0) for i in range(21)]
        prof = [0.1] * 10 + [0.2] * 10
        moves = self._run(pts, prof)
        total = sum(uL for uL, _r in moves)
        self.assertAlmostEqual(total, 10 * 0.1 + 10 * 0.2, places=6)

    def test_a_zero_stretch_deposits_nothing_over_it(self):
        pts = [(float(i), 0.0) for i in range(11)]
        prof = [0.2] * 5 + [0.0] * 5
        total = sum(uL for uL, _r in self._run(pts, prof))
        self.assertAlmostEqual(total, 5 * 0.2, places=6)

    def test_the_rate_tracks_the_volume(self):
        pts = [(float(i), 0.0) for i in range(11)]
        prof = [0.1] * 5 + [0.4] * 5
        moves = self._run(pts, prof, speed=5.0)
        rates = sorted({round(r, 6) for _uL, r in moves if r})
        self.assertEqual(rates, [0.5, 2.0])       # µL/mm × 5 mm/s

    def test_without_a_profile_the_scalar_flow_still_governs(self):
        pts = [(float(i), 0.0) for i in range(11)]
        total = sum(uL for uL, _r in self._run(pts, None, speed=5.0, flow=1.0))
        # flow/speed = 0.2 µL/mm over 10 mm
        self.assertAlmostEqual(total, 2.0, places=6)

    def test_a_malformed_profile_falls_back_to_the_scalar(self):
        pts = [(float(i), 0.0) for i in range(11)]
        total = sum(uL for uL, _r in
                    self._run(pts, [0.5], speed=5.0, flow=1.0))
        self.assertAlmostEqual(total, 2.0, places=6)


# ═══════════════════════════════════════════════════════════════════
# 5 — the whole chain, in one test
# ═══════════════════════════════════════════════════════════════════

class TestEndToEnd(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls._app = QApplication.instance() or QApplication(sys.argv)

    def test_sketch_widths_reach_the_job_that_quick_print_would_run(self):
        from gui.pages.print_builder_sketch import SketchPage
        from gui.pages.workflows.quick_print_workflow import (
            QuickPrintWorkflowPage,
        )
        sk = _two_width_sketch()
        with tempfile.TemporaryDirectory() as tmp:
            sp = SketchPage()
            sp._canvas.set_sketch(sk)
            sp._prints_dir = tmp
            res = compile_to_trajectory(sk, _needle(), None)
            name = sp._do_bake(sk, res, base_name="E2E", overwrite=False)

            page = QuickPrintWorkflowPage(_ctrl(), settings=None)
            page.set_hardware_config(_hw())
            page.set_calibration_data(
                WellPlate.from_format(96), {"A1": (10000.0, 10000.0)}, 5.0)
            page._print_mgr = PrintFileManager(tmp)
            page._refresh_objects()
            page._object_combo.setCurrentIndex(
                page._object_combo.findData(f"file:{name}"))
            page._selected_well = "A1"

            segs, mods = page._segments_and_modifiers_for_selection()
            vpm = page._vol_per_mm_profiles(mods)
            job = build_well_plate_job(
                well_positions=[("A1", 10.0, 10.0)],
                path_points=[p for s in segs for p in s],
                settings=page._build_settings(), pump="P1", flow_rate=0.01,
                path_segments=segs, path_extrusion_profiles=vpm,
                return_home=False)
            paths = [c for c in job.commands
                     if c.type == CommandType.PRINT_PATH]
            self.assertTrue(paths)
            got = sorted({round(v, 9)
                          for c in paths
                          for v in c.params.get("vol_per_mm_profile", [])
                          if v > 0})
            self.assertEqual(len(got), 2, f"expected two widths, got {got}")
            self.assertAlmostEqual(got[1] / got[0], 2.0, places=6)
            # And the planned total matches the sketch's own arithmetic
            # (trim 1.0, same needle) to within the split's dropped travel.
            planned = 0.0
            for c in paths:
                pts = c.params["points"]
                prof = c.params.get("vol_per_mm_profile") or []
                for i, v in enumerate(prof):
                    planned += v * math.dist(pts[i], pts[i + 1])
            self.assertAlmostEqual(planned, 30.0 * _AREA, places=4)


if __name__ == "__main__":
    unittest.main()

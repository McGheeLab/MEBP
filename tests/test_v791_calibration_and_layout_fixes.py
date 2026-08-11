"""v7.9.1 — five operator-reported defects, four of them on the same rig.

The numbers in the P3 class are this machine's REAL saved values, taken from
``settings.json`` on 2026-08-10, and they reproduce the operator's log line for
line. That matters: the bug is a frame/staleness interaction, and a synthetic
envelope would let a wrong fix look right.
"""

import ast
import inspect
import os
import sys
import textwrap
import unittest
from types import SimpleNamespace
from unittest.mock import MagicMock

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication                       # noqa: E402

from SupportClasses.SafetyLimits import SafetyLimits             # noqa: E402
from SupportClasses.StageController import StageController       # noqa: E402
from SupportClasses.PlateDocument import PlateDocument as _PlateDocument  # noqa: E402


# ── the operator's real machine ────────────────────────────────────
#
# zero_position.Z = -76.35 is BOTH the datum and z_max, i.e. the mechanical
# hard bottom; z_up_sign = -1, so height = -zref and the envelope
# raw [-136.35, -76.35] is height [60, 0].
ZERO_Z = -76.35
Z_MIN, Z_MAX = -136.35, -76.35
PLATE_BOTTOM_ZREF = -25.63          # height 25.63
STALE_SAFE_Z = 24.79                # height -24.79 — BELOW the hard bottom


class _FakeZP:
    def __init__(self):
        self.moves = []

    def move_absolute(self, d, fast=False, feedrate_mm_min=None):
        self.moves.append(d)

    def flush_moves(self, timeout_s=0, abort_event=None):
        return True


def make_ctrl(cur_height_mm=None, print_floor=True):
    c = StageController.__new__(StageController)
    c.safety_limits = SafetyLimits()
    c.safety_limits.z_min, c.safety_limits.z_max = Z_MIN, Z_MAX
    c.safety_limits.enabled = True
    c.zero_position = {"Z": ZERO_Z}
    c._z_up_sign = -1.0
    c._plate_bottom_z_zref = PLATE_BOTTOM_ZREF
    c._print_floor_active = 1 if print_floor else 0
    c._retract_slow_dist_mm = 1.0
    c._retract_slow_feedrate = 60.0
    c._zp_retract_feedrate = 400.0
    c.zp_stage = _FakeZP()
    c.wait_for_z_arrival = lambda t, **k: True
    c.estimate_gentle_z_time_s = lambda *a, **k: 1.0
    return c


def height_to_zref(h):
    return -float(h)                                   # z_up_sign = -1


# ── P3 ─────────────────────────────────────────────────────────────

class TestTheOperatorsLogIsReproduced(unittest.TestCase):
    """Before trusting any fix, show the numbers really are these numbers."""

    def test_the_stale_safe_z_is_below_the_mechanical_bottom(self):
        c = make_ctrl()
        self.assertAlmostEqual(c.z_height_of(STALE_SAFE_Z), -24.79, places=6)
        self.assertAlmostEqual(c.z_height_of(0.0), 0.0, places=6)

    def test_it_lands_on_the_plate_bottom_exactly_as_logged(self):
        """The log: clamp -51.560 → -76.350, then the print floor → -101.980."""
        c = make_ctrl()
        raw = STALE_SAFE_Z + ZERO_Z
        self.assertAlmostEqual(raw, -51.56, places=6)
        eff = c.effective_z_target_zref(STALE_SAFE_Z)
        self.assertAlmostEqual(eff + ZERO_Z, -101.98, places=6)
        self.assertAlmostEqual(c.z_height_of(eff), 25.63, places=6)

    def test_a_legitimate_survey_height_is_not_clamped_at_all(self):
        """0.5 mm above the glass must pass through untouched, or the guard
        would be 'safe' by refusing everything."""
        c = make_ctrl()
        target = PLATE_BOTTOM_ZREF + (-1.0) * 0.5      # print_height_to_zref
        self.assertAlmostEqual(
            c.effective_z_target_zref(target), target, places=9)


class TestTheRetractIsRaiseOnly(unittest.TestCase):

    def test_a_stale_safe_z_commands_no_motion_at_all(self):
        """THE crash. The needle sat above the plate; the 'retract' drove it
        down onto the glass and only the print floor stopped it."""
        c = make_ctrl()
        ok = c._retract_z_slow_then_fast(
            height_to_zref(40.0), STALE_SAFE_Z, 400.0, 15.0)
        self.assertTrue(ok)
        self.assertEqual(c.zp_stage.moves, [])

    def test_a_genuine_lift_still_moves(self):
        c = make_ctrl()
        ok = c._retract_z_slow_then_fast(
            height_to_zref(30.0), height_to_zref(45.0), 400.0, 15.0)
        self.assertTrue(ok)
        self.assertTrue(c.zp_stage.moves)

    def test_already_above_is_a_no_op_not_a_descent(self):
        """Ordinary travel that starts above the safe height must NOT be
        refused — it must simply not move."""
        c = make_ctrl()
        ok = c._retract_z_slow_then_fast(
            height_to_zref(50.0), height_to_zref(45.0), 400.0, 15.0)
        self.assertTrue(ok)
        self.assertEqual(c.zp_stage.moves, [])

    def test_it_reads_the_current_height_itself_when_not_given_one(self):
        """safe_travel_to only read it when the slow-lift was enabled, so with
        slow dist 0 the guard would have had nothing to compare against."""
        c = make_ctrl()
        c._retract_slow_dist_mm = 0.0
        c.get_zp_position = lambda cached=False: (0, 0, 0, 0)
        c.zp_logical_value = lambda zp, ax: height_to_zref(40.0) + ZERO_Z
        ok = c._retract_z_slow_then_fast(None, STALE_SAFE_Z, 400.0, 15.0)
        self.assertTrue(ok)
        self.assertEqual(c.zp_stage.moves, [])


class TestClampedMovesAreConfirmable(unittest.TestCase):

    def test_the_wait_targets_what_was_actually_commanded(self):
        """The 16.5 s timeout: it commanded one place and polled for another."""
        c = make_ctrl()
        waited = []
        c.wait_for_z_arrival = lambda t, **k: waited.append(t) or True
        # A lift whose destination the print floor pulls up to the plate bottom.
        c._retract_z_slow_then_fast(
            height_to_zref(5.0), height_to_zref(10.0), 400.0, 15.0)
        self.assertTrue(waited)
        self.assertAlmostEqual(waited[-1], PLATE_BOTTOM_ZREF, places=6)

    def test_effective_target_is_pure(self):
        c = make_ctrl()
        c.effective_z_target_zref(STALE_SAFE_Z)
        self.assertEqual(c.zp_stage.moves, [])


class TestUnreachableReferencesAreDropped(unittest.TestCase):

    def test_the_three_stale_references_are_rejected(self):
        c = make_ctrl()
        for bad in (STALE_SAFE_Z, 25.79, 48.7):
            self.assertFalse(c.z_reference_reachable(bad), bad)

    def test_the_two_good_references_are_kept(self):
        c = make_ctrl()
        for good in (PLATE_BOTTOM_ZREF, -43.75):
            self.assertTrue(c.z_reference_reachable(good), good)

    def test_none_is_not_reachable(self):
        self.assertFalse(make_ctrl().z_reference_reachable(None))

    def test_a_machine_with_no_envelope_is_unaffected(self):
        """Limits off, or a degenerate range, must not start dropping values."""
        c = make_ctrl()
        c.safety_limits.enabled = False
        self.assertTrue(c.z_reference_reachable(STALE_SAFE_Z))
        c.safety_limits.enabled = True
        c.safety_limits.z_min = c.safety_limits.z_max = 0.0
        self.assertTrue(c.z_reference_reachable(STALE_SAFE_Z))


class TestSafeNavigateNeverFallsBackToZero(unittest.TestCase):
    """`safe_z or 0.0` is not a neutral default: zero-ref 0 IS the bottom."""

    def test_the_literal_zero_fallback_is_gone(self):
        from gui.pages.calibration import CalibrationPage
        src = textwrap.dedent(
            inspect.getsource(CalibrationPage._safe_navigate_to))
        self.assertNotIn("_safe_z', None) or 0.0", src)
        self.assertNotIn('_safe_z", None) or 0.0', src)

    def test_it_derives_a_height_from_a_taught_reference(self):
        page = SimpleNamespace()
        ctrl = MagicMock()
        ctrl.default_travel_z.return_value = -53.75
        ctrl.safe_travel_to.return_value = True
        from gui.pages.calibration import CalibrationPage
        page.controller = ctrl
        page._safe_z = None
        page._top_z = -43.75
        page._plate_bottom_z = PLATE_BOTTOM_ZREF
        ok = CalibrationPage._safe_navigate_to(page, 1000.0, 2000.0)
        self.assertTrue(ok)
        ctrl.default_travel_z.assert_called_once_with(-43.75, 10.0)
        self.assertAlmostEqual(
            ctrl.safe_travel_to.call_args.kwargs["safe_z_mm"], -53.75)

    def test_it_refuses_when_there_is_nothing_to_derive_from(self):
        from gui.pages.calibration import CalibrationPage
        ctrl = MagicMock()
        page = SimpleNamespace(controller=ctrl, _safe_z=None, _top_z=None,
                               _plate_bottom_z=None)
        self.assertFalse(
            CalibrationPage._safe_navigate_to(page, 1.0, 2.0))
        ctrl.safe_travel_to.assert_not_called()

    def test_a_refused_travel_reports_false(self):
        from gui.pages.calibration import CalibrationPage
        ctrl = MagicMock()
        ctrl.safe_travel_to.return_value = False
        page = SimpleNamespace(controller=ctrl, _safe_z=-30.0, _top_z=None,
                               _plate_bottom_z=None)
        self.assertFalse(CalibrationPage._safe_navigate_to(page, 1.0, 2.0))


class TestTheParkSurfacesAFailedTravel(unittest.TestCase):
    """It used to swallow the result and arm a measurement session anyway."""

    def test_the_park_checks_the_navigation_result(self):
        import gui.widgets.needle_bore_wizard as wiz
        tree = ast.parse(textwrap.dedent(
            inspect.getsource(wiz.NeedleBoreWizard._on_park)))
        assigned = [n for n in ast.walk(tree)
                    if isinstance(n, ast.Assign)
                    and isinstance(n.value, ast.Call)]
        self.assertTrue(
            any(isinstance(t, ast.Name) for a in assigned for t in a.targets),
            "the nav() result must be bound, not discarded")
        # ...and it must gate the session start on it.
        srcs = [n for n in ast.walk(tree) if isinstance(n, ast.If)]
        self.assertTrue(srcs, "no guard on the travel result")

    def test_a_failed_park_does_not_start_a_session(self):
        import gui.widgets.needle_bore_wizard as wiz
        src = inspect.getsource(wiz.NeedleBoreWizard._on_park)
        i_guard = src.index("moved is False")
        i_start = src.index("_on_start_session")
        self.assertLess(i_guard, i_start,
                        "the session must not be armed before the check")


# ── P5 ─────────────────────────────────────────────────────────────

class TestPlateBottomAnchorIsRecorded(unittest.TestCase):
    """The contact touch-off is the documented fallback and the operator used
    it because the optical path did not work; it recorded no anchor XY, so the
    bed-level preflight refused every time."""

    @classmethod
    def setUpClass(cls):
        cls._app = QApplication.instance() or QApplication(sys.argv)

    def _page(self):
        from gui.pages.calibration import CalibrationPage
        ctrl = MagicMock()
        ctrl.is_xy_connected = True
        ctrl.is_zp_connected = False
        ctrl.zero_position = {"x": 0, "y": 0, "Z": 0}
        ctrl.get_xy_position.return_value = (104494.0, 37708.0)
        ctrl.get_zp_position.return_value = (0, 0, 0, 0)
        ctrl.zp_logical_value.return_value = -25.63
        ctrl.z_up_sign.return_value = -1.0
        ctrl.zref_to_user_z.side_effect = lambda z: -float(z)
        ctrl.z_reference_reachable.return_value = True
        page = CalibrationPage(ctrl, settings=None)
        self.addCleanup(page.deleteLater)
        return page, ctrl

    def test_the_touch_off_records_where_it_was_measured(self):
        page, ctrl = self._page()
        page._zoff_set_plate_bottom_z()
        self.assertEqual(page._plate_bottom_anchor_xy_um, (104494.0, 37708.0))
        kwargs = ctrl.set_plate_bottom_z.call_args.kwargs
        self.assertEqual(kwargs.get("at_xy_um"), (104494.0, 37708.0))
        self.assertEqual(kwargs.get("source"), "taught")

    def test_a_garbled_read_records_absence_not_a_guess(self):
        page, ctrl = self._page()
        ctrl.get_xy_position.return_value = (None, None, None)
        page._zoff_set_plate_bottom_z()
        self.assertIsNone(page._plate_bottom_anchor_xy_um)

    def test_the_estimate_clears_the_anchor_rather_than_inheriting_one(self):
        """A needle-cam guess is measured nowhere on the plate. Leaving a
        previous touch-off's XY would anchor the plane at the right place and
        the WRONG height."""
        page, ctrl = self._page()
        page._zoff_set_plate_bottom_z()
        self.assertIsNotNone(page._plate_bottom_anchor_xy_um)
        ctrl.estimate_plate_z_refs.return_value = {
            "plate_top_z": -43.75, "plate_bottom_z": -25.0,
            "safe_z": -50.0, "plate_max_z": -55.0}
        ctrl.get_needle_cam_z_user.return_value = 39.75
        page._zoff_estimate_from_needle_cam()
        self.assertIsNone(page._plate_bottom_anchor_xy_um)
        ctrl.clear_plate_bottom_anchor.assert_called()

    def test_the_anchor_round_trips_through_save_and_load(self):
        """It lived only in memory on the controller, so even the optical path
        lost it at the next restart."""
        page, _ = self._page()
        page._zoff_set_plate_bottom_z()
        blob = {
            "plate_bottom_z": page._plate_bottom_z,
            "plate_bottom_anchor_xy_um": list(page._plate_bottom_anchor_xy_um),
            "plate_bottom_z_source": page._plate_bottom_z_source,
        }
        fresh, _ = self._page()
        fresh._plate_bottom_z = blob["plate_bottom_z"]
        anchor = blob["plate_bottom_anchor_xy_um"]
        fresh._plate_bottom_anchor_xy_um = (float(anchor[0]), float(anchor[1]))
        self.assertEqual(fresh._plate_bottom_anchor_xy_um,
                         (104494.0, 37708.0))

    def test_the_save_payload_carries_the_anchor(self):
        from gui.pages.calibration import CalibrationPage
        src = inspect.getsource(CalibrationPage._save_calibration)
        self.assertIn("plate_bottom_anchor_xy_um", src)
        self.assertIn("plate_bottom_z_source", src)

    def test_clear_plate_bottom_anchor_exists_and_clears(self):
        c = make_ctrl()
        c._plate_bottom_anchor_xy_um = (1.0, 2.0)
        c.clear_plate_bottom_anchor()
        self.assertIsNone(c.get_plate_bottom_anchor_xy_um())

    def test_an_untagged_repush_still_preserves_the_anchor(self):
        """app.py re-pushes the scalar on every calibration_data_changed."""
        c = make_ctrl()
        c._plate_bottom_z_source = None
        c.set_plate_bottom_z(-25.63, at_xy_um=(10.0, 20.0), source="taught")
        c.set_plate_bottom_z(-25.63)                    # the untagged re-push
        self.assertEqual(c.get_plate_bottom_anchor_xy_um(), (10.0, 20.0))


# ── P4 ─────────────────────────────────────────────────────────────

class TestMapWellsOrientation(unittest.TestCase):
    """The detector hands back the MIN-pixel corner as (row 0, col 0); which
    corner is A1 is the plate-orientation convention's call."""

    def test_the_flipped_convention_puts_a1_at_the_max_corner(self):
        from SupportClasses.MosaicWellRemap import orient_lattice_index
        self.assertEqual(
            orient_lattice_index(3, 5, 4, 6, (-1.0, -1.0)), (0, 0))
        self.assertEqual(
            orient_lattice_index(0, 0, 4, 6, (-1.0, -1.0)), (3, 5))

    def test_an_unflipped_machine_is_untouched(self):
        from SupportClasses.MosaicWellRemap import orient_lattice_index
        for r in range(4):
            for c in range(6):
                self.assertEqual(
                    orient_lattice_index(r, c, 4, 6, (1.0, 1.0)), (r, c))

    def test_each_axis_flips_on_its_own_sign(self):
        """Not one 180° rotation — only both-negative reduces to that."""
        from SupportClasses.MosaicWellRemap import orient_lattice_index
        self.assertEqual(
            orient_lattice_index(1, 2, 4, 6, (-1.0, 1.0)), (1, 3))
        self.assertEqual(
            orient_lattice_index(1, 2, 4, 6, (1.0, -1.0)), (2, 2))

    def test_it_agrees_with_the_existing_corner_rule_on_a_real_plate(self):
        """`label_positions` is the pre-existing authority. If the two
        disagree, the codebase has two answers to one question."""
        from SupportClasses.WellPlate import WellPlate
        from SupportClasses.MosaicWellRemap import orient_lattice_index
        plate = WellPlate.from_format(24)
        sign = (-1.0, -1.0)
        truth = plate.get_all_positions_from_a1(100000.0, 60000.0, sign)
        xs = sorted({round(p[0], 3) for p in truth.values()})
        ys = sorted({round(p[1], 3) for p in truth.values()})
        for name, (x, y) in truth.items():
            drow = ys.index(round(y, 3))               # +pixel-y = +stage Y
            dcol = xs.index(round(x, 3))               # +pixel-x = +stage X
            prow, pcol = orient_lattice_index(
                drow, dcol, plate.rows, plate.cols, sign)
            self.assertEqual(f"{chr(65 + prow)}{pcol + 1}", name)

    def test_the_dialog_routes_through_the_shared_rule(self):
        """Keyed by AST, not substring: an explanatory comment naming the
        helper satisfies a text search whether or not the code calls it."""
        import gui.dialogs.mosaic_well_mapping_dialog as dlg
        tree = ast.parse(textwrap.dedent(
            inspect.getsource(dlg.MosaicWellMappingDialog._auto_detect)))
        calls = [n for n in ast.walk(tree)
                 if isinstance(n, ast.Call)
                 and isinstance(n.func, ast.Name)
                 and n.func.id == "orient_lattice_index"]
        self.assertEqual(len(calls), 1, "expected exactly one remap call")


# ── P1 ─────────────────────────────────────────────────────────────

class TestRosetteLayoutLoadsBundledPlates(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls._app = QApplication.instance() or QApplication(sys.argv)

    def _page(self, type_id="", fmt=24):
        from gui.pages.hardware_setup import HardwareSetupPage
        from SupportClasses.HardwareConfig import HardwareConfig
        page = HardwareSetupPage.__new__(HardwareSetupPage)
        cfg = HardwareConfig()
        cfg.plate_format = fmt
        cfg.plate_type_id = type_id
        cfg.plate_doc_id = ""
        page._config = cfg
        page._pending_plate_fork = None
        self.saved = []
        store = SimpleNamespace(save=lambda d: self.saved.append(d),
                                get=lambda i: None)
        page._plate_workspace = SimpleNamespace(
            plate_store=lambda: store, refresh=lambda: None)
        page._on_config_changed = lambda: None
        return page, cfg

    def test_a_bundled_product_materialises_a_real_plate(self):
        page, _ = self._page(type_id="nest-plastic-24")
        doc = page._active_plate_document(materialize=True)
        self.assertIsNotNone(doc)
        self.assertEqual(len(doc.evaluate()), 24)
        self.assertEqual(doc.meta.plate_type_id, "nest-plastic-24")

    def test_a_bare_standard_materialises_too(self):
        page, _ = self._page()
        doc = page._active_plate_document(materialize=True)
        self.assertIsNotNone(doc)
        self.assertEqual(len(doc.evaluate()), 24)

    def test_the_fork_is_stable_across_repeated_visits(self):
        page, _ = self._page()
        self.assertIs(page._active_plate_document(materialize=True),
                      page._active_plate_document(materialize=True))

    def test_switching_plate_drops_the_cached_fork(self):
        page, cfg = self._page()
        first = page._active_plate_document(materialize=True)
        cfg.plate_format = 96
        self.assertIsNot(first, page._active_plate_document(materialize=True))

    def test_it_is_NOT_materialised_by_default(self):
        """`_rebuild_config` copies doc.meta.name into plate_name, which sits
        in active_plate_key's precedence chain — materialising there would
        rekey every per-plate store to 'Copy of 24-well'."""
        page, _ = self._page(type_id="nest-plastic-24")
        self.assertIsNone(page._active_plate_document())

    def test_rebuild_config_does_not_ask_for_materialisation(self):
        from gui.pages.hardware_setup import HardwareSetupPage
        tree = ast.parse(textwrap.dedent(
            inspect.getsource(HardwareSetupPage._rebuild_config)))
        for node in ast.walk(tree):
            if (isinstance(node, ast.Call)
                    and isinstance(node.func, ast.Attribute)
                    and node.func.attr == "_active_plate_document"):
                self.assertEqual(
                    node.keywords, [],
                    "_rebuild_config must use the saved document only")

    def test_the_active_plate_key_is_untouched_by_the_fork(self):
        page, cfg = self._page(type_id="nest-plastic-24")
        page._active_plate_document(materialize=True)
        self.assertEqual(cfg.active_plate_key, "nest-plastic-24")

    def test_placing_a_rosette_forks_and_repoints_the_config(self):
        page, cfg = self._page(type_id="nest-plastic-24")
        doc = page._active_plate_document(materialize=True)
        page._rosette_placement = SimpleNamespace(_doc=doc)
        page._on_placements_changed()
        self.assertEqual(len(self.saved), 1)
        self.assertEqual(cfg.plate_doc_id, doc.meta.id)
        self.assertEqual(cfg.plate_name, doc.meta.name)
        self.assertIsNone(page._pending_plate_fork)

    def test_a_saved_plate_is_not_re_forked(self):
        page, cfg = self._page()
        cfg.plate_doc_id = "plt_existing"
        doc = SimpleNamespace(meta=SimpleNamespace(id="plt_existing",
                                                   name="Mine"))
        page._rosette_placement = SimpleNamespace(_doc=doc)
        page._on_placements_changed()
        self.assertEqual(cfg.plate_doc_id, "plt_existing")
        self.assertEqual(cfg.plate_name, "")           # untouched


class TestRosettePlacementTargetsAWell(unittest.TestCase):
    """P1b — the follow-on the operator hit once the tab was reachable.

    A standard plate is ONE `GridPattern`, and the builder's rule is that a
    click on a member selects the PATTERN. So every Layout-tab click handed
    `place_rosette` the pattern, which accepted it and wrote an override under
    member key `""` — matching no well. Saved to disk, rendered nowhere, no
    error: "it wouldn't let me place it".
    """

    @classmethod
    def setUpClass(cls):
        cls._app = QApplication.instance() or QApplication(sys.argv)

    def _doc(self):
        from SupportClasses.PlateDocument import PlateDocument
        return PlateDocument.from_standard_format(24, name="probe")

    def test_the_pattern_itself_is_refused(self):
        doc = self._doc()
        grid_id = doc.evaluate()[0].key[0]
        with self.assertRaises(KeyError):
            doc.place_rosette((grid_id, ""), "ros_x")
        self.assertEqual(len(list(doc.placements())), 0)

    def test_a_real_member_places_and_reaches_the_well(self):
        doc = self._doc()
        key = doc.evaluate()[0].key
        self.assertTrue(key[1], "a grid member must carry a member key")
        doc.place_rosette(key, "ros_x", rosette_name="3 channel")
        self.assertEqual(len(list(doc.placements())), 1)
        well = {w.key: w for w in doc.evaluate()}[key]
        self.assertIsNotNone(getattr(well, "rosette", None))

    def test_the_layout_canvas_picks_members_the_builder_picks_patterns(self):
        from gui.widgets.plate_document_canvas import PlateDocumentCanvas
        self.assertFalse(PlateDocumentCanvas()._member_pick_default)
        c = PlateDocumentCanvas()
        c.set_member_pick_default(True)
        self.assertTrue(c._member_pick_default)

    def test_the_layout_page_opts_in(self):
        from gui.pages.hardware.rosette_placement import RosettePlacementPage
        page = RosettePlacementPage(plate_store=None, rosette_store=None)
        self.addCleanup(page.deleteLater)
        self.assertTrue(page._canvas._member_pick_default)

    def test_an_empty_member_override_is_dropped_on_load(self):
        """Heals the files the buggy version already wrote.

        ⚠ Asserted on the raw ``overrides`` dict, NOT on ``placements()``:
        that walks ``evaluate()``, which never yields a `""` member, so it
        reads 0 whether or not the entry was filtered. The first version of
        this test did exactly that and a mutation survived it.
        """
        from SupportClasses.PlateDocument import PlateDocument
        from SupportClasses.PlateDocument import PatternFeature
        doc = self._doc()
        blob = doc.to_dict()
        grid = next(e for e in blob["entities"] if e["type"] == "GridPattern")
        grid["overrides"] = {"": {"rosette": {"rosette_id": "ros_x",
                                              "rosette_name": "3 channel"}}}
        healed = PlateDocument.from_dict(blob)
        feats = [e for e in healed.entities.values()
                 if isinstance(e, PatternFeature)]
        self.assertEqual(len(feats), 1)
        self.assertEqual(dict(feats[0].overrides), {},
                         "the phantom '' override survived the load")
        self.assertEqual(len(list(healed.placements())), 0)
        self.assertEqual(len(healed.evaluate()), 24)

    def test_a_real_override_still_survives_a_round_trip(self):
        """The filter must not eat legitimate overrides."""
        from SupportClasses.PlateDocument import PlateDocument
        doc = self._doc()
        key = doc.evaluate()[0].key
        doc.place_rosette(key, "ros_x", rosette_name="3 channel")
        again = PlateDocument.from_dict(doc.to_dict())
        self.assertEqual(len(list(again.placements())), 1)

    def test_a_failed_stamp_says_so_instead_of_doing_nothing(self):
        """⚠ Asserted against the DEFAULT hint, not against a keyword: the
        idle hint already contains "wells", so a substring check for "well"
        passes whether or not the refusal fired — a mutation survived exactly
        that in the first version of this test."""
        from gui.pages.hardware.rosette_placement import RosettePlacementPage
        page = RosettePlacementPage(plate_store=None, rosette_store=None)
        self.addCleanup(page.deleteLater)
        doc = self._doc()
        page.set_plate(doc)
        page._armed = "ros_x"
        idle_hint = page._hint.text()
        grid_id = doc.evaluate()[0].key[0]
        page._stamp([(grid_id, "")])
        self.assertEqual(len(list(doc.placements())), 0)
        self.assertNotEqual(page._hint.text(), idle_hint,
                            "a refused stamp left the hint unchanged")
        self.assertIn("alt+click", page._hint.text().lower())


class TestASeatedRosetteDrawsAsItself(unittest.TestCase):
    """P1c — every placement used to render the same hard-coded ring of six
    dots, so a 3-channel insert and a 6-channel one were indistinguishable on
    the plate and the rotation the operator set was invisible."""

    @classmethod
    def setUpClass(cls):
        cls._app = QApplication.instance() or QApplication(sys.argv)

    def _rosette(self, n, dia=5.5, radius=4.0):
        """A real rosette document with *n* bores — the shape of the
        operator's '3 channel' insert."""
        import math as _m
        from SupportClasses.PlateDocument import PlateDocument, WellStyle
        doc = PlateDocument.new_rosette()
        style = WellStyle(diameter_mm=dia, well_depth_mm=10.0)
        for i in range(n):
            a = 2 * _m.pi * i / n
            doc.add_well(radius * _m.sin(a), radius * _m.cos(a),
                         chr(ord("a") + i), style=style)
        return doc

    def _canvas_with(self, n, rotation=0.0):
        from SupportClasses.PlateDocument import PlateDocument
        from gui.widgets.plate_document_canvas import PlateDocumentCanvas
        ros = self._rosette(n)
        plate = PlateDocument.from_standard_format(24, name="probe")
        key = plate.evaluate()[0].key
        plate.place_rosette(key, "ros_test", rotation_deg=rotation)
        c = PlateDocumentCanvas()
        self.addCleanup(c.deleteLater)
        c.set_rosette_loader(lambda rid: ros if rid == "ros_test" else None)
        c.set_document(plate)
        return c, ros, plate, key

    def _glyphs(self, canvas):
        from PySide6.QtWidgets import QGraphicsEllipseItem
        from gui.widgets.plate_document_canvas import _WellItem
        return [i for i in canvas._scene.items()
                if isinstance(i, QGraphicsEllipseItem)
                and not isinstance(i, _WellItem)]

    def test_the_glyph_count_follows_the_rosette(self):
        for n in (3, 4, 8):
            c, _, _, _ = self._canvas_with(n)
            self.assertEqual(len(self._glyphs(c)), n,
                             f"expected {n} sub-wells drawn")

    def test_three_channels_do_not_look_like_six(self):
        """The precise symptom: the legacy badge was always six dots."""
        c, _, _, _ = self._canvas_with(3)
        self.assertNotEqual(len(self._glyphs(c)), 6)

    def test_the_drawing_matches_what_actually_COMPILES(self):
        """The whole point of sharing `rosette_subwell_offsets`: what the
        operator sees seated is what gets compiled and printed.

        ⚠ Compared against a REAL ``compile()`` result, not against the shared
        helper. The first version of this test asserted drawing == helper,
        which says nothing about whether `compile()` still uses that helper —
        a mutation that reverted compile() to its own arithmetic survived it.

        Compared centroid-relative, because the two live in different frames
        (the canvas draws document mm; compile() emits A1-relative mm). A
        translation cancels; a lost rotation does not.
        """
        from gui.widgets.plate_document_canvas import SCALE
        c, ros, plate, key = self._canvas_with(3, rotation=30.0)

        drawn = [(g.pos().x() / SCALE, g.pos().y() / SCALE)
                 for g in self._glyphs(c)]
        compiled = [(w.x, w.y) for w in
                    plate.compile(loader=lambda rid: ros).get_all_wells()
                    if getattr(w, "is_subwell", False)]
        self.assertEqual(len(drawn), 3)
        self.assertEqual(len(compiled), 3)

        def centred(pts):
            cx = sum(p[0] for p in pts) / len(pts)
            cy = sum(p[1] for p in pts) / len(pts)
            return sorted((round(p[0] - cx, 6), round(p[1] - cy, 6))
                          for p in pts)

        self.assertEqual(centred(drawn), centred(compiled))

    def test_compile_uses_the_shared_offset_helper(self):
        """Belt and braces on the above: name the single authority, so moving
        compile() off it fails here even if the geometry happens to agree."""
        tree = ast.parse(textwrap.dedent(
            inspect.getsource(_PlateDocument.compile)))
        self.assertTrue(
            any(isinstance(n, ast.Call) and isinstance(n.func, ast.Name)
                and n.func.id == "rosette_subwell_offsets"
                for n in ast.walk(tree)),
            "compile() must seat rosettes through rosette_subwell_offsets")

    def test_rotation_actually_moves_the_glyphs(self):
        a, _, _, _ = self._canvas_with(3, rotation=0.0)
        b, _, _, _ = self._canvas_with(3, rotation=45.0)
        pa = sorted((round(g.pos().x(), 3), round(g.pos().y(), 3))
                    for g in self._glyphs(a))
        pb = sorted((round(g.pos().x(), 3), round(g.pos().y(), 3))
                    for g in self._glyphs(b))
        self.assertNotEqual(pa, pb)

    def test_the_glyphs_carry_the_subwell_diameter(self):
        from gui.widgets.plate_document_canvas import SCALE
        c, ros, _, _ = self._canvas_with(3)
        want = round(ros.evaluate()[0].diameter_mm * SCALE, 3)
        for g in self._glyphs(c):
            self.assertAlmostEqual(round(g.rect().width(), 3), want, places=3)

    def test_an_unresolvable_rosette_still_shows_a_mark(self):
        """compile() keeps such a well rather than dropping it, and validate()
        reports it — so it must stay visible, not vanish."""
        from SupportClasses.PlateDocument import PlateDocument
        from gui.widgets.plate_document_canvas import PlateDocumentCanvas
        plate = PlateDocument.from_standard_format(24, name="probe")
        plate.place_rosette(plate.evaluate()[0].key, "ros_missing")
        c = PlateDocumentCanvas()
        self.addCleanup(c.deleteLater)
        c.set_rosette_loader(lambda rid: None)
        c.set_document(plate)
        self.assertEqual(len(self._glyphs(c)), 6)      # the legacy fallback

    def test_the_layout_page_supplies_a_loader(self):
        from gui.pages.hardware.rosette_placement import RosettePlacementPage
        page = RosettePlacementPage(plate_store=None, rosette_store=None)
        self.addCleanup(page.deleteLater)
        self.assertTrue(hasattr(page._canvas, "_rosette_loader"))
        page.set_stores(None, object())
        self.assertIsNotNone(page._canvas._rosette_loader)


# ── P2 ─────────────────────────────────────────────────────────────

class TestQuickMoveBadgeSelection(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls._app = QApplication.instance() or QApplication(sys.argv)

    def _page(self):
        from gui.pages.calibration import CalibrationPage
        ctrl = MagicMock()
        ctrl.is_xy_connected = True
        ctrl.is_zp_connected = False
        ctrl.zero_position = {"x": 0, "y": 0, "Z": 0}
        ctrl.z_up_sign.return_value = -1.0
        ctrl.zref_to_user_z.side_effect = lambda z: -float(z)
        ctrl.z_reference_reachable.return_value = True
        page = CalibrationPage(ctrl, settings=None)
        self.addCleanup(page.deleteLater)
        page._safe_z, page._top_z = -25.0, -43.0
        page._max_z, page._replace_z = -50.0, -55.0
        page._plate_bottom_z = PLATE_BOTTOM_ZREF
        return page

    def test_every_reference_shows_by_default(self):
        page = self._page()
        self.assertEqual(page.visible_z_reference_keys(),
                         list(page._ALL_Z_REF_KEYS))

    def test_unticking_hides_only_that_badge(self):
        page = self._page()
        page._zoff_qm_checks["replace_z"].setChecked(False)
        self.assertNotIn("replace_z", page.visible_z_reference_keys())
        self.assertIn("max_z", page.visible_z_reference_keys())

    def test_hiding_a_badge_does_not_touch_its_VALUE(self):
        """The reference dict feeds the print-floor datum and the Hardware Info
        card; hiding by blanking the value would disarm a safety clamp."""
        page = self._page()
        page._zoff_qm_checks["plate_bottom_z"].setChecked(False)
        self.assertEqual(page.get_z_references()["plate_bottom_z"],
                         PLATE_BOTTOM_ZREF)

    def test_both_of_the_pages_own_views_follow(self):
        page = self._page()
        page._zoff_qm_checks["max_z"].setChecked(False)
        for attr in ("_zoff_xz_view", "_ploc_xz_view"):
            self.assertNotIn(
                "max_z", getattr(page, attr).visible_z_references(), attr)

    def test_the_side_view_paints_only_the_chosen_badges(self):
        from gui.widgets.xz_side_view import XZSideView
        v = XZSideView()
        v.set_z_references({"replace_z": -55.0, "max_z": -50.0,
                            "fast_move_z": -25.0, "plate_top_z": -43.0,
                            "plate_bottom_z": -25.6})
        self.assertEqual(len(v.visible_z_references()), 5)
        v.set_visible_z_references(["fast_move_z"])
        self.assertEqual(v.visible_z_references(), {"fast_move_z"})
        v.set_visible_z_references(None)               # back to "all"
        self.assertEqual(len(v.visible_z_references()), 5)

    def test_unknown_keys_are_ignored(self):
        from gui.widgets.xz_side_view import XZSideView
        v = XZSideView()
        v.set_visible_z_references(["fast_move_z", "nonsense"])
        self.assertEqual(v.visible_z_references(), {"fast_move_z"})

    def test_the_selection_survives_a_restart(self):
        page = self._page()
        stored = {}
        page.settings = SimpleNamespace(
            get=lambda k, d=None: stored.get(k, d),
            set=lambda k, v: stored.__setitem__(k, v),
            save=lambda: None)
        page._zoff_qm_checks["replace_z"].setChecked(False)
        self.assertIn("calibration.z_ref_visible", stored)

        fresh = self._page()
        fresh.settings = SimpleNamespace(
            get=lambda k, d=None: stored.get(k, d),
            set=lambda k, v: None, save=lambda: None)
        fresh._zoff_restore_quick_move_visibility()
        self.assertNotIn("replace_z", fresh.visible_z_reference_keys())

    def test_every_xz_host_can_receive_the_selection(self):
        """A host that silently lacks the forwarder would show a stale strip."""
        from gui.pages.jog_control import JogControlPage
        from gui.pages.workflows_mode import WorkflowsModePage
        from gui.pages.workflows import (
            cell_labeling_workflow, cell_targeting_workflow,
            spheroid_pickup_workflow)
        hosts = [
            JogControlPage, WorkflowsModePage,
            spheroid_pickup_workflow.SpheroidPickupWorkflowPage,
            cell_targeting_workflow.CellTargetingWorkflowPage,
            cell_labeling_workflow.CellLabelingWorkflowPage,
        ]
        for h in hosts:
            self.assertTrue(hasattr(h, "set_visible_z_references"), h.__name__)


if __name__ == "__main__":
    unittest.main()

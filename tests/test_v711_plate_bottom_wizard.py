"""v7.11 — step 4's two modes, its gates, and the training capture.

Optical is the default because it is the one that never touches the glass. The
contact touch-off stays reachable and unchanged: a rig with no motorised focus,
or no microscope at all, must keep working exactly as it did.
"""

import os
import sys
import unittest
from types import SimpleNamespace

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication            # noqa: E402

import gui.widgets.needle_bore_wizard as wiz          # noqa: E402
from SupportClasses.PlateBottomOptical import (       # noqa: E402
    DEFAULT_OFFSETS_UM, RungMeasurement, needle_target_zref)

B_TRUE = 21.130
F0 = 1200.0
ZDIR = -1.0


class _Bore:
    orifice_id_um = 210.0
    od_um = 500.0


class _Needle:
    needle_type = "hypodermic"
    max_bore_z_offset_mm = 0.0
    tip_length_mm = None

    def bore(self, k):
        return _Bore()


def make_host(**kw):
    ctrl = kw.pop("ctrl", None) or SimpleNamespace(
        is_xy_connected=True, is_zp_connected=True,
        print_z_dir=lambda: ZDIR,
        print_height_to_zref=lambda h, *a, **k: B_TRUE + ZDIR * h,
        get_plate_bottom_z_source=lambda: "estimated",
        set_plate_bottom_z=lambda *a, **k: None,
        set_print_floor_active=lambda on: None,
        z_height_of=lambda z: ZDIR * z)
    host = SimpleNamespace(
        controller=ctrl,
        _hardware_config=SimpleNamespace(needle=_Needle()),
        _plate_bottom_z=kw.pop("plate_bottom", B_TRUE),
        _safe_z=25.0,
        _camera_manager=None,
        _ploc_camera_objective_key=lambda: "cam|10x",
        _ploc_plate_key=lambda: "24",
        _emit_calibration_data_changed=lambda: None)
    for k, v in kw.items():
        setattr(host, k, v)
    return host


class _Base(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication(sys.argv)

    def setUp(self):
        """An unanswered modal blocks FOREVER under the offscreen platform, so
        every refusal path that pops one must be stubbed. Default the question
        to No: a test that means to confirm says so explicitly, and one that
        forgets fails rather than silently accepting."""
        self._answers = []
        self._boxes = []
        patched = {}
        for name, ret in (("warning", None), ("information", None),
                          ("critical", None),
                          ("question", wiz.QMessageBox.No)):
            patched[name] = getattr(wiz.QMessageBox, name)

            def stub(*a, _ret=ret, _name=name, **k):
                self._boxes.append((_name, a[2] if len(a) > 2 else ""))
                if _name == "question" and self._answers:
                    return self._answers.pop(0)
                return _ret
            setattr(wiz.QMessageBox, name, staticmethod(stub))
        self.addCleanup(
            lambda: [setattr(wiz.QMessageBox, n, v) for n, v in patched.items()])

    def answer_yes(self, n=1):
        self._answers.extend([wiz.QMessageBox.Yes] * n)

    def make(self, **kw):
        w = wiz.NeedleBoreWizard(host=make_host(**kw))
        w.go_to_step(wiz.STEP_TOUCHOFF)
        return w


class TestTheModeSelector(_Base):
    def test_optical_is_the_default(self):
        w = self.make()
        self.assertEqual(w._s4_mode.currentData(), "optical")
        self.assertTrue(w._s4_optical_mode())

    def test_only_one_panel_shows_at_a_time(self):
        w = self.make()
        self.assertFalse(w._s4_optical_box.isHidden())
        self.assertTrue(w._s4_contact_box.isHidden())
        w._s4_mode.setCurrentIndex(1)
        self.assertTrue(w._s4_optical_box.isHidden())
        self.assertFalse(w._s4_contact_box.isHidden())

    def test_the_contact_path_keeps_every_widget_it_had(self):
        """The fallback must not be quietly degraded by the rework."""
        w = self.make()
        for attr in ("_s4_clearance", "_s4_goto", "_s4_confirm",
                     "_s4_update_bottom", "_s4_focus_ok"):
            self.assertTrue(hasattr(w, attr), attr)
        self.assertTrue(w._s4_focus_ok.isChecked())
        self.assertFalse(w._s4_update_bottom.isChecked())

    def test_the_contact_confirm_still_routes_to_the_original_handler(self):
        import inspect
        src = inspect.getsource(wiz.NeedleBoreWizard._build_step4_contact)
        self.assertIn("self._on_confirm_touchoff", src)


class TestTheMargins(_Base):
    def test_the_default_ladder_is_offered_largest_first(self):
        w = self.make()
        self.assertEqual(w._margins_um(), list(DEFAULT_OFFSETS_UM))

    def test_a_typo_yields_an_empty_list_rather_than_a_partial_one(self):
        """A partial parse would silently drop a margin from the ladder."""
        w = self.make()
        w._s4_margins.setText("1000, oops, 100")
        self.assertEqual(w._margins_um(), [])

    def test_separators_are_forgiving(self):
        w = self.make()
        w._s4_margins.setText("800 ; 400,200")
        self.assertEqual(w._margins_um(), [800.0, 400.0, 200.0])


class TestTheGate(_Base):
    def test_no_microscope_refuses_and_points_at_the_fallback(self):
        w = self.make()
        ok, why = w._optical_gate()
        self.assertFalse(ok)
        self.assertIn("motorised", why)
        self.assertIn("contact", why)

    def test_an_uncalibrated_objective_refuses_rather_than_guessing(self):
        """Falling back to the live manager's µm/px is how 'I literally just
        calibrated it' happens; the sweep step and the collision bound are both
        sized from these numbers."""
        w = self.make()
        w._scope = lambda: SimpleNamespace(
            state=lambda: SimpleNamespace(has_focus=True))
        w._optics = lambda: None
        ok, why = w._optical_gate()
        self.assertFalse(ok)
        self.assertIn("µm/px", why)
        self.assertIn("working distance", why)

    def test_a_margin_under_the_longest_bore_refuses(self):
        w = self.make()
        w._scope = lambda: SimpleNamespace(
            state=lambda: SimpleNamespace(has_focus=True))
        w._optics = lambda: object()
        w._longest_bore_mm = lambda: 0.200
        ok, why = w._optical_gate()
        self.assertFalse(ok)
        self.assertIn("glass", why.lower())

    def test_the_glass_focus_must_be_set_first(self):
        w = self.make()
        w._scope = lambda: SimpleNamespace(
            state=lambda: SimpleNamespace(has_focus=True))
        w._optics = lambda: object()
        ok, why = w._optical_gate()
        self.assertFalse(ok)
        self.assertIn("Set plate-bottom focus", why)

    def test_with_everything_ready_it_passes(self):
        w = self.make()
        w._scope = lambda: SimpleNamespace(
            state=lambda: SimpleNamespace(has_focus=True))
        w._optics = lambda: object()
        w._opt_f0 = F0
        self.assertTrue(w._optical_gate()[0], w._optical_gate()[1])


class TestTheBoundNarrows(_Base):
    """Only the first descent may trust the geometric guess."""

    def test_the_first_rung_uses_the_guess(self):
        w = self.make(plate_bottom=B_TRUE)
        self.assertAlmostEqual(w._plate_bottom_guess(), B_TRUE, places=9)

    def test_later_rungs_use_the_measurement(self):
        w = self.make(plate_bottom=B_TRUE + 0.500)     # a badly wrong guess
        w._opt_f0 = F0
        w._focus_up_sign = lambda: 1.0
        w._zdir = lambda: ZDIR
        w._opt_rungs = [RungMeasurement(
            margin_um=1000.0,
            needle_z_zref_mm=needle_target_zref(B_TRUE, 1000.0, ZDIR),
            focus_tip_um=F0 + 1000.0)]
        self.assertAlmostEqual(w._plate_bottom_guess(), B_TRUE, places=9)


class TestAcceptIsGated(_Base):
    def _ready(self, rungs):
        w = self.make()
        w._opt_f0 = F0
        w._focus_up_sign = lambda: 1.0
        w._zdir = lambda: ZDIR
        w._opt_rungs = rungs
        return w

    def _rung(self, x, err_um=0.0):
        return RungMeasurement(
            margin_um=x,
            needle_z_zref_mm=needle_target_zref(B_TRUE, x, ZDIR),
            focus_tip_um=F0 + x + err_um, focus_sigma_um=1.7)

    def test_disagreeing_margins_block_accept(self):
        w = self._ready([self._rung(1000.0), self._rung(500.0),
                         self._rung(200.0), self._rung(100.0, err_um=140.0)])
        rec = w._reconcile()
        self.assertFalse(rec.ok)
        self.assertIn("disagree", rec.refusal)

    def test_agreeing_margins_allow_accept(self):
        w = self._ready([self._rung(x) for x in DEFAULT_OFFSETS_UM])
        rec = w._reconcile()
        self.assertTrue(rec.ok, rec.refusal)
        self.assertAlmostEqual(rec.plate_bottom_zref_mm, B_TRUE, places=9)

    def test_nothing_measured_reconciles_to_none(self):
        w = self._ready([])
        self.assertIsNone(w._reconcile())

    def test_accept_writes_the_bottom_tagged_optical(self):
        seen = {}
        ctrl = SimpleNamespace(
            is_xy_connected=True, is_zp_connected=True,
            print_z_dir=lambda: ZDIR,
            print_height_to_zref=lambda h, *a, **k: B_TRUE + ZDIR * h,
            get_plate_bottom_z_source=lambda: "optical",
            set_print_floor_active=lambda on: None,
            z_height_of=lambda z: ZDIR * z,
            set_plate_bottom_z=lambda z, **k: seen.update(z=z, **k))
        w = self.make(ctrl=ctrl)
        w._opt_f0 = F0
        w._focus_up_sign = lambda: 1.0
        w._zdir = lambda: ZDIR
        w._opt_rungs = [self._rung(x) for x in DEFAULT_OFFSETS_UM]
        w._read_xy = lambda: (1000.0, 2000.0)
        w._write_focus_datum = lambda rec: None
        w._on_accept_optical()
        self.assertAlmostEqual(seen["z"], B_TRUE, places=9)
        self.assertEqual(seen["source"], "optical")
        self.assertAlmostEqual(w._host._plate_bottom_z, B_TRUE, places=9)


class TestTheLiveReadout(_Base):
    def test_it_tracks_the_running_maximum(self):
        w = self.make()
        w._note_live_focus(1000.0, 50.0)
        w._note_live_focus(1010.0, 90.0)
        w._note_live_focus(1020.0, 70.0)
        self.assertEqual(w._opt_live, (90.0, 1010.0))

    def test_it_says_it_is_not_a_fitted_peak(self):
        """A running max resolves to about half a jog step. Presenting it beside
        a sub-step fitted peak without saying so invites the operator to trust
        it equally."""
        w = self.make()
        w._note_live_focus(1000.0, 50.0)
        self.assertIn("not a fitted peak", w._s4_live.text())

    def test_override_without_a_live_sample_does_nothing(self):
        w = self.make()
        w._opt_pending = object()
        w._opt_live = None
        w._on_use_live_focus()                 # must not raise
        self.assertEqual(w._opt_rungs, [])

    def _pending(self, w, margin=1000.0, auto_um=None):
        w._opt_f0 = F0
        w._focus_up_sign = lambda: 1.0
        w._zdir = lambda: ZDIR
        w._opt_pending = RungMeasurement(
            margin_um=margin,
            needle_z_zref_mm=needle_target_zref(B_TRUE, margin, ZDIR),
            focus_tip_um=(F0 + margin if auto_um is None else auto_um))
        return w

    def test_declining_the_confirmation_records_nothing(self):
        """Ground truth is created ONLY by an explicit confirmation."""
        w = self._pending(self.make())
        w._write_focus_training = lambda *a, **k: None
        w._opt_live = (99.0, F0 + 990.0)
        w._on_use_live_focus()                 # setUp defaults question to No
        self.assertEqual(w._opt_rungs, [])

    def test_confirming_records_ground_truth_and_the_delta(self):
        w = self._pending(self.make(), auto_um=F0 + 1000.0)
        captured = []
        w._write_focus_training = lambda m, **k: captured.append((m, k))
        w._opt_live = (99.0, F0 + 992.0)
        self.answer_yes()
        w._on_use_live_focus()
        self.assertEqual(len(w._opt_rungs), 1)
        m, kw = captured[0]
        self.assertEqual(m.adopted_by, "operator")
        self.assertAlmostEqual(m.focus_tip_um, F0 + 992.0, places=6)
        self.assertTrue(kw["ground_truth"])
        self.assertAlmostEqual(kw["auto_focus_um"], F0 + 1000.0, places=6)

    def test_the_confirmation_shows_the_difference_being_chosen(self):
        w = self._pending(self.make(), auto_um=F0 + 1000.0)
        w._write_focus_training = lambda *a, **k: None
        w._opt_live = (99.0, F0 + 992.0)
        self.answer_yes()
        w._on_use_live_focus()
        text = " ".join(t for _n, t in self._boxes)
        self.assertIn("-8.0", text.replace("−", "-"))

    def test_an_overridden_rung_carries_no_sigma(self):
        """A running max is not a fit; giving it a sigma would let it be
        weighted as if it were one."""
        w = self._pending(self.make())
        w._write_focus_training = lambda *a, **k: None
        w._opt_live = (99.0, F0 + 990.0)
        self.answer_yes()
        w._on_use_live_focus()
        self.assertEqual(w._opt_rungs[0].focus_sigma_um, 0.0)


class TestTheTrainingCapture(_Base):
    def test_a_rung_is_adopted_and_captured_exactly_once(self):
        """The override path once wrote two captures for one decision."""
        w = self.make()
        calls = []
        w._write_focus_training = lambda m, **k: calls.append((m, k))
        w._report_optical_progress = lambda: None
        m = RungMeasurement(margin_um=1000.0, needle_z_zref_mm=20.0,
                            focus_tip_um=F0 + 1000.0)
        w._adopt_rung(m, ground_truth=True, auto_focus_um=2190.0)
        self.assertEqual(len(calls), 1)
        self.assertTrue(calls[0][1]["ground_truth"])
        self.assertEqual(calls[0][1]["auto_focus_um"], 2190.0)

    def test_an_automatic_adoption_is_not_ground_truth(self):
        w = self.make()
        calls = []
        w._write_focus_training = lambda m, **k: calls.append(k)
        w._report_optical_progress = lambda: None
        w._adopt_rung(RungMeasurement(margin_um=100.0, needle_z_zref_mm=20.0,
                                      focus_tip_um=F0))
        self.assertFalse(calls[0]["ground_truth"])
        self.assertIsNone(calls[0]["auto_focus_um"])

    def test_the_capture_carries_the_needle_type_and_the_focus_score(self):
        """D3 — both were dropped by the only writer, which is why
        reference_focus_score() returned None for every capture on disk."""
        import inspect
        for fn in (wiz.NeedleBoreWizard._write_focus_training,
                   wiz.NeedleBoreWizard._write_touchoff_capture):
            src = inspect.getsource(fn)
            self.assertIn("needle_type=", src, fn.__name__)
            self.assertIn("needle_bore_um=", src, fn.__name__)
            self.assertIn("focus_score=", src, fn.__name__)


class TestProvenance(unittest.TestCase):
    """A guess and a measurement must not look alike downstream.

    Checked by AST, not by text: an explanatory comment mentioning
    ``source="estimated"`` satisfies a substring search whether or not the CODE
    passes it. That is the same trap that let an earlier guard survive on an
    import line alone.
    """

    #: The controller sink, plus (v7.9.1) the calibration page's shared writer
    #: that now fronts it. `_zoff_push_plate_bottom_to_controller(z, source)`
    #: takes the provenance POSITIONALLY, so both shapes are collected — the
    #: guard follows the delegation instead of being defeated by it.
    _PLATE_BOTTOM_SINKS = (
        "set_plate_bottom_z", "_zoff_push_plate_bottom_to_controller")

    @classmethod
    def _source_kwargs(cls, fn):
        """Every provenance literal fn passes toward the plate-bottom datum."""
        import ast
        import inspect
        import textwrap
        tree = ast.parse(textwrap.dedent(inspect.getsource(fn)))
        out = []
        for node in ast.walk(tree):
            if not (isinstance(node, ast.Call)
                    and isinstance(node.func, ast.Attribute)
                    and node.func.attr in cls._PLATE_BOTTOM_SINKS):
                continue
            for kw in node.keywords:
                if kw.arg == "source" and isinstance(kw.value, ast.Constant):
                    out.append(kw.value.value)
            if node.func.attr == "_zoff_push_plate_bottom_to_controller":
                # (z_zref_mm, source) — provenance is the 2nd positional.
                if len(node.args) >= 2 and isinstance(node.args[1], ast.Constant):
                    out.append(node.args[1].value)
        return out

    def test_the_shared_writer_forwards_provenance_and_anchor(self):
        """The delegation the guard above now follows must itself be honest.

        Without this, moving the push behind a helper would let the helper drop
        ``source=``/``at_xy_um=`` while every caller still 'passes' them.
        """
        import ast
        import inspect
        import textwrap
        from gui.pages.calibration import CalibrationPage
        src = textwrap.dedent(inspect.getsource(
            CalibrationPage._zoff_push_plate_bottom_to_controller))
        keys = set()
        for node in ast.walk(ast.parse(src)):
            if isinstance(node, ast.Dict):
                keys |= {k.value for k in node.keys
                         if isinstance(k, ast.Constant)}
        self.assertIn("source", keys)
        self.assertIn("at_xy_um", keys)

    def test_the_needle_cam_estimate_tags_itself_estimated(self):
        """It is the ONLY path that produces an estimate. Pushing it untagged
        made the plate-level wizard's "plate bottom is only estimated" warning
        unreachable — it reads get_plate_bottom_z_source()."""
        from gui.pages.calibration import CalibrationPage
        self.assertIn(
            "estimated",
            self._source_kwargs(CalibrationPage._zoff_estimate_from_needle_cam))

    def test_the_optical_measurement_tags_itself_optical(self):
        self.assertIn(
            "optical",
            self._source_kwargs(wiz.NeedleBoreWizard._on_accept_optical))

    def test_the_optical_measurement_records_where_it_was_taken(self):
        """Without the anchor XY the scalar cannot anchor a tilt plane."""
        import ast
        import inspect
        import textwrap
        tree = ast.parse(textwrap.dedent(
            inspect.getsource(wiz.NeedleBoreWizard._on_accept_optical)))
        anchored = any(
            kw.arg == "at_xy_um"
            for node in ast.walk(tree)
            if (isinstance(node, ast.Call)
                and isinstance(node.func, ast.Attribute)
                and node.func.attr == "set_plate_bottom_z")
            for kw in node.keywords)
        self.assertTrue(anchored)

    def test_the_warning_that_reads_the_source_still_exists(self):
        """Guards the other half: tagging is pointless if nothing reads it."""
        import inspect
        import gui.widgets.plate_level_wizard as plw
        src = inspect.getsource(plw.PlateLevelWizard)
        self.assertIn("get_plate_bottom_z_source", src)
        self.assertIn("estimated", src)


class TestTeardown(_Base):
    def test_hiding_the_page_tears_down_the_optical_worker(self):
        """It holds the microscope lease and moves the focus."""
        import inspect
        src = inspect.getsource(wiz.NeedleBoreWizard.hideEvent)
        self.assertIn("_teardown_rung_worker", src)

    def test_teardown_disconnects_before_stopping(self):
        src = __import__("inspect").getsource(
            wiz.NeedleBoreWizard._teardown_rung_worker)
        self.assertLess(src.index("disconnect"), src.index("w.stop()"))


if __name__ == "__main__":
    unittest.main()

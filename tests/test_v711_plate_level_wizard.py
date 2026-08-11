"""
Wizard gates and the accept path.

Every gate refusal here is a sentence an operator can act on. They are driven as
DATA so that adding a gate requires adding a row — a gate with no test is a gate
that can silently stop refusing.

The accept path is pinned hard: a cancelled or failed run must leave the
previously-installed plane byte-identical, because "the tilt got worse during
calibration" is the one outcome a calibration must never produce.
"""

import os
import sys
import unittest
from types import SimpleNamespace

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication            # noqa: E402

import gui.widgets.plate_level_wizard as wizmod       # noqa: E402
from gui.widgets.plate_level_wizard import (          # noqa: E402
    PlateLevelWizard, SiteSpec, SurveyState, STEP_ORDER, STEP_PREFLIGHT,
    STEP_OPTICS, STEP_SITES, STEP_SURVEY, STEP_VERIFY)
from SupportClasses.PlateLeveling import SiteMeasurement    # noqa: E402


class _FakeMB:
    """An unpatched QMessageBox blocks forever offscreen."""
    warned = []
    infos = []

    @classmethod
    def warning(cls, *a, **k):
        cls.warned.append(a[2] if len(a) > 2 else "")

    @classmethod
    def information(cls, *a, **k):
        cls.infos.append(a[2] if len(a) > 2 else "")


class _Ctrl:
    def __init__(self, **kw):
        self.is_xy_connected = kw.get("xy", True)
        self.is_zp_connected = kw.get("zp", True)
        self._bottom = kw.get("bottom", 0.090)
        self._anchor = kw.get("anchor", (0.0, 0.0))
        self._stale = kw.get("stale", False)
        self._source = kw.get("source", "taught")
        self.zero_position = {"Z": 0.0}
        self.installed = []
        self.accept_ok = kw.get("accept_ok", True)

    def get_plate_bottom_z(self):
        return self._bottom

    def get_plate_bottom_anchor_xy_um(self):
        return self._anchor

    def get_plate_bottom_z_source(self):
        return self._source

    def plate_bottom_z_is_stale(self):
        return self._stale

    def plate_flip_180(self):
        return True

    def z_up_sign(self):
        return 1.0

    def print_z_dir(self):
        return 1.0

    def would_accept_plate_z_plane(self, plane):
        return (self.accept_ok, "" if self.accept_ok else "controller says no")

    def set_plate_z_plane(self, plane):
        if self.accept_ok:
            self.installed.append(plane)
        return (self.accept_ok, "" if self.accept_ok else "controller says no")


class _State:
    connected = True
    has_focus = True
    objective_count = 3
    objective_position = 1
    focus_min_um = 0.0
    focus_max_um = 10000.0
    mounted_objectives = ()


class _Scope:
    def __init__(self, state=None):
        self._state = state or _State()

    def state(self):
        return self._state


WELLS = {}
for _r, _row in enumerate("ABCD"):
    for _c in range(1, 7):
        WELLS[f"{_row}{_c}"] = (_c * 19300.0, _r * 19300.0)


def host(**kw):
    return SimpleNamespace(
        controller=kw.get("ctrl", _Ctrl()),
        camera_manager=kw.get("mgr", SimpleNamespace(
            cameras=lambda: [], camera_identity=lambda i: ("CAM", "path"))),
        _safe_z=kw.get("safe_z", 44.0),
        _calibrated_positions=kw.get("wells", dict(WELLS)),
        _hardware_config=None,
        _ploc_microscope_cam_idx=lambda: kw.get("cam_idx", 0),
        _ploc_plate_key=lambda: "plate-24")


def _rung(pos=1, name="10x", mag=10.0, calibrated=True):
    from SupportClasses.ObjectiveOptics import ObjectiveOptics
    return SimpleNamespace(
        turret_position=pos, objective_name=name, magnification=mag,
        um_per_px=1.28, calibrated=calibrated, why_not="", product_code="MRH",
        numerical_aperture=0.30, working_distance_mm=16.0,
        describe=lambda: name, selectable=True,
        optics=ObjectiveOptics(label=name, numerical_aperture=0.30,
                               working_distance_mm=16.0,
                               um_per_px_sample=1.28, frame_wh=(2600, 2048)))


class _WizardCase(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication(sys.argv)

    def setUp(self):
        _FakeMB.warned = []
        _FakeMB.infos = []
        self._real_mb = wizmod.QMessageBox
        wizmod.QMessageBox = _FakeMB
        self.addCleanup(lambda: setattr(wizmod, "QMessageBox", self._real_mb))

        self._real_limits = PlateLevelWizard._focus_soft_limits
        PlateLevelWizard._focus_soft_limits = lambda self: (200.0, 9000.0)
        self.addCleanup(lambda: setattr(PlateLevelWizard, "_focus_soft_limits",
                                        self._real_limits))
        self._real_scope = PlateLevelWizard._scope
        PlateLevelWizard._scope = property(lambda self: _Scope())
        self.addCleanup(lambda: setattr(PlateLevelWizard, "_scope",
                                        self._real_scope))

    def make(self, ladder=True, **kw):
        w = PlateLevelWizard(host(**kw))
        self.addCleanup(w.deleteLater)
        if ladder:
            # Gates are CUMULATIVE — a later step re-runs every earlier one — so
            # a site test would otherwise be stopped by the objective gate and
            # never reach the assertion it is about.
            w._resolve_ladder = lambda: [_rung()]
            w._ladder_positions = [1]
        return w


class TestPreflightGates(_WizardCase):
    """One row per refusal. A gate with no row can stop refusing unnoticed."""

    CASES = [
        ("xy disconnected", dict(ctrl=_Ctrl(xy=False)), "Connect the XY stage"),
        ("zp disconnected", dict(ctrl=_Ctrl(zp=False)), "Connect the Z board"),
        ("no safe z", dict(safe_z=None), "Fast Move (Safe) Z"),
        ("no microscope camera", dict(cam_idx=None), "Microscope role"),
        ("plate bottom untaught", dict(ctrl=_Ctrl(bottom=None)),
         "Teach the Plate Bottom Z"),
        ("anchor xy missing", dict(ctrl=_Ctrl(anchor=None)),
         "no recorded XY"),
        ("zero z moved", dict(ctrl=_Ctrl(stale=True)), "Set Z Zero has run"),
        ("no well map", dict(wells={}), "Finish Plate Location"),
    ]

    def test_each_refusal_names_what_to_do(self):
        for name, kw, expect in self.CASES:
            with self.subTest(name):
                w = self.make(**kw)
                ok, why = w.gate(STEP_PREFLIGHT)
                self.assertFalse(ok, f"{name} should refuse")
                self.assertIn(expect, why)

    def test_a_ready_rig_passes_preflight(self):
        w = self.make()
        ok, why = w.gate(STEP_PREFLIGHT)
        self.assertTrue(ok, why)

    def test_missing_focus_soft_limits_refuse_with_the_collision_reason(self):
        PlateLevelWizard._focus_soft_limits = lambda self: (None, None)
        w = self.make()
        ok, why = w.gate(STEP_PREFLIGHT)
        self.assertFalse(ok)
        self.assertIn("front lens into the plate", why)

    def test_no_motorised_focus_refuses(self):
        st = _State()
        st.has_focus = False
        PlateLevelWizard._scope = property(lambda self: _Scope(st))
        w = self.make()
        ok, why = w.gate(STEP_PREFLIGHT)
        self.assertFalse(ok)
        self.assertIn("no motorised focus", why)

    def test_disconnected_body_refuses(self):
        st = _State()
        st.connected = False
        PlateLevelWizard._scope = property(lambda self: _Scope(st))
        w = self.make()
        ok, why = w.gate(STEP_PREFLIGHT)
        self.assertFalse(ok)
        self.assertIn("Connect the microscope body", why)


class TestSiteGates(_WizardCase):
    def test_too_few_sites_refuses(self):
        w = self.make()
        w._sites = [SiteSpec(label="A1", x_um=0.0, y_um=0.0)]
        ok, why = w.gate(STEP_SITES)
        self.assertFalse(ok)

    def test_collinear_sites_refuse_before_any_motion(self):
        """The whole point of gating here: the operator learns this before a
        twenty-minute survey, not after it."""
        w = self.make()
        w._sites = [SiteSpec(label=f"S{i}", x_um=i * 19300.0, y_um=0.0)
                    for i in range(4)]
        ok, why = w.gate(STEP_SITES)
        self.assertFalse(ok)
        self.assertIn("collinear", why)

    def test_proposed_sites_pass_their_own_gate(self):
        w = self.make()
        w._propose()
        ok, why = w.gate(STEP_SITES)
        self.assertTrue(ok, why)

    def test_the_first_proposed_site_is_the_anchor(self):
        w = self.make(ctrl=_Ctrl(anchor=WELLS["C3"]))
        w._propose()
        self.assertEqual(w._sites[0].label, "C3")


class TestStepStates(_WizardCase):
    def test_all_steps_report_a_known_state(self):
        w = self.make()
        for k in STEP_ORDER:
            self.assertIn(w.step_state(k), ("ok", "warn", "todo"))

    def test_survey_is_todo_until_every_site_is_measured(self):
        w = self.make()
        w._propose()
        self.assertEqual(w.step_state(STEP_SURVEY), "todo")

    def test_partial_survey_reports_the_count(self):
        w = self.make()
        w._propose()
        st = SurveyState(sites=list(w._sites))
        st.measurements["X"] = SiteMeasurement(label="X", x_stage_um=0.0,
                                               y_stage_um=0.0, focus_um=1.0)
        w._state = st
        ok, why = w.gate(STEP_SURVEY)
        self.assertFalse(ok)
        self.assertIn("of", why)


class TestAcceptPath(_WizardCase):
    def _completed(self, w, err_um=0.0):
        pts = [("A1", 0.0, 0.0), ("A6", 60000.0, 0.0),
               ("D1", 0.0, 40000.0), ("D6", 60000.0, 40000.0)]
        w._sites = [SiteSpec(label=n, x_um=x, y_um=y) for n, x, y in pts]
        st = SurveyState(sites=list(w._sites))
        for i, (n, x, y) in enumerate(pts):
            f = 1000.0 + 0.0199 * y / 1000.0 * 1000.0
            if i == 2:
                f += err_um
            st.measurements[n] = SiteMeasurement(
                label=n, x_stage_um=x, y_stage_um=y, focus_um=f,
                focus_sigma_um=1.7, fwhm_um=20.0, prominence=2.0)
        st.anchor_focus_um = st.measurements["A1"].focus_um
        st.anchor_focus_closing_um = st.anchor_focus_um
        w._state = st
        w._solve_and_show()
        return w

    def test_a_clean_survey_can_be_accepted_and_installs_once(self):
        w = self._completed(self.make())
        self.assertTrue(w._btn_accept.isEnabled(), w._verify_text.toPlainText())
        w._accept()
        self.assertEqual(len(w._host.controller.installed), 1)

    def test_a_corrupted_site_blocks_accept(self):
        w = self._completed(self.make(), err_um=140.0)
        self.assertFalse(w._btn_accept.isEnabled())
        self.assertIn("BLOCKED", w._verify_text.toPlainText())

    def test_a_controller_refusal_blocks_accept_and_is_quoted(self):
        w = self._completed(self.make(ctrl=_Ctrl(accept_ok=False)))
        self.assertFalse(w._btn_accept.isEnabled())
        self.assertIn("controller says no", w._verify_text.toPlainText())

    def test_anchor_drift_blocks_accept(self):
        """If the first site's focus moved during the run, the measurements were
        not all taken against the same reference."""
        w = self._completed(self.make())
        w._state.anchor_focus_closing_um = w._state.anchor_focus_um + 40.0
        w._render_verify()
        self.assertFalse(w._btn_accept.isEnabled())
        self.assertIn("drifted", w._verify_text.toPlainText())

    def test_nothing_is_installed_before_accept(self):
        w = self._completed(self.make())
        self.assertEqual(w._host.controller.installed, [])

    def test_a_failed_run_installs_nothing(self):
        w = self.make()
        w._on_failed("the microscope refused")
        self.assertEqual(w._host.controller.installed, [])
        self.assertIsNone(w._solution)

    def test_the_verify_text_states_the_consequence_first(self):
        w = self._completed(self.make())
        text = w._verify_text.toPlainText()
        self.assertIn("Plate bottom varies", text.split("\n")[0])

    def test_the_anchor_row_is_labelled_exact_by_construction(self):
        """Its zero residual is a definition, not evidence, and must not be
        mistaken for a measurement that agreed."""
        w = self._completed(self.make())
        self.assertIn("exact by construction", w._verify_text.toPlainText())


class TestLifecycle(_WizardCase):
    def test_hiding_the_page_tears_the_worker_down(self):
        w = self.make()
        calls = []
        w._teardown_worker = lambda: calls.append(1)
        w.show()                 # Qt only delivers hideEvent to a shown widget
        self.app.processEvents()
        w.hide()
        self.app.processEvents()
        self.assertTrue(calls)

    def test_teardown_with_no_worker_is_safe(self):
        w = self.make()
        w._teardown_worker()
        w._teardown_worker()


if __name__ == "__main__":
    unittest.main()

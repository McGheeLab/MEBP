"""v7.17 — step 2's direct setters, Max Z retirement, and the estimate/clamp split.

Three operator-requested changes, each with a distinct failure mode:

1. Every reference the wizard owns has its own setter button on step 2, so no
   reference sends the operator off to the Advanced group.
2. Max Z is retired. It had NO behavioural consumer anywhere — every use was a
   readout — and it was one more zero-ref height to re-teach after a Z re-datum
   (the v7.9.1 stale-reference incident had `max_z` among the three that came
   back 24-49 mm below the machine's hard bottom).
3. 🔴 The safety one: an ESTIMATED plate bottom must never arm the print-floor
   clamp. Only a MEASURED bottom (contact touch-off / optical) may.
"""

import os
import unittest
from types import SimpleNamespace

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication

import gui.widgets.needle_bore_wizard as wizmod
from SupportClasses.StageController import StageController
from gui.pages.calibration import CalibrationPage
from gui.widgets.needle_bore_wizard import (
    STEP_BORES, STEP_TOUCHOFF, STEP_Z_REFS, NeedleBoreWizard,
)
from gui.widgets.xz_side_view import XZSideView

_app = QApplication.instance() or QApplication([])


class _FakeMB:
    """An unpatched QMessageBox blocks forever offscreen."""
    class StandardButton:
        Yes = 1
        No = 0

    @classmethod
    def warning(cls, *a, **k):
        return None

    @classmethod
    def question(cls, *a, **k):
        return cls.StandardButton.Yes

    @classmethod
    def information(cls, *a, **k):
        return None


# ── 1. step-2 direct setters ────────────────────────────────────────

class _Host:
    """Stand-in host exposing the setter contract the wizard calls."""

    def __init__(self, *, z=12.345):
        self._z = z
        self._safe_z = None
        self._top_z = None
        self._replace_z = None
        self._plate_bottom_z = None
        self._needle_origin_um = (1.0, 2.0)
        self._needle_loc_xy_um = (100_000.0, 50_000.0)
        self._hardware_config = SimpleNamespace(needle=None)
        self._camera_manager = None
        self.controller = SimpleNamespace(
            is_xy_connected=True,
            z_up_sign=lambda: 1.0,
            print_floor_datum_zref=lambda: None,
        )
        self.applied = []

    # the four setters step 2 drives
    def _zoff_set_safe_z(self):
        self._safe_z = self._z

    def _zoff_set_top_z(self):
        self._top_z = self._z

    def _zoff_set_replace_z(self):
        self._replace_z = self._z

    def _zoff_apply_plate_bottom_z(self, z, source="estimated"):
        self._plate_bottom_z = z
        self.applied.append((z, source))

    def _ploc_microscope_cam_idx(self):
        return None


class TestStep2HasASetterForEveryReference(unittest.TestCase):

    def _wiz(self, host=None):
        self._orig_mb = wizmod.QMessageBox
        wizmod.QMessageBox = _FakeMB
        self.addCleanup(lambda: setattr(wizmod, "QMessageBox", self._orig_mb))
        w = NeedleBoreWizard(host or _Host())
        self.addCleanup(w.deleteLater)
        return w

    def test_every_reference_has_its_own_button(self):
        w = self._wiz()
        for attr in ("_s2_set_top", "_s2_set_safe", "_s2_set_replace",
                     "_s2_apply_bottom"):
            self.assertTrue(hasattr(w, attr), f"step 2 is missing {attr}")

    def test_replace_z_setter_captures_the_current_z(self):
        """The one that was missing — it used to live only in the Advanced
        group, so a wizard-only operator could not set it at all."""
        host = _Host(z=41.5)
        w = self._wiz(host)
        w._on_set_replace()
        self.assertEqual(host._replace_z, 41.5)
        self.assertIsNone(w._refusal_text)

    def test_safe_z_setter_still_captures(self):
        host = _Host(z=44.0)
        w = self._wiz(host)
        w._on_set_safe()
        self.assertEqual(host._safe_z, 44.0)

    def test_a_setter_refuses_when_the_z_board_cannot_be_read(self):
        """The host setters return None either way, so the wizard must verify
        the ATTRIBUTE landed — otherwise a disconnected board reads as success."""
        host = _Host()
        host._zoff_set_replace_z = lambda: None      # never assigns
        w = self._wiz(host)
        w._on_set_replace()
        self.assertIsNone(host._replace_z)
        self.assertIn("Z board", w._refusal_text or "")

    def test_a_missing_host_setter_refuses_by_name(self):
        host = _Host()
        host._zoff_set_replace_z = None      # an older host without it
        w = self._wiz(host)
        w._on_set_replace()
        self.assertIn("replace Z", w._refusal_text or "")

    def test_the_step2_readout_names_all_four(self):
        host = _Host()
        host._safe_z, host._top_z = 44.0, 30.0
        host._replace_z, host._plate_bottom_z = 41.5, 29.0
        w = self._wiz(host)
        w.go_to_step(STEP_Z_REFS)
        txt = w._s2_state.text()
        for name in ("Plate top", "Plate bottom", "Fast-move", "Replace"):
            self.assertIn(name, txt)


# ── 2. Max Z is gone ────────────────────────────────────────────────

class TestMaxZIsRetired(unittest.TestCase):
    """It had no consumer: every reference was a readout. Removing it also
    removes one more stale zero-ref height to re-teach after a Z re-datum."""

    def test_the_page_setter_is_gone(self):
        self.assertFalse(hasattr(CalibrationPage, "_zoff_set_max_z"))

    def test_max_z_is_not_a_z_reference_key(self):
        self.assertNotIn("max_z", CalibrationPage._ALL_Z_REF_KEYS)

    def test_the_xz_side_view_has_no_max_badge(self):
        keys = [k for k, _lbl, _c in XZSideView._Z_REF_META]
        self.assertNotIn("max_z", keys)
        # The others are untouched.
        for k in ("replace_z", "fast_move_z", "plate_top_z", "plate_bottom_z"):
            self.assertIn(k, keys)

    def test_the_xz_view_survives_a_refs_dict_with_no_max_z(self):
        """The real regression risk: a consumer that assumed the key exists."""
        v = XZSideView()
        self.addCleanup(v.deleteLater)
        v.set_z_references({"replace_z": 41.5, "fast_move_z": 44.0,
                            "plate_top_z": 30.0, "plate_bottom_z": 29.0})
        self.assertNotIn("max_z", v._z_refs)

    def test_a_legacy_saved_max_z_is_simply_ignored_not_crashed_on(self):
        v = XZSideView()
        self.addCleanup(v.deleteLater)
        v.set_z_references({"max_z": 16.4, "fast_move_z": 44.0})
        v.set_visible_z_references({"max_z", "fast_move_z"})
        self.assertEqual(v._z_refs.get("fast_move_z"), 44.0)

    def test_no_production_source_still_references_it(self):
        """A rename sweep that misses a site leaves a dead readout behind."""
        import pathlib
        import re
        root = pathlib.Path(__file__).resolve().parent.parent
        pat = re.compile(r'"max_z"|\'max_z\'|\b_max_z\b')
        hits = []
        for sub in ("gui", "SupportClasses"):
            for p in (root / sub).rglob("*.py"):
                try:
                    txt = p.read_text(encoding="utf-8", errors="replace")
                except OSError:                        # pragma: no cover
                    continue
                for i, line in enumerate(txt.splitlines(), 1):
                    if pat.search(line):
                        hits.append(f"{p.relative_to(root)}:{i}: {line.strip()}")
        self.assertEqual(hits, [], "stale max_z references:\n" + "\n".join(hits))


# ── 3. 🔴 an ESTIMATED plate bottom never clamps ────────────────────

class TestEstimatedPlateBottomNeverClamps(unittest.TestCase):
    """The clamp datum must be MEASURED.

    An estimate is unsafe in both directions: too high and it blocks the very
    touch-off that would measure the truth; too low and it is false protection.
    """

    def _ctrl(self, *, bottom, source, zdir=1.0):
        c = StageController.__new__(StageController)
        c.zero_position = {"Z": 0.0}
        c._plate_bottom_z_zref = bottom
        c._plate_bottom_z_source = source
        c._print_floor_active = True
        c._plate_top_z_zref = None
        c._z_up_sign = zdir
        return c

    def test_an_estimate_is_not_a_floor_datum(self):
        c = self._ctrl(bottom=10.0, source="estimated")
        self.assertIsNone(c.print_floor_datum_zref())

    def test_a_taught_bottom_is_a_floor_datum(self):
        c = self._ctrl(bottom=10.0, source="taught")
        self.assertEqual(c.print_floor_datum_zref(), 10.0)

    def test_an_untagged_legacy_bottom_still_clamps(self):
        """Only an explicit 'estimated' disarms — treating None as
        non-clamping would silently drop the floor on every pre-v7.17 path."""
        c = self._ctrl(bottom=10.0, source=None)
        self.assertEqual(c.print_floor_datum_zref(), 10.0)

    def test_a_restored_taught_bottom_still_clamps(self):
        c = self._ctrl(bottom=10.0, source="restored")
        self.assertEqual(c.print_floor_datum_zref(), 10.0)

    def test_the_clamp_passes_a_deep_z_when_the_bottom_is_only_estimated(self):
        """The behaviour, not just the helper: with zdir=+1 a Z of 5.0 is deeper
        than a bottom of 10.0, so a TAUGHT bottom clamps it to 10.0 and an
        ESTIMATE must let it through."""
        taught = self._ctrl(bottom=10.0, source="taught")
        self.assertEqual(taught._apply_print_floor_raw(5.0), 10.0)
        est = self._ctrl(bottom=10.0, source="estimated")
        self.assertEqual(est._apply_print_floor_raw(5.0), 5.0)

    def test_the_estimate_blocking_the_touch_off_is_the_motivating_case(self):
        """⚠ The direction that matters: an estimate reading HIGHER than the real
        glass would clamp the descent short, so the touch-off could never reach
        the surface it exists to measure. Under the fix the descent proceeds."""
        est = self._ctrl(bottom=10.0, source="estimated")
        real_glass = 9.2                       # 0.8 mm lower than the guess
        self.assertEqual(est._apply_print_floor_raw(real_glass), real_glass)

    def test_an_unarmed_floor_is_still_a_no_op_whatever_the_source(self):
        c = self._ctrl(bottom=10.0, source="taught")
        c._print_floor_active = False
        self.assertEqual(c._apply_print_floor_raw(5.0), 5.0)

    def test_no_bottom_at_all_is_no_datum(self):
        self.assertIsNone(
            self._ctrl(bottom=None, source="taught").print_floor_datum_zref())

    def test_the_estimate_is_still_available_as_a_reference(self):
        """It must remain readable — print heights, survey clearances and the
        readouts all want a best guess; only the CLAMP insists on a measurement."""
        c = self._ctrl(bottom=10.0, source="estimated")
        self.assertEqual(c.get_plate_bottom_z(), 10.0)
        self.assertEqual(c.get_plate_bottom_z_source(), "estimated")

    def test_every_MEASURED_source_still_clamps(self):
        """⚠ The other half of the rule, and the one that would be a real safety
        regression: narrowing what clamps must not disarm the floor for a
        measurement. These are the tags the production writers actually use —
        'taught' (contact touch-off), 'optical' (the step-4 measurement),
        'restored' (a saved measured value re-pushed on load)."""
        for src in ("taught", "optical", "restored"):
            with self.subTest(source=src):
                c = self._ctrl(bottom=10.0, source=src)
                self.assertEqual(c.print_floor_datum_zref(), 10.0)
                self.assertEqual(c._apply_print_floor_raw(5.0), 10.0,
                                 f"a {src} bottom must still clamp")

    def test_only_estimated_is_excluded(self):
        """Pin the list itself: it must contain exactly the one guess tag."""
        self.assertEqual(
            tuple(StageController.NON_CLAMPING_PLATE_BOTTOM_SOURCES),
            ("estimated",))

    def test_the_rule_lives_in_exactly_one_place(self):
        """Two copies of a safety rule agree until one is edited."""
        import ast
        import inspect
        src = inspect.getsource(StageController._apply_print_floor_raw)
        tree = ast.parse(src.lstrip())
        calls = [n.func.attr for n in ast.walk(tree)
                 if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)]
        self.assertIn("print_floor_datum_zref", calls,
                      "the clamp must resolve its datum through the one helper")


class TestTheWizardDisclosesAnInactiveClamp(unittest.TestCase):
    """An estimate LOOKS taught (the readout shows a number) but does not clamp,
    so it is the case an operator is most likely to over-trust."""

    def _wiz(self, *, bottom, floor_datum):
        self._orig_mb = wizmod.QMessageBox
        wizmod.QMessageBox = _FakeMB
        self.addCleanup(lambda: setattr(wizmod, "QMessageBox", self._orig_mb))
        host = _Host()
        host._plate_bottom_z = bottom
        host.controller.print_floor_datum_zref = lambda: floor_datum
        w = NeedleBoreWizard(host)
        self.addCleanup(w.deleteLater)
        return w

    def test_an_estimated_bottom_is_called_out_as_an_estimate(self):
        w = self._wiz(bottom=10.0, floor_datum=None)
        msg = w._floor_advisory()
        self.assertIn("ESTIMATE", msg)
        self.assertIn("inactive", msg)

    def test_no_bottom_reports_the_other_case(self):
        w = self._wiz(bottom=None, floor_datum=None)
        msg = w._floor_advisory()
        self.assertIn("No plate bottom", msg)
        self.assertNotIn("ESTIMATE", msg)

    def test_a_measured_bottom_is_silent(self):
        w = self._wiz(bottom=10.0, floor_datum=10.0)
        self.assertEqual(w._floor_advisory(), "")

    def test_an_older_controller_without_the_rule_is_silent(self):
        """It clamps on any datum — the pre-v7.17 behaviour, not a warning."""
        w = self._wiz(bottom=10.0, floor_datum=None)
        del w._host.controller.print_floor_datum_zref
        self.assertEqual(w._floor_advisory(), "")


class TestTheApplyButtonTagsItsValueAsEstimated(unittest.TestCase):
    """The tag is what keeps the clamp disarmed, and it must survive app.py's
    untagged re-push (set_plate_bottom_z only overwrites source when non-None)."""

    def test_step2_applies_with_the_estimated_tag(self):
        self._orig_mb = wizmod.QMessageBox
        wizmod.QMessageBox = _FakeMB
        self.addCleanup(lambda: setattr(wizmod, "QMessageBox", self._orig_mb))
        host = _Host()
        host._top_z = 30.0
        w = NeedleBoreWizard(host)
        self.addCleanup(w.deleteLater)
        w._s2_bottom_offset.setValue(1.0)
        w._on_apply_bottom()
        self.assertEqual(len(host.applied), 1)
        _z, source = host.applied[0]
        self.assertEqual(source, "estimated")

    def test_an_untagged_repush_cannot_promote_an_estimate_to_a_floor(self):
        c = StageController.__new__(StageController)
        c.zero_position = {"Z": 0.0}
        c._plate_bottom_z_zref = None
        c._plate_bottom_z_source = None
        c._plate_bottom_anchor_xy_um = None
        c._print_floor_active = True
        c.set_plate_bottom_z(10.0, source="estimated")
        c.set_plate_bottom_z(10.0)                  # app.py's re-push, untagged
        self.assertEqual(c.get_plate_bottom_z_source(), "estimated")
        self.assertIsNone(c.print_floor_datum_zref())


if __name__ == "__main__":
    unittest.main()

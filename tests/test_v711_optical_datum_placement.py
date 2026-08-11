"""
WHERE the optical Z datum may be captured — and where it must not be.

The datum pairs a microscope focus reading with a needle Z. It is only
meaningful when both are at the SAME physical plane, which happens at exactly
one moment in the calibration: the plate touch-off, with the microscope focused
on the plate bottom and the needle tip touching it.

It specifically cannot come from the needle-centring step. The needle side
cameras are mounted far above the focal plane, so when the needle is centred in
their crosshairs its tip is nowhere the objective can focus — the focus axis is
then reading an unrelated position, and pairing it with that needle Z would
produce a datum wrong by however far apart the two planes are. Every downstream
consumer treats the datum as ground truth for converting focus to needle Z, so
that error would propagate into the plate-bottom height everywhere.

Absent is recoverable. Wrong is not.
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication            # noqa: E402

import gui.widgets.needle_bore_wizard as wiz          # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class _Box:
    def __init__(self, checked):
        self._c = checked

    def isChecked(self):
        return self._c


class _Stub:
    """Only the attributes the datum gate reads."""

    def __init__(self, checked=True, focus=1190.0):
        self._s4_focus_ok = _Box(checked)
        self._focus = focus

    def _microscope_focus_um(self):
        return self._focus


class TestTheGate(unittest.TestCase):
    def test_confirmed_focus_records_the_reading(self):
        got = wiz.NeedleBoreWizard._optical_datum_focus_um(_Stub(True, 1190.0))
        self.assertEqual(got, 1190.0)

    def test_unconfirmed_focus_records_nothing(self):
        """Not a fallback to 'whatever the axis reads' — an explicit absence."""
        got = wiz.NeedleBoreWizard._optical_datum_focus_um(_Stub(False, 1190.0))
        self.assertIsNone(got)

    def test_no_motorised_focus_records_nothing_even_when_confirmed(self):
        got = wiz.NeedleBoreWizard._optical_datum_focus_um(_Stub(True, None))
        self.assertIsNone(got)

    def test_a_missing_checkbox_does_not_crash_a_partial_widget(self):
        class _Bare:
            def _microscope_focus_um(self):
                return 900.0
        self.assertEqual(
            wiz.NeedleBoreWizard._optical_datum_focus_um(_Bare()), 900.0)


class TestTheCaptureUsesTheGate(unittest.TestCase):
    def test_the_store_write_goes_through_the_gate_not_the_raw_reading(self):
        """Pins the wiring: if the capture ever reads the axis directly again,
        an unfocused touch-off silently stores a meaningless datum."""
        import inspect
        src = inspect.getsource(wiz.NeedleBoreWizard._write_touchoff_capture)
        self.assertIn("microscope_focus_um=self._optical_datum_focus_um()", src)
        self.assertNotIn("microscope_focus_um=self._microscope_focus_um()", src)


class TestTheNeedleCentringPathNeverCapturesFocus(unittest.TestCase):
    """The needle side cameras sit far above the focal plane."""

    def test_every_datum_writer_is_a_justified_one(self):
        """The datum may only be written where the focal plane and the needle
        tip are provably at the SAME plane. There are exactly two such moments,
        both in the bore wizard's step 4:

        * the contact touch-off, where the operator asserts it (and must tick
          the confirmation, so an unfocused touch-off records absence); and
        * the optical measurement, where the focus sweep FOUND the tip, so the
          coincidence is measured rather than asserted.

        Any third site — and in particular any site in ``calibration.py``, which
        owns the needle-centring step whose side cameras sit far above the focal
        plane — is the bug this test exists to catch.
        """
        writers = []
        for root, dirs, files in os.walk(REPO):
            dirs[:] = [d for d in dirs
                       if d not in (".git", "__pycache__", "build", "dist",
                                    ".venv", "venv", "DLLs", "tests")]
            for fn in files:
                if not fn.endswith(".py"):
                    continue
                path = os.path.join(root, fn)
                try:
                    with open(path, encoding="utf-8") as f:
                        src = f.read()
                except (OSError, UnicodeDecodeError):
                    continue
                for i, line in enumerate(src.splitlines(), 1):
                    if "microscope_focus_um=" in line and "def " not in line:
                        writers.append(
                            (os.path.relpath(path, REPO).replace("\\", "/"),
                             i, line.strip()))

        for rel, line, text in writers:
            self.assertEqual(
                rel, "gui/widgets/needle_bore_wizard.py",
                f"{rel}:{line} writes the optical Z datum. Only the bore "
                f"wizard's step 4 may: {text}")

        sources = {t for _r, _l, t in writers}
        self.assertEqual(len(writers), 2, writers)
        self.assertIn("microscope_focus_um=self._optical_datum_focus_um(),",
                      sources)                       # contact, operator-gated
        self.assertIn("microscope_focus_um=m.focus_tip_um,",
                      sources)                       # optical, measured

    def test_the_optical_writer_pairs_the_measured_peak_with_that_needle_z(self):
        """The optical capture's validity rests on both numbers describing the
        same instant: the focus at which the tip was found sharp, and the needle
        Z it was parked at while that sweep ran (it does not move mid-sweep)."""
        import inspect
        src = inspect.getsource(wiz.NeedleBoreWizard._write_focus_training)
        self.assertIn("microscope_focus_um=m.focus_tip_um", src)
        self.assertIn("needle_z_user_mm=self._read_z_user()", src)

    def test_the_needle_centre_recorder_touches_no_focus_axis(self):
        """`_needle_loc_record_origin_here` records the needle-cam Z FIDUCIAL —
        a needle height — and must never reach for the microscope focus."""
        import inspect
        from gui.pages.calibration import CalibrationPage
        src = inspect.getsource(
            CalibrationPage._needle_loc_record_origin_here)
        self.assertNotIn("focus", src.lower())

    def test_center_and_save_touches_no_focus_axis(self):
        import inspect
        from gui.pages.calibration import CalibrationPage
        src = inspect.getsource(CalibrationPage._needle_loc_center_and_save)
        self.assertNotIn("microscope_focus", src)


class TestStep4SaysWhatHappened(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication(sys.argv)

    def test_the_checkbox_exists_and_defaults_to_the_documented_gesture(self):
        """Step 4's own instruction is to focus on the plate bottom first, so
        the default matches it — but it is overridable, and unticking records
        an absence rather than a wrong number."""
        from types import SimpleNamespace
        w = wiz.NeedleBoreWizard.__new__(wiz.NeedleBoreWizard)
        w._host = SimpleNamespace()
        # _build_step4 only needs the widget machinery, not a live host.
        import PySide6.QtWidgets as qtw
        w.__class__.__bases__[0].__init__(w)
        page = wiz.NeedleBoreWizard._build_step4(w)
        self.assertIsInstance(page, qtw.QWidget)
        self.assertTrue(hasattr(w, "_s4_focus_ok"))
        self.assertTrue(w._s4_focus_ok.isChecked())
        self.assertIn("focused on the plate bottom", w._s4_focus_ok.text())


if __name__ == "__main__":
    unittest.main()

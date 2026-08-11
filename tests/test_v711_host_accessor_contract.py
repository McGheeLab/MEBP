"""v7.11 — the wizards' duck-typed host must match the REAL host.

Both wizards reach into their host by ``getattr`` name, and both are exercised
in tests against hand-written stub hosts. That is the right shape — it keeps the
wizard testable without booting a page — but it has one failure mode, and this
file exists because it happened:

``PlateLevelWizard._cam()`` read ``host.camera_manager``. ``CalibrationPage``
stores ``_camera_manager``; there is no public spelling. It also called
``mgr.cameras()`` when ``cameras`` is a PROPERTY returning a list, so that line
raised ``TypeError`` into a bare ``except``. Net effect on hardware: the survey
worker was constructed with ``cam=None`` and could not grab a single frame — the
optical bed-levelling survey could not run at all. Every test stayed green,
because the stub host defined ``camera_manager``.

So a stub agreeing with the wizard proves nothing. What has to be checked is the
wizard agreeing with the REAL class.
"""

import ast
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WIZARDS = ("gui/widgets/plate_level_wizard.py",
           "gui/widgets/needle_bore_wizard.py")

#: Names the wizards read off the host that CalibrationPage does not define as
#: class-level attributes because they are assigned in __init__ or are private
#: helpers resolved dynamically. Each one still has to exist — the test below
#: checks them against the source text of the class.
_SOURCE_LEVEL = ()


def _host_attrs_read(path: str) -> set:
    """Every ``getattr(self._host, "NAME", …)`` and ``self._host.NAME``."""
    with open(os.path.join(REPO, path), encoding="utf-8") as f:
        tree = ast.parse(f.read())
    names = set()
    for node in ast.walk(tree):
        if (isinstance(node, ast.Call)
                and isinstance(node.func, ast.Name)
                and node.func.id == "getattr"
                and len(node.args) >= 2
                and isinstance(node.args[1], ast.Constant)
                and isinstance(node.args[1].value, str)):
            tgt = node.args[0]
            if (isinstance(tgt, ast.Attribute) and tgt.attr == "_host"):
                names.add(node.args[1].value)
        if (isinstance(node, ast.Attribute)
                and isinstance(node.value, ast.Attribute)
                and node.value.attr == "_host"):
            names.add(node.attr)
    return names


def _calibration_page_source() -> str:
    with open(os.path.join(REPO, "gui/pages/calibration.py"),
              encoding="utf-8") as f:
        return f.read()


class TestHostAccessorsResolveAgainstTheRealPage(unittest.TestCase):
    def test_every_name_the_wizards_read_exists_on_calibration_page(self):
        src = _calibration_page_source()
        missing = {}
        for wiz in WIZARDS:
            for name in sorted(_host_attrs_read(wiz)):
                # Defined as a method, or assigned on self anywhere in the page.
                if (f"def {name}(" in src or f"self.{name} =" in src
                        or f"self.{name}=" in src or f"self.{name}:" in src):
                    continue
                missing.setdefault(wiz, []).append(name)
        self.assertEqual(
            missing, {},
            "These wizard host accessors do not exist on CalibrationPage, so "
            "they silently resolve to None on hardware while a stub host keeps "
            "the tests green.")

    def test_the_public_camera_manager_spelling_is_not_used(self):
        """The exact bug: `camera_manager` looks right and is not."""
        for wiz in WIZARDS:
            names = _host_attrs_read(wiz)
            self.assertNotIn(
                "camera_manager", names,
                f"{wiz} reads host.camera_manager; the page stores it as "
                f"_camera_manager, so this is always None.")

    def test_camera_manager_dot_cameras_is_never_called(self):
        """`CameraManager.cameras` is a property returning a list. Calling it
        raises TypeError, which every one of these call sites swallows."""
        import inspect
        from gui.widgets.camera_manager import CameraManager
        self.assertIsInstance(
            inspect.getattr_static(CameraManager, "cameras"), property)
        for wiz in WIZARDS:
            with open(os.path.join(REPO, wiz), encoding="utf-8") as f:
                src = f.read()
            self.assertNotIn(".cameras()", src, wiz)


class TestThePlateLevelWizardCanReachItsCamera(unittest.TestCase):
    """Closes the loop: the accessor must work against a host shaped like the
    real page, not merely be spelled correctly."""

    def test_cam_resolves_through_the_private_attribute(self):
        from types import SimpleNamespace
        import gui.widgets.plate_level_wizard as plw

        sentinel = object()
        mgr = SimpleNamespace(cameras=[sentinel, "other"])
        host = SimpleNamespace(_camera_manager=mgr,
                               _ploc_microscope_cam_idx=lambda: 0)
        w = plw.PlateLevelWizard.__new__(plw.PlateLevelWizard)
        w._host = host
        self.assertIs(plw.PlateLevelWizard._cam(w), sentinel)
        self.assertIs(plw.PlateLevelWizard._mgr(w), mgr)

    def test_cam_is_none_when_no_microscope_role_is_assigned(self):
        from types import SimpleNamespace
        import gui.widgets.plate_level_wizard as plw
        host = SimpleNamespace(_camera_manager=SimpleNamespace(cameras=[]),
                               _ploc_microscope_cam_idx=lambda: None)
        w = plw.PlateLevelWizard.__new__(plw.PlateLevelWizard)
        w._host = host
        self.assertIsNone(plw.PlateLevelWizard._cam(w))

    def test_an_out_of_range_index_is_none_not_an_exception(self):
        from types import SimpleNamespace
        import gui.widgets.plate_level_wizard as plw
        host = SimpleNamespace(_camera_manager=SimpleNamespace(cameras=[]),
                               _ploc_microscope_cam_idx=lambda: 3)
        w = plw.PlateLevelWizard.__new__(plw.PlateLevelWizard)
        w._host = host
        self.assertIsNone(plw.PlateLevelWizard._cam(w))


class TestTheDanglingSignalIsConnected(unittest.TestCase):
    def test_focus_sample_reaches_a_slot(self):
        """It was declared, emitted per frame and disconnected in teardown —
        but never connected, so a multi-minute survey showed nothing moving."""
        import inspect
        import gui.widgets.plate_level_wizard as plw
        src = inspect.getsource(plw.PlateLevelWizard._start)
        self.assertIn("focus_sample.connect", src)
        self.assertTrue(hasattr(plw.PlateLevelWizard, "_on_focus_sample"))


if __name__ == "__main__":
    unittest.main()

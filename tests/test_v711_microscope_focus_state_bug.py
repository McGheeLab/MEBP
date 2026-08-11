"""
B1 — ``get_microscope().state`` was missing its ``()``.

``MicroscopeController.state`` is a METHOD, so ``get_microscope().state`` yields a
bound method. ``getattr(bound_method, "has_focus", False)`` is therefore always
False, and the whole block sits inside a broad ``except`` — so
``_microscope_focus_um()`` returned ``None`` on every call, silently, forever.

The consequence was not cosmetic: every ``NeedleFocusTemplateStore`` capture
recorded ``microscope_focus_um=None``, which means **there was no focus↔needle
datum anywhere on disk**. Optical plate-bottom teaching had nothing to bind to.

The second test is a cheap repo-wide guard that pins the whole class of bug.
"""

import os
import re
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class TestFocusReadingWorks(unittest.TestCase):
    def test_microscope_focus_is_actually_read(self):
        """Fails on the pre-fix code. That is the point of the test."""
        from PySide6.QtWidgets import QApplication
        app = QApplication.instance() or QApplication(sys.argv)
        self.assertIsNotNone(app)

        import gui.widgets.needle_bore_wizard as wiz

        class _State:
            has_focus = True
            focus_um = 1234.5

        class _Scope:
            def state(self):
                return _State()

        import SupportClasses.MicroscopeControl as mc
        real = mc.get_microscope
        mc.get_microscope = lambda: _Scope()
        self.addCleanup(lambda: setattr(mc, "get_microscope", real))

        got = wiz.NeedleBoreWizard._microscope_focus_um(object())
        self.assertEqual(got, 1234.5,
                         "the focus reading is still being dropped")

    def test_no_focus_drive_still_returns_none(self):
        import gui.widgets.needle_bore_wizard as wiz

        class _State:
            has_focus = False
            focus_um = None

        class _Scope:
            def state(self):
                return _State()

        import SupportClasses.MicroscopeControl as mc
        real = mc.get_microscope
        mc.get_microscope = lambda: _Scope()
        self.addCleanup(lambda: setattr(mc, "get_microscope", real))

        self.assertIsNone(wiz.NeedleBoreWizard._microscope_focus_um(object()))


class TestRepoWideGuard(unittest.TestCase):
    def test_state_is_never_used_without_calling_it(self):
        """`state` is a method on MicroscopeController. Reading it as an
        attribute yields a bound method whose fields are all absent — which
        fails silently rather than raising."""
        pattern = re.compile(r"\.state(?!\s*\()(?![\w_])")
        offenders = []
        for root, dirs, files in os.walk(REPO):
            dirs[:] = [d for d in dirs
                       if d not in (".git", "__pycache__", "build", "dist",
                                    ".venv", "venv", "DLLs")]
            for fn in files:
                if not fn.endswith(".py"):
                    continue
                path = os.path.join(root, fn)
                if os.path.abspath(path) == os.path.abspath(__file__):
                    continue        # this file QUOTES the buggy form on purpose
                try:
                    with open(path, encoding="utf-8") as f:
                        src = f.read()
                except (OSError, UnicodeDecodeError):
                    continue
                for i, line in enumerate(src.splitlines(), 1):
                    if "scope" not in line and "microscope" not in line.lower():
                        continue
                    if pattern.search(line):
                        offenders.append(
                            f"{os.path.relpath(path, REPO)}:{i}: {line.strip()}")
        self.assertEqual(offenders, [], "\n".join(offenders))


if __name__ == "__main__":
    unittest.main()

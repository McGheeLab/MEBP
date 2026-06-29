"""test_v75x_print_setup_print_manager_forward.py — regression guard.

The v7.5.0 Print Builder refactor made the Printing-mode setup page a
``WizardPrintSetupPage`` shell that composes the legacy ``PrintSetupPage`` as
``self._legacy``. The ``PrintManager`` is created/owned by the legacy page, but
``gui/app.py`` reaches it as ``setup_page.print_manager`` from five sites
(Start / recorder + bridge wiring / Pause / Resume / Abort).

The wizard shell did NOT forward ``print_manager``, so
``hasattr(setup_page, "print_manager")`` was False and ``_on_monitor_start``
logged "No print_manager on setup page" and silently returned — pressing Start
in the Monitor never launched a print.

These tests pin the contract that app.py depends on: the setup page (both the
wizard widget directly AND via ``PrintingModePage.setup_page``) exposes a
``print_manager`` that resolves to the legacy page's manager.

No hardware / Qt event loop needed beyond an offscreen QApplication.
"""

import os
import sys
import unittest
from unittest.mock import MagicMock

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication


def _mock_controller():
    ctrl = MagicMock()
    ctrl.is_xy_connected = False
    ctrl.is_zp_connected = False
    ctrl.zero_position = {"x": 0.0, "y": 0.0, "Z": 0.0}
    return ctrl


class _Base(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls._app = QApplication.instance() or QApplication(sys.argv)


class TestWizardForwardsPrintManager(_Base):
    def setUp(self):
        from gui.pages.print_setup import PrintSetupPage
        # PrintSetupPage from the package is the WizardPrintSetupPage shell.
        self.page = PrintSetupPage(_mock_controller(), settings=None)

    def test_setup_page_has_print_manager_attr(self):
        # The exact gate app.py uses: ``not hasattr(setup_page, "print_manager")``
        # must be False so _on_monitor_start does not bail out.
        self.assertTrue(hasattr(self.page, "print_manager"))

    def test_print_manager_resolves_to_legacy_manager(self):
        self.assertIsNotNone(self.page.print_manager)
        self.assertIs(self.page.print_manager, self.page._legacy.print_manager)

    def test_is_wizard_shell_composing_legacy(self):
        # Guards the assumption behind the forward: the page composes a legacy
        # page that owns the manager.
        self.assertEqual(type(self.page).__name__, "WizardPrintSetupPage")
        self.assertTrue(hasattr(self.page, "_legacy"))


class TestPrintingModeSetupPage(_Base):
    """The real app.py access path: self._printing_mode.setup_page."""

    def test_setup_page_exposes_print_manager(self):
        from gui.pages.printing_mode import PrintingModePage
        mode = PrintingModePage(_mock_controller(), None)
        setup_page = mode.setup_page
        # Mirrors gui/app.py::_on_monitor_start / _wire_print_manager_to_monitor.
        self.assertTrue(hasattr(setup_page, "print_manager"))
        self.assertIsNotNone(setup_page.print_manager)


if __name__ == "__main__":
    unittest.main()

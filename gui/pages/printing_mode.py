"""
printing_mode.py — Printing mode container page.

v7.3.3: Wraps Print Setup, Print Monitor, Print Results, and Helper
Functions as sub-pages within a ModePage, providing right-side icon
navigation between them.

Sub-page layout (right sidebar):
    🖨️  Print Setup
    📈  Print Monitor
    📋  Print Results
    🧰  Helper Functions
"""

from __future__ import annotations

import logging

from gui.pages.mode_page import ModePage
from gui.pages.print_setup import PrintSetupPage
from gui.pages.print_monitor import PrintMonitorPage
from gui.pages.print_results import PrintResultsPage
from gui.pages.helper_functions import HelperFunctionsPage

from SupportClasses.StageController import StageController
from SupportClasses.Settings import Settings

logger = logging.getLogger(__name__)


class PrintingModePage(ModePage):
    """Printing mode — contains Print Setup, Monitor, Results, and Helpers."""

    def __init__(self, controller: StageController, settings: Settings,
                 parent=None):
        super().__init__(parent)

        self.controller = controller
        self.settings = settings

        # Create sub-pages
        self._setup_page = PrintSetupPage(controller)
        self._monitor_page = PrintMonitorPage(controller, settings)
        self._results_page = PrintResultsPage(controller, settings)
        self._helpers_page = HelperFunctionsPage()

        # v7.4.2: solid-white SVG icons via the factory.
        self.add_sub_page("printer",   "Print Setup",      self._setup_page)
        self.add_sub_page("chart",     "Print Monitor",    self._monitor_page)
        self.add_sub_page("clipboard", "Print Results",    self._results_page)
        self.add_sub_page("wrench",    "Helper Functions", self._helpers_page)

        logger.info("PrintingModePage initialized with 4 sub-pages")

    # ── Convenience accessors ────────────────────────────────────

    @property
    def setup_page(self) -> PrintSetupPage:
        return self._setup_page

    @property
    def monitor_page(self) -> PrintMonitorPage:
        return self._monitor_page

    @property
    def results_page(self) -> PrintResultsPage:
        return self._results_page

    @property
    def helpers_page(self) -> HelperFunctionsPage:
        return self._helpers_page

    # ── Overrides ────────────────────────────────────────────────

    def get_page_title(self) -> str:
        return self.get_sub_page_title()

    def switch_to_monitor(self):
        """Switch to the Print Monitor sub-page (index 1)."""
        self.switch_to(1)

    def switch_to_results(self):
        """Switch to the Print Results sub-page (index 2)."""
        self.switch_to(2)

    def switch_to_setup(self):
        """Switch to the Print Setup sub-page (index 0)."""
        self.switch_to(0)

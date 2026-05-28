"""
printing_mode.py — Printing mode container page.

v7.3.3: Wraps Print Setup, Print Monitor, and Print Results as sub-pages
within a ModePage, providing right-side icon navigation between them.

v7.5.x: Helper Functions, Hardware, and Print Settings moved out to the new
Print Builder mode page (``print_builder.py``). Printing mode is now
run-focused with three sub-pages:

    🖨  Print Setup
    📈  Print Monitor
    📋  Print Results
"""

from __future__ import annotations

import logging

from gui.pages.mode_page import ModePage
from gui.pages.print_setup import PrintSetupPage
from gui.pages.print_monitor import PrintMonitorPage
from gui.pages.print_results import PrintResultsPage

from SupportClasses.StageController import StageController
from SupportClasses.Settings import Settings

logger = logging.getLogger(__name__)


# ═══════════════════════════════════════════════════════════════════
# Mode container
# ═══════════════════════════════════════════════════════════════════


class PrintingModePage(ModePage):
    """Printing mode — contains Setup, Monitor, Results."""

    def __init__(self, controller: StageController, settings: Settings,
                 parent=None):
        # v7.5.2: use the vertical icon strip variant so the
        # sub-page tabs no longer eat ~60 px of top headroom.
        super().__init__(parent, tab_orientation="vertical")

        self.controller = controller
        self.settings = settings

        # Create sub-pages
        self._setup_page = PrintSetupPage(controller)
        self._monitor_page = PrintMonitorPage(controller, settings)
        self._results_page = PrintResultsPage(controller, settings)

        self.add_sub_page("printer",   "Print Setup",   self._setup_page)
        self.add_sub_page("chart",     "Print Monitor", self._monitor_page)
        self.add_sub_page("clipboard", "Print Results", self._results_page)

        logger.info("PrintingModePage initialized with 3 sub-pages")

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

    # ── Overrides ────────────────────────────────────────────────

    def get_context_widget(self):
        """v7.5.5: the active sub-page's context widget (Tools) is
        embedded INSIDE this mode page — to the right of the nav
        stack — instead of being mounted by app.py outside the mode
        page. Return None so app.py's ``ui_extraLeftBox`` stays
        hidden."""
        return None

    def get_page_title(self) -> str:
        return self.get_sub_page_title()

    def switch_to_monitor(self):
        self.switch_to(1)

    def switch_to_results(self):
        self.switch_to(2)

    def switch_to_setup(self):
        self.switch_to(0)

"""
pick_place_mode.py — Pick & Place mode container page.

v7.3.3: Wraps Operation Setup, Target Selection, and Execution
as sub-pages within a ModePage, providing right-side icon navigation.

Sub-page layout (right sidebar):
    ⚙️   Operation Setup      (select mode + configure)
    🎯  Target Selection     (camera feed, click to mark)
    ▶️   Execution            (run queue, context shows queue)

Data flow:
    OperationSetup.config_changed → TargetSelection.set_operation_config
    OperationSetup.config_changed → Execution.set_operation_config
    TargetSelection.queue_changed → Execution.update_queue
"""

from __future__ import annotations

import logging

from gui.pages.mode_page import ModePage
from gui.pages.pp_operation_setup import PPOperationSetupPage
from gui.pages.pp_target_selection import PPTargetSelectionPage
from gui.pages.pp_execution import PPExecutionPage

from SupportClasses.StageController import StageController
from SupportClasses.Settings import Settings

logger = logging.getLogger(__name__)


class PickPlaceModePage(ModePage):
    """Pick & Place mode — contains Operation Setup, Target Selection, Execution."""

    def __init__(self, controller: StageController, settings: Settings,
                 camera_manager=None, parent=None):
        super().__init__(parent)

        self.controller = controller
        self.settings = settings

        # Create sub-pages
        self._setup_page = PPOperationSetupPage()
        self._target_page = PPTargetSelectionPage(
            controller, settings, camera_manager=camera_manager)
        self._execution_page = PPExecutionPage(controller)

        # Register sub-pages with right sidebar icons
        self.add_sub_page("⚙️", "Operation Setup", self._setup_page)
        self.add_sub_page("🎯", "Target Selection", self._target_page)
        self.add_sub_page("▶️", "Execution", self._execution_page)

        # Wire inter-page data flow
        # Setup → Target Selection + Execution (operation config)
        self._setup_page.config_changed.connect(
            self._target_page.set_operation_config)
        self._setup_page.config_changed.connect(
            self._execution_page.set_operation_config)

        # Target Selection → Execution (queue updates)
        self._target_page.queue_changed.connect(
            self._execution_page.update_queue)

        logger.info("PickPlaceModePage initialized with 3 sub-pages")

    # ── Convenience accessors ────────────────────────────────────

    @property
    def setup_page(self) -> PPOperationSetupPage:
        return self._setup_page

    @property
    def target_page(self) -> PPTargetSelectionPage:
        return self._target_page

    @property
    def execution_page(self) -> PPExecutionPage:
        return self._execution_page

    # ── Overrides ────────────────────────────────────────────────

    def get_page_title(self) -> str:
        return self.get_sub_page_title()

    # ── Well list propagation ────────────────────────────────────

    def set_well_list(self, wells: list[str]):
        """Propagate well list to setup and target selection pages."""
        self._setup_page.set_well_list(wells)
        self._target_page.set_well_list(wells)

    def set_well_positions(self, positions: dict[str, tuple[float, float]]):
        """Propagate well positions to all sub-pages."""
        self._target_page.set_well_positions(positions)
        # Also set on executor when it runs

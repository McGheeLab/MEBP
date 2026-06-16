"""workflows_mode.py — top-level Workflows mode container.

v7.4.3: Replaces the v7.3.3 Pick & Place mode. The landing view is a
`WorkflowPickerPage` showing one tile per workflow. Clicking a tile
swaps the internal QStackedWidget to that workflow's page. Each
workflow page has a Back button that returns to the picker.

Not a `ModePage` subclass — workflows are *modal* (commit-then-back),
not parallel tabs. The active sub-page's title is propagated up to the
main window via `get_page_title()`.
"""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING

from PySide6.QtCore import Signal
from PySide6.QtWidgets import QStackedWidget, QVBoxLayout, QWidget

from gui.pages.workflows.workflow_picker import WorkflowPickerPage, WORKFLOWS
from gui.pages.workflows._stub_workflow import StubWorkflowPage
from gui.pages.workflows.spheroid_pickup_workflow import (
    SpheroidPickupWorkflowPage,
)
from gui.pages.workflows.quick_print_workflow import QuickPrintWorkflowPage

if TYPE_CHECKING:
    from SupportClasses.StageController import StageController
    from SupportClasses.Settings import Settings

logger = logging.getLogger(__name__)


class WorkflowsModePage(QWidget):
    """Workflows mode root widget.

    Internal stack:
        [0] WorkflowPickerPage  — landing, always index 0
        [1..n] workflow pages, indexed by `_workflow_index` dict

    Sub-pages expose `back_requested` (returns to picker).
    """

    # Forwarded for parity with ModePage subclasses
    sub_page_changed = Signal(int)

    def __init__(
        self,
        controller: "StageController",
        settings: "Settings",
        camera_manager=None,
        parent: QWidget | None = None,
    ):
        super().__init__(parent)
        self.controller = controller
        self.settings = settings
        self._camera_manager = camera_manager
        self._hw_config = None

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        self._stack = QStackedWidget(self)
        layout.addWidget(self._stack)

        # ── Index 0: picker (always present) ──
        self._picker = WorkflowPickerPage()
        self._picker.workflow_selected.connect(self._on_workflow_selected)
        self._stack.addWidget(self._picker)

        # ── Indices 1..n: one page per workflow tile ──
        self._workflow_index: dict[str, int] = {}
        for tile in WORKFLOWS:
            if tile.workflow_id == "spheroid_pickup":
                page = SpheroidPickupWorkflowPage(
                    controller=controller,
                    settings=settings,
                    camera_manager=camera_manager,
                )
            elif tile.workflow_id == "quick_print":
                page = QuickPrintWorkflowPage(
                    controller=controller,
                    settings=settings,
                )
            else:
                page = StubWorkflowPage(tile.title)
            page.back_requested.connect(self._show_picker)
            idx = self._stack.addWidget(page)
            self._workflow_index[tile.workflow_id] = idx

        self._stack.setCurrentIndex(0)
        logger.info("WorkflowsModePage initialized with %d workflows",
                    len(self._workflow_index))

    # ── Navigation ────────────────────────────────────────────────

    def _on_workflow_selected(self, workflow_id: str):
        idx = self._workflow_index.get(workflow_id)
        if idx is None:
            logger.warning("Unknown workflow_id: %s", workflow_id)
            return
        self._stack.setCurrentIndex(idx)
        self.sub_page_changed.emit(idx)

    def _show_picker(self):
        self._stack.setCurrentIndex(0)
        self.sub_page_changed.emit(0)

    # ── Hooks called by MainWindow ─────────────────────────────────

    def get_page_title(self) -> str:
        """Title shown in the top bar — delegate to the active workflow."""
        current = self._stack.currentWidget()
        if current is not None and hasattr(current, "get_page_title"):
            try:
                t = current.get_page_title()
                if t:
                    return t
            except Exception:
                pass
        if current is self._picker:
            return "Workflows"
        for wid, idx in self._workflow_index.items():
            if idx == self._stack.currentIndex():
                for tile in WORKFLOWS:
                    if tile.workflow_id == wid:
                        return tile.title
        return "Workflows"

    def get_sub_page_title(self) -> str:
        """Title shown above the left context panel."""
        current = self._stack.currentWidget()
        if current is not None and hasattr(current, "get_sub_page_title"):
            try:
                t = current.get_sub_page_title()
                if t:
                    return t
            except Exception:
                pass
        return self.get_page_title()

    def get_context_widget(self):
        """Delegate to the active workflow page; None on the picker."""
        current = self._stack.currentWidget()
        if current is None or current is self._picker:
            return None
        if hasattr(current, "get_context_widget"):
            try:
                return current.get_context_widget()
            except Exception as e:
                logger.debug("get_context_widget delegate failed: %s", e)
        return None

    def on_status_update(self):
        """Forward MainWindow's periodic tick to the active workflow."""
        current = self._stack.currentWidget()
        if current is not None and hasattr(current, "on_status_update"):
            try:
                current.on_status_update()
            except Exception:
                pass

    def set_hardware_config(self, hw_config):
        """Push the latest hardware config into workflow pages that need it."""
        self._hw_config = hw_config
        for i in range(1, self._stack.count()):
            page = self._stack.widget(i)
            if hasattr(page, "set_hardware_config"):
                page.set_hardware_config(hw_config)

    def set_well_list(self, wells: list[str]):
        for i in range(1, self._stack.count()):
            page = self._stack.widget(i)
            if hasattr(page, "set_well_list"):
                page.set_well_list(wells)

    def set_well_positions(self, positions: dict[str, tuple[float, float]]):
        for i in range(1, self._stack.count()):
            page = self._stack.widget(i)
            if hasattr(page, "set_well_positions"):
                page.set_well_positions(positions)

    def set_calibration_data(self, plate, well_positions, safe_z):
        """Forward plate + well positions + safe Z to workflow sub-pages."""
        for i in range(1, self._stack.count()):
            page = self._stack.widget(i)
            if hasattr(page, "set_calibration_data"):
                try:
                    page.set_calibration_data(plate, well_positions, safe_z)
                except Exception as e:
                    logger.debug("set_calibration_data fanout failed: %s", e)

    def set_z_references(self, refs):
        for i in range(1, self._stack.count()):
            page = self._stack.widget(i)
            if hasattr(page, "set_z_references"):
                try:
                    page.set_z_references(refs)
                except Exception as e:
                    logger.debug("set_z_references fanout failed: %s", e)

    def set_settings(self, settings):
        for i in range(1, self._stack.count()):
            page = self._stack.widget(i)
            if hasattr(page, "set_settings"):
                try:
                    page.set_settings(settings)
                except Exception:
                    pass

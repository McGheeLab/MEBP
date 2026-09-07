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
from gui.pages.workflows.cell_targeting_workflow import (
    CellTargetingWorkflowPage,
)
from gui.pages.workflows.cell_labeling_workflow import (
    CellLabelingWorkflowPage,
)
from gui.pages.workflows.quick_print_workflow import QuickPrintWorkflowPage
from gui.pages.workflows.print_calibrator_workflow import (
    PrintCalibratorWorkflowPage,
)
from gui.pages.workflows.full_print_workflow import FullPrintWorkflowPage
from gui.pages.workflows.fluorescence_mosaic_workflow import (
    FluorescenceMosaicWorkflowPage,
)
from gui.pages.workflows.stress_test_workflow import StressTestWorkflowPage
from gui.pages.workflows.timing_calibration_workflow import (
    TimingCalibrationWorkflowPage,
)
from gui.pages.workflows.common_print_settings_workflow import (
    CommonPrintSettingsWorkflowPage,
)
from gui.pages.workflows.lablink_workflow import LabLinkWorkflowPage
from gui.pages.workflows.incubator_workflow import IncubatorWorkflowPage

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
            elif tile.workflow_id == "cell_targeting":
                page = CellTargetingWorkflowPage(
                    controller=controller,
                    settings=settings,
                    camera_manager=camera_manager,
                )
            elif tile.workflow_id == "cell_labeling":
                page = CellLabelingWorkflowPage(
                    controller=controller,
                    settings=settings,
                    camera_manager=camera_manager,
                )
            elif tile.workflow_id == "quick_print":
                page = QuickPrintWorkflowPage(
                    controller=controller,
                    settings=settings,
                    camera_manager=camera_manager,
                )
            elif tile.workflow_id == "print_calibrator":
                page = PrintCalibratorWorkflowPage(
                    controller=controller,
                    settings=settings,
                    camera_manager=camera_manager,
                )
            elif tile.workflow_id == "full_print":
                page = FullPrintWorkflowPage(
                    controller=controller,
                    settings=settings,
                    camera_manager=camera_manager,
                )
            elif tile.workflow_id == "fluorescence_mosaic":
                page = FluorescenceMosaicWorkflowPage(
                    controller=controller,
                    settings=settings,
                    camera_manager=camera_manager,
                )
            elif tile.workflow_id == "stress_test":
                page = StressTestWorkflowPage(
                    controller=controller,
                    settings=settings,
                    camera_manager=camera_manager,
                )
            elif tile.workflow_id == "timing_calibration":
                page = TimingCalibrationWorkflowPage(
                    controller=controller,
                    settings=settings,
                    camera_manager=camera_manager,
                )
            elif tile.workflow_id == "common_print_settings":
                page = CommonPrintSettingsWorkflowPage(
                    controller=controller,
                    settings=settings,
                    camera_manager=camera_manager,
                )
            elif tile.workflow_id == "lablink":
                page = LabLinkWorkflowPage(
                    controller=controller,
                    settings=settings,
                    camera_manager=camera_manager,
                )
            elif tile.workflow_id == "incubator":
                page = IncubatorWorkflowPage(
                    controller=controller,
                    settings=settings,
                    camera_manager=camera_manager,
                )
            else:
                page = StubWorkflowPage(tile.title)
            page.back_requested.connect(self._show_picker)
            # Some workflow pages (e.g. Full Print) host their own internal
            # sub-page nav; re-surface their sub_page_changed so the top-bar
            # title refreshes when the user switches inner sub-pages.
            if hasattr(page, "sub_page_changed"):
                page.sub_page_changed.connect(
                    lambda *_: self.sub_page_changed.emit(
                        self._stack.currentIndex()))
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

    def open_workflow(self, workflow_id: str) -> bool:
        """Programmatically open a workflow by id (used by app.py to route a
        print job/file straight to the Full Print tile). Returns False if the
        id is unknown."""
        idx = self._workflow_index.get(workflow_id)
        if idx is None:
            return False
        self._stack.setCurrentIndex(idx)
        self.sub_page_changed.emit(idx)
        return True

    @property
    def full_print_page(self):
        """The FullPrintWorkflowPage instance (or None if the tile is absent),
        so app.py can reach the re-homed print stack's
        setup_page/monitor_page/results_page + switch_to_* surface."""
        idx = self._workflow_index.get("full_print")
        return self._stack.widget(idx) if idx is not None else None

    @property
    def quick_print_page(self):
        """The QuickPrintWorkflowPage instance (or None), so app.py can refresh
        its saved-prints combo when the Print Library changes files on disk."""
        idx = self._workflow_index.get("quick_print")
        return self._stack.widget(idx) if idx is not None else None

    @property
    def incubator_page(self):
        """The IncubatorWorkflowPage instance (or None if the tile is absent)."""
        idx = self._workflow_index.get("incubator")
        return self._stack.widget(idx) if idx is not None else None

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

    def context_label(self):
        """v7.19: the name for the left box's native pill, delegated.

        MainWindow labels that pill from a map keyed on the PAGE class, which
        for every workflow is this one class — so a workflow whose panel is not
        a jog panel (Fluorescence Mosaic's signal controls) would be labelled
        "Jog". Returning None keeps MainWindow's default.
        """
        current = self._stack.currentWidget()
        if current is None or current is self._picker:
            return None
        if hasattr(current, "context_label"):
            try:
                return current.context_label()
            except Exception as e:
                logger.debug("context_label delegate failed: %s", e)
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

    def set_common_print_settings(self, common):
        """Fan the shared CommonPrintSettings model out to workflow pages so
        their inheriting fields re-sync to the current common values."""
        self._common_print_settings = common
        for i in range(1, self._stack.count()):
            page = self._stack.widget(i)
            if hasattr(page, "set_common_print_settings"):
                try:
                    page.set_common_print_settings(common)
                except Exception as e:
                    logger.debug("set_common_print_settings fanout failed: %s", e)

    def refresh_speed_limits(self):
        """v7.21.2: forward the speed-limit broadcast to the workflow pages.

        ``MainWindow._refresh_all_speed_limits`` walks the top-level pages and their
        jog/control panels, but the workflow pages live inside this mode page's
        stack — so without this forwarder Quick Print never learned that the XY
        calibration had just changed the machine's top speed, and kept showing a
        stale speed cap and a stale "stage motion not characterised" warning until
        the app was restarted.
        """
        for i in range(1, self._stack.count()):
            page = self._stack.widget(i)
            fn = getattr(page, "refresh_speed_limits", None)
            if callable(fn):
                try:
                    fn()
                except Exception as e:
                    logger.debug("refresh_speed_limits fanout failed: %s", e)

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

    def set_visible_z_references(self, keys):
        """v7.9.1: fan out the quick-move badge selection (see the Jog page)."""
        for i in range(1, self._stack.count()):
            page = self._stack.widget(i)
            if hasattr(page, "set_visible_z_references"):
                try:
                    page.set_visible_z_references(keys)
                except Exception as e:
                    logger.debug(
                        "set_visible_z_references fanout failed: %s", e)

    def set_settings(self, settings):
        for i in range(1, self._stack.count()):
            page = self._stack.widget(i)
            if hasattr(page, "set_settings"):
                try:
                    page.set_settings(settings)
                except Exception:
                    pass

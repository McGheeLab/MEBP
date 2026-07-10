"""full_print_workflow.py — Full Print workflow page.

v7.5.x: Re-homes the former top-level "Printing" mode (the rich
Setup → Monitor → Results stack) into the Workflows mode as a tile.

Quick Print is for *testing printing conditions* on a single object in a
single well; Full Print is the *full-plate* workflow — assign multiple
objects to multiple wells, run with live monitoring, then review results.

Design: this page is a thin wrapper that INTERNALLY hosts a
:class:`PrintingModePage` instance (the existing, mature mode page with its
vertical icon nav, embedded Tools context, wizard step strip, and all three
sub-pages). Nothing about the print stack is rewritten — it is simply mounted
inside a workflow shell that adds a "← Back to Workflows" header and forwards
the standard workflow-page contract methods down to the inner mode page.

``gui/app.py`` reaches the print stack through this wrapper's
``setup_page`` / ``monitor_page`` / ``results_page`` accessors and
``switch_to_*`` helpers (identical surface to ``PrintingModePage``), so the
job pipeline / monitor / results wiring is unchanged apart from the handle it
goes through.
"""

from __future__ import annotations

import logging

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
)

from gui.styles import COLORS
from gui.scaling import s, sf
from gui.pages.printing_mode import PrintingModePage

logger = logging.getLogger(__name__)


class FullPrintWorkflowPage(QWidget):
    """Full Print workflow page — wraps a :class:`PrintingModePage`.

    Signals:
        back_requested: user clicked "← Back to Workflows".
        sub_page_changed: re-emitted from the inner mode page so the
            MainWindow can refresh the top-bar title when the user switches
            between the Setup / Monitor / Results sub-pages.
    """

    back_requested = Signal()
    sub_page_changed = Signal(int)

    def __init__(self, controller, settings, camera_manager=None, parent=None):
        super().__init__(parent)
        self._controller = controller
        self._settings = settings
        # Accepted for ctor parity with the other workflow pages; the print
        # stack does not use a camera (PrintMonitorPage has no camera feed).
        self._camera_manager = camera_manager

        # The whole Setup → Monitor → Results stack, re-homed unchanged.
        self._mode = PrintingModePage(controller, settings)
        self._mode.sub_page_changed.connect(self.sub_page_changed.emit)

        outer = QVBoxLayout(self)
        outer.setContentsMargins(s(12), s(10), s(12), s(12))
        outer.setSpacing(s(10))
        outer.addLayout(self._build_header())
        outer.addWidget(self._mode, stretch=1)

    # ── Header ────────────────────────────────────────────────────────

    def _build_header(self) -> QHBoxLayout:
        row = QHBoxLayout()
        row.setSpacing(s(8))
        back = QPushButton("← Back to Workflows")
        back.setCursor(Qt.PointingHandCursor)
        back.clicked.connect(self.back_requested.emit)
        row.addWidget(back)

        title = QLabel("Full Print")
        title.setStyleSheet(
            f"color: {COLORS['blue']};"
            f"font-size: {sf(14)}pt;"
            f"font-weight: 600;"
        )
        row.addWidget(title)
        row.addStretch(1)
        return row

    # ── Accessors app.py needs (mirror PrintingModePage) ──────────────

    @property
    def setup_page(self):
        return self._mode.setup_page

    @property
    def monitor_page(self):
        return self._mode.monitor_page

    @property
    def results_page(self):
        return self._mode.results_page

    def switch_to_setup(self):
        self._mode.switch_to_setup()

    def switch_to_monitor(self):
        self._mode.switch_to_monitor()

    def switch_to_results(self):
        self._mode.switch_to_results()

    # ── Workflow-page contract (delegated to the inner mode page) ─────

    def set_hardware_config(self, config) -> None:
        if hasattr(self._mode, "set_hardware_config"):
            self._mode.set_hardware_config(config)

    def set_calibration_data(self, plate, well_positions, safe_z) -> None:
        setup = self._mode.setup_page
        if hasattr(setup, "set_calibration_data"):
            try:
                setup.set_calibration_data(plate, well_positions, safe_z)
            except Exception as e:
                logger.debug("full print set_calibration_data failed: %s", e)

    def set_z_references(self, refs) -> None:
        setup = self._mode.setup_page
        if hasattr(setup, "set_z_references"):
            try:
                setup.set_z_references(refs)
            except Exception as e:
                logger.debug("full print set_z_references failed: %s", e)

    def set_common_print_settings(self, common) -> None:
        setup = self._mode.setup_page
        if hasattr(setup, "set_common_print_settings"):
            try:
                setup.set_common_print_settings(common)
            except Exception as e:
                logger.debug("full print set_common_print_settings failed: %s", e)

    def set_settings(self, settings) -> None:
        self._settings = settings
        if hasattr(self._mode, "set_settings"):
            try:
                self._mode.set_settings(settings)
            except Exception:
                pass

    def on_status_update(self) -> None:
        if hasattr(self._mode, "on_status_update"):
            try:
                self._mode.on_status_update()
            except Exception:
                pass

    def get_page_title(self) -> str:
        if hasattr(self._mode, "get_page_title"):
            try:
                return self._mode.get_page_title()
            except Exception:
                pass
        return "Full Print"

    def get_sub_page_title(self) -> str:
        if hasattr(self._mode, "get_sub_page_title"):
            try:
                return self._mode.get_sub_page_title()
            except Exception:
                pass
        return self.get_page_title()

    def get_context_widget(self):
        # PrintingModePage embeds the active sub-page's Tools context inside
        # itself and returns None here, so the MainWindow's external left
        # context box stays hidden — identical to the old Printing mode.
        if hasattr(self._mode, "get_context_widget"):
            try:
                return self._mode.get_context_widget()
            except Exception:
                pass
        return None

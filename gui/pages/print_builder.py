"""
print_builder.py — Print Builder mode container page (v7.5.x).

A dedicated top-level page for *building* print trajectories, separate from
the *running*-focused Printing mode. Four sub-pages in the vertical icon
strip:

    ✏️  Sketch          draw-to-print canvas (new) — the easy way to author
    🖼  Image Import     image-stack → raster toolpath (the former Helper
                         Functions page)
    🪡  Hardware         read-only hardware config summary
    ⚙   Print Settings   display toggles for the Print Setup wizard

Both the Sketch and Image Import sub-pages emit ``print_file_created`` —
their baked ``csv_import`` object flows into Print Setup's custom-prints
area (wired in ``gui/app.py``).

The Hardware and Print Settings sub-page classes were relocated here from
``printing_mode.py`` in this version (they are build-time references, not
run-time controls).
"""

from __future__ import annotations

import logging

from PySide6.QtWidgets import (
    QCheckBox, QComboBox, QGroupBox, QHBoxLayout, QLabel, QVBoxLayout, QWidget,
)

from gui.pages.mode_page import ModePage
from gui.pages.helper_functions import HelperFunctionsPage
from gui.pages.print_builder_sketch import SketchPage
from gui.pages.print_workspace import HardwareSummaryWidget
from gui.scaling import s as _s, scaled_font_size as _sf
from gui.styles import COLORS

logger = logging.getLogger(__name__)


# ═══════════════════════════════════════════════════════════════════
# Hardware summary sub-page  (relocated from printing_mode.py v7.5.x)
# ═══════════════════════════════════════════════════════════════════

class PrintingHardwarePage(QWidget):
    """Read-only Hardware Configuration Summary.

    Hosts the existing :class:`HardwareSummaryWidget`. The user can click
    "Edit Hardware Setup" inside the summary to navigate to the main
    Hardware Setup page.
    """

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setStyleSheet(f"background: {COLORS['base']};")

        lay = QVBoxLayout(self)
        lay.setContentsMargins(_s(16), _s(16), _s(16), _s(16))
        lay.setSpacing(_s(8))

        self._summary = HardwareSummaryWidget(self)
        lay.addWidget(self._summary, 1)

    def set_hardware_config(self, config) -> None:
        if hasattr(self._summary, "update_from_config"):
            self._summary.update_from_config(config)

    def get_page_title(self) -> str:
        return "Hardware"


# ═══════════════════════════════════════════════════════════════════
# Print Settings sub-page  (relocated from printing_mode.py v7.5.x)
# ═══════════════════════════════════════════════════════════════════

class PrintingSettingsPage(QWidget):
    """Print Settings — display toggles for the Print Setup wizard.

    Visual-only toggles surfacing well-plate / object-preview options.
    """

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setStyleSheet(f"background: {COLORS['base']};")

        lay = QVBoxLayout(self)
        lay.setContentsMargins(_s(16), _s(16), _s(16), _s(16))
        lay.setSpacing(_s(12))

        title = QLabel("Print Settings")
        title.setStyleSheet(
            f"color: {COLORS['text']}; "
            f"font-size: {_sf(15)}pt; font-weight: 700;")
        lay.addWidget(title)

        sub = QLabel("Display options for the Print Setup wizard.")
        sub.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {_sf(10)}pt;")
        lay.addWidget(sub)

        plate_grp = self._make_group("Well Plate")
        plate_lay = plate_grp.layout()
        self._show_well_labels_cb = QCheckBox("Show well labels")
        self._show_well_labels_cb.setChecked(True)
        plate_lay.addWidget(self._show_well_labels_cb)
        self._show_trajectory_cb = QCheckBox("Show trajectory paths")
        self._show_trajectory_cb.setChecked(True)
        plate_lay.addWidget(self._show_trajectory_cb)
        color_row = QHBoxLayout()
        color_row.addWidget(QLabel("Color by:"))
        self._color_mode_combo = QComboBox()
        self._color_mode_combo.addItems(["Role", "Ink", "Status"])
        color_row.addWidget(self._color_mode_combo)
        color_row.addStretch(1)
        plate_lay.addLayout(color_row)
        lay.addWidget(plate_grp)

        obj_grp = self._make_group("Object Preview")
        obj_lay = obj_grp.layout()
        self._show_grid_cb = QCheckBox("Show grid")
        self._show_grid_cb.setChecked(True)
        obj_lay.addWidget(self._show_grid_cb)
        self._show_axes_cb = QCheckBox("Show axes")
        self._show_axes_cb.setChecked(True)
        obj_lay.addWidget(self._show_axes_cb)
        self._show_dimensions_cb = QCheckBox("Show dimensions")
        self._show_dimensions_cb.setChecked(False)
        obj_lay.addWidget(self._show_dimensions_cb)
        lay.addWidget(obj_grp)
        lay.addStretch(1)

    def _make_group(self, title: str) -> QGroupBox:
        grp = QGroupBox(title)
        grp.setStyleSheet(
            f"QGroupBox {{"
            f"  background: {COLORS['base']};"
            f"  color: {COLORS['text']};"
            f"  border: 1px solid {COLORS['surface1']};"
            f"  border-radius: {_s(6)}px;"
            f"  margin-top: {_s(10)}px; padding-top: {_s(20)}px;"
            f"  font-weight: 700;"
            f"}}"
            f"QGroupBox::title {{"
            f"  subcontrol-origin: margin;"
            f"  subcontrol-position: top left;"
            f"  padding: {_s(2)}px {_s(8)}px;"
            f"  background: {COLORS['surface1']};"
            f"  border-top-left-radius: {_s(6)}px;"
            f"  border-top-right-radius: {_s(6)}px;"
            f"}}")
        glay = QVBoxLayout(grp)
        glay.setContentsMargins(_s(10), _s(8), _s(10), _s(8))
        glay.setSpacing(_s(4))
        return grp

    def get_page_title(self) -> str:
        return "Print Settings"

    def set_hardware_config(self, config) -> None:
        return None


# ═══════════════════════════════════════════════════════════════════
# Mode container
# ═══════════════════════════════════════════════════════════════════

class PrintBuilderPage(ModePage):
    """Print Builder — Sketch, Image Import, Hardware, Print Settings."""

    def __init__(self, controller=None, settings=None, parent=None):
        super().__init__(parent, tab_orientation="vertical")
        self.controller = controller
        self.settings = settings

        self._sketch_page = SketchPage()
        self._image_page = HelperFunctionsPage()
        self._hardware_page = PrintingHardwarePage()
        self._settings_page = PrintingSettingsPage()

        self._titles = ["Sketch", "Image Import", "Hardware", "Print Settings"]
        self.add_sub_page("pencil",   "Sketch",         self._sketch_page)
        self.add_sub_page("camera",   "Image Import",   self._image_page)
        self.add_sub_page("needle",   "Hardware",       self._hardware_page)
        self.add_sub_page("settings", "Print Settings", self._settings_page)

        logger.info("PrintBuilderPage initialized with 4 sub-pages")

    # ── Convenience accessors ─────────────────────────────────────

    @property
    def sketch_page(self) -> SketchPage:
        return self._sketch_page

    @property
    def image_import_page(self) -> HelperFunctionsPage:
        return self._image_page

    @property
    def hardware_page(self) -> PrintingHardwarePage:
        return self._hardware_page

    @property
    def settings_page(self) -> PrintingSettingsPage:
        return self._settings_page

    # ── Calibration fanout ────────────────────────────────────────

    def set_z_references(self, refs: dict) -> None:
        """v7.5.x: forward the calibration Z-reference set (incl.
        ``plate_bottom_z``) to any sub-page that wants it (the Sketch page
        uses it to express print Z as a height above the plate bottom)."""
        for page in self._sub_pages:
            if hasattr(page, "set_z_references"):
                try:
                    page.set_z_references(refs)
                except Exception as e:
                    logger.debug(f"set_z_references on sub-page failed: {e}")

    # ── Overrides ─────────────────────────────────────────────────

    def get_context_widget(self):
        # No left context panel for any sub-page.
        return None

    def get_sub_page_title(self) -> str:
        # Use the registered tab labels (e.g. "Image Import") rather than the
        # sub-page's own get_page_title (which would read "Helper Functions").
        i = self.get_active_index()
        if 0 <= i < len(self._titles):
            return self._titles[i]
        return super().get_sub_page_title()

    def get_page_title(self) -> str:
        return self.get_sub_page_title()

    def switch_to_sketch(self):
        self.switch_to(0)

    def switch_to_image_import(self):
        self.switch_to(1)

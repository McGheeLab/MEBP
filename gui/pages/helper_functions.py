"""
helper_functions.py — Helper Functions page for MEBP.

Image-to-toolpath generator with three input modes:
  A) TIFF stack — multi-frame TIFF, RGB channels split across pumps
  B) Numbered sequences — multi-select files per pump (layer order)
  C) Single images — one image per pump, repeated for N layers

Uses mask-based rastering (pathplan3.py approach):
  1. Combine all pump images → binary mask
  2. Find edge pixels on each raster row
  3. Walk path between edges
  4. Check each pump image for flow state at every pixel

Travel moves between segments shown as grey; print moves colored by pump.

v7.2.7 — inserted between Print Monitor/Results and Settings.
"""

from __future__ import annotations

import logging
import os
from pathlib import Path

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QPushButton, QLabel, QDoubleSpinBox, QSpinBox,
    QCheckBox, QGroupBox, QFileDialog, QSplitter,
    QScrollArea, QFrame, QSizePolicy, QMessageBox,
    QComboBox, QListWidget, QListWidgetItem, QTabWidget,
)
from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QPixmap, QImage, QPainter, QColor, QPen, QFont

try:
    from gui.styles import COLORS
except ImportError:
    COLORS = {
        "base": "#1e1e2e", "text": "#cdd6f4", "surface0": "#313244",
        "surface1": "#45475a", "blue": "#89b4fa", "green": "#a6e3a1",
        "red": "#f38ba8", "yellow": "#f9e2af", "mauve": "#cba6f7",
    }

try:
    import numpy as np
    HAS_NUMPY = True
except ImportError:
    HAS_NUMPY = False

logger = logging.getLogger(__name__)

_ImagePathPlanner = None


def _get_planner_class():
    global _ImagePathPlanner
    if _ImagePathPlanner is None:
        try:
            from SupportClasses.ImagePathPlanner import ImagePathPlanner
            _ImagePathPlanner = ImagePathPlanner
        except ImportError:
            logger.warning("ImagePathPlanner not available")
            _ImagePathPlanner = False
    return _ImagePathPlanner if _ImagePathPlanner is not False else None


PUMP_COLORS = [
    QColor("#f38ba8"),  # P1 red/pink
    QColor("#a6e3a1"),  # P2 green
    QColor("#89b4fa"),  # P3 blue
]
PUMP_LABELS = ["Pump 1 (Red)", "Pump 2 (Green)", "Pump 3 (Blue)"]

IMAGE_FILTER = "Images (*.png *.jpg *.jpeg *.bmp *.tiff *.tif);;All Files (*)"
TIFF_FILTER = "TIFF Stacks (*.tiff *.tif);;All Files (*)"


# ═══════════════════════════════════════════════════════════════════
#  Path Preview Widget
# ═══════════════════════════════════════════════════════════════════

class _PathPreviewWidget(QWidget):
    """2D preview of the generated raster toolpath with travel detection."""

    TRAVEL_COLOR = QColor(100, 100, 100, 60)  # Dim grey for travel moves

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(200, 200)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self._waypoints = None
        self._pump_states = None
        self._image_size = (0, 0)

    def set_data(self, waypoints, image_size_um, pump_states=None):
        self._waypoints = waypoints
        self._image_size = image_size_um
        self._pump_states = pump_states
        self.update()

    def clear(self):
        self._waypoints = None
        self._pump_states = None
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        painter.fillRect(0, 0, w, h, QColor(COLORS.get("base", "#1e1e2e")))

        wps = self._waypoints
        states = self._pump_states

        if wps is None or len(wps) < 2:
            painter.setPen(QColor(COLORS.get("text", "#cdd6f4")))
            painter.drawText(self.rect(), Qt.AlignCenter,
                             "Generate a toolpath to see preview")
            painter.end()
            return

        img_w, img_h = self._image_size
        if img_w <= 0 or img_h <= 0:
            painter.end()
            return

        margin = 20
        scale = min((w - 2 * margin) / img_w, (h - 2 * margin) / img_h)
        ox = margin + ((w - 2 * margin) - img_w * scale) / 2
        oy = margin + ((h - 2 * margin) - img_h * scale) / 2

        # Image boundary
        painter.setPen(QPen(QColor(COLORS.get("surface1", "#45475a")), 1, Qt.DashLine))
        painter.drawRect(int(ox), int(oy), int(img_w * scale), int(img_h * scale))

        # Draw each segment between consecutive waypoints
        for i in range(len(wps) - 1):
            wp_a = wps[i]
            wp_b = wps[i + 1]

            x1 = ox + wp_a[0] * scale
            y1 = oy + wp_a[1] * scale
            x2 = ox + wp_b[0] * scale
            y2 = oy + wp_b[1] * scale

            # Determine color from pump states at this waypoint
            color = self.TRAVEL_COLOR
            line_width = 0.5

            if states and i < len(states):
                st = states[i]
                total_flow = sum(f for f in st if f > 0)

                if total_flow > 0:
                    # Active print segment — color by pump contribution
                    r = g = b = 0.0
                    for j in range(min(3, len(st))):
                        frac = st[j]
                        if frac > 0:
                            c = PUMP_COLORS[j]
                            r += c.redF() * frac
                            g += c.greenF() * frac
                            b += c.blueF() * frac
                    # Normalize if needed
                    max_c = max(r, g, b, 1.0)
                    color = QColor.fromRgbF(
                        min(r / max_c, 1.0),
                        min(g / max_c, 1.0),
                        min(b / max_c, 1.0),
                        0.85,
                    )
                    line_width = 1.5
                # else: total_flow == 0 → travel move, keep grey

            painter.setPen(QPen(color, line_width))
            painter.drawLine(int(x1), int(y1), int(x2), int(y2))

        painter.end()


# ═══════════════════════════════════════════════════════════════════
#  Helper Functions Page
# ═══════════════════════════════════════════════════════════════════

class HelperFunctionsPage(QWidget):
    """
    Helper Functions page — Image-to-Toolpath generator.

    Three input modes:
      A) TIFF Stack — multi-frame TIFF with RGB channels
      B) Numbered Sequence — multi-select per pump
      C) Single Image — one image per pump, repeated N layers
    """

    print_file_created = Signal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._hw_config = None
        self._planner = None
        self._build_ui()

    # ── Page Interface ────────────────────────────────────────────

    def get_page_title(self) -> str:
        return "Helper Functions"

    def get_context_widget(self) -> QWidget | None:
        return None

    def on_status_update(self):
        pass

    def set_hardware_config(self, config):
        self._hw_config = config

    def set_microsteps_per_micron(self, value: float):
        pass

    # ══════════════════════════════════════════════════════════════
    #  UI CONSTRUCTION
    # ══════════════════════════════════════════════════════════════

    def _build_ui(self):
        _bg = COLORS.get('base', '#1e1e2e')
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)

        splitter = QSplitter(Qt.Horizontal)
        splitter.setChildrenCollapsible(False)

        # ── Left Panel ────────────────────────────────────────────
        left_scroll = QScrollArea()
        left_scroll.setWidgetResizable(True)
        left_scroll.setMinimumWidth(360)
        left_scroll.setMaximumWidth(460)
        left_scroll.setStyleSheet(f"QScrollArea {{ background-color: {_bg}; border: none; }}")
        left_widget = QWidget()
        left_widget.setStyleSheet(f"background-color: {_bg};")
        left_layout = QVBoxLayout(left_widget)
        left_layout.setSpacing(8)
        left_layout.setContentsMargins(12, 12, 12, 12)

        title = QLabel("Image -> Toolpath Generator")
        title.setFont(QFont("Segoe UI", 13, QFont.Bold))
        title.setStyleSheet(f"color: {COLORS.get('text', '#cdd6f4')};")
        left_layout.addWidget(title)

        desc = QLabel(
            "Rasters within black-pixel boundaries of all pump images.\n"
            "Pumps turn on/off at pixel edges.\n"
            "Dark pixels = high flow, white = no flow."
        )
        desc.setWordWrap(True)
        desc.setStyleSheet(f"color: {COLORS.get('surface1', '#45475a')}; font-size: 11px;")
        left_layout.addWidget(desc)

        self._build_input_tabs(left_layout)
        self._build_params_section(left_layout)
        self._build_actions_section(left_layout)

        self._status_label = QLabel("")
        self._status_label.setWordWrap(True)
        self._status_label.setStyleSheet(f"color: {COLORS.get('text', '#cdd6f4')}; font-size: 11px;")
        left_layout.addWidget(self._status_label)

        left_layout.addStretch()
        left_scroll.setWidget(left_widget)
        splitter.addWidget(left_scroll)

        # ── Right Panel ───────────────────────────────────────────
        right_widget = QWidget()
        right_widget.setStyleSheet(f"background-color: {_bg};")
        right_layout = QVBoxLayout(right_widget)
        right_layout.setSpacing(8)
        right_layout.setContentsMargins(8, 8, 8, 8)

        # Thumbnails
        thumb_label = QLabel("Loaded Images")
        thumb_label.setFont(QFont("Segoe UI", 11, QFont.Bold))
        thumb_label.setStyleSheet(f"color: {COLORS.get('text', '#cdd6f4')};")
        right_layout.addWidget(thumb_label)

        thumb_row = QHBoxLayout()
        self._thumb_labels: list[QLabel] = []
        for i in range(3):
            frame = QFrame()
            frame.setFrameShape(QFrame.StyledPanel)
            frame.setStyleSheet(
                f"QFrame {{ background: {COLORS.get('surface0', '#313244')}; "
                f"border: 1px solid {COLORS.get('surface1', '#45475a')}; "
                f"border-radius: 4px; }}"
            )
            frame.setMinimumSize(120, 120)
            frame.setMaximumSize(200, 200)
            fl = QVBoxLayout(frame)
            fl.setContentsMargins(4, 4, 4, 4)
            lbl = QLabel(f"P{i + 1}: No image")
            lbl.setAlignment(Qt.AlignCenter)
            lbl.setStyleSheet(f"color: {PUMP_COLORS[i].name()}; font-size: 10px;")
            fl.addWidget(lbl)
            self._thumb_labels.append(lbl)
            thumb_row.addWidget(frame)
        right_layout.addLayout(thumb_row)

        # Path preview
        plbl = QLabel("Toolpath Preview")
        plbl.setFont(QFont("Segoe UI", 11, QFont.Bold))
        plbl.setStyleSheet(f"color: {COLORS.get('text', '#cdd6f4')};")
        right_layout.addWidget(plbl)

        self._path_preview = _PathPreviewWidget()
        right_layout.addWidget(self._path_preview, 1)

        splitter.addWidget(right_widget)
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        outer.addWidget(splitter)

    # ── Input Mode Tabs ───────────────────────────────────────────

    def _build_input_tabs(self, parent_layout: QVBoxLayout):
        group = QGroupBox("Image Input")
        group.setStyleSheet(
            f"QGroupBox {{ color: {COLORS.get('text', '#cdd6f4')}; font-weight: bold; "
            f"border: 1px solid {COLORS.get('surface1', '#45475a')}; "
            f"border-radius: 4px; margin-top: 6px; padding-top: 14px; }}"
        )
        layout = QVBoxLayout(group)

        self._input_tabs = QTabWidget()
        self._input_tabs.setStyleSheet(
            f"QTabBar::tab {{ color: {COLORS.get('text', '#cdd6f4')}; "
            f"padding: 4px 10px; }} "
            f"QTabBar::tab:selected {{ background: {COLORS.get('surface0', '#313244')}; }}"
        )

        # Tab A: TIFF Stack
        tab_tiff = QWidget()
        tl = QVBoxLayout(tab_tiff)
        tl.setSpacing(6)
        tl.addWidget(QLabel("Load a multi-frame TIFF.\nRGB -> pumps, frames -> layers."))

        tiff_row = QHBoxLayout()
        self._tiff_path_label = QLabel("No file")
        self._tiff_path_label.setWordWrap(True)
        self._tiff_path_label.setStyleSheet(f"color: {COLORS.get('text', '#cdd6f4')}; font-size: 10px;")
        btn_tiff = QPushButton("Browse TIFF...")
        btn_tiff.clicked.connect(self._browse_tiff)
        tiff_row.addWidget(btn_tiff)
        tiff_row.addWidget(self._tiff_path_label, 1)
        tl.addLayout(tiff_row)

        self._tiff_pump_combo = QComboBox()
        self._tiff_pump_combo.addItem("Auto (RGB -> 3 pumps)", None)
        for i in range(3):
            self._tiff_pump_combo.addItem(f"All frames -> Pump {i + 1}", i)
        tl.addWidget(QLabel("Channel assignment:"))
        tl.addWidget(self._tiff_pump_combo)
        tl.addStretch()
        self._input_tabs.addTab(tab_tiff, "A: TIFF Stack")

        # Tab B: Numbered Sequence
        tab_seq = QWidget()
        sl = QVBoxLayout(tab_seq)
        sl.setSpacing(6)
        sl.addWidget(QLabel(
            "Multi-select images per pump (layer order).\n"
            "e.g. color1_01.png, color1_02.png, ..."
        ))

        self._seq_lists: list[QListWidget] = []
        self._seq_paths: list[list[str]] = [[], [], []]
        for i in range(3):
            row = QHBoxLayout()
            btn = QPushButton(f"P{i + 1} Browse...")
            btn.setStyleSheet(f"color: {PUMP_COLORS[i].name()};")
            btn.clicked.connect(lambda checked, idx=i: self._browse_sequence(idx))
            row.addWidget(btn)
            btn_clear = QPushButton("x")
            btn_clear.setFixedWidth(28)
            btn_clear.clicked.connect(lambda checked, idx=i: self._clear_sequence(idx))
            row.addWidget(btn_clear)
            lw = QListWidget()
            lw.setMaximumHeight(60)
            lw.setStyleSheet(
                f"QListWidget {{ background: {COLORS.get('surface0', '#313244')}; "
                f"color: {COLORS.get('text', '#cdd6f4')}; font-size: 9px; }}"
            )
            self._seq_lists.append(lw)
            sl.addLayout(row)
            sl.addWidget(lw)
        sl.addStretch()
        self._input_tabs.addTab(tab_seq, "B: Sequence")

        # Tab C: Single Images
        tab_single = QWidget()
        cl = QVBoxLayout(tab_single)
        cl.setSpacing(6)
        cl.addWidget(QLabel("One image per pump, repeated for N layers."))

        self._single_paths: list[str | None] = [None, None, None]
        self._single_labels: list[QLabel] = []
        for i in range(3):
            row = QHBoxLayout()
            btn = QPushButton(f"P{i + 1} Browse...")
            btn.setStyleSheet(f"color: {PUMP_COLORS[i].name()};")
            btn.clicked.connect(lambda checked, idx=i: self._browse_single(idx))
            row.addWidget(btn)
            btn_clear = QPushButton("x")
            btn_clear.setFixedWidth(28)
            btn_clear.clicked.connect(lambda checked, idx=i: self._clear_single(idx))
            row.addWidget(btn_clear)
            lbl = QLabel("No file")
            lbl.setStyleSheet(f"color: {COLORS.get('text', '#cdd6f4')}; font-size: 10px;")
            lbl.setWordWrap(True)
            row.addWidget(lbl, 1)
            self._single_labels.append(lbl)
            cl.addLayout(row)

        cl.addWidget(QLabel("Number of layers:"))
        self._spin_num_layers = QSpinBox()
        self._spin_num_layers.setRange(1, 1000)
        self._spin_num_layers.setValue(1)
        cl.addWidget(self._spin_num_layers)
        cl.addStretch()
        self._input_tabs.addTab(tab_single, "C: Single")

        layout.addWidget(self._input_tabs)
        parent_layout.addWidget(group)

    # ── Parameters ────────────────────────────────────────────────

    def _build_params_section(self, parent_layout: QVBoxLayout):
        group = QGroupBox("Parameters")
        group.setStyleSheet(
            f"QGroupBox {{ color: {COLORS.get('text', '#cdd6f4')}; font-weight: bold; "
            f"border: 1px solid {COLORS.get('surface1', '#45475a')}; "
            f"border-radius: 4px; margin-top: 6px; padding-top: 14px; }}"
        )
        grid = QGridLayout(group)
        grid.setSpacing(6)
        row = 0

        def _dspin(label, default, lo, hi, dec=1, suffix=""):
            nonlocal row
            grid.addWidget(QLabel(label), row, 0)
            s = QDoubleSpinBox(); s.setRange(lo, hi); s.setValue(default); s.setDecimals(dec)
            if suffix: s.setSuffix(f" {suffix}")
            grid.addWidget(s, row, 1); row += 1; return s

        def _ispin(label, default, lo, hi, suffix=""):
            nonlocal row
            grid.addWidget(QLabel(label), row, 0)
            s = QSpinBox(); s.setRange(lo, hi); s.setValue(default)
            if suffix: s.setSuffix(f" {suffix}")
            grid.addWidget(s, row, 1); row += 1; return s

        self._spin_threshold = _ispin("Threshold:", 128, 0, 255)
        self._spin_tool_diam = _dspin("Tool Diameter:", 300.0, 1.0, 10000.0, 1, "um")
        self._spin_overlap = _dspin("Path Overlap:", 0.8, 0.01, 1.0, 2)

        # Sizing mode: manual pixels/µm vs target footprint
        grid.addWidget(QLabel("Sizing Mode:"), row, 0)
        self._sizing_combo = QComboBox()
        self._sizing_combo.addItem("Pixels/µm (manual)", "manual")
        self._sizing_combo.addItem("Target Footprint (mm)", "footprint")
        self._sizing_combo.setCurrentIndex(1)  # Default to footprint
        self._sizing_combo.currentIndexChanged.connect(self._on_sizing_mode_changed)
        grid.addWidget(self._sizing_combo, row, 1); row += 1

        # Manual: pixels/µm
        self._spin_px_per_um = _dspin("Pixels/µm:", 0.09, 0.001, 10.0, 3)
        self._lbl_px_per_um = grid.itemAtPosition(row - 1, 0).widget()

        # Target footprint: width/height in mm
        self._spin_footprint_w = _dspin("Footprint Width:", 4.0, 0.1, 100.0, 2, "mm")
        self._lbl_footprint_w = grid.itemAtPosition(row - 1, 0).widget()
        self._spin_footprint_h = _dspin("Footprint Height:", 4.0, 0.1, 100.0, 2, "mm")
        self._lbl_footprint_h = grid.itemAtPosition(row - 1, 0).widget()

        # Trigger initial visibility
        self._on_sizing_mode_changed()

        self._spin_feedrate = _dspin("Feedrate:", 1000.0, 1.0, 100000.0, 0, "um/s")
        self._spin_corner_slow = _dspin("Corner Slowdown:", 0.5, 0.01, 1.0, 2)
        self._spin_corner_angle = _dspin("Corner Angle:", 120.0, 0.0, 180.0, 0, "deg")
        self._spin_flow_factor = _dspin("Flow Factor:", 0.01, 0.0001, 10.0, 4)
        self._spin_z_start = _dspin("Z Start:", 0.0, -10000.0, 10000.0, 1, "um")
        self._spin_z_inc = _dspin("Z Increment:", 10.0, 0.0, 10000.0, 1, "um")

        self._chk_invert = QCheckBox("Invert (dark = high flow)")
        self._chk_invert.setChecked(True)
        self._chk_invert.setStyleSheet(f"color: {COLORS.get('text', '#cdd6f4')};")
        grid.addWidget(self._chk_invert, row, 0, 1, 2)

        parent_layout.addWidget(group)

    # ── Actions ───────────────────────────────────────────────────

    def _build_actions_section(self, parent_layout: QVBoxLayout):
        group = QGroupBox("Actions")
        group.setStyleSheet(
            f"QGroupBox {{ color: {COLORS.get('text', '#cdd6f4')}; font-weight: bold; "
            f"border: 1px solid {COLORS.get('surface1', '#45475a')}; "
            f"border-radius: 4px; margin-top: 6px; padding-top: 14px; }}"
        )
        layout = QVBoxLayout(group)
        layout.setSpacing(6)

        btn_gen = QPushButton("Generate Toolpath")
        btn_gen.setMinimumHeight(36)
        btn_gen.setStyleSheet(
            f"QPushButton {{ background: {COLORS.get('blue', '#89b4fa')}; "
            f"color: {COLORS.get('base', '#1e1e2e')}; font-weight: bold; "
            f"border-radius: 4px; padding: 6px; }}"
        )
        btn_gen.clicked.connect(self._generate)
        layout.addWidget(btn_gen)

        row = QHBoxLayout()
        btn_csv = QPushButton("Save as CSV")
        btn_csv.clicked.connect(self._save_csv)
        row.addWidget(btn_csv)
        btn_print = QPushButton("Load as Print Object")
        btn_print.setStyleSheet(
            f"QPushButton {{ background: {COLORS.get('green', '#a6e3a1')}; "
            f"color: {COLORS.get('base', '#1e1e2e')}; font-weight: bold; "
            f"border-radius: 4px; padding: 4px; }}"
        )
        btn_print.clicked.connect(self._load_as_print_object)
        row.addWidget(btn_print)
        layout.addLayout(row)
        parent_layout.addWidget(group)

    # ── Sizing Mode Toggle ─────────────────────────────────────────

    def _on_sizing_mode_changed(self):
        """Show/hide sizing controls based on selected mode."""
        is_footprint = (self._sizing_combo.currentData() == "footprint")
        self._spin_px_per_um.setVisible(not is_footprint)
        self._lbl_px_per_um.setVisible(not is_footprint)
        self._spin_footprint_w.setVisible(is_footprint)
        self._lbl_footprint_w.setVisible(is_footprint)
        self._spin_footprint_h.setVisible(is_footprint)
        self._lbl_footprint_h.setVisible(is_footprint)

    # ══════════════════════════════════════════════════════════════
    #  FILE BROWSING
    # ══════════════════════════════════════════════════════════════

    def _browse_tiff(self):
        path, _ = QFileDialog.getOpenFileName(self, "Select TIFF Stack", "", TIFF_FILTER)
        if path:
            self._tiff_path_label.setText(os.path.basename(path))
            self._tiff_path_label.setProperty("full_path", path)
            self._update_thumbnail(0, path)
            self._status_label.setText(f"TIFF loaded: {os.path.basename(path)}")

    def _browse_sequence(self, pump_idx: int):
        paths, _ = QFileDialog.getOpenFileNames(
            self, f"Select Pump {pump_idx + 1} Layer Images", "", IMAGE_FILTER)
        if paths:
            paths = sorted(paths, key=lambda p: _natural_sort_key(p))
            self._seq_paths[pump_idx] = paths
            lw = self._seq_lists[pump_idx]
            lw.clear()
            for p in paths:
                lw.addItem(os.path.basename(p))
            if paths:
                self._update_thumbnail(pump_idx, paths[0])
            self._status_label.setText(f"P{pump_idx + 1}: {len(paths)} layer(s)")

    def _clear_sequence(self, pump_idx: int):
        self._seq_paths[pump_idx] = []
        self._seq_lists[pump_idx].clear()
        self._thumb_labels[pump_idx].setPixmap(QPixmap())
        self._thumb_labels[pump_idx].setText(f"P{pump_idx + 1}: No image")

    def _browse_single(self, pump_idx: int):
        path, _ = QFileDialog.getOpenFileName(
            self, f"Select Pump {pump_idx + 1} Image", "", IMAGE_FILTER)
        if path:
            self._single_paths[pump_idx] = path
            self._single_labels[pump_idx].setText(os.path.basename(path))
            self._update_thumbnail(pump_idx, path)

    def _clear_single(self, pump_idx: int):
        self._single_paths[pump_idx] = None
        self._single_labels[pump_idx].setText("No file")
        self._thumb_labels[pump_idx].setPixmap(QPixmap())
        self._thumb_labels[pump_idx].setText(f"P{pump_idx + 1}: No image")

    def _update_thumbnail(self, pump_idx: int, path: str):
        try:
            pixmap = QPixmap(path)
            if pixmap.isNull():
                self._thumb_labels[pump_idx].setText(f"P{pump_idx + 1}: load error")
                return
            scaled = pixmap.scaled(160, 160, Qt.KeepAspectRatio, Qt.SmoothTransformation)
            self._thumb_labels[pump_idx].setPixmap(scaled)
            self._thumb_labels[pump_idx].setText("")
        except Exception as e:
            self._thumb_labels[pump_idx].setText(f"P{pump_idx + 1}: Error")
            logger.warning(f"Thumbnail failed: {e}")

    # ══════════════════════════════════════════════════════════════
    #  GENERATION
    # ══════════════════════════════════════════════════════════════

    def _make_planner(self):
        PlannerClass = _get_planner_class()
        if PlannerClass is None:
            QMessageBox.critical(self, "Missing Module",
                "ImagePathPlanner not available.\nEnsure SupportClasses/ImagePathPlanner.py exists.")
            return None

        # Determine sizing: target footprint (mm→µm) or manual pixels/µm
        target_w_um = None
        target_h_um = None
        if self._sizing_combo.currentData() == "footprint":
            target_w_um = self._spin_footprint_w.value() * 1000.0  # mm → µm
            target_h_um = self._spin_footprint_h.value() * 1000.0

        return PlannerClass(
            threshold=self._spin_threshold.value(),
            tool_diameter=self._spin_tool_diam.value(),
            path_overlap=self._spin_overlap.value(),
            pixels_per_micron=self._spin_px_per_um.value(),
            base_feedrate=self._spin_feedrate.value(),
            corner_slowdown_factor=self._spin_corner_slow.value(),
            corner_angle_threshold=self._spin_corner_angle.value(),
            flow_factor=self._spin_flow_factor.value(),
            initial_z=self._spin_z_start.value(),
            z_increment=self._spin_z_inc.value(),
            invert=self._chk_invert.isChecked(),
            target_width_um=target_w_um,
            target_height_um=target_h_um,
        )

    def _load_images_into_planner(self, planner) -> bool:
        tab_idx = self._input_tabs.currentIndex()
        try:
            if tab_idx == 0:
                tiff_path = self._tiff_path_label.property("full_path")
                if not tiff_path:
                    QMessageBox.warning(self, "No TIFF", "Select a TIFF file first.")
                    return False
                planner.load_tiff_stack(tiff_path, pump_index=self._tiff_pump_combo.currentData())
            elif tab_idx == 1:
                any_loaded = False
                for i in range(3):
                    if self._seq_paths[i]:
                        planner.load_image_sequence(self._seq_paths[i], i)
                        any_loaded = True
                if not any_loaded:
                    QMessageBox.warning(self, "No Images", "Select images for at least one pump.")
                    return False
            elif tab_idx == 2:
                n = self._spin_num_layers.value()
                any_loaded = False
                for i in range(3):
                    if self._single_paths[i]:
                        planner.load_single_image(self._single_paths[i], i, n)
                        any_loaded = True
                if not any_loaded:
                    QMessageBox.warning(self, "No Images", "Select at least one pump image.")
                    return False
            return True
        except (FileNotFoundError, ValueError, RuntimeError) as e:
            QMessageBox.critical(self, "Load Error", str(e))
            return False

    def _generate(self):
        planner = self._make_planner()
        if planner is None:
            return
        if not self._load_images_into_planner(planner):
            return
        try:
            planner.generate_toolpath()
            self._planner = planner

            summary = planner.get_summary()
            # Waypoints are in mm; use mm image size for preview
            self._path_preview.set_data(
                planner.waypoints,
                (summary["image_width_mm"], summary["image_height_mm"]),
                planner.pump_states_all,
            )

            disps = summary["pump_displacements"]
            disp_parts = [f"P{i+1}={d:.2f}" for i, d in enumerate(disps) if d > 0]
            w_mm = summary["image_width_mm"]
            h_mm = summary["image_height_mm"]
            self._status_label.setText(
                f"OK: {summary['num_waypoints']} waypoints | "
                f"{summary['num_layers']} layer(s) | "
                f"~{summary['estimated_time_s']:.1f}s\n"
                f"Footprint: {w_mm:.2f} x {h_mm:.2f} mm | "
                f"Pump disp: {', '.join(disp_parts) or 'none'}"
            )
            self._status_label.setStyleSheet(
                f"color: {COLORS.get('green', '#a6e3a1')}; font-size: 11px;")
        except Exception as e:
            logger.error(f"Generation failed: {e}", exc_info=True)
            QMessageBox.critical(self, "Error", f"Generation failed:\n{e}")

    # ══════════════════════════════════════════════════════════════
    #  EXPORT
    # ══════════════════════════════════════════════════════════════

    def _save_csv(self):
        if self._planner is None or not self._planner.waypoints:
            QMessageBox.warning(self, "No Toolpath", "Generate a toolpath first.")
            return
        path, _ = QFileDialog.getSaveFileName(
            self, "Save Toolpath CSV", "image_toolpath.csv", "CSV (*.csv);;All (*)")
        if path:
            try:
                self._planner.save_csv(path)
                self._status_label.setText(f"Saved to {path}")
            except Exception as e:
                QMessageBox.critical(self, "Save Error", str(e))

    def _load_as_print_object(self):
        if self._planner is None or not self._planner.waypoints:
            QMessageBox.warning(self, "No Toolpath", "Generate a toolpath first.")
            return
        try:
            import json
            from datetime import datetime, timezone

            prints_dir = Path("config/prints")
            prints_dir.mkdir(parents=True, exist_ok=True)

            base = "ImageToolpath"
            counter = 1
            while (prints_dir / f"{base}_{counter}.csv").exists():
                counter += 1
            name = f"{base}_{counter}"
            csv_path = prints_dir / f"{name}.csv"
            self._planner.save_csv(str(csv_path))

            summary = self._planner.get_summary()
            print_data = {
                "schema_version": "7.2.3",
                "metadata": {
                    "name": name,
                    "description": (
                        f"Image toolpath: {summary['num_waypoints']} wpts, "
                        f"{summary['num_layers']} layers, "
                        f"{summary['image_width_mm']:.2f}x"
                        f"{summary['image_height_mm']:.2f} mm"
                    ),
                    "created": datetime.now(timezone.utc).isoformat(),
                    "modified": datetime.now(timezone.utc).isoformat(),
                    "author": "Helper Functions",
                },
                "objects": {
                    "ImagePath_1": {
                        "object_type": "csv_import",
                        "params": {"csv_path": str(csv_path), "source": "ImagePathPlanner",
                                   "num_waypoints": summary["num_waypoints"],
                                   "num_layers": summary["num_layers"]},
                        "position": [0.0, 0.0, 0.0], "color": "#a6e3a1",
                        "ink": "", "in_well": True,
                    },
                },
                "collections": {}, "layout_presets": {},
            }
            json_path = prints_dir / f"{name}.json"
            with open(json_path, "w") as f:
                json.dump(print_data, f, indent=2)

            self._status_label.setText(f"Created: {name}")
            self._status_label.setStyleSheet(
                f"color: {COLORS.get('green', '#a6e3a1')}; font-size: 11px;")
            self.print_file_created.emit(name)
        except Exception as e:
            logger.error(f"Create print object failed: {e}", exc_info=True)
            QMessageBox.critical(self, "Error", f"Failed:\n{e}")


# ═══════════════════════════════════════════════════════════════════
import re as _re_module

def _natural_sort_key(path: str):
    basename = os.path.basename(path)
    parts = _re_module.split(r'(\d+)', basename)
    return [int(p) if p.isdigit() else p.lower() for p in parts]

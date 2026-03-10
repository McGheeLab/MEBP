"""
print_results.py — Print Results Diagnostic Page for MEBP v7.2.6.

Post-print analysis page providing:
- Ideal (planned) vs actual path comparison in XY/XZ/YZ projections
- Animated playback simulation with speed control and scrub bar
- Comprehensive tracking error statistics and per-segment breakdown
- Error-over-time chart with playback cursor
- Display filters (print/travel/retract, segment selection)
- Export capabilities (CSV data, summary report)

Data source: PrintRecorder JSON+CSV recordings from print_records/ directory.

Layout:
    Main content = Path view + Error chart + Playback bar + Statistics
    Context panel = Recording selector, filters, workspace info, export

Interface contract:
    get_page_title()     → str
    get_context_widget() → QWidget
    on_status_update()   → called by MainWindow timer
    set_hardware_config  → receives HardwareConfig from app.py
    set_recorder(rec)    → receives PrintRecorder reference
    load_latest_recording() → auto-load most recent recording
"""

from __future__ import annotations

import logging
import math
from pathlib import Path
from typing import Any

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QPushButton, QLabel, QComboBox, QFrame,
    QCheckBox, QGroupBox, QSizePolicy, QSplitter,
    QTableWidget, QTableWidgetItem, QHeaderView,
    QAbstractItemView, QListWidget, QListWidgetItem,
    QSlider, QScrollArea, QStackedWidget,
)
from PySide6.QtCore import Qt, Signal, QTimer, QRectF, QPointF
from PySide6.QtGui import (
    QPainter, QPen, QColor, QBrush, QFont,
    QPainterPath, QLinearGradient, QMouseEvent, QWheelEvent,
)

from gui.styles import COLORS

logger = logging.getLogger(__name__)

# ── Constants ─────────────────────────────────────────────────────

ERROR_COLOR_GOOD = QColor("#a6e3a1")       # green  < 20µm
ERROR_COLOR_WARN = QColor("#f9e2af")       # yellow 20-50µm
ERROR_COLOR_BAD = QColor("#f38ba8")        # red    > 50µm
PLANNED_COLOR = QColor("#89b4fa")          # blue
TRAVEL_COLOR = QColor("#6c7086")           # overlay0 gray
CURSOR_COLOR = QColor("#cba6f7")           # mauve
RETRACT_COLOR = QColor("#fab387")          # peach

ERROR_THRESH_GOOD_UM = 20.0
ERROR_THRESH_WARN_UM = 50.0

MAX_DISPLAY_POINTS = 5000  # downsample for rendering performance
PLAYBACK_FPS = 30


def _error_to_color(error_mm: float) -> QColor:
    """Map tracking error (mm) to a green→yellow→red color."""
    error_um = error_mm * 1000.0
    if error_um <= ERROR_THRESH_GOOD_UM:
        t = error_um / ERROR_THRESH_GOOD_UM
        r = int(ERROR_COLOR_GOOD.red() + t * (ERROR_COLOR_WARN.red() - ERROR_COLOR_GOOD.red()))
        g = int(ERROR_COLOR_GOOD.green() + t * (ERROR_COLOR_WARN.green() - ERROR_COLOR_GOOD.green()))
        b = int(ERROR_COLOR_GOOD.blue() + t * (ERROR_COLOR_WARN.blue() - ERROR_COLOR_GOOD.blue()))
        return QColor(r, g, b)
    elif error_um <= ERROR_THRESH_WARN_UM:
        t = (error_um - ERROR_THRESH_GOOD_UM) / (ERROR_THRESH_WARN_UM - ERROR_THRESH_GOOD_UM)
        r = int(ERROR_COLOR_WARN.red() + t * (ERROR_COLOR_BAD.red() - ERROR_COLOR_WARN.red()))
        g = int(ERROR_COLOR_WARN.green() + t * (ERROR_COLOR_BAD.green() - ERROR_COLOR_WARN.green()))
        b = int(ERROR_COLOR_WARN.blue() + t * (ERROR_COLOR_BAD.blue() - ERROR_COLOR_WARN.blue()))
        return QColor(r, g, b)
    else:
        return QColor(ERROR_COLOR_BAD)


def _downsample(samples: list[dict], max_points: int) -> list[dict]:
    """Downsample a list of sample dicts to max_points for display."""
    if len(samples) <= max_points:
        return samples
    step = len(samples) / max_points
    return [samples[int(i * step)] for i in range(max_points)]


# ═══════════════════════════════════════════════════════════════════
# Path Comparison Widget — QPainter-based XY/XZ/YZ overlay
# ═══════════════════════════════════════════════════════════════════

class PathComparisonWidget(QWidget):
    """
    Draws planned vs actual paths with error-colored actual path.

    Supports XY, XZ, YZ projection modes.
    Zoomable (scroll wheel) and pannable (middle-click drag).
    Shows playback cursor at the current sample index.
    """

    hover_sample = Signal(int)  # emitted on mouse hover with nearest sample idx

    def __init__(self, parent=None):
        super().__init__(parent)
        self._samples: list[dict] = []
        self._display_samples: list[dict] = []
        self._projection = "XY"  # "XY", "XZ", "YZ"
        self._cursor_idx = -1
        self._show_travel = True
        self._show_retract = False
        self._show_planned = True
        self._segment_filter = -1  # -1 = all segments

        # View transform
        self._zoom = 1.0
        self._pan_x = 0.0
        self._pan_y = 0.0
        self._dragging = False
        self._drag_start = QPointF()
        self._drag_pan_start = (0.0, 0.0)

        self.setMinimumSize(300, 250)
        self.setMouseTracking(True)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setStyleSheet(f"background-color: {COLORS['crust']};")

    def set_data(self, samples: list[dict]):
        """Set sample data and refresh display."""
        self._samples = samples
        self._display_samples = _downsample(samples, MAX_DISPLAY_POINTS)
        self._auto_fit()
        self.update()

    def set_projection(self, proj: str):
        """Set projection mode: 'XY', 'XZ', or 'YZ'."""
        self._projection = proj
        self._auto_fit()
        self.update()

    def set_cursor(self, idx: int):
        """Set playback cursor position."""
        self._cursor_idx = idx
        self.update()

    def set_show_travel(self, show: bool):
        self._show_travel = show
        self.update()

    def set_show_retract(self, show: bool):
        self._show_retract = show
        self.update()

    def set_show_planned(self, show: bool):
        self._show_planned = show
        self.update()

    def set_segment_filter(self, seg_id: int):
        """Filter to specific segment (-1 = all)."""
        self._segment_filter = seg_id
        self.update()

    def _get_coords(self, sample: dict, prefix: str) -> tuple[float, float]:
        """Extract (h, v) coordinates based on projection and prefix."""
        if self._projection == "XY":
            return (sample.get(f"{prefix}_x", 0.0),
                    sample.get(f"{prefix}_y", 0.0))
        elif self._projection == "XZ":
            return (sample.get(f"{prefix}_x", 0.0),
                    sample.get(f"{prefix}_z", 0.0))
        else:  # YZ
            return (sample.get(f"{prefix}_y", 0.0),
                    sample.get(f"{prefix}_z", 0.0))

    def _auto_fit(self):
        """Auto-fit view to show all data."""
        if not self._display_samples:
            self._zoom = 1.0
            self._pan_x = 0.0
            self._pan_y = 0.0
            return

        min_h = min_v = float("inf")
        max_h = max_v = float("-inf")

        for s in self._display_samples:
            for prefix in ("planned", "actual"):
                h, v = self._get_coords(s, prefix)
                min_h = min(min_h, h)
                max_h = max(max_h, h)
                min_v = min(min_v, v)
                max_v = max(max_v, v)

        range_h = max_h - min_h if max_h > min_h else 1.0
        range_v = max_v - min_v if max_v > min_v else 1.0
        margin = 0.15

        w = self.width() - 60
        h = self.height() - 60
        if w <= 0 or h <= 0:
            return

        zoom_h = w / (range_h * (1 + 2 * margin)) if range_h > 0 else 1.0
        zoom_v = h / (range_v * (1 + 2 * margin)) if range_v > 0 else 1.0
        self._zoom = min(zoom_h, zoom_v)

        center_h = (min_h + max_h) / 2.0
        center_v = (min_v + max_v) / 2.0
        self._pan_x = self.width() / 2.0 - center_h * self._zoom
        self._pan_y = self.height() / 2.0 + center_v * self._zoom

    def _to_screen(self, h: float, v: float) -> QPointF:
        """Convert data coordinates to screen pixel coordinates."""
        sx = h * self._zoom + self._pan_x
        sy = -v * self._zoom + self._pan_y  # flip Y for screen
        return QPointF(sx, sy)

    def _should_draw(self, sample: dict) -> bool:
        """Check if a sample passes current filters."""
        if self._segment_filter >= 0:
            if sample.get("segment_id", 0) != self._segment_filter:
                return False
        is_travel = sample.get("is_travel", False)
        is_retract = sample.get("is_retract", False)
        if is_travel and not self._show_travel:
            return False
        if is_retract and not self._show_retract:
            return False
        return True

    def paintEvent(self, event):
        """Custom paint: grid, planned path, actual path, cursor."""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        # Background
        painter.fillRect(self.rect(), QColor(COLORS["crust"]))

        if not self._display_samples:
            painter.setPen(QColor(COLORS["overlay0"]))
            painter.setFont(QFont("Segoe UI", 11))
            painter.drawText(self.rect(), Qt.AlignCenter,
                             "No recording loaded\nSelect a recording from the context panel")
            painter.end()
            return

        # Draw grid
        self._draw_grid(painter)

        # Draw planned path
        if self._show_planned:
            self._draw_path(painter, "planned", PLANNED_COLOR, Qt.DashLine, 1.5)

        # Draw actual path (error-colored)
        self._draw_actual_path(painter)

        # Draw playback cursor
        self._draw_cursor(painter)

        # Draw legend
        self._draw_legend(painter)

        # Draw axis labels
        self._draw_axis_labels(painter)

        painter.end()

    def _draw_grid(self, painter: QPainter):
        """Draw a subtle grid."""
        pen = QPen(QColor(COLORS["surface0"]), 1, Qt.DotLine)
        painter.setPen(pen)

        # Determine grid spacing based on zoom
        base_spacing = 1.0  # mm
        if self._zoom > 0:
            pixel_spacing = base_spacing * self._zoom
            while pixel_spacing < 40:
                base_spacing *= 2
                pixel_spacing = base_spacing * self._zoom
            while pixel_spacing > 200:
                base_spacing /= 2
                pixel_spacing = base_spacing * self._zoom

        w, h = self.width(), self.height()

        # Vertical lines
        start_h = -self._pan_x / self._zoom
        start_h = math.floor(start_h / base_spacing) * base_spacing
        current = start_h
        while True:
            sx = current * self._zoom + self._pan_x
            if sx > w:
                break
            if sx >= 0:
                painter.drawLine(int(sx), 0, int(sx), h)
            current += base_spacing

        # Horizontal lines
        start_v = (self._pan_y - h) / self._zoom
        start_v = math.floor(start_v / base_spacing) * base_spacing
        current = start_v
        while True:
            sy = -current * self._zoom + self._pan_y
            if sy > h:
                break
            if sy >= 0:
                painter.drawLine(0, int(sy), w, int(sy))
            current += base_spacing

    def _draw_path(self, painter: QPainter, prefix: str,
                   color: QColor, style, width: float):
        """Draw a complete path for a given prefix (planned/actual)."""
        pen = QPen(color, width, style)
        painter.setPen(pen)

        prev_pt = None
        for s in self._display_samples:
            if not self._should_draw(s):
                prev_pt = None
                continue
            h, v = self._get_coords(s, prefix)
            pt = self._to_screen(h, v)
            if prev_pt is not None:
                painter.drawLine(prev_pt, pt)
            prev_pt = pt

    def _draw_actual_path(self, painter: QPainter):
        """Draw actual path colored by tracking error magnitude."""
        prev_pt = None
        for s in self._display_samples:
            if not self._should_draw(s):
                prev_pt = None
                continue

            h, v = self._get_coords(s, "actual")
            pt = self._to_screen(h, v)

            if prev_pt is not None:
                err = s.get("tracking_error_xy", 0.0)
                color = _error_to_color(err)

                is_travel = s.get("is_travel", False)
                is_retract = s.get("is_retract", False)

                if is_travel:
                    pen = QPen(TRAVEL_COLOR, 1.0, Qt.DotLine)
                elif is_retract:
                    pen = QPen(RETRACT_COLOR, 1.5, Qt.DashDotLine)
                else:
                    pen = QPen(color, 2.0, Qt.SolidLine)

                painter.setPen(pen)
                painter.drawLine(prev_pt, pt)

            prev_pt = pt

    def _draw_cursor(self, painter: QPainter):
        """Draw playback cursor crosshair."""
        if self._cursor_idx < 0 or self._cursor_idx >= len(self._samples):
            return

        s = self._samples[self._cursor_idx]
        h_a, v_a = self._get_coords(s, "actual")
        pt = self._to_screen(h_a, v_a)

        # Crosshair
        pen = QPen(CURSOR_COLOR, 2.0)
        painter.setPen(pen)
        size = 12
        painter.drawLine(pt.x() - size, pt.y(), pt.x() + size, pt.y())
        painter.drawLine(pt.x(), pt.y() - size, pt.x(), pt.y() + size)

        # Circle
        painter.setBrush(Qt.NoBrush)
        painter.drawEllipse(pt, 6, 6)

        # Also show planned position
        h_p, v_p = self._get_coords(s, "planned")
        pt_p = self._to_screen(h_p, v_p)
        pen_p = QPen(PLANNED_COLOR, 1.5, Qt.DashLine)
        painter.setPen(pen_p)
        painter.drawEllipse(pt_p, 4, 4)

        # Connect planned to actual
        painter.drawLine(pt_p, pt)

    def _draw_legend(self, painter: QPainter):
        """Draw a small legend in top-right corner."""
        painter.setFont(QFont("Segoe UI", 8))
        x = self.width() - 155
        y = 10

        items = [
            (PLANNED_COLOR, "Planned path"),
            (ERROR_COLOR_GOOD, f"Error < {ERROR_THRESH_GOOD_UM:.0f}µm"),
            (ERROR_COLOR_WARN, f"Error {ERROR_THRESH_GOOD_UM:.0f}-{ERROR_THRESH_WARN_UM:.0f}µm"),
            (ERROR_COLOR_BAD, f"Error > {ERROR_THRESH_WARN_UM:.0f}µm"),
        ]
        if self._show_travel:
            items.append((TRAVEL_COLOR, "Travel"))

        bg = QColor(COLORS["surface0"])
        bg.setAlpha(200)
        painter.setBrush(bg)
        painter.setPen(Qt.NoPen)
        painter.drawRoundedRect(x - 5, y - 2, 160, len(items) * 18 + 6, 4, 4)

        for color, label in items:
            painter.setPen(QPen(color, 2))
            painter.drawLine(x, y + 8, x + 20, y + 8)
            painter.setPen(QColor(COLORS["text"]))
            painter.drawText(x + 25, y + 12, label)
            y += 18

    def _draw_axis_labels(self, painter: QPainter):
        """Draw axis labels based on projection."""
        axes = {"XY": ("X (mm)", "Y (mm)"),
                "XZ": ("X (mm)", "Z (mm)"),
                "YZ": ("Y (mm)", "Z (mm)")}
        h_label, v_label = axes.get(self._projection, ("H", "V"))

        painter.setPen(QColor(COLORS["subtext0"]))
        painter.setFont(QFont("Segoe UI", 9))
        # H axis label (bottom center)
        painter.drawText(self.width() // 2 - 20, self.height() - 5, h_label)
        # V axis label (left, rotated)
        painter.save()
        painter.translate(12, self.height() // 2 + 20)
        painter.rotate(-90)
        painter.drawText(0, 0, v_label)
        painter.restore()

    # ── Mouse interaction ─────────────────────────────────────────

    def wheelEvent(self, event: QWheelEvent):
        """Zoom with scroll wheel."""
        factor = 1.15 if event.angleDelta().y() > 0 else 1 / 1.15
        mouse_pos = event.position()

        # Zoom centered on mouse position
        self._pan_x = mouse_pos.x() - factor * (mouse_pos.x() - self._pan_x)
        self._pan_y = mouse_pos.y() - factor * (mouse_pos.y() - self._pan_y)
        self._zoom *= factor
        self.update()

    def mousePressEvent(self, event: QMouseEvent):
        if event.button() == Qt.MiddleButton:
            self._dragging = True
            self._drag_start = event.position()
            self._drag_pan_start = (self._pan_x, self._pan_y)

    def mouseMoveEvent(self, event: QMouseEvent):
        if self._dragging:
            delta = event.position() - self._drag_start
            self._pan_x = self._drag_pan_start[0] + delta.x()
            self._pan_y = self._drag_pan_start[1] + delta.y()
            self.update()

    def mouseReleaseEvent(self, event: QMouseEvent):
        if event.button() == Qt.MiddleButton:
            self._dragging = False

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if self._display_samples:
            self._auto_fit()


# ═══════════════════════════════════════════════════════════════════
# Error Time-Series Widget
# ═══════════════════════════════════════════════════════════════════

class ErrorTimeSeriesWidget(QWidget):
    """Chart showing tracking error vs time with playback cursor."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._samples: list[dict] = []
        self._display_samples: list[dict] = []
        self._cursor_idx = -1
        self._max_time = 1.0
        self._max_error_um = 100.0

        self.setMinimumSize(300, 120)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setStyleSheet(f"background-color: {COLORS['crust']};")

    def set_data(self, samples: list[dict]):
        self._samples = samples
        self._display_samples = _downsample(samples, MAX_DISPLAY_POINTS)

        if samples:
            self._max_time = max(s.get("t", 0) for s in samples)
            errors_xy = [s.get("tracking_error_xy", 0) * 1000 for s in samples]
            errors_z = [s.get("tracking_error_z", 0) * 1000 for s in samples]
            all_errors = errors_xy + errors_z
            self._max_error_um = max(all_errors) * 1.2 if all_errors else 100.0
            self._max_error_um = max(self._max_error_um, 10.0)  # minimum scale
        self.update()

    def set_cursor(self, idx: int):
        self._cursor_idx = idx
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.fillRect(self.rect(), QColor(COLORS["crust"]))

        if not self._display_samples:
            painter.setPen(QColor(COLORS["overlay0"]))
            painter.setFont(QFont("Segoe UI", 10))
            painter.drawText(self.rect(), Qt.AlignCenter, "No data")
            painter.end()
            return

        margin_l, margin_r, margin_t, margin_b = 50, 15, 15, 30
        plot_w = self.width() - margin_l - margin_r
        plot_h = self.height() - margin_t - margin_b

        if plot_w <= 0 or plot_h <= 0:
            painter.end()
            return

        def to_screen(t_val, err_um):
            sx = margin_l + (t_val / self._max_time) * plot_w if self._max_time > 0 else margin_l
            sy = margin_t + plot_h - (err_um / self._max_error_um) * plot_h
            return QPointF(sx, sy)

        # Grid lines
        pen_grid = QPen(QColor(COLORS["surface0"]), 1, Qt.DotLine)
        painter.setPen(pen_grid)
        for frac in (0.25, 0.5, 0.75, 1.0):
            y = margin_t + plot_h * (1 - frac)
            painter.drawLine(margin_l, int(y), margin_l + plot_w, int(y))

        # Axes
        pen_axis = QPen(QColor(COLORS["surface1"]), 1)
        painter.setPen(pen_axis)
        painter.drawLine(margin_l, margin_t, margin_l, margin_t + plot_h)
        painter.drawLine(margin_l, margin_t + plot_h,
                         margin_l + plot_w, margin_t + plot_h)

        # XY error line
        pen_xy = QPen(ERROR_COLOR_GOOD, 1.5)
        painter.setPen(pen_xy)
        prev = None
        for s in self._display_samples:
            t = s.get("t", 0)
            err = s.get("tracking_error_xy", 0) * 1000
            pt = to_screen(t, err)
            if prev is not None:
                painter.drawLine(prev, pt)
            prev = pt

        # Z error line
        pen_z = QPen(QColor("#89b4fa"), 1.5)
        painter.setPen(pen_z)
        prev = None
        for s in self._display_samples:
            t = s.get("t", 0)
            err = s.get("tracking_error_z", 0) * 1000
            pt = to_screen(t, err)
            if prev is not None:
                painter.drawLine(prev, pt)
            prev = pt

        # Playback cursor
        if 0 <= self._cursor_idx < len(self._samples):
            t_cur = self._samples[self._cursor_idx].get("t", 0)
            sx = margin_l + (t_cur / self._max_time) * plot_w if self._max_time > 0 else margin_l
            pen_c = QPen(CURSOR_COLOR, 1.5, Qt.DashLine)
            painter.setPen(pen_c)
            painter.drawLine(int(sx), margin_t, int(sx), margin_t + plot_h)

        # Labels
        painter.setPen(QColor(COLORS["subtext0"]))
        painter.setFont(QFont("Segoe UI", 8))
        painter.drawText(margin_l - 45, margin_t + 4, f"{self._max_error_um:.0f}µm")
        painter.drawText(margin_l - 25, margin_t + plot_h + 4, "0")
        painter.drawText(margin_l, margin_t + plot_h + 15, "0s")
        painter.drawText(margin_l + plot_w - 30, margin_t + plot_h + 15,
                         f"{self._max_time:.1f}s")

        # Legend
        painter.setFont(QFont("Segoe UI", 8))
        lx = margin_l + 10
        ly = margin_t + 5
        painter.setPen(QPen(ERROR_COLOR_GOOD, 2))
        painter.drawLine(lx, ly + 5, lx + 15, ly + 5)
        painter.setPen(QColor(COLORS["text"]))
        painter.drawText(lx + 20, ly + 9, "XY error")
        painter.setPen(QPen(QColor("#89b4fa"), 2))
        painter.drawLine(lx + 85, ly + 5, lx + 100, ly + 5)
        painter.setPen(QColor(COLORS["text"]))
        painter.drawText(lx + 105, ly + 9, "Z error")

        painter.end()


# ═══════════════════════════════════════════════════════════════════
# Playback Controller Bar
# ═══════════════════════════════════════════════════════════════════

class PlaybackController(QWidget):
    """Transport bar with play/pause, speed, scrub slider, time display."""

    sample_changed = Signal(int)  # emitted when current sample changes

    def __init__(self, parent=None):
        super().__init__(parent)
        self._total_samples = 0
        self._current_idx = 0
        self._playing = False
        self._speed = 1.0
        self._max_time = 0.0
        self._samples: list[dict] = []

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._advance)
        self._timer.setInterval(int(1000 / PLAYBACK_FPS))

        self._build_ui()

    def _build_ui(self):
        layout = QHBoxLayout(self)
        layout.setContentsMargins(8, 4, 8, 4)
        layout.setSpacing(6)

        btn_style = (
            f"QPushButton {{ background: {COLORS['surface0']}; "
            f"color: {COLORS['text']}; border-radius: 4px; padding: 4px 10px; "
            f"font-size: 14px; min-width: 32px; }}"
            f"QPushButton:hover {{ background: {COLORS['surface1']}; }}"
            f"QPushButton:disabled {{ color: {COLORS['overlay0']}; }}"
        )

        # Rewind
        self._btn_rewind = QPushButton("⏮")
        self._btn_rewind.setStyleSheet(btn_style)
        self._btn_rewind.setToolTip("Rewind to start")
        self._btn_rewind.clicked.connect(self._rewind)
        layout.addWidget(self._btn_rewind)

        # Play/Pause
        self._btn_play = QPushButton("▶")
        self._btn_play.setStyleSheet(btn_style)
        self._btn_play.setToolTip("Play / Pause")
        self._btn_play.clicked.connect(self._toggle_play)
        layout.addWidget(self._btn_play)

        # Stop
        self._btn_stop = QPushButton("⏹")
        self._btn_stop.setStyleSheet(btn_style)
        self._btn_stop.setToolTip("Stop")
        self._btn_stop.clicked.connect(self._stop)
        layout.addWidget(self._btn_stop)

        # Speed selector
        lbl_speed = QLabel("Speed:")
        lbl_speed.setStyleSheet(f"color: {COLORS['subtext0']}; font-size: 10px;")
        layout.addWidget(lbl_speed)

        self._speed_combo = QComboBox()
        self._speed_combo.addItems(["0.25x", "0.5x", "1x", "2x", "5x", "10x"])
        self._speed_combo.setCurrentText("1x")
        self._speed_combo.setStyleSheet(
            f"QComboBox {{ background: {COLORS['surface0']}; "
            f"color: {COLORS['text']}; border-radius: 3px; padding: 2px 6px; }}")
        self._speed_combo.currentTextChanged.connect(self._on_speed_changed)
        layout.addWidget(self._speed_combo)

        # Scrub slider
        self._slider = QSlider(Qt.Horizontal)
        self._slider.setMinimum(0)
        self._slider.setMaximum(0)
        self._slider.setStyleSheet(
            f"QSlider::groove:horizontal {{ background: {COLORS['surface0']}; "
            f"height: 6px; border-radius: 3px; }}"
            f"QSlider::handle:horizontal {{ background: {COLORS['mauve']}; "
            f"width: 14px; margin: -4px 0; border-radius: 7px; }}"
            f"QSlider::sub-page:horizontal {{ background: {COLORS['mauve']}; "
            f"border-radius: 3px; }}")
        self._slider.valueChanged.connect(self._on_slider_moved)
        layout.addWidget(self._slider, stretch=1)

        # Time label
        self._time_label = QLabel("0.0s / 0.0s")
        self._time_label.setStyleSheet(
            f"color: {COLORS['text']}; font-size: 10px; font-family: monospace;")
        self._time_label.setMinimumWidth(100)
        layout.addWidget(self._time_label)

    def set_data(self, samples: list[dict]):
        """Set sample data for playback."""
        self._samples = samples
        self._total_samples = len(samples)
        self._max_time = samples[-1].get("t", 0) if samples else 0.0
        self._slider.setMaximum(max(0, self._total_samples - 1))
        self._current_idx = 0
        self._slider.setValue(0)
        self._update_time_label()

    def _toggle_play(self):
        if self._playing:
            self._pause()
        else:
            self._play()

    def _play(self):
        if self._total_samples == 0:
            return
        if self._current_idx >= self._total_samples - 1:
            self._current_idx = 0
        self._playing = True
        self._btn_play.setText("⏸")
        self._timer.start()

    def _pause(self):
        self._playing = False
        self._btn_play.setText("▶")
        self._timer.stop()

    def _stop(self):
        self._pause()
        self._current_idx = 0
        self._slider.setValue(0)
        self.sample_changed.emit(0)
        self._update_time_label()

    def _rewind(self):
        self._current_idx = 0
        self._slider.setValue(0)
        self.sample_changed.emit(0)
        self._update_time_label()

    def _advance(self):
        """Called by timer to advance playback."""
        if not self._samples or self._current_idx >= self._total_samples - 1:
            self._pause()
            return

        # Compute how many samples to advance based on speed and real time
        current_t = self._samples[self._current_idx].get("t", 0)
        target_t = current_t + self._speed / PLAYBACK_FPS

        # Find the sample at or past target_t
        while (self._current_idx < self._total_samples - 1
               and self._samples[self._current_idx].get("t", 0) < target_t):
            self._current_idx += 1

        self._slider.blockSignals(True)
        self._slider.setValue(self._current_idx)
        self._slider.blockSignals(False)
        self.sample_changed.emit(self._current_idx)
        self._update_time_label()

    def _on_slider_moved(self, value: int):
        self._current_idx = value
        self.sample_changed.emit(value)
        self._update_time_label()

    def _on_speed_changed(self, text: str):
        try:
            self._speed = float(text.replace("x", ""))
        except ValueError:
            self._speed = 1.0

    def _update_time_label(self):
        if self._samples and 0 <= self._current_idx < len(self._samples):
            cur_t = self._samples[self._current_idx].get("t", 0)
        else:
            cur_t = 0.0
        self._time_label.setText(f"{cur_t:.1f}s / {self._max_time:.1f}s")


# ═══════════════════════════════════════════════════════════════════
# Statistics Panel
# ═══════════════════════════════════════════════════════════════════

class StatisticsPanel(QWidget):
    """Shows tracking error stats, path lengths, timing, per-segment table."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._build_ui()

    def _build_ui(self):
        layout = QHBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(8)

        # ── Tracking Error group ──────────────────────────────────
        err_group = QGroupBox("Tracking Error")
        err_group.setStyleSheet(self._group_style())
        err_lay = QVBoxLayout(err_group)
        err_lay.setSpacing(2)

        self._lbl_xy_mean = QLabel("XY Mean: —")
        self._lbl_xy_max = QLabel("XY Max:  —")
        self._lbl_xy_rms = QLabel("XY RMS:  —")
        self._lbl_z_mean = QLabel("Z Mean:  —")
        self._lbl_z_max = QLabel("Z Max:   —")

        for lbl in (self._lbl_xy_mean, self._lbl_xy_max, self._lbl_xy_rms,
                     self._lbl_z_mean, self._lbl_z_max):
            lbl.setStyleSheet(f"color: {COLORS['text']}; font-size: 10px; "
                              f"font-family: monospace;")
            err_lay.addWidget(lbl)

        layout.addWidget(err_group)

        # ── Path Length group ─────────────────────────────────────
        path_group = QGroupBox("Path Length")
        path_group.setStyleSheet(self._group_style())
        path_lay = QVBoxLayout(path_group)
        path_lay.setSpacing(2)

        self._lbl_planned_len = QLabel("Planned: —")
        self._lbl_actual_len = QLabel("Actual:  —")
        self._lbl_deviation = QLabel("Deviation: —")

        for lbl in (self._lbl_planned_len, self._lbl_actual_len, self._lbl_deviation):
            lbl.setStyleSheet(f"color: {COLORS['text']}; font-size: 10px; "
                              f"font-family: monospace;")
            path_lay.addWidget(lbl)

        path_lay.addStretch()
        layout.addWidget(path_group)

        # ── Timing group ──────────────────────────────────────────
        time_group = QGroupBox("Timing")
        time_group.setStyleSheet(self._group_style())
        time_lay = QVBoxLayout(time_group)
        time_lay.setSpacing(2)

        self._lbl_total_time = QLabel("Total:   —")
        self._lbl_print_time = QLabel("Print:   —")
        self._lbl_travel_time = QLabel("Travel:  —")
        self._lbl_retract_time = QLabel("Retract: —")
        self._lbl_samples = QLabel("Samples: —")

        for lbl in (self._lbl_total_time, self._lbl_print_time,
                     self._lbl_travel_time, self._lbl_retract_time,
                     self._lbl_samples):
            lbl.setStyleSheet(f"color: {COLORS['text']}; font-size: 10px; "
                              f"font-family: monospace;")
            time_lay.addWidget(lbl)

        layout.addWidget(time_group)

        # ── Per-Segment Table ─────────────────────────────────────
        seg_group = QGroupBox("Per-Segment Breakdown")
        seg_group.setStyleSheet(self._group_style())
        seg_lay = QVBoxLayout(seg_group)

        self._seg_table = QTableWidget()
        self._seg_table.setColumnCount(5)
        self._seg_table.setHorizontalHeaderLabels(
            ["Segment", "Samples", "Mean Err (µm)", "Max Err (µm)", "Duration (s)"])
        self._seg_table.horizontalHeader().setStretchLastSection(True)
        self._seg_table.horizontalHeader().setSectionResizeMode(
            QHeaderView.ResizeMode.Stretch)
        self._seg_table.setSelectionBehavior(
            QAbstractItemView.SelectionBehavior.SelectRows)
        self._seg_table.setEditTriggers(
            QAbstractItemView.EditTrigger.NoEditTriggers)
        self._seg_table.setMaximumHeight(150)
        self._seg_table.setStyleSheet(
            f"QTableWidget {{ background: {COLORS['surface0']}; "
            f"color: {COLORS['text']}; gridline-color: {COLORS['surface1']}; "
            f"font-size: 10px; }}"
            f"QHeaderView::section {{ background: {COLORS['surface1']}; "
            f"color: {COLORS['text']}; padding: 3px; font-weight: bold; "
            f"font-size: 9px; }}")
        seg_lay.addWidget(self._seg_table)

        layout.addWidget(seg_group, stretch=1)

    def set_data(self, samples: list[dict], meta: dict):
        """Populate all statistics from sample data and metadata."""
        if not samples:
            return

        summary = meta.get("summary", {})

        # ── Tracking Error ────────────────────────────────────────
        errors_xy = [s.get("tracking_error_xy", 0) for s in samples
                     if not s.get("is_travel", False)]
        errors_z = [s.get("tracking_error_z", 0) for s in samples
                    if not s.get("is_travel", False)]

        if errors_xy:
            mean_xy = sum(errors_xy) / len(errors_xy) * 1000
            max_xy = max(errors_xy) * 1000
            rms_xy = (sum(e * e for e in errors_xy) / len(errors_xy)) ** 0.5 * 1000
            self._lbl_xy_mean.setText(f"XY Mean: {mean_xy:6.1f} µm")
            self._lbl_xy_max.setText(f"XY Max:  {max_xy:6.1f} µm")
            self._lbl_xy_rms.setText(f"XY RMS:  {rms_xy:6.1f} µm")

            # Color-code based on mean error
            color = COLORS['green'] if mean_xy < 20 else (
                COLORS['yellow'] if mean_xy < 50 else COLORS['red'])
            self._lbl_xy_mean.setStyleSheet(
                f"color: {color}; font-size: 10px; font-family: monospace; font-weight: bold;")

        if errors_z:
            mean_z = sum(errors_z) / len(errors_z) * 1000
            max_z = max(errors_z) * 1000
            self._lbl_z_mean.setText(f"Z Mean:  {mean_z:6.1f} µm")
            self._lbl_z_max.setText(f"Z Max:   {max_z:6.1f} µm")

        # ── Path Length ───────────────────────────────────────────
        planned_len = self._compute_path_length(samples, "planned")
        actual_len = summary.get("actual_print_path_length_mm",
                                 self._compute_path_length(samples, "actual"))

        self._lbl_planned_len.setText(f"Planned: {planned_len:7.2f} mm")
        self._lbl_actual_len.setText(f"Actual:  {actual_len:7.2f} mm")

        if planned_len > 0:
            dev = abs(actual_len - planned_len) / planned_len * 100
            self._lbl_deviation.setText(f"Deviation: {dev:5.1f} %")

        # ── Timing ────────────────────────────────────────────────
        total_t = meta.get("duration_s", 0)
        if not total_t and samples:
            total_t = samples[-1].get("t", 0)

        print_samples = [s for s in samples
                         if not s.get("is_travel") and not s.get("is_retract")]
        travel_samples = [s for s in samples if s.get("is_travel")]
        retract_samples = [s for s in samples if s.get("is_retract")]

        self._lbl_total_time.setText(f"Total:   {total_t:7.1f} s")
        self._lbl_print_time.setText(f"Print:   {len(print_samples):>5d} pts")
        self._lbl_travel_time.setText(f"Travel:  {len(travel_samples):>5d} pts")
        self._lbl_retract_time.setText(f"Retract: {len(retract_samples):>5d} pts")
        self._lbl_samples.setText(f"Samples: {len(samples):>5d}")

        # ── Per-Segment Table ─────────────────────────────────────
        self._populate_segment_table(samples)

    def _compute_path_length(self, samples: list[dict], prefix: str) -> float:
        """Compute XY path length for print moves only."""
        length = 0.0
        prev_x = prev_y = None
        for s in samples:
            if s.get("is_travel") or s.get("is_retract"):
                prev_x = prev_y = None
                continue
            x = s.get(f"{prefix}_x", 0)
            y = s.get(f"{prefix}_y", 0)
            if prev_x is not None:
                dx = x - prev_x
                dy = y - prev_y
                length += (dx * dx + dy * dy) ** 0.5
            prev_x, prev_y = x, y
        return length

    def _populate_segment_table(self, samples: list[dict]):
        """Fill per-segment breakdown table."""
        segments: dict[int, list[dict]] = {}
        for s in samples:
            seg_id = s.get("segment_id", 0)
            segments.setdefault(seg_id, []).append(s)

        self._seg_table.setRowCount(len(segments))
        for row, (seg_id, seg_samples) in enumerate(sorted(segments.items())):
            errors = [s.get("tracking_error_xy", 0) * 1000 for s in seg_samples
                      if not s.get("is_travel")]
            mean_err = sum(errors) / len(errors) if errors else 0
            max_err = max(errors) if errors else 0
            duration = 0
            if seg_samples:
                t_vals = [s.get("t", 0) for s in seg_samples]
                duration = max(t_vals) - min(t_vals) if t_vals else 0

            items = [
                str(seg_id),
                str(len(seg_samples)),
                f"{mean_err:.1f}",
                f"{max_err:.1f}",
                f"{duration:.2f}",
            ]
            for col, text in enumerate(items):
                item = QTableWidgetItem(text)
                item.setTextAlignment(Qt.AlignCenter)
                self._seg_table.setItem(row, col, item)

    def _group_style(self) -> str:
        return (
            f"QGroupBox {{ border: 1px solid {COLORS['surface1']}; "
            f"border-radius: 4px; margin-top: 8px; padding-top: 12px; "
            f"color: {COLORS['subtext0']}; font-size: 10px; font-weight: bold; }}"
            f"QGroupBox::title {{ subcontrol-origin: margin; left: 8px; "
            f"padding: 0 4px; }}"
        )


# ═══════════════════════════════════════════════════════════════════
# Print Results Page — Main Page Widget
# ═══════════════════════════════════════════════════════════════════

class PrintResultsPage(QWidget):
    """
    Post-print diagnostic page: path comparison, playback, statistics.

    Page index: 6 (between Print Monitor and Settings).
    """

    navigate_to_page = Signal(int)

    def __init__(self, controller=None, settings=None, parent=None):
        super().__init__(parent)
        self._controller = controller
        self._settings = settings
        self._recorder = None
        self._hardware_config = None
        self._context_widget = None
        self._current_meta: dict = {}
        self._current_samples: list[dict] = []

        self._build_ui()

    # ── Page Interface ────────────────────────────────────────────

    def get_page_title(self) -> str:
        return "Print Results"

    def get_page_subtitle(self) -> str:
        return "Post-print diagnostics and path analysis"

    def set_hardware_config(self, config):
        """v7.2.6: Receive hardware config from app.py."""
        self._hardware_config = config

    def set_recorder(self, recorder):
        """Set PrintRecorder reference for loading recordings."""
        self._recorder = recorder

    def on_status_update(self):
        """Called by MainWindow timer — no-op for static results page."""
        pass

    # ── UI Construction ───────────────────────────────────────────

    def _build_ui(self):
        outer = QVBoxLayout(self)
        outer.setContentsMargins(8, 8, 8, 4)
        outer.setSpacing(6)

        # ── Header with projection buttons ────────────────────────
        header = QHBoxLayout()

        self._proj_buttons = {}
        btn_style = (
            f"QPushButton {{ background: {COLORS['surface0']}; "
            f"color: {COLORS['subtext0']}; border-radius: 4px; padding: 4px 12px; "
            f"font-weight: bold; font-size: 10px; }}"
            f"QPushButton:checked {{ background: {COLORS['mauve']}; "
            f"color: {COLORS['crust']}; }}"
            f"QPushButton:hover {{ background: {COLORS['surface1']}; }}"
        )
        for proj in ("XY", "XZ", "YZ"):
            btn = QPushButton(proj)
            btn.setCheckable(True)
            btn.setStyleSheet(btn_style)
            btn.clicked.connect(lambda checked, p=proj: self._on_projection_changed(p))
            self._proj_buttons[proj] = btn
            header.addWidget(btn)

        self._proj_buttons["XY"].setChecked(True)

        header.addStretch()

        # Recording name label
        self._recording_label = QLabel("No recording loaded")
        self._recording_label.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: 11px;")
        header.addWidget(self._recording_label)

        # Auto-fit button
        btn_fit = QPushButton("⊞ Fit")
        btn_fit.setStyleSheet(
            f"QPushButton {{ background: {COLORS['surface0']}; "
            f"color: {COLORS['text']}; border-radius: 4px; padding: 4px 10px; }}"
            f"QPushButton:hover {{ background: {COLORS['surface1']}; }}")
        btn_fit.clicked.connect(self._on_auto_fit)
        header.addWidget(btn_fit)

        outer.addLayout(header)

        # ── Main content: path view + error chart (vertical splitter) ─
        self._main_splitter = QSplitter(Qt.Vertical)

        # Top: path + error side by side
        top_splitter = QSplitter(Qt.Horizontal)
        self._path_widget = PathComparisonWidget()
        top_splitter.addWidget(self._path_widget)

        self._error_chart = ErrorTimeSeriesWidget()
        top_splitter.addWidget(self._error_chart)
        top_splitter.setSizes([600, 400])

        self._main_splitter.addWidget(top_splitter)

        # Bottom: stats panel
        self._stats_panel = StatisticsPanel()
        self._main_splitter.addWidget(self._stats_panel)

        self._main_splitter.setSizes([400, 200])
        outer.addWidget(self._main_splitter, stretch=1)

        # ── Playback controller bar ───────────────────────────────
        self._playback = PlaybackController()
        self._playback.setMaximumHeight(50)
        self._playback.setStyleSheet(
            f"background-color: {COLORS['surface0']}; border-radius: 4px;")
        outer.addWidget(self._playback)

        # ── Wire playback signals ─────────────────────────────────
        self._playback.sample_changed.connect(self._on_sample_changed)

    # ── Context Panel ─────────────────────────────────────────────

    def get_context_widget(self) -> QWidget:
        """Build context panel with recording browser and filters."""
        if self._context_widget is not None:
            return self._context_widget

        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)

        section_style = (
            f"font-size: 11pt; font-weight: bold; color: {COLORS['text']}; "
            f"margin-top: 4px;")

        # ── Recording Selector ────────────────────────────────────
        lbl_rec = QLabel("Recordings")
        lbl_rec.setStyleSheet(section_style)
        layout.addWidget(lbl_rec)

        self._ctx_recording_list = QListWidget()
        self._ctx_recording_list.setMaximumHeight(180)
        self._ctx_recording_list.setStyleSheet(
            f"QListWidget {{ background: {COLORS['surface0']}; "
            f"color: {COLORS['text']}; border-radius: 4px; font-size: 10px; }}"
            f"QListWidget::item:selected {{ background: {COLORS['surface1']}; }}")
        self._ctx_recording_list.itemClicked.connect(self._on_recording_clicked)
        layout.addWidget(self._ctx_recording_list)

        btn_load = QPushButton("📂 Load Selected")
        btn_load.setStyleSheet(
            f"QPushButton {{ background: {COLORS['green']}; "
            f"color: {COLORS['crust']}; font-weight: bold; "
            f"border-radius: 4px; padding: 6px; }}")
        btn_load.clicked.connect(self._load_selected_recording)
        layout.addWidget(btn_load)

        btn_refresh = QPushButton("🔄 Refresh List")
        btn_refresh.setStyleSheet(
            f"QPushButton {{ background: {COLORS['surface0']}; "
            f"color: {COLORS['text']}; border-radius: 4px; padding: 4px; }}")
        btn_refresh.clicked.connect(self._refresh_recording_list)
        layout.addWidget(btn_refresh)

        # ── Display Filters ───────────────────────────────────────
        sep1 = QFrame()
        sep1.setFrameShape(QFrame.HLine)
        sep1.setStyleSheet(f"color: {COLORS['surface1']};")
        layout.addWidget(sep1)

        lbl_filt = QLabel("Display Filters")
        lbl_filt.setStyleSheet(section_style)
        layout.addWidget(lbl_filt)

        chk_style = f"QCheckBox {{ color: {COLORS['text']}; font-size: 10px; }}"

        self._chk_planned = QCheckBox("Show planned path")
        self._chk_planned.setChecked(True)
        self._chk_planned.setStyleSheet(chk_style)
        self._chk_planned.toggled.connect(
            lambda v: self._path_widget.set_show_planned(v))
        layout.addWidget(self._chk_planned)

        self._chk_travel = QCheckBox("Show travel moves")
        self._chk_travel.setChecked(True)
        self._chk_travel.setStyleSheet(chk_style)
        self._chk_travel.toggled.connect(
            lambda v: self._path_widget.set_show_travel(v))
        layout.addWidget(self._chk_travel)

        self._chk_retract = QCheckBox("Show retract moves")
        self._chk_retract.setChecked(False)
        self._chk_retract.setStyleSheet(chk_style)
        self._chk_retract.toggled.connect(
            lambda v: self._path_widget.set_show_retract(v))
        layout.addWidget(self._chk_retract)

        # Segment filter
        seg_row = QHBoxLayout()
        seg_row.addWidget(QLabel("Segment:"))
        self._seg_combo = QComboBox()
        self._seg_combo.addItem("All", -1)
        self._seg_combo.setStyleSheet(
            f"QComboBox {{ background: {COLORS['surface0']}; "
            f"color: {COLORS['text']}; border-radius: 3px; padding: 2px 6px; }}")
        self._seg_combo.currentIndexChanged.connect(self._on_segment_filter_changed)
        seg_row.addWidget(self._seg_combo, stretch=1)
        layout.addLayout(seg_row)

        # ── Recording Info ────────────────────────────────────────
        sep2 = QFrame()
        sep2.setFrameShape(QFrame.HLine)
        sep2.setStyleSheet(f"color: {COLORS['surface1']};")
        layout.addWidget(sep2)

        lbl_info = QLabel("Recording Info")
        lbl_info.setStyleSheet(section_style)
        layout.addWidget(lbl_info)

        self._ctx_info_label = QLabel("No recording loaded")
        self._ctx_info_label.setWordWrap(True)
        self._ctx_info_label.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: 10px; "
            f"font-family: monospace;")
        layout.addWidget(self._ctx_info_label)

        # ── Export ────────────────────────────────────────────────
        sep3 = QFrame()
        sep3.setFrameShape(QFrame.HLine)
        sep3.setStyleSheet(f"color: {COLORS['surface1']};")
        layout.addWidget(sep3)

        btn_export_csv = QPushButton("📤 Export Summary CSV")
        btn_export_csv.setStyleSheet(
            f"QPushButton {{ background: {COLORS['surface0']}; "
            f"color: {COLORS['text']}; border-radius: 4px; padding: 4px; }}")
        btn_export_csv.clicked.connect(self._export_summary_csv)
        layout.addWidget(btn_export_csv)

        layout.addStretch()

        self._context_widget = widget

        # Initial refresh after a short delay
        QTimer.singleShot(500, self._refresh_recording_list)

        return widget

    # ── Recording Loading ─────────────────────────────────────────

    def _refresh_recording_list(self):
        """Refresh the list of available recordings."""
        self._ctx_recording_list.clear()

        try:
            from SupportClasses.PrintRecorder import PrintRecorder
            recorder = self._recorder or PrintRecorder()
            recordings = recorder.list_recordings()

            for rec in recordings:
                label = f"{rec.job_name}  ({rec.timestamp})"
                item = QListWidgetItem(label)
                item.setData(Qt.ItemDataRole.UserRole, str(rec.meta_path))
                item.setToolTip(
                    f"Samples: {rec.num_samples}\n"
                    f"Duration: {rec.duration_s:.1f}s\n"
                    f"Status: {getattr(rec, 'status', 'unknown')}\n"
                    f"Path: {rec.meta_path}")
                self._ctx_recording_list.addItem(item)

            if not recordings:
                self._ctx_info_label.setText(
                    "No recordings found.\n"
                    "Complete a print to generate recording data.")
            else:
                self._ctx_info_label.setText(
                    f"{len(recordings)} recording(s) available")

        except Exception as e:
            logger.warning(f"Failed to list recordings: {e}")
            self._ctx_info_label.setText(f"Error: {e}")

    def _on_recording_clicked(self, item: QListWidgetItem):
        """Handle recording selection — show tooltip info."""
        self._ctx_info_label.setText(item.toolTip())

    def _load_selected_recording(self):
        """Load the currently selected recording."""
        item = self._ctx_recording_list.currentItem()
        if not item:
            return

        meta_path = item.data(Qt.ItemDataRole.UserRole)
        if meta_path:
            self._load_recording(meta_path)

    def load_latest_recording(self):
        """Auto-load the most recent recording (called after print completes)."""
        try:
            from SupportClasses.PrintRecorder import PrintRecorder
            recorder = self._recorder or PrintRecorder()
            recordings = recorder.list_recordings()
            if recordings:
                # Recordings are sorted newest-first
                latest = recordings[0]
                self._load_recording(str(latest.meta_path))
                self._refresh_recording_list()
        except Exception as e:
            logger.warning(f"Failed to load latest recording: {e}")

    def _load_recording(self, meta_path: str):
        """Load a recording and populate all widgets."""
        try:
            from SupportClasses.PrintRecorder import PrintRecorder
            recorder = self._recorder or PrintRecorder()
            result = recorder.load_recording(meta_path)

            if result is None:
                logger.warning(f"Failed to load recording: {meta_path}")
                return

            # Handle both tuple and dict return formats
            if isinstance(result, tuple):
                meta, samples_list = result
                if isinstance(samples_list, list):
                    samples = samples_list
                else:
                    # Could be a list of RecordingSample or dicts
                    samples = [s if isinstance(s, dict) else vars(s)
                               for s in samples_list]
            elif isinstance(result, dict):
                meta = result.get("meta", {})
                samples = result.get("samples", [])
            else:
                logger.warning(f"Unexpected load_recording result type: {type(result)}")
                return

            # Convert samples to dicts if they aren't already
            if samples and not isinstance(samples[0], dict):
                samples = [vars(s) if hasattr(s, '__dict__') else {} for s in samples]

            self._current_meta = meta
            self._current_samples = samples

            # Update all widgets
            self._path_widget.set_data(samples)
            self._error_chart.set_data(samples)
            self._stats_panel.set_data(samples, meta)
            self._playback.set_data(samples)

            # Update recording label
            job_name = meta.get("job_name", "Unknown")
            status = meta.get("status", "?")
            timestamp = meta.get("timestamp_str", meta.get("timestamp", "?"))
            self._recording_label.setText(
                f"📋 {job_name} — {status} — {timestamp}")

            # Update segment filter combo
            self._seg_combo.blockSignals(True)
            self._seg_combo.clear()
            self._seg_combo.addItem("All", -1)
            segments = sorted(set(s.get("segment_id", 0) for s in samples))
            for seg_id in segments:
                self._seg_combo.addItem(f"Segment {seg_id}", seg_id)
            self._seg_combo.blockSignals(False)

            # Update context info
            summary = meta.get("summary", {})
            info_lines = [
                f"Job: {job_name}",
                f"Status: {status}",
                f"Date: {timestamp}",
                f"Samples: {len(samples)}",
                f"Duration: {meta.get('duration_s', 0):.1f}s",
                f"Segments: {len(segments)}",
            ]

            # Workspace info from meta
            ws = meta.get("workspace", {})
            if ws:
                needle = ws.get("needle_gauge", "?")
                plate = ws.get("plate_format", "?")
                info_lines.append(f"\nWorkspace:")
                info_lines.append(f"  Needle: {needle}G")
                info_lines.append(f"  Plate: {plate}")

            self._ctx_info_label.setText("\n".join(info_lines))

            logger.info(f"Loaded recording: {job_name} ({len(samples)} samples)")

        except Exception as e:
            logger.error(f"Error loading recording: {e}", exc_info=True)
            self._recording_label.setText(f"Error loading recording")
            self._ctx_info_label.setText(f"Error: {e}")

    # ── Signal Handlers ───────────────────────────────────────────

    def _on_sample_changed(self, idx: int):
        """Playback cursor moved — update path and error views."""
        self._path_widget.set_cursor(idx)
        self._error_chart.set_cursor(idx)

    def _on_projection_changed(self, proj: str):
        """Switch projection mode."""
        for name, btn in self._proj_buttons.items():
            btn.setChecked(name == proj)
        self._path_widget.set_projection(proj)

    def _on_auto_fit(self):
        """Reset view to auto-fit."""
        self._path_widget._auto_fit()
        self._path_widget.update()

    def _on_segment_filter_changed(self, index: int):
        """Filter display to a specific segment."""
        seg_id = self._seg_combo.itemData(index)
        if seg_id is not None:
            self._path_widget.set_segment_filter(seg_id)

    def _export_summary_csv(self):
        """Export summary statistics to CSV."""
        if not self._current_samples:
            return

        try:
            from PySide6.QtWidgets import QFileDialog
            path, _ = QFileDialog.getSaveFileName(
                self, "Export Summary", "", "CSV Files (*.csv)")
            if not path:
                return

            meta = self._current_meta
            summary = meta.get("summary", {})

            with open(path, "w") as f:
                f.write("Metric,Value\n")
                f.write(f"Job Name,{meta.get('job_name', '')}\n")
                f.write(f"Status,{meta.get('status', '')}\n")
                f.write(f"Timestamp,{meta.get('timestamp', '')}\n")
                f.write(f"Duration (s),{meta.get('duration_s', 0)}\n")
                f.write(f"Total Samples,{len(self._current_samples)}\n")

                for key, val in sorted(summary.items()):
                    f.write(f"{key},{val}\n")

            logger.info(f"Exported summary to {path}")

        except Exception as e:
            logger.error(f"Export failed: {e}")

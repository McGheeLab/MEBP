"""
trajectory_view.py — Real-time needle path visualization for MEBP v7.1.

L-shaped triple-projection widget showing:
- XY view (large, top-left): top-down needle path
- ZY view (tall, right): side view
- XZ view (wide, bottom): front view

Each projection renders:
- Well boundary (circle/rect with correct diameter)
- Completed path as solid colored lines
- Next N upcoming waypoints as dashed lines (configurable, default N=20)
- Needle position as a crosshair dot
- Tracking error indicator (optional ring around needle)

Updated at position poller rate (~3 Hz) from the print monitor.

Session H — Tasks P6.4, P6.5.
"""

from __future__ import annotations

import logging
import math
from collections import deque
from dataclasses import dataclass

from PySide6.QtWidgets import (
    QWidget, QGraphicsView, QGraphicsScene,
    QVBoxLayout, QHBoxLayout, QSplitter, QSizePolicy,
)
from PySide6.QtCore import Qt, QRectF, QPointF
from PySide6.QtGui import (
    QColor, QPen, QBrush, QPainter, QFont,
    QPainterPath,
)

from gui.styles import COLORS

logger = logging.getLogger(__name__)


# ═══════════════════════════════════════════════════════════════════
# Constants
# ═══════════════════════════════════════════════════════════════════

SCALE = 8.0                   # mm → scene px
NEEDLE_DOT_RADIUS = 4         # px
CROSSHAIR_SIZE = 10           # px arm length
PATH_PEN_WIDTH = 2.0
UPCOMING_PEN_WIDTH = 1.5
UPCOMING_DASH_PATTERN = [4, 4]
DEFAULT_UPCOMING_COUNT = 20

# Colors
COMPLETED_PATH_COLOR = "#a6e3a1"     # Green
UPCOMING_PATH_COLOR = "#f9e2af"      # Yellow dashed
NEEDLE_COLOR = "#f5c2e7"            # Pink
WELL_BOUNDARY_COLOR = "#45475a"
TRACKING_ERROR_COLOR = "#f38ba8"     # Red ring
BG_COLOR = "#181825"


# ═══════════════════════════════════════════════════════════════════
# Single Projection View
# ═══════════════════════════════════════════════════════════════════

class ProjectionView(QGraphicsView):
    """
    A single 2D projection (XY, ZY, or XZ) with path rendering.

    Axes are selected at construction:
    - "XY": h_axis=x, v_axis=y  (top-down, Y increases downward)
    - "ZY": h_axis=z, v_axis=y  (side view)
    - "XZ": h_axis=x, v_axis=z  (front view)
    """

    def __init__(
        self,
        mode: str = "XY",
        parent: QWidget | None = None,
    ):
        super().__init__(parent)
        self._scene = QGraphicsScene(self)
        self.setScene(self._scene)
        self._mode = mode

        # Data buffers
        self._completed_points: deque[tuple[float, float]] = deque(maxlen=5000)
        self._upcoming_points: list[tuple[float, float]] = []
        self._needle_pos: tuple[float, float] | None = None
        self._tracking_error_mm: float = 0.0
        self._well_diameter_mm: float = 0.0

        # View settings
        self.setRenderHint(QPainter.RenderHint.Antialiasing)
        self.setStyleSheet(f"background-color: {BG_COLOR}; border: 1px solid #45475a;")
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.setTransformationAnchor(QGraphicsView.ViewportAnchor.AnchorUnderMouse)

        # Mode-specific sizing
        if mode == "XY":
            self.setMinimumSize(200, 200)
            self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        elif mode == "ZY":
            self.setFixedWidth(140)
            self.setMinimumHeight(150)
            self.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Expanding)
        else:  # XZ
            self.setFixedHeight(100)
            self.setMinimumWidth(200)
            self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)

    # ── Public API ────────────────────────────────────────────────

    def set_well_diameter(self, diameter_mm: float) -> None:
        """Set well boundary circle diameter."""
        self._well_diameter_mm = diameter_mm

    def add_completed_point(self, x: float, y: float, z: float) -> None:
        """Add a point to the completed path."""
        h, v = self._project(x, y, z)
        self._completed_points.append((h, v))

    def set_upcoming_waypoints(
        self,
        waypoints: list[tuple[float, float, float]],
    ) -> None:
        """Set the upcoming waypoint list (next N points)."""
        self._upcoming_points = [self._project(*pt) for pt in waypoints]

    def set_needle_position(
        self,
        x: float | None,
        y: float | None,
        z: float | None,
    ) -> None:
        """Update needle position."""
        if x is not None and y is not None and z is not None:
            h, v = self._project(x, y, z)
            self._needle_pos = (h, v)
        else:
            self._needle_pos = None

    def set_tracking_error(self, error_mm: float) -> None:
        """Set current tracking error for visual indicator."""
        self._tracking_error_mm = error_mm

    def clear_path(self) -> None:
        """Clear all path data."""
        self._completed_points.clear()
        self._upcoming_points.clear()
        self._needle_pos = None
        self._tracking_error_mm = 0.0
        self._redraw()

    def refresh(self) -> None:
        """Trigger a full scene redraw."""
        self._redraw()

    # ── Projection ────────────────────────────────────────────────

    def _project(self, x: float, y: float, z: float) -> tuple[float, float]:
        """Project 3D (x,y,z) onto this view's 2D axes."""
        if self._mode == "XY":
            return (x, y)
        elif self._mode == "ZY":
            return (z, y)
        else:  # XZ
            return (x, z)

    # ── Rendering ─────────────────────────────────────────────────

    def _redraw(self) -> None:
        """Redraw the entire scene."""
        self._scene.clear()

        # Well boundary
        if self._well_diameter_mm > 0:
            r = self._well_diameter_mm / 2 * SCALE
            pen = QPen(QColor(WELL_BOUNDARY_COLOR), 1.5, Qt.PenStyle.DotLine)
            if self._mode == "XY":
                self._scene.addEllipse(-r, -r, 2 * r, 2 * r, pen)
            else:
                # Side/front view: show as horizontal line at well bottom
                self._scene.addLine(-r, 0, r, 0, pen)

        # Completed path (solid green polyline)
        if len(self._completed_points) >= 2:
            path = QPainterPath()
            pts = list(self._completed_points)
            path.moveTo(pts[0][0] * SCALE, pts[0][1] * SCALE)
            for h, v in pts[1:]:
                path.lineTo(h * SCALE, v * SCALE)
            pen = QPen(QColor(COMPLETED_PATH_COLOR), PATH_PEN_WIDTH)
            self._scene.addPath(path, pen)

        # Upcoming waypoints (dashed yellow polyline)
        if len(self._upcoming_points) >= 2:
            path = QPainterPath()
            path.moveTo(
                self._upcoming_points[0][0] * SCALE,
                self._upcoming_points[0][1] * SCALE,
            )
            for h, v in self._upcoming_points[1:]:
                path.lineTo(h * SCALE, v * SCALE)
            pen = QPen(QColor(UPCOMING_PATH_COLOR), UPCOMING_PEN_WIDTH)
            pen.setDashPattern(UPCOMING_DASH_PATTERN)
            self._scene.addPath(path, pen)

        # Upcoming waypoint dots (small circles)
        dot_pen = QPen(QColor(UPCOMING_PATH_COLOR), 1)
        dot_brush = QBrush(QColor(UPCOMING_PATH_COLOR))
        for h, v in self._upcoming_points:
            self._scene.addEllipse(
                h * SCALE - 2, v * SCALE - 2, 4, 4,
                dot_pen, dot_brush,
            )

        # Needle crosshair
        if self._needle_pos:
            nh, nv = self._needle_pos
            sx, sy = nh * SCALE, nv * SCALE

            # Tracking error ring (if significant)
            if self._tracking_error_mm > 0.01:
                err_r = self._tracking_error_mm * SCALE
                err_pen = QPen(QColor(TRACKING_ERROR_COLOR), 1, Qt.PenStyle.DotLine)
                self._scene.addEllipse(
                    sx - err_r, sy - err_r, 2 * err_r, 2 * err_r, err_pen,
                )

            # Crosshair lines
            needle_pen = QPen(QColor(NEEDLE_COLOR), 2)
            self._scene.addLine(
                sx - CROSSHAIR_SIZE, sy, sx + CROSSHAIR_SIZE, sy, needle_pen)
            self._scene.addLine(
                sx, sy - CROSSHAIR_SIZE, sx, sy + CROSSHAIR_SIZE, needle_pen)

            # Center dot
            self._scene.addEllipse(
                sx - NEEDLE_DOT_RADIUS, sy - NEEDLE_DOT_RADIUS,
                NEEDLE_DOT_RADIUS * 2, NEEDLE_DOT_RADIUS * 2,
                QPen(QColor(NEEDLE_COLOR), 1),
                QBrush(QColor(NEEDLE_COLOR)),
            )

        # Axis label
        font = QFont("Segoe UI", 8)
        label = self._scene.addText(self._mode, font)
        label.setDefaultTextColor(QColor(COLORS["overlay0"]))
        br = self._scene.itemsBoundingRect()
        label.setPos(br.left() - 5, br.top() - 15)

        # Fit view
        rect = self._scene.itemsBoundingRect().adjusted(-15, -15, 15, 15)
        if not rect.isEmpty():
            self.fitInView(rect, Qt.AspectRatioMode.KeepAspectRatio)


# ═══════════════════════════════════════════════════════════════════
# L-Shaped Triple Projection Widget
# ═══════════════════════════════════════════════════════════════════

class TrajectoryView(QWidget):
    """
    L-shaped triple projection for trajectory monitoring:

    ┌──────────────┬────────┐
    │              │        │
    │   XY (large) │ ZY     │
    │              │ (tall) │
    │              │        │
    ├──────────────┴────────┤
    │   XZ (wide)           │
    └───────────────────────┘

    Provides a unified API to update all three views simultaneously.
    """

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(2)

        # Top row: XY + ZY
        top_splitter = QSplitter(Qt.Orientation.Horizontal)
        self.xy_view = ProjectionView("XY")
        self.zy_view = ProjectionView("ZY")
        top_splitter.addWidget(self.xy_view)
        top_splitter.addWidget(self.zy_view)
        top_splitter.setStretchFactor(0, 4)
        top_splitter.setStretchFactor(1, 1)
        layout.addWidget(top_splitter, stretch=3)

        # Bottom row: XZ
        self.xz_view = ProjectionView("XZ")
        layout.addWidget(self.xz_view, stretch=1)

        self._views = [self.xy_view, self.zy_view, self.xz_view]

    # ── Public API ────────────────────────────────────────────────

    def set_well_diameter(self, diameter_mm: float) -> None:
        """Set well boundary on all views."""
        for v in self._views:
            v.set_well_diameter(diameter_mm)

    def add_completed_point(self, x: float, y: float, z: float) -> None:
        """Add a completed path point to all views."""
        for v in self._views:
            v.add_completed_point(x, y, z)

    def set_upcoming_waypoints(
        self,
        waypoints: list[tuple[float, float, float]],
    ) -> None:
        """Set upcoming waypoints on all views."""
        for v in self._views:
            v.set_upcoming_waypoints(waypoints)

    def set_needle_position(
        self,
        x: float | None,
        y: float | None,
        z: float | None,
    ) -> None:
        """Update needle position on all views."""
        for v in self._views:
            v.set_needle_position(x, y, z)

    def set_tracking_error(self, error_mm: float) -> None:
        """Set tracking error on all views."""
        for v in self._views:
            v.set_tracking_error(error_mm)

    def clear_path(self) -> None:
        """Clear all path data on all views."""
        for v in self._views:
            v.clear_path()

    def refresh(self) -> None:
        """Trigger redraw on all views."""
        for v in self._views:
            v.refresh()

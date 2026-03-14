"""
print_monitor.py — Print Monitor Page for MEBP v7.2.8.

Direct position polling, YZ view, syringe display.

Layout:
┌──────────────────────┬──────────────────────────┬──────────────┐
│  Plate Overview       │  XY Detail View          │  YZ View     │
│  (wells + needle)     │  (zoomed, waypoints)     │  (Z height)  │
├──────────────────────┴──────────────────────────┴──────────────┤
│  Syringe Pumps [P1][P2][P3]         │  Controls + Progress     │
│  fill bars + µL readout             │  [Pause] [Abort] ████ 52%│
└─────────────────────────────────────┴──────────────────────────┘
"""

from __future__ import annotations

import logging
import math
import re
import time
from collections import deque
from enum import Enum

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QPushButton, QLabel, QProgressBar, QGroupBox,
    QSplitter, QListWidget, QFrame, QSizePolicy, QCheckBox,
)
from PySide6.QtCore import Qt, Signal, QRectF, QPointF
from PySide6.QtGui import QPainter, QPen, QColor, QBrush, QFont, QLinearGradient, QImage

from gui.styles import COLORS

try:
    from SupportClasses.PrintManager import PrintState
except ImportError:
    class PrintState(Enum):
        IDLE = "IDLE"; RUNNING = "RUNNING"; PAUSED = "PAUSED"
        COMPLETED = "COMPLETED"; ABORTED = "ABORTED"; ERROR = "ERROR"

try:
    from SupportClasses.PhysicalModels import WellRole, ROLE_COLORS, WorkspaceConfig
except ImportError:
    WellRole = None; ROLE_COLORS = {}
    class WorkspaceConfig:
        plate_format = 24

logger = logging.getLogger(__name__)


# ═══════════════════════════════════════════════════════════════════
#  Plate Overview Widget
# ═══════════════════════════════════════════════════════════════════

class PlateOverviewWidget(QWidget):
    """Miniature plate with color-coded wells + needle crosshair."""

    STATE_COLORS = {
        "pending": QColor("#585b70"), "active": QColor("#f9e2af"),
        "done": QColor("#a6e3a1"), "error": QColor("#f38ba8"),
    }
    NEEDLE_COLOR = QColor("#f5c2e7")

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(180, 140)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self._wells: list[dict] = []
        self._well_roles: dict[str, str] = {}
        self._well_states: dict[str, str] = {}
        self._needle_x: float | None = None
        self._needle_y: float | None = None
        self._margin = 28
        self._active_well = ""

    def set_plate(self, plate):
        self._wells = [{"name": w.name, "x": w.x, "y": w.y,
                        "diameter": w.diameter, "row": w.row, "col": w.col}
                       for w in plate.get_all_wells()]
        self._well_states.clear()
        self._well_roles.clear()
        self.update()

    def set_well_roles(self, roles: dict):
        self._well_roles.clear()
        fallback = {"print": "#a6e3a1", "wash": "#89b4fa", "waste": "#f38ba8",
                    "buffer": "#cba6f7", "ink": "#f9e2af", "empty": "#585b70", "sorted": "#94e2d5"}
        for name, role in roles.items():
            if ROLE_COLORS and role in ROLE_COLORS:
                self._well_roles[name] = ROLE_COLORS[role]
            elif hasattr(role, 'value'):
                self._well_roles[name] = fallback.get(role.value, "#585b70")
        self.update()

    def set_all_pending(self, names: list[str]):
        for n in names: self._well_states[n] = "pending"
        self.update()

    def set_needle_position(self, x_mm, y_mm):
        self._needle_x = x_mm; self._needle_y = y_mm; self.update()

    def set_active_well(self, name: str):
        """Mark a well as actively printing. Only print wells change state."""
        if self._active_well and self._active_well != name:
            prev = self._well_states.get(self._active_well)
            if prev in ("active", "pending"):
                self._well_states[self._active_well] = "done"
        if self._well_states.get(name) in ("pending", None):
            self._well_states[name] = "active"
        self._active_well = name
        self.update()

    def paintEvent(self, event):
        if not self._wells: return
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.fillRect(self.rect(), QColor(COLORS.get("mantle", "#181825")))

        max_x = max((w["x"] for w in self._wells), default=1) or 1
        max_y = max((w["y"] for w in self._wells), default=1) or 1
        aw = self.width() - 2 * self._margin
        ah = self.height() - 2 * self._margin
        scale = min(aw / max(max_x, 0.1), ah / max(max_y, 0.1))
        wr = max(self._wells[0]["diameter"] * scale / 2, 4) * 0.75

        for w in self._wells:
            cx, cy = self._margin + w["x"] * scale, self._margin + w["y"] * scale
            name = w["name"]
            state = self._well_states.get(name)
            if state and state in self.STATE_COLORS:
                color = self.STATE_COLORS[state]
            elif name in self._well_roles:
                color = QColor(self._well_roles[name])
            else:
                color = QColor("#45475a")
            p.setPen(QPen(QColor("#6c7086"), 1))
            p.setBrush(QBrush(color))
            p.drawEllipse(QPointF(cx, cy), wr, wr)
            if scale > 3:
                p.setPen(QColor("#cdd6f4"))
                p.setFont(QFont("Arial", max(int(wr * 0.55), 5)))
                p.drawText(QRectF(cx - wr, cy - wr, wr * 2, wr * 2),
                           Qt.AlignmentFlag.AlignCenter, name)

        if self._needle_x is not None and self._needle_y is not None:
            nx = self._margin + self._needle_x * scale
            ny = self._margin + self._needle_y * scale
            pen = QPen(self.NEEDLE_COLOR, 2)
            p.setPen(pen)
            p.drawLine(QPointF(nx - 8, ny), QPointF(nx + 8, ny))
            p.drawLine(QPointF(nx, ny - 8), QPointF(nx, ny + 8))
            p.setBrush(Qt.BrushStyle.NoBrush)
            p.drawEllipse(QPointF(nx, ny), 4, 4)

        # Headers
        p.setPen(QColor("#6c7086")); p.setFont(QFont("Arial", 7))
        rows_done, cols_done = set(), set()
        for w in self._wells:
            cx, cy = self._margin + w["x"] * scale, self._margin + w["y"] * scale
            if w["col"] == 0 and w["row"] not in rows_done:
                rows_done.add(w["row"]); p.drawText(QPointF(3, cy + 3), w["name"][0])
            if w["row"] == 0 and w["col"] not in cols_done:
                cols_done.add(w["col"]); p.drawText(QPointF(cx - 3, self._margin - 5), str(w["col"] + 1))
        p.end()


# ═══════════════════════════════════════════════════════════════════
#  XY Detail View
# ═══════════════════════════════════════════════════════════════════

class XYDetailView(QWidget):
    """Zoomed XY view: planned trajectory + actual needle trail.

    Displays planned print waypoints color-coded by completion state,
    the actual needle trail from position polling, travel segments,
    and a well boundary circle centered on the current well.

    Coordinate system: all positions in plate-absolute mm (zero-referenced).
    View auto-centers on the current well's waypoints.
    """

    C_DONE = QColor("#a6e3a1")        # Completed print path
    C_NEXT = QColor("#f9e2af")        # Upcoming print path
    C_TRAVEL = QColor("#45475a")      # Travel segments
    C_WP_DONE = QColor("#a6e3a1")     # Completed waypoint dot
    C_WP_CURRENT = QColor("#f9e2af")  # Current waypoint dot
    C_WP_EMPTY = QColor("#6c7086")    # Upcoming waypoint dot
    C_TRAIL = QColor("#cba6f7")       # Actual needle trail (purple)
    C_NEEDLE = QColor("#f5c2e7")      # Needle crosshair
    C_WELL = QColor("#585b70")        # Well boundary
    C_BG = QColor("#1e1e2e")          # Background

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(250, 200)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)

        # Planned trajectory — ALL waypoints (print + travel)
        self._waypoints: list[tuple[float, float]] = []
        self._wp_is_travel: list[bool] = []    # parallel: True = travel move
        self._wp_well: list[str] = []          # parallel: well name
        self._completed_idx = 0

        # Actual needle trail from position polling
        self._needle_trail: deque[tuple[float, float]] = deque(maxlen=2000)
        self._nx: float | None = None
        self._ny: float | None = None

        # View parameters
        self._well_diam = 15.0
        self._well_center_x = 0.0
        self._well_center_y = 0.0
        self._view_cx = 0.0       # center of view in mm
        self._view_cy = 0.0
        self._view_radius = 12.0  # half-extent of view in mm

        # Per-well centers cache
        self._well_centers: dict[str, tuple[float, float]] = {}
        self._current_well = ""

        # v7.3.2: Camera overlay
        self._cam_frame: QImage | None = None   # Latest camera QImage
        self._cam_um_per_px: float = 0.0        # Camera scale
        self._cam_overlay_enabled: bool = False  # Toggle

    # ── Data setters ──────────────────────────────────────────────

    def set_trajectory(self, waypoints, is_travel=None, wells=None):
        """Set the full planned trajectory with metadata.

        Args:
            waypoints: list of (x, y) tuples in plate-absolute mm.
            is_travel: parallel list of booleans (True = travel move).
            wells: parallel list of well name strings.
        """
        self._waypoints = list(waypoints)
        n = len(self._waypoints)
        self._wp_is_travel = list(is_travel) if is_travel else [False] * n
        self._wp_well = list(wells) if wells else [""] * n
        self._completed_idx = 0
        self._needle_trail.clear()

        # Compute per-well centers from print waypoints
        self._well_centers.clear()
        well_points: dict[str, list[tuple[float, float]]] = {}
        for i, (x, y) in enumerate(self._waypoints):
            w = self._wp_well[i] if i < len(self._wp_well) else ""
            if w and (i >= len(self._wp_is_travel) or not self._wp_is_travel[i]):
                well_points.setdefault(w, []).append((x, y))
        for w, pts in well_points.items():
            self._well_centers[w] = (
                sum(p[0] for p in pts) / len(pts),
                sum(p[1] for p in pts) / len(pts),
            )

        # Focus on first well with print waypoints
        first_well = ""
        for w in self._wp_well:
            if w:
                first_well = w
                break
        if first_well:
            self._focus_well(first_well)
        else:
            self._auto_fit_view()
        self.update()

    def set_waypoints(self, pts):
        """Legacy API: set print waypoints without metadata."""
        self.set_trajectory(pts)

    def set_completed_index(self, i):
        self._completed_idx = min(i, len(self._waypoints))
        self.update()

    def set_well_diameter(self, d):
        self._well_diam = d
        self.update()

    def set_needle_position(self, x, y):
        self._nx = x
        self._ny = y
        self._needle_trail.append((x, y))
        self.update()

    # ── v7.3.2: Camera overlay ───────────────────────────────────

    def set_camera_overlay(self, enabled: bool):
        """Enable/disable camera overlay on the XY view."""
        self._cam_overlay_enabled = enabled
        self.update()

    def set_camera_frame(self, qimage: QImage | None, um_per_px: float = 0.0):
        """Set the latest camera frame for overlay.

        Args:
            qimage: Camera frame as QImage (RGB888 or similar).
            um_per_px: Microns per pixel — used to scale the frame to stage coords.
        """
        self._cam_frame = qimage
        if um_per_px > 0:
            self._cam_um_per_px = um_per_px
        if self._cam_overlay_enabled:
            self.update()

    def zoom_to_camera(self):
        """Zoom the view to fit the camera FOV around the needle."""
        if self._cam_frame is None or self._cam_um_per_px <= 0:
            return
        if self._nx is None:
            return
        fov_w_mm = self._cam_frame.width() * self._cam_um_per_px / 1000.0
        fov_h_mm = self._cam_frame.height() * self._cam_um_per_px / 1000.0
        self._view_cx = self._nx
        self._view_cy = self._ny
        self._view_radius = max(fov_w_mm, fov_h_mm) / 2 + 1.0
        self.update()

    def zoom_to_plate(self):
        """Zoom the view to fit the entire trajectory."""
        self._auto_fit_view()
        self.update()

    def focus_well(self, well_name):
        """Center and zoom the view on a specific well's waypoints."""
        if well_name and well_name != self._current_well:
            self._focus_well(well_name)
            self.update()

    def clear(self):
        self._waypoints.clear()
        self._wp_is_travel.clear()
        self._wp_well.clear()
        self._completed_idx = 0
        self._needle_trail.clear()
        self._nx = self._ny = None
        self._current_well = ""
        self._well_centers.clear()
        self.update()

    # ── View management ───────────────────────────────────────────

    def _focus_well(self, well_name):
        """Center and zoom view on a specific well's print waypoints."""
        self._current_well = well_name

        # Gather this well's print waypoints
        well_pts = [
            self._waypoints[i]
            for i in range(len(self._waypoints))
            if (i < len(self._wp_well) and self._wp_well[i] == well_name
                and (i >= len(self._wp_is_travel) or not self._wp_is_travel[i]))
        ]

        if well_pts:
            xs = [p[0] for p in well_pts]
            ys = [p[1] for p in well_pts]
            cx = (min(xs) + max(xs)) / 2
            cy = (min(ys) + max(ys)) / 2
            span = max(max(xs) - min(xs), max(ys) - min(ys), self._well_diam)
            self._view_cx = cx
            self._view_cy = cy
            self._view_radius = span / 2 + 2.0  # padding
            self._well_center_x = cx
            self._well_center_y = cy
        elif well_name in self._well_centers:
            cx, cy = self._well_centers[well_name]
            self._view_cx = cx
            self._view_cy = cy
            self._view_radius = self._well_diam / 2 + 2.0
            self._well_center_x = cx
            self._well_center_y = cy

    def _auto_fit_view(self):
        """Fit view to all waypoints (used when no well info available)."""
        if not self._waypoints:
            return
        xs = [p[0] for p in self._waypoints]
        ys = [p[1] for p in self._waypoints]
        self._view_cx = (min(xs) + max(xs)) / 2
        self._view_cy = (min(ys) + max(ys)) / 2
        span = max(max(xs) - min(xs), max(ys) - min(ys), 1.0)
        self._view_radius = span / 2 + 2.0
        self._well_center_x = self._view_cx
        self._well_center_y = self._view_cy

    # ── Coordinate transform ──────────────────────────────────────

    def _to_px(self, xm, ym):
        """Convert plate-absolute mm → pixel coordinates."""
        w, h = self.width(), self.height()
        r = self._view_radius if self._view_radius > 0 else 1.0
        s = min(w, h) / (2 * r)
        return w / 2 + (xm - self._view_cx) * s, h / 2 + (ym - self._view_cy) * s

    def _get_scale(self):
        w, h = self.width(), self.height()
        r = self._view_radius if self._view_radius > 0 else 1.0
        return min(w, h) / (2 * r)

    # ── Drawing ───────────────────────────────────────────────────

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.fillRect(self.rect(), self.C_BG)
        w, h = self.width(), self.height()

        if self._view_radius <= 0:
            p.end()
            return

        s = self._get_scale()
        wp = self._waypoints
        ci = self._completed_idx
        wpr = max(2.5, min(5, 50 / max(len(wp), 1)))

        # 1. Well boundary circle (at well center, not origin)
        if self._well_diam > 0:
            wcx, wcy = self._to_px(self._well_center_x, self._well_center_y)
            r_px = self._well_diam / 2 * s
            p.setPen(QPen(self.C_WELL, 1, Qt.PenStyle.DashLine))
            p.setBrush(Qt.BrushStyle.NoBrush)
            p.drawEllipse(QPointF(wcx, wcy), r_px, r_px)

        # 2. Completed print path (solid green)
        if ci > 1:
            p.setPen(QPen(self.C_DONE, 2))
            for i in range(1, min(ci, len(wp))):
                # Skip travel segments in print path
                if (i < len(self._wp_is_travel) and self._wp_is_travel[i]) or \
                   (i - 1 < len(self._wp_is_travel) and self._wp_is_travel[i - 1]):
                    continue
                p.drawLine(QPointF(*self._to_px(*wp[i - 1])),
                           QPointF(*self._to_px(*wp[i])))

        # 3. Upcoming print path (dashed yellow, next 30 segments)
        if ci < len(wp):
            p.setPen(QPen(self.C_NEXT, 1.5, Qt.PenStyle.DashLine))
            lo = max(0, ci - 1)
            hi = min(len(wp), ci + 30)
            for i in range(lo + 1, hi):
                if (i < len(self._wp_is_travel) and self._wp_is_travel[i]) or \
                   (i - 1 < len(self._wp_is_travel) and self._wp_is_travel[i - 1]):
                    continue
                p.drawLine(QPointF(*self._to_px(*wp[i - 1])),
                           QPointF(*self._to_px(*wp[i])))

        # 4. Travel segments (thin grey dotted, only if on-screen)
        p.setPen(QPen(self.C_TRAVEL, 1, Qt.PenStyle.DotLine))
        for i in range(1, len(wp)):
            if i >= len(self._wp_is_travel):
                break
            if not (self._wp_is_travel[i] or self._wp_is_travel[i - 1]):
                continue
            px0, py0 = self._to_px(*wp[i - 1])
            px1, py1 = self._to_px(*wp[i])
            if (px0 < -50 and px1 < -50) or (px0 > w + 50 and px1 > w + 50):
                continue
            if (py0 < -50 and py1 < -50) or (py0 > h + 50 and py1 > h + 50):
                continue
            p.drawLine(QPointF(px0, py0), QPointF(px1, py1))

        # 5. Waypoint dots (print waypoints only)
        for i, (wx, wy) in enumerate(wp):
            if i < len(self._wp_is_travel) and self._wp_is_travel[i]:
                continue
            px, py = self._to_px(wx, wy)
            if not (-30 < px < w + 30 and -30 < py < h + 30):
                continue
            if i < ci:
                p.setPen(QPen(self.C_WP_DONE, 1))
                p.setBrush(QBrush(self.C_WP_DONE))
            elif i == ci:
                p.setPen(QPen(self.C_WP_CURRENT, 1.5))
                p.setBrush(QBrush(self.C_WP_CURRENT))
            else:
                p.setPen(QPen(self.C_WP_EMPTY, 1))
                p.setBrush(Qt.BrushStyle.NoBrush)
            p.drawEllipse(QPointF(px, py), wpr, wpr)

        # 6. Actual needle trail (purple line — where the needle really went)
        trail = list(self._needle_trail)
        if len(trail) > 1:
            p.setPen(QPen(self.C_TRAIL, 1.5))
            for i in range(1, len(trail)):
                px0, py0 = self._to_px(*trail[i - 1])
                px1, py1 = self._to_px(*trail[i])
                # Skip off-screen segments
                if (px0 < -50 and px1 < -50) or (px0 > w + 50 and px1 > w + 50):
                    continue
                if (py0 < -50 and py1 < -50) or (py0 > h + 50 and py1 > h + 50):
                    continue
                p.drawLine(QPointF(px0, py0), QPointF(px1, py1))

        # 6.5 v7.3.2: Camera overlay (semi-transparent, true-to-scale)
        if (self._cam_overlay_enabled and self._cam_frame is not None
                and self._cam_um_per_px > 0 and self._nx is not None):
            cam_mm_per_px = self._cam_um_per_px / 1000.0
            cam_w_mm = self._cam_frame.width() * cam_mm_per_px
            cam_h_mm = self._cam_frame.height() * cam_mm_per_px
            # Camera top-left in stage mm (centered on needle)
            cam_tl_x = self._nx - cam_w_mm / 2
            cam_tl_y = self._ny - cam_h_mm / 2
            # Convert to pixel coords
            px_tl = self._to_px(cam_tl_x, cam_tl_y)
            px_br = self._to_px(cam_tl_x + cam_w_mm, cam_tl_y + cam_h_mm)
            target_rect = QRectF(
                px_tl[0], px_tl[1],
                px_br[0] - px_tl[0], px_br[1] - px_tl[1])
            p.setOpacity(0.5)
            p.drawImage(target_rect, self._cam_frame)
            p.setOpacity(1.0)
            # Draw camera FOV border
            p.setPen(QPen(QColor("#89b4fa"), 1, Qt.PenStyle.DashLine))
            p.setBrush(Qt.BrushStyle.NoBrush)
            p.drawRect(target_rect)

        # 7. Needle crosshair
        if self._nx is not None:
            nx, ny = self._to_px(self._nx, self._ny)
            p.setPen(QPen(self.C_NEEDLE, 2))
            p.drawLine(QPointF(nx - 10, ny), QPointF(nx + 10, ny))
            p.drawLine(QPointF(nx, ny - 10), QPointF(nx, ny + 10))
            p.setBrush(Qt.BrushStyle.NoBrush)
            p.drawEllipse(QPointF(nx, ny), 5, 5)

        # 8. Info overlay
        p.setPen(QColor("#6c7086"))
        p.setFont(QFont("Arial", 9))
        if self._nx is not None:
            p.drawText(6, h - 6, f"XY: ({self._nx:.1f}, {self._ny:.1f})")
        if wp:
            done_print = sum(1 for i in range(min(ci, len(wp)))
                             if i >= len(self._wp_is_travel) or not self._wp_is_travel[i])
            total_print = sum(1 for i in range(len(wp))
                              if i >= len(self._wp_is_travel) or not self._wp_is_travel[i])
            p.drawText(6, 14, f"Waypoints: {done_print}/{total_print}")
        if self._current_well:
            p.drawText(6, 28, f"Well: {self._current_well}")
        p.end()


# ═══════════════════════════════════════════════════════════════════
#  YZ Side View
# ═══════════════════════════════════════════════════════════════════

class YZSideView(QWidget):
    """Side view showing Z (needle height) vs Y position.

    Visualises needle tip entering/retracting from wells.
    Y axis = horizontal, Z axis = vertical (0 at top, deeper down).
    """

    C_NEEDLE = QColor("#f5c2e7"); C_PATH = QColor("#a6e3a1")
    C_WELL_WALL = QColor("#585b70"); C_BG = QColor("#1e1e2e")
    C_TRAVEL = QColor("#45475a")

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(120, 200)
        self.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Expanding)
        self._ny: float | None = None
        self._nz: float | None = None
        self._well_diam = 15.0
        self._well_depth = 17.0      # mm
        self._travel_z = 5.0         # mm above plate
        self._print_z = 0.1          # mm printing height
        self._z_history: deque[tuple[float, float]] = deque(maxlen=200)  # (y, z) pairs
        self._margin = 20

    def set_needle_position(self, y_mm, z_mm):
        self._ny = y_mm; self._nz = z_mm
        self._z_history.append((y_mm, z_mm))
        self.update()

    def set_well_geometry(self, diameter, depth=17.0, travel_z=5.0, print_z=0.1):
        self._well_diam = diameter; self._well_depth = depth
        self._travel_z = travel_z; self._print_z = print_z; self.update()

    def clear(self): self._z_history.clear(); self._ny = self._nz = None; self.update()

    def paintEvent(self, event):
        p = QPainter(self); p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.fillRect(self.rect(), self.C_BG)
        w, h = self.width(), self.height()
        m = self._margin

        # Z range: travel_z (top) to -well_depth (bottom)
        z_top = self._travel_z + 2
        z_bot = -self._well_depth * 0.3
        z_range = z_top - z_bot
        if z_range <= 0: z_range = 1

        def z_to_py(z_mm):
            """Z in mm → pixel Y. Positive Z = higher = higher on screen."""
            return m + (z_top - z_mm) / z_range * (h - 2 * m)

        # Draw plate surface line (Z=0)
        y_plate = z_to_py(0)
        p.setPen(QPen(QColor("#585b70"), 2))
        p.drawLine(QPointF(0, y_plate), QPointF(w, y_plate))
        p.setPen(QColor("#6c7086")); p.setFont(QFont("Arial", 8))
        p.drawText(QPointF(4, y_plate - 4), "plate surface")

        # Travel height line
        y_travel = z_to_py(self._travel_z)
        p.setPen(QPen(self.C_TRAVEL, 1, Qt.PenStyle.DotLine))
        p.drawLine(QPointF(0, y_travel), QPointF(w, y_travel))
        p.setPen(QColor("#6c7086"))
        p.drawText(QPointF(4, y_travel - 3), f"travel Z={self._travel_z:.1f}")

        # Print height line
        y_print = z_to_py(self._print_z)
        p.setPen(QPen(QColor("#a6e3a1"), 1, Qt.PenStyle.DotLine))
        p.drawLine(QPointF(0, y_print), QPointF(w, y_print))

        # Z history trail
        if len(self._z_history) > 1:
            p.setPen(QPen(self.C_PATH, 1.5))
            hist = list(self._z_history)
            for i in range(1, len(hist)):
                x0 = m + (i - 1) / max(len(hist) - 1, 1) * (w - 2 * m)
                x1 = m + i / max(len(hist) - 1, 1) * (w - 2 * m)
                y0 = z_to_py(hist[i-1][1])
                y1 = z_to_py(hist[i][1])
                p.drawLine(QPointF(x0, y0), QPointF(x1, y1))

        # Needle tip
        if self._nz is not None:
            nx = w / 2
            ny = z_to_py(self._nz)
            p.setPen(QPen(self.C_NEEDLE, 2))
            # Draw needle as a vertical line with tip
            p.drawLine(QPointF(nx, m), QPointF(nx, ny))
            # Tip triangle
            p.setBrush(QBrush(self.C_NEEDLE))
            from PySide6.QtGui import QPolygonF
            tip = QPolygonF([QPointF(nx, ny + 4), QPointF(nx - 3, ny - 2), QPointF(nx + 3, ny - 2)])
            p.drawPolygon(tip)

        # Z readout
        p.setPen(QColor("#cdd6f4")); p.setFont(QFont("Arial", 9))
        if self._nz is not None:
            p.drawText(6, h - 6, f"Z: {self._nz:.2f} mm")
        p.end()


# ═══════════════════════════════════════════════════════════════════
#  Syringe Pump Display
# ═══════════════════════════════════════════════════════════════════

class SyringePumpWidget(QWidget):
    """Single syringe pump visualization: fill bar + µL readout."""

    def __init__(self, pump_id: str = "P1", parent=None):
        super().__init__(parent)
        self.pump_id = pump_id
        self.setFixedWidth(60)
        self.setMinimumHeight(100)
        self._position_mm = 0.0       # Current plunger position
        self._zero_mm = 0.0           # Zero reference
        self._stroke_mm = 30.0        # Total syringe stroke
        self._uL_per_mm = 3.378       # Default for 250µL syringe
        self._ink_name = ""
        self._ink_color = QColor("#89b4fa")
        self._active = False
        self._enabled = False

    def set_config(self, enabled=True, stroke_mm=30.0, uL_per_mm=3.378,
                   ink_name="", ink_color="#89b4fa"):
        self._enabled = enabled; self._stroke_mm = stroke_mm
        self._uL_per_mm = uL_per_mm; self._ink_name = ink_name
        self._ink_color = QColor(ink_color); self.update()

    def set_position(self, pos_mm: float, zero_mm: float = 0.0):
        self._position_mm = pos_mm; self._zero_mm = zero_mm; self.update()

    def set_active(self, active: bool):
        self._active = active; self.update()

    def paintEvent(self, event):
        p = QPainter(self); p.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()

        # Background
        bg = QColor("#313244") if self._enabled else QColor("#1e1e2e")
        p.fillRect(self.rect(), bg)

        if not self._enabled:
            p.setPen(QColor("#585b70")); p.setFont(QFont("Arial", 10, QFont.Weight.Bold))
            p.drawText(self.rect(), Qt.AlignmentFlag.AlignCenter, f"{self.pump_id}\n—")
            p.end(); return

        # Syringe barrel
        bx, bw = 15, 30
        by, bh = 25, h - 55
        p.setPen(QPen(QColor("#6c7086"), 1))
        p.setBrush(QBrush(QColor("#45475a")))
        p.drawRoundedRect(QRectF(bx, by, bw, bh), 3, 3)

        # Fill level
        rel_pos = self._position_mm - self._zero_mm
        fill_frac = max(0, min(1, rel_pos / self._stroke_mm)) if self._stroke_mm > 0 else 0
        fill_h = bh * fill_frac
        if fill_h > 0:
            grad = QLinearGradient(bx, by + bh - fill_h, bx, by + bh)
            grad.setColorAt(0, self._ink_color.lighter(120))
            grad.setColorAt(1, self._ink_color)
            p.setBrush(QBrush(grad))
            p.setPen(Qt.PenStyle.NoPen)
            p.drawRoundedRect(QRectF(bx + 1, by + bh - fill_h, bw - 2, fill_h), 2, 2)

        # Active indicator
        if self._active:
            p.setPen(QPen(QColor("#f5c2e7"), 2))
            p.setBrush(Qt.BrushStyle.NoBrush)
            p.drawRoundedRect(QRectF(bx - 2, by - 2, bw + 4, bh + 4), 4, 4)

        # Pump label
        p.setPen(QColor("#cdd6f4")); p.setFont(QFont("Arial", 10, QFont.Weight.Bold))
        p.drawText(QRectF(0, 2, w, 20), Qt.AlignmentFlag.AlignCenter, self.pump_id)

        # µL readout
        uL = rel_pos * self._uL_per_mm
        p.setFont(QFont("Arial", 8))
        p.drawText(QRectF(0, h - 28, w, 14), Qt.AlignmentFlag.AlignCenter, f"{uL:.1f} µL")

        # Ink name
        if self._ink_name:
            p.setPen(QColor("#6c7086")); p.setFont(QFont("Arial", 7))
            p.drawText(QRectF(0, h - 14, w, 14), Qt.AlignmentFlag.AlignCenter,
                       self._ink_name[:8])
        p.end()

# ═══════════════════════════════════════════════════════════════════
#  Print Monitor Page
# ═══════════════════════════════════════════════════════════════════

class PrintMonitorPage(QWidget):
    """Print Monitor — v7.2.8 with direct position polling."""

    pause_requested = Signal()
    resume_requested = Signal()
    abort_requested = Signal()
    start_requested = Signal(object)

    def __init__(self, controller=None, settings=None, workspace=None, parent=None):
        super().__init__(parent)
        self._controller = controller
        self._settings = settings
        self._workspace = workspace or WorkspaceConfig()
        self._print_state = PrintState.IDLE
        self._print_start_time: float | None = None
        self._job_queue: list = []
        self._current_job = None
        self._context_widget = None
        self._hardware_config = None
        self._current_well_name = ""
        self._is_trajectory_job = False
        self._trajectory_waypoints = None
        self._total_duration_s = 0.0
        self._recorder = None
        from gui.unit_helpers import DEFAULT_XY_POSITION_SCALE
        self._xy_position_scale = DEFAULT_XY_POSITION_SCALE
        self._active_pump = "P1"

        self._build_ui()
        self._connect_signals()

    # ── Page interface ────────────────────────────────────────────

    def get_page_title(self) -> str: return "Print Monitor"
    def get_page_subtitle(self) -> str: return "Live print visualization"
    def set_hardware_config(self, config): self._hardware_config = config
    def set_xy_position_scale(self, v): self._xy_position_scale = v
    def set_recorder(self, r): self._recorder = r
    def _get_controller(self):
        return getattr(self, '_controller', None) or getattr(self, 'controller', None)

    # ── UI construction ───────────────────────────────────────────

    def _build_ui(self):
        outer = QVBoxLayout(self)
        outer.setContentsMargins(6, 6, 6, 6)
        outer.setSpacing(4)

        # ── TOP ROW: Plate | XY Detail | YZ Side ─────────────────
        top_split = QSplitter(Qt.Orientation.Horizontal)

        # Plate overview
        pg = QGroupBox("Plate Overview")
        pg.setStyleSheet(self._gs())
        pl = QVBoxLayout(pg)
        self.plate_view = PlateOverviewWidget()
        pl.addWidget(self.plate_view)
        leg = QHBoxLayout()
        for txt, c in [("Pending", "#585b70"), ("Active", "#f9e2af"),
                       ("Done", "#a6e3a1"), ("Error", "#f38ba8")]:
            l = QLabel(f"● {txt}"); l.setStyleSheet(f"color: {c}; font-size: 9px;")
            leg.addWidget(l)
        leg.addStretch()
        pl.addLayout(leg)
        top_split.addWidget(pg)

        # XY Detail
        xg = QGroupBox("XY Detail")
        xg.setStyleSheet(self._gs())
        xl = QVBoxLayout(xg)
        self.xy_detail = XYDetailView()
        xl.addWidget(self.xy_detail)
        dl = QHBoxLayout()
        for txt, c in [("── Done", "#a6e3a1"), ("╌╌ Next", "#f9e2af"),
                        ("── Trail", "#cba6f7"), ("✛ Needle", "#f5c2e7")]:
            l = QLabel(txt); l.setStyleSheet(f"color: {c}; font-size: 9px;")
            dl.addWidget(l)
        dl.addStretch()
        # v7.3.2: Camera overlay toggle + zoom controls
        self._chk_cam_overlay = QCheckBox("Camera")
        self._chk_cam_overlay.setStyleSheet("font-size: 9px;")
        self._chk_cam_overlay.setToolTip("Overlay live camera feed on XY view")
        self._chk_cam_overlay.toggled.connect(self._toggle_cam_overlay)
        dl.addWidget(self._chk_cam_overlay)
        btn_zoom_cam = QPushButton("Zoom Cam")
        btn_zoom_cam.setMaximumHeight(20)
        btn_zoom_cam.setStyleSheet("font-size: 9px; padding: 1px 4px;")
        btn_zoom_cam.setToolTip("Zoom to camera FOV")
        btn_zoom_cam.clicked.connect(lambda: self.xy_detail.zoom_to_camera())
        dl.addWidget(btn_zoom_cam)
        btn_zoom_plate = QPushButton("Zoom All")
        btn_zoom_plate.setMaximumHeight(20)
        btn_zoom_plate.setStyleSheet("font-size: 9px; padding: 1px 4px;")
        btn_zoom_plate.setToolTip("Zoom to fit all waypoints")
        btn_zoom_plate.clicked.connect(lambda: self.xy_detail.zoom_to_plate())
        dl.addWidget(btn_zoom_plate)
        xl.addLayout(dl)
        top_split.addWidget(xg)

        # YZ Side view
        yg = QGroupBox("YZ Side")
        yg.setStyleSheet(self._gs())
        yl = QVBoxLayout(yg)
        self.yz_view = YZSideView()
        yl.addWidget(self.yz_view)
        top_split.addWidget(yg)

        top_split.setStretchFactor(0, 2)
        top_split.setStretchFactor(1, 3)
        top_split.setStretchFactor(2, 1)
        outer.addWidget(top_split, stretch=3)

        # ── BOTTOM ROW: Syringes | Controls + Progress ───────────
        bot = QWidget()
        bot_lay = QHBoxLayout(bot)
        bot_lay.setContentsMargins(0, 0, 0, 0)
        bot_lay.setSpacing(8)

        # Syringe pumps
        syr_group = QGroupBox("Syringe Pumps")
        syr_group.setStyleSheet(self._gs())
        syr_lay = QHBoxLayout(syr_group)
        syr_lay.setSpacing(4)
        self._pump_widgets: dict[str, SyringePumpWidget] = {}
        for pid in ["P1", "P2", "P3"]:
            pw = SyringePumpWidget(pid)
            syr_lay.addWidget(pw)
            self._pump_widgets[pid] = pw
        bot_lay.addWidget(syr_group, stretch=1)

        # Controls + progress
        ctrl_group = QGroupBox("Print Controls")
        ctrl_group.setStyleSheet(self._gs())
        ctrl_lay = QVBoxLayout(ctrl_group)
        ctrl_lay.setSpacing(4)

        # Progress labels
        info = QGridLayout(); info.setSpacing(3)
        self._progress_labels: dict[str, QLabel] = {}
        for col, (lbl, key) in enumerate([("Job:", "job_name"), ("Well:", "well_info"),
                                           ("Step:", "step_info"), ("Time:", "time_info")]):
            nl = QLabel(lbl); nl.setStyleSheet(f"color: {COLORS.get('subtext0', '#a6adc8')}; font-size: 10px;")
            vl = QLabel("—"); vl.setStyleSheet(f"color: {COLORS.get('text', '#cdd6f4')}; font-size: 10px; font-weight: bold;")
            info.addWidget(nl, 0, col * 2); info.addWidget(vl, 0, col * 2 + 1)
            self._progress_labels[key] = vl
        ctrl_lay.addLayout(info)

        # Progress bar
        self.progress_bar = QProgressBar()
        self.progress_bar.setRange(0, 100); self.progress_bar.setValue(0)
        self.progress_bar.setMaximumHeight(18)
        self.progress_bar.setStyleSheet(f"""
            QProgressBar {{ background: {COLORS.get('surface0','#313244')};
                border: 1px solid {COLORS.get('surface1','#45475a')};
                border-radius: 3px; text-align: center;
                color: {COLORS.get('text','#cdd6f4')}; font-size: 9px; }}
            QProgressBar::chunk {{ background: {COLORS.get('green','#a6e3a1')}; border-radius: 2px; }}""")
        ctrl_lay.addWidget(self.progress_bar)

        # Buttons row
        br = QHBoxLayout()
        self.state_label = QLabel("IDLE")
        self.state_label.setStyleSheet(
            f"color: {COLORS.get('overlay0','#6c7086')}; font-size: 13px; font-weight: bold;")
        br.addWidget(self.state_label)
        br.addStretch()

        self.btn_pause = QPushButton("⏸ Pause")
        self.btn_pause.setMinimumHeight(30); self.btn_pause.setEnabled(False)
        self.btn_pause.setStyleSheet(
            f"QPushButton {{ background: {COLORS.get('yellow','#f9e2af')}; "
            f"color: {COLORS.get('crust','#11111b')}; font-weight: bold; "
            f"border-radius: 4px; padding: 4px 14px; }}")
        br.addWidget(self.btn_pause)

        self.btn_abort = QPushButton("⏹ Abort")
        self.btn_abort.setMinimumHeight(30); self.btn_abort.setEnabled(False)
        self.btn_abort.setStyleSheet(
            f"QPushButton {{ background: {COLORS.get('red','#f38ba8')}; "
            f"color: {COLORS.get('crust','#11111b')}; font-weight: bold; "
            f"border-radius: 4px; padding: 4px 14px; }}")
        br.addWidget(self.btn_abort)
        ctrl_lay.addLayout(br)

        bot_lay.addWidget(ctrl_group, stretch=2)
        outer.addWidget(bot, stretch=0)

    def _connect_signals(self):
        self.btn_pause.clicked.connect(self._on_pause)
        self.btn_abort.clicked.connect(self._on_abort)

    # ── v7.3.2: Camera overlay ─────────────────────────────────────

    def _toggle_cam_overlay(self, checked: bool):
        """Toggle camera overlay on XY detail view."""
        self.xy_detail.set_camera_overlay(checked)

    def _feed_camera_frame(self):
        """Capture latest camera frame and push to XY detail view.

        Called from on_status_update when camera overlay is enabled.
        """
        if not self._chk_cam_overlay.isChecked():
            return
        # Get camera widget from calibration page (shared camera)
        # Access through main window's page list
        try:
            main_win = self.window()
            cal_page = None
            if hasattr(main_win, '_page_widgets'):
                for pg in main_win._page_widgets:
                    if hasattr(pg, '_cameras') and pg._cameras:
                        cal_page = pg
                        break
            if cal_page is None:
                return
            for cam in cal_page._cameras:
                if cam.isVisible() and hasattr(cam, '_last_frame') and cam._last_frame is not None:
                    import numpy as np
                    frame = cam._last_frame  # BGR numpy array
                    h, w = frame.shape[:2]
                    if frame.ndim == 3:
                        # BGR → RGB
                        rgb = frame[:, :, ::-1].copy()
                        qimg = QImage(rgb.data, w, h, 3 * w, QImage.Format.Format_RGB888)
                    else:
                        qimg = QImage(frame.data, w, h, w, QImage.Format.Format_Grayscale8)
                    # Get um_per_px from hardware config
                    um_per_px = 0.0
                    if self._hardware_config and hasattr(self._hardware_config, 'camera_config'):
                        cc = self._hardware_config.camera_config
                        if cc and hasattr(cc, 'micron_per_pixel') and cc.micron_per_pixel:
                            um_per_px = cc.micron_per_pixel
                    self.xy_detail.set_camera_frame(qimg.copy(), um_per_px)
                    break
        except Exception as e:
            logger.debug(f"Camera overlay feed error: {e}")

    # ── Context panel ─────────────────────────────────────────────

    def get_context_widget(self) -> QWidget:
        if self._context_widget: return self._context_widget
        ctx = QWidget()
        lay = QVBoxLayout(ctx); lay.setContentsMargins(4, 4, 4, 4); lay.setSpacing(6)

        self._ctx_btn_start = QPushButton("▶ Start Print")
        self._ctx_btn_start.setMinimumHeight(34); self._ctx_btn_start.setEnabled(False)
        self._ctx_btn_start.setStyleSheet(
            f"QPushButton {{ background: {COLORS.get('green','#a6e3a1')}; "
            f"color: {COLORS.get('crust','#11111b')}; font-weight: bold; "
            f"border-radius: 4px; padding: 6px; font-size: 12px; }}"
            f"QPushButton:disabled {{ background: {COLORS.get('surface1','#45475a')}; "
            f"color: {COLORS.get('overlay0','#6c7086')}; }}")
        self._ctx_btn_start.clicked.connect(self._on_ctx_start)
        lay.addWidget(self._ctx_btn_start)

        lay.addWidget(QLabel("Job Queue"))
        self._queue_list = QListWidget(); self._queue_list.setMaximumHeight(120)
        self._queue_list.setStyleSheet(
            f"QListWidget {{ background: {COLORS.get('surface0','#313244')}; "
            f"color: {COLORS.get('text','#cdd6f4')}; border-radius: 4px; font-size: 10px; }}")
        lay.addWidget(self._queue_list)

        qb = QHBoxLayout()
        br = QPushButton("Remove"); br.setMaximumHeight(22); br.clicked.connect(self._on_ctx_remove)
        bc = QPushButton("Clear"); bc.setMaximumHeight(22); bc.clicked.connect(self._on_ctx_clear)
        qb.addWidget(br); qb.addWidget(bc)
        lay.addLayout(qb)

        sep = QFrame(); sep.setFrameShape(QFrame.Shape.HLine)
        lay.addWidget(sep)

        self.needle_label = QLabel("Needle: —")
        self.needle_label.setStyleSheet(f"color: {COLORS.get('subtext0','#a6adc8')}; font-size: 10px;")
        lay.addWidget(self.needle_label)

        lay.addStretch()
        self._context_widget = ctx
        return ctx

    # ── Job management ────────────────────────────────────────────

    def receive_job(self, job):
        self._job_queue.append(job)
        if self._current_job is None: self._current_job = job
        self._refresh_queue()
        if "job_name" in self._progress_labels:
            self._progress_labels["job_name"].setText(job.name)
        if "step_info" in self._progress_labels:
            self._progress_labels["step_info"].setText(f"0 / {getattr(job, 'total_steps', 0):,}")
        self._load_job_waypoints(job)
        logger.info(f"Job received: {job.name} ({len(self._job_queue)} in queue)")

    def setup_plate(self, plate, well_roles=None, print_wells=None, well_diameter_mm=0.0):
        self.plate_view.set_plate(plate)
        if well_roles: self.plate_view.set_well_roles(well_roles)
        if print_wells: self.plate_view.set_all_pending(print_wells)
        if well_diameter_mm > 0:
            self.xy_detail.set_well_diameter(well_diameter_mm)
            depth = getattr(plate, 'well_depth_mm', 17.0)
            self.yz_view.set_well_geometry(well_diameter_mm, depth)
        # Configure syringe widgets from hardware config
        self._configure_pumps()
        logger.info(f"Plate setup: {len(print_wells or [])} print wells, diam={well_diameter_mm:.1f}")

    def _configure_pumps(self):
        """Configure syringe pump widgets from HardwareConfig."""
        hw = self._hardware_config
        if hw is None: return
        pumps = getattr(hw, 'pumps', {})
        for pid, widget in self._pump_widgets.items():
            pcfg = pumps.get(pid)
            if pcfg and getattr(pcfg, 'enabled', False):
                syringe = getattr(pcfg, 'syringe', None)
                ink = getattr(pcfg, 'ink', None)
                stroke = getattr(syringe, 'stroke_mm', 30.0) if syringe else 30.0
                uL_mm = getattr(syringe, 'uL_per_mm', 3.378) if syringe else 3.378
                ink_name = getattr(ink, 'name', '') if ink else ''
                ink_color = getattr(ink, 'display_color', '#89b4fa') if ink else '#89b4fa'
                widget.set_config(True, stroke, uL_mm, ink_name, ink_color)
            else:
                widget.set_config(enabled=False)

    def _load_job_waypoints(self, job):
        """Load trajectory waypoints for full-path visualization.

        Stores ALL waypoints (print + travel) with per-waypoint metadata
        (well name, travel flag) so the XY detail view can:
        - Center on the current well
        - Draw travel segments differently
        - Track completed waypoints robustly across well transitions

        Prefers trajectory_waypoints, falls back to command extraction.
        """
        # ── v7.2.9: Hybrid estimate — use step-based progress, not time ──
        hybrid_est = getattr(job, 'estimated_duration_s', 0.0)
        if hybrid_est > 0:
            self._is_trajectory_job = False
            self._total_duration_s = hybrid_est
            logger.info(f"Hybrid time estimate: {hybrid_est:.1f}s (step-based progress)")

        # ── Prefer trajectory waypoints ───────────────────────────
        traj_wps = getattr(job, 'trajectory_waypoints', None)
        if traj_wps and len(traj_wps) > 0:
            self._trajectory_waypoints = traj_wps

            if hybrid_est <= 0:
                self._is_trajectory_job = True

            # Build full waypoint list with metadata for XY detail
            xy_all = [(wp.x, wp.y) for wp in traj_wps]
            is_travel = [getattr(wp, 'is_travel', False) for wp in traj_wps]
            wells = [getattr(wp, 'well', '') for wp in traj_wps]

            self.xy_detail.set_trajectory(xy_all, is_travel=is_travel, wells=wells)

            # Z profile for YZ view
            z_profile = [(wp.y, wp.z) for wp in traj_wps]
            if z_profile and hasattr(self.yz_view, '_z_history'):
                self.yz_view._z_history.clear()
                for y, z in z_profile[::3]:
                    self.yz_view._z_history.append((y, z))
                self.yz_view.update()

            # Total duration — hybrid estimate takes precedence
            if hybrid_est <= 0:
                traj_result = getattr(job, 'trajectory_result', None)
                if traj_result:
                    self._total_duration_s = getattr(traj_result, 'total_duration_s', 0)
                elif traj_wps:
                    self._total_duration_s = traj_wps[-1].t if traj_wps[-1].t > 0 else 0

            n_print = sum(1 for t in is_travel if not t)
            logger.info(
                f"Trajectory loaded: {len(traj_wps)} waypoints, "
                f"{n_print} print points, "
                f"{self._total_duration_s:.1f}s total, "
                f"progress={'time' if self._is_trajectory_job else 'step'}")
            return

        # ── Fallback: extract from commands ───────────────────────
        self._is_trajectory_job = False
        self._trajectory_waypoints = None
        self._total_duration_s = 0
        waypoints = []
        try:
            from SupportClasses.PrintManager import CommandType
            for cmd in job.commands:
                if cmd.type == CommandType.MOVE_XY:
                    waypoints.append((cmd.params.get("x", 0), cmd.params.get("y", 0)))
                elif cmd.type == CommandType.PRINT_PATH:
                    for pt in cmd.params.get("points", []):
                        if len(pt) >= 2:
                            waypoints.append((pt[0], pt[1]))
        except Exception as exc:
            logger.debug(f"Waypoint extraction: {exc}")

        if waypoints:
            self.xy_detail.set_trajectory(waypoints)
            logger.info(f"Loaded {len(waypoints)} command waypoints")
    def _refresh_queue(self):
        if not hasattr(self, '_queue_list') or self._queue_list is None: return
        self._queue_list.clear()
        for j in self._job_queue:
            pfx = "▶ " if j is self._current_job else "  "
            self._queue_list.addItem(f"{pfx}{j.name}  [{getattr(j, 'total_steps', '?')} steps]")
        if hasattr(self, '_ctx_btn_start') and self._ctx_btn_start:
            self._ctx_btn_start.setEnabled(
                self._current_job is not None and
                self._print_state in (PrintState.IDLE, PrintState.COMPLETED,
                                      PrintState.ABORTED, PrintState.ERROR))

    # ── Button handlers ───────────────────────────────────────────

    def _on_pause(self):
        if self._print_state == PrintState.RUNNING: self.pause_requested.emit()
        elif self._print_state == PrintState.PAUSED: self.resume_requested.emit()

    def _on_abort(self):
        if self._print_state in (PrintState.RUNNING, PrintState.PAUSED):
            self.abort_requested.emit()

    def _on_ctx_start(self):
        if self._current_job and self._print_state in (
            PrintState.IDLE, PrintState.COMPLETED, PrintState.ABORTED, PrintState.ERROR):
            self.start_requested.emit(self._current_job)

    def _on_ctx_remove(self):
        if not hasattr(self, '_queue_list'): return
        r = self._queue_list.currentRow()
        if 0 <= r < len(self._job_queue):
            rm = self._job_queue.pop(r)
            if rm is self._current_job:
                self._current_job = self._job_queue[0] if self._job_queue else None
            self._refresh_queue()

    def _on_ctx_clear(self):
        self._job_queue.clear(); self._current_job = None; self._refresh_queue()

    # ── Progress / state ──────────────────────────────────────────

    def on_print_progress(self, step: int, total: int, message: str):
        well = ""
        wm = re.search(r'[Ww]ell\s+([A-P]\d{1,2})', message)
        if wm:
            well = wm.group(1)
            self._current_well_name = well
            self.plate_view.set_active_well(well)
            # Re-center XY detail on the new well (message-based detection
            # complements position-based detection in on_status_update)
            self.xy_detail.focus_well(well)

        # Parse active pump from message
        pm = re.search(r'\b(P[123])\b', message)
        if pm:
            self._active_pump = pm.group(1)

        # Time-based progress for trajectory jobs, step-based otherwise
        if self._is_trajectory_job and self._total_duration_s > 0 and self._print_start_time:
            import time as _time
            elapsed = _time.time() - self._print_start_time
            pct = min(100, int(100 * elapsed / self._total_duration_s))
        else:
            pct = int(100 * step / max(total, 1))
        self.progress_bar.setValue(pct)
        self.progress_bar.setFormat(f"{pct}%  {message[:35]}")

        self._progress_labels.get("job_name", QLabel()).setText(
            self._current_job.name if self._current_job else "—")
        self._progress_labels.get("well_info", QLabel()).setText(well or "—")
        self._progress_labels.get("step_info", QLabel()).setText(f"{step:,} / {total:,}")

        if self._print_start_time:
            el = time.time() - self._print_start_time
            if self._total_duration_s > 0:
                rem = max(0, self._total_duration_s - el)
                self._progress_labels.get("time_info", QLabel()).setText(
                    f"{self._ft(el)} / ~{self._ft(rem)}")
            elif pct > 0:
                rem = el / (pct / 100) - el
                self._progress_labels.get("time_info", QLabel()).setText(
                    f"{self._ft(el)} / ~{self._ft(rem)}")
            else:
                self._progress_labels.get("time_info", QLabel()).setText(
                    f"{self._ft(el)} / —")

        # Update active pump indicator
        for pid, pw in self._pump_widgets.items():
            pw.set_active(pid == self._active_pump)

    def on_print_state_changed(self, state):
        self._print_state = state
        styles = {
            PrintState.IDLE: ("IDLE", "overlay0"), PrintState.RUNNING: ("RUNNING", "green"),
            PrintState.PAUSED: ("PAUSED", "yellow"), PrintState.COMPLETED: ("COMPLETED", "green"),
            PrintState.ABORTED: ("ABORTED", "red"), PrintState.ERROR: ("ERROR", "red"),
        }
        nm, ck = styles.get(state, (str(state), "overlay0"))
        self.state_label.setText(nm)
        self.state_label.setStyleSheet(
            f"color: {COLORS.get(ck, '#a6adc8')}; font-size: 13px; font-weight: bold;")

        if state == PrintState.RUNNING:
            self.btn_pause.setText("⏸ Pause"); self.btn_pause.setEnabled(True)
            if self._print_start_time is None: self._print_start_time = time.time()
        elif state == PrintState.PAUSED:
            self.btn_pause.setText("▶ Resume"); self.btn_pause.setEnabled(True)
        else:
            self.btn_pause.setText("⏸ Pause"); self.btn_pause.setEnabled(False)

        self.btn_abort.setEnabled(state in (PrintState.RUNNING, PrintState.PAUSED))

        if state in (PrintState.COMPLETED, PrintState.ABORTED, PrintState.ERROR):
            self._print_start_time = None
        if state == PrintState.COMPLETED:
            self.progress_bar.setValue(100); self._advance_queue()
        self._refresh_queue()

    def _advance_queue(self):
        if self._current_job in self._job_queue: self._job_queue.remove(self._current_job)
        self._current_job = self._job_queue[0] if self._job_queue else None
        self._refresh_queue()
        logger.info(f"Queue: {'next=' + self._current_job.name if self._current_job else 'empty'}")

    # ── Position polling (300ms from app.py) ─────────────────────

    def on_status_update(self):
        """Called by MainWindow timer (~300ms). Update needle position and tracking.

        Polls the controller for current position, updates all visualization
        widgets, and performs robust waypoint-to-position matching with
        well transition detection.
        """
        if self._print_state != PrintState.RUNNING:
            return
        ctrl = self._get_controller()
        if ctrl is None:
            return

        try:
            xy = ctrl.get_xy_position(cached=True) if getattr(ctrl, 'is_xy_connected', False) else None
            zp = ctrl.get_zp_position(cached=True) if getattr(ctrl, 'is_zp_connected', False) else None
        except Exception:
            return

        if xy and xy[0] is not None:
            zero = getattr(ctrl, 'zero_position', {})
            px = (xy[0] - zero.get('x', 0)) / 1000.0
            py = (xy[1] - zero.get('y', 0)) / 1000.0
            pz = (zp[0] - zero.get('Z', 0)) if zp and zp[0] is not None else 0.0

            # Update all views (set_needle_position appends to trail)
            self.plate_view.set_needle_position(px, py)
            self.xy_detail.set_needle_position(px, py)
            self.yz_view.set_needle_position(py, pz)

            # v7.3.2: Feed camera frame to XY overlay
            self._feed_camera_frame()

            # Position-based trajectory progress — robust two-phase search
            wps = self.xy_detail._waypoints
            if wps and len(wps) > 1:
                cur_idx = self.xy_detail._completed_idx
                best_idx = self._find_nearest_waypoint(px, py, wps, cur_idx)

                # Only advance forward (never regress)
                if best_idx > cur_idx:
                    self.xy_detail.set_completed_index(best_idx)

                    # Detect well transition from waypoint metadata
                    wp_wells = self.xy_detail._wp_well
                    if wp_wells and best_idx < len(wp_wells):
                        new_well = wp_wells[best_idx]
                        if new_well and new_well != self.xy_detail._current_well:
                            self.xy_detail.focus_well(new_well)
                            logger.debug(f"XY view: focused on well {new_well}")

            # Positional well detection fallback — if the needle has
            # drifted far from the current view center, find the nearest
            # well center and re-focus.  This handles cases where waypoint
            # index matching fails (e.g., same pattern in multiple wells).
            well_centers = self.xy_detail._well_centers
            if well_centers and self.xy_detail._well_diam > 0:
                vcx = self.xy_detail._view_cx
                vcy = self.xy_detail._view_cy
                drift_sq = (px - vcx) ** 2 + (py - vcy) ** 2
                threshold_sq = (self.xy_detail._well_diam * 1.2) ** 2
                if drift_sq > threshold_sq:
                    best_well = ""
                    best_d = float('inf')
                    for name, (cx, cy) in well_centers.items():
                        d = (px - cx) ** 2 + (py - cy) ** 2
                        if d < best_d:
                            best_d = d
                            best_well = name
                    if best_well and best_well != self.xy_detail._current_well:
                        self.xy_detail.focus_well(best_well)
                        logger.debug(f"XY view: proximity focus → {best_well}")

            # Update pump positions
            if zp:
                for i, pid in enumerate(["P1", "P2", "P3"], 1):
                    if i < len(zp) and zp[i] is not None:
                        pw = self._pump_widgets.get(pid)
                        if pw:
                            pw.set_position(zp[i], zero.get(pid, 0))

    def _find_nearest_waypoint(self, px, py, waypoints, current_idx):
        """Find the nearest waypoint to the current needle position.

        Two-phase search for robustness:
        1. Local: search current_idx to current_idx + 200 (normal progression)
        2. Forward-only global: search current_idx to end if local match is
           too far (handles well-to-well jumps across service sequences)

        Both phases only search FORWARD from current_idx to avoid matching
        earlier waypoints in a different well that happen to share the same
        spatial pattern (same object printed in multiple wells).

        Returns the index of the best matching waypoint (>= current_idx).
        """
        n = len(waypoints)
        if n == 0:
            return 0

        best_idx = current_idx
        best_dist = float('inf')

        # Phase 1: Local forward search (current to current + 200)
        hi = min(n, current_idx + 200)
        for i in range(current_idx, hi):
            dx = waypoints[i][0] - px
            dy = waypoints[i][1] - py
            d = dx * dx + dy * dy
            if d < best_dist:
                best_dist = d
                best_idx = i

        # Phase 2: Extended forward search if local match is > 2mm away
        if best_dist > 4.0:
            for i in range(current_idx, n):
                dx = waypoints[i][0] - px
                dy = waypoints[i][1] - py
                d = dx * dx + dy * dy
                if d < best_dist:
                    best_dist = d
                    best_idx = i

        return best_idx

    # ── Helpers ───────────────────────────────────────────────────

    @staticmethod
    def _ft(s):
        s = max(0, s)
        if s < 3600: return f"{int(s)//60}:{int(s)%60:02d}"
        return f"{int(s)//3600}:{(int(s)%3600)//60:02d}:{int(s)%60:02d}"

    @staticmethod
    def _gs():
        return (f"QGroupBox {{ color: {COLORS.get('text','#cdd6f4')}; font-weight: bold; "
                f"border: 1px solid {COLORS.get('surface1','#45475a')}; border-radius: 4px; "
                f"margin-top: 6px; padding-top: 14px; }}")

    def _update_needle_info(self):
        ws = self._workspace
        needle = getattr(ws, 'needle', None)
        if needle and hasattr(self, 'needle_label'):
            self.needle_label.setText(
                f"Needle: {getattr(needle,'gauge','?')}G × "
                f"{getattr(needle,'length_inches','?')}\"  "
                f"ID: {getattr(needle,'id_um',0):.0f} µm")

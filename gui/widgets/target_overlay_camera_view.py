"""
target_overlay_camera_view.py — CameraFeedView subclass with target overlays.

v7.3.3: Extends CameraFeedView to draw pick-and-place target markers
on top of the live camera feed. Target circles are drawn at their
stage coordinates, converted to image pixels using the camera's
um_per_px and current stage position.

Used by the Pick & Place target selection page. The base CameraFeedView
(used by calibration page) is NOT modified.
"""

from __future__ import annotations

import logging
from typing import Optional

from PySide6.QtCore import Qt, QPointF
from PySide6.QtGui import QImage, QPixmap, QPainter, QPen, QColor, QBrush, QFont

from gui.styles import COLORS
from gui.widgets.camera_feed_view import CameraFeedView
from SupportClasses.PickAndPlaceManager import PickPlaceTarget

logger = logging.getLogger(__name__)


class TargetOverlayCameraView(CameraFeedView):
    """CameraFeedView with target marker overlays.

    Draws circles at target positions (stage coords) on the live feed,
    with size scaled by um_per_px. Supports hover/selection state coloring.
    """

    def __init__(self, camera_manager=None, cam_idx: int = 0,
                 show_crosshair: bool = True, label: str = "",
                 parent=None):
        super().__init__(camera_manager, cam_idx, show_crosshair, label, parent)
        self._targets: list[PickPlaceTarget] = []
        self._stage_x_um: float = 0.0
        self._stage_y_um: float = 0.0
        self._um_per_px: float = 1.67
        self._selected_target: Optional[str] = None

    # ── Public API ───────────────────────────────────────────────

    def set_targets(self, targets: list[PickPlaceTarget]):
        """Update the target list for overlay drawing."""
        self._targets = list(targets)

    def set_stage_position(self, x_um: float, y_um: float):
        """Set the current camera/stage center position."""
        self._stage_x_um = x_um
        self._stage_y_um = y_um

    def set_um_per_px(self, value: float):
        """Set microns per pixel for coordinate conversion."""
        if value > 0:
            self._um_per_px = value

    def set_selected_target(self, target_id: Optional[str]):
        """Highlight a specific target."""
        self._selected_target = target_id

    # ── Rendering override ───────────────────────────────────────

    def _render_frame(self, q_img: QImage):
        """Override: draw crosshair + target overlays, then scale and display."""
        pixmap = QPixmap.fromImage(q_img)

        # Draw crosshair on full-res pixmap
        if self._show_crosshair:
            self._draw_crosshair(pixmap)

        # Draw target overlays on full-res pixmap
        self._draw_targets(pixmap)

        # Scale to fit display label
        display_size = self._display.size()
        if display_size.width() < 1 or display_size.height() < 1:
            return
        scaled = pixmap.scaled(
            display_size,
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation,
        )
        self._last_pixmap = scaled
        self._display.setPixmap(scaled)

    def _draw_targets(self, pixmap: QPixmap):
        """Draw target markers at their stage positions on the pixmap."""
        if not self._targets:
            return

        img_w = pixmap.width()
        img_h = pixmap.height()
        if img_w == 0 or img_h == 0:
            return

        painter = QPainter(pixmap)
        painter.setRenderHint(QPainter.Antialiasing)

        for target in self._targets:
            # Convert stage coords → image pixel coords
            # Camera center = stage position = image center
            ix = (target.x_um - self._stage_x_um) / self._um_per_px + img_w / 2
            iy = (target.y_um - self._stage_y_um) / self._um_per_px + img_h / 2

            # Skip targets outside the frame
            if ix < -50 or iy < -50 or ix > img_w + 50 or iy > img_h + 50:
                continue

            # Radius in image pixels
            if target.size_um > 0:
                r_px = (target.size_um / 2) / self._um_per_px
                r_px = max(6, r_px)
            else:
                r_px = 8

            # Color based on state
            if target.target_id == self._selected_target:
                color = QColor(COLORS["mauve"])
                pen_width = 3
            elif target.selected:
                color = QColor(COLORS["green"])
                pen_width = 2
            else:
                color = QColor(COLORS["overlay0"])
                pen_width = 1

            painter.setPen(QPen(color, pen_width))
            painter.setBrush(QBrush(QColor(
                color.red(), color.green(), color.blue(), 40)))
            painter.drawEllipse(QPointF(ix, iy), r_px, r_px)

            # ID label
            painter.setPen(QPen(color, 1))
            painter.setFont(QFont("Consolas", 8))
            painter.drawText(int(ix + r_px + 3), int(iy - 2),
                             target.target_id)

        painter.end()

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
                 auto_orient: bool = True, parent=None):
        # Forward by KEYWORD: the base signature now has `enable_settings`
        # before `parent`, so positional forwarding would bind `parent` to
        # `enable_settings`. Pick/place overlays don't want the HW-settings gear.
        #
        # v7.10: ``auto_orient`` defaults ON and is now FORWARDED. It previously
        # could not be set at all, so every pick/place live view (spheroid, cell
        # targeting, cell labeling) rendered RAW while the microscope feed on
        # the neighbouring page rendered corrected — the same camera, two
        # orientations. The base class inverts clicks back to raw frame pixels
        # (``_widget_to_image``), so ``pixel_to_stage_offset`` and every target
        # coordinate are unaffected; only what the operator sees changes.
        super().__init__(camera_manager=camera_manager, cam_idx=cam_idx,
                         show_crosshair=show_crosshair, label=label,
                         enable_settings=False, auto_orient=auto_orient,
                         parent=parent)
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
        """Override: draw crosshair + target overlays, then scale and display.

        v7.15: goes through the base class's ``_compose_pixmap`` instead of
        building the pixmap itself. It previously skipped ``_orient_qimage``
        entirely, which left ``_view_true_xform`` unset and
        ``_displayed_image_size`` at (0, 0) — the clicks only stayed correct
        because ``_widget_to_image`` had fallbacks for exactly that. Zoom
        would not have applied here at all, and this is the click-to-select
        surface (spheroid pickup, cell targeting, cell labeling).
        """
        pixmap, _true_xf = self._compose_pixmap(q_img)

        # Draw crosshair on full-res pixmap
        if self._show_crosshair:
            self._draw_crosshair(pixmap)

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
        # Publish BEFORE drawing targets: _draw_targets projects through the
        # geometry, so it must describe the pixmap it is about to paint on.
        self._publish_geometry(scaled)
        self._draw_targets(scaled)
        self._display.setPixmap(scaled)

    def _draw_targets(self, pixmap: QPixmap):
        """Draw target markers at their stage positions on the pixmap.

        v7.15: the pixmap handed in is the SCALED one, and the projection runs
        stage µm → RAW frame px → pixmap px through the shared
        :class:`ViewGeometry`. Previously it went straight to full-resolution
        image pixels with the frame centre assumed at ``w/2`` and no view
        transform at all — correct only on an unrotated, unzoomed camera.
        """
        if not self._targets:
            return

        img_w = pixmap.width()
        img_h = pixmap.height()
        if img_w == 0 or img_h == 0:
            return
        geo = self.geometry_map()
        raw_w, raw_h = (geo.raw_size if geo.raw_size[0]
                        else self._last_image_size)
        if not raw_w or not raw_h:
            return
        scale = geo.widget_scale() if geo.valid else 1.0

        painter = QPainter(pixmap)
        painter.setRenderHint(QPainter.Antialiasing)

        for target in self._targets:
            # stage µm → RAW frame px (camera centre = stage position)
            rx = (target.x_um - self._stage_x_um) / self._um_per_px + raw_w / 2
            ry = (target.y_um - self._stage_y_um) / self._um_per_px + raw_h / 2
            pt = geo.to_pixmap(rx, ry) if geo.valid else (rx, ry)
            if pt is None:
                continue
            ix, iy = pt

            # Skip targets outside the frame
            if ix < -50 or iy < -50 or ix > img_w + 50 or iy > img_h + 50:
                continue

            # Radius in PIXMAP pixels — scales with the zoom, as the picture does
            if target.size_um > 0:
                r_px = max(6, (target.size_um / 2) / self._um_per_px * scale)
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

"""
measurement_camera_view.py — Camera feed with a draggable measurement line.

Subclasses `CameraFeedView` and adds a two-endpoint measurement overlay
used by the objective calibration workflow. The user clicks two points
on the live feed (typically the ends of a known feature on a calibration
slide), drags the endpoints around to refine, and the widget reports
the pixel distance. The caller converts that distance to µm/px using
the user-supplied real-world distance.

Endpoint coordinates are stored in *frame pixels* (floats — sub-pixel
precision survives drag), and re-projected onto the displayed pixmap
on every paint so the overlay tracks resolution changes and window
resizes correctly.
"""

from __future__ import annotations

import math
import logging
from typing import Optional

from PySide6.QtCore import Qt, Signal, QEvent, QPointF, QRectF
from PySide6.QtGui import QImage, QPixmap, QPainter, QPen, QBrush, QColor, QFont

from gui.scaling import s, scaled_font_size
from gui.styles import COLORS
from gui.widgets.camera_feed_view import CameraFeedView

logger = logging.getLogger(__name__)

# Snap radius in widget pixels — within this distance the click grabs an
# existing endpoint rather than placing/ignoring.
_SNAP_RADIUS_WIDGET_PX = 12


class MeasurementCameraView(CameraFeedView):
    """Live camera feed with two draggable measurement endpoints.

    Signals:
        endpoints_changed: emitted whenever p1 or p2 changes (placement,
            drag, reset).
    """

    endpoints_changed = Signal()

    def __init__(self, camera_manager=None, cam_idx: int = 0,
                 label: str = "", auto_orient: bool = False, parent=None):
        # Crosshair would clutter the measurement line; suppress it.
        # v7.5.x: ``auto_orient`` is now FORWARDED (it previously could not be set
        # at all through this subclass), so a measurement view can show the
        # camera's calibrated orientation like every other microscope feed.
        # Default stays False — a measurement made on a transformed view must be
        # opted into deliberately by the caller.
        super().__init__(
            camera_manager=camera_manager,
            cam_idx=cam_idx,
            show_crosshair=False,
            label=label,
            auto_orient=auto_orient,
            # Measurement is itself a calibration step — no HW-settings gear
            # (a resolution change would invalidate the measurement).
            enable_settings=False,
            parent=parent,
        )
        self._p1: Optional[tuple[float, float]] = None
        self._p2: Optional[tuple[float, float]] = None
        self._dragging: Optional[int] = None  # 0 = p1, 1 = p2, None = free
        # Cache for endpoint hit-testing — populated each paint.
        self._last_widget_offset: tuple[float, float] = (0.0, 0.0)
        self._last_widget_scale: float = 1.0

    # ── Public API ────────────────────────────────────────────────

    def points(self) -> tuple[Optional[tuple[float, float]],
                              Optional[tuple[float, float]]]:
        """Return (p1, p2) in frame-pixel coordinates."""
        return (self._p1, self._p2)

    def pixel_distance(self) -> Optional[float]:
        """Pixel distance between p1 and p2 in frame coords, or None."""
        if self._p1 is None or self._p2 is None:
            return None
        return math.hypot(self._p2[0] - self._p1[0], self._p2[1] - self._p1[1])

    def reset(self) -> None:
        """Clear both endpoints."""
        self._p1 = None
        self._p2 = None
        self._dragging = None
        self.endpoints_changed.emit()
        if self._last_qimage is not None:
            self._render_frame(self._last_qimage)

    # ── Rendering ─────────────────────────────────────────────────

    def _render_frame(self, q_img: QImage):
        """Render the frame, then overlay measurement endpoints + line."""
        # Build the scaled pixmap exactly like the base class does.
        pixmap = QPixmap.fromImage(q_img)
        display_size = self._display.size()
        if display_size.width() < 1 or display_size.height() < 1:
            return
        scaled = pixmap.scaled(
            display_size,
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation,
        )
        # Compute frame→widget transform; cached so eventFilter can
        # hit-test endpoints without recomputing.
        img_w, img_h = self._last_image_size
        if img_w > 0 and img_h > 0:
            self._last_widget_scale = scaled.width() / img_w
        else:
            self._last_widget_scale = 1.0
        # Pixmap is centered inside the QLabel (KeepAspectRatio + AlignCenter).
        lbl_w, lbl_h = self._display.width(), self._display.height()
        ox = (lbl_w - scaled.width()) / 2.0
        oy = (lbl_h - scaled.height()) / 2.0
        self._last_widget_offset = (ox, oy)

        # Draw endpoints + line directly onto the scaled pixmap so the
        # marker sizes are widget-pixels (crisp regardless of resolution).
        self._draw_measurement(scaled)

        self._last_pixmap = scaled
        self._display.setPixmap(scaled)

    def _draw_measurement(self, pixmap: QPixmap) -> None:
        if self._p1 is None and self._p2 is None:
            return
        painter = QPainter(pixmap)
        try:
            painter.setRenderHint(QPainter.RenderHint.Antialiasing, True)
            scale = self._last_widget_scale
            # Endpoints in pixmap coords (pixmap origin = (0,0) of the
            # scaled pixmap — widget offset is applied by the QLabel).
            p1_pm = self._p1 and (self._p1[0] * scale, self._p1[1] * scale)
            p2_pm = self._p2 and (self._p2[0] * scale, self._p2[1] * scale)

            if p1_pm and p2_pm:
                line_pen = QPen(QColor(COLORS.get("lavender", "#b4befe")))
                line_pen.setWidth(s(2))
                painter.setPen(line_pen)
                painter.drawLine(
                    QPointF(p1_pm[0], p1_pm[1]),
                    QPointF(p2_pm[0], p2_pm[1]),
                )
                # Midpoint distance label.
                midx = (p1_pm[0] + p2_pm[0]) / 2.0
                midy = (p1_pm[1] + p2_pm[1]) / 2.0
                dist = self.pixel_distance() or 0.0
                self._draw_label(painter, midx, midy, f"{dist:.1f} px")

            dot_color = QColor(COLORS.get("mauve", "#cba6f7"))
            painter.setBrush(QBrush(dot_color))
            painter.setPen(QPen(QColor(COLORS.get("base", "#1e1e2e")), s(1)))
            radius = s(6)
            for pt in (p1_pm, p2_pm):
                if pt is None:
                    continue
                painter.drawEllipse(QPointF(pt[0], pt[1]), radius, radius)
        finally:
            painter.end()

    def _draw_label(self, painter: QPainter, x: float, y: float, text: str) -> None:
        font = QFont(painter.font())
        font.setPointSize(scaled_font_size(9))
        font.setBold(True)
        painter.setFont(font)
        metrics = painter.fontMetrics()
        text_w = metrics.horizontalAdvance(text)
        text_h = metrics.height()
        pad = s(4)
        rect = QRectF(
            x + s(10), y - text_h / 2 - pad,
            text_w + 2 * pad, text_h + 2 * pad,
        )
        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(QBrush(QColor(0, 0, 0, 170)))
        painter.drawRoundedRect(rect, s(3), s(3))
        painter.setPen(QColor(COLORS.get("text", "#cdd6f4")))
        painter.drawText(
            QPointF(rect.x() + pad, rect.y() + pad + metrics.ascent()),
            text,
        )

    # ── Mouse interaction ────────────────────────────────────────

    def eventFilter(self, obj, event):
        if obj is self._display:
            t = event.type()
            if t == QEvent.Type.MouseButtonPress and event.button() == Qt.LeftButton:
                if self._handle_press(event):
                    return True
            elif t == QEvent.Type.MouseMove and self._dragging is not None:
                self._handle_drag(event)
                return True
            elif t == QEvent.Type.MouseButtonRelease and event.button() == Qt.LeftButton:
                if self._dragging is not None:
                    self._dragging = None
                    return True
        # Fall through to base class — but skip the base `clicked` emit
        # when we have a pixmap-backed measurement line so callers don't
        # see spurious click events from endpoint manipulation. The base
        # class will still emit `clicked` if neither branch above fires
        # and the event reaches it, which is desirable for callers that
        # actually want clicks (this widget is meant to be used inside
        # the calibration dialog, which doesn't listen to `clicked`).
        return super().eventFilter(obj, event)

    def _handle_press(self, event) -> bool:
        if self._last_pixmap is None:
            return False
        wx, wy = event.position().x(), event.position().y()
        frame_xy = self._widget_to_image(wx, wy)
        if frame_xy is None:
            return False

        # Hit-test existing endpoints in widget coords (snap radius is a
        # widget-pixel constant — independent of frame resolution).
        hit_idx = self._endpoint_hit(wx, wy)
        if hit_idx is not None:
            self._dragging = hit_idx
            return True

        # Place a new endpoint if room remains.
        if self._p1 is None:
            self._p1 = frame_xy
        elif self._p2 is None:
            self._p2 = frame_xy
        else:
            return False  # both placed — clicks elsewhere are no-ops
        self.endpoints_changed.emit()
        if self._last_qimage is not None:
            self._render_frame(self._last_qimage)
        return True

    def _handle_drag(self, event) -> None:
        wx, wy = event.position().x(), event.position().y()
        frame_xy = self._widget_to_image(wx, wy)
        if frame_xy is None:
            # Allow drag outside the pixmap area — clamp to nearest valid
            # frame coord so the endpoint doesn't lurch back to the press
            # location when the cursor crosses a letterbox edge.
            img_w, img_h = self._last_image_size
            if img_w == 0 or img_h == 0:
                return
            ox, oy = self._last_widget_offset
            scale = self._last_widget_scale or 1.0
            ix = max(0.0, min(img_w, (wx - ox) / scale))
            iy = max(0.0, min(img_h, (wy - oy) / scale))
            frame_xy = (ix, iy)

        if self._dragging == 0:
            self._p1 = frame_xy
        elif self._dragging == 1:
            self._p2 = frame_xy
        self.endpoints_changed.emit()
        if self._last_qimage is not None:
            self._render_frame(self._last_qimage)

    def _endpoint_hit(self, wx: float, wy: float) -> Optional[int]:
        """Return 0 if widget coord is near p1, 1 if near p2, else None."""
        scale = self._last_widget_scale or 1.0
        ox, oy = self._last_widget_offset
        candidates = []
        if self._p1 is not None:
            candidates.append((0, self._p1))
        if self._p2 is not None:
            candidates.append((1, self._p2))
        # If both endpoints are within snap, prefer the closer one so the
        # user can refine a pile-up cleanly.
        best: Optional[tuple[int, float]] = None
        for idx, (fx, fy) in candidates:
            ex = fx * scale + ox
            ey = fy * scale + oy
            dist = math.hypot(wx - ex, wy - ey)
            if dist <= _SNAP_RADIUS_WIDGET_PX and (best is None or dist < best[1]):
                best = (idx, dist)
        return None if best is None else best[0]

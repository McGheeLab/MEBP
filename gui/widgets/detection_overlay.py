"""
DetectionOverlay — Transparent QPainter overlay for vision detection results.

Draws detected circles, crosshairs, needle indicators, and focus quality
bars on top of the camera feed. Uses Catppuccin Mocha palette colors.

Usage::

    overlay = DetectionOverlay(parent=camera_widget.video_label)
    overlay.resize(camera_widget.video_label.size())

    # From DetectionWorker signal handler:
    overlay.set_well_detection(result)      # Green circle + crosshair
    overlay.set_needle_detection(result)    # Peach circle + indicator
    overlay.set_focus_result(focus_result)  # Focus quality bar
    overlay.clear()                         # Remove all overlays
"""

from __future__ import annotations

import logging
from dataclasses import dataclass

from PySide6.QtWidgets import QWidget
from PySide6.QtCore import Qt, QRectF, QPointF, QTimer
from PySide6.QtGui import QPainter, QPen, QColor, QBrush, QFont, QPainterPath

logger = logging.getLogger(__name__)

# Try to import detection result types for type hints
try:
    from SupportClasses.VisionDetector import DetectionResult, FocusResult
except ImportError:
    DetectionResult = None
    FocusResult = None


# ---------------------------------------------------------------------------
# Catppuccin Mocha Colors (from gui/styles.py)
# ---------------------------------------------------------------------------

class OverlayColors:
    """Detection overlay color constants — Catppuccin Mocha palette."""
    WELL_DETECTED = QColor("#a6e3a1")       # Green — well found
    WELL_CANDIDATE = QColor("#f9e2af")      # Yellow — low confidence
    NEEDLE_DETECTED = QColor("#fab387")     # Peach — needle found
    FOCUS_GOOD = QColor("#a6e3a1")          # Green — in focus
    FOCUS_POOR = QColor("#f38ba8")          # Red — out of focus
    FOCUS_MID = QColor("#f9e2af")           # Yellow — mid focus
    CROSSHAIR = QColor("#89b4fa")           # Blue — alignment crosshair
    TEXT = QColor("#cdd6f4")                # Primary text
    TEXT_DIM = QColor("#a6adc8")            # Secondary text
    BAR_BG = QColor("#313244")             # Bar background (Surface0)
    CONFIDENCE_RING = QColor("#a6e3a1")     # Green — confidence ring base


# ---------------------------------------------------------------------------
# DetectionOverlay Widget
# ---------------------------------------------------------------------------

class DetectionOverlay(QWidget):
    """
    Transparent overlay widget drawn on top of the camera feed.

    Must be positioned as a sibling over the CameraWidget's video_label.
    Call resize() when the parent resizes to keep the overlay aligned.

    All drawing methods store state and trigger update() for the next
    paintEvent — they never draw directly.
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        # Transparent background — mouse events pass through
        self.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents)
        self.setAttribute(Qt.WidgetAttribute.WA_NoSystemBackground)
        self.setStyleSheet("background: transparent;")

        # Detection state
        self._well_result = None          # DetectionResult or None
        self._needle_result = None        # DetectionResult or None
        self._focus_result = None         # FocusResult or None
        self._offset_text: str = ""       # "Δx: +45 µm, Δy: -12 µm"
        self._info_text: str = ""         # Additional status text

        # Scaling: maps detection pixel coords → overlay widget coords
        self._frame_size: tuple[int, int] = (640, 480)

        # Focus bar geometry
        self._focus_bar_width = 16
        self._focus_bar_margin = 8

        # Font
        self._font = QFont("monospace", 9)
        self._font_small = QFont("monospace", 8)

    # ── Public API ─────────────────────────────────────────────

    def set_frame_size(self, width: int, height: int) -> None:
        """Set the camera frame dimensions for coordinate scaling."""
        self._frame_size = (max(1, width), max(1, height))
        self.update()

    def set_well_detection(self, result) -> None:
        """Show well detection overlay. Pass None to clear."""
        self._well_result = result
        self.update()

    def set_needle_detection(self, result) -> None:
        """Show needle detection overlay. Pass None to clear."""
        self._needle_result = result
        self.update()

    def set_focus_result(self, result) -> None:
        """Update focus quality bar. Pass None to clear."""
        self._focus_result = result
        self.update()

    def set_offset_text(self, text: str) -> None:
        """Set the offset text displayed below the detection circle."""
        self._offset_text = text
        self.update()

    def set_info_text(self, text: str) -> None:
        """Set additional status text displayed at the top."""
        self._info_text = text
        self.update()

    def clear(self) -> None:
        """Clear all detection overlays."""
        self._well_result = None
        self._needle_result = None
        self._focus_result = None
        self._offset_text = ""
        self._info_text = ""
        self.update()

    # ── Coordinate Mapping ─────────────────────────────────────
    # The camera frame is displayed with KeepAspectRatio, so it may be
    # letterboxed (black bars) and centered within the QLabel. We must
    # compute the actual displayed image region to map correctly.

    def _display_transform(self) -> tuple[float, float, float]:
        """
        Compute (scale, offset_x, offset_y) for the aspect-ratio-preserved
        display of the camera frame within this overlay widget.

        Returns:
            scale: uniform scale factor (frame pixels → widget pixels)
            offset_x: horizontal offset for centering
            offset_y: vertical offset for centering
        """
        fw, fh = self._frame_size
        ow, oh = self.width(), self.height()
        if fw <= 0 or fh <= 0 or ow <= 0 or oh <= 0:
            return 1.0, 0.0, 0.0

        # KeepAspectRatio: use the smaller scale factor
        scale = min(ow / fw, oh / fh)
        displayed_w = fw * scale
        displayed_h = fh * scale
        # AlignCenter: padding on both sides
        offset_x = (ow - displayed_w) / 2.0
        offset_y = (oh - displayed_h) / 2.0
        return scale, offset_x, offset_y

    def _map_x(self, px: float) -> float:
        """Map frame X pixel coordinate to overlay widget coordinate."""
        scale, offset_x, _ = self._display_transform()
        return px * scale + offset_x

    def _map_y(self, py: float) -> float:
        """Map frame Y pixel coordinate to overlay widget coordinate."""
        scale, _, offset_y = self._display_transform()
        return py * scale + offset_y

    def _map_radius(self, r_px: float) -> float:
        """Map frame pixel radius to overlay widget radius."""
        scale, _, _ = self._display_transform()
        return r_px * scale

    # ── Paint Event ────────────────────────────────────────────

    def paintEvent(self, event):
        """Render all detection overlays."""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        if self._well_result is not None:
            self._paint_well(painter, self._well_result)

        if self._needle_result is not None:
            self._paint_needle(painter, self._needle_result)

        if self._focus_result is not None:
            self._paint_focus_bar(painter, self._focus_result)

        if self._info_text:
            self._paint_info_text(painter, self._info_text)

        painter.end()

    # ── Well Detection Rendering ───────────────────────────────

    def _paint_well(self, painter: QPainter, result) -> None:
        """Draw detected well circle with confidence-scaled opacity and crosshair."""
        cx = self._map_x(result.center_px[0])
        cy = self._map_y(result.center_px[1])
        r = self._map_radius(result.radius_px)

        confidence = getattr(result, 'confidence', 0.5)

        # Choose color based on confidence
        if confidence >= 0.6:
            color = QColor(OverlayColors.WELL_DETECTED)
        else:
            color = QColor(OverlayColors.WELL_CANDIDATE)

        # Scale opacity with confidence (min 40%, max 90%)
        alpha = int(255 * (0.4 + 0.5 * confidence))
        color.setAlpha(alpha)

        # Draw detection circle
        pen = QPen(color, 2)
        painter.setPen(pen)
        painter.setBrush(Qt.BrushStyle.NoBrush)
        painter.drawEllipse(QPointF(cx, cy), r, r)

        # Draw crosshair at detected center
        self._draw_crosshair(painter, cx, cy, r * 0.3, color)

        # Draw confidence text
        painter.setFont(self._font)
        painter.setPen(QPen(OverlayColors.TEXT, 1))
        text = f"{confidence * 100:.0f}%"
        painter.drawText(
            int(cx + r + 8), int(cy - 4),
            text,
        )

        # Draw offset text below circle
        if self._offset_text:
            painter.setFont(self._font_small)
            painter.setPen(QPen(OverlayColors.TEXT_DIM, 1))
            painter.drawText(
                int(cx - r), int(cy + r + 16),
                self._offset_text,
            )

    # ── Needle Detection Rendering ─────────────────────────────

    def _paint_needle(self, painter: QPainter, result) -> None:
        """Draw detected needle circle with indicator ring."""
        cx = self._map_x(result.center_px[0])
        cy = self._map_y(result.center_px[1])
        r = self._map_radius(result.radius_px)

        confidence = getattr(result, 'confidence', 0.5)
        color = QColor(OverlayColors.NEEDLE_DETECTED)
        alpha = int(255 * (0.4 + 0.5 * confidence))
        color.setAlpha(alpha)

        # Outer detection ring
        pen = QPen(color, 2)
        painter.setPen(pen)
        painter.setBrush(Qt.BrushStyle.NoBrush)
        painter.drawEllipse(QPointF(cx, cy), r, r)

        # Inner crosshair
        self._draw_crosshair(painter, cx, cy, r * 0.4, color)

        # Confidence indicator ring (dashed, slightly larger)
        dash_color = QColor(OverlayColors.NEEDLE_DETECTED)
        dash_color.setAlpha(int(255 * 0.4))
        dash_pen = QPen(dash_color, 1, Qt.PenStyle.DashLine)
        painter.setPen(dash_pen)
        painter.drawEllipse(QPointF(cx, cy), r + 6, r + 6)

        # Label
        painter.setFont(self._font_small)
        painter.setPen(QPen(OverlayColors.TEXT, 1))
        painter.drawText(
            int(cx + r + 10), int(cy),
            f"Needle {confidence * 100:.0f}%",
        )

    # ── Focus Quality Bar ──────────────────────────────────────

    def _paint_focus_bar(self, painter: QPainter, result) -> None:
        """Draw vertical focus quality bar on the right edge."""
        normalized = getattr(result, 'normalized_score', 0.0)
        is_in_focus = getattr(result, 'is_in_focus', False)

        bar_w = self._focus_bar_width
        margin = self._focus_bar_margin
        bar_h = self.height() - 2 * margin
        bar_x = self.width() - bar_w - margin
        bar_y = margin

        if bar_h <= 0:
            return

        # Background
        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(QBrush(OverlayColors.BAR_BG))
        painter.drawRoundedRect(
            int(bar_x), int(bar_y), int(bar_w), int(bar_h), 4, 4,
        )

        # Fill level (bottom-up)
        fill_h = max(1, int(bar_h * min(1.0, normalized)))
        fill_y = bar_y + bar_h - fill_h

        # Color gradient: red → yellow → green
        if normalized >= 0.7:
            fill_color = QColor(OverlayColors.FOCUS_GOOD)
        elif normalized >= 0.4:
            fill_color = QColor(OverlayColors.FOCUS_MID)
        else:
            fill_color = QColor(OverlayColors.FOCUS_POOR)

        if is_in_focus:
            fill_color.setAlpha(220)
        else:
            fill_color.setAlpha(180)

        painter.setBrush(QBrush(fill_color))
        painter.drawRoundedRect(
            int(bar_x), int(fill_y), int(bar_w), int(fill_h), 4, 4,
        )

        # Focus percentage text
        painter.setFont(self._font_small)
        painter.setPen(QPen(OverlayColors.TEXT, 1))
        text = f"{normalized * 100:.0f}%"
        text_x = bar_x - 36
        text_y = fill_y + fill_h // 2 + 4
        painter.drawText(int(text_x), int(text_y), text)

        # "FOCUS" label at top
        painter.setPen(QPen(OverlayColors.TEXT_DIM, 1))
        painter.drawText(int(bar_x - 4), int(bar_y - 4), "F")

    # ── Shared Drawing Helpers ─────────────────────────────────

    def _draw_crosshair(
        self,
        painter: QPainter,
        cx: float,
        cy: float,
        arm_length: float,
        color: QColor,
    ) -> None:
        """Draw a small crosshair at (cx, cy)."""
        pen = QPen(color, 1)
        painter.setPen(pen)
        # Horizontal
        painter.drawLine(
            QPointF(cx - arm_length, cy),
            QPointF(cx + arm_length, cy),
        )
        # Vertical
        painter.drawLine(
            QPointF(cx, cy - arm_length),
            QPointF(cx, cy + arm_length),
        )

    def _paint_info_text(self, painter: QPainter, text: str) -> None:
        """Draw status text at the top-left corner."""
        painter.setFont(self._font)

        # Semi-transparent background for readability
        metrics = painter.fontMetrics()
        text_w = metrics.horizontalAdvance(text) + 12
        text_h = metrics.height() + 6

        bg_color = QColor("#1e1e2e")
        bg_color.setAlpha(180)
        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(QBrush(bg_color))
        painter.drawRoundedRect(4, 4, text_w, text_h, 3, 3)

        painter.setPen(QPen(OverlayColors.TEXT, 1))
        painter.drawText(10, 4 + metrics.ascent() + 3, text)

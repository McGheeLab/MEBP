"""
camera_feed_view.py — Lightweight live camera display widget.

v7.3.3: Provides a consistent camera feed view that can be used on
any page. Connects to a CameraWidget's frame_captured signal and
displays frames in a QLabel with optional crosshair overlay.

Multiple CameraFeedView instances can display the same camera
simultaneously without reparenting issues.

Usage::

    from gui.widgets.camera_feed_view import CameraFeedView

    # Create a feed view for camera 0
    view = CameraFeedView(camera_manager, cam_idx=0)
    layout.addWidget(view)

    # The view auto-connects to the CameraWidget's frame_captured signal
    # and updates whenever a new frame arrives.
"""

from __future__ import annotations

import logging
from typing import Optional

from PySide6.QtWidgets import QWidget, QVBoxLayout, QLabel, QSizePolicy
from PySide6.QtCore import Qt, Signal, QEvent
from PySide6.QtGui import QImage, QPixmap, QPainter, QPen, QColor, QMouseEvent

from gui.styles import COLORS

logger = logging.getLogger(__name__)


class CameraFeedView(QWidget):
    """Lightweight display widget for a camera feed.

    Subscribes to a CameraWidget's frame_captured signal and renders
    frames in a QLabel. Multiple views can show the same camera.

    Signals:
        clicked(float, float): Emitted when the view is clicked,
            with (px_x, px_y) in image pixel coordinates.
    """

    clicked = Signal(float, float)  # image pixel coords of click

    def __init__(self, camera_manager=None, cam_idx: int = 0,
                 show_crosshair: bool = True, label: str = "",
                 parent=None):
        super().__init__(parent)
        self._manager = camera_manager
        self._cam_idx = cam_idx
        self._show_crosshair = show_crosshair
        self._label_text = label
        self._connected_cam = None  # CameraWidget we're subscribed to
        self._last_pixmap: Optional[QPixmap] = None
        self._last_qimage: Optional[QImage] = None  # full-res for re-render on show
        self._last_image_size = (0, 0)  # (w, h) of last received image

        self._setup_ui()
        self._connect_camera()

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        self._display = QLabel()
        self._display.setAlignment(Qt.AlignCenter)
        self._display.setMinimumSize(200, 150)
        self._display.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self._display.setStyleSheet(
            f"background-color: #181825; "
            f"border: 1px solid {COLORS.get('surface1', '#45475a')};"
        )
        if self._label_text:
            self._display.setText(self._label_text)
        else:
            self._display.setText(f"Camera {self._cam_idx + 1} — no feed")
        self._display.setStyleSheet(
            f"background-color: #181825; "
            f"border: 1px solid {COLORS.get('surface1', '#45475a')}; "
            f"color: {COLORS.get('text', '#cdd6f4')}; "
            f"font-size: 11pt;")
        layout.addWidget(self._display, stretch=1)

        # v7.3.5 BF-2: Install event filter on the QLabel to reliably
        # capture mouse clicks. Relying on event propagation from QLabel
        # to parent QWidget is unreliable in PySide6 when a pixmap is set.
        self._display.installEventFilter(self)

    # ── Camera connection ─────────────────────────────────────────

    def set_camera(self, cam_idx: int):
        """Switch to a different camera index."""
        self._disconnect_camera()
        self._cam_idx = cam_idx
        self._connect_camera()

    def _connect_camera(self):
        """Subscribe to the CameraWidget's frame_captured signal."""
        self._disconnect_camera()
        if self._manager is None:
            return
        cameras = self._manager.cameras
        if self._cam_idx < 0 or self._cam_idx >= len(cameras):
            return
        cam = cameras[self._cam_idx]
        if hasattr(cam, 'frame_captured'):
            cam.frame_captured.connect(self._on_frame)
            self._connected_cam = cam

    def _disconnect_camera(self):
        """Unsubscribe from current camera."""
        if self._connected_cam is not None:
            try:
                self._connected_cam.frame_captured.disconnect(self._on_frame)
            except (RuntimeError, TypeError):
                pass
            self._connected_cam = None
        # Reset display
        self._last_pixmap = None
        self._last_qimage = None
        txt = self._label_text or f"Camera {self._cam_idx + 1} — no feed"
        self._display.setText(txt)

    def _on_frame(self, q_img: QImage):
        """Handle a new frame from the CameraWidget."""
        if q_img is None:
            return
        self._last_image_size = (q_img.width(), q_img.height())
        self._last_qimage = q_img.copy()  # store full-res for re-render on show

        # Skip expensive scaling when widget is not visible
        if not self.isVisible():
            return

        self._render_frame(q_img)

    def _render_frame(self, q_img: QImage):
        """Scale and display a QImage on the label."""
        pixmap = QPixmap.fromImage(q_img)

        # Draw crosshair
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
        self._display.setPixmap(scaled)

    def _draw_crosshair(self, pixmap: QPixmap):
        """Draw crosshair overlay on the pixmap."""
        painter = QPainter(pixmap)
        pen = QPen(QColor("#f38ba8"), 1, Qt.PenStyle.DashLine)
        painter.setPen(pen)
        cx = pixmap.width() // 2
        cy = pixmap.height() // 2
        painter.drawLine(0, cy, pixmap.width(), cy)
        painter.drawLine(cx, 0, cx, pixmap.height())
        painter.end()

    # ── Properties ────────────────────────────────────────────────

    @property
    def cam_idx(self) -> int:
        return self._cam_idx

    @property
    def show_crosshair(self) -> bool:
        return self._show_crosshair

    @show_crosshair.setter
    def show_crosshair(self, value: bool):
        self._show_crosshair = value

    @property
    def image_size(self) -> tuple[int, int]:
        """(width, height) of the last received image in pixels."""
        return self._last_image_size

    # ── Mouse interaction ─────────────────────────────────────────

    def eventFilter(self, obj, event):
        """v7.3.5 BF-2: Intercept mouse clicks on the QLabel directly.

        This is more reliable than overriding mousePressEvent on the parent
        and waiting for event propagation from the child QLabel, which can
        fail in PySide6 when the QLabel has a pixmap set.
        """
        if (obj is self._display
                and event.type() == QEvent.Type.MouseButtonPress
                and event.button() == Qt.LeftButton
                and self._last_pixmap):
            # event.position() is in QLabel coordinates
            img_coords = self._widget_to_image(
                event.position().x(), event.position().y())
            if img_coords:
                self.clicked.emit(img_coords[0], img_coords[1])
                return True  # consumed
        return super().eventFilter(obj, event)

    def _widget_to_image(self, wx: float, wy: float
                         ) -> Optional[tuple[float, float]]:
        """Convert widget pixel coords to original image pixel coords."""
        pm = self._last_pixmap
        if pm is None or pm.isNull():
            return None
        lbl = self._display
        # The pixmap is centered in the label (KeepAspectRatio + AlignCenter)
        lbl_w, lbl_h = lbl.width(), lbl.height()
        pm_w, pm_h = pm.width(), pm.height()
        # Offset of pixmap within label
        ox = (lbl_w - pm_w) / 2
        oy = (lbl_h - pm_h) / 2
        # Position within the scaled pixmap
        px = wx - ox
        py = wy - oy
        if px < 0 or py < 0 or px > pm_w or py > pm_h:
            return None
        # Scale back to original image coords
        img_w, img_h = self._last_image_size
        if img_w == 0 or img_h == 0:
            return None
        ix = px / pm_w * img_w
        iy = py / pm_h * img_h
        return (ix, iy)

    # ── Lifecycle ─────────────────────────────────────────────────

    def showEvent(self, event):
        """Re-render last frame when widget becomes visible.

        Handles the case where the widget was hidden (e.g. inside a
        QStackedWidget) and frames arrived while invisible.  Also
        ensures the signal connection is still alive.
        """
        super().showEvent(event)
        # Ensure signal connection (safe to call repeatedly)
        if self._connected_cam is None and self._manager is not None:
            self._connect_camera()
        # Re-render the most recent frame at correct display size
        if self._last_qimage is not None:
            self._render_frame(self._last_qimage)

    def hideEvent(self, event):
        # Keep connected — the signal is cheap when we skip rendering
        super().hideEvent(event)

    def closeEvent(self, event):
        self._disconnect_camera()
        super().closeEvent(event)

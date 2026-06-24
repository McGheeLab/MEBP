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

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QLabel, QSizePolicy, QPushButton,
)
from PySide6.QtCore import Qt, Signal, QEvent, QPointF
from PySide6.QtGui import (
    QImage, QPixmap, QPainter, QPen, QColor, QBrush, QFont, QMouseEvent,
)

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
                 enable_settings: bool = True,
                 parent=None):
        super().__init__(parent)
        self._manager = camera_manager
        self._cam_idx = cam_idx
        self._show_crosshair = show_crosshair
        self._label_text = label
        # v7.5.x: when True, a gear button appears top-right whenever the
        # backing camera reports controllable hardware (e.g. the ToupCam
        # microscope), opening a pop-out hardware-settings dialog. Disabled
        # for the small preview the dialog itself hosts (avoids recursion).
        self._enable_settings = bool(enable_settings)
        self._settings_btn = None
        self._settings_dialog = None
        self._frame_count = 0  # throttle for gear-visibility SDK polling
        self._connected_cam = None  # CameraWidget we're subscribed to
        self._last_pixmap: Optional[QPixmap] = None
        self._last_qimage: Optional[QImage] = None  # full-res for re-render on show
        self._last_image_size = (0, 0)  # (w, h) of last received image
        # v7.5.x: optional displacement-vector overlay drawn from image
        # center — (dx_px, dy_px, label) in image pixels, or None.
        self._overlay_vector: Optional[tuple[float, float, str]] = None
        # v7.5.x: persistent reference markers (e.g. taught well centres) in
        # ABSOLUTE stage µm, projected into the live frame so the operator can
        # verify registration. camera centre == stage centre.
        self._ref_markers: list[tuple[str, float, float]] = []
        self._ref_stage_um: tuple[float, float] = (0.0, 0.0)
        self._ref_um_per_px: float = 0.0

        self._setup_ui()
        self._connect_camera()

    def set_reference_markers(self, markers, stage_x_um: Optional[float] = None,
                              stage_y_um: Optional[float] = None,
                              um_per_px: Optional[float] = None) -> None:
        """Overlay persistent reference points on the feed.

        ``markers``: iterable of ``(name, x_um, y_um)`` in absolute stage µm.
        Each is projected to a pixel via ``(x_um - stage_x)/um_per_px + w/2``
        (camera centre = stage centre), so the markers track as the stage
        moves. Supply ``stage_*``/``um_per_px`` here or via
        ``set_reference_stage_position`` / once at setup."""
        self._ref_markers = [
            (str(n), float(x), float(y)) for (n, x, y) in markers
        ]
        if stage_x_um is not None and stage_y_um is not None:
            self._ref_stage_um = (float(stage_x_um), float(stage_y_um))
        if um_per_px is not None and um_per_px > 0:
            self._ref_um_per_px = float(um_per_px)
        self._rerender_last()

    def set_reference_stage_position(self, x_um: float, y_um: float) -> None:
        """Update the stage centre used to project reference markers."""
        new = (float(x_um), float(y_um))
        if new != self._ref_stage_um:
            self._ref_stage_um = new
            if self._ref_markers:
                self._rerender_last()

    def _rerender_last(self) -> None:
        if self._last_qimage is not None and self.isVisible():
            self._render_frame(self._last_qimage)

    def set_overlay_vector(self, dx_px: Optional[float], dy_px: Optional[float],
                           label: str = ""):
        """Draw (or clear) an arrow from the image center along (dx, dy) px.

        Pass ``None`` for dx/dy to clear. Used by the µm/px calibration
        dialog to show the phase-correlation displacement it detected.
        """
        if dx_px is None or dy_px is None:
            self._overlay_vector = None
        else:
            self._overlay_vector = (float(dx_px), float(dy_px), str(label))
        if self._last_qimage is not None and self.isVisible():
            self._render_frame(self._last_qimage)

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

        # v7.5.x: gear button (overlay, top-right of the video) → pop-out
        # hardware settings. Hidden until the camera reports controllable HW.
        if self._enable_settings:
            self._settings_btn = QPushButton("⚙", self._display)
            self._settings_btn.setToolTip("Camera settings (exposure, gamma, …)")
            self._settings_btn.setCursor(Qt.PointingHandCursor)
            self._settings_btn.setFixedSize(26, 26)
            self._settings_btn.setStyleSheet(
                "QPushButton {"
                "  background: rgba(30,30,46,170); color: #cdd6f4;"
                "  border: 1px solid rgba(180,190,254,120);"
                "  border-radius: 13px; font-size: 14px; padding: 0px; }"
                "QPushButton:hover { background: rgba(49,50,68,210); }")
            self._settings_btn.clicked.connect(self._open_settings_dialog)
            self._settings_btn.hide()
            self._position_settings_btn()

    # ── Settings gear (v7.5.x) ────────────────────────────────────

    def _position_settings_btn(self):
        if self._settings_btn is None:
            return
        margin = 6
        x = max(margin, self._display.width() - self._settings_btn.width() - margin)
        self._settings_btn.move(x, margin)
        self._settings_btn.raise_()

    def _update_settings_visibility(self):
        """Show the gear only when the backing camera has controllable HW."""
        btn = self._settings_btn
        if btn is None:
            return
        controllable = False
        if self._manager is not None:
            try:
                caps = self._manager.hardware_capabilities(self._cam_idx)
                controllable = bool(caps.get("controllable"))
            except Exception:
                controllable = False
        btn.setVisible(controllable)
        if controllable:
            self._position_settings_btn()

    def _open_settings_dialog(self):
        if self._manager is None:
            return
        dlg = self._settings_dialog
        if dlg is None:
            try:
                from gui.dialogs.camera_settings_dialog import CameraSettingsDialog
            except Exception as exc:
                logger.warning(f"camera settings dialog unavailable: {exc}")
                return

            def _identity():
                mgr = self._manager
                if mgr is not None and hasattr(mgr, "camera_identity"):
                    return mgr.camera_identity(self._cam_idx)
                return None

            dlg = CameraSettingsDialog(
                self._manager, self._cam_idx,
                identity_getter=_identity, parent=self.window())
            self._settings_dialog = dlg
        else:
            dlg.reload()
        dlg.show()
        dlg.raise_()
        dlg.activateWindow()

    # ── Camera connection ─────────────────────────────────────────

    def set_camera(self, cam_idx: int):
        """Switch to a different camera index."""
        self._disconnect_camera()
        self._cam_idx = cam_idx
        self._connect_camera()
        if self._settings_dialog is not None:
            # Different camera now — drop the stale dialog.
            try:
                self._settings_dialog.close()
            except Exception:
                pass
            self._settings_dialog = None
        self._update_settings_visibility()

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

        # Reconcile the gear's visibility with the backing camera's HW support.
        # hardware_capabilities() does synchronous SDK round-trips for ToupCam,
        # so throttle it (once per ~30 frames) rather than calling it every
        # frame on the GUI thread; fire on the first visible frame for snappiness.
        self._frame_count += 1
        if self._settings_btn is not None and (self._frame_count % 30 == 1):
            self._update_settings_visibility()

        self._render_frame(q_img)

    def _render_frame(self, q_img: QImage):
        """Scale and display a QImage on the label."""
        pixmap = QPixmap.fromImage(q_img)

        # Draw crosshair
        if self._show_crosshair:
            self._draw_crosshair(pixmap)

        # v7.5.x: optional displacement-vector overlay
        if self._overlay_vector is not None:
            self._draw_overlay_vector(pixmap)

        # v7.5.x: persistent reference markers projected from stage µm
        if self._ref_markers and self._ref_um_per_px > 0:
            self._draw_reference_markers(pixmap)

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
        # Keep the gear pinned to the top-right above the pixmap.
        if self._settings_btn is not None and self._settings_btn.isVisible():
            self._position_settings_btn()

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

    def _draw_overlay_vector(self, pixmap: QPixmap):
        """Draw the displacement arrow (image px) from the image center."""
        import math
        dx, dy, label = self._overlay_vector
        cx = pixmap.width() / 2.0
        cy = pixmap.height() / 2.0
        ex, ey = cx + dx, cy + dy
        painter = QPainter(pixmap)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing, True)
        pen = QPen(QColor(COLORS.get("green", "#a6e3a1")),
                   max(2, pixmap.width() // 320))
        painter.setPen(pen)
        painter.drawLine(int(cx), int(cy), int(ex), int(ey))
        # Arrowhead
        ang = math.atan2(dy, dx)
        head = max(8.0, pixmap.width() / 60.0)
        for da in (math.radians(150), math.radians(-150)):
            hx = ex + head * math.cos(ang + da)
            hy = ey + head * math.sin(ang + da)
            painter.drawLine(int(ex), int(ey), int(hx), int(hy))
        if label:
            painter.drawText(int(ex) + 6, int(ey) - 6, label)
        painter.end()

    def _draw_reference_markers(self, pixmap: QPixmap):
        """Project absolute-stage-µm reference markers into the frame and draw
        each as a ring + crosshair + label (camera centre = stage centre)."""
        img_w = pixmap.width()
        img_h = pixmap.height()
        if not img_w or not img_h:
            return
        upp = self._ref_um_per_px
        sx, sy = self._ref_stage_um
        painter = QPainter(pixmap)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing, True)
        color = QColor(COLORS.get("pink", "#f5c2e7"))
        pen_w = max(2, pixmap.width() // 400)
        r = max(7.0, pixmap.width() / 70.0)
        font = QFont("Consolas", max(8, pixmap.width() // 110))
        painter.setFont(font)
        for name, mx, my in self._ref_markers:
            ix = (mx - sx) / upp + img_w / 2.0
            iy = (my - sy) / upp + img_h / 2.0
            if ix < -40 or iy < -40 or ix > img_w + 40 or iy > img_h + 40:
                continue
            painter.setPen(QPen(color, pen_w))
            painter.setBrush(Qt.BrushStyle.NoBrush)
            painter.drawEllipse(QPointF(ix, iy), r, r)
            painter.drawLine(QPointF(ix - r - 3, iy), QPointF(ix + r + 3, iy))
            painter.drawLine(QPointF(ix, iy - r - 3), QPointF(ix, iy + r + 3))
            if name:
                painter.drawText(int(ix + r + 5), int(iy - 4), name)
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
        self._update_settings_visibility()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._position_settings_btn()

    def hideEvent(self, event):
        # Keep connected — the signal is cheap when we skip rendering
        super().hideEvent(event)

    def closeEvent(self, event):
        self._disconnect_camera()
        super().closeEvent(event)

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
    # v7.5.x: (cam_idx, width, height) — re-emitted from the settings dialog when
    # the DEVICE capture resolution changes, so the owning page (Hardware Setup)
    # can update the camera-setup block's active_resolution.
    resolution_changed = Signal(int, int, int)

    def __init__(self, camera_manager=None, cam_idx: int = 0,
                 show_crosshair: bool = True, label: str = "",
                 enable_settings: bool = True,
                 auto_orient: bool = False,
                 parent=None):
        super().__init__(parent)
        self._manager = camera_manager
        self._cam_idx = cam_idx
        self._show_crosshair = show_crosshair
        self._label_text = label
        # v7.5.x: when True, the view keeps ITSELF synced to the camera's saved
        # mirror + rotation each frame (reading CameraManager.view_orientation),
        # so what you calibrate against equals what you image against — the
        # objective-calibration view stays consistent with the live views even
        # as the orientation is changed. Display-only + click-inverting (see
        # set_view_orientation); suppressed in edge-pick mode.
        self._auto_orient = bool(auto_orient)
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
        # v7.5.x: display-only view orientation (correct a mirrored/rotated
        # camera so the operator sees an upright, un-mirrored feed). The RAW
        # frame is untouched; clicks are inverted back to raw pixel coords so
        # pixel_to_stage_offset is unaffected. Default = no transform.
        self._view_mirror = False             # flip X (horizontal)
        self._view_flip_y = False             # flip Y (vertical)
        self._view_rot_deg = 0.0
        self._view_true_xform = None          # QTransform raw→displayed, or None
        self._displayed_image_size = (0, 0)   # (w, h) after the view transform
        self._edge_pick_mode = False          # feeds skip the transform when set

        # v7.5.x: aspect the widget last laid itself out for, so updateGeometry()
        # only fires when the camera's shape actually changes (not every frame).
        self._laid_out_aspect: tuple[int, int] = (0, 0)

        self._setup_ui()
        self._connect_camera()

    # ── Aspect ratio — the widget shape tracks the camera's true W:H ──
    #
    # v7.5.x: the displayed video should MATCH the camera's output, not sit in a
    # fixed-shape box with dark letterbox bars. We report the incoming frame's
    # true aspect via heightForWidth so box layouts / splitters allocate a
    # camera-shaped cell that the KeepAspectRatio pixmap then fills edge-to-edge.
    # Layouts that ignore heightForWidth (e.g. QGridLayout) fall back to today's
    # undistorted KeepAspectRatio fit — never distorted, so no regression.

    def _display_wh(self) -> tuple[int, int]:
        """(w, h) of the frame as DISPLAYED — swapped when the view is rotated
        ~90°, so heightForWidth matches what the operator actually sees."""
        w, h = self._last_image_size
        if w <= 0 or h <= 0:
            return (0, 0)
        if not self._edge_pick_mode:
            r = abs(self._view_rot_deg) % 180.0
            if 45.0 <= r <= 135.0:          # a ~90° view rotation swaps W/H
                return (h, w)
        return (int(w), int(h))

    def hasHeightForWidth(self) -> bool:  # noqa: N802 (Qt override)
        return True

    def heightForWidth(self, width: int) -> int:  # noqa: N802 (Qt override)
        dw, dh = self._display_wh()
        if dw <= 0 or dh <= 0 or width <= 0:
            return -1                        # unknown → no constraint yet
        return int(round(float(width) * float(dh) / float(dw)))

    def _maybe_update_geometry(self) -> None:
        """Re-lay-out only when the camera's displayed aspect changes."""
        wh = self._display_wh()
        if wh != self._laid_out_aspect:
            self._laid_out_aspect = wh
            self.updateGeometry()

    def _sync_auto_orientation(self) -> None:
        """When ``auto_orient`` is on, mirror the camera's saved orientation onto
        this view so it stays consistent with the calibration/live/mosaic views
        (they all derive from the same ``view_orientation``)."""
        if (not self._auto_orient or self._manager is None
                or self._edge_pick_mode):
            return
        try:
            fo = getattr(self._manager, "full_orientation", None)
            if callable(fo):
                mir, fy, rot = fo(self._cam_idx)
                self.set_view_orientation(bool(mir), float(rot), bool(fy))
                return
            vo = getattr(self._manager, "view_orientation", None)
            if callable(vo):
                mir, rot = vo(self._cam_idx)
                self.set_view_orientation(bool(mir), float(rot))  # no-op if same
        except Exception:
            pass

    def set_view_orientation(self, mirrored: bool = False,
                             rotation_deg: float = 0.0,
                             flip_y: bool = False) -> None:
        """v7.5.x: correct the DISPLAYED feed for a mirrored/rotated camera.

        Flips (``mirrored``) and rotates (``rotation_deg``) only what is SHOWN,
        so the operator sees an upright, un-mirrored view; the raw frame the
        rest of the app uses is unchanged, and clicks are inverted back to raw
        frame pixel coords. Default ``(False, 0.0)`` = no transform (today's
        behavior). Matches ``MosaicBuilder._orient_tile`` so the live feed and
        the mosaic agree. No effect while ``_edge_pick_mode`` is set."""
        mirrored = bool(mirrored)
        rot = float(rotation_deg or 0.0)
        fy = bool(flip_y)
        if (mirrored == self._view_mirror
                and fy == getattr(self, "_view_flip_y", False)
                and abs(rot - self._view_rot_deg) < 1e-6):
            return
        self._view_mirror = mirrored
        self._view_flip_y = fy
        self._view_rot_deg = rot
        self._maybe_update_geometry()   # a ~90° rotation swaps the displayed W/H
        self._rerender_last()

    def set_edge_pick_mode(self, enabled: bool) -> None:
        """v7.5.x: needle edge-pick relies on raw row=Z geometry — suppress the
        view transform while it is active."""
        self._edge_pick_mode = bool(enabled)
        self._maybe_update_geometry()
        self._rerender_last()

    def _orient_qimage(self, q_img: QImage):
        """Return ``(displayed_qimage, true_xform)`` after the view orientation.

        ``true_xform`` (QTransform, raw→displayed incl. Qt's fit offset) is used
        to invert clicks and to map overlays; ``None`` when no transform."""
        fy = getattr(self, "_view_flip_y", False)
        if ((not self._view_mirror and not fy and abs(self._view_rot_deg) < 0.05)
                or self._edge_pick_mode):
            return q_img, None
        import math
        from PySide6.QtGui import QTransform
        t = math.radians(self._view_rot_deg)
        c, s = math.cos(t), math.sin(t)
        mx = -1.0 if self._view_mirror else 1.0
        my = -1.0 if fy else 1.0
        # Linear map A = R(θ)·diag(mx, my) as a Qt row-vector transform (p' = p·T),
        # matching MosaicBuilder._orient_tile. Qt fits + offsets the result; the
        # true matrix (incl. that offset) inverts clicks / maps overlays.
        base = QTransform(c * mx, s * mx, -s * my, c * my, 0.0, 0.0)
        try:
            disp = q_img.transformed(base, Qt.SmoothTransformation)
            true_xf = QImage.trueMatrix(base, q_img.width(), q_img.height())
            return disp, true_xf
        except Exception:
            return q_img, None

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
        # v7.5.x: let the widget's height follow its width at the camera's true
        # aspect (see heightForWidth above). Horizontal Expanding so it still
        # fills the width a stretch=1 layout gives it.
        sp = QSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        sp.setHeightForWidth(True)
        self.setSizePolicy(sp)

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
            # Bubble a device-resolution change up so the page can refresh the
            # camera-setup block (the block was pulling a stale resolution).
            try:
                dlg.resolution_applied.connect(self.resolution_changed)
            except Exception:
                pass
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
        # Keep the displayed orientation in sync with the camera's saved
        # mirror/rotation (opt-in), so this view matches the calibration/live
        # views. no-ops when unchanged; suppressed in edge-pick mode.
        self._sync_auto_orientation()
        # Track the camera's true aspect so the widget lays itself out to match
        # it (no letterbox bars); only relayouts when the shape actually changes.
        self._maybe_update_geometry()

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
        # v7.5.x: apply the display-only view orientation first, then draw
        # overlays in the DISPLAYED frame (markers/vector mapped through the
        # same transform so they stay registered).
        disp_img, true_xf = self._orient_qimage(q_img)
        self._view_true_xform = true_xf
        self._displayed_image_size = (disp_img.width(), disp_img.height())
        pixmap = QPixmap.fromImage(disp_img)

        # Draw crosshair (display centre, axis-aligned)
        if self._show_crosshair:
            self._draw_crosshair(pixmap)

        # v7.5.x: optional displacement-vector overlay
        if self._overlay_vector is not None:
            self._draw_overlay_vector(pixmap, true_xf)

        # v7.5.x: persistent reference markers projected from stage µm
        if self._ref_markers and self._ref_um_per_px > 0:
            self._draw_reference_markers(pixmap, true_xf)

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

    def _draw_overlay_vector(self, pixmap: QPixmap, true_xf=None):
        """Draw the displacement arrow (image px) from the image center."""
        import math
        dx, dy, label = self._overlay_vector
        if true_xf is not None and self._last_image_size[0]:
            # Map the RAW centre + endpoint through the view transform so the
            # arrow rotates/flips with the displayed frame.
            rw, rh = self._last_image_size
            cp = true_xf.map(QPointF(rw / 2.0, rh / 2.0))
            ep = true_xf.map(QPointF(rw / 2.0 + dx, rh / 2.0 + dy))
            cx, cy, ex, ey = cp.x(), cp.y(), ep.x(), ep.y()
        else:
            cx = pixmap.width() / 2.0
            cy = pixmap.height() / 2.0
            ex, ey = cx + dx, cy + dy
        painter = QPainter(pixmap)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing, True)
        pen = QPen(QColor(COLORS.get("green", "#a6e3a1")),
                   max(2, pixmap.width() // 320))
        painter.setPen(pen)
        painter.drawLine(int(cx), int(cy), int(ex), int(ey))
        # Arrowhead — angle from the (possibly transformed) drawn direction.
        ang = math.atan2(ey - cy, ex - cx)
        head = max(8.0, pixmap.width() / 60.0)
        for da in (math.radians(150), math.radians(-150)):
            hx = ex + head * math.cos(ang + da)
            hy = ey + head * math.sin(ang + da)
            painter.drawLine(int(ex), int(ey), int(hx), int(hy))
        if label:
            painter.drawText(int(ex) + 6, int(ey) - 6, label)
        painter.end()

    def _draw_reference_markers(self, pixmap: QPixmap, true_xf=None):
        """Project absolute-stage-µm reference markers into the frame and draw
        each as a ring + crosshair + label (camera centre = stage centre).

        v7.5.x: markers are projected in RAW frame coords (using the raw image
        size), then mapped through the view transform so they stay registered
        on a flipped/rotated display."""
        disp_w = pixmap.width()
        disp_h = pixmap.height()
        if not disp_w or not disp_h:
            return
        raw_w, raw_h = self._last_image_size
        if not raw_w or not raw_h:
            raw_w, raw_h = disp_w, disp_h
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
            ix = (mx - sx) / upp + raw_w / 2.0
            iy = (my - sy) / upp + raw_h / 2.0
            if true_xf is not None:
                pt = true_xf.map(QPointF(ix, iy))
                ix, iy = pt.x(), pt.y()
            if ix < -40 or iy < -40 or ix > disp_w + 40 or iy > disp_h + 40:
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
        # Scale back to the DISPLAYED image coords (the pixmap is the oriented
        # frame). Falls back to the raw size when no view transform is active.
        disp_w, disp_h = self._displayed_image_size
        if not disp_w or not disp_h:
            disp_w, disp_h = self._last_image_size
        if not disp_w or not disp_h:
            return None
        ix = px / pm_w * disp_w
        iy = py / pm_h * disp_h
        # v7.5.x: invert the view transform so we always emit RAW frame pixel
        # coords (pixel_to_stage_offset works on the raw frame).
        if self._view_true_xform is not None:
            inv, ok = self._view_true_xform.inverted()
            if ok:
                pt = inv.map(QPointF(ix, iy))
                return (pt.x(), pt.y())
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

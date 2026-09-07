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
import math
from pathlib import Path
from typing import Optional

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QLabel, QSizePolicy, QPushButton,
)
from PySide6.QtCore import Qt, Signal, QEvent, QPointF, QTimer
from PySide6.QtGui import (
    QImage, QPixmap, QPainter, QPen, QColor, QBrush, QFont, QMouseEvent,
    QPainterPath,
)

from gui.styles import COLORS
from gui.widgets.view_geometry import (
    NULL_GEOMETRY, ZOOM_MAX, ZOOM_MIN, ViewGeometry, clamp_zoom, visible_rect)

logger = logging.getLogger(__name__)

# v7.13 — fraction of (sampled) raw pixels at/above the sensor clip level that
# lights the red SATURATED badge on the live feed. 0.5% is deliberate: single
# hot pixels shouldn't nag, genuine highlight clipping should. Bench-tunable.
SATURATION_WARN_FRAC = 0.005

# v7.14 — shared styling for every overlay button (gear / capture / record) so
# a new one cannot look like a different app.
_OVERLAY_BTN_QSS = (
    "QPushButton {"
    "  background: rgba(30,30,46,170); color: #cdd6f4;"
    "  border: 1px solid rgba(180,190,254,120);"
    "  border-radius: 13px; font-size: 14px; padding: 0px; }"
    "QPushButton:hover { background: rgba(49,50,68,210); }"
    "QPushButton:disabled { color: #6c7086; }")

# Recording: red pill with the elapsed time, so it can never be mistaken for
# the idle state at a glance.
_RECORD_ACTIVE_QSS = (
    "QPushButton {"
    "  background: rgba(243,139,168,225); color: #11111b;"
    "  border: 1px solid rgba(243,139,168,255);"
    "  border-radius: 13px; font-size: 12px; font-weight: bold;"
    "  padding: 0px 6px; }")


def view_transform_coeffs(mirrored: bool, flip_y: bool, rot_deg: float
                          ) -> tuple[float, float, float, float]:
    """Pure ``(m11, m12, m21, m22)`` of the view display transform.

    The display applies ``A = R(θ)·diag(mx, my)`` (flips first, then
    rotation) as a Qt ROW-VECTOR transform ``p' = p·T`` (``T = Aᵀ``), so a
    raw image vector ``(dx, dy)`` renders as::

        dx' = dx*m11 + dy*m21
        dy' = dx*m12 + dy*m22

    This is the single sign-convention anchor shared by ``_orient_qimage``
    and the roll calibration (``view_roll_from_displacement``) — tests
    compose the two to pin that a measured motion vector displays level.
    Importable without Qt side effects (pure math).
    """
    t = math.radians(float(rot_deg))
    c, s = math.cos(t), math.sin(t)
    mx = -1.0 if mirrored else 1.0
    my = -1.0 if flip_y else 1.0
    return (c * mx, s * mx, -s * my, c * my)


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
                 enable_capture: bool = True,
                 auto_orient: bool = False,
                 snap_rotation_to_cardinal: bool = False,
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
        # v7.10: display the nearest 0/90/180/270 instead of the exact measured
        # angle, leaving the RESIDUAL visible.
        #
        # This is the correct policy for a camera whose mount the operator
        # squares up by hand — the needle side cams. Silently un-rotating the
        # residual is self-defeating: the tilt they are trying to remove
        # disappears from the view, and as they turn the camera the correction
        # tracks them, so the picture never appears to change. It also resamples
        # every frame for a few degrees of correction, where a cardinal turn is
        # a lossless fast path.
        #
        # The FULL measured angle still drives the geometry — clicks
        # (``pixel_to_stage_offset``) and mosaic tiles (``_orient_tile``) — which
        # must be right to a fraction of a degree. Only what is DISPLAYED snaps.
        self._snap_rotation = bool(snap_rotation_to_cardinal)
        # v7.5.x: when True, a gear button appears top-right whenever the
        # backing camera reports controllable hardware (e.g. the ToupCam
        # microscope), opening a pop-out hardware-settings dialog. Disabled
        # for the small preview the dialog itself hosts (avoids recursion).
        self._enable_settings = bool(enable_settings)
        # v7.14 — 📷 capture + ⏺ record overlay buttons. On by default so the
        # operator gets them on EVERY live feed (their explicit ask); turned
        # off only for previews inside dialogs that are already about capture.
        self._enable_capture = bool(enable_capture)
        self._settings_btn = None
        self._settings_dialog = None
        self._frame_count = 0  # throttle for gear-visibility SDK polling
        # v7.13 — raw-count saturation flag (mono scientific cameras only).
        # The display auto-scale HIDES clipping, so the badge is driven by the
        # backend's raw 16-bit statistics, polled as a lock snapshot (no SDK
        # traffic) every few frames.
        self._saturated = False
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
        # v7.13: needle-bore dots — (label, dx_um, dy_um, color_hex) where
        # (dx, dy) is the bore's offset from the CAMERA CENTRE in stage-frame
        # µm (pixel_to_stage_offset label space). Fixed in the frame, so no
        # stage tracking is needed; projected via the calibrated inverse map.
        self._bore_markers: list[tuple[str, float, float, str]] = []
        # v7.20: polylines in the SAME camera-centre-relative µm frame as the
        # bore dots — (points, color_hex, dashed). Used to lay a commanded
        # toolpath over the feed so the operator can see the stroke against the
        # real bead. See set_stage_paths for why the frame choice matters.
        self._stage_paths: list[tuple[tuple, str, bool]] = []
        # v7.5.x: display-only view orientation (correct a mirrored/rotated
        # camera so the operator sees an upright, un-mirrored feed). The RAW
        # frame is untouched; clicks are inverted back to raw pixel coords so
        # pixel_to_stage_offset is unaffected. Default = no transform.
        self._view_mirror = False             # flip X (horizontal)
        self._view_flip_y = False             # flip Y (vertical)
        self._view_rot_deg = 0.0
        self._view_true_xform = None          # QTransform raw→displayed, or None
        self._displayed_image_size = (0, 0)   # (w, h) after the view transform
        # v7.15 — zoom/pan. The geometry object is rebuilt on every render and
        # is the SINGLE source for both painting and hit-testing.
        self._zoom = 1.0
        self._view_center = None              # displayed-image coords, or None
        self._crop_origin = (0.0, 0.0)        # top-left of the zoom crop
        self._cropped_size = (0, 0)           # size after that crop
        self._geometry = NULL_GEOMETRY
        self._zoom_enabled = True
        self._panning = False
        self._pan_anchor = None               # (widget px, view centre) at grab
        # v7.15 — hover-revealed toolbar. "Available" is what the camera
        # state allows; "visible" additionally needs the pointer over the feed
        # and room in the row (see _position_settings_btn).
        self._hovering = False
        self._settings_available = False
        self._capture_available = False
        self._overflow_actions = []
        self._overflow_btn = None
        self._popout_btn = None
        self._zoom_in_btn = self._zoom_out_btn = self._zoom_fit_btn = None
        self._popout_window = None
        self._edge_pick_mode = False          # feeds skip the transform when set
        # v7.10: alignment ghost — a translucent reference image composited over
        # the live feed so the operator can turn a camera in its mount until the
        # two coincide. None = no ghost (every other feed in the app).
        self._ghost_img: Optional[QImage] = None
        self._ghost_opacity = 0.45
        self._ghost_offset = (0.0, 0.0)       # display px
        self._ghost_cache: Optional[QPixmap] = None
        self._ghost_cache_key = None

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
        """When ``auto_orient`` is on, apply the camera's saved LIVE-VIEW
        orientation to this view.

        v7.16: this reads ``display_orientation``, not ``full_orientation``. The
        latter is the measured camera→stage matrix, which the objective/mosaic
        calibration rewrites every time it runs — so while the view was driven
        from it, setting the live view up and then calibrating flipped the view
        back. ``display_orientation`` falls back to the measured value when the
        operator has expressed no preference, so an untouched camera behaves
        exactly as before.

        Clicks are unaffected either way: the click handler reports RAW frame
        coordinates (it inverts this view's transform via ``ViewGeometry``) and
        ``pixel_to_stage_offset`` then applies the MEASURED orientation. That is
        what makes the display safe to change independently.
        """
        if (not self._auto_orient or self._manager is None
                or self._edge_pick_mode):
            return
        try:
            do = getattr(self._manager, "display_orientation", None)
            if callable(do):
                mir, fy, rot = do(self._cam_idx)
                self.set_view_orientation(
                    bool(mir), self._display_rotation(rot), bool(fy))
                return
            fo = getattr(self._manager, "full_orientation", None)
            if callable(fo):
                mir, fy, rot = fo(self._cam_idx)
                self.set_view_orientation(
                    bool(mir), self._display_rotation(rot), bool(fy))
                return
            vo = getattr(self._manager, "view_orientation", None)
            if callable(vo):
                mir, rot = vo(self._cam_idx)
                # no-op if unchanged
                self.set_view_orientation(bool(mir),
                                          self._display_rotation(rot))
        except Exception:
            pass

    def _display_rotation(self, rot) -> float:
        """The rotation to APPLY to the view for a measured ``rot``.

        Identity unless this view snaps to cardinals, in which case the nearest
        quarter-turn is used and the residual is left on screen for the operator
        to square out physically.
        """
        rot = float(rot or 0.0)
        if not getattr(self, "_snap_rotation", False):
            return rot
        try:
            from SupportClasses.CameraRotationTracker import (
                nearest_square_rotation)
            return float(nearest_square_rotation(rot)[0])
        except Exception:
            return rot

    def set_throttled(self, on: bool) -> None:
        """v7.6: forward a display-rate throttle to the underlying camera
        widget (halves the GUI-thread grab rate during a print so the live
        position sampling isn't starved). No-op when the widget is absent or
        doesn't support it."""
        try:
            widget = getattr(self, "_connected_cam", None)
            if widget is None:
                mgr = getattr(self, "_manager", None)
                getter = getattr(mgr, "_widget", None)
                if callable(getter):
                    widget = getter(self._cam_idx)
            fn = getattr(widget, "set_throttled", None)
            if callable(fn):
                fn(bool(on))
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
        self._invalidate_ghost_cache()
        self._maybe_update_geometry()   # a ~90° rotation swaps the displayed W/H
        self._rerender_last()

    def set_edge_pick_mode(self, enabled: bool) -> None:
        """v7.5.x: needle edge-pick relies on raw row=Z geometry — suppress the
        view transform while it is active."""
        self._edge_pick_mode = bool(enabled)
        # This also changes _orient_qimage's output, so the ghost cache (keyed
        # on the resolved transform) must go with it.
        self._invalidate_ghost_cache()
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
        from PySide6.QtGui import QTransform
        # Linear map A = R(θ)·diag(mx, my) as a Qt row-vector transform (p' = p·T),
        # matching MosaicBuilder._orient_tile. Qt fits + offsets the result; the
        # true matrix (incl. that offset) inverts clicks / maps overlays.
        m11, m12, m21, m22 = view_transform_coeffs(
            self._view_mirror, fy, self._view_rot_deg)
        base = QTransform(m11, m12, m21, m22, 0.0, 0.0)
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

    def set_bore_markers(self, markers) -> None:
        """v7.13: overlay colored needle-bore dots on the feed.

        ``markers``: iterable of ``(label, dx_um, dy_um, color_hex)`` where
        ``(dx, dy)`` is the bore's offset from the CAMERA CENTRE in stage-frame
        µm — the ``pixel_to_stage_offset`` label space. The dot is FIXED in the
        frame (a bore rides the same body the camera does), so no stage
        position is needed and the markers never lag a move.

        ``None``/empty clears. Change-gated: callers push from 3–7 Hz refresh
        ticks, so an identical push must not cost a re-render.
        """
        norm: list[tuple[str, float, float, str]] = []
        for m in (markers or ()):
            try:
                norm.append((str(m[0]), float(m[1]), float(m[2]), str(m[3])))
            except (TypeError, ValueError, IndexError):
                continue
        if norm == self._bore_markers:
            return
        self._bore_markers = norm
        self._rerender_last()

    def _bore_marker_raw_px(self, dx_um: float, dy_um: float,
                            raw_w: int, raw_h: int) -> tuple[float, float]:
        """Camera-centre FOV offset (µm) → RAW frame px.

        Routes through ``CameraManager.stage_offset_to_pixel`` — the exact
        inverse of the click path — NEVER the naive identity divide: on a
        camera with a calibrated rotation/mirror the identity map draws the dot
        away from the bore (the v7.8 ``to_px`` lesson, and the known flaw in
        ``_draw_reference_markers``). Identity fallback only when no manager is
        attached (test doubles / headless builds).
        """
        mgr = self._manager
        fn = getattr(mgr, "stage_offset_to_pixel", None) if mgr else None
        if callable(fn):
            try:
                px, py = fn(self._cam_idx, float(dx_um), float(dy_um),
                            int(raw_w), int(raw_h))
                return float(px), float(py)
            except Exception:
                pass
        upp = 0.0
        eff = getattr(mgr, "effective_um_per_px", None) if mgr else None
        if callable(eff):
            try:
                upp = float(eff(self._cam_idx, raw_w))
            except Exception:
                upp = 0.0
        if upp <= 0.0:
            upp = 1.0
        return raw_w / 2.0 + float(dx_um) / upp, raw_h / 2.0 + float(dy_um) / upp

    def set_stage_paths(self, paths) -> None:
        """v7.20: overlay POLYLINES on the feed, in the bore-marker frame.

        ``paths``: iterable of ``(points, color_hex, dashed)`` where ``points``
        is a sequence of ``(dx_um, dy_um)`` offsets from the CAMERA CENTRE in
        stage-frame µm — the SAME label space as :meth:`set_bore_markers`, and
        therefore the exact inverse of the click path. Sharing that one frame is
        what keeps a drawn path, the dots at its ends and a click on it
        registered with each other on a rotated or mirrored camera; a second
        convention here would be a sign error waiting to happen.

        To pin a path to an ABSOLUTE stage position, subtract the live stage
        position before pushing and re-push on the status tick: panning the
        stage then slides the overlay across the frame and it stays on the
        physical feature.

        ``None``/empty clears. Change-gated like the markers, so pushing from a
        3–7 Hz tick costs nothing when nothing moved.
        """
        norm: list[tuple[tuple, str, bool]] = []
        for p in (paths or ()):
            try:
                pts, colour, dashed = p[0], str(p[1]), bool(p[2])
                clean = tuple((float(q[0]), float(q[1])) for q in pts)
            except (TypeError, ValueError, IndexError):
                continue
            if len(clean) >= 2:
                norm.append((clean, colour, dashed))
        if norm == self._stage_paths:
            return
        self._stage_paths = norm
        self._rerender_last()

    def _draw_stage_paths(self, pixmap: QPixmap, true_xf=None):
        """v7.20: stroke each polyline through the calibrated inverse map.

        Deliberately NO per-point off-frame skip (unlike the bore dots): for a
        LINE the interesting case is that both endpoints are outside the view
        and only the middle crosses it, so culling on the endpoints would blank
        the overlay exactly when it is most useful. The clip rect does the work
        instead, and it also keeps a very long line at a small µm/px from
        reaching the rasteriser un-bounded.
        """
        disp_w, disp_h = pixmap.width(), pixmap.height()
        if not disp_w or not disp_h:
            return
        raw_w, raw_h = self._last_image_size
        if not raw_w or not raw_h:
            raw_w, raw_h = disp_w, disp_h
        painter = QPainter(pixmap)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing, True)
        painter.setClipRect(-8, -8, disp_w + 16, disp_h + 16)
        w = max(2, pixmap.width() // 500)
        for pts, colour_hex, dashed in self._stage_paths:
            colour = QColor(colour_hex)
            if not colour.isValid():
                colour = QColor(COLORS.get("teal", "#94e2d5"))
            pen = QPen(colour, w)
            pen.setCapStyle(Qt.PenCapStyle.RoundCap)
            pen.setJoinStyle(Qt.PenJoinStyle.RoundJoin)
            if dashed:
                # setDashPattern IMPLICITLY sets CustomDashLine, so a preceding
                # setStyle(DashLine) would be dead — and a redundant second
                # enforcement point is what lets a mutation of one of them look
                # harmless. Pattern units are pen widths.
                pen.setDashPattern([5.0, 4.0])
            painter.setPen(pen)
            path = QPainterPath()
            for i, (dx_um, dy_um) in enumerate(pts):
                ix, iy = self._bore_marker_raw_px(dx_um, dy_um, raw_w, raw_h)
                ix, iy = self._map_to_pixmap(true_xf, ix, iy)
                if i == 0:
                    path.moveTo(ix, iy)
                else:
                    path.lineTo(ix, iy)
            painter.drawPath(path)
        painter.end()

    def set_alignment_ghost(self, image: Optional[QImage],
                            opacity: float = 0.45,
                            offset_px: tuple = (0.0, 0.0)) -> None:
        """v7.10: composite a translucent reference image over the live feed.

        Used by the *Square up mount* tool: the ghost is the frozen reference
        frame pre-rotated to the target, so the operator turns the camera until
        live and ghost coincide. ``None`` clears it, which is the state every
        other feed in the app stays in.

        ``image`` must have the SAME aspect as the raw frame — it is stretched
        onto the displayed pixmap. ``offset_px`` is in DISPLAYED pixels and
        keeps the ghost overlaid when the mount axis is not the optical axis,
        so the operator judges angle rather than chasing a slide.

        ⚠ Only meaningful on a view whose ``rotation_deg`` is 0 (or a multiple
        of 90). ``QImage.transformed`` grows the bounding box for an arbitrary
        angle, and the ghost and the frame — different source sizes — would
        then grow differently and stop registering.
        """
        self._ghost_img = image
        self._ghost_opacity = max(0.0, min(1.0, float(opacity)))
        try:
            self._ghost_offset = (float(offset_px[0]), float(offset_px[1]))
        except Exception:
            self._ghost_offset = (0.0, 0.0)
        self._invalidate_ghost_cache()
        self._rerender_last()

    def _invalidate_ghost_cache(self) -> None:
        self._ghost_cache = None
        self._ghost_cache_key = None

    def _ghost_for(self, size) -> Optional[QPixmap]:
        """The ghost, oriented + scaled to ``size``, cached.

        Load-bearing, not an optimisation: re-running ``_orient_qimage`` on the
        ghost every frame costs ~33 ms at 3664x2748 (measured), which would
        halve the GUI thread's frame budget on the microscope camera.
        """
        if self._ghost_img is None or size.width() < 1 or size.height() < 1:
            return None
        key = (id(self._ghost_img), self._ghost_img.cacheKey(),
               self._view_mirror, getattr(self, "_view_flip_y", False),
               round(self._view_rot_deg, 4), self._edge_pick_mode,
               size.width(), size.height())
        if self._ghost_cache is not None and self._ghost_cache_key == key:
            return self._ghost_cache
        try:
            oriented, _ = self._orient_qimage(self._ghost_img)
            scaled = QPixmap.fromImage(oriented).scaled(
                size, Qt.AspectRatioMode.IgnoreAspectRatio,
                Qt.TransformationMode.SmoothTransformation)
        except Exception:
            return None
        self._ghost_cache = scaled
        self._ghost_cache_key = key
        return scaled

    def _draw_alignment_ghost(self, pixmap: QPixmap) -> None:
        ghost = self._ghost_for(pixmap.size())
        if ghost is None:
            return
        painter = QPainter(pixmap)
        painter.setOpacity(self._ghost_opacity)
        painter.drawPixmap(int(round(self._ghost_offset[0])),
                           int(round(self._ghost_offset[1])), ghost)
        painter.setOpacity(1.0)
        painter.end()

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
            self._settings_btn.setStyleSheet(_OVERLAY_BTN_QSS)
            self._settings_btn.clicked.connect(self._open_settings_dialog)
            self._settings_btn.hide()

        # v7.14 — capture + record overlay buttons. Same styling as the gear,
        # laid out to its left. Shown once the camera delivers a frame (a
        # capture only means anything on a live feed), NOT gated on the SDK
        # capability poll — that is a round-trip and would delay them a second.
        self._capture_btn = self._record_btn = None
        self._capture_ctrl = None
        if self._enable_capture:
            self._capture_btn = self._make_overlay_button(
                "📷", "Capture an image from this camera")
            self._capture_btn.clicked.connect(self._on_capture_clicked)
            self._capture_btn.setContextMenuPolicy(Qt.CustomContextMenu)
            self._capture_btn.customContextMenuRequested.connect(
                lambda _p: self.open_capture_settings("still"))
            self._record_btn = self._make_overlay_button(
                "⏺", "Record video from this camera")
            self._record_btn.clicked.connect(self._on_record_clicked)
            self._record_btn.setContextMenuPolicy(Qt.CustomContextMenu)
            self._record_btn.customContextMenuRequested.connect(
                lambda _p: self.open_capture_settings("video"))
            for b, tip, what in ((self._capture_btn, "Capture an image",
                                  "image capture"),
                                 (self._record_btn, "Record video",
                                  "video recording")):
                b.setToolTip(f"{tip} from this camera\n"
                             f"(right-click for {what} settings)")

        # v7.15 — view controls. Lower priority than the capture actions, so
        # these are the ones that fall into the ⋯ menu on a small feed.
        self._zoom_in_btn = self._make_overlay_button(
            "＋", "Zoom in (mouse wheel up)")
        self._zoom_in_btn.clicked.connect(self.zoom_in)
        self._zoom_out_btn = self._make_overlay_button(
            "－", "Zoom out (mouse wheel down)")
        self._zoom_out_btn.clicked.connect(self.zoom_out)
        self._zoom_fit_btn = self._make_overlay_button(
            "⛶", "Fit the whole frame\n(middle-drag pans while zoomed)")
        self._zoom_fit_btn.clicked.connect(self.zoom_fit)
        self._popout_btn = self._make_overlay_button(
            "⇱", "Open this view in its own window")
        self._popout_btn.clicked.connect(self.pop_out)
        self._overflow_btn = self._make_overlay_button(
            "⋯", "More view controls")
        self._overflow_btn.clicked.connect(self._show_overflow_menu)
        self._position_settings_btn()

    def _make_overlay_button(self, glyph: str, tip: str):
        btn = QPushButton(glyph, self._display)
        btn.setToolTip(tip)
        btn.setCursor(Qt.PointingHandCursor)
        btn.setFixedSize(26, 26)
        btn.setStyleSheet(_OVERLAY_BTN_QSS)
        btn.hide()
        return btn

    # ── Overlay buttons (gear v7.5.x · capture + record v7.14) ────

    def _position_settings_btn(self):
        """Lay the visible overlay buttons out from the right edge.

        Kept under its historical name because several call sites and the
        existing tests use it. The gear is FIRST in the right-to-left order,
        so a feed with no capture buttons puts it in exactly the position it
        has always had — pinned by a test.

        v7.15: seven buttons at 26 px do not fit a 200 px feed, so anything
        that would not fit moves into a ``⋯`` overflow menu. Sizing decides
        it, not per-call-site configuration, which is what makes the same code
        work in the 4-up camera grid and the height-capped settings preview.
        """
        margin, gap = 6, 4
        avail = max(0, self._display.width() - 2 * margin)
        buttons = [b for b in self._overlay_buttons()
                   if b is not None and self._wants_visible(b)]
        overflow = self._overflow_btn

        # How many fit, in priority order, leaving room for ⋯ if needed?
        def _fits(items, with_more):
            need = sum(b.width() for b in items) + gap * max(0, len(items) - 1)
            if with_more and overflow is not None:
                need += overflow.width() + gap
            return need <= avail

        shown = list(buttons)
        hidden = []
        while shown and not _fits(shown, bool(hidden)):
            hidden.insert(0, shown.pop())          # drop the lowest priority
        self._overflow_actions = hidden

        if overflow is not None:
            overflow.setVisible(bool(hidden) and self._hover_active())
        for b in buttons:
            b.setVisible(b in shown and self._button_shown(b))

        x = max(margin, self._display.width() - margin)
        row = ([overflow] if (hidden and overflow is not None) else []) + shown
        for btn in row:
            # ⚠ isHidden(), NOT isVisible(). isVisible() is False while ANY
            # ancestor is unshown, so a page that has not been displayed yet
            # would lay none of these out and they would all pile up at (0,0)
            # the first time it appeared. CLAUDE.md records this exact trap.
            if btn is None or btn.isHidden():
                continue
            x -= btn.width()
            btn.move(max(margin, x), margin)
            btn.raise_()
            x -= gap

    def _overlay_buttons(self):
        """Right-to-left order = priority. Gear first (its position is pinned
        by tests), then the two capture actions, then the view controls —
        which are the ones that fall into the overflow menu on a small feed.
        """
        rec = getattr(self, "_record_btn", None)
        order = [self._settings_btn, rec,
                 getattr(self, "_capture_btn", None),
                 getattr(self, "_popout_btn", None),
                 getattr(self, "_zoom_in_btn", None),
                 getattr(self, "_zoom_out_btn", None),
                 getattr(self, "_zoom_fit_btn", None)]
        if getattr(self, "_recording", False) and rec is not None:
            # While recording the stop pill outranks everything: it must never
            # be the button that gets pushed into the overflow menu.
            order.remove(rec)
            order.insert(0, rec)
        return order

    # ── Hover reveal (v7.15) ──────────────────────────────────────

    def _hover_active(self) -> bool:
        """Buttons are shown only under the pointer — except while recording.

        The operator asked for the controls to stop crowding the picture. A
        recording is the one thing that must stay visible and stoppable
        without hunting for it, so the pill is exempt (see _wants_visible).
        """
        return bool(self._hovering)

    def _button_shown(self, btn) -> bool:
        """Hover gate, with the recording pill exempt."""
        if (getattr(self, "_recording", False)
                and btn is getattr(self, "_record_btn", None)):
            return True
        return self._hover_active()

    def _wants_visible(self, btn) -> bool:
        """Whether this button would be shown if there were room."""
        if btn is self._settings_btn:
            return bool(self._settings_available)
        if btn in (getattr(self, "_capture_btn", None),
                   getattr(self, "_record_btn", None)):
            return bool(self._capture_available)
        if btn in (getattr(self, "_zoom_in_btn", None),
                   getattr(self, "_zoom_out_btn", None),
                   getattr(self, "_zoom_fit_btn", None)):
            return bool(self._zoom_enabled and self._capture_available)
        if btn is getattr(self, "_popout_btn", None):
            return bool(self._capture_available)
        return True

    def enterEvent(self, event):
        self._hovering = True
        self._position_settings_btn()
        super().enterEvent(event)

    def leaveEvent(self, event):
        self._hovering = False
        self._position_settings_btn()
        super().leaveEvent(event)

    # ── Pop out (v7.15) ───────────────────────────────────────────

    def pop_out(self):
        """Move this view into its own resizable window.

        Returns the dialog, or None if the view could not be detached (it is
        not in a layout, or one is already open).
        """
        if self._popout_window is not None:
            self._popout_window.raise_()
            self._popout_window.activateWindow()
            return self._popout_window
        try:
            from gui.dialogs.camera_popout_dialog import CameraPopoutDialog
        except Exception as exc:
            logger.warning(f"pop-out unavailable: {exc}")
            return None
        parent = self.parentWidget()
        layout = parent.layout() if parent is not None else None
        if layout is None or layout.indexOf(self) < 0:
            logger.info("this view is not in a layout — cannot pop it out")
            return None

        # The cached settings dialog is parented to the OLD window; drop it so
        # it cannot outlive the window it belongs to.
        self._settings_dialog = None
        title = (self._label_text or f"Camera {self._cam_idx + 1}").split("—")[0]
        dlg = CameraPopoutDialog(title=title.strip() or "Camera",
                                 parent=self.window())
        if not dlg.take(self):
            dlg.deleteLater()
            return None
        self._popout_window = dlg
        dlg.finished.connect(lambda _r: self._on_popout_closed())
        dlg.show()
        self._rerender()          # the new window has a different size
        return dlg

    def _on_popout_closed(self):
        self._popout_window = None

    def on_returned_from_popout(self):
        """Called by the dialog once the view is back in its slot.

        ``eventFilter`` refuses to emit ``clicked`` while ``_last_pixmap`` is
        None, and ``_on_frame`` skips rendering while hidden — so a view that
        merely waited for the next frame would be click-dead in between, which
        on a long exposure is seconds.
        """
        self._popout_window = None
        if self._connected_cam is None and self._manager is not None:
            self._connect_camera()
        self._rerender()
        self._position_settings_btn()

    def _show_overflow_menu(self):
        """The buttons that did not fit, as a menu."""
        from PySide6.QtWidgets import QMenu
        menu = QMenu(self)
        for btn in self._overflow_actions:
            act = menu.addAction(f"{btn.text()}  {btn.toolTip().splitlines()[0]}")
            act.triggered.connect(btn.click)
        if self._overflow_btn is not None:
            menu.exec(self._overflow_btn.mapToGlobal(
                self._overflow_btn.rect().bottomLeft()))

    # ── Capture + record (v7.14) ──────────────────────────────────

    def capture_controller(self):
        """The lazily-built :class:`CaptureController` for this slot."""
        ctrl = getattr(self, "_capture_ctrl", None)
        if ctrl is None and self._manager is not None:
            try:
                from gui.widgets.capture_controller import CaptureController
                ctrl = CaptureController(self._manager, self._cam_idx, self)
                ctrl.captured.connect(self._on_capture_done)
                ctrl.failed.connect(self._on_capture_failed)
                ctrl.record_state.connect(self._on_record_state)
                from SupportClasses.CaptureContext import register_recorder
                register_recorder(ctrl)
                self._capture_ctrl = ctrl
            except Exception as exc:
                logger.warning(f"capture unavailable on this feed: {exc}")
        return ctrl

    def open_capture_settings(self, kind: str = "still"):
        """Open the settings for ONE kind of capture.

        v7.15: two dialogs, reached by right-clicking the button they belong
        to — the operator reported the combined one confusing. ``kind`` is
        "still" (📷) or "video" (⏺).
        """
        try:
            from gui.dialogs.capture_settings_dialog import (
                ImageCaptureSettingsDialog, VideoRecordingSettingsDialog)
            from SupportClasses.CaptureContext import get_settings
            cls = (VideoRecordingSettingsDialog if kind == "video"
                   else ImageCaptureSettingsDialog)
            dlg = cls(get_settings(), self._manager, self._cam_idx, parent=self)
            # Freshly built per open, so it must be released — otherwise every
            # visit leaves a copy parented to this live feed forever. See
            # gui.widgets.components.exec_dialog.
            from gui.widgets.components import exec_dialog
            exec_dialog(dlg)
        except Exception as exc:
            logger.warning(f"capture settings unavailable: {exc}")

    def set_capture_context_provider(self, fn):
        """Let the host page name what is being imaged (channel/well/plate).

        Those land in the filename template AND the embedded metadata.
        """
        ctrl = self.capture_controller()
        if ctrl is not None:
            ctrl.set_context_provider(fn)

    def _on_capture_clicked(self):
        ctrl = self.capture_controller()
        if ctrl is None:
            return
        ctrl.set_orientation_provider(
            lambda: (self._view_rot_deg, self._view_mirror,
                     getattr(self, "_view_flip_y", False)))
        if ctrl.capture_still():
            self._capture_btn.setEnabled(False)

    def _on_record_clicked(self):
        ctrl = self.capture_controller()
        if ctrl is None:
            return
        if ctrl.is_recording:
            ctrl.stop_recording()
            return
        ctrl.set_orientation_provider(
            lambda: (self._view_rot_deg, self._view_mirror,
                     getattr(self, "_view_flip_y", False)))
        ctrl.start_recording()

    def _on_capture_done(self, path, note):
        if self._capture_btn is not None:
            self._capture_btn.setEnabled(True)
        name = Path(path).name if path else ""
        self._flash_status(f"Saved {name}")
        logger.info(f"Capture saved: {path} ({note})")

    def _on_capture_failed(self, reason):
        if self._capture_btn is not None:
            self._capture_btn.setEnabled(True)
        self._flash_status(f"Capture failed: {reason}", ms=6000)
        logger.warning(f"Capture failed: {reason}")

    def _on_record_state(self, active, elapsed_s, frames):
        self._recording = bool(active)
        btn = getattr(self, "_record_btn", None)
        if btn is None:
            return
        if active:
            mins, secs = divmod(int(elapsed_s), 60)
            btn.setText(f"⏹ {mins}:{secs:02d}")
            btn.setStyleSheet(_RECORD_ACTIVE_QSS)
            btn.setFixedSize(max(56, btn.fontMetrics().horizontalAdvance(
                btn.text()) + 18), 26)
            btn.setToolTip("Stop recording")
        else:
            btn.setText("⏺")
            btn.setStyleSheet(_OVERLAY_BTN_QSS)
            btn.setFixedSize(26, 26)
            btn.setToolTip("Record video from this camera")
        self._position_settings_btn()      # the pill's width changed

    def _flash_status(self, text: str, ms: int = 3500):
        """Transient caption over the feed confirming what was written."""
        lbl = getattr(self, "_capture_toast", None)
        if lbl is None:
            from PySide6.QtWidgets import QLabel
            lbl = QLabel("", self._display)
            lbl.setStyleSheet(
                "QLabel { background: rgba(30,30,46,205); color: #a6e3a1;"
                "  border-radius: 4px; padding: 3px 8px; font-size: 11px; }")
            self._capture_toast = lbl
        lbl.setText(text)
        lbl.adjustSize()
        lbl.move(6, max(6, self._display.height() - lbl.height() - 6))
        lbl.show()
        lbl.raise_()
        QTimer.singleShot(int(ms), lbl.hide)

    def _update_capture_visibility(self):
        """Capture/record become AVAILABLE once the camera is delivering.

        Deliberately NOT on the gear's 30-frame capability poll: that is an
        SDK round-trip, and a capture button that takes a second to appear
        reads as broken.

        v7.15: sets availability; whether a button is actually on screen is
        decided by the hover state and the space available (see
        ``_position_settings_btn``).
        """
        live = self._frame_count > 0 and self._manager is not None
        if live != self._capture_available:
            self._capture_available = live
            self._position_settings_btn()

    def _update_settings_visibility(self):
        """The gear is available only when the camera has controllable HW."""
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
        if controllable != self._settings_available:
            self._settings_available = controllable
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
        # v7.14 — capture/record: cheap local check (no SDK), so they appear on
        # the FIRST frame rather than up to a second later.
        if self._frame_count == 1 and getattr(self, "_capture_btn", None):
            self._update_capture_visibility()

        # v7.13 — raw-count saturation badge (every 10th frame; the stats read
        # is a lock snapshot, not an SDK round-trip).
        if self._frame_count % 10 == 1:
            self._update_saturation_flag()

        self._render_frame(q_img)

    def _update_saturation_flag(self):
        """Poll the backend's raw-frame stats for clipping (guarded no-op on
        cameras without raw retention)."""
        sat = False
        try:
            getter = getattr(self._manager, "get_raw_frame_stats", None)
            stats = getter(self._cam_idx) if callable(getter) else None
            if stats is not None:
                sat = float(stats.get("clipped_frac", 0.0)) >= SATURATION_WARN_FRAC
        except Exception:
            sat = False
        self._saturated = sat

    def _compose_pixmap(self, q_img: QImage):
        """Orientation + zoom crop, as ``(pixmap, displayed_image, true_xf)``.

        v7.15: extracted so the subclasses that override ``_render_frame``
        (TargetOverlayCameraView, MeasurementCameraView, _PickerCameraView)
        share this geometry instead of each re-implementing the pipeline.
        They previously skipped ``_orient_qimage`` entirely, which left
        ``_view_true_xform`` unset — so zoom added only to the base class
        would silently not apply to the picker, the primary click-to-select
        surface.

        The zoom is a CROP of the oriented image rather than a scale-up of the
        whole thing: only the visible region is resampled, so the cost does
        not grow with the zoom factor on a 2048² frame.
        """
        disp_img, true_xf = self._orient_qimage(q_img)
        self._view_true_xform = true_xf
        self._displayed_image_size = (disp_img.width(), disp_img.height())
        self._crop_origin = (0.0, 0.0)
        self._cropped_size = (disp_img.width(), disp_img.height())
        if self._zoom > 1.0:
            rect = visible_rect(disp_img.width(), disp_img.height(),
                                self._zoom, self._view_center)
            ir = rect.toRect()
            disp_img = disp_img.copy(ir)
            self._crop_origin = (float(ir.x()), float(ir.y()))
            self._cropped_size = (disp_img.width(), disp_img.height())
        return QPixmap.fromImage(disp_img), true_xf

    def _map_to_pixmap(self, true_xf, x: float, y: float) -> tuple:
        """RAW frame px → coords on the pixmap currently being painted.

        v7.15: the pixmap is a CROP of the oriented image when zoomed, so the
        crop origin has to come off after the orientation transform. Every
        overlay goes through here; a site that mapped with ``true_xf`` alone
        would drift the moment the operator zoomed.
        """
        if true_xf is not None:
            pt = true_xf.map(QPointF(float(x), float(y)))
            x, y = pt.x(), pt.y()
        ox, oy = self._crop_origin
        return (x - ox, y - oy)

    def _publish_geometry(self, scaled):
        """Record how the pixmap now on screen maps to the raw frame.

        The src_rect comes from the crop that was ACTUALLY applied, not from
        recomputing it — see ViewGeometry.build.
        """
        from PySide6.QtCore import QRectF
        cw, ch = self._cropped_size
        self._geometry = ViewGeometry.build(
            raw_size=self._last_image_size,
            disp_size=self._displayed_image_size,
            pixmap_size=(scaled.width(), scaled.height()),
            label_size=(self._display.width(), self._display.height()),
            true_xform=self._view_true_xform,
            src_rect=QRectF(self._crop_origin[0], self._crop_origin[1],
                            float(cw), float(ch)))

    def _render_frame(self, q_img: QImage):
        """Scale and display a QImage on the label."""
        # v7.5.x: apply the display-only view orientation first, then draw
        # overlays in the DISPLAYED frame (markers/vector mapped through the
        # same transform so they stay registered).
        pixmap, true_xf = self._compose_pixmap(q_img)

        # Draw crosshair (display centre, axis-aligned)
        if self._show_crosshair:
            self._draw_crosshair(pixmap)

        # v7.5.x: optional displacement-vector overlay
        if self._overlay_vector is not None:
            self._draw_overlay_vector(pixmap, true_xf)

        # v7.5.x: persistent reference markers projected from stage µm
        if self._ref_markers and self._ref_um_per_px > 0:
            self._draw_reference_markers(pixmap, true_xf)

        # v7.20: commanded toolpaths, UNDER the dots — the dots label the ends
        # of these paths, so they must not be painted over.
        if self._stage_paths:
            self._draw_stage_paths(pixmap, true_xf)

        # v7.13: needle-bore dots (camera-centre-relative, fixed in frame)
        if self._bore_markers:
            self._draw_bore_markers(pixmap, true_xf)

        # Scale to fit display label
        display_size = self._display.size()
        if display_size.width() < 1 or display_size.height() < 1:
            return
        scaled = pixmap.scaled(
            display_size,
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation,
        )
        # v7.10: the alignment ghost is composited HERE, after the downscale —
        # blending a ~600x450 pixmap is sub-millisecond, whereas orienting a
        # full-res ghost per frame costs ~33 ms on a 10 Mpx camera.
        if self._ghost_img is not None:
            self._draw_alignment_ghost(scaled)
        # v7.13 — saturation badge, drawn post-downscale (sub-millisecond) in
        # the top-LEFT corner (the gear owns the top-right).
        if self._saturated:
            self._draw_saturation_badge(scaled)
        self._last_pixmap = scaled
        self._publish_geometry(scaled)
        self._display.setPixmap(scaled)
        # Keep the gear pinned to the top-right above the pixmap.
        if self._settings_btn is not None and self._settings_btn.isVisible():
            self._position_settings_btn()

    def _draw_saturation_badge(self, pixmap: QPixmap):
        """Small red 'SATURATED' pill, top-left of the displayed frame.

        Signals raw-count clipping the auto-scaled display can't show —
        reduce exposure (or raise the white level) when this is lit.
        """
        painter = QPainter(pixmap)
        try:
            painter.setRenderHint(QPainter.RenderHint.Antialiasing, True)
            font = QFont()
            font.setPointSizeF(max(7.0, pixmap.height() / 28.0))
            font.setBold(True)
            painter.setFont(font)
            text = "SATURATED"
            metrics = painter.fontMetrics()
            pad = max(3, metrics.height() // 3)
            w = metrics.horizontalAdvance(text) + 2 * pad
            h = metrics.height() + pad
            painter.setPen(Qt.PenStyle.NoPen)
            painter.setBrush(QBrush(QColor(243, 139, 168, 200)))  # red, ~80%
            painter.drawRoundedRect(6, 6, w, h, 4, 4)
            painter.setPen(QPen(QColor("#11111b")))
            painter.drawText(6 + pad, 6 + pad // 2 + metrics.ascent(), text)
        finally:
            painter.end()

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
            cx, cy = self._map_to_pixmap(true_xf, rw / 2.0, rh / 2.0)
            ex, ey = self._map_to_pixmap(true_xf, rw / 2.0 + dx,
                                         rh / 2.0 + dy)
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
            ix, iy = self._map_to_pixmap(true_xf, ix, iy)
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

    def _draw_bore_markers(self, pixmap: QPixmap, true_xf=None):
        """v7.13: filled colored dot + label per bore.

        Each marker's (dx, dy) is camera-centre-relative stage µm, projected to
        RAW frame px through the calibrated inverse map
        (:meth:`_bore_marker_raw_px`), then through the view transform — so the
        dots stay registered on a flipped/rotated display exactly like clicks.
        """
        disp_w, disp_h = pixmap.width(), pixmap.height()
        if not disp_w or not disp_h:
            return
        raw_w, raw_h = self._last_image_size
        if not raw_w or not raw_h:
            raw_w, raw_h = disp_w, disp_h
        painter = QPainter(pixmap)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing, True)
        r = max(5.0, pixmap.width() / 150.0)
        font = QFont("Consolas", max(8, pixmap.width() // 110))
        font.setBold(True)
        painter.setFont(font)
        for label, dx_um, dy_um, color_hex in self._bore_markers:
            ix, iy = self._bore_marker_raw_px(dx_um, dy_um, raw_w, raw_h)
            ix, iy = self._map_to_pixmap(true_xf, ix, iy)
            if ix < -40 or iy < -40 or ix > disp_w + 40 or iy > disp_h + 40:
                continue
            color = QColor(color_hex)
            if not color.isValid():
                color = QColor(COLORS.get("green", "#a6e3a1"))
            fill = QColor(color)
            fill.setAlpha(170)
            painter.setPen(QPen(color, max(2, pixmap.width() // 400)))
            painter.setBrush(QBrush(fill))
            painter.drawEllipse(QPointF(ix, iy), r, r)
            if label:
                painter.setPen(QPen(color))
                painter.drawText(int(ix + r + 4), int(iy - 4), label)
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
        if obj is self._display and self._pan_event(event):
            return True
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

    def _pan_event(self, event) -> bool:
        """Middle-button drag pans while zoomed in. True if consumed.

        ⚠ MIDDLE, not left. Left-click on these views SELECTS — a well rim, a
        target, a needle edge — so a left-drag pan (Qt's ScrollHandDrag, or a
        RubberBandDrag) would claim the button the whole feature exists for.
        The same split the repo already uses in _ZoomImageView.
        """
        et = event.type()
        if et == QEvent.Type.MouseButtonPress:
            if (event.button() == Qt.MiddleButton and self._zoom > 1.0
                    and self._last_pixmap):
                self._panning = True
                self._pan_anchor = ((event.position().x(),
                                     event.position().y()),
                                    self._current_center())
                self.setCursor(Qt.ClosedHandCursor)
                return True
            return False
        if et == QEvent.Type.MouseMove and self._panning:
            geo = self.geometry_map()
            (ax, ay), c0 = self._pan_anchor
            if c0 is None or not geo.valid:
                return True
            sc = geo.scale or 1.0
            # Drag the picture WITH the cursor: the view centre moves opposite.
            self._view_center = (c0[0] - (event.position().x() - ax) / sc,
                                 c0[1] - (event.position().y() - ay) / sc)
            self._rerender()
            return True
        if et == QEvent.Type.MouseButtonRelease and self._panning:
            self._panning = False
            self._pan_anchor = None
            self.unsetCursor()
            return True
        return False

    # ── Zoom / pan (v7.15) ────────────────────────────────────────

    def set_zoom_enabled(self, enabled: bool) -> None:
        """Opt out for a view where magnifying makes no sense."""
        self._zoom_enabled = bool(enabled)
        if not self._zoom_enabled:
            self.zoom_fit()

    @property
    def zoom(self) -> float:
        return self._zoom

    def zoom_fit(self) -> None:
        """Back to the whole frame, centred."""
        self._zoom = 1.0
        self._view_center = None
        self._rerender()

    def set_zoom(self, z: float, *, about=None) -> None:
        """Set the zoom, optionally holding a WIDGET point still.

        Holding the point under the cursor is what makes wheel-zoom feel
        right; the buttons pass ``about=None`` and zoom about the centre.
        """
        if not self._zoom_enabled:
            return
        new = clamp_zoom(z)
        if abs(new - self._zoom) < 1e-9:
            return
        anchor = None
        if about is not None:
            geo = self.geometry_map()
            if geo.valid:
                pt = geo.to_image(about[0], about[1], clamp=True)
                if pt is not None and geo.true_xform is not None:
                    p = geo.true_xform.map(QPointF(pt[0], pt[1]))
                    anchor = (p.x(), p.y())
                elif pt is not None:
                    anchor = pt
        old_center = self._current_center()
        self._zoom = new
        if anchor is not None and new > 1.0:
            # Keep the anchor at the same fraction across the view.
            dw, dh = self._displayed_image_size
            frac_x = ((about[0] - self.geometry_map().offset[0])
                      / max(1.0, self.geometry_map().pixmap_size[0]))
            frac_y = ((about[1] - self.geometry_map().offset[1])
                      / max(1.0, self.geometry_map().pixmap_size[1]))
            vw, vh = dw / new, dh / new
            self._view_center = (anchor[0] - (frac_x - 0.5) * vw,
                                 anchor[1] - (frac_y - 0.5) * vh)
        elif new <= 1.0:
            self._view_center = None
        else:
            self._view_center = old_center
        self._rerender()

    def zoom_in(self):
        self.set_zoom(self._zoom * 1.6)

    def zoom_out(self):
        self.set_zoom(self._zoom / 1.6)

    def _current_center(self):
        dw, dh = self._displayed_image_size
        if self._view_center is not None:
            return self._view_center
        return (dw / 2.0, dh / 2.0) if dw and dh else None

    def _rerender(self):
        """Redraw the last frame with the current zoom, without waiting for
        the next one — a zoom must feel immediate even at 4 fps."""
        if self._last_qimage is not None:
            self._render_frame(self._last_qimage)

    def wheelEvent(self, event):
        if not self._zoom_enabled or self._last_pixmap is None:
            super().wheelEvent(event)
            return
        step = 1.25 if event.angleDelta().y() > 0 else 0.8
        pos = event.position()
        # The wheel arrives on the view; the geometry is in LABEL coords.
        p = self._display.mapFrom(self, pos.toPoint())
        self.set_zoom(self._zoom * step, about=(p.x(), p.y()))
        event.accept()

    def geometry_map(self):
        """How the pixmap on screen maps to the raw frame (v7.15).

        The ONE transform. Overlay painters and hit-testing both take it from
        here, so they cannot disagree — which is what adding zoom would
        otherwise have caused.

        Falls back to deriving the mapping from the current pixmap when none
        has been published. That keeps a caller that sets ``_last_pixmap``
        directly working (several tests drive the view without rendering a
        frame), and means a subclass that overrides ``_render_frame`` without
        publishing still gets correct clicks rather than silently dead ones.
        """
        geo = getattr(self, "_geometry", None)
        if geo is not None and geo.valid:
            return geo
        pm = self._last_pixmap
        if pm is None or pm.isNull():
            return NULL_GEOMETRY
        disp = self._displayed_image_size
        if not disp[0] or not disp[1]:
            disp = self._last_image_size
        if not disp[0] or not disp[1]:
            return NULL_GEOMETRY
        return ViewGeometry.build(
            raw_size=self._last_image_size, disp_size=disp,
            pixmap_size=(pm.width(), pm.height()),
            label_size=(self._display.width(), self._display.height()),
            true_xform=self._view_true_xform)

    def _widget_to_image(self, wx: float, wy: float, *, clamp: bool = False
                         ) -> Optional[tuple[float, float]]:
        """Widget pixel coords → RAW frame pixel coords.

        v7.15: delegates to :class:`ViewGeometry`. This used to hand-roll the
        letterbox + scale + orientation-inverse chain, and three other places
        hand-rolled it again; none knew about zoom.
        """
        geo = self.geometry_map()
        if not geo.valid:
            return None
        return geo.to_image(wx, wy, clamp=clamp)

    def _image_to_widget(self, ix: float, iy: float
                         ) -> Optional[tuple[float, float]]:
        """RAW frame px → widget px. Exact inverse of :meth:`_widget_to_image`."""
        geo = self.geometry_map()
        if not geo.valid:
            return None
        return geo.to_widget(ix, iy)

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

"""
mosaic_registration_view.py — Zoomable tile view for FOV / spacing calibration.

v7.5.x: The Calibrate… pop-out shows the calibration mosaic as INDIVIDUAL,
semi-transparent tiles (not one blended composite) so the operator can adjust the
inter-tile spacing and align features in the OVERLAP regions by eye. From the
spacing factor that makes the overlaps line up, the dialog derives the corrected
effective FOV (µm/px) / grid step for the full mosaic — "the best settings".

Each tile is drawn at its stage-commanded footprint, scaled about the tile-grid
centroid by a per-axis spacing factor ``(kx, ky)``:

    drawn_center = centroid + (kx·(nominal_x − Cx), ky·(nominal_y − Cy))

so ``k > 1`` spreads tiles apart and ``k < 1`` pulls them together — moving the
images toward/away from each other. Tiles are semi-transparent so overlapping
features visibly double until aligned. A peach outline marks each tile.

Wheel-zoom (about the cursor) + drag-pan come free from QGraphicsView.
Coordinates are mosaic pixels. Zero hardware dependencies.
"""

from __future__ import annotations

import logging

from PySide6.QtCore import Qt, QRectF
from PySide6.QtGui import QColor, QPainter, QPen
from PySide6.QtWidgets import (
    QGraphicsScene, QGraphicsView, QGraphicsSimpleTextItem,
)

from gui.styles import COLORS

logger = logging.getLogger(__name__)


class MosaicRegistrationView(QGraphicsView):
    """Zoom/pan view of individual mosaic tiles with adjustable spacing."""

    _ZOOM_STEP = 1.0015          # per wheel-delta unit
    _ZOOM_MIN = 0.05
    _ZOOM_MAX = 80.0

    def __init__(self, parent=None):
        super().__init__(parent)
        self._scene = QGraphicsScene(self)
        self.setScene(self._scene)
        self.setRenderHint(QPainter.RenderHint.SmoothPixmapTransform)
        self.setRenderHint(QPainter.RenderHint.Antialiasing)
        self.setDragMode(QGraphicsView.DragMode.ScrollHandDrag)
        self.setTransformationAnchor(
            QGraphicsView.ViewportAnchor.AnchorUnderMouse)
        self.setResizeAnchor(QGraphicsView.ViewportAnchor.AnchorViewCenter)
        self.setBackgroundBrush(QColor(COLORS["base"]))
        self.setMinimumSize(280, 220)

        # Per-tile state: nominal (centre_x, centre_y, w, h) in mosaic px + items.
        self._tiles: list = []                 # (cx, cy, w, h)
        self._pix_items: list = []
        self._outline_items: list = []
        self._centroid = (0.0, 0.0)
        self._kx: float = 1.0
        self._ky: float = 1.0
        self._opacity: float = 0.6
        self._zoom: float = 1.0
        # v7.5.x: display the tiles in the SAME plate-display frame as the full
        # plate view (JogWorkspaceView rotates the mosaic 180° when
        # ``plate_flip_180``). Without this the calibration view shows the STAGE
        # frame while the full mosaic shows the PLATE frame, so an orientation
        # tuned here looked "upside down" / on the wrong side there. Applied as a
        # whole-view 180° rotation (re-asserted in ``fit``).
        self._flip_180: bool = False
        self._fitted_once: bool = False
        self._placeholder = None
        self._show_placeholder("Build a calibration mosaic to see it here.")

    # ── Content ────────────────────────────────────────────────────

    def _show_placeholder(self, text: str) -> None:
        if self._pix_items:
            return
        if self._placeholder is None:
            self._placeholder = QGraphicsSimpleTextItem(text)
            self._placeholder.setBrush(QColor(COLORS["subtext0"]))
            self._scene.addItem(self._placeholder)
        else:
            self._placeholder.setText(text)

    def has_content(self) -> bool:
        return bool(self._pix_items)

    def tile_count(self) -> int:
        return len(self._pix_items)

    def set_tiles(self, tiles, scale_px_per_um=1.0) -> None:
        """Render each tile individually.

        ``tiles`` is a list of ``(frame_bgr, left, top, w, h)`` (mosaic px, from
        ``MosaicBuilder.tile_images_px()``). Tiles are drawn semi-transparent so
        overlapping features blend; the spacing factor places them about the
        centroid so the operator can align the overlaps.
        """
        from gui.widgets.jog_workspace_view import pixmap_from_bgr
        # Clear old items.
        for it in self._pix_items + self._outline_items:
            self._scene.removeItem(it)
        self._pix_items = []
        self._outline_items = []
        self._tiles = []
        if self._placeholder is not None:
            self._scene.removeItem(self._placeholder)
            self._placeholder = None

        pen = QPen(QColor(COLORS["peach"]))
        pen.setCosmetic(True)
        pen.setWidthF(1.2)
        cxs, cys = [], []
        for t in (tiles or []):
            try:
                frame, left, top, w, h = t
                w = max(1.0, float(w))
                h = max(1.0, float(h))
            except (TypeError, ValueError):
                continue
            pm = pixmap_from_bgr(frame)
            if pm is None or pm.isNull():
                continue
            pm = pm.scaled(int(round(w)), int(round(h)),
                           Qt.AspectRatioMode.IgnoreAspectRatio,
                           Qt.TransformationMode.SmoothTransformation)
            item = self._scene.addPixmap(pm)
            item.setOpacity(self._opacity)
            item.setZValue(0)
            self._pix_items.append(item)
            outline = self._scene.addRect(QRectF(0, 0, w, h), pen)
            outline.setZValue(2)
            self._outline_items.append(outline)
            cx = float(left) + w / 2.0
            cy = float(top) + h / 2.0
            self._tiles.append((cx, cy, w, h))
            cxs.append(cx)
            cys.append(cy)

        if cxs:
            self._centroid = (sum(cxs) / len(cxs), sum(cys) / len(cys))
        self._place_tiles()
        if not self._fitted_once and self._pix_items:
            self.fit()

    def _place_tiles(self) -> None:
        """Position every tile (and its outline) at the spacing-scaled centre."""
        cx0, cy0 = self._centroid
        for (cx, cy, w, h), pix, outline in zip(
                self._tiles, self._pix_items, self._outline_items):
            dx = cx0 + self._kx * (cx - cx0)
            dy = cy0 + self._ky * (cy - cy0)
            left = dx - w / 2.0
            top = dy - h / 2.0
            pix.setPos(left, top)
            outline.setPos(left, top)
        self._update_scene_rect()

    def _update_scene_rect(self) -> None:
        br = self._scene.itemsBoundingRect()
        if br.isEmpty():
            return
        pad_w = max(br.width() * 0.5, 50.0)
        pad_h = max(br.height() * 0.5, 50.0)
        self._scene.setSceneRect(br.adjusted(-pad_w, -pad_h, pad_w, pad_h))

    # ── Spacing + opacity ──────────────────────────────────────────

    def set_spacing_factor(self, kx: float, ky: float) -> None:
        """Scale the tile spacing about the centroid (kx, ky > 0). >1 spreads
        tiles apart, <1 pulls them together (move images toward/away)."""
        self._kx = max(0.1, float(kx))
        self._ky = max(0.1, float(ky))
        self._place_tiles()

    def spacing_factor(self) -> tuple[float, float]:
        return (self._kx, self._ky)

    def set_image_opacity(self, opacity: float) -> None:
        self._opacity = max(0.05, min(1.0, float(opacity)))
        for pix in self._pix_items:
            pix.setOpacity(self._opacity)

    # ── Zoom / pan ─────────────────────────────────────────────────

    def set_flip_180(self, flip: bool) -> None:
        """Show the tiles in the plate-display frame (180° whole-view rotation),
        matching JogWorkspaceView when ``plate_flip_180`` — so what the operator
        orients here is exactly what the full mosaic shows."""
        flip = bool(flip)
        if flip == self._flip_180:
            return
        self._flip_180 = flip
        self._fitted_once = False
        self.fit()

    def set_output_rotation(self, rotation_deg: float) -> None:
        """Additional DISPLAY-ONLY whole-view rotation (0/90/180/270).

        v7.5.x: the operator's whole-mosaic output rotation, composed with the
        plate ``flip_180`` above so this preview shows the mosaic the same way up
        as the finished one. Purely a ``QGraphicsView`` transform — scene
        coordinates (and therefore any back-projection) are untouched.
        """
        try:
            quad = int(round(float(rotation_deg) / 90.0)) % 4 * 90
        except (TypeError, ValueError):
            quad = 0
        if quad == self.output_rotation():
            return
        self._output_rotation = quad
        self._fitted_once = False
        self.fit()

    def output_rotation(self) -> int:
        return int(getattr(self, "_output_rotation", 0) or 0)

    def fit(self) -> None:
        br = self._scene.itemsBoundingRect()
        if br.isEmpty():
            return
        self.resetTransform()
        self._zoom = 1.0
        # Plate-frame flip composed with the operator's output rotation.
        total = ((180 if self._flip_180 else 0) + self.output_rotation()) % 360
        if total:
            # Rotate about the view centre so the fitted content stays centred.
            self.setTransformationAnchor(
                QGraphicsView.ViewportAnchor.AnchorViewCenter)
            self.rotate(total)
            self.setTransformationAnchor(
                QGraphicsView.ViewportAnchor.AnchorUnderMouse)
        self.fitInView(br, Qt.AspectRatioMode.KeepAspectRatio)
        self._fitted_once = True

    def reset_view(self) -> None:
        self.fit()

    def wheelEvent(self, event) -> None:
        delta = event.angleDelta().y()
        if delta == 0 or not self._pix_items:
            event.accept()
            return
        factor = self._ZOOM_STEP ** delta
        new_zoom = max(self._ZOOM_MIN, min(self._ZOOM_MAX, self._zoom * factor))
        applied = new_zoom / self._zoom
        if abs(applied - 1.0) < 1e-9:
            event.accept()
            return
        self._zoom = new_zoom
        self.scale(applied, applied)     # AnchorUnderMouse → zooms about cursor
        event.accept()

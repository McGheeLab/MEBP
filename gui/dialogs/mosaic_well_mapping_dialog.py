"""
mosaic_well_mapping_dialog.py — Manual well mapping on a stitched mosaic.

v7.5.x: When automatic well detection fails on a full-plate mosaic, the operator
maps the plate by hand here:

  1. Click the 3 corner wells on the mosaic image.
  2. The rest of the wells are AUTO-PLACED from the plate layout (affine fit
     through the 3 corners).
  3. Drag any well circle to refine it.
  4. Confirm — the well centres (mosaic px → absolute stage µm) feed the same warp
     fit as the manual teach, AND the mosaic + labels are saved as a training
     sample (``WellTrainingStore``) for improving auto-detection later.

The view shows the mosaic image itself (zoom = wheel, pan = middle-drag). Coords
are mosaic pixels; px → stage µm uses the stored extent + scale (the inverse of
``MosaicBuilder`` placement, identical to ``_ploc_fit_from_mosaic_detections``).
"""

from __future__ import annotations

import logging

from PySide6.QtCore import Qt, QPointF, QRectF, Signal
from PySide6.QtGui import QColor, QPainter, QPen, QBrush
from PySide6.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QGraphicsScene,
    QGraphicsView, QGraphicsEllipseItem, QGraphicsSimpleTextItem, QApplication,
)

from gui.styles import COLORS
from gui.scaling import s

logger = logging.getLogger(__name__)


def solve_affine_3(nom_pts, tgt_pts):
    """Affine (a,b,c,d,e,f) with tx=a·nx+b·ny+c, ty=d·nx+e·ny+f from 3 pairs.
    Returns None if the 3 nominal points are collinear (singular)."""
    import numpy as np
    M = np.array([[nom_pts[i][0], nom_pts[i][1], 1.0] for i in range(3)],
                 dtype=float)
    try:
        ax = np.linalg.solve(M, np.array([tgt_pts[i][0] for i in range(3)],
                                         dtype=float))
        ay = np.linalg.solve(M, np.array([tgt_pts[i][1] for i in range(3)],
                                         dtype=float))
    except Exception:
        return None
    return (float(ax[0]), float(ax[1]), float(ax[2]),
            float(ay[0]), float(ay[1]), float(ay[2]))


def apply_affine(aff, nx, ny):
    a, b, c, d, e, f = aff
    return (a * nx + b * ny + c, d * nx + e * ny + f)


def corner_well_names(plate):
    """Three non-collinear corner wells (origin, +X end, +Y end) by nominal
    position extremes — robust to naming / custom plates."""
    names = list(plate.well_names)
    if len(names) < 3:
        return names
    pos = {}
    for n in names:
        try:
            pos[n] = plate.get_well_position(n)
        except Exception:
            pos[n] = (0.0, 0.0)
    origin = min(names, key=lambda n: pos[n][0] + pos[n][1])     # min x+y (A1)
    xend = max(names, key=lambda n: pos[n][0] - pos[n][1])       # max x, min y
    yend = max(names, key=lambda n: pos[n][1] - pos[n][0])       # min x, max y
    chosen = [origin]
    for c in (xend, yend):
        if c not in chosen:
            chosen.append(c)
    for n in names:                 # fallback if any collided
        if len(chosen) >= 3:
            break
        if n not in chosen:
            chosen.append(n)
    return chosen[:3]


class _MappingView(QGraphicsView):
    """Mosaic image view: wheel-zoom (about cursor), middle-drag pan, and a
    left-click signal used to place corner wells (when click-mode is on)."""

    clicked = Signal(QPointF)
    _ZOOM_STEP = 1.0015
    _ZOOM_MIN = 0.05
    _ZOOM_MAX = 80.0

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setRenderHint(QPainter.RenderHint.SmoothPixmapTransform)
        self.setRenderHint(QPainter.RenderHint.Antialiasing)
        self.setDragMode(QGraphicsView.DragMode.NoDrag)
        self.setTransformationAnchor(
            QGraphicsView.ViewportAnchor.AnchorUnderMouse)
        self.setBackgroundBrush(QColor(COLORS["base"]))
        self.setMinimumSize(s(420), s(320))
        self._zoom = 1.0
        self.click_mode = False
        self._panning = False
        self._pan_start = QPointF(0, 0)
        self._h0 = 0
        self._v0 = 0

    def wheelEvent(self, event):
        d = event.angleDelta().y()
        if d == 0:
            event.accept()
            return
        factor = self._ZOOM_STEP ** d
        new_zoom = max(self._ZOOM_MIN, min(self._ZOOM_MAX, self._zoom * factor))
        applied = new_zoom / self._zoom
        if abs(applied - 1.0) < 1e-9:
            event.accept()
            return
        self._zoom = new_zoom
        self.scale(applied, applied)
        event.accept()

    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.MiddleButton:
            self._panning = True
            self._pan_start = event.position()
            self._h0 = self.horizontalScrollBar().value()
            self._v0 = self.verticalScrollBar().value()
            self.setCursor(Qt.CursorShape.ClosedHandCursor)
            event.accept()
            return
        if (event.button() == Qt.MouseButton.LeftButton and self.click_mode):
            self.clicked.emit(self.mapToScene(event.position().toPoint()))
            event.accept()
            return
        super().mousePressEvent(event)     # adjust phase → items drag

    def mouseMoveEvent(self, event):
        if self._panning:
            delta = event.position() - self._pan_start
            self.horizontalScrollBar().setValue(int(self._h0 - delta.x()))
            self.verticalScrollBar().setValue(int(self._v0 - delta.y()))
            event.accept()
            return
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.MouseButton.MiddleButton and self._panning:
            self._panning = False
            self.setCursor(Qt.CursorShape.ArrowCursor)
            event.accept()
            return
        super().mouseReleaseEvent(event)


class MosaicWellMappingDialog(QDialog):
    """Map plate wells on a stitched mosaic: 3 corners → auto-fill → adjust."""

    def __init__(self, plate, image_bgr, extent_um, mosaic_scale,
                 um_per_px=0.0, plate_key=None, parent=None):
        super().__init__(parent)
        self._plate = plate
        self._image = image_bgr
        self._extent = tuple(float(v) for v in extent_um) if extent_um else None
        self._scale = float(mosaic_scale) if mosaic_scale else 1.0
        self._um_per_px = float(um_per_px or 0.0)
        self._plate_key = plate_key if plate_key is not None else getattr(
            plate, "format", "plate")

        self._corners = corner_well_names(plate) if plate is not None else []
        self._corner_idx = 0
        self._corner_clicks: dict = {}          # well → QPointF (scene px)
        self._corner_markers: list = []
        self._well_items: dict = {}             # well → QGraphicsEllipseItem
        self._results: dict = {}                # well → (sx_um, sy_um) on confirm
        self._saved_path = None

        self.setWindowTitle("Map Wells on Mosaic")
        self.setModal(True)
        self._build_ui()
        self._load_image()
        self._begin_corner_phase()
        # Try automatic detection immediately; if it can't lock the grid, the
        # 3-corner click workflow (already armed) stays as the fallback.
        try:
            self._auto_detect()
        except Exception as e:
            logger.debug(f"Auto-detect on open skipped: {e}")

    # ── UI ─────────────────────────────────────────────────────────

    def _build_ui(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(s(10), s(10), s(10), s(10))
        root.setSpacing(s(6))

        self._instr = QLabel("")
        self._instr.setWordWrap(True)
        self._instr.setStyleSheet(f"color: {COLORS['text']}; font-weight: bold;")
        root.addWidget(self._instr)

        self._view = _MappingView()
        self._view.clicked.connect(self._on_view_clicked)
        self._scene = QGraphicsScene(self)
        self._view.setScene(self._scene)
        root.addWidget(self._view, stretch=1)

        self._status = QLabel("")
        self._status.setWordWrap(True)
        self._status.setStyleSheet(f"color: {COLORS['subtext0']};")
        root.addWidget(self._status)

        row = QHBoxLayout()
        self._btn_auto = QPushButton("Auto-detect wells")
        self._btn_auto.setToolTip(
            "Find all wells automatically (filled-disc detection + grid fit). "
            "Then drag any to refine. Falls back to the 3-corner method below.")
        self._btn_auto.clicked.connect(self._auto_detect)
        row.addWidget(self._btn_auto)
        self._btn_recorners = QPushButton("Re-place corners")
        self._btn_recorners.clicked.connect(self._begin_corner_phase)
        row.addWidget(self._btn_recorners)
        self._btn_fit = QPushButton("Fit view")
        self._btn_fit.clicked.connect(self._fit)
        row.addWidget(self._btn_fit)
        row.addStretch()
        self._btn_confirm = QPushButton("Confirm + calibrate + save")
        self._btn_confirm.setObjectName("accentBtn")
        self._btn_confirm.setEnabled(False)
        self._btn_confirm.clicked.connect(self._on_confirm)
        row.addWidget(self._btn_confirm)
        self._btn_close = QPushButton("Cancel")
        self._btn_close.clicked.connect(self.reject)
        row.addWidget(self._btn_close)
        root.addLayout(row)

        self.resize(s(900), s(680))
        try:
            scr = self.screen() or QApplication.primaryScreen()
            if scr is not None:
                self.setMaximumHeight(int(scr.availableGeometry().height() - s(60)))
        except Exception:
            pass

    def _load_image(self):
        from gui.widgets.jog_workspace_view import pixmap_from_bgr
        pm = pixmap_from_bgr(self._image)
        if pm is not None and not pm.isNull():
            self._pixmap_item = self._scene.addPixmap(pm)
            self._pixmap_item.setZValue(0)
            self._scene.setSceneRect(self._pixmap_item.boundingRect())
            self._fit()

    def _fit(self):
        br = self._scene.itemsBoundingRect()
        if not br.isEmpty():
            self._view.resetTransform()
            self._view._zoom = 1.0
            self._view.fitInView(br, Qt.AspectRatioMode.KeepAspectRatio)

    # ── Corner phase ───────────────────────────────────────────────

    def _begin_corner_phase(self):
        for it in self._corner_markers:
            self._scene.removeItem(it)
        self._corner_markers = []
        for it in self._well_items.values():
            self._scene.removeItem(it)
        self._well_items = {}
        self._corner_clicks = {}
        self._corner_idx = 0
        self._results = {}
        self._btn_confirm.setEnabled(False)
        self._view.click_mode = True
        if len(self._corners) < 3:
            self._instr.setText("This plate has fewer than 3 wells — cannot map.")
            self._view.click_mode = False
            return
        self._prompt_next_corner()

    def _prompt_next_corner(self):
        w = self._corners[self._corner_idx]
        self._instr.setText(
            f"Click corner well {self._corner_idx + 1}/3:  «{w}»   "
            f"(wheel = zoom · middle-drag = pan)")

    def _on_view_clicked(self, scene_pt: QPointF):
        if not self._view.click_mode or self._corner_idx >= 3:
            return
        well = self._corners[self._corner_idx]
        self._corner_clicks[well] = scene_pt
        self._add_corner_marker(scene_pt, well)
        self._corner_idx += 1
        if self._corner_idx >= 3:
            self._autofill()
        else:
            self._prompt_next_corner()

    def _add_corner_marker(self, pt: QPointF, label: str):
        r = self._well_radius_px(label)
        pen = QPen(QColor(COLORS["yellow"]))
        pen.setCosmetic(True)
        pen.setWidthF(2.0)
        item = self._scene.addEllipse(
            QRectF(pt.x() - r, pt.y() - r, 2 * r, 2 * r), pen)
        item.setZValue(3)
        self._corner_markers.append(item)
        txt = self._scene.addSimpleText(label)
        txt.setBrush(QBrush(QColor(COLORS["yellow"])))
        txt.setPos(pt.x() + r, pt.y() - r)
        txt.setFlag(QGraphicsSimpleTextItem.GraphicsItemFlag
                    .ItemIgnoresTransformations, True)
        txt.setZValue(3)
        self._corner_markers.append(txt)

    # ── Auto-fill + adjust ─────────────────────────────────────────

    def _well_radius_px(self, name) -> float:
        d_mm = 0.0
        try:
            d_mm = float(self._plate.get_well_info(name).diameter)
        except Exception:
            d_mm = 0.0
        if d_mm <= 0:
            d_mm = float(getattr(self._plate, "well_diameter", 0.0) or 0.0)
        if d_mm <= 0:
            d_mm = 3.0
        r = (d_mm * 1000.0 / 2.0) * self._scale
        return max(3.0, r)

    def _autofill(self):
        nom = []
        tgt = []
        for w in self._corners:
            try:
                nom.append(self._plate.get_well_position(w))
            except Exception:
                nom.append((0.0, 0.0))
            p = self._corner_clicks[w]
            tgt.append((p.x(), p.y()))
        aff = solve_affine_3(nom, tgt)
        if aff is None:
            self._status.setText(
                "The 3 corner clicks are collinear — re-place them at true "
                "corners.")
            return
        self._view.click_mode = False           # switch to adjust (drag wells)
        for it in self._corner_markers:
            self._scene.removeItem(it)
        self._corner_markers = []
        for name in self._plate.well_names:
            try:
                nx, ny = self._plate.get_well_position(name)
            except Exception:
                continue
            px, py = apply_affine(aff, nx, ny)
            self._add_well_item(name, px, py)
        self._btn_confirm.setEnabled(True)
        self._instr.setText(
            f"Auto-placed {len(self._well_items)} wells. Drag any circle to "
            f"refine (middle-drag = pan). Then Confirm.")
        self._status.setText("")

    def _auto_detect(self):
        """Find ALL wells automatically (filled-disc detection + grid fit), then
        place each well at its detected centre — or the grid prediction for any
        well that wasn't directly detected. Leaves the corner workflow intact if
        detection can't lock the grid."""
        import numpy as np
        rows = int(getattr(self._plate, "rows", 0) or 0)
        cols = int(getattr(self._plate, "cols", 0) or 0)
        if rows < 1 or cols < 1:
            self._status.setText("Plate grid unknown — use the 3-corner method.")
            return
        try:
            from SupportClasses.VisionDetector import WellDetector
            dets = WellDetector.detect_filled_wells(self._image)
            centers = [(d.center_px[0], d.center_px[1], d.radius_px)
                       for d in dets]
            aff, assign = WellDetector.fit_well_grid(
                [(c[0], c[1]) for c in centers], rows, cols)
        except Exception as e:
            self._status.setText(f"Auto-detect failed: {e}")
            return
        if not assign or aff is None:
            self._status.setText(
                "Auto-detect couldn't find the grid — click the 3 corner wells "
                "instead.")
            return
        self._view.click_mode = False
        for it in self._corner_markers:
            self._scene.removeItem(it)
        self._corner_markers = []
        for it in self._well_items.values():
            self._scene.removeItem(it)
        self._well_items = {}
        grid_names = {}
        try:
            for w in self._plate.get_all_wells():
                grid_names[(w.row, w.col)] = w.name
        except Exception:
            pass
        A, t = aff[:, :2], aff[:, 2]
        radii = [centers[idx][2] for idx in assign.values()]
        rmed = float(np.median(radii)) if radii else 0.0
        n_det = 0
        for i in range(rows):
            for j in range(cols):
                name = grid_names.get((i, j)) or f"{chr(ord('A') + i)}{j + 1}"
                if (i, j) in assign:
                    cx, cy, _r = centers[assign[(i, j)]]
                    n_det += 1
                else:
                    p = A @ np.array([j, i]) + t
                    cx, cy = float(p[0]), float(p[1])
                self._add_well_item(name, cx, cy)
        self._btn_confirm.setEnabled(True)
        n_fill = len(self._well_items) - n_det
        self._instr.setText(
            f"Auto-detected {n_det} wells"
            + (f" (+{n_fill} filled from grid)" if n_fill else "")
            + ". Drag any circle to refine (middle-drag = pan). Then Confirm.")
        self._status.setText("")

    def _add_well_item(self, name, cx, cy):
        r = self._well_radius_px(name)
        pen = QPen(QColor(COLORS["green"]))
        pen.setCosmetic(True)
        pen.setWidthF(1.6)
        item = QGraphicsEllipseItem(QRectF(cx - r, cy - r, 2 * r, 2 * r))
        item.setPen(pen)
        item.setBrush(QBrush(QColor(0, 0, 0, 0)))
        item.setFlag(QGraphicsEllipseItem.GraphicsItemFlag.ItemIsMovable, True)
        item.setFlag(QGraphicsEllipseItem.GraphicsItemFlag
                     .ItemIsSelectable, True)
        item.setZValue(2)
        item.setToolTip(name)
        self._scene.addItem(item)
        self._well_items[name] = item

    def _well_center_scene(self, item) -> QPointF:
        # rect centre (original placement) + the drag offset (item pos).
        c = item.rect().center()
        return QPointF(c.x() + item.pos().x(), c.y() + item.pos().y())

    # ── Confirm / export ───────────────────────────────────────────

    def _on_confirm(self):
        if not self._well_items or self._extent is None or not self._scale:
            self._status.setText("Place the corner wells first.")
            return
        ox, oy = self._extent[0], self._extent[1]
        results: dict = {}
        wells_label: dict = {}
        for name, item in self._well_items.items():
            c = self._well_center_scene(item)
            sx = ox + c.x() / self._scale       # mosaic px → absolute stage µm
            sy = oy + c.y() / self._scale
            results[name] = (sx, sy)
            wells_label[name] = {
                "px": (c.x(), c.y()), "um": (sx, sy),
                "r_px": self._well_radius_px(name)}
        self._results = results
        # Save the labeled training sample (image + well centres).
        try:
            from SupportClasses.WellTrainingStore import get_store
            self._saved_path = get_store().save_sample(
                self._plate_key, self._image, wells_label,
                extent_um=self._extent, mosaic_scale=self._scale,
                um_per_px=self._um_per_px)
        except Exception as e:
            logger.warning(f"Well training sample save failed: {e}")
        self.accept()

    def results(self) -> dict:
        """{well_name: (stage_x_um, stage_y_um)} after Confirm (else empty)."""
        return self._results

    def saved_sample_path(self):
        return self._saved_path

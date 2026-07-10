"""
mosaic_well_mapping_dialog.py — Guided well mapping on a stitched mosaic.

v7.5.x rework: the operator maps wells on the mosaic image with a guided,
per-well confirm flow (full-plate AND single-well/rosette modes):

  * The mosaic is presented in the PLATE orientation — when the machine's
    ``plate_flip_180`` convention is set, the view is rotated 180° so the image
    matches the plate overlay / live preview (A1 top-left). The rotation is a
    pure VIEW transform: every scene coordinate (clicks, drags, the px → µm
    back-projection, the training-sample labels) is byte-identical to the raw
    stage frame. Derived from the plate orientation convention, NOT the camera's
    calibrated rotation (mosaics are placed by stage motion, not camera frames).
  * Two toggleable selection modes for the current well:
      - "Centre click": one click places the well circle at its nominal radius;
        the circle is draggable to refine.
      - "Edge points": click 3+ points on the well rim → live least-squares
        circle fit (with a ±50% radius sanity check when the nominal diameter
        is known).
  * The operator must CONFIRM each guided well before the next one starts
    (Confirm well / Redo / Skip). Confirmed circles lock (green).
  * Full-plate mode: confirm the 3 corner wells → the rest auto-place from the
    affine fit (or the auto-detector pre-places everything) → per-well refine.
  * Rosette / single-well mode (``rosette=True``): place the sub-well PATTERN
    CENTRE once — all sub-wells auto-place from the rosette design geometry
    (centroid-relative predicted stage-µm offsets, so rotation and plate
    orientation are already folded in) — then drag any sub-well to refine.

Coords are mosaic pixels; px → stage µm uses the stored extent + scale (the
inverse of ``MosaicBuilder`` placement, identical to
``_ploc_fit_from_mosaic_detections``). On Confirm the mosaic + labels are also
saved as a training sample (``WellTrainingStore``).
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass
from enum import Enum, auto

from PySide6.QtCore import Qt, QPointF, QRectF, Signal
from PySide6.QtGui import QColor, QPainter, QPen, QBrush
from PySide6.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QGraphicsScene,
    QGraphicsView, QGraphicsItem, QGraphicsEllipseItem,
    QGraphicsSimpleTextItem, QApplication, QButtonGroup,
)

from gui.styles import COLORS
from gui.scaling import s

logger = logging.getLogger(__name__)

# Radius sanity for the edge-point circle fit — mirrors
# CalibrationPage._PLOC_RADIUS_TOL (reject a fit whose radius deviates > 50%
# from the well's nominal radius; skipped when the nominal is unknown).
_RADIUS_TOL = 0.50

# Sentinel queue entry for the rosette pattern centre (never exported).
_CENTER_KEY = "⊕centre"


class _Phase(Enum):
    IDLE = auto()            # unmappable (plate < 3 wells) / not started
    CORNERS = auto()         # plate mode: guided 3-corner queue
    REFINE = auto()          # plate mode: per-well confirm/refine queue
    ROSETTE_CENTER = auto()  # rosette mode: place the pattern centre
    ROSETTE_REFINE = auto()  # rosette mode: sub-wells placed, drag/confirm


class _SelMode(Enum):
    CENTER = auto()          # one click = centre at nominal radius, draggable
    EDGE = auto()            # 3+ rim clicks → live circle fit


@dataclass
class _Marker:
    """One well's circle on the scene + its guided-flow state."""
    circle: QGraphicsEllipseItem
    label: QGraphicsSimpleTextItem | None
    state: str               # "placed" | "active" | "confirmed" | "skipped"
    r_px: float


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


def main_well_names(plate):
    """Plate-level mapping names: MAIN wells only.

    Flattened rosette SUB-wells ("A1.a"…) are excluded — sub-well centres are
    calibrated in their own Rosettes flow, not on the full-plate mosaic where
    they aren't individually resolvable. The rosette PARENT wells (dropped at
    plate compile) are re-added so the plate map still carries one centre per
    physical well; their nominal position is the sub-well pattern centroid."""
    names: list = []
    parents: list = []
    try:
        for w in plate.get_all_wells():
            if getattr(w, "is_subwell", False):
                p = getattr(w, "parent_well", None)
                if p and p not in parents:
                    parents.append(p)
                continue
            names.append(w.name)
    except Exception:
        try:
            return list(plate.well_names)
        except Exception:
            return []
    for p in parents:
        if p not in names:
            names.append(p)
    return names


def corner_well_names(plate, names=None, positions=None):
    """Three non-collinear corner wells (origin, +X end, +Y end) by nominal
    position extremes — robust to naming / custom plates.

    ``names`` restricts the candidates (e.g. main wells only); ``positions``
    overrides nominal plate-local positions for names the plate can't resolve
    (rosette parents)."""
    names = list(names) if names is not None else list(plate.well_names)
    if len(names) < 3:
        return names
    pos = {}
    for n in names:
        if positions is not None and n in positions:
            pos[n] = positions[n]
            continue
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


# The displayed corner positions are orientation-INVARIANT: on a flipped
# machine A1 sits at stage-max (raw bottom-right) and the 180° view rotation
# shows it top-left; on an unflipped machine A1 is at stage-min = raw top-left.
_CORNER_HINTS = ("top-left", "top-right", "bottom-left")


class _MappingView(QGraphicsView):
    """Mosaic image view: wheel-zoom (about cursor), middle-drag pan, and a
    left-click signal for the guided placement flow.

    v7.5.x: optional 180° VIEW rotation (``set_flip_180``) presents the mosaic
    in the plate orientation. ``mapToScene`` still returns raw scene px, so all
    coordinates are unaffected — only the rendering rotates. While
    ``click_mode`` is on, a left-click on a MOVABLE item passes through to the
    normal drag behaviour (unless ``click_takes_all`` — edge-point mode — is
    set), so guided picking and marker dragging coexist."""

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
        self.click_takes_all = False    # edge mode: never drag, always pick
        self.flip_180 = False
        self._panning = False
        self._pan_start = QPointF(0, 0)
        self._h0 = 0
        self._v0 = 0

    def set_flip_180(self, flip: bool) -> None:
        """Rotate the VIEW 180° (display-only; scene coords unchanged)."""
        flip = bool(flip)
        if flip == self.flip_180:
            return
        self.flip_180 = flip
        self.reset_view_transform()

    def reset_view_transform(self) -> None:
        """Reset zoom to 1:1 and re-apply the orientation rotation."""
        self.resetTransform()
        self._zoom = 1.0
        if self.flip_180:
            self.rotate(180)

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
            # Let a click ON a draggable marker start a drag (centre mode);
            # edge-point mode consumes every click as a rim point.
            if not self.click_takes_all:
                it = self.itemAt(event.position().toPoint())
                if it is not None and bool(
                        it.flags()
                        & QGraphicsItem.GraphicsItemFlag.ItemIsMovable):
                    super().mousePressEvent(event)
                    return
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
    """Guided well mapping on a stitched mosaic (plate + rosette modes)."""

    _STATE_COLORS = {
        "active": "yellow",
        "placed": "blue",
        "confirmed": "green",
        "skipped": "blue",
    }

    def __init__(self, plate, image_bgr, extent_um, mosaic_scale,
                 um_per_px=0.0, plate_key=None,
                 plate_flip_180: bool = False,
                 rosette: bool = False,
                 predicted_um: dict | None = None,
                 center_label: str = "",
                 parent=None):
        super().__init__(parent)
        self._plate = plate
        self._image = image_bgr
        self._extent = tuple(float(v) for v in extent_um) if extent_um else None
        self._scale = float(mosaic_scale) if mosaic_scale else 1.0
        self._um_per_px = float(um_per_px or 0.0)
        self._plate_key = plate_key if plate_key is not None else getattr(
            plate, "format", "plate")
        self._flip_180 = bool(plate_flip_180)
        self._rosette = bool(rosette)
        self._predicted_um = dict(predicted_um or {})
        self._center_label = str(center_label or "")

        # Plate-level mapping uses MAIN wells only (sub-wells belong to the
        # Rosettes flow); rosette parents anchor at their pattern centroid.
        self._map_names = (main_well_names(plate)
                           if (plate is not None and not self._rosette)
                           else (list(getattr(plate, "well_names", []))
                                 if plate is not None else []))
        self._nominal_pos = {n: self._nominal_position(n)
                             for n in self._map_names}
        self._corners = (corner_well_names(
            plate, names=self._map_names, positions=self._nominal_pos)
            if plate is not None else [])
        self._corner_clicks: dict = {}          # well → (cx, cy) scene px
        self._well_items: dict[str, _Marker] = {}
        self._results: dict = {}                # well → (sx_um, sy_um) on confirm
        self._saved_path = None

        # Guided-flow state.
        self._phase = _Phase.IDLE
        self._sel_mode = _SelMode.CENTER
        self._queue: list[str] = []
        self._qidx = 0
        self._edge_pts: list[QPointF] = []      # current well's rim clicks
        self._edge_items: list = []             # cosmetic rim-click dots
        self._fit_item: QGraphicsEllipseItem | None = None
        self._edge_fit: tuple | None = None          # accepted (cx, cy, r) fit
        self._center_marker: _Marker | None = None   # rosette centre (not exported)
        self._center_confirmed = False

        self.setWindowTitle("Map Wells on Mosaic")
        self.setModal(True)
        self._build_ui()
        self._load_image()
        if self._flip_180:
            self._view.set_flip_180(True)
            self._fit()
        if self._rosette:
            self._begin_rosette_flow()
        else:
            self._begin_plate_flow()
            # Try automatic detection immediately; if it can't lock the grid,
            # the guided 3-corner workflow (already armed) stays as the
            # fallback.
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

        # Selection-mode toggle + per-well confirm row.
        wrow = QHBoxLayout()
        mode_lbl = QLabel("Select by:")
        mode_lbl.setStyleSheet(f"color: {COLORS['subtext0']};")
        wrow.addWidget(mode_lbl)
        self._btn_mode_center = QPushButton("Centre click")
        self._btn_mode_center.setCheckable(True)
        self._btn_mode_center.setChecked(True)
        self._btn_mode_center.setToolTip(
            "One click places the well circle at its centre (nominal radius); "
            "drag the circle to refine.")
        self._btn_mode_edge = QPushButton("Edge points (3+)")
        self._btn_mode_edge.setCheckable(True)
        self._btn_mode_edge.setToolTip(
            "Click 3 or more points ON the well's rim — the circle is fitted "
            "through them (least squares). Extra clicks refine the fit.")
        self._mode_group = QButtonGroup(self)
        self._mode_group.setExclusive(True)
        self._mode_group.addButton(self._btn_mode_center)
        self._mode_group.addButton(self._btn_mode_edge)
        self._btn_mode_center.toggled.connect(self._on_mode_toggled)
        wrow.addWidget(self._btn_mode_center)
        wrow.addWidget(self._btn_mode_edge)
        self._btn_clear_pts = QPushButton("Clear points")
        self._btn_clear_pts.setToolTip("Discard the rim points clicked so far.")
        self._btn_clear_pts.clicked.connect(self._clear_edge_state)
        self._btn_clear_pts.setEnabled(False)
        wrow.addWidget(self._btn_clear_pts)
        wrow.addStretch()
        self._btn_confirm_well = QPushButton("Confirm well")
        self._btn_confirm_well.setObjectName("accentBtn")
        self._btn_confirm_well.setToolTip(
            "Lock the current well's position and move to the next.")
        self._btn_confirm_well.setEnabled(False)
        self._btn_confirm_well.clicked.connect(self._on_confirm_well)
        wrow.addWidget(self._btn_confirm_well)
        self._btn_redo = QPushButton("Redo")
        self._btn_redo.setToolTip("Discard the current well's placement and "
                                  "pick it again.")
        self._btn_redo.setEnabled(False)
        self._btn_redo.clicked.connect(self._on_redo_well)
        wrow.addWidget(self._btn_redo)
        self._btn_skip = QPushButton("Skip")
        self._btn_skip.setToolTip(
            "Leave this well at its auto-placed position and move on.")
        self._btn_skip.setEnabled(False)
        self._btn_skip.clicked.connect(self._on_skip_well)
        wrow.addWidget(self._btn_skip)
        root.addLayout(wrow)

        row = QHBoxLayout()
        self._btn_auto = QPushButton("Auto-detect wells")
        self._btn_auto.setToolTip(
            "Find all wells automatically (filled-disc detection + grid fit). "
            "Then confirm/drag each to refine. Falls back to the guided "
            "3-corner method.")
        self._btn_auto.clicked.connect(self._auto_detect)
        if self._rosette:
            self._btn_auto.setVisible(False)
        row.addWidget(self._btn_auto)
        self._btn_recorners = QPushButton("Restart mapping")
        self._btn_recorners.clicked.connect(self._restart_mapping)
        row.addWidget(self._btn_recorners)
        self._btn_fit = QPushButton("Fit view")
        self._btn_fit.clicked.connect(self._fit)
        row.addWidget(self._btn_fit)
        row.addStretch()
        self._btn_confirm = QPushButton("Confirm all + calibrate + save")
        self._btn_confirm.setObjectName("accentBtn")
        self._btn_confirm.setEnabled(False)
        self._btn_confirm.clicked.connect(self._on_confirm)
        row.addWidget(self._btn_confirm)
        self._btn_close = QPushButton("Cancel")
        self._btn_close.clicked.connect(self.reject)
        row.addWidget(self._btn_close)
        root.addLayout(row)

        self.resize(s(900), s(720))
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
            # Re-apply the orientation rotation, then fit (fitInView scales
            # relative to the current transform, preserving the rotation).
            self._view.reset_view_transform()
            self._view.fitInView(br, Qt.AspectRatioMode.KeepAspectRatio)

    # ── Guided flow: start / restart ───────────────────────────────

    def _restart_mapping(self):
        if self._rosette:
            self._begin_rosette_flow()
        else:
            self._begin_plate_flow()

    def _begin_plate_flow(self):
        """Guided plate mapping: confirm the 3 corner wells → autofill →
        per-well refine."""
        self._clear_all_markers()
        self._corner_clicks = {}
        self._results = {}
        self._btn_confirm.setEnabled(False)
        self._view.click_mode = True
        if len(self._corners) < 3:
            self._phase = _Phase.IDLE
            self._instr.setText("This plate has fewer than 3 wells — cannot map.")
            self._view.click_mode = False
            self._refresh_well_row()
            return
        self._phase = _Phase.CORNERS
        self._queue = list(self._corners)
        self._qidx = 0
        self._update_banner()
        self._refresh_well_row()

    def _begin_rosette_flow(self):
        """Rosette / single-well mapping: place the sub-well pattern centre,
        auto-place the sub-wells from the design geometry, then refine."""
        self._clear_all_markers()
        self._results = {}
        self._btn_confirm.setEnabled(False)
        self._center_confirmed = False
        self._view.click_mode = True
        self._phase = _Phase.ROSETTE_CENTER
        self._queue = [_CENTER_KEY]
        self._qidx = 0
        self._update_banner()
        self._refresh_well_row()

    def _clear_all_markers(self):
        for m in self._well_items.values():
            self._scene.removeItem(m.circle)
            if m.label is not None:
                self._scene.removeItem(m.label)
        self._well_items = {}
        if self._center_marker is not None:
            self._scene.removeItem(self._center_marker.circle)
            if self._center_marker.label is not None:
                self._scene.removeItem(self._center_marker.label)
            self._center_marker = None
        self._clear_edge_state()

    # ── Queue helpers ──────────────────────────────────────────────

    def _current_name(self):
        if 0 <= self._qidx < len(self._queue):
            return self._queue[self._qidx]
        return None

    def _advance_queue(self):
        self._qidx += 1
        while self._qidx < len(self._queue):
            m = self._marker_for(self._queue[self._qidx])
            if m is None or m.state != "confirmed":
                break
            self._qidx += 1
        self._clear_edge_state()
        nxt = self._current_name()
        if nxt is not None:
            m = self._marker_for(nxt)
            if m is not None:
                try:
                    self._view.centerOn(self._marker_center(m))
                except Exception:
                    pass
        self._update_banner()
        self._refresh_well_row()

    def _marker_for(self, name):
        if name == _CENTER_KEY:
            return self._center_marker
        return self._well_items.get(name)

    def _confirmed_count(self) -> int:
        return sum(1 for m in self._well_items.values()
                   if m.state == "confirmed")

    # ── Banner + button state ──────────────────────────────────────

    def _mode_hint(self) -> str:
        if self._sel_mode == _SelMode.EDGE:
            return "click 3+ points ON its rim"
        return "click its centre (drag to refine)"

    def _update_banner(self):
        name = self._current_name()
        total = len(self._well_items)
        done = self._confirmed_count()
        if self._phase == _Phase.IDLE:
            return
        if self._phase == _Phase.CORNERS:
            hint = _CORNER_HINTS[min(self._qidx, 2)]
            self._instr.setText(
                f"Corner {self._qidx + 1}/3: «{name}» — {hint} in this view. "
                f"{self._mode_hint().capitalize()}, then Confirm well.  "
                f"(wheel = zoom · middle-drag = pan)")
        elif self._phase == _Phase.ROSETTE_CENTER:
            lbl = self._center_label or "sub-well pattern"
            self._instr.setText(
                f"Click the CENTRE of the «{lbl}» sub-well pattern — "
                f"{self._mode_hint()} — then Confirm well. The sub-wells will "
                f"auto-place around it.")
        elif name is None:
            self._instr.setText(
                f"All wells visited ({done}/{total} confirmed). Drag any "
                f"unlocked circle to refine, then Confirm all.")
        elif self._phase == _Phase.ROSETTE_REFINE:
            self._instr.setText(
                f"Refine sub-well «{name}» ({done}/{total} confirmed) — drag "
                f"its circle or click to re-place. Confirm well locks it; "
                f"Skip keeps the auto placement.")
        else:   # REFINE
            self._instr.setText(
                f"Refine «{name}» ({done}/{total} confirmed) — drag its "
                f"circle, click to re-place, or Skip. Confirm well locks it. "
                f"Confirm all finishes at any time.")

    def _refresh_well_row(self):
        name = self._current_name()
        has_current = name is not None and self._phase != _Phase.IDLE
        marker = self._marker_for(name) if has_current else None
        if self._sel_mode == _SelMode.EDGE:
            can_confirm = has_current and self._edge_fit is not None
        else:
            can_confirm = has_current and marker is not None
        self._btn_confirm_well.setEnabled(bool(can_confirm))
        self._btn_redo.setEnabled(bool(has_current and marker is not None))
        self._btn_skip.setEnabled(
            bool(has_current
                 and self._phase in (_Phase.REFINE, _Phase.ROSETTE_REFINE)))
        self._btn_clear_pts.setEnabled(
            self._sel_mode == _SelMode.EDGE and bool(self._edge_pts))

    def _on_mode_toggled(self, _checked: bool):
        self._sel_mode = (_SelMode.CENTER if self._btn_mode_center.isChecked()
                          else _SelMode.EDGE)
        # Edge mode consumes every click (no drag-through); centre mode lets a
        # click on a marker start a drag.
        self._view.click_takes_all = (self._sel_mode == _SelMode.EDGE)
        self._clear_edge_state()
        self._update_banner()
        self._refresh_well_row()

    # ── Markers ────────────────────────────────────────────────────

    def _marker_center(self, m: _Marker) -> QPointF:
        c = m.circle.rect().center()
        return QPointF(c.x() + m.circle.pos().x(), c.y() + m.circle.pos().y())

    def _label_anchor(self, cx: float, cy: float, r: float) -> tuple:
        """Anchor so the (transform-ignoring) text sits top-right of the
        circle in VIEW coords, under either orientation."""
        if self._flip_180:
            return (cx - r, cy + r)
        return (cx + r, cy - r)

    def _remove_marker(self, name: str):
        m = self._marker_for(name)
        if m is None:
            return
        self._scene.removeItem(m.circle)
        if m.label is not None:
            self._scene.removeItem(m.label)
        if name == _CENTER_KEY:
            self._center_marker = None
        else:
            self._well_items.pop(name, None)

    def _place_marker(self, name: str, cx: float, cy: float,
                      r: float | None = None, state: str = "active"):
        """Create/replace a marker circle for ``name`` at scene (cx, cy)."""
        self._remove_marker(name)
        if r is None or r <= 0:
            r = self._well_radius_px(name)
        color = QColor(COLORS[self._STATE_COLORS.get(state, "blue")])
        pen = QPen(color)
        pen.setCosmetic(True)
        pen.setWidthF(2.0 if state in ("active", "confirmed") else 1.6)
        circle = QGraphicsEllipseItem(QRectF(cx - r, cy - r, 2 * r, 2 * r))
        circle.setPen(pen)
        circle.setBrush(QBrush(QColor(0, 0, 0, 0)))
        movable = state != "confirmed"
        circle.setFlag(QGraphicsEllipseItem.GraphicsItemFlag.ItemIsMovable,
                       movable)
        circle.setFlag(QGraphicsEllipseItem.GraphicsItemFlag.ItemIsSelectable,
                       movable)
        circle.setZValue(2)
        circle.setToolTip(name)
        self._scene.addItem(circle)
        lbl_text = self._center_label if name == _CENTER_KEY else name
        label = self._scene.addSimpleText(lbl_text)
        label.setBrush(QBrush(color))
        ax, ay = self._label_anchor(cx, cy, r)
        label.setPos(ax, ay)
        label.setFlag(QGraphicsSimpleTextItem.GraphicsItemFlag
                      .ItemIgnoresTransformations, True)
        label.setZValue(3)
        marker = _Marker(circle=circle, label=label, state=state, r_px=r)
        if name == _CENTER_KEY:
            self._center_marker = marker
        else:
            self._well_items[name] = marker
        return marker

    def _set_marker_state(self, name: str, state: str):
        m = self._marker_for(name)
        if m is None:
            return
        m.state = state
        color = QColor(COLORS[self._STATE_COLORS.get(state, "blue")])
        pen = m.circle.pen()
        pen.setColor(color)
        pen.setWidthF(2.0 if state in ("active", "confirmed") else 1.6)
        m.circle.setPen(pen)
        movable = state != "confirmed"
        m.circle.setFlag(QGraphicsEllipseItem.GraphicsItemFlag.ItemIsMovable,
                         movable)
        m.circle.setFlag(QGraphicsEllipseItem.GraphicsItemFlag
                         .ItemIsSelectable, movable)
        if m.label is not None:
            m.label.setBrush(QBrush(color))

    # ── Click dispatch + edge-point fit ────────────────────────────

    def _on_view_clicked(self, scene_pt: QPointF):
        if self._phase == _Phase.IDLE:
            return
        name = self._current_name()
        if name is None:
            return                          # queue exhausted → drag only
        if self._sel_mode == _SelMode.EDGE:
            self._edge_pts.append(scene_pt)
            dot_r = 2.5
            pen = QPen(QColor(COLORS["peach"]))
            pen.setCosmetic(True)
            pen.setWidthF(2.0)
            dot = self._scene.addEllipse(
                QRectF(scene_pt.x() - dot_r, scene_pt.y() - dot_r,
                       2 * dot_r, 2 * dot_r), pen)
            dot.setZValue(4)
            self._edge_items.append(dot)
            self._update_fit_preview()
        else:
            r = None
            if name == _CENTER_KEY:
                r = self._center_radius_px()
            self._place_marker(name, scene_pt.x(), scene_pt.y(), r,
                               state="active")
        self._refresh_well_row()

    def _clear_edge_state(self):
        self._edge_pts = []
        for it in self._edge_items:
            try:
                self._scene.removeItem(it)
            except Exception:
                pass
        self._edge_items = []
        if self._fit_item is not None:
            try:
                self._scene.removeItem(self._fit_item)
            except Exception:
                pass
            self._fit_item = None
        self._edge_fit = None
        if hasattr(self, "_btn_clear_pts"):
            self._refresh_well_row()

    def _main_well_diameter_mm(self) -> float:
        """The plate's NORMAL (main-well) diameter in mm: the median
        diameter of the non-sub wells, else the plate's uniform
        ``well_diameter`` attribute (custom plates store 0.0 = "varies"),
        else 0.0 (unknown)."""
        ds = []
        try:
            for w in self._plate.get_all_wells():
                if getattr(w, "is_subwell", False):
                    continue
                d = float(getattr(w, "diameter", 0.0) or 0.0)
                if d > 0:
                    ds.append(d)
        except Exception:
            pass
        if ds:
            ds.sort()
            return ds[len(ds) // 2]
        try:
            return float(getattr(self._plate, "well_diameter", 0.0) or 0.0)
        except Exception:
            return 0.0

    def _fit_diameter_mm(self, name) -> float:
        """Diameter (mm) to FIT ``name`` with. A rosette PARENT (dropped at
        plate compile → no WellInfo) always resolves to the plate's NORMAL
        well diameter — never rosette/sub-well geometry (operator rule: the
        physical feature being fitted is the plate's standard well the
        rosette insert sits in). Sub-well names (Rosettes-tab mode) keep
        their own per-well diameters."""
        try:
            d = float(self._plate.get_well_info(name).diameter)
            if d > 0:
                return d
        except Exception:
            pass
        return self._main_well_diameter_mm()

    def _nominal_radius_px(self, name) -> float:
        """Nominal well radius in mosaic px for the SANITY check — 0.0 when
        unknown (no 3 mm fallback), which skips the check (mirrors the
        click-rim flow's r_exp > 0 gate)."""
        if name is None or name == _CENTER_KEY:
            return 0.0
        d_mm = self._fit_diameter_mm(name)
        if d_mm <= 0:
            return 0.0
        return (d_mm * 1000.0 / 2.0) * self._scale

    def _update_fit_preview(self):
        """Live circle fit through the clicked rim points (3+ needed)."""
        if self._fit_item is not None:
            try:
                self._scene.removeItem(self._fit_item)
            except Exception:
                pass
            self._fit_item = None
        self._edge_fit = None
        n = len(self._edge_pts)
        if n < 3:
            self._status.setText(f"{n}/3+ rim points — keep clicking the rim.")
            return
        try:
            from SupportClasses.VisionDetector import fit_circle_to_points
            fit = fit_circle_to_points(
                [(p.x(), p.y()) for p in self._edge_pts])
        except Exception as e:
            logger.debug(f"circle fit failed: {e}")
            fit = None
        if fit is None:
            self._status.setText(
                "The rim points are collinear — spread the clicks around the "
                "circle (or Clear points).")
            return
        cx, cy, r = fit
        r_exp = self._nominal_radius_px(self._current_name())
        ok = r_exp <= 0 or abs(r - r_exp) / r_exp <= _RADIUS_TOL
        color = QColor(COLORS["yellow" if ok else "red"])
        pen = QPen(color)
        pen.setCosmetic(True)
        pen.setWidthF(2.0)
        if not ok:
            pen.setStyle(Qt.PenStyle.DashLine)
        self._fit_item = self._scene.addEllipse(
            QRectF(cx - r, cy - r, 2 * r, 2 * r), pen)
        self._fit_item.setZValue(3)
        if ok:
            self._edge_fit = (float(cx), float(cy), float(r))
            self._status.setText(
                f"Fitted circle from {n} points (r = {r:.0f} px) — Confirm "
                f"well, or add more points to refine.")
        else:
            self._status.setText(
                f"Fit radius {r:.0f} px vs expected {r_exp:.0f} px — looks "
                f"wrong. Add better rim points or Clear points.")

    def _center_radius_px(self) -> float:
        """Display radius for the rosette pattern-centre marker: half the
        sub-well pattern span when known, else a small default."""
        offs = self._subwell_offsets_px()
        if offs:
            span = max(math.hypot(dx, dy) for dx, dy in offs.values())
            sub_r = 0.0
            try:
                names = list(self._plate.well_names)
                if names:
                    sub_r = self._nominal_radius_px(names[0])
            except Exception:
                pass
            return max(10.0, span + sub_r)
        return 25.0

    # ── Per-well confirm / redo / skip ─────────────────────────────

    def _on_confirm_well(self):
        name = self._current_name()
        if name is None:
            return
        # Edge mode: commit the fitted circle as the marker.
        if self._sel_mode == _SelMode.EDGE:
            if self._edge_fit is None:
                return
            cx, cy, r = self._edge_fit
            self._place_marker(name, cx, cy, r, state="active")
            self._clear_edge_state()
        marker = self._marker_for(name)
        if marker is None:
            return
        self._set_marker_state(name, "confirmed")
        c = self._marker_center(marker)
        if self._phase == _Phase.CORNERS:
            self._corner_clicks[name] = (c.x(), c.y())
            if len(self._corner_clicks) >= 3:
                self._autofill()
                return
            self._advance_queue()
            return
        if self._phase == _Phase.ROSETTE_CENTER:
            self._center_confirmed = True
            self._rosette_place_subwells(c)
            return
        # REFINE / ROSETTE_REFINE
        self._advance_queue()

    def _on_redo_well(self):
        name = self._current_name()
        if name is None:
            return
        self._remove_marker(name)
        if name == _CENTER_KEY:
            self._center_confirmed = False
        self._corner_clicks.pop(name, None)
        self._clear_edge_state()
        self._update_banner()
        self._refresh_well_row()

    def _on_skip_well(self):
        if self._phase not in (_Phase.REFINE, _Phase.ROSETTE_REFINE):
            return
        name = self._current_name()
        if name is None:
            return
        m = self._marker_for(name)
        if m is not None and m.state != "confirmed":
            self._set_marker_state(name, "skipped")
        self._advance_queue()

    # ── Auto-fill + auto-detect (plate mode) ───────────────────────

    def _nominal_position(self, name):
        """Plate-local nominal (x, y) mm for a mapping name. A rosette PARENT
        (dropped at plate compile) resolves to the centroid of its sub-wells;
        unresolvable names fall back to (0, 0)."""
        try:
            return self._plate.get_well_position(name)
        except Exception:
            pass
        try:
            subs = [(w.x, w.y) for w in self._plate.get_all_wells()
                    if getattr(w, "is_subwell", False)
                    and getattr(w, "parent_well", None) == name]
            if subs:
                return (sum(p[0] for p in subs) / len(subs),
                        sum(p[1] for p in subs) / len(subs))
        except Exception:
            pass
        return (0.0, 0.0)

    def _well_radius_px(self, name) -> float:
        d_mm = self._fit_diameter_mm(name)
        if d_mm <= 0:
            d_mm = 3.0
        r = (d_mm * 1000.0 / 2.0) * self._scale
        return max(3.0, r)

    def _autofill(self):
        """After the 3 corners are confirmed: place every remaining well from
        the affine fit through the corner clicks, then enter the per-well
        refine queue."""
        nom = []
        tgt = []
        for w in self._corners:
            nom.append(self._nominal_pos.get(w) or self._nominal_position(w))
            tgt.append(self._corner_clicks.get(w, (0.0, 0.0)))
        aff = solve_affine_3(nom, tgt)
        if aff is None:
            self._begin_plate_flow()
            self._status.setText(
                "The 3 corner picks are collinear — re-place them at true "
                "corners.")
            return
        for name in self._map_names:            # MAIN wells only (no sub-wells)
            if name in self._well_items:        # corners keep their confirms
                continue
            nx, ny = self._nominal_pos.get(name, (0.0, 0.0))
            px, py = apply_affine(aff, nx, ny)
            self._place_marker(name, px, py, state="placed")
        self._phase = _Phase.REFINE
        self._queue = [n for n in self._map_names
                       if n not in self._corner_clicks]
        self._qidx = 0
        self._btn_confirm.setEnabled(True)
        self._status.setText(
            f"Auto-placed {len(self._well_items)} wells from the corner fit.")
        self._update_banner()
        self._refresh_well_row()

    def _auto_detect(self):
        """Find ALL wells automatically (filled-disc detection + grid fit),
        pre-place each at its detected centre — or the grid prediction for any
        well that wasn't directly detected — then enter the per-well refine
        queue. Leaves the guided corner workflow intact if detection can't
        lock the grid."""
        import numpy as np
        if self._rosette:
            return
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
        self._clear_all_markers()
        self._corner_clicks = {}
        grid_names = {}
        try:
            for w in self._plate.get_all_wells():
                if getattr(w, "is_subwell", False):
                    # Sub-wells share the parent's (row, col) — the grid cell
                    # belongs to the MAIN well; the letter-fallback below
                    # produces the parent name for a dropped rosette parent.
                    continue
                grid_names[(w.row, w.col)] = w.name
        except Exception:
            pass
        A, t = aff[:, :2], aff[:, 2]
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
                self._place_marker(name, cx, cy, state="placed")
        self._phase = _Phase.REFINE
        self._queue = [n for n in self._map_names if n in self._well_items]
        self._qidx = 0
        self._btn_confirm.setEnabled(True)
        n_fill = len(self._well_items) - n_det
        self._status.setText(
            f"Auto-detected {n_det} wells"
            + (f" (+{n_fill} filled from grid)" if n_fill else "")
            + " — confirm/drag each, or Confirm all now.")
        self._update_banner()
        self._refresh_well_row()

    # ── Rosette sub-well placement ─────────────────────────────────

    def _subwell_offsets_px(self) -> dict | None:
        """Centroid-relative sub-well offsets in mosaic px.

        Prefers the predicted STAGE-µm positions (rosette rotation and the
        plate orientation sign are already baked in; mosaic px axes align with
        stage axes, so px = µm × scale). Falls back to plate-local mm offsets
        with the orientation sign derived from the flip flag."""
        names = list(getattr(self._plate, "well_names", []) or [])
        if not names:
            return None
        pts = {n: self._predicted_um[n] for n in names
               if n in self._predicted_um}
        if len(pts) == len(names):
            cx = sum(p[0] for p in pts.values()) / len(pts)
            cy = sum(p[1] for p in pts.values()) / len(pts)
            return {n: ((x - cx) * self._scale, (y - cy) * self._scale)
                    for n, (x, y) in pts.items()}
        # Fallback: plate-local mm → µm × orientation sign.
        try:
            loc = {n: self._plate.get_well_position(n) for n in names}
        except Exception:
            return None
        if not loc:
            return None
        sx = sy = -1.0 if self._flip_180 else 1.0
        cx = sum(p[0] for p in loc.values()) / len(loc)
        cy = sum(p[1] for p in loc.values()) / len(loc)
        return {n: ((x - cx) * 1000.0 * sx * self._scale,
                    (y - cy) * 1000.0 * sy * self._scale)
                for n, (x, y) in loc.items()}

    def _rosette_place_subwells(self, center: QPointF):
        """Place every sub-well marker around the confirmed pattern centre."""
        offs = self._subwell_offsets_px()
        names = list(getattr(self._plate, "well_names", []) or [])
        if offs:
            for name in names:
                dx, dy = offs.get(name, (0.0, 0.0))
                self._place_marker(name, center.x() + dx, center.y() + dy,
                                   state="placed")
            self._status.setText(
                f"Placed {len(names)} sub-wells from the rosette geometry — "
                f"drag any to refine.")
        else:
            # No geometry available → guided manual placement of each.
            self._status.setText(
                "No sub-well geometry available — place each sub-well "
                "manually.")
        self._phase = _Phase.ROSETTE_REFINE
        self._queue = list(names)
        self._qidx = 0
        self._btn_confirm.setEnabled(True)
        self._update_banner()
        self._refresh_well_row()

    # ── Confirm / export ───────────────────────────────────────────

    def _on_confirm(self):
        if not self._well_items or self._extent is None or not self._scale:
            self._status.setText("Place the wells first.")
            return
        if self._rosette:
            if not self._center_confirmed:
                self._status.setText("Confirm the pattern centre first.")
                return
        elif len(self._well_items) < 3:
            self._status.setText("At least 3 wells are needed to calibrate.")
            return
        ox, oy = self._extent[0], self._extent[1]
        results: dict = {}
        wells_label: dict = {}
        for name, m in self._well_items.items():
            c = self._marker_center(m)
            sx = ox + c.x() / self._scale       # mosaic px → absolute stage µm
            sy = oy + c.y() / self._scale
            results[name] = (sx, sy)
            wells_label[name] = {
                "px": (c.x(), c.y()), "um": (sx, sy),
                "r_px": m.r_px}
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

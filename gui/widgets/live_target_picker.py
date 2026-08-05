"""live_target_picker.py — v7.4.x live-view target-picking tool.

The first composable tool in the new Workflows system. ONE live
microscope feed feeds TWO paired target lists:

    - Pick targets   (P001, P002, …):  extraction locations  (green)
    - Place targets  (D001, D002, …):  destination locations (mauve)

Pick i is paired with Place i by index: spheroid picked at P001 is
deposited at D001, P002 → D002, and so on. A connector line is drawn
between each paired pick/place on the camera overlay.

The user toggles which list the next click goes into via a Mode
selector. Both overlays draw on the same camera view at the same time.

Camera→stage transform pipeline (no new math is written here —
existing helpers are composed):

    widget click (QPointF)
      → CameraFeedView._widget_to_image()      (letterbox unwrap)
      → CameraManager.pixel_to_stage_offset()  (frame → stage µm offset)
      → + current StageController.get_xy_position()  (stage center)
      = absolute stage µm

Sub-pixel precision: `_widget_to_image()` returns floats; all
subsequent conversions stay float.
"""

from __future__ import annotations

import logging
from typing import Optional

from PySide6.QtCore import Qt, QEvent, QPointF, QTimer, Signal
from PySide6.QtGui import QBrush, QColor, QFont, QPainter, QPen, QPixmap
from PySide6.QtWidgets import (
    QAbstractItemView, QButtonGroup, QCheckBox, QDialog, QDialogButtonBox,
    QDoubleSpinBox, QFrame, QFormLayout, QHBoxLayout, QLabel, QLineEdit,
    QListWidget, QListWidgetItem, QPushButton, QRadioButton, QSizePolicy,
    QSplitter, QVBoxLayout, QWidget,
)

from gui.styles import COLORS
from gui.scaling import s, sf
from gui.widgets.target_overlay_camera_view import TargetOverlayCameraView

from SupportClasses.HardwareConfig import CameraRole
from SupportClasses.PickAndPlaceManager import PickPlaceTarget

logger = logging.getLogger(__name__)


_REMOVE_RADIUS_UM = 250.0  # right-click radius to remove a target

# Grab tolerance for the resize/move handles, in WIDGET pixels — the same
# resolution-independent convention MeasurementCameraView uses. A tolerance in
# frame pixels would be a 3-px grab zone at 4K and a 40-px one at VGA.
_SNAP_RADIUS_WIDGET_PX = 12

# Provenance of a target's POSITION. A "mosaic" position came from a stitched
# mosaic and is a search hint (see SpheroidDetector: the registration shift is
# bounded by 20% of a FOV); a "live" or "confirmed" position was derived from
# this frame through pixel_to_stage_offset and is stage-referenced.
PROV_LIVE = "live"
PROV_MOSAIC = "mosaic"
PROV_CONFIRMED = "confirmed"


# ── camera view with paired pick/place overlays + right-click ──────

class _PickerCameraView(TargetOverlayCameraView):
    """Camera view that renders two target lists (pick + place) with a
    connector line between paired entries, and emits `right_clicked`.

    Picks render green, places mauve. Pair i (pick[i] ↔ place[i]) is
    joined by a dashed connector. Overrides `_draw_targets` so the
    base's single-`selected_target` colouring isn't used.

    v7.8 — measure mode. When enabled the operator can size a target's circle
    on the live feed, by either gesture:

      * drag its RING outward/inward to set the diameter, or
      * click 3+ points on the spheroid's rim and let a circle be fitted.

    Both are gated on measure mode, so with it off every left click still simply
    adds a target — which is what protects the legacy workflow and the
    ``pick_only`` consumers (Cell Labeling).

    Coordinates: ``_widget_to_image`` gives RAW frame pixels (this branch of the
    hierarchy bypasses ``_orient_qimage``, so there is no view transform to
    invert), the model is in µm, and hit-testing is in WIDGET pixels. The
    µm↔px projection is FROZEN for the duration of a drag: the host polls the
    stage at 5 Hz and calls ``set_stage_position``, so a drag spanning a stage
    settle would otherwise silently resize the circle.
    """

    right_clicked = Signal(float, float)
    # (target_id, new_diameter_um) while dragging a ring, and on release.
    size_dragged = Signal(str, float)
    size_committed = Signal(str, float)
    # (target_id, x_um, y_um) — a target's centre was dragged.
    center_dragged = Signal(str, float, float)
    center_committed = Signal(str, float, float)
    # A rim-point fit completed: (x_um, y_um, diameter_um).
    rim_fitted = Signal(float, float, float)

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._pick_targets: list[PickPlaceTarget] = []
        self._place_targets: list[PickPlaceTarget] = []
        self._provenance: dict[str, str] = {}
        self._measure_mode = False
        self._rim_mode = False
        self._rim_points: list[tuple[float, float]] = []
        self._rim_fit: Optional[tuple[float, float, float]] = None
        # Drag state. ``_drag_frozen`` is the (stage_x, stage_y, um_per_px)
        # snapshot taken at press; see the class docstring.
        self._drag_kind: Optional[str] = None       # "size" | "center"
        self._drag_id: str = ""
        self._drag_frozen: Optional[tuple[float, float, float]] = None

    def set_pick_targets(self, targets: list[PickPlaceTarget]):
        self._pick_targets = list(targets)

    def set_place_targets(self, targets: list[PickPlaceTarget]):
        self._place_targets = list(targets)

    def set_provenance(self, provenance: dict):
        """``target_id → PROV_*``; drives the dashed "unconfirmed" ring."""
        self._provenance = dict(provenance or {})

    def set_measure_mode(self, on: bool):
        on = bool(on)
        if on == self._measure_mode:
            return
        self._measure_mode = on
        if not on:
            self._rim_mode = False
            self.clear_rim_points()
        self._end_drag()
        self._rerender_last()

    def measure_mode(self) -> bool:
        return self._measure_mode

    def set_rim_mode(self, on: bool):
        """Rim-point sub-mode: clicks place rim points instead of targets."""
        self._rim_mode = bool(on) and self._measure_mode
        if not self._rim_mode:
            self.clear_rim_points()
        self._rerender_last()

    def rim_mode(self) -> bool:
        return self._rim_mode

    def clear_rim_points(self):
        self._rim_points = []
        self._rim_fit = None
        self._rerender_last()

    def rim_fit(self):
        """The current rim fit in stage µm ``(x, y, diameter)``, or None."""
        return self._rim_fit

    def is_dragging(self) -> bool:
        return self._drag_kind is not None

    # ── projection ────────────────────────────────────────────────

    def _projection(self) -> tuple[float, float, float]:
        """``(stage_x_um, stage_y_um, um_per_px)`` — frozen during a drag."""
        if self._drag_frozen is not None:
            return self._drag_frozen
        return (self._stage_x_um, self._stage_y_um, self._um_per_px)

    def _um_to_image_px(self, x_um: float, y_um: float,
                        img_w: int, img_h: int) -> QPointF:
        """Stage µm → raw frame pixel, through the calibrated camera→stage map.

        Uses ``CameraManager.stage_offset_to_pixel``, the documented exact
        inverse of the click path's ``pixel_to_stage_offset``. Falls back to the
        identity mapping only when no manager is available (test doubles) — the
        identity is what this used to do unconditionally, and it mis-placed every
        marker on a rotated or mirrored camera.
        """
        sx, sy, eff = self._projection()
        dx_um = float(x_um) - sx
        dy_um = float(y_um) - sy
        mgr = self._manager
        inv = getattr(mgr, "stage_offset_to_pixel", None) if mgr else None
        if callable(inv):
            try:
                px, py = inv(self._cam_idx, dx_um, dy_um, img_w, img_h)
                return QPointF(px, py)
            except Exception:
                pass
        if not eff:
            return QPointF(img_w / 2.0, img_h / 2.0)
        return QPointF(dx_um / eff + img_w / 2.0, dy_um / eff + img_h / 2.0)

    def _image_to_widget(self, ix: float, iy: float):
        """Raw frame pixel → widget pixel — exact inverse of `_widget_to_image`."""
        pm = self._last_pixmap
        if pm is None or pm.isNull():
            return None
        img_w, img_h = (self._displayed_image_size
                        if self._displayed_image_size[0]
                        else self._last_image_size)
        if not img_w or not img_h:
            return None
        scale = pm.width() / float(img_w)
        ox = (self._display.width() - pm.width()) / 2.0
        oy = (self._display.height() - pm.height()) / 2.0
        return (ix * scale + ox, iy * scale + oy)

    def _widget_scale(self) -> float:
        """Widget px per raw frame px (for converting a grab tolerance)."""
        pm = self._last_pixmap
        if pm is None or pm.isNull():
            return 1.0
        img_w = (self._displayed_image_size[0]
                 if self._displayed_image_size[0] else self._last_image_size[0])
        if not img_w:
            return 1.0
        return pm.width() / float(img_w)

    def _radius_image_px(self, t: PickPlaceTarget) -> float:
        _sx, _sy, eff = self._projection()
        if getattr(t, "size_um", 0.0) and eff:
            return max(6.0, (float(t.size_um) / 2.0) / eff)
        return 8.0

    def _all_targets(self):
        return list(self._pick_targets) + list(self._place_targets)

    def _target_by_id(self, tid: str):
        for t in self._all_targets():
            if t.target_id == tid:
                return t
        return None

    # ── hit tests (widget space) ──────────────────────────────────

    def _hit(self, wx: float, wy: float):
        """``(kind, target)`` under a widget point: "size" on a ring, "center"
        near a centre, else ``(None, None)``.

        Ring BEFORE centre: on a small spheroid the two overlap, and resizing a
        40 µm circle must not be hijacked into a move.
        """
        img_w, img_h = self._last_image_size
        if not img_w or not img_h:
            return (None, None)
        tol = float(_SNAP_RADIUS_WIDGET_PX)
        best_ring = None
        best_ring_d = tol
        best_centre = None
        best_centre_d = None
        for t in self._all_targets():
            pos = self._um_to_image_px(t.x_um, t.y_um, img_w, img_h)
            w = self._image_to_widget(pos.x(), pos.y())
            if w is None:
                continue
            r_w = self._radius_image_px(t) * self._widget_scale()
            d = ((wx - w[0]) ** 2 + (wy - w[1]) ** 2) ** 0.5
            ring_err = abs(d - r_w)
            if ring_err <= best_ring_d:
                best_ring_d = ring_err
                best_ring = t
            centre_tol = max(tol * 0.5, r_w * 0.5)
            if d <= centre_tol and (best_centre_d is None or d < best_centre_d):
                best_centre_d = d
                best_centre = t
        if best_ring is not None:
            return ("size", best_ring)
        if best_centre is not None:
            return ("center", best_centre)
        return (None, None)

    # ── mouse ─────────────────────────────────────────────────────

    def eventFilter(self, obj, event):
        if obj is not self._display or not self._last_pixmap:
            return super().eventFilter(obj, event)
        etype = event.type()

        # Right-click keeps its single existing meaning (remove nearest).
        if (etype == QEvent.Type.MouseButtonPress
                and event.button() == Qt.RightButton):
            img = self._widget_to_image(
                event.position().x(), event.position().y())
            if img is not None:
                self.right_clicked.emit(img[0], img[1])
                return True
            return super().eventFilter(obj, event)

        if not self._measure_mode:
            return super().eventFilter(obj, event)

        if (etype == QEvent.Type.MouseButtonPress
                and event.button() == Qt.LeftButton):
            wx, wy = event.position().x(), event.position().y()
            if self._rim_mode:
                return self._add_rim_point(wx, wy)
            kind, target = self._hit(wx, wy)
            if kind is not None and target is not None:
                # Freeze the projection so a stage poll mid-drag cannot alter
                # the geometry under the operator's hand.
                self._drag_kind = kind
                self._drag_id = target.target_id
                self._drag_frozen = (self._stage_x_um, self._stage_y_um,
                                     self._um_per_px)
                return True
            return super().eventFilter(obj, event)

        if etype == QEvent.Type.MouseMove and self._drag_kind is not None:
            self._apply_drag(event.position().x(), event.position().y(),
                             commit=False)
            return True

        if (etype == QEvent.Type.MouseButtonRelease
                and event.button() == Qt.LeftButton
                and self._drag_kind is not None):
            self._apply_drag(event.position().x(), event.position().y(),
                             commit=True)
            self._end_drag()
            return True

        return super().eventFilter(obj, event)

    def _clamped_image_point(self, wx: float, wy: float):
        """Widget point → frame pixel, clamped to the frame.

        Clamping (rather than bailing) means the circle keeps following the
        cursor when it strays off the pixmap instead of lurching back — the same
        behaviour MeasurementCameraView's endpoint drag has.
        """
        img = self._widget_to_image(wx, wy)
        if img is not None:
            return img
        pm = self._last_pixmap
        img_w, img_h = self._last_image_size
        if pm is None or not img_w or not img_h:
            return None
        scale = self._widget_scale() or 1.0
        ox = (self._display.width() - pm.width()) / 2.0
        oy = (self._display.height() - pm.height()) / 2.0
        ix = min(max(0.0, (wx - ox) / scale), float(img_w))
        iy = min(max(0.0, (wy - oy) / scale), float(img_h))
        return (ix, iy)

    def _apply_drag(self, wx: float, wy: float, *, commit: bool):
        target = self._target_by_id(self._drag_id)
        img_w, img_h = self._last_image_size
        if target is None or not img_w:
            return
        pt = self._clamped_image_point(wx, wy)
        if pt is None:
            return
        _sx, _sy, eff = self._projection()
        if self._drag_kind == "size":
            centre = self._um_to_image_px(target.x_um, target.y_um,
                                          img_w, img_h)
            r_px = ((pt[0] - centre.x()) ** 2 + (pt[1] - centre.y()) ** 2) ** 0.5
            d_um = max(1.0, 2.0 * r_px * (eff or 1.0))
            (self.size_committed if commit else self.size_dragged).emit(
                target.target_id, d_um)
        else:
            mgr = self._manager
            if mgr is None:
                return
            try:
                dx_um, dy_um = mgr.pixel_to_stage_offset(
                    self._cam_idx, pt[0], pt[1], img_w, img_h)
            except Exception:
                return
            sx, sy, _e = self._projection()
            (self.center_committed if commit else self.center_dragged).emit(
                target.target_id, sx + dx_um, sy + dy_um)

    def _end_drag(self):
        self._drag_kind = None
        self._drag_id = ""
        self._drag_frozen = None

    def _add_rim_point(self, wx: float, wy: float) -> bool:
        pt = self._clamped_image_point(wx, wy)
        if pt is None:
            return True
        self._rim_points.append((float(pt[0]), float(pt[1])))
        self._refit_rim()
        self._rerender_last()
        return True

    def _refit_rim(self):
        """Least-squares circle through the clicked rim points (3+ needed).

        More accurate than a single dragged radius on an irregular or fuzzy
        spheroid, which is why both gestures ship.
        """
        self._rim_fit = None
        if len(self._rim_points) < 3:
            return
        try:
            from SupportClasses.VisionDetector import fit_circle_to_points
            fit = fit_circle_to_points(self._rim_points)
        except Exception as exc:
            logger.debug("rim circle fit failed: %s", exc)
            return
        if fit is None:
            return
        cx, cy, r = fit
        img_w, img_h = self._last_image_size
        mgr = self._manager
        if mgr is None or not img_w:
            return
        try:
            dx_um, dy_um = mgr.pixel_to_stage_offset(
                self._cam_idx, cx, cy, img_w, img_h)
        except Exception:
            return
        sx, sy, eff = self._projection()
        self._rim_fit = (sx + dx_um, sy + dy_um, 2.0 * r * (eff or 1.0))

    def commit_rim_fit(self) -> bool:
        """Emit the current rim fit and clear the points. False if none yet."""
        if self._rim_fit is None:
            return False
        x_um, y_um, d_um = self._rim_fit
        self.clear_rim_points()
        self.rim_fitted.emit(x_um, y_um, d_um)
        return True

    # ── painting ──────────────────────────────────────────────────

    # Override: draw picks (green) + places (mauve) + connectors.
    def _draw_targets(self, pixmap: QPixmap):
        img_w = pixmap.width()
        img_h = pixmap.height()
        if img_w == 0 or img_h == 0:
            return
        if not self._pick_targets and not self._place_targets \
                and not self._rim_points:
            return

        painter = QPainter(pixmap)
        painter.setRenderHint(QPainter.Antialiasing)

        def to_px(t: PickPlaceTarget) -> QPointF:
            return self._um_to_image_px(t.x_um, t.y_um, img_w, img_h)

        # Connector lines for paired entries.
        painter.setPen(QPen(QColor(COLORS["overlay0"]), 1, Qt.DashLine))
        for pk, pl in zip(self._pick_targets, self._place_targets):
            painter.drawLine(to_px(pk), to_px(pl))

        self._draw_marker_set(painter, self._pick_targets,
                              QColor(COLORS["green"]), img_w, img_h, to_px)
        self._draw_marker_set(painter, self._place_targets,
                              QColor(COLORS["mauve"]), img_w, img_h, to_px)
        self._draw_rim(painter)
        painter.end()

    def _draw_marker_set(self, painter, targets, color, img_w, img_h, to_px):
        painter.setFont(QFont("Consolas", 8))
        for t in targets:
            pos = to_px(t)
            if (pos.x() < -50 or pos.y() < -50
                    or pos.x() > img_w + 50 or pos.y() > img_h + 50):
                continue
            r_px = self._radius_image_px(t)
            # A mosaic-derived position is a hint until a live click confirms it
            # — dashed says "not yet verified against this frame".
            unconfirmed = (self._provenance.get(t.target_id) == PROV_MOSAIC)
            pen = QPen(color, 2)
            if unconfirmed:
                pen.setStyle(Qt.DashLine)
            painter.setPen(pen)
            painter.setBrush(QBrush(QColor(
                color.red(), color.green(), color.blue(), 50)))
            painter.drawEllipse(pos, r_px, r_px)
            if self._measure_mode and not self._rim_mode:
                # A visible grab point, so the resize hit zone is discoverable.
                painter.setPen(QPen(color, 1))
                painter.setBrush(QBrush(color))
                painter.drawEllipse(
                    QPointF(pos.x() + r_px, pos.y()), 2.5, 2.5)
            painter.setBrush(Qt.NoBrush)
            painter.setPen(QPen(color, 1))
            label = t.target_id
            if getattr(t, "size_um", 0.0):
                label = f"{t.target_id} Ø{float(t.size_um):.0f}"
            painter.drawText(
                int(pos.x() + r_px + 3), int(pos.y() - 2), label)

    def _draw_rim(self, painter):
        if not self._rim_points:
            return
        peach = QColor(COLORS["peach"])
        painter.setPen(QPen(peach, 1))
        painter.setBrush(QBrush(peach))
        for (px, py) in self._rim_points:
            painter.drawEllipse(QPointF(px, py), 2.5, 2.5)
        painter.setBrush(Qt.NoBrush)
        if self._rim_fit is None:
            return
        img_w, img_h = self._last_image_size
        x_um, y_um, d_um = self._rim_fit
        pos = self._um_to_image_px(x_um, y_um, img_w, img_h)
        _sx, _sy, eff = self._projection()
        r_px = (d_um / 2.0) / (eff or 1.0)
        pen = QPen(QColor(COLORS["yellow"]), 2, Qt.DashLine)
        painter.setPen(pen)
        painter.drawEllipse(pos, r_px, r_px)
        painter.setPen(QPen(QColor(COLORS["yellow"]), 1))
        painter.drawText(int(pos.x() + r_px + 3), int(pos.y() + 12),
                         f"Ø{d_um:.0f} µm")


# ── the tool ────────────────────────────────────────────────────────

class LiveTargetPicker(QWidget):
    """Single live-view picker with paired pick + place target lists.

    One camera view, one camera→stage transform, two lists paired by
    index:

      - Pick targets  (multi, green, IDs P001…)
      - Place targets (multi, mauve, IDs D001…)

    Pick i is deposited at Place i. A connector line shows each pairing.

    Click behavior is routed by the Mode toggle:

      - Mode = Pick   → left-click appends a pick; right-click removes
                        the nearest pick within `_REMOVE_RADIUS_UM`.
      - Mode = Place  → left-click appends a place; right-click removes
                        the nearest place.

    Public API:
        set_hardware_config(hw_config)
        picks()  -> list[PickPlaceTarget]
        places() -> list[PickPlaceTarget]
        pairs()  -> list[tuple[PickPlaceTarget, PickPlaceTarget]]
        is_balanced() -> bool   # ≥1 pair AND equal pick/place counts
        clear_picks(); clear_places()
        add_pick(x_um, y_um, size_um=0.0, provenance="live")
        add_place(x_um, y_um, size_um=0.0, provenance="live")
        target_by_id(target_id); set_target_size(target_id, d_um)
        set_measure_mode(on)

    Signals:
        picks_changed(list[PickPlaceTarget])
        places_changed(list[PickPlaceTarget])
        target_changed(str target_id)      # size / position edited
        goto_requested(str target_id)      # host owns the motion

    v7.8: this widget never commands motion. "Go to" emits ``goto_requested`` and
    the host page routes it through ``SafeTravelWorker`` (retract Z, wait, travel
    XY, never descend) — the picker stays hardware-light.
    """

    picks_changed = Signal(list)
    places_changed = Signal(list)
    target_changed = Signal(str)
    goto_requested = Signal(str)
    # (target_id, provenance) the moment a pick is created — lets a host react to
    # a LIVE-clicked pick specifically (e.g. bank a training crop, which is only
    # meaningful when the stage is actually on the spheroid).
    pick_added = Signal(str, str)

    MODE_PICK = "pick"
    MODE_PLACE = "place"

    def __init__(self, controller, camera_manager, parent: QWidget | None = None,
                 *, pick_only: bool = False):
        super().__init__(parent)
        self._controller = controller
        self._camera_manager = camera_manager
        # pick_only: hide the Place list + Mode toggle and force every click to
        # the Pick list. Used by workflows that only "select regions" with no
        # paired placement (e.g. Cell Labeling). The picker stays usable as the
        # shared multi-target picker; only the place machinery is hidden.
        self._pick_only: bool = bool(pick_only)

        self._hw_config = None
        self._cam_idx: int = 0
        self._objective_name: Optional[str] = None
        self._camera_model: Optional[str] = None

        # µm/px calibration state. ``_base_um_per_px`` is the value as measured
        # (at ``_cal_resolution`` px); the value actually used for the live
        # feed is rescaled to the current frame width — see
        # ``_sync_live_um_per_px``. ``_last_live_w`` debounces the rescale to
        # actual resolution changes.
        self._base_um_per_px: Optional[float] = None
        self._cal_resolution: Optional[tuple[int, int]] = None
        self._last_live_w: Optional[int] = None

        self._picks: list[PickPlaceTarget] = []
        self._places: list[PickPlaceTarget] = []
        self._next_pick_id: int = 1
        self._next_place_id: int = 1
        self._mode: str = self.MODE_PICK
        # target_id → PROV_* for the POSITION. Kept here rather than on
        # PickPlaceTarget so the serialized dataclass is untouched (and a stale
        # provenance can never travel with a saved target).
        self._provenance: dict[str, str] = {}
        self._measure_mode = False
        # kind ("pick"/"place") → (goto, edit, delete) buttons.
        self._action_buttons: dict[str, tuple] = {}

        self._build_ui()

        # Stage position poll @ 5 Hz so the overlay tracks XY motion
        self._poll = QTimer(self)
        self._poll.setInterval(200)
        self._poll.timeout.connect(self._refresh_stage_position)
        self._poll.start()

    # ── UI ────────────────────────────────────────────────────────

    def _build_ui(self):
        outer = QVBoxLayout(self)
        outer.setContentsMargins(s(8), s(8), s(8), s(8))
        outer.setSpacing(s(6))

        header = QHBoxLayout()
        header.setSpacing(s(8))
        title = QLabel("Live target picker")
        title.setStyleSheet(
            f"color: {COLORS['blue']};"
            f"font-size: {sf(11)}pt;"
            f"font-weight: 600;"
        )
        header.addWidget(title)
        self._um_per_px_label = QLabel("µm/px: —")
        self._um_per_px_label.setStyleSheet(
            f"color: {COLORS['subtext0']};"
            f"font-size: {sf(8)}pt;"
        )
        header.addWidget(self._um_per_px_label)
        header.addStretch(1)
        outer.addLayout(header)

        split = QSplitter(Qt.Horizontal, self)
        outer.addWidget(split, stretch=1)

        # ── Left: shared camera view ──
        self._view = _PickerCameraView(
            camera_manager=self._camera_manager,
            cam_idx=0,
            show_crosshair=True,
            label="No microscope camera",
        )
        self._view.clicked.connect(self._on_left_click_pixel)
        self._view.right_clicked.connect(self._on_right_click_pixel)
        self._view.size_dragged.connect(self._on_size_dragged)
        self._view.size_committed.connect(self._on_size_committed)
        self._view.center_dragged.connect(self._on_center_dragged)
        self._view.center_committed.connect(self._on_center_committed)
        self._view.rim_fitted.connect(self._on_rim_fitted)
        split.addWidget(self._view)

        # ── Right: mode toggle + both lists + pairing status ──
        right = QWidget(self)
        rl = QVBoxLayout(right)
        rl.setContentsMargins(s(6), s(6), s(6), s(6))
        rl.setSpacing(s(8))

        mode_toggle = self._build_mode_toggle()
        measure_row = self._build_measure_row()
        pick_section = self._build_pick_section()
        place_section = self._build_place_section()
        self._mode_toggle_widget = mode_toggle
        self._place_section_widget = place_section
        if self._pick_only:
            # Force pick mode and hide the place machinery entirely.
            self._mode = self.MODE_PICK
            mode_toggle.setVisible(False)
            place_section.setVisible(False)
        rl.addWidget(mode_toggle)
        rl.addWidget(measure_row)
        rl.addWidget(pick_section, stretch=1)
        rl.addWidget(place_section, stretch=1)
        rl.addWidget(self._build_pairing_status())

        split.addWidget(right)
        split.setStretchFactor(0, 5)
        split.setStretchFactor(1, 2)

    def _build_mode_toggle(self) -> QFrame:
        frame = QFrame(self)
        frame.setObjectName("modeToggle")
        frame.setStyleSheet(
            f"QFrame#modeToggle {{"
            f"  background-color: {COLORS['surface0']};"
            f"  border: 1px solid {COLORS['surface1']};"
            f"  border-radius: 6px;"
            f"}}"
        )
        row = QHBoxLayout(frame)
        row.setContentsMargins(s(8), s(6), s(8), s(6))
        row.setSpacing(s(10))

        row.addWidget(QLabel("Click adds to:"))
        self._mode_group = QButtonGroup(frame)
        self._radio_pick = QRadioButton("Pick")
        self._radio_place = QRadioButton("Place")
        self._radio_pick.setChecked(True)
        self._mode_group.addButton(self._radio_pick)
        self._mode_group.addButton(self._radio_place)
        self._radio_pick.toggled.connect(
            lambda on: on and self._set_mode(self.MODE_PICK))
        self._radio_place.toggled.connect(
            lambda on: on and self._set_mode(self.MODE_PLACE))
        row.addWidget(self._radio_pick)
        row.addWidget(self._radio_place)
        row.addStretch(1)
        return frame

    def _build_measure_row(self) -> QFrame:
        """Measure-mode + rim-fit controls.

        OFF is the legacy behaviour exactly: every left click adds a target, no
        handles, no rim points. This flag is the non-regression guarantee, and
        every new gesture is gated on it.
        """
        frame = QFrame(self)
        frame.setObjectName("measureRow")
        frame.setStyleSheet(
            f"QFrame#measureRow {{"
            f"  background-color: {COLORS['surface0']};"
            f"  border: 1px solid {COLORS['surface1']};"
            f"  border-radius: 6px;"
            f"}}"
        )
        v = QVBoxLayout(frame)
        v.setContentsMargins(s(8), s(6), s(8), s(6))
        v.setSpacing(s(4))

        self._measure_check = QCheckBox("Measure Ø on the live view")
        self._measure_check.setToolTip(
            "Drag a target's ring to set its diameter, or switch on rim points "
            "and click 3+ points around the spheroid's edge to fit a circle.\n"
            "Off: a left click simply adds a target, exactly as before.")
        self._measure_check.toggled.connect(self.set_measure_mode)
        v.addWidget(self._measure_check)

        row = QHBoxLayout()
        row.setSpacing(s(6))
        self._rim_check = QCheckBox("Rim points")
        self._rim_check.setToolTip(
            "Click 3+ points around the spheroid's edge; a circle is fitted "
            "through them. More accurate than one dragged radius on an "
            "irregular spheroid.")
        self._rim_check.setEnabled(False)
        self._rim_check.toggled.connect(self._on_rim_toggled)
        row.addWidget(self._rim_check)
        self._rim_apply = QPushButton("Apply fit")
        self._rim_apply.setEnabled(False)
        self._rim_apply.clicked.connect(self._on_apply_rim_fit)
        row.addWidget(self._rim_apply)
        self._rim_clear = QPushButton("Clear pts")
        self._rim_clear.setEnabled(False)
        self._rim_clear.clicked.connect(self._on_clear_rim)
        row.addWidget(self._rim_clear)
        row.addStretch(1)
        v.addLayout(row)
        return frame

    def _build_pick_section(self) -> QWidget:
        wrap = QWidget(self)
        v = QVBoxLayout(wrap)
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(s(4))

        header = QLabel("Pick targets")
        header.setStyleSheet(
            f"color: {COLORS['green']};"
            f"font-weight: 600;"
            f"font-size: {sf(10)}pt;"
        )
        v.addWidget(header)

        self._pick_list = QListWidget()
        self._pick_list.setStyleSheet(self._list_qss())
        # "Remove selected" always intended plural (its loop walks rows in
        # reverse), but the list was left on the default single-selection mode,
        # so it could only ever remove one. The row actions below stay gated on
        # exactly one selection, since each acts on a single target.
        self._pick_list.setSelectionMode(
            QAbstractItemView.SelectionMode.ExtendedSelection)
        self._pick_list.itemSelectionChanged.connect(
            self._refresh_action_buttons)
        self._pick_list.itemDoubleClicked.connect(
            lambda _i: self._edit_selected(self._pick_list, self._picks))
        v.addWidget(self._pick_list, stretch=1)

        v.addLayout(self._build_action_row(self._pick_list, self._picks,
                                           self.picks_changed, "pick"))

        btn_row = QHBoxLayout()
        btn_row.setSpacing(s(6))
        clear = QPushButton("Clear all")
        clear.clicked.connect(self.clear_picks)
        btn_row.addWidget(clear)
        remove_sel = QPushButton("Remove selected")
        remove_sel.clicked.connect(self._remove_selected_pick)
        btn_row.addWidget(remove_sel)
        v.addLayout(btn_row)
        return wrap

    def _build_place_section(self) -> QWidget:
        wrap = QWidget(self)
        v = QVBoxLayout(wrap)
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(s(4))

        header = QLabel("Place targets")
        header.setStyleSheet(
            f"color: {COLORS['mauve']};"
            f"font-weight: 600;"
            f"font-size: {sf(10)}pt;"
        )
        v.addWidget(header)

        self._place_list = QListWidget()
        self._place_list.setStyleSheet(self._list_qss())
        self._place_list.setSelectionMode(
            QAbstractItemView.SelectionMode.ExtendedSelection)
        self._place_list.itemSelectionChanged.connect(
            self._refresh_action_buttons)
        self._place_list.itemDoubleClicked.connect(
            lambda _i: self._edit_selected(self._place_list, self._places))
        v.addWidget(self._place_list, stretch=1)

        v.addLayout(self._build_action_row(self._place_list, self._places,
                                           self.places_changed, "place"))

        btn_row = QHBoxLayout()
        btn_row.setSpacing(s(6))
        clear = QPushButton("Clear all")
        clear.clicked.connect(self.clear_places)
        btn_row.addWidget(clear)
        remove_sel = QPushButton("Remove selected")
        remove_sel.clicked.connect(self._remove_selected_place)
        btn_row.addWidget(remove_sel)
        v.addLayout(btn_row)
        return wrap

    def _build_action_row(self, list_widget, lst, signal, kind: str):
        """[Go to] [Edit…] [Delete] for the selected row of one list.

        Enabled only on exactly one selection — every action is single-target, so
        a multi-selection has no unambiguous meaning.
        """
        row = QHBoxLayout()
        row.setSpacing(s(6))
        goto = QPushButton("Go to")
        goto.setToolTip("Retract the needle to safe Z, then travel to this "
                        "target. The stage never descends.")
        goto.clicked.connect(lambda: self._goto_selected(list_widget))
        row.addWidget(goto)
        edit = QPushButton("Edit…")
        edit.clicked.connect(lambda: self._edit_selected(list_widget, lst))
        row.addWidget(edit)
        delete = QPushButton("Delete")
        delete.clicked.connect(
            lambda: self._remove_selected_rows(list_widget, lst, signal))
        row.addWidget(delete)
        row.addStretch(1)
        self._action_buttons[kind] = (goto, edit, delete)
        return row

    def _build_pairing_status(self) -> QLabel:
        self._pairing_label = QLabel("No targets yet.")
        self._pairing_label.setWordWrap(True)
        self._pairing_label.setStyleSheet(
            f"color: {COLORS['subtext0']};"
            f"font-size: {sf(9)}pt;"
            f"padding: {s(4)}px;"
        )
        return self._pairing_label

    @staticmethod
    def _list_qss() -> str:
        return (
            f"QListWidget {{"
            f"  background-color: {COLORS['surface0']};"
            f"  color: {COLORS['text']};"
            f"  font-size: {sf(9)}pt;"
            f"}}"
        )

    # ── public API ────────────────────────────────────────────────

    def picks(self) -> list[PickPlaceTarget]:
        return list(self._picks)

    def places(self) -> list[PickPlaceTarget]:
        return list(self._places)

    def pairs(self) -> list[tuple[PickPlaceTarget, PickPlaceTarget]]:
        """Aligned (pick, place) pairs by index (truncated to the
        shorter list)."""
        n = min(len(self._picks), len(self._places))
        return [(self._picks[i], self._places[i]) for i in range(n)]

    def is_balanced(self) -> bool:
        return len(self._picks) > 0 and len(self._picks) == len(self._places)

    def clear_picks(self):
        if not self._picks:
            return
        for t in self._picks:
            self._provenance.pop(t.target_id, None)
        self._picks.clear()
        self._next_pick_id = 1
        self._sync_overlay_and_lists()
        self.picks_changed.emit(list(self._picks))

    def clear_places(self):
        if not self._places:
            return
        for t in self._places:
            self._provenance.pop(t.target_id, None)
        self._places.clear()
        self._next_place_id = 1
        self._sync_overlay_and_lists()
        self.places_changed.emit(list(self._places))

    def target_by_id(self, target_id: str):
        """The pick or place target with this id, or None."""
        for t in self._picks + self._places:
            if t.target_id == target_id:
                return t
        return None

    def provenance(self, target_id: str) -> str:
        """Where this target's POSITION came from (``PROV_*``)."""
        return self._provenance.get(target_id, PROV_LIVE)

    def set_measure_mode(self, on: bool):
        """Enable live-view Ø editing. OFF is byte-identical legacy behaviour."""
        on = bool(on)
        self._measure_mode = on
        self._view.set_measure_mode(on)
        if hasattr(self, "_measure_check") and self._measure_check.isChecked() != on:
            self._measure_check.setChecked(on)
        if hasattr(self, "_rim_check"):
            self._rim_check.setEnabled(on)
            if not on:
                self._rim_check.setChecked(False)
        self._refresh_rim_buttons()

    def measure_mode(self) -> bool:
        return self._measure_mode

    def set_target_size(self, target_id: str, diameter_um: float) -> bool:
        """Set a target's measured diameter (µm) — the single source of truth.

        A diameter in µm is frame-invariant, so both editing surfaces (this live
        view and the mosaic) simply write it here and read it back; there is no
        two-way binding to keep in step. A pixel radius is deliberately never
        stored: the live µm/px changes with resolution and would silently resize
        every target.
        """
        t = self.target_by_id(target_id)
        if t is None:
            return False
        d = max(0.0, float(diameter_um or 0.0))
        if abs(float(t.size_um) - d) < 1e-9:
            return False
        t.size_um = d
        self._sync_overlay_and_lists()
        self.target_changed.emit(target_id)
        return True

    def set_target_position(self, target_id: str, x_um: float, y_um: float,
                            provenance: str | None = None) -> bool:
        """Move a target, optionally re-stamping where its position came from."""
        t = self.target_by_id(target_id)
        if t is None:
            return False
        t.x_um = float(x_um)
        t.y_um = float(y_um)
        if provenance:
            self._provenance[target_id] = str(provenance)
        self._sync_overlay_and_lists()
        self.target_changed.emit(target_id)
        return True

    def confirm_target_position(self, target_id: str, x_um: float,
                               y_um: float) -> bool:
        """Adopt a LIVE-derived position for a target, marking it confirmed.

        This is what upgrades a mosaic-derived hint into a trustworthy target:
        the coordinate is re-derived from the current frame through
        ``pixel_to_stage_offset``, so it carries none of the mosaic's
        registration uncertainty.
        """
        return self.set_target_position(target_id, x_um, y_um,
                                        provenance=PROV_CONFIRMED)

    @property
    def cam_idx(self) -> int:
        """The microscope camera slot this picker maps clicks through.

        Exposed so a HOST page can own the camera's start/stop without keeping
        its own copy of the ``camera_for_role(MICROSCOPE)`` resolution — two
        copies drift, and a host that started slot 0 while the picker mapped
        clicks through slot 2 would silently project them through the wrong
        µm/px.
        """
        return int(self._cam_idx)

    def set_hardware_config(self, hw_config):
        """Re-resolve microscope camera + active objective from hw_config."""
        self._hw_config = hw_config
        if hw_config is None:
            return
        try:
            cam_idx = hw_config.camera_for_role(CameraRole.MICROSCOPE)
        except Exception as e:
            logger.warning("camera_for_role failed: %s", e)
            cam_idx = None
        if cam_idx is None:
            cam_idx = 0
        self._cam_idx = cam_idx
        self._view.set_camera(cam_idx)

        try:
            cam_cfg = hw_config.cameras[cam_idx] if cam_idx < len(
                hw_config.cameras) else None
        except Exception:
            cam_cfg = None
        if cam_cfg is not None:
            self._objective_name = getattr(cam_cfg, "current_objective_name", None)
            spec = getattr(cam_cfg, "camera_spec", None)
            self._camera_model = getattr(spec, "model", None) if spec else None
        else:
            self._objective_name = None
            self._camera_model = None

        self._refresh_um_per_px()

    # ── click routing ─────────────────────────────────────────────

    def _set_mode(self, mode: str):
        self._mode = mode

    def _on_left_click_pixel(self, px_x: float, px_y: float):
        coords = self._pixel_to_stage_um(px_x, px_y)
        if coords is None:
            return
        x_um, y_um = coords
        logger.debug(
            "live picker LEFT click mode=%s: img (%.2f, %.2f) → stage (%.2f, %.2f) µm",
            self._mode, px_x, px_y, x_um, y_um,
        )
        if self._mode == self.MODE_PICK:
            self._add_pick(x_um, y_um)
        else:
            self._add_place(x_um, y_um)

    def _on_right_click_pixel(self, px_x: float, px_y: float):
        coords = self._pixel_to_stage_um(px_x, px_y)
        if coords is None:
            return
        x_um, y_um = coords
        if self._mode == self.MODE_PICK:
            self._remove_nearest(self._picks, x_um, y_um, self.picks_changed)
        else:
            self._remove_nearest(self._places, x_um, y_um, self.places_changed)

    # ── live-view circle editing ──────────────────────────────────

    def _on_size_dragged(self, target_id: str, diameter_um: float):
        """Live feedback while the ring is dragged (no signal storm downstream)."""
        t = self.target_by_id(target_id)
        if t is None:
            return
        t.size_um = max(0.0, float(diameter_um))
        self._sync_overlay_and_lists()

    def _on_size_committed(self, target_id: str, diameter_um: float):
        t = self.target_by_id(target_id)
        if t is None:
            return
        t.size_um = max(0.0, float(diameter_um))
        self._sync_overlay_and_lists()
        self.target_changed.emit(target_id)

    def _on_center_dragged(self, target_id: str, x_um: float, y_um: float):
        t = self.target_by_id(target_id)
        if t is None:
            return
        t.x_um, t.y_um = float(x_um), float(y_um)
        self._sync_overlay_and_lists()

    def _on_center_committed(self, target_id: str, x_um: float, y_um: float):
        # Dragged on the LIVE view, so the position is now frame-derived.
        self.confirm_target_position(target_id, x_um, y_um)
        signal = (self.picks_changed
                  if any(t.target_id == target_id for t in self._picks)
                  else self.places_changed)
        signal.emit(list(self._picks if signal is self.picks_changed
                         else self._places))

    def _on_rim_toggled(self, on: bool):
        self._view.set_rim_mode(bool(on))
        self._refresh_rim_buttons()

    def _on_clear_rim(self):
        self._view.clear_rim_points()
        self._refresh_rim_buttons()

    def _refresh_rim_buttons(self):
        if not hasattr(self, "_rim_apply"):
            return
        rim_on = self._measure_mode and self._rim_check.isChecked()
        self._rim_clear.setEnabled(rim_on)
        self._rim_apply.setEnabled(rim_on and self._view.rim_fit() is not None)

    def _on_rim_fitted(self, x_um: float, y_um: float, diameter_um: float):
        """A rim fit was applied: size the selected target, or add a new one."""
        tid = (self._sole_selected_id(self._pick_list)
               or self._sole_selected_id(self._place_list))
        if tid:
            t = self.target_by_id(tid)
            if t is not None:
                t.size_um = max(0.0, float(diameter_um))
                self.confirm_target_position(tid, x_um, y_um)
                self._refresh_rim_buttons()
                return
        if self._mode == self.MODE_PICK:
            self._add_pick(x_um, y_um, diameter_um)
        else:
            self._add_place(x_um, y_um, diameter_um)
        self._refresh_rim_buttons()

    def _on_apply_rim_fit(self):
        if not self._view.commit_rim_fit():
            return
        self._refresh_rim_buttons()

    # ── mutators ──────────────────────────────────────────────────

    def add_pick(self, x_um: float, y_um: float, size_um: float = 0.0,
                 provenance: str = PROV_LIVE) -> str:
        """Append a pick target and return its id. Public since v7.8 so the
        spheroid survey can transfer a curated list in."""
        return self._add_pick(x_um, y_um, size_um, provenance)

    def add_place(self, x_um: float, y_um: float, size_um: float = 0.0,
                  provenance: str = PROV_LIVE) -> str:
        return self._add_place(x_um, y_um, size_um, provenance)

    def _add_pick(self, x_um: float, y_um: float, size_um: float = 0.0,
                  provenance: str = PROV_LIVE) -> str:
        tid = f"P{self._next_pick_id:03d}"
        self._next_pick_id += 1
        self._picks.append(PickPlaceTarget(
            target_id=tid, x_um=x_um, y_um=y_um, well_name="",
            size_um=max(0.0, float(size_um or 0.0)), selected=True))
        self._provenance[tid] = str(provenance or PROV_LIVE)
        self._sync_overlay_and_lists()
        self.picks_changed.emit(list(self._picks))
        self.pick_added.emit(tid, self._provenance[tid])
        return tid

    def _add_place(self, x_um: float, y_um: float, size_um: float = 0.0,
                   provenance: str = PROV_LIVE) -> str:
        tid = f"D{self._next_place_id:03d}"
        self._next_place_id += 1
        self._places.append(PickPlaceTarget(
            target_id=tid, x_um=x_um, y_um=y_um, well_name="",
            size_um=max(0.0, float(size_um or 0.0)), selected=True))
        self._provenance[tid] = str(provenance or PROV_LIVE)
        self._sync_overlay_and_lists()
        self.places_changed.emit(list(self._places))
        return tid

    def _remove_radius_um(self, t) -> float:
        """Right-click removal radius, widened for a large spheroid.

        The flat 250 µm is narrower than a 400 µm spheroid's own drawn ring, so
        right-clicking its rim would not remove it.
        """
        return max(_REMOVE_RADIUS_UM, float(getattr(t, "size_um", 0.0) or 0.0))

    def _remove_nearest(self, lst, x_um, y_um, signal):
        if not lst:
            return
        best_idx = None
        best_score = None
        for i, t in enumerate(lst):
            d2 = (t.x_um - x_um) ** 2 + (t.y_um - y_um) ** 2
            if d2 > self._remove_radius_um(t) ** 2:
                continue
            if best_score is None or d2 < best_score:
                best_score = d2
                best_idx = i
        if best_idx is None:
            return
        self._provenance.pop(lst[best_idx].target_id, None)
        del lst[best_idx]
        self._sync_overlay_and_lists()
        signal.emit(list(lst))

    def _remove_selected_pick(self):
        self._remove_selected_rows(self._pick_list, self._picks, self.picks_changed)

    def _remove_selected_place(self):
        self._remove_selected_rows(self._place_list, self._places, self.places_changed)

    def _remove_selected_rows(self, list_widget, lst, signal):
        ids = self._selected_ids(list_widget)
        if not ids:
            return
        for tid in ids:
            self._provenance.pop(tid, None)
        # Delete by ID, not by row index: an index is only valid until the first
        # deletion, and stops meaning anything at all once rows can be reordered.
        keep = [t for t in lst if t.target_id not in ids]
        if len(keep) == len(lst):
            return
        lst[:] = keep
        self._sync_overlay_and_lists()
        signal.emit(list(lst))

    # ── row actions ───────────────────────────────────────────────

    def _selected_ids(self, list_widget) -> set:
        out = set()
        for item in list_widget.selectedItems():
            tid = item.data(Qt.ItemDataRole.UserRole)
            if tid:
                out.add(str(tid))
        return out

    def _sole_selected_id(self, list_widget) -> Optional[str]:
        ids = self._selected_ids(list_widget)
        return next(iter(ids)) if len(ids) == 1 else None

    def _refresh_action_buttons(self):
        for kind, buttons in self._action_buttons.items():
            lw = self._pick_list if kind == "pick" else self._place_list
            on = self._sole_selected_id(lw) is not None
            for b in buttons:
                b.setEnabled(on)

    def _goto_selected(self, list_widget):
        tid = self._sole_selected_id(list_widget)
        if tid:
            self.goto_requested.emit(tid)

    def _edit_selected(self, list_widget, lst):
        tid = self._sole_selected_id(list_widget)
        if not tid:
            return
        t = self.target_by_id(tid)
        if t is None:
            return
        dlg = _TargetEditDialog(t, parent=self, show_size=not self._pick_only)
        if dlg.exec() != QDialog.DialogCode.Accepted:
            return
        x_um, y_um, d_um, label = dlg.values()
        moved = (abs(t.x_um - x_um) > 1e-9) or (abs(t.y_um - y_um) > 1e-9)
        t.x_um, t.y_um, t.label = x_um, y_um, label
        if not self._pick_only:
            t.size_um = max(0.0, d_um)
        if moved:
            # A typed position is operator-asserted, so it is as good as a live
            # click — clear any "mosaic, unconfirmed" flag.
            self._provenance[tid] = PROV_CONFIRMED
        self._sync_overlay_and_lists()
        self.target_changed.emit(tid)
        (self.picks_changed if lst is self._picks
         else self.places_changed).emit(list(lst))

    # ── transform helpers ─────────────────────────────────────────

    def _pixel_to_stage_um(
        self, px_x: float, px_y: float,
    ) -> Optional[tuple[float, float]]:
        img_w, img_h = self._view.image_size
        if img_w == 0 or img_h == 0 or self._camera_manager is None:
            return None
        # pixel_to_stage_offset resolves µm/px against the LIVE frame width
        # (img_w), so the click→stage scale tracks the current resolution.
        dx_um, dy_um = self._camera_manager.pixel_to_stage_offset(
            self._cam_idx, px_x, px_y, img_w, img_h)
        stage_xy_um = self._read_stage_xy_um()
        if stage_xy_um is None:
            return None
        # stage_xy_um is ABSOLUTE stage µm (the camera centre); add the
        # in-frame offset (also µm) to get the absolute target.
        return (stage_xy_um[0] + dx_um, stage_xy_um[1] + dy_um)

    def _read_stage_xy_um(self) -> Optional[tuple[float, float]]:
        """Current absolute stage XY in µm.

        ``StageController.get_xy_position`` already returns absolute stage
        µm (``get_xy_position_mm`` divides it by 1000), so it is used
        directly — the earlier ``× 1000`` here inflated every target ~1000×,
        driving the stage into the envelope corner.
        """
        if self._controller is None:
            return None
        try:
            pos = self._controller.get_xy_position(cached=True)
            if pos is None or pos[0] is None or pos[1] is None:
                return None
            return float(pos[0]), float(pos[1])
        except Exception as e:
            logger.debug("get_xy_position failed: %s", e)
            return None

    # ── overlay / list sync ──────────────────────────────────────

    def _sync_overlay_and_lists(self):
        self._view.set_pick_targets(self._picks)
        self._view.set_place_targets(self._places)
        self._view.set_provenance(self._provenance)

        # Selection is captured and restored by target ID around the rebuild:
        # the lists are cleared and refilled on every add/remove/edit, so without
        # this a row's own [Go to] / [Edit…] / [Delete] would be unusable — the
        # selection died the instant anything changed.
        pick_sel = self._selected_ids(self._pick_list)
        place_sel = self._selected_ids(self._place_list)

        self._fill_list(self._pick_list, self._picks, self._places, pick_sel,
                        pick_side=True)
        self._fill_list(self._place_list, self._places, self._picks, place_sel,
                        pick_side=False)

        self._update_pairing_status()
        self._refresh_action_buttons()

    def _fill_list(self, list_widget, lst, other, keep_ids, *, pick_side: bool):
        list_widget.blockSignals(True)   # a restore must not re-fire selection
        try:
            list_widget.clear()
            for i, t in enumerate(lst):
                partner = other[i].target_id if i < len(other) else None
                if pick_side:
                    paired = f" → {partner}" if partner else "  (unpaired)"
                    text = f"{t.target_id}{paired}"
                else:
                    paired = f"{partner} → " if partner else "(unpaired) "
                    text = f"{paired}{t.target_id}"
                text += f"   ({t.x_um:>8.1f},{t.y_um:>8.1f})"
                if getattr(t, "size_um", 0.0):
                    text += f"  Ø{float(t.size_um):.0f}µm"
                if self._provenance.get(t.target_id) == PROV_MOSAIC:
                    text += "  ~mosaic"
                item = QListWidgetItem(text)
                item.setData(Qt.ItemDataRole.UserRole, t.target_id)
                if t.label:
                    item.setToolTip(t.label)
                list_widget.addItem(item)
                if t.target_id in keep_ids:
                    item.setSelected(True)
        finally:
            list_widget.blockSignals(False)

    def _update_pairing_status(self):
        np_, nd = len(self._picks), len(self._places)
        if self._pick_only:
            if np_ == 0:
                self._pairing_label.setText("No regions selected yet.")
                self._pairing_label.setStyleSheet(
                    f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt; "
                    f"padding: {s(4)}px;")
            else:
                self._pairing_label.setText(
                    f"✓ {np_} region(s) selected.")
                self._pairing_label.setStyleSheet(
                    f"color: {COLORS['green']}; font-size: {sf(9)}pt; "
                    f"padding: {s(4)}px;")
            return
        if np_ == 0 and nd == 0:
            self._pairing_label.setText("No targets yet.")
            self._pairing_label.setStyleSheet(
                f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt; padding: {s(4)}px;")
            return
        if np_ == nd:
            self._pairing_label.setText(f"✓ {np_} pick→place pair(s) ready.")
            self._pairing_label.setStyleSheet(
                f"color: {COLORS['green']}; font-size: {sf(9)}pt; padding: {s(4)}px;")
        else:
            self._pairing_label.setText(
                f"⚠ {np_} pick(s) / {nd} place(s) — counts must match. "
                f"Each pick needs its own place.")
            self._pairing_label.setStyleSheet(
                f"color: {COLORS['peach']}; font-size: {sf(9)}pt; padding: {s(4)}px;")

    def _refresh_stage_position(self):
        # Keep µm/px in step with the live resolution before projecting
        # markers / converting clicks (the live feed may switch resolution).
        self._sync_live_um_per_px()
        xy_um = self._read_stage_xy_um()
        if xy_um is None:
            return
        # set_stage_position expects absolute stage µm — get_xy_position is
        # already µm, so pass it straight through (no × 1000).
        self._view.set_stage_position(xy_um[0], xy_um[1])

    def _refresh_um_per_px(self):
        """Resolve the calibration µm/px (+ the resolution it was measured at)
        for the current camera + objective, push it to the shared
        CameraManager, then apply the live-resolution-scaled value."""
        base = None
        cal_res: Optional[tuple[int, int]] = None
        if self._camera_model and self._objective_name:
            try:
                from SupportClasses.ObjectiveCalibration import get_store
                cal = get_store().get_calibration(
                    self._camera_model, self._objective_name)
                if cal:
                    base = float(cal.get("measured_um_per_px", 0.0)) or None
                    res = cal.get("resolution")
                    if res and len(res) >= 2 and res[0] and res[1]:
                        cal_res = (int(res[0]), int(res[1]))
            except Exception as e:
                logger.debug("ObjectiveCalibrationStore lookup failed: %s", e)
        from_objective_store = base is not None
        if base is None and self._camera_manager is not None:
            # Fallback: the per-device µm/px already in the manager. Read its
            # calibration resolution too, so the rescale below still applies.
            base = self._camera_manager.get_um_per_px(self._cam_idx)
            try:
                getter = getattr(
                    self._camera_manager, "get_um_per_px_resolution", None)
                cal_res = getter(self._cam_idx) if callable(getter) else None
            except Exception:
                cal_res = None
        if base is None or base <= 0:
            return

        self._base_um_per_px = base
        self._cal_resolution = cal_res
        # Push the calibration value + resolution to the manager so the shared
        # pixel→stage transform (pixel_to_stage_offset) rescales to whatever
        # resolution the live feed runs at — used by the click conversion and
        # any downstream consumer.
        #
        # v7.5.x: push ONLY when the value came from the objective store. On the
        # fallback branch the value IS the manager's own, so writing it back is a
        # no-op that would nonetheless re-stamp the resolution — and this widget
        # cannot always resolve one, so it could CLEAR a correct stamp and
        # silently turn effective_um_per_px into a passthrough for every later
        # consumer (the mosaic included).
        if from_objective_store and self._camera_manager is not None:
            try:
                self._camera_manager.set_um_per_px(
                    self._cam_idx, base, resolution=cal_res)
            except TypeError:
                # Older signature without the resolution kwarg.
                self._camera_manager.set_um_per_px(self._cam_idx, base)
        self._sync_live_um_per_px(force=True)

    def _sync_live_um_per_px(self, force: bool = False):
        """Recompute the µm/px for the current live frame width and push it to
        the overlay view + readout, so the overlay's µm→pixel projection stays
        consistent with the (resolution-aware) click transform."""
        base = self._base_um_per_px
        if base is None or base <= 0:
            return
        img = self._view.image_size
        live_w = int(img[0]) if img and img[0] else 0
        if not force and live_w == self._last_live_w:
            return
        self._last_live_w = live_w

        eff = base
        mgr = self._camera_manager
        if mgr is not None and live_w and hasattr(mgr, "effective_um_per_px"):
            try:
                eff = mgr.effective_um_per_px(self._cam_idx, live_w)
            except Exception:
                eff = base
        elif self._cal_resolution and self._cal_resolution[0] and live_w:
            eff = base * float(self._cal_resolution[0]) / float(live_w)

        self._view.set_um_per_px(eff)
        cal_w = self._cal_resolution[0] if self._cal_resolution else None
        if cal_w and live_w and abs(live_w - cal_w) > 1:
            self._um_per_px_label.setText(
                f"µm/px: {eff:.3f}  (cal {base:.3f}@{cal_w} → live {live_w}px)")
        else:
            self._um_per_px_label.setText(f"µm/px: {eff:.3f}")


# ── per-target edit modal ──────────────────────────────────────────

class _TargetEditDialog(QDialog):
    """Edit one target's position, diameter and label.

    The authoritative surface for a TYPED diameter, and the fallback for nudging
    a position without a live click. Modal and tiny on purpose: in-scene /
    in-overlay numeric editors are a known crash source in this codebase (see the
    note in ``plate_designer_canvas``), so numbers are always edited in a plain
    widget.

    Under ``pick_only`` the Ø row is hidden — a "region" target has no diameter.
    """

    def __init__(self, target, parent=None, *, show_size: bool = True):
        super().__init__(parent)
        self.setWindowTitle(f"Edit {target.target_id}")
        self._show_size = bool(show_size)

        form = QFormLayout()
        form.addRow("Target", QLabel(target.target_id))

        self._x = QDoubleSpinBox()
        self._x.setRange(-1e7, 1e7)
        self._x.setDecimals(1)
        self._x.setSuffix(" µm")
        self._x.setValue(float(target.x_um))
        form.addRow("Stage X", self._x)

        self._y = QDoubleSpinBox()
        self._y.setRange(-1e7, 1e7)
        self._y.setDecimals(1)
        self._y.setSuffix(" µm")
        self._y.setValue(float(target.y_um))
        form.addRow("Stage Y", self._y)

        self._d = QDoubleSpinBox()
        self._d.setRange(0.0, 5000.0)
        self._d.setDecimals(1)
        self._d.setSingleStep(10.0)
        self._d.setSuffix(" µm")
        self._d.setValue(float(getattr(target, "size_um", 0.0) or 0.0))
        self._d.setToolTip(
            "Measured spheroid diameter. 0 means unmeasured — the run then uses "
            "the default diameter from the settings popout.")
        if self._show_size:
            form.addRow("Diameter Ø", self._d)

        self._label = QLineEdit(str(getattr(target, "label", "") or ""))
        form.addRow("Label", self._label)

        buttons = QDialogButtonBox(
            QDialogButtonBox.StandardButton.Ok
            | QDialogButtonBox.StandardButton.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)

        outer = QVBoxLayout(self)
        outer.setContentsMargins(s(12), s(12), s(12), s(12))
        outer.setSpacing(s(8))
        outer.addLayout(form)
        outer.addWidget(buttons)

    def values(self) -> tuple:
        """``(x_um, y_um, diameter_um, label)``."""
        return (float(self._x.value()), float(self._y.value()),
                float(self._d.value()) if self._show_size else 0.0,
                self._label.text().strip())

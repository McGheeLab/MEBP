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
    QButtonGroup, QFrame, QHBoxLayout, QLabel, QListWidget, QListWidgetItem,
    QPushButton, QRadioButton, QSizePolicy, QSplitter, QVBoxLayout, QWidget,
)

from gui.styles import COLORS
from gui.scaling import s, sf
from gui.widgets.target_overlay_camera_view import TargetOverlayCameraView

from SupportClasses.HardwareConfig import CameraRole
from SupportClasses.PickAndPlaceManager import PickPlaceTarget

logger = logging.getLogger(__name__)


_REMOVE_RADIUS_UM = 250.0  # right-click radius to remove a target


# ── camera view with paired pick/place overlays + right-click ──────

class _PickerCameraView(TargetOverlayCameraView):
    """Camera view that renders two target lists (pick + place) with a
    connector line between paired entries, and emits `right_clicked`.

    Picks render green, places mauve. Pair i (pick[i] ↔ place[i]) is
    joined by a dashed connector. Overrides `_draw_targets` so the
    base's single-`selected_target` colouring isn't used.
    """

    right_clicked = Signal(float, float)

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._pick_targets: list[PickPlaceTarget] = []
        self._place_targets: list[PickPlaceTarget] = []

    def set_pick_targets(self, targets: list[PickPlaceTarget]):
        self._pick_targets = list(targets)

    def set_place_targets(self, targets: list[PickPlaceTarget]):
        self._place_targets = list(targets)

    def eventFilter(self, obj, event):
        if (obj is self._display
                and event.type() == QEvent.Type.MouseButtonPress
                and event.button() == Qt.RightButton
                and self._last_pixmap):
            img = self._widget_to_image(
                event.position().x(), event.position().y())
            if img is not None:
                self.right_clicked.emit(img[0], img[1])
                return True
        return super().eventFilter(obj, event)

    # Override: draw picks (green) + places (mauve) + connectors.
    def _draw_targets(self, pixmap: QPixmap):
        img_w = pixmap.width()
        img_h = pixmap.height()
        if img_w == 0 or img_h == 0:
            return
        if not self._pick_targets and not self._place_targets:
            return

        painter = QPainter(pixmap)
        painter.setRenderHint(QPainter.Antialiasing)

        def to_px(t: PickPlaceTarget) -> QPointF:
            ix = (t.x_um - self._stage_x_um) / self._um_per_px + img_w / 2
            iy = (t.y_um - self._stage_y_um) / self._um_per_px + img_h / 2
            return QPointF(ix, iy)

        # Connector lines for paired entries.
        painter.setPen(QPen(QColor(COLORS["overlay0"]), 1, Qt.DashLine))
        for pk, pl in zip(self._pick_targets, self._place_targets):
            painter.drawLine(to_px(pk), to_px(pl))

        self._draw_marker_set(painter, self._pick_targets,
                              QColor(COLORS["green"]), img_w, img_h, to_px)
        self._draw_marker_set(painter, self._place_targets,
                              QColor(COLORS["mauve"]), img_w, img_h, to_px)
        painter.end()

    def _draw_marker_set(self, painter, targets, color, img_w, img_h, to_px):
        painter.setFont(QFont("Consolas", 8))
        for t in targets:
            pos = to_px(t)
            if (pos.x() < -50 or pos.y() < -50
                    or pos.x() > img_w + 50 or pos.y() > img_h + 50):
                continue
            if t.size_um > 0:
                r_px = max(6.0, (t.size_um / 2) / self._um_per_px)
            else:
                r_px = 8.0
            painter.setPen(QPen(color, 2))
            painter.setBrush(QBrush(QColor(
                color.red(), color.green(), color.blue(), 50)))
            painter.drawEllipse(pos, r_px, r_px)
            painter.setPen(QPen(color, 1))
            painter.drawText(
                int(pos.x() + r_px + 3), int(pos.y() - 2), t.target_id)


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

    Signals:
        picks_changed(list[PickPlaceTarget])
        places_changed(list[PickPlaceTarget])
    """

    picks_changed = Signal(list)
    places_changed = Signal(list)

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
        split.addWidget(self._view)

        # ── Right: mode toggle + both lists + pairing status ──
        right = QWidget(self)
        rl = QVBoxLayout(right)
        rl.setContentsMargins(s(6), s(6), s(6), s(6))
        rl.setSpacing(s(8))

        mode_toggle = self._build_mode_toggle()
        pick_section = self._build_pick_section()
        place_section = self._build_place_section()
        if self._pick_only:
            # Force pick mode and hide the place machinery entirely.
            self._mode = self.MODE_PICK
            mode_toggle.setVisible(False)
            place_section.setVisible(False)
        rl.addWidget(mode_toggle)
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
        v.addWidget(self._pick_list, stretch=1)

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
        v.addWidget(self._place_list, stretch=1)

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
        self._picks.clear()
        self._next_pick_id = 1
        self._sync_overlay_and_lists()
        self.picks_changed.emit(list(self._picks))

    def clear_places(self):
        if not self._places:
            return
        self._places.clear()
        self._next_place_id = 1
        self._sync_overlay_and_lists()
        self.places_changed.emit(list(self._places))

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

    # ── mutators ──────────────────────────────────────────────────

    def _add_pick(self, x_um: float, y_um: float):
        tid = f"P{self._next_pick_id:03d}"
        self._next_pick_id += 1
        self._picks.append(PickPlaceTarget(
            target_id=tid, x_um=x_um, y_um=y_um, well_name="", selected=True))
        self._sync_overlay_and_lists()
        self.picks_changed.emit(list(self._picks))

    def _add_place(self, x_um: float, y_um: float):
        tid = f"D{self._next_place_id:03d}"
        self._next_place_id += 1
        self._places.append(PickPlaceTarget(
            target_id=tid, x_um=x_um, y_um=y_um, well_name="", selected=True))
        self._sync_overlay_and_lists()
        self.places_changed.emit(list(self._places))

    def _remove_nearest(self, lst, x_um, y_um, signal):
        if not lst:
            return
        best_idx = None
        best_d2 = _REMOVE_RADIUS_UM ** 2
        for i, t in enumerate(lst):
            d2 = (t.x_um - x_um) ** 2 + (t.y_um - y_um) ** 2
            if d2 < best_d2:
                best_d2 = d2
                best_idx = i
        if best_idx is None:
            return
        del lst[best_idx]
        self._sync_overlay_and_lists()
        signal.emit(list(lst))

    def _remove_selected_pick(self):
        self._remove_selected_rows(self._pick_list, self._picks, self.picks_changed)

    def _remove_selected_place(self):
        self._remove_selected_rows(self._place_list, self._places, self.places_changed)

    def _remove_selected_rows(self, list_widget, lst, signal):
        rows = sorted(
            {i.row() for i in list_widget.selectedIndexes()}, reverse=True)
        if not rows:
            return
        for r in rows:
            if 0 <= r < len(lst):
                del lst[r]
        self._sync_overlay_and_lists()
        signal.emit(list(lst))

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

        self._pick_list.clear()
        for i, t in enumerate(self._picks):
            paired = " → " + self._places[i].target_id if i < len(self._places) else "  (unpaired)"
            self._pick_list.addItem(QListWidgetItem(
                f"{t.target_id}{paired}   ({t.x_um:>8.1f},{t.y_um:>8.1f})"))

        self._place_list.clear()
        for i, t in enumerate(self._places):
            paired = self._picks[i].target_id + " → " if i < len(self._picks) else "(unpaired) "
            self._place_list.addItem(QListWidgetItem(
                f"{paired}{t.target_id}   ({t.x_um:>8.1f},{t.y_um:>8.1f})"))

        self._update_pairing_status()

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
        if base is None and self._camera_manager is not None:
            # Fallback: the per-device µm/px. Its calibration resolution isn't
            # exposed here, so no rescale is applied (assume it matches live).
            base = self._camera_manager.get_um_per_px(self._cam_idx)
        if base is None or base <= 0:
            return

        self._base_um_per_px = base
        self._cal_resolution = cal_res
        # Push the calibration value + resolution to the manager so the shared
        # pixel→stage transform (pixel_to_stage_offset) rescales to whatever
        # resolution the live feed runs at — used by the click conversion and
        # any downstream consumer.
        if self._camera_manager is not None:
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

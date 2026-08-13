"""
spheroid_survey_panel.py — detect, curate and transfer spheroids from a mosaic.

v7.8: the right-hand side of the Spheroid Pick & Place "Spheroid survey" tab. The
left-hand side is a live instance of the Fluorescence Mosaic page, which owns the
scan; this panel owns everything that happens to the resulting image:

    Detect → curate (tick / edit Ø / delete / add by hand) → Transfer to picks

Design notes worth keeping in view:

* **Detection may legitimately find nothing.** On a nuclear stain a spheroid is a
  cluster of puncta, not a filled disc, so the two realistic outcomes are "0
  found" and "hundreds found". So: adding a spheroid by hand is first-class (not
  a fallback), and the status line reports the per-reason rejection counters so
  the operator can tell which failure they are looking at.
* **Detection runs on a worker thread.** A full-well mosaic can be 12 000 px and
  seconds of OpenCV; running it in a button handler would freeze the camera feed.
* **Detection runs on ONE raw channel**, never the blended overlay: the blend
  goes through grayscale and a saturating add, which clips overlapping channels
  and pushes a saturated blob's edge outward — it over-reads diameters.
* **A mosaic position is a search hint, not a motion target.** Transfer stamps
  ``PROV_MOSAIC`` on the pick so it renders dashed until a live click confirms
  it, and both Transfer and Go to are refused outright when the mosaic has no
  recorded registration shift (its pixel→stage mapping can then be off by up to
  20 % of a field of view).
* The table sorts NUMERICALLY on Ø via ``Qt.EditRole`` — the classic trap is
  lexicographic ordering putting "1000" before "200".
"""

from __future__ import annotations

import logging
from typing import Optional

from PySide6.QtCore import QObject, Qt, QThread, Signal
from PySide6.QtGui import QColor
from PySide6.QtWidgets import (
    QAbstractItemView, QCheckBox, QComboBox, QDoubleSpinBox, QFrame,
    QHBoxLayout, QHeaderView, QLabel, QPushButton, QTableWidget,
    QTableWidgetItem, QVBoxLayout, QWidget,
)

from gui.styles import COLORS
from gui.scaling import s, sf
from gui.worker_retirement import retire_worker

from SupportClasses import SpheroidDetector as sd

logger = logging.getLogger(__name__)

_COL_ON, _COL_ID, _COL_D, _COL_SRC, _COL_CONF, _COL_WARN = range(6)


class _DetectWorker(QThread):
    """Run the detector off the GUI thread (a big mosaic is seconds of OpenCV)."""

    done = Signal(object)
    failed = Signal(str)

    def __init__(self, image, extent_um, mosaic_scale, kwargs, parent=None):
        super().__init__(parent)
        self._image = image
        self._extent = extent_um
        self._scale = mosaic_scale
        self._kwargs = dict(kwargs)

    def run(self):
        try:
            report = sd.detect_spheroids(
                self._image, self._extent, self._scale, **self._kwargs)
        except Exception as exc:            # pragma: no cover - defensive
            logger.exception("spheroid detection crashed")
            self.failed.emit(str(exc))
            return
        self.done.emit(report)


class SpheroidSurveyPanel(QWidget):
    """Detection + curation for one well's mosaic.

    Owns no hardware and commands no motion: ``goto_requested`` and
    ``transfer_requested`` are emitted for the host page, which routes travel
    through ``SafeTravelWorker`` and adds picks to the ``LiveTargetPicker``.
    """

    # (x_um, y_um) absolute stage µm — the host retracts + travels.
    goto_requested = Signal(float, float)
    # list[dict(x_um, y_um, diameter_um, det_id)] — the curated transfer.
    transfer_requested = Signal(list)
    # (x_um, y_um, diameter_um, det_id) — bank a training crop for this one.
    crop_requested = Signal(float, float, float, str)
    # A detection's selection changed (det_id or "") so the mosaic can highlight.
    selection_changed = Signal(str)
    detections_changed = Signal()

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self._context: Optional[dict] = None
        self._dets: list[sd.SpheroidDetection] = []
        self._enabled_ids: set = set()
        self._worker: Optional[_DetectWorker] = None
        self._next_manual = 1
        # Set by the host from the workflow settings so one owner keeps the
        # authoritative values (see the plan's "two homes for one value" note).
        self._fit_badge_fn = None
        self._context_provider = None
        self._build_ui()
        self._refresh_buttons()

    # ── UI ────────────────────────────────────────────────────────

    def _build_ui(self):
        outer = QVBoxLayout(self)
        outer.setContentsMargins(s(8), s(8), s(8), s(8))
        outer.setSpacing(s(8))

        title = QLabel("Spheroid survey")
        title.setStyleSheet(
            f"color: {COLORS['blue']}; font-size: {sf(11)}pt; font-weight: 600;")
        outer.addWidget(title)

        outer.addWidget(self._build_detect_box())

        self._status = QLabel("Scan or select a well, then Detect.")
        self._status.setWordWrap(True)
        self._status.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt;")
        outer.addWidget(self._status)

        self._provenance_note = QLabel("")
        self._provenance_note.setWordWrap(True)
        self._provenance_note.setStyleSheet(
            f"color: {COLORS['peach']}; font-size: {sf(9)}pt;")
        self._provenance_note.setVisible(False)
        outer.addWidget(self._provenance_note)

        outer.addWidget(self._build_table(), stretch=1)
        outer.addWidget(self._build_row_actions())
        outer.addWidget(self._build_transfer_row())

    def _build_detect_box(self) -> QFrame:
        frame = QFrame(self)
        frame.setObjectName("detectBox")
        frame.setStyleSheet(
            f"QFrame#detectBox {{ background-color: {COLORS['surface0']};"
            f" border: 1px solid {COLORS['surface1']}; border-radius: 6px; }}")
        v = QVBoxLayout(frame)
        v.setContentsMargins(s(8), s(6), s(8), s(6))
        v.setSpacing(s(5))

        band = QHBoxLayout()
        band.setSpacing(s(6))
        band.addWidget(QLabel("Ø"))
        self._min_d = QDoubleSpinBox()
        self._min_d.setRange(1.0, 5000.0)
        self._min_d.setDecimals(0)
        self._min_d.setSingleStep(10.0)
        self._min_d.setSuffix(" µm")
        self._min_d.setValue(sd.DEFAULT_MIN_DIAMETER_UM)
        self._min_d.setToolTip("Smaller than this is not reported as a spheroid.")
        band.addWidget(self._min_d)
        band.addWidget(QLabel("to"))
        self._max_d = QDoubleSpinBox()
        self._max_d.setRange(1.0, 5000.0)
        self._max_d.setDecimals(0)
        self._max_d.setSingleStep(10.0)
        self._max_d.setSuffix(" µm")
        self._max_d.setValue(sd.DEFAULT_MAX_DIAMETER_UM)
        self._max_d.setToolTip("Larger than this is not reported (a doublet "
                               "usually lands here).")
        band.addWidget(self._max_d)
        band.addStretch(1)
        v.addLayout(band)

        row = QHBoxLayout()
        row.setSpacing(s(6))
        row.addWidget(QLabel("Channel"))
        self._channel = QComboBox()
        self._channel.setMinimumWidth(s(90))
        self._channel.setToolTip(
            "Detect on this channel's raw image. Which channel defines a "
            "spheroid's boundary is an experimental choice — a nuclear stain "
            "reads smaller than a membrane stain.")
        row.addWidget(self._channel)
        self._detect_btn = QPushButton("Detect")
        self._detect_btn.clicked.connect(self._on_detect)
        row.addWidget(self._detect_btn)
        self._clear_btn = QPushButton("Clear")
        self._clear_btn.clicked.connect(self.clear_detections)
        row.addWidget(self._clear_btn)
        row.addStretch(1)
        v.addLayout(row)

        opts = QHBoxLayout()
        opts.setSpacing(s(6))
        self._restrict_well = QCheckBox("Only inside the well")
        self._restrict_well.setChecked(True)
        self._restrict_well.setToolTip(
            "Reject anything outside the well circle. The wall / meniscus ring "
            "is the largest false-positive source and its geometry is known.")
        opts.addWidget(self._restrict_well)
        opts.addStretch(1)
        v.addLayout(opts)
        return frame

    def _build_table(self) -> QTableWidget:
        self._table = QTableWidget(0, 6, self)
        self._table.setHorizontalHeaderLabels(
            ["✓", "#", "Ø µm", "src", "conf", "⚠"])
        self._table.setSelectionBehavior(
            QAbstractItemView.SelectionBehavior.SelectRows)
        self._table.setSelectionMode(
            QAbstractItemView.SelectionMode.SingleSelection)
        self._table.setEditTriggers(
            QAbstractItemView.EditTrigger.NoEditTriggers)
        self._table.verticalHeader().setVisible(False)
        self._table.setSortingEnabled(True)
        self._table.setStyleSheet(
            f"QTableWidget {{ background-color: {COLORS['surface0']};"
            f" color: {COLORS['text']}; font-size: {sf(9)}pt; }}")
        hh = self._table.horizontalHeader()
        hh.setSectionResizeMode(QHeaderView.ResizeMode.ResizeToContents)
        hh.setSectionResizeMode(_COL_WARN, QHeaderView.ResizeMode.Stretch)
        self._table.itemSelectionChanged.connect(self._on_row_selected)
        self._table.itemChanged.connect(self._on_item_changed)
        self._table.itemDoubleClicked.connect(
            lambda _i: self._on_edit_diameter())
        return self._table

    def _build_row_actions(self) -> QFrame:
        frame = QFrame(self)
        row = QHBoxLayout(frame)
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(s(6))
        self._sort_asc = QPushButton("Ø ↑")
        self._sort_asc.setToolTip("Sort smallest first")
        self._sort_asc.clicked.connect(lambda: self._sort_by_size(True))
        row.addWidget(self._sort_asc)
        self._sort_desc = QPushButton("Ø ↓")
        self._sort_desc.setToolTip("Sort largest first")
        self._sort_desc.clicked.connect(lambda: self._sort_by_size(False))
        row.addWidget(self._sort_desc)
        self._goto_btn = QPushButton("Go to")
        self._goto_btn.setToolTip(
            "Retract the needle to safe Z, then travel to this spheroid. "
            "Confirm it on the live view before picking.")
        self._goto_btn.clicked.connect(self._on_goto)
        row.addWidget(self._goto_btn)
        self._edit_btn = QPushButton("Edit Ø…")
        self._edit_btn.clicked.connect(self._on_edit_diameter)
        row.addWidget(self._edit_btn)
        self._del_btn = QPushButton("Delete")
        self._del_btn.clicked.connect(self._on_delete)
        row.addWidget(self._del_btn)
        row.addStretch(1)
        return frame

    def _build_transfer_row(self) -> QFrame:
        frame = QFrame(self)
        v = QVBoxLayout(frame)
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(s(4))

        row = QHBoxLayout()
        row.setSpacing(s(6))
        self._check_all = QPushButton("Tick all")
        self._check_all.clicked.connect(lambda: self._set_all_enabled(True))
        row.addWidget(self._check_all)
        self._check_none = QPushButton("Untick all")
        self._check_none.clicked.connect(lambda: self._set_all_enabled(False))
        row.addWidget(self._check_none)
        self._crop_btn = QPushButton("Save training crop")
        self._crop_btn.setToolTip(
            "Bank a cropped image of this spheroid for future detector "
            "training. Requires the stage to be on it.")
        self._crop_btn.clicked.connect(self._on_save_crop)
        row.addWidget(self._crop_btn)
        row.addStretch(1)
        v.addLayout(row)

        self._transfer_btn = QPushButton("→ Transfer ticked to picks")
        self._transfer_btn.setToolTip(
            "Copy the ticked spheroids (position + measured Ø) into the pick "
            "list on the Pick & Place tab.")
        self._transfer_btn.clicked.connect(self._on_transfer)
        v.addWidget(self._transfer_btn)
        return frame

    # ── host wiring ───────────────────────────────────────────────

    def set_fit_badge_provider(self, fn) -> None:
        """``fn(diameter_um) -> (badge, message)`` for the needle-fit column.

        Supplied by the host page so the 1.5× rule has ONE owner (the workflow
        settings) rather than a duplicate spin box here.
        """
        self._fit_badge_fn = fn

    def set_mosaic_context(self, context: Optional[dict]) -> None:
        """Adopt a well's mosaic geometry (or None when it has no mosaic).

        A new well invalidates every detection: the coordinates were derived from
        the previous well's extent, so keeping them would be a silent
        mis-mapping.
        """
        prev_well = (self._context or {}).get("well")
        self._context = context
        well = (context or {}).get("well")
        if well != prev_well:
            self.clear_detections()
        self._refresh_channels()
        self._refresh_provenance_note()
        self._refresh_buttons()
        if context is None:
            self._status.setText(
                "No mosaic for this well yet — scan it, or pick a well that "
                "already has one.")
        elif context.get("scale_warning"):
            self._status.setText(f"⚠ {context['scale_warning']}")
        else:
            chans = ", ".join(context.get("channels") or [])
            self._status.setText(
                f"Mosaic ready for {well} ({chans}) — set the size range and "
                f"press Detect.")

    def _refresh_channels(self):
        ctx = self._context or {}
        chans = list(ctx.get("channels") or [])
        current = self._channel.currentText()
        self._channel.blockSignals(True)
        try:
            self._channel.clear()
            self._channel.addItems(chans)
            if current in chans:
                self._channel.setCurrentText(current)
        finally:
            self._channel.blockSignals(False)

    def _refresh_provenance_note(self):
        ctx = self._context
        if ctx is None or ctx.get("has_shift"):
            self._provenance_note.setVisible(False)
            return
        # Bound the error in the operator's own units so the refusal is
        # actionable rather than mysterious.
        eff = float(ctx.get("um_per_px") or 0.0)
        image = ctx.get("image")
        bound = ""
        if eff > 0 and image is not None:
            try:
                fov_w = float(image.shape[1]) * eff
                bound = f" (up to about ±{0.2 * fov_w:.0f} µm)"
            except Exception:
                bound = ""
        self._provenance_note.setText(
            "This saved mosaic predates registration-shift recording, so its "
            f"pixel→stage mapping may be off{bound}. Diameters are still "
            "trustworthy; re-scan the well to travel to or transfer these "
            "spheroids.")
        self._provenance_note.setVisible(True)

    def can_command_motion(self) -> bool:
        """True only when this mosaic's pixel→stage mapping is trustworthy."""
        return bool(self._context and self._context.get("has_shift")
                    and self._context.get("mosaic_scale"))

    # ── detection ─────────────────────────────────────────────────

    def _on_detect(self):
        ctx = self._context
        if ctx is None:
            self._status.setText("No mosaic loaded for this well.")
            return
        if self._worker is not None and self._worker.isRunning():
            return
        channel = self._channel.currentText() or None
        if channel and channel != ctx.get("channel"):
            # Re-resolve so the image, extent and scale all come from the SAME
            # channel — they are only mutually consistent per channel.
            fresh = self._reresolve(channel)
            if fresh is not None:
                self._context = ctx = fresh
                self._refresh_provenance_note()
        refusal = sd.refuse_reason(
            ctx.get("mosaic_scale") or 0.0,
            self._min_d.value(), self._max_d.value())
        if refusal:
            self._status.setText(f"⚠ {refusal}")
            return

        kwargs = dict(
            min_diameter_um=float(self._min_d.value()),
            max_diameter_um=float(self._max_d.value()),
            global_shift=ctx.get("shift_um") or (0.0, 0.0),
        )
        if self._restrict_well.isChecked():
            kwargs["well_center_um"] = ctx.get("well_center_um")
            kwargs["well_radius_um"] = ctx.get("well_radius_um")
        self._status.setText("Detecting…")
        self._detect_btn.setEnabled(False)
        self._worker = _DetectWorker(
            ctx["image"], ctx["extent_um"], ctx["mosaic_scale"], kwargs)
        self._worker.done.connect(self._on_detect_done)
        self._worker.failed.connect(self._on_detect_failed)
        self._worker.start()

    def _reresolve(self, channel: str) -> Optional[dict]:
        fn = self._context_provider
        if callable(fn):
            try:
                return fn(channel)
            except Exception as exc:
                logger.debug("context re-resolve failed: %s", exc)
        return None

    def set_context_provider(self, fn) -> None:
        """``fn(channel) -> context dict`` so a channel switch re-resolves."""
        self._context_provider = fn

    def _on_detect_done(self, report):
        # `done` is emitted from inside run() and the worker has no Qt parent,
        # so clearing this attribute is what frees it — see
        # gui/worker_retirement.py. The window is narrower here than in the
        # mosaic workers (the emit is run()'s last statement) but the failure
        # mode is the same hard abort.
        retire_worker(self._worker)
        self._worker = None
        self._detect_btn.setEnabled(True)
        manual = [d for d in self._dets if d.source == "user"]
        self._dets = list(report.detections) + manual
        self._enabled_ids = {d.det_id for d in self._dets}
        self._rebuild_table()
        self._status.setText(report.summary())
        self.detections_changed.emit()

    def _on_detect_failed(self, msg: str):
        retire_worker(self._worker)
        self._worker = None
        self._detect_btn.setEnabled(True)
        self._status.setText(f"Detection failed: {msg}")

    def clear_detections(self):
        self._dets = []
        self._enabled_ids = set()
        self._next_manual = 1
        self._rebuild_table()
        self.detections_changed.emit()

    def is_detecting(self) -> bool:
        return self._worker is not None and self._worker.isRunning()

    # ── manual add / edit ─────────────────────────────────────────

    def add_manual(self, cx_px: float, cy_px: float, r_px: float) -> Optional[str]:
        """Add a hand-placed spheroid from mosaic pixels. Returns its id.

        First-class, not a fallback: on some channels detection legitimately
        finds nothing, and the workflow has to stay fully usable at zero
        detections.
        """
        ctx = self._context
        if ctx is None or not ctx.get("mosaic_scale"):
            return None
        det_id = f"M{self._next_manual:03d}"
        self._next_manual += 1
        det = sd.manual_detection(
            (cx_px, cy_px), r_px, ctx["extent_um"], ctx["mosaic_scale"],
            ctx.get("shift_um") or (0.0, 0.0), det_id=det_id)
        self._dets.append(det)
        self._enabled_ids.add(det_id)
        self._rebuild_table()
        self.detections_changed.emit()
        return det_id

    def apply_radius_px(self, det_id: str, r_px: float, *, commit: bool = True):
        """An operator's redrawn radius from the mosaic (px → µm)."""
        ctx = self._context
        det = self.detection(det_id)
        if det is None or ctx is None or not ctx.get("mosaic_scale"):
            return
        sd.resize_detection(det, r_px, ctx["mosaic_scale"])
        if commit:
            self._rebuild_table()
            self.detections_changed.emit()
        else:
            self._update_row_diameter(det)

    def apply_center_px(self, det_id: str, cx_px: float, cy_px: float):
        """An operator's dragged centre from the mosaic (px → stage µm)."""
        ctx = self._context
        det = self.detection(det_id)
        if det is None or ctx is None or not ctx.get("mosaic_scale"):
            return
        sd.recentre_detection(det, (cx_px, cy_px), ctx["extent_um"],
                              ctx["mosaic_scale"],
                              ctx.get("shift_um") or (0.0, 0.0))
        self._rebuild_table()
        self.detections_changed.emit()

    def _on_edit_diameter(self):
        det = self.selected_detection()
        if det is None:
            return
        from PySide6.QtWidgets import QInputDialog
        value, ok = QInputDialog.getDouble(
            self, f"Edit {det.det_id}", "Diameter (µm):",
            float(det.diameter_um), 1.0, 5000.0, 1)
        if not ok:
            return
        ctx = self._context
        det.diameter_um = float(value)
        det.user_edited = True
        if ctx and ctx.get("mosaic_scale"):
            det.radius_px = sd.radius_px_for_diameter_um(
                value, ctx["mosaic_scale"])
        self._rebuild_table()
        self.detections_changed.emit()

    def _on_delete(self):
        det = self.selected_detection()
        if det is None:
            return
        self._dets = [d for d in self._dets if d.det_id != det.det_id]
        self._enabled_ids.discard(det.det_id)
        self._rebuild_table()
        self.detections_changed.emit()

    # ── table ─────────────────────────────────────────────────────

    def _rebuild_table(self):
        keep = self.selected_id()
        self._table.setSortingEnabled(False)
        self._table.blockSignals(True)
        try:
            self._table.setRowCount(0)
            for det in self._dets:
                self._append_row(det)
        finally:
            self._table.blockSignals(False)
            self._table.setSortingEnabled(True)
        if keep:
            self.select_detection(keep)
        self._refresh_buttons()

    def _append_row(self, det):
        row = self._table.rowCount()
        self._table.insertRow(row)

        on = QTableWidgetItem()
        on.setFlags(Qt.ItemFlag.ItemIsUserCheckable
                    | Qt.ItemFlag.ItemIsEnabled | Qt.ItemFlag.ItemIsSelectable)
        on.setCheckState(Qt.CheckState.Checked
                         if det.det_id in self._enabled_ids
                         else Qt.CheckState.Unchecked)
        on.setData(Qt.ItemDataRole.UserRole, det.det_id)
        self._table.setItem(row, _COL_ON, on)

        ident = QTableWidgetItem(det.det_id)
        ident.setData(Qt.ItemDataRole.UserRole, det.det_id)
        self._table.setItem(row, _COL_ID, ident)

        d_item = QTableWidgetItem()
        # Numeric sort value, so 1000 does not sort before 200.
        d_item.setData(Qt.ItemDataRole.EditRole, float(det.diameter_um))
        d_item.setData(Qt.ItemDataRole.UserRole, det.det_id)
        self._table.setItem(row, _COL_D, d_item)

        src = "man" if det.source == "user" else ("edit" if det.user_edited
                                                 else "auto")
        self._table.setItem(row, _COL_SRC, QTableWidgetItem(src))

        conf = QTableWidgetItem()
        conf.setData(Qt.ItemDataRole.EditRole, round(float(det.confidence), 2))
        self._table.setItem(row, _COL_CONF, conf)

        badge, message = ("", "")
        if callable(self._fit_badge_fn):
            try:
                badge, message = self._fit_badge_fn(det.diameter_um)
            except Exception:
                badge, message = ("", "")
        warn = QTableWidgetItem(badge)
        if badge:
            warn.setForeground(QColor(COLORS["peach"]))
            warn.setToolTip(message)
        self._table.setItem(row, _COL_WARN, warn)

    def _update_row_diameter(self, det):
        """Cheap in-place Ø update for a live drag (no full rebuild)."""
        for row in range(self._table.rowCount()):
            item = self._table.item(row, _COL_D)
            if item is None:
                continue
            if item.data(Qt.ItemDataRole.UserRole) == det.det_id:
                self._table.blockSignals(True)
                try:
                    item.setData(Qt.ItemDataRole.EditRole,
                                 float(det.diameter_um))
                finally:
                    self._table.blockSignals(False)
                return

    def _on_item_changed(self, item):
        if item.column() != _COL_ON:
            return
        det_id = item.data(Qt.ItemDataRole.UserRole)
        if not det_id:
            return
        if item.checkState() == Qt.CheckState.Checked:
            self._enabled_ids.add(str(det_id))
        else:
            self._enabled_ids.discard(str(det_id))
        self._refresh_buttons()

    def _on_row_selected(self):
        self.selection_changed.emit(self.selected_id() or "")
        self._refresh_buttons()

    def _sort_by_size(self, ascending: bool):
        self._table.sortItems(
            _COL_D, Qt.SortOrder.AscendingOrder if ascending
            else Qt.SortOrder.DescendingOrder)

    # ── selection / queries ───────────────────────────────────────

    def selected_id(self) -> Optional[str]:
        rows = {i.row() for i in self._table.selectedIndexes()}
        if len(rows) != 1:
            return None
        item = self._table.item(next(iter(rows)), _COL_ID)
        return str(item.text()) if item is not None else None

    def selected_detection(self):
        return self.detection(self.selected_id() or "")

    def detection(self, det_id: str):
        for d in self._dets:
            if d.det_id == det_id:
                return d
        return None

    def detections(self) -> list:
        return list(self._dets)

    def enabled_detections(self) -> list:
        return [d for d in self._dets if d.det_id in self._enabled_ids]

    def select_detection(self, det_id: str):
        for row in range(self._table.rowCount()):
            item = self._table.item(row, _COL_ID)
            if item is not None and item.text() == det_id:
                self._table.selectRow(row)
                return

    def _set_all_enabled(self, on: bool):
        self._enabled_ids = ({d.det_id for d in self._dets} if on else set())
        self._rebuild_table()

    # ── actions ───────────────────────────────────────────────────

    def _refresh_buttons(self):
        has_ctx = self._context is not None
        det = self.selected_detection()
        motion_ok = self.can_command_motion()
        self._detect_btn.setEnabled(has_ctx and not self.is_detecting())
        self._clear_btn.setEnabled(bool(self._dets))
        self._edit_btn.setEnabled(det is not None)
        self._del_btn.setEnabled(det is not None)
        self._goto_btn.setEnabled(det is not None and motion_ok)
        self._crop_btn.setEnabled(det is not None and motion_ok)
        n_on = len(self._enabled_ids)
        self._transfer_btn.setEnabled(n_on > 0 and motion_ok)
        self._transfer_btn.setText(
            f"→ Transfer {n_on} ticked to picks" if n_on
            else "→ Transfer ticked to picks")
        for b in (self._sort_asc, self._sort_desc, self._check_all,
                  self._check_none):
            b.setEnabled(bool(self._dets))

    def _on_goto(self):
        det = self.selected_detection()
        if det is None or not self.can_command_motion():
            return
        self.goto_requested.emit(float(det.center_um[0]), float(det.center_um[1]))

    def _on_transfer(self):
        dets = self.enabled_detections()
        if not dets or not self.can_command_motion():
            return
        self.transfer_requested.emit([
            {"x_um": float(d.center_um[0]), "y_um": float(d.center_um[1]),
             "diameter_um": float(d.diameter_um), "det_id": d.det_id}
            for d in dets])

    def _on_save_crop(self):
        det = self.selected_detection()
        if det is None:
            return
        self.crop_requested.emit(
            float(det.center_um[0]), float(det.center_um[1]),
            float(det.diameter_um), det.det_id)

    def set_status(self, text: str):
        self._status.setText(text)

    def refresh_badges(self):
        """Re-evaluate the needle-fit column (the clearance setting changed)."""
        self._rebuild_table()

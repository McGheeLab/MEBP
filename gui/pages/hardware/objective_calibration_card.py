"""
objective_calibration_card.py — Hardware Setup → Cameras Section C.

Adds the "Objective Calibration Setup" card to the Cameras sub-page.
Users define their own microscope objectives (no prepopulated list)
and measure the real µm/px for each with the stage-motion
``PixelCalibrationDialog`` — the same workflow the needle cameras use
(move the stage a known distance, correlate the resulting image
displacement). The active microscope camera is selected in the
Microscope Camera Setup section (Section A); this card only reads that
assignment from the HardwareConfig.

Responsibilities:
- Add / remove user-defined objectives (persisted via
  ``ObjectiveCalibrationStore``).
- Track which objective the user has installed under the scope
  (``CameraConfig.current_objective_name``); pushing the stored µm/px
  (and rotation) into ``CameraManager`` on the swap so the live pipeline
  immediately reflects the new objective.
- Launch the modal stage-motion ``PixelCalibrationDialog`` for the
  selected objective and persist its measured µm/px per-objective.
"""

from __future__ import annotations

import logging
from typing import Callable, Optional

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QGroupBox, QVBoxLayout, QHBoxLayout, QLabel, QComboBox, QPushButton,
    QTableWidget, QTableWidgetItem, QHeaderView, QAbstractItemView,
    QMessageBox, QWidget, QDialog, QDialogButtonBox, QFormLayout,
    QLineEdit, QDoubleSpinBox,
)
from PySide6.QtGui import QColor

from gui.scaling import s, scaled_font_size
from gui.styles import COLORS
from gui.dialogs.pixel_calibration_dialog import PixelCalibrationDialog
from SupportClasses.HardwareConfig import CameraRole
from SupportClasses.ObjectiveCalibration import get_store as _get_store

logger = logging.getLogger(__name__)


class _AddObjectiveDialog(QDialog):
    """Tiny modal: name + nominal magnification for a new objective."""

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.setWindowTitle("Add Objective")
        self.setModal(True)
        self.setMinimumWidth(s(320))
        self.setStyleSheet(
            f"background-color: {COLORS['base']}; color: {COLORS['text']};"
        )
        layout = QVBoxLayout(self)

        instructions = QLabel(
            "Define a microscope objective. Use any name you like — "
            "the nominal magnification only seeds the spec-computed "
            "µm/px until you run a real calibration."
        )
        instructions.setWordWrap(True)
        instructions.setStyleSheet(
            f"color: {COLORS['subtext0']}; "
            f"font-size: {scaled_font_size(9)}pt;"
        )
        layout.addWidget(instructions)

        form = QFormLayout()
        form.setLabelAlignment(Qt.AlignmentFlag.AlignRight)
        self._name = QLineEdit()
        self._name.setPlaceholderText("e.g. 4x, 10x Plan-Apo, 20x dry")
        form.addRow("Name:", self._name)
        self._mag = QDoubleSpinBox()
        self._mag.setRange(0.01, 1000.0)
        self._mag.setDecimals(3)
        self._mag.setValue(4.0)
        self._mag.setSuffix("×")
        form.addRow("Nominal magnification:", self._mag)
        layout.addLayout(form)

        btns = QDialogButtonBox(
            QDialogButtonBox.StandardButton.Ok
            | QDialogButtonBox.StandardButton.Cancel
        )
        btns.accepted.connect(self.accept)
        btns.rejected.connect(self.reject)
        layout.addWidget(btns)

    def values(self) -> tuple[str, float]:
        return self._name.text().strip(), float(self._mag.value())


class ObjectiveCalibrationCard(QGroupBox):
    """Cameras sub-page card for objective calibration (Section C)."""

    calibration_changed = Signal()
    # (cam_idx, um_per_px) — Cameras sub-page wires this into
    # ``HardwareSetupPage.set_calibrated_um_per_px`` so the per-slot
    # override spinbox in Needle Cameras Setup stays consistent.
    um_per_px_committed = Signal(int, float)

    def __init__(
        self,
        camera_manager,
        config_getter: Callable[[], object],
        parent: Optional[QWidget] = None,
        controller_getter: Optional[Callable[[], object]] = None,
        settings_getter: Optional[Callable[[], object]] = None,
    ):
        super().__init__("Mosaic && Camera Calibration", parent)
        self._camera_manager = camera_manager
        self._config_getter = config_getter
        # v7.5.x: resolved lazily — the StageController may arrive after the
        # card is built. Used to drive the stage-motion µm/px dialog.
        self._controller_getter = controller_getter or (lambda: None)
        # v7.5.x: the app Settings, for the shared ``mosaic_scan`` section (tile
        # overlap lives with the SCAN, not per objective). Optional — absent just
        # means the confirm step seeds overlap from the defaults and cannot
        # persist a change to it.
        self._settings_getter = settings_getter or (lambda: None)
        self._store = _get_store()
        self._loading = False

        self.setStyleSheet(self._group_style())
        self._build_ui()
        self._wire_signals()
        self._refresh_visible_state()

    # ── Style ─────────────────────────────────────────────────────

    def _group_style(self) -> str:
        return (
            f"QGroupBox {{ border: 1px solid {COLORS['surface1']}; "
            f"border-radius: {s(8)}px; margin-top: {s(10)}px; "
            f"padding-top: {s(18)}px; color: {COLORS['text']}; "
            f"font-size: {scaled_font_size(10)}pt; }}"
            f"QGroupBox::title {{ subcontrol-position: top left; "
            f"padding: 2px 8px; }}"
        )

    # ── UI ────────────────────────────────────────────────────────

    def _build_ui(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(s(14), s(20), s(14), s(14))
        root.setSpacing(s(12))

        # Intro / context line — always visible.
        intro = QLabel(
            "Define your microscope objectives below, then measure the "
            "real µm/px for each one by moving the stage — the same "
            "calibration the needle cameras use."
        )
        intro.setWordWrap(True)
        intro.setStyleSheet(
            f"color: {COLORS.get('subtext0', '#a6adc8')}; "
            f"font-size: {scaled_font_size(9)}pt;"
        )
        root.addWidget(intro)

        # Helper banner — shown when no microscope camera is assigned.
        self._lbl_helper = QLabel(
            "↑ Assign a microscope camera in the Detection & Assignment "
            "section above to enable objective calibration."
        )
        self._lbl_helper.setWordWrap(True)
        self._lbl_helper.setStyleSheet(
            f"color: {COLORS['subtext0']}; "
            f"font-size: {scaled_font_size(9)}pt; "
            f"padding: {s(8)}px; "
            f"background-color: rgba(166, 173, 200, 18); "
            f"border-radius: {s(6)}px; "
            f"font-style: italic;"
        )
        self._lbl_helper.setAlignment(Qt.AlignCenter)
        root.addWidget(self._lbl_helper)

        # ── Active block ──────────────────────────────────────────
        self._active = QWidget()
        active_lay = QVBoxLayout(self._active)
        active_lay.setSpacing(s(10))
        active_lay.setContentsMargins(0, 0, 0, 0)

        # Empty-library banner — shown when there are no objectives.
        self._lbl_empty = QLabel(
            "No objectives defined yet. Click <b>Add Objective…</b> "
            "below to create your first — each entry is a name plus a "
            "nominal magnification. Real µm/px is captured per-objective "
            "by moving the stage (same as the needle cameras)."
        )
        self._lbl_empty.setWordWrap(True)
        self._lbl_empty.setStyleSheet(
            f"color: {COLORS['yellow']}; "
            f"font-size: {scaled_font_size(9)}pt; "
            f"padding: {s(10)}px; "
            f"background-color: rgba(249, 226, 175, 18); "
            f"border: 1px solid rgba(249, 226, 175, 60); "
            f"border-radius: {s(6)}px;"
        )
        active_lay.addWidget(self._lbl_empty)

        # Row: currently-installed objective.
        obj_row = QHBoxLayout()
        obj_row.setSpacing(s(8))
        installed_lbl = QLabel("Currently installed:")
        installed_lbl.setStyleSheet(
            f"color: {COLORS.get('text', '#cdd6f4')}; font-weight: 600;"
        )
        obj_row.addWidget(installed_lbl)
        self._cmb_objective = QComboBox()
        self._cmb_objective.setMinimumWidth(s(180))
        obj_row.addWidget(self._cmb_objective)
        self._lbl_obj_note = QLabel("")
        self._lbl_obj_note.setStyleSheet(
            f"color: {COLORS['subtext0']}; "
            f"font-size: {scaled_font_size(9)}pt;"
        )
        obj_row.addWidget(self._lbl_obj_note, stretch=1)
        active_lay.addLayout(obj_row)

        # v7.5.x: camera-vs-stage orientation readout. The camera can be mounted
        # rotated relative to the stage axes (≈180°, but "not exactly"); the
        # measured angle is applied to the live-view click→stage mapping (see
        # CameraManager.pixel_to_stage_offset) so re-anchor / well-fit clicks land
        # in the correct XY direction. Objective µm/px calibration measures this
        # as a side effect; "Calibrate orientation…" measures it on its own.
        orient_row = QHBoxLayout()
        orient_row.setSpacing(s(8))
        self._lbl_orient = QLabel("Camera rotation vs stage: not calibrated")
        self._lbl_orient.setStyleSheet(
            f"color: {COLORS['subtext0']}; "
            f"font-size: {scaled_font_size(9)}pt;"
        )
        orient_row.addWidget(self._lbl_orient, stretch=1)
        # v7.5.x: hidden — rotation is measured by "Mosaic & Camera Calibration…"
        # together with the mirror, axis directions and µm/px (they are one 2x2
        # relationship, and measuring them separately is how they drifted apart).
        # Kept as a widget so its handler and the existing tests still work.
        self._btn_orient = QPushButton("Calibrate orientation…")
        self._btn_orient.setVisible(False)
        orient_row.addWidget(self._btn_orient)
        active_lay.addLayout(orient_row)

        # Library + status table.
        self._table = QTableWidget(0, 4)
        self._table.setHorizontalHeaderLabels(
            ["Objective", "Nominal", "Calibrated µm/px", "Date"]
        )
        self._table.verticalHeader().setVisible(False)
        self._table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self._table.setSelectionBehavior(
            QAbstractItemView.SelectionBehavior.SelectRows
        )
        self._table.setSelectionMode(
            QAbstractItemView.SelectionMode.SingleSelection
        )
        self._table.horizontalHeader().setSectionResizeMode(
            QHeaderView.ResizeMode.Stretch
        )
        self._table.setMinimumHeight(s(160))
        self._table.setShowGrid(False)
        self._table.setAlternatingRowColors(True)
        self._table.setStyleSheet(
            f"QTableWidget {{"
            f"  background-color: rgba(205, 214, 244, 8);"
            f"  alternate-background-color: rgba(205, 214, 244, 14);"
            f"  border: 1px solid {COLORS.get('surface1', '#45475a')};"
            f"  border-radius: {s(6)}px;"
            f"  gridline-color: transparent;"
            f"}}"
            f"QHeaderView::section {{"
            f"  background-color: rgba(137, 180, 250, 20);"
            f"  color: {COLORS.get('text', '#cdd6f4')};"
            f"  padding: {s(6)}px;"
            f"  border: none;"
            f"  font-weight: 600;"
            f"}}"
            f"QTableWidget::item {{ padding: {s(6)}px; }}"
            f"QTableWidget::item:selected {{"
            f"  background-color: rgba(137, 180, 250, 40);"
            f"  color: {COLORS.get('text', '#cdd6f4')};"
            f"}}"
        )
        active_lay.addWidget(self._table)

        # Buttons: library actions on the left, calibration on the right.
        btn_row = QHBoxLayout()
        btn_row.setSpacing(s(8))
        self._btn_add = QPushButton("Add Objective…")
        self._btn_add.setObjectName("accentBtn")
        self._btn_remove = QPushButton("Remove Selected")
        self._btn_remove.setObjectName("dangerBtn")
        self._btn_remove.setEnabled(False)
        # v7.5.x: THE one mosaic/camera calibration entry point. One stage-motion
        # measurement covers all five ways a mosaic goes wrong — mirror, stage-vs-
        # pixel axis direction, rotation about +Z, pixel size at the real capture
        # resolution — then the confirm step adds tile overlap and the whole-mosaic
        # output rotation, and gates the commit on a test mosaic.
        self._btn_autocal = QPushButton("Mosaic && Camera Calibration…")
        self._btn_autocal.setObjectName("accentBtn")
        self._btn_autocal.setToolTip(
            "The one place camera geometry is calibrated. Moves the stage in X "
            "then Y and tracks a feature to MEASURE, together:\n"
            "  • whether the camera is mirrored\n"
            "  • which way stage +X / +Y run across the image\n"
            "  • the camera's rotation about the Z axis\n"
            "  • µm/px, stamped with the real captured resolution\n"
            "then lets you set the tile overlap and the mosaic's output rotation "
            "and confirm with a test mosaic before anything is saved.\n\n"
            "The result is used by every mosaic (full plate, rosette, "
            "fluorescence) and every microscope live view.")
        self._btn_autocal.setEnabled(False)
        # v7.5.x: "Calibrate Selected…" (µm/px only) and "Calibrate orientation…"
        # are RETIRED as separate surfaces — they measured subsets of what the one
        # calibration above measures, wrote through different commit paths, and
        # were a large part of why the operator had "multiple surfaces for
        # calibrating mosaics". They remain as hidden widgets so the existing
        # handlers, table gating and tests keep working unchanged.
        self._btn_calibrate = QPushButton("Calibrate Selected…")
        self._btn_calibrate.setObjectName("accentBtn")
        self._btn_calibrate.setEnabled(False)
        self._btn_calibrate.setVisible(False)
        self._btn_clear = QPushButton("Clear Calibration")
        self._btn_clear.setEnabled(False)
        btn_row.addWidget(self._btn_add)
        btn_row.addWidget(self._btn_remove)
        btn_row.addStretch(1)
        btn_row.addWidget(self._btn_autocal)
        btn_row.addWidget(self._btn_calibrate)
        btn_row.addWidget(self._btn_clear)
        active_lay.addLayout(btn_row)

        root.addWidget(self._active)

    def _wire_signals(self) -> None:
        self._cmb_objective.currentTextChanged.connect(self._on_objective_changed)
        self._table.itemSelectionChanged.connect(self._refresh_button_state)
        self._btn_add.clicked.connect(self._on_add_clicked)
        self._btn_remove.clicked.connect(self._on_remove_clicked)
        self._btn_calibrate.clicked.connect(self._on_calibrate_clicked)
        self._btn_autocal.clicked.connect(self._on_autocal_scale_fov_clicked)
        self._btn_orient.clicked.connect(self._on_calibrate_orientation_clicked)
        self._btn_clear.clicked.connect(self._on_clear_clicked)

    # ── Public API ────────────────────────────────────────────────

    def set_camera_manager(self, manager) -> None:
        self._camera_manager = manager

    def apply_config(self, config) -> None:
        """Restore card state from a HardwareConfig snapshot."""
        self._loading = True
        try:
            current_obj = getattr(config.camera_config, "current_objective_name", None)
            self._reload_objective_combo(preferred=current_obj)
            self._refresh_visible_state()
            self._refresh_table()
        finally:
            self._loading = False

    def write_to_config(self, config) -> None:
        """Push card state into a HardwareConfig snapshot."""
        config.camera_config.current_objective_name = (
            self._cmb_objective.currentText() or None
        )

    def selected_objective_name(self) -> str:
        return self._cmb_objective.currentText()

    # ── Visibility / refresh ──────────────────────────────────────

    def _refresh_visible_state(self) -> None:
        config = self._config_getter()
        has_microscope = False
        if config is not None:
            has_microscope = config.camera_for_role(CameraRole.MICROSCOPE) is not None
        self._active.setVisible(has_microscope)
        self._lbl_helper.setVisible(not has_microscope)
        has_objectives = bool(self._store.objective_names())
        self._lbl_empty.setVisible(has_microscope and not has_objectives)
        self._cmb_objective.setEnabled(has_objectives)
        self._refresh_button_state()
        self._refresh_orientation_readout()
        if has_microscope:
            self._refresh_objective_note()

    def _reload_objective_combo(self, preferred: Optional[str] = None) -> None:
        self._cmb_objective.blockSignals(True)
        self._cmb_objective.clear()
        for name in self._store.objective_names():
            nominal = self._store.nominal_magnification(name) or 0.0
            self._cmb_objective.addItem(f"{name} ({nominal:g}×)", name)
        if preferred:
            idx = self._cmb_objective.findData(preferred)
            if idx >= 0:
                self._cmb_objective.setCurrentIndex(idx)
        self._cmb_objective.blockSignals(False)

    # ── Currently installed ───────────────────────────────────────

    def _current_objective_name(self) -> str:
        return self._cmb_objective.currentData() or ""

    def _on_objective_changed(self, _label: str) -> None:
        if self._loading:
            return
        name = self._current_objective_name()
        config = self._config_getter()
        if config is not None:
            config.camera_config.current_objective_name = name or None
        self._refresh_objective_note()
        self._push_stored_um_per_px_to_manager(name)
        self._refresh_orientation_readout()
        self._refresh_table()
        self.calibration_changed.emit()

    def _push_stored_um_per_px_to_manager(self, objective_name: str) -> None:
        cam_idx = self._microscope_idx()
        if cam_idx is None:
            return
        cam_key = self._camera_key()
        if not cam_key or not objective_name:
            return
        cal = self._store.get_calibration(cam_key, objective_name)
        if not cal:
            return
        um_per_px = float(cal["measured_um_per_px"])
        rotation_deg = cal.get("rotation_deg")
        cal_resolution = cal.get("resolution")
        if self._camera_manager is not None:
            try:
                self._camera_manager.set_um_per_px(
                    cam_idx, um_per_px, resolution=cal_resolution)
                if rotation_deg is not None:
                    self._camera_manager.set_rotation_deg(
                        cam_idx, float(rotation_deg))
            except Exception as exc:
                logger.debug(f"ObjectiveCalibrationCard: set_um_per_px — {exc}")
        self.um_per_px_committed.emit(cam_idx, um_per_px)

    def _refresh_objective_note(self) -> None:
        cam_key = self._camera_key()
        name = self._current_objective_name()
        if not cam_key or not name:
            self._lbl_obj_note.setText("")
            return
        cal = self._store.get_calibration(cam_key, name)
        if cal is None:
            self._lbl_obj_note.setText(
                "No calibration — using spec-computed µm/px"
            )
            self._lbl_obj_note.setStyleSheet(
                f"color: {COLORS['yellow']}; "
                f"font-size: {scaled_font_size(9)}pt;"
            )
        else:
            self._lbl_obj_note.setText(
                f"Calibrated: {cal['measured_um_per_px']:.4f} µm/px"
            )
            self._lbl_obj_note.setStyleSheet(
                f"color: {COLORS['green']}; "
                f"font-size: {scaled_font_size(9)}pt;"
            )

    # ── Table refresh ─────────────────────────────────────────────

    def _refresh_table(self) -> None:
        config = self._config_getter()
        cam_key = self._camera_key()
        names = self._store.objective_names()
        self._table.setRowCount(len(names))
        active_res = (
            tuple(config.camera_config.active_resolution)
            if config is not None
            else (0, 0)
        )
        cals = self._store.all_calibrations_for_camera(cam_key) if cam_key else {}
        current = self._current_objective_name()
        highlight = QColor(COLORS.get("surface1", "#45475a"))

        for row, name in enumerate(names):
            nominal = self._store.nominal_magnification(name)
            cal = cals.get(name)
            items = [
                QTableWidgetItem(name),
                QTableWidgetItem(f"{nominal:g}×" if nominal is not None else "—"),
            ]
            if cal is None:
                items.append(QTableWidgetItem("—"))
                items.append(QTableWidgetItem("—"))
            else:
                items.append(QTableWidgetItem(f"{cal['measured_um_per_px']:.4f}"))
                date_text = cal.get("date", "?")
                stored_res = tuple(cal.get("resolution") or (0, 0))
                if (
                    active_res != (0, 0)
                    and stored_res != (0, 0)
                    and stored_res != active_res
                ):
                    date_text += " ⚠"
                    date_item = QTableWidgetItem(date_text)
                    date_item.setToolTip(
                        f"Calibrated at {stored_res[0]}×{stored_res[1]}; "
                        f"active resolution is {active_res[0]}×{active_res[1]}"
                    )
                    items.append(date_item)
                else:
                    items.append(QTableWidgetItem(date_text))

            for col, item in enumerate(items):
                item.setFlags(item.flags() & ~Qt.ItemFlag.ItemIsEditable)
                if name == current:
                    item.setBackground(highlight)
                self._table.setItem(row, col, item)

        # Restore selection on the current objective if any.
        for row in range(self._table.rowCount()):
            if self._table.item(row, 0).text() == current:
                self._table.selectRow(row)
                break
        self._refresh_button_state()

    def _refresh_button_state(self) -> None:
        has_microscope = self._microscope_idx() is not None
        selected = self._selected_objective_name()
        has_selection = bool(selected)
        self._btn_add.setEnabled(True)
        self._btn_remove.setEnabled(has_selection)
        self._btn_calibrate.setEnabled(has_microscope and has_selection)
        self._btn_autocal.setEnabled(has_microscope and has_selection)
        # Orientation is objective-independent → only needs a microscope camera.
        self._btn_orient.setEnabled(has_microscope)
        if has_microscope and has_selection:
            cam_key = self._camera_key()
            cal = (
                self._store.get_calibration(cam_key, selected)
                if cam_key else None
            )
            self._btn_clear.setEnabled(cal is not None)
        else:
            self._btn_clear.setEnabled(False)

    # ── CRUD ──────────────────────────────────────────────────────

    def _on_add_clicked(self) -> None:
        dlg = _AddObjectiveDialog(parent=self)
        if dlg.exec() != QDialog.DialogCode.Accepted:
            return
        name, nominal = dlg.values()
        if not name:
            QMessageBox.warning(self, "Add Objective", "Name cannot be empty.")
            return
        if not self._store.add_objective(name, nominal):
            QMessageBox.warning(
                self, "Add Objective",
                f"An objective named {name!r} already exists.",
            )
            return
        # If this is the user's first objective, install it.
        if len(self._store.objective_names()) == 1:
            config = self._config_getter()
            if config is not None:
                config.camera_config.current_objective_name = name
        self._reload_objective_combo(preferred=name)
        self._refresh_visible_state()
        self._refresh_table()
        self.calibration_changed.emit()

    def _on_remove_clicked(self) -> None:
        name = self._selected_objective_name()
        if not name:
            return
        confirm = QMessageBox.question(
            self, "Remove Objective",
            f"Remove the {name!r} objective and all of its stored "
            "calibrations on every camera? This cannot be undone.",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
        )
        if confirm != QMessageBox.StandardButton.Yes:
            return
        self._store.remove_objective(name)
        config = self._config_getter()
        if (
            config is not None
            and getattr(config.camera_config, "current_objective_name", None)
                == name
        ):
            config.camera_config.current_objective_name = None
        self._reload_objective_combo(
            preferred=getattr(
                config.camera_config, "current_objective_name", None
            ) if config else None
        )
        self._refresh_visible_state()
        self._refresh_table()
        self.calibration_changed.emit()

    # ── Calibrate / Clear ─────────────────────────────────────────

    def _on_calibrate_clicked(self) -> None:
        cam_idx = self._microscope_idx()
        if cam_idx is None:
            return
        cam_key = self._camera_key()
        if not cam_key:
            QMessageBox.warning(
                self, "Calibrate",
                "No camera identity is set for the microscope slot. "
                "Pick a camera model on the Microscope Camera Setup card first.",
            )
            return
        objective = self._selected_objective_name() or self._current_objective_name()
        if not objective:
            return

        # v7.5.x: objective µm/px is now measured the same way as the needle
        # cameras — move the stage a known distance and correlate the image
        # displacement ("flow") of the plate under the scope. Requires a
        # running microscope camera and a connected stage controller.
        controller = self._controller_getter()
        if controller is None or not getattr(controller, "xy_stage", None):
            QMessageBox.warning(
                self, "Calibrate",
                "Stage controller not connected — objective calibration moves "
                "the stage to measure µm/px. Connect hardware first.",
            )
            return
        if self._camera_manager is None or not self._camera_manager.is_running(cam_idx):
            QMessageBox.warning(
                self, "Calibrate",
                f"Start the microscope camera (Cam {cam_idx + 1}) before "
                "calibrating so the dialog can see the live feed.",
            )
            return

        dlg = PixelCalibrationDialog(
            self._camera_manager, controller, cam_idx=cam_idx, parent=self,
        )
        if dlg.exec() != QDialog.DialogCode.Accepted:
            return
        um_per_px = dlg.result_um_per_px
        if um_per_px is None or um_per_px <= 0:
            return
        rotation_deg = dlg.result_rotation_deg

        # Persist per-objective (µm/px is objective-specific) and push live.
        # v7.5.x: stamp the camera's ACTUAL captured resolution (the real pixels
        # off the sensor), NOT config.active_resolution which can be stale — a
        # wrong stamp is exactly what made the mosaic FOV "assume a resolution".
        resolution = self._true_capture_resolution(cam_idx)
        self._store.set_calibration(
            cam_key, objective, um_per_px, resolution, rotation_deg=rotation_deg,
        )
        # A fresh µm/px supersedes any stale "Store FOV/spacing" learned value.
        self._clear_stale_mosaic_fov(cam_idx, objective)
        if self._camera_manager is not None:
            try:
                self._camera_manager.set_um_per_px(
                    cam_idx, um_per_px, resolution=resolution)
                if rotation_deg is not None:
                    self._camera_manager.set_rotation_deg(cam_idx, rotation_deg)
            except Exception as exc:
                logger.debug(f"ObjectiveCalibrationCard: push to manager — {exc}")

        if self._current_objective_name() == objective:
            self.um_per_px_committed.emit(cam_idx, um_per_px)
        self._refresh_table()
        self._refresh_objective_note()
        self._refresh_orientation_readout()
        self.calibration_changed.emit()

    def _true_capture_resolution(self, cam_idx: int) -> tuple:
        """The camera's ACTUAL captured (w, h) — the real pixels off the sensor.

        v7.5.x: µm/px must be stamped with the resolution it was measured at so
        the mosaic can rescale to whatever the live feed runs at. Reading the
        LIVE frame shape (authoritative) instead of ``config.active_resolution``
        (which can be stale/assumed) is the fix for the FOV that "assumes a
        camera resolution". Falls back to the HW settings, then the config."""
        mgr = self._camera_manager
        try:
            frame = mgr.cameras[cam_idx].get_current_frame()
            if frame is not None and getattr(frame, "shape", None):
                return (int(frame.shape[1]), int(frame.shape[0]))
        except Exception:
            pass
        try:
            if hasattr(mgr, "get_hw_settings"):
                hw = mgr.get_hw_settings(cam_idx)
                res = hw.get("resolution") if isinstance(hw, dict) else None
                if res and len(res) >= 2 and res[0] and res[1]:
                    return (int(res[0]), int(res[1]))
        except Exception:
            pass
        config = self._config_getter()
        try:
            return tuple(config.camera_config.active_resolution)
        except Exception:
            return (0, 0)

    def _confirm_calibration(self, cam_idx: int, dlg):
        """Show the final check for a fresh measurement. Returns the confirmed
        ``MosaicCalibration``, or None if the operator cancelled.

        Builds the candidate directly from the dialog's measured results — NOT by
        re-resolving from the stores, which have not been written yet. That is the
        whole point: the operator sees and confirms the new numbers before they
        can affect any mosaic or live view.
        """
        try:
            from SupportClasses.MosaicCalibration import (
                MosaicCalibration, SCAN_DEFAULTS, warnings_for)
            from gui.dialogs.mosaic_calibration_confirm_dialog import (
                MosaicCalibrationConfirmDialog)
        except Exception as exc:
            logger.warning(f"confirm dialog unavailable — {exc}")
            return None
        res = dlg.result_resolution or self._true_capture_resolution(cam_idx)
        scan = dict(self._scan_settings() or {})
        overlap_pct = float(scan.get(
            "overlap_pct", SCAN_DEFAULTS["overlap_pct"]) or
            SCAN_DEFAULTS["overlap_pct"])
        candidate = MosaicCalibration(
            um_per_px=float(dlg.result_um_per_px),
            base_um_per_px=float(dlg.result_um_per_px),
            calib_resolution=(tuple(res) if res and res[0] else None),
            live_resolution=(tuple(res) if res and res[0] else None),
            rotation_deg=float(dlg.result_rotation_deg or 0.0),
            flip_x=bool(dlg.result_flip_x),
            flip_y=bool(dlg.result_flip_y),
            overlap_frac=max(0.05, min(0.60, overlap_pct / 100.0)),
            target_px=int(scan.get("target_px", SCAN_DEFAULTS["target_px"])),
            reg_method=str(scan.get("reg_method",
                                    SCAN_DEFAULTS["reg_method"])),
            settle_ms=int(scan.get("settle_ms", SCAN_DEFAULTS["settle_ms"])),
            output_rotation_deg=self._stored_output_rotation(cam_idx),
            provenance={"um_per_px": "this measurement",
                        "rotation_deg": "this measurement"},
        )
        confirm = MosaicCalibrationConfirmDialog(
            candidate, warnings=warnings_for(candidate),
            tile_count_getter=self._estimate_plate_tiles,
            test_mosaic_runner=lambda cal: dlg.build_test_mosaic(cal),
            parent=self)
        if confirm.exec() != QDialog.DialogCode.Accepted:
            return None
        return confirm.result_calibration()

    def _scan_settings(self) -> dict:
        """The shared ``mosaic_scan`` settings section, or {}."""
        getter = getattr(self, "_settings_getter", None)
        try:
            settings = getter() if callable(getter) else None
            if settings is not None:
                return dict(settings.get_section("mosaic_scan") or {})
        except Exception as exc:
            logger.debug(f"mosaic_scan settings unavailable: {exc}")
        return {}

    def _stored_output_rotation(self, cam_idx: int) -> float:
        try:
            from SupportClasses.CameraCalibrationStore import get_store
            ident = self._camera_manager.camera_identity(cam_idx)
            if ident and ident[0]:
                return float(get_store().get_mosaic_output_rotation(ident[0]))
        except Exception:
            pass
        return 0.0

    def _estimate_plate_tiles(self, cal) -> int | None:
        """Rough full-plate tile count, so the overlap choice is PRICED.

        Raising overlap from 5% to 25% is the right call for stitch quality but it
        costs scan time; showing the count makes that a visible trade instead of a
        surprise.
        """
        try:
            sx, sy = cal.spacing_um
            if sx <= 0 or sy <= 0:
                return None
            # A standard plate footprint (~127.8 x 85.5 mm) is a good enough
            # yardstick for an order-of-magnitude count.
            import math
            return int(math.ceil(127800.0 / sx) * math.ceil(85500.0 / sy))
        except Exception:
            return None

    def _persist_mosaic_settings(self, cal, cam_idx: int) -> None:
        """Write the confirmed overlap + output rotation.

        Overlap belongs to the SCAN (shared ``mosaic_scan``); the output rotation
        is display-only and belongs to the camera identity.
        """
        getter = getattr(self, "_settings_getter", None)
        try:
            settings = getter() if callable(getter) else None
            if settings is not None:
                section = dict(settings.get_section("mosaic_scan") or {})
                section["overlap_pct"] = int(round(cal.overlap_frac * 100))
                settings.set_section("mosaic_scan", section)
                save = getattr(settings, "save", None)
                if callable(save):
                    save()
        except Exception as exc:
            logger.warning(f"overlap persist failed: {exc}")
        try:
            from SupportClasses.CameraCalibrationStore import get_store
            ident = self._camera_manager.camera_identity(cam_idx)
            if ident and ident[0]:
                get_store().set_mosaic_output_rotation(
                    ident[0], cal.output_rotation_deg,
                    name=(ident[1] if len(ident) > 1 else ""))
        except Exception as exc:
            logger.warning(f"output-rotation persist failed: {exc}")

    def _on_autocal_scale_fov_clicked(self) -> None:
        """v7.5.x: auto-calibrate µm/px AND the camera's FOV extent by tracking a
        feature across a large stage move, anchored to the ACTUAL captured
        resolution. Persists per-objective (so the mosaic picks it up) and pushes
        live, exactly like ``_on_calibrate_clicked`` but with the better
        large-baseline measurement + true resolution + FOV report."""
        cam_idx = self._microscope_idx()
        if cam_idx is None:
            return
        cam_key = self._camera_key()
        if not cam_key:
            QMessageBox.warning(
                self, "Auto-calibrate",
                "No camera identity is set for the microscope slot. Pick a "
                "camera model on the Microscope Camera Setup card first.")
            return
        objective = (self._selected_objective_name()
                     or self._current_objective_name())
        if not objective:
            return
        controller = self._controller_getter()
        if controller is None or not getattr(controller, "xy_stage", None):
            QMessageBox.warning(
                self, "Auto-calibrate",
                "Stage controller not connected — this calibration moves the "
                "stage to measure µm/px. Connect hardware first.")
            return
        if self._camera_manager is None or not self._camera_manager.is_running(cam_idx):
            QMessageBox.warning(
                self, "Auto-calibrate",
                f"Start the microscope camera (Cam {cam_idx + 1}) first.")
            return

        try:
            from gui.dialogs.scale_fov_calibration_dialog import (
                ScaleFovCalibrationDialog)
        except Exception as exc:
            logger.warning(f"Scale/FOV dialog unavailable: {exc}")
            QMessageBox.warning(self, "Auto-calibrate",
                                "Scale/FOV calibration dialog unavailable.")
            return
        # Clear any stale "Store FOV/spacing" learned value up front — BEFORE the
        # dialog — so it doesn't shadow the new measurement AND so a fresh
        # correction the operator makes in the Verify step (which writes a new
        # learned value) survives (clearing after would wipe it).
        self._clear_stale_mosaic_fov(cam_idx, objective)
        dlg = ScaleFovCalibrationDialog(
            self._camera_manager, controller, cam_idx=cam_idx,
            align_key=self._mosaic_align_key(cam_idx, objective),
            objective=objective, cam_key=cam_key,
            # v7.5.x: the real scan settings, so the test/verify mosaic builds the
            # way the actual scan will (it used to be handed an empty dict).
            scan_settings=self._scan_settings(), parent=self)
        if dlg.exec() != QDialog.DialogCode.Accepted:
            return
        um_per_px = dlg.result_um_per_px
        if um_per_px is None or um_per_px <= 0:
            return
        # v7.5.x: FINAL CHECK BEFORE COMMIT (operator: "after settings are
        # applied, there should be a final check with the new settings for
        # confirmation"). The measurement becomes a CANDIDATE; the operator
        # reviews it, sets the tile overlap and the whole-mosaic output rotation,
        # builds a test mosaic through the shared build path, and only then is
        # anything written. Cancel leaves the previous calibration byte-identical.
        confirmed = self._confirm_calibration(cam_idx, dlg)
        if confirmed is None:
            logger.info("Mosaic & camera calibration cancelled — nothing saved.")
            return
        rotation_deg = dlg.result_rotation_deg
        # v7.5.x: the FULL camera→stage orientation MEASURED from the two-axis
        # move (rotation AND handedness/mirror). A mirror can't be a rotation, so
        # a mirrored/180° microscope was invisible to the old rotation-only
        # estimate and the operator had to guess flip-X/flip-Y — landing "upside
        # down" and the full mosaic placing tiles on the wrong side. Now measured.
        flip_x = dlg.result_flip_x
        flip_y = dlg.result_flip_y
        # The dialog measured the resolution directly (frame.shape) — the real
        # captured pixels. Fall back to the live read only if absent.
        resolution = dlg.result_resolution or self._true_capture_resolution(cam_idx)

        self._store.set_calibration(
            cam_key, objective, um_per_px, resolution, rotation_deg=rotation_deg)
        # NOTE: the stale learned value was cleared BEFORE the dialog — do NOT
        # clear it here, or a correction made in the Verify step (which writes a
        # fresh learned value that should win) would be wiped.
        if self._camera_manager is not None:
            try:
                self._camera_manager.set_um_per_px(
                    cam_idx, um_per_px, resolution=resolution)
                if rotation_deg is not None:
                    self._camera_manager.set_rotation_deg(cam_idx, rotation_deg)
                # Push the measured handedness so the live view, the mosaic
                # (_orient_tile) and click-mapping all use it — ONE orientation.
                if flip_x is not None:
                    self._camera_manager.set_mirrored(cam_idx, bool(flip_x))
                sfy = getattr(self._camera_manager, "set_flip_y", None)
                if flip_y is not None and callable(sfy):
                    sfy(cam_idx, bool(flip_y))
            except Exception as exc:
                logger.debug(f"autocal: push to manager — {exc}")
        # Persist rotation + handedness per camera identity (ground truth the
        # full-plate mosaic reads; restored on slot assignment).
        try:
            from SupportClasses.CameraCalibrationStore import (
                get_store as _cam_store)
            ident = None
            cid = getattr(self._camera_manager, "camera_identity", None)
            if callable(cid):
                ident = cid(cam_idx)
            if ident and ident[0]:
                st = _cam_store()
                nm = ident[1] if len(ident) > 1 else ""
                if rotation_deg is not None:
                    st.set_rotation(ident[0], float(rotation_deg), name=nm)
                if flip_x is not None:
                    st.set_mirrored(ident[0], bool(flip_x), name=nm)
                sfy = getattr(st, "set_flip_y", None)
                if flip_y is not None and callable(sfy):
                    sfy(ident[0], bool(flip_y), name=nm)
        except Exception as exc:
            logger.debug(f"autocal: orientation persist — {exc}")

        # v7.5.x: the confirmed overlap (shared scan setting) + the display-only
        # whole-mosaic output rotation (per camera identity).
        self._persist_mosaic_settings(confirmed, cam_idx)
        if self._current_objective_name() == objective:
            self.um_per_px_committed.emit(cam_idx, um_per_px)
        self._refresh_table()
        self._refresh_objective_note()
        self._refresh_orientation_readout()
        self.calibration_changed.emit()
        fov = dlg.result_fov_um or (0.0, 0.0)
        flips = ([("flip X")] if flip_x else []) + ([("flip Y")] if flip_y else [])
        flip_txt = ", ".join(flips) if flips else "none"
        QMessageBox.information(
            self, "Auto-calibrate scale + FOV",
            f"Saved for {objective}:\n\n"
            f"µm/px = {um_per_px:.4f}  @ {resolution[0]}×{resolution[1]} px\n"
            f"Field of view = {fov[0]:.0f} × {fov[1]:.0f} µm\n"
            f"Rotation vs stage = "
            f"{('%.1f°' % rotation_deg) if rotation_deg is not None else '—'}\n"
            f"Camera axis flips (measured from stage motion) = {flip_txt}\n\n"
            "Mosaics now size their tiles from this FOV and orient them with the "
            "measured rotation + flips — so tiles land on the correct side.")

    def _on_calibrate_orientation_clicked(self) -> None:
        """Measure the camera's rotation vs the stage axes (does NOT change
        µm/px), push it live so live-view clicks map correctly, and persist it
        per camera identity so it restores when the camera is reassigned."""
        cam_idx = self._microscope_idx()
        if cam_idx is None:
            return
        controller = self._controller_getter()
        if controller is None or not getattr(controller, "xy_stage", None):
            QMessageBox.warning(
                self, "Calibrate orientation",
                "Stage controller not connected — orientation calibration moves "
                "the stage to measure the camera's rotation. Connect hardware "
                "first.",
            )
            return
        if self._camera_manager is None or not self._camera_manager.is_running(cam_idx):
            QMessageBox.warning(
                self, "Calibrate orientation",
                f"Start the microscope camera (Cam {cam_idx + 1}) before "
                "calibrating so the dialog can see the live feed.",
            )
            return

        dlg = PixelCalibrationDialog(
            self._camera_manager, controller, cam_idx=cam_idx, parent=self,
        )
        if dlg.exec() != QDialog.DialogCode.Accepted:
            return
        rotation_deg = dlg.result_rotation_deg
        if rotation_deg is None:
            QMessageBox.information(
                self, "Calibrate orientation",
                "No rotation was measured (move too small / low confidence). "
                "Try a larger stage move along a clear plate feature.",
            )
            return

        # Push live so the click→stage mapping is corrected immediately.
        try:
            self._camera_manager.set_rotation_deg(cam_idx, float(rotation_deg))
        except Exception as exc:
            logger.debug(f"orientation: push to manager — {exc}")
        # Persist per camera IDENTITY (independent of µm/px; restored on slot
        # assignment via hardware_setup._restore_calibration_for_slot).
        try:
            from SupportClasses.CameraCalibrationStore import (
                get_store as _cam_store)
            ident = None
            cam_identity = getattr(self._camera_manager, "camera_identity", None)
            if callable(cam_identity):
                ident = cam_identity(cam_idx)
            if ident and ident[0]:
                _cam_store().set_rotation(
                    ident[0], float(rotation_deg),
                    name=(ident[1] if len(ident) > 1 else ""))
        except Exception as exc:
            logger.debug(f"orientation: persist per identity — {exc}")
        # Sync the fresh rotation into every per-objective calibration and
        # refresh the readout (shared with the Hardware Setup per-slot
        # rotation calibration — see adopt_camera_rotation).
        self.adopt_camera_rotation(float(rotation_deg))
        self.calibration_changed.emit()
        QMessageBox.information(
            self, "Calibrate orientation",
            f"Camera rotation vs stage measured: {rotation_deg:.1f}°.\n\n"
            "Live-view clicks (re-anchor, well fits) now map in the corrected "
            "direction. A previously scanned mosaic is unaffected — the rotation "
            "only changes the click→stage mapping, not the stored image.",
        )

    def adopt_camera_rotation(self, rotation_deg: float) -> None:
        """v7.5.x: adopt a freshly-measured camera→stage rotation.

        Called by the card's own "Calibrate orientation…" AND by the
        per-slot rotation calibration on the Camera Detection & Assignment
        card when the calibrated slot holds the MICROSCOPE role. The mount
        rotation is a property of the CAMERA, not the objective — sync it
        into EVERY objective that has a µm/px calibration for this camera,
        so a later objective swap (_push_stored_um_per_px_to_manager pushes
        the per-objective rotation) can't push a STALE rotation back over
        this fresh measurement. Then refresh the readout.
        """
        try:
            cam_key = self._camera_key()
            if cam_key:
                cals = self._store.all_calibrations_for_camera(cam_key) or {}
                for obj, cal in cals.items():
                    if cal and cal.get("measured_um_per_px"):
                        self._store.set_calibration(
                            cam_key, obj, float(cal["measured_um_per_px"]),
                            cal.get("resolution") or (0, 0),
                            rotation_deg=float(rotation_deg))
        except Exception as exc:
            logger.debug(f"orientation: sync per-objective — {exc}")
        self._refresh_orientation_readout()

    def _refresh_orientation_readout(self) -> None:
        """Show the current camera→stage rotation (from the live manager)."""
        if getattr(self, "_lbl_orient", None) is None:
            return
        cam_idx = self._microscope_idx()
        theta = None
        if cam_idx is not None and self._camera_manager is not None:
            try:
                theta = self._camera_manager.get_rotation_deg(cam_idx)
            except Exception:
                theta = None
        if theta is None:
            self._lbl_orient.setText("Camera rotation vs stage: not calibrated")
            self._lbl_orient.setStyleSheet(
                f"color: {COLORS['subtext0']}; "
                f"font-size: {scaled_font_size(9)}pt;")
        else:
            self._lbl_orient.setText(
                f"Camera rotation vs stage: {float(theta):.1f}°")
            self._lbl_orient.setStyleSheet(
                f"color: {COLORS['green']}; "
                f"font-size: {scaled_font_size(9)}pt;")

    def _on_clear_clicked(self) -> None:
        cam_key = self._camera_key()
        objective = self._selected_objective_name()
        if not cam_key or not objective:
            return
        confirm = QMessageBox.question(
            self, "Clear Calibration",
            f"Remove the stored calibration for {objective} on "
            f"{cam_key}? The spec-computed µm/px will be used until "
            f"you recalibrate.",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
        )
        if confirm != QMessageBox.StandardButton.Yes:
            return
        self._store.clear_calibration(cam_key, objective)
        self._refresh_table()
        self._refresh_objective_note()
        self.calibration_changed.emit()

    # ── Helpers ───────────────────────────────────────────────────

    def _microscope_idx(self) -> Optional[int]:
        config = self._config_getter()
        if config is None:
            return None
        return config.camera_for_role(CameraRole.MICROSCOPE)

    def _camera_key(self) -> Optional[str]:
        """Stable key used by `ObjectiveCalibrationStore` for this slot."""
        cam_idx = self._microscope_idx()
        if cam_idx is None:
            return None
        config = self._config_getter()
        spec = getattr(config.camera_config, "camera_spec", None) if config else None
        if spec is not None and getattr(spec, "name", None):
            return str(spec.name)
        return f"camera_{cam_idx}"

    def _selected_objective_name(self) -> str:
        row = self._table.currentRow()
        if row < 0:
            return ""
        item = self._table.item(row, 0)
        return item.text() if item is not None else ""

    def _mosaic_align_key(self, cam_idx: int, objective: str) -> str:
        """The MosaicAlignmentStore key the mosaic build uses for this camera +
        objective — MUST match ``CalibrationPage._ploc_camera_objective_key``
        (``{camera_identity}|{objective}`` or ``{objective}``)."""
        ident = None
        try:
            cid = getattr(self._camera_manager, "camera_identity", None)
            if callable(cid):
                r = cid(cam_idx)
                if r:
                    ident = r[0]
        except Exception:
            ident = None
        obj = objective or "default"
        return f"{ident}|{obj}" if ident else str(obj)

    def _clear_stale_mosaic_fov(self, cam_idx: int, objective: str) -> None:
        """v7.5.x: a fresh µm/px calibration supersedes any "Store FOV/spacing"
        learned value the mosaic build PREFERS over the objective calibration.
        Clear it so the mosaic uses THIS calibration — otherwise the stale
        learned FOV shadows the recalibration (operator report: "the calibration
        did not correctly assign the values to the mosaic, it was still wrong")."""
        try:
            from SupportClasses.MosaicAlignmentStore import get_store
            get_store().clear(self._mosaic_align_key(cam_idx, objective))
            logger.info(
                "Cleared stale mosaic FOV/alignment for "
                f"'{self._mosaic_align_key(cam_idx, objective)}' — fresh "
                "calibration now drives the mosaic.")
        except Exception as exc:
            logger.debug(f"clear stale mosaic FOV skipped: {exc}")

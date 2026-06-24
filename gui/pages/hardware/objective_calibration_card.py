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
    ):
        super().__init__("Objective Calibration Setup", parent)
        self._camera_manager = camera_manager
        self._config_getter = config_getter
        # v7.5.x: resolved lazily — the StageController may arrive after the
        # card is built. Used to drive the stage-motion µm/px dialog.
        self._controller_getter = controller_getter or (lambda: None)
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
        self._btn_calibrate = QPushButton("Calibrate Selected…")
        self._btn_calibrate.setObjectName("accentBtn")
        self._btn_calibrate.setEnabled(False)
        self._btn_clear = QPushButton("Clear Calibration")
        self._btn_clear.setEnabled(False)
        btn_row.addWidget(self._btn_add)
        btn_row.addWidget(self._btn_remove)
        btn_row.addStretch(1)
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
        config = self._config_getter()
        resolution = (
            tuple(config.camera_config.active_resolution)
            if config is not None else (0, 0)
        )
        self._store.set_calibration(
            cam_key, objective, um_per_px, resolution, rotation_deg=rotation_deg,
        )
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
        self.calibration_changed.emit()

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

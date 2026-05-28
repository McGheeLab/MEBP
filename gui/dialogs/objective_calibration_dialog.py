"""
objective_calibration_dialog.py — Slide-marking objective µm/px calibration.

Modal dialog: shows the microscope camera's live feed on the left and
controls on the right. The user clicks two points on a slide marking
of known real-world distance, drags the endpoints to refine, enters
the real-world distance + unit, presses Calculate to compute µm/px,
and Accept Calibration to persist the result.

Persistence flows through `ObjectiveCalibrationStore` (keyed by camera
model + objective name) and through `CameraManager.set_um_per_px()` so
the live pipeline updates immediately. This dialog does not touch
`CameraConfig.micron_per_pixel_override` directly — that propagation
is the caller's responsibility (see `ObjectiveCalibrationCard`).

Coexists with `gui/dialogs/pixel_calibration_dialog.py` — that one
estimates µm/px via stage motion + phase correlation; this one uses a
known-distance fiducial on a slide.
"""

from __future__ import annotations

import logging
from typing import Optional

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QDialog, QHBoxLayout, QVBoxLayout, QFormLayout, QLabel, QPushButton,
    QDoubleSpinBox, QComboBox, QGroupBox, QMessageBox, QWidget,
)

from gui.scaling import s, scaled_font_size
from gui.styles import COLORS
from gui.widgets.measurement_camera_view import MeasurementCameraView
from SupportClasses.ObjectiveCalibration import get_store as _get_store

logger = logging.getLogger(__name__)


class ObjectiveCalibrationDialog(QDialog):
    """Modal calibration dialog for a single microscope objective.

    Args:
        camera_manager: CameraManager owning the microscope camera.
        hardware_config: HardwareConfig snapshot (used for sanity refs;
            not mutated here).
        microscope_cam_idx: which slot in CameraManager.cameras is the
            microscope camera.
        camera_name: stable identifier used as the store key (typically
            ``hardware_config.camera_config.camera_spec.name`` or a
            ``camera_{idx}`` fallback).
        initial_objective: objective the dialog is pre-targeted at.
    """

    calibration_committed = Signal(str, float)  # (objective_name, um_per_px)

    def __init__(
        self,
        camera_manager,
        hardware_config,
        microscope_cam_idx: int,
        camera_name: str,
        initial_objective: str,
        parent: Optional[QWidget] = None,
    ):
        super().__init__(parent)
        self._camera_manager = camera_manager
        self._hardware_config = hardware_config
        self._cam_idx = microscope_cam_idx
        self._camera_name = camera_name
        self._initial_objective = initial_objective
        self._store = _get_store()

        self._computed_um_per_px: Optional[float] = None
        self._started_by_dialog: bool = False

        self.setWindowTitle("Calibrate Objective µm/px")
        self.setModal(True)
        self.setMinimumSize(s(960), s(560))
        self.setStyleSheet(
            f"background-color: {COLORS['base']}; color: {COLORS['text']};"
        )

        self._build_ui()
        self._wire_signals()
        self._start_camera_if_needed()
        self._refresh_for_objective(self._initial_objective)

    # ── UI construction ───────────────────────────────────────────

    def _build_ui(self) -> None:
        outer = QHBoxLayout(self)
        outer.setContentsMargins(s(12), s(12), s(12), s(12))
        outer.setSpacing(s(12))

        # Live feed + measurement overlay.
        self._view = MeasurementCameraView(
            camera_manager=self._camera_manager,
            cam_idx=self._cam_idx,
            label=f"{self._camera_name} — microscope",
        )
        self._view.setMinimumSize(s(640), s(480))
        outer.addWidget(self._view, stretch=1)

        # Right panel.
        side = QVBoxLayout()
        side.setSpacing(s(10))
        outer.addLayout(side, stretch=0)

        header = QLabel(f"<b>Calibrating:</b> {self._camera_name}")
        header.setStyleSheet(
            f"color: {COLORS['text']}; font-size: {scaled_font_size(11)}pt;"
        )
        side.addWidget(header)

        instr = QLabel(
            "Place a slide with two known markings under the objective. "
            "Click each marking once to plant an endpoint, then drag the "
            "endpoints to refine. Enter the real-world distance between "
            "them, click Calculate, then Accept Calibration to save."
        )
        instr.setWordWrap(True)
        instr.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {scaled_font_size(9)}pt;"
        )
        side.addWidget(instr)

        # Form: objective + distance + unit.
        form_box = QGroupBox("Inputs")
        form_box.setStyleSheet(self._group_style())
        form = QFormLayout(form_box)
        form.setLabelAlignment(Qt.AlignmentFlag.AlignRight)

        self._cmb_objective = QComboBox()
        self._cmb_objective.addItems(self._store.objective_names())
        if self._initial_objective in self._store.objective_names():
            self._cmb_objective.setCurrentText(self._initial_objective)
        form.addRow("Objective:", self._cmb_objective)

        dist_row = QHBoxLayout()
        dist_row.setSpacing(s(6))
        self._spin_distance = QDoubleSpinBox()
        self._spin_distance.setRange(0.001, 100000.0)
        self._spin_distance.setDecimals(3)
        self._spin_distance.setValue(1.0)
        self._spin_distance.setMinimumWidth(s(110))
        self._cmb_unit = QComboBox()
        self._cmb_unit.addItems(["mm", "µm"])
        dist_row.addWidget(self._spin_distance, stretch=1)
        dist_row.addWidget(self._cmb_unit, stretch=0)
        dist_widget = QWidget()
        dist_widget.setLayout(dist_row)
        form.addRow("Real distance:", dist_widget)

        side.addWidget(form_box)

        # Result group.
        result_box = QGroupBox("Result")
        result_box.setStyleSheet(self._group_style())
        result_form = QFormLayout(result_box)
        result_form.setLabelAlignment(Qt.AlignmentFlag.AlignRight)

        self._lbl_pixel = QLabel("—")
        result_form.addRow("Pixel distance:", self._lbl_pixel)
        self._lbl_um_per_px = QLabel("—")
        self._lbl_um_per_px.setStyleSheet(
            f"color: {COLORS['green']}; font-weight: bold; "
            f"font-size: {scaled_font_size(11)}pt;"
        )
        result_form.addRow("Computed µm/px:", self._lbl_um_per_px)
        self._lbl_eff_mag = QLabel("—")
        result_form.addRow("Effective mag.:", self._lbl_eff_mag)
        self._lbl_resolution = QLabel("—")
        result_form.addRow("Resolution:", self._lbl_resolution)
        self._lbl_previous = QLabel("—")
        self._lbl_previous.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {scaled_font_size(9)}pt;"
        )
        result_form.addRow("Previously:", self._lbl_previous)

        side.addWidget(result_box)
        side.addStretch(1)

        # Button row.
        btn_row = QHBoxLayout()
        btn_row.setSpacing(s(8))

        self._btn_reset = QPushButton("Reset Line")
        self._btn_reset.clicked.connect(self._on_reset)
        btn_row.addWidget(self._btn_reset)

        self._btn_calculate = QPushButton("Calculate")
        self._btn_calculate.clicked.connect(self._on_calculate)
        btn_row.addWidget(self._btn_calculate)

        self._btn_accept = QPushButton("Accept Calibration")
        self._btn_accept.setObjectName("accentBtn")
        self._btn_accept.setEnabled(False)
        self._btn_accept.clicked.connect(self._on_accept)
        btn_row.addWidget(self._btn_accept)

        self._btn_cancel = QPushButton("Cancel")
        self._btn_cancel.clicked.connect(self.reject)
        btn_row.addWidget(self._btn_cancel)

        side.addLayout(btn_row)

    def _group_style(self) -> str:
        return (
            f"QGroupBox {{ border: 1px solid {COLORS['surface1']}; "
            f"border-radius: 4px; margin-top: {s(10)}px; "
            f"padding-top: {s(14)}px; color: {COLORS['text']}; }}"
            f"QGroupBox::title {{ subcontrol-position: top left; "
            f"padding: 2px 6px; }}"
        )

    def _wire_signals(self) -> None:
        self._view.endpoints_changed.connect(self._refresh_pixel_distance)
        self._cmb_objective.currentTextChanged.connect(self._refresh_for_objective)

    # ── Camera lifecycle ──────────────────────────────────────────

    def _start_camera_if_needed(self) -> None:
        if self._camera_manager is None:
            return
        try:
            cameras = self._camera_manager.cameras
            if 0 <= self._cam_idx < len(cameras):
                cam = cameras[self._cam_idx]
                if hasattr(cam, "is_running") and not cam.is_running():
                    self._camera_manager.start(self._cam_idx)
                    self._started_by_dialog = True
        except Exception as exc:
            logger.debug(f"ObjectiveCalibrationDialog: start camera skipped — {exc}")

    def _stop_camera_if_we_started_it(self) -> None:
        if self._started_by_dialog and self._camera_manager is not None:
            try:
                self._camera_manager.stop(self._cam_idx)
            except Exception:
                pass
            self._started_by_dialog = False

    def closeEvent(self, event):
        self._stop_camera_if_we_started_it()
        super().closeEvent(event)

    # ── Refresh helpers ───────────────────────────────────────────

    def _refresh_pixel_distance(self) -> None:
        dist = self._view.pixel_distance()
        img_size = self._view.image_size
        if img_size and img_size[0] > 0:
            self._lbl_resolution.setText(f"{img_size[0]} × {img_size[1]}")
        if dist is None:
            self._lbl_pixel.setText("—")
        else:
            self._lbl_pixel.setText(f"{dist:.2f} px")
        # Invalidate the previously-computed µm/px the moment the line changes.
        self._computed_um_per_px = None
        self._lbl_um_per_px.setText("—")
        self._lbl_eff_mag.setText("—")
        self._btn_accept.setEnabled(False)

    def _refresh_for_objective(self, objective_name: str) -> None:
        existing = self._store.get_calibration(self._camera_name, objective_name)
        if existing:
            self._lbl_previous.setText(
                f"{existing['measured_um_per_px']:.4f} µm/px "
                f"@ {existing['resolution'][0]}×{existing['resolution'][1]}, "
                f"{existing.get('date', '?')}"
            )
        else:
            self._lbl_previous.setText("(no prior calibration)")

    # ── Actions ───────────────────────────────────────────────────

    def _on_reset(self) -> None:
        self._view.reset()
        self._refresh_pixel_distance()

    def _on_calculate(self) -> None:
        dist_px = self._view.pixel_distance()
        if dist_px is None or dist_px <= 0:
            QMessageBox.warning(
                self, "Calibrate",
                "Place both endpoints on the slide marking before "
                "calculating.",
            )
            return
        real_um = self._spin_distance.value()
        if self._cmb_unit.currentText() == "mm":
            real_um *= 1000.0
        if real_um <= 0:
            QMessageBox.warning(
                self, "Calibrate",
                "Real-world distance must be greater than zero.",
            )
            return

        um_per_px = real_um / dist_px
        self._computed_um_per_px = um_per_px
        self._lbl_um_per_px.setText(f"{um_per_px:.4f} µm/px")
        self._lbl_eff_mag.setText(self._format_effective_mag(um_per_px))
        self._btn_accept.setEnabled(True)

    def _format_effective_mag(self, measured_um_per_px: float) -> str:
        cam_cfg = getattr(self._hardware_config, "camera_config", None)
        spec = getattr(cam_cfg, "camera_spec", None) if cam_cfg else None
        nominal = self._store.nominal_magnification(
            self._cmb_objective.currentText()
        )
        if spec is None or nominal is None or measured_um_per_px <= 0:
            return "N/A"
        try:
            sensor_px = spec.effective_pixel_size_um(cam_cfg.active_resolution)
        except Exception:
            return "N/A"
        effective_mag = sensor_px / measured_um_per_px
        return f"{effective_mag:.3f}× (nominal {nominal:g}×)"

    def _on_accept(self) -> None:
        if self._computed_um_per_px is None:
            return
        objective_name = self._cmb_objective.currentText()
        resolution = self._view.image_size
        if resolution[0] == 0 or resolution[1] == 0:
            # Fall back to the spec's active resolution if no frame
            # has arrived yet — better than refusing to save.
            cam_cfg = getattr(self._hardware_config, "camera_config", None)
            resolution = getattr(cam_cfg, "active_resolution", (0, 0))
        self._store.set_calibration(
            self._camera_name,
            objective_name,
            self._computed_um_per_px,
            resolution,
        )
        if self._camera_manager is not None:
            try:
                self._camera_manager.set_um_per_px(
                    self._cam_idx, self._computed_um_per_px
                )
            except Exception as exc:
                logger.debug(
                    f"ObjectiveCalibrationDialog: set_um_per_px failed — {exc}"
                )
        self.calibration_committed.emit(objective_name, self._computed_um_per_px)
        self.accept()

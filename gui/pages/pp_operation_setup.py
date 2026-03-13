"""
pp_operation_setup.py — Pick & Place Operation Setup page.

v7.3.3: First sub-page in the Pick & Place mode. Users select
the operation type (Spheroid / Trypsin / Fluorescent) and
configure mode-specific parameters before marking targets.

The selected operation type and config are emitted via
config_changed so that the target selection page can create
operations with the correct parameters on each click.
"""

from __future__ import annotations

import logging
from typing import Optional

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox,
    QPushButton, QLabel, QComboBox, QDoubleSpinBox, QSpinBox,
    QCheckBox, QRadioButton, QButtonGroup, QFrame, QSizePolicy,
    QStackedWidget, QScrollArea,
)
from PySide6.QtCore import Qt, Signal

from gui.styles import COLORS
from SupportClasses.PickAndPlaceManager import (
    OperationType, SpheroidPickupConfig, TrypsinPickupConfig,
    FluorescentTaggingConfig, DyeConfig,
)

logger = logging.getLogger(__name__)


class PPOperationSetupPage(QWidget):
    """Pick & Place Operation Setup — select operation type and configure."""

    # Emits (OperationType, config_object) whenever settings change
    config_changed = Signal(object, object)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._hardware_config = None
        self._build_ui()
        # Emit initial config
        self._emit_config_changed()

    def _build_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(8)

        _bg = COLORS['base']
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        scroll.setStyleSheet(f"QScrollArea {{ background-color: {_bg}; border: none; }}")

        content = QWidget()
        content.setStyleSheet(f"background-color: {_bg};")
        content_layout = QVBoxLayout(content)
        content_layout.setSpacing(12)

        # ── Mode selector ────────────────────────────────────────
        mode_group = QGroupBox("Operation Mode")
        mode_group.setStyleSheet(self._group_style())
        mode_layout = QHBoxLayout(mode_group)

        self._rb_spheroid = QRadioButton("Spheroid Pickup")
        self._rb_trypsin = QRadioButton("Trypsin Cell Pickup")
        self._rb_fluorescent = QRadioButton("Fluorescent Tagging")
        self._rb_spheroid.setChecked(True)

        self._mode_group = QButtonGroup(self)
        self._mode_group.addButton(self._rb_spheroid, 0)
        self._mode_group.addButton(self._rb_trypsin, 1)
        self._mode_group.addButton(self._rb_fluorescent, 2)
        self._mode_group.idToggled.connect(self._on_mode_changed)

        mode_layout.addWidget(self._rb_spheroid)
        mode_layout.addWidget(self._rb_trypsin)
        mode_layout.addWidget(self._rb_fluorescent)
        mode_layout.addStretch()
        content_layout.addWidget(mode_group)

        # ── Config panels (stacked) ──────────────────────────────
        self._config_stack = QStackedWidget()
        self._spheroid_panel = self._build_spheroid_config()
        self._trypsin_panel = self._build_trypsin_config()
        self._fluorescent_panel = self._build_fluorescent_config()
        self._config_stack.addWidget(self._spheroid_panel)
        self._config_stack.addWidget(self._trypsin_panel)
        self._config_stack.addWidget(self._fluorescent_panel)
        content_layout.addWidget(self._config_stack)

        # ── Destination well selector ────────────────────────────
        dest_group = QGroupBox("Destination")
        dest_group.setStyleSheet(self._group_style())
        dest_layout = QHBoxLayout(dest_group)
        dest_layout.addWidget(QLabel("Dest well:"))
        self._dest_well_combo = QComboBox()
        self._dest_well_combo.setPlaceholderText("Select destination well")
        self._dest_well_combo.setMinimumWidth(120)
        self._dest_well_combo.currentIndexChanged.connect(
            lambda _: self._emit_config_changed())
        dest_layout.addWidget(self._dest_well_combo)
        dest_layout.addStretch()
        content_layout.addWidget(dest_group)

        content_layout.addStretch()
        scroll.setWidget(content)
        layout.addWidget(scroll, 1)

    # ── Config panels ────────────────────────────────────────────

    def _build_spheroid_config(self) -> QWidget:
        panel = QGroupBox("Spheroid Pickup Settings")
        panel.setStyleSheet(self._group_style())
        lay = QVBoxLayout(panel)

        row1 = QHBoxLayout()
        row1.addWidget(QLabel("Diameter (um):"))
        self._sp_diameter = QDoubleSpinBox()
        self._sp_diameter.setRange(10, 5000)
        self._sp_diameter.setValue(200)
        self._sp_diameter.setSingleStep(10)
        self._sp_diameter.valueChanged.connect(self._on_spheroid_param_changed)
        row1.addWidget(self._sp_diameter)

        row1.addWidget(QLabel("Safety factor:"))
        self._sp_safety = QDoubleSpinBox()
        self._sp_safety.setRange(1.0, 10.0)
        self._sp_safety.setValue(1.5)
        self._sp_safety.setSingleStep(0.1)
        self._sp_safety.valueChanged.connect(self._on_spheroid_param_changed)
        row1.addWidget(self._sp_safety)
        lay.addLayout(row1)

        row2 = QHBoxLayout()
        row2.addWidget(QLabel("Bore:"))
        self._sp_bore = QComboBox()
        self._sp_bore.addItems(["P1", "P2", "P3"])
        self._sp_bore.currentIndexChanged.connect(
            lambda _: self._emit_config_changed())
        row2.addWidget(self._sp_bore)

        row2.addWidget(QLabel("Pickup speed (uL/s):"))
        self._sp_pickup_speed = QDoubleSpinBox()
        self._sp_pickup_speed.setRange(0.01, 50)
        self._sp_pickup_speed.setValue(1.0)
        self._sp_pickup_speed.valueChanged.connect(
            lambda _: self._emit_config_changed())
        row2.addWidget(self._sp_pickup_speed)

        row2.addWidget(QLabel("Release speed (uL/s):"))
        self._sp_release_speed = QDoubleSpinBox()
        self._sp_release_speed.setRange(0.01, 50)
        self._sp_release_speed.setValue(1.0)
        self._sp_release_speed.valueChanged.connect(
            lambda _: self._emit_config_changed())
        row2.addWidget(self._sp_release_speed)
        lay.addLayout(row2)

        # Volume preview
        self._lbl_volume = QLabel("Computed volume: --")
        self._lbl_volume.setStyleSheet(f"color: {COLORS['peach']};")
        lay.addWidget(self._lbl_volume)

        self._update_volume_preview()
        return panel

    def _build_trypsin_config(self) -> QWidget:
        panel = QGroupBox("Trypsin Cell Pickup Settings")
        panel.setStyleSheet(self._group_style())
        lay = QVBoxLayout(panel)

        row1 = QHBoxLayout()
        row1.addWidget(QLabel("Trypsin volume (uL):"))
        self._tp_volume = QDoubleSpinBox()
        self._tp_volume.setRange(0.1, 100)
        self._tp_volume.setValue(5.0)
        self._tp_volume.valueChanged.connect(
            lambda _: self._emit_config_changed())
        row1.addWidget(self._tp_volume)

        row1.addWidget(QLabel("Dwell time (s):"))
        self._tp_dwell = QDoubleSpinBox()
        self._tp_dwell.setRange(1, 3600)
        self._tp_dwell.setValue(120)
        self._tp_dwell.valueChanged.connect(
            lambda _: self._emit_config_changed())
        row1.addWidget(self._tp_dwell)
        lay.addLayout(row1)

        row2 = QHBoxLayout()
        self._tp_single = QCheckBox("Single bore mode")
        self._tp_single.setChecked(True)
        self._tp_single.toggled.connect(self._on_trypsin_mode_changed)
        row2.addWidget(self._tp_single)

        row2.addWidget(QLabel("Trypsin bore:"))
        self._tp_tryp_bore = QComboBox()
        self._tp_tryp_bore.addItems(["P1", "P2", "P3"])
        self._tp_tryp_bore.currentIndexChanged.connect(
            lambda _: self._emit_config_changed())
        row2.addWidget(self._tp_tryp_bore)

        row2.addWidget(QLabel("Extraction bore:"))
        self._tp_ext_bore = QComboBox()
        self._tp_ext_bore.addItems(["P1", "P2", "P3"])
        self._tp_ext_bore.setEnabled(False)
        self._tp_ext_bore.currentIndexChanged.connect(
            lambda _: self._emit_config_changed())
        row2.addWidget(self._tp_ext_bore)
        lay.addLayout(row2)

        row3 = QHBoxLayout()
        row3.addWidget(QLabel("Extraction volume (uL):"))
        self._tp_ext_vol = QDoubleSpinBox()
        self._tp_ext_vol.setRange(0.1, 200)
        self._tp_ext_vol.setValue(6.0)
        self._tp_ext_vol.valueChanged.connect(
            lambda _: self._emit_config_changed())
        row3.addWidget(self._tp_ext_vol)

        row3.addWidget(QLabel("Trypsin well:"))
        self._tp_tryp_well = QComboBox()
        self._tp_tryp_well.setPlaceholderText("Source well")
        self._tp_tryp_well.currentIndexChanged.connect(
            lambda _: self._emit_config_changed())
        row3.addWidget(self._tp_tryp_well)
        lay.addLayout(row3)

        return panel

    def _build_fluorescent_config(self) -> QWidget:
        panel = QGroupBox("Fluorescent Tagging Settings")
        panel.setStyleSheet(self._group_style())
        lay = QVBoxLayout(panel)

        row1 = QHBoxLayout()
        row1.addWidget(QLabel("Number of bores:"))
        self._fl_num_bores = QSpinBox()
        self._fl_num_bores.setRange(1, 3)
        self._fl_num_bores.setValue(1)
        self._fl_num_bores.valueChanged.connect(
            lambda _: self._emit_config_changed())
        row1.addWidget(self._fl_num_bores)

        row1.addWidget(QLabel("Dwell time (s):"))
        self._fl_dwell = QDoubleSpinBox()
        self._fl_dwell.setRange(1, 7200)
        self._fl_dwell.setValue(300)
        self._fl_dwell.valueChanged.connect(
            lambda _: self._emit_config_changed())
        row1.addWidget(self._fl_dwell)
        lay.addLayout(row1)

        # Waste bore mode
        row2 = QHBoxLayout()
        self._fl_waste_mode = QCheckBox("Use waste bore (skip waste well travel)")
        self._fl_waste_mode.toggled.connect(self._fl_on_waste_mode)
        row2.addWidget(self._fl_waste_mode)
        row2.addWidget(QLabel("Waste bore:"))
        self._fl_waste_bore = QComboBox()
        self._fl_waste_bore.addItems(["P1", "P2", "P3"])
        self._fl_waste_bore.setEnabled(False)
        self._fl_waste_bore.currentIndexChanged.connect(
            lambda _: self._emit_config_changed())
        row2.addWidget(self._fl_waste_bore)
        lay.addLayout(row2)

        # Dye configuration (3 rows, one per bore)
        for i in range(3):
            bore_row = QHBoxLayout()
            bore_row.addWidget(QLabel(f"Bore {i+1}:"))

            bore_combo = QComboBox()
            bore_combo.addItems(["P1", "P2", "P3"])
            bore_combo.setCurrentIndex(i)
            bore_combo.currentIndexChanged.connect(
                lambda _, _i=i: self._emit_config_changed())
            bore_row.addWidget(bore_combo)

            bore_row.addWidget(QLabel("Dye:"))
            dye_name = QComboBox()
            dye_name.setEditable(True)
            dye_name.addItems(["DAPI", "GFP", "mCherry", "Cy5", "Custom"])
            bore_row.addWidget(dye_name)

            bore_row.addWidget(QLabel("Well:"))
            dye_well = QComboBox()
            dye_well.setPlaceholderText("Dye well")
            dye_well.currentIndexChanged.connect(
                lambda _, _i=i: self._emit_config_changed())
            bore_row.addWidget(dye_well)

            bore_row.addWidget(QLabel("Vol (uL):"))
            dye_vol = QDoubleSpinBox()
            dye_vol.setRange(0.01, 100)
            dye_vol.setValue(2.0)
            dye_vol.valueChanged.connect(
                lambda _, _i=i: self._emit_config_changed())
            bore_row.addWidget(dye_vol)

            lay.addLayout(bore_row)
            setattr(self, f"_fl_bore_{i}", bore_combo)
            setattr(self, f"_fl_dye_{i}", dye_name)
            setattr(self, f"_fl_well_{i}", dye_well)
            setattr(self, f"_fl_vol_{i}", dye_vol)

        # Service wells
        service_row = QHBoxLayout()
        for label, attr in [("Waste:", "_fl_waste_well"),
                            ("Buffer:", "_fl_buffer_well"),
                            ("Wash:", "_fl_wash_well")]:
            service_row.addWidget(QLabel(label))
            combo = QComboBox()
            combo.setPlaceholderText("Well")
            combo.currentIndexChanged.connect(
                lambda _, _a=attr: self._emit_config_changed())
            service_row.addWidget(combo)
            setattr(self, attr, combo)
        lay.addLayout(service_row)

        return panel

    # ── Mode switching ───────────────────────────────────────────

    def _on_mode_changed(self, button_id: int, checked: bool):
        if checked:
            self._config_stack.setCurrentIndex(button_id)
            self._emit_config_changed()

    def _on_trypsin_mode_changed(self, single: bool):
        self._tp_ext_bore.setEnabled(not single)
        self._emit_config_changed()

    def _fl_on_waste_mode(self, enabled: bool):
        self._fl_waste_bore.setEnabled(enabled)
        self._emit_config_changed()

    def _on_spheroid_param_changed(self):
        self._update_volume_preview()
        self._emit_config_changed()

    # ── Volume preview ───────────────────────────────────────────

    def _update_volume_preview(self):
        cfg = SpheroidPickupConfig(
            spheroid_diameter_um=self._sp_diameter.value(),
            safety_factor=self._sp_safety.value(),
        )
        vol = cfg.compute_volume_uL()
        self._lbl_volume.setText(f"Computed volume: {vol:.4f} uL "
                                 f"({vol * 1000:.2f} nL)")

    # ── Config builders ──────────────────────────────────────────

    def get_current_mode(self) -> OperationType:
        idx = self._mode_group.checkedId()
        return [OperationType.SPHEROID_PICKUP,
                OperationType.TRYPSIN_CELL_PICKUP,
                OperationType.FLUORESCENT_TAGGING][idx]

    def get_current_config(self):
        """Return (OperationType, config_object) for the current settings."""
        return (self.get_current_mode(), self._build_config_for_mode())

    def _build_config_for_mode(self):
        mode = self.get_current_mode()
        if mode == OperationType.SPHEROID_PICKUP:
            return self._build_spheroid_config_obj()
        elif mode == OperationType.TRYPSIN_CELL_PICKUP:
            return self._build_trypsin_config_obj()
        else:
            return self._build_fluorescent_config_obj()

    def _build_spheroid_config_obj(self) -> SpheroidPickupConfig:
        return SpheroidPickupConfig(
            spheroid_diameter_um=self._sp_diameter.value(),
            safety_factor=self._sp_safety.value(),
            pickup_bore=self._sp_bore.currentText(),
            pickup_speed_uL_s=self._sp_pickup_speed.value(),
            release_speed_uL_s=self._sp_release_speed.value(),
        )

    def _build_trypsin_config_obj(self) -> TrypsinPickupConfig:
        return TrypsinPickupConfig(
            trypsin_volume_uL=self._tp_volume.value(),
            dwell_time_s=self._tp_dwell.value(),
            single_bore=self._tp_single.isChecked(),
            trypsin_bore=self._tp_tryp_bore.currentText(),
            extraction_bore=self._tp_ext_bore.currentText(),
            extraction_volume_uL=self._tp_ext_vol.value(),
            trypsin_well=self._tp_tryp_well.currentText(),
            dest_well=self._dest_well_combo.currentText(),
        )

    def _build_fluorescent_config_obj(self) -> FluorescentTaggingConfig:
        num = self._fl_num_bores.value()
        dyes = []
        for i in range(num):
            dyes.append(DyeConfig(
                bore=getattr(self, f"_fl_bore_{i}").currentText(),
                dye_well=getattr(self, f"_fl_well_{i}").currentText(),
                dye_name=getattr(self, f"_fl_dye_{i}").currentText(),
                volume_uL=getattr(self, f"_fl_vol_{i}").value(),
            ))
        return FluorescentTaggingConfig(
            num_bores=num,
            dye_configs=dyes,
            dwell_time_s=self._fl_dwell.value(),
            waste_bore=(self._fl_waste_bore.currentText()
                        if self._fl_waste_mode.isChecked() else None),
            use_waste_bore_mode=self._fl_waste_mode.isChecked(),
            waste_well=self._fl_waste_well.currentText(),
            buffer_well=self._fl_buffer_well.currentText(),
            wash_well=self._fl_wash_well.currentText(),
        )

    def _emit_config_changed(self):
        """Emit the current operation type and config."""
        mode = self.get_current_mode()
        config = self._build_config_for_mode()
        self.config_changed.emit(mode, config)

    # ── External interface ───────────────────────────────────────

    def set_well_list(self, wells: list[str]):
        """Update well combo boxes."""
        for combo in [self._dest_well_combo, self._tp_tryp_well]:
            combo.blockSignals(True)
            combo.clear()
            combo.addItems(wells)
            combo.blockSignals(False)
        for i in range(3):
            combo = getattr(self, f"_fl_well_{i}", None)
            if combo:
                combo.blockSignals(True)
                combo.clear()
                combo.addItems(wells)
                combo.blockSignals(False)
        if hasattr(self, '_fl_waste_well'):
            for combo in [self._fl_waste_well, self._fl_buffer_well,
                          self._fl_wash_well]:
                combo.blockSignals(True)
                combo.clear()
                combo.addItems(wells)
                combo.blockSignals(False)

    def set_hardware_config(self, config):
        self._hardware_config = config

    def get_page_title(self) -> str:
        return "Operation Setup"

    def get_context_widget(self) -> None:
        return None

    def on_status_update(self):
        pass

    # ── Style ────────────────────────────────────────────────────

    def _group_style(self) -> str:
        return f"""
            QGroupBox {{
                border: 1px solid {COLORS['surface1']};
                border-radius: 8px;
                color: {COLORS['text']};
                font-size: 11pt;
                font-weight: 600;
                margin-top: 12px;
                padding-top: 16px;
            }}
            QGroupBox::title {{
                color: {COLORS['blue']};
                subcontrol-origin: margin;
                left: 12px;
                padding: 0 4px;
            }}
        """

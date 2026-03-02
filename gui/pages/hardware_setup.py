"""
Hardware Setup Page — v7.2.3 — Required configuration before any other page works.

This is Page 0 in the sidebar. All other pages (Jog, Calibration, Print, etc.)
are disabled until the hardware setup is valid (at least one pump with a syringe
and a needle gauge selected).

v7.2.3 Changes:
    - Reordered sections: Name → Needle → Plate → Ink Library → Pumps → Rosettes
    - Added Rosette Library UI section with add/edit/delete
    - Fixed _apply_config_to_ui() to restore ink library BEFORE pump combos
    - Fixed PumpChannelWidget.set_config() to resolve ink names properly
    - Auto-load from settings now fully restores all fields

Sections:
    1. Setup Name & Notes
    2. Needle configuration (gauge selector)
    3. Well plate format selector
    4. Ink library (add/edit/delete)         ← MOVED UP from Section 5
    5. Pump channels (P1/P2/P3)              ← MOVED DOWN from Section 4
    6. Rosette library (add/edit/delete)     ← NEW
    7. Save/Load buttons + validity

Signals:
    config_changed: Emitted whenever the hardware config changes
    config_validated: Emitted with (bool) when validity state changes
"""

from __future__ import annotations

import logging
import math
from pathlib import Path

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QGroupBox,
    QPushButton, QLabel, QComboBox, QDoubleSpinBox, QSpinBox,
    QLineEdit, QTextEdit, QFrame, QSizePolicy, QMessageBox,
    QFileDialog, QScrollArea, QTableWidget, QTableWidgetItem,
    QHeaderView, QDialog, QFormLayout, QDialogButtonBox,
    QCheckBox, QAbstractItemView,
)
from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QFont, QColor

from SupportClasses.HardwareConfig import HardwareConfig, PumpChannelConfig
from SupportClasses.PhysicalModels import (
    NeedleSpec, SyringeSpec, InkSpec, PrintingMode, RosetteInsert,
    load_needle_catalog, load_syringe_catalog,
)
from SupportClasses.WellPlate import PLATE_DEFINITIONS
from gui.styles import COLORS

logger = logging.getLogger(__name__)


# ═══════════════════════════════════════════════════════════════════
# Ink Editor Dialog
# ═══════════════════════════════════════════════════════════════════

class InkEditorDialog(QDialog):
    """Dialog for adding/editing an ink in the library."""

    INK_TYPES = ["hydrogel", "cells", "media", "buffer", "granular", "custom"]

    def __init__(self, ink: InkSpec | None = None, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Edit Ink" if ink else "Add Ink")
        self.setMinimumWidth(400)
        self._build_ui(ink)

    def _build_ui(self, ink: InkSpec | None):
        layout = QFormLayout(self)

        self.name_edit = QLineEdit(ink.name if ink else "")
        self.name_edit.setPlaceholderText("e.g. Hydrogel A")
        layout.addRow("Name:", self.name_edit)

        self.type_combo = QComboBox()
        self.type_combo.addItems(self.INK_TYPES)
        if ink:
            idx = self.type_combo.findText(ink.ink_type)
            if idx >= 0:
                self.type_combo.setCurrentIndex(idx)
        layout.addRow("Type:", self.type_combo)

        self.viscosity_spin = QDoubleSpinBox()
        self.viscosity_spin.setRange(0.1, 100000.0)
        self.viscosity_spin.setDecimals(1)
        self.viscosity_spin.setSuffix(" cP")
        self.viscosity_spin.setValue(ink.viscosity_cP if ink else 1.0)
        layout.addRow("Viscosity:", self.viscosity_spin)

        self.granule_spin = QDoubleSpinBox()
        self.granule_spin.setRange(0.0, 5000.0)
        self.granule_spin.setDecimals(1)
        self.granule_spin.setSuffix(" µm")
        self.granule_spin.setValue(ink.granule_diameter_um if ink else 0.0)
        layout.addRow("Granule Ø:", self.granule_spin)

        self.cell_spin = QDoubleSpinBox()
        self.cell_spin.setRange(0.0, 1000.0)
        self.cell_spin.setDecimals(1)
        self.cell_spin.setSuffix(" µm")
        self.cell_spin.setValue(ink.cell_diameter_um if ink else 0.0)
        layout.addRow("Cell Ø:", self.cell_spin)

        self.density_spin = QDoubleSpinBox()
        self.density_spin.setRange(0.5, 5.0)
        self.density_spin.setDecimals(2)
        self.density_spin.setSuffix(" g/mL")
        self.density_spin.setValue(ink.density_g_mL if ink else 1.0)
        layout.addRow("Density:", self.density_spin)

        self.color_edit = QLineEdit(ink.color if ink else "#a6e3a1")
        self.color_edit.setPlaceholderText("#rrggbb")
        layout.addRow("Color:", self.color_edit)

        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addRow(buttons)

    def get_ink(self) -> InkSpec | None:
        name = self.name_edit.text().strip()
        if not name:
            return None
        return InkSpec(
            name=name,
            ink_type=self.type_combo.currentText(),
            viscosity_cP=self.viscosity_spin.value(),
            granule_diameter_um=self.granule_spin.value(),
            cell_diameter_um=self.cell_spin.value(),
            density_g_mL=self.density_spin.value(),
            color=self.color_edit.text().strip() or "#a6e3a1",
        )


# ═══════════════════════════════════════════════════════════════════
# Rosette Editor Dialog  (v7.2.3: added to hardware setup)
# ═══════════════════════════════════════════════════════════════════

class RosetteEditorDialog(QDialog):
    """Dialog for adding/editing a rosette insert."""

    def __init__(self, rosette: RosetteInsert | None = None, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Edit Rosette" if rosette else "New Rosette")
        self.setMinimumWidth(320)

        layout = QFormLayout(self)

        self.name_edit = QLineEdit(rosette.name if rosette else "")
        layout.addRow("Name:", self.name_edit)

        self.format_combo = QComboBox()
        for fmt in sorted(PLATE_DEFINITIONS.keys()):
            self.format_combo.addItem(f"{fmt}-well", fmt)
        if rosette:
            idx = self.format_combo.findData(rosette.well_format)
            if idx >= 0:
                self.format_combo.setCurrentIndex(idx)
        layout.addRow("Fits plate:", self.format_combo)

        self.ring_spin = QSpinBox()
        self.ring_spin.setRange(2, 12)
        ring_count = 6
        if rosette:
            ring_count = rosette.num_subwells - (1 if rosette.has_center_well else 0)
        self.ring_spin.setValue(ring_count)
        layout.addRow("Ring sub-wells:", self.ring_spin)

        self.center_check = QComboBox()
        self.center_check.addItems(["Yes", "No"])
        if rosette and not rosette.has_center_well:
            self.center_check.setCurrentIndex(1)
        layout.addRow("Center well:", self.center_check)

        self.diameter_spin = QDoubleSpinBox()
        self.diameter_spin.setRange(0.5, 20.0)
        self.diameter_spin.setDecimals(1)
        self.diameter_spin.setSuffix(" mm")
        self.diameter_spin.setValue(rosette.subwell_diameter_mm if rosette else 1.5)
        layout.addRow("Sub-well Ø:", self.diameter_spin)

        self.depth_spin = QDoubleSpinBox()
        self.depth_spin.setRange(1.0, 30.0)
        self.depth_spin.setDecimals(1)
        self.depth_spin.setSuffix(" mm")
        self.depth_spin.setValue(rosette.subwell_depth_mm if rosette else 6.0)
        layout.addRow("Sub-well depth:", self.depth_spin)

        self.z_offset_spin = QDoubleSpinBox()
        self.z_offset_spin.setRange(0.0, 20.0)
        self.z_offset_spin.setDecimals(1)
        self.z_offset_spin.setSuffix(" mm")
        self.z_offset_spin.setValue(rosette.insert_z_offset_mm if rosette else 0.0)
        layout.addRow("Insert Z offset:", self.z_offset_spin)

        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addRow(buttons)

    def get_rosette(self) -> RosetteInsert:
        return RosetteInsert.create_standard(
            name=self.name_edit.text().strip() or "Unnamed",
            well_format=self.format_combo.currentData(),
            num_ring=self.ring_spin.value(),
            has_center=self.center_check.currentIndex() == 0,
            subwell_diameter_mm=self.diameter_spin.value(),
            subwell_depth_mm=self.depth_spin.value(),
            insert_z_offset_mm=self.z_offset_spin.value(),
        )


# ═══════════════════════════════════════════════════════════════════
# Pump Channel Widget
# ═══════════════════════════════════════════════════════════════════

class PumpChannelWidget(QGroupBox):
    """
    Compact widget for configuring a single pump channel.

    Shows: Enable checkbox, syringe selector, ink selector, mode selector,
    and computed syringe info line.
    """

    changed = Signal()  # Emitted on any config change

    def __init__(self, pump_id: str, syringe_catalog: dict, parent=None):
        super().__init__(pump_id, parent)
        self.pump_id = pump_id
        self.syringe_catalog = syringe_catalog
        self._ink_names: list[str] = []
        self._build_ui()

    def _build_ui(self):
        self.setStyleSheet(f"""
            QGroupBox {{
                font-weight: bold; color: {COLORS.get('blue', '#89b4fa')};
                border: 1px solid {COLORS.get('surface1', '#45475a')};
                border-radius: 4px; margin-top: 6px; padding-top: 14px;
            }}
            QGroupBox::title {{
                subcontrol-origin: margin; left: 8px; padding: 0 4px;
            }}
        """)
        layout = QGridLayout(self)
        layout.setContentsMargins(8, 4, 8, 4)
        layout.setSpacing(4)

        # Row 0: Enable + Syringe
        self.enable_check = QCheckBox("Enable")
        self.enable_check.setChecked(False)
        self.enable_check.toggled.connect(self._on_enable_changed)
        layout.addWidget(self.enable_check, 0, 0)

        layout.addWidget(QLabel("Syringe:"), 0, 1)
        self.syringe_combo = QComboBox()
        self.syringe_combo.addItem("— None —", None)
        for vol in sorted(self.syringe_catalog.keys()):
            spec = self.syringe_catalog[vol]
            self.syringe_combo.addItem(f"{vol} µL  (Hamilton)", vol)
        self.syringe_combo.currentIndexChanged.connect(self._on_syringe_changed)
        layout.addWidget(self.syringe_combo, 0, 2, 1, 2)

        # Row 1: Ink + Mode
        layout.addWidget(QLabel("Ink:"), 1, 0)
        self.ink_combo = QComboBox()
        self.ink_combo.addItem("— None —", None)
        self.ink_combo.currentIndexChanged.connect(lambda: self.changed.emit())
        layout.addWidget(self.ink_combo, 1, 1, 1, 1)

        layout.addWidget(QLabel("Mode:"), 1, 2)
        self.mode_combo = QComboBox()
        self.mode_combo.addItem("Incremental", PrintingMode.INCREMENTAL.value)
        self.mode_combo.addItem("Continuous", PrintingMode.CONTINUOUS.value)
        self.mode_combo.currentIndexChanged.connect(lambda: self.changed.emit())
        layout.addWidget(self.mode_combo, 1, 3)

        # Row 2: Info line
        self.info_label = QLabel("Pump disabled")
        self.info_label.setStyleSheet(
            f"color: {COLORS.get('subtext0', '#a6adc8')}; font-size: 10px;")
        self.info_label.setWordWrap(True)
        layout.addWidget(self.info_label, 2, 0, 1, 4)

        # Initial state
        self._on_enable_changed(False)

    def _on_enable_changed(self, enabled: bool):
        self.syringe_combo.setEnabled(enabled)
        self.ink_combo.setEnabled(enabled)
        self.mode_combo.setEnabled(enabled)
        if not enabled:
            self.info_label.setText("Pump disabled")
        else:
            self._update_info()
        self.changed.emit()

    def _on_syringe_changed(self):
        self._update_info()
        self.changed.emit()

    def _update_info(self):
        if not self.enable_check.isChecked():
            self.info_label.setText("Pump disabled")
            return

        vol = self.syringe_combo.currentData()
        if vol is None:
            self.info_label.setText("No syringe selected")
            return

        spec = self.syringe_catalog.get(vol)
        if spec:
            uL_per_mm = spec.uL_per_mm
            self.info_label.setText(
                f"Capacity: {spec.volume_uL} µL | "
                f"Stroke: {spec.stroke_length_mm} mm | "
                f"Resolution: {uL_per_mm:.2f} µL/mm | "
                f"Barrel ID: {spec.barrel_id_mm:.3f} mm"
            )
        else:
            self.info_label.setText("Unknown syringe")

    def update_ink_list(self, ink_names: list[str]):
        """Refresh the ink dropdown with current library names."""
        current = self.ink_combo.currentData()
        self.ink_combo.blockSignals(True)
        self.ink_combo.clear()
        self.ink_combo.addItem("— None —", None)
        for name in ink_names:
            self.ink_combo.addItem(name, name)
        # Restore selection if still available
        if current:
            idx = self.ink_combo.findData(current)
            if idx >= 0:
                self.ink_combo.setCurrentIndex(idx)
        self.ink_combo.blockSignals(False)
        self._ink_names = ink_names

    def get_config(self) -> PumpChannelConfig:
        """Extract current config from the widget state."""
        config = PumpChannelConfig(pump_id=self.pump_id)
        config.enabled = self.enable_check.isChecked()

        vol = self.syringe_combo.currentData()
        if vol and vol in self.syringe_catalog:
            config.syringe = self.syringe_catalog[vol]

        ink_name = self.ink_combo.currentData()
        if ink_name:
            # Placeholder — parent resolves full InkSpec from library
            config.ink = InkSpec(name=ink_name)

        mode_val = self.mode_combo.currentData()
        config.printing_mode = PrintingMode(mode_val) if mode_val else PrintingMode.INCREMENTAL

        return config

    def set_config(self, config: PumpChannelConfig, ink_names: list[str] | None = None):
        """
        Apply a config to this widget.

        v7.2.3 FIX: Accepts optional ink_names to ensure the ink combo
        is populated BEFORE attempting to set the ink selection.
        """
        self.enable_check.blockSignals(True)
        self.syringe_combo.blockSignals(True)
        self.ink_combo.blockSignals(True)
        self.mode_combo.blockSignals(True)

        # v7.2.3: If ink_names provided, refresh ink combo first
        if ink_names is not None:
            self.update_ink_list(ink_names)

        self.enable_check.setChecked(config.enabled)

        # Set syringe by matching volume_uL
        if config.syringe:
            idx = self.syringe_combo.findData(config.syringe.volume_uL)
            if idx >= 0:
                self.syringe_combo.setCurrentIndex(idx)
            else:
                logger.warning(f"{self.pump_id}: syringe {config.syringe.volume_uL}µL "
                               f"not found in catalog")
                self.syringe_combo.setCurrentIndex(0)
        else:
            self.syringe_combo.setCurrentIndex(0)

        # v7.2.3 FIX: Set ink by matching ink name in the combo
        if config.ink and config.ink.name:
            idx = self.ink_combo.findData(config.ink.name)
            if idx >= 0:
                self.ink_combo.setCurrentIndex(idx)
            else:
                logger.warning(f"{self.pump_id}: ink '{config.ink.name}' "
                               f"not found in ink combo (available: {self._ink_names})")
                self.ink_combo.setCurrentIndex(0)
        else:
            self.ink_combo.setCurrentIndex(0)

        # Set printing mode
        if config.printing_mode:
            idx = self.mode_combo.findData(config.printing_mode.value)
            if idx >= 0:
                self.mode_combo.setCurrentIndex(idx)

        self.enable_check.blockSignals(False)
        self.syringe_combo.blockSignals(False)
        self.ink_combo.blockSignals(False)
        self.mode_combo.blockSignals(False)

        self._on_enable_changed(config.enabled)


# ═══════════════════════════════════════════════════════════════════
# Main Hardware Setup Page
# ═══════════════════════════════════════════════════════════════════

class HardwareSetupPage(QWidget):
    """
    Page 0: Hardware Setup — must be completed before other pages unlock.

    v7.2.3: Reordered sections, added rosette library, fixed config restore.
    """

    _page_title_text = "Hardware Setup"

    # Signals
    config_changed = Signal(object)       # Emits HardwareConfig
    config_validated = Signal(bool)       # Emits validity state

    def __init__(self, parent=None):
        super().__init__(parent)
        self._context_widget = None

        # Load catalogs
        self._needle_catalog = load_needle_catalog()
        self._syringe_catalog = load_syringe_catalog()

        # Current config
        self._config = HardwareConfig()
        self._last_valid = False

        self._setup_ui()

    def get_page_title(self) -> str:
        return "Hardware Setup"

    @property
    def hardware_config(self) -> HardwareConfig:
        return self._config

    # ════════════════════════════════════════════════════════════════
    #  UI CONSTRUCTION — v7.2.3 reordered
    # ════════════════════════════════════════════════════════════════

    def _setup_ui(self):
        """Build the main page layout with v7.2.3 section ordering."""
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setFrameShape(QFrame.NoFrame)

        scroll_content = QWidget()
        self._content_layout = QVBoxLayout(scroll_content)
        self._content_layout.setContentsMargins(12, 8, 12, 8)
        self._content_layout.setSpacing(8)

        # ── Section 1: Name & Notes ───────────────────────────────
        name_group = QGroupBox("Setup Identity")
        name_group.setStyleSheet(self._group_style())
        name_lay = QVBoxLayout(name_group)

        nr = QHBoxLayout()
        nr.addWidget(QLabel("Name:"))
        self.name_edit = QLineEdit("Untitled Setup")
        self.name_edit.textChanged.connect(self._on_config_changed)
        nr.addWidget(self.name_edit)
        name_lay.addLayout(nr)

        notes_row = QHBoxLayout()
        notes_row.addWidget(QLabel("Notes:"))
        self.notes_edit = QLineEdit()
        self.notes_edit.setPlaceholderText("Optional description...")
        notes_row.addWidget(self.notes_edit)
        name_lay.addLayout(notes_row)

        self._content_layout.addWidget(name_group)

        # ── Section 2: Needle Configuration ───────────────────────
        needle_group = QGroupBox("Needle Configuration")
        needle_group.setStyleSheet(self._group_style())
        needle_lay = QVBoxLayout(needle_group)

        gauge_row = QHBoxLayout()
        gauge_row.addWidget(QLabel("Gauge:"))
        self.gauge_combo = QComboBox()
        self.gauge_combo.addItem("— Select —", None)
        for gauge in sorted(self._needle_catalog.keys()):
            spec = self._needle_catalog[gauge]
            self.gauge_combo.addItem(
                f"{gauge}G  (ID: {spec.id_um} µm, OD: {spec.od_um} µm)", gauge)
        self.gauge_combo.currentIndexChanged.connect(self._on_needle_changed)
        gauge_row.addWidget(self.gauge_combo)
        needle_lay.addLayout(gauge_row)

        len_row = QHBoxLayout()
        len_row.addWidget(QLabel("Length:"))
        self.length_combo = QComboBox()
        for length in [0.5, 1.0, 1.5, 2.0, 3.0]:
            self.length_combo.addItem(f'{length}"', length)
        idx = self.length_combo.findData(1.0)
        if idx >= 0:
            self.length_combo.setCurrentIndex(idx)
        self.length_combo.currentIndexChanged.connect(self._on_config_changed)
        len_row.addWidget(self.length_combo)

        len_row.addWidget(QLabel("Channels:"))
        self.channels_spin = QSpinBox()
        self.channels_spin.setRange(1, 7)
        self.channels_spin.setValue(1)
        self.channels_spin.valueChanged.connect(self._on_config_changed)
        len_row.addWidget(self.channels_spin)
        len_row.addStretch()
        needle_lay.addLayout(len_row)

        self.needle_info_label = QLabel("Select a needle gauge above")
        self.needle_info_label.setStyleSheet(
            f"color: {COLORS.get('subtext0', '#a6adc8')}; font-size: 10px;")
        needle_lay.addWidget(self.needle_info_label)

        self._content_layout.addWidget(needle_group)

        # ── Section 3: Well Plate ─────────────────────────────────
        plate_group = QGroupBox("Well Plate Format")
        plate_group.setStyleSheet(self._group_style())
        plate_lay = QHBoxLayout(plate_group)

        plate_lay.addWidget(QLabel("Format:"))
        self.plate_combo = QComboBox()
        for fmt in sorted(PLATE_DEFINITIONS.keys()):
            pdef = PLATE_DEFINITIONS[fmt]
            self.plate_combo.addItem(
                f"{fmt}-well  ({pdef['rows']}×{pdef['cols']})", fmt)
        idx = self.plate_combo.findData(24)
        if idx >= 0:
            self.plate_combo.setCurrentIndex(idx)
        self.plate_combo.currentIndexChanged.connect(self._on_config_changed)
        plate_lay.addWidget(self.plate_combo)
        plate_lay.addStretch()
        self._content_layout.addWidget(plate_group)

        # ── Section 4: Ink Library  (v7.2.3: MOVED UP) ───────────
        ink_group = QGroupBox("Ink Library")
        ink_group.setStyleSheet(self._group_style())
        ink_lay = QVBoxLayout(ink_group)

        ink_btn_row = QHBoxLayout()
        self.add_ink_btn = QPushButton("+ Add Ink")
        self.add_ink_btn.clicked.connect(self._add_ink)
        ink_btn_row.addWidget(self.add_ink_btn)
        self.edit_ink_btn = QPushButton("Edit")
        self.edit_ink_btn.clicked.connect(self._edit_ink)
        ink_btn_row.addWidget(self.edit_ink_btn)
        self.remove_ink_btn = QPushButton("Remove")
        self.remove_ink_btn.clicked.connect(self._remove_ink)
        ink_btn_row.addWidget(self.remove_ink_btn)
        ink_btn_row.addStretch()
        ink_lay.addLayout(ink_btn_row)

        self.ink_table = QTableWidget()
        self.ink_table.setColumnCount(5)
        self.ink_table.setHorizontalHeaderLabels(
            ["Name", "Type", "Viscosity", "Granule Ø", "Cell Ø"])
        self.ink_table.horizontalHeader().setSectionResizeMode(
            QHeaderView.ResizeMode.Stretch)
        self.ink_table.setSelectionBehavior(
            QAbstractItemView.SelectionBehavior.SelectRows)
        self.ink_table.setSelectionMode(
            QAbstractItemView.SelectionMode.SingleSelection)
        self.ink_table.setEditTriggers(
            QAbstractItemView.EditTrigger.NoEditTriggers)
        self.ink_table.setMaximumHeight(120)
        self.ink_table.verticalHeader().setDefaultSectionSize(22)
        self.ink_table.verticalHeader().setVisible(False)
        ink_lay.addWidget(self.ink_table)

        self._content_layout.addWidget(ink_group)

        # ── Section 5: Pump Channels (v7.2.3: MOVED DOWN) ────────
        pumps_group = QGroupBox("Pump Channels  (all values in µL)")
        pumps_group.setStyleSheet(self._group_style())
        pumps_lay = QVBoxLayout(pumps_group)

        self._pump_widgets: dict[str, PumpChannelWidget] = {}
        for pid in ["P1", "P2", "P3"]:
            pw = PumpChannelWidget(pid, self._syringe_catalog)
            pw.changed.connect(self._on_config_changed)
            pumps_lay.addWidget(pw)
            self._pump_widgets[pid] = pw

        self._content_layout.addWidget(pumps_group)

        # ── Section 6: Rosette Library (v7.2.3: NEW) ─────────────
        rosette_group = QGroupBox("Rosette Library")
        rosette_group.setStyleSheet(self._group_style())
        rosette_lay = QVBoxLayout(rosette_group)

        ros_btn_row = QHBoxLayout()
        self.add_rosette_btn = QPushButton("+ New Rosette")
        self.add_rosette_btn.clicked.connect(self._add_rosette)
        ros_btn_row.addWidget(self.add_rosette_btn)
        self.edit_rosette_btn = QPushButton("Edit")
        self.edit_rosette_btn.clicked.connect(self._edit_rosette)
        ros_btn_row.addWidget(self.edit_rosette_btn)
        self.remove_rosette_btn = QPushButton("Remove")
        self.remove_rosette_btn.clicked.connect(self._remove_rosette)
        ros_btn_row.addWidget(self.remove_rosette_btn)
        ros_btn_row.addStretch()
        rosette_lay.addLayout(ros_btn_row)

        self.rosette_table = QTableWidget()
        self.rosette_table.setColumnCount(5)
        self.rosette_table.setHorizontalHeaderLabels(
            ["Name", "Sub-wells", "Fits", "Depth", "Z-offset"])
        self.rosette_table.horizontalHeader().setSectionResizeMode(
            QHeaderView.ResizeMode.Stretch)
        self.rosette_table.setSelectionBehavior(
            QAbstractItemView.SelectionBehavior.SelectRows)
        self.rosette_table.setSelectionMode(
            QAbstractItemView.SelectionMode.SingleSelection)
        self.rosette_table.setEditTriggers(
            QAbstractItemView.EditTrigger.NoEditTriggers)
        self.rosette_table.setMaximumHeight(110)
        self.rosette_table.verticalHeader().setDefaultSectionSize(22)
        self.rosette_table.verticalHeader().setVisible(False)
        rosette_lay.addWidget(self.rosette_table)

        self._content_layout.addWidget(rosette_group)

        # ── Section 7: Save/Load + Validity ───────────────────────
        action_group = QGroupBox("Actions")
        action_group.setStyleSheet(self._group_style())
        action_lay = QVBoxLayout(action_group)

        btn_row = QHBoxLayout()
        self.save_btn = QPushButton("💾 Save to File")
        self.save_btn.clicked.connect(self._save_config)
        btn_row.addWidget(self.save_btn)
        self.load_btn = QPushButton("📂 Load from File")
        self.load_btn.clicked.connect(self._load_config)
        btn_row.addWidget(self.load_btn)
        btn_row.addStretch()
        action_lay.addLayout(btn_row)

        self.validity_label = QLabel("⚠ Setup incomplete")
        self.validity_label.setStyleSheet(
            f"color: {COLORS.get('yellow', '#f9e2af')}; font-weight: bold;")
        action_lay.addWidget(self.validity_label)

        self._content_layout.addWidget(action_group)

        self._content_layout.addStretch()
        scroll.setWidget(scroll_content)
        outer.addWidget(scroll)

    def _group_style(self) -> str:
        """Shared GroupBox stylesheet."""
        return f"""
            QGroupBox {{
                font-weight: bold;
                color: {COLORS.get('text', '#cdd6f4')};
                border: 1px solid {COLORS.get('surface1', '#45475a')};
                border-radius: 6px;
                margin-top: 8px;
                padding-top: 14px;
            }}
            QGroupBox::title {{
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 6px;
            }}
        """

    # ════════════════════════════════════════════════════════════════
    #  NEEDLE CHANGE HANDLER
    # ════════════════════════════════════════════════════════════════

    def _on_needle_changed(self):
        gauge = self.gauge_combo.currentData()
        if gauge and gauge in self._needle_catalog:
            spec = self._needle_catalog[gauge]
            self.needle_info_label.setText(
                f"ID: {spec.id_um} µm | OD: {spec.od_um} µm | "
                f"Wall: {spec.wall_um} µm")
        else:
            self.needle_info_label.setText("Select a needle gauge above")
        self._on_config_changed()

    # ════════════════════════════════════════════════════════════════
    #  CONFIG CHANGE / REBUILD
    # ════════════════════════════════════════════════════════════════

    def _on_config_changed(self):
        """Called whenever any config widget changes."""
        self._rebuild_config()
        valid = self._config.is_valid
        if valid != self._last_valid:
            self._last_valid = valid
            self.config_validated.emit(valid)

        if valid:
            self.validity_label.setText("✓ Setup complete")
            self.validity_label.setStyleSheet(
                f"color: {COLORS.get('green', '#a6e3a1')};")
        else:
            _, issues = self._config.validate()
            self.validity_label.setText(
                f"⚠ {issues[0]}" if issues else "⚠ Setup incomplete")
            self.validity_label.setStyleSheet(
                f"color: {COLORS.get('yellow', '#f9e2af')};")

        self.config_changed.emit(self._config)

    def _rebuild_config(self):
        """Rebuild HardwareConfig from all widget states."""
        # Name & notes
        self._config.config_name = self.name_edit.text().strip() or "Untitled Setup"
        self._config.notes = self.notes_edit.text().strip()

        # Needle
        gauge = self.gauge_combo.currentData()
        if gauge and gauge in self._needle_catalog:
            needle = self._needle_catalog[gauge]
            self._config.needle = NeedleSpec(
                gauge=needle.gauge,
                od_um=needle.od_um,
                id_um=needle.id_um,
                wall_um=needle.wall_um,
                length_inches=self.length_combo.currentData() or 1.0,
                num_channels=self.channels_spin.value(),
            )
        else:
            self._config.needle = None

        # Well plate
        self._config.plate_format = self.plate_combo.currentData() or 24

        # Pumps — resolve ink from library
        for pid, pw in self._pump_widgets.items():
            pcfg = pw.get_config()
            if pcfg.ink and pcfg.ink.name in self._config.ink_library:
                pcfg.ink = self._config.ink_library[pcfg.ink.name]
            elif pcfg.ink:
                pcfg.ink = None  # Ink no longer in library
            self._config.pumps[pid] = pcfg

    # ════════════════════════════════════════════════════════════════
    #  INK LIBRARY CRUD
    # ════════════════════════════════════════════════════════════════

    def _add_ink(self):
        dlg = InkEditorDialog(parent=self)
        if dlg.exec() == QDialog.Accepted:
            ink = dlg.get_ink()
            if ink:
                self._config.add_ink(ink)
                self._refresh_ink_table()
                self._refresh_pump_ink_combos()
                self._on_config_changed()

    def _edit_ink(self):
        row = self.ink_table.currentRow()
        if row < 0:
            return
        name = self.ink_table.item(row, 0).text()
        ink = self._config.get_ink(name)
        if not ink:
            return
        dlg = InkEditorDialog(ink=ink, parent=self)
        if dlg.exec() == QDialog.Accepted:
            new_ink = dlg.get_ink()
            if new_ink:
                if new_ink.name != name:
                    self._config.remove_ink(name)
                self._config.add_ink(new_ink)
                self._refresh_ink_table()
                self._refresh_pump_ink_combos()
                self._on_config_changed()

    def _remove_ink(self):
        row = self.ink_table.currentRow()
        if row < 0:
            return
        name = self.ink_table.item(row, 0).text()
        self._config.remove_ink(name)
        self._refresh_ink_table()
        self._refresh_pump_ink_combos()
        self._on_config_changed()

    def _refresh_ink_table(self):
        """Rebuild the ink table from the config."""
        self.ink_table.setRowCount(0)
        for name, ink in self._config.ink_library.items():
            row = self.ink_table.rowCount()
            self.ink_table.insertRow(row)
            self.ink_table.setItem(row, 0, QTableWidgetItem(ink.name))
            self.ink_table.setItem(row, 1, QTableWidgetItem(ink.ink_type))
            self.ink_table.setItem(row, 2, QTableWidgetItem(
                f"{ink.viscosity_cP:.1f} cP"))
            self.ink_table.setItem(row, 3, QTableWidgetItem(
                f"{ink.granule_diameter_um:.0f} µm"
                if ink.granule_diameter_um > 0 else "—"))
            self.ink_table.setItem(row, 4, QTableWidgetItem(
                f"{ink.cell_diameter_um:.0f} µm"
                if ink.cell_diameter_um > 0 else "—"))

    def _refresh_pump_ink_combos(self):
        """Update all pump channel ink dropdowns from current library."""
        ink_names = list(self._config.ink_library.keys())
        for pw in self._pump_widgets.values():
            pw.update_ink_list(ink_names)

    # ════════════════════════════════════════════════════════════════
    #  ROSETTE LIBRARY CRUD  (v7.2.3: NEW)
    # ════════════════════════════════════════════════════════════════

    def _add_rosette(self):
        dlg = RosetteEditorDialog(parent=self)
        if dlg.exec() == QDialog.Accepted:
            ros = dlg.get_rosette()
            self._config.rosette_library[ros.name] = ros
            self._refresh_rosette_table()
            self._on_config_changed()

    def _edit_rosette(self):
        row = self.rosette_table.currentRow()
        if row < 0:
            return
        old_name = self.rosette_table.item(row, 0).text()
        ros = self._config.rosette_library.get(old_name)
        if ros is None:
            return
        dlg = RosetteEditorDialog(rosette=ros, parent=self)
        if dlg.exec() == QDialog.Accepted:
            new_ros = dlg.get_rosette()
            if new_ros.name != old_name:
                del self._config.rosette_library[old_name]
            self._config.rosette_library[new_ros.name] = new_ros
            self._refresh_rosette_table()
            self._on_config_changed()

    def _remove_rosette(self):
        row = self.rosette_table.currentRow()
        if row < 0:
            return
        name = self.rosette_table.item(row, 0).text()
        self._config.rosette_library.pop(name, None)
        self._refresh_rosette_table()
        self._on_config_changed()

    def _refresh_rosette_table(self):
        """Rebuild the rosette table from the config."""
        self.rosette_table.setRowCount(0)
        for name, ros in self._config.rosette_library.items():
            row = self.rosette_table.rowCount()
            self.rosette_table.insertRow(row)
            center_str = "+center" if ros.has_center_well else ""
            ring_n = ros.num_subwells - (1 if ros.has_center_well else 0)
            self.rosette_table.setItem(row, 0, QTableWidgetItem(ros.name))
            self.rosette_table.setItem(row, 1, QTableWidgetItem(
                f"{ring_n}{center_str}"))
            self.rosette_table.setItem(row, 2, QTableWidgetItem(
                f"{ros.well_format}w"))
            self.rosette_table.setItem(row, 3, QTableWidgetItem(
                f"{ros.subwell_depth_mm:.1f} mm"))
            self.rosette_table.setItem(row, 4, QTableWidgetItem(
                f"{ros.insert_z_offset_mm:.1f} mm"))

    # ════════════════════════════════════════════════════════════════
    #  SAVE / LOAD
    # ════════════════════════════════════════════════════════════════

    def _save_config(self):
        """Save the current config to a JSON file."""
        self._rebuild_config()
        path, _ = QFileDialog.getSaveFileName(
            self, "Save Hardware Setup",
            f"{self._config.config_name}.json",
            "Hardware Setup (*.json)",
        )
        if path:
            try:
                self._config.save(path)
                QMessageBox.information(
                    self, "Saved", f"Configuration saved to:\n{path}")
            except Exception as e:
                QMessageBox.critical(
                    self, "Error", f"Failed to save:\n{e}")

    def _load_config(self):
        """Load a config from a JSON file."""
        path, _ = QFileDialog.getOpenFileName(
            self, "Load Hardware Setup", "",
            "Hardware Setup (*.json)",
        )
        if path:
            try:
                self._config = HardwareConfig.load(path)
                self._apply_config_to_ui()
                QMessageBox.information(
                    self, "Loaded", f"Configuration loaded from:\n{path}")
            except Exception as e:
                QMessageBox.critical(
                    self, "Error", f"Failed to load:\n{e}")

    # ════════════════════════════════════════════════════════════════
    #  APPLY CONFIG TO UI  (v7.2.3: COMPLETE REWRITE)
    # ════════════════════════════════════════════════════════════════

    def _apply_config_to_ui(self):
        """
        Push the current config state into all UI widgets.

        v7.2.3 FIX: Restores in dependency order so that ink library
        is populated before pump channel combos try to resolve ink names.

        Restore order:
        1. Name & Notes
        2. Ink library → table + pump combo refresh
        3. Rosette library → table
        4. Needle gauge + length + channels
        5. Plate format
        6. Pump channels (with ink_names available)
        7. Emit config_changed + config_validated
        """
        logger.info("Applying config to UI (v7.2.3 dependency-ordered restore)")

        # ── 1. Name & Notes ───────────────────────────────────────
        self.name_edit.blockSignals(True)
        self.name_edit.setText(self._config.config_name)
        self.name_edit.blockSignals(False)
        self.notes_edit.setText(self._config.notes)

        # ── 2. Ink Library FIRST (pumps depend on this) ───────────
        self._refresh_ink_table()
        # This populates pump ink combos with all ink names
        self._refresh_pump_ink_combos()
        ink_names = list(self._config.ink_library.keys())
        logger.debug(f"  Ink library restored: {ink_names}")

        # ── 3. Rosette Library ────────────────────────────────────
        self._refresh_rosette_table()
        logger.debug(f"  Rosette library restored: "
                     f"{list(self._config.rosette_library.keys())}")

        # ── 4. Needle ─────────────────────────────────────────────
        self.gauge_combo.blockSignals(True)
        if self._config.needle:
            idx = self.gauge_combo.findData(self._config.needle.gauge)
            if idx >= 0:
                self.gauge_combo.setCurrentIndex(idx)
            else:
                logger.warning(f"  Needle gauge {self._config.needle.gauge} "
                               f"not in catalog")
                self.gauge_combo.setCurrentIndex(0)
            # Length
            lidx = self.length_combo.findData(self._config.needle.length_inches)
            if lidx >= 0:
                self.length_combo.setCurrentIndex(lidx)
            # Channels
            self.channels_spin.setValue(self._config.needle.num_channels)
            logger.debug(f"  Needle restored: {self._config.needle.gauge}G, "
                         f"{self._config.needle.length_inches}\", "
                         f"{self._config.needle.num_channels}ch")
        else:
            self.gauge_combo.setCurrentIndex(0)
        self.gauge_combo.blockSignals(False)
        # Update needle info label directly (avoid triggering _on_config_changed)
        gauge = self.gauge_combo.currentData()
        if gauge and gauge in self._needle_catalog:
            spec = self._needle_catalog[gauge]
            self.needle_info_label.setText(
                f"ID: {spec.id_um} µm | OD: {spec.od_um} µm | "
                f"Wall: {spec.wall_um} µm")

        # ── 5. Plate Format ───────────────────────────────────────
        self.plate_combo.blockSignals(True)
        pidx = self.plate_combo.findData(self._config.plate_format)
        if pidx >= 0:
            self.plate_combo.setCurrentIndex(pidx)
        self.plate_combo.blockSignals(False)
        logger.debug(f"  Plate format restored: {self._config.plate_format}")

        # ── 6. Pump Channels (ink combos are now populated) ───────
        for pid, pw in self._pump_widgets.items():
            if pid in self._config.pumps:
                pcfg = self._config.pumps[pid]
                # v7.2.3: Pass ink_names so the combo is guaranteed populated
                pw.set_config(pcfg, ink_names=ink_names)
                logger.debug(
                    f"  {pid}: enabled={pcfg.enabled}, "
                    f"syringe={pcfg.syringe.volume_uL if pcfg.syringe else None}µL, "
                    f"ink={pcfg.ink.name if pcfg.ink else None}, "
                    f"mode={pcfg.printing_mode.value}")

        # ── 7. Emit signals ──────────────────────────────────────
        self._on_config_changed()
        logger.info("Config restore complete")

    # ════════════════════════════════════════════════════════════════
    #  EXTERNAL API
    # ════════════════════════════════════════════════════════════════

    def set_config(self, config: HardwareConfig):
        """Set config programmatically (e.g. from settings restore on launch)."""
        self._config = config
        self._apply_config_to_ui()

    def get_config(self) -> HardwareConfig:
        """Get the current config after rebuilding from UI."""
        self._rebuild_config()
        return self._config

    def set_hardware_config(self, config):
        """v7.2 interface: same as set_config (this IS the hardware page)."""
        if isinstance(config, HardwareConfig):
            self.set_config(config)

    def on_status_update(self):
        """Called by MainWindow timer. No periodic refresh needed for this page."""
        pass

    def get_context_widget(self) -> QWidget | None:
        """Hardware Setup has no context panel."""
        return self._context_widget

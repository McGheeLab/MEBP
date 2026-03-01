"""
Hardware Setup Page — Required configuration before any other page works.

This is Page 0 in the sidebar. All other pages (Jog, Calibration, Print, etc.)
are disabled until the hardware setup is valid (at least one pump with a syringe
and a needle gauge selected).

Sections:
    1. Setup Name & Notes
    2. Needle configuration (gauge selector)
    3. Well plate format selector
    4. Pump channels (P1/P2/P3) — syringe selection, ink assignment, mode
    5. Ink library (add/edit/delete)
    6. Save/Load buttons

All pump-related values are displayed in µL throughout.

Signals:
    config_changed: Emitted whenever the hardware config changes
    config_validated: Emitted with (bool) when validity state changes
"""

from __future__ import annotations

import logging
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
    NeedleSpec, SyringeSpec, InkSpec, PrintingMode,
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
# Pump Channel Widget
# ═══════════════════════════════════════════════════════════════════

class PumpChannelWidget(QGroupBox):
    """
    Compact widget for configuring a single pump channel.
    Shows syringe selection, ink assignment, printing mode, and capacity info.
    All values displayed in µL.
    """

    changed = Signal()

    def __init__(
        self,
        pump_id: str,
        syringe_catalog: dict[int, SyringeSpec],
        parent=None,
    ):
        super().__init__(f"Pump {pump_id}", parent)
        self.pump_id = pump_id
        self.syringe_catalog = syringe_catalog
        self._ink_names: list[str] = []
        self._build_ui()

    def _build_ui(self):
        layout = QGridLayout(self)
        layout.setContentsMargins(8, 12, 8, 8)
        layout.setSpacing(6)

        # Row 0: Enable checkbox + Syringe selector
        self.enable_check = QCheckBox("Enabled")
        self.enable_check.toggled.connect(self._on_enable_changed)
        layout.addWidget(self.enable_check, 0, 0)

        layout.addWidget(QLabel("Syringe:"), 0, 1)
        self.syringe_combo = QComboBox()
        self.syringe_combo.addItem("— None —", None)
        for vol in sorted(self.syringe_catalog.keys()):
            spec = self.syringe_catalog[vol]
            self.syringe_combo.addItem(f"{vol} µL ({spec.part_number})", vol)
        self.syringe_combo.currentIndexChanged.connect(self._on_syringe_changed)
        layout.addWidget(self.syringe_combo, 0, 2, 1, 2)

        # Row 1: Ink selector + Mode
        layout.addWidget(QLabel("Ink:"), 1, 0)
        self.ink_combo = QComboBox()
        self.ink_combo.addItem("— None —", None)
        self.ink_combo.currentIndexChanged.connect(self._on_changed)
        layout.addWidget(self.ink_combo, 1, 1, 1, 2)

        layout.addWidget(QLabel("Mode:"), 1, 3)
        self.mode_combo = QComboBox()
        self.mode_combo.addItem("Incremental", PrintingMode.INCREMENTAL.value)
        self.mode_combo.addItem("Continuous", PrintingMode.CONTINUOUS.value)
        self.mode_combo.currentIndexChanged.connect(self._on_changed)
        layout.addWidget(self.mode_combo, 1, 4)

        # Row 2: Info labels
        self.info_label = QLabel("No syringe selected")
        self.info_label.setStyleSheet(f"color: {COLORS.get('subtext0', '#6c7086')};")
        layout.addWidget(self.info_label, 2, 0, 1, 5)

        # Initial state
        self._on_enable_changed(False)

    def _on_enable_changed(self, enabled):
        self.syringe_combo.setEnabled(enabled)
        self.ink_combo.setEnabled(enabled)
        self.mode_combo.setEnabled(enabled)
        self._update_info()
        self._on_changed()

    def _on_syringe_changed(self):
        self._update_info()
        self._on_changed()

    def _on_changed(self):
        self.changed.emit()

    def _update_info(self):
        """Update the info label with syringe capacity details."""
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
        """Refresh the ink dropdown with current library."""
        current = self.ink_combo.currentData()
        self.ink_combo.blockSignals(True)
        self.ink_combo.clear()
        self.ink_combo.addItem("— None —", None)
        for name in ink_names:
            self.ink_combo.addItem(name, name)
        # Restore selection
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
            # Ink object will be resolved by the parent from the library
            config.ink = InkSpec(name=ink_name)  # Placeholder — parent resolves full spec

        mode_val = self.mode_combo.currentData()
        config.printing_mode = PrintingMode(mode_val) if mode_val else PrintingMode.INCREMENTAL

        return config

    def set_config(self, config: PumpChannelConfig):
        """Apply a config to this widget."""
        self.enable_check.blockSignals(True)
        self.syringe_combo.blockSignals(True)
        self.ink_combo.blockSignals(True)
        self.mode_combo.blockSignals(True)

        self.enable_check.setChecked(config.enabled)

        # Set syringe
        if config.syringe:
            idx = self.syringe_combo.findData(config.syringe.volume_uL)
            if idx >= 0:
                self.syringe_combo.setCurrentIndex(idx)
        else:
            self.syringe_combo.setCurrentIndex(0)

        # Set ink
        if config.ink:
            idx = self.ink_combo.findData(config.ink.name)
            if idx >= 0:
                self.ink_combo.setCurrentIndex(idx)
        else:
            self.ink_combo.setCurrentIndex(0)

        # Set mode
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

    Configures needle, syringes, inks, well plate, and printing modes.
    All pump values are in µL.
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
        """Get the current hardware configuration."""
        return self._config

    # ════════════════════════════════════════════════════════════════
    #  UI CONSTRUCTION
    # ════════════════════════════════════════════════════════════════

    def _setup_ui(self):
        """Build the main page layout."""
        outer = QVBoxLayout(self)
        outer.setContentsMargins(16, 16, 16, 16)
        outer.setSpacing(12)

        # Title + validity indicator
        title_row = QHBoxLayout()
        title = QLabel("Hardware Setup")
        title.setFont(QFont("Segoe UI", 16, QFont.Bold))
        title.setStyleSheet(f"color: {COLORS.get('text', '#cdd6f4')};")
        title_row.addWidget(title)
        title_row.addStretch()

        self.validity_label = QLabel("⚠ Setup incomplete")
        self.validity_label.setStyleSheet(f"color: {COLORS.get('yellow', '#f9e2af')};")
        self.validity_label.setFont(QFont("Segoe UI", 11))
        title_row.addWidget(self.validity_label)
        outer.addLayout(title_row)

        # Scrollable content
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        content = QWidget()
        self._content_layout = QVBoxLayout(content)
        self._content_layout.setSpacing(12)

        # ── Section 1: Setup Name ─────────────────────────────────
        name_group = QGroupBox("Setup Identity")
        name_lay = QFormLayout(name_group)
        self.name_edit = QLineEdit("Untitled Setup")
        self.name_edit.textChanged.connect(self._on_config_changed)
        name_lay.addRow("Name:", self.name_edit)
        self.notes_edit = QLineEdit()
        self.notes_edit.setPlaceholderText("Optional experiment notes...")
        self.notes_edit.textChanged.connect(self._on_config_changed)
        name_lay.addRow("Notes:", self.notes_edit)
        self._content_layout.addWidget(name_group)

        # ── Section 2: Needle ─────────────────────────────────────
        needle_group = QGroupBox("Needle Configuration")
        needle_lay = QGridLayout(needle_group)

        needle_lay.addWidget(QLabel("Gauge:"), 0, 0)
        self.gauge_combo = QComboBox()
        self.gauge_combo.addItem("— Select —", None)
        for gauge in sorted(self._needle_catalog.keys()):
            spec = self._needle_catalog[gauge]
            self.gauge_combo.addItem(
                f"{gauge}G  (ID: {spec.id_um:.0f} µm, OD: {spec.od_um:.0f} µm)",
                gauge,
            )
        self.gauge_combo.currentIndexChanged.connect(self._on_needle_changed)
        needle_lay.addWidget(self.gauge_combo, 0, 1, 1, 2)

        needle_lay.addWidget(QLabel("Length:"), 1, 0)
        self.length_combo = QComboBox()
        self.length_combo.addItem("1.0 inch (25.4 mm)", 1.0)
        self.length_combo.addItem("1.5 inch (38.1 mm)", 1.5)
        self.length_combo.addItem("2.0 inch (50.8 mm)", 2.0)
        self.length_combo.currentIndexChanged.connect(self._on_needle_changed)
        needle_lay.addWidget(self.length_combo, 1, 1)

        needle_lay.addWidget(QLabel("Channels:"), 1, 2)
        self.channels_spin = QSpinBox()
        self.channels_spin.setRange(1, 3)
        self.channels_spin.setValue(1)
        self.channels_spin.valueChanged.connect(self._on_needle_changed)
        needle_lay.addWidget(self.channels_spin, 1, 3)

        self.needle_info = QLabel("Select a needle gauge")
        self.needle_info.setStyleSheet(f"color: {COLORS.get('subtext0', '#6c7086')};")
        needle_lay.addWidget(self.needle_info, 2, 0, 1, 4)

        self._content_layout.addWidget(needle_group)

        # ── Section 3: Well Plate ─────────────────────────────────
        plate_group = QGroupBox("Well Plate")
        plate_lay = QHBoxLayout(plate_group)
        plate_lay.addWidget(QLabel("Format:"))
        self.plate_combo = QComboBox()
        for fmt in sorted(PLATE_DEFINITIONS.keys()):
            pdef = PLATE_DEFINITIONS[fmt]
            self.plate_combo.addItem(
                f"{fmt}-well  ({pdef['rows']}×{pdef['cols']})",
                fmt,
            )
        # Default to 24-well
        idx = self.plate_combo.findData(24)
        if idx >= 0:
            self.plate_combo.setCurrentIndex(idx)
        self.plate_combo.currentIndexChanged.connect(self._on_config_changed)
        plate_lay.addWidget(self.plate_combo)
        plate_lay.addStretch()
        self._content_layout.addWidget(plate_group)

        # ── Section 4: Pump Channels ──────────────────────────────
        pumps_group = QGroupBox("Pump Channels  (all values in µL)")
        pumps_lay = QVBoxLayout(pumps_group)

        self._pump_widgets: dict[str, PumpChannelWidget] = {}
        for pid in ["P1", "P2", "P3"]:
            pw = PumpChannelWidget(pid, self._syringe_catalog)
            pw.changed.connect(self._on_config_changed)
            pumps_lay.addWidget(pw)
            self._pump_widgets[pid] = pw

        self._content_layout.addWidget(pumps_group)

        # ── Section 5: Ink Library ────────────────────────────────
        ink_group = QGroupBox("Ink Library")
        ink_lay = QVBoxLayout(ink_group)

        # Buttons
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

        # Table
        self.ink_table = QTableWidget(0, 5)
        self.ink_table.setHorizontalHeaderLabels(["Name", "Type", "Viscosity", "Granule Ø", "Cell Ø"])
        self.ink_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.ink_table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.ink_table.setSelectionMode(QAbstractItemView.SingleSelection)
        self.ink_table.setMaximumHeight(180)
        ink_lay.addWidget(self.ink_table)

        self._content_layout.addWidget(ink_group)

        # ── Spacer ────────────────────────────────────────────────
        self._content_layout.addStretch()

        scroll.setWidget(content)
        outer.addWidget(scroll)

        # ── Bottom bar: Save / Load / Apply ───────────────────────
        btn_row = QHBoxLayout()

        self.save_btn = QPushButton("💾 Save Setup")
        self.save_btn.clicked.connect(self._save_config)
        btn_row.addWidget(self.save_btn)

        self.load_btn = QPushButton("📂 Load Setup")
        self.load_btn.clicked.connect(self._load_config)
        btn_row.addWidget(self.load_btn)

        btn_row.addStretch()

        self.apply_btn = QPushButton("✓ Apply & Continue")
        self.apply_btn.setMinimumWidth(160)
        self.apply_btn.clicked.connect(self._apply_config)
        btn_row.addWidget(self.apply_btn)

        outer.addLayout(btn_row)

    # ════════════════════════════════════════════════════════════════
    #  CONTEXT PANEL
    # ════════════════════════════════════════════════════════════════

    def get_context_widget(self) -> QWidget:
        """Context panel: quick syringe reference + recent configs."""
        if self._context_widget:
            return self._context_widget

        ctx = QWidget()
        lay = QVBoxLayout(ctx)
        lay.setContentsMargins(8, 8, 8, 8)

        # Quick reference: Syringe catalog
        ref_group = QGroupBox("Syringe Reference")
        ref_lay = QVBoxLayout(ref_group)
        for vol in sorted(self._syringe_catalog.keys()):
            spec = self._syringe_catalog[vol]
            lbl = QLabel(f"{vol} µL — {spec.uL_per_mm:.2f} µL/mm — Ø{spec.barrel_id_mm:.3f} mm")
            lbl.setStyleSheet(f"color: {COLORS.get('subtext1', '#a6adc8')}; font-size: 11px;")
            ref_lay.addWidget(lbl)
        lay.addWidget(ref_group)

        # Quick reference: Needle catalog
        needle_group = QGroupBox("Needle Reference")
        needle_lay = QVBoxLayout(needle_group)
        for gauge in sorted(self._needle_catalog.keys()):
            spec = self._needle_catalog[gauge]
            lbl = QLabel(f"{gauge}G — ID: {spec.id_um:.0f} µm, OD: {spec.od_um:.0f} µm")
            lbl.setStyleSheet(f"color: {COLORS.get('subtext1', '#a6adc8')}; font-size: 11px;")
            needle_lay.addWidget(lbl)
        lay.addWidget(needle_group)

        lay.addStretch()
        self._context_widget = ctx
        return ctx

    # ════════════════════════════════════════════════════════════════
    #  EVENT HANDLERS
    # ════════════════════════════════════════════════════════════════

    def _on_needle_changed(self):
        """Update needle info and config when gauge/length/channels change."""
        gauge = self.gauge_combo.currentData()
        if gauge and gauge in self._needle_catalog:
            spec = self._needle_catalog[gauge]
            length = self.length_combo.currentData() or 1.0
            channels = self.channels_spin.value()
            self.needle_info.setText(
                f"ID: {spec.id_um:.0f} µm ({spec.id_mm:.3f} mm) | "
                f"OD: {spec.od_um:.0f} µm ({spec.od_mm:.3f} mm) | "
                f"Length: {length}\" ({length * 25.4:.1f} mm) | "
                f"Channels: {channels}"
            )
        else:
            self.needle_info.setText("Select a needle gauge")
        self._on_config_changed()

    def _on_config_changed(self):
        """Rebuild the config from current widget state and emit signals."""
        self._rebuild_config()
        valid = self._config.is_valid
        if valid != self._last_valid:
            self._last_valid = valid
            self.config_validated.emit(valid)

        if valid:
            self.validity_label.setText("✓ Setup complete")
            self.validity_label.setStyleSheet(f"color: {COLORS.get('green', '#a6e3a1')};")
        else:
            _, issues = self._config.validate()
            self.validity_label.setText(f"⚠ {issues[0]}" if issues else "⚠ Setup incomplete")
            self.validity_label.setStyleSheet(f"color: {COLORS.get('yellow', '#f9e2af')};")

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
            # Create a new NeedleSpec with user-selected length and channels
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

        # Pumps
        for pid, pw in self._pump_widgets.items():
            pcfg = pw.get_config()
            # Resolve ink from library
            if pcfg.ink and pcfg.ink.name in self._config.ink_library:
                pcfg.ink = self._config.ink_library[pcfg.ink.name]
            elif pcfg.ink:
                pcfg.ink = None  # Ink not in library anymore
            self._config.pumps[pid] = pcfg

    # ── Ink Library ───────────────────────────────────────────────

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
                # Remove old, add new (handles name change)
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
            self.ink_table.setItem(row, 2, QTableWidgetItem(f"{ink.viscosity_cP:.1f} cP"))
            self.ink_table.setItem(row, 3, QTableWidgetItem(
                f"{ink.granule_diameter_um:.0f} µm" if ink.granule_diameter_um > 0 else "—"
            ))
            self.ink_table.setItem(row, 4, QTableWidgetItem(
                f"{ink.cell_diameter_um:.0f} µm" if ink.cell_diameter_um > 0 else "—"
            ))

    def _refresh_pump_ink_combos(self):
        """Update all pump channel ink dropdowns."""
        ink_names = list(self._config.ink_library.keys())
        for pw in self._pump_widgets.values():
            pw.update_ink_list(ink_names)

    # ── Save / Load ───────────────────────────────────────────────

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
                QMessageBox.information(self, "Saved", f"Configuration saved to:\n{path}")
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Failed to save:\n{e}")

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
                QMessageBox.information(self, "Loaded", f"Configuration loaded from:\n{path}")
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Failed to load:\n{e}")

    def _apply_config_to_ui(self):
        """Push the current config state into all UI widgets."""
        # Block signals during bulk update
        self.name_edit.setText(self._config.config_name)
        self.notes_edit.setText(self._config.notes)

        # Needle
        if self._config.needle:
            idx = self.gauge_combo.findData(self._config.needle.gauge)
            if idx >= 0:
                self.gauge_combo.setCurrentIndex(idx)
            # Length
            lidx = self.length_combo.findData(self._config.needle.length_inches)
            if lidx >= 0:
                self.length_combo.setCurrentIndex(lidx)
            self.channels_spin.setValue(self._config.needle.num_channels)
        else:
            self.gauge_combo.setCurrentIndex(0)

        # Plate
        pidx = self.plate_combo.findData(self._config.plate_format)
        if pidx >= 0:
            self.plate_combo.setCurrentIndex(pidx)

        # Ink library
        self._refresh_ink_table()
        self._refresh_pump_ink_combos()

        # Pumps
        for pid, pw in self._pump_widgets.items():
            if pid in self._config.pumps:
                pw.set_config(self._config.pumps[pid])

        self._on_config_changed()

    def _apply_config(self):
        """Apply config and signal the app to unlock other pages."""
        self._rebuild_config()
        valid, issues = self._config.validate()
        if not valid:
            QMessageBox.warning(
                self, "Incomplete Setup",
                "Please resolve the following:\n\n" + "\n".join(f"• {i}" for i in issues),
            )
            return
        self.config_changed.emit(self._config)
        self.config_validated.emit(True)
        logger.info(f"Hardware config applied: {self._config}")

    # ════════════════════════════════════════════════════════════════
    #  EXTERNAL API
    # ════════════════════════════════════════════════════════════════

    def set_config(self, config: HardwareConfig):
        """Set config programmatically (e.g. from settings restore)."""
        self._config = config
        self._apply_config_to_ui()

    def get_config(self) -> HardwareConfig:
        """Get the current config after rebuilding from UI."""
        self._rebuild_config()
        return self._config

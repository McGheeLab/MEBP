"""
Hardware Setup Page — v7.2.4 — Required configuration before any other page works.

This is Page 0 in the sidebar. All other pages (Jog, Calibration, Print, etc.)
are disabled until the hardware setup is valid (at least one pump with a syringe
and a needle gauge selected).

v7.2.4 Changes (Session 3):
    - Reordered: Name → Plate → Inks → Pumps → Needle → Channel Map → Rosettes
    - Pump ink combos are exclusive (inks already assigned to other pumps grayed out)
    - Pump-ink summary label below pumps section
    - Dynamic "Needle Channel Assignment" section
    - Channel mapping combos list only enabled pumps
    - _apply_config_to_ui restores channel map after pumps and needle
    - _rebuild_config captures channel map state

v7.2.3 Changes:
    - Reordered sections: Name → Needle → Plate → Ink Library → Pumps → Rosettes
    - Added Rosette Library UI section with add/edit/delete
    - Fixed _apply_config_to_ui() to restore ink library BEFORE pump combos
    - Fixed PumpChannelWidget.set_config() to resolve ink names properly

Sections (v7.2.4):
    1. Setup Name & Notes
    2. Well Plate Format
    3. Ink Library
    4. Pump Channels (P1/P2/P3) with exclusive ink assignment
    5. Needle Configuration (gauge + length + channels)
    6. Needle Channel → Pump Mapping (dynamic)
    7. Rosette Library
    8. Save/Load + Validity

Signals:
    config_changed: Emitted whenever the hardware config changes
    config_validated: Emitted with (bool) when validity state changes
"""

from __future__ import annotations

import logging
import math
from pathlib import Path
from functools import partial

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QGroupBox,
    QPushButton, QLabel, QComboBox, QDoubleSpinBox, QSpinBox,
    QLineEdit, QTextEdit, QFrame, QSizePolicy, QMessageBox,
    QFileDialog, QScrollArea, QTableWidget, QTableWidgetItem,
    QHeaderView, QDialog, QFormLayout, QDialogButtonBox,
    QCheckBox, QAbstractItemView, QListWidget, QListWidgetItem,
)
from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QFont, QColor, QStandardItem

from SupportClasses.HardwareConfig import HardwareConfig, PumpChannelConfig
from SupportClasses.PhysicalModels import (
    NeedleSpec, SyringeSpec, InkSpec, PrintingMode, RosetteInsert,
    load_needle_catalog, load_syringe_catalog,
)
from SupportClasses.WellPlate import PLATE_DEFINITIONS
from gui.styles import COLORS, SECTION_TITLE_STYLE

logger = logging.getLogger(__name__)

# v7.2.4: Default directory for hardware config files
CONFIG_HARDWARE_DIR = Path(__file__).resolve().parent.parent.parent / "config" / "hardware"



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
        self.name_edit.setPlaceholderText("e.g. Alginate 2%")
        layout.addRow("Name:", self.name_edit)

        self.type_combo = QComboBox()
        self.type_combo.addItems(self.INK_TYPES)
        if ink:
            idx = self.type_combo.findText(ink.ink_type)
            if idx >= 0:
                self.type_combo.setCurrentIndex(idx)
        layout.addRow("Type:", self.type_combo)

        self.viscosity_spin = QDoubleSpinBox()
        self.viscosity_spin.setRange(0.1, 100000)
        self.viscosity_spin.setDecimals(1)
        self.viscosity_spin.setSuffix(" cP")
        self.viscosity_spin.setValue(ink.viscosity_cP if ink else 1.0)
        layout.addRow("Viscosity:", self.viscosity_spin)

        self.granule_spin = QDoubleSpinBox()
        self.granule_spin.setRange(0, 5000)
        self.granule_spin.setDecimals(1)
        self.granule_spin.setSuffix(" µm")
        self.granule_spin.setValue(ink.granule_diameter_um if ink else 0)
        layout.addRow("Granule Ø:", self.granule_spin)

        self.cell_spin = QDoubleSpinBox()
        self.cell_spin.setRange(0, 500)
        self.cell_spin.setDecimals(1)
        self.cell_spin.setSuffix(" µm")
        self.cell_spin.setValue(ink.cell_diameter_um if ink else 0)
        layout.addRow("Cell Ø:", self.cell_spin)

        self.density_spin = QDoubleSpinBox()
        self.density_spin.setRange(0.5, 5.0)
        self.density_spin.setDecimals(2)
        self.density_spin.setSuffix(" g/mL")
        self.density_spin.setValue(ink.density_g_mL if ink else 1.0)
        layout.addRow("Density:", self.density_spin)

        self.color_edit = QLineEdit(ink.color if ink else "#a6e3a1")
        self.color_edit.setPlaceholderText("#RRGGBB")
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
# Rosette Editor Dialog
# ═══════════════════════════════════════════════════════════════════

class RosetteEditorDialog(QDialog):
    """Dialog for adding/editing a rosette insert."""

    def __init__(self, rosette: RosetteInsert | None = None, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Edit Rosette" if rosette else "Add Rosette")
        self.setMinimumWidth(400)
        self._build_ui(rosette)

    def _build_ui(self, rosette: RosetteInsert | None):
        layout = QFormLayout(self)

        self.name_edit = QLineEdit(rosette.name if rosette else "")
        self.name_edit.setPlaceholderText("e.g. 4-subwell ring")
        layout.addRow("Name:", self.name_edit)

        self.format_combo = QComboBox()
        for fmt in sorted(PLATE_DEFINITIONS.keys()):
            self.format_combo.addItem(f"{fmt}-well", fmt)
        if rosette:
            idx = self.format_combo.findData(rosette.well_format)
            if idx >= 0:
                self.format_combo.setCurrentIndex(idx)
        layout.addRow("Well format:", self.format_combo)

        self.ring_spin = QSpinBox()
        self.ring_spin.setRange(1, 12)
        self.ring_spin.setValue(rosette.num_subwells - (1 if rosette and rosette.has_center_well else 0) if rosette else 4)
        layout.addRow("Ring sub-wells:", self.ring_spin)

        self.center_check = QComboBox()
        self.center_check.addItems(["Yes", "No"])
        if rosette and not rosette.has_center_well:
            self.center_check.setCurrentIndex(1)
        layout.addRow("Center well:", self.center_check)

        self.diameter_spin = QDoubleSpinBox()
        self.diameter_spin.setRange(0.1, 10.0)
        self.diameter_spin.setDecimals(2)
        self.diameter_spin.setSuffix(" mm")
        self.diameter_spin.setValue(rosette.subwell_diameter_mm if rosette else 2.0)
        layout.addRow("Sub-well Ø:", self.diameter_spin)

        self.depth_spin = QDoubleSpinBox()
        self.depth_spin.setRange(0.1, 20.0)
        self.depth_spin.setDecimals(1)
        self.depth_spin.setSuffix(" mm")
        self.depth_spin.setValue(rosette.subwell_depth_mm if rosette else 3.0)
        layout.addRow("Sub-well depth:", self.depth_spin)

        self.z_offset_spin = QDoubleSpinBox()
        self.z_offset_spin.setRange(-10.0, 10.0)
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
# Pump Channel Widget (v7.2.4: exclusive ink assignment)
# ═══════════════════════════════════════════════════════════════════

class PumpChannelWidget(QGroupBox):
    """
    Compact widget for configuring a single pump channel.

    Shows: Enable checkbox, syringe selector, ink selector, mode selector,
    and computed syringe info line.

    v7.2.4: Supports exclusive ink assignment via set_excluded_inks().
    """

    changed = Signal()  # Emitted on any config change

    def __init__(self, pump_id: str, syringe_catalog: dict, parent=None):
        super().__init__(pump_id, parent)
        self.pump_id = pump_id
        self.syringe_catalog = syringe_catalog
        self._ink_names: list[str] = []
        self._excluded_inks: set[str] = set()  # v7.2.4: inks used by OTHER pumps
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
            self.syringe_combo.addItem(f"{vol} µL", vol)
        self.syringe_combo.currentIndexChanged.connect(self._on_change)
        layout.addWidget(self.syringe_combo, 0, 2)

        # Row 1: Ink + Mode
        layout.addWidget(QLabel("Ink:"), 1, 0)
        self.ink_combo = QComboBox()
        self.ink_combo.addItem("— None —", None)
        self.ink_combo.currentIndexChanged.connect(self._on_change)
        layout.addWidget(self.ink_combo, 1, 1, 1, 2)

        layout.addWidget(QLabel("Mode:"), 2, 0)
        self.mode_combo = QComboBox()
        self.mode_combo.addItem("Incremental", "incremental")
        self.mode_combo.addItem("Continuous", "continuous")
        self.mode_combo.currentIndexChanged.connect(self._on_change)
        layout.addWidget(self.mode_combo, 2, 1, 1, 2)

        # Row 3: Info line
        self.info_label = QLabel("")
        self.info_label.setStyleSheet(
            f"color: {COLORS.get('subtext0', '#a6adc8')}; font-size: 9pt;")
        layout.addWidget(self.info_label, 3, 0, 1, 3)

        self._update_controls()

    def _on_enable_changed(self, checked: bool):
        self._update_controls()
        self.changed.emit()

    def _on_change(self, _idx=None):
        self._update_info()
        self.changed.emit()

    def _update_controls(self):
        enabled = self.enable_check.isChecked()
        self.syringe_combo.setEnabled(enabled)
        self.ink_combo.setEnabled(enabled)
        self.mode_combo.setEnabled(enabled)
        self._update_info()

    def _update_info(self):
        vol = self.syringe_combo.currentData()
        if vol and vol in self.syringe_catalog:
            spec = self.syringe_catalog[vol]
            self.info_label.setText(
                f"{spec.volume_uL} µL | "
                f"{spec.uL_per_mm:.2f} µL/mm | "
                f"Stroke: {spec.stroke_length_mm} mm")
        else:
            self.info_label.setText("")

    def set_ink_names(self, ink_names: list[str], excluded: set[str] | None = None):
        """
        Update ink combo options.

        v7.2.4: excluded inks are shown grayed out (assigned to other pumps).
        """
        current = self.ink_combo.currentData()
        self._ink_names = ink_names
        self._excluded_inks = excluded or set()

        self.ink_combo.blockSignals(True)
        self.ink_combo.clear()
        self.ink_combo.addItem("— None —", None)

        for name in ink_names:
            self.ink_combo.addItem(name, name)
            idx = self.ink_combo.count() - 1
            # v7.2.4: Gray out inks assigned to other pumps
            if name in self._excluded_inks:
                item = self.ink_combo.model().item(idx)
                if item:
                    item.setFlags(item.flags() & ~Qt.ItemIsEnabled)
                    item.setForeground(QColor(COLORS.get('surface2', '#585b70')))

        # Restore selection if still available and not excluded
        if current:
            idx = self.ink_combo.findData(current)
            if idx >= 0:
                self.ink_combo.setCurrentIndex(idx)
        self.ink_combo.blockSignals(False)

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
        self.blockSignals(True)

        # Populate ink names first if provided
        if ink_names is not None:
            self.set_ink_names(ink_names)

        self.enable_check.setChecked(config.enabled)

        # Syringe
        if config.syringe:
            idx = self.syringe_combo.findData(config.syringe.volume_uL)
            if idx >= 0:
                self.syringe_combo.setCurrentIndex(idx)
            else:
                logger.warning(
                    f"{self.pump_id}: Syringe {config.syringe.volume_uL}µL "
                    f"not found in catalog")
        else:
            self.syringe_combo.setCurrentIndex(0)

        # Ink
        if config.ink:
            idx = self.ink_combo.findData(config.ink.name)
            if idx >= 0:
                self.ink_combo.setCurrentIndex(idx)
            else:
                logger.warning(
                    f"{self.pump_id}: Ink '{config.ink.name}' "
                    f"not found in ink combo")
        else:
            self.ink_combo.setCurrentIndex(0)

        # Mode
        mode_idx = self.mode_combo.findData(config.printing_mode.value)
        if mode_idx >= 0:
            self.mode_combo.setCurrentIndex(mode_idx)

        self._update_controls()
        self.blockSignals(False)

    def get_selected_ink_name(self) -> str | None:
        """Get currently selected ink name (for exclusion tracking)."""
        return self.ink_combo.currentData()


# ═══════════════════════════════════════════════════════════════════
# Hardware Setup Page (v7.2.4)
# ═══════════════════════════════════════════════════════════════════

class HardwareSetupPage(QWidget):
    """
    Hardware Setup — the gating page that must be completed before
    any other page can operate.

    v7.2.4: Reordered sections, added rosette library, fixed config restore.
    v7.2.4 Session 3: Pump-ink exclusivity, needle channel mapping.
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

        # v7.2.4: Channel mapping widgets (dynamic)
        self._channel_map_widgets: list[tuple[QLabel, QComboBox]] = []

        self._setup_ui()

    def get_page_title(self) -> str:
        return "Hardware Setup"

    @property
    def hardware_config(self) -> HardwareConfig:
        return self._config

    # ════════════════════════════════════════════════════════════════
    #  UI CONSTRUCTION — v7.2.4 reordered
    # ════════════════════════════════════════════════════════════════

    def _setup_ui(self):
        """Build the main page layout with v7.2.4 section ordering."""
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)

        scroll_content = QWidget()
        self._content_layout = QVBoxLayout(scroll_content)
        self._content_layout.setSpacing(12)
        self._content_layout.setContentsMargins(12, 12, 12, 12)

        # ── Section 1: Setup Name & Notes ─────────────────────────
        name_group = QGroupBox("Setup Name && Notes")
        name_group.setStyleSheet(self._group_style())
        name_lay = QFormLayout(name_group)

        self.name_edit = QLineEdit()
        self.name_edit.setPlaceholderText("My Experiment Setup")
        self.name_edit.textChanged.connect(self._on_config_changed)
        name_lay.addRow("Name:", self.name_edit)

        self.notes_edit = QLineEdit()
        self.notes_edit.setPlaceholderText("Optional notes...")
        self.notes_edit.textChanged.connect(self._on_config_changed)
        name_lay.addRow("Notes:", self.notes_edit)

        self._content_layout.addWidget(name_group)

        # ── Section 2: Well Plate Format (v7.2.4: MOVED UP) ──────
        plate_group = QGroupBox("Well Plate Format")
        plate_group.setStyleSheet(self._group_style())
        plate_lay = QHBoxLayout(plate_group)

        plate_lay.addWidget(QLabel("Format:"))
        self.plate_combo = QComboBox()
        for fmt in sorted(PLATE_DEFINITIONS.keys()):
            pdef = PLATE_DEFINITIONS[fmt]
            rows = pdef.get("rows", "?")
            cols = pdef.get("cols", "?")
            self.plate_combo.addItem(f"{fmt}-well ({rows}×{cols})", fmt)
        self.plate_combo.currentIndexChanged.connect(self._on_config_changed)
        plate_lay.addWidget(self.plate_combo)
        plate_lay.addStretch()

        self._content_layout.addWidget(plate_group)

        # ── Section 3: Ink Library ────────────────────────────────
        ink_group = QGroupBox("Ink Library")
        ink_group.setStyleSheet(self._group_style())
        ink_lay = QVBoxLayout(ink_group)

        self.ink_table = QTableWidget(0, 5)
        self.ink_table.setHorizontalHeaderLabels(
            ["Name", "Type", "Viscosity", "Granule Ø", "Cell Ø"])
        self.ink_table.horizontalHeader().setSectionResizeMode(
            QHeaderView.Stretch)
        self.ink_table.setSelectionBehavior(
            QAbstractItemView.SelectRows)
        self.ink_table.setSelectionMode(
            QAbstractItemView.SingleSelection)
        self.ink_table.setEditTriggers(
            QAbstractItemView.NoEditTriggers)
        self.ink_table.setMaximumHeight(160)
        ink_lay.addWidget(self.ink_table)

        ink_btns = QHBoxLayout()
        btn_add_ink = QPushButton("+ Add Ink")
        btn_add_ink.clicked.connect(self._add_ink)
        ink_btns.addWidget(btn_add_ink)
        btn_edit_ink = QPushButton("Edit")
        btn_edit_ink.clicked.connect(self._edit_ink)
        ink_btns.addWidget(btn_edit_ink)
        btn_del_ink = QPushButton("Remove")
        btn_del_ink.clicked.connect(self._remove_ink)
        ink_btns.addWidget(btn_del_ink)
        ink_btns.addStretch()
        ink_lay.addLayout(ink_btns)

        self._content_layout.addWidget(ink_group)

        # ── Section 4: Pump Channels (v7.2.4: with exclusive inks) ─
        pump_group = QGroupBox("Pump Channels")
        pump_group.setStyleSheet(self._group_style())
        pump_lay = QVBoxLayout(pump_group)

        self._pump_widgets: dict[str, PumpChannelWidget] = {}
        for pid in ["P1", "P2", "P3"]:
            pw = PumpChannelWidget(pid, self._syringe_catalog)
            pw.changed.connect(self._on_pump_changed)
            pump_lay.addWidget(pw)
            self._pump_widgets[pid] = pw

        # v7.2.4: Pump-ink summary label (S3.6)
        self.pump_ink_summary = QLabel("")
        self.pump_ink_summary.setStyleSheet(
            f"color: {COLORS.get('subtext0', '#a6adc8')}; "
            f"font-size: 9pt; padding: 4px 8px;")
        self.pump_ink_summary.setWordWrap(True)
        pump_lay.addWidget(self.pump_ink_summary)

        self._content_layout.addWidget(pump_group)

        # ── Section 5: Needle Configuration (v7.2.4: MOVED DOWN) ─
        needle_group = QGroupBox("Needle Configuration")
        needle_group.setStyleSheet(self._group_style())
        needle_lay = QGridLayout(needle_group)

        needle_lay.addWidget(QLabel("Gauge:"), 0, 0)
        self.gauge_combo = QComboBox()
        self.gauge_combo.addItem("— Select —", None)
        for gauge in sorted(self._needle_catalog.keys()):
            self.gauge_combo.addItem(f"{gauge}G", gauge)
        self.gauge_combo.currentIndexChanged.connect(self._on_needle_changed)
        needle_lay.addWidget(self.gauge_combo, 0, 1)

        needle_lay.addWidget(QLabel("Length:"), 0, 2)
        self.length_combo = QComboBox()
        self.length_combo.addItem("1.0\"", 1.0)
        self.length_combo.addItem("1.5\"", 1.5)
        self.length_combo.addItem("2.0\"", 2.0)
        self.length_combo.currentIndexChanged.connect(self._on_config_changed)
        needle_lay.addWidget(self.length_combo, 0, 3)

        needle_lay.addWidget(QLabel("Channels:"), 1, 0)
        self.channels_spin = QSpinBox()
        self.channels_spin.setRange(1, 3)
        self.channels_spin.setValue(1)
        self.channels_spin.valueChanged.connect(self._on_channels_changed)
        needle_lay.addWidget(self.channels_spin, 1, 1)

        self.needle_info_label = QLabel("Select a needle gauge above")
        self.needle_info_label.setStyleSheet(
            f"color: {COLORS.get('subtext0', '#a6adc8')}; font-size: 9pt;")
        needle_lay.addWidget(self.needle_info_label, 2, 0, 1, 4)

        self._content_layout.addWidget(needle_group)

        # ── Section 6: Needle Channel → Pump Mapping (v7.2.4: NEW) ─
        self.channel_map_group = QGroupBox("Needle Channel Assignment")
        self.channel_map_group.setStyleSheet(self._group_style())
        self._channel_map_layout = QVBoxLayout(self.channel_map_group)

        # Info label
        self.channel_map_info = QLabel(
            "Each needle channel must be assigned to a unique enabled pump.")
        self.channel_map_info.setStyleSheet(
            f"color: {COLORS.get('subtext0', '#a6adc8')}; font-size: 9pt;")
        self.channel_map_info.setWordWrap(True)
        self._channel_map_layout.addWidget(self.channel_map_info)

        # Container for dynamic rows
        self._channel_rows_widget = QWidget()
        self._channel_rows_layout = QVBoxLayout(self._channel_rows_widget)
        self._channel_rows_layout.setContentsMargins(0, 0, 0, 0)
        self._channel_rows_layout.setSpacing(4)
        self._channel_map_layout.addWidget(self._channel_rows_widget)

        # Validation indicator
        self.channel_map_status = QLabel("")
        self.channel_map_status.setStyleSheet(
            f"font-size: 9pt; padding: 2px 4px;")
        self._channel_map_layout.addWidget(self.channel_map_status)

        self._content_layout.addWidget(self.channel_map_group)

        # Build initial channel rows
        self._rebuild_channel_map_rows()

        # ── Section 7: Rosette Library ────────────────────────────
        ros_group = QGroupBox("Rosette Library")
        ros_group.setStyleSheet(self._group_style())
        ros_lay = QVBoxLayout(ros_group)

        self.rosette_table = QTableWidget(0, 5)
        self.rosette_table.setHorizontalHeaderLabels(
            ["Name", "Sub-wells", "Fits", "Depth", "Z-offset"])
        self.rosette_table.horizontalHeader().setSectionResizeMode(
            QHeaderView.Stretch)
        self.rosette_table.setSelectionBehavior(
            QAbstractItemView.SelectRows)
        self.rosette_table.setSelectionMode(
            QAbstractItemView.SingleSelection)
        self.rosette_table.setEditTriggers(
            QAbstractItemView.NoEditTriggers)
        self.rosette_table.setMaximumHeight(140)
        ros_lay.addWidget(self.rosette_table)

        ros_btns = QHBoxLayout()
        btn_add_ros = QPushButton("+ Add Rosette")
        btn_add_ros.clicked.connect(self._add_rosette)
        ros_btns.addWidget(btn_add_ros)
        btn_edit_ros = QPushButton("Edit")
        btn_edit_ros.clicked.connect(self._edit_rosette)
        ros_btns.addWidget(btn_edit_ros)
        btn_del_ros = QPushButton("Remove")
        btn_del_ros.clicked.connect(self._remove_rosette)
        ros_btns.addWidget(btn_del_ros)
        ros_btns.addStretch()
        ros_lay.addLayout(ros_btns)

        self._content_layout.addWidget(ros_group)

        # ── Section 8: Actions ────────────────────────────────────
        actions_group = QGroupBox("Actions")
        actions_group.setStyleSheet(self._group_style())
        actions_lay = QHBoxLayout(actions_group)

        btn_save = QPushButton("💾 Save Config")
        btn_save.clicked.connect(self._save_config)
        actions_lay.addWidget(btn_save)

        btn_load = QPushButton("📂 Load Config")
        btn_load.clicked.connect(self._load_config)
        actions_lay.addWidget(btn_load)

        actions_lay.addStretch()

        self.validity_label = QLabel("⚠ Setup incomplete")
        self.validity_label.setStyleSheet(
            f"color: {COLORS.get('yellow', '#f9e2af')};")
        self.validity_label.setFont(QFont("", 10, QFont.Bold))
        actions_lay.addWidget(self.validity_label)

        self._content_layout.addWidget(actions_group)

        # ── Finalize scroll area ──────────────────────────────────
        self._content_layout.addStretch()
        scroll.setWidget(scroll_content)
        outer.addWidget(scroll)

    # ════════════════════════════════════════════════════════════════
    #  SHARED STYLES
    # ════════════════════════════════════════════════════════════════

    @staticmethod
    def _group_style() -> str:
        """v7.2.4: Delegates to centralized SECTION_TITLE_STYLE."""
        return SECTION_TITLE_STYLE

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

    def _on_channels_changed(self, value: int):
        """v7.2.4 S3.9: Rebuild channel mapping rows when channel count changes."""
        self._rebuild_channel_map_rows()
        self._on_config_changed()

    # ════════════════════════════════════════════════════════════════
    #  v7.2.4: PUMP CHANGE HANDLER (exclusive inks + channel map)
    # ════════════════════════════════════════════════════════════════

    def _on_pump_changed(self):
        """
        Called when any pump widget changes.

        v7.2.4: Refreshes ink exclusions across pumps and
        updates channel mapping pump combos.
        """
        self._refresh_pump_ink_exclusions()
        self._update_pump_ink_summary()
        self._refresh_channel_map_pump_options()
        self._on_config_changed()

    def _refresh_pump_ink_exclusions(self):
        """
        v7.2.4 S3.5: Update ink combos to gray out inks
        already assigned to other pumps.
        """
        ink_names = list(self._config.ink_library.keys())

        for pid, pw in self._pump_widgets.items():
            # Collect inks used by OTHER pumps
            excluded = set()
            for other_pid, other_pw in self._pump_widgets.items():
                if other_pid != pid:
                    selected = other_pw.get_selected_ink_name()
                    if selected:
                        excluded.add(selected)
            pw.set_ink_names(ink_names, excluded)

    def _update_pump_ink_summary(self):
        """v7.2.4 S3.6: Update pump-ink summary label."""
        parts = []
        for pid in ["P1", "P2", "P3"]:
            pw = self._pump_widgets[pid]
            if pw.enable_check.isChecked():
                ink_name = pw.get_selected_ink_name()
                parts.append(f"{pid}→{ink_name or '(none)'}")
        if parts:
            self.pump_ink_summary.setText("Assignment: " + ", ".join(parts))
        else:
            self.pump_ink_summary.setText("No pumps enabled")

    # ════════════════════════════════════════════════════════════════
    #  v7.2.4: NEEDLE CHANNEL MAP (S3.7-S3.10)
    # ════════════════════════════════════════════════════════════════

    def _rebuild_channel_map_rows(self):
        """
        v7.2.4 S3.7: Build N rows for channel→pump assignment
        based on current needle channel count.
        """
        # Clear existing rows
        self._channel_map_widgets.clear()
        while self._channel_rows_layout.count():
            item = self._channel_rows_layout.takeAt(0)
            if item.widget():
                item.widget().deleteLater()

        num_channels = self.channels_spin.value()
        enabled_pumps = self._get_enabled_pump_ids()

        for ch_idx in range(num_channels):
            row_widget = QWidget()
            row_layout = QHBoxLayout(row_widget)
            row_layout.setContentsMargins(0, 0, 0, 0)
            row_layout.setSpacing(8)

            if num_channels == 1:
                label = QLabel("Bore →")
            else:
                label = QLabel(f"Channel {ch_idx + 1} →")
            label.setMinimumWidth(80)
            row_layout.addWidget(label)

            combo = QComboBox()
            combo.addItem("— Unassigned —", None)
            for pid in enabled_pumps:
                combo.addItem(pid, pid)
            combo.currentIndexChanged.connect(
                partial(self._on_channel_map_changed, ch_idx))
            row_layout.addWidget(combo)
            row_layout.addStretch()

            self._channel_rows_layout.addWidget(row_widget)
            self._channel_map_widgets.append((label, combo))

        self._update_channel_map_status()

    def _refresh_channel_map_pump_options(self):
        """
        v7.2.4 S3.10: Refresh the pump combos in channel mapping
        when pumps are enabled/disabled.
        """
        enabled_pumps = self._get_enabled_pump_ids()

        for ch_idx, (label, combo) in enumerate(self._channel_map_widgets):
            current = combo.currentData()
            combo.blockSignals(True)
            combo.clear()
            combo.addItem("— Unassigned —", None)
            for pid in enabled_pumps:
                combo.addItem(pid, pid)
            # Restore selection if still valid
            if current:
                idx = combo.findData(current)
                if idx >= 0:
                    combo.setCurrentIndex(idx)
            combo.blockSignals(False)

        self._update_channel_map_status()

    def _on_channel_map_changed(self, ch_idx: int, _combo_idx: int = None):
        """v7.2.4 S3.8: Handle channel mapping combo change."""
        self._update_channel_map_status()
        self._on_config_changed()

    def _update_channel_map_status(self):
        """v7.2.4: Update channel mapping validation indicator."""
        num_channels = self.channels_spin.value()
        assigned_pumps = set()
        all_assigned = True
        has_duplicate = False

        for ch_idx, (label, combo) in enumerate(self._channel_map_widgets):
            pid = combo.currentData()
            if pid is None:
                all_assigned = False
            elif pid in assigned_pumps:
                has_duplicate = True
            else:
                assigned_pumps.add(pid)

        if num_channels == 0:
            self.channel_map_status.setText("")
        elif has_duplicate:
            self.channel_map_status.setText("⚠ Duplicate pump assignment")
            self.channel_map_status.setStyleSheet(
                f"color: {COLORS.get('red', '#f38ba8')}; font-size: 9pt;")
        elif not all_assigned:
            self.channel_map_status.setText(
                f"⚠ {num_channels - len(assigned_pumps)} channel(s) unassigned")
            self.channel_map_status.setStyleSheet(
                f"color: {COLORS.get('yellow', '#f9e2af')}; font-size: 9pt;")
        else:
            self.channel_map_status.setText("✓ All channels assigned")
            self.channel_map_status.setStyleSheet(
                f"color: {COLORS.get('green', '#a6e3a1')}; font-size: 9pt;")

    def _get_enabled_pump_ids(self) -> list[str]:
        """Get list of currently enabled pump IDs from widgets."""
        return [
            pid for pid, pw in self._pump_widgets.items()
            if pw.enable_check.isChecked()
        ]

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
            # v7.2.4: Also update context panel validity
            if hasattr(self, '_ctx_validity_label'):
                self._ctx_validity_label.setText("✓ Setup complete")
                self._ctx_validity_label.setStyleSheet(
                    f"color: {COLORS.get('green', '#a6e3a1')};")
        else:
            _, issues = self._config.validate()
            self.validity_label.setText(
                f"⚠ {issues[0]}" if issues else "⚠ Setup incomplete")
            self.validity_label.setStyleSheet(
                f"color: {COLORS.get('yellow', '#f9e2af')};")
            # v7.2.4: Also update context panel validity
            if hasattr(self, '_ctx_validity_label'):
                issue_text = issues[0] if issues else "Setup incomplete"
                self._ctx_validity_label.setText(f"⚠ {issue_text}")
                self._ctx_validity_label.setStyleSheet(
                    f"color: {COLORS.get('yellow', '#f9e2af')};")

        self.config_changed.emit(self._config)

    def _rebuild_config(self):
        """Rebuild HardwareConfig from all widget states."""
        # Name & notes
        self._config.config_name = self.name_edit.text().strip() or "Untitled Setup"
        self._config.notes = self.notes_edit.text().strip()

        # Well plate
        self._config.plate_format = self.plate_combo.currentData() or 24

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

        # Pumps — resolve ink from library
        for pid, pw in self._pump_widgets.items():
            pcfg = pw.get_config()
            if pcfg.ink and pcfg.ink.name in self._config.ink_library:
                pcfg.ink = self._config.ink_library[pcfg.ink.name]
            elif pcfg.ink:
                pcfg.ink = None  # Ink no longer in library
            self._config.pumps[pid] = pcfg

        # v7.2.4 S3.12: Capture channel map state
        self._config.needle_channel_pump_map.clear()
        for ch_idx, (label, combo) in enumerate(self._channel_map_widgets):
            pid = combo.currentData()
            if pid:
                self._config.needle_channel_pump_map[ch_idx] = pid

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
            # Color indicator in name cell
            name_item = QTableWidgetItem(ink.name)
            name_item.setForeground(QColor(ink.color))
            self.ink_table.setItem(row, 0, name_item)
            self.ink_table.setItem(row, 1, QTableWidgetItem(ink.ink_type))
            self.ink_table.setItem(row, 2, QTableWidgetItem(
                f"{ink.viscosity_cP:.1f} cP"))
            self.ink_table.setItem(row, 3, QTableWidgetItem(
                f"{ink.granule_diameter_um:.0f} µm" if ink.granule_diameter_um else "—"))
            self.ink_table.setItem(row, 4, QTableWidgetItem(
                f"{ink.cell_diameter_um:.0f} µm" if ink.cell_diameter_um else "—"))

    def _refresh_pump_ink_combos(self):
        """Refresh ink names in all pump combos with exclusion support."""
        self._refresh_pump_ink_exclusions()

    # ════════════════════════════════════════════════════════════════
    #  ROSETTE LIBRARY CRUD
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
        name = self.rosette_table.item(row, 0).text()
        ros = self._config.rosette_library.get(name)
        if not ros:
            return
        dlg = RosetteEditorDialog(rosette=ros, parent=self)
        if dlg.exec() == QDialog.Accepted:
            new_ros = dlg.get_rosette()
            if new_ros.name != name:
                self._config.rosette_library.pop(name, None)
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
        """Rebuild the rosette table from config."""
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
                # v7.2.4: Refresh the config file browser
                self._scan_config_directory()
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
                # v7.2.4: Refresh and highlight in file browser
                self._scan_config_directory()
            except Exception as e:
                QMessageBox.critical(
                    self, "Error", f"Failed to load:\n{e}")

    # ════════════════════════════════════════════════════════════════
    #  APPLY CONFIG TO UI (v7.2.4: adds channel map restore)
    # ════════════════════════════════════════════════════════════════

    def _apply_config_to_ui(self):
        """
        Push the current config state into all UI widgets.

        v7.2.4 FIX: Restores in dependency order:
        1. Name & Notes
        2. Plate format
        3. Ink library → table + pump combo refresh
        4. Rosette library → table
        5. Needle gauge + length + channels
        6. Pump channels (with ink_names available)
        7. Needle channel → pump map (v7.2.4 new)
        8. Emit signals
        """
        logger.info("Applying config to UI...")

        # ── 1. Name & Notes ──────────────────────────────────────
        self.name_edit.blockSignals(True)
        self.name_edit.setText(self._config.config_name)
        self.name_edit.blockSignals(False)

        self.notes_edit.blockSignals(True)
        self.notes_edit.setText(self._config.notes)
        self.notes_edit.blockSignals(False)
        logger.debug(f"  Name: {self._config.config_name}")

        # ── 2. Plate Format ──────────────────────────────────────
        self.plate_combo.blockSignals(True)
        pidx = self.plate_combo.findData(self._config.plate_format)
        if pidx >= 0:
            self.plate_combo.setCurrentIndex(pidx)
        self.plate_combo.blockSignals(False)
        logger.debug(f"  Plate format: {self._config.plate_format}")

        # ── 3. Ink Library (MUST come before pumps) ──────────────
        self._refresh_ink_table()
        ink_names = list(self._config.ink_library.keys())
        logger.debug(f"  Ink library: {len(ink_names)} inks")

        # ── 4. Rosette Library ───────────────────────────────────
        self._refresh_rosette_table()
        logger.debug(
            f"  Rosette library: {len(self._config.rosette_library)} rosettes")

        # ── 5. Needle Config ─────────────────────────────────────
        self.gauge_combo.blockSignals(True)
        if self._config.needle:
            gidx = self.gauge_combo.findData(self._config.needle.gauge)
            if gidx >= 0:
                self.gauge_combo.setCurrentIndex(gidx)
            # Length
            lidx = self.length_combo.findData(self._config.needle.length_inches)
            if lidx >= 0:
                self.length_combo.blockSignals(True)
                self.length_combo.setCurrentIndex(lidx)
                self.length_combo.blockSignals(False)
            # Channels
            self.channels_spin.blockSignals(True)
            self.channels_spin.setValue(self._config.needle.num_channels)
            self.channels_spin.blockSignals(False)
            logger.debug(
                f"  Needle: {self._config.needle.gauge}G, "
                f"{self._config.needle.num_channels} channel(s)")
        else:
            self.gauge_combo.setCurrentIndex(0)
        self.gauge_combo.blockSignals(False)
        self._on_needle_changed()  # Update info label

        # ── 6. Pump Channels (ink combos now populated) ──────────
        for pid, pw in self._pump_widgets.items():
            if pid in self._config.pumps:
                pcfg = self._config.pumps[pid]
                pw.set_config(pcfg, ink_names=ink_names)
                logger.debug(
                    f"  {pid}: enabled={pcfg.enabled}, "
                    f"syringe={pcfg.syringe.volume_uL if pcfg.syringe else None}µL, "
                    f"ink={pcfg.ink.name if pcfg.ink else None}, "
                    f"mode={pcfg.printing_mode.value}")

        # Refresh exclusions after all pumps loaded
        self._refresh_pump_ink_exclusions()
        self._update_pump_ink_summary()

        # ── 7. Needle Channel → Pump Map (v7.2.4 S3.11) ─────────
        self._rebuild_channel_map_rows()
        for ch_idx, (label, combo) in enumerate(self._channel_map_widgets):
            mapped_pump = self._config.needle_channel_pump_map.get(ch_idx)
            if mapped_pump:
                idx = combo.findData(mapped_pump)
                if idx >= 0:
                    combo.blockSignals(True)
                    combo.setCurrentIndex(idx)
                    combo.blockSignals(False)
        self._update_channel_map_status()
        logger.debug(
            f"  Channel map: {self._config.needle_channel_pump_map}")

        # ── 8. Emit signals ──────────────────────────────────────
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
        """v7.2.4: Context panel with saved config file browser."""
        if self._context_widget is not None:
            return self._context_widget

        ctx = QWidget()
        layout = QVBoxLayout(ctx)
        layout.setContentsMargins(10, 8, 10, 8)
        layout.setSpacing(6)

        # ── Saved Configurations ────────────────────────────
        configs_label = QLabel("Saved Configurations")
        configs_label.setObjectName("contextSectionLabel")
        layout.addWidget(configs_label)

        self._config_list = QListWidget()
        self._config_list.setAlternatingRowColors(True)
        self._config_list.setMaximumHeight(260)
        self._config_list.itemDoubleClicked.connect(
            self._on_config_list_double_click)
        self._config_list.currentItemChanged.connect(
            self._on_config_list_selection)
        layout.addWidget(self._config_list)

        # Active config indicator
        self._lbl_active_config = QLabel("Active: (unsaved)")
        self._lbl_active_config.setStyleSheet(
            f"color: {COLORS.get('subtext0', '#a6adc8')}; "
            f"font-size: 9pt; font-style: italic;")
        layout.addWidget(self._lbl_active_config)

        # Buttons row
        btn_row = QHBoxLayout()
        btn_load = QPushButton("Load")
        btn_load.setMaximumHeight(26)
        btn_load.clicked.connect(self._on_config_list_load)
        btn_row.addWidget(btn_load)

        btn_del = QPushButton("Delete")
        btn_del.setMaximumHeight(26)
        btn_del.setObjectName("dangerBtn")
        btn_del.clicked.connect(self._on_config_list_delete)
        btn_row.addWidget(btn_del)

        btn_refresh = QPushButton("🔄")
        btn_refresh.setMaximumHeight(26)
        btn_refresh.setMaximumWidth(32)
        btn_refresh.setToolTip("Refresh config file list")
        btn_refresh.clicked.connect(self._scan_config_directory)
        btn_row.addWidget(btn_refresh)
        layout.addLayout(btn_row)

        # ── Validity Status ─────────────────────────────────
        status_label = QLabel("Setup Status")
        status_label.setObjectName("contextSectionLabel")
        layout.addWidget(status_label)

        self._ctx_validity_label = QLabel("⚠ Setup incomplete")
        self._ctx_validity_label.setStyleSheet(
            f"color: {COLORS.get('yellow', '#f9e2af')}; font-size: 9pt;")
        self._ctx_validity_label.setWordWrap(True)
        layout.addWidget(self._ctx_validity_label)

        layout.addStretch()

        self._context_widget = ctx

        # Initial scan
        self._scan_config_directory()

        return ctx

    # ════════════════════════════════════════════════════════════════
    #  CONFIG FILE BROWSER (v7.2.4)
    # ════════════════════════════════════════════════════════════════

    def _scan_config_directory(self):
        """Scan config/hardware/ for saved .json config files."""
        if not hasattr(self, '_config_list'):
            return
        self._config_list.clear()
        self._config_file_paths = {}  # name → path

        config_dir = CONFIG_HARDWARE_DIR
        if not config_dir.is_dir():
            logger.warning(f"Config directory not found: {config_dir}")
            return

        json_files = sorted(config_dir.glob("*.json"))
        for jf in json_files:
            try:
                import json
                with open(jf, "r") as f:
                    data = json.load(f)
                cfg_name = data.get("config_name", jf.stem)
                display = f"{cfg_name}  ({jf.name})"
                self._config_list.addItem(display)
                self._config_file_paths[display] = jf
            except Exception as e:
                logger.debug(f"Skipping {jf.name}: {e}")
                self._config_list.addItem(f"⚠ {jf.name} (invalid)")

        # Highlight active config if it matches
        self._highlight_active_config()
        logger.debug(f"Config browser: found {len(json_files)} files in {config_dir}")

    def _highlight_active_config(self):
        """Highlight the currently active config in the list."""
        if not hasattr(self, '_config_list'):
            return
        active_name = self._config.config_name
        for i in range(self._config_list.count()):
            item = self._config_list.item(i)
            text = item.text()
            if text.startswith(active_name):
                item.setSelected(True)
                self._config_list.setCurrentItem(item)
                break
        if hasattr(self, '_lbl_active_config'):
            self._lbl_active_config.setText(f"Active: {active_name}")

    def _on_config_list_selection(self, current, previous):
        """Handle selection change in config list."""
        pass  # Selection visual is handled by QListWidget

    def _on_config_list_double_click(self, item):
        """Double-click loads the config."""
        self._on_config_list_load()

    def _on_config_list_load(self):
        """Load the selected config from the file browser."""
        if not hasattr(self, '_config_list'):
            return
        current = self._config_list.currentItem()
        if current is None:
            return
        display = current.text()
        path = self._config_file_paths.get(display)
        if path and path.exists():
            try:
                self._config = HardwareConfig.load(str(path))
                self._apply_config_to_ui()
                self._highlight_active_config()
                logger.info(f"Loaded config from browser: {path.name}")
            except Exception as e:
                QMessageBox.critical(
                    self, "Load Error", f"Failed to load:\n{e}")

    def _on_config_list_delete(self):
        """Delete the selected config file (with confirmation)."""
        if not hasattr(self, '_config_list'):
            return
        current = self._config_list.currentItem()
        if current is None:
            return
        display = current.text()
        path = self._config_file_paths.get(display)
        if not path or not path.exists():
            return
        reply = QMessageBox.question(
            self, "Delete Config",
            f"Delete config file?\n\n{path.name}\n\n"
            f"This cannot be undone.",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if reply == QMessageBox.Yes:
            try:
                path.unlink()
                self._scan_config_directory()
                logger.info(f"Deleted config file: {path.name}")
            except Exception as e:
                QMessageBox.critical(
                    self, "Delete Error", f"Failed to delete:\n{e}")

"""
print_workspace.py — Tab 1: Setup Workspace for MEBP v7.1.

Configure the physical setup before designing prints:
- Plate format selection
- Needle gauge + length from JSON catalog
- Pump loadout: syringe selection, printing mode, fluid column display
- Ink library manager (add/edit/delete, persisted)
- Rosette library manager (add/edit/delete)
- Buffer material + dead volume
- Print settings defaults (travel Z, layer height, speeds, retract/prime)
- Auto-computed compatibility report (flow physics)
- Save/Load workspace JSON
- Signal propagation to other tabs

All needle/syringe catalogs loaded from config/hardware/ JSON files.
"""

from __future__ import annotations

import json
import logging
from pathlib import Path
from functools import partial

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QGroupBox,
    QLabel, QPushButton, QComboBox, QDoubleSpinBox, QSpinBox,
    QTabWidget, QFileDialog, QFrame, QTableWidget, QTableWidgetItem,
    QHeaderView, QAbstractItemView, QDialog, QFormLayout, QLineEdit,
    QDialogButtonBox, QScrollArea, QSizePolicy, QMessageBox,
)
from PySide6.QtCore import Qt, Signal, QSize
from PySide6.QtGui import QColor, QPainter, QFont, QPen, QBrush

from gui.styles import COLORS

from SupportClasses.PhysicalModels import (
    NeedleSpec, SyringeSpec, InkSpec, FluidColumn, PumpLoadout,
    PrintingMode, RosetteInsert, WorkspaceConfig,
    load_needle_catalog, load_syringe_catalog,
)
from SupportClasses.FlowPhysics import (
    calculate_flow_safety, generate_compatibility_report,
)

logger = logging.getLogger(__name__)


# ═══════════════════════════════════════════════════════════════════
# Fluid Column Bar Widget
# ═══════════════════════════════════════════════════════════════════

class FluidColumnBar(QWidget):
    """
    Horizontal stacked bar showing oil/buffer/ink layers in a syringe.

    Colors: oil=gray, buffer=light blue, ink=ink's color, empty=dark.
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(18)
        self.setMaximumHeight(22)
        self._fractions = {"oil": 0.0, "buffer": 0.0, "ink": 0.0, "empty": 1.0}
        self._ink_color = "#a6e3a1"
        self._label_text = ""

    def set_data(self, fractions: dict, ink_color: str = "#a6e3a1", label: str = ""):
        self._fractions = fractions
        self._ink_color = ink_color
        self._label_text = label
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        r = 4  # corner radius

        colors = {
            "oil": QColor("#6c7086"),
            "buffer": QColor("#89b4fa"),
            "ink": QColor(self._ink_color),
            "empty": QColor("#313244"),
        }

        # Draw segments left to right: oil → buffer → ink → empty
        x = 0
        for key in ["oil", "buffer", "ink", "empty"]:
            frac = self._fractions.get(key, 0.0)
            seg_w = int(frac * w)
            if seg_w <= 0:
                continue
            painter.setBrush(QBrush(colors[key]))
            painter.setPen(Qt.NoPen)
            painter.drawRoundedRect(int(x), 0, seg_w, h, r, r)
            x += seg_w

        # Border
        painter.setBrush(Qt.NoBrush)
        painter.setPen(QPen(QColor(COLORS["surface1"]), 1))
        painter.drawRoundedRect(0, 0, w - 1, h - 1, r, r)

        # Needle tip indicator
        painter.setPen(QPen(QColor(COLORS["text"]), 1))
        painter.drawText(w - 20, 0, 18, h, Qt.AlignCenter, "→")

        # Label
        if self._label_text:
            painter.setPen(QPen(QColor(COLORS["text"]), 1))
            f = painter.font()
            f.setPointSize(7)
            painter.setFont(f)
            painter.drawText(4, 0, w - 26, h, Qt.AlignVCenter | Qt.AlignLeft,
                           self._label_text)
        painter.end()


# ═══════════════════════════════════════════════════════════════════
# Ink Editor Dialog
# ═══════════════════════════════════════════════════════════════════

class InkEditorDialog(QDialog):
    """Dialog for adding/editing an ink specification."""

    INK_TYPES = ["hydrogel", "cells", "media", "buffer", "granular", "custom"]
    COLOR_PRESETS = [
        ("#a6e3a1", "Green"), ("#89b4fa", "Blue"), ("#f38ba8", "Red"),
        ("#f9e2af", "Yellow"), ("#cba6f7", "Mauve"), ("#fab387", "Peach"),
        ("#f5c2e7", "Pink"), ("#94e2d5", "Teal"), ("#74c7ec", "Sapphire"),
    ]

    def __init__(self, ink: InkSpec | None = None, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Edit Ink" if ink else "New Ink")
        self.setMinimumWidth(350)
        self._ink = ink

        layout = QFormLayout(self)

        self.name_edit = QLineEdit(ink.name if ink else "")
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
        self.granule_spin.setRange(0, 5000)
        self.granule_spin.setDecimals(1)
        self.granule_spin.setSuffix(" µm")
        self.granule_spin.setValue(ink.granule_diameter_um if ink else 0)
        layout.addRow("Granule Ø:", self.granule_spin)

        self.cell_spin = QDoubleSpinBox()
        self.cell_spin.setRange(0, 5000)
        self.cell_spin.setDecimals(1)
        self.cell_spin.setSuffix(" µm")
        self.cell_spin.setValue(ink.cell_diameter_um if ink else 0)
        layout.addRow("Cell Ø:", self.cell_spin)

        self.density_spin = QDoubleSpinBox()
        self.density_spin.setRange(0.1, 20.0)
        self.density_spin.setDecimals(2)
        self.density_spin.setSuffix(" g/mL")
        self.density_spin.setValue(ink.density_g_mL if ink else 1.0)
        layout.addRow("Density:", self.density_spin)

        self.color_combo = QComboBox()
        for hex_val, name in self.COLOR_PRESETS:
            self.color_combo.addItem(f"{name} ({hex_val})", hex_val)
        if ink:
            for i in range(self.color_combo.count()):
                if self.color_combo.itemData(i) == ink.color:
                    self.color_combo.setCurrentIndex(i)
                    break
        layout.addRow("Color:", self.color_combo)

        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addRow(buttons)

    def get_ink(self) -> InkSpec:
        return InkSpec(
            name=self.name_edit.text().strip() or "Unnamed",
            ink_type=self.type_combo.currentText(),
            viscosity_cP=self.viscosity_spin.value(),
            granule_diameter_um=self.granule_spin.value(),
            cell_diameter_um=self.cell_spin.value(),
            density_g_mL=self.density_spin.value(),
            color=self.color_combo.currentData() or "#a6e3a1",
        )


# ═══════════════════════════════════════════════════════════════════
# Rosette Editor Dialog
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
        for fmt in [6, 12, 24, 48, 96]:
            self.format_combo.addItem(f"{fmt}-well", fmt)
        if rosette:
            idx = self.format_combo.findData(rosette.well_format)
            if idx >= 0:
                self.format_combo.setCurrentIndex(idx)
        layout.addRow("Fits plate:", self.format_combo)

        self.ring_spin = QSpinBox()
        self.ring_spin.setRange(2, 12)
        self.ring_spin.setValue((rosette.num_subwells - (1 if rosette and rosette.has_center_well else 0))
                               if rosette else 6)
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
# Main Workspace Tab
# ═══════════════════════════════════════════════════════════════════

class WorkspaceTab(QWidget):
    """
    Tab 1: Setup Workspace — configure physical hardware before printing.

    Sections:
    1. Plate format selector
    2. Needle configuration (gauge, length, channels)
    3. Pump loadout (syringe, mode, fluid column bar)
    4. Ink library (add/edit/delete table)
    5. Rosette library (add/edit/delete table)
    6. Print settings defaults
    7. Auto-compatibility report
    """

    # Emitted when workspace config changes — other tabs listen to this
    workspace_changed = Signal(object)  # WorkspaceConfig

    def __init__(self, controller=None, settings=None, parent=None):
        super().__init__(parent)
        self.controller = controller
        self.settings = settings

        # Load catalogs from JSON
        self._needle_catalog = load_needle_catalog()
        self._syringe_catalog = load_syringe_catalog()

        # Active workspace config
        self.workspace = WorkspaceConfig()

        # Build UI
        self._setup_ui()
        self._apply_defaults()

    # ────────────────────────────────────────────────────────────────
    #  UI Construction
    # ────────────────────────────────────────────────────────────────

    def _setup_ui(self):
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(8, 4, 8, 4)
        main_layout.setSpacing(6)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setFrameShape(QFrame.NoFrame)

        scroll_content = QWidget()
        self._content_layout = QVBoxLayout(scroll_content)
        self._content_layout.setContentsMargins(4, 4, 4, 4)
        self._content_layout.setSpacing(8)

        self._build_plate_section()
        self._build_needle_section()
        self._build_pump_section()
        self._build_ink_library_section()
        self._build_rosette_library_section()
        self._build_print_settings_section()
        self._build_compatibility_section()

        self._content_layout.addStretch()
        scroll.setWidget(scroll_content)
        main_layout.addWidget(scroll)

    # ── Section builders ───────────────────────────────────────────

    def _make_section(self, title: str) -> tuple[QGroupBox, QVBoxLayout]:
        group = QGroupBox(title)
        group.setStyleSheet(f"""
            QGroupBox {{
                font-weight: bold;
                color: {COLORS['text']};
                border: 1px solid {COLORS['surface1']};
                border-radius: 6px;
                margin-top: 8px;
                padding-top: 14px;
            }}
            QGroupBox::title {{
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 6px;
            }}
        """)
        layout = QVBoxLayout(group)
        layout.setContentsMargins(10, 6, 10, 8)
        layout.setSpacing(4)
        self._content_layout.addWidget(group)
        return group, layout

    # ── 1. Plate Format ────────────────────────────────────────────

    def _build_plate_section(self):
        _, layout = self._make_section("Plate Format")
        row = QHBoxLayout()
        row.addWidget(QLabel("Format:"))
        self._plate_combo = QComboBox()
        for fmt in [6, 12, 24, 48, 96, 384]:
            self._plate_combo.addItem(f"{fmt}-well plate", fmt)
        self._plate_combo.setCurrentIndex(2)  # 24-well default
        self._plate_combo.currentIndexChanged.connect(self._on_plate_changed)
        row.addWidget(self._plate_combo)
        row.addStretch()
        layout.addLayout(row)

    def _on_plate_changed(self):
        self.workspace.plate_format = self._plate_combo.currentData()
        self._emit_workspace()

    # ── 2. Needle Configuration ────────────────────────────────────

    def _build_needle_section(self):
        _, layout = self._make_section("Needle Configuration")
        grid = QGridLayout()
        grid.setSpacing(6)

        grid.addWidget(QLabel("Gauge:"), 0, 0)
        self._gauge_combo = QComboBox()
        gauges = sorted(self._needle_catalog.keys())
        for g in gauges:
            self._gauge_combo.addItem(f"{g}G", g)
        self._gauge_combo.setCurrentIndex(gauges.index(22) if 22 in gauges else 0)
        self._gauge_combo.currentIndexChanged.connect(self._on_needle_changed)
        grid.addWidget(self._gauge_combo, 0, 1)

        grid.addWidget(QLabel("Length:"), 0, 2)
        self._length_combo = QComboBox()
        self._length_combo.addItem('1.0"', 1.0)
        self._length_combo.addItem('2.0"', 2.0)
        self._length_combo.currentIndexChanged.connect(self._on_needle_changed)
        grid.addWidget(self._length_combo, 0, 3)

        grid.addWidget(QLabel("Channels:"), 1, 0)
        self._channels_spin = QSpinBox()
        self._channels_spin.setRange(1, 3)
        self._channels_spin.setValue(1)
        self._channels_spin.valueChanged.connect(self._on_needle_changed)
        grid.addWidget(self._channels_spin, 1, 1)

        # Auto-calculated dimensions
        self._needle_info = QLabel("")
        self._needle_info.setStyleSheet(f"color: {COLORS['subtext0']}; font-size: 11px;")
        grid.addWidget(self._needle_info, 1, 2, 1, 2)

        layout.addLayout(grid)

    def _on_needle_changed(self):
        gauge = self._gauge_combo.currentData()
        if gauge is None:
            return
        spec = self._needle_catalog.get(gauge)
        if spec is None:
            return

        # Clone and apply user selections
        needle = NeedleSpec(
            gauge=spec.gauge,
            od_um=spec.od_um,
            id_um=spec.id_um,
            wall_um=spec.wall_um,
            length_inches=self._length_combo.currentData() or 1.0,
            num_channels=self._channels_spin.value(),
        )
        self.workspace.needle = needle
        self._needle_info.setText(
            f"ID: {needle.id_um:.0f} µm  OD: {needle.od_um:.0f} µm  "
            f"Length: {needle.length_mm:.1f} mm"
        )
        self._update_compatibility()
        self._emit_workspace()

    # ── 3. Pump Loadout ────────────────────────────────────────────

    def _build_pump_section(self):
        _, layout = self._make_section("Pump Loadout")

        self._pump_widgets: dict[str, dict] = {}
        for pump_id in ["P1", "P2", "P3"]:
            pw = self._build_single_pump(pump_id)
            layout.addLayout(pw["layout"])
            self._pump_widgets[pump_id] = pw

        # Buffer material + dead volume
        buf_row = QHBoxLayout()
        buf_row.addWidget(QLabel("Buffer material:"))
        self._buffer_combo = QComboBox()
        self._buffer_combo.addItem("(none)")
        self._buffer_combo.currentIndexChanged.connect(self._on_buffer_changed)
        buf_row.addWidget(self._buffer_combo)

        buf_row.addWidget(QLabel("Dead volume:"))
        self._dead_vol_spin = QDoubleSpinBox()
        self._dead_vol_spin.setRange(0.0, 50.0)
        self._dead_vol_spin.setDecimals(1)
        self._dead_vol_spin.setSuffix(" µL")
        self._dead_vol_spin.setValue(2.0)
        self._dead_vol_spin.valueChanged.connect(self._on_dead_volume_changed)
        buf_row.addWidget(self._dead_vol_spin)
        buf_row.addStretch()
        layout.addLayout(buf_row)

    def _build_single_pump(self, pump_id: str) -> dict:
        row_layout = QHBoxLayout()
        row_layout.setSpacing(6)

        lbl = QLabel(f"{pump_id}:")
        lbl.setFixedWidth(28)
        lbl.setStyleSheet(f"font-weight: bold; color: {COLORS['blue']};")
        row_layout.addWidget(lbl)

        syringe_combo = QComboBox()
        syringe_combo.addItem("None", None)
        for vol in sorted(self._syringe_catalog.keys()):
            spec = self._syringe_catalog[vol]
            syringe_combo.addItem(f"{vol} µL ({spec.part_number})", vol)
        syringe_combo.setFixedWidth(160)
        syringe_combo.currentIndexChanged.connect(
            partial(self._on_syringe_changed, pump_id)
        )
        row_layout.addWidget(syringe_combo)

        mode_combo = QComboBox()
        mode_combo.addItem("Incremental", PrintingMode.INCREMENTAL.value)
        mode_combo.addItem("Continuous", PrintingMode.CONTINUOUS.value)
        mode_combo.setFixedWidth(110)
        mode_combo.currentIndexChanged.connect(
            partial(self._on_mode_changed, pump_id)
        )
        row_layout.addWidget(mode_combo)

        fluid_bar = FluidColumnBar()
        fluid_bar.setMinimumWidth(120)
        row_layout.addWidget(fluid_bar, 1)

        ink_label = QLabel("Empty")
        ink_label.setStyleSheet(f"color: {COLORS['subtext0']}; font-size: 10px;")
        ink_label.setFixedWidth(130)
        row_layout.addWidget(ink_label)

        return {
            "layout": row_layout,
            "syringe_combo": syringe_combo,
            "mode_combo": mode_combo,
            "fluid_bar": fluid_bar,
            "ink_label": ink_label,
        }

    def _on_syringe_changed(self, pump_id: str, _index: int = 0):
        pw = self._pump_widgets[pump_id]
        vol = pw["syringe_combo"].currentData()
        pump = self.workspace.pumps[pump_id]

        if vol is None:
            pump.syringe = None
        else:
            pump.syringe = SyringeSpec.from_dict(self._syringe_catalog[vol].to_dict())

        self._refresh_pump_display(pump_id)
        self._update_compatibility()
        self._emit_workspace()

    def _on_mode_changed(self, pump_id: str, _index: int = 0):
        pw = self._pump_widgets[pump_id]
        mode_val = pw["mode_combo"].currentData()
        self.workspace.pumps[pump_id].printing_mode = PrintingMode(mode_val)
        self._emit_workspace()

    def _on_buffer_changed(self):
        idx = self._buffer_combo.currentIndex()
        if idx <= 0:
            self.workspace.buffer_ink = None
        else:
            name = self._buffer_combo.currentText()
            self.workspace.buffer_ink = self.workspace.ink_library.get(name)
        self._emit_workspace()

    def _on_dead_volume_changed(self, val: float):
        for pump in self.workspace.pumps.values():
            pump.fluid_column.dead_volume_uL = val
        self._emit_workspace()

    def _refresh_pump_display(self, pump_id: str):
        pw = self._pump_widgets[pump_id]
        pump = self.workspace.pumps[pump_id]
        fc = pump.fluid_column
        ink = fc.ink_spec

        fracs = fc.volume_fractions(pump.syringe)
        color = ink.color if ink else "#a6e3a1"
        label = f"{fc.ink_volume_uL:.1f} µL" if ink else ""
        pw["fluid_bar"].set_data(fracs, color, label)

        if ink:
            pw["ink_label"].setText(f'{fc.ink_volume_uL:.1f} µL "{ink.name}"')
            pw["ink_label"].setStyleSheet(f"color: {ink.color}; font-size: 10px;")
        elif pump.syringe:
            pw["ink_label"].setText("No ink loaded")
            pw["ink_label"].setStyleSheet(f"color: {COLORS['overlay0']}; font-size: 10px;")
        else:
            pw["ink_label"].setText("No syringe")
            pw["ink_label"].setStyleSheet(f"color: {COLORS['overlay0']}; font-size: 10px;")

    def refresh_all_pump_displays(self):
        """Public method to refresh all pump UI (called after print operations)."""
        for pid in ["P1", "P2", "P3"]:
            self._refresh_pump_display(pid)

    # ── 4. Ink Library ─────────────────────────────────────────────

    def _build_ink_library_section(self):
        _, layout = self._make_section("Ink Library")

        btn_row = QHBoxLayout()
        btn_add = QPushButton("+ Add")
        btn_add.clicked.connect(self._add_ink)
        btn_edit = QPushButton("Edit")
        btn_edit.clicked.connect(self._edit_ink)
        btn_del = QPushButton("Delete")
        btn_del.clicked.connect(self._delete_ink)
        for b in [btn_add, btn_edit, btn_del]:
            b.setFixedHeight(26)
            b.setMaximumWidth(80)
            btn_row.addWidget(b)
        btn_row.addStretch()
        layout.addLayout(btn_row)

        self._ink_table = QTableWidget(0, 5)
        self._ink_table.setHorizontalHeaderLabels(
            ["Name", "Type", "Viscosity", "Granule Ø", "Cell Ø"]
        )
        self._ink_table.horizontalHeader().setSectionResizeMode(0, QHeaderView.Stretch)
        for i in range(1, 5):
            self._ink_table.horizontalHeader().setSectionResizeMode(i, QHeaderView.ResizeToContents)
        self._ink_table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self._ink_table.setSelectionMode(QAbstractItemView.SingleSelection)
        self._ink_table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self._ink_table.setMaximumHeight(140)
        self._ink_table.verticalHeader().setDefaultSectionSize(22)
        self._ink_table.verticalHeader().setVisible(False)
        layout.addWidget(self._ink_table)

    def _refresh_ink_table(self):
        self._ink_table.setRowCount(0)
        for name, ink in self.workspace.ink_library.items():
            row = self._ink_table.rowCount()
            self._ink_table.insertRow(row)

            name_item = QTableWidgetItem(ink.name)
            name_item.setForeground(QColor(ink.color))
            self._ink_table.setItem(row, 0, name_item)
            self._ink_table.setItem(row, 1, QTableWidgetItem(ink.ink_type))
            self._ink_table.setItem(row, 2, QTableWidgetItem(f"{ink.viscosity_cP:.1f} cP"))
            self._ink_table.setItem(row, 3, QTableWidgetItem(
                f"{ink.granule_diameter_um:.0f} µm" if ink.granule_diameter_um > 0 else "—"))
            self._ink_table.setItem(row, 4, QTableWidgetItem(
                f"{ink.cell_diameter_um:.0f} µm" if ink.cell_diameter_um > 0 else "—"))

        # Refresh buffer combo
        current_buf = self._buffer_combo.currentText()
        self._buffer_combo.blockSignals(True)
        self._buffer_combo.clear()
        self._buffer_combo.addItem("(none)")
        for name in self.workspace.ink_library:
            self._buffer_combo.addItem(name)
        idx = self._buffer_combo.findText(current_buf)
        if idx >= 0:
            self._buffer_combo.setCurrentIndex(idx)
        self._buffer_combo.blockSignals(False)

    def _add_ink(self):
        dlg = InkEditorDialog(parent=self)
        if dlg.exec() == QDialog.Accepted:
            ink = dlg.get_ink()
            self.workspace.ink_library[ink.name] = ink
            self._refresh_ink_table()
            self._update_compatibility()
            self._persist_workspace_to_settings()
            self._emit_workspace()

    def _edit_ink(self):
        row = self._ink_table.currentRow()
        if row < 0:
            return
        old_name = self._ink_table.item(row, 0).text()
        ink = self.workspace.ink_library.get(old_name)
        if ink is None:
            return
        dlg = InkEditorDialog(ink=ink, parent=self)
        if dlg.exec() == QDialog.Accepted:
            new_ink = dlg.get_ink()
            if new_ink.name != old_name:
                del self.workspace.ink_library[old_name]
            self.workspace.ink_library[new_ink.name] = new_ink
            self._refresh_ink_table()
            self._update_compatibility()
            self._persist_workspace_to_settings()
            self._emit_workspace()

    def _delete_ink(self):
        row = self._ink_table.currentRow()
        if row < 0:
            return
        name = self._ink_table.item(row, 0).text()
        self.workspace.ink_library.pop(name, None)
        self._refresh_ink_table()
        self._update_compatibility()
        self._persist_workspace_to_settings()
        self._emit_workspace()

    # ── 5. Rosette Library ─────────────────────────────────────────

    def _build_rosette_library_section(self):
        _, layout = self._make_section("Rosette Library")

        btn_row = QHBoxLayout()
        btn_add = QPushButton("+ New")
        btn_add.clicked.connect(self._add_rosette)
        btn_edit = QPushButton("Edit")
        btn_edit.clicked.connect(self._edit_rosette)
        btn_del = QPushButton("Delete")
        btn_del.clicked.connect(self._delete_rosette)
        for b in [btn_add, btn_edit, btn_del]:
            b.setFixedHeight(26)
            b.setMaximumWidth(80)
            btn_row.addWidget(b)
        btn_row.addStretch()
        layout.addLayout(btn_row)

        self._rosette_table = QTableWidget(0, 4)
        self._rosette_table.setHorizontalHeaderLabels(
            ["Name", "Sub-wells", "Fits", "Depth"]
        )
        self._rosette_table.horizontalHeader().setSectionResizeMode(0, QHeaderView.Stretch)
        for i in range(1, 4):
            self._rosette_table.horizontalHeader().setSectionResizeMode(
                i, QHeaderView.ResizeToContents)
        self._rosette_table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self._rosette_table.setSelectionMode(QAbstractItemView.SingleSelection)
        self._rosette_table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self._rosette_table.setMaximumHeight(110)
        self._rosette_table.verticalHeader().setDefaultSectionSize(22)
        self._rosette_table.verticalHeader().setVisible(False)
        layout.addWidget(self._rosette_table)

    def _refresh_rosette_table(self):
        self._rosette_table.setRowCount(0)
        for name, ros in self.workspace.rosette_library.items():
            row = self._rosette_table.rowCount()
            self._rosette_table.insertRow(row)
            center_str = "+center" if ros.has_center_well else ""
            ring_n = ros.num_subwells - (1 if ros.has_center_well else 0)
            self._rosette_table.setItem(row, 0, QTableWidgetItem(ros.name))
            self._rosette_table.setItem(row, 1, QTableWidgetItem(f"{ring_n}{center_str}"))
            self._rosette_table.setItem(row, 2, QTableWidgetItem(f"{ros.well_format}w"))
            self._rosette_table.setItem(row, 3, QTableWidgetItem(
                f"{ros.subwell_depth_mm:.1f} mm"))

    def _add_rosette(self):
        dlg = RosetteEditorDialog(parent=self)
        if dlg.exec() == QDialog.Accepted:
            ros = dlg.get_rosette()
            self.workspace.rosette_library[ros.name] = ros
            self._refresh_rosette_table()
            self._persist_workspace_to_settings()
            self._emit_workspace()

    def _edit_rosette(self):
        row = self._rosette_table.currentRow()
        if row < 0:
            return
        old_name = self._rosette_table.item(row, 0).text()
        ros = self.workspace.rosette_library.get(old_name)
        if ros is None:
            return
        dlg = RosetteEditorDialog(rosette=ros, parent=self)
        if dlg.exec() == QDialog.Accepted:
            new_ros = dlg.get_rosette()
            if new_ros.name != old_name:
                del self.workspace.rosette_library[old_name]
            self.workspace.rosette_library[new_ros.name] = new_ros
            self._refresh_rosette_table()
            self._persist_workspace_to_settings()
            self._emit_workspace()

    def _delete_rosette(self):
        row = self._rosette_table.currentRow()
        if row < 0:
            return
        name = self._rosette_table.item(row, 0).text()
        self.workspace.rosette_library.pop(name, None)
        self._refresh_rosette_table()
        self._persist_workspace_to_settings()
        self._emit_workspace()

    # ── 6. Print Settings ──────────────────────────────────────────

    def _build_print_settings_section(self):
        _, layout = self._make_section("Print Settings (Defaults)")

        grid = QGridLayout()
        grid.setSpacing(4)

        settings_def = [
            ("Travel Z:", "travel_z_mm", "mm", 0.1, 50.0, 1, 5.0),
            ("Layer height:", "layer_height_mm", "mm", 0.01, 5.0, 2, 0.2),
            ("Print speed:", "print_speed_mm_s", "mm/s", 0.1, 50.0, 1, 5.0),
            ("Z feed rate:", "z_feed_rate_mm_s", "mm/s", 0.1, 20.0, 1, 2.0),
            ("Pump feed:", "pump_feed_rate_uL_s", "µL/s", 0.01, 50.0, 2, 1.0),
            ("Retract:", "retract_distance_uL", "µL", 0.0, 10.0, 2, 0.5),
            ("Prime:", "prime_distance_uL", "µL", 0.0, 10.0, 2, 0.5),
            ("Retract speed:", "retract_speed_uL_s", "µL/s", 0.1, 20.0, 1, 2.0),
            ("Prime speed:", "prime_speed_uL_s", "µL/s", 0.1, 20.0, 1, 1.0),
        ]

        self._settings_spins: dict[str, QDoubleSpinBox] = {}
        for i, (label, key, suffix, mn, mx, dec, default) in enumerate(settings_def):
            r, c = divmod(i, 3)
            grid.addWidget(QLabel(label), r, c * 2)
            spin = QDoubleSpinBox()
            spin.setRange(mn, mx)
            spin.setDecimals(dec)
            spin.setSuffix(f" {suffix}")
            spin.setValue(self.workspace.print_settings.get(key, default))
            spin.valueChanged.connect(partial(self._on_setting_changed, key))
            spin.setFixedWidth(110)
            grid.addWidget(spin, r, c * 2 + 1)
            self._settings_spins[key] = spin

        layout.addLayout(grid)

    def _on_setting_changed(self, key: str, value: float):
        self.workspace.print_settings[key] = value
        self._emit_workspace()

    # ── 7. Compatibility Report ────────────────────────────────────

    def _build_compatibility_section(self):
        _, layout = self._make_section("Compatibility Report")
        self._compat_label = QLabel("Configure needle and pump loadout to see report.")
        self._compat_label.setWordWrap(True)
        self._compat_label.setStyleSheet(f"color: {COLORS['subtext0']}; font-size: 11px;")
        layout.addWidget(self._compat_label)

    def _update_compatibility(self):
        needle = self.workspace.needle
        if needle is None:
            self._compat_label.setText("Select a needle to see compatibility report.")
            return

        reports = generate_compatibility_report(
            needle, self.workspace.pumps,
            default_flow_rate_uL_s=self.workspace.print_settings.get(
                "pump_feed_rate_uL_s", 1.0),
        )

        lines = []
        for rep in reports:
            pid = rep["pump_id"]
            if rep["status"] == "no_config":
                lines.append(f"  {pid}: (no syringe or ink)")
                continue

            icon = {"ok": "✅", "warning": "⚠️", "info": "ℹ️"}.get(rep["status"], "•")
            msg = rep["message"]
            rate = rep.get("max_rate_uL_s", 0)
            lines.append(f"  {icon} {pid}: {msg}")
            if rate > 0:
                lines.append(f"       Max safe rate: {rate:.2f} µL/s")

        self._compat_label.setText("\n".join(lines) if lines else "No pump data to report.")

    # ────────────────────────────────────────────────────────────────
    #  Save / Load / Persistence
    # ────────────────────────────────────────────────────────────────

    def _persist_workspace_to_settings(self):
        """Save workspace state into the Settings object for persistence."""
        if self.settings is None:
            return
        try:
            self.settings.set("workspace", self.workspace.to_dict())
            self.settings.save()
        except Exception as e:
            logger.warning(f"Failed to persist workspace to settings: {e}")

    def _load_workspace_from_settings(self):
        """Restore workspace state from Settings."""
        if self.settings is None:
            return False
        try:
            data = self.settings.get("workspace")
            if data and isinstance(data, dict):
                self.workspace = WorkspaceConfig.from_dict(data)
                return True
        except Exception as e:
            logger.warning(f"Failed to load workspace from settings: {e}")
        return False

    def save_workspace_file(self):
        """Save workspace config to a user-chosen JSON file."""
        path, _ = QFileDialog.getSaveFileName(
            self, "Save Workspace", "", "JSON Files (*.json)")
        if path:
            self.workspace.save_json(path)

    def load_workspace_file(self):
        """Load workspace config from a user-chosen JSON file."""
        path, _ = QFileDialog.getOpenFileName(
            self, "Load Workspace", "", "JSON Files (*.json)")
        if path:
            try:
                self.workspace = WorkspaceConfig.load_json(path)
                self._apply_workspace_to_ui()
                self._emit_workspace()
            except Exception as e:
                QMessageBox.warning(self, "Load Error", f"Failed to load workspace:\n{e}")

    def _apply_workspace_to_ui(self):
        """Push current workspace config into all UI widgets."""
        ws = self.workspace

        # Plate format
        idx = self._plate_combo.findData(ws.plate_format)
        if idx >= 0:
            self._plate_combo.blockSignals(True)
            self._plate_combo.setCurrentIndex(idx)
            self._plate_combo.blockSignals(False)

        # Needle
        if ws.needle:
            idx = self._gauge_combo.findData(ws.needle.gauge)
            if idx >= 0:
                self._gauge_combo.blockSignals(True)
                self._gauge_combo.setCurrentIndex(idx)
                self._gauge_combo.blockSignals(False)
            idx = self._length_combo.findData(ws.needle.length_inches)
            if idx >= 0:
                self._length_combo.blockSignals(True)
                self._length_combo.setCurrentIndex(idx)
                self._length_combo.blockSignals(False)
            self._channels_spin.blockSignals(True)
            self._channels_spin.setValue(ws.needle.num_channels)
            self._channels_spin.blockSignals(False)
            self._on_needle_changed()

        # Pump loadout
        for pid in ["P1", "P2", "P3"]:
            pump = ws.pumps.get(pid)
            pw = self._pump_widgets[pid]
            if pump and pump.syringe:
                idx = pw["syringe_combo"].findData(pump.syringe.volume_uL)
                if idx >= 0:
                    pw["syringe_combo"].blockSignals(True)
                    pw["syringe_combo"].setCurrentIndex(idx)
                    pw["syringe_combo"].blockSignals(False)
            mode_idx = pw["mode_combo"].findData(pump.printing_mode.value if pump else "incremental")
            if mode_idx >= 0:
                pw["mode_combo"].blockSignals(True)
                pw["mode_combo"].setCurrentIndex(mode_idx)
                pw["mode_combo"].blockSignals(False)
            self._refresh_pump_display(pid)

        # Ink library
        self._refresh_ink_table()

        # Rosette library
        self._refresh_rosette_table()

        # Print settings
        for key, spin in self._settings_spins.items():
            val = ws.print_settings.get(key)
            if val is not None:
                spin.blockSignals(True)
                spin.setValue(val)
                spin.blockSignals(False)

        # Dead volume
        if ws.pumps.get("P1"):
            self._dead_vol_spin.blockSignals(True)
            self._dead_vol_spin.setValue(ws.pumps["P1"].fluid_column.dead_volume_uL)
            self._dead_vol_spin.blockSignals(False)

        self._update_compatibility()

    def _apply_defaults(self):
        """Apply initial defaults or restore from settings."""
        if not self._load_workspace_from_settings():
            # Set default needle
            self._on_needle_changed()
        else:
            self._apply_workspace_to_ui()

    # ────────────────────────────────────────────────────────────────
    #  Signal Emission
    # ────────────────────────────────────────────────────────────────

    def _emit_workspace(self):
        """Emit the workspace_changed signal with current config."""
        self.workspace_changed.emit(self.workspace)

    # ────────────────────────────────────────────────────────────────
    #  Context Panel (for app.py right-side panel)
    # ────────────────────────────────────────────────────────────────

    def get_context_widget(self) -> QWidget:
        """Build workspace context panel with save/load buttons."""
        ctx = QWidget()
        layout = QVBoxLayout(ctx)
        layout.setContentsMargins(12, 8, 12, 8)
        layout.setSpacing(6)

        title = QLabel("Workspace")
        title.setObjectName("contextSectionLabel")
        title.setStyleSheet(f"font-weight: bold; color: {COLORS['text']}; font-size: 13px;")
        layout.addWidget(title)

        btn_save = QPushButton("💾 Save Workspace")
        btn_save.clicked.connect(self.save_workspace_file)
        layout.addWidget(btn_save)

        btn_load = QPushButton("📂 Load Workspace")
        btn_load.clicked.connect(self.load_workspace_file)
        layout.addWidget(btn_load)

        # Syringe quick reference
        layout.addSpacing(10)
        ref_lbl = QLabel("Hamilton 1700 Series")
        ref_lbl.setStyleSheet(f"font-weight: bold; color: {COLORS['subtext0']}; font-size: 11px;")
        layout.addWidget(ref_lbl)

        for vol, spec in sorted(self._syringe_catalog.items()):
            txt = f"  {vol} µL — ID {spec.barrel_id_mm:.3f} mm — {spec.uL_per_mm:.2f} µL/mm"
            lbl = QLabel(txt)
            lbl.setStyleSheet(f"color: {COLORS['overlay0']}; font-size: 10px;")
            layout.addWidget(lbl)

        layout.addStretch()
        return ctx

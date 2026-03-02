"""
print_workspace.py — Tab 1: Hardware Summary (Read-Only) for MEBP v7.2.3.

v7.2.3 REWRITE: This tab no longer has editable hardware controls.
Instead it displays a read-only summary of the HardwareConfig received
from Page 0 (Hardware Setup). All editing is done on Page 0.

Key responsibilities:
- Display current hardware config as read-only labels and tables
- Build a WorkspaceConfig FROM HardwareConfig (bridge method)
- Emit workspace_changed signal so Tab 2 and Tab 3 still work
- Provide "Edit Hardware Setup" button → navigates to Page 0

Signal flow:
    HardwareConfig (from app.py via set_hardware_config)
        → _hardware_config_to_workspace() bridge
        → WorkspaceConfig
        → workspace_changed signal
        → Tab 2 (Print Objects), Tab 3 (Well Setup)

v7.2.3 Changes:
    - Removed all editable hardware controls (needle dropdowns, syringe combos,
      ink library editor, rosette editor, pump config, print settings)
    - Replaced with read-only summary sections
    - Added _hardware_config_to_workspace() bridge method
    - Added "Edit Hardware Setup ▶" navigation button
    - Kept workspace_changed signal for backward compatibility
    - Kept save/load workspace context panel for print settings export
"""

from __future__ import annotations

import logging

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox,
    QPushButton, QLabel, QFrame, QScrollArea,
    QTableWidget, QTableWidgetItem, QHeaderView,
    QAbstractItemView, QFileDialog, QMessageBox,
)
from PySide6.QtCore import Qt, Signal

from SupportClasses.PhysicalModels import (
    WorkspaceConfig, PumpLoadout, NeedleSpec, InkSpec,
    RosetteInsert, PrintingMode,
    load_needle_catalog, load_syringe_catalog,
)
from SupportClasses.WellPlate import PLATE_DEFINITIONS
from gui.styles import COLORS

try:
    from SupportClasses.HardwareConfig import HardwareConfig, PumpChannelConfig
    HAS_HW_CONFIG = True
except ImportError:
    HAS_HW_CONFIG = False

logger = logging.getLogger(__name__)


# ═══════════════════════════════════════════════════════════════════
# Hardware Summary Widget (Read-Only)
# ═══════════════════════════════════════════════════════════════════

class HardwareSummaryWidget(QWidget):
    """
    Read-only display of the current HardwareConfig.

    Shows needle, plate, pump channels, ink library, and rosette library
    as non-editable labels and tables. The only action is "Edit Hardware
    Setup" which emits a navigation signal.
    """

    edit_requested = Signal()  # Emitted when user clicks "Edit Hardware Setup"

    def __init__(self, parent=None):
        super().__init__(parent)
        self._hw_config = None
        self._build_ui()

    def _build_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(8)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setFrameShape(QFrame.NoFrame)

        scroll_content = QWidget()
        self._content = QVBoxLayout(scroll_content)
        self._content.setContentsMargins(4, 4, 4, 4)
        self._content.setSpacing(8)

        # ── Header with Edit button ──────────────────────────────
        header = QHBoxLayout()
        title = QLabel("Hardware Configuration Summary")
        title.setStyleSheet(
            f"font-weight: bold; color: {COLORS['text']}; font-size: 14px;")
        header.addWidget(title)
        header.addStretch()

        self.edit_btn = QPushButton("✏ Edit Hardware Setup")
        self.edit_btn.setStyleSheet(f"""
            QPushButton {{
                background: {COLORS.get('blue', '#89b4fa')};
                color: {COLORS.get('base', '#1e1e2e')};
                font-weight: bold; padding: 6px 14px;
                border-radius: 4px;
            }}
            QPushButton:hover {{
                background: {COLORS.get('sapphire', '#74c7ec')};
            }}
        """)
        self.edit_btn.clicked.connect(self.edit_requested.emit)
        header.addWidget(self.edit_btn)
        self._content.addLayout(header)

        # ── Needle Section ────────────────────────────────────────
        self._needle_group, needle_lay = self._make_section("Needle")
        self._needle_label = QLabel("Not configured")
        self._needle_label.setStyleSheet(self._value_style())
        self._needle_label.setWordWrap(True)
        needle_lay.addWidget(self._needle_label)

        # ── Plate Section ─────────────────────────────────────────
        self._plate_group, plate_lay = self._make_section("Well Plate")
        self._plate_label = QLabel("Not configured")
        self._plate_label.setStyleSheet(self._value_style())
        plate_lay.addWidget(self._plate_label)

        # ── Pump Channels Section ─────────────────────────────────
        self._pump_group, pump_lay = self._make_section("Pump Channels")
        self._pump_labels: dict[str, QLabel] = {}
        for pid in ["P1", "P2", "P3"]:
            row = QHBoxLayout()
            pid_lbl = QLabel(f"{pid}:")
            pid_lbl.setFixedWidth(30)
            pid_lbl.setStyleSheet(
                f"font-weight: bold; color: {COLORS.get('blue', '#89b4fa')};")
            row.addWidget(pid_lbl)
            val_lbl = QLabel("Disabled")
            val_lbl.setStyleSheet(self._value_style())
            val_lbl.setWordWrap(True)
            row.addWidget(val_lbl, 1)
            pump_lay.addLayout(row)
            self._pump_labels[pid] = val_lbl

        # ── Ink Library Section ───────────────────────────────────
        self._ink_group, ink_lay = self._make_section("Ink Library")
        self._ink_table = QTableWidget()
        self._ink_table.setColumnCount(4)
        self._ink_table.setHorizontalHeaderLabels(
            ["Name", "Type", "Viscosity", "Granule/Cell Ø"])
        self._ink_table.horizontalHeader().setSectionResizeMode(
            QHeaderView.ResizeMode.Stretch)
        self._ink_table.setEditTriggers(
            QAbstractItemView.EditTrigger.NoEditTriggers)
        self._ink_table.setSelectionMode(
            QAbstractItemView.SelectionMode.NoSelection)
        self._ink_table.setMaximumHeight(100)
        self._ink_table.verticalHeader().setDefaultSectionSize(20)
        self._ink_table.verticalHeader().setVisible(False)
        ink_lay.addWidget(self._ink_table)
        self._ink_empty_label = QLabel("No inks defined")
        self._ink_empty_label.setStyleSheet(
            f"color: {COLORS.get('overlay0', '#6c7086')}; font-style: italic;")
        ink_lay.addWidget(self._ink_empty_label)

        # ── Rosette Library Section ───────────────────────────────
        self._ros_group, ros_lay = self._make_section("Rosette Library")
        self._ros_table = QTableWidget()
        self._ros_table.setColumnCount(4)
        self._ros_table.setHorizontalHeaderLabels(
            ["Name", "Sub-wells", "Fits", "Depth"])
        self._ros_table.horizontalHeader().setSectionResizeMode(
            QHeaderView.ResizeMode.Stretch)
        self._ros_table.setEditTriggers(
            QAbstractItemView.EditTrigger.NoEditTriggers)
        self._ros_table.setSelectionMode(
            QAbstractItemView.SelectionMode.NoSelection)
        self._ros_table.setMaximumHeight(90)
        self._ros_table.verticalHeader().setDefaultSectionSize(20)
        self._ros_table.verticalHeader().setVisible(False)
        ros_lay.addWidget(self._ros_table)
        self._ros_empty_label = QLabel("No rosettes defined")
        self._ros_empty_label.setStyleSheet(
            f"color: {COLORS.get('overlay0', '#6c7086')}; font-style: italic;")
        ros_lay.addWidget(self._ros_empty_label)

        # ── Status ────────────────────────────────────────────────
        self._status_label = QLabel("⚠ No hardware configuration loaded")
        self._status_label.setStyleSheet(
            f"color: {COLORS.get('yellow', '#f9e2af')}; font-weight: bold;")
        self._content.addWidget(self._status_label)

        self._content.addStretch()
        scroll.setWidget(scroll_content)
        layout.addWidget(scroll)

    def _make_section(self, title: str) -> tuple[QGroupBox, QVBoxLayout]:
        group = QGroupBox(title)
        group.setStyleSheet(f"""
            QGroupBox {{
                font-weight: bold; color: {COLORS['text']};
                border: 1px solid {COLORS.get('surface1', '#45475a')};
                border-radius: 6px; margin-top: 8px; padding-top: 14px;
            }}
            QGroupBox::title {{
                subcontrol-origin: margin; left: 10px; padding: 0 6px;
            }}
        """)
        layout = QVBoxLayout(group)
        layout.setContentsMargins(10, 6, 10, 8)
        layout.setSpacing(4)
        self._content.addWidget(group)
        return group, layout

    def _value_style(self) -> str:
        return f"color: {COLORS.get('subtext0', '#a6adc8')}; font-size: 11px;"

    # ── Update from HardwareConfig ────────────────────────────────

    def update_from_config(self, config) -> None:
        """Refresh all read-only displays from a HardwareConfig."""
        self._hw_config = config

        if config is None:
            self._needle_label.setText("Not configured")
            self._plate_label.setText("Not configured")
            for lbl in self._pump_labels.values():
                lbl.setText("Disabled")
            self._ink_table.setRowCount(0)
            self._ink_empty_label.setVisible(True)
            self._ros_table.setRowCount(0)
            self._ros_empty_label.setVisible(True)
            self._status_label.setText("⚠ No hardware configuration loaded")
            self._status_label.setStyleSheet(
                f"color: {COLORS.get('yellow', '#f9e2af')}; font-weight: bold;")
            return

        # Needle
        if config.needle:
            n = config.needle
            ch_str = f" | {n.num_channels} channel(s)" if hasattr(n, 'num_channels') and n.num_channels > 1 else ""
            len_str = f" | {n.length_inches}\"" if hasattr(n, 'length_inches') else ""
            self._needle_label.setText(
                f"{n.gauge}G — ID: {n.id_um} µm, OD: {n.od_um} µm, "
                f"Wall: {n.wall_um} µm{len_str}{ch_str}")
        else:
            self._needle_label.setText("Not selected")

        # Plate
        fmt = config.plate_format
        pdef = PLATE_DEFINITIONS.get(fmt)
        if pdef:
            self._plate_label.setText(
                f"{fmt}-well  ({pdef['rows']} × {pdef['cols']})  —  "
                f"Ø {pdef.get('well_diameter', '?')} mm, "
                f"Spacing: {pdef.get('well_spacing_x', '?')} mm")
        else:
            self._plate_label.setText(f"{fmt}-well")

        # Pumps
        for pid in ["P1", "P2", "P3"]:
            pcfg = config.pumps.get(pid)
            if pcfg and pcfg.enabled:
                parts = []
                if pcfg.syringe:
                    parts.append(f"{pcfg.syringe.volume_uL} µL syringe")
                if pcfg.ink:
                    parts.append(f"Ink: {pcfg.ink.name}")
                if pcfg.printing_mode:
                    parts.append(pcfg.printing_mode.value.capitalize())
                self._pump_labels[pid].setText(" | ".join(parts) if parts else "Enabled (no syringe)")
            else:
                self._pump_labels[pid].setText("Disabled")

        # Ink Library
        inks = config.ink_library
        self._ink_table.setRowCount(0)
        self._ink_empty_label.setVisible(len(inks) == 0)
        self._ink_table.setVisible(len(inks) > 0)
        for name, ink in inks.items():
            row = self._ink_table.rowCount()
            self._ink_table.insertRow(row)
            self._ink_table.setItem(row, 0, QTableWidgetItem(ink.name))
            self._ink_table.setItem(row, 1, QTableWidgetItem(ink.ink_type))
            self._ink_table.setItem(row, 2, QTableWidgetItem(
                f"{ink.viscosity_cP:.1f} cP"))
            extras = []
            if ink.granule_diameter_um > 0:
                extras.append(f"G: {ink.granule_diameter_um:.0f} µm")
            if ink.cell_diameter_um > 0:
                extras.append(f"C: {ink.cell_diameter_um:.0f} µm")
            self._ink_table.setItem(row, 3, QTableWidgetItem(
                " / ".join(extras) if extras else "—"))

        # Rosette Library
        rosettes = config.rosette_library
        self._ros_table.setRowCount(0)
        self._ros_empty_label.setVisible(len(rosettes) == 0)
        self._ros_table.setVisible(len(rosettes) > 0)
        for name, ros in rosettes.items():
            row = self._ros_table.rowCount()
            self._ros_table.insertRow(row)
            center_str = "+center" if ros.has_center_well else ""
            ring_n = ros.num_subwells - (1 if ros.has_center_well else 0)
            self._ros_table.setItem(row, 0, QTableWidgetItem(ros.name))
            self._ros_table.setItem(row, 1, QTableWidgetItem(
                f"{ring_n}{center_str}"))
            self._ros_table.setItem(row, 2, QTableWidgetItem(
                f"{ros.well_format}w"))
            self._ros_table.setItem(row, 3, QTableWidgetItem(
                f"{ros.subwell_depth_mm:.1f} mm"))

        # Status
        valid = config.is_valid if hasattr(config, 'is_valid') else False
        if valid:
            self._status_label.setText(
                f"✓ Hardware configured: {config.config_name}")
            self._status_label.setStyleSheet(
                f"color: {COLORS.get('green', '#a6e3a1')}; font-weight: bold;")
        else:
            issues = []
            if hasattr(config, 'validate'):
                _, issues = config.validate()
            msg = issues[0] if issues else "Setup incomplete"
            self._status_label.setText(f"⚠ {msg}")
            self._status_label.setStyleSheet(
                f"color: {COLORS.get('yellow', '#f9e2af')}; font-weight: bold;")


# ═══════════════════════════════════════════════════════════════════
# Main Workspace Tab (v7.2.3 — Read-Only)
# ═══════════════════════════════════════════════════════════════════

class WorkspaceTab(QWidget):
    """
    Tab 1: Hardware Summary — read-only view of HardwareConfig.

    v7.2.3: No longer has editable controls. All hardware editing
    is done on Page 0 (Hardware Setup). This tab:
    1. Displays the current hardware config as a read-only summary
    2. Bridges HardwareConfig → WorkspaceConfig for downstream tabs
    3. Emits workspace_changed for Tab 2 and Tab 3
    """

    # Emitted when workspace config changes — other tabs listen to this
    workspace_changed = Signal(object)  # WorkspaceConfig

    # v7.2.3: Request navigation to Hardware Setup page
    navigate_to_page = Signal(int)  # page index (0 = Hardware Setup)

    def __init__(self, controller=None, settings=None, parent=None):
        super().__init__(parent)
        self.controller = controller
        self.settings = settings

        # Load catalogs (still needed for bridge method)
        self._syringe_catalog = load_syringe_catalog()

        # Active workspace config (built from HardwareConfig)
        self.workspace = WorkspaceConfig()

        # Hardware config reference
        self._hw_config = None

        # Build UI
        self._setup_ui()

    # ────────────────────────────────────────────────────────────────
    #  UI Construction
    # ────────────────────────────────────────────────────────────────

    def _setup_ui(self):
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(8, 4, 8, 4)
        main_layout.setSpacing(6)

        # The entire tab is the read-only summary widget
        self._summary = HardwareSummaryWidget()
        self._summary.edit_requested.connect(self._on_edit_requested)
        main_layout.addWidget(self._summary)

    # ────────────────────────────────────────────────────────────────
    #  Public API
    # ────────────────────────────────────────────────────────────────

    def set_hardware_config(self, config) -> None:
        """
        v7.2.3: Receive hardware config and update summary + workspace.

        Called by PrintSetupPage.set_hardware_config() when the hardware
        config changes (from Page 0 or on app startup).
        """
        self._hw_config = config

        # Update read-only summary display
        self._summary.update_from_config(config)

        # Build WorkspaceConfig from HardwareConfig
        self.workspace = self._hardware_config_to_workspace(config)

        # Emit for downstream tabs
        self._emit_workspace()

        logger.info("Workspace updated from HardwareConfig")

    def get_workspace(self) -> WorkspaceConfig:
        """Return the current WorkspaceConfig."""
        return self.workspace

    # ────────────────────────────────────────────────────────────────
    #  HardwareConfig → WorkspaceConfig Bridge
    # ────────────────────────────────────────────────────────────────

    def _hardware_config_to_workspace(self, hw_config) -> WorkspaceConfig:
        """
        Build a WorkspaceConfig from a HardwareConfig.

        This is the bridge between the v7.2 HardwareConfig (used by Page 0)
        and the v7.1 WorkspaceConfig (used by Tab 2 and Tab 3).

        Maps:
            HardwareConfig.needle       → WorkspaceConfig.needle
            HardwareConfig.pumps        → WorkspaceConfig.pumps (as PumpLoadout)
            HardwareConfig.plate_format → WorkspaceConfig.plate_format
            HardwareConfig.ink_library  → WorkspaceConfig.ink_library
            HardwareConfig.rosette_library → WorkspaceConfig.rosette_library
            HardwareConfig.buffer_ink_name → WorkspaceConfig.buffer_ink (resolved)
        """
        ws = WorkspaceConfig()

        if hw_config is None:
            return ws

        # Needle (direct copy)
        ws.needle = hw_config.needle

        # Plate format
        ws.plate_format = hw_config.plate_format

        # Ink library (direct copy)
        ws.ink_library = dict(hw_config.ink_library)

        # Rosette library (direct copy)
        ws.rosette_library = dict(hw_config.rosette_library)

        # Buffer ink (resolve from name)
        if hasattr(hw_config, 'buffer_ink_name') and hw_config.buffer_ink_name:
            ws.buffer_ink = hw_config.ink_library.get(hw_config.buffer_ink_name)

        # Pumps: Convert PumpChannelConfig → PumpLoadout
        for pid in ["P1", "P2", "P3"]:
            pcfg = hw_config.pumps.get(pid)
            if pcfg is None:
                continue

            pump = ws.pumps[pid]

            if pcfg.enabled and pcfg.syringe:
                pump.syringe = pcfg.syringe
                pump.printing_mode = pcfg.printing_mode

                # Copy ink reference
                if pcfg.ink:
                    # Resolve full InkSpec from library if it's a placeholder
                    resolved = hw_config.ink_library.get(pcfg.ink.name, pcfg.ink)
                    pump.fluid_column.ink_spec = resolved

                # Copy fluid column state if available
                if hasattr(pcfg, 'fluid_column') and pcfg.fluid_column:
                    fc = pcfg.fluid_column
                    pump.fluid_column.oil_volume_uL = fc.oil_volume_uL
                    pump.fluid_column.buffer_volume_uL = fc.buffer_volume_uL
                    pump.fluid_column.ink_volume_uL = fc.ink_volume_uL
                    pump.fluid_column.dead_volume_uL = fc.dead_volume_uL
            else:
                pump.syringe = None

        # Preserve print settings from existing workspace if they exist
        if self.workspace and self.workspace.print_settings:
            ws.print_settings = dict(self.workspace.print_settings)

        return ws

    # ────────────────────────────────────────────────────────────────
    #  Signal Emission
    # ────────────────────────────────────────────────────────────────

    def _emit_workspace(self):
        """Emit the workspace_changed signal with current config."""
        self.workspace_changed.emit(self.workspace)

    # ────────────────────────────────────────────────────────────────
    #  Navigation
    # ────────────────────────────────────────────────────────────────

    def _on_edit_requested(self):
        """User clicked 'Edit Hardware Setup' → navigate to Page 0."""
        self.navigate_to_page.emit(0)
        logger.info("User requested navigation to Hardware Setup page")

    # ────────────────────────────────────────────────────────────────
    #  Context Panel
    # ────────────────────────────────────────────────────────────────

    def get_context_widget(self) -> QWidget:
        """Build workspace context panel with save/load buttons."""
        ctx = QWidget()
        layout = QVBoxLayout(ctx)
        layout.setContentsMargins(12, 8, 12, 8)
        layout.setSpacing(6)

        title = QLabel("Workspace")
        title.setObjectName("contextSectionLabel")
        title.setStyleSheet(
            f"font-weight: bold; color: {COLORS['text']}; font-size: 13px;")
        layout.addWidget(title)

        btn_save = QPushButton("💾 Save Workspace")
        btn_save.clicked.connect(self._save_workspace_file)
        layout.addWidget(btn_save)

        btn_load = QPushButton("📂 Load Workspace")
        btn_load.clicked.connect(self._load_workspace_file)
        layout.addWidget(btn_load)

        # Quick reference
        layout.addSpacing(10)
        ref_lbl = QLabel("Hamilton 1700 Series")
        ref_lbl.setStyleSheet(
            f"font-weight: bold; color: {COLORS.get('subtext0', '#a6adc8')}; font-size: 11px;")
        layout.addWidget(ref_lbl)

        for vol, spec in sorted(self._syringe_catalog.items()):
            txt = f"  {vol} µL — ID {spec.barrel_id_mm:.3f} mm — {spec.uL_per_mm:.2f} µL/mm"
            lbl = QLabel(txt)
            lbl.setStyleSheet(
                f"color: {COLORS.get('overlay0', '#6c7086')}; font-size: 10px;")
            layout.addWidget(lbl)

        layout.addStretch()
        return ctx

    def _save_workspace_file(self):
        path, _ = QFileDialog.getSaveFileName(
            self, "Save Workspace", "", "JSON Files (*.json)")
        if path:
            try:
                self.workspace.save_json(path)
            except Exception as e:
                QMessageBox.warning(self, "Save Error", f"Failed to save:\n{e}")

    def _load_workspace_file(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "Load Workspace", "", "JSON Files (*.json)")
        if path:
            try:
                self.workspace = WorkspaceConfig.load_json(path)
                self._emit_workspace()
            except Exception as e:
                QMessageBox.warning(self, "Load Error", f"Failed to load:\n{e}")

    # ────────────────────────────────────────────────────────────────
    #  Compatibility API (v7.1 interface preserved)
    # ────────────────────────────────────────────────────────────────

    def on_status_update(self):
        """Called by parent tab timer. No periodic work needed."""
        pass

    def save_workspace_file(self):
        """Public API for save (called from context panel)."""
        self._save_workspace_file()

    def load_workspace_file(self):
        """Public API for load (called from context panel)."""
        self._load_workspace_file()

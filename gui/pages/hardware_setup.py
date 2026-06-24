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
    QSlider,
)
from PySide6.QtCore import Qt, Signal, QTimer
from PySide6.QtGui import QFont, QColor, QStandardItem

from SupportClasses.HardwareConfig import (
    HardwareConfig, PumpChannelConfig, CameraConfig, CameraRole,
)
from SupportClasses.PhysicalModels import (
    NeedleSpec, SyringeSpec, InkSpec, PrintingMode, RosetteInsert, CameraSpec,
    load_needle_catalog, load_syringe_catalog, load_camera_catalog,
    WellRole, ROLE_COLORS, well_role_for_ink_type, ink_type_border_color,
    WELL_TYPES, INK_SUBTYPES, is_service_reagent,
)
from SupportClasses.WellPlate import PLATE_DEFINITIONS, WellPlate
from gui.styles import COLORS, SECTION_TITLE_STYLE
from gui.scaling import s, sf, sp, scaled_font_size
from gui.pages.mode_page import ModePage  # v7.4.0-b
from gui.widgets.icons import icon, icon_button
from gui.widgets.components import StatusBadge  # v7.4.x rev3 polish

logger = logging.getLogger(__name__)

# v7.2.4: Default directory for hardware config files
CONFIG_HARDWARE_DIR = Path(__file__).resolve().parent.parent.parent / "config" / "hardware"

# v7.3.0: Nikon Ti2-U objective magnifications available on the microscope
NIKON_TI2U_OBJECTIVES = [1.0, 2.0, 4.0, 10.0, 20.0]



# ═══════════════════════════════════════════════════════════════════
# Ink Editor Dialog
# ═══════════════════════════════════════════════════════════════════

class InkEditorDialog(QDialog):
    """Dialog for adding/editing an ink in the library."""

    # v7.5.x: ``ink_type`` is the WELL TYPE (drives behavior). Only the printable
    # ``ink`` type carries an informational SUBTYPE (INK_SUBTYPES); service types
    # (wash/buffer/waste/oil) have no subtype.
    INK_TYPES = list(WELL_TYPES)            # ["ink", "wash", "buffer", "waste", "oil"]
    INK_SUBTYPE_PRESETS = list(INK_SUBTYPES)

    def __init__(self, ink: InkSpec | None = None, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Edit Ink" if ink else "Add Ink")
        self.setMinimumWidth(s(400))
        self._build_ui(ink)

    def _build_ui(self, ink: InkSpec | None):
        layout = QFormLayout(self)

        self.name_edit = QLineEdit(ink.name if ink else "")
        self.name_edit.setPlaceholderText("e.g. Alginate 2%")
        layout.addRow("Name:", self.name_edit)

        self.type_combo = QComboBox()
        self.type_combo.addItems(self.INK_TYPES)
        if ink:
            idx = self.type_combo.findText((ink.ink_type or "").strip().lower())
            if idx >= 0:
                self.type_combo.setCurrentIndex(idx)
        self.type_combo.currentTextChanged.connect(self._on_type_changed)
        layout.addRow("Well type:", self.type_combo)

        # v7.5.x: informational ink subtype — editable, pre-filled with the
        # standard presets, only meaningful (and enabled) when type == "ink".
        self.subtype_combo = QComboBox()
        self.subtype_combo.setEditable(True)
        self.subtype_combo.addItem("")          # "(none)"
        self.subtype_combo.addItems(self.INK_SUBTYPE_PRESETS)
        if ink and (ink.ink_subtype or "").strip():
            self.subtype_combo.setCurrentText(ink.ink_subtype.strip())
        else:
            self.subtype_combo.setCurrentText("")
        layout.addRow("Ink subtype:", self.subtype_combo)
        self._on_type_changed(self.type_combo.currentText())

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

        color_row = QHBoxLayout()
        self._ink_color = ink.color if ink else "#a6e3a1"
        self.color_btn = QPushButton()
        self.color_btn.setFixedSize(s(28), s(28))
        self.color_btn.setStyleSheet(
            f"background: {self._ink_color}; border: 1px solid #585b70; border-radius: {sp(4)};")
        self.color_btn.clicked.connect(self._pick_ink_color)
        color_row.addWidget(self.color_btn)
        self.color_label = QLabel(self._ink_color)
        self.color_label.setStyleSheet("color: #cdd6f4;")
        color_row.addWidget(self.color_label)
        color_row.addStretch()
        layout.addRow("Color:", color_row)

        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addRow(buttons)

    def _on_type_changed(self, text: str):
        """Subtype only applies to the printable ``ink`` well type."""
        is_ink = (text or "").strip().lower() == "ink"
        self.subtype_combo.setEnabled(is_ink)
        if not is_ink:
            self.subtype_combo.setCurrentText("")

    def _pick_ink_color(self):
        from PySide6.QtWidgets import QColorDialog
        color = QColorDialog.getColor(QColor(self._ink_color), self, "Ink Color")
        if color.isValid():
            self._ink_color = color.name()
            self.color_btn.setStyleSheet(
                f"background: {self._ink_color}; border: 1px solid #585b70; border-radius: {sp(4)};")
            self.color_label.setText(self._ink_color)

    def get_ink(self) -> InkSpec | None:
        name = self.name_edit.text().strip()
        if not name:
            return None
        well_type = self.type_combo.currentText()
        subtype = (self.subtype_combo.currentText().strip()
                   if well_type == "ink" else "")
        return InkSpec(
            name=name,
            ink_type=well_type,
            ink_subtype=subtype,
            viscosity_cP=self.viscosity_spin.value(),
            granule_diameter_um=self.granule_spin.value(),
            cell_diameter_um=self.cell_spin.value(),
            density_g_mL=self.density_spin.value(),
            color=self._ink_color,
        )


# ═══════════════════════════════════════════════════════════════════
# Rosette Editor Dialog
# ═══════════════════════════════════════════════════════════════════

class RosetteEditorDialog(QDialog):
    """Dialog for adding/editing a rosette insert."""

    def __init__(self, rosette: RosetteInsert | None = None, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Edit Rosette" if rosette else "Add Rosette")
        self.setMinimumWidth(s(400))
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
        # v7.4.2 polish: inherit the central section title style so pump
        # channel sub-cards visually match the rest of the hardware page.
        # Aligned label/input grid — column 0 is the row label (fixed
        # width so all rows line up), column 1 is the input.
        self.setStyleSheet(SECTION_TITLE_STYLE)
        layout = QGridLayout(self)
        layout.setContentsMargins(s(10), s(10), s(10), s(10))
        layout.setHorizontalSpacing(s(12))
        layout.setVerticalSpacing(s(10))
        layout.setColumnStretch(0, 0)
        layout.setColumnStretch(1, 1)

        def _row_label(text: str) -> QLabel:
            lbl = QLabel(text)
            lbl.setStyleSheet(
                f"color: {COLORS['text']}; font-weight: 500;")
            lbl.setMinimumWidth(s(80))
            return lbl

        # Row 0: Enable toggle (spans both columns, sits flush left)
        self.enable_check = QCheckBox("Enable this pump")
        self.enable_check.setChecked(False)
        self.enable_check.setStyleSheet(
            f"color: {COLORS['text']}; font-weight: 500;")
        self.enable_check.toggled.connect(self._on_enable_changed)
        layout.addWidget(self.enable_check, 0, 0, 1, 2)

        # Row 1: Syringe
        layout.addWidget(_row_label("Syringe"), 1, 0)
        self.syringe_combo = QComboBox()
        self.syringe_combo.addItem("— None —", None)
        for vol in sorted(self.syringe_catalog.keys()):
            self.syringe_combo.addItem(f"{vol} µL", vol)
        self.syringe_combo.currentIndexChanged.connect(self._on_change)
        layout.addWidget(self.syringe_combo, 1, 1)

        # Row 2: Mode
        layout.addWidget(_row_label("Mode"), 2, 0)
        self.mode_combo = QComboBox()
        self.mode_combo.addItem("Incremental", "incremental")
        self.mode_combo.addItem("Continuous", "continuous")
        self.mode_combo.currentIndexChanged.connect(self._on_change)
        layout.addWidget(self.mode_combo, 2, 1)

        # Row 3: Inks — multi-select checklist (taller, so label aligns top)
        layout.addWidget(
            _row_label("Inks"), 3, 0, Qt.AlignmentFlag.AlignTop)
        self.ink_list = QListWidget()
        self.ink_list.setMaximumHeight(s(80))
        self.ink_list.setSelectionMode(QAbstractItemView.SelectionMode.NoSelection)
        self.ink_list.itemChanged.connect(self._on_change)
        layout.addWidget(self.ink_list, 3, 1)

        # Backward-compat shim
        self.ink_combo = None

        # Row 4: Info line (spans both columns)
        self.info_label = QLabel("")
        self.info_label.setStyleSheet(
            f"color: {COLORS.get('subtext0', '#a6adc8')}; ")
        self.info_label.setWordWrap(True)
        layout.addWidget(self.info_label, 4, 0, 1, 2)

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
        self.ink_list.setEnabled(enabled)
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
        Update ink checklist options. Preserves check state.

        v7.2.8: Multi-ink per pump — exclusion no longer enforced.
        """
        # Remember currently checked inks
        checked = set(self.get_selected_ink_names())
        self._ink_names = ink_names

        self.ink_list.blockSignals(True)
        self.ink_list.clear()

        for name in ink_names:
            item = QListWidgetItem(name)
            item.setFlags(item.flags() | Qt.ItemFlag.ItemIsUserCheckable)
            item.setCheckState(
                Qt.CheckState.Checked if name in checked
                else Qt.CheckState.Unchecked)
            self.ink_list.addItem(item)

        self.ink_list.blockSignals(False)

    def get_config(self) -> PumpChannelConfig:
        """Extract current config from the widget state."""
        config = PumpChannelConfig(pump_id=self.pump_id)
        config.enabled = self.enable_check.isChecked()

        vol = self.syringe_combo.currentData()
        if vol and vol in self.syringe_catalog:
            config.syringe = self.syringe_catalog[vol]

        # Multi-ink: collect all checked ink names as placeholder InkSpecs
        config.inks = [InkSpec(name=n) for n in self.get_selected_ink_names()]

        mode_val = self.mode_combo.currentData()
        config.printing_mode = PrintingMode(mode_val) if mode_val else PrintingMode.INCREMENTAL

        return config

    def set_config(self, config: PumpChannelConfig, ink_names: list[str] | None = None):
        """
        Apply a config to this widget.

        Populates the ink checklist before restoring selections.
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

        # Inks — check matching items
        assigned_names = set(config.ink_names)
        self.ink_list.blockSignals(True)
        for i in range(self.ink_list.count()):
            item = self.ink_list.item(i)
            item.setCheckState(
                Qt.CheckState.Checked if item.text() in assigned_names
                else Qt.CheckState.Unchecked)
        self.ink_list.blockSignals(False)

        # Mode
        mode_idx = self.mode_combo.findData(config.printing_mode.value)
        if mode_idx >= 0:
            self.mode_combo.setCurrentIndex(mode_idx)

        self._update_controls()
        self.blockSignals(False)

    def get_selected_ink_names(self) -> list[str]:
        """Get list of checked ink names."""
        names = []
        for i in range(self.ink_list.count()):
            item = self.ink_list.item(i)
            if item.checkState() == Qt.CheckState.Checked:
                names.append(item.text())
        return names

    def get_selected_ink_name(self) -> str | None:
        """Backward compat: return first checked ink name, or None."""
        names = self.get_selected_ink_names()
        return names[0] if names else None


# ═══════════════════════════════════════════════════════════════════
# Hardware Setup Page (v7.2.4)
# ═══════════════════════════════════════════════════════════════════

class HardwareSetupPage(ModePage):
    """
    Hardware Setup — the gating page that must be completed before
    any other page can operate.

    v7.4.0-b: Restructured as a ModePage with 7 sub-pages
        (Identity, Plate, Pumps & Inks, Needle, Rosette, Cameras, Stage)
        for navigability. Existing widgets/signals/handlers preserved;
        only the layout changes.
    v7.2.4: Reordered sections, added rosette library, fixed config restore.
    v7.2.4 Session 3: Pump-ink exclusivity, needle channel mapping.
    """

    _page_title_text = "Hardware Setup"

    # Signals
    config_changed = Signal(object)       # Emits HardwareConfig
    config_validated = Signal(bool)       # Emits validity state
    # v7.5.x: emitted when the Stage sub-page saves new XY safety limits to
    # the live controller, so the app can re-centre the default plate.
    safety_limits_changed = Signal()
    # v7.5.x: carries the background camera probe result (OpenCV indices, or
    # None on failure) from the worker thread back to the GUI thread so startup
    # camera detection + start runs without freezing the window.
    _cameras_probed = Signal(object)

    # v7.4.0-b: Settings reference used by the Stage sub-page (safety limits,
    # ZP feedrates, axis flips). Set via set_settings() from MainWindow.
    _settings = None
    _controller = None

    def __init__(self, parent=None):
        super().__init__(parent)
        self._context_widget = None

        # Load catalogs
        self._needle_catalog = load_needle_catalog()
        self._syringe_catalog = load_syringe_catalog()
        self._camera_catalog = load_camera_catalog()

        # Current config
        self._config = HardwareConfig()
        self._last_valid = False
        self._restoring = False  # v7.2.6: guard for config restore

        # v7.5.x: one-time auto-detect of cameras on first display so a
        # remembered camera setup (role→identity assignments + per-camera
        # µm/px) auto-restores without the user clicking Detect each session.
        self._auto_detect_done = False
        # Background-probe result → GUI-thread finish (queued connection).
        self._cameras_probed.connect(self._finish_auto_load)

        # v7.2.4: Channel mapping widgets (dynamic)
        self._channel_map_widgets: list[tuple[QLabel, QComboBox]] = []

        self._setup_ui()

        # v7.4.x rev3: seed the role-derived visuals so the page shows
        # its disabled / "Not assigned" state on first display rather
        # than waiting for the first config-apply.
        self._refresh_role_derived_displays()

        # v7.4.2: Persistent control panel — Connect + Jog + Live Position.
        # Lives in the existing per-page left context panel
        # (returned from ``get_context_widget``) so it replaces the old
        # "Saved Configurations" browser that used to live there.
        # Always visible across all 7 sub-pages because the context
        # panel doesn't swap when the sub-page changes.
        from gui.pages.hardware.control_panel import HardwareControlPanel
        self._control_panel = HardwareControlPanel()

    def set_settings(self, settings):
        """v7.4.0-b: Inject Settings instance for the Stage sub-page widgets.

        v7.4.2: Also propagates to the Xbox sub-page and the persistent
        left-side control panel.
        """
        self._settings = settings
        if hasattr(self, '_stage_panel'):
            self._stage_panel.set_settings(settings)
        if hasattr(self, '_xbox_panel'):
            self._xbox_panel.set_settings(settings)
        if hasattr(self, '_control_panel'):
            self._control_panel.set_settings(settings)

    def get_page_title(self) -> str:
        return "Hardware Setup"

    @property
    def hardware_config(self) -> HardwareConfig:
        return self._config

    # ════════════════════════════════════════════════════════════════
    #  UI CONSTRUCTION — v7.2.4 reordered
    # ════════════════════════════════════════════════════════════════

    def _setup_ui(self):
        """v7.4.0-b: Build 7 sub-pages partitioning the hardware config UI.

        Existing section-building code is preserved; each section is added
        to the appropriate sub-page's layout instead of one long scroll.
        After all sections are built, sub-pages are registered with
        ModePage via add_sub_page().
        """
        _bg = COLORS['base']

        # Create per-sub-page scaffolds (scroll, content widget, layout)
        self._sub_layouts: dict[str, QVBoxLayout] = {}
        self._sub_scrolls: dict[str, QScrollArea] = {}
        for key in ("identity", "plate", "pumps_inks", "inks", "needle",
                    "rosette", "cameras"):
            scroll, lay = self._make_subpage_scaffold(_bg)
            self._sub_scrolls[key] = scroll
            self._sub_layouts[key] = lay

        # Stage sub-page is custom (StageHardwarePanel) — defined separately
        from gui.pages.hardware.stage_panel import StageHardwarePanel
        self._stage_panel = StageHardwarePanel(self)
        stage_scroll = QScrollArea()
        stage_scroll.setWidgetResizable(True)
        stage_scroll.setFrameShape(QFrame.NoFrame)
        stage_scroll.setStyleSheet(
            f"QScrollArea {{ background-color: {_bg}; border: none; }}")
        stage_scroll.setWidget(self._stage_panel)
        self._sub_scrolls["stage"] = stage_scroll

        # v7.4.2: dedicated Xbox controller sub-page.
        from gui.pages.hardware.xbox_panel import XboxHardwarePanel
        self._xbox_panel = XboxHardwarePanel(self)
        xbox_scroll = QScrollArea()
        xbox_scroll.setWidgetResizable(True)
        xbox_scroll.setFrameShape(QFrame.NoFrame)
        xbox_scroll.setStyleSheet(
            f"QScrollArea {{ background-color: {_bg}; border: none; }}")
        xbox_scroll.setWidget(self._xbox_panel)
        self._sub_scrolls["xbox"] = xbox_scroll

        # Backward-compat alias: existing code still references
        # self._content_layout in a few spots — make it point at the
        # identity sub-page so any orphaned addWidget calls land somewhere
        # visible rather than crashing.
        self._content_layout = self._sub_layouts["identity"]

        # ── Section 1: Setup Name & Notes ─────────────────────────
        # v7.4.2: per-user feedback — a free-text Name + Notes pair
        # PLUS a scrollable list of every saved hardware config in
        # ``config/hardware/``. Click an entry then Load to apply that
        # accessory setup (plate / pumps / inks / needle / rosette /
        # cameras). The Name field stays free-text; it becomes the
        # filename on the next save.
        name_group = QGroupBox("Setup Name && Notes")
        name_group.setStyleSheet(self._group_style())
        name_outer = QVBoxLayout(name_group)
        name_outer.setSpacing(s(10))

        form = QFormLayout()
        form.setHorizontalSpacing(s(10))
        form.setVerticalSpacing(s(10))
        self.name_edit = QLineEdit()
        self.name_edit.setPlaceholderText("My Experiment Setup")
        self.name_edit.setMinimumWidth(s(350))
        self.name_edit.textChanged.connect(self._on_config_changed)
        form.addRow("Name:", self.name_edit)

        self.notes_edit = QLineEdit()
        self.notes_edit.setPlaceholderText("Optional notes...")
        self.notes_edit.setMinimumWidth(s(350))
        self.notes_edit.textChanged.connect(self._on_config_changed)
        form.addRow("Notes:", self.notes_edit)
        name_outer.addLayout(form)

        # Scrollable list of saved setups
        saved_label = QLabel("Saved setups")
        saved_label.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-weight: 600; "
            f"padding-top: {sp(6)};")
        name_outer.addWidget(saved_label)

        self._setup_list = QListWidget()
        self._setup_list.setAlternatingRowColors(True)
        self._setup_list.setMinimumHeight(s(140))
        self._setup_list.setMaximumHeight(s(220))
        self._setup_list.itemDoubleClicked.connect(
            lambda _item: self._load_selected_setup())
        name_outer.addWidget(self._setup_list)

        setup_btn_row = QHBoxLayout()
        setup_btn_row.setSpacing(s(8))
        self._btn_load_setup = icon_button(
            "Load Setup", "folder-open", object_name="accentBtn",
            tooltip="Apply the selected saved setup to every section below.")
        self._btn_load_setup.clicked.connect(self._load_selected_setup)
        setup_btn_row.addWidget(self._btn_load_setup)
        self._btn_refresh_setups = icon_button(
            "", "refresh",
            tooltip="Re-scan saved hardware configurations")
        self._btn_refresh_setups.setFixedWidth(s(36))
        self._btn_refresh_setups.clicked.connect(
            self._refresh_setup_list)
        setup_btn_row.addWidget(self._btn_refresh_setups)
        setup_btn_row.addStretch(1)
        self._lbl_setup_status = QLabel("")
        self._lbl_setup_status.setStyleSheet(
            f"color: {COLORS['subtext0']};")
        setup_btn_row.addWidget(self._lbl_setup_status, 2)
        name_outer.addLayout(setup_btn_row)

        # Populate
        self._refresh_setup_list()

        self._content_layout.addWidget(name_group)

        # ── Section 2: Well Plate Designer (v7.4.5) ──────────────
        # Inline parametric well-plate sketcher replaces the old format
        # combo. Bundled standards still appear in the picker; users
        # can drop a Grid / Single-Well and impose constraints (Lock,
        # Distance, Coincident, Concentric, Equal-Ø, etc.) — and save
        # custom plates under config/hardware/plates/user/.
        #
        # Backward-compat shim: a hidden QComboBox mirrors the picker so
        # external code that still calls self.plate_combo.currentData()
        # or .findData() keeps working until the migration finishes.
        from gui.pages.hardware.plate_designer import PlateDesignerWidget
        self._plate_designer = PlateDesignerWidget(self, mode="plate")
        self._plate_designer.plate_changed.connect(
            lambda _key: self._on_config_changed())
        self._sub_layouts["plate"].addWidget(self._plate_designer)

        # v7.4.8: the Rosette sub-page hosts a rosette-mode designer that
        # mirrors the plate layout; double-clicking a well drills into its
        # rosette. It shares the plate design object with the Plate page
        # (synced on sub-page switch — see _on_hw_sub_page_changed).
        self._rosette_designer = PlateDesignerWidget(self, mode="rosette")
        self._rosette_designer.save_requested.connect(
            self._on_rosette_save_requested)
        self._rosette_designer.design_edited.connect(
            self._on_rosette_design_edited)

        self.plate_combo = QComboBox()
        self.plate_combo.hide()
        for fmt in sorted(PLATE_DEFINITIONS.keys()):
            pdef = PLATE_DEFINITIONS[fmt]
            rows = pdef.get("rows", "?")
            cols = pdef.get("cols", "?")
            self.plate_combo.addItem(f"{fmt}-well ({rows}×{cols})", fmt)

        # ── Section 3: Ink Library ────────────────────────────────
        ink_group = QGroupBox("Ink Library")
        ink_group.setStyleSheet(self._group_style())
        ink_lay = QVBoxLayout(ink_group)

        self.ink_table = QTableWidget(0, 6)
        self.ink_table.setHorizontalHeaderLabels(
            ["Name", "Type", "Subtype", "Viscosity", "Granule Ø", "Cell Ø"])
        self.ink_table.horizontalHeader().setSectionResizeMode(
            QHeaderView.Stretch)
        self.ink_table.setSelectionBehavior(
            QAbstractItemView.SelectRows)
        self.ink_table.setSelectionMode(
            QAbstractItemView.SingleSelection)
        self.ink_table.setEditTriggers(
            QAbstractItemView.NoEditTriggers)
        self.ink_table.setMaximumHeight(s(160))
        ink_lay.addWidget(self.ink_table)

        # v7.4.2 polish: action row — primary add on the left, edit/remove
        # secondary, destructive in red.
        ink_btns = QHBoxLayout()
        ink_btns.setSpacing(s(8))
        btn_add_ink = icon_button("Add Ink", "plus", object_name="accentBtn")
        btn_add_ink.clicked.connect(self._add_ink)
        ink_btns.addWidget(btn_add_ink)
        btn_edit_ink = icon_button("Edit", "pencil")
        btn_edit_ink.clicked.connect(self._edit_ink)
        ink_btns.addWidget(btn_edit_ink)
        btn_del_ink = icon_button("Remove", "trash", object_name="dangerBtn")
        btn_del_ink.clicked.connect(self._remove_ink)
        ink_btns.addWidget(btn_del_ink)
        ink_btns.addStretch()
        ink_lay.addLayout(ink_btns)

        self._sub_layouts["inks"].addWidget(ink_group)

        # ── v7.5.x: Reagent Locations ─────────────────────────────
        # Pin each reagent (ink/wash/buffer/oil…) to one or more plate
        # wells. Reuses the print workflow's WellPlateView so rosette
        # sub-wells (A1.a) show up automatically once the plate is built.
        self._build_reagent_locations_group()

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
            f"padding: {sp(4)} {sp(8)};")
        self.pump_ink_summary.setWordWrap(True)
        pump_lay.addWidget(self.pump_ink_summary)

        self._sub_layouts["pumps_inks"].addWidget(pump_group)

        # v7.5.x: the per-pump Plunger Setup (Set Dispensed/Set Aspirated) now
        # lives on the Stage sub-page, directly beneath the Z Axis Setup block
        # (order Z, P1, P2, P3) — see StageHardwarePanel.
        # ── v7.5.x: Global pump timing (settle + prime) ──────────────
        # Settle = blocking dwell before AND after every discrete pump
        # actuation (prime, aspirate, dispense, pick/place, EXTRUDE) so the
        # fluid/pressure settles before the next step. Prime = the pre-flow
        # lead-in duration that printing workflows use (volume = flow × time).
        timing_group = QGroupBox("Pump Timing (all pumps)")
        timing_group.setStyleSheet(self._group_style())
        timing_lay = QGridLayout(timing_group)
        timing_lay.setHorizontalSpacing(s(12))
        timing_lay.setVerticalSpacing(s(10))

        timing_lay.addWidget(QLabel("Settle time:"), 0, 0)
        self._pump_settle_spin = QDoubleSpinBox()
        self._pump_settle_spin.setRange(0.0, 10.0)
        self._pump_settle_spin.setDecimals(2)
        self._pump_settle_spin.setSingleStep(0.05)
        self._pump_settle_spin.setSuffix(" s")
        self._pump_settle_spin.setToolTip(
            "Dwell held BEFORE and AFTER every discrete pump move (prime, "
            "aspirate, dispense, pick/place) so the fluid settles "
            "before the workflow advances. Does not apply to the streamed "
            "print path or manual jog. 0 = no dwell.")
        self._pump_settle_spin.valueChanged.connect(self._on_config_changed)
        timing_lay.addWidget(self._pump_settle_spin, 0, 1)

        timing_lay.addWidget(QLabel("Prime time:"), 1, 0)
        self._pump_prime_spin = QDoubleSpinBox()
        self._pump_prime_spin.setRange(0.0, 10.0)
        self._pump_prime_spin.setDecimals(2)
        self._pump_prime_spin.setSingleStep(0.05)
        self._pump_prime_spin.setSuffix(" s")
        self._pump_prime_spin.setToolTip(
            "Pre-flow lead-in duration printing workflows use to prime the "
            "needle: the pump runs at the print flow for this long before the "
            "path starts (prime volume = flow × time). Quick Print seeds its "
            "per-run pre-flow knob from this value.")
        self._pump_prime_spin.valueChanged.connect(self._on_config_changed)
        timing_lay.addWidget(self._pump_prime_spin, 1, 1)
        timing_lay.setColumnStretch(2, 1)

        self._sub_layouts["pumps_inks"].addWidget(timing_group)

        # v7.2.9: Ink Swap Strategy UI moved to Print Setup → Plan of Action

        # ── Section 5: Needle Configuration (v7.2.4: MOVED DOWN) ─
        needle_group = QGroupBox("Needle Configuration")
        needle_group.setStyleSheet(self._group_style())
        needle_lay = QGridLayout(needle_group)
        # v7.4.2 polish: roomier grid spacing.
        needle_lay.setHorizontalSpacing(s(12))
        needle_lay.setVerticalSpacing(s(10))

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
            f"color: {COLORS.get('subtext0', '#a6adc8')}; ")
        needle_lay.addWidget(self.needle_info_label, 2, 0, 1, 4)

        self._sub_layouts["needle"].addWidget(needle_group)

        # ── Section 6: Needle Channel → Pump Mapping (v7.2.4: NEW) ─
        self.channel_map_group = QGroupBox("Needle Channel Assignment")
        self.channel_map_group.setStyleSheet(self._group_style())
        self._channel_map_layout = QVBoxLayout(self.channel_map_group)

        # Info label
        self.channel_map_info = QLabel(
            "Each needle channel must be assigned to a unique enabled pump.")
        self.channel_map_info.setStyleSheet(
            f"color: {COLORS.get('subtext0', '#a6adc8')}; ")
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
            f"padding: {sp(2)} {sp(4)};")
        self._channel_map_layout.addWidget(self.channel_map_status)

        self._sub_layouts["needle"].addWidget(self.channel_map_group)

        # Build initial channel rows
        self._rebuild_channel_map_rows()

        # ── Section 7a: Rosette Designer (v7.4.8) ─────────────────
        # Primary content of the Rosette sub-page: the plate layout with
        # double-click-to-drill-into-a-well rosette design.
        self._sub_layouts["rosette"].addWidget(self._rosette_designer, 1)

        # ── Section 7b: Rosette Library (legacy RosetteInsert presets) ─
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
        self.rosette_table.setMaximumHeight(s(140))
        ros_lay.addWidget(self.rosette_table)

        # v7.4.2 polish: action row — primary add, secondary edit, danger remove.
        ros_btns = QHBoxLayout()
        ros_btns.setSpacing(s(8))
        btn_add_ros = icon_button("Add Rosette", "plus", object_name="accentBtn")
        btn_add_ros.clicked.connect(self._add_rosette)
        ros_btns.addWidget(btn_add_ros)
        btn_edit_ros = icon_button("Edit", "pencil")
        btn_edit_ros.clicked.connect(self._edit_rosette)
        ros_btns.addWidget(btn_edit_ros)
        btn_del_ros = icon_button("Remove", "trash", object_name="dangerBtn")
        btn_del_ros.clicked.connect(self._remove_rosette)
        ros_btns.addWidget(btn_del_ros)
        ros_btns.addStretch()
        ros_lay.addLayout(ros_btns)

        self._sub_layouts["rosette"].addWidget(ros_group)

        # ── Section 0: Camera Detection & Assignment (v7.4.x rev3) ──
        # Top-of-page block. Detect row stays single-line; per-slot
        # rows are compact cards with a Cam-N pill, source picker,
        # role picker, and a colored role badge that lights up as
        # soon as the user picks a non-Unassigned role.
        assign_group = QGroupBox("Camera Detection & Assignment")
        assign_group.setStyleSheet(self._group_style())
        assign_lay = QVBoxLayout(assign_group)
        assign_lay.setContentsMargins(s(14), s(20), s(14), s(14))
        assign_lay.setSpacing(s(12))

        intro = QLabel(
            "Discover live camera sources, assign each slot a source, "
            "and tag it with its workflow role. Roles drive the rest of "
            "the page below."
        )
        intro.setWordWrap(True)
        intro.setStyleSheet(
            f"color: {COLORS.get('subtext0', '#a6adc8')}; "
            f"font-size: {scaled_font_size(9)}pt;"
        )
        assign_lay.addWidget(intro)

        # v7.5.x: Save / Load the FULL camera setup as this machine's default.
        # Save snapshots every section (sources→roles, µm/px, image correction,
        # camera-side hardware controls, and which cameras are running). Load
        # detects, restores all of that, and starts the saved cameras. The same
        # restore runs automatically at startup.
        setup_row = QHBoxLayout()
        setup_row.setSpacing(s(10))
        self._btn_save_cam_setup = icon_button(
            "Save Camera Settings", "save", object_name="accentBtn",
            tooltip="Save the current camera setup (sources, roles, µm/px, "
                    "image correction, camera hardware controls, and which "
                    "cameras are running) as this machine's default.")
        self._btn_save_cam_setup.clicked.connect(self._on_save_camera_settings)
        setup_row.addWidget(self._btn_save_cam_setup)
        self._btn_load_cam_setup = icon_button(
            "Load Cameras", "play",
            tooltip="Detect cameras, restore the saved setup for every "
                    "section, and start the cameras that were running when "
                    "you last saved.")
        self._btn_load_cam_setup.clicked.connect(self._on_load_cameras)
        setup_row.addWidget(self._btn_load_cam_setup)
        self._lbl_cam_setup_status = StatusBadge("", variant="pending")
        self._lbl_cam_setup_status.setVisible(False)
        setup_row.addWidget(self._lbl_cam_setup_status)
        setup_row.addStretch()
        assign_lay.addLayout(setup_row)

        detect_row = QHBoxLayout()
        detect_row.setSpacing(s(10))
        self._btn_detect_live_cams = icon_button(
            "Detect Cameras", "search", object_name="accentBtn",
            tooltip="Scan for available cameras (OpenCV, ToupCam, Simulated)")
        self._btn_detect_live_cams.setMinimumWidth(s(170))
        self._btn_detect_live_cams.clicked.connect(self._on_detect_live_cameras)
        detect_row.addWidget(self._btn_detect_live_cams)
        self._lbl_live_cam_count = StatusBadge("0 sources found", variant="pending")
        detect_row.addWidget(self._lbl_live_cam_count)
        detect_row.addStretch()
        assign_lay.addLayout(detect_row)

        self._live_cam_rows_container = QVBoxLayout()
        self._live_cam_rows_container.setSpacing(s(8))
        self._live_cam_source_combos: list[QComboBox] = []
        self._live_cam_role_combos: list[QComboBox] = []
        self._live_cam_role_badges: list[StatusBadge] = []
        # v7.5.x: per-slot Start/Stop + collapsible live preview so the user
        # can power a camera on and verify the feed right on the calibration
        # page. Previews (CameraFeedView) are created lazily once the shared
        # CameraManager arrives (see set_camera_manager).
        self._live_cam_start_btns: list[QPushButton] = []
        self._live_cam_preview_holders: list[QWidget] = []
        self._live_cam_previews: list = []
        # v7.5.x: per-slot image-correction control strip (brightness/contrast/
        # gamma sliders), built lazily under each preview in _ensure_camera_previews.
        self._live_cam_correction: list = []
        self._cam_preview_signals_wired = False

        from gui.styles import build_glass_panel_style
        assign_glass = build_glass_panel_style("camMiniCard")
        max_cams = 3  # CameraManager default; updated via set_camera_manager.
        for i in range(max_cams):
            row = QFrame()
            row.setObjectName("camMiniCard")
            row.setStyleSheet(assign_glass)
            rl = QGridLayout(row)
            rl.setContentsMargins(s(14), s(12), s(14), s(12))
            rl.setHorizontalSpacing(s(12))
            rl.setVerticalSpacing(s(8))
            rl.setColumnStretch(0, 0)
            rl.setColumnStretch(1, 0)
            rl.setColumnStretch(2, 1)
            rl.setColumnStretch(3, 0)
            rl.setColumnStretch(4, 0)

            # Cam N pill (uses the accent color so it reads as a chip).
            cam_pill = QLabel(f"Cam {i + 1}")
            cam_pill.setAlignment(Qt.AlignCenter)
            cam_pill.setMinimumWidth(s(54))
            cam_pill.setStyleSheet(
                f"QLabel {{"
                f"  background-color: rgba(137, 180, 250, 30);"
                f"  color: {COLORS['blue']};"
                f"  border: 1px solid {COLORS['blue']};"
                f"  border-radius: {sp(10)};"
                f"  padding: {sp(2)} {sp(10)};"
                f"  font-size: {sf(10)}pt;"
                f"  font-weight: 700;"
                f"}}"
            )
            rl.addWidget(cam_pill, 0, 0, 2, 1, Qt.AlignVCenter)

            rl.addWidget(self._field_label("Source"), 0, 1)
            src = QComboBox()
            src.addItem("— None —", None)
            src.setMinimumWidth(s(220))
            rl.addWidget(src, 0, 2)
            self._live_cam_source_combos.append(src)
            # Applying the chosen source to the shared CameraManager so a
            # Start below opens *this* slot's selected camera. The combo was
            # previously display-only (never wired to set_source).
            src.currentIndexChanged.connect(
                lambda _idx, cam_i=i: self._on_live_cam_source_changed(cam_i)
            )

            # Role badge — lives in column 3 and tracks the combo's
            # current selection. Empty/Unassigned variant by default.
            role_badge = StatusBadge("Unassigned", variant="pending")
            rl.addWidget(role_badge, 0, 3, 2, 1, Qt.AlignVCenter)
            self._live_cam_role_badges.append(role_badge)

            rl.addWidget(self._field_label("Role"), 1, 1)
            role_combo = QComboBox()
            role_combo.addItem("Unassigned", CameraRole.UNASSIGNED)
            role_combo.addItem("Microscope", CameraRole.MICROSCOPE)
            role_combo.addItem("Needle X-view", CameraRole.NEEDLE_X)
            role_combo.addItem("Needle Y-view", CameraRole.NEEDLE_Y)
            role_combo.setToolTip(
                "Workflow role for this camera slot. All non-Unassigned "
                "roles are singletons — assigning one here automatically "
                "clears it from any other slot."
            )
            role_combo.currentIndexChanged.connect(
                lambda idx_combo, cam_i=i: self._on_live_cam_role_changed(cam_i)
            )
            rl.addWidget(role_combo, 1, 2)
            self._live_cam_role_combos.append(role_combo)

            # Start/Stop toggle (col 4) — powers this slot's camera on/off
            # via the shared CameraManager. Disabled until a source is picked.
            start_btn = QPushButton("▶ Start")
            start_btn.setObjectName("accentBtn")
            start_btn.setEnabled(False)
            start_btn.setMinimumWidth(s(104))
            start_btn.setToolTip(
                "Start/stop this camera so you can verify the live feed below "
                "and run its calibration."
            )
            start_btn.clicked.connect(
                lambda _checked=False, cam_i=i: self._on_toggle_camera(cam_i)
            )
            rl.addWidget(start_btn, 0, 4, 2, 1, Qt.AlignVCenter)
            self._live_cam_start_btns.append(start_btn)

            # Collapsible live preview (row 2, full width) — hidden until the
            # camera is running. Filled with a CameraFeedView lazily once the
            # CameraManager is available (set_camera_manager).
            preview = QWidget()
            preview.setVisible(False)
            pv_lay = QVBoxLayout(preview)
            pv_lay.setContentsMargins(0, s(8), 0, 0)
            pv_lay.setSpacing(0)
            preview.setMinimumHeight(s(170))
            preview.setMaximumHeight(s(260))
            rl.addWidget(preview, 2, 0, 1, 5)
            self._live_cam_preview_holders.append(preview)
            self._live_cam_previews.append(None)
            self._live_cam_correction.append(None)

            self._live_cam_rows_container.addWidget(row)

        assign_lay.addLayout(self._live_cam_rows_container)
        self._sub_layouts["cameras"].addWidget(assign_group)

        # ── Section A: Microscope Camera Setup (v7.4.x) ──────────
        # Houses the camera specification (model, resolution, computed/
        # override scale, FOV). The slot designated MICROSCOPE in the
        # Detection & Assignment section above is the camera this
        # section describes. The objective the user has installed
        # lives in Section C (Objective Calibration Setup) and drives
        # the effective magnification used for
        # `computed_micron_per_pixel`.
        cam_group = QGroupBox("Microscope Camera Setup")
        cam_group.setStyleSheet(self._group_style())
        cam_outer = QVBoxLayout(cam_group)
        cam_outer.setContentsMargins(s(14), s(20), s(14), s(14))
        cam_outer.setSpacing(s(12))

        # Header strip: "Assigned camera" + status badge.
        mic_header = QHBoxLayout()
        mic_header.setSpacing(s(10))
        mic_header.addWidget(QLabel("Assigned camera:"))
        self._microscope_status = StatusBadge("Not assigned", variant="pending")
        self._microscope_status.setToolTip(
            "Set the microscope camera by picking the MICROSCOPE role "
            "in the Camera Detection & Assignment section at the top."
        )
        mic_header.addWidget(self._microscope_status)
        mic_header.addStretch()
        cam_outer.addLayout(mic_header)

        # Spec frame: everything below is the camera's spec + calibration
        # configuration. Wrapped in its own QFrame so we can disable the
        # whole block in one call when no microscope is assigned.
        self._microscope_spec_frame = QFrame()
        self._microscope_spec_frame.setObjectName("camMiniCard")
        from gui.styles import build_glass_panel_style
        self._microscope_spec_frame.setStyleSheet(
            build_glass_panel_style("camMiniCard")
        )
        spec_form = QFormLayout(self._microscope_spec_frame)
        spec_form.setLabelAlignment(Qt.AlignmentFlag.AlignRight)
        spec_form.setContentsMargins(s(14), s(12), s(14), s(12))
        spec_form.setHorizontalSpacing(s(14))
        spec_form.setVerticalSpacing(s(10))

        self.camera_combo = QComboBox()
        self.camera_combo.addItem("None", None)
        for name, spec in self._camera_catalog.items():
            self.camera_combo.addItem(name, name)
        self.camera_combo.currentIndexChanged.connect(self._on_camera_changed)
        spec_form.addRow(self._field_label("Camera spec"), self.camera_combo)

        self.cam_resolution_combo = QComboBox()
        self.cam_resolution_combo.currentIndexChanged.connect(self._on_camera_changed)
        spec_form.addRow(self._field_label("Resolution"), self.cam_resolution_combo)

        self.cam_objective_combo = QComboBox()
        self.cam_objective_combo.setEditable(False)
        self.cam_objective_combo.setToolTip(
            "Set this in the Objective Calibration Setup section below "
            "via the 'Currently installed' combo."
        )
        self.cam_objective_combo.setEnabled(False)
        spec_form.addRow(
            self._field_label("Installed objective"), self.cam_objective_combo
        )

        self.cam_scale_label = QLabel("—")
        self.cam_scale_label.setStyleSheet(
            f"color: {COLORS.get('text', '#cdd6f4')}; font-weight: 600;"
        )
        self.cam_scale_label.setWordWrap(True)
        spec_form.addRow(self._field_label("Pixel scale"), self.cam_scale_label)

        # Override checkbox + custom scale on the same row.
        override_row = QHBoxLayout()
        override_row.setSpacing(s(8))
        self.cam_override_check = QCheckBox("Use custom scale")
        self.cam_override_check.toggled.connect(self._on_camera_override_toggled)
        override_row.addWidget(self.cam_override_check)
        self.cam_override_spin = QDoubleSpinBox()
        self.cam_override_spin.setRange(0.01, 1000.0)
        self.cam_override_spin.setDecimals(2)
        self.cam_override_spin.setSuffix(" µm/px")
        self.cam_override_spin.setValue(1.67)
        self.cam_override_spin.setEnabled(False)
        self.cam_override_spin.valueChanged.connect(self._on_config_changed)
        override_row.addWidget(self.cam_override_spin)
        override_row.addStretch(1)
        override_holder = QWidget()
        override_holder.setLayout(override_row)
        spec_form.addRow(self._field_label("Override"), override_holder)

        self.cam_fov_label = QLabel("—")
        self.cam_fov_label.setStyleSheet(
            f"color: {COLORS.get('subtext0', '#a6adc8')};"
        )
        spec_form.addRow(self._field_label("Field of view"), self.cam_fov_label)

        cam_outer.addWidget(self._microscope_spec_frame)
        self._sub_layouts["cameras"].addWidget(cam_group)

        # ── Section B: Needle Cameras Setup (v7.4.x rev3) ───────
        # Exactly two needle cameras, side-by-side. Each card binds
        # to a fixed role; assignment + calibration are independent.
        live_cam_group = QGroupBox("Needle Cameras Setup")
        live_cam_group.setStyleSheet(self._group_style())
        live_cam_lay = QVBoxLayout(live_cam_group)
        live_cam_lay.setContentsMargins(s(14), s(20), s(14), s(14))
        live_cam_lay.setSpacing(s(10))

        needle_intro = QLabel(
            "Two side cameras look down the X and Y axes. Assign each a "
            "camera in the section above; then run an independent stage-"
            "motion µm/px calibration for each."
        )
        needle_intro.setWordWrap(True)
        needle_intro.setStyleSheet(
            f"color: {COLORS.get('subtext0', '#a6adc8')}; "
            f"font-size: {scaled_font_size(9)}pt;"
        )
        live_cam_lay.addWidget(needle_intro)

        # Two cards side-by-side.
        from gui.styles import build_glass_panel_style
        glass_style = build_glass_panel_style("camMiniCard")
        needle_row = QHBoxLayout()
        needle_row.setSpacing(s(12))
        self._needle_cards: dict[CameraRole, dict[str, QWidget]] = {}
        for role, title, axis_color in (
            (CameraRole.NEEDLE_X, "Needle X-view", COLORS.get("peach", "#fab387")),
            (CameraRole.NEEDLE_Y, "Needle Y-view", COLORS.get("sky", "#89dceb")),
        ):
            card = QFrame()
            card.setObjectName("camMiniCard")
            card.setStyleSheet(glass_style)
            inner = QVBoxLayout(card)
            inner.setContentsMargins(s(14), s(12), s(14), s(12))
            inner.setSpacing(s(10))

            # Header: colored title + status badge on the right.
            head = QHBoxLayout()
            head.setSpacing(s(8))
            title_lbl = QLabel(title)
            title_lbl.setStyleSheet(
                f"color: {axis_color}; font-weight: 700; "
                f"font-size: {scaled_font_size(11)}pt;"
            )
            head.addWidget(title_lbl)
            head.addStretch(1)
            status = StatusBadge("Not assigned", variant="pending")
            head.addWidget(status)
            inner.addLayout(head)

            # Info form: current µm/px.
            info = QFormLayout()
            info.setLabelAlignment(Qt.AlignmentFlag.AlignRight)
            info.setHorizontalSpacing(s(12))
            info.setVerticalSpacing(s(6))
            info.setContentsMargins(0, 0, 0, 0)
            umpx = QLabel("—")
            umpx.setStyleSheet(
                f"color: {COLORS.get('text', '#cdd6f4')}; "
                f"font-weight: 700; font-size: {scaled_font_size(11)}pt;"
            )
            info.addRow(self._field_label("Current µm/px"), umpx)
            inner.addLayout(info)

            # Action: calibrate button (full-width).
            cal_btn = icon_button(
                "Calibrate µm/px…", "ruler",
                object_name="accentBtn",
                tooltip=(
                    "Stage-motion calibration: move the stage a known "
                    "distance and correlate pixel displacement to "
                    "compute µm/px for this needle camera."
                ),
            )
            cal_btn.setEnabled(False)
            cal_btn.clicked.connect(
                lambda _checked=False, r=role: self._on_calibrate_needle(r)
            )
            inner.addWidget(cal_btn)

            # Empty-state hint (only visible when not assigned).
            hint = QLabel("↑ Assign a camera with this role above")
            hint.setAlignment(Qt.AlignCenter)
            hint.setStyleSheet(
                f"color: {COLORS.get('subtext0', '#a6adc8')}; "
                f"font-style: italic; "
                f"font-size: {scaled_font_size(8)}pt;"
            )
            inner.addWidget(hint)

            needle_row.addWidget(card, stretch=1)
            self._needle_cards[role] = {
                "card": card,
                "status": status,
                "umpx": umpx,
                "calibrate": cal_btn,
                "hint": hint,
            }

        live_cam_lay.addLayout(needle_row)
        self._sub_layouts["cameras"].addWidget(live_cam_group)

        # ── Section 8c: Microscope Objective Calibration (v7.4.x) ──
        from gui.pages.hardware.objective_calibration_card import (
            ObjectiveCalibrationCard,
        )
        self._objective_cal_card = ObjectiveCalibrationCard(
            getattr(self, "_camera_manager", None),
            lambda: self._config,
            parent=self,
            controller_getter=lambda: getattr(self, "_controller", None),
        )
        self._objective_cal_card.calibration_changed.connect(
            self._on_config_changed
        )
        self._objective_cal_card.calibration_changed.connect(
            self._refresh_installed_objective_combo
        )
        self._objective_cal_card.calibration_changed.connect(
            self._update_camera_info_labels
        )
        self._objective_cal_card.um_per_px_committed.connect(
            self.set_calibrated_um_per_px
        )
        self._sub_layouts["cameras"].addWidget(self._objective_cal_card)

        # ── Section 9: Setup Status ───────────────────────────────
        status_group = QGroupBox("Setup Status")
        status_group.setStyleSheet(self._group_style())
        status_lay = QVBoxLayout(status_group)

        self.validity_label = QLabel("⚠ Setup incomplete")
        self.validity_label.setStyleSheet(
            f"color: {COLORS.get('yellow', '#f9e2af')};")
        self.validity_label.setFont(QFont("", scaled_font_size(10), QFont.Bold))
        self.validity_label.setWordWrap(True)
        status_lay.addWidget(self.validity_label)

        self._content_layout.addWidget(status_group)

        # ── Section 9: Actions ────────────────────────────────────
        # v7.4.2 polish: primary save button + secondary load.
        actions_group = QGroupBox("Actions")
        actions_group.setStyleSheet(self._group_style())
        actions_lay = QHBoxLayout(actions_group)
        actions_lay.setSpacing(s(8))

        btn_save = icon_button(
            "Save Config", "save", object_name="accentBtn",
            tooltip="Save the current hardware configuration to disk.")
        btn_save.clicked.connect(self._save_config)
        actions_lay.addStretch()
        actions_lay.addWidget(btn_save)

        btn_load = icon_button(
            "Load Config", "folder-open",
            tooltip="Load a previously saved hardware configuration.")
        btn_load.clicked.connect(self._load_config)
        actions_lay.addWidget(btn_load)

        self._content_layout.addWidget(actions_group)

        # ── Finalize sub-pages (v7.4.0-b) ─────────────────────────
        # Add stretch to each sub-page layout so groups stack at the top.
        for key in ("identity", "plate", "pumps_inks", "inks", "needle",
                    "rosette", "cameras"):
            self._sub_layouts[key].addStretch()

        # Register sub-pages with ModePage in user-facing order.
        # v7.4.1: Device (Stage) is FIRST because it's the one-time
        # initial setup of the physical machine. Everything below
        # configures the experiment.
        # v7.4.2: pass icon-factory names instead of emoji so the tab
        # bar renders crisp solid-white SVG icons.
        self.add_sub_page("settings",  "Device",          self._sub_scrolls["stage"])
        self.add_sub_page("file-text", "Identity",        self._sub_scrolls["identity"])
        self._plate_sub_index = len(self._sub_pages)
        self.add_sub_page("microscope","Plate",           self._sub_scrolls["plate"])
        # v7.5.x: order the dependent setups left-to-right so each
        # section's options build on the ones to its left:
        # Rosette → Ink → Needle → Pump.
        self._rosette_sub_index = len(self._sub_pages)
        self.add_sub_page("flower",    "Rosette",         self._sub_scrolls["rosette"])
        self.add_sub_page("flask",     "Ink",             self._sub_scrolls["inks"])
        self.add_sub_page("needle",    "Needle",          self._sub_scrolls["needle"])
        self.add_sub_page("droplet",   "Pump",            self._sub_scrolls["pumps_inks"])
        self.add_sub_page("camera",    "Cameras",         self._sub_scrolls["cameras"])
        self.add_sub_page("gamepad",   "Xbox Controller", self._sub_scrolls["xbox"])

        # v7.4.8: sync the rosette designer to the plate layout when the
        # Rosette sub-page is shown, and refresh the plate when returning.
        self.sub_page_changed.connect(self._on_hw_sub_page_changed)

    # v7.4.0-b: Helper to build a per-sub-page scroll + content layout
    def _make_subpage_scaffold(self, bg: str) -> tuple[QScrollArea, QVBoxLayout]:
        # v7.4.2 polish: generous margins + section spacing so cards
        # breathe instead of crowding each other.
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        scroll.setStyleSheet(
            f"QScrollArea {{ background-color: {bg}; border: none; }}")
        content = QWidget()
        content.setStyleSheet(f"background-color: {bg};")
        layout = QVBoxLayout(content)
        layout.setSpacing(s(18))
        layout.setContentsMargins(s(20), s(20), s(20), s(20))
        scroll.setWidget(content)
        return scroll, layout

    # ── v7.4.8: Plate ↔ Rosette designer sync ────────────────────

    def _on_hw_sub_page_changed(self, index: int) -> None:
        """Keep the rosette designer mirroring the plate layout.

        On showing the Rosette sub-page, adopt the Plate page's current
        design (shared object) so the rosette designer starts from the
        same plate. On returning to the Plate sub-page, re-render so any
        rosettes added on the Rosette page show their badges.
        """
        if not hasattr(self, "_rosette_designer"):
            return
        if index == getattr(self, "_rosette_sub_index", -1):
            self._rosette_designer.adopt_design(
                self._plate_designer.current_design(),
                key=self._plate_designer.current_plate_key())
        elif index == getattr(self, "_plate_sub_index", -1):
            self._plate_designer.refresh()

    def _on_rosette_design_edited(self) -> None:
        """A rosette edit mutates the shared plate design → mark dirty so
        the user knows to save (Save works on either sub-page)."""
        self._plate_designer._dirty = True
        self._plate_designer._update_dirty_label()
        self._on_config_changed()

    def _on_rosette_save_requested(self) -> None:
        """Rosette page 'Save plate' → save the shared plate via the Plate
        page (forks standards to a custom name), then clear the rosette
        page's unsaved flag + re-sync its key."""
        self._plate_designer._on_save()
        # Mirror the (possibly new) key + cleared dirty state onto the
        # rosette designer so its header stops showing "unsaved".
        self._rosette_designer._current_key = (
            self._plate_designer.current_plate_key())
        self._rosette_designer._dirty = False
        self._rosette_designer._update_dirty_label()

    # ════════════════════════════════════════════════════════════════
    #  SHARED STYLES
    # ════════════════════════════════════════════════════════════════

    @staticmethod
    def _group_style() -> str:
        """v7.2.4: Delegates to centralized SECTION_TITLE_STYLE."""
        return SECTION_TITLE_STYLE

    # ── v7.4.x rev3 polish helpers ────────────────────────────────

    @staticmethod
    def _field_label(text: str) -> QLabel:
        """Compact label for form-style rows on the Cameras sub-page."""
        lbl = QLabel(text)
        lbl.setStyleSheet(
            f"color: {COLORS.get('subtext0', '#a6adc8')}; "
            f"font-size: {scaled_font_size(9)}pt; letter-spacing: 0.4px;"
        )
        return lbl

    @staticmethod
    def _role_badge_props(role) -> tuple[str, str]:
        """Map a CameraRole to a (label, StatusBadge variant) pair."""
        if role == CameraRole.MICROSCOPE:
            return ("Microscope", "info")
        if role == CameraRole.NEEDLE_X:
            return ("Needle X", "warn")
        if role == CameraRole.NEEDLE_Y:
            return ("Needle Y", "warn")
        return ("Unassigned", "pending")

    # ════════════════════════════════════════════════════════════════
    #  v7.4.0-b: SUB-PAGE TITLE FOR MODE PAGE
    # ════════════════════════════════════════════════════════════════

    def get_sub_page_title(self) -> str:
        """Override ModePage to return descriptive sub-page name.

        v7.4.1: Device sub-page first (initial setup), then experiment
        sub-pages.
        """
        labels = ["Hardware: Device", "Hardware: Identity", "Hardware: Plate",
                  "Hardware: Pump", "Hardware: Needle", "Hardware: Ink",
                  "Hardware: Rosette", "Hardware: Cameras",
                  "Hardware: Xbox Controller"]
        idx = self.get_active_index()
        return labels[idx] if 0 <= idx < len(labels) else "Hardware Setup"

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

    def _pump_ink_names(self) -> list[str]:
        """Ink-library names a pump may aspirate — PRINTABLE inks only.

        v7.5.x: service reagents (wash/waste/buffer/oil — by well type OR by an
        ink literally named one of those roles) are excluded; a pump never
        aspirates from a service well.
        """
        return [
            name for name, ink in self._config.ink_library.items()
            if not is_service_reagent(name, getattr(ink, "ink_type", ""))
        ]

    def _refresh_pump_ink_exclusions(self):
        """
        v7.2.8: Refresh ink checklists in all pump widgets.
        No exclusion — inks can be assigned to multiple pumps.
        v7.5.x: service reagents are filtered out (printable inks only).
        """
        ink_names = self._pump_ink_names()
        for pid, pw in self._pump_widgets.items():
            pw.set_ink_names(ink_names)

    def _update_pump_ink_summary(self):
        """v7.2.8: Update pump-ink summary label (multi-ink)."""
        parts = []
        for pid in ["P1", "P2", "P3"]:
            pw = self._pump_widgets[pid]
            if pw.enable_check.isChecked():
                ink_names = pw.get_selected_ink_names()
                if ink_names:
                    parts.append(f"{pid}→[{', '.join(ink_names)}]")
                else:
                    parts.append(f"{pid}→(none)")
        if parts:
            self.pump_ink_summary.setText("Assignment: " + ", ".join(parts))
        else:
            self.pump_ink_summary.setText("No pumps enabled")

    # ════════════════════════════════════════════════════════════════
    #  v7.3.0: CAMERA CONFIGURATION
    # ════════════════════════════════════════════════════════════════

    def _on_camera_changed(self):
        """Called when camera selection, resolution, or magnification changes."""
        cam_name = self.camera_combo.currentData()
        spec = self._camera_catalog.get(cam_name) if cam_name else None

        # Rebuild resolution combo when camera changes
        sender = self.sender()
        if sender is self.camera_combo:
            self.cam_resolution_combo.blockSignals(True)
            self.cam_resolution_combo.clear()
            if spec:
                for res in spec.preview_resolutions:
                    self.cam_resolution_combo.addItem(
                        f"{res[0]} × {res[1]}", list(res))
                # Default to lowest resolution for live preview
                if spec.preview_resolutions:
                    self.cam_resolution_combo.setCurrentIndex(
                        len(spec.preview_resolutions) - 1)
            self.cam_resolution_combo.blockSignals(False)

        # v7.5.x: if the microscope camera is live, push the chosen resolution
        # to the *device* too (mapped to the nearest supported mode), so the
        # spec combo and the hardware stay in sync. The gear-button dialog
        # offers the device's exact native resolutions.
        if sender is self.cam_resolution_combo:
            self._maybe_apply_resolution_to_device()

        self._update_camera_info_labels()
        self._on_config_changed()

    def _maybe_apply_resolution_to_device(self):
        """Drive the live microscope camera to the spec-combo resolution."""
        mgr = getattr(self, "_camera_manager", None)
        if mgr is None:
            return
        mic_idx = self._config.camera_for_role(CameraRole.MICROSCOPE)
        if mic_idx is None or not mgr.is_running(mic_idx):
            return
        data = self.cam_resolution_combo.currentData()
        if not data:
            return
        actual = mgr.set_capture_resolution(mic_idx, int(data[0]), int(data[1]))
        logger.info(
            f"Microscope capture resolution requested {tuple(data)} -> "
            f"device adopted {actual}")

    def _on_camera_override_toggled(self, checked: bool):
        """Toggle between computed and custom micron/pixel scale."""
        self.cam_override_spin.setEnabled(checked)
        self._update_camera_info_labels()
        self._on_config_changed()

    # ── Microscope camera slot (v7.4.x) ───────────────────────────

    def _refresh_installed_objective_combo(self):
        """Sync the read-only 'Installed objective' display to the
        user's objectives library + the card's current selection."""
        from SupportClasses.ObjectiveCalibration import get_store
        store = get_store()
        current = getattr(
            self._config.camera_config, "current_objective_name", None
        )
        self.cam_objective_combo.blockSignals(True)
        self.cam_objective_combo.clear()
        if not store.objective_names():
            self.cam_objective_combo.addItem("(no objectives defined)", None)
            self.cam_objective_combo.setCurrentIndex(0)
        else:
            for name in store.objective_names():
                nominal = store.nominal_magnification(name) or 0.0
                self.cam_objective_combo.addItem(f"{name} ({nominal:g}×)", name)
            target = self.cam_objective_combo.findData(current)
            if target >= 0:
                self.cam_objective_combo.setCurrentIndex(target)
            else:
                self.cam_objective_combo.setCurrentIndex(0)
        self.cam_objective_combo.blockSignals(False)

    def _refresh_role_derived_displays(self):
        """Refresh every visual element that mirrors role assignments.

        Run after any role-combo change so the Section 0 role badges,
        the Microscope Camera Setup readout, and the Needle Cameras
        Setup cards all reflect reality.
        """
        # Section 0 — per-slot role badges.
        for i, badge in enumerate(getattr(self, "_live_cam_role_badges", [])):
            role = (
                self._config.camera_roles[i]
                if i < len(self._config.camera_roles)
                else CameraRole.UNASSIGNED
            )
            label, variant = self._role_badge_props(role)
            badge.set_status(variant, label)

        # Section A — microscope assignment status badge.
        mic_idx = self._config.camera_for_role(CameraRole.MICROSCOPE)
        if hasattr(self, "_microscope_status"):
            if mic_idx is None:
                self._microscope_status.set_status("pending", "Not assigned")
            else:
                self._microscope_status.set_status("info", f"Cam {mic_idx + 1}")
        # Section A — dim the spec controls when nothing is assigned.
        if hasattr(self, "_microscope_spec_frame"):
            self._microscope_spec_frame.setEnabled(mic_idx is not None)

        # Section B — per-needle assignment + current µm/px + button.
        mgr = getattr(self, "_camera_manager", None)
        for role, widgets in getattr(self, "_needle_cards", {}).items():
            slot = self._config.camera_for_role(role)
            if slot is None:
                widgets["status"].set_status("pending", "Not assigned")
                widgets["umpx"].setText("—")
                widgets["calibrate"].setEnabled(False)
                widgets["hint"].setVisible(True)
            else:
                widgets["status"].set_status("ok", f"Cam {slot + 1}")
                if mgr is not None:
                    try:
                        # v7.5.x: only show a number once the camera has been
                        # explicitly calibrated — otherwise the 1.67 seed
                        # default reads as a real (but bogus) calibration.
                        if mgr.is_um_per_px_calibrated(slot):
                            umpx = mgr.get_um_per_px(slot)
                            rot = mgr.get_rotation_deg(slot)
                            rot_txt = (f"  @ {rot:.0f}°"
                                       if rot is not None else "")
                            widgets["umpx"].setText(f"{umpx:.4f} µm/px{rot_txt}")
                        else:
                            widgets["umpx"].setText("— (not calibrated)")
                    except Exception:
                        widgets["umpx"].setText("—")
                widgets["calibrate"].setEnabled(True)
                widgets["hint"].setVisible(False)

    # ── Live Camera Sources (v7.3.3) ─────────────────────────────

    def set_camera_manager(self, manager):
        """v7.3.3: Receive shared CameraManager for live camera detection."""
        self._camera_manager = manager
        if hasattr(self, "_objective_cal_card"):
            self._objective_cal_card.set_camera_manager(manager)
        # v7.5.x: wire the per-slot Start/Stop + live preview to the manager.
        if manager is not None and not self._cam_preview_signals_wired:
            try:
                manager.camera_started.connect(
                    lambda *_: self._refresh_camera_preview_state())
                manager.camera_stopped.connect(
                    lambda *_: self._refresh_camera_preview_state())
                # v7.5.x: on every camera start, restore persisted hardware
                # controls and log the read-back settings to the terminal so
                # the operator can confirm they come FROM the camera.
                manager.camera_started.connect(self._on_camera_started_hw)
                self._cam_preview_signals_wired = True
            except Exception as exc:
                logger.debug(f"camera preview signal wiring skipped: {exc}")
        self._ensure_camera_previews()
        # Re-activate stored µm/px for any already-assigned slots (no-op
        # until cameras are detected + a source is assigned).
        self._restore_all_calibrations()
        self._refresh_camera_preview_state()
        # v7.5.x: the manager is now available — schedule the one-time
        # auto-detect that restores a remembered camera setup. Runs at startup
        # (manager is wired before the window is shown) so the calibration is
        # live for every page, not only after the user opens Hardware Setup.
        # Idempotent with showEvent via the _auto_detect_done guard.
        self._maybe_auto_detect_cameras()

    def showEvent(self, event):
        """v7.5.x: on first display, auto-restore a remembered camera setup.

        Hardware Setup is the startup landing page, so this fires at launch
        (and again the first time the user navigates here if not). It triggers
        a one-time camera detect → source auto-assign → per-camera µm/px
        restore, so a previously-calibrated needle setup comes back without
        the user clicking Detect every session.
        """
        super().showEvent(event)
        self._maybe_auto_detect_cameras()

    def _maybe_auto_detect_cameras(self):
        """Run camera detection once if a camera setup was remembered.

        Gated on the calibration store having at least one role→identity
        assignment: only then is there a setup to restore (auto-assign uses
        those mappings to re-select each slot's source and re-apply its stored
        µm/px). A first-ever run with nothing remembered still waits for a
        manual Detect — we don't probe cameras unprompted. Detection is
        deferred via QTimer so the page paints before the (blocking) probe.
        """
        if self._auto_detect_done:
            return
        mgr = getattr(self, "_camera_manager", None)
        if mgr is None:
            return
        try:
            from SupportClasses.CameraCalibrationStore import get_store
            store = get_store()
            if not store.all_assignments() and not store.any_autostart():
                return  # nothing remembered yet — wait for a manual Detect
        except Exception as exc:
            logger.debug(f"auto-detect gate check failed: {exc}")
            return
        self._auto_detect_done = True  # set before scheduling to avoid re-entry
        logger.info(
            "Hardware Setup: remembered camera setup found — auto-loading "
            "to restore source assignments + calibrations and start the "
            "cameras that were running when the setup was saved")
        # Detect + restore every section, then start the saved (autostart)
        # cameras — the startup reload of the last-used camera setup.
        QTimer.singleShot(0, self._auto_load_cameras)

    # ── Live camera Start/Stop + preview (v7.5.x) ─────────────────

    def _ensure_camera_previews(self):
        """Lazily create one CameraFeedView per assignment slot.

        Uses CameraFeedView (which subscribes to the camera's frame_captured
        signal) rather than reparenting the shared CameraWidget, so the same
        camera can also be previewed elsewhere (e.g. the Calibration page's
        needle-location feeds) without conflict.
        """
        mgr = getattr(self, "_camera_manager", None)
        if mgr is None or not hasattr(self, "_live_cam_preview_holders"):
            return
        try:
            from gui.widgets.camera_feed_view import CameraFeedView
        except Exception as exc:
            logger.debug(f"CameraFeedView unavailable — previews disabled: {exc}")
            return
        for i, holder in enumerate(self._live_cam_preview_holders):
            if i < len(self._live_cam_previews) and self._live_cam_previews[i] is not None:
                continue
            try:
                fv = CameraFeedView(
                    camera_manager=mgr,
                    cam_idx=i,
                    show_crosshair=True,
                    label=f"Cam {i + 1} — live",
                    parent=holder,
                )
                holder.layout().addWidget(fv, stretch=1)
                self._live_cam_previews[i] = fv
                # v7.5.x: image-correction sliders beneath the live preview.
                if i < len(self._live_cam_correction) and self._live_cam_correction[i] is None:
                    strip = self._build_correction_strip(i)
                    holder.layout().addWidget(strip)
            except Exception as exc:
                logger.warning(f"failed to build camera preview {i}: {exc}")

    def _build_correction_strip(self, cam_idx: int) -> QWidget:
        """Build the per-slot brightness/contrast/gamma control strip.

        Each slider pushes live to the shared CameraManager (display-only
        correction) and, on release, persists the value keyed by the camera's
        device identity so it auto-restores next session.
        """
        from gui.styles import build_glass_panel_style
        strip = QFrame()
        strip.setObjectName("camMiniCard")
        strip.setStyleSheet(build_glass_panel_style("camMiniCard"))
        grid = QGridLayout(strip)
        grid.setContentsMargins(s(10), s(8), s(10), s(8))
        grid.setHorizontalSpacing(s(10))
        grid.setVerticalSpacing(s(6))
        grid.setColumnStretch(1, 1)

        title = QLabel("Image Correction")
        title.setStyleSheet(
            f"color: {COLORS.get('subtext0', '#a6adc8')}; "
            f"font-size: {sf(9)}pt; font-weight: 700;")
        grid.addWidget(title, 0, 0, 1, 3)

        refs: dict = {}

        def _add(row: int, name: str, key: str, lo: int, hi: int,
                 init: int, fmt) -> None:
            grid.addWidget(self._field_label(name), row, 0)
            sld = QSlider(Qt.Horizontal)
            sld.setRange(lo, hi)
            sld.setValue(init)
            grid.addWidget(sld, row, 1)
            val = QLabel(fmt(init))
            val.setMinimumWidth(s(34))
            val.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            grid.addWidget(val, row, 2)
            sld.valueChanged.connect(
                lambda v, k=key, l=val, f=fmt: (
                    l.setText(f(v)),
                    self._on_correction_changed(cam_idx, k, v),
                ))
            sld.sliderReleased.connect(
                lambda c=cam_idx: self._persist_correction(c))
            refs[key] = sld
            refs[key + "_lbl"] = val

        _add(1, "Brightness", "brightness", -100, 100, 0, lambda v: str(int(v)))
        _add(2, "Contrast", "contrast", 10, 300, 100,
             lambda v: f"{v / 100.0:.2f}")
        _add(3, "Gamma", "gamma", 10, 300, 100,
             lambda v: f"{v / 100.0:.2f}")

        reset_btn = QPushButton("Reset")
        reset_btn.setMinimumWidth(s(72))
        reset_btn.setToolTip("Reset brightness / contrast / gamma to neutral")
        reset_btn.clicked.connect(lambda _=False, c=cam_idx: self._reset_correction(c))
        grid.addWidget(reset_btn, 4, 2, Qt.AlignRight)

        self._live_cam_correction[cam_idx] = refs
        return strip

    def _on_correction_changed(self, cam_idx: int, key: str, raw: int):
        """Push a live slider value to the CameraManager (display-only)."""
        mgr = getattr(self, "_camera_manager", None)
        if mgr is None:
            return
        if key == "brightness":
            mgr.set_brightness(cam_idx, int(raw))
        elif key == "contrast":
            mgr.set_contrast(cam_idx, raw / 100.0)
        elif key == "gamma":
            mgr.set_gamma(cam_idx, raw / 100.0)

    def _persist_correction(self, cam_idx: int):
        """Save the slot's current correction keyed by device identity."""
        mgr = getattr(self, "_camera_manager", None)
        if mgr is None:
            return
        identity = mgr.camera_identity(cam_idx)
        if identity is None:
            return
        corr = mgr.image_correction(cam_idx)
        try:
            from SupportClasses.CameraCalibrationStore import get_store
            get_store().set_image_correction(
                identity[0],
                brightness=corr.get("brightness", 0),
                contrast=corr.get("contrast", 1.0),
                gamma=corr.get("gamma", 1.0),
                name=identity[1],
            )
        except Exception as exc:
            logger.debug(f"persist correction slot {cam_idx}: {exc}")

    def _reset_correction(self, cam_idx: int):
        """Reset a slot's correction to neutral, sync sliders, and persist."""
        mgr = getattr(self, "_camera_manager", None)
        if mgr is not None:
            mgr.reset_image_correction(cam_idx)
        self._sync_correction_sliders(cam_idx)
        self._persist_correction(cam_idx)

    def _sync_correction_sliders(self, cam_idx: int):
        """Reflect the manager's current correction values on the sliders."""
        mgr = getattr(self, "_camera_manager", None)
        if (mgr is None or cam_idx >= len(self._live_cam_correction)
                or self._live_cam_correction[cam_idx] is None):
            return
        refs = self._live_cam_correction[cam_idx]
        corr = mgr.image_correction(cam_idx)
        vals = {
            "brightness": int(corr.get("brightness", 0)),
            "contrast": int(round(corr.get("contrast", 1.0) * 100)),
            "gamma": int(round(corr.get("gamma", 1.0) * 100)),
        }
        for key, iv in vals.items():
            sld = refs.get(key)
            if sld is not None and sld.value() != iv:
                sld.blockSignals(True)
                sld.setValue(iv)
                sld.blockSignals(False)
            lbl = refs.get(key + "_lbl")
            if lbl is not None:
                lbl.setText(str(iv) if key == "brightness"
                            else f"{iv / 100.0:.2f}")

    def _on_live_cam_source_changed(self, cam_idx: int):
        """Apply the slot's selected source to the CameraManager.

        Restarts the camera if it was already running so the new source
        takes effect, then refreshes the Start button + preview state.
        """
        mgr = getattr(self, "_camera_manager", None)
        if mgr is None or cam_idx >= len(self._live_cam_source_combos):
            return
        data = self._live_cam_source_combos[cam_idx].currentData()
        was_running = mgr.is_running(cam_idx)
        if was_running:
            mgr.stop(cam_idx)
        if data is not None:
            mgr.set_source(cam_idx, data)
            # v7.5.x: the slot now points at a known camera — re-apply its
            # stored µm/px (by device identity) if we have one, and remember
            # which physical camera plays this slot's role so it auto-restores
            # on the next detect.
            self._restore_calibration_for_slot(cam_idx)
            self._remember_assignment(cam_idx)
            if hasattr(self, "_needle_cards"):
                self._refresh_role_derived_displays()
            if was_running:
                mgr.start(cam_idx)
        self._refresh_camera_preview_state()

    def _remember_assignment(self, cam_idx: int):
        """Persist role→device-identity so the source auto-restores next session."""
        mgr = getattr(self, "_camera_manager", None)
        if mgr is None or cam_idx >= len(self._config.camera_roles):
            return
        role = self._config.camera_roles[cam_idx]
        role_val = getattr(role, "value", role)
        if not role_val or role_val == CameraRole.UNASSIGNED.value:
            return
        identity = mgr.camera_identity(cam_idx)
        if identity is None:
            return
        try:
            from SupportClasses.CameraCalibrationStore import get_store
            get_store().set_assignment(role_val, identity[0])
        except Exception as exc:
            logger.debug(f"remember assignment failed: {exc}")

    def _auto_assign_sources_from_store(self):
        """v7.5.x: after detect, re-select each slot's source from the stored
        role→identity assignment so calibrations auto-load without the user
        re-picking which camera is needle_x / needle_y each session."""
        mgr = getattr(self, "_camera_manager", None)
        if mgr is None:
            return
        try:
            from SupportClasses.CameraCalibrationStore import get_store
            from gui.widgets.camera_identity import source_for_identity
            store = get_store()
        except Exception as exc:
            logger.debug(f"auto-assign unavailable: {exc}")
            return
        ds = getattr(mgr, "_ds_cameras", [])
        for i, combo in enumerate(getattr(self, "_live_cam_source_combos", [])):
            if combo.currentData() is not None:
                continue  # already assigned this session
            if i >= len(self._config.camera_roles):
                continue
            role_val = getattr(self._config.camera_roles[i], "value",
                               self._config.camera_roles[i])
            identity = store.get_assignment(role_val)
            if not identity:
                continue
            source = source_for_identity(identity, ds)
            if source is None:
                continue  # that camera isn't present this session
            # NB: QComboBox.findData can't match Python tuples — compare itemData.
            for j in range(combo.count()):
                if combo.itemData(j) == source:
                    combo.setCurrentIndex(j)  # fires _on_live_cam_source_changed
                    break

    def _on_toggle_camera(self, cam_idx: int):
        """Start or stop the camera assigned to this slot."""
        mgr = getattr(self, "_camera_manager", None)
        if mgr is None:
            return
        if mgr.is_running(cam_idx):
            mgr.stop(cam_idx)
            # Implicit "last-used": a deliberate stop means "don't auto-start
            # next launch" for the microscope. App shutdown stops via stop_all
            # (not this handler), so it preserves the flag.
            try:
                if cam_idx == self._config.camera_for_role(CameraRole.MICROSCOPE):
                    identity = mgr.camera_identity(cam_idx)
                    if identity is not None:
                        from SupportClasses.CameraCalibrationStore import get_store
                        get_store().set_autostart(identity[0], False)
            except Exception as exc:
                logger.debug(f"microscope autostart clear failed: {exc}")
        else:
            data = (
                self._live_cam_source_combos[cam_idx].currentData()
                if cam_idx < len(self._live_cam_source_combos) else None
            )
            if data is None:
                QMessageBox.information(
                    self, "Start Camera",
                    "Pick a source for this slot first, then Start.")
                return
            mgr.set_source(cam_idx, data)
            self._restore_calibration_for_slot(cam_idx)
            self._remember_assignment(cam_idx)
            if hasattr(self, "_needle_cards"):
                self._refresh_role_derived_displays()
            mgr.start(cam_idx)
        self._refresh_camera_preview_state()

    def _refresh_camera_preview_state(self):
        """Sync each slot's Start button label/enabled-state and preview
        visibility with the live camera state."""
        mgr = getattr(self, "_camera_manager", None)
        for i, btn in enumerate(getattr(self, "_live_cam_start_btns", [])):
            running = mgr.is_running(i) if mgr is not None else False
            has_source = (
                self._live_cam_source_combos[i].currentData() is not None
                if i < len(self._live_cam_source_combos) else False
            )
            btn.setEnabled(mgr is not None and (running or has_source))
            btn.setText("⏹ Stop" if running else "▶ Start")
            if i < len(self._live_cam_preview_holders):
                self._live_cam_preview_holders[i].setVisible(running)

    def set_controller(self, controller):
        """v7.3.3: Receive StageController for pixel calibration.

        v7.4.0-b: Also propagates to the Stage sub-page panel.
        v7.4.2: Also propagates to the persistent left-side control panel.
        """
        self._controller = controller
        if hasattr(self, '_stage_panel'):
            self._stage_panel.set_controller(controller)
        if hasattr(self, '_xbox_panel'):
            self._xbox_panel.set_controller(controller)
        if hasattr(self, '_control_panel'):
            self._control_panel.set_controller(controller)

    def set_calibrated_um_per_px(self, cam_idx: int, value: float,
                                 rotation_deg: float | None = None):
        """v7.3.3/v7.4.x: Apply a calibrated µm/px from any source.

        Writes through to `CameraManager.set_um_per_px` (the canonical
        live value) and refreshes the role-derived displays so the
        needle card's readout tracks the change. The microscope's
        µm/px is sourced from `ObjectiveCalibrationStore` instead and
        does not flow through here.

        v7.5.x: ``rotation_deg`` (the camera's in-plane lateral stage
        direction, from the stage-motion calibration) is stored alongside
        and fed to the needle-centering aligner. None leaves it untouched
        (e.g. the microscope objective path, which doesn't measure it).
        """
        mgr = getattr(self, "_camera_manager", None)
        if mgr is not None:
            try:
                mgr.set_um_per_px(cam_idx, value)
                if rotation_deg is not None:
                    mgr.set_rotation_deg(cam_idx, rotation_deg)
            except Exception as exc:
                logger.debug(f"set_um_per_px({cam_idx}, {value}) — {exc}")
        # v7.5.x: persist keyed by the camera's stable device identity
        # (name + USB port) in a *per-machine* store — NOT in the hardware
        # config, which is swappable: loading a saved setup file would replace
        # the in-memory config and the auto-save would then wipe the
        # calibration. The store makes the calibration follow the physical
        # camera/port across setup-file loads and restarts.
        identity = mgr.camera_identity(cam_idx) if mgr is not None else None
        if identity is not None:
            key, name = identity
            try:
                from SupportClasses.CameraCalibrationStore import get_store
                get_store().set_calibration(
                    key, float(value), rotation_deg=rotation_deg, name=name)
            except Exception as exc:
                logger.warning(f"camera calibration store write failed: {exc}")
        if hasattr(self, "_needle_cards"):
            self._refresh_role_derived_displays()
        logger.info(
            f"Camera {cam_idx + 1} µm/px set to {value:.4f}"
            f"{f' @ {rotation_deg:.1f}°' if rotation_deg is not None else ''} "
            f"(identity={identity[0] if identity else '?'})"
        )

    def _restore_calibration_for_slot(self, cam_idx: int) -> bool:
        """v7.5.x: if the camera now assigned to ``cam_idx`` has a stored
        µm/px (keyed by its device identity), push it into the live manager
        and mark the slot calibrated. Returns True if a value was restored."""
        mgr = getattr(self, "_camera_manager", None)
        if mgr is None:
            return False
        # Only restore for a slot the user has actually assigned a source to.
        # (After detect, the underlying CameraWidget combos auto-select the
        # first source, so get_source would otherwise resolve an identity for
        # every slot before assignment.)
        if (cam_idx >= len(self._live_cam_source_combos)
                or self._live_cam_source_combos[cam_idx].currentData() is None):
            return False
        identity = mgr.camera_identity(cam_idx)
        if identity is None:
            return False
        try:
            from SupportClasses.CameraCalibrationStore import get_store
            entry = get_store().get_calibration(identity[0])
        except Exception as exc:
            logger.debug(f"camera calibration store read failed: {exc}")
            entry = None
        if not entry:
            return False
        # v7.5.x: restore the display correction (independent of µm/px).
        corr = entry.get("image_correction")
        if isinstance(corr, dict):
            try:
                mgr.set_image_correction(
                    cam_idx,
                    brightness=corr.get("brightness"),
                    contrast=corr.get("contrast"),
                    gamma=corr.get("gamma"),
                )
                self._sync_correction_sliders(cam_idx)
            except Exception as exc:
                logger.debug(f"restore correction slot {cam_idx}: {exc}")
        if entry.get("um_per_px") is None:
            return False
        try:
            mgr.set_um_per_px(cam_idx, float(entry["um_per_px"]))
            rot = entry.get("rotation_deg")
            if rot is not None:
                mgr.set_rotation_deg(cam_idx, float(rot))
            return True
        except Exception as exc:
            logger.debug(f"restore calibration slot {cam_idx}: {exc}")
            return False

    def _restore_all_calibrations(self):
        """Re-apply stored µm/px to every slot whose assigned camera matches
        a stored identity (called after detect / source assignment)."""
        mgr = getattr(self, "_camera_manager", None)
        if mgr is None:
            return
        for i in range(len(getattr(self, "_live_cam_source_combos", []))):
            self._restore_calibration_for_slot(i)

    def _on_camera_started_hw(self, cam_idx: int):
        """On camera start: restore persisted hardware controls + log readback.

        The readback log is the operator's confirmation that the live settings
        come from the camera (not a software default). Runs for any start,
        including from other pages, since it's wired to the manager signal.
        """
        mgr = getattr(self, "_camera_manager", None)
        if mgr is None:
            return
        identity = None
        hw = None
        try:
            identity = mgr.camera_identity(cam_idx)
        except Exception:
            identity = None
        if identity is not None:
            try:
                from SupportClasses.CameraCalibrationStore import get_store
                hw = get_store().get_hw_controls(identity[0])
            except Exception as exc:
                logger.debug(f"hw controls read failed: {exc}")
                hw = None
            if hw:
                self._apply_hw_controls(cam_idx, hw)
        # v7.5.x: microscope "apply the last used camera automatically".
        # (1) Bring it up at its last-used resolution even without an explicit
        # "Save Camera Settings": when no per-camera hw_controls resolution was
        # restored above, apply the persisted spec-combo resolution
        # (camera_config.active_resolution, saved with the normal hardware
        # config). hw_controls.resolution, when present, already set it above
        # and takes precedence. (2) Flag the microscope to auto-start on the
        # next launch — implicit "last-used" tracking. Cleared only on a
        # deliberate user stop (see _on_toggle_camera); app shutdown stops via
        # stop_all, which does NOT route there, so the flag survives.
        try:
            if cam_idx == self._config.camera_for_role(CameraRole.MICROSCOPE):
                if not (hw and hw.get("resolution")):
                    self._maybe_apply_resolution_to_device()
                if identity is not None:
                    from SupportClasses.CameraCalibrationStore import get_store
                    get_store().set_autostart(identity[0], True)
        except Exception as exc:
            logger.debug(f"microscope auto-apply/autostart failed: {exc}")
        # Always log the resulting settings, read back from the device.
        try:
            mgr.log_hw_settings(cam_idx, prefix="[camera start] ")
        except Exception as exc:
            logger.debug(f"log_hw_settings failed: {exc}")

    def _apply_hw_controls(self, cam_idx: int, hw: dict):
        """Push a persisted hardware-control set onto a running camera."""
        mgr = getattr(self, "_camera_manager", None)
        if mgr is None or not isinstance(hw, dict):
            return
        auto = hw.get("auto_exposure")
        # Only drive auto-exposure when we have a CLEAN boolean. OpenCV/DShow
        # cameras often report a raw CAP_PROP value (e.g. -1.0) or nothing for
        # auto-exposure; coercing that via bool() would wrongly force auto ON,
        # so ignore non-bool values rather than guess.
        if isinstance(auto, bool):
            mgr.set_hw_auto_exposure(cam_idx, auto)
        res = hw.get("resolution")
        if res:
            try:
                mgr.set_capture_resolution(cam_idx, int(res[0]), int(res[1]))
            except Exception as exc:
                logger.debug(f"restore resolution failed: {exc}")
        # Restore manual exposure/gain UNLESS auto-exposure is explicitly on.
        # (When auto is unknown/None — e.g. an OpenCV cam — we still restore the
        # saved exposure so "reload exactly" holds; there's no auto state to
        # clobber. Previously `if not auto:` skipped restore only when auto was
        # truthy, which was correct, but `auto is not True` is clearer + robust
        # to the non-bool values now filtered above.)
        if auto is not True:
            if hw.get("exposure_us") is not None:
                mgr.set_hw_exposure_us(cam_idx, hw["exposure_us"])
            if hw.get("exposure_gain_pct") is not None:
                mgr.set_hw_exposure_gain(cam_idx, hw["exposure_gain_pct"])
        if hw.get("gamma") is not None:
            mgr.set_hw_gamma(cam_idx, hw["gamma"])
        if hw.get("brightness") is not None:
            mgr.set_hw_brightness(cam_idx, hw["brightness"])
        if hw.get("contrast") is not None:
            mgr.set_hw_contrast(cam_idx, hw["contrast"])

    def _on_calibrate_needle(self, role: CameraRole):
        """Launch the stage-motion µm/px calibration for a needle camera.

        The slot is resolved from the active role assignment so the
        Needle X and Needle Y buttons each target their own camera
        independently.
        """
        from gui.dialogs.pixel_calibration_dialog import PixelCalibrationDialog
        from PySide6.QtWidgets import QDialog, QMessageBox

        mgr = getattr(self, "_camera_manager", None)
        ctrl = getattr(self, "_controller", None)
        if mgr is None:
            QMessageBox.warning(
                self, "Cannot Calibrate", "Camera manager not available."
            )
            return
        if ctrl is None:
            QMessageBox.warning(
                self, "Cannot Calibrate", "Stage controller not connected."
            )
            return

        cam_idx = self._config.camera_for_role(role)
        if cam_idx is None:
            QMessageBox.warning(
                self, "Cannot Calibrate",
                f"No camera is assigned the {role.value} role. Set the "
                "role in the Camera Detection & Assignment section.",
            )
            return
        try:
            cam = mgr.cameras[cam_idx]
        except (AttributeError, IndexError):
            return
        if not getattr(cam, "_running", False):
            QMessageBox.warning(
                self, "Cannot Calibrate",
                f"Start Cam {cam_idx + 1} (the {role.value} camera) "
                "before launching the calibration.",
            )
            return

        dlg = PixelCalibrationDialog(mgr, ctrl, cam_idx=cam_idx, parent=self)
        if dlg.exec() == QDialog.DialogCode.Accepted:
            result = dlg.result_um_per_px
            if result is not None:
                # v7.5.x: also store the measured in-plane rotation (the move
                # direction that produced clean lateral motion) for the aligner.
                self.set_calibrated_um_per_px(
                    cam_idx, result, rotation_deg=dlg.result_rotation_deg)
                logger.info(
                    f"Needle calibration applied: Cam {cam_idx + 1} "
                    f"({role.value}) = {result:.4f} µm/px"
                )

    def _on_detect_live_cameras(self, opencv_indices=None):
        """Detect live cameras via CameraManager and populate source combos.

        ``opencv_indices`` (a list) may carry a pre-probed OpenCV index list
        from the background startup probe, so the slow device opens aren't
        repeated on the GUI thread. QPushButton.clicked emits a bool, so any
        non-list value is treated as "no pre-probe" → synchronous detect.
        """
        if not isinstance(opencv_indices, list):
            opencv_indices = None
        mgr = getattr(self, '_camera_manager', None)
        if mgr is None:
            logger.warning("No CameraManager set on HardwareSetupPage")
            return

        self._btn_detect_live_cams.setEnabled(False)
        self._btn_detect_live_cams.setText("Detecting...")

        mgr.detect_cameras(opencv_indices)
        sources = mgr.available_sources
        num = len(sources)

        if num == 0:
            self._lbl_live_cam_count.set_status("warn", "No sources found")
        elif num == 1:
            self._lbl_live_cam_count.set_status("ok", "1 source found")
        else:
            self._lbl_live_cam_count.set_status("ok", f"{num} sources found")

        # Populate each per-camera source combo
        for combo in self._live_cam_source_combos:
            prev = combo.currentData()
            combo.blockSignals(True)
            combo.clear()
            combo.addItem("— None —", None)
            for text, data in sources:
                combo.addItem(text, data)
            if prev:
                idx = combo.findData(prev)
                if idx >= 0:
                    combo.setCurrentIndex(idx)
            combo.blockSignals(False)

        self._btn_detect_live_cams.setText("Detect Cameras")
        self._btn_detect_live_cams.setEnabled(True)
        # v7.5.x: device identities are known now. First auto-assign each
        # slot's source from the remembered role→identity map (fires the
        # source-changed handler, which restores that camera's stored µm/px);
        # then restore any already-assigned slots + refresh cards/buttons.
        self._auto_assign_sources_from_store()
        self._restore_all_calibrations()
        if hasattr(self, "_needle_cards"):
            self._refresh_role_derived_displays()
        self._refresh_camera_preview_state()
        logger.info(f"Hardware Setup: detected {num} live camera sources")

    # ── Save / Load the whole camera setup (v7.5.x) ───────────────

    def _set_cam_setup_status(self, variant: str, text: str) -> None:
        lbl = getattr(self, "_lbl_cam_setup_status", None)
        if lbl is None:
            return
        lbl.set_status(variant, text)
        lbl.setVisible(True)

    def _on_save_camera_settings(self):
        """Snapshot the current camera setup as this machine's default.

        For each slot with an assigned source: re-affirm its role→identity
        assignment, persist its image correction and (while running) its
        camera-side hardware controls, and flag whether it is currently running
        so Load / startup can bring the cameras back exactly as they are now.
        µm/px + rotation already persist when calibrated, so they are left as-is.
        """
        mgr = getattr(self, "_camera_manager", None)
        if mgr is None:
            return
        try:
            from SupportClasses.CameraCalibrationStore import get_store
            store = get_store()
        except Exception as exc:
            logger.warning(f"camera store unavailable: {exc}")
            self._set_cam_setup_status("warn", "Save failed (no store)")
            return
        saved = 0
        for i, combo in enumerate(getattr(self, "_live_cam_source_combos", [])):
            if combo.currentData() is None:
                continue
            identity = mgr.camera_identity(i)
            if identity is None:
                continue
            key, name = identity
            self._remember_assignment(i)          # role → identity
            try:
                self._persist_correction(i)        # brightness/contrast/gamma
            except Exception as exc:
                logger.debug(f"save correction slot {i}: {exc}")
            running = mgr.is_running(i)
            if running:                            # hw controls only read live
                try:
                    st = mgr.get_hw_settings(i)
                    if st.get("source") in ("toupcam", "opencv"):
                        store.set_hw_controls(key, {
                            "auto_exposure": st.get("auto_exposure"),
                            "exposure_us": st.get("exposure_us"),
                            "exposure_gain_pct": st.get("exposure_gain_pct"),
                            "gamma": st.get("gamma"),
                            "brightness": st.get("brightness"),
                            "contrast": st.get("contrast"),
                            "resolution": (list(st["resolution"])
                                           if st.get("resolution") else None),
                        }, name=name)
                except Exception as exc:
                    logger.debug(f"save hw controls slot {i}: {exc}")
            store.set_autostart(key, running)
            saved += 1
        self._set_cam_setup_status(
            "ok" if saved else "warn",
            f"Saved {saved} camera(s) as default" if saved
            else "No assigned cameras to save")
        logger.info(f"Hardware Setup: saved camera setup for {saved} slot(s)")

    def _on_load_cameras(self):
        """Detect + restore the saved setup for every section, then start the
        cameras that were running when the setup was last saved."""
        self._on_detect_live_cameras()
        n = self._start_saved_cameras()
        self._set_cam_setup_status(
            "ok", f"Loaded saved setup — started {n} camera(s)")

    def _auto_load_cameras(self):
        """Startup path: probe cameras in the BACKGROUND so the GUI loads and
        stays responsive, then finish on the GUI thread (restore every section
        + start the saved cameras). The slow OpenCV device opens run off the
        GUI thread; the result returns via the queued ``_cameras_probed`` signal
        → ``_finish_auto_load``."""
        mgr = getattr(self, "_camera_manager", None)
        if mgr is None:
            return
        if hasattr(mgr, "detect_cameras_async"):
            mgr.detect_cameras_async(self._cameras_probed.emit)
        else:  # older manager — synchronous fallback
            self._on_detect_live_cameras()
            self._start_saved_cameras()

    def _finish_auto_load(self, opencv_indices):
        """GUI-thread continuation of the background startup probe: populate
        combos from the pre-probed inventory (no re-probe), restore every
        section, then start the cameras flagged autostart."""
        self._on_detect_live_cameras(opencv_indices)
        self._start_saved_cameras()

    def _start_saved_cameras(self) -> int:
        """Start every assigned-but-stopped slot whose camera was flagged
        autostart in the store. Returns the number started."""
        mgr = getattr(self, "_camera_manager", None)
        if mgr is None:
            return 0
        try:
            from SupportClasses.CameraCalibrationStore import get_store
            store = get_store()
        except Exception as exc:
            logger.debug(f"start-saved-cameras: store unavailable: {exc}")
            return 0
        started = 0
        for i, combo in enumerate(getattr(self, "_live_cam_source_combos", [])):
            data = combo.currentData()
            if data is None or mgr.is_running(i):
                continue
            # Sync the slot's source FIRST, then resolve identity: camera_identity
            # reads the manager's selected source, which can lag the UI combo
            # (the detect "restore-prev" branch sets the combo with signals
            # blocked, so set_source was never fired). Resolving before the sync
            # would check the autostart flag against a stale identity.
            try:
                mgr.set_source(i, data)
            except Exception as exc:
                logger.debug(f"auto-start set_source slot {i} failed: {exc}")
                continue
            identity = mgr.camera_identity(i)
            if identity is None or not store.get_autostart(identity[0]):
                continue
            try:
                self._restore_calibration_for_slot(i)
                self._remember_assignment(i)
                mgr.start(i)
                started += 1
            except Exception as exc:
                logger.debug(f"auto-start camera slot {i} failed: {exc}")
        if started and hasattr(self, "_needle_cards"):
            self._refresh_role_derived_displays()
        self._refresh_camera_preview_state()
        if started:
            logger.info(f"Hardware Setup: started {started} saved camera(s)")
        return started

    def _on_live_cam_role_changed(self, cam_idx: int):
        """v7.4.x: Persist the workflow role assigned to this camera.

        Writes through to `self._config.camera_roles[cam_idx]` (with
        singleton enforcement clearing the role from any other slot),
        then refreshes every section that mirrors role state and
        fires the standard config-changed pipeline.
        """
        if cam_idx >= len(self._live_cam_role_combos):
            return
        combo = self._live_cam_role_combos[cam_idx]
        role = combo.currentData() or CameraRole.UNASSIGNED
        try:
            self._config.set_camera_role(cam_idx, role)
        except Exception as e:
            logger.warning(f"set_camera_role({cam_idx}, {role}) failed: {e}")
        # Singleton enforcement may have cleared another slot — rehydrate
        # every role combo from the (now-canonical) config state.
        for i, other in enumerate(self._live_cam_role_combos):
            target = self._config.camera_roles[i] if i < len(
                self._config.camera_roles) else CameraRole.UNASSIGNED
            tidx = other.findData(target)
            if tidx >= 0 and other.currentIndex() != tidx:
                other.blockSignals(True)
                other.setCurrentIndex(tidx)
                other.blockSignals(False)

        # Skip the rebuild + label refresh churn while a saved config
        # is being applied to the UI; the role list is already correct.
        if not getattr(self, '_restoring', False):
            # v7.5.x: a slot that already has a source assigned just gained
            # (or changed) its role — remember role→identity so the source
            # auto-restores next session, and restore that camera's stored cal.
            self._restore_calibration_for_slot(cam_idx)
            self._remember_assignment(cam_idx)
            self._refresh_role_derived_displays()
            if hasattr(self, "_objective_cal_card"):
                self._objective_cal_card.apply_config(self._config)
            self._on_config_changed()

    def _update_camera_info_labels(self):
        """Update computed pixel scale and FOV labels."""
        cam_name = self.camera_combo.currentData()
        spec = self._camera_catalog.get(cam_name) if cam_name else None
        # v7.4.x: the installed objective (chosen by the user in
        # Section C) drives the spec-computed µm/px. Falls back to the
        # CameraConfig field for legacy configs without a selection.
        from SupportClasses.ObjectiveCalibration import get_store
        store = get_store()
        current_obj = getattr(
            self._config.camera_config, "current_objective_name", None
        )
        nominal = store.nominal_magnification(current_obj) if current_obj else None
        mag = (
            nominal
            if nominal is not None
            else (self._config.camera_config.objective_magnification or 2.0)
        )

        res_data = self.cam_resolution_combo.currentData()
        active_res = tuple(res_data) if res_data else (916, 686)

        if spec is not None:
            effective = spec.effective_pixel_size_um(active_res)
            computed = effective / mag
            self.cam_scale_label.setText(
                f"{computed:.2f} µm/px  (sensor: {spec.sensor_pixel_size_um} µm, "
                f"bin: {spec.max_resolution[0] // max(active_res[0], 1)}×, "
                f"mag: {mag}×)")

            # Determine active scale
            if self.cam_override_check.isChecked():
                scale = self.cam_override_spin.value()
            else:
                scale = computed

            fov_w = active_res[0] * scale
            fov_h = active_res[1] * scale
            self.cam_fov_label.setText(
                f"{fov_w:.0f} × {fov_h:.0f} µm  "
                f"({fov_w / 1000:.2f} × {fov_h / 1000:.2f} mm)")
        else:
            self.cam_scale_label.setText("—")
            self.cam_fov_label.setText("—")

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
            label.setMinimumWidth(s(80))
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
                f"color: {COLORS.get('red', '#f38ba8')}; ")
        elif not all_assigned:
            self.channel_map_status.setText(
                f"⚠ {num_channels - len(assigned_pumps)} channel(s) unassigned")
            self.channel_map_status.setStyleSheet(
                f"color: {COLORS.get('yellow', '#f9e2af')}; ")
        else:
            self.channel_map_status.setText("✓ All channels assigned")
            self.channel_map_status.setStyleSheet(
                f"color: {COLORS.get('green', '#a6e3a1')}; ")

    def _get_enabled_pump_ids(self) -> list[str]:
        """Get list of currently enabled pump IDs from widgets."""
        return [
            pid for pid, pw in self._pump_widgets.items()
            if pw.enable_check.isChecked()
        ]

    # ════════════════════════════════════════════════════════════════
    #  CONFIG CHANGE / REBUILD
    # ════════════════════════════════════════════════════════════════

    def _sync_validity_display(self) -> None:
        """Sync all validity labels to current config state."""
        valid, issues = self._config.validate() if hasattr(self._config, "validate") else (True, [])

        if valid:
            full_text = "✓ Setup complete"
            color = COLORS.get('green', '#a6e3a1')
        else:
            lines = [f"⚠ {issue}" for issue in issues] or ["⚠ Setup incomplete"]
            full_text = "\n".join(lines)
            color = COLORS.get('yellow', '#f9e2af')

        # Main page label — show all issues
        lbl = getattr(self, "validity_label", None)
        if lbl is not None:
            lbl.setText(full_text)
            lbl.setStyleSheet(f"color: {color};")

        # Context panel label — show first issue only (compact)
        ctx_lbl = getattr(self, "_ctx_validity_label", None)
        if ctx_lbl is not None:
            ctx_text = "✓ Setup complete" if valid else (
                f"⚠ {issues[0]}" if issues else "⚠ Setup incomplete")
            ctx_lbl.setText(ctx_text)
            ctx_lbl.setStyleSheet(f"color: {color}; ")

    def _on_config_changed(self):
        """Called whenever any config widget changes."""
        if getattr(self, '_restoring', False):
            return
        self._rebuild_config()
        valid = self._config.is_valid
        if valid != self._last_valid:
            self._last_valid = valid
            self.config_validated.emit(valid)

        self._sync_validity_display()
        # v7.5.x: a plate-format/custom-plate change must reload the reagent
        # locations well view (guarded — no-op unless the key changed).
        self._sync_loc_plate_on_config_change()
        self.config_changed.emit(self._config)

    def _rebuild_config(self):
        """Rebuild HardwareConfig from all widget states."""
        # Name & notes — name_edit is a plain QLineEdit (v7.4.2 rev2).
        self._config.config_name = (
            self.name_edit.text().strip() or "Untitled Setup")
        self._config.notes = self.notes_edit.text().strip()

        # Well plate (v7.4.5: designer widget owns both fields)
        if hasattr(self, "_plate_designer"):
            self._config.plate_format = (
                self._plate_designer.current_plate_format())
            self._config.plate_name = (
                self._plate_designer.current_plate_name())
            # Keep the shim combo in sync for legacy readers.
            idx = self.plate_combo.findData(self._config.plate_format)
            if idx >= 0:
                self.plate_combo.blockSignals(True)
                self.plate_combo.setCurrentIndex(idx)
                self.plate_combo.blockSignals(False)
        else:
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

        # Pumps — resolve inks from library
        for pid, pw in self._pump_widgets.items():
            pcfg = pw.get_config()
            resolved_inks = []
            for ink in pcfg.inks:
                if ink.name in self._config.ink_library:
                    resolved_inks.append(self._config.ink_library[ink.name])
                else:
                    logger.warning(f"{pid}: Ink '{ink.name}' no longer in library")
            pcfg.inks = resolved_inks
            self._config.pumps[pid] = pcfg

        # v7.5.x: global pump timing (settle + prime)
        if hasattr(self, "_pump_settle_spin"):
            self._config.pump_settle_time_s = float(
                self._pump_settle_spin.value())
        if hasattr(self, "_pump_prime_spin"):
            self._config.pump_prime_time_s = float(
                self._pump_prime_spin.value())

        # v7.2.4 S3.12: Capture channel map state
        self._config.needle_channel_pump_map.clear()
        for ch_idx, (label, combo) in enumerate(self._channel_map_widgets):
            pid = combo.currentData()
            if pid:
                self._config.needle_channel_pump_map[ch_idx] = pid

        # v7.2.9: Ink swap strategy UI moved to Print Setup → Plan of Action
        # Keep existing ink_swap_strategy in config unchanged

        # v7.3.0: Camera config
        cam_name = self.camera_combo.currentData()
        cam_spec = self._camera_catalog.get(cam_name) if cam_name else None
        res_data = self.cam_resolution_combo.currentData()
        active_res = tuple(res_data) if res_data else (916, 686)
        override = (self.cam_override_spin.value()
                     if self.cam_override_check.isChecked() else None)
        # v7.4.x: preserve the per-machine objective selection through
        # the rebuild (the Objective Calibration card writes it).
        prev_cam_cfg = self._config.camera_config
        current_objective = getattr(prev_cam_cfg, "current_objective_name", None)
        # Derive nominal magnification from the user's objectives
        # library (no prepopulated combo). Fall back to the previous
        # value when the user hasn't picked an objective yet.
        from SupportClasses.ObjectiveCalibration import get_store
        _store = get_store()
        nominal = (
            _store.nominal_magnification(current_objective)
            if current_objective else None
        )
        objective_mag = (
            nominal
            if nominal is not None
            else (prev_cam_cfg.objective_magnification or 2.0)
        )
        self._config.camera_config = CameraConfig(
            camera_spec=cam_spec,
            objective_magnification=objective_mag,
            active_resolution=active_res,
            micron_per_pixel_override=override,
            camera_to_needle_offset_um=prev_cam_cfg.camera_to_needle_offset_um,
            current_objective_name=current_objective,
        )

        # v7.4.x rev2: Per-camera workflow roles. All roles (incl.
        # MICROSCOPE) are now selectable via the per-slot combo in
        # the Camera Detection & Assignment section, so we can rewrite
        # them all from combo state without any preservation logic.
        for i, combo in enumerate(self._live_cam_role_combos):
            role = combo.currentData() or CameraRole.UNASSIGNED
            self._config.set_camera_role(i, role)

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
                self._refresh_reagent_locations()
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
                    # v7.5.x: carry the reagent's pinned wells to the new name
                    self._config.rename_ink_location(name, new_ink.name)
                self._config.add_ink(new_ink)
                self._refresh_ink_table()
                self._refresh_pump_ink_combos()
                self._refresh_reagent_locations()
                self._on_config_changed()

    def _remove_ink(self):
        row = self.ink_table.currentRow()
        if row < 0:
            return
        name = self.ink_table.item(row, 0).text()
        self._config.remove_ink(name)
        # v7.5.x: drop this reagent's pinned wells too
        self._config.clear_ink_location(name)
        self._refresh_ink_table()
        self._refresh_pump_ink_combos()
        self._refresh_reagent_locations()
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
                (ink.ink_subtype or "—") if ink.ink_type == "ink" else "—"))
            self.ink_table.setItem(row, 3, QTableWidgetItem(
                f"{ink.viscosity_cP:.1f} cP"))
            self.ink_table.setItem(row, 4, QTableWidgetItem(
                f"{ink.granule_diameter_um:.0f} µm" if ink.granule_diameter_um else "—"))
            self.ink_table.setItem(row, 5, QTableWidgetItem(
                f"{ink.cell_diameter_um:.0f} µm" if ink.cell_diameter_um else "—"))

    def _refresh_pump_ink_combos(self):
        """Refresh ink names in all pump combos with exclusion support."""
        self._refresh_pump_ink_exclusions()

    # ════════════════════════════════════════════════════════════════
    #  v7.5.x: REAGENT LOCATIONS (ink → wells)
    # ════════════════════════════════════════════════════════════════

    def _build_reagent_locations_group(self):
        """Build the Reagent Locations group on the Ink sub-page.

        Reuses the print workflow's :class:`WellPlateView` so the operator
        picks wells (incl. rosette sub-wells like ``A1.a``) for each reagent
        exactly as in Print Setup. Pinned wells persist in
        ``HardwareConfig.ink_locations`` and auto-fill the per-print Well
        Setup (see ``seed_assignments_from_ink_locations``).
        """
        from gui.widgets.well_plate_view import WellPlateView

        self._loc_plate_key = None  # track the currently-loaded plate key
        self._loc_plate = None      # the loaded WellPlate (for well lookups)

        loc_group = QGroupBox("Reagent Locations")
        loc_group.setStyleSheet(self._group_style())
        loc_lay = QVBoxLayout(loc_group)

        loc_help = QLabel(
            "Pin each reagent (ink / wash / buffer / oil…) to one or more "
            "plate wells. Rosette sub-wells (e.g. A1.a) appear automatically "
            "once the plate is built on the Plate / Rosette pages. These "
            "locations auto-fill the per-print Well Setup and are reused "
            "across workflows.")
        loc_help.setWordWrap(True)
        loc_help.setStyleSheet(f"color: {COLORS.get('subtext0', '#a6adc8')};")
        loc_lay.addWidget(loc_help)

        sel_row = QHBoxLayout()
        sel_row.setSpacing(s(8))
        sel_row.addWidget(QLabel("Reagent:"))
        self._loc_ink_combo = QComboBox()
        self._loc_ink_combo.setMinimumWidth(s(200))
        self._loc_ink_combo.currentIndexChanged.connect(self._on_loc_ink_changed)
        sel_row.addWidget(self._loc_ink_combo)
        sel_row.addStretch()
        loc_lay.addLayout(sel_row)

        self._loc_plate_view = WellPlateView()
        self._loc_plate_view.setMinimumHeight(s(240))
        self._loc_plate_view.setMaximumHeight(s(380))
        self._loc_plate_view.selection_changed.connect(
            self._on_loc_selection_changed)
        loc_lay.addWidget(self._loc_plate_view)

        loc_btns = QHBoxLayout()
        loc_btns.setSpacing(s(8))
        self._loc_btn_assign = icon_button(
            "Assign selected", "plus", object_name="accentBtn")
        self._loc_btn_assign.clicked.connect(self._loc_assign_selected)
        loc_btns.addWidget(self._loc_btn_assign)
        btn_clear_sel = icon_button("Clear selected", "minus")
        btn_clear_sel.clicked.connect(self._loc_clear_selected)
        loc_btns.addWidget(btn_clear_sel)
        btn_clear_ink = icon_button(
            "Clear reagent", "trash", object_name="dangerBtn")
        btn_clear_ink.clicked.connect(self._loc_clear_ink)
        loc_btns.addWidget(btn_clear_ink)
        loc_btns.addStretch()
        loc_lay.addLayout(loc_btns)

        self._loc_summary = QLabel("")
        self._loc_summary.setWordWrap(True)
        self._loc_summary.setStyleSheet(
            f"color: {COLORS.get('subtext0', '#a6adc8')};")
        loc_lay.addWidget(self._loc_summary)

        self._sub_layouts["inks"].addWidget(loc_group)

        # Seed initial state (default config → empty, but keeps widgets valid).
        self._refresh_reagent_locations()

    @staticmethod
    def _reagent_type_label(ink) -> str:
        """"(ink · cells)" / "(wash)" — well type plus subtype for inks."""
        if ink is None:
            return ""
        t = (ink.ink_type or "").strip()
        sub = (getattr(ink, "ink_subtype", "") or "").strip()
        return f"({t} · {sub})" if t == "ink" and sub else f"({t})"

    def _loc_selected_ink_name(self) -> str | None:
        """Currently-selected reagent name in the locations combo, if any."""
        combo = getattr(self, "_loc_ink_combo", None)
        if combo is None or combo.count() == 0:
            return None
        return combo.currentData()

    def _refresh_reagent_locations(self):
        """Full refresh of the Reagent Locations group from the config."""
        if not hasattr(self, "_loc_ink_combo"):
            return
        self._refresh_loc_ink_combo()
        self._refresh_loc_plate()
        self._refresh_loc_colors()
        self._refresh_loc_summary()
        # Re-highlight the selected reagent's wells.
        self._on_loc_ink_changed()

    def _refresh_loc_ink_combo(self):
        """Populate the reagent selector from the ink library."""
        combo = self._loc_ink_combo
        prev = combo.currentData()
        combo.blockSignals(True)
        combo.clear()
        for name, ink in self._config.ink_library.items():
            label = f"{name}  {self._reagent_type_label(ink)}"
            combo.addItem(label, name)
            idx = combo.count() - 1
            try:
                combo.setItemData(idx, QColor(ink.color), Qt.ForegroundRole)
            except Exception:
                pass
        if prev is not None:
            pidx = combo.findData(prev)
            if pidx >= 0:
                combo.setCurrentIndex(pidx)
        combo.blockSignals(False)
        has_inks = combo.count() > 0
        if hasattr(self, "_loc_btn_assign"):
            self._loc_btn_assign.setEnabled(has_inks)

    def _refresh_loc_plate(self, *, force: bool = False) -> bool:
        """(Re)load the active plate into the locations well view.

        Reloads only when the active plate key changed (or ``force``),
        because ``set_plate`` clears the current selection. Returns True
        when a reload actually happened.
        """
        key = self._config.active_plate_key
        if not force and key == self._loc_plate_key:
            return False
        try:
            plate = WellPlate.load(key)
        except Exception as exc:
            logger.warning(f"Reagent-locations plate load failed ({key}): {exc}")
            return False
        self._loc_plate_view.set_plate(plate)
        self._loc_plate = plate
        self._loc_plate_key = key
        self._loc_plate_view.fit_view()
        return True

    def _sync_loc_plate_on_config_change(self):
        """Reload the locations plate if the active plate key changed."""
        if not hasattr(self, "_loc_plate_view"):
            return
        if self._refresh_loc_plate():
            self._refresh_loc_colors()
            self._refresh_loc_summary()
            self._on_loc_ink_changed()

    def _refresh_loc_colors(self):
        """Color each well by its assigned reagent.

        Center fill  = the reagent's own ``InkSpec.color`` (the color set in
        the Ink editor above), so the plate matches the ink swatches.
        Border       = a type color (ink / wash / waste / buffer / oil) via
        :func:`ink_type_border_color`, so the functional type reads at a
        glance. Both are persisted on the well item so they survive a
        hover-leave (see ``WellGraphicsItem.set_appearance``).
        """
        view = getattr(self, "_loc_plate_view", None)
        plate = getattr(self, "_loc_plate", None)
        if view is None or plate is None:
            return
        empty_fill = ROLE_COLORS.get(WellRole.EMPTY, "#585b70")
        reagent_of = self._config.well_reagent_map
        lib = self._config.ink_library
        appearances: dict = {}
        for well in plate.well_names:
            ink_name = reagent_of.get(well)
            ink = lib.get(ink_name) if ink_name else None
            if ink is not None:
                fill = ink.color or empty_fill
                border = ink_type_border_color(ink.ink_type)
                label = f"{ink_name} ({ink.ink_type})"
                appearances[well] = (fill, border, label)
            else:
                # Unassigned (or dangling) well → gray, default border.
                appearances[well] = (empty_fill, None, "")
        view.update_reagent_appearances(appearances)

    def _refresh_loc_summary(self):
        """Human-readable list of current reagent → wells assignments."""
        lbl = getattr(self, "_loc_summary", None)
        if lbl is None:
            return
        locs = self._config.ink_locations
        if not locs:
            lbl.setText("No reagent locations assigned yet.")
            return
        lib = self._config.ink_library
        parts = []
        for name, wells in locs.items():
            if not wells:
                continue
            ink = lib.get(name)
            t = f" {self._reagent_type_label(ink)}" if ink else ""
            parts.append(f"{name}{t}: {', '.join(wells)}")
        lbl.setText("  •  ".join(parts) if parts else
                    "No reagent locations assigned yet.")

    def _on_loc_ink_changed(self, *_):
        """Reagent selection changed — highlight its wells + update button."""
        view = getattr(self, "_loc_plate_view", None)
        ink_name = self._loc_selected_ink_name()
        if hasattr(self, "_loc_btn_assign"):
            self._loc_btn_assign.setText(
                f"Assign selected → {ink_name}" if ink_name
                else "Assign selected")
        if view is None or ink_name is None:
            return
        wells = list(self._config.ink_locations.get(ink_name, []))
        view.blockSignals(True)
        view.set_selection(wells)
        view.blockSignals(False)

    def _on_loc_selection_changed(self, _wells):
        """Selection changed in the well view (currently informational)."""
        # Hook kept for future live-count feedback; no state change needed.
        pass

    def _loc_assign_selected(self):
        """Pin the well-view selection to the selected reagent."""
        view = getattr(self, "_loc_plate_view", None)
        ink_name = self._loc_selected_ink_name()
        if view is None or ink_name is None:
            return
        wells = view.get_selected_wells()
        if not wells:
            QMessageBox.information(
                self, "No wells selected",
                "Select one or more wells on the plate first.")
            return
        self._config.assign_wells_to_ink(ink_name, wells)
        self._refresh_loc_colors()
        self._refresh_loc_summary()
        self._on_config_changed()

    def _loc_clear_selected(self):
        """Unassign whichever reagents currently own the selected wells."""
        view = getattr(self, "_loc_plate_view", None)
        if view is None:
            return
        wells = view.get_selected_wells()
        if not wells:
            return
        for well in wells:
            self._config.unassign_well(well)
        self._refresh_loc_colors()
        self._refresh_loc_summary()
        self._on_config_changed()

    def _loc_clear_ink(self):
        """Clear ALL pinned wells for the selected reagent."""
        ink_name = self._loc_selected_ink_name()
        if ink_name is None:
            return
        self._config.clear_ink_location(ink_name)
        self._refresh_loc_colors()
        self._refresh_loc_summary()
        self._on_loc_ink_changed()  # re-highlight (now empty)
        self._on_config_changed()

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
                # v7.4.2: Refresh the Setup Name combo so the new file
                # appears as a pickable option immediately.
                self._refresh_setup_name_combo()
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
        self._restoring = True  # v7.2.6

        logger.info("Applying config to UI...")

        # ── 1. Name & Notes ──────────────────────────────────────
        # v7.4.2 rev2: name_edit is a QLineEdit again; the picker
        # lives below as a scrollable list with a Load button.
        self.name_edit.blockSignals(True)
        self.name_edit.setText(self._config.config_name)
        self.name_edit.blockSignals(False)

        self.notes_edit.blockSignals(True)
        self.notes_edit.setText(self._config.notes)
        self.notes_edit.blockSignals(False)
        logger.debug(f"  Name: {self._config.config_name}")

        # ── 2. Plate Format / Custom Plate (v7.4.5) ──────────────
        # active_plate_key returns plate_name (str) when set, else
        # plate_format (int). Designer handles both.
        active_key = self._config.active_plate_key
        if hasattr(self, "_plate_designer"):
            self._plate_designer.load_plate(active_key)
        self.plate_combo.blockSignals(True)
        pidx = self.plate_combo.findData(self._config.plate_format)
        if pidx >= 0:
            self.plate_combo.setCurrentIndex(pidx)
        self.plate_combo.blockSignals(False)
        logger.debug(f"  Plate: {active_key}")

        # ── 3. Ink Library (MUST come before pumps) ──────────────
        self._refresh_ink_table()
        # v7.2.5: Force pump ink combo refresh BEFORE pump restore
        self._refresh_pump_ink_combos()
        # v7.5.x: pumps only ever offer printable inks (no service reagents).
        ink_names = self._pump_ink_names()
        logger.debug(f"  Ink library: {len(ink_names)} printable inks — pump combos refreshed")

        # ── 4. Rosette Library ───────────────────────────────────
        self._refresh_rosette_table()
        logger.debug(
            f"  Rosette library: {len(self._config.rosette_library)} rosettes")

        # v7.5.x: reagent locations (force a plate reload — the active plate
        # key may have changed with the loaded config; _on_config_changed is
        # suppressed during restore so refresh here explicitly).
        self._refresh_loc_plate(force=True)
        self._refresh_reagent_locations()

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
                # v7.2.6: Block signals during pump restore
                pw.blockSignals(True)
                pw.set_config(pcfg, ink_names=ink_names)
                pw.blockSignals(False)
                logger.debug(
                    f"  {pid}: enabled={pcfg.enabled}, "
                    f"syringe={pcfg.syringe.volume_uL if pcfg.syringe else None}µL, "
                    f"inks={pcfg.ink_names}, "
                    f"mode={pcfg.printing_mode.value}")

        # v7.2.8: Verify pump ink assignments after restore.
        # v7.5.x: only the PRINTABLE inks are offered to a pump, so compare
        # against that filtered set — a legacy config that assigned a service
        # reagent (now excluded) must not log a spurious mismatch every load
        # (it is intentionally dropped on the next save).
        offered = set(ink_names)
        for pid, pw in self._pump_widgets.items():
            actual = set(pw.get_selected_ink_names())
            expected = (set(self._config.pumps[pid].ink_names) & offered
                        if pid in self._config.pumps else set())
            if expected and actual != expected:
                logger.warning(f"  {pid} ink mismatch: expected={expected}, actual={actual}")
                pw.set_config(self._config.pumps[pid], ink_names=ink_names)

        # Refresh ink lists after all pumps loaded
        self._refresh_pump_ink_exclusions()
        self._update_pump_ink_summary()

        # ── 6b. Global pump timing (v7.5.x) ──────────────────────
        if hasattr(self, "_pump_settle_spin"):
            self._pump_settle_spin.blockSignals(True)
            self._pump_settle_spin.setValue(
                float(getattr(self._config, "pump_settle_time_s", 0.0) or 0.0))
            self._pump_settle_spin.blockSignals(False)
        if hasattr(self, "_pump_prime_spin"):
            self._pump_prime_spin.blockSignals(True)
            self._pump_prime_spin.setValue(
                float(getattr(self._config, "pump_prime_time_s", 0.25) or 0.0))
            self._pump_prime_spin.blockSignals(False)

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
                else:
                    logger.warning(
                        f"  Channel {ch_idx}: pump {mapped_pump!r} "
                        f"not found in combo. Available: "
                        f"{[combo.itemData(i) for i in range(combo.count())]}")
        self._update_channel_map_status()
        logger.debug(
            f"  Channel map: {self._config.needle_channel_pump_map}")

        # v7.2.9: Ink swap strategy UI moved to Print Setup → Plan of Action

        # ── 8. Camera Configuration (v7.3.0) ──────────────────────
        cam_cfg = self._config.camera_config
        self.camera_combo.blockSignals(True)
        if cam_cfg.camera_spec:
            # v7.5.x: the spec combo stores the catalog KEY as item data
            # (addItem(name, name) iterates catalog KEYS, e.g. "BUC3D-1000C"),
            # but cam_cfg.camera_spec.name is the long display name from
            # cameras.json ("Bestscope BUC3D-1000C (ToupTek C3CMOS10000KPA)").
            # findData matches DATA exactly, so searching by the long name never
            # matched → the spec dropdown reset to "None" every launch and the
            # operator had to re-pick it. Resolve the KEY whose spec.name matches
            # the saved spec, with a legacy fallback for configs that saved the
            # key itself.
            _saved_name = cam_cfg.camera_spec.name
            _catalog_key = next(
                (k for k, sp in self._camera_catalog.items()
                 if getattr(sp, "name", None) == _saved_name),
                None,
            )
            if _catalog_key is None and _saved_name in self._camera_catalog:
                _catalog_key = _saved_name
            cidx = (self.camera_combo.findData(_catalog_key)
                    if _catalog_key is not None else -1)
            if cidx >= 0:
                self.camera_combo.setCurrentIndex(cidx)
            # Populate resolution combo for this camera
            self.cam_resolution_combo.blockSignals(True)
            self.cam_resolution_combo.clear()
            for res in cam_cfg.camera_spec.preview_resolutions:
                self.cam_resolution_combo.addItem(
                    f"{res[0]} × {res[1]}", list(res))
            # Select matching resolution
            for ri in range(self.cam_resolution_combo.count()):
                rd = self.cam_resolution_combo.itemData(ri)
                if rd and tuple(rd) == cam_cfg.active_resolution:
                    self.cam_resolution_combo.setCurrentIndex(ri)
                    break
            self.cam_resolution_combo.blockSignals(False)
        else:
            self.camera_combo.setCurrentIndex(0)  # "None"
        self.camera_combo.blockSignals(False)

        # v7.4.x: Installed objective is a read-only mirror of the
        # Objective Calibration Setup card. Populate from the user's
        # objectives library (custom, no prepopulated list).
        self._refresh_installed_objective_combo()

        # v7.4.x rev2: microscope-slot is now expressed via the role
        # combo in the Detection & Assignment section. The dedicated
        # combo is gone; the read-only readout is updated by
        # `_refresh_role_derived_displays` further down in this method
        # (called from the role-combo restore block).

        self.cam_override_check.blockSignals(True)
        has_override = cam_cfg.micron_per_pixel_override is not None
        self.cam_override_check.setChecked(has_override)
        self.cam_override_spin.setEnabled(has_override)
        if has_override:
            self.cam_override_spin.blockSignals(True)
            self.cam_override_spin.setValue(cam_cfg.micron_per_pixel_override)
            self.cam_override_spin.blockSignals(False)
        self.cam_override_check.blockSignals(False)

        self._update_camera_info_labels()
        logger.debug(
            f"  Camera: {cam_cfg.camera_spec.name if cam_cfg.camera_spec else 'None'}, "
            f"mag={cam_cfg.objective_magnification}×, "
            f"scale={cam_cfg.micron_per_pixel}")

        # v7.4.x rev2: Per-camera workflow roles. Role combos now own
        # all five role values including MICROSCOPE.
        for i, combo in enumerate(self._live_cam_role_combos):
            if i >= len(self._config.camera_roles):
                continue
            role = self._config.camera_roles[i]
            ridx = combo.findData(role)
            combo.blockSignals(True)
            combo.setCurrentIndex(ridx if ridx >= 0 else 0)
            combo.blockSignals(False)
        logger.debug(f"  Camera roles: {self._config.camera_roles}")

        # v7.5.x: re-activate stored µm/px for any already-assigned slots
        # before the role-derived refresh so the needle cards show the
        # restored calibration. (Identity-keyed: actual restore happens as
        # each source is assigned via _on_live_cam_source_changed.)
        self._restore_all_calibrations()

        # v7.4.x rev2: Refresh the section labels and per-needle cards
        # that mirror the role assignments.
        self._refresh_role_derived_displays()

        # v7.4.x: Objective Calibration card — refresh microscope
        # assignment, current-objective combo, and status table.
        if hasattr(self, "_objective_cal_card"):
            self._objective_cal_card.apply_config(self._config)

        # ── 9. Emit signals ──────────────────────────────────────
        self._restoring = False
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
        """v7.4.2: Tick the persistent control panel + forward to the
        Stage sub-page so its per-axis position cells refresh too."""
        if hasattr(self, '_control_panel'):
            try:
                self._control_panel.on_status_update()
            except Exception:
                pass
        if hasattr(self, '_stage_panel'):
            try:
                self._stage_panel.on_status_update()
            except Exception:
                pass

    def get_context_widget(self) -> QWidget | None:
        """v7.4.2: Return the HardwareControlPanel as the context widget.

        Previously this returned a "Saved Configurations" file browser.
        Per user request, that browser is gone — config save/load lives
        on the Identity sub-page's Actions row, and the device profile
        picker lives on the Device sub-page. The context panel now
        hosts the always-on Connect / Jog / Live Position controls so
        the user can drive the stages from any Hardware Setup sub-page.
        """
        return self._control_panel

        return ctx

    # ════════════════════════════════════════════════════════════════
    #  CONFIG FILE BROWSER (v7.2.4)
    # ════════════════════════════════════════════════════════════════

    # ── v7.4.2: Setup list (replaces the older combo picker) ─

    def _refresh_setup_list(self) -> None:
        """Re-scan ``config/hardware/`` and populate the Saved Setups list.

        Items map to file paths via ``self._setup_file_paths``. Only
        files that look like saved HardwareConfig JSONs are listed —
        a file is considered a config if it has a ``config_name`` key
        OR a ``pumps`` / ``plate_format`` / ``needle_gauge`` key. This
        filters out the catalog files (cameras.json, needles.json,
        objectives.json, syringes.json) that share the directory.

        The current selection is preserved across refresh when possible.
        """
        if not hasattr(self, '_setup_list'):
            return
        import json
        self._setup_file_paths: dict[str, "Path"] = {}
        previous = None
        current = self._setup_list.currentItem()
        if current is not None:
            previous = current.text()
        self._setup_list.clear()
        config_dir = CONFIG_HARDWARE_DIR
        if config_dir.is_dir():
            for jf in sorted(config_dir.glob("*.json")):
                try:
                    with open(jf, "r") as f:
                        data = json.load(f)
                except Exception as e:
                    logger.debug(f"Skipping malformed config {jf.name}: {e}")
                    continue
                if not isinstance(data, dict):
                    continue
                cfg_name = data.get("config_name")
                looks_like_config = (
                    cfg_name is not None
                    or "pumps" in data
                    or "plate_format" in data
                    or "needle_gauge" in data
                )
                if not looks_like_config:
                    continue
                display = cfg_name or jf.stem
                if cfg_name and cfg_name != jf.stem:
                    display = f"{cfg_name}  —  {jf.stem}"
                base_display = display
                suffix = 2
                while display in self._setup_file_paths:
                    display = f"{base_display} #{suffix}"
                    suffix += 1
                self._setup_list.addItem(display)
                self._setup_file_paths[display] = jf
        # Restore prior selection
        if previous:
            for i in range(self._setup_list.count()):
                item = self._setup_list.item(i)
                if item.text() == previous:
                    self._setup_list.setCurrentRow(i)
                    break
        if hasattr(self, '_lbl_setup_status'):
            n = self._setup_list.count()
            self._lbl_setup_status.setText(
                f"{n} saved setup{'s' if n != 1 else ''} on disk.")

    def _load_selected_setup(self) -> None:
        """Apply the currently-selected setup from the list."""
        if not hasattr(self, '_setup_list'):
            return
        item = self._setup_list.currentItem()
        if item is None:
            if hasattr(self, '_lbl_setup_status'):
                self._lbl_setup_status.setText(
                    "Pick a setup from the list above first.")
            return
        path = getattr(self, "_setup_file_paths", {}).get(item.text())
        if path is None or not path.exists():
            if hasattr(self, '_lbl_setup_status'):
                self._lbl_setup_status.setText(
                    f"File missing: {item.text()}")
            return
        try:
            self._config = HardwareConfig.load(str(path))
        except Exception as e:
            QMessageBox.critical(
                self, "Load Setup", f"Failed to load {path.name}:\n{e}")
            return
        self._apply_config_to_ui()
        if hasattr(self, '_lbl_setup_status'):
            self._lbl_setup_status.setText(f"Loaded: {path.name}")
        logger.info(f"Loaded hardware setup from list: {path.name}")

    # Back-compat shim: a few callers still invoke the old name.
    def _refresh_setup_name_combo(self) -> None:
        self._refresh_setup_list()

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

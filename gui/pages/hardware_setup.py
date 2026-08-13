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
import re
from dataclasses import replace
from pathlib import Path
from functools import partial

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QGroupBox,
    QPushButton, QLabel, QComboBox, QDoubleSpinBox, QSpinBox,
    QLineEdit, QTextEdit, QFrame, QSizePolicy, QMessageBox,
    QFileDialog, QScrollArea, QTableWidget, QTableWidgetItem,
    QHeaderView, QDialog, QFormLayout, QDialogButtonBox,
    QCheckBox, QAbstractItemView, QListWidget, QListWidgetItem,
    QSlider, QInputDialog, QApplication,
)
from PySide6.QtCore import Qt, Signal, QTimer
from PySide6.QtGui import QFont, QColor, QStandardItem

from SupportClasses.HardwareConfig import (
    HardwareConfig, PumpChannelConfig, CameraConfig, CameraRole,
    MAX_LIVE_CAMERAS,
)
from SupportClasses.PhysicalModels import (
    NeedleSpec, NeedleBore, SyringeSpec, InkSpec, PrintingMode, RosetteInsert,
    CameraSpec,
    load_needle_catalog, load_syringe_catalog, load_camera_catalog,
    WellRole, ROLE_COLORS, well_role_for_ink_type, ink_type_border_color,
    WELL_TYPES, INK_SUBTYPES, is_service_reagent,
    NEEDLE_TYPE_HYPODERMIC, NEEDLE_TYPE_CAPILLARY,
    TIP_PROFILE_CYLINDER, TIP_PROFILE_CONE,
    NEEDLE_FORM_SINGLE, NEEDLE_FORM_BACKPACK, NEEDLE_FORM_SEPTUM,
    NEEDLE_FORM_TRIPLE,
    NEEDLE_FORM_BORE_COUNT, needle_form_bores_are_uniform,
)
from SupportClasses.NeedleTypeStore import (
    NeedleType, get_store as get_needle_type_store, safe_id as safe_needle_type_id,
)
from SupportClasses.WellPlate import PLATE_DEFINITIONS, WellPlate
from SupportClasses.CameraRotationTracker import (  # v7.10
    nearest_nominal, nearest_square_rotation, wrap_deg)
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

# ── v7.5.x: per-camera rotation-vs-stage nominals ─────────────────────
# Every camera carries a calibrated in-plane rotation relative to the
# stage axes (CameraManager.get/set_rotation_deg, measured by the
# stage-motion PixelCalibrationDialog). The NOMINAL mount depends on the
# role: the microscope views along Z, so its rotation is in the stage XY
# plane (axis-aligned nominal); the needle side cameras are mounted
# symmetric about the stage +X axis at +45° and −45° (which cam is which
# comes from the measured column_dir_deg, not the role); the monitor
# overview camera rests on the stage (axis-aligned nominal). The nominal/Δ
# shown on the slot cards is a sanity hint ONLY — motion always uses the
# measured angle itself. For needle cams the Δ is computed against the
# MOUNT direction (column_dir_deg); their display rotation_deg is the
# small sensor roll, nominal 0°.
_AXIS_ALIGNED_NOMINALS = (0.0, 90.0, 180.0, -90.0)
_DIAGONAL_NOMINALS = (45.0, -45.0, 135.0, -135.0)


def role_nominal_rotations(role) -> tuple[float, ...]:
    """Nominal mount rotations (deg, vs stage axes) for a camera role."""
    if role in (CameraRole.NEEDLE_X, CameraRole.NEEDLE_Y):
        return _DIAGONAL_NOMINALS
    return _AXIS_ALIGNED_NOMINALS


def nominal_rotation_delta(theta_deg: float, role) -> tuple[float, float]:
    """(nearest_nominal, signed_delta) of a measured rotation vs the
    role's nominal mount set. ``theta = nominal + delta`` (mod 360).

    Delegates to the Qt-free ``CameraRotationTracker.nearest_nominal`` so the
    slot-card readout and the live square-up tool share ONE implementation.
    """
    return nearest_nominal(theta_deg, role_nominal_rotations(role))


def role_rotation_hint(role) -> str:
    """One-line description of the expected mount orientation per role."""
    if role == CameraRole.MICROSCOPE:
        return "views along Z — rotation is in the stage XY plane (nominal 0°)"
    if role in (CameraRole.NEEDLE_X, CameraRole.NEEDLE_Y):
        return ("side view, symmetric about stage +X at ±45° "
                "(nominal mount ±45°, roll ≈ 0°)")
    if role == CameraRole.MONITOR:
        return "overview camera on the stage (nominal axis-aligned)"
    return "rotation vs the stage X/Y axes"



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
# Pulled-capillary geometry spin definitions (v7.9)
# ═══════════════════════════════════════════════════════════════════
# ONE table of (min, max, step, decimals, suffix, default, tooltip) shared by
# the single-needle capillary card AND every per-bore row, so a backpack's
# second bore can never end up with a different range, default or tooltip than
# the first. Ranges are deliberately WIDER than the common band (10–100 µm tips,
# 1–10 mm pulls, 100 mm blanks): a spin range that clips a legitimate outlier
# corrupts the value silently, whereas ``validate()`` can warn.
_CAP_SPIN_SPECS = {
    "barrel_id": (50.0, 3000.0, 10.0, 1, " µm", 1000.0,
                  "Bore of the glass blank before the pull."),
    "barrel_od": (100.0, 4000.0, 10.0, 1, " µm", 1500.0,
                  "Outside of the glass blank — drives the drawn needle width."),
    "barrel_len": (1.0, 200.0, 1.0, 1, " mm", 100.0,
                   "Bulk section only — the total needle length is barrel + tip."),
    "tip_id": (0.5, 500.0, 1.0, 1, " µm", 30.0,
               "The orifice. Sets the deposited feature size and dominates the "
               "flow resistance (flow scales with diameter to the 4th power)."),
    "tip_od": (0.0, 1000.0, 1.0, 1, " µm", 0.0,
               "Leave at 0 if unknown — the drawn needle and clearance views then "
               "fall back to the barrel's OD/ID ratio applied to the tip Ø."),
    "tip_len": (0.1, 50.0, 0.1, 2, " mm", 5.0,
                "Length of the pulled section."),
}

# Tip-profile combo entries, shared for the same anti-divergence reason.
_TIP_PROFILE_ITEMS = (
    ("Straight cylinder", TIP_PROFILE_CYLINDER),
    ("Tapered cone", TIP_PROFILE_CONE),
)
_TIP_PROFILE_TOOLTIP = (
    "Straight cylinder: the tip is a uniform tube of the tip Ø over its "
    "length. Conservative (it over-estimates resistance vs a real "
    "taper, so the flow ceiling errs low — the safe direction for glass).\n"
    "Tapered cone: the tip narrows linearly from the barrel Ø to the tip "
    "Ø, using the exact conical Poiseuille resistance."
)


def _make_cap_spin(key: str, on_change=None) -> QDoubleSpinBox:
    """A capillary geometry spin box built from :data:`_CAP_SPIN_SPECS`."""
    lo, hi, step, decimals, suffix, value, tip = _CAP_SPIN_SPECS[key]
    sb = QDoubleSpinBox()
    sb.setRange(lo, hi)
    sb.setSingleStep(step)
    sb.setDecimals(decimals)
    sb.setSuffix(suffix)
    sb.setValue(value)
    sb.setToolTip(tip)
    if on_change is not None:
        sb.valueChanged.connect(on_change)
    return sb


def _make_tip_profile_combo(on_change=None) -> QComboBox:
    """The tip-profile picker, identical everywhere it appears."""
    combo = QComboBox()
    for text, data in _TIP_PROFILE_ITEMS:
        combo.addItem(text, data)
    combo.setToolTip(_TIP_PROFILE_TOOLTIP)
    if on_change is not None:
        combo.currentIndexChanged.connect(on_change)
    return combo


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

        # v7.9: per-bore geometry rows (one dict of widgets per bore) and the
        # geometry of bores hidden by a form SHRINK. The cache is what makes
        # Triple → Single → Triple non-destructive: the operator's bore 2/3
        # geometry is held here rather than reconstructed from defaults.
        self._bore_rows: list[dict] = []
        self._bore_cache: dict[int, dict] = {}
        # MEASURED mount offsets, remembered per bore index for the session and
        # deliberately kept OUT of `_bore_cache`: they are owned by the Needle
        # Location calibration, not by this page, so neither a geometry edit nor a
        # form round-trip may drop them. Cleared on a config LOAD — the incoming
        # setup's offsets (including their absence) are authoritative.
        self._bore_offsets: dict[int, tuple[tuple[float, float], float]] = {}

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
        # v7.5.x: on Hardware Setup ("hardware calibration") the speed section
        # edits the ABSOLUTE per-axis max speed (the single common ceiling every
        # %-of-max page reads), not a % of it.
        self._control_panel = HardwareControlPanel(speed_as_max=True)

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

        # v7.5.x: dedicated Microscope sub-page — filter cubes, objectives and
        # focus preferences for the motorized body. Hosts the SAME
        # MicroscopeSetupPanel the jog card's ⚙ dialog wraps, so there is only
        # one microscope setup surface.
        from gui.pages.hardware.microscope_setup_panel import (
            MicroscopeSetupPanel)
        self._microscope_panel = MicroscopeSetupPanel()
        micro_holder = QWidget()
        micro_holder.setStyleSheet(f"background-color: {_bg};")
        micro_lay = QVBoxLayout(micro_holder)
        micro_lay.setSpacing(s(18))
        micro_lay.setContentsMargins(s(20), s(20), s(20), s(20))
        micro_lay.addWidget(self._microscope_panel)
        micro_scroll = QScrollArea()
        micro_scroll.setWidgetResizable(True)
        micro_scroll.setFrameShape(QFrame.NoFrame)
        micro_scroll.setStyleSheet(
            f"QScrollArea {{ background-color: {_bg}; border: none; }}")
        micro_scroll.setWidget(micro_holder)
        self._sub_scrolls["microscope"] = micro_scroll

        # v7.18: dedicated Incubator sub-page — transport, zone naming and
        # the setpoint ceiling for the two-zone heater stage. Hosts the SAME
        # IncubatorSetupPanel pattern as the Microscope tab; settings only —
        # connecting lives on the Incubator page / Connect Hardware card.
        from gui.pages.hardware.incubator_panel import IncubatorSetupPanel
        self._incubator_panel = IncubatorSetupPanel()
        incu_holder = QWidget()
        incu_holder.setStyleSheet(f"background-color: {_bg};")
        incu_lay = QVBoxLayout(incu_holder)
        incu_lay.setSpacing(s(18))
        incu_lay.setContentsMargins(s(20), s(20), s(20), s(20))
        incu_lay.addWidget(self._incubator_panel)
        incu_scroll = QScrollArea()
        incu_scroll.setWidgetResizable(True)
        incu_scroll.setFrameShape(QFrame.NoFrame)
        incu_scroll.setStyleSheet(
            f"QScrollArea {{ background-color: {_bg}; border: none; }}")
        incu_scroll.setWidget(incu_holder)
        self._sub_scrolls["incubator"] = incu_scroll

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
        # v7.12: the Plate sub-page opens as a LIBRARY of plate documents and
        # drills into a parametric builder. Plates and rosettes are the same
        # kind of document behind a mode pill, so one builder serves both —
        # the rosette case differs only in that its boundary is a bore.
        #
        # This replaces the v7.4.x pair of PlateDesignerWidget instances that
        # shared one design object across two sub-pages, and the Format→Type
        # card that sat above them. Splitting plate identity across a card and
        # a picker gave `plate_format` / `plate_name` / `plate_type_id` two
        # owners, and each silently cleared the other's choice; the active
        # design is now the single owner.
        from gui.pages.hardware.plate_workspace import PlateWorkspacePage
        from gui.pages.hardware.rosette_placement import RosettePlacementPage

        # v7.9.1: the unsaved stand-in document for a BUNDLED plate selection,
        # as ``((format, plate_type_id), PlateDocument)``. Written on disk only
        # once a rosette is actually placed (see _on_placements_changed).
        self._pending_plate_fork = None

        self._plate_workspace = PlateWorkspacePage()
        self._plate_workspace.active_plate_changed.connect(
            self._on_active_plate_changed)
        self._plate_workspace.library_changed.connect(self._on_library_changed)
        self._sub_layouts["plate"].addWidget(self._plate_workspace, 1)

        # The old Rosette sub-page hosted a second designer reached by
        # double-clicking a well. Rosettes are library documents now, so this
        # slot becomes PLACEMENT: design once, seat many.
        self._rosette_placement = RosettePlacementPage(
            plate_store=self._plate_workspace.plate_store(),
            rosette_store=self._plate_workspace.rosette_store())
        self._rosette_placement.placements_changed.connect(
            self._on_placements_changed)
        self._rosette_placement.edit_rosette_requested.connect(
            self._on_edit_rosette_requested)

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
        self._pump_settle_spin.setRange(0.0, 30.0)
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
        self._pump_prime_spin.setRange(0.0, 30.0)
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

        # v7.5.x: pressure relief / compliance is now an absolute µL value PER
        # PUMP (retired the old "% of syringe" spin + 3 context toggles). It is
        # measured by the Needle Location compliance calibration and applied as
        # backlash compensation (take-up on reversal + unload on stop), enabled
        # from the pump jog panel. Per-pump values are edited on the Common Print
        # Settings page.
        _relief_note = QLabel(
            "Pressure relief is now per-pump (µL): calibrate it on "
            "Calibration → Needle Location, review/edit values on Workflows → "
            "Common Print Settings, and enable backlash compensation from the "
            "pump jog panel.")
        _relief_note.setWordWrap(True)
        timing_lay.addWidget(_relief_note, 2, 0, 1, 3)

        # v7.5.x: needle-derived MAX SAFE FLOW readout (Hagen–Poiseuille from the
        # needle bore + length at a fixed water-reference viscosity). This is the
        # ceiling SafetyLimits hard-caps every pump move to — shown here so the
        # operator can see why a too-fast flow is being limited (over-pressure
        # ingests air). Refreshed on any config change via _refresh_max_flow_display.
        self._pump_maxflow_lbl = QLabel("Max safe pump flow: —")
        self._pump_maxflow_lbl.setWordWrap(True)
        self._pump_maxflow_lbl.setToolTip(
            "Maximum safe pump flow rate computed from the configured needle "
            "bore + length (Hagen–Poiseuille, water-reference viscosity). Every "
            "pump move — print path, ink pickup, reagent deposit — is hard-"
            "capped to this so the needle never over-pressures and ingests air. "
            "Quick Print also reduces its max print speed to stay under it.")
        self._pump_maxflow_lbl.setStyleSheet(
            f"color: {COLORS.get('green', '#a6e3a1')}; "
            f"padding: {sp(2)} {sp(2)}; font-weight: 600;")
        timing_lay.addWidget(self._pump_maxflow_lbl, 6, 0, 1, 3)

        timing_lay.setColumnStretch(2, 1)

        self._sub_layouts["pumps_inks"].addWidget(timing_group)
        self._refresh_max_flow_display()

        # v7.2.9: Ink Swap Strategy UI moved to Print Setup → Plan of Action

        # ── Section 5: Needle Configuration (v7.2.4: MOVED DOWN) ─
        needle_group = QGroupBox("Needle Configuration")
        needle_group.setStyleSheet(self._group_style())
        needle_lay = QGridLayout(needle_group)
        # v7.4.2 polish: roomier grid spacing.
        needle_lay.setHorizontalSpacing(s(12))
        needle_lay.setVerticalSpacing(s(10))

        # v7.9: the assembly FORM — how many needles are bound together. This is
        # ORTHOGONAL to the needle TYPE below (one bore's taper): a backpack of
        # two pulled capillaries needs both axes, which is exactly why the form
        # must never be smuggled into `needle_type` (NeedleSpec.__post_init__
        # silently rewrites an unknown needle_type to "hypodermic", so the whole
        # assembly would vanish with no error).
        needle_lay.addWidget(QLabel("Assembly form:"), 0, 0)
        self._needle_form_combo = QComboBox()
        self._needle_form_combo.addItem("Single needle — 1 bore",
                                        NEEDLE_FORM_SINGLE)
        self._needle_form_combo.addItem(
            "Backpack — 2 bores, different sizes", NEEDLE_FORM_BACKPACK)
        self._needle_form_combo.addItem("Septum — 2 identical bores",
                                        NEEDLE_FORM_SEPTUM)
        self._needle_form_combo.addItem("Triple — 3 identical bores",
                                        NEEDLE_FORM_TRIPLE)
        self._needle_form_combo.setToolTip(
            "How the mounted assembly is built. This ONE choice decides how much "
            "geometry you enter:\n"
            "Single: one bore — identical to every pre-v7.9 setup.\n"
            "Backpack: two needles of DIFFERENT sizes bound together — enter the "
            "second needle's diameters below; the two share a length.\n"
            "Septum: two identical bores split by a septum — bore 2 copies bore 1.\n"
            "Triple: three of the same needle fused — bores 2 and 3 copy bore 1.\n"
            "Every bore still gets its own pump, its own label and its own "
            "measured mount offset.")
        self._needle_form_combo.currentIndexChanged.connect(self._on_needle_form_changed)
        needle_lay.addWidget(self._needle_form_combo, 0, 1, 1, 3)

        # v7.6: needle TYPE picks which geometry block is shown below.
        needle_lay.addWidget(QLabel("Needle type:"), 1, 0)
        self._needle_type_combo = QComboBox()
        self._needle_type_combo.addItem("Hypodermic (gauge)", NEEDLE_TYPE_HYPODERMIC)
        self._needle_type_combo.addItem("Pulled glass capillary", NEEDLE_TYPE_CAPILLARY)
        self._needle_type_combo.setToolTip(
            "Hypodermic: one straight bore, geometry from the ASTM gauge catalog.\n"
            "Pulled glass capillary: TWO flow stages — a wide barrel feeding a "
            "narrow pulled tip. The tip sets the deposited feature size and "
            "dominates the flow resistance; the barrel sets the held volume.")
        self._needle_type_combo.currentIndexChanged.connect(self._on_needle_type_changed)
        needle_lay.addWidget(self._needle_type_combo, 1, 1, 1, 3)

        # v7.9: on a multi-bore assembly this block edits BORE 1 — the datum bore
        # whose geometry every legacy reader of `needle.id_um` /
        # `cross_section_area_mm2` sees (NeedleSpec mirrors bores[0] onto its flat
        # fields). What the OTHER bores are is decided by the form, right here:
        # a septum/triple copies this bore, and a backpack takes its second set of
        # diameters from the row below. Nothing about a bore's geometry is entered
        # anywhere else on the page, so there is exactly one editor per value.
        self._needle_datum_note = QLabel("")
        self._needle_datum_note.setWordWrap(True)
        self._needle_datum_note.setStyleSheet(
            f"color: {COLORS.get('subtext0', '#a6adc8')}; ")
        self._needle_datum_note.setVisible(False)
        needle_lay.addWidget(self._needle_datum_note, 2, 0, 1, 4)

        # ── Hypodermic row (the legacy widgets, unchanged) ──
        self._hypo_row = QWidget()
        hypo_lay = QHBoxLayout(self._hypo_row)
        hypo_lay.setContentsMargins(0, 0, 0, 0)
        hypo_lay.setSpacing(s(12))
        hypo_lay.addWidget(QLabel("Gauge:"))
        self.gauge_combo = QComboBox()
        self.gauge_combo.addItem("— Select —", None)
        for gauge in sorted(self._needle_catalog.keys()):
            self.gauge_combo.addItem(f"{gauge}G", gauge)
        self.gauge_combo.currentIndexChanged.connect(self._on_needle_changed)
        hypo_lay.addWidget(self.gauge_combo)
        hypo_lay.addSpacing(s(12))
        hypo_lay.addWidget(QLabel("Length:"))
        self.length_combo = QComboBox()
        self.length_combo.addItem("1.0\"", 1.0)
        self.length_combo.addItem("1.5\"", 1.5)
        self.length_combo.addItem("2.0\"", 2.0)
        self.length_combo.currentIndexChanged.connect(self._on_config_changed)
        hypo_lay.addWidget(self.length_combo)
        hypo_lay.addStretch(1)
        needle_lay.addWidget(self._hypo_row, 3, 0, 1, 4)

        # ── Pulled glass capillary card ──
        self._cap_row = self._build_capillary_card()
        needle_lay.addWidget(self._cap_row, 4, 0, 1, 4)

        # ── Backpack second needle (v7.9.x) ──
        # The ONE form whose bores differ, and they differ in DIAMETER only —
        # two needles bound side by side share a length. So this row asks for
        # exactly the two numbers that can differ and derives the rest from
        # bore 1: no second length to contradict the first, and no way to save a
        # backpack whose bores disagree about how far the tips reach.
        self._bp_row = self._build_backpack_card()
        needle_lay.addWidget(self._bp_row, 5, 0, 1, 4)

        # v7.9: the bore COUNT is derived from the assembly form — one number,
        # one control. `channels_spin` is kept alive (hidden) as a mirror because
        # `_rebuild_config`, `_rebuild_channel_map_rows` and
        # `_update_channel_map_status` all read it and tests assert on it; the
        # form combo is the only thing that writes it. Two independently editable
        # controls for the same quantity is exactly how a count/geometry mismatch
        # gets shipped.
        self.channels_spin = QSpinBox()
        self.channels_spin.setRange(1, 3)
        self.channels_spin.setValue(1)
        self.channels_spin.setToolTip(
            "Bores in the assembly — derived from the assembly form above.")
        self.channels_spin.valueChanged.connect(self._on_channels_changed)
        self.channels_spin.setVisible(False)
        needle_lay.addWidget(self.channels_spin, 6, 0)

        self.needle_info_label = QLabel("Select a needle gauge above")
        self.needle_info_label.setWordWrap(True)
        self.needle_info_label.setStyleSheet(
            f"color: {COLORS.get('subtext0', '#a6adc8')}; ")
        needle_lay.addWidget(self.needle_info_label, 7, 0, 1, 4)

        self._sub_layouts["needle"].addWidget(needle_group)

        # ── Section 5b: per-bore geometry (v7.9) ──────────────────
        # Hidden entirely for a single needle, so the Single form is visually
        # and behaviourally identical to every pre-v7.9 setup.
        self._sub_layouts["needle"].addWidget(self._build_bore_group())

        # ── Section 6: Needle Bore → Pump Mapping (v7.2.4: NEW) ────
        # v7.9: display strings say BORE — the widget/attribute names stay as
        # they are (referenced across the page and asserted in tests), but the
        # operator must not read "channel" here while `validate()` reports
        # "Bore 2 → P3" about the very same row.
        self.channel_map_group = QGroupBox("Needle Bore Assignment")
        self.channel_map_group.setStyleSheet(self._group_style())
        self._channel_map_layout = QVBoxLayout(self.channel_map_group)

        # Info label
        self.channel_map_info = QLabel(
            "Each needle bore must be assigned to its own pump. Picking one here "
            "enables it on the Pump tab — nothing needs setting up there first.")
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

        # v7.6: apply the needle-type visibility now that BOTH the needle group
        # and the bore-map rows exist (it rebuilds those rows, so it has to run
        # after `_channel_rows_layout` is built). Visibility only — the full
        # handler rebuilds the config, which needs widgets built later.
        self._apply_needle_type_visibility()

        # ── Section 7: Rosette placement (v7.12) ──────────────────
        # The "Layout" sub-page. The v7.4.8 rosette DESIGNER moved into the
        # Plate tab's Rosettes mode; this slot is now where designed rosettes
        # are seated into wells.
        #
        # The legacy "Rosette Library" table and its RosetteEditorDialog were
        # removed here: they edited `HardwareConfig.rosette_library`, whose only
        # consumer (`WellSetup.attach_rosette`) has no callers repo-wide, so the
        # table was write-only and unrelated to the designer beside it. The
        # config FIELD is retained so older setup JSON still round-trips.
        self._sub_layouts["rosette"].addWidget(self._rosette_placement, 1)

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
            tooltip="Scan for available cameras "
                    "(OpenCV, ToupCam, Andor, Tucsen, Simulated)")
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
        # v7.5.x: per-slot holder in the controls column where the image-
        # correction strip is mounted lazily (kept OUT of the preview holder so
        # the live feed gets the full preview area).
        self._live_cam_correction_holders: list[QWidget] = []
        # v7.5.x: per-slot rotation-vs-stage readout + calibrate button.
        self._live_cam_rot_labels: list[QLabel] = []
        self._live_cam_rot_btns: list[QPushButton] = []
        # v7.10: per-slot "square up the mount" button (live rotation aid).
        self._live_cam_square_btns: list[QPushButton] = []
        # v7.5.x: per-slot orientation controls (ONE unified system applied to
        # the live view + mosaic + click-mapping): flip X (mirror), flip Y, and
        # a custom rotation spin.
        self._live_cam_mirror_checks: list[QCheckBox] = []
        self._live_cam_flip_y_checks: list[QCheckBox] = []
        self._live_cam_rot_spins: list = []
        # v7.16: per-slot centred crop — a sensor wider than the illuminated
        # field images the dark tube wall at the edges. Applied at the frame
        # SOURCE, so it reaches the live view, the mosaic, captures and
        # recordings alike.
        self._live_cam_crop_checks: list[QCheckBox] = []
        self._live_cam_crop_spins: list = []
        self._live_cam_crop_labels: list[QLabel] = []
        # WHERE the crop sits — the lit circle is centred on the optical axis,
        # which need not be the middle of the sensor.
        self._live_cam_crop_off_x_spins: list = []
        self._live_cam_crop_off_y_spins: list = []
        self._live_cam_crop_centre_btns: list[QPushButton] = []
        # Debounced persistence. The offset spins auto-repeat, so a held arrow
        # would otherwise write the calibration file on every step — file I/O
        # on the GUI thread, at a rate set by Qt's key-repeat. See
        # _apply_slot_crop.
        self._pending_crop_writes: dict[int, object] = {}
        self._crop_persist_timer = QTimer(self)
        self._crop_persist_timer.setSingleShot(True)
        self._crop_persist_timer.setInterval(400)
        self._crop_persist_timer.timeout.connect(self._flush_crop_persist)
        # hideEvent covers navigating away; this covers closing the app while
        # still ON this page, which no page-level event reliably reports.
        try:
            _qapp = QApplication.instance()
            if _qapp is not None:
                _qapp.aboutToQuit.connect(self._flush_crop_persist)
        except Exception:
            pass
        self._cam_preview_signals_wired = False

        from gui.styles import build_glass_panel_style
        assign_glass = build_glass_panel_style("camMiniCard")
        max_cams = MAX_LIVE_CAMERAS  # one mini-card per CameraManager slot
        for i in range(max_cams):
            row = QFrame()
            row.setObjectName("camMiniCard")
            row.setStyleSheet(assign_glass)
            # v7.5.x: two-column card — a LARGE live preview on the left and a
            # tidy controls column on the right. Previously everything (source,
            # role, start, rotation, flips, and the image-correction sliders)
            # was stacked in one narrow grid with the preview squeezed into a
            # height-capped holder shared with the correction sliders, so the
            # live feed rendered tiny. Splitting them gives the feed real
            # estate while keeping each card's height compact.
            card = QHBoxLayout(row)
            card.setContentsMargins(s(14), s(12), s(14), s(12))
            card.setSpacing(s(14))

            # ── LEFT: live preview (hidden until the camera is running) ──────
            # No height cap here anymore — CameraFeedView respects the camera's
            # aspect ratio (heightForWidth), so it fills this column cleanly.
            preview = QWidget()
            preview.setVisible(False)
            pv_lay = QVBoxLayout(preview)
            pv_lay.setContentsMargins(0, 0, 0, 0)
            pv_lay.setSpacing(0)
            preview.setMinimumWidth(s(300))
            preview.setMinimumHeight(s(240))
            preview.setMaximumHeight(s(420))
            card.addWidget(preview, 3)
            self._live_cam_preview_holders.append(preview)
            self._live_cam_previews.append(None)
            self._live_cam_correction.append(None)

            # ── RIGHT: controls column ──────────────────────────────────────
            controls = QWidget()
            cl = QVBoxLayout(controls)
            cl.setContentsMargins(0, 0, 0, 0)
            cl.setSpacing(s(8))

            # Header: Cam N pill + role badge.
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
            role_badge = StatusBadge("Unassigned", variant="pending")
            self._live_cam_role_badges.append(role_badge)
            hdr = QHBoxLayout()
            hdr.setSpacing(s(8))
            hdr.addWidget(cam_pill, 0, Qt.AlignVCenter)
            hdr.addStretch(1)
            hdr.addWidget(role_badge, 0, Qt.AlignVCenter)
            cl.addLayout(hdr)

            # Source picker.
            src = QComboBox()
            src.addItem("— None —", None)
            src.setMinimumWidth(s(160))
            self._live_cam_source_combos.append(src)
            src.currentIndexChanged.connect(
                lambda _idx, cam_i=i: self._on_live_cam_source_changed(cam_i)
            )
            src_row = QHBoxLayout()
            src_row.setSpacing(s(8))
            src_row.addWidget(self._field_label("Source"))
            src_row.addWidget(src, 1)
            cl.addLayout(src_row)

            # Role picker.
            role_combo = QComboBox()
            role_combo.addItem("Unassigned", CameraRole.UNASSIGNED)
            role_combo.addItem("Microscope", CameraRole.MICROSCOPE)
            role_combo.addItem("Needle cam 1 (side)", CameraRole.NEEDLE_X)
            role_combo.addItem("Needle cam 2 (side)", CameraRole.NEEDLE_Y)
            role_combo.addItem("Monitor (overview)", CameraRole.MONITOR)
            role_combo.setToolTip(
                "Workflow role for this camera slot. All non-Unassigned "
                "roles are singletons — assigning one here automatically "
                "clears it from any other slot."
            )
            role_combo.currentIndexChanged.connect(
                lambda idx_combo, cam_i=i: self._on_live_cam_role_changed(cam_i)
            )
            self._live_cam_role_combos.append(role_combo)
            role_row = QHBoxLayout()
            role_row.setSpacing(s(8))
            role_row.addWidget(self._field_label("Role"))
            role_row.addWidget(role_combo, 1)
            cl.addLayout(role_row)

            # Start/Stop toggle.
            start_btn = QPushButton("▶ Start")
            start_btn.setObjectName("accentBtn")
            start_btn.setEnabled(False)
            start_btn.setToolTip(
                "Start/stop this camera so you can verify the live feed and "
                "run its calibration."
            )
            start_btn.clicked.connect(
                lambda _checked=False, cam_i=i: self._on_toggle_camera(cam_i)
            )
            self._live_cam_start_btns.append(start_btn)
            cl.addWidget(start_btn)

            # ── Orientation & calibration sub-group ─────────────────────────
            orient_caption = QLabel("Orientation & calibration")
            orient_caption.setStyleSheet(
                f"color: {COLORS.get('subtext0', '#a6adc8')}; "
                f"font-size: {sf(9)}pt; font-weight: 700;")
            cl.addWidget(orient_caption)

            # Rotation-vs-stage readout + calibrate button.
            rot_lbl = QLabel("Rotation vs stage: not calibrated")
            rot_lbl.setWordWrap(True)
            rot_lbl.setStyleSheet(
                f"color: {COLORS.get('subtext0', '#a6adc8')}; "
                f"font-size: {scaled_font_size(9)}pt;"
            )
            self._live_cam_rot_labels.append(rot_lbl)
            cl.addWidget(rot_lbl)

            rot_btn = QPushButton("⟳ Rotation…")
            rot_btn.setEnabled(False)
            rot_btn.setToolTip(
                "Calibrate this camera's rotation relative to the stage "
                "axes: the stage moves a known direction and the dialog "
                "measures the image displacement. Rotation only — the "
                "camera's µm/px calibration is untouched. Requires the "
                "camera running and the XY stage connected."
            )
            rot_btn.clicked.connect(
                lambda _checked=False, cam_i=i:
                self._on_calibrate_slot_rotation(cam_i)
            )
            self._live_cam_rot_btns.append(rot_btn)
            cl.addWidget(rot_btn)

            # v7.10: physically square the mount, with live optical feedback.
            square_btn = QPushButton("⊾ Square up mount…")
            square_btn.setEnabled(False)
            square_btn.setToolTip(
                "Turn this camera in its mount until its measured rotation "
                "lands on 0/90/180/270°, watching a live degrees-to-go "
                "readout and a translucent ghost of the target. Moves "
                "nothing — the measurement is purely optical. Requires the "
                "camera running and a rotation already calibrated."
            )
            square_btn.clicked.connect(
                lambda _checked=False, cam_i=i:
                self._on_square_up_mount(cam_i)
            )
            self._live_cam_square_btns.append(square_btn)
            cl.addWidget(square_btn)

            # v7.16: Flip X / Flip Y / Rotation set the LIVE VIEW only — how the
            # operator wants the feed to look. They used to write the measured
            # camera→stage orientation, which the objective/mosaic calibration
            # then overwrote, so setting the view up and calibrating flipped it
            # back. The measured orientation now has its own controls (⟳
            # Rotation…, ⊾ Square up mount…, the objective calibration) and is
            # reported in the readout above.
            mirror_cb = QCheckBox("⇄ Flip X axis")
            mirror_cb.setEnabled(False)
            mirror_cb.setToolTip(
                "Flip the LIVE VIEW horizontally (mirror the X axis).\n\n"
                "Display only — this does not change how mosaic tiles are "
                "placed or where a click drives the stage. Those follow the "
                "measured camera-vs-stage orientation shown above, and a "
                "calibration will no longer reset this view setting."
            )
            mirror_cb.toggled.connect(
                lambda checked, cam_i=i: self._on_toggle_mirror(cam_i, checked)
            )
            self._live_cam_mirror_checks.append(mirror_cb)

            flip_y_cb = QCheckBox("⇅ Flip Y axis")
            flip_y_cb.setEnabled(False)
            flip_y_cb.setToolTip(
                "Flip the LIVE VIEW vertically (mirror the Y axis).\n\n"
                "Display only — the mosaic and the click→stage mapping are "
                "unaffected (they use the measured orientation above)."
            )
            flip_y_cb.toggled.connect(
                lambda checked, cam_i=i: self._on_toggle_flip_y(cam_i, checked)
            )
            self._live_cam_flip_y_checks.append(flip_y_cb)
            flip_row = QHBoxLayout()
            flip_row.setSpacing(s(10))
            flip_row.addWidget(mirror_cb)
            flip_row.addWidget(flip_y_cb)
            flip_row.addStretch(1)
            cl.addLayout(flip_row)

            # Custom in-plane rotation spin.
            rot_spin = QDoubleSpinBox()
            rot_spin.setRange(-180.0, 180.0)
            rot_spin.setSingleStep(1.0)
            rot_spin.setDecimals(1)
            rot_spin.setEnabled(False)
            rot_spin.setToolTip(
                "Rotate the LIVE VIEW by this angle.\n\n"
                "Display only — it does not change the measured camera-vs-stage "
                "rotation above, so mosaic tiles and click→stage mapping are "
                "unaffected and a calibration will not reset it. Typically 0/90/"
                "180/270 to get the plate the way up you want to see it.")
            rot_spin.valueChanged.connect(
                lambda v, cam_i=i: self._on_slot_rotation_spin(cam_i, v))
            self._live_cam_rot_spins.append(rot_spin)
            rot_spin_row = QHBoxLayout()
            rot_spin_row.setSpacing(s(4))
            rot_spin_row.addWidget(QLabel("Rotation °:"))
            rot_spin_row.addWidget(rot_spin, 1)
            cl.addLayout(rot_spin_row)

            # v7.16: square crop. A microscope's illuminated field is a circle;
            # a sensor wider than it images the dark tube wall at the left and
            # right edges. Those pixels carry no specimen, they drag the
            # mosaic's flat-field estimate toward black, and the raster still
            # steps by their width. Unlike the flips above this really removes
            # pixels — at the frame source, so every surface agrees.
            crop_cb = QCheckBox("⬛ Square crop")
            crop_cb.setEnabled(False)
            crop_cb.setToolTip(
                "Crop every frame from this camera to a centred square, so the "
                "dark edges outside the illuminated field are discarded.\n\n"
                "Applied at the frame source: the live view, mosaic tiles, "
                "still captures, video recordings and detection all see the "
                "cropped frame. The centre is preserved exactly, so click-to-"
                "stage stays correct, and µm/px is unaffected (cropping removes "
                "pixels; it does not change what a pixel spans).")
            crop_cb.toggled.connect(
                lambda checked, cam_i=i: self._on_toggle_crop(cam_i, checked))
            self._live_cam_crop_checks.append(crop_cb)

            crop_spin = QDoubleSpinBox()
            crop_spin.setRange(10.0, 100.0)
            crop_spin.setSingleStep(5.0)
            crop_spin.setDecimals(0)
            crop_spin.setValue(100.0)
            crop_spin.setSuffix(" %")
            crop_spin.setEnabled(False)
            crop_spin.setToolTip(
                "Size of the square as a percentage of the sensor's SHORT "
                "side. 100 % is the largest square that fits; lower it when "
                "the illuminated circle is smaller than the short side and the "
                "corners are still dark.\n\n"
                "A percentage rather than pixels, so the cropped region stays "
                "the same physical part of the field if the capture resolution "
                "changes (e.g. switching 2x2 binning off).")
            crop_spin.valueChanged.connect(
                lambda v, cam_i=i: self._on_crop_scale_spin(cam_i, v))
            self._live_cam_crop_spins.append(crop_spin)

            crop_row = QHBoxLayout()
            crop_row.setSpacing(s(6))
            crop_row.addWidget(crop_cb)
            crop_row.addWidget(crop_spin, 1)
            cl.addLayout(crop_row)

            # WHERE the square sits. The illuminated circle is centred on the
            # optical axis, which need not pass through the middle of the
            # sensor — so a centred square can still clip one side. The
            # displacement is compensated in the pixel↔stage conversions, so
            # re-aiming the crop does NOT move taught coordinates.
            off_x_spin = QDoubleSpinBox()
            off_y_spin = QDoubleSpinBox()
            for sp_, ax in ((off_x_spin, "X"), (off_y_spin, "Y")):
                sp_.setRange(-50.0, 50.0)
                sp_.setSingleStep(1.0)
                sp_.setDecimals(1)
                sp_.setValue(0.0)
                sp_.setSuffix(" %")
                sp_.setEnabled(False)
                sp_.setToolTip(
                    f"Move the crop along {ax}, as a percentage of the frame. "
                    f"0 % is centred.\n\n"
                    f"Use this when the lit circle is not centred on the sensor "
                    f"and a centred square still clips one edge. The offset is "
                    f"cancelled out of the click-to-stage map and the mosaic "
                    f"tile placement, so re-aiming it does NOT shift a taught "
                    f"plate calibration. It is clamped to keep the crop inside "
                    f"the frame.")
            off_x_spin.valueChanged.connect(
                lambda v, cam_i=i: self._on_crop_offset_spin(cam_i))
            off_y_spin.valueChanged.connect(
                lambda v, cam_i=i: self._on_crop_offset_spin(cam_i))
            self._live_cam_crop_off_x_spins.append(off_x_spin)
            self._live_cam_crop_off_y_spins.append(off_y_spin)

            centre_btn = QPushButton("⌖")
            centre_btn.setEnabled(False)
            centre_btn.setFixedWidth(s(28))
            centre_btn.setToolTip("Re-centre the crop (offset back to 0 %).")
            centre_btn.clicked.connect(
                lambda _checked=False, cam_i=i: self._on_crop_recentre(cam_i))
            self._live_cam_crop_centre_btns.append(centre_btn)

            off_row = QHBoxLayout()
            off_row.setSpacing(s(4))
            off_row.addWidget(QLabel("Offset:"))
            off_row.addWidget(off_x_spin, 1)
            off_row.addWidget(off_y_spin, 1)
            off_row.addWidget(centre_btn)
            cl.addLayout(off_row)

            crop_lbl = QLabel("Crop: off (full sensor)")
            crop_lbl.setWordWrap(True)
            crop_lbl.setStyleSheet(
                f"color: {COLORS.get('subtext0', '#a6adc8')}; "
                f"font-size: {scaled_font_size(9)}pt;")
            self._live_cam_crop_labels.append(crop_lbl)
            cl.addWidget(crop_lbl)

            # Holder for the image-correction strip (mounted lazily once the
            # CameraManager arrives, so the feed can be previewed first).
            corr_holder = QWidget()
            corr_lay = QVBoxLayout(corr_holder)
            corr_lay.setContentsMargins(0, 0, 0, 0)
            corr_lay.setSpacing(0)
            self._live_cam_correction_holders.append(corr_holder)
            cl.addWidget(corr_holder)

            cl.addStretch(1)
            card.addWidget(controls, 2)

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
            "Two interchangeable side cameras sit symmetric about the stage "
            "+X axis at +45° and −45°. Assign each a camera in the section "
            "above; then run an independent stage-motion µm/px calibration "
            "for each — the calibration measures which camera looks along "
            "which direction (and its small view roll)."
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
            (CameraRole.NEEDLE_X, "Needle cam 1", COLORS.get("peach", "#fab387")),
            (CameraRole.NEEDLE_Y, "Needle cam 2", COLORS.get("sky", "#89dceb")),
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
            # v7.5.x: for the shared ``mosaic_scan`` section — the confirm step
            # seeds and persists the tile overlap there (it belongs to the scan,
            # not to one objective).
            settings_getter=lambda: getattr(self, "_settings", None),
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
            "Save Setup", "save", object_name="accentBtn",
            tooltip="Save this setup into the Saved setups folder "
                    "(config/hardware) under the Name above, so it appears "
                    "in the Saved setups list.")
        btn_save.clicked.connect(self._save_config)
        actions_lay.addStretch()
        actions_lay.addWidget(btn_save)

        btn_save_as = icon_button(
            "Save As…", "save",
            tooltip="Save a copy to any folder. A setup saved outside "
                    "config/hardware will NOT appear in the Saved setups list.")
        btn_save_as.clicked.connect(self._save_config_as)
        actions_lay.addWidget(btn_save_as)

        btn_load = icon_button(
            "Load Config", "folder-open",
            tooltip="Load a previously saved hardware configuration.")
        btn_load.clicked.connect(self._load_config)
        actions_lay.addWidget(btn_load)

        self._content_layout.addWidget(actions_group)

        # ── Finalize sub-pages (v7.4.0-b) ─────────────────────────
        # Add stretch to each sub-page layout so groups stack at the top.
        # v7.12: "plate" and "rosette" host full-height workspaces now, so a
        # trailing stretch would squash them to nothing.
        for key in ("identity", "pumps_inks", "inks", "needle", "cameras"):
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
        # v7.5.x: the plate is a GRID of wells; the microscope icon now names
        # the actual microscope sub-page below.
        self.add_sub_page("grid",      "Plate",           self._sub_scrolls["plate"])
        # v7.5.x: order the dependent setups left-to-right so each
        # section's options build on the ones to its left:
        # Rosette → Ink → Needle → Pump.
        # v7.12: the rosette DESIGNER lives on the Plate tab now; this slot is
        # rosette PLACEMENT, so it is named for what it does.
        self._rosette_sub_index = len(self._sub_pages)
        self._layout_sub_index = self._rosette_sub_index
        self.add_sub_page("flower",    "Layout",          self._sub_scrolls["rosette"])
        self.add_sub_page("flask",     "Ink",             self._sub_scrolls["inks"])
        self.add_sub_page("needle",    "Needle",          self._sub_scrolls["needle"])
        self.add_sub_page("droplet",   "Pump",            self._sub_scrolls["pumps_inks"])
        self.add_sub_page("camera",    "Cameras",         self._sub_scrolls["cameras"])
        self.add_sub_page("microscope","Microscope",      self._sub_scrolls["microscope"])
        # v7.18: incubator heater settings (transport / zones / ceiling).
        self.add_sub_page("incubator", "Incubator",       self._sub_scrolls["incubator"])
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

    # ── v7.12: Plate workspace ↔ Layout placement ────────────────

    def _on_hw_sub_page_changed(self, index: int) -> None:
        """Point the Layout tab at the active plate when it is shown.

        Also guards unsaved builder work: leaving the Plate tab mid-edit used
        to discard geometry with no prompt.
        """
        if not hasattr(self, "_plate_workspace"):
            return
        if index == getattr(self, "_rosette_sub_index", -1):
            self._plate_workspace.maybe_discard()
            self._rosette_placement.set_plate(
                self._active_plate_document(materialize=True))
        elif index == getattr(self, "_plate_sub_index", -1):
            self._plate_workspace.refresh()

    def _active_plate_document(self, materialize: bool = False):
        """The `PlateDocument` the hardware config points at, or None.

        v7.9.1 ``materialize``: a BUNDLED plate (a bare standard format, or a
        `PlateType` product) has no document on disk — `_on_active_plate_changed`
        clears `plate_doc_id` for exactly that reason — so the Layout tab showed
        "No plate selected" for every plate the library ships, which is every
        plate in a fresh install and every saved hardware config in this repo.
        The operator's report ("I can't add a rosette because the plate I have
        selected doesn't load") is that state.

        With ``materialize=True`` one is stood up on demand, the same way the
        builder already does when you open a bundled card: standards auto-fork
        on first edit. The fork is held unsaved until a rosette is actually
        placed — `_on_placements_changed` is what writes it and re-points the
        config. Cached so re-entering the tab, or a config refresh, cannot swap
        the document out from under in-progress placements.

        ⚠ It defaults to **False**, and `_rebuild_config` must keep it that way:
        that method copies `doc.meta.name` into `plate_name`, which sits in
        `active_plate_key`'s precedence chain. Materialising there would rekey
        every per-plate store (mosaic, taught calibration, well training) to
        "Copy of 24-well" for a plate the operator never forked.
        """
        doc_id = getattr(self._config, "plate_doc_id", "") if self._config \
            else ""
        if doc_id:
            self._pending_plate_fork = None
            try:
                return self._plate_workspace.plate_store().get(doc_id)
            except Exception:                          # pragma: no cover
                return None
        if self._config is None or not materialize:
            return None
        return self._bundled_plate_fork()

    def _bundled_plate_fork(self):
        """An unsaved PlateDocument standing in for the selected bundled plate.

        Keyed by (format, product id) so the cache is dropped the moment the
        operator picks a different plate — otherwise placements stamped onto one
        standard would reappear on the next.
        """
        from gui.pages.hardware.plate_library import product_id, standard_id
        fmt = getattr(self._config, "plate_format", 0) or 0
        type_id = getattr(self._config, "plate_type_id", "") or ""
        if fmt not in PLATE_DEFINITIONS:
            return None
        key = (fmt, type_id)
        cached = getattr(self, "_pending_plate_fork", None)
        if cached is not None and cached[0] == key:
            return cached[1]
        label = type_id or f"{fmt}-well"
        try:
            from SupportClasses.PlateDocument import PlateDocument
            doc = PlateDocument.from_standard_format(
                fmt, name=f"Copy of {label}")
        except Exception as exc:                       # pragma: no cover
            logger.warning("Could not materialise bundled plate %s: %s",
                           label, exc)
            return None
        if type_id:
            # Keep the product identity on the fork so its Z offsets and its
            # per-plate stores still resolve (same rule as the builder's fork).
            doc.meta.plate_type_id = type_id
        self._pending_plate_fork = (key, doc)
        logger.info(
            "Layout tab: '%s' is a bundled plate with no document — editing it "
            "will save a new plate '%s'. (library card %s)",
            label, doc.meta.name,
            product_id(type_id) if type_id else standard_id(fmt))
        return doc

    def _on_active_plate_changed(self, doc_id: str) -> None:
        """★ on a library card → this plate becomes the setup's plate.

        Two shapes, because ``active_plate_key`` resolves
        ``plate_type_id → plate_doc_id → plate_format``:

        * a **saved design** is carried by ``plate_doc_id``;
        * a **bundled standard** has no document, so it is carried by
          ``plate_format`` with ``plate_doc_id`` cleared.

        The combo is synced in the standard case because ``_rebuild_config``
        falls back to it whenever ``plate_doc_id`` is empty — leaving it stale
        would quietly revert the choice on the very next rebuild.
        """
        from gui.pages.hardware.plate_library import (
            base_format_of, is_bundled, is_product, product_type_id)
        if self._config is None:
            return
        if is_bundled(doc_id):
            self._config.plate_doc_id = ""
            self._config.plate_name = ""
            # A product keeps its type id — that is what carries its Z offsets
            # and gives it a mosaic separate from the bare format's.
            self._config.plate_type_id = (product_type_id(doc_id)
                                          if is_product(doc_id) else "")
            fmt = base_format_of(doc_id)
            if fmt in PLATE_DEFINITIONS:
                self._config.plate_format = fmt
            idx = self.plate_combo.findData(self._config.plate_format)
            if idx >= 0:
                self.plate_combo.blockSignals(True)
                self.plate_combo.setCurrentIndex(idx)
                self.plate_combo.blockSignals(False)
        else:
            self._config.plate_doc_id = doc_id or ""
            doc = self._plate_workspace.plate_store().get(doc_id)
            if doc is not None:
                # A display cache only — `plate_doc_id` is the key.
                self._config.plate_name = doc.meta.name
                self._config.plate_type_id = doc.meta.plate_type_id or ""
        if hasattr(self, "_rosette_placement"):
            self._rosette_placement.set_plate(
                self._active_plate_document(materialize=True))
        self._on_config_changed()

    def _active_library_id(self) -> str:
        """What the library should draw the ★ on, for the current config."""
        from gui.pages.hardware.plate_library import product_id, standard_id
        if self._config is None:
            return ""
        # Mirrors `active_plate_key`'s precedence, so the ★ always lands on
        # the card whose key the per-plate stores are actually using.
        doc_id = getattr(self._config, "plate_doc_id", "") or ""
        type_id = getattr(self._config, "plate_type_id", "") or ""
        if type_id and not doc_id:
            return product_id(type_id)
        if doc_id:
            return doc_id
        fmt = getattr(self._config, "plate_format", 0) or 0
        return standard_id(fmt) if fmt in PLATE_DEFINITIONS else ""

    def _on_library_changed(self) -> None:
        """A plate was created, renamed, duplicated or deleted."""
        if hasattr(self, "_rosette_placement"):
            self._rosette_placement.refresh()
        self._on_config_changed()

    def _on_placements_changed(self) -> None:
        """A rosette was seated or removed — persist and tell the app.

        v7.9.1: this is the "first edit" that turns a bundled plate into a real
        one. Saving the fork mints its id, so the config must be re-pointed at
        it here — otherwise the placements land in a document nothing refers to
        and the tab reverts to a blank fork on the next visit.
        """
        doc = self._rosette_placement._doc
        if doc is None:
            self._on_config_changed()
            return
        was_fork = (self._config is not None
                    and not getattr(self._config, "plate_doc_id", ""))
        try:
            self._plate_workspace.plate_store().save(doc)
        except Exception as exc:                       # noqa: BLE001
            logger.warning("Could not save placements: %s", exc)
            self._on_config_changed()
            return
        if was_fork:
            self._config.plate_doc_id = doc.meta.id
            self._config.plate_name = doc.meta.name
            self._pending_plate_fork = None
            logger.info(
                "Placed a rosette on a bundled plate — forked it to '%s' (%s) "
                "and pointed the hardware config at it.",
                doc.meta.name, doc.meta.id)
            try:
                self._plate_workspace.refresh()
            except Exception:                          # pragma: no cover
                pass
        self._on_config_changed()

    def _on_edit_rosette_requested(self, rosette_id: str) -> None:
        """Layout → 'edit this rosette' opens its DESIGN on the Plate tab."""
        idx = getattr(self, "_plate_sub_index", -1)
        if idx >= 0:
            self.switch_to(idx)
        self._plate_workspace.open_rosette(rosette_id)

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
            return ("Needle 1", "warn")
        if role == CameraRole.NEEDLE_Y:
            return ("Needle 2", "warn")
        if role == CameraRole.MONITOR:
            return ("Monitor", "info")
        return ("Unassigned", "pending")

    # ════════════════════════════════════════════════════════════════
    #  v7.4.0-b: SUB-PAGE TITLE FOR MODE PAGE
    # ════════════════════════════════════════════════════════════════

    def get_sub_page_title(self) -> str:
        """Override ModePage to return the descriptive sub-page name.

        v7.12: derived from the titles recorded at registration rather than a
        hand-maintained parallel list. The old list had drifted badly — nine
        labels for ten tabs, in the wrong order from index 3 on (it claimed
        Pump/Needle/Ink/Rosette where the tabs are Rosette/Ink/Needle/Pump) and
        with Microscope missing entirely — so most tabs reported the wrong name
        and the last one reported "Hardware Setup".
        """
        title = self.sub_page_title()
        return f"Hardware: {title}" if title else "Hardware Setup"

    # ════════════════════════════════════════════════════════════════
    #  NEEDLE CHANGE HANDLER
    # ════════════════════════════════════════════════════════════════

    def _on_needle_changed(self):
        if self._needle_type_combo.currentData() == NEEDLE_TYPE_CAPILLARY:
            g = self._capillary_geometry()
            b_vol = math.pi * (g["barrel_id_um"] / 2000.0) ** 2 * g["barrel_length_mm"]
            if g["tip_profile"] == TIP_PROFILE_CONE:
                d1, d2 = g["barrel_id_um"] / 1000.0, g["tip_id_um"] / 1000.0
                t_vol = (math.pi * g["tip_length_mm"] / 12.0) * (
                    d1 * d1 + d1 * d2 + d2 * d2)
            else:
                t_vol = math.pi * (g["tip_id_um"] / 2000.0) ** 2 * g["tip_length_mm"]
            tip_od_txt = (f" / OD {g['tip_od_um']:.0f} µm"
                          if g["tip_od_um"] else "")
            self.needle_info_label.setText(
                f"Barrel: ID {g['barrel_id_um']:.0f} µm / OD {g['barrel_od_um']:.0f} µm "
                f"× {g['barrel_length_mm']:.1f} mm  ({b_vol:.3f} µL)\n"
                f"Pulled tip: ID {g['tip_id_um']:.1f} µm{tip_od_txt} × "
                f"{g['tip_length_mm']:.2f} mm  ({t_vol:.4f} µL)   ·   "
                f"total held volume {b_vol + t_vol:.3f} µL")
            self._sync_needle_preset_combo()
        else:
            gauge = self.gauge_combo.currentData()
            if gauge and gauge in self._needle_catalog:
                spec = self._needle_catalog[gauge]
                self.needle_info_label.setText(
                    f"ID: {spec.id_um} µm | OD: {spec.od_um} µm | "
                    f"Wall: {spec.wall_um} µm")
            else:
                self.needle_info_label.setText("Select a needle gauge above")
        # v7.9.x: every bore's geometry is DERIVED from this block (a copy, or
        # bore 1's length with the second needle's diameters), so a geometry edit
        # changes what the per-bore rows resolve to — including each bore's own
        # flow ceiling. Refreshing here is what keeps the echo from claiming a
        # needle that is no longer configured; the readouts never rebuild the
        # config, so there is no loop. Guarded, so this is a no-op while the page
        # is still being built.
        self._refresh_bore_readouts()
        self._on_config_changed()

    def _on_channels_changed(self, value: int):
        """v7.2.4 S3.9: Rebuild channel mapping rows when channel count changes.

        v7.9: ``channels_spin`` is a hidden mirror of the assembly-form combo, so
        this normally fires only if something writes the spin directly. Rebuild
        the per-bore rows too so the count and the geometry can never diverge
        even on that path.
        """
        self._rebuild_channel_map_rows()
        self._rebuild_bore_rows()
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

    # ── v7.6: pulled glass capillary needle ───────────────────────────

    def _build_capillary_card(self) -> QWidget:
        """The two-stage geometry editor shown for a pulled glass capillary.

        v7.9: the spin ranges/defaults/tooltips come from the module-level
        ``_CAP_SPIN_SPECS`` table so this card and every per-bore row are
        guaranteed identical (see that table's comment for why the ranges are
        deliberately wider than the common band).
        """
        card = QWidget()
        lay = QGridLayout(card)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setHorizontalSpacing(s(12))
        lay.setVerticalSpacing(s(8))

        def _spin(key):
            return _make_cap_spin(key, self._on_needle_changed)

        def _sub(text):
            lbl = QLabel(text)
            lbl.setStyleSheet(
                f"color: {COLORS.get('mauve', '#cba6f7')}; font-weight: 600;")
            return lbl

        # Barrel (bulk section)
        lay.addWidget(_sub("Barrel (bulk)"), 0, 0, 1, 6)
        lay.addWidget(QLabel("Inner Ø:"), 1, 0)
        self._cap_barrel_id_spin = _spin("barrel_id")
        lay.addWidget(self._cap_barrel_id_spin, 1, 1)
        lay.addWidget(QLabel("Outer Ø:"), 1, 2)
        self._cap_barrel_od_spin = _spin("barrel_od")
        lay.addWidget(self._cap_barrel_od_spin, 1, 3)
        lay.addWidget(QLabel("Length:"), 1, 4)
        self._cap_barrel_len_spin = _spin("barrel_len")
        lay.addWidget(self._cap_barrel_len_spin, 1, 5)

        # Pulled tip
        lay.addWidget(_sub("Pulled tip"), 2, 0, 1, 6)
        lay.addWidget(QLabel("Inner Ø:"), 3, 0)
        self._cap_tip_id_spin = _spin("tip_id")
        lay.addWidget(self._cap_tip_id_spin, 3, 1)
        lay.addWidget(QLabel("Outer Ø:"), 3, 2)
        self._cap_tip_od_spin = _spin("tip_od")
        lay.addWidget(self._cap_tip_od_spin, 3, 3)
        lay.addWidget(QLabel("Length:"), 3, 4)
        self._cap_tip_len_spin = _spin("tip_len")
        lay.addWidget(self._cap_tip_len_spin, 3, 5)

        lay.addWidget(QLabel("Tip profile:"), 4, 0)
        self._cap_tip_profile_combo = _make_tip_profile_combo(self._on_needle_changed)
        lay.addWidget(self._cap_tip_profile_combo, 4, 1, 1, 2)

        # Preset row
        lay.addWidget(QLabel("Preset:"), 5, 0)
        self._needle_type_preset_combo = QComboBox()
        self._needle_type_preset_combo.setToolTip(
            "Saved pull recipes. Selecting one COPIES its geometry in — the "
            "needle never references the preset afterwards, so editing or "
            "deleting a preset can't change a saved setup or a calibration.")
        lay.addWidget(self._needle_type_preset_combo, 5, 1, 1, 3)
        self._needle_preset_save_btn = QPushButton("Save as needle type…")
        self._needle_preset_save_btn.clicked.connect(self._save_current_as_needle_type)
        lay.addWidget(self._needle_preset_save_btn, 5, 4)
        self._needle_preset_del_btn = QPushButton("Delete")
        self._needle_preset_del_btn.clicked.connect(self._delete_selected_needle_type)
        lay.addWidget(self._needle_preset_del_btn, 5, 5)

        self._refresh_needle_type_preset_combo()
        # Connected AFTER the initial populate so seeding can't fire it.
        self._needle_type_preset_combo.currentIndexChanged.connect(
            self._on_needle_type_preset_selected)
        return card

    def _build_backpack_card(self) -> QWidget:
        """The backpack's SECOND needle — its diameters, and nothing else.

        A backpack is "two needles of different sizes bound together": bound side
        by side, so they are the same length and only the diameters differ. Asking
        for a second length would offer the operator a way to describe an assembly
        that cannot exist, and the descend is planned against the LONGEST bore —
        so a stray second length is a real Z error, not a cosmetic one.
        """
        card = QGroupBox("Backpack — second needle")
        card.setStyleSheet(self._group_style())
        lay = QGridLayout(card)
        lay.setHorizontalSpacing(s(10))
        lay.setVerticalSpacing(s(6))

        note = QLabel(
            "Only the <b>diameters</b> differ. Bore 2 is bound alongside bore 1, "
            "so it takes bore 1's length (and, for a capillary, its barrel).")
        note.setWordWrap(True)
        note.setStyleSheet(f"color: {COLORS.get('subtext0', '#a6adc8')}; ")
        lay.addWidget(note, 0, 0, 1, 6)

        # ── hypodermic: one gauge picks both diameters ──
        self._bp_hypo_row = QWidget()
        bp_hypo = QHBoxLayout(self._bp_hypo_row)
        bp_hypo.setContentsMargins(0, 0, 0, 0)
        bp_hypo.setSpacing(s(10))
        bp_hypo.addWidget(QLabel("Bore 2 gauge:"))
        self._bp_gauge_combo = QComboBox()
        self._bp_gauge_combo.addItem("— same as Bore 1 —", None)
        for gauge in sorted(self._needle_catalog.keys()):
            self._bp_gauge_combo.addItem(f"{gauge}G", gauge)
        self._bp_gauge_combo.setToolTip(
            "The second needle's gauge — one number, because a gauge fixes both "
            "the inner and the outer Ø from the ASTM catalog.\n"
            "Left at 'same as Bore 1' the backpack is two identical needles, "
            "which is legitimate (and is what a septum is).")
        self._bp_gauge_combo.currentIndexChanged.connect(self._on_needle_changed)
        bp_hypo.addWidget(self._bp_gauge_combo)
        bp_hypo.addStretch(1)
        lay.addWidget(self._bp_hypo_row, 1, 0, 1, 6)

        # ── capillary: the pulled tip's two diameters ──
        self._bp_cap_row = QWidget()
        bp_cap = QGridLayout(self._bp_cap_row)
        bp_cap.setContentsMargins(0, 0, 0, 0)
        bp_cap.setHorizontalSpacing(s(10))
        bp_cap.setVerticalSpacing(s(6))
        bp_cap.addWidget(QLabel("Bore 2 tip inner Ø:"), 0, 0)
        self._bp_tip_id_spin = _make_cap_spin("tip_id", self._on_needle_changed)
        bp_cap.addWidget(self._bp_tip_id_spin, 0, 1)
        bp_cap.addWidget(QLabel("Tip outer Ø:"), 0, 2)
        self._bp_tip_od_spin = _make_cap_spin("tip_od", self._on_needle_changed)
        bp_cap.addWidget(self._bp_tip_od_spin, 0, 3)
        bp_cap.setColumnStretch(4, 1)
        lay.addWidget(self._bp_cap_row, 2, 0, 1, 6)

        lay.setColumnStretch(5, 1)
        card.setVisible(False)
        return card

    def _backpack_bore2_gauge(self) -> int | None:
        """Bore 2's gauge, or None for "same as bore 1"."""
        combo = getattr(self, "_bp_gauge_combo", None)
        return combo.currentData() if combo is not None else None

    def _capillary_geometry(self) -> dict:
        """The six live capillary values, as NeedleType/NeedleSpec kwargs."""
        tip_od = float(self._cap_tip_od_spin.value())
        return {
            "barrel_id_um": float(self._cap_barrel_id_spin.value()),
            "barrel_od_um": float(self._cap_barrel_od_spin.value()),
            "barrel_length_mm": float(self._cap_barrel_len_spin.value()),
            "tip_id_um": float(self._cap_tip_id_spin.value()),
            "tip_length_mm": float(self._cap_tip_len_spin.value()),
            "tip_od_um": (tip_od if tip_od > 0 else None),
            "tip_profile": self._cap_tip_profile_combo.currentData()
                           or TIP_PROFILE_CYLINDER,
        }

    def _refresh_needle_type_preset_combo(self, select_id: str | None = None) -> None:
        """Rebuild the preset picker; '(custom)' plus every stored needle type."""
        combo = getattr(self, "_needle_type_preset_combo", None)
        if combo is None:
            return
        combo.blockSignals(True)
        try:
            combo.clear()
            combo.addItem("(custom)", None)
            for nt in get_needle_type_store().all():
                combo.addItem(nt.label, nt.id)
            idx = combo.findData(select_id) if select_id else 0
            combo.setCurrentIndex(idx if idx >= 0 else 0)
        finally:
            combo.blockSignals(False)
        self._refresh_needle_preset_buttons()

    def _refresh_needle_preset_buttons(self) -> None:
        btn = getattr(self, "_needle_preset_del_btn", None)
        if btn is None:
            return
        nt_id = self._needle_type_preset_combo.currentData()
        nt = get_needle_type_store().get(nt_id) if nt_id else None
        btn.setEnabled(bool(nt) and not nt.builtin)

    def _on_needle_type_preset_selected(self) -> None:
        """Stamp the chosen preset's geometry onto the live spin boxes."""
        nt_id = self._needle_type_preset_combo.currentData()
        nt = get_needle_type_store().get(nt_id) if nt_id else None
        self._refresh_needle_preset_buttons()
        if nt is None:
            return
        for spin, value in (
            (self._cap_barrel_id_spin, nt.barrel_id_um),
            (self._cap_barrel_od_spin, nt.barrel_od_um),
            (self._cap_barrel_len_spin, nt.barrel_length_mm),
            (self._cap_tip_id_spin, nt.tip_id_um),
            (self._cap_tip_len_spin, nt.tip_length_mm),
            (self._cap_tip_od_spin, nt.tip_od_um or 0.0),
        ):
            spin.blockSignals(True)
            spin.setValue(float(value or 0.0))
            spin.blockSignals(False)
        combo = self._cap_tip_profile_combo
        combo.blockSignals(True)
        idx = combo.findData(nt.tip_profile)
        combo.setCurrentIndex(idx if idx >= 0 else 0)
        combo.blockSignals(False)
        self._on_needle_changed()

    def _sync_needle_preset_combo(self) -> None:
        """Flip the picker to '(custom)' as soon as the live geometry diverges
        from the selected preset, so the combo never claims a recipe the needle
        no longer matches."""
        combo = getattr(self, "_needle_type_preset_combo", None)
        if combo is None:
            return
        nt_id = combo.currentData()
        if not nt_id:
            return
        nt = get_needle_type_store().get(nt_id)
        if nt is not None and nt.matches(**self._capillary_geometry()):
            return
        combo.blockSignals(True)
        combo.setCurrentIndex(0)          # "(custom)"
        combo.blockSignals(False)
        self._refresh_needle_preset_buttons()

    def _save_current_as_needle_type(self) -> None:
        name, ok = QInputDialog.getText(
            self, "Save needle type",
            "Name for this pull recipe:",
            text=f"Capillary {self._cap_tip_id_spin.value():g} µm tip")
        if not ok or not name.strip():
            return
        name = name.strip()
        nt = NeedleType(
            id=safe_needle_type_id(name.lower().replace(" ", "-")),
            display_name=name,
            **self._capillary_geometry(),
        )
        if not get_needle_type_store().save_user(nt):
            QMessageBox.critical(self, "Save failed",
                                 f"Could not save the needle type '{name}'.")
            return
        self._refresh_needle_type_preset_combo(select_id=nt.id)

    def _delete_selected_needle_type(self) -> None:
        nt_id = self._needle_type_preset_combo.currentData()
        nt = get_needle_type_store().get(nt_id) if nt_id else None
        if nt is None or nt.builtin:
            return
        if QMessageBox.question(
                self, "Delete needle type",
                f"Delete the saved needle type '{nt.label}'?\n\n"
                "The needle currently configured keeps its geometry — a preset "
                "is only a starting point.") != QMessageBox.Yes:
            return
        get_needle_type_store().delete_user(nt_id)
        self._refresh_needle_type_preset_combo()

    def _apply_needle_type_visibility(self):
        """Show the geometry block for the selected needle type.

        Pure UI state — safe to call during ``_setup_ui`` before the rest of the
        page exists (unlike :meth:`_on_needle_type_changed`, which rebuilds the
        whole config). v7.9: it no longer CLAMPS the bore count (see below), but
        it still rebuilds the bore→pump rows, so it must run after
        ``_channel_rows_layout`` exists.
        """
        cap = self._needle_type_combo.currentData() == NEEDLE_TYPE_CAPILLARY
        self._hypo_row.setVisible(not cap)
        self._cap_row.setVisible(cap)
        # v7.9.x: the second needle's diameters follow the SAME taper — the two
        # halves of one bound assembly, not two independently-shaped needles.
        if hasattr(self, "_bp_row"):
            self._bp_row.setVisible(
                self._current_needle_form() == NEEDLE_FORM_BACKPACK)
            self._bp_hypo_row.setVisible(not cap)
            self._bp_cap_row.setVisible(cap)
        # v7.9: a pulled capillary is NO LONGER clamped to one bore — the
        # assembly FORM (how many needles are bound together) is orthogonal to
        # one bore's taper, so a backpack of two pulled capillaries is a
        # legitimate build. The count applies to either type.
        self.channels_spin.setToolTip(
            "Bores in the assembly: 1 = single needle, 2 = backpack, "
            "3 = triple. Applies to hypodermic and pulled-capillary alike.")
        self._rebuild_channel_map_rows()
        # v7.9: bore 1's read-only summary in the per-bore group mirrors THIS
        # block, so it has to follow a type switch.
        self._rebuild_bore_rows()

    def _on_needle_type_changed(self):
        """Needle-type combo changed — re-apply visibility, then rebuild."""
        self._apply_needle_type_visibility()
        self._on_needle_changed()

    # ── v7.9: assembly FORM + per-bore geometry ───────────────────────

    def _form_bore_count(self) -> int:
        """Bore count implied by the selected assembly form (1 for anything
        unrecognised — the fail-safe direction, since a single bore is what every
        legacy consumer already handles)."""
        combo = getattr(self, "_needle_form_combo", None)
        if combo is None:
            return 1
        return int(NEEDLE_FORM_BORE_COUNT.get(combo.currentData(), 1))

    def _current_needle_form(self) -> str:
        combo = getattr(self, "_needle_form_combo", None)
        return (combo.currentData() if combo is not None else None) or NEEDLE_FORM_SINGLE

    def _uniform_bores(self) -> bool:
        """True when the selected form's bores all copy the datum bore."""
        return needle_form_bores_are_uniform(self._current_needle_form())

    @staticmethod
    def _form_for_bore_count(n_bores: int, needle=None) -> str:
        """The form to show for a needle whose stored form contradicts its bores.

        Reads the answer off the BORES rather than guessing from the count, which
        matters at 2: a septum's bores are identical and a backpack's are not, so
        a legacy ``num_channels: 2`` needle (whose two bores are synthesized
        identical from the flat fields) restores as a SEPTUM and needs no second
        geometry — where calling it a backpack would demand a second diameter the
        file never had.
        """
        if n_bores >= 3:
            return NEEDLE_FORM_TRIPLE
        if n_bores == 2:
            try:
                b0, b1 = list(needle.bores_resolved())[:2]
                same = (abs(float(b0.orifice_id_um or 0.0)
                            - float(b1.orifice_id_um or 0.0)) < 1e-6
                        and abs(float(b0.od_um or 0.0)
                                - float(b1.od_um or 0.0)) < 1e-6)
            except Exception:
                same = False
            return NEEDLE_FORM_SEPTUM if same else NEEDLE_FORM_BACKPACK
        return NEEDLE_FORM_SINGLE

    def _needle_form_rule_text(self) -> str:
        """What the selected form means, in the operator's terms.

        The form is the page's one structural choice, so it has to say what it
        committed the operator to — otherwise "3 bores" and "one geometry block"
        look like a missing editor rather than a deliberate copy.
        """
        form = self._current_needle_form()
        if form == NEEDLE_FORM_BACKPACK:
            return ("<b>Backpack</b> — 2 bores. Bore 1 is configured above; "
                    "bore 2 takes its <b>diameters</b> from the second-needle "
                    "row and bore 1's length. Each bore gets its own pump below.")
        if form == NEEDLE_FORM_SEPTUM:
            return ("<b>Septum</b> — 2 bores, <b>both identical to bore 1</b>. "
                    "Only the pump (and the measured mount offset) differs per "
                    "bore.")
        if form == NEEDLE_FORM_TRIPLE:
            return ("<b>Triple</b> — 3 bores, <b>each a copy of bore 1</b>. Only "
                    "the pump (and the measured mount offset) differs per bore.")
        return ""

    def _on_needle_form_changed(self):
        """Assembly-form combo changed — resize the bore rows, then rebuild.

        The form is the ONLY writer of ``channels_spin`` (see ``_setup_ui``), so
        the count and the geometry rows can never disagree.
        """
        n = self._form_bore_count()
        self.channels_spin.blockSignals(True)
        self.channels_spin.setValue(n)
        self.channels_spin.blockSignals(False)
        # Visibility first: the backpack row is part of the geometry the rows
        # below echo, so it has to be right before the readouts are derived.
        # It rebuilds the bore→pump rows AND the per-bore rows for the new count.
        self._apply_needle_type_visibility()
        self._on_needle_changed()

    def _build_bore_group(self) -> QWidget:
        """The bore→pump table — hidden entirely for a single needle.

        v7.9.x: geometry is NOT entered here. The assembly form above decides each
        bore's geometry (a copy of bore 1, or bore 1's length with the second
        needle's diameters), so this table is one decision per bore: which pump
        pushes it. Each row echoes the resolved geometry read-only so the operator
        can see what they wired a pump to.
        """
        self._bore_group = QGroupBox("Bore → Pump Assignment")
        self._bore_group.setStyleSheet(self._group_style())
        lay = QVBoxLayout(self._bore_group)
        lay.setSpacing(s(8))

        info = QLabel(
            "One row per bore: give it a pump. Geometry comes from the assembly "
            "form above and is shown read-only — a bore cannot be sized twice. "
            "Two bores may not share a pump: one pump pushes one volume, so the "
            "second bore would be driven blind.")
        info.setWordWrap(True)
        info.setStyleSheet(f"color: {COLORS.get('subtext0', '#a6adc8')}; ")
        lay.addWidget(info)

        self._bore_rows_widget = QWidget()
        self._bore_rows_layout = QVBoxLayout(self._bore_rows_widget)
        self._bore_rows_layout.setContentsMargins(0, 0, 0, 0)
        self._bore_rows_layout.setSpacing(s(8))
        lay.addWidget(self._bore_rows_widget)

        # Mount-offset provenance. Read-only here BY DESIGN: a fused assembly's
        # rotation in the holder is arbitrary, so the offsets are a per-MOUNT
        # calibration that must be re-measured on every needle change or re-seat.
        # Typing them here (or storing them in a preset library) is the
        # CAMERA_CAL_PERSIST_STORE mistake.
        self._bore_offset_note = QLabel(
            "Mount offsets are <b>measured</b>, not typed — calibrate them on "
            "<b>Calibration → Needle Location</b>. They are a per-mount "
            "calibration (the assembly's rotation in the holder is arbitrary), so "
            "re-measure after every needle change or re-seat. A bore whose offset "
            "reads <b>not measured</b> will be positioned as if it sat exactly "
            "where bore 1 does — off target by the real bore spacing "
            "(~100–500 µm, larger than a cell).")
        self._bore_offset_note.setWordWrap(True)
        self._bore_offset_note.setStyleSheet(
            f"color: {COLORS.get('subtext0', '#a6adc8')}; ")
        lay.addWidget(self._bore_offset_note)

        self._bore_status = QLabel("")
        self._bore_status.setWordWrap(True)
        self._bore_status.setStyleSheet(f"padding: {sp(2)} {sp(4)};")
        lay.addWidget(self._bore_status)

        self._bore_group.setVisible(False)
        return self._bore_group

    def _build_bore_row(self, bore_index: int) -> dict:
        """Widgets for one bore's row: a label, a pump, and a read-only echo.

        ``bore_index`` is 0-based; the operator sees "Bore N+1" — the numbering
        ``validate()`` already uses. There are deliberately NO geometry editors
        here: the assembly form owns geometry (see :meth:`_build_bore_group`).
        """
        row = QGroupBox(f"Bore {bore_index + 1}"
                        + (" · datum" if bore_index == 0 else ""))
        row.setStyleSheet(self._group_style())
        grid = QGridLayout(row)
        grid.setHorizontalSpacing(s(10))
        grid.setVerticalSpacing(s(6))

        on_change = partial(self._on_bore_row_changed, bore_index)

        # ── line 0: label · pump ──
        grid.addWidget(QLabel("Label:"), 0, 0)
        label_edit = QLineEdit()
        label_edit.setPlaceholderText("optional, e.g. trypsin")
        label_edit.setToolTip(
            "Operator name for this bore — shown wherever a bore has to be "
            "picked (per-bore roles, logs).")
        label_edit.textChanged.connect(on_change)
        grid.addWidget(label_edit, 0, 1)

        grid.addWidget(QLabel("Pump:"), 0, 2)
        pump_combo = QComboBox()
        pump_combo.setToolTip(
            "The syringe pump that feeds this bore. Every pump is offered here — "
            "picking one enables it on the Pump tab, so the wiring is a single "
            "decision made in one place.\n"
            "Two bores cannot share a pump: one pump can only push one volume, "
            "so the second bore would be driven blind.")
        # Populated by `_refresh_bore_pump_options`; signal wired after, so the
        # populate can't re-enter the config rebuild.
        grid.addWidget(pump_combo, 0, 3)

        # ── line 1: read-only geometry echo ──
        geom_lbl = QLabel("")
        geom_lbl.setWordWrap(True)
        geom_lbl.setStyleSheet(f"color: {COLORS.get('subtext0', '#a6adc8')}; ")
        grid.addWidget(geom_lbl, 1, 0, 1, 4)

        flow_lbl = QLabel("")
        flow_lbl.setWordWrap(True)
        flow_lbl.setToolTip(
            "This BORE's own Hagen–Poiseuille flow ceiling at the water "
            "reference viscosity. It is per bore because bores differ by orders "
            "of magnitude — applying a coarse bore's ceiling to a fine bore's "
            "pump over-pressures it and shatters a pulled glass tip.")
        flow_lbl.setStyleSheet(
            f"color: {COLORS.get('green', '#a6e3a1')}; font-weight: 600;")
        grid.addWidget(flow_lbl, 2, 0, 1, 4)

        grid.setColumnStretch(3, 1)

        # Signal wired LAST: `_rebuild_bore_rows` seeds this combo right after
        # building the row, and a seed that re-entered `_on_config_changed` would
        # persist a half-built assembly.
        pump_combo.currentIndexChanged.connect(on_change)

        return {
            "widget": row, "label": label_edit, "pump": pump_combo,
            "geom": geom_lbl, "flow": flow_lbl,
        }

    def _rebuild_bore_rows(self):
        """Rebuild the bore→pump rows for the selected form.

        Each row's label + pump are PRESERVED across the rebuild, and those
        belonging to bores the new form drops are parked in ``_bore_cache`` so
        Triple → Single → Triple is non-destructive. Without that, the
        ``_on_needle_form_changed`` → ``_rebuild_config`` that follows would
        persist whatever the freshly-defaulted widgets happen to say — the same
        silent-destruction shape the bore-count spin used to have.

        v7.9.x: there is no per-row geometry to preserve any more (the form owns
        it), so what is at stake here is the WIRING — and losing a bore→pump
        binding is what leaves a fine bore's pump on a coarse bore's flow ceiling.
        """
        if not hasattr(self, "_bore_rows_layout"):
            return

        n = self._form_bore_count()

        if getattr(self, "_restoring", False):
            # A config load is authoritative: a cache entry from the OUTGOING
            # setup restored on top of it would silently graft one machine's bore
            # onto another's assembly.
            self._bore_cache.clear()
        else:
            # Snapshot every live row, then park the ones the new form drops. A
            # row the operator never touched is not worth reporting as a loss, so
            # only real content is parked.
            for k in range(len(self._bore_rows)):
                g = self._bore_row_geometry(k)
                if not self._bore_geometry_is_pristine(g):
                    self._bore_cache[k] = g
        # A bore the new form KEEPS is restored below, so its cache entry is
        # consumed; only the dropped ones stay parked (and are reported).
        for k in range(len(self._bore_rows)):
            self._bore_rows[k]["widget"].setParent(None)
            self._bore_rows[k]["widget"].deleteLater()
        self._bore_rows.clear()

        for k in range(n):
            row = self._build_bore_row(k)
            self._bore_rows_layout.addWidget(row["widget"])
            self._bore_rows.append(row)

        self._refresh_bore_pump_options()
        for k in range(n):
            cached = self._bore_cache.pop(k, None)
            if cached:
                self._set_bore_row_geometry(k, cached)

        multi = n > 1
        self._bore_group.setVisible(multi)
        self._needle_datum_note.setText(self._needle_form_rule_text())
        self._needle_datum_note.setVisible(multi)
        # The per-bore rows are a strict superset of the bore→pump rows, and on a
        # multi-bore assembly `NeedleBore.pump_id` is the authority, so showing
        # both would give the operator two pickers for one decision. The map rows
        # stay built (and mirrored) because `_rebuild_config` reads them.
        if hasattr(self, "channel_map_group"):
            self.channel_map_group.setVisible(not multi)
        self._refresh_bore_readouts()

    def _on_bore_row_changed(self, bore_index: int, *_args):
        """A per-bore widget changed."""
        if getattr(self, "_restoring", False):
            return
        # Claiming a pump here enables it. The operator's decision is "this bore
        # is fed by P2"; making them repeat it as a checkbox on another tab is how
        # a bore ends up bound to a pump that never runs — and the binding is what
        # the per-pump flow ceiling resolves through.
        self._ensure_claimed_pumps_enabled()
        # The per-bore pump combo is the authority on a multi-bore assembly, so
        # push it into the (now redundant) bore→pump rows before the rebuild
        # reads them. One-directional, single point — no sync loop.
        self._sync_channel_map_from_bores()
        self._refresh_bore_readouts()
        self._on_config_changed()

    def _ensure_claimed_pumps_enabled(self):
        """Enable every pump a bore claims — never disable one.

        Enable-only on purpose: a bore that loses its pump (or a form that shrinks)
        must not silently switch off a pump the operator configured for something
        else.
        A pump enabled without a syringe is reported by ``validate()`` as
        "enabled but no syringe", which names the one remaining action instead of
        hiding the wiring.
        """
        want = {str(row["pump"].currentData()).strip().upper()
                for row in self._bore_rows if row["pump"].currentData()}
        for pid, pw in self._pump_widgets.items():
            if pid.strip().upper() in want and not pw.enable_check.isChecked():
                pw.enable_check.blockSignals(True)
                pw.enable_check.setChecked(True)
                pw.enable_check.blockSignals(False)
                # `toggled` was blocked so this cannot re-enter the config rebuild
                # already in flight (the widget's `changed` signal lands there);
                # apply the enable's own side-effects — which sub-controls are
                # editable — explicitly instead.
                pw._update_controls()
                logger.debug("Pump %s enabled: claimed by a needle bore", pid)

    def _refresh_bore_pump_options(self):
        """Re-populate every per-bore pump combo with EVERY pump.

        Not just the enabled ones (which is what the legacy bore→pump map offers):
        the bore→pump wiring is a property of how the needle is plumbed, and the
        operator should not have to go and enable a pump on another tab before
        they are allowed to say which bore it feeds. Picking one enables it (see
        :meth:`_ensure_claimed_pumps_enabled`); until then the row is annotated so
        the pending action is visible rather than implied.

        Seeded from the bore→pump map rows so an assembly that was mapped before
        the per-bore editor existed keeps its assignments.
        """
        enabled = set(self._get_enabled_pump_ids())
        for k, row in enumerate(self._bore_rows):
            combo = row["pump"]
            current = combo.currentData()
            if not current:
                current = self._mapped_pump_for_bore(k)
            combo.blockSignals(True)
            combo.clear()
            combo.addItem("— Unassigned —", None)
            for pid in self._pump_widgets:
                combo.addItem(
                    pid if pid in enabled else f"{pid} (will be enabled)", pid)
            combo.blockSignals(False)
            if current:
                self._select_bore_pump(combo, current)

    @staticmethod
    def _select_bore_pump(combo: QComboBox, pump_id: str) -> None:
        """Select ``pump_id`` in a per-bore pump combo, adding it if unknown.

        The combo offers every pump this machine has, so the add-it branch is for a
        pump id the page does not know at all — a hand-edited or newer setup file.
        Dropping the selection there would silently forget which bore that pump
        feeds, and ``NeedleBore.pump_id`` is the AUTHORITY the per-pump flow ceiling
        resolves through, so it is kept and flagged instead: ``validate()`` then
        says "Bore N → P4 but P4 is not enabled", which names the operator's next
        action rather than losing their wiring.
        """
        idx = combo.findData(pump_id)
        if idx < 0:
            combo.blockSignals(True)
            combo.addItem(f"{pump_id} (unknown pump)", pump_id)
            idx = combo.count() - 1
            combo.blockSignals(False)
        if combo.currentIndex() != idx:
            combo.blockSignals(True)
            combo.setCurrentIndex(idx)
            combo.blockSignals(False)

    def _mapped_pump_for_bore(self, bore_index: int) -> str | None:
        """This bore's pump according to the bore→pump map rows, else the config."""
        if bore_index < len(self._channel_map_widgets):
            pid = self._channel_map_widgets[bore_index][1].currentData()
            if pid:
                return pid
        cfg = getattr(self, "_config", None)
        if cfg is not None:
            return getattr(cfg, "needle_channel_pump_map", {}).get(bore_index)
        return None

    def _sync_channel_map_from_bores(self):
        """Mirror the per-bore pump selections into the bore→pump map rows.

        On a multi-bore assembly ``NeedleBore.pump_id`` is the authority — the
        serialized ``needle_channel_pump_map`` is DERIVED from it by
        ``HardwareConfig.resolved_bore_pump_map``. Keeping the map rows in step
        means ``_rebuild_config``'s existing map capture stays truthful and the
        operator never sees the two surfaces disagree.
        """
        if len(self._bore_rows) < 2:
            return
        for k, row in enumerate(self._bore_rows):
            if k >= len(self._channel_map_widgets):
                break
            combo = self._channel_map_widgets[k][1]
            want = row["pump"].currentData()
            if combo.currentData() == want:
                continue
            idx = combo.findData(want)
            if idx >= 0:
                combo.blockSignals(True)
                combo.setCurrentIndex(idx)
                combo.blockSignals(False)
        self._update_channel_map_status()

    def _reapply_saved_bore_pumps(self):
        """Re-select each saved bore's pump once the ENABLED pump list is known.

        Ordering fix, and the reason it is a separate pass: ``_apply_config_to_ui``
        builds the bore rows in its needle section, which runs BEFORE the pump
        widgets are restored — so ``_get_enabled_pump_ids()`` was still empty, every
        per-bore pump combo held only "— Unassigned —", and
        ``_set_bore_row_geometry``'s ``findData`` dropped the saved binding without
        a word. Losing it is not cosmetic: ``NeedleBore.pump_id`` is the AUTHORITY
        on a multi-bore assembly, so the per-PUMP flow ceiling loses the one link
        that tells it which bore feeds which pump — and a fine bore's pump
        inheriting a coarse bore's ceiling over-pressures it. The legacy map rows
        never hit this because they are rebuilt after the pumps are up.
        """
        needle = getattr(getattr(self, "_config", None), "needle", None)
        for k, bore in enumerate(list(getattr(needle, "bores", None) or [])):
            if k >= len(self._bore_rows):
                break
            want = getattr(bore, "pump_id", None)
            if not want:
                continue
            self._select_bore_pump(self._bore_rows[k]["pump"], want)

    # ── per-bore row data ↔ widgets ───────────────────────────────────
    #
    # A row carries the two things that are genuinely per-bore and cannot be
    # derived: the operator's label and the pump. Geometry is NOT here — see
    # `_build_bore_group` — so there is exactly one editor for every value.

    @staticmethod
    def _bore_geometry_is_pristine(g: dict) -> bool:
        """True when a row holds nothing the operator entered."""
        if not g:
            return True
        return not g.get("label") and not g.get("pump_id")

    def _bore_row_geometry(self, bore_index: int) -> dict:
        """The live values of one row, as plain data (cacheable / comparable)."""
        if bore_index >= len(self._bore_rows):
            return {}
        row = self._bore_rows[bore_index]
        return {
            "label": row["label"].text().strip(),
            "pump_id": row["pump"].currentData(),
        }

    def _set_bore_row_geometry(self, bore_index: int, g: dict):
        """Push cached/loaded values back into a row without firing handlers."""
        if bore_index >= len(self._bore_rows) or not g:
            return
        row = self._bore_rows[bore_index]
        row["label"].blockSignals(True)
        try:
            row["label"].setText(g.get("label", "") or "")
        finally:
            row["label"].blockSignals(False)
        pump_id = g.get("pump_id")
        if pump_id:
            # Keeps a pump the enabled list has not caught up with — dropping the
            # selection there is how a saved bore→pump binding used to vanish.
            self._select_bore_pump(row["pump"], pump_id)

    def _restore_backpack_bore2(self, saved_bores: list) -> None:
        """Put a saved backpack's second-needle diameters back in their row.

        Silent on any other form: a septum/triple derives bore 2 from bore 1, so
        writing a stale gauge into this row would resurface the moment the operator
        switched to backpack — as a size they never typed.
        """
        if self._current_needle_form() != NEEDLE_FORM_BACKPACK:
            return
        if len(saved_bores) < 2:
            return
        b1 = saved_bores[1]

        combo = self._bp_gauge_combo
        combo.blockSignals(True)
        try:
            idx = combo.findData(getattr(b1, "gauge", None))
            combo.setCurrentIndex(idx if idx >= 0 else 0)
        finally:
            combo.blockSignals(False)

        for spin, value in ((self._bp_tip_id_spin,
                             getattr(b1, "tip_id_um", None)),
                            (self._bp_tip_od_spin,
                             getattr(b1, "tip_od_um", None))):
            spin.blockSignals(True)
            try:
                spin.setValue(float(value or 0.0))
            finally:
                spin.blockSignals(False)

    def _bore_zero_from_ui(self) -> NeedleBore:
        """Bore 1 — geometry from the top block, label + pump from its own row.

        The top block IS bore 1's editor: ``NeedleSpec.__post_init__`` mirrors
        ``bores[0]`` onto the flat fields, so every legacy reader of
        ``needle.id_um`` / ``cross_section_area_mm2`` keeps seeing this bore's
        real numbers.
        """
        row = self._bore_rows[0] if self._bore_rows else None
        label = row["label"].text().strip() if row else ""
        pump = row["pump"].currentData() if row else None
        if self._needle_type_combo.currentData() == NEEDLE_TYPE_CAPILLARY:
            g = self._capillary_geometry()
            b_id, b_od = g["barrel_id_um"], g["barrel_od_um"]
            return NeedleBore(
                id_um=b_id, od_um=b_od,
                wall_um=max(0.0, (b_od - b_id) / 2.0),
                length_mm=g["barrel_length_mm"], gauge=None,
                needle_type=NEEDLE_TYPE_CAPILLARY,
                tip_id_um=g["tip_id_um"], tip_length_mm=g["tip_length_mm"],
                tip_od_um=g["tip_od_um"], tip_profile=g["tip_profile"],
                pump_id=pump, label=label,
            )
        gauge = self.gauge_combo.currentData()
        spec = self._needle_catalog.get(gauge) if gauge else None
        return NeedleBore(
            id_um=spec.id_um if spec else 0.0,
            od_um=spec.od_um if spec else 0.0,
            wall_um=spec.wall_um if spec else 0.0,
            length_mm=float(self.length_combo.currentData() or 1.0) * 25.4,
            gauge=gauge, pump_id=pump, label=label,
        )

    def _bore_from_row(self, bore_index: int, datum: NeedleBore | None = None
                       ) -> NeedleBore:
        """Bore ``bore_index`` (≥ 1) — DERIVED from the datum bore + the form.

        The form decides how much of the datum carries over:

        * **septum / triple** — everything. Bores 2..N are the same needle.
        * **backpack** — everything except the DIAMETERS of bore 2, which come
          from the second-needle row. The length carries over because the two
          needles are bound side by side, and the descend is planned against the
          longest bore — so a second, independently-typed length is a Z error
          waiting to be typed, not a capability.

        Only the label and the pump come from the row itself.
        """
        g = self._bore_row_geometry(bore_index)
        base = datum if datum is not None else self._bore_zero_from_ui()
        # A copy, so a later mutation of one bore can never reach the datum.
        bore = replace(base, pump_id=g.get("pump_id"), label=g.get("label", ""),
                       offset_um=(0.0, 0.0), z_offset_mm=0.0)

        if bore_index == 1 and self._current_needle_form() == NEEDLE_FORM_BACKPACK:
            if base.needle_type == NEEDLE_TYPE_CAPILLARY:
                tip_id = float(self._bp_tip_id_spin.value())
                tip_od = float(self._bp_tip_od_spin.value())
                # A 0 reads "not entered" → keep the datum's value rather than
                # fabricating a 0 µm orifice, which validate() would report as an
                # error the operator never caused.
                if tip_id > 0:
                    bore.tip_id_um = tip_id
                if tip_od > 0:
                    bore.tip_od_um = tip_od
            else:
                gauge = self._backpack_bore2_gauge()
                spec = self._needle_catalog.get(gauge) if gauge else None
                if spec is not None:
                    bore.gauge = spec.gauge
                    bore.id_um = spec.id_um
                    bore.od_um = spec.od_um
                    bore.wall_um = spec.wall_um
        return bore

    def _bores_from_ui(self) -> list[NeedleBore] | None:
        """The assembly's bores, or None for a single needle.

        None is what keeps a single-bore needle byte-identical: ``to_dict``
        emits ``bores`` only when the list was explicitly supplied AND holds more
        than one bore.
        """
        n = self._form_bore_count()
        if n <= 1 or len(self._bore_rows) < n:
            return None
        datum = self._bore_zero_from_ui()
        bores = [datum]
        bores.extend(self._bore_from_row(k, datum) for k in range(1, n))
        # Carry the MEASURED mount offsets through the rebuild — losing them on a
        # geometry edit (or a form round-trip) would silently un-calibrate the
        # assembly, and the error is a right-distance-wrong-place miss of
        # 100–500 µm, larger than a cell.
        self._absorb_live_bore_offsets()
        for k, bore in enumerate(bores):
            if k == 0:
                continue        # datum — NeedleSpec pins bores[0] at the origin
            off, dz = self._bore_offsets.get(k, ((0.0, 0.0), 0.0))
            bore.offset_um = off
            bore.z_offset_mm = dz
        return bores

    def _absorb_live_bore_offsets(self):
        """Learn any measured offsets carried by the live needle spec.

        The calibration writes offsets onto ``config.needle.bores``; picking them
        up here means a later geometry edit rebuilds the spec WITHOUT dropping
        them, whether the calibration re-pushed the whole config or mutated it in
        place. Only non-zero values are learned — a zero is "not measured", which
        must never overwrite a real measurement.
        """
        prev = getattr(getattr(self, "_config", None), "needle", None)
        for k, b in enumerate(list(getattr(prev, "bores", None) or [])):
            try:
                off = tuple(float(v) for v in getattr(b, "offset_um", (0.0, 0.0)))
                dz = float(getattr(b, "z_offset_mm", 0.0) or 0.0)
            except (TypeError, ValueError):
                continue
            if len(off) == 2 and (off != (0.0, 0.0) or dz):
                self._bore_offsets[k] = (off, dz)

    def _bore_mount_offset(self, bore_index: int) -> tuple[tuple[float, float], float]:
        """This bore's measured mount offset — ``((0, 0), 0.0)`` = not measured."""
        if bore_index == 0:
            return ((0.0, 0.0), 0.0)
        return self._bore_offsets.get(bore_index, ((0.0, 0.0), 0.0))

    # ── per-bore readouts ─────────────────────────────────────────────

    @staticmethod
    def _bore_flow_ceiling_uL_s(bore) -> float:
        """This bore's own Hagen–Poiseuille ceiling at the water reference.

        Per BORE, never per assembly: with one pump per bore there are N
        independent pressure sources, so there is no assembly-level ceiling to
        compute. Best-effort — never raises into a config rebuild.
        """
        try:
            from SupportClasses.FlowPhysics import (
                max_safe_flow_rate_uL_s, DEFAULT_PRESSURE_LIMIT_PA,
            )
            from SupportClasses.SafetyLimits import REFERENCE_VISCOSITY_CP
            ref_ink = InkSpec(name="__reference__",
                              viscosity_cP=REFERENCE_VISCOSITY_CP)
            return float(max_safe_flow_rate_uL_s(
                bore, ref_ink, DEFAULT_PRESSURE_LIMIT_PA) or 0.0)
        except Exception as e:
            logger.debug("per-bore flow ceiling compute failed: %s", e)
            return 0.0

    def _refresh_bore_readouts(self):
        """Per-bore geometry echo, flow ceiling, measured offset and warnings."""
        if not self._bore_rows or not hasattr(self, "_bore_status"):
            return
        n = len(self._bore_rows)
        datum = None
        bores = []
        for k in range(n):
            try:
                if k == 0:
                    datum = self._bore_zero_from_ui()
                    bores.append(datum)
                else:
                    bores.append(self._bore_from_row(k, datum))
            except Exception as e:      # never break a config rebuild
                logger.debug("bore %d readout build failed: %s", k, e)
                bores.append(None)

        # Measured mount offsets are owned by the calibration, not this page.
        self._absorb_live_bore_offsets()

        uniform = self._uniform_bores()
        claimed: dict[str, list[int]] = {}
        for k, bore in enumerate(bores):
            row = self._bore_rows[k]
            if bore is None:
                row["geom"].setText("—")
                row["flow"].setText("")
                continue
            # Every row echoes the geometry it RESOLVED to. None of it is typed
            # here, so showing it is the only way the operator can confirm which
            # needle they just wired a pump to.
            summary = bore.summary_line() or "—"
            if k == 0:
                row["geom"].setText(f"Geometry (from above): {summary}")
            elif uniform:
                row["geom"].setText(f"Geometry: copy of Bore 1 — {summary}")
            else:
                row["geom"].setText(
                    f"Geometry: second needle, Bore 1's length — {summary}")
            row["geom"].setVisible(True)

            flow = self._bore_flow_ceiling_uL_s(bore)
            bits = []
            if flow > 0:
                bits.append(f"Max safe flow {flow:.3g} µL/s")
            else:
                bits.append("Max safe flow — (geometry incomplete)")
            bits.append(f"holds {bore.internal_volume_uL:.3f} µL")
            if k == 0:
                bits.append("mount offset: datum (0, 0)")
            else:
                (ox, oy), dz = self._bore_mount_offset(k)
                if ox or oy or dz:
                    bits.append(f"mount offset ({ox:+.0f}, {oy:+.0f}) µm"
                                + (f", Z {dz:+.3f} mm" if dz else ""))
                else:
                    bits.append("mount offset: not measured")
            row["flow"].setText(" · ".join(bits))

            pid = bore.pump_id
            if pid:
                claimed.setdefault(str(pid).strip().upper(), []).append(k + 1)

        # Report problems using the SAME rule the config validates with, so the
        # page and `HardwareConfig.validate()` can never disagree.
        issues: list[str] = []
        try:
            probe = NeedleSpec(needle_form=self._current_needle_form(),
                               bores=[b for b in bores if b is not None])
            issues = HardwareConfig._needle_bore_issues(probe)
        except Exception as e:
            logger.debug("per-bore validation probe failed: %s", e)
        unassigned = [k + 1 for k, b in enumerate(bores)
                      if b is not None and not b.pump_id]

        if issues:
            text = "⚠ " + " · ".join(issues)
            colour = COLORS.get("red", "#f38ba8")
        elif unassigned:
            text = ("⚠ Bore(s) without a pump: "
                    + ", ".join(str(i) for i in unassigned))
            colour = COLORS.get("yellow", "#f9e2af")
        else:
            text = f"✓ {n} bore(s) configured, each with its own pump"
            colour = COLORS.get("green", "#a6e3a1")

        # A form SHRINK hides bores rather than deleting them. This is APPENDED
        # rather than prioritised, because a data loss the operator is about to
        # save must never be crowded out by a validation warning.
        parked = sorted(k + 1 for k in self._bore_cache)
        if parked:
            text += ("\n⚠ Bore(s) " + ", ".join(str(i) for i in parked)
                     + " are hidden by the current form. Their label and pump are "
                       "remembered while this page stays open and return if you "
                       "pick the larger form again — but they are NOT saved with "
                       "the setup.")
            colour = COLORS.get("yellow", "#f9e2af")

        self._bore_status.setText(text)
        self._bore_status.setStyleSheet(
            f"color: {colour}; padding: {sp(2)} {sp(4)};")

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
        """Drive the live microscope camera to the SINGLE source of truth —
        ``camera_config.active_resolution`` (the "Microscope Camera Setup"
        block). The camera reads it as ground truth; the spec combo is only a
        fallback if the config has none."""
        mgr = getattr(self, "_camera_manager", None)
        if mgr is None:
            return
        mic_idx = self._config.camera_for_role(CameraRole.MICROSCOPE)
        if mic_idx is None or not mgr.is_running(mic_idx):
            return
        # v7.5.x: active_resolution is THE source; fall back to the combo only if
        # it is unset, so there is one authoritative resolution.
        cam_cfg = getattr(self._config, "camera_config", None)
        res = getattr(cam_cfg, "active_resolution", None) if cam_cfg else None
        if not (res and len(res) >= 2 and res[0] and res[1]):
            data = self.cam_resolution_combo.currentData()
            res = tuple(data) if data else None
        if not res:
            return
        actual = mgr.set_capture_resolution(mic_idx, int(res[0]), int(res[1]))
        logger.info(
            f"Microscope resolution (source of truth = active_resolution) "
            f"{tuple(res)} -> device adopted {actual}")

    def _on_slot_resolution_changed(self, cam_idx: int, w: int, h: int) -> None:
        """v7.5.x: the DEVICE capture resolution changed (via the camera
        settings gear on a slot preview). If it's the microscope slot, update
        the "Microscope Camera Setup" block's ``active_resolution`` + the
        resolution combo so the block reflects the TRUE current resolution — it
        was pulling a stale value, so µm/px stamping / mosaic FOV used the wrong
        pixel count (operator report #1)."""
        try:
            mic_idx = self._config.camera_for_role(CameraRole.MICROSCOPE)
        except Exception:
            mic_idx = None
        if mic_idx is None or int(cam_idx) != int(mic_idx):
            return
        try:
            cam_cfg = getattr(self._config, "camera_config", None)
            if cam_cfg is not None:
                cam_cfg.active_resolution = (int(w), int(h))
        except Exception:
            pass
        combo = getattr(self, "cam_resolution_combo", None)
        if combo is not None:
            try:
                combo.blockSignals(True)
                idx = -1
                for k in range(combo.count()):
                    d = combo.itemData(k)
                    if d and tuple(d) == (int(w), int(h)):
                        idx = k
                        break
                if idx < 0:
                    combo.addItem(f"{w} × {h}", (int(w), int(h)))
                    idx = combo.count() - 1
                combo.setCurrentIndex(idx)
            except Exception:
                pass
            finally:
                combo.blockSignals(False)
        try:
            self._update_camera_info_labels()
        except Exception:
            pass
        logger.info(f"Camera-setup block resolution synced to device: {w}×{h}")

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

        # v7.5.x: a role change also changes the slot's nominal-mount hint
        # (microscope XY-plane vs needle ±45° vs monitor), and a needle µm/px
        # calibration may have carried a fresh rotation — resync the strips.
        self._refresh_slot_rotation_displays()

    def square_up_refusal(self, cam_idx: int, running: bool) -> str:
        """Why the square-up tool cannot run on this slot, or '' if it can.

        Pure enough to test directly. Three refusals, each for a real hazard:

        * no running camera — the tool is entirely optical;
        * no measured rotation — there is nothing to aim at, and
          ``nearest_square_rotation(None)`` would throw;
        * a needle-role slot with no ``column_dir_deg``. That combination is
          the pre-v7.5.x conflated state, where ``rotation_deg`` still holds
          the ±45° MOUNT rather than the sensor roll. Reading it as a roll
          would command a 45° physical turn, which rolls the camera out of
          level and scales its column µm/px by cos 45° — the needle aligner
          would then drive to the wrong XY. Three such entries still sit in
          this machine's store (the v1.0→1.1 migration only converts
          identities that were assigned at the time).
        """
        if not running:
            return "Start the camera first — this tool watches the live feed."
        mgr = getattr(self, "_camera_manager", None)
        if mgr is None:
            return "Camera manager not available."
        try:
            theta = mgr.get_rotation_deg(cam_idx)
        except Exception:
            theta = None
        if theta is None:
            return ("Calibrate this camera's rotation first (⟳ Rotation…) — "
                    "there is no measured angle to square up against.")
        role = (self._config.camera_roles[cam_idx]
                if cam_idx < len(self._config.camera_roles)
                else CameraRole.UNASSIGNED)
        if role in (CameraRole.NEEDLE_X, CameraRole.NEEDLE_Y):
            col = None
            try:
                gcd = getattr(mgr, "get_column_dir_deg", None)
                col = gcd(cam_idx) if callable(gcd) else None
            except Exception:
                col = None
            if col is None:
                return ("Run ⟳ Rotation… on this needle camera first, so the "
                        "±45° mount direction and the sensor roll are stored "
                        "separately. Until then its rotation may still be the "
                        "legacy mount value, and squaring that up would roll "
                        "the camera out of level.")
        return ""

    def _refresh_square_up_gate(self, cam_idx: int, running: bool) -> None:
        btns = getattr(self, "_live_cam_square_btns", [])
        if cam_idx >= len(btns):
            return
        why = self.square_up_refusal(cam_idx, running)
        btns[cam_idx].setEnabled(not why)
        btns[cam_idx].setToolTip(why or (
            "Turn this camera in its mount until its measured rotation lands "
            "on 0/90/180/270°, with a live degrees-to-go readout and a "
            "translucent ghost of the target. Moves nothing."))

    def _on_square_up_mount(self, cam_idx: int):
        """v7.10: open the live mount-alignment tool for one slot."""
        from PySide6.QtWidgets import QMessageBox
        mgr = getattr(self, "_camera_manager", None)
        running = bool(mgr is not None and mgr.is_running(cam_idx))
        why = self.square_up_refusal(cam_idx, running)
        if why:
            QMessageBox.information(self, "Square up mount", why)
            return
        try:
            from gui.dialogs.camera_rotation_align_dialog import (
                CameraRotationAlignDialog, suggest_reverification)
        except Exception as e:
            QMessageBox.warning(
                self, "Square up mount",
                f"The alignment tool is unavailable: {e}")
            return
        role = (self._config.camera_roles[cam_idx]
                if cam_idx < len(self._config.camera_roles)
                else CameraRole.UNASSIGNED)
        before = None
        try:
            before = mgr.get_rotation_deg(cam_idx)
        except Exception:
            pass
        # The dialog puts a SECOND live view on this camera. On a 10 Mpx sensor
        # each view spends ~33 ms per frame in QImage.transformed, so leaving
        # the card's preview at full rate would stack them and stall the GUI.
        self._set_slot_preview_throttled(cam_idx, True)
        try:
            dlg = CameraRotationAlignDialog(
                mgr, cam_idx, role=role,
                remeasure=lambda: bool(
                    self._on_calibrate_slot_rotation(cam_idx)),
                parent=self)
            dlg.exec()
        finally:
            self._set_slot_preview_throttled(cam_idx, False)
        self._refresh_slot_rotation_displays()
        after = None
        try:
            after = mgr.get_rotation_deg(cam_idx)
        except Exception:
            pass
        if before is not None and after is not None and abs(
                wrap_deg(float(after) - float(before))) > 0.01:
            nom, delta = nearest_square_rotation(float(after))
            QMessageBox.information(
                self, "Square up mount",
                f"Cam {cam_idx + 1} rotation vs stage is now "
                f"{float(after):+.2f}° — {abs(delta):.2f}° from {nom:g}°.\n\n"
                + suggest_reverification(role))

    def _set_slot_preview_throttled(self, cam_idx: int, on: bool) -> None:
        views = getattr(self, "_live_cam_previews", None) or []
        if cam_idx < len(views) and views[cam_idx] is not None:
            fn = getattr(views[cam_idx], "set_throttled", None)
            if callable(fn):
                try:
                    fn(bool(on))
                except Exception:
                    pass

    def _slot_view_orientation_note(self, cam_idx: int) -> str:
        """A short clause describing this slot's LIVE-VIEW orientation.

        v7.16: the readout's headline figures (rotation vs stage, mirrored) are
        the MEASURED geometry the mosaic uses. The live view is now independent,
        so it gets its own words — otherwise an operator reading "Rotation vs
        stage: 180°" would reasonably conclude the feed is rotated 180°, which is
        exactly the ambiguity this split introduces.
        """
        mgr = getattr(self, "_camera_manager", None)
        if mgr is None:
            return ""
        try:
            has = getattr(mgr, "has_display_orientation", None)
            if not callable(has) or not has(cam_idx):
                return ""          # following the measurement — nothing to add
            fx, fy, rot = self._slot_view_orientation_now(cam_idx)
        except Exception:
            return ""
        parts = []
        if fx:
            parts.append("flip X")
        if fy:
            parts.append("flip Y")
        if abs(float(rot or 0.0)) > 0.05:
            parts.append(f"{float(rot):+.1f}°")
        return f"  · live view: {', '.join(parts) if parts else 'as captured'}"

    def _refresh_slot_rotation_displays(self):
        """v7.5.x: sync each slot's 'Rotation vs stage' readout with the live
        CameraManager rotation and the slot's role (which sets the
        nominal-mount hint + the Δ-from-nominal sanity figure).

        v7.16: also syncs the Flip X / Flip Y / Rotation controls, which now
        describe the LIVE VIEW (display only) rather than the measured
        orientation reported in the readout itself.
        """
        mgr = getattr(self, "_camera_manager", None)
        for i, lbl in enumerate(getattr(self, "_live_cam_rot_labels", [])):
            role = (
                self._config.camera_roles[i]
                if i < len(self._config.camera_roles)
                else CameraRole.UNASSIGNED
            )
            hint = role_rotation_hint(role)
            theta = None
            column_dir = None
            mirrored = False
            if mgr is not None:
                try:
                    theta = mgr.get_rotation_deg(i)
                except Exception:
                    theta = None
                try:
                    gcd = getattr(mgr, "get_column_dir_deg", None)
                    column_dir = gcd(i) if callable(gcd) else None
                except Exception:
                    column_dir = None
                try:
                    mirrored = bool(mgr.get_mirrored(i))
                except Exception:
                    mirrored = False
            mir_txt = "  · mirrored" if mirrored else ""
            # v7.16: the live view is a SEPARATE, display-only orientation, so
            # say what it is doing rather than let the operator assume it
            # matches the measurement (they routinely differ by 180° on a rig
            # with plate_flip_180). Named here so every branch below can append
            # it and the two can never be confused for one number.
            mir_txt += self._slot_view_orientation_note(i)
            is_needle = role in (CameraRole.NEEDLE_X, CameraRole.NEEDLE_Y)
            if is_needle and (column_dir is not None or theta is not None):
                # v7.5.x (rotated rig): a needle cam carries TWO angles — the
                # ±45° column→stage mount direction (aligner) and the small
                # display roll. Δ-vs-nominal makes sense for the mount only.
                if column_dir is not None:
                    nom, delta = nominal_rotation_delta(
                        float(column_dir), role)
                    mount_txt = (f"mount {float(column_dir):.1f}° "
                                 f"(Δ {delta:+.1f}° from nominal {nom:g}°)")
                else:
                    mount_txt = "mount: not calibrated"
                roll_txt = (f"roll {float(theta):+.1f}°"
                            if theta is not None else "roll —")
                lbl.setText(f"{mount_txt} · {roll_txt} — {hint}{mir_txt}")
                lbl.setStyleSheet(
                    f"color: {COLORS['green'] if column_dir is not None else COLORS.get('subtext0', '#a6adc8')}; "
                    f"font-size: {scaled_font_size(9)}pt;"
                )
            elif theta is None:
                lbl.setText(
                    f"Rotation vs stage: not calibrated — {hint}{mir_txt}")
                lbl.setStyleSheet(
                    f"color: {COLORS.get('subtext0', '#a6adc8')}; "
                    f"font-size: {scaled_font_size(9)}pt;"
                )
            else:
                nom, delta = nominal_rotation_delta(float(theta), role)
                lbl.setText(
                    f"Rotation vs stage: {float(theta):.1f}°  "
                    f"(Δ {delta:+.1f}° from nominal {nom:g}°) — {hint}{mir_txt}"
                )
                lbl.setStyleSheet(
                    f"color: {COLORS['green']}; "
                    f"font-size: {scaled_font_size(9)}pt;"
                )
            # v7.10: θ and column_dir are gate inputs for the square-up tool,
            # so re-evaluate it wherever they are re-read.
            try:
                self._refresh_square_up_gate(
                    i, bool(mgr is not None and mgr.is_running(i)))
            except Exception:
                pass
            # v7.16: the three controls below describe the LIVE VIEW, not the
            # measured orientation the readout above reports, so they sync from
            # display_orientation. While they synced from the measured values,
            # running the objective calibration visibly reset them.
            view_fx, view_fy, view_rot = False, False, 0.0
            if mgr is not None:
                try:
                    do = getattr(mgr, "display_orientation", None)
                    if callable(do):
                        view_fx, view_fy, view_rot = do(i)
                    else:
                        view_fx = bool(mgr.get_mirrored(i))
                        gfy = getattr(mgr, "get_flip_y", None)
                        view_fy = bool(gfy(i)) if callable(gfy) else False
                        view_rot = float(theta) if theta is not None else 0.0
                except Exception:
                    view_fx, view_fy, view_rot = False, False, 0.0
            # Sync the Flip X checkbox without re-triggering its handler.
            if i < len(getattr(self, "_live_cam_mirror_checks", [])):
                cb = self._live_cam_mirror_checks[i]
                if cb.isChecked() != bool(view_fx):
                    cb.blockSignals(True)
                    cb.setChecked(bool(view_fx))
                    cb.blockSignals(False)
            if i < len(getattr(self, "_live_cam_flip_y_checks", [])):
                fcb = self._live_cam_flip_y_checks[i]
                if fcb.isChecked() != bool(view_fy):
                    fcb.blockSignals(True)
                    fcb.setChecked(bool(view_fy))
                    fcb.blockSignals(False)
            if i < len(getattr(self, "_live_cam_rot_spins", [])):
                sp = self._live_cam_rot_spins[i]
                cur = float(view_rot or 0.0)
                if abs(sp.value() - cur) > 1e-6:
                    sp.blockSignals(True)
                    sp.setValue(cur)
                    sp.blockSignals(False)
            # v7.5.x: apply the unified orientation to the slot's live PREVIEW so
            # the operator SEES the corrected view (idempotent — no-op if same).
            self._push_slot_view_orientation(i)

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

    def hideEvent(self, event):
        """v7.16: a debounced crop write must not outlive the page.

        Navigating away is the ordinary way this page stops being visible, and
        a crop that was applied but never reached disk would come back wrong on
        the next launch — the failure mode this store exists to prevent.
        """
        try:
            self._flush_crop_persist()
        except Exception:
            pass
        super().hideEvent(event)

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
                    # v7.10: show the CORRECTED (un-mirrored, upright) view.
                    # This is the surface where the operator sets the mirror /
                    # flip-Y / rotation for this slot, and it was the ONE feed
                    # still showing raw pixels — so ticking "Mirrored view"
                    # changed the mosaic and every other page's feed while the
                    # preview six inches away did not move. auto_orient re-reads
                    # the manager each frame, so a flip is visible immediately.
                    auto_orient=True,
                    parent=holder,
                )
                holder.layout().addWidget(fv, stretch=1)
                try:
                    fv.resolution_changed.connect(
                        self._on_slot_resolution_changed)
                except Exception:
                    pass
                self._live_cam_previews[i] = fv
                # v7.5.x: image-correction sliders live in the controls column
                # (their own holder) — NOT inside the preview holder — so the
                # live feed keeps the full preview area.
                if (i < len(self._live_cam_correction)
                        and self._live_cam_correction[i] is None
                        and i < len(self._live_cam_correction_holders)):
                    strip = self._build_correction_strip(i)
                    self._live_cam_correction_holders[i].layout().addWidget(strip)
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
            # v7.5.x: rotation calibration needs a live feed (the stage-motion
            # dialog watches the image); the controller gate is re-checked in
            # the click handler with a clear message.
            if i < len(getattr(self, "_live_cam_rot_btns", [])):
                self._live_cam_rot_btns[i].setEnabled(running)
            # v7.10: the square-up tool needs a live feed AND an existing
            # rotation to aim at. Refreshed from the SAME places as rot_btn —
            # a gate evaluated once at page build and never re-run is exactly
            # the v7.10 bore-calibration bug.
            self._refresh_square_up_gate(i, running)
            # v7.5.x: the flip flags + custom rotation are declarations (no live
            # feed needed) — enabled once the slot has a source to persist to.
            if i < len(getattr(self, "_live_cam_mirror_checks", [])):
                self._live_cam_mirror_checks[i].setEnabled(has_source)
            if i < len(getattr(self, "_live_cam_flip_y_checks", [])):
                self._live_cam_flip_y_checks[i].setEnabled(has_source)
            if i < len(getattr(self, "_live_cam_rot_spins", [])):
                self._live_cam_rot_spins[i].setEnabled(has_source)
            # v7.16: crop is likewise a declaration — the size spin follows the
            # checkbox so a disabled crop cannot be silently "sized".
            if i < len(getattr(self, "_live_cam_crop_checks", [])):
                self._live_cam_crop_checks[i].setEnabled(has_source)
            self._sync_crop_spin_enabled(i)
        # Rotation values + mirror flag restore on source assignment — keep the
        # readouts/checkboxes in sync with the manager on the same triggers.
        self._refresh_slot_rotation_displays()
        self._refresh_slot_crop_displays()

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
                                 rotation_deg: float | None = None,
                                 column_dir_deg: float | None = None,
                                 resolution: tuple[int, int] | None = None):
        """v7.3.3/v7.4.x: Apply a calibrated µm/px from any source.

        Writes through to `CameraManager.set_um_per_px` (the canonical
        live value) and refreshes the role-derived displays so the
        needle card's readout tracks the change. The microscope's
        µm/px is sourced from `ObjectiveCalibrationStore` instead and
        does not flow through here.

        v7.5.x: ``rotation_deg`` is the camera's DISPLAY orientation (for
        the needle side cams: the small sensor roll — deviation from
        parallel). ``column_dir_deg`` is the column→stage MOUNT direction
        (±45° about +X on the rotated rig), consumed only by the
        needle-centering aligner. None leaves either untouched (e.g. the
        microscope objective path, which measures neither).

        ``resolution`` is the (w, h) frame size the µm/px was measured at.
        µm/px ∝ 1/frame_width, so without it the value cannot be rescaled when
        the camera later captures at a different resolution — and both the live
        manager and the store treat it as unknown rather than assume it is valid
        at whatever width happens to be running.
        """
        mgr = getattr(self, "_camera_manager", None)
        if mgr is not None:
            # Each push is guarded SEPARATELY: these are three independent
            # quantities, and one failing must not silently drop the others.
            # (Sharing one try meant a µm/px push failure also skipped
            # ``column_dir_deg`` — the value the two-camera needle aligner
            # REFUSES to run without.)
            try:
                mgr.set_um_per_px(cam_idx, value, resolution=resolution)
            except TypeError:
                # Manager predating the resolution kwarg.
                try:
                    mgr.set_um_per_px(cam_idx, value)
                except Exception as exc:
                    logger.debug(f"set_um_per_px({cam_idx}, {value}) — {exc}")
            except Exception as exc:
                logger.debug(f"set_um_per_px({cam_idx}, {value}) — {exc}")
            if rotation_deg is not None:
                try:
                    mgr.set_rotation_deg(cam_idx, rotation_deg)
                except Exception as exc:
                    logger.debug(f"set_rotation_deg({cam_idx}) — {exc}")
            if column_dir_deg is not None:
                try:
                    scd = getattr(mgr, "set_column_dir_deg", None)
                    if callable(scd):
                        scd(cam_idx, column_dir_deg)
                except Exception as exc:
                    logger.debug(f"set_column_dir_deg({cam_idx}) — {exc}")
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
                    key, float(value), rotation_deg=rotation_deg, name=name,
                    column_dir_deg=column_dir_deg,
                    um_per_px_resolution=resolution)
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
        # v7.16: the crop is restored BEFORE the no-entry early-return, and is
        # pushed even when there is no entry at all. The CameraWidget for a slot
        # persists across source changes, so a camera with no stored crop must
        # actively CLEAR whatever the previously-assigned camera left behind —
        # otherwise reassigning a slot silently crops a camera that should see
        # its full sensor.
        try:
            from SupportClasses.CameraCrop import CameraCrop
            scr = getattr(mgr, "set_crop", None)
            if callable(scr):
                scr(cam_idx, CameraCrop.from_dict((entry or {}).get("crop")))
                self._sync_crop_controls(cam_idx)
        except Exception as exc:
            logger.debug(f"restore crop slot {cam_idx}: {exc}")
        # v7.16: the LIVE-VIEW orientation is restored on the SAME terms as the
        # crop — before the no-entry early-return, and pushed even with nothing
        # stored. A slot's CameraWidget outlives the source assigned to it, so a
        # camera with no view preference must actively hand the view back to the
        # measurement; otherwise reassigning a slot leaves the new camera showing
        # the previous one's rotation.
        try:
            vo = None
            if entry:
                from SupportClasses.CameraCalibrationStore import get_store as _gs
                vo = _gs().get_view_orientation(identity[0])
            if vo is None:
                cdo = getattr(mgr, "clear_display_orientation", None)
                if callable(cdo):
                    cdo(cam_idx)
            else:
                sdo = getattr(mgr, "set_display_orientation", None)
                if callable(sdo):
                    sdo(cam_idx, bool(vo.get("flip_x", False)),
                        bool(vo.get("flip_y", False)),
                        float(vo.get("rotation_deg", 0.0) or 0.0))
        except Exception as exc:
            logger.debug(f"restore view orientation slot {cam_idx}: {exc}")
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
        # v7.5.x: restore the camera→stage rotation independent of µm/px — the
        # mount orientation is calibrated on its own ("Calibrate orientation…")
        # and must load even for a camera that has no µm/px value yet.
        rot = entry.get("rotation_deg")
        if rot is not None:
            try:
                mgr.set_rotation_deg(cam_idx, float(rot))
            except Exception as exc:
                logger.debug(f"restore rotation slot {cam_idx}: {exc}")
        # v7.5.x (rotated rig): restore the needle-aligner mount direction —
        # a separate quantity from the display rotation above.
        cd = entry.get("column_dir_deg")
        if cd is not None:
            try:
                scd = getattr(mgr, "set_column_dir_deg", None)
                if callable(scd):
                    scd(cam_idx, float(cd))
            except Exception as exc:
                logger.debug(f"restore column dir slot {cam_idx}: {exc}")
        # v7.5.x: restore the mirrored-view (flip X) flag (also independent of
        # µm/px).
        try:
            mgr.set_mirrored(cam_idx, bool(entry.get("mirrored", False)))
        except Exception as exc:
            logger.debug(f"restore mirror slot {cam_idx}: {exc}")
        # v7.5.x: restore the flip-Y flag.
        try:
            sfy = getattr(mgr, "set_flip_y", None)
            if callable(sfy):
                sfy(cam_idx, bool(entry.get("flip_y", False)))
        except Exception as exc:
            logger.debug(f"restore flip_y slot {cam_idx}: {exc}")
        if entry.get("um_per_px") is None:
            return False
        try:
            # v7.5.x: restore the µm/px WITH the resolution it was measured at.
            # Omitting it (as this did before) left CameraManager with no
            # calibration resolution, so effective_um_per_px degraded to a
            # passthrough and every restored camera scaled its mosaic by the
            # raw value — a 2x error for an objective calibrated at 2048 px and
            # running at 1024. None = genuinely unknown (pre-1.2 entry the
            # migration could not recover) and stays unknown, not assumed.
            res = None
            try:
                res = get_store().get_um_per_px_resolution(identity[0])
            except Exception:
                res = None
            mgr.set_um_per_px(
                cam_idx, float(entry["um_per_px"]), resolution=res)
            if res is None:
                logger.warning(
                    f"Camera {cam_idx + 1} µm/px restored WITHOUT a measurement "
                    f"resolution — it cannot be rescaled if the capture "
                    f"resolution changes. Re-run the camera calibration.")
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
                # v7.5.x: ALWAYS drive the microscope to active_resolution (the
                # single source of truth) on start, so the camera reads it as
                # ground truth. Previously this was gated on hw_controls having
                # no resolution, which let a stale per-identity resolution win
                # and diverge from the setup block.
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
            # v7.5.x: the MICROSCOPE resolution has ONE source of truth
            # (active_resolution), applied on start — don't let a per-identity
            # hw_controls resolution compete for it (non-microscope cameras keep
            # their per-identity resolution).
            try:
                is_mic = (cam_idx
                          == self._config.camera_for_role(CameraRole.MICROSCOPE))
            except Exception:
                is_mic = False
            if not is_mic:
                try:
                    mgr.set_capture_resolution(cam_idx, int(res[0]), int(res[1]))
                except Exception as exc:
                    logger.debug(f"restore resolution failed: {exc}")
        # v7.13 — Andor sensor-quality features, BEFORE the exposure restore
        # (v7.13.x: the achievable exposure range depends on the readout rate
        # and gain mode, so the saved exposure must be applied against the
        # constraint set it was saved UNDER, not the open-time defaults) and
        # BEFORE the display-scaling block (the manual black/white levels are
        # raw counts whose meaning depends on the bit depth the gain mode
        # selects, so the stored levels must be the LAST thing applied).
        # Within this block: gain mode first (it constrains bit depth and the
        # legal readout rates), then readout rate, then the booleans.
        if hasattr(mgr, "set_hw_andor_feature"):
            if isinstance(hw.get("andor_gain_mode"), str):
                mgr.set_hw_andor_feature(cam_idx, "andor_gain_mode",
                                         hw["andor_gain_mode"])
            if isinstance(hw.get("andor_readout_rate"), str):
                mgr.set_hw_andor_feature(cam_idx, "andor_readout_rate",
                                         hw["andor_readout_rate"])
            for key in ("andor_sensor_cooling", "andor_noise_filter",
                        "andor_blemish_correction"):
                if isinstance(hw.get(key), bool):
                    mgr.set_hw_andor_feature(cam_idx, key, hw[key])
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
        # Andor (Zyla) display scaling. Auto flag FIRST — turning auto off
        # seeds the levels from the last auto frame, so the stored manual
        # levels must be applied after it to win.
        ascale = hw.get("andor_auto_scale")
        if isinstance(ascale, bool) and hasattr(mgr, "set_hw_andor_auto_scale"):
            mgr.set_hw_andor_auto_scale(cam_idx, ascale)
        if hw.get("andor_scale_lo") is not None and hasattr(mgr, "set_hw_andor_scale_lo"):
            mgr.set_hw_andor_scale_lo(cam_idx, hw["andor_scale_lo"])
        if hw.get("andor_scale_hi") is not None and hasattr(mgr, "set_hw_andor_scale_hi"):
            mgr.set_hw_andor_scale_hi(cam_idx, hw["andor_scale_hi"])

    def _on_calibrate_slot_rotation(self, cam_idx: int):
        """v7.5.x: measure THIS slot's camera rotation relative to the stage.

        Launches the stage-motion PixelCalibrationDialog and commits ONLY
        the measured orientation — the camera's µm/px is deliberately
        untouched (the needle cards / objective calibration own that, with
        their own commit policies). The nominal mount per role: microscope
        views along Z so its rotation is in the stage XY plane; the needle
        side cameras sit symmetric about stage +X at ±45°; the monitor
        overview camera is nominally axis-aligned.

        v7.5.x (rotated rig): for a NEEDLE-role slot the measured stage
        direction is the mount direction (→ ``column_dir_deg``, aligner
        only) and only the sensor roll (deviation from parallel) becomes
        the display ``rotation_deg`` — storing the ±45° mount as the
        display rotation was what tilted the live view.

        v7.10: returns True only when a rotation was actually COMMITTED. The
        square-up tool re-uses this as its confirmation step and must not
        report success when the operator cancelled or the measurement was
        refused — this handler returns early on five separate paths.
        """
        from gui.dialogs.pixel_calibration_dialog import PixelCalibrationDialog
        from PySide6.QtWidgets import QDialog, QMessageBox

        mgr = getattr(self, "_camera_manager", None)
        ctrl = getattr(self, "_controller", None)
        if mgr is None:
            QMessageBox.warning(
                self, "Calibrate rotation", "Camera manager not available.")
            return False
        if ctrl is None or not getattr(ctrl, "xy_stage", None):
            QMessageBox.warning(
                self, "Calibrate rotation",
                "Stage controller not connected — rotation calibration moves "
                "the stage to measure the camera's orientation. Connect "
                "hardware first.",
            )
            return False
        if not mgr.is_running(cam_idx):
            QMessageBox.warning(
                self, "Calibrate rotation",
                f"Start Cam {cam_idx + 1} before calibrating so the dialog "
                "can watch the live feed.",
            )
            return False

        role = (
            self._config.camera_roles[cam_idx]
            if cam_idx < len(self._config.camera_roles)
            else CameraRole.UNASSIGNED
        )
        is_needle = role in (CameraRole.NEEDLE_X, CameraRole.NEEDLE_Y)
        # v7.10: the role decides whether the Z leg is offered, so it has to be
        # resolved BEFORE the dialog is built rather than after it returns.
        dlg = PixelCalibrationDialog(mgr, ctrl, cam_idx=cam_idx, parent=self,
                                     needle_mode=is_needle)
        if dlg.exec() != QDialog.DialogCode.Accepted:
            return False
        rotation_deg = dlg.result_rotation_deg
        axes_only = getattr(dlg, "result_needle_axes", None)
        if rotation_deg is None and is_needle and axes_only is not None:
            # v7.10: a Z-ONLY run. It fully determines the roll, the scale and
            # the Z direction; only the aligner's mount direction needs an XY
            # move, so any previously measured one is left untouched rather
            # than the whole result being thrown away.
            self._apply_slot_rotation(cam_idx, -float(axes_only.roll_deg))
            self._apply_slot_z_row_sign(cam_idx, axes_only.z_row_sign)
            if dlg.result_um_per_px:
                self.set_calibrated_um_per_px(
                    cam_idx, float(dlg.result_um_per_px),
                    resolution=self._live_capture_resolution(cam_idx))
            QMessageBox.information(
                self, "Calibrate rotation",
                f"Cam {cam_idx + 1} measured from the Z leg alone:\n\n"
                f"• sensor roll {-float(axes_only.roll_deg):+.2f}°\n"
                f"• µm/px {float(axes_only.um_per_px):.4f}\n"
                f"• moving the needle up "
                f"{'decreases' if axes_only.z_row_sign > 0 else 'increases'} "
                f"the image row\n\n"
                "The live view stays at a fixed quarter-turn, so the roll above "
                "is the residual still to be taken out by turning the camera in "
                "its mount. The column→stage mount direction was NOT changed — "
                "that needs an XY leg.",
            )
            return True
        if rotation_deg is None:
            QMessageBox.information(
                self, "Calibrate rotation",
                "No rotation was measured (move too small / low confidence). "
                "Try a larger stage move along a clear feature.",
            )
            return False
        if is_needle:
            # v7.5.x (rotated rig): for a needle side cam the measured stage
            # direction is the ±45° MOUNT direction — aligner-only. Only the
            # sensor roll (deviation of the measured vector from parallel)
            # becomes the display rotation, so the live view stays level.
            roll = dlg.result_view_roll_deg
            self._apply_slot_rotation(
                cam_idx, float(roll) if roll is not None else 0.0)
            self._apply_slot_column_dir(cam_idx, float(rotation_deg))
            axes = getattr(dlg, "result_needle_axes", None)
            self._apply_slot_z_row_sign(
                cam_idx, getattr(axes, "z_row_sign", None))
            nom, delta = nominal_rotation_delta(float(rotation_deg), role)
            extra = ""
            if axes is None:
                extra = ("\n\n⚠ The Z leg was not measured, so the roll above "
                         "is inferred from the XY move rather than from a known "
                         "vertical, and the Z direction still comes from the "
                         "Invert Z checkbox.")
            else:
                extra = (f"\n\nZ leg: µm/px {axes.um_per_px:.4f} "
                         f"(the XY move alone read "
                         f"{axes.um_per_px_lateral:.4f}), axes "
                         f"{90 + axes.orthogonality_err_deg:.1f}° apart, "
                         f"moving the needle up "
                         f"{'decreases' if axes.z_row_sign > 0 else 'increases'}"
                         f" the image row.")
            QMessageBox.information(
                self, "Calibrate rotation",
                f"Cam {cam_idx + 1} mount direction: "
                f"{float(rotation_deg):.1f}° "
                f"(Δ {delta:+.1f}° from the nominal {nom:g}° mount) · "
                f"view roll {0.0 if roll is None else float(roll):+.1f}°.\n\n"
                f"This camera {role_rotation_hint(role)}. The mount direction "
                "drives the needle-centering math; only the roll tilts the "
                "displayed view." + extra,
            )
            return True
        self._apply_slot_rotation(cam_idx, float(rotation_deg))
        nom, delta = nominal_rotation_delta(float(rotation_deg), role)
        QMessageBox.information(
            self, "Calibrate rotation",
            f"Cam {cam_idx + 1} rotation vs stage: {float(rotation_deg):.1f}° "
            f"(Δ {delta:+.1f}° from the nominal {nom:g}° mount).\n\n"
            f"This camera {role_rotation_hint(role)}.",
        )
        return True

    def _apply_slot_rotation(self, cam_idx: int, rotation_deg: float):
        """v7.5.x: commit a measured camera→stage rotation for a slot.

        Pushes it live (``CameraManager.set_rotation_deg`` — corrects the
        click→stage mapping immediately) and persists it rotation-only per
        device identity (``CameraCalibrationStore.set_rotation`` preserves
        any µm/px / image-correction / hw-control siblings). When the slot
        holds the MICROSCOPE role, the fresh rotation is also synced into
        every per-objective calibration via the objective card, so a later
        objective swap can't push a stale rotation back over it.
        """
        mgr = getattr(self, "_camera_manager", None)
        if mgr is None:
            return
        try:
            mgr.set_rotation_deg(cam_idx, float(rotation_deg))
        except Exception as exc:
            logger.debug(f"slot rotation: push to manager — {exc}")
        identity = None
        try:
            identity = mgr.camera_identity(cam_idx)
        except Exception:
            identity = None
        if identity is not None and identity[0]:
            try:
                from SupportClasses.CameraCalibrationStore import get_store
                get_store().set_rotation(
                    identity[0], float(rotation_deg),
                    name=(identity[1] if len(identity) > 1 else ""))
            except Exception as exc:
                logger.warning(f"slot rotation: store write failed — {exc}")
        if (cam_idx == self._config.camera_for_role(CameraRole.MICROSCOPE)
                and hasattr(self, "_objective_cal_card")):
            try:
                self._objective_cal_card.adopt_camera_rotation(
                    float(rotation_deg))
            except Exception as exc:
                logger.debug(f"slot rotation: objective sync — {exc}")
        if hasattr(self, "_needle_cards"):
            self._refresh_role_derived_displays()  # also resyncs the strips
        else:
            self._refresh_slot_rotation_displays()  # pushes the preview orient
        logger.info(
            f"Camera {cam_idx + 1} rotation vs stage set to "
            f"{float(rotation_deg):.2f}° "
            f"(identity={identity[0] if identity else '?'})"
        )

    def _live_capture_resolution(self, cam_idx: int):
        """The slot's ACTUAL captured (w, h), or None.

        µm/px must be stamped with the resolution it was measured at or it
        cannot be rescaled when the camera later runs at a different one.
        Reads the live frame (authoritative) rather than a configured value.
        """
        mgr = getattr(self, "_camera_manager", None)
        try:
            frame = mgr.cameras[cam_idx].get_current_frame()
            if frame is not None and getattr(frame, "shape", None):
                return (int(frame.shape[1]), int(frame.shape[0]))
        except Exception:
            pass
        return None

    def _apply_slot_z_row_sign(self, cam_idx: int, z_row_sign) -> None:
        """v7.10: persist a MEASURED needle-camera Z→image-row sign.

        ``None`` (no Z leg measured) leaves any stored value untouched, so a
        camera calibrated the old way keeps behaving exactly as before and the
        manual *Invert Z* checkbox still governs it.
        """
        if z_row_sign is None:
            return
        mgr = getattr(self, "_camera_manager", None)
        if mgr is None:
            return
        try:
            identity = mgr.camera_identity(cam_idx)
        except Exception:
            identity = None
        if not identity or not identity[0]:
            return
        try:
            from SupportClasses.CameraCalibrationStore import get_store
            get_store().set_z_row_sign(
                identity[0], float(z_row_sign),
                name=(identity[1] if len(identity) > 1 else ""))
        except Exception as exc:
            logger.warning(f"slot Z-row sign: store write failed — {exc}")

    def _apply_slot_column_dir(self, cam_idx: int, column_dir_deg: float):
        """v7.5.x (rotated rig): commit a needle camera's measured column→stage
        MOUNT direction (deg CCW from stage +X).

        Pushes it live (``CameraManager.set_column_dir_deg`` — the
        needle-centering aligner reads it) and persists per device identity
        (``CameraCalibrationStore.set_column_dir``, preserving all siblings).
        Never touches the display orientation — that is ``rotation_deg``.
        """
        mgr = getattr(self, "_camera_manager", None)
        if mgr is None:
            return
        try:
            scd = getattr(mgr, "set_column_dir_deg", None)
            if callable(scd):
                scd(cam_idx, float(column_dir_deg))
        except Exception as exc:
            logger.debug(f"slot column dir: push to manager — {exc}")
        identity = None
        try:
            identity = mgr.camera_identity(cam_idx)
        except Exception:
            identity = None
        if identity is not None and identity[0]:
            try:
                from SupportClasses.CameraCalibrationStore import get_store
                get_store().set_column_dir(
                    identity[0], float(column_dir_deg),
                    name=(identity[1] if len(identity) > 1 else ""))
            except Exception as exc:
                logger.warning(f"slot column dir: store write failed — {exc}")
        if hasattr(self, "_needle_cards"):
            self._refresh_role_derived_displays()
        else:
            self._refresh_slot_rotation_displays()
        logger.info(
            f"Camera {cam_idx + 1} column→stage mount direction set to "
            f"{float(column_dir_deg):.2f}° "
            f"(identity={identity[0] if identity else '?'})"
        )

    def _push_slot_view_orientation(self, cam_idx: int) -> None:
        """v7.5.x: mirror/rotate the slot's live PREVIEW (CameraFeedView) so the
        operator SEES the corrected (un-mirrored, upright) view the moment they
        set the flag. The raw frame is unchanged — CameraFeedView flips/rotates
        only the shown pixmap and inverts clicks back to raw coords."""
        previews = getattr(self, "_live_cam_previews", None)
        if not previews or cam_idx < 0 or cam_idx >= len(previews):
            return
        view = previews[cam_idx]
        if view is None or not hasattr(view, "set_view_orientation"):
            return
        mgr = getattr(self, "_camera_manager", None)
        mir, fy, rot = False, False, 0.0
        if mgr is not None:
            try:
                # v7.16: the LIVE-VIEW orientation, not the measured one — this
                # preview is the surface the Flip/Rotation controls below it
                # edit, so it must show what they set and must NOT be rewritten
                # by the objective calibration.
                do = getattr(mgr, "display_orientation", None)
                if callable(do):
                    mir, fy, rot = do(cam_idx)
                else:
                    fo = getattr(mgr, "full_orientation", None)
                    if callable(fo):
                        mir, fy, rot = fo(cam_idx)
                    else:
                        vo = getattr(mgr, "view_orientation", None)
                        if callable(vo):
                            mir, rot = vo(cam_idx)
            except Exception:
                mir, fy, rot = False, False, 0.0
        try:
            view.set_view_orientation(mir, rot, fy)
        except Exception as exc:
            logger.debug(f"slot preview orientation ({cam_idx}): {exc}")

    def _on_toggle_mirror(self, cam_idx: int, checked: bool):
        """v7.5.x: user toggled the slot's 'Flip X axis' checkbox."""
        self._apply_slot_mirror(cam_idx, bool(checked))

    def _slot_view_orientation_now(self, cam_idx: int
                                   ) -> tuple[bool, bool, float]:
        """This slot's current LIVE-VIEW orientation ``(flip_x, flip_y, rot)``.

        Read from the manager (which falls back to the measured orientation when
        the operator has set no preference) rather than from the widgets, so a
        partial commit starts from what the feed is actually showing.
        """
        mgr = getattr(self, "_camera_manager", None)
        if mgr is not None:
            try:
                do = getattr(mgr, "display_orientation", None)
                if callable(do):
                    fx, fy, rot = do(cam_idx)
                    return (bool(fx), bool(fy), float(rot or 0.0))
            except Exception as exc:
                logger.debug(f"slot view orientation read ({cam_idx}): {exc}")
        return (False, False, 0.0)

    def _commit_slot_view_orientation(
            self, cam_idx: int, *, flip_x: Optional[bool] = None,
            flip_y: Optional[bool] = None, rot: Optional[float] = None) -> None:
        """THE one place this slot's LIVE-VIEW orientation is committed.

        v7.16. These three controls used to write the MEASURED camera→stage
        orientation (``set_mirrored`` / ``set_flip_y`` / ``set_rotation``) — the
        very numbers ``MosaicBuilder._orient_tile`` and ``pixel_to_stage_offset``
        consume. That gave ONE quantity TWO writers: the operator here, and
        ``derive_camera_stage_orientation`` inside the objective/mosaic
        calibration. The measurement won, so the reported workflow — set the live
        view up, then calibrate — always ended with the view flipped back.

        They are now separate. This writes **display only**: nothing here changes
        where a click lands or how a mosaic tile is placed. The measured
        orientation keeps its own controls (⟳ Rotation…, ⊾ Square up mount…, the
        objective calibration) and its own readout above.

        Persisted as one triple because it is one decision — "this is how I want
        the feed to look" — and a half-applied view is worse than either half;
        unspecified fields keep their current value.
        """
        mgr = getattr(self, "_camera_manager", None)
        if mgr is None:
            return
        cur_fx, cur_fy, cur_rot = self._slot_view_orientation_now(cam_idx)
        fx = cur_fx if flip_x is None else bool(flip_x)
        fy = cur_fy if flip_y is None else bool(flip_y)
        rr = cur_rot if rot is None else float(rot)
        try:
            sdo = getattr(mgr, "set_display_orientation", None)
            if callable(sdo):
                sdo(cam_idx, fx, fy, rr)
        except Exception as exc:
            logger.debug(f"slot view orientation: push to manager — {exc}")
        identity = None
        try:
            identity = mgr.camera_identity(cam_idx)
        except Exception:
            identity = None
        if identity is not None and identity[0]:
            try:
                from SupportClasses.CameraCalibrationStore import get_store
                get_store().set_view_orientation(
                    identity[0], rr, fx, fy,
                    name=(identity[1] if len(identity) > 1 else ""))
            except Exception as exc:
                logger.warning(
                    f"slot view orientation: store write failed — {exc}")
        # Refresh the readout + controls AND re-orient the live preview so the
        # operator sees the change immediately (via the shared
        # _refresh_slot_rotation_displays → _push_slot_view_orientation).
        self._refresh_slot_rotation_displays()
        logger.info(
            f"Camera {cam_idx + 1} live-view orientation set to "
            f"flip_x={fx} flip_y={fy} rot={rr:.1f}° (display only; "
            f"identity={identity[0] if identity else '?'})")

    def _apply_slot_mirror(self, cam_idx: int, mirrored: bool):
        """v7.5.x: commit a slot's horizontal-flip (Flip X) LIVE-VIEW flag.

        v7.16: display only — see :meth:`_commit_slot_view_orientation`. This
        used to write the measured ``mirrored`` (mosaic + click mapping), which
        the objective calibration then overwrote.
        """
        self._commit_slot_view_orientation(cam_idx, flip_x=bool(mirrored))

    def _on_toggle_flip_y(self, cam_idx: int, checked: bool):
        """v7.5.x: user toggled the slot's 'Flip Y axis' checkbox."""
        self._apply_slot_flip_y(cam_idx, bool(checked))

    def _apply_slot_flip_y(self, cam_idx: int, flip_y: bool):
        """v7.5.x: commit a slot's vertical-flip (Flip Y) LIVE-VIEW flag.
        v7.16: display only (sibling of Flip X)."""
        self._commit_slot_view_orientation(cam_idx, flip_y=bool(flip_y))

    # ── v7.16: per-slot square crop ───────────────────────────────

    def _slot_crop_from_ui(self, cam_idx: int):
        """Build a ``CameraCrop`` from this slot's checkbox + size spin."""
        from SupportClasses.CameraCrop import CameraCrop, MODE_NONE, MODE_SQUARE
        on = False
        pct = 100.0
        off_x = off_y = 0.0
        checks = getattr(self, "_live_cam_crop_checks", [])
        spins = getattr(self, "_live_cam_crop_spins", [])
        xs = getattr(self, "_live_cam_crop_off_x_spins", [])
        ys = getattr(self, "_live_cam_crop_off_y_spins", [])
        if cam_idx < len(checks):
            on = bool(checks[cam_idx].isChecked())
        if cam_idx < len(spins):
            pct = float(spins[cam_idx].value())
        if cam_idx < len(xs):
            off_x = float(xs[cam_idx].value()) / 100.0
        if cam_idx < len(ys):
            off_y = float(ys[cam_idx].value()) / 100.0
        return CameraCrop(mode=(MODE_SQUARE if on else MODE_NONE),
                          scale=pct / 100.0,
                          offset_x=off_x, offset_y=off_y)

    def _on_toggle_crop(self, cam_idx: int, checked: bool):
        """v7.16: user toggled the slot's 'Square crop' checkbox.

        Persisted immediately — a checkbox is one discrete decision, so there is
        nothing to coalesce, and turning the crop OFF is the action an operator
        takes when something looks wrong. That must reach disk even if the app
        is closed (or dies) a moment later.
        """
        self._sync_crop_spin_enabled(cam_idx)
        self._apply_slot_crop(cam_idx, self._slot_crop_from_ui(cam_idx),
                              persist_now=True)

    def _on_crop_scale_spin(self, cam_idx: int, value: float):
        """v7.16: user changed the slot's crop size (% of the short side).

        Only commits while the crop is ON — otherwise dialling the size of a
        disabled crop would write a store entry (and log a change) for
        something the operator has not switched on.
        """
        checks = getattr(self, "_live_cam_crop_checks", [])
        if cam_idx < len(checks) and not checks[cam_idx].isChecked():
            self._refresh_slot_crop_displays()
            return
        self._apply_slot_crop(cam_idx, self._slot_crop_from_ui(cam_idx))

    def _on_crop_offset_spin(self, cam_idx: int):
        """v7.16: user moved the crop. Committed only while the crop is ON, for
        the same reason as the size spin — nudging a disabled crop must not
        write a store entry for something not switched on."""
        checks = getattr(self, "_live_cam_crop_checks", [])
        if cam_idx < len(checks) and not checks[cam_idx].isChecked():
            self._refresh_slot_crop_displays()
            return
        self._apply_slot_crop(cam_idx, self._slot_crop_from_ui(cam_idx))

    def _on_crop_recentre(self, cam_idx: int):
        """v7.16: put the crop back in the middle of the sensor."""
        for lst in ("_live_cam_crop_off_x_spins", "_live_cam_crop_off_y_spins"):
            spins = getattr(self, lst, [])
            if cam_idx < len(spins):
                sp = spins[cam_idx]
                sp.blockSignals(True)
                sp.setValue(0.0)
                sp.blockSignals(False)
        self._apply_slot_crop(cam_idx, self._slot_crop_from_ui(cam_idx),
                              persist_now=True)

    def _apply_slot_crop(self, cam_idx: int, crop, *, persist_now: bool = False):
        """Commit a slot's crop: push it live, and persist per device identity.

        Pushing live reaches EVERY surface at once, because the crop is applied
        where frames enter the application (``CameraWidget._apply_crop``) rather
        than by each consumer.

        The live push is immediate — the operator is aiming the crop by eye, so
        the preview has to follow the spin box. The STORE WRITE is debounced,
        because the offset spins auto-repeat: holding one arrow walked the crop
        1 %→23 % in seven seconds and wrote the calibration file forty times.
        The store is small and fast here, but a disk write per auto-repeat step
        is exactly the shape the image-correction sliders already avoid (they
        persist on ``sliderReleased``), and it puts file I/O — which is NOT
        bounded by anything we control, e.g. an on-access virus scanner — in
        the middle of a drag on the GUI thread.

        ``persist_now`` skips the debounce for a discrete decision (the
        checkbox, the ⌖ button) where there is no repeat to coalesce.
        """
        mgr = getattr(self, "_camera_manager", None)
        if mgr is not None:
            try:
                setter = getattr(mgr, "set_crop", None)
                if callable(setter):
                    setter(cam_idx, crop)
            except Exception as exc:
                logger.debug(f"slot crop: push to manager — {exc}")
        self._refresh_slot_crop_displays()
        # The Microscope Camera Setup FOV line quotes the DELIVERED field, so it
        # has to follow a crop change on the microscope slot.
        try:
            self._update_camera_info_labels()
        except Exception:
            pass
        logger.debug(f"Camera {cam_idx + 1} crop → {crop.describe()}")
        self._pending_crop_writes[cam_idx] = crop
        if persist_now:
            self._flush_crop_persist()
        else:
            self._crop_persist_timer.start()

    def _flush_crop_persist(self):
        """Write every pending crop to the per-identity store.

        Deliberately keyed by SLOT: coalescing repeats of one slot is the point,
        and a pending write must never be dropped — a crop that survives the
        session but not a restart is worse than one that was never applied.
        """
        try:
            self._crop_persist_timer.stop()
        except Exception:
            pass
        pending = getattr(self, "_pending_crop_writes", None)
        if not pending:
            return
        mgr = getattr(self, "_camera_manager", None)
        for cam_idx, crop in list(pending.items()):
            identity = None
            if mgr is not None:
                try:
                    identity = mgr.camera_identity(cam_idx)
                except Exception:
                    identity = None
            if identity is None or not identity[0]:
                continue
            try:
                from SupportClasses.CameraCalibrationStore import get_store
                get_store().set_crop(
                    identity[0], crop.to_dict() if crop.enabled else None,
                    name=(identity[1] if len(identity) > 1 else ""))
                logger.info(
                    f"Camera {cam_idx + 1} crop set to {crop.describe()} "
                    f"(identity={identity[0]})")
            except Exception as exc:
                logger.warning(f"slot crop: store write failed — {exc}")
        pending.clear()

    def _sync_crop_controls(self, cam_idx: int):
        """Point this slot's checkbox + size spin at the LIVE crop.

        Signals are blocked while setting: these widgets' handlers WRITE to the
        store, so echoing a restored value back through them would re-persist
        (and re-log) a change the operator never made.
        """
        mgr = getattr(self, "_camera_manager", None)
        if mgr is None:
            return
        try:
            getter = getattr(mgr, "get_crop", None)
            crop = getter(cam_idx) if callable(getter) else None
        except Exception:
            crop = None
        if crop is None:
            return
        checks = getattr(self, "_live_cam_crop_checks", [])
        spins = getattr(self, "_live_cam_crop_spins", [])
        if cam_idx < len(checks):
            cb = checks[cam_idx]
            cb.blockSignals(True)
            cb.setChecked(bool(crop.enabled))
            cb.blockSignals(False)
        if cam_idx < len(spins):
            sp = spins[cam_idx]
            sp.blockSignals(True)
            sp.setValue(float(crop.scale) * 100.0)
            sp.blockSignals(False)
        for name, val in (("_live_cam_crop_off_x_spins", crop.offset_x),
                          ("_live_cam_crop_off_y_spins", crop.offset_y)):
            spl = getattr(self, name, [])
            if cam_idx < len(spl):
                sp = spl[cam_idx]
                sp.blockSignals(True)
                sp.setValue(float(val) * 100.0)
                sp.blockSignals(False)
        self._sync_crop_spin_enabled(cam_idx)
        self._refresh_slot_crop_displays()

    def _sync_crop_spin_enabled(self, cam_idx: int):
        """The size and offset controls are live only when the crop itself is —
        one rule, so a disabled crop can never be given a size or a position."""
        checks = getattr(self, "_live_cam_crop_checks", [])
        if cam_idx >= len(checks):
            return
        cb = checks[cam_idx]
        live = cb.isEnabled() and cb.isChecked()
        for name in ("_live_cam_crop_spins", "_live_cam_crop_off_x_spins",
                     "_live_cam_crop_off_y_spins",
                     "_live_cam_crop_centre_btns"):
            widgets = getattr(self, name, [])
            if cam_idx < len(widgets):
                widgets[cam_idx].setEnabled(live)

    def _refresh_slot_crop_displays(self):
        """Re-render every slot's crop readout from the LIVE camera.

        Reads the pre-crop size off the manager rather than trusting the spin
        boxes, so the line states the delivered frame the operator will actually
        get — including "waiting for a frame" before one has arrived.
        """
        mgr = getattr(self, "_camera_manager", None)
        labels = getattr(self, "_live_cam_crop_labels", [])
        for i, lbl in enumerate(labels):
            crop = self._slot_crop_from_ui(i)
            cap = None
            if mgr is not None:
                try:
                    getter = getattr(mgr, "capture_resolution", None)
                    cap = getter(i) if callable(getter) else None
                except Exception:
                    cap = None
            if not crop.enabled:
                lbl.setText("Crop: off (full sensor)")
            elif cap and cap[0] and cap[1]:
                x0, y0, cw, ch = crop.rect_for(cap[0], cap[1])
                dx, dy = crop.center_offset_px(cap[0], cap[1])
                # Report the ACHIEVED displacement, not the requested one — the
                # rect is clamped to the frame, so at the edge the two differ
                # and the compensated value is the one that matters.
                moved = ""
                if abs(dx) > 0.5 or abs(dy) > 0.5:
                    moved = f", offset {dx:+.0f}, {dy:+.0f} px"
                lbl.setText(f"Crop: {cw} × {ch} px at ({x0}, {y0}) "
                            f"of {int(cap[0])} × {int(cap[1])}{moved}")
            else:
                lbl.setText(f"Crop: square {crop.scale * 100:.0f}% "
                            f"— start the camera to see the size")

    def _on_slot_rotation_spin(self, cam_idx: int, value: float):
        """v7.5.x: user typed a custom rotation on the slot's Rotation spin."""
        self._apply_slot_rotation_value(cam_idx, float(value))

    def _apply_slot_rotation_value(self, cam_idx: int, rot: float):
        """v7.5.x: commit a slot's custom LIVE-VIEW rotation (deg).

        v7.16: display only. This spin used to write ``rotation_deg`` — the
        MEASURED camera→stage angle that orients every mosaic tile and rotates
        every click — so a hand-typed viewing angle silently moved the geometry,
        and the next objective calibration silently moved it back. The measured
        angle is now only ever written by a measurement (⟳ Rotation…, ⊾ Square up
        mount…, the objective calibration) and is reported in the readout above.
        """
        self._commit_slot_view_orientation(cam_idx, rot=float(rot))

    def _on_calibrate_needle(self, role: CameraRole):
        """Launch the stage-motion µm/px calibration for a needle camera.

        The slot is resolved from the active role assignment so the
        Needle cam 1 and Needle cam 2 buttons each target their own camera
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

        # v7.10: needle_mode offers the Z leg — a move that displaces the NEEDLE
        # and nothing else, which is the only way to measure this camera's roll,
        # its Z direction and a µm/px free of lateral foreshortening.
        dlg = PixelCalibrationDialog(mgr, ctrl, cam_idx=cam_idx, parent=self,
                                     needle_mode=True)
        if dlg.exec() == QDialog.DialogCode.Accepted:
            result = dlg.result_um_per_px
            if result is not None:
                # v7.5.x (rotated rig): two DIFFERENT angles from one measure —
                # the column→stage mount direction (±45° about +X) feeds the
                # needle aligner (column_dir_deg), while only the sensor roll
                # (deviation of the measured vector from parallel) becomes the
                # display orientation (rotation_deg). Storing the mount angle
                # as rotation was what tilted the live view ~45°.
                #
                # v7.10: stamp the resolution the value was measured at. It was
                # being dropped, so `effective_um_per_px` had nothing to rescale
                # from and the needle aligner silently used a value that was
                # only right at one capture resolution.
                self.set_calibrated_um_per_px(
                    cam_idx, result,
                    rotation_deg=dlg.result_view_roll_deg,
                    column_dir_deg=dlg.result_rotation_deg,
                    resolution=self._live_capture_resolution(cam_idx))
                self._apply_slot_z_row_sign(
                    cam_idx, getattr(
                        getattr(dlg, "result_needle_axes", None),
                        "z_row_sign", None))
                axes = getattr(dlg, "result_needle_axes", None)
                detail = ""
                if axes is not None:
                    detail = (f" (Z leg: roll {axes.roll_deg:+.2f}°, "
                              f"z_row_sign {axes.z_row_sign:+.0f}")
                    # um_per_px_lateral is None on a Z-only run.
                    detail += (
                        f", XY-alone would have said "
                        f"{axes.um_per_px_lateral:.4f})" if axes.has_lateral
                        else ", Z-only)")
                logger.info(
                    f"Needle calibration applied: Cam {cam_idx + 1} "
                    f"({role.value}) = {result:.4f} µm/px" + detail)

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
                    if st.get("source") in ("toupcam", "opencv", "andor",
                                            "tucam"):
                        # v7.13 — ONE shared key list with the settings
                        # dialog's _persist(). This bulk save used to
                        # hand-list keys and silently dropped the andor_*
                        # ones (documented gap since v7.9) — the shared
                        # snapshot makes that drift structurally impossible.
                        from gui.widgets.hw_controls_snapshot import (
                            hw_controls_snapshot)
                        store.set_hw_controls(
                            key, hw_controls_snapshot(st), name=name)
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
                # v7.5.x: open OFF the GUI thread — the blocking device open of
                # each saved camera used to freeze the UI at startup. Falls back
                # to the synchronous start on an older manager.
                if hasattr(mgr, "start_async"):
                    mgr.start_async(i)
                else:
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
            # v7.16: an unknown sensor pitch must READ as unknown. Showing a
            # made-up "theoretical µm/px" is worse than showing none — it is
            # the seed a first calibration is sanity-checked against.
            computed = (effective / mag) if (effective and mag) else None
            if computed is None:
                self.cam_scale_label.setText(
                    "µm/px unknown for this model — run Mosaic && Camera "
                    "Calibration to measure it")
            else:
                self.cam_scale_label.setText(
                    f"{computed:.2f} µm/px  (sensor: "
                    f"{spec.sensor_pixel_size_um} µm, "
                    f"bin: {spec.max_resolution[0] // max(active_res[0], 1)}×, "
                    f"mag: {mag}×)")

            # Determine active scale
            if self.cam_override_check.isChecked():
                scale = self.cam_override_spin.value()
            else:
                scale = computed

            if scale:
                # v7.16: report the DELIVERED field. µm/px is unchanged by a
                # crop, but the field is not — and this label is what the
                # operator reads to judge how many tiles a plate will take, so
                # quoting the full sensor while the app delivers a square would
                # under-state the tile count by the crop fraction.
                fov_res = active_res
                crop_note = ""
                try:
                    mic_idx = self._config.camera_for_role(CameraRole.MICROSCOPE)
                    mgr = getattr(self, "_camera_manager", None)
                    getter = getattr(mgr, "get_crop", None) if mgr else None
                    crop = (getter(mic_idx)
                            if callable(getter) and mic_idx is not None else None)
                    if crop is not None and crop.is_active_for(*active_res):
                        fov_res = crop.size_for(*active_res)
                        crop_note = (f"  — cropped to {fov_res[0]}×{fov_res[1]} "
                                     f"of {active_res[0]}×{active_res[1]} px")
                except Exception:
                    pass
                fov_w = fov_res[0] * scale
                fov_h = fov_res[1] * scale
                self.cam_fov_label.setText(
                    f"{fov_w:.0f} × {fov_h:.0f} µm  "
                    f"({fov_w / 1000:.2f} × {fov_h / 1000:.2f} mm){crop_note}")
            else:
                self.cam_fov_label.setText("— (measure µm/px first)")
        else:
            self.cam_scale_label.setText("—")
            self.cam_fov_label.setText("—")

    # ════════════════════════════════════════════════════════════════
    #  v7.2.4: NEEDLE CHANNEL MAP (S3.7-S3.10)
    # ════════════════════════════════════════════════════════════════

    def _rebuild_channel_map_rows(self):
        """
        v7.2.4 S3.7: Build N rows for bore→pump assignment
        based on current needle bore count.

        v7.9: existing per-bore selections are PRESERVED across the rebuild.
        Without that, bumping the bore-count spin left every new combo on
        "— Unassigned —", and the `_on_channels_changed` → `_rebuild_config`
        that follows persists exactly what the combos say — silently destroying
        the operator's assignments. Same preserve-and-restore shape as
        :meth:`_refresh_channel_map_pump_options`.
        """
        # Snapshot before the widgets are destroyed. While RESTORING a config the
        # combos still hold the OUTGOING setup's assignments, so the incoming
        # config's map is the only correct source — otherwise loading a setup
        # would inherit stale bore→pump pairs for any index its map omits.
        previous: dict[int, str] = {}
        if not getattr(self, "_restoring", False):
            previous = {
                ch_idx: combo.currentData()
                for ch_idx, (_lbl, combo) in enumerate(self._channel_map_widgets)
                if combo.currentData()
            }
        if not previous and getattr(self, "_config", None) is not None:
            previous = dict(getattr(self._config, "needle_channel_pump_map", {}))

        # Clear existing rows
        self._channel_map_widgets.clear()
        while self._channel_rows_layout.count():
            item = self._channel_rows_layout.takeAt(0)
            if item.widget():
                item.widget().deleteLater()

        num_channels = self.channels_spin.value()
        # v7.9.x: every pump, not just the enabled ones — same reasoning as
        # `_refresh_bore_pump_options`. This card is the pump surface for a SINGLE
        # bore, so an operator who has not been to the Pump tab yet must still be
        # able to say which pump feeds their needle; claiming one enables it.
        enabled_pumps = set(self._get_enabled_pump_ids())

        for ch_idx in range(num_channels):
            row_widget = QWidget()
            row_layout = QHBoxLayout(row_widget)
            row_layout.setContentsMargins(0, 0, 0, 0)
            row_layout.setSpacing(8)

            if num_channels == 1:
                label = QLabel("Bore →")
            else:
                # 0-based in code, displayed "Bore N+1" — matching the numbering
                # `validate()` and the per-bore geometry messages already use.
                label = QLabel(f"Bore {ch_idx + 1} →")
            label.setMinimumWidth(s(80))
            row_layout.addWidget(label)

            combo = QComboBox()
            combo.addItem("— Unassigned —", None)
            for pid in self._pump_widgets:
                combo.addItem(
                    pid if pid in enabled_pumps else f"{pid} (will be enabled)",
                    pid)
            # Restore this bore's previous pump before wiring the signal, so the
            # restore itself can't re-enter `_on_config_changed`.
            prev = previous.get(ch_idx)
            if prev:
                idx = combo.findData(prev)
                if idx >= 0:
                    combo.setCurrentIndex(idx)
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
        enabled_pumps = set(self._get_enabled_pump_ids())

        for ch_idx, (label, combo) in enumerate(self._channel_map_widgets):
            current = combo.currentData()
            combo.blockSignals(True)
            combo.clear()
            combo.addItem("— Unassigned —", None)
            for pid in self._pump_widgets:
                combo.addItem(
                    pid if pid in enabled_pumps else f"{pid} (will be enabled)",
                    pid)
            # Restore selection if still valid
            if current:
                idx = combo.findData(current)
                if idx >= 0:
                    combo.setCurrentIndex(idx)
            combo.blockSignals(False)

        # v7.9: the per-bore rows carry their own pump picker on a multi-bore
        # assembly — refresh it from the same enabled-pump list so the two
        # surfaces can never offer different options.
        self._refresh_bore_pump_options()
        self._update_channel_map_status()

    def _on_channel_map_changed(self, ch_idx: int, _combo_idx: int = None):
        """v7.2.4 S3.8: Handle channel mapping combo change."""
        self._update_channel_map_status()
        # v7.9: a map row is still the pump editor for a SINGLE bore, so mirror
        # it back into the per-bore row's combo (which is what the NeedleBore
        # takes its pump_id from) — and do it BEFORE enabling, since that reads
        # the bore rows.
        self._sync_bores_from_channel_map()
        self._ensure_claimed_pumps_enabled()
        self._on_config_changed()

    def _sync_bores_from_channel_map(self):
        """Mirror the bore→pump map rows into the per-bore pump combos.

        The reverse of :meth:`_sync_channel_map_from_bores`, and deliberately
        guarded to the SINGLE-bore case: on a multi-bore assembly the per-bore
        combo is the authority, so copying back would let a stale map row
        overwrite it.
        """
        if len(self._bore_rows) != 1 or not self._channel_map_widgets:
            return
        combo = self._bore_rows[0]["pump"]
        want = self._channel_map_widgets[0][1].currentData()
        idx = combo.findData(want)
        if idx >= 0 and combo.currentIndex() != idx:
            combo.blockSignals(True)
            combo.setCurrentIndex(idx)
            combo.blockSignals(False)

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
                f"⚠ {num_channels - len(assigned_pumps)} bore(s) unassigned")
            self.channel_map_status.setStyleSheet(
                f"color: {COLORS.get('yellow', '#f9e2af')}; ")
        else:
            self.channel_map_status.setText("✓ All bores assigned")
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
        # v7.5.x: keep the needle-derived max-flow readout in sync with the needle.
        self._refresh_max_flow_display()
        self.config_changed.emit(self._config)

    def _refresh_max_flow_display(self):
        """v7.5.x: recompute + show the needle-derived max safe pump flow rate.

        Mirrors ``SafetyLimits.update_from_hardware_config``: Hagen–Poiseuille
        Q_max from the configured needle bore + length at the fixed water
        reference viscosity (ink-independent). This is the ceiling every pump
        move is hard-clamped to. Best-effort — never raises into config rebuild.

        v7.6: a pulled capillary's barrel and tip are in SERIES, so the readout
        names which stage sets the limit — normally the tip, by orders of
        magnitude."""
        lbl = getattr(self, "_pump_maxflow_lbl", None)
        if lbl is None:
            return
        needle = getattr(self._config, "needle", None)
        flow = 0.0
        gauge = getattr(needle, "gauge", None) if needle else None
        limiting = ""
        try:
            if needle is not None and getattr(needle, "id_m", 0) and needle.id_m > 0:
                from SupportClasses.FlowPhysics import (
                    max_safe_flow_rate_uL_s, DEFAULT_PRESSURE_LIMIT_PA,
                )
                from SupportClasses.SafetyLimits import REFERENCE_VISCOSITY_CP
                from SupportClasses.PhysicalModels import InkSpec
                ref_ink = InkSpec(name="__reference__",
                                  viscosity_cP=REFERENCE_VISCOSITY_CP)
                flow = float(max_safe_flow_rate_uL_s(
                    needle, ref_ink, DEFAULT_PRESSURE_LIMIT_PA) or 0.0)
                from SupportClasses.FlowPhysics import limiting_flow_segment
                limiting = limiting_flow_segment(needle)[0]
        except Exception as e:
            logger.debug("max-flow display compute failed: %s", e)
            flow = 0.0
        if flow > 0:
            if getattr(needle, "has_tip", False):
                tip = getattr(needle, "orifice_id_um", 0.0)
                lbl.setText(
                    f"Max safe pump flow (capillary, {tip:.0f} µm tip, water ref): "
                    f"{flow:.3g} µL/s — set by the {limiting or 'tip'} "
                    f"(barrel + tip resist in series); hard-capped on all pumps; "
                    f"bounds the max print speed")
            else:
                g = f"{gauge}G, " if gauge else ""
                lbl.setText(
                    f"Max safe pump flow ({g}water ref): {flow:.2f} µL/s — "
                    f"hard-capped on all pumps; bounds the max print speed")
        else:
            lbl.setText("Max safe pump flow: — (configure the needle bore)")
        # v7.5.x: a syringe/pump change alters each pump's per-pump max-rate
        # µL/s equivalent — re-seed the control panel's per-pump speed rows so a
        # newly-configured pump appears and its µL/s readout tracks the syringe.
        cp = getattr(self, "_control_panel", None)
        if cp is not None and hasattr(cp, "refresh_speed_limits"):
            try:
                cp.refresh_speed_limits()
            except Exception:
                pass

    def _rebuild_config(self):
        """Rebuild HardwareConfig from all widget states."""
        # Name & notes — name_edit is a plain QLineEdit (v7.4.2 rev2).
        self._config.config_name = (
            self.name_edit.text().strip() or "Untitled Setup")
        self._config.notes = self.notes_edit.text().strip()

        # Well plate (v7.4.5: designer widget owns geometry; v7.5.x: the Plate
        # Type card owns the product id). A selected plate type keeps the
        # designer on its base-format geometry, so current_plate_format() is
        # the base int and current_plate_name() is "" — active_plate_key then
        # resolves to plate_type_id.
        # v7.12: the ACTIVE DESIGN is the single owner of plate identity —
        # `plate_doc_id` is set by the library's ★ (see
        # `_on_active_plate_changed`) and is not rebuilt from widgets here.
        # Splitting ownership between a Format→Type card and a designer picker
        # is what made each silently clear the other.
        doc = self._active_plate_document()
        if doc is not None:
            self._config.plate_name = doc.meta.name
            self._config.plate_type_id = doc.meta.plate_type_id or ""
            wells = len(doc.evaluate())
            if wells in PLATE_DEFINITIONS:
                self._config.plate_format = wells
            # Keep the shim combo in sync for legacy readers.
            idx = self.plate_combo.findData(self._config.plate_format)
            if idx >= 0:
                self.plate_combo.blockSignals(True)
                self.plate_combo.setCurrentIndex(idx)
                self.plate_combo.blockSignals(False)
        elif not getattr(self._config, "plate_doc_id", ""):
            self._config.plate_format = self.plate_combo.currentData() or 24

        # Needle (v7.6: hypodermic gauge OR pulled glass capillary;
        # v7.9: plus the assembly FORM and, for a multi-bore assembly, the
        # explicit per-bore list. `bores` stays None for a single needle, which
        # is what keeps `to_dict()` byte-identical to every pre-v7.9 setup.)
        bores = self._bores_from_ui()
        needle_form = self._current_needle_form()
        if self._needle_type_combo.currentData() == NEEDLE_TYPE_CAPILLARY:
            g = self._capillary_geometry()
            b_id, b_od = g["barrel_id_um"], g["barrel_od_um"]
            self._config.needle = NeedleSpec(
                gauge=None,                          # a capillary has no gauge
                od_um=b_od,
                id_um=b_id,                          # base fields == the BARREL
                wall_um=max(0.0, (b_od - b_id) / 2.0),
                # Stored as inches so `length_mm` and every barrel-length
                # consumer keep working untouched.
                length_inches=g["barrel_length_mm"] / 25.4,
                # v7.9: no longer hardcoded to 1 — a capillary may be one bore
                # of a backpack (the FORM and the taper are orthogonal axes).
                num_channels=self.channels_spin.value(),
                needle_type=NEEDLE_TYPE_CAPILLARY,
                tip_id_um=g["tip_id_um"],
                tip_length_mm=g["tip_length_mm"],
                tip_od_um=g["tip_od_um"],
                tip_profile=g["tip_profile"],
                needle_type_id=self._needle_type_preset_combo.currentData() or None,
                needle_form=needle_form,
                bores=bores,
            )
        else:
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
                    needle_form=needle_form,
                    bores=bores,
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

        # v7.5.x: global pump timing (settle + prime). Pressure relief /
        # compliance is now per-pump µL (device profile), not a HardwareConfig
        # field — configured via the Needle Location compliance calibration.
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
        btn_clear_all = icon_button(
            "Clear all wells", "trash", object_name="dangerBtn")
        btn_clear_all.setToolTip(
            "Remove every reagent → well assignment (fresh ink landscape); "
            "also wipes stale/hidden entries.")
        btn_clear_all.clicked.connect(self._loc_clear_all)
        loc_btns.addWidget(btn_clear_all)
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
        # v7.5.x: geometry_plate_key so reagent locations can be assigned to a
        # rosette/custom design layered under a plate TYPE (falls back to
        # active_plate_key when no custom design is present).
        key = getattr(self._config, "geometry_plate_key", None) \
            or self._config.active_plate_key
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

    def _loc_clear_all(self):
        """Clear EVERY reagent's pinned wells — a fresh-start reset.

        Wipes the whole ``ink_locations`` map, including any stale/hidden
        entries (e.g. a flattened-rosette parent no longer shown in the
        picker), so a new ink landscape starts with no residual members.
        """
        if not self._config.ink_locations:
            return
        if QMessageBox.question(
            self, "Clear all reagent locations",
            "Remove ALL reagent → well assignments?\n\n"
            "This wipes every reagent's pinned wells — including any stale or "
            "hidden entries (e.g. a rosette parent no longer shown) — so you "
            "can set up a fresh ink landscape. This cannot be undone.",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No,
        ) != QMessageBox.Yes:
            return
        self._config.clear_all_ink_locations()
        self._refresh_loc_colors()
        self._refresh_loc_summary()
        self._on_loc_ink_changed()  # clear the highlight
        self._on_config_changed()

    # ════════════════════════════════════════════════════════════════
    #  SAVE / LOAD
    # ════════════════════════════════════════════════════════════════

    def _setup_dir(self) -> str:
        """The folder saved setups are READ from — and so saved to.

        v7.16: both dialogs used to default to the process CWD (Save passed a
        bare filename, Load passed ""), while the file browser and the Setup
        Name combo only ever scan ``config/hardware``. So a saved setup landed
        somewhere the page could not see it and did not appear in the list —
        which is also how loose setup .json files ended up in the repo root.
        """
        try:
            CONFIG_HARDWARE_DIR.mkdir(parents=True, exist_ok=True)
        except Exception:
            pass
        return str(CONFIG_HARDWARE_DIR)

    def _setup_path_for_name(self, name: str) -> Path:
        """``<setups folder>/<name>.json`` — where Save puts a setup.

        The Saved-setups list scans exactly one folder, so a setup saved
        anywhere else is invisible to the page that is supposed to load it.
        Passing that folder as a file-dialog *default* was not enough: the
        Windows native dialog re-opens wherever it was last used, so Save
        kept landing in the process CWD (which is how loose setup .json
        files ended up in the repo root). The destination is therefore
        derived, not chosen.
        """
        stem = re.sub(r'[<>:"/\\|?*\x00-\x1f]', "_", (name or "").strip())
        stem = stem.rstrip(". ") or "Untitled Setup"
        return Path(self._setup_dir()) / f"{stem}.json"

    def _write_config_to(self, path: Path | str) -> bool:
        """Write the current config to ``path`` and refresh both browsers."""
        try:
            self._config.save(str(path))
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to save:\n{e}")
            return False
        QMessageBox.information(
            self, "Saved", f"Configuration saved to:\n{path}")
        # v7.2.4: Refresh the config file browser
        self._scan_config_directory()
        # v7.4.2: Refresh the Setup Name list so the new file appears as a
        # pickable option immediately.
        self._refresh_setup_name_combo()
        return True

    def _save_config(self):
        """Save the current config into the saved-setups folder.

        Deliberately no folder chooser: the file must land where the
        Saved-setups list reads from, and the name comes from the Name
        field. Use *Save As…* to put a copy somewhere else.
        """
        self._rebuild_config()
        if not (self._config.config_name or "").strip():
            QMessageBox.warning(
                self, "Name this setup",
                "Enter a Name for this setup before saving — the name "
                "becomes the filename and the entry in the Saved setups "
                "list.")
            if hasattr(self, "name_edit"):
                self.name_edit.setFocus()
            return
        path = self._setup_path_for_name(self._config.config_name)
        if path.exists():
            if QMessageBox.question(
                self, "Overwrite setup?",
                f"“{path.name}” already exists in:\n{path.parent}\n\n"
                "Overwrite it?",
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No,
            ) != QMessageBox.Yes:
                return
        self._write_config_to(path)

    def _save_config_as(self):
        """Save a copy anywhere (escape hatch for the derived path above).

        Defaults to the setups folder; a file saved elsewhere will NOT
        appear in the Saved setups list.
        """
        self._rebuild_config()
        path, _ = QFileDialog.getSaveFileName(
            self, "Save Hardware Setup As",
            str(self._setup_path_for_name(self._config.config_name)),
            "Hardware Setup (*.json)",
        )
        if path:
            self._write_config_to(path)

    def _load_config(self):
        """Load a config from a JSON file."""
        path, _ = QFileDialog.getOpenFileName(
            self, "Load Hardware Setup", self._setup_dir(),
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

        # ── 2. Active plate (v7.12) ──────────────────────────────
        # The library's ★ is the plate selector; restoring the config just
        # points the library and the Layout tab at the saved document.
        active_key = self._config.active_plate_key
        if hasattr(self, "_plate_workspace"):
            self._plate_workspace.set_active_plate_id(self._active_library_id())
            self._plate_workspace.refresh()
        if hasattr(self, "_rosette_placement"):
            self._rosette_placement.set_plate(
                self._active_plate_document(materialize=True))
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

        # ── 4. Rosettes ──────────────────────────────────────────
        # v7.12: rosettes are library documents placed from the Layout tab.
        # `config.rosette_library` is a retired field, kept only so older
        # setup JSON still round-trips.

        # v7.5.x: reagent locations (force a plate reload — the active plate
        # key may have changed with the loaded config; _on_config_changed is
        # suppressed during restore so refresh here explicitly).
        self._refresh_loc_plate(force=True)
        self._refresh_reagent_locations()

        # ── 5. Needle Config ─────────────────────────────────────
        n = self._config.needle
        is_cap = bool(n is not None and getattr(
            n, "needle_type", NEEDLE_TYPE_HYPODERMIC) == NEEDLE_TYPE_CAPILLARY)

        self._needle_type_combo.blockSignals(True)
        tidx = self._needle_type_combo.findData(
            NEEDLE_TYPE_CAPILLARY if is_cap else NEEDLE_TYPE_HYPODERMIC)
        self._needle_type_combo.setCurrentIndex(tidx if tidx >= 0 else 0)
        self._needle_type_combo.blockSignals(False)

        # v7.9: assembly FORM first — it drives the bore count, so the bore rows
        # `_on_needle_type_changed()` rebuilds below already have the right shape.
        # The count is read from the RESOLVED bores rather than `needle_form`, so
        # a hand-edited file whose form and bore list disagree still restores
        # every bore the operator actually saved.
        n_bores = 1
        if n is not None:
            try:
                n_bores = max(1, int(n.bore_count))
            except Exception:
                n_bores = max(1, int(getattr(n, "num_channels", 1) or 1))
        form = getattr(n, "needle_form", NEEDLE_FORM_SINGLE) if n else NEEDLE_FORM_SINGLE
        if NEEDLE_FORM_BORE_COUNT.get(form) != n_bores:
            form = self._form_for_bore_count(n_bores, n)
        self._needle_form_combo.blockSignals(True)
        fidx = self._needle_form_combo.findData(form)
        self._needle_form_combo.setCurrentIndex(fidx if fidx >= 0 else 0)
        self._needle_form_combo.blockSignals(False)
        # The bore count mirror. v7.9 also fixes a pre-existing gap: the count was
        # only restored on the hypodermic branch below, so a saved capillary came
        # back as one bore regardless of what was stored.
        self.channels_spin.blockSignals(True)
        self.channels_spin.setValue(n_bores)
        self.channels_spin.blockSignals(False)

        self.gauge_combo.blockSignals(True)
        if n is not None and is_cap:
            for spin, value in (
                (self._cap_barrel_id_spin, n.id_um),
                (self._cap_barrel_od_spin, n.od_um),
                (self._cap_barrel_len_spin, n.length_mm),
                (self._cap_tip_id_spin, getattr(n, "tip_id_um", 0.0) or 0.0),
                (self._cap_tip_od_spin, getattr(n, "tip_od_um", None) or 0.0),
                (self._cap_tip_len_spin, getattr(n, "tip_length_mm", 0.0) or 0.0),
            ):
                spin.blockSignals(True)
                spin.setValue(float(value or 0.0))
                spin.blockSignals(False)
            combo = self._cap_tip_profile_combo
            combo.blockSignals(True)
            pidx = combo.findData(getattr(n, "tip_profile", TIP_PROFILE_CYLINDER))
            combo.setCurrentIndex(pidx if pidx >= 0 else 0)
            combo.blockSignals(False)
            self._refresh_needle_type_preset_combo(
                select_id=getattr(n, "needle_type_id", None))
            logger.debug(
                f"  Needle: pulled capillary, tip "
                f"{getattr(n, 'tip_id_um', None)} µm")
        elif n is not None:
            gidx = self.gauge_combo.findData(n.gauge)
            if gidx >= 0:
                self.gauge_combo.setCurrentIndex(gidx)
            # Length. v7.6: a length not among the presets (e.g. 0.5") used to
            # findData to -1 and be silently rewritten to 1.0" on the next save;
            # add it instead so the stored value survives a round-trip.
            self.length_combo.blockSignals(True)
            lidx = self.length_combo.findData(n.length_inches)
            if lidx < 0:
                self.length_combo.addItem(f'{n.length_inches:g}"', n.length_inches)
                lidx = self.length_combo.count() - 1
            self.length_combo.setCurrentIndex(lidx)
            self.length_combo.blockSignals(False)
            logger.debug(
                f"  Needle: {n.gauge}G, {n.num_channels} bore(s)")
        else:
            self.gauge_combo.setCurrentIndex(0)
        self.gauge_combo.blockSignals(False)
        # Visibility + bore-map rows; also refreshes the info label. Safe during
        # restore — `_restoring` short-circuits `_on_config_changed`.
        self._on_needle_type_changed()

        # v7.9: the per-bore rows, now that they exist. Bore 1's geometry came from
        # the flat fields above (it mirrors `bores[0]`); v7.9.x: every OTHER bore's
        # geometry is derived from the form, so all a row needs is its label + pump.
        # A backpack's one non-derivable value — bore 2's diameters — is restored
        # into the second-needle row below.
        # The incoming setup's mount offsets are authoritative INCLUDING their
        # absence — carrying the outgoing machine's over would place bores using
        # another rig's calibration.
        self._bore_offsets.clear()
        # ⚠ RESOLVED, not raw. The row COUNT above reads `n.bore_count` (resolved),
        # so reading the raw `bores` field here disagreed with it for a legacy
        # `num_channels: 2` needle — which has a bore count of 2 and NO bores list.
        # Two rows were built, row 2 was never filled, the form combo was set to
        # "backpack", and saving then persisted a FABRICATED 0 µm bore — which
        # `HardwareConfig.validate()` reports as a blocking "Bore 2: No needle
        # gauge selected" the operator never caused, and which SafetyLimits and the
        # prep planner would then resolve as a real bore. `bores_resolved()` exists
        # for exactly this case: it synthesizes N identical bores from the flat
        # fields, which IS the documented meaning of the legacy bore count.
        saved_bores = []
        if n is not None:
            try:
                saved_bores = list(n.bores_resolved())
            except Exception:
                logger.debug("bores_resolved() failed; falling back to the raw "
                             "bore list", exc_info=True)
                saved_bores = list(getattr(n, "bores", None) or [])
        for k, b in enumerate(saved_bores):
            if k >= len(self._bore_rows):
                break
            self._set_bore_row_geometry(k, {
                "label": getattr(b, "label", "") or "",
                "pump_id": getattr(b, "pump_id", None),
            })
        self._restore_backpack_bore2(saved_bores)
        self._refresh_bore_readouts()

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
        # v7.5.x: pressure relief / compliance is now per-pump µL (device
        # profile), not a HardwareConfig field — no UI to sync here.
        # v7.5.x: refresh the needle-derived max-flow readout on config load.
        self._refresh_max_flow_display()

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
        # v7.9: the per-bore pump combos were populated back in the needle section,
        # when the pump enable checkboxes still held the OUTGOING setup's state, so
        # the enabled-pump list was empty and every saved bore→pump binding was
        # dropped. This is the first point where the pumps are known to be enabled,
        # so re-offer them and re-apply what was saved — BEFORE the sync below,
        # which mirrors the bore combos over the map rows and would otherwise
        # propagate the loss to the legacy map as well.
        self._refresh_bore_pump_options()
        self._reapply_saved_bore_pumps()
        # v7.9: on a multi-bore assembly the bores own the mapping, so let them
        # have the last word over a stored map that may predate them.
        self._sync_channel_map_from_bores()
        self._sync_bores_from_channel_map()
        # The readouts built in the needle section saw the unassigned combos, so
        # the flow ceilings / duplicate-pump status need re-deriving.
        self._refresh_bore_readouts()
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

    def on_motion_tick(self):
        """v7.5.x: fast (~30 fps) display-only tick — forward to the embedded
        control + stage panels so a jog's position readout animates smoothly
        (fired by MainWindow only while a motion estimate is live)."""
        for attr in ("_control_panel", "_stage_panel"):
            w = getattr(self, attr, None)
            fn = getattr(w, "on_motion_tick", None) if w is not None else None
            if callable(fn):
                try:
                    fn()
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

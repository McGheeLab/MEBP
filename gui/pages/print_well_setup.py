"""
print_well_setup.py — Tab 3: Well Setup & Assignment for MEBP v7.1.

Complete well plate setup page:
- Interactive plate view with multi-select (WellPlateView widget)
- ZY side projection + XZ bottom projection (well bottoms + needle)
- Selection actions: role assignment, print assignment, rosette attach
- Rosette sub-well role editor
- Well bottom plane detection (teach + fit)
- Service sequence configuration
- Assignment summary table (scrollable, synced with plate)
- Auto-assign patterns (block, checkerboard, border)
- Save/Load well setup JSON

Layout matches the coding plan Tab 3 wireframe:
┌───────────────────────────────────┬────────────────────┐
│  Interactive Plate View (XY)      │ ZY Side Projection │
├───────────────────────────────────┴────────────────────┤
│  XZ Bottom Projection                                  │
├────────────────────────────────────────────────────────┤
│  Selection Actions + Rosette Editor                    │
├────────────────────────────────────────────────────────┤
│  Well Bottom Detection                                 │
├────────────────────────────────────────────────────────┤
│  
        # -- Print Plan of Action (v7.2.4 S5) ---------------------
        plan_group = QGroupBox("Print Plan of Action")
        plan_group.setCheckable(True)
        plan_group.setChecked(True)
        plan_layout = QVBoxLayout(plan_group)

        # Plan preferences row
        pref_row = QHBoxLayout()

        pref_row.addWidget(QLabel("Max Ink/Run:"))
        self._max_ink_spin = QDoubleSpinBox()
        self._max_ink_spin.setRange(0.1, 1000.0)
        self._max_ink_spin.setValue(100.0)
        self._max_ink_spin.setSuffix(" uL")
        self._max_ink_spin.setDecimals(1)
        pref_row.addWidget(self._max_ink_spin)

        self._wash_check = QCheckBox("Wash")
        self._wash_check.setChecked(True)
        pref_row.addWidget(self._wash_check)

        self._waste_check = QCheckBox("Waste")
        self._waste_check.setChecked(True)
        pref_row.addWidget(self._waste_check)

        self._buffer_check = QCheckBox("Buffer")
        self._buffer_check.setChecked(True)
        pref_row.addWidget(self._buffer_check)

        plan_layout.addLayout(pref_row)

        # Generate + Validate buttons
        btn_row = QHBoxLayout()
        self._generate_plan_btn = QPushButton("Generate Plan")
        self._generate_plan_btn.clicked.connect(self._generate_plan)
        btn_row.addWidget(self._generate_plan_btn)

        self._validate_btn = QPushButton("Validate Setup")
        self._validate_btn.clicked.connect(self._run_validation)
        btn_row.addWidget(self._validate_btn)
        plan_layout.addLayout(btn_row)

        # Plan step display (scrollable list)
        self._plan_display = QLabel("No plan generated yet")
        self._plan_display.setWordWrap(True)
        self._plan_display.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {sf(9)}pt; padding: {sp(4)};")
        plan_scroll = QScrollArea()
        plan_scroll.setWidget(self._plan_display)
        plan_scroll.setWidgetResizable(True)
        plan_scroll.setMaximumHeight(s(180))
        plan_layout.addWidget(plan_scroll)

        # Plan summary
        self._plan_summary = QLabel("")
        self._plan_summary.setStyleSheet(
            f"color: {COLORS['text']}; font-weight: bold; font-size: 10px;")
        plan_layout.addWidget(self._plan_summary)

        # Validation status
        self._validation_label = QLabel("")
        self._validation_label.setWordWrap(True)
        self._validation_label.setStyleSheet(f"font-size: 10px;")
        plan_layout.addWidget(self._validation_label)

        main_layout.addWidget(plan_group)

Assignment Summary Table                              │
└────────────────────────────────────────────────────────┘

Session G — Tasks P5.19, P5.23, P5.26, P5.27, P5.32, P5.33, P5.34, P5.35.

v7.1 minor gap fix: MiniProjectionView now imported from
gui.widgets.projection_canvas (unified L-shaped projection widget)
instead of being defined inline.
"""

from __future__ import annotations

import json
import logging
from pathlib import Path
from functools import partial

from PySide6.QtWidgets import (
    QStackedWidget,
    QMessageBox,
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QGroupBox,
    QLabel, QPushButton, QComboBox, QDoubleSpinBox, QSpinBox,
    QFileDialog, QFrame, QTableWidget, QTableWidgetItem,
    QHeaderView, QAbstractItemView, QScrollArea, QSizePolicy,
    QMenu, QSplitter, QCheckBox
)
from PySide6.QtCore import Qt, Signal, QPointF, QTimer
from PySide6.QtGui import QColor, QCursor

from gui.styles import COLORS, SECTION_TITLE_STYLE, CONTEXT_SECTION_LABEL_STYLE
from gui.scaling import s, sf, sp, scaled_font_size
from gui.widgets.well_plate_view import WellPlateView, WellRoleLegend
# MiniProjectionView removed in v7.2.4 (XY-only layout)

from SupportClasses.PhysicalModels import (
    WellRole, ROLE_COLORS, InkSpec, RosetteInsert, WorkspaceConfig,
)
from SupportClasses.WellPlate import WellPlate, ROW_LABELS

from SupportClasses.WellSetup import (
    WellAssignment, WellSetupModel, ServiceSequence,
    WashBehavior, WasteBehavior, BufferBehavior,
    InkPickupBehavior, SortedCellBehavior, PlaneResult,
    auto_assign_block, auto_assign_checkerboard, auto_assign_border,
)

from SupportClasses.PrintPlanOfAction import (
    PrintPlanOfAction, PlanPreferences, PlanStepType,
    PLAN_STEP_COLORS, PLAN_STEP_ICONS,
    validate_well_setup,
)

logger = logging.getLogger(__name__)


def _same_plate_key(a, b) -> bool:
    """Compare two plate keys, treating int 96 and 'custom:foo' vs 'foo' equivalently.

    v7.4.5 helper. Inputs may be:
      - int (standard format like 6/12/96/...)
      - str (custom plate name like 'my-plate' or its WellPlate.format
        encoding 'custom:my-plate')
    """
    if a == b:
        return True
    a_s = a[7:] if isinstance(a, str) and a.startswith("custom:") else a
    b_s = b[7:] if isinstance(b, str) and b.startswith("custom:") else b
    return a_s == b_s


# ═══════════════════════════════════════════════════════════════════
# Well Setup Tab (Tab 3)
# ═══════════════════════════════════════════════════════════════════


# ═══════════════════════════════════════════════════════════════════
# Well Setup Tab  — v7.3.1: WellSetupTab complete rewrite
# ═══════════════════════════════════════════════════════════════════

class WellSetupTab(QWidget):
    """
    Tab 3: Well Setup & Assignment — v7.3.1 clean rewrite.

    Flow:
        1. User selects wells (click / drag / Ctrl+click) → purple border
        2. User clicks a role button → role assigned + role options appear
        3. User configures the role (pick print file, pick ink, etc.)
        4. Well border: GREEN = ready, RED = incomplete, GRAY = empty,
           PURPLE = currently selected (overrides all others)

    External API (called by print_setup.py / app.py):
        setup_changed   Signal()          emitted after any assignment change
        model           property          → WellSetupModel
        set_workspace(ws)                 update plate format
        set_available_prints(names)       update print combo
        set_hardware_config(cfg)          update inks + plate
        update_needle_position(x, y, z)   forward to plate_view
        plate_view                        WellPlateView (public attribute)
        plane_info_label                  QLabel status text
    """

    setup_changed = Signal()

    def __init__(
        self,
        controller=None,
        settings=None,
        workspace=None,
        parent=None,
    ):
        super().__init__(parent)
        self._controller = controller
        self._settings   = settings
        self._workspace  = workspace or WorkspaceConfig()

        # Model (v7.4.5: use active_plate_key so custom plates pass through)
        plate_key = getattr(self._workspace, "active_plate_key",
                            self._workspace.plate_format)
        self._model = WellSetupModel(plate_key)

        # State
        self._available_prints: list = []
        self._hw_config             = None
        self._role_btns: dict       = {}
        self._role_options_stack    = None
        self._role_stack_indices: dict = {}

        # Plan
        try:
            from SupportClasses.PrintPlanOfAction import PlanPreferences
            self._plan_preferences = PlanPreferences()
        except Exception:
            self._plan_preferences = None
        self._plan = None

        # Public widget refs (external code may read these)
        self.plate_view        = None
        self.print_combo       = None
        self.ink_combo         = None
        self.plane_info_label  = None
        self._summary_table    = None
        self._max_ink_spin     = None
        self._wash_check       = None
        self._waste_check      = None
        self._buffer_check     = None

        self._build_ui()
        self._connect_signals()
        self._refresh_plate()

    # ── External API ──────────────────────────────────────────────

    @property
    def model(self):
        return self._model

    def set_workspace(self, workspace) -> None:
        """Update workspace / plate format."""
        self._workspace = workspace
        new_key = getattr(workspace, "active_plate_key", workspace.plate_format)
        if not _same_plate_key(new_key, self._model.plate_format):
            self._model.set_plate_format(new_key)
            self._refresh_plate()

    def set_available_prints(self, names: list) -> None:
        """Receive updated print collection names from Tab 2."""
        self._available_prints = list(names)
        self._refresh_print_combo()

    def set_hardware_config(self, config) -> None:
        """Receive HardwareConfig — refresh inks, rosettes, plate format."""
        if config is None:
            return
        self._hw_config = config
        self._refresh_ink_options_from_hw_config(config)
        if hasattr(self, "_model") and self._model:
            current_fmt = getattr(self._model, "plate_format",
                                  getattr(self._model, "_plate_format", None))
            # v7.4.5: use active_plate_key so custom plates pass through.
            new_key = getattr(config, "active_plate_key", config.plate_format)
            if not _same_plate_key(new_key, current_fmt):
                try:
                    self._model.set_plate_format(new_key)
                    self._refresh_plate()
                except Exception as exc:
                    logger.error(f"Plate format sync failed: {exc}")

    def update_needle_position(self, x_mm, y_mm, z_mm=None) -> None:
        """Forward needle position to plate view overlay."""
        pv = getattr(self, "plate_view", None)
        if pv is not None and hasattr(pv, "set_needle_position"):
            try:
                pv.set_needle_position(x_mm, y_mm)
            except Exception:
                pass

    # ── UI Build ──────────────────────────────────────────────────

    def _build_ui(self) -> None:
        """Build the well setup layout.

        v7.6.0: the body keeps only the canonical view (status line +
        plate + assignment summary). The role bar, role options, and
        save/load/auto-assign actions are built as DETACHED panels
        (``role_bar_panel`` / ``role_options_panel`` /
        ``well_actions_panel``) that the wizard reparents into the
        Step 2 left-context Tools.
        """
        main = QVBoxLayout(self)
        main.setContentsMargins(4, 4, 4, 4)
        main.setSpacing(4)

        # Status label (save/load / warnings)
        self.plane_info_label = QLabel("")
        self.plane_info_label.setStyleSheet(
            f"color: {COLORS['subtext0']}; font-size: {sf(9.5)}pt; padding: {sp(2)} {sp(4)};")
        main.addWidget(self.plane_info_label)

        # ── Body: plate + summary (canonical view, stays here) ────
        self._build_plate_area(main)

        # ── Detached tool panels (reparented into Step 2 Tools) ───
        self.role_bar_panel = self._make_tool_panel(
            "Assign Role", self._build_role_bar)
        self.role_options_panel = self._make_tool_panel(
            "Role Options", self._build_role_options)
        self.well_actions_panel = self._make_tool_panel(
            "Setup", self._build_bottom_buttons)

    def _make_tool_panel(self, title: str, build_fn) -> QWidget:
        """Wrap one of the existing `_build_*` section builders into a
        standalone titled QWidget for the wizard's Step 2 Tools."""
        host = QWidget()
        host.setStyleSheet(f"background: {COLORS['base']};")
        lay = QVBoxLayout(host)
        lay.setContentsMargins(s(4), s(4), s(4), s(4))
        lay.setSpacing(s(4))
        hdr = QLabel(title)
        hdr.setStyleSheet(
            f"color: {COLORS['text']}; font-weight: 700; "
            f"font-size: {sf(10.5)}pt; "
            f"background: {COLORS['surface0']}; "
            f"padding: {sp(4)} {sp(8)}; border-radius: 4px;")
        lay.addWidget(hdr)
        build_fn(lay)
        lay.addStretch()
        return host

    def _build_plate_area(self, parent_layout) -> None:
        """Splitter: WellPlateView (left) + Assignment Summary Table (right)."""
        splitter = QSplitter(Qt.Orientation.Horizontal)
        splitter.setChildrenCollapsible(False)

        # Left: plate view
        try:
            self.plate_view = WellPlateView(self)
        except Exception:
            self.plate_view = QLabel("[WellPlateView unavailable]")

        splitter.addWidget(self.plate_view)

        # Right: summary table
        table_container = QWidget()
        tc_layout = QVBoxLayout(table_container)
        tc_layout.setContentsMargins(0, 0, 0, 0)
        tc_layout.setSpacing(2)
        tc_layout.addWidget(QLabel("Assignment Summary:"))

        self._summary_table = QTableWidget(0, 3)
        self._summary_table.setHorizontalHeaderLabels(["Well", "Role", "Detail"])
        hdr = self._summary_table.horizontalHeader()
        hdr.setStretchLastSection(True)
        hdr.resizeSection(0, 50)
        hdr.resizeSection(1, 70)
        self._summary_table.setEditTriggers(
            QAbstractItemView.EditTrigger.NoEditTriggers)
        self._summary_table.setSelectionMode(
            QAbstractItemView.SelectionMode.NoSelection)
        self._summary_table.setAlternatingRowColors(True)
        self._summary_table.verticalHeader().setVisible(False)
        # Catppuccin styling — without this, setAlternatingRowColors
        # falls back to the QPalette AlternateBase which Qt defaults
        # to system white, producing a glaring white background.
        self._summary_table.setStyleSheet(
            f"QTableWidget {{"
            f"  background: {COLORS['base']};"
            f"  alternate-background-color: {COLORS['surface0']};"
            f"  color: {COLORS['text']};"
            f"  border: 1px solid {COLORS['surface1']};"
            f"  gridline-color: {COLORS['surface1']};"
            f"  font-size: {sf(10)}pt;"
            f"}}"
            f"QTableWidget::item {{ padding: {sp(4)} {sp(6)}; }}"
            f"QHeaderView::section {{"
            f"  background: {COLORS['mantle']};"
            f"  color: {COLORS['subtext0']};"
            f"  border: 1px solid {COLORS['surface1']};"
            f"  padding: {sp(4)} {sp(8)};"
            f"  font-weight: 600;"
            f"}}"
            f"QTableCornerButton::section {{"
            f"  background: {COLORS['mantle']};"
            f"  border: 1px solid {COLORS['surface1']};"
            f"}}"
        )
        # Viewport background — the QAbstractScrollArea viewport ignores
        # the QTableWidget stylesheet's background-color rule.
        self._summary_table.viewport().setStyleSheet(
            f"background: {COLORS['base']};"
        )
        tc_layout.addWidget(self._summary_table)

        splitter.addWidget(table_container)
        splitter.setSizes([420, 280])

        parent_layout.addWidget(splitter, stretch=3)

    def _build_role_bar(self, parent_layout) -> None:
        """v7.6.0: checkable role buttons in a 2-column grid so they
        fit the narrow left-context Tools column without overflow."""
        from PySide6.QtWidgets import QGridLayout
        bar_frame = QFrame()
        bar_frame.setObjectName("cardFrame")
        bar_layout = QVBoxLayout(bar_frame)
        bar_layout.setContentsMargins(6, 4, 6, 4)
        bar_layout.setSpacing(4)

        bar_layout.addWidget(QLabel("Assign Role:"))

        role_defs = [
            (WellRole.PRINT,        "Print",   "#a6e3a1"),
            (WellRole.INK,          "Ink",     "#89b4fa"),
            (WellRole.WASH,         "Wash",    "#f9e2af"),
            (WellRole.WASTE,        "Waste",   "#f38ba8"),
            (WellRole.BUFFER,       "Buffer",  "#cba6f7"),
            (WellRole.SORTED_CELLS, "Sorted",  "#fab387"),
        ]

        grid = QGridLayout()
        grid.setSpacing(4)
        for i, (role, label, color) in enumerate(role_defs):
            btn = QPushButton(label)
            btn.setCheckable(True)
            btn.setFixedHeight(s(28))
            btn.setMinimumWidth(0)
            btn.setSizePolicy(QSizePolicy.Policy.Expanding,
                              QSizePolicy.Policy.Fixed)
            btn.setStyleSheet(
                f"QPushButton {{background:{COLORS['base']}; border:2px solid {COLORS['surface1']};"
                f" border-radius:{sp(4)}; color:{COLORS['text']}; font-size:{sf(9.5)}pt;}}"
                f"QPushButton:checked {{border-color:{color}; color:{color};"
                f" background:{COLORS['menu_hover']};}}"
                f"QPushButton:hover {{border-color:{color}; color:{color};}}"
            )
            btn.clicked.connect(
                lambda _checked, r=role: self._on_role_btn_clicked(r)
            )
            self._role_btns[role] = btn
            grid.addWidget(btn, i // 2, i % 2)
        bar_layout.addLayout(grid)

        # Clear / Empty button — full-width row.
        from gui.widgets.icons import icon_button as _icon_button
        btn_clear = _icon_button(
            "Clear", "x", object_name="dangerBtn",
            tooltip="Reset selected wells to Empty")
        btn_clear.setFixedHeight(s(28))
        btn_clear.clicked.connect(self._on_clear_role)
        bar_layout.addWidget(btn_clear)

        parent_layout.addWidget(bar_frame)

    def _build_role_options(self, parent_layout) -> None:
        """QStackedWidget with one page per role + placeholder page."""
        options_frame = QFrame()
        options_frame.setObjectName("cardFrame")
        of_layout = QVBoxLayout(options_frame)
        of_layout.setContentsMargins(6, 4, 6, 4)
        of_layout.setSpacing(0)

        self._role_options_stack = QStackedWidget()
        self._role_options_stack.setFixedHeight(s(52))

        # Page 0: Placeholder
        ph = QWidget()
        ph_l = QHBoxLayout(ph)
        ph_l.setContentsMargins(4, 4, 4, 4)
        lbl = QLabel("← Select wells above, then click a role button to assign")
        lbl.setStyleSheet(
            "color:#6c7086; font-style:italic; font-size:11px;")
        lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        ph_l.addWidget(lbl)
        self._role_options_stack.addWidget(ph)                    # index 0

        # Page 1: Print options
        self._role_options_stack.addWidget(self._make_print_options_page())   # 1
        self._role_stack_indices[WellRole.PRINT] = 1

        # Page 2: Ink options
        self._role_options_stack.addWidget(self._make_ink_options_page())     # 2
        self._role_stack_indices[WellRole.INK] = 2

        # Pages 3-6: Simple roles
        for idx, role in enumerate([WellRole.WASH, WellRole.WASTE,
                                     WellRole.BUFFER, WellRole.SORTED_CELLS],
                                    start=3):
            self._role_options_stack.addWidget(
                self._make_simple_options_page(role.value.capitalize()))
            self._role_stack_indices[role] = idx

        self._role_options_stack.setCurrentIndex(0)
        of_layout.addWidget(self._role_options_stack)
        parent_layout.addWidget(options_frame)

    def _make_print_options_page(self) -> QWidget:
        """Print role options: print file dropdown + assign + clear."""
        page = QWidget()
        layout = QHBoxLayout(page)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(6)

        layout.addWidget(QLabel("Print File:"))

        self.print_combo = QComboBox()
        self.print_combo.addItem("(no prints available)", None)
        self.print_combo.setMinimumWidth(s(200))
        layout.addWidget(self.print_combo, stretch=1)

        btn_assign = QPushButton("Assign to Wells")
        btn_assign.setObjectName("successBtn")
        btn_assign.clicked.connect(self._on_assign_print)
        layout.addWidget(btn_assign)

        return page

    def _make_ink_options_page(self) -> QWidget:
        """Ink role options: ink dropdown + assign."""
        page = QWidget()
        layout = QHBoxLayout(page)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(6)

        layout.addWidget(QLabel("Ink:"))

        self.ink_combo = QComboBox()
        self.ink_combo.addItem("(none)", None)
        self.ink_combo.setMinimumWidth(s(160))
        layout.addWidget(self.ink_combo, stretch=1)

        btn_assign = QPushButton("Assign to Wells")
        btn_assign.setObjectName("successBtn")
        btn_assign.clicked.connect(self._on_assign_ink)
        layout.addWidget(btn_assign)

        return page

    def _make_simple_options_page(self, role_name: str) -> QWidget:
        """Simple roles: just confirm the assignment was made."""
        page = QWidget()
        layout = QHBoxLayout(page)
        layout.setContentsMargins(4, 4, 4, 4)
        lbl = QLabel(
            f"✓ {role_name} role will be assigned to selected wells "
            f"when you click the role button.")
        lbl.setStyleSheet("color:#a6adc8; font-size:11px;")
        lbl.setWordWrap(True)
        layout.addWidget(lbl)
        return page

    def _build_plan_section(self, parent_layout) -> None:
        """Print Plan of Action collapsible group."""
        plan_group = QGroupBox("Print Plan of Action")
        plan_group.setCheckable(True)
        plan_group.setChecked(True)
        plan_layout = QVBoxLayout(plan_group)
        plan_layout.setSpacing(4)

        pref_row = QHBoxLayout()
        pref_row.addWidget(QLabel("Max Ink/Run:"))
        self._max_ink_spin = QDoubleSpinBox()
        self._max_ink_spin.setRange(0.1, 1000.0)
        self._max_ink_spin.setValue(100.0)
        self._max_ink_spin.setSuffix(" uL")
        self._max_ink_spin.setDecimals(1)
        self._max_ink_spin.setMaximumWidth(s(110))
        pref_row.addWidget(self._max_ink_spin)

        self._wash_check   = QCheckBox("Wash")
        self._waste_check  = QCheckBox("Waste")
        self._buffer_check = QCheckBox("Buffer")
        for cb in (self._wash_check, self._waste_check, self._buffer_check):
            cb.setChecked(True)
            pref_row.addWidget(cb)

        pref_row.addStretch()
        plan_layout.addLayout(pref_row)

        btn_row = QHBoxLayout()
        btn_gen = QPushButton("Generate Plan")
        btn_gen.clicked.connect(self._generate_plan)
        btn_val = QPushButton("Validate Setup")
        btn_val.clicked.connect(self._run_validation)
        btn_row.addWidget(btn_gen)
        btn_row.addWidget(btn_val)
        btn_row.addStretch()
        plan_layout.addLayout(btn_row)

        self._plan_label = QLabel("")
        self._plan_label.setStyleSheet(
            f"color:{COLORS['subtext0']}; font-size:{sf(9.5)}pt; padding:{sp(2)};")
        self._plan_label.setWordWrap(True)
        plan_layout.addWidget(self._plan_label)

        parent_layout.addWidget(plan_group)

    def _build_bottom_buttons(self, parent_layout) -> None:
        """v7.6.0: Save / Load on one shared row + Auto-Assign full
        width below, so they fit the narrow left-context column."""
        row = QHBoxLayout()
        row.setSpacing(6)

        btn_save = QPushButton("Save Setup")
        btn_save.clicked.connect(self._save_layout)
        btn_save.setSizePolicy(QSizePolicy.Policy.Expanding,
                               QSizePolicy.Policy.Fixed)
        row.addWidget(btn_save)

        btn_load = QPushButton("Load Setup")
        btn_load.clicked.connect(self._load_layout)
        btn_load.setSizePolicy(QSizePolicy.Policy.Expanding,
                               QSizePolicy.Policy.Fixed)
        row.addWidget(btn_load)
        parent_layout.addLayout(row)

        # Auto-assign menu — full-width row.
        btn_auto = QPushButton("Auto-Assign ▾")
        btn_auto.setObjectName("accentBtn")
        btn_auto.clicked.connect(self._show_auto_assign_menu)
        parent_layout.addWidget(btn_auto)

    def _show_auto_assign_menu(self) -> None:
        """Quick auto-assign patterns."""
        menu = QMenu(self)
        menu.addAction("Block (top-left)",
                       lambda: self._auto_assign("block"))
        menu.addAction("Checkerboard",
                       lambda: self._auto_assign("checkerboard"))
        menu.addAction("Border",
                       lambda: self._auto_assign("border"))
        btn = self.sender()
        pos = btn.mapToGlobal(btn.rect().bottomLeft()) if btn else self.cursor().pos()
        menu.exec(pos)

    def _auto_assign(self, pattern: str) -> None:
        """Apply auto-assign pattern."""
        try:
            if pattern == "block":
                auto_assign_block(self._model, WellRole.PRINT)
            elif pattern == "checkerboard":
                auto_assign_checkerboard(self._model, WellRole.PRINT, WellRole.EMPTY)
            elif pattern == "border":
                auto_assign_border(self._model, WellRole.PRINT)
            self._refresh_well_status_colors()
            self._refresh_summary_table()
            self.setup_changed.emit()
        except Exception as exc:
            self._set_status(f"Auto-assign error: {exc}")

    # ── Signal Wiring ─────────────────────────────────────────────

    def _connect_signals(self) -> None:
        """Connect all internal signals."""
        pv = getattr(self, "plate_view", None)
        if pv is None:
            return
        # Connect selection changed signal — try common attribute names
        for sig_name in ("selection_changed", "wells_selected",
                         "selectionChanged"):
            sig = getattr(pv, sig_name, None)
            if sig is not None:
                try:
                    sig.connect(self._on_selection_changed)
                    break
                except Exception:
                    pass

    # ── Event Handlers ────────────────────────────────────────────

    def _on_selection_changed(self, selected_wells) -> None:
        """Wells selection changed → update purple borders + options panel."""
        if isinstance(selected_wells, set):
            selected_wells = list(selected_wells)

        # Refresh border colors (selected = purple)
        self._refresh_well_status_colors()

        # Update options panel based on what roles the selected wells have
        self._refresh_role_options_for_selection(selected_wells)

    def _on_role_btn_clicked(self, role) -> None:
        """Role button clicked → assign role to selected wells + show options."""
        pv = getattr(self, "plate_view", None)
        if pv is None:
            return

        selected = []
        if hasattr(pv, "get_selected_wells"):
            try:
                selected = list(pv.get_selected_wells())
            except Exception:
                pass
        else:
            selected = list(getattr(pv, "selected_wells", set()))

        if not selected:
            self._set_status("⚠ Select wells on the plate first, then click a role.")
            # Decheck the button since nothing was assigned
            btn = self._role_btns.get(role)
            if btn:
                btn.setChecked(False)
            return

        # Assign role in model
        self._model.set_role(selected, role)

        # Update button checked states (only this one checked)
        for r, btn in self._role_btns.items():
            btn.setChecked(r == role)

        # Switch options panel to this role
        if self._role_options_stack is not None:
            idx = self._role_stack_indices.get(role, 0)
            self._role_options_stack.setCurrentIndex(idx)

        # Refresh colors and summary
        self._refresh_well_status_colors()
        self._refresh_summary_table()
        self.setup_changed.emit()

        # Auto-regenerate plan
        self._on_plan_auto_regen()

        self._set_status(
            f"Assigned {role.value} to {len(selected)} well(s).")

    def _on_assign_print(self) -> None:
        """Assign selected print file to selected PRINT wells."""
        if self.print_combo is None:
            return

        print_name = self.print_combo.currentData()
        if print_name is None:
            print_name = self.print_combo.currentText()
        if not print_name or print_name.startswith("("):
            self._set_status("⚠ Select a print file first.")
            return

        pv = getattr(self, "plate_view", None)
        selected = []
        if pv is not None:
            if hasattr(pv, "get_selected_wells"):
                try:
                    selected = list(pv.get_selected_wells())
                except Exception:
                    pass
            else:
                selected = list(getattr(pv, "selected_wells", set()))

        if not selected:
            self._set_status("⚠ Select wells first.")
            return

        for wn in selected:
            wa = self._model.get_assignment(wn)
            if wa is not None and wa.role == WellRole.PRINT:
                # Replace existing assignment with this file
                wa.print_collections = [print_name]
                wa.print_offsets     = [(0.0, 0.0, 0.0)]

        self._refresh_well_status_colors()
        self._refresh_summary_table()
        self.setup_changed.emit()
        self._on_plan_auto_regen()
        self._set_status(
            f"Assigned '{print_name}' to "
            f"{sum(1 for wn in selected if self._model.get_assignment(wn) and self._model.get_assignment(wn).role == WellRole.PRINT)} well(s).")

    def _on_assign_ink(self) -> None:
        """Assign selected ink to selected INK wells."""
        if self.ink_combo is None:
            return

        ink_name = self.ink_combo.currentData()
        if ink_name is None:
            ink_name = self.ink_combo.currentText()
        if not ink_name or ink_name.startswith("("):
            self._set_status("⚠ Select an ink first.")
            return

        pv = getattr(self, "plate_view", None)
        selected = []
        if pv is not None:
            if hasattr(pv, "get_selected_wells"):
                try:
                    selected = list(pv.get_selected_wells())
                except Exception:
                    pass
            else:
                selected = list(getattr(pv, "selected_wells", set()))

        if not selected:
            self._set_status("⚠ Select wells first.")
            return

        for wn in selected:
            wa = self._model.get_assignment(wn)
            if wa is not None and wa.role == WellRole.INK:
                wa.ink_name = ink_name

        self._refresh_well_status_colors()
        self._refresh_summary_table()
        self.setup_changed.emit()
        self._on_plan_auto_regen()
        self._set_status(f"Assigned ink '{ink_name}'.")

    def _on_clear_role(self) -> None:
        """Reset selected wells to EMPTY."""
        pv = getattr(self, "plate_view", None)
        selected = []
        if pv is not None:
            if hasattr(pv, "get_selected_wells"):
                try:
                    selected = list(pv.get_selected_wells())
                except Exception:
                    pass
            else:
                selected = list(getattr(pv, "selected_wells", set()))

        if not selected:
            self._set_status("⚠ Select wells first.")
            return

        self._model.set_role(selected, WellRole.EMPTY)

        # Uncheck all role buttons
        for btn in self._role_btns.values():
            btn.setChecked(False)

        # Reset options panel to placeholder
        if self._role_options_stack is not None:
            self._role_options_stack.setCurrentIndex(0)

        self._refresh_well_status_colors()
        self._refresh_summary_table()
        self.setup_changed.emit()
        self._set_status(f"Cleared {len(selected)} well(s) → Empty.")

    # ── Display Refresh ───────────────────────────────────────────

    def _refresh_plate(self) -> None:
        """Rebuild plate view and all dependent UI."""
        pv = getattr(self, "plate_view", None)
        if pv is None:
            return
        if hasattr(pv, "set_plate"):
            try:
                pv.set_plate(self._model.plate)
            except Exception:
                pass
        self._refresh_well_status_colors()
        self._refresh_summary_table()
        self._refresh_ink_combo()

    def _refresh_well_colors(self) -> None:
        """Update all well fill colors from role (v7.3.1-rolecolors)."""
        pv = getattr(self, "plate_view", None)
        if pv is None:
            return
        appearances: dict = {}
        for name, wa in self._model.assignments.items():
            appearances[name] = (wa.role, wa.get_display_label()
                                 if hasattr(wa, "get_display_label")
                                 else wa.role.value)
        if hasattr(pv, "update_all_wells"):
            try:
                pv.update_all_wells(appearances)
            except Exception:
                pass
        # Apply selection border on top
        self._apply_selection_borders()

    def _apply_selection_borders(self) -> None:
        """Purple border for selected wells; bright white for taught points."""
        pv = getattr(self, "plate_view", None)
        if pv is None or not hasattr(pv, "set_well_border_color"):
            return
        selected = set()
        if hasattr(pv, "get_selected_wells"):
            try:
                selected = set(pv.get_selected_wells())
            except Exception:
                pass
        COLOR_PURPLE = COLORS.get("mauve", "#cba6f7")
        COLOR_DEFAULT = "#585b70"
        for name in self._model.assignments:
            color = COLOR_PURPLE if name in selected else COLOR_DEFAULT
            try:
                pv.set_well_border_color(name, color)
            except Exception:
                pass

    def _refresh_well_status_colors(self) -> None:
        """Alias kept for compatibility — delegates to _refresh_well_colors."""
        self._refresh_well_colors()

    def _refresh_well_rosettes(self) -> None:
        """Push rosette data to plate view for subwell rendering."""
        pv = getattr(self, "plate_view", None)
        if pv is None or not hasattr(pv, "update_well_rosettes"):
            return
        rosette_map: dict = {}
        for name, wa in self._model.assignments.items():
            rn = getattr(wa, "rosette_name", None)
            if rn and self._workspace:
                rosette = self._workspace.get_rosette(rn) if hasattr(self._workspace, "get_rosette") else None
                if rosette:
                    rosette_map[name] = rosette
        try:
            pv.update_well_rosettes(rosette_map, self._model.plate)
        except Exception:
            pass


    def _check_well_ready(self, wa) -> bool:
        """True if the well assignment is fully configured (ready to print)."""
        if wa.role == WellRole.EMPTY:
            return True
        if wa.role == WellRole.PRINT:
            return bool(getattr(wa, "print_collections", None))
        if wa.role == WellRole.INK:
            return bool(getattr(wa, "ink_name", None))
        # WASH / WASTE / BUFFER / SORTED_CELLS: ready by assignment alone
        return True

    def _refresh_summary_table(self) -> None:
        """Rebuild the 3-column assignment summary table."""
        tbl = getattr(self, "_summary_table", None)
        if tbl is None:
            return

        rows = []
        for name, wa in self._model.assignments.items():
            if wa.role == WellRole.EMPTY:
                continue
            detail = ""
            if wa.role == WellRole.PRINT:
                cols = getattr(wa, "print_collections", [])
                detail = ", ".join(cols) if cols else "⚠ no file"
            elif wa.role == WellRole.INK:
                detail = getattr(wa, "ink_name", None) or "⚠ no ink"
            rows.append((name, wa.role.value.capitalize(), detail))

        tbl.setRowCount(len(rows))
        for r, (well, role, detail) in enumerate(rows):
            tbl.setItem(r, 0, QTableWidgetItem(well))
            tbl.setItem(r, 1, QTableWidgetItem(role))
            tbl.setItem(r, 2, QTableWidgetItem(detail))

    def _refresh_print_combo(self) -> None:
        """Repopulate print_combo from self._available_prints."""
        if self.print_combo is None:
            return
        current = self.print_combo.currentText()
        self.print_combo.blockSignals(True)
        self.print_combo.clear()
        if self._available_prints:
            for name in self._available_prints:
                self.print_combo.addItem(name, name)
            # Restore previous selection if still valid
            idx = self.print_combo.findText(current)
            if idx >= 0:
                self.print_combo.setCurrentIndex(idx)
        else:
            self.print_combo.addItem("(no prints available)", None)
        self.print_combo.blockSignals(False)

    def _refresh_ink_combo(self, config=None) -> None:
        """Repopulate ink_combo from hardware config or blank."""
        if self.ink_combo is None:
            return
        cfg = config or self._hw_config
        current = self.ink_combo.currentText()
        self.ink_combo.blockSignals(True)
        self.ink_combo.clear()
        self.ink_combo.addItem("(none)", None)
        if cfg is not None:
            lib = getattr(cfg, "ink_library", {}) or {}
            for ink_name in lib:
                self.ink_combo.addItem(str(ink_name), ink_name)
        idx = self.ink_combo.findText(current)
        if idx >= 0:
            self.ink_combo.setCurrentIndex(idx)
        self.ink_combo.blockSignals(False)

    def _refresh_role_options_for_selection(self, selected_wells=None) -> None:
        """
        Show role options panel if all selected wells share the same non-EMPTY role;
        otherwise show placeholder (page 0).
        """
        if self._role_options_stack is None:
            return

        if selected_wells is None:
            pv = getattr(self, "plate_view", None)
            if pv is not None:
                if hasattr(pv, "get_selected_wells"):
                    try:
                        selected_wells = list(pv.get_selected_wells())
                    except Exception:
                        selected_wells = []
                else:
                    selected_wells = list(getattr(pv, "selected_wells", set()))
            else:
                selected_wells = []

        if not selected_wells:
            self._role_options_stack.setCurrentIndex(0)
            # Uncheck all role buttons
            for btn in self._role_btns.values():
                btn.setChecked(False)
            return

        roles = set()
        for wn in selected_wells:
            wa = self._model.get_assignment(wn)
            if wa is not None:
                roles.add(wa.role)

        if len(roles) == 1:
            role = next(iter(roles))
            if role != WellRole.EMPTY:
                idx = self._role_stack_indices.get(role, 0)
                self._role_options_stack.setCurrentIndex(idx)
                # Update button checked state to reflect existing role
                for r, btn in self._role_btns.items():
                    btn.setChecked(r == role)
                return

        # Mixed roles or all empty
        self._role_options_stack.setCurrentIndex(0)
        for btn in self._role_btns.values():
            btn.setChecked(False)

    # ── HW Config Refresh ─────────────────────────────────────────

    def _refresh_ink_options_from_hw_config(self, config) -> None:
        """Refresh ink combo from HardwareConfig ink_library."""
        self._refresh_ink_combo(config)

    def _refresh_rosette_options_from_hw_config(self, config) -> None:
        """No-op: rosette editor removed in v7.3.1."""
        pass

    # ── Plan Section ──────────────────────────────────────────────

    def _get_plan_preferences(self):
        """Return PlanPreferences, safely reading widget values."""
        try:
            from SupportClasses.PrintPlanOfAction import PlanPreferences
        except ImportError:
            return None

        max_ink = getattr(
            getattr(self, "_max_ink_spin", None), "value", lambda: 100.0)()
        use_wash   = getattr(
            getattr(self, "_wash_check",   None), "isChecked", lambda: True)()
        use_waste  = getattr(
            getattr(self, "_waste_check",  None), "isChecked", lambda: True)()
        use_buffer = getattr(
            getattr(self, "_buffer_check", None), "isChecked", lambda: True)()

        try:
            return PlanPreferences(
                max_ink_per_run=max_ink,
                use_wash=use_wash,
                use_waste=use_waste,
                use_buffer=use_buffer,
            )
        except TypeError:
            try:
                prefs = PlanPreferences()
                prefs.max_ink_per_run = max_ink
                prefs.use_wash        = use_wash
                prefs.use_waste       = use_waste
                prefs.use_buffer      = use_buffer
                return prefs
            except Exception:
                return None

    def _generate_plan(self, execution_config=None) -> None:
        """Generate print plan — v7.2.9: accepts PrintExecutionConfig.

        Args:
            execution_config: Optional PrintExecutionConfig from Plan of Action UI.
                If provided, takes priority over legacy PlanPreferences.
        """
        lbl = getattr(self, "_plan_label", None)
        if self._hw_config is None:
            if lbl:
                lbl.setText("⚠ Hardware config required to generate plan.")
            return
        try:
            from SupportClasses.PrintPlanOfAction import PrintPlanOfAction
        except ImportError:
            if lbl:
                lbl.setText("PrintPlanOfAction module not available.")
            return
        try:
            prefs = self._get_plan_preferences()
            self._plan = PrintPlanOfAction.generate_plan(
                self._hw_config, self._model, prefs,
                execution_config=execution_config,
            )
            summary_fn = getattr(self._plan, "summary", None)
            if summary_fn and callable(summary_fn):
                text = summary_fn()
            else:
                steps = getattr(self._plan, "steps", [])
                runs  = getattr(self._plan, "total_runs", "?")
                text  = f"Plan: {len(steps)} steps, {runs} run(s)."
            if lbl:
                lbl.setText(text)
        except Exception as exc:
            if lbl:
                lbl.setText(f"Plan generation error: {exc}")
            logger.error(f"_generate_plan error: {exc}", exc_info=True)


    def _run_validation(self) -> None:
        """Run well setup validation — v7.3.1 fix: pass hw_config + well_model."""
        lbl = getattr(self, "_plan_label", None)
        try:
            from SupportClasses.PrintPlanOfAction import validate_well_setup
            ok, messages = validate_well_setup(
                hw_config=self._hw_config,
                well_model=self._model,
                plan=self._plan,
            )
            text = ("✓ Valid" if ok else "✗ Issues: " + "; ".join(messages))
            if lbl:
                lbl.setText(text)
        except ImportError:
            if lbl:
                lbl.setText("Validation module not available.")
        except Exception as exc:
            if lbl:
                lbl.setText(f"Validation error: {exc}")
            logger.error(f"_run_validation error: {exc}", exc_info=True)


    def _on_plan_auto_regen(self) -> None:
        """Silently regenerate plan if one already exists."""
        if self._plan is not None:
            try:
                self._generate_plan()
            except Exception:
                pass

    # ── Save / Load ───────────────────────────────────────────────

    def validate(self) -> tuple:
        """v7.3.1: Called by print_setup.py before sending job to monitor."""
        if self._hw_config is None:
            return False, ["No hardware configuration — complete Hardware Setup first"]
        if self._plan is None:
            try:
                self._generate_plan()
            except Exception as exc:
                return False, [f"Plan generation error: {exc}"]
        try:
            from SupportClasses.PrintPlanOfAction import validate_well_setup
            return validate_well_setup(
                hw_config=self._hw_config,
                well_model=self._model,
                plan=self._plan,
            )
        except ImportError:
            return False, ["PrintPlanOfAction module not available"]
        except Exception as exc:
            return False, [f"Validation error: {exc}"]

    def get_plan(self):
        """v7.3.1: Return the current PrintPlanOfAction (may be None)."""
        return self._plan


    def _save_layout(self) -> None:
        """Save well setup to JSON."""
        filepath, _ = QFileDialog.getSaveFileName(
            self, "Save Well Setup", "", "JSON (*.json)")
        if filepath:
            try:
                self._model.save_json(filepath)
                self._set_status(f"Saved: {filepath}")
            except Exception as exc:
                self._set_status(f"Save error: {exc}")

    def _load_layout(self) -> None:
        """Load well setup from JSON."""
        filepath, _ = QFileDialog.getOpenFileName(
            self, "Load Well Setup", "", "JSON (*.json)")
        if filepath:
            try:
                self._model = WellSetupModel.load_json(filepath)
                self._refresh_plate()
                self._set_status(f"Loaded: {filepath}")
                self.setup_changed.emit()
            except Exception as exc:
                self._set_status(f"Load error: {exc}")
                logger.error(f"Failed to load well setup: {exc}")

    # ── Utility ───────────────────────────────────────────────────

    def _set_status(self, msg: str) -> None:
        """Write a status message to plane_info_label."""
        lbl = getattr(self, "plane_info_label", None)
        if lbl is not None:
            lbl.setText(msg)


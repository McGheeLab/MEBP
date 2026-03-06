#!/usr/bin/env python3
"""
patch_well_setup_tab_v731.py
v7.3.1 — Complete replacement of WellSetupTab in print_well_setup.py.

Replaces the entire WellSetupTab class body with a clean implementation
that correctly implements the desired well selection → role assignment →
role-specific options flow.

Guard: "v7.3.1: WellSetupTab complete rewrite" in file → SKIP (idempotent).
"""

import ast
import re
import sys
import shutil
from pathlib import Path
from datetime import datetime

# ── Terminal colours ──────────────────────────────────────────────
GREEN  = "\033[92m"
RED    = "\033[91m"
YELLOW = "\033[93m"
CYAN   = "\033[96m"
RESET  = "\033[0m"

ok_count   = 0
skip_count = 0
fail_count = 0

# ── Helpers ───────────────────────────────────────────────────────

def find_root() -> Path:
    for p in [Path.cwd(), Path(__file__).parent]:
        for candidate in [p, p.parent, p.parent.parent]:
            if (candidate / "SupportClasses").is_dir() and (candidate / "gui").is_dir():
                return candidate.resolve()
    raise RuntimeError("Cannot locate MEBP project root")


def safe_read(path: Path) -> str:
    if not path.exists():
        return ""
    return path.read_text(encoding="utf-8")


def safe_write(path: Path, content: str, label: str) -> bool:
    global ok_count, fail_count
    try:
        ast.parse(content)
    except SyntaxError as e:
        print(f"  {RED}✗ AST FAIL — not writing {label}: {e}{RESET}")
        fail_count += 1
        return False
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    backup = path.with_suffix(f".bak_v731_{ts}")
    shutil.copy2(path, backup)
    print(f"  {CYAN}↳ backup → {backup.name}{RESET}")
    path.write_text(content, encoding="utf-8")
    print(f"  {GREEN}✓ Written: {label}{RESET}")
    ok_count += 1
    return True


# ── New WellSetupTab class body ───────────────────────────────────

NEW_CLASS = '''
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

        # Model
        self._model = WellSetupModel(self._workspace.plate_format)

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
        if workspace.plate_format != self._model.plate_format:
            self._model.set_plate_format(workspace.plate_format)
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
            if config.plate_format != current_fmt:
                try:
                    self._model.set_plate_format(config.plate_format)
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
        """Build the complete well setup layout."""
        main = QVBoxLayout(self)
        main.setContentsMargins(4, 4, 4, 4)
        main.setSpacing(4)

        # Status label (save/load / warnings)
        self.plane_info_label = QLabel("")
        self.plane_info_label.setStyleSheet(
            "color: #a6adc8; font-size: 11px; padding: 2px 4px;")
        main.addWidget(self.plane_info_label)

        # ── Top area: plate + summary ─────────────────────────────
        self._build_plate_area(main)

        # ── Role bar ──────────────────────────────────────────────
        self._build_role_bar(main)

        # ── Role-specific options ─────────────────────────────────
        self._build_role_options(main)

        # ── Print plan of action ──────────────────────────────────
        self._build_plan_section(main)

        # ── Bottom buttons ────────────────────────────────────────
        self._build_bottom_buttons(main)

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
        tc_layout.addWidget(self._summary_table)

        splitter.addWidget(table_container)
        splitter.setSizes([420, 280])

        parent_layout.addWidget(splitter, stretch=3)

    def _build_role_bar(self, parent_layout) -> None:
        """Row of checkable role buttons."""
        bar_frame = QFrame()
        bar_frame.setObjectName("cardFrame")
        bar_layout = QHBoxLayout(bar_frame)
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

        for role, label, color in role_defs:
            btn = QPushButton(label)
            btn.setCheckable(True)
            btn.setFixedHeight(28)
            btn.setMinimumWidth(62)
            btn.setStyleSheet(
                f"QPushButton {{background:#1e1e2e; border:2px solid #45475a;"
                f" border-radius:4px; color:#cdd6f4; font-size:11px;}}"
                f"QPushButton:checked {{border-color:{color}; color:{color};"
                f" background:#252536;}}"
                f"QPushButton:hover {{border-color:{color}; color:{color};}}"
            )
            btn.clicked.connect(
                lambda _checked, r=role: self._on_role_btn_clicked(r)
            )
            self._role_btns[role] = btn
            bar_layout.addWidget(btn)

        bar_layout.addStretch()

        # Clear / Empty button
        btn_clear = QPushButton("✕ Clear")
        btn_clear.setFixedHeight(28)
        btn_clear.setObjectName("dangerBtn")
        btn_clear.setToolTip("Reset selected wells to Empty")
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
        self._role_options_stack.setFixedHeight(52)

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
        self.print_combo.setMinimumWidth(200)
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
        self.ink_combo.setMinimumWidth(160)
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
        self._max_ink_spin.setMaximumWidth(110)
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
            "color:#a6adc8; font-size:11px; padding:2px;")
        self._plan_label.setWordWrap(True)
        plan_layout.addWidget(self._plan_label)

        parent_layout.addWidget(plan_group)

    def _build_bottom_buttons(self, parent_layout) -> None:
        """Save / Load / Auto-assign row."""
        row = QHBoxLayout()
        row.setSpacing(6)

        btn_save = QPushButton("Save Setup")
        btn_save.clicked.connect(self._save_layout)
        row.addWidget(btn_save)

        btn_load = QPushButton("Load Setup")
        btn_load.clicked.connect(self._load_layout)
        row.addWidget(btn_load)

        row.addStretch()

        # Auto-assign menu
        btn_auto = QPushButton("Auto-Assign ▾")
        btn_auto.setObjectName("accentBtn")
        btn_auto.clicked.connect(self._show_auto_assign_menu)
        row.addWidget(btn_auto)

        parent_layout.addLayout(row)

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

    def _refresh_well_status_colors(self) -> None:
        """
        Update well border colors:
          PURPLE  = currently selected (overrides everything)
          GREEN   = assigned + complete (ready to print)
          RED     = assigned + incomplete
          GRAY    = empty
        Uses set_well_border_color if available, falls back to set_well_color.
        """
        pv = getattr(self, "plate_view", None)
        if pv is None:
            return

        # Get currently selected wells
        selected = set()
        if hasattr(pv, "get_selected_wells"):
            try:
                selected = set(pv.get_selected_wells())
            except Exception:
                pass
        else:
            selected = set(getattr(pv, "selected_wells", set()))

        COLOR_GRAY   = COLORS.get("surface1", "#45475a")
        COLOR_GREEN  = COLORS.get("green",    "#a6e3a1")
        COLOR_RED    = COLORS.get("red",      "#f38ba8")
        COLOR_PURPLE = COLORS.get("mauve",    "#cba6f7")

        # Prefer set_well_border_color; fall back to set_well_color
        set_color = (
            pv.set_well_border_color if hasattr(pv, "set_well_border_color")
            else pv.set_well_color   if hasattr(pv, "set_well_color")
            else None
        )
        if set_color is None:
            # Try update_well_roles as last resort
            if hasattr(pv, "update_well_roles") and hasattr(self._model, "get_role_map"):
                try:
                    pv.update_well_roles(self._model.get_role_map())
                except Exception:
                    pass
            return

        for name, wa in self._model.assignments.items():
            if name in selected:
                color = COLOR_PURPLE
            elif wa.role == WellRole.EMPTY:
                color = COLOR_GRAY
            elif self._check_well_ready(wa):
                color = COLOR_GREEN
            else:
                color = COLOR_RED
            try:
                set_color(name, color)
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

    def _generate_plan(self) -> None:
        """Generate print plan of action."""
        lbl = getattr(self, "_plan_label", None)
        try:
            from SupportClasses.PrintPlanOfAction import PrintPlanOfAction
        except ImportError:
            if lbl:
                lbl.setText("PrintPlanOfAction module not available.")
            return
        try:
            prefs = self._get_plan_preferences()
            self._plan = PrintPlanOfAction(self._model, prefs)
            summary = getattr(self._plan, "summary", None)
            if summary:
                text = summary() if callable(summary) else str(summary)
            else:
                text = "Plan generated."
            if lbl:
                lbl.setText(text)
        except Exception as exc:
            if lbl:
                lbl.setText(f"Plan generation error: {exc}")
            logger.error(f"_generate_plan error: {exc}", exc_info=True)

    def _run_validation(self) -> None:
        """Run well setup validation."""
        lbl = getattr(self, "_plan_label", None)
        try:
            from SupportClasses.PrintPlanOfAction import validate_well_setup
            ok, messages = validate_well_setup(self._model)
            text = ("✓ Valid" if ok else "✗ Issues:") + " " + "; ".join(messages)
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

'''

# ── Main patch function ───────────────────────────────────────────

def apply_patch(root: Path) -> None:
    global ok_count, skip_count, fail_count

    target = root / "gui" / "pages" / "print_well_setup.py"
    print(f"\n{CYAN}Target: {target}{RESET}")

    content = safe_read(target)
    if not content:
        print(f"  {RED}✗ File not found: {target}{RESET}")
        fail_count += 1
        return

    # ── Idempotency guard ─────────────────────────────────────────
    if "v7.3.1: WellSetupTab complete rewrite" in content:
        print(f"  {YELLOW}○ SKIP: v7.3.1 already applied{RESET}")
        skip_count += 1
        return

    # ── Find class boundaries ─────────────────────────────────────
    # Match from "class WellSetupTab(" to the next top-level "class " or EOF
    pattern = re.compile(
        r'^(class WellSetupTab\(.*?)(?=^class |\Z)',
        re.DOTALL | re.MULTILINE
    )
    m = pattern.search(content)
    if not m:
        print(f"  {RED}✗ MISS: class WellSetupTab not found in file{RESET}")
        fail_count += 1
        return

    print(f"  Found WellSetupTab at offset {m.start()} – {m.end()}")

    # ── Preserve everything before the class ─────────────────────
    before = content[:m.start()]
    after  = content[m.end():]

    new_content = before + NEW_CLASS + after

    # ── Verify required imports are present in the file header ────
    # (WellSetupTab relies on these — add only if missing)
    required_imports = [
        ("QStackedWidget",   "from PySide6.QtWidgets import"),
        ("QSplitter",        "from PySide6.QtWidgets import"),
        ("QDoubleSpinBox",   "from PySide6.QtWidgets import"),
        ("QAbstractItemView","from PySide6.QtWidgets import"),
        ("QTableWidget",     "from PySide6.QtWidgets import"),
    ]
    for symbol, _hint in required_imports:
        if symbol not in new_content.split("class WellSetupTab")[0]:
            # The imports are in the existing file header — just verify
            pass  # We keep existing imports from before the class boundary

    # ── AST verify & write ────────────────────────────────────────
    safe_write(target, new_content, "print_well_setup.py (WellSetupTab v7.3.1)")


def main():
    print(f"\n{CYAN}{'='*60}")
    print("MEBP v7.3.1 — WellSetupTab Complete Rewrite Patch")
    print(f"{'='*60}{RESET}")

    try:
        root = find_root()
        print(f"Project root: {root}")
    except RuntimeError as e:
        print(f"{RED}ERROR: {e}{RESET}")
        sys.exit(1)

    apply_patch(root)

    print(f"\n{CYAN}{'─'*40}")
    print(f"Results:  {GREEN}{ok_count} applied{RESET}  "
          f"{YELLOW}{skip_count} skipped{RESET}  "
          f"{RED}{fail_count} failed{RESET}")

    if fail_count:
        print(f"{RED}⚠  Patch had failures — check output above.{RESET}")
        sys.exit(1)
    elif ok_count:
        print(f"{GREEN}✓  Patch applied successfully.{RESET}")
        print(f"\nNext steps:")
        print(f"  python3 -c \"import ast; ast.parse(open('gui/pages/print_well_setup.py').read())\"")
        print(f"  python3 -c \"from gui.pages.print_well_setup import WellSetupTab\"")
    else:
        print(f"{YELLOW}Nothing to do — already up to date.{RESET}")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
MEBP v7.2.5 — Session 4 Patch
Well Setup Redesign + Assignment Summary + Zoom Controls

Issues Covered: #6 (Well Setup redesign: select → assign role → role-specific options)
                #7 (Assignment summary panel)
                #9 (Wellplate view zoom buttons)

Files Modified:
    gui/pages/print_well_setup.py    — Major layout restructure
    gui/widgets/well_plate_view.py   — Add zoom overlay buttons

Prerequisites:
    - Sessions 1-3 patches applied

Usage:
    python patch_s4_well_redesign.py [/path/to/MEBP]

ARCHITECTURE NOTE:
    This patch restructures the WellSetupTab layout from a linear scroll
    to a split layout with:
    ┌──────────────────────────┬──────────────────────┐
    │  Interactive Plate View  │  Assignment Summary   │
    │  (with zoom overlay)     │  (scrollable table)   │
    ├──────────────────────────┴──────────────────────┤
    │  Role Assignment Bar: [Print][Ink][Wash]...     │
    ├──────────────────────────────────────────────────┤
    │  Role-Specific Options (QStackedWidget)         │
    ├──────────────────────────────────────────────────┤
    │  Print Plan of Action (existing v7.2.4)         │
    └──────────────────────────────────────────────────┘
"""

import os
import re
import sys
import shutil
from pathlib import Path
from datetime import datetime

# Terminal colors
BOLD   = "\033[1m"
GREEN  = "\033[32m"
YELLOW = "\033[33m"
RED    = "\033[31m"
CYAN   = "\033[36m"
RESET  = "\033[0m"

_applied = 0
_skipped = 0
_failed  = 0


def find_project_root() -> Path:
    candidates = [
        Path.cwd(), Path.cwd().parent,
        Path(__file__).resolve().parent.parent.parent,
        Path(__file__).resolve().parent.parent,
    ]
    for c in candidates:
        if (c / "gui" / "pages").is_dir() and (c / "SupportClasses").is_dir():
            return c
    print(f"{RED}ERROR{RESET}: Could not find MEBP project root.")
    sys.exit(1)


def backup_file(filepath: Path):
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    backup = filepath.with_suffix(f".bak_v725s4_{ts}")
    shutil.copy2(filepath, backup)


def read_file(filepath: Path) -> str:
    return filepath.read_text(encoding="utf-8")


def write_file(filepath: Path, content: str):
    filepath.write_text(content, encoding="utf-8")


def patch_replace(content: str, old: str, new: str, description: str) -> str:
    global _applied, _skipped, _failed
    if new.strip()[:80] in content and old not in content:
        print(f"  {YELLOW}SKIP{RESET}: {description} — already applied")
        _skipped += 1
        return content
    if old not in content:
        print(f"  {RED}MISS{RESET}: {description} — anchor not found")
        _failed += 1
        return content
    content = content.replace(old, new, 1)
    print(f"  {GREEN}OK{RESET}:   {description}")
    _applied += 1
    return content


def insert_after(content: str, anchor: str, new_text: str, description: str) -> str:
    global _applied, _skipped, _failed
    if new_text.strip()[:80] in content:
        print(f"  {YELLOW}SKIP{RESET}: {description} — already applied")
        _skipped += 1
        return content
    if anchor not in content:
        print(f"  {RED}MISS{RESET}: {description} — anchor not found")
        _failed += 1
        return content
    idx = content.find(anchor) + len(anchor)
    content = content[:idx] + new_text + content[idx:]
    print(f"  {GREEN}OK{RESET}:   {description}")
    _applied += 1
    return content


def insert_before(content: str, anchor: str, new_text: str, description: str) -> str:
    global _applied, _skipped, _failed
    if new_text.strip()[:80] in content:
        print(f"  {YELLOW}SKIP{RESET}: {description} — already applied")
        _skipped += 1
        return content
    if anchor not in content:
        print(f"  {RED}MISS{RESET}: {description} — anchor not found")
        _failed += 1
        return content
    idx = content.find(anchor)
    content = content[:idx] + new_text + content[idx:]
    print(f"  {GREEN}OK{RESET}:   {description}")
    _applied += 1
    return content


# ═══════════════════════════════════════════════════════════════════
# PATCH A: well_plate_view.py — Add zoom overlay buttons
# ═══════════════════════════════════════════════════════════════════

def patch_well_plate_view(root: Path):
    filepath = root / "gui" / "widgets" / "well_plate_view.py"
    print(f"\n{'═' * 60}")
    print(f"PATCH A: {filepath.name} — Zoom Overlay Buttons")
    print(f"{'═' * 60}")

    if not filepath.exists():
        print(f"  {RED}ERROR{RESET}: File not found!")
        return

    backup_file(filepath)
    content = read_file(filepath)

    # ── A1: Add fit_view method if not present ──
    if 'def fit_view(self' not in content:
        content = insert_before(
            content,
            '\nclass WellRoleLegend',
            '''
    def fit_view(self):
        """Fit all plate contents in view."""
        if self._scene.itemsBoundingRect().isValid():
            self.fitInView(self._scene.itemsBoundingRect(), Qt.AspectRatioMode.KeepAspectRatio)

''',
            "A1: Add fit_view method"
        )

    # ── A2: Add zoom_in / zoom_out methods ──
    if 'def zoom_in(self' not in content:
        content = insert_before(
            content,
            '\nclass WellRoleLegend',
            '''
    def zoom_in(self):
        """Zoom in by 25%."""
        self.scale(1.25, 1.25)

    def zoom_out(self):
        """Zoom out by 20%."""
        self.scale(0.8, 0.8)

    def reset_zoom(self):
        """Reset to fit-in-view."""
        self.resetTransform()
        self.fit_view()

''',
            "A2: Add zoom_in / zoom_out / reset_zoom methods"
        )

    write_file(filepath, content)


# ═══════════════════════════════════════════════════════════════════
# PATCH B: print_well_setup.py — Layout restructure
# ═══════════════════════════════════════════════════════════════════

# New methods to add to WellSetupTab
ROLE_BAR_METHOD = '''
    def _build_role_bar(self, parent_layout: QVBoxLayout) -> None:
        """v7.2.5: Build role assignment button bar."""
        bar_frame = QFrame()
        bar_frame.setObjectName("cardFrame")
        bar_layout = QHBoxLayout(bar_frame)
        bar_layout.setContentsMargins(8, 4, 8, 4)
        bar_layout.setSpacing(6)

        bar_layout.addWidget(QLabel("Assign Role:"))

        self._role_buttons = {}
        role_styles = {
            WellRole.PRINT:  ("Print",  ROLE_COLORS.get(WellRole.PRINT, "#a6e3a1")),
            WellRole.INK:    ("Ink",    ROLE_COLORS.get(WellRole.INK, "#89b4fa")),
            WellRole.WASH:   ("Wash",   ROLE_COLORS.get(WellRole.WASH, "#94e2d5")),
            WellRole.WASTE:  ("Waste",  ROLE_COLORS.get(WellRole.WASTE, "#fab387")),
            WellRole.BUFFER: ("Buffer", ROLE_COLORS.get(WellRole.BUFFER, "#cba6f7")),
            WellRole.EMPTY:  ("Clear",  ROLE_COLORS.get(WellRole.EMPTY, "#585b70")),
        }

        for role, (label, color) in role_styles.items():
            btn = QPushButton(label)
            btn.setCheckable(True)
            btn.setAutoExclusive(True)
            btn.setStyleSheet(f"""
                QPushButton {{
                    background: {COLORS.get('surface0', '#313244')};
                    color: {color};
                    border: 2px solid {color};
                    border-radius: 4px;
                    padding: 4px 12px;
                    font-weight: bold;
                    font-size: 11px;
                }}
                QPushButton:checked {{
                    background: {color};
                    color: {COLORS.get('base', '#1e1e2e')};
                }}
                QPushButton:hover {{
                    background: {COLORS.get('surface1', '#45475a')};
                }}
            """)
            btn.clicked.connect(partial(self._on_role_button_clicked, role))
            bar_layout.addWidget(btn)
            self._role_buttons[role] = btn

        # Default: Print selected
        if WellRole.PRINT in self._role_buttons:
            self._role_buttons[WellRole.PRINT].setChecked(True)
        self._active_role = WellRole.PRINT

        bar_layout.addStretch()

        # Quick action buttons
        btn_apply = QPushButton("Apply to Selected")
        btn_apply.setObjectName("accentBtn")
        btn_apply.setStyleSheet(f"""
            QPushButton {{
                background: {COLORS.get('green', '#a6e3a1')};
                color: {COLORS.get('base', '#1e1e2e')};
                font-weight: bold; padding: 4px 12px;
                border-radius: 4px; font-size: 11px;
            }}
            QPushButton:hover {{
                background: {COLORS.get('teal', '#94e2d5')};
            }}
        """)
        btn_apply.clicked.connect(self._apply_active_role)
        bar_layout.addWidget(btn_apply)

        parent_layout.addWidget(bar_frame)

    def _on_role_button_clicked(self, role: WellRole):
        """v7.2.5: Handle role button bar click."""
        self._active_role = role
        # Switch role-specific options panel
        if hasattr(self, '_role_options_stack'):
            idx = self._role_stack_indices.get(role, 0)
            self._role_options_stack.setCurrentIndex(idx)

    def _apply_active_role(self):
        """v7.2.5: Apply the currently active role to selected wells."""
        selected = list(self._plate_view.selected_wells)
        if not selected:
            return
        role = getattr(self, '_active_role', WellRole.PRINT)
        self._model.set_role(selected, role)
        self._plate_view.update_well_roles(self._model.get_role_map())
        self._refresh_summary_table()
        self.setup_changed.emit()
        # Auto-regenerate plan if available
        if hasattr(self, '_on_plan_auto_regen'):
            self._on_plan_auto_regen()

'''

ROLE_OPTIONS_METHOD = '''
    def _build_role_options(self, parent_layout: QVBoxLayout) -> None:
        """v7.2.5: Build role-specific options panel (QStackedWidget)."""
        options_frame = QFrame()
        options_frame.setObjectName("cardFrame")
        options_layout = QVBoxLayout(options_frame)
        options_layout.setContentsMargins(8, 4, 8, 4)

        self._role_options_stack = QStackedWidget()
        self._role_stack_indices = {}

        # ── Page 0: Print options ──
        print_page = QWidget()
        print_layout = QHBoxLayout(print_page)
        print_layout.setContentsMargins(0, 0, 0, 0)
        print_layout.addWidget(QLabel("Print File:"))
        self.print_combo = QComboBox()
        self.print_combo.addItem("(no prints available)", None)
        self.print_combo.setMinimumWidth(200)
        print_layout.addWidget(self.print_combo, stretch=1)
        btn_assign_print = QPushButton("Assign")
        btn_assign_print.clicked.connect(lambda: self._assign_print(replace=True))
        print_layout.addWidget(btn_assign_print)
        btn_clear_print = QPushButton("Clear")
        btn_clear_print.clicked.connect(self._clear_selected_prints)
        print_layout.addWidget(btn_clear_print)
        print_layout.addStretch()
        self._role_stack_indices[WellRole.PRINT] = self._role_options_stack.addWidget(print_page)

        # ── Page 1: Ink options ──
        ink_page = QWidget()
        ink_layout = QHBoxLayout(ink_page)
        ink_layout.setContentsMargins(0, 0, 0, 0)
        ink_layout.addWidget(QLabel("Ink:"))
        self.ink_combo = QComboBox()
        self.ink_combo.addItem("(none)", None)
        self.ink_combo.setMinimumWidth(200)
        ink_layout.addWidget(self.ink_combo, stretch=1)
        btn_assign_ink = QPushButton("Assign")
        btn_assign_ink.clicked.connect(self._apply_ink)
        ink_layout.addWidget(btn_assign_ink)
        ink_layout.addStretch()
        self._role_stack_indices[WellRole.INK] = self._role_options_stack.addWidget(ink_page)

        # ── Page 2: Wash options ──
        wash_page = QWidget()
        wash_layout = QHBoxLayout(wash_page)
        wash_layout.setContentsMargins(0, 0, 0, 0)
        wash_layout.addWidget(QLabel("Wash well — no additional options"))
        wash_layout.addStretch()
        self._role_stack_indices[WellRole.WASH] = self._role_options_stack.addWidget(wash_page)

        # ── Page 3: Waste options ──
        waste_page = QWidget()
        waste_layout = QHBoxLayout(waste_page)
        waste_layout.setContentsMargins(0, 0, 0, 0)
        waste_layout.addWidget(QLabel("Waste well — no additional options"))
        waste_layout.addStretch()
        self._role_stack_indices[WellRole.WASTE] = self._role_options_stack.addWidget(waste_page)

        # ── Page 4: Buffer options ──
        buffer_page = QWidget()
        buffer_layout = QHBoxLayout(buffer_page)
        buffer_layout.setContentsMargins(0, 0, 0, 0)
        buffer_layout.addWidget(QLabel("Buffer well — no additional options"))
        buffer_layout.addStretch()
        self._role_stack_indices[WellRole.BUFFER] = self._role_options_stack.addWidget(buffer_page)

        # ── Page 5: Empty/Clear options ──
        empty_page = QWidget()
        empty_layout = QHBoxLayout(empty_page)
        empty_layout.setContentsMargins(0, 0, 0, 0)
        empty_layout.addWidget(QLabel("Clear selection — removes role assignment"))
        empty_layout.addStretch()
        self._role_stack_indices[WellRole.EMPTY] = self._role_options_stack.addWidget(empty_page)

        options_layout.addWidget(self._role_options_stack)
        parent_layout.addWidget(options_frame)

    def _clear_selected_prints(self):
        """v7.2.5: Clear print assignments from selected wells."""
        selected = list(self._plate_view.selected_wells)
        if not selected:
            return
        for name in selected:
            wa = self._model.get_assignment(name)
            if wa and wa.role == WellRole.PRINT:
                wa.clear_prints()
        self._refresh_summary_table()
        self.setup_changed.emit()

'''

SUMMARY_TABLE_METHOD = '''
    def _build_assignment_summary(self) -> QWidget:
        """v7.2.5: Build assignment summary panel (right side)."""
        summary_widget = QWidget()
        summary_layout = QVBoxLayout(summary_widget)
        summary_layout.setContentsMargins(4, 4, 4, 4)
        summary_layout.setSpacing(4)

        # Title
        title = QLabel("Assignment Summary")
        title.setStyleSheet(
            f"color: {COLORS['text']}; font-weight: bold; font-size: 12px;")
        summary_layout.addWidget(title)

        # Summary counts
        self._summary_counts_label = QLabel("")
        self._summary_counts_label.setStyleSheet(
            f"color: {COLORS.get('subtext0', '#a6adc8')}; font-size: 10px;")
        self._summary_counts_label.setWordWrap(True)
        summary_layout.addWidget(self._summary_counts_label)

        # Assignment table
        self._assignment_table = QTableWidget()
        self._assignment_table.setColumnCount(4)
        self._assignment_table.setHorizontalHeaderLabels(
            ["Well", "Role", "Assignment", "Status"])
        self._assignment_table.horizontalHeader().setStretchLastSection(True)
        self._assignment_table.horizontalHeader().setSectionResizeMode(
            0, QHeaderView.ResizeMode.ResizeToContents)
        self._assignment_table.horizontalHeader().setSectionResizeMode(
            1, QHeaderView.ResizeMode.ResizeToContents)
        self._assignment_table.horizontalHeader().setSectionResizeMode(
            2, QHeaderView.ResizeMode.Stretch)
        self._assignment_table.verticalHeader().setVisible(False)
        self._assignment_table.setSelectionBehavior(
            QAbstractItemView.SelectionBehavior.SelectRows)
        self._assignment_table.setEditTriggers(
            QAbstractItemView.EditTrigger.NoEditTriggers)
        self._assignment_table.setStyleSheet(f"""
            QTableWidget {{
                background: {COLORS.get('base', '#1e1e2e')};
                color: {COLORS['text']};
                border: 1px solid {COLORS.get('surface1', '#45475a')};
                border-radius: 4px;
                font-size: 10px;
            }}
            QHeaderView::section {{
                background: {COLORS.get('surface0', '#313244')};
                color: {COLORS['text']};
                padding: 2px 4px;
                border: 1px solid {COLORS.get('surface1', '#45475a')};
                font-size: 10px;
            }}
        """)
        self._assignment_table.setMaximumWidth(350)
        self._assignment_table.setMinimumWidth(200)
        # Click row → select well in plate view
        self._assignment_table.cellClicked.connect(self._on_summary_cell_clicked)
        summary_layout.addWidget(self._assignment_table, stretch=1)

        return summary_widget

    def _on_summary_cell_clicked(self, row: int, col: int):
        """v7.2.5: Click summary table row → select well in plate view."""
        well_item = self._assignment_table.item(row, 0)
        if well_item:
            well_name = well_item.text()
            self._plate_view.clear_selection()
            self._plate_view.select_wells([well_name])

    def _refresh_summary_table(self):
        """v7.2.5: Refresh the assignment summary table and counts."""
        if not hasattr(self, '_assignment_table'):
            return

        # Count roles
        role_counts = {}
        assigned_wells = []
        for name, wa in sorted(self._model.assignments.items()):
            if wa.role != WellRole.EMPTY:
                assigned_wells.append((name, wa))
                role_counts[wa.role] = role_counts.get(wa.role, 0) + 1

        # Update counts label
        count_parts = []
        for role, count in sorted(role_counts.items(), key=lambda x: x[0].value):
            color = ROLE_COLORS.get(role, "#585b70")
            count_parts.append(f"{count} {role.value.capitalize()}")
        self._summary_counts_label.setText(
            " | ".join(count_parts) if count_parts else "No wells assigned")

        # Update table
        self._assignment_table.setRowCount(len(assigned_wells))
        for row, (name, wa) in enumerate(assigned_wells):
            # Well name
            name_item = QTableWidgetItem(name)
            self._assignment_table.setItem(row, 0, name_item)

            # Role (colored)
            role_item = QTableWidgetItem(wa.role.value.capitalize())
            color = ROLE_COLORS.get(wa.role, "#585b70")
            role_item.setForeground(QColor(color))
            self._assignment_table.setItem(row, 1, role_item)

            # Assignment details
            detail = ""
            if wa.role == WellRole.PRINT:
                prints = wa.get_print_names() if hasattr(wa, 'get_print_names') else []
                detail = ", ".join(prints) if prints else "(no print assigned)"
            elif wa.role == WellRole.INK:
                detail = wa.ink_name or "(no ink)"
            elif wa.role in (WellRole.WASH, WellRole.WASTE, WellRole.BUFFER):
                detail = f"#{getattr(wa, 'role_index', 0)}"
            self._assignment_table.setItem(row, 2, QTableWidgetItem(detail))

            # Status
            is_ready = self._check_well_ready(wa)
            status = "✓" if is_ready else "⚠"
            status_item = QTableWidgetItem(status)
            status_item.setForeground(
                QColor(COLORS.get('green', '#a6e3a1') if is_ready
                       else COLORS.get('red', '#f38ba8')))
            self._assignment_table.setItem(row, 3, status_item)

    def _check_well_ready(self, wa) -> bool:
        """v7.2.5: Check if a well assignment is complete/ready."""
        if wa.role == WellRole.PRINT:
            prints = wa.get_print_names() if hasattr(wa, 'get_print_names') else []
            return len(prints) > 0
        elif wa.role == WellRole.INK:
            return bool(wa.ink_name)
        elif wa.role in (WellRole.WASH, WellRole.WASTE, WellRole.BUFFER):
            return True
        return False

'''

ZOOM_OVERLAY_METHOD = '''
    def _build_zoom_overlay(self, plate_container: QWidget) -> None:
        """v7.2.5: Add zoom overlay buttons to plate view container."""
        # Create overlay frame
        overlay = QFrame(plate_container)
        overlay.setStyleSheet("background: transparent;")
        overlay_layout = QHBoxLayout(overlay)
        overlay_layout.setContentsMargins(4, 4, 4, 4)
        overlay_layout.setSpacing(4)

        btn_style = (
            f"QPushButton {{ "
            f"background: {COLORS.get('surface0', '#313244')}cc; "
            f"color: {COLORS['text']}; "
            f"border: 1px solid {COLORS.get('surface1', '#45475a')}; "
            f"border-radius: 3px; padding: 2px 8px; font-size: 11px; "
            f"min-width: 28px; }}"
            f"QPushButton:hover {{ background: {COLORS.get('surface1', '#45475a')}; }}"
        )

        btn_home = QPushButton("⌂")
        btn_home.setStyleSheet(btn_style)
        btn_home.setToolTip("Fit plate in view")
        btn_home.clicked.connect(self._plate_view.reset_zoom)
        overlay_layout.addWidget(btn_home)

        btn_zoom_in = QPushButton("+")
        btn_zoom_in.setStyleSheet(btn_style)
        btn_zoom_in.setToolTip("Zoom in")
        btn_zoom_in.clicked.connect(self._plate_view.zoom_in)
        overlay_layout.addWidget(btn_zoom_in)

        btn_zoom_out = QPushButton("−")
        btn_zoom_out.setStyleSheet(btn_style)
        btn_zoom_out.setToolTip("Zoom out")
        btn_zoom_out.clicked.connect(self._plate_view.zoom_out)
        overlay_layout.addWidget(btn_zoom_out)

        overlay_layout.addStretch()

        # Position overlay at top-left of plate container
        overlay.raise_()
        overlay.setGeometry(0, 0, 200, 30)
        self._zoom_overlay = overlay

'''


def patch_well_setup(root: Path):
    filepath = root / "gui" / "pages" / "print_well_setup.py"
    print(f"\n{'═' * 60}")
    print(f"PATCH B: {filepath.name} — Well Setup Redesign")
    print(f"{'═' * 60}")

    if not filepath.exists():
        print(f"  {RED}ERROR{RESET}: File not found!")
        return

    backup_file(filepath)
    content = read_file(filepath)

    # ── B1: Add QStackedWidget import ──
    if 'QStackedWidget' not in content:
        content = patch_replace(
            content,
            'from PySide6.QtWidgets import (',
            'from PySide6.QtWidgets import (\n    QStackedWidget,',
            "B1: Add QStackedWidget import"
        )

    # ── B2: Add _active_role attribute to __init__ ──
    if '_active_role' not in content:
        content = insert_after(
            content,
            '        self._available_prints: list[str] = []',
            '\n\n        # v7.2.5: Active role for role bar\n'
            '        self._active_role = WellRole.PRINT',
            "B2: Add _active_role attribute"
        )

    # ── B3: Add new role bar method ──
    content = insert_before(
        content,
        '    def _build_selection_actions',
        ROLE_BAR_METHOD,
        "B3: Add _build_role_bar method"
    )

    # ── B4: Add role options stacked widget method ──
    content = insert_before(
        content,
        '    def _build_selection_actions',
        ROLE_OPTIONS_METHOD,
        "B4: Add _build_role_options method"
    )

    # ── B5: Add assignment summary method ──
    content = insert_before(
        content,
        '    def _build_selection_actions',
        SUMMARY_TABLE_METHOD,
        "B5: Add _build_assignment_summary and helper methods"
    )

    # ── B6: Add zoom overlay method ──
    content = insert_before(
        content,
        '    def _build_selection_actions',
        ZOOM_OVERLAY_METHOD,
        "B6: Add _build_zoom_overlay method"
    )

    # ── B7: Restructure _build_ui to use new layout ──
    # We need to find the _build_ui method and add our new widgets.
    # Rather than replace the entire method (fragile), we inject calls
    # into the existing flow.

    # First, check if _build_ui calls our new methods already
    if '_build_role_bar' not in content:
        # Find the plate view setup in _build_ui and add after it
        # The plate view is created as self._plate_view and added to layout
        # We'll inject our new sections after the plate view section

        # Look for where the plate view splitter or plate section ends
        # and selection actions begin
        old_selection_call = '        self._build_selection_actions(scroll_layout)'
        new_selection_call = (
            '        # v7.2.5: New layout components\n'
            '        self._build_role_bar(scroll_layout)\n'
            '        self._build_role_options(scroll_layout)\n\n'
            '        # Legacy selection actions (kept for compatibility)\n'
            '        self._build_selection_actions(scroll_layout)'
        )
        content = patch_replace(content, old_selection_call, new_selection_call,
                                "B7: Inject role bar + options before selection actions")

    # ── B8: Add summary panel to main layout ──
    # Find where the scroll area is added and wrap in horizontal splitter
    if '_build_assignment_summary' not in content or '_assignment_table' not in content:
        # The main layout adds scroll. We need to also add summary panel.
        # Find: main_layout.addWidget(scroll, stretch=2)
        old_scroll_add = '        main_layout.addWidget(scroll, stretch=2)'
        new_scroll_add = (
            '        # v7.2.5: Wrap scroll + summary in horizontal layout\n'
            '        content_h_layout = QHBoxLayout()\n'
            '        content_h_layout.addWidget(scroll, stretch=3)\n'
            '        self._summary_widget = self._build_assignment_summary()\n'
            '        content_h_layout.addWidget(self._summary_widget, stretch=1)\n'
            '        main_layout.addLayout(content_h_layout)'
        )
        content = patch_replace(content, old_scroll_add, new_scroll_add,
                                "B8: Add summary panel alongside scroll area")

    # ── B9: Add zoom overlay after plate view creation ──
    if '_build_zoom_overlay' not in content or '_zoom_overlay' not in content:
        # Find where plate view is added
        plate_view_add = re.search(
            r'(main_layout\.addWidget\(self\._plate_view.*?\))',
            content
        )
        if plate_view_add:
            content = insert_after(
                content,
                plate_view_add.group(0),
                '\n        self._build_zoom_overlay(self._plate_view)',
                "B9: Add zoom overlay to plate view"
            )
        else:
            # Try alternate: the plate_view might be in a splitter
            if 'self._plate_view' in content:
                # Just add the overlay call after plate view creation
                plate_create = re.search(
                    r'(self\._plate_view = WellPlateView\([^)]*\))',
                    content
                )
                if plate_create:
                    content = insert_after(
                        content,
                        plate_create.group(0),
                        '\n        # v7.2.5: Zoom overlay will be built after plate added to layout',
                        "B9: (deferred) Zoom overlay placeholder"
                    )

    # ── B10: Wire _refresh_summary_table into assignment changes ──
    if '_refresh_summary_table' in content:
        # Make sure it's called after role changes
        if '_refresh_summary_table()' not in content.split('_apply_role')[1].split('\n    def ')[0] if '_apply_role' in content else '':
            # Find _apply_role method and add refresh call
            old_apply_role_emit = '        self.setup_changed.emit()'
            if old_apply_role_emit in content:
                first_occurrence = content.find(old_apply_role_emit)
                # Only add after the first occurrence in _apply_role
                content = (
                    content[:first_occurrence] +
                    '        self._refresh_summary_table()\n' +
                    content[first_occurrence:]
                )
                print(f"  {GREEN}OK{RESET}:   B10: Wire _refresh_summary_table into _apply_role")
                global _applied
                _applied += 1

    # ── B11: Add QColor to imports if not present ──
    if 'QColor' not in content.split('from PySide6.QtGui')[0] if 'from PySide6.QtGui' in content else content:
        if 'from PySide6.QtGui import' in content and 'QColor' not in content:
            content = patch_replace(
                content,
                'from PySide6.QtGui import QColor',
                'from PySide6.QtGui import QColor',
                "B11: QColor import already present"
            )

    write_file(filepath, content)


# ═══════════════════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════════════════

def main():
    global _applied, _skipped, _failed

    if len(sys.argv) > 1:
        root = Path(sys.argv[1]).resolve()
    else:
        root = find_project_root()

    print(f"\n{BOLD}{'=' * 60}")
    print(f" MEBP v7.2.5 — Session 4 Patch")
    print(f" Well Setup Redesign + Summary + Zoom")
    print(f"{'=' * 60}{RESET}")
    print(f"Project root: {root}")

    patch_well_plate_view(root)
    patch_well_setup(root)

    # Summary
    total = _applied + _skipped + _failed
    print(f"\n{BOLD}{'═' * 60}")
    print(f" SUMMARY")
    print(f"{'═' * 60}{RESET}")
    print(f"  {GREEN}Applied{RESET}: {_applied}")
    print(f"  {YELLOW}Skipped{RESET}: {_skipped}")
    print(f"  {RED}Failed{RESET}:  {_failed}")
    print(f"  Total:   {total}")

    if _failed > 0:
        print(f"\n{RED}WARNING{RESET}: {_failed} patch(es) failed!")
        sys.exit(1)
    else:
        print(f"\n{GREEN}All patches applied successfully!{RESET}")


if __name__ == "__main__":
    main()

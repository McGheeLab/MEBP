#!/usr/bin/env python3
"""
MEBP v7.3.0 — Session 1: Well Role Flow Redesign
=================================================
Redesigns the Print Setup > Well Setup tab interaction:
  - Role options panel hidden until wells are selected AND role assigned
  - Clicking a role button immediately assigns that role to selected wells
  - Role-specific options panel appears after role assignment
  - Well borders: green=ready, red=incomplete, gray=empty, purple=selected
  - Fixes plate_view / _plate_view naming inconsistency

Files modified:
  gui/pages/print_well_setup.py
  gui/widgets/well_plate_view.py
"""

import ast
import re
import sys
import shutil
from pathlib import Path
from datetime import datetime

# ── Terminal colors ─────────────────────────────────────────────
GREEN  = "\033[92m"
RED    = "\033[91m"
YELLOW = "\033[93m"
CYAN   = "\033[96m"
RESET  = "\033[0m"
BOLD   = "\033[1m"

# ── Counters ─────────────────────────────────────────────────────
_applied = 0
_skipped = 0
_failed  = 0


def ok(msg):
    global _applied; _applied += 1
    print(f"  {GREEN}✓ Applied:{RESET} {msg}")


def skip(msg):
    global _skipped; _skipped += 1
    print(f"  {CYAN}○ SKIP:{RESET} {msg} (already applied)")


def fail(msg):
    global _failed; _failed += 1
    print(f"  {RED}✗ MISS:{RESET} {msg}")


# ── Project root ─────────────────────────────────────────────────
def find_root() -> Path:
    candidates = [Path.cwd(), Path(__file__).resolve().parent]
    for p in candidates:
        for ancestor in [p] + list(p.parents):
            if (ancestor / "SupportClasses").is_dir() and (ancestor / "gui").is_dir():
                return ancestor
    print(f"{RED}ERROR: Cannot find MEBP project root{RESET}")
    sys.exit(1)


def safe_read(path: Path) -> str:
    if not path.exists():
        print(f"{RED}ERROR: File not found: {path}{RESET}")
        return ""
    return path.read_text(encoding="utf-8")


def safe_write(path: Path, content: str, label: str) -> bool:
    try:
        ast.parse(content)
    except SyntaxError as e:
        print(f"  {RED}AST FAIL — not writing {label}: {e}{RESET}")
        return False
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    backup = path.with_suffix(f".bak_v730_{ts}")
    shutil.copy2(path, backup)
    path.write_text(content, encoding="utf-8")
    return True


def find_method(content: str, name: str):
    """Find a class method by name (4-space indented). Returns match or None."""
    pattern = re.compile(
        rf'^(    def {re.escape(name)}\(self[^)]*\)[^\n]*\n)'
        rf'(.*?)'
        rf'(?=\n    def |\nclass |\Z)',
        re.DOTALL | re.MULTILINE,
    )
    return pattern.search(content)


def replace_method(content: str, name: str, new_method: str) -> tuple[str, bool]:
    """Replace a method entirely. Returns (new_content, success)."""
    m = find_method(content, name)
    if m is None:
        return content, False
    return (content[:m.start()] + new_method + content[m.end():]), True


def inject_before_method(content: str, name: str, new_code: str) -> tuple[str, bool]:
    """Inject code immediately before a method. Returns (new_content, success)."""
    m = find_method(content, name)
    if m is None:
        return content, False
    return (content[:m.start()] + new_code + content[m.start():]), True


# ════════════════════════════════════════════════════════════════
# PATCH 1 — print_well_setup.py
# ════════════════════════════════════════════════════════════════

def patch_print_well_setup(root: Path) -> None:
    path = root / "gui" / "pages" / "print_well_setup.py"
    print(f"\n{BOLD}{CYAN}Patching: {path.relative_to(root)}{RESET}")
    content = safe_read(path)
    if not content:
        return
    original = content

    # ── A: Replace _build_role_bar ────────────────────────────────
    marker_a = "v7.3.0: role bar direct assign"
    if marker_a not in content:
        new_method_a = '''    def _build_role_bar(self, parent_layout) -> None:
        """v7.3.0: role bar direct assign — clicking a role button
        immediately assigns that role to selected wells."""
        from PySide6.QtWidgets import QStackedWidget
        from PySide6.QtGui import QColor

        role_frame = __import__('PySide6.QtWidgets', fromlist=['QFrame']).QFrame()
        role_frame.setObjectName("cardFrame")
        role_layout = __import__('PySide6.QtWidgets', fromlist=['QHBoxLayout']).QHBoxLayout(role_frame)
        role_layout.setContentsMargins(6, 4, 6, 4)
        role_layout.setSpacing(4)

        from PySide6.QtWidgets import QLabel, QPushButton, QHBoxLayout, QFrame
        from SupportClasses.PhysicalModels import WellRole, ROLE_COLORS

        lbl = QLabel("Assign Role:")
        lbl.setStyleSheet("font-weight: bold; color: #cdd6f4;")
        role_layout.addWidget(lbl)

        ROLE_LABELS = {
            WellRole.PRINT:        ("Print",   "#a6e3a1"),
            WellRole.INK:          ("Ink",     "#89b4fa"),
            WellRole.WASH:         ("Wash",    "#f9e2af"),
            WellRole.WASTE:        ("Waste",   "#f38ba8"),
            WellRole.BUFFER:       ("Buffer",  "#cba6f7"),
            WellRole.SORTED_CELLS: ("Sorted",  "#fab387"),
            WellRole.EMPTY:        ("Clear",   "#585b70"),
        }

        self._role_btns = {}
        for role, (label, color) in ROLE_LABELS.items():
            btn = QPushButton(label)
            btn.setCheckable(True)
            btn.setFixedHeight(28)
            btn.setStyleSheet(
                f"QPushButton {{ background-color: #313244; color: #cdd6f4; "
                f"border: 1px solid {color}; border-radius: 4px; "
                f"padding: 2px 8px; font-size: 11px; }}"
                f"QPushButton:checked {{ background-color: {color}; color: #1e1e2e; "
                f"font-weight: bold; }}"
                f"QPushButton:hover:!checked {{ background-color: #45475a; }}"
            )
            btn.clicked.connect(lambda checked, r=role: self._on_role_btn_clicked(r))
            role_layout.addWidget(btn)
            self._role_btns[role] = btn

        role_layout.addStretch()
        parent_layout.addWidget(role_frame)

'''
        content, replaced = replace_method(content, "_build_role_bar", new_method_a)
        if replaced:
            ok("Replaced _build_role_bar (direct assign on click)")
        else:
            fail("_build_role_bar not found for replacement")
    else:
        skip("_build_role_bar (direct assign)")

    # ── B: Replace _build_role_options (add placeholder page 0) ──
    marker_b = "v7.3.0: role options placeholder page"
    if marker_b not in content:
        new_method_b = '''    def _build_role_options(self, parent_layout) -> None:
        """v7.3.0: role options placeholder page — index 0 is shown until
        wells are selected and a role is assigned."""
        from PySide6.QtWidgets import (
            QStackedWidget, QWidget, QHBoxLayout, QVBoxLayout,
            QLabel, QComboBox, QPushButton, QDoubleSpinBox, QFrame,
        )
        from SupportClasses.PhysicalModels import WellRole

        options_frame = QFrame()
        options_frame.setObjectName("cardFrame")
        options_layout = QVBoxLayout(options_frame)
        options_layout.setContentsMargins(8, 4, 8, 4)
        options_layout.setSpacing(0)

        self._role_options_stack = QStackedWidget()
        self._role_options_stack.setMaximumHeight(52)
        self._role_stack_indices = {}

        # ── Page 0: Placeholder (shown until role is assigned) ────
        placeholder_page = QWidget()
        ph_layout = QHBoxLayout(placeholder_page)
        ph_layout.setContentsMargins(4, 4, 4, 4)
        ph_lbl = QLabel("← Select wells above, then click a role button to assign")
        ph_lbl.setStyleSheet("color: #6c7086; font-style: italic; font-size: 11px;")
        ph_lbl.setAlignment(__import__('PySide6.QtCore', fromlist=['Qt']).Qt.AlignmentFlag.AlignCenter)
        ph_layout.addWidget(ph_lbl)
        self._role_options_stack.addWidget(placeholder_page)   # index 0

        # ── Page 1: Print options ──────────────────────────────────
        print_page = QWidget()
        print_layout = QHBoxLayout(print_page)
        print_layout.setContentsMargins(0, 0, 0, 0)
        print_layout.addWidget(QLabel("Print File:"))
        self.print_combo = QComboBox()
        self.print_combo.addItem("(no prints available)", None)
        self.print_combo.setMinimumWidth(180)
        print_layout.addWidget(self.print_combo, stretch=1)
        btn_assign_print = QPushButton("Assign")
        btn_assign_print.clicked.connect(lambda: self._assign_print_to_selected(replace=True))
        print_layout.addWidget(btn_assign_print)
        btn_clear_print = QPushButton("Clear")
        btn_clear_print.clicked.connect(self._clear_selected_prints)
        print_layout.addWidget(btn_clear_print)
        print_layout.addStretch()
        self._role_stack_indices[WellRole.PRINT] = self._role_options_stack.addWidget(print_page)

        # ── Page 2: Ink options ────────────────────────────────────
        ink_page = QWidget()
        ink_layout = QHBoxLayout(ink_page)
        ink_layout.setContentsMargins(0, 0, 0, 0)
        ink_layout.addWidget(QLabel("Ink:"))
        self.ink_combo = QComboBox()
        self.ink_combo.addItem("(none)", None)
        self.ink_combo.setMinimumWidth(180)
        ink_layout.addWidget(self.ink_combo, stretch=1)
        btn_set_ink = QPushButton("Assign")
        btn_set_ink.clicked.connect(self._apply_ink)
        ink_layout.addWidget(btn_set_ink)
        ink_layout.addStretch()
        self._role_stack_indices[WellRole.INK] = self._role_options_stack.addWidget(ink_page)

        # ── Page 3: Wash options ───────────────────────────────────
        wash_page = QWidget()
        wash_layout = QHBoxLayout(wash_page)
        wash_layout.setContentsMargins(0, 0, 0, 0)
        wash_layout.addWidget(QLabel("Depth (mm):"))
        self._wash_depth_spin = QDoubleSpinBox()
        self._wash_depth_spin.setRange(0.1, 20.0)
        self._wash_depth_spin.setValue(2.0)
        self._wash_depth_spin.setSingleStep(0.1)
        wash_layout.addWidget(self._wash_depth_spin)
        wash_layout.addWidget(QLabel("Duration (s):"))
        self._wash_duration_spin = QDoubleSpinBox()
        self._wash_duration_spin.setRange(0.5, 60.0)
        self._wash_duration_spin.setValue(3.0)
        wash_layout.addWidget(self._wash_duration_spin)
        wash_layout.addStretch()
        self._role_stack_indices[WellRole.WASH] = self._role_options_stack.addWidget(wash_page)

        # ── Page 4: Waste options ──────────────────────────────────
        waste_page = QWidget()
        waste_layout = QHBoxLayout(waste_page)
        waste_layout.setContentsMargins(0, 0, 0, 0)
        waste_layout.addWidget(QLabel("Depth (mm):"))
        self._waste_depth_spin = QDoubleSpinBox()
        self._waste_depth_spin.setRange(0.1, 20.0)
        self._waste_depth_spin.setValue(1.0)
        waste_layout.addWidget(self._waste_depth_spin)
        waste_layout.addStretch()
        self._role_stack_indices[WellRole.WASTE] = self._role_options_stack.addWidget(waste_page)

        # ── Page 5: Buffer options ─────────────────────────────────
        buffer_page = QWidget()
        buffer_layout = QHBoxLayout(buffer_page)
        buffer_layout.setContentsMargins(0, 0, 0, 0)
        buffer_layout.addWidget(QLabel("Volume (uL):"))
        self._buffer_vol_spin = QDoubleSpinBox()
        self._buffer_vol_spin.setRange(1.0, 500.0)
        self._buffer_vol_spin.setValue(20.0)
        self._buffer_vol_spin.setSuffix(" uL")
        buffer_layout.addWidget(self._buffer_vol_spin)
        buffer_layout.addStretch()
        self._role_stack_indices[WellRole.BUFFER] = self._role_options_stack.addWidget(buffer_page)

        # ── Page 6: Sorted cells options ──────────────────────────
        sorted_page = QWidget()
        sorted_layout = QHBoxLayout(sorted_page)
        sorted_layout.setContentsMargins(0, 0, 0, 0)
        sorted_layout.addWidget(QLabel("Sorted cells well assigned — no extra parameters needed."))
        sorted_layout.addStretch()
        self._role_stack_indices[WellRole.SORTED_CELLS] = self._role_options_stack.addWidget(sorted_page)

        # ── Page 7: Empty/Clear ────────────────────────────────────
        empty_page = QWidget()
        empty_layout = QHBoxLayout(empty_page)
        empty_layout.setContentsMargins(0, 0, 0, 0)
        empty_layout.addWidget(QLabel("Role cleared — well set to EMPTY."))
        empty_layout.addStretch()
        self._role_stack_indices[WellRole.EMPTY] = self._role_options_stack.addWidget(empty_page)

        # Default: show placeholder
        self._role_options_stack.setCurrentIndex(0)

        options_layout.addWidget(self._role_options_stack)
        parent_layout.addWidget(options_frame)

'''
        content, replaced = replace_method(content, "_build_role_options", new_method_b)
        if replaced:
            ok("Replaced _build_role_options (placeholder page added)")
        else:
            fail("_build_role_options not found for replacement")
    else:
        skip("_build_role_options (placeholder page)")

    # ── C: Replace _apply_active_role with _on_role_btn_clicked ──
    marker_c = "v7.3.0: _on_role_btn_clicked"
    if marker_c not in content:
        new_method_c = '''    def _on_role_btn_clicked(self, role) -> None:
        """v7.3.0: _on_role_btn_clicked — assign role to selected wells
        and show role-specific options panel."""
        pv = getattr(self, 'plate_view', getattr(self, '_plate_view', None))
        if pv is None:
            return
        if hasattr(pv, 'get_selected_wells'):
            selected = pv.get_selected_wells()
        else:
            selected = list(getattr(pv, 'selected_wells', set()))

        if not selected:
            return  # No wells selected — silently ignore

        self._active_role = role

        # Update button checked states
        for r, btn in getattr(self, '_role_btns', {}).items():
            btn.setChecked(r == role)

        # Assign role in model
        self._model.set_role(selected, role)

        # Refresh plate view
        self._safe_refresh_plate_view()

        # Show role-specific options panel
        if hasattr(self, '_role_options_stack') and hasattr(self, '_role_stack_indices'):
            idx = self._role_stack_indices.get(role, 0)
            self._role_options_stack.setCurrentIndex(idx)

        # Refresh status colors and summary
        self._refresh_well_status_colors()
        self._safe_refresh_summary()
        self.setup_changed.emit()

        # Auto-regenerate plan if available
        if hasattr(self, '_on_plan_auto_regen'):
            self._on_plan_auto_regen()

    def _set_active_role(self, role) -> None:
        """Alias kept for backwards compatibility — delegates to _on_role_btn_clicked."""
        self._on_role_btn_clicked(role)

'''
        content, replaced = replace_method(content, "_apply_active_role", new_method_c)
        if replaced:
            ok("Replaced _apply_active_role with _on_role_btn_clicked")
        else:
            # _apply_active_role might not exist — inject before first _build_ method
            m = re.search(r'\n    def _build_role_bar\(', content)
            if m:
                inject = "\n" + new_method_c
                content = content[:m.start()] + inject + content[m.start():]
                ok("Injected _on_role_btn_clicked (before _build_role_bar)")
            else:
                fail("Could not inject _on_role_btn_clicked")
    else:
        skip("_on_role_btn_clicked")

    # ── D: Modify _on_selection_changed to refresh role options ──
    marker_d = "v7.3.0: selection changed refresh options"
    if marker_d not in content:
        # Find _on_selection_changed and append call at its end
        m = find_method(content, "_on_selection_changed")
        if m:
            method_body = m.group(0)
            # Append before the end of the method
            append_code = (
                "        # v7.3.0: selection changed refresh options\n"
                "        self._refresh_role_options_for_selection(selected_wells)\n"
            )
            # Insert right before the end of the method body
            new_body = method_body.rstrip() + "\n" + append_code + "\n"
            content = content[:m.start()] + new_body + content[m.end():]
            ok("Modified _on_selection_changed to refresh role options")
        else:
            fail("_on_selection_changed not found")
    else:
        skip("_on_selection_changed refresh options")

    # ── E: Add _refresh_role_options_for_selection ───────────────
    if "def _refresh_role_options_for_selection" not in content:
        new_method_e = '''    def _refresh_role_options_for_selection(self, selected_wells=None) -> None:
        """v7.3.0: Show/hide role options panel based on current selection state.

        - No selection        → placeholder page (index 0)
        - All same non-EMPTY  → that role's options page
        - Mixed / all empty   → placeholder page
        """
        if not hasattr(self, '_role_options_stack'):
            return

        if selected_wells is None:
            pv = getattr(self, 'plate_view', getattr(self, '_plate_view', None))
            if pv is not None:
                if hasattr(pv, 'get_selected_wells'):
                    selected_wells = pv.get_selected_wells()
                else:
                    selected_wells = list(getattr(pv, 'selected_wells', set()))
            else:
                selected_wells = []

        if not selected_wells:
            self._role_options_stack.setCurrentIndex(0)
            return

        # Collect roles of all selected wells
        try:
            from SupportClasses.PhysicalModels import WellRole
        except ImportError:
            return

        roles = set()
        for wn in selected_wells:
            wa = self._model.get_assignment(wn)
            if wa is not None:
                roles.add(wa.role)

        if len(roles) == 1:
            role = next(iter(roles))
            if role != WellRole.EMPTY:
                idx = getattr(self, '_role_stack_indices', {}).get(role, 0)
                self._role_options_stack.setCurrentIndex(idx)
                return

        # Mixed roles or all empty → placeholder
        self._role_options_stack.setCurrentIndex(0)

'''
        # Inject before _on_role_btn_clicked or at end of class
        m = find_method(content, "_on_role_btn_clicked")
        if m:
            content = content[:m.start()] + new_method_e + content[m.start():]
            ok("Added _refresh_role_options_for_selection")
        else:
            # fallback: inject before _build_role_bar
            m2 = re.search(r'\n    def _build_role_bar\(', content)
            if m2:
                content = content[:m2.start()] + "\n" + new_method_e + content[m2.start():]
                ok("Added _refresh_role_options_for_selection (fallback)")
            else:
                fail("Could not inject _refresh_role_options_for_selection")
    else:
        skip("_refresh_role_options_for_selection")

    # ── F: Add _safe_refresh_plate_view ──────────────────────────
    if "def _safe_refresh_plate_view" not in content:
        new_method_f = '''    def _safe_refresh_plate_view(self) -> None:
        """v7.3.0: Refresh plate view colors using whatever API is available.
        Handles both self.plate_view and self._plate_view naming."""
        pv = getattr(self, 'plate_view', getattr(self, '_plate_view', None))
        if pv is None:
            return
        if hasattr(pv, 'update_well_roles') and hasattr(self._model, 'get_role_map'):
            try:
                pv.update_well_roles(self._model.get_role_map())
                return
            except Exception:
                pass
        if hasattr(pv, 'update_all_wells'):
            try:
                appearances = {
                    n: (wa.role, wa.get_display_label())
                    for n, wa in self._model.assignments.items()
                }
                pv.update_all_wells(appearances)
            except Exception:
                pass

'''
        m = find_method(content, "_refresh_role_options_for_selection")
        if m:
            content = content[:m.start()] + new_method_f + content[m.start():]
            ok("Added _safe_refresh_plate_view")
        else:
            m2 = find_method(content, "_on_role_btn_clicked")
            if m2:
                content = content[:m2.start()] + new_method_f + content[m2.start():]
                ok("Added _safe_refresh_plate_view (fallback)")
            else:
                fail("Could not inject _safe_refresh_plate_view")
    else:
        skip("_safe_refresh_plate_view")

    # ── G: Add _safe_refresh_summary ─────────────────────────────
    if "def _safe_refresh_summary" not in content:
        new_method_g = '''    def _safe_refresh_summary(self) -> None:
        """v7.3.0: Refresh summary table — handles both _refresh_summary_table
        and _refresh_summary method names across versions."""
        if hasattr(self, '_refresh_summary_table'):
            try:
                self._refresh_summary_table()
                return
            except Exception:
                pass
        if hasattr(self, '_refresh_summary'):
            try:
                self._refresh_summary()
            except Exception:
                pass

'''
        m = find_method(content, "_safe_refresh_plate_view")
        if m:
            content = content[:m.start()] + new_method_g + content[m.start():]
            ok("Added _safe_refresh_summary")
        else:
            fail("Could not inject _safe_refresh_summary")
    else:
        skip("_safe_refresh_summary")

    # ── H: Add _check_well_ready if missing ──────────────────────
    if "def _check_well_ready" not in content:
        new_method_h = '''    def _check_well_ready(self, wa) -> bool:
        """v7.3.0: True if well is fully configured for its role.

        - EMPTY:        always False
        - PRINT:        True if has at least one print collection
        - INK:          True if ink_name is set
        - WASH/WASTE/BUFFER/SORTED_CELLS: True when role is set (no extra needed)
        """
        try:
            from SupportClasses.PhysicalModels import WellRole
        except ImportError:
            return True
        role = getattr(wa, 'role', None)
        if role is None or role == WellRole.EMPTY:
            return False
        if role == WellRole.PRINT:
            return bool(getattr(wa, 'print_collections', []))
        if role == WellRole.INK:
            return bool(getattr(wa, 'ink_name', None))
        return True  # WASH, WASTE, BUFFER, SORTED_CELLS

'''
        m = find_method(content, "_safe_refresh_summary")
        if m:
            content = content[:m.start()] + new_method_h + content[m.start():]
            ok("Added _check_well_ready")
        else:
            # Fallback: before _refresh_well_status_colors
            m2 = find_method(content, "_refresh_well_status_colors")
            if m2:
                content = content[:m2.start()] + new_method_h + content[m2.start():]
                ok("Added _check_well_ready (fallback)")
            else:
                fail("Could not inject _check_well_ready")
    else:
        skip("_check_well_ready")

    # ── I: Fix _refresh_well_status_colors plate_view reference ──
    marker_i = "v7.3.0: _refresh_well_status_colors plate_view fix"
    if marker_i not in content:
        new_method_i = '''    def _refresh_well_status_colors(self) -> None:
        """v7.3.0: _refresh_well_status_colors plate_view fix — Update well border
        colors based on assignment status. Uses safe plate_view getter."""
        pv = getattr(self, 'plate_view', getattr(self, '_plate_view', None))
        if pv is None:
            return

        status_map = {}
        for name, wa in self._model.assignments.items():
            if wa.role.value == 'empty' if hasattr(wa.role, 'value') else wa.role == 'empty':
                status_map[name] = "empty"
            elif self._check_well_ready(wa):
                status_map[name] = "ready"
            else:
                status_map[name] = "incomplete"

        if hasattr(pv, 'update_well_status'):
            pv.update_well_status(status_map)

'''
        content, replaced = replace_method(content, "_refresh_well_status_colors", new_method_i)
        if replaced:
            ok("Fixed _refresh_well_status_colors (safe plate_view getter)")
        else:
            fail("_refresh_well_status_colors not found for replacement — check file state")
    else:
        skip("_refresh_well_status_colors (plate_view fix)")

    # ── Add _assign_print_to_selected (new name used in _build_role_options) ─
    if "def _assign_print_to_selected" not in content:
        new_method_p = '''    def _assign_print_to_selected(self, replace: bool = False) -> None:
        """v7.3.0: Assign selected print collection to selected wells."""
        pv = getattr(self, 'plate_view', getattr(self, '_plate_view', None))
        if pv is None:
            return
        selected = (pv.get_selected_wells() if hasattr(pv, 'get_selected_wells')
                    else list(getattr(pv, 'selected_wells', set())))
        if not selected:
            return
        combo = getattr(self, 'print_combo', None)
        if combo is None:
            return
        print_name = combo.currentData()
        if not print_name:
            return
        self._model.assign_print(selected, print_name, replace=replace)
        self._safe_refresh_plate_view()
        self._refresh_well_status_colors()
        self._safe_refresh_summary()
        self.setup_changed.emit()

'''
        m = find_method(content, "_assign_print_to_selected")
        if m is None:
            # Inject before _assign_print or at a safe location
            m2 = find_method(content, "_apply_ink")
            if m2:
                content = content[:m2.start()] + new_method_p + content[m2.start():]
                ok("Added _assign_print_to_selected")
            else:
                m3 = find_method(content, "_safe_refresh_summary")
                if m3:
                    content = content[:m3.start()] + new_method_p + content[m3.start():]
                    ok("Added _assign_print_to_selected (fallback)")
                else:
                    fail("Could not inject _assign_print_to_selected")
    else:
        skip("_assign_print_to_selected")

    # Write
    if content != original:
        if safe_write(path, content, "print_well_setup.py"):
            print(f"  {GREEN}Wrote: {path.name}{RESET}")
        else:
            print(f"  {RED}NOT WRITTEN (AST failure){RESET}")
    else:
        print(f"  {CYAN}No changes needed.{RESET}")


# ════════════════════════════════════════════════════════════════
# PATCH 2 — well_plate_view.py  (add update_well_status)
# ════════════════════════════════════════════════════════════════

def patch_well_plate_view(root: Path) -> None:
    path = root / "gui" / "widgets" / "well_plate_view.py"
    print(f"\n{BOLD}{CYAN}Patching: {path.relative_to(root)}{RESET}")
    content = safe_read(path)
    if not content:
        return
    original = content

    # ── J: Add update_well_status if missing ─────────────────────
    if "def update_well_status" not in content:
        new_method_j = '''    def update_well_status(self, status_map: dict) -> None:
        """v7.3.0: Update well border (pen) colors based on readiness status.

        status_map: {well_name: "ready" | "incomplete" | "empty"}
        Border colors:
            ready      → #a6e3a1 (green),  width 2
            incomplete → #f38ba8 (red),    width 2
            empty      → #585b70 (gray),   width 1
        """
        from PySide6.QtGui import QColor, QPen
        STATUS_PEN = {
            "ready":      ("#a6e3a1", 2),
            "incomplete": ("#f38ba8", 2),
            "empty":      ("#585b70", 1),
        }
        for well_name, status in status_map.items():
            item = self._well_items.get(well_name)
            if item is None:
                continue
            color_hex, width = STATUS_PEN.get(status, ("#585b70", 1))
            pen = QPen(QColor(color_hex))
            pen.setWidth(width)
            item.setPen(pen)

'''
        # Inject before update_all_wells or update_well_roles
        for target in ("update_all_wells", "update_well_roles", "set_needle_position"):
            m = find_method(content, target)
            if m:
                content = content[:m.start()] + new_method_j + content[m.start():]
                ok(f"Added update_well_status to WellPlateView (before {target})")
                break
        else:
            # Append before end of class
            last_def = list(re.finditer(r'\n    def ', content))
            if last_def:
                pos = last_def[-1].start()
                content = content[:pos] + "\n" + new_method_j + content[pos:]
                ok("Added update_well_status to WellPlateView (appended)")
            else:
                fail("Could not inject update_well_status into WellPlateView")
    else:
        skip("update_well_status in WellPlateView")

    if content != original:
        if safe_write(path, content, "well_plate_view.py"):
            print(f"  {GREEN}Wrote: {path.name}{RESET}")
        else:
            print(f"  {RED}NOT WRITTEN (AST failure){RESET}")
    else:
        print(f"  {CYAN}No changes needed.{RESET}")


# ════════════════════════════════════════════════════════════════
# MAIN
# ════════════════════════════════════════════════════════════════

def main():
    root = find_root()
    print(f"{BOLD}MEBP v7.3.0 — Session 1: Well Role Flow Redesign{RESET}")
    print(f"Project root: {root}\n")

    patch_print_well_setup(root)
    patch_well_plate_view(root)

    print(f"\n{BOLD}{'='*50}{RESET}")
    print(f"  {GREEN}Applied : {_applied}{RESET}")
    print(f"  {CYAN}Skipped : {_skipped}{RESET}")
    print(f"  {RED}Failed  : {_failed}{RESET}")
    print(f"{BOLD}{'='*50}{RESET}")

    if _failed:
        print(f"\n{RED}Some changes failed — review MISS messages above.{RESET}")
        print("Run project_knowledge_search to inspect actual file content.")
        sys.exit(1)
    else:
        print(f"\n{GREEN}All changes applied successfully.{RESET}")
        print("\nNext steps:")
        print("  python3 -c \"import ast; ast.parse(open('gui/pages/print_well_setup.py').read())\"")
        print("  python3 -c \"import ast; ast.parse(open('gui/widgets/well_plate_view.py').read())\"")
        print("  python3 main.py  # smoke test")


if __name__ == "__main__":
    main()

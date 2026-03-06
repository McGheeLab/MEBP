#!/usr/bin/env python3
"""
MEBP v7.2.6 — Batch 3 Deployment: Large File Rewrites

Files modified:
    1. gui/pages/print_objects.py    — Splitters, staging list, OOB flash
    2. gui/pages/print_well_setup.py — Remove projections, role workflow, colors

Each change finds its target via method signatures or structural regex,
NOT exact string matches. AST-verified before every write.

Usage:
    cd /path/to/McGheeLab/MEBP
    python patches/v726/deploy_v726_batch3.py
"""

import ast
import re
import sys
import shutil
from pathlib import Path
from datetime import datetime

G = "\033[92m"; R = "\033[91m"; Y = "\033[93m"; B = "\033[1m"; C = "\033[96m"; X = "\033[0m"
_ok = 0; _skip = 0; _fail = 0
_ts = datetime.now().strftime("%Y%m%d_%H%M%S")

def find_root() -> Path:
    for c in [Path("."), Path(".."), Path(__file__).parent.parent.parent]:
        if (c / "SupportClasses").is_dir() and (c / "gui").is_dir():
            return c.resolve()
    print(f"{R}Cannot find project root{X}"); sys.exit(1)

def safe_read(p):
    if not p.exists(): print(f"  {R}NOT FOUND: {p}{X}"); return ""
    return p.read_text(encoding="utf-8")

def safe_write(p, content, label):
    global _ok, _fail
    try: ast.parse(content)
    except SyntaxError as e:
        print(f"  {R}AST FAIL: {label} — {e}{X}"); _fail += 1; return False
    shutil.copy2(p, p.with_suffix(f".bak_v726_{_ts}"))
    p.write_text(content, encoding="utf-8")
    print(f"  {G}WROTE{X}: {p.name} ({label})"); _ok += 1; return True

def rc(ok, label):
    global _skip
    if ok: print(f"    {G}✓{X} {label}")
    else: print(f"    {Y}○{X} {label} (skip)"); _skip += 1

def find_method(content, name):
    """Find method boundaries: returns (start, end) of the full method."""
    pattern = re.compile(rf'^(    def {name}\(self.*?\n)(.*?)(?=\n    def |\nclass |\Z)', re.DOTALL | re.MULTILINE)
    return pattern.search(content)


# ═══════════════════════════════════════════════════════════════════
#  FILE 1: gui/pages/print_objects.py
# ═══════════════════════════════════════════════════════════════════

def fix_print_objects(root: Path):
    path = root / "gui" / "pages" / "print_objects.py"
    print(f"\n{C}[1/2] {path.name}{X}")
    content = safe_read(path)
    if not content: return

    changed = False

    # ──────────────────────────────────────────────────────────────
    # 1A. Replace _build_ui layout: horizontal splitter → nested splitters
    # ──────────────────────────────────────────────────────────────

    if "v7.2.6: All-splitter" not in content:
        # Use regex to match the entire layout block in _build_ui
        layout_re = re.compile(
            r'(        # ── Main splitter ─+\n)'
            r'(.*?)'
            r'(        outer\.addWidget\(splitter\))',
            re.DOTALL
        )
        match = layout_re.search(content)
        if match:
            new_layout = '''        # ── v7.2.6: All-splitter layout — every panel resizable ──
        main_splitter = QSplitter(Qt.Orientation.Horizontal)
        main_splitter.setChildrenCollapsible(False)

        # Left: vertical splitter (Designer / Auto-Layout / CSV)
        left_splitter = QSplitter(Qt.Orientation.Vertical)
        left_splitter.setChildrenCollapsible(False)

        _dw = QWidget(); _dl = QVBoxLayout(_dw); _dl.setContentsMargins(4,4,4,4); _dl.setSpacing(4)
        self._build_designer_section(_dl); _dw.setMinimumHeight(120)
        left_splitter.addWidget(_dw)

        _aw = QWidget(); _al = QVBoxLayout(_aw); _al.setContentsMargins(4,4,4,4); _al.setSpacing(4)
        self._build_auto_layout_section(_al); _aw.setMinimumHeight(80)
        left_splitter.addWidget(_aw)

        _cw = QWidget(); _cl = QVBoxLayout(_cw); _cl.setContentsMargins(4,4,4,4); _cl.setSpacing(4)
        self._build_csv_import_section(_cl); _cw.setMinimumHeight(60)
        left_splitter.addWidget(_cw)

        left_splitter.setStretchFactor(0, 3); left_splitter.setStretchFactor(1, 2); left_splitter.setStretchFactor(2, 1)
        main_splitter.addWidget(left_splitter)

        # Right: vertical splitter (Preview / Objects List / Summary+Staging)
        right_splitter = QSplitter(Qt.Orientation.Vertical)
        right_splitter.setChildrenCollapsible(False)

        _pw = QWidget(); _pl = QVBoxLayout(_pw); _pl.setContentsMargins(4,4,4,4); _pl.setSpacing(4)
        self._build_preview_section(_pl); _pw.setMinimumHeight(150)
        right_splitter.addWidget(_pw)

        _ow = QWidget(); _ol = QVBoxLayout(_ow); _ol.setContentsMargins(4,4,4,4); _ol.setSpacing(4)
        self._build_objects_list_section(_ol); _ow.setMinimumHeight(100)
        right_splitter.addWidget(_ow)

        _sw = QWidget(); _sl = QVBoxLayout(_sw); _sl.setContentsMargins(4,4,4,4); _sl.setSpacing(4)
        self._build_summary_section(_sl); _sw.setMinimumHeight(60)
        right_splitter.addWidget(_sw)

        right_splitter.setStretchFactor(0, 4); right_splitter.setStretchFactor(1, 3); right_splitter.setStretchFactor(2, 1)
        main_splitter.addWidget(right_splitter)
        main_splitter.setStretchFactor(0, 2); main_splitter.setStretchFactor(1, 3)

        outer.addWidget(main_splitter)'''

            content = content[:match.start()] + new_layout + content[match.end():]
            changed = True
            rc(True, "Replaced layout with all-splitter design")
        else:
            rc(False, "Main splitter pattern not found (may already be restructured)")
    else:
        rc(False, "v7.2.6 splitter already applied")

    # ──────────────────────────────────────────────────────────────
    # 1B. Add _staging_print_names + _flash_timers to __init__
    # ──────────────────────────────────────────────────────────────

    if "_staging_print_names" not in content:
        match = re.search(r'(self\._editing_index.*?=.*?None[^\n]*\n)', content)
        if match:
            inject = ("\n        # v7.2.6: Staging list + OOB flash timers\n"
                      "        self._staging_print_names: list[str] = []\n"
                      "        self._flash_timers: dict[int, QTimer] = {}\n")
            content = content[:match.end()] + inject + content[match.end():]
            changed = True
            rc(True, "Added _staging_print_names + _flash_timers")
        else:
            rc(False, "_editing_index not found")
    else:
        rc(False, "Staging attrs already present")

    # ──────────────────────────────────────────────────────────────
    # 1C. Replace _build_summary_section with staging list version
    # ──────────────────────────────────────────────────────────────

    if "_prints_staging_list" not in content:
        m = find_method(content, "_build_summary_section")
        if m:
            new_summary = '''    def _build_summary_section(self, parent_layout):
        """v7.2.6: Summary + staging list for selective print sending."""
        self._summary_label = QLabel("No objects")
        self._summary_label.setStyleSheet(
            f"color: {COLORS['subtext0']}; background: {COLORS['surface0']}; "
            f"padding: 4px 8px; border-radius: 4px; font-size: 11px;")
        parent_layout.addWidget(self._summary_label)

        # Staging list
        stg_lbl = QLabel("Prints to Send:")
        stg_lbl.setStyleSheet(f"color: {COLORS['blue']}; font-weight: bold; font-size: 10px;")
        parent_layout.addWidget(stg_lbl)

        self._prints_staging_list = QListWidget()
        self._prints_staging_list.setMaximumHeight(80)
        self._prints_staging_list.setStyleSheet(f"""
            QListWidget {{ background: {COLORS['surface0']}; color: {COLORS['text']};
                border: 1px solid {COLORS['surface1']}; border-radius: 4px; font-size: 10px; }}
            QListWidget::item:selected {{ background: {COLORS['surface2']}; }}""")
        parent_layout.addWidget(self._prints_staging_list)

        btns = QHBoxLayout()
        b1 = QPushButton("+ Add Current"); b1.setFixedHeight(24); b1.clicked.connect(self._staging_add_current); btns.addWidget(b1)
        b2 = QPushButton("- Remove"); b2.setFixedHeight(24); b2.clicked.connect(self._staging_remove_selected); btns.addWidget(b2)
        self._btn_send_to_prints = QPushButton("\\U0001f4cb Send"); self._btn_send_to_prints.setFixedHeight(24)
        self._btn_send_to_prints.setStyleSheet(f"QPushButton {{ background: {COLORS.get('mauve','#cba6f7')}; color: {COLORS.get('base','#1e1e2e')}; font-weight: bold; border-radius: 4px; font-size: 10px; }}")
        self._btn_send_to_prints.clicked.connect(self._send_staged_prints); btns.addWidget(self._btn_send_to_prints)
        btns.addStretch()
        parent_layout.addLayout(btns)

'''
            content = content[:m.start()] + new_summary + content[m.end():]
            changed = True
            rc(True, "Replaced _build_summary_section with staging list")
        else:
            rc(False, "_build_summary_section not found")
    else:
        rc(False, "Staging list already in summary section")

    # ──────────────────────────────────────────────────────────────
    # 1D. Add staging + flash methods
    # ──────────────────────────────────────────────────────────────

    if "def _staging_add_current" not in content:
        methods = '''
    def _staging_add_current(self):
        """v7.2.6: Add currently loaded print to staging list."""
        name = getattr(self, '_active_file_name', None)
        if not name: return
        if name not in self._staging_print_names:
            self._staging_print_names.append(name)
            self._prints_staging_list.addItem(name)

    def _staging_remove_selected(self):
        """v7.2.6: Remove selected from staging."""
        for item in self._prints_staging_list.selectedItems():
            name = item.text()
            if name in self._staging_print_names: self._staging_print_names.remove(name)
            self._prints_staging_list.takeItem(self._prints_staging_list.row(item))

    def _send_staged_prints(self):
        """v7.2.6: Send only staged prints to well setup."""
        if not self._staging_print_names:
            return
        self.prints_changed.emit(list(self._staging_print_names))
        n = len(self._staging_print_names)
        if hasattr(self, '_btn_send_to_prints'):
            self._btn_send_to_prints.setText(f"\\u2713 {n} sent!")
            QTimer.singleShot(1500, lambda: self._btn_send_to_prints.setText("\\U0001f4cb Send"))

    def _stop_all_flashing(self):
        """v7.2.6: Stop OOB flash timers."""
        for t in self._flash_timers.values():
            if hasattr(t, 'stop'): t.stop()
        self._flash_timers.clear()

    def _start_flash_item(self, index: int):
        """v7.2.6: Flash objects list item red for out-of-bounds."""
        if index in self._flash_timers: return
        if not hasattr(self, '_objects_list'): return
        item = self._objects_list.item(index)
        if not item: return
        _state = [True]
        def _toggle():
            it = self._objects_list.item(index) if index < self._objects_list.count() else None
            if it is None:
                if index in self._flash_timers: self._flash_timers[index].stop(); del self._flash_timers[index]
                return
            it.setForeground(QColor(COLORS['red']) if _state[0] else QColor(COLORS['text']))
            _state[0] = not _state[0]
        timer = QTimer(self); timer.timeout.connect(_toggle); timer.start(500)
        self._flash_timers[index] = timer
        item.setForeground(QColor(COLORS['red']))

'''
        # Insert before _emit_prints_changed or _send_to_available_prints
        for anchor_name in ['_send_to_available_prints', '_emit_prints_changed', '_update_summary']:
            anchor = f'    def {anchor_name}(self'
            if anchor in content:
                content = content.replace(anchor, methods + anchor, 1)
                changed = True
                rc(True, "Added staging + flash methods")
                break
        else:
            rc(False, "No insertion anchor found for staging methods")
    else:
        rc(False, "Staging methods already exist")

    # ──────────────────────────────────────────────────────────────
    # 1E. Auto-add to staging in _on_new_print
    # ──────────────────────────────────────────────────────────────

    if "_staging_add_current" in content and "_on_new_print" in content:
        m = find_method(content, "_on_new_print")
        if m and "_staging_add_current" not in m.group(0):
            # Find _emit_prints_changed() call in the method and add after
            emit_pos = content.find("self._emit_prints_changed()", m.start())
            if emit_pos > 0 and emit_pos < (m.end() if m.end() else len(content)):
                line_end = content.index('\n', emit_pos)
                content = content[:line_end] + \
                    "\n        self._staging_add_current()  # v7.2.6" + \
                    content[line_end:]
                changed = True
                rc(True, "Auto-stage new prints")
            else:
                rc(False, "_emit_prints_changed not in _on_new_print")
        else:
            rc(False, "_staging_add_current already in _on_new_print")
    else:
        rc(False, "Prerequisites for auto-staging not met")

    # ──────────────────────────────────────────────────────────────
    # 1F. Enhance _refresh_oob_state to use flash timers
    # ──────────────────────────────────────────────────────────────

    if "_stop_all_flashing" in content and "_refresh_oob_state" in content:
        m = find_method(content, "_refresh_oob_state")
        if m and "_stop_all_flashing" not in m.group(0):
            # Add _stop_all_flashing() at start of method body
            doc_end = content.find('"""', m.start() + 20)
            if doc_end > 0:
                doc_end = content.index('\n', doc_end) + 1
                content = content[:doc_end] + \
                    "        self._stop_all_flashing()  # v7.2.6\n" + \
                    content[doc_end:]
                changed = True
                rc(True, "Added _stop_all_flashing to _refresh_oob_state")
            else:
                rc(False, "Could not find docstring end in _refresh_oob_state")
        else:
            rc(False, "_stop_all_flashing already in _refresh_oob_state")
    else:
        rc(False, "Prerequisites for flash in _refresh_oob_state not met")

    if changed:
        safe_write(path, content, "splitters + staging + flash")
    else:
        print(f"  {Y}No changes needed{X}")


# ═══════════════════════════════════════════════════════════════════
#  FILE 2: gui/pages/print_well_setup.py
# ═══════════════════════════════════════════════════════════════════

def fix_print_well_setup(root: Path):
    path = root / "gui" / "pages" / "print_well_setup.py"
    print(f"\n{C}[2/2] {path.name}{X}")
    content = safe_read(path)
    if not content: return

    changed = False

    # ──────────────────────────────────────────────────────────────
    # 2A. Remove ZY/XZ projections from _build_ui
    # ──────────────────────────────────────────────────────────────

    # Remove ZY view
    zy_re = re.compile(
        r'\n\s*# ZY side projection.*?\n'
        r'\s*self\.zy_view = MiniProjectionView\("ZY"\)\n'
        r'\s*top_splitter\.addWidget\(self\.zy_view\)\n'
        r'\s*top_splitter\.setStretchFactor\(0, 5\)\n'
        r'\s*top_splitter\.setStretchFactor\(1, 1\)\n',
        re.DOTALL
    )
    if zy_re.search(content):
        content = zy_re.sub('\n', content)
        changed = True
        rc(True, "Removed ZY projection")
    else:
        rc('MiniProjectionView("ZY")' not in content, "ZY projection")

    # Remove XZ view
    xz_re = re.compile(
        r'\n\s*# ── XZ bottom projection ─+\n'
        r'\s*self\.xz_view = MiniProjectionView\("XZ"\)\n'
        r'\s*main_layout\.addWidget\(self\.xz_view\)\n',
        re.DOTALL
    )
    if xz_re.search(content):
        content = xz_re.sub('\n', content)
        changed = True
        rc(True, "Removed XZ projection")
    else:
        rc('MiniProjectionView("XZ")' not in content, "XZ projection")

    # Simplify top_splitter → direct plate_container if ZY removed
    if 'top_splitter = QSplitter' in content and 'zy_view' not in content:
        content = re.sub(
            r'        top_splitter = QSplitter\(Qt\.Orientation\.Horizontal\)\n\n'
            r'        # Plate view\n',
            '        # v7.2.6: Plate view (XY only)\n',
            content, count=1
        )
        content = content.replace('        top_splitter.addWidget(plate_container)\n', '', 1)
        content = content.replace(
            '        main_layout.addWidget(top_splitter, stretch=3)',
            '        main_layout.addWidget(plate_container, stretch=3)',
            1
        )
        changed = True
        rc(True, "Simplified top_splitter → direct plate_container")

    # ──────────────────────────────────────────────────────────────
    # 2B. Hide role options stack until assignment
    # ──────────────────────────────────────────────────────────────

    if '_role_options_stack' in content:
        # Check if setVisible(False) is near the creation
        creation_match = re.search(r'self\._role_options_stack\s*=\s*QStackedWidget\(\)', content)
        if creation_match:
            # Check next ~200 chars for setVisible
            after = content[creation_match.end():creation_match.end()+200]
            if 'setVisible(False)' not in after:
                content = content[:creation_match.end()] + \
                    '\n        self._role_options_stack.setVisible(False)  # v7.2.6: hidden until role assigned' + \
                    content[creation_match.end():]
                changed = True
                rc(True, "Hidden role_options_stack by default")
            else:
                rc(False, "setVisible(False) already near creation")
        else:
            rc(False, "_role_options_stack creation not found")

        # Make visible on role assignment
        for method_name in ['_apply_active_role', '_on_role_button_clicked']:
            m = find_method(content, method_name)
            if m and 'setCurrentIndex' in m.group(0) and 'setVisible(True)' not in m.group(0):
                idx_match = re.search(r'(self\._role_options_stack\.setCurrentIndex\([^)]+\)\n)', content[m.start():m.end()])
                if idx_match:
                    abs_pos = m.start() + idx_match.end()
                    indent = '            ' if '        if' in content[m.start():abs_pos] else '        '
                    content = content[:abs_pos] + \
                        f'{indent}self._role_options_stack.setVisible(True)  # v7.2.6\n' + \
                        content[abs_pos:]
                    changed = True
                    rc(True, f"Show role_options_stack in {method_name}")
                    break
        else:
            rc(False, "Role visibility toggle")
    else:
        rc(False, "No _role_options_stack found")

    # ──────────────────────────────────────────────────────────────
    # 2C. Add _check_well_complete helper
    # ──────────────────────────────────────────────────────────────

    if '_check_well_complete' not in content:
        helper = '''
    def _check_well_complete(self, wa) -> bool:
        """v7.2.6: Check if a well assignment is fully configured."""
        try:
            from SupportClasses.WellSetupModel import WellRole
        except ImportError:
            return True
        if wa.role == WellRole.EMPTY: return True
        if wa.role == WellRole.PRINT: return bool(getattr(wa, 'print_collections', None))
        if wa.role == WellRole.INK: return bool(getattr(wa, 'ink_name', None))
        return True  # Wash/Waste/Buffer are complete by assignment

'''
        anchor = '    def _refresh_well_colors(self'
        if anchor in content:
            content = content.replace(anchor, helper + anchor, 1)
            changed = True
            rc(True, "Added _check_well_complete helper")
        else:
            rc(False, "_refresh_well_colors not found for insertion")
    else:
        rc(False, "_check_well_complete already exists")

    # ──────────────────────────────────────────────────────────────
    # 2D. Replace _refresh_well_colors with green/red/purple logic
    # ──────────────────────────────────────────────────────────────

    if 'v7.2.6: Green/red/purple' not in content:
        m = find_method(content, "_refresh_well_colors")
        if m:
            new_method = '''    def _refresh_well_colors(self):
        """v7.2.6: Green/red/purple border color coding."""
        if not hasattr(self, 'plate_view'): return
        try:
            from SupportClasses.WellSetupModel import WellRole
        except ImportError: return

        selected = set(self.plate_view.get_selected_wells()) if hasattr(self.plate_view, 'get_selected_wells') else set()

        for name in self._model.well_names:
            wa = self._model.get_assignment(name)
            if wa is None: continue

            if name in selected:
                border = COLORS.get('mauve', '#cba6f7')      # Purple = selected
            elif wa.role == WellRole.EMPTY:
                border = COLORS.get('surface1', '#45475a')    # Gray = empty
            elif hasattr(self, '_check_well_complete') and self._check_well_complete(wa):
                border = COLORS.get('green', '#a6e3a1')       # Green = ready
            else:
                border = COLORS.get('red', '#f38ba8')         # Red = incomplete

            if hasattr(self.plate_view, 'set_well_border_color'):
                self.plate_view.set_well_border_color(name, border)
            elif hasattr(self.plate_view, 'set_well_color'):
                self.plate_view.set_well_color(name, border)

        if hasattr(self, '_refresh_summary_table'): self._refresh_summary_table()
        elif hasattr(self, '_refresh_summary'): self._refresh_summary()

'''
            content = content[:m.start()] + new_method + content[m.end():]
            changed = True
            rc(True, "Replaced _refresh_well_colors with green/red/purple")
        else:
            rc(False, "_refresh_well_colors method not found")
    else:
        rc(False, "Green/red/purple already applied")

    # ──────────────────────────────────────────────────────────────
    # 2E. Add _on_summary_row_clicked
    # ──────────────────────────────────────────────────────────────

    if '_on_summary_row_clicked' not in content and '_summary_table' in content:
        handler = '''
    def _on_summary_row_clicked(self, row, col):
        """v7.2.6: Click summary row → select well."""
        if not hasattr(self, '_summary_table'): return
        item = self._summary_table.item(row, 0)
        if item and hasattr(self, 'plate_view') and hasattr(self.plate_view, 'set_selection'):
            self.plate_view.set_selection([item.text()])

'''
        anchor = '    def _check_well_complete'
        if anchor not in content:
            anchor = '    def _refresh_well_colors'
        if anchor in content:
            content = content.replace(anchor, handler + anchor, 1)
            changed = True
            rc(True, "Added _on_summary_row_clicked")

            # Wire signal
            if 'cellClicked.connect(self._on_summary_row_clicked)' not in content:
                wire_match = re.search(r'(self\._summary_table\s*=\s*QTableWidget\(\)\n)', content)
                if wire_match:
                    content = content[:wire_match.end()] + \
                        '        self._summary_table.cellClicked.connect(self._on_summary_row_clicked)  # v7.2.6\n' + \
                        content[wire_match.end():]
                    rc(True, "Wired summary table cellClicked")
        else:
            rc(False, "No anchor for _on_summary_row_clicked")
    else:
        rc('_on_summary_row_clicked' in content, "_on_summary_row_clicked")

    # ──────────────────────────────────────────────────────────────
    # 2F. Add _refresh_ink/rosette_options_from_hw_config
    # ──────────────────────────────────────────────────────────────

    if '_refresh_ink_options_from_hw_config' not in content:
        helpers = '''
    def _refresh_ink_options_from_hw_config(self, config):
        """v7.2.6: Refresh ink combos from HardwareConfig."""
        if config is None: return
        ink_to_pump = {}
        if hasattr(config, 'pumps'):
            for pid, pcfg in config.pumps.items():
                if pcfg.is_configured and pcfg.ink and pcfg.ink.name:
                    ink_to_pump[pcfg.ink.name] = pid
        ink_names = list(config.ink_library.keys())
        for attr in ['ink_combo', '_ink_combo', '_role_ink_combo', 'print_combo']:
            combo = getattr(self, attr, None)
            if combo is None or not hasattr(combo, 'clear'): continue
            if attr == 'print_combo': continue  # Don't touch print combo
            cur = combo.currentData()
            combo.blockSignals(True); combo.clear(); combo.addItem("(none)", None)
            for name in ink_names:
                tag = ink_to_pump.get(name, "")
                combo.addItem(f"{name} ({tag})" if tag else name, name)
            if cur:
                idx = combo.findData(cur)
                if idx >= 0: combo.setCurrentIndex(idx)
            combo.blockSignals(False)

    def _refresh_rosette_options_from_hw_config(self, config):
        """v7.2.6: Refresh rosette combos from HardwareConfig."""
        if config is None: return
        for attr in ['rosette_combo', '_rosette_combo', '_role_rosette_combo']:
            combo = getattr(self, attr, None)
            if combo is None or not hasattr(combo, 'clear'): continue
            cur = combo.currentData()
            combo.blockSignals(True); combo.clear(); combo.addItem("None", None)
            for name, ros in config.rosette_library.items():
                n_sub = getattr(ros, 'num_subwells', '?')
                combo.addItem(f"{name} ({n_sub} sub-wells)", name)
            if cur:
                idx = combo.findData(cur)
                if idx >= 0: combo.setCurrentIndex(idx)
            combo.blockSignals(False)

'''
        anchor = '    def set_workspace(self'
        if anchor not in content:
            anchor = '    def set_available_prints(self'
        if anchor in content:
            content = content.replace(anchor, helpers + anchor, 1)
            changed = True
            rc(True, "Added ink/rosette refresh helpers")
        else:
            rc(False, "No anchor for refresh helpers")
    else:
        rc(False, "Refresh helpers already exist")

    # ──────────────────────────────────────────────────────────────
    # 2G. Enhance set_hardware_config to use new helpers
    # ──────────────────────────────────────────────────────────────

    if '_refresh_ink_options_from_hw_config' in content:
        m = find_method(content, "set_hardware_config")
        if m and '_refresh_ink_options_from_hw_config' not in m.group(0):
            # Replace method body
            new_shc = '''    def set_hardware_config(self, config):
        """v7.2.6: Receive HardwareConfig — refresh inks, rosettes, plate."""
        if config is None: return
        self._hw_config = config
        logger.info(f"WellSetup: HW config received "
                     f"(inks={list(config.ink_library.keys())}, "
                     f"rosettes={list(config.rosette_library.keys())})")
        self._refresh_ink_options_from_hw_config(config)
        self._refresh_rosette_options_from_hw_config(config)
        if hasattr(self, '_model') and self._model:
            if config.plate_format != getattr(self._model, '_plate_format', None):
                try:
                    self._model.set_plate_format(config.plate_format)
                    self._refresh_plate()
                except Exception as e:
                    logger.error(f"Plate format sync failed: {e}")

'''
            content = content[:m.start()] + new_shc + content[m.end():]
            changed = True
            rc(True, "Enhanced set_hardware_config with new helpers")
        else:
            rc(False, "set_hardware_config already uses helpers")
    else:
        rc(False, "Helpers not available for set_hardware_config")

    # ──────────────────────────────────────────────────────────────
    # 2H. Ensure validate + get_plan exist
    # ──────────────────────────────────────────────────────────────

    if 'def validate(self) -> tuple' not in content:
        validate_code = '''
    def validate(self) -> tuple[bool, list[str]]:
        """v7.2.6: Validation for send-to-monitor."""
        if getattr(self, '_plan', None) is None and getattr(self, '_hw_config', None) is not None:
            if hasattr(self, '_generate_plan'): self._generate_plan()
        try:
            from SupportClasses.PrintPlanOfAction import validate_well_setup
            return validate_well_setup(hw_config=self._hw_config, well_model=self._model, plan=getattr(self, '_plan', None))
        except ImportError:
            return False, ["PrintPlanOfAction not available"]

    def get_plan(self):
        """v7.2.6: Get current execution plan."""
        return getattr(self, '_plan', None)

'''
        anchor = '    def _save_layout'
        if anchor not in content:
            anchor = '    def _validate_setup'
        if anchor in content:
            content = content.replace(anchor, validate_code + anchor, 1)
            changed = True
            rc(True, "Added validate() + get_plan()")
        else:
            rc(False, "No anchor for validate/get_plan")
    else:
        rc(False, "validate() already exists")

    if changed:
        safe_write(path, content, "projections removed + role workflow + colors")
    else:
        print(f"  {Y}No changes needed{X}")


# ═══════════════════════════════════════════════════════════════════
def main():
    root = Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else find_root()
    print(f"\n{B}{'═' * 60}{X}")
    print(f"{B}MEBP v7.2.6 — Batch 3: Large File Rewrites{X}")
    print(f"{B}{'═' * 60}{X}")
    print(f"Project root: {root}")

    fix_print_objects(root)
    fix_print_well_setup(root)

    print(f"\n{B}{'═' * 60}{X}")
    print(f"{B}SUMMARY{X}")
    print(f"  {G}Written{X}: {_ok}  {Y}Skipped{X}: {_skip}  {R}Failed{X}: {_fail}")
    print(f"{B}{'═' * 60}{X}")
    if _fail > 0: sys.exit(1)
    print(f"{G}✓ Batch 3 complete.{X}")

if __name__ == "__main__":
    main()

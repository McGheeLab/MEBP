#!/usr/bin/env python3
"""
MEBP v7.2.5 — Session 3 Patch
Prints List + HW Ink/Rosette Sync

Issues Covered: #5 (Send to Available Prints button, populate well setup)
                #8 (Ensure all inks/rosettes from HardwareConfig in well setup)

Files Modified:
    gui/pages/print_objects.py    — Add explicit "Send to Available Prints" button
    gui/pages/print_well_setup.py — Enhance ink combo to show "InkName (P1)" format,
                                    refresh rosette combo with HW library
    gui/pages/print_setup.py      — Ensure _on_collections_changed forwards correctly

Prerequisites:
    - Session 1 patch applied
    - Session 2 patch applied

Usage:
    python patch_s3_prints_and_hw_sync.py [/path/to/MEBP]
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
    backup = filepath.with_suffix(f".bak_v725s3_{ts}")
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


def regex_replace(content: str, pattern: str, replacement: str, description: str) -> str:
    global _applied, _skipped, _failed
    match = re.search(pattern, content, re.DOTALL)
    if match is None:
        if replacement.strip()[:60] in content:
            print(f"  {YELLOW}SKIP{RESET}: {description} — already applied")
            _skipped += 1
        else:
            print(f"  {RED}MISS{RESET}: {description} — pattern not found")
            _failed += 1
        return content
    content = content[:match.start()] + replacement + content[match.end():]
    print(f"  {GREEN}OK{RESET}:   {description}")
    _applied += 1
    return content


# ═══════════════════════════════════════════════════════════════════
# PATCH A: print_objects.py — "Send to Available Prints" button
# ═══════════════════════════════════════════════════════════════════

def patch_print_objects(root: Path):
    filepath = root / "gui" / "pages" / "print_objects.py"
    print(f"\n{'═' * 60}")
    print(f"PATCH A: {filepath.name} — Send to Available Prints")
    print(f"{'═' * 60}")

    if not filepath.exists():
        print(f"  {RED}ERROR{RESET}: File not found!")
        return

    backup_file(filepath)
    content = read_file(filepath)

    # ── A1: Add "Send to Available Prints" button after summary label ──
    # The _build_ui method has a _summary_label. We add a button after it.
    # Look for the summary label creation and add button after
    send_button_code = '''
        # v7.2.5: "Send to Available Prints" button
        self._btn_send_to_prints = QPushButton("📋 Send to Available Prints")
        self._btn_send_to_prints.setToolTip(
            "Refresh the list of available prints for well setup assignment")
        self._btn_send_to_prints.setStyleSheet(f"""
            QPushButton {{
                background: {COLORS.get('mauve', '#cba6f7')};
                color: {COLORS.get('base', '#1e1e2e')};
                font-weight: bold; padding: 6px 12px;
                border-radius: 4px; font-size: 11px;
            }}
            QPushButton:hover {{
                background: {COLORS.get('pink', '#f5c2e7')};
            }}
        """)
        self._btn_send_to_prints.clicked.connect(self._send_to_available_prints)
'''

    # Find a good anchor — after _summary_label is added to layout
    if '_btn_send_to_prints' in content:
        print(f"  {YELLOW}SKIP{RESET}: A1: Send to Available Prints button already exists")
        _skipped_inc()
    else:
        # Try to find the summary label addition point
        anchor_pattern = re.compile(
            r'(self\._summary_label\.setText\("No objects"\)\s*\n)'
            r'|'
            r'(self\._summary_label = QLabel\([^)]*\)[^\n]*\n)',
            re.DOTALL
        )
        match = anchor_pattern.search(content)
        if match:
            insert_pos = match.end()
            content = content[:insert_pos] + send_button_code + content[insert_pos:]
            print(f"  {GREEN}OK{RESET}:   A1: Added Send to Available Prints button")
            global _applied
            _applied += 1
        else:
            # Fallback: insert before _restore_last_print call
            content = insert_before(
                content,
                '        self._restore_last_print()',
                send_button_code,
                "A1: Added Send to Available Prints button (fallback anchor)"
            )

    # ── A2: Add the _send_to_available_prints method ──
    send_method = '''
    def _send_to_available_prints(self):
        """v7.2.5: Explicitly send current prints list to well setup."""
        self._emit_prints_changed()
        # Provide visual feedback
        if hasattr(self, '_btn_send_to_prints'):
            original_text = self._btn_send_to_prints.text()
            self._btn_send_to_prints.setText("✓ Prints list sent!")
            self._btn_send_to_prints.setStyleSheet(f"""
                QPushButton {{
                    background: {COLORS.get('green', '#a6e3a1')};
                    color: {COLORS.get('base', '#1e1e2e')};
                    font-weight: bold; padding: 6px 12px;
                    border-radius: 4px; font-size: 11px;
                }}
            """)
            QTimer.singleShot(1500, lambda: self._restore_send_button(original_text))

    def _restore_send_button(self, text: str):
        """Restore send button to default style after feedback."""
        if hasattr(self, '_btn_send_to_prints'):
            self._btn_send_to_prints.setText(text)
            self._btn_send_to_prints.setStyleSheet(f"""
                QPushButton {{
                    background: {COLORS.get('mauve', '#cba6f7')};
                    color: {COLORS.get('base', '#1e1e2e')};
                    font-weight: bold; padding: 6px 12px;
                    border-radius: 4px; font-size: 11px;
                }}
                QPushButton:hover {{
                    background: {COLORS.get('pink', '#f5c2e7')};
                }}
            """)

'''
    # Insert before _emit_prints_changed
    content = insert_before(
        content,
        '    def _emit_prints_changed(self):',
        send_method,
        "A2: Add _send_to_available_prints method"
    )

    # ── A3: Also emit prints_changed on file create/save/delete/load ──
    # Ensure _do_auto_save emits prints_changed
    if '_emit_prints_changed()' not in content.split('_do_auto_save')[1].split('\n    def ')[0] if '_do_auto_save' in content else '':
        content = insert_after(
            content,
            '        self._auto_save_timer.timeout.connect(self._do_auto_save)',
            '',
            "A3: (info) Auto-save already connected"
        )

    # ── A4: Ensure prints_changed emits on _new_print ──
    # Check if _new_print calls _emit_prints_changed
    if '_new_print' in content:
        new_print_section = content[content.find('def _new_print'):content.find('\n    def ', content.find('def _new_print') + 20)]
        if '_emit_prints_changed' not in new_print_section:
            # Add it at end of _new_print
            content = regex_replace(
                content,
                r'(def _new_print\(self\):[^\n]*\n(?:.*?\n)*?)(        self\._refresh_file_combo\(\))',
                r'\1\2\n        self._emit_prints_changed()',
                "A4: Add _emit_prints_changed to _new_print"
            )

    # ── A5: Ensure prints_changed emits on _delete_print ──
    if '_delete_print' in content:
        delete_section = content[content.find('def _delete_print'):content.find('\n    def ', content.find('def _delete_print') + 20)]
        if '_emit_prints_changed' not in delete_section:
            content = regex_replace(
                content,
                r'(def _delete_print\(self\):[^\n]*\n(?:.*?\n)*?)(        self\._refresh_file_combo\(\))',
                r'\1\2\n        self._emit_prints_changed()',
                "A5: Add _emit_prints_changed to _delete_print"
            )

    # ── A6: Add button to layout (find objects_right_layout or similar) ──
    # The button was added to self but we need to add it to a layout
    # Look for where _summary_label is added to a layout
    if '_btn_send_to_prints' in content and 'addWidget(self._btn_send_to_prints)' not in content:
        # Find where _summary_label is added
        summary_add = re.search(
            r'(\w+)\.addWidget\(self\._summary_label\)',
            content
        )
        if summary_add:
            layout_var = summary_add.group(1)
            content = insert_after(
                content,
                f'{layout_var}.addWidget(self._summary_label)',
                f'\n        {layout_var}.addWidget(self._btn_send_to_prints)',
                "A6: Add Send button to layout after summary label"
            )

    write_file(filepath, content)


def _skipped_inc():
    global _skipped
    _skipped += 1


# ═══════════════════════════════════════════════════════════════════
# PATCH B: print_well_setup.py — Enhanced ink/rosette combos
# ═══════════════════════════════════════════════════════════════════

def patch_well_setup(root: Path):
    filepath = root / "gui" / "pages" / "print_well_setup.py"
    print(f"\n{'═' * 60}")
    print(f"PATCH B: {filepath.name} — Enhanced Ink/Rosette Sync")
    print(f"{'═' * 60}")

    if not filepath.exists():
        print(f"  {RED}ERROR{RESET}: File not found!")
        return

    backup_file(filepath)
    content = read_file(filepath)

    # ── B1: Enhance set_hardware_config ink combo to show "InkName (P1)" format ──
    old_ink_refresh = (
        '        # Refresh ink combo from HardwareConfig ink library\n'
        '        if hasattr(self, \'ink_combo\'):\n'
        '            current_ink = self.ink_combo.currentData()\n'
        '            self.ink_combo.clear()\n'
        '            self.ink_combo.addItem("(none)", None)\n'
        '            for name in config.ink_library:\n'
        '                self.ink_combo.addItem(name, name)\n'
        '            # Restore previous selection if still valid\n'
        '            if current_ink:\n'
        '                idx = self.ink_combo.findData(current_ink)\n'
        '                if idx >= 0:\n'
        '                    self.ink_combo.setCurrentIndex(idx)'
    )
    new_ink_refresh = (
        '        # v7.2.5: Refresh ink combo — show "InkName (P1)" format\n'
        '        if hasattr(self, \'ink_combo\'):\n'
        '            current_ink = self.ink_combo.currentData()\n'
        '            self.ink_combo.clear()\n'
        '            self.ink_combo.addItem("(none)", None)\n'
        '            # Build pump→ink reverse mapping\n'
        '            ink_to_pump = {}\n'
        '            if hasattr(config, \'pumps\'):\n'
        '                for pid, pcfg in config.pumps.items():\n'
        '                    if pcfg.is_configured and pcfg.ink_name:\n'
        '                        ink_to_pump[pcfg.ink_name] = pid\n'
        '            for name in config.ink_library:\n'
        '                pump_tag = ink_to_pump.get(name, "")\n'
        '                display = f"{name} ({pump_tag})" if pump_tag else name\n'
        '                self.ink_combo.addItem(display, name)\n'
        '            # Restore previous selection if still valid\n'
        '            if current_ink:\n'
        '                idx = self.ink_combo.findData(current_ink)\n'
        '                if idx >= 0:\n'
        '                    self.ink_combo.setCurrentIndex(idx)\n'
        '            logger.debug(f"Well setup ink combo: {self.ink_combo.count()-1} inks, "\n'
        '                         f"pump mapping: {ink_to_pump}")'
    )
    content = patch_replace(content, old_ink_refresh, new_ink_refresh,
                            "B1: Enhance ink combo to show InkName (P1) format")

    # ── B2: Enhance set_available_prints to actually populate the print_combo ──
    old_set_prints = '    def set_available_prints(self, names: list[str]) -> None:\n        """Update available print collections (called when Tab 2 changes)."""'
    # Find the full method body
    set_prints_match = re.search(
        r'(    def set_available_prints\(self, names: list\[str\]\) -> None:.*?)'
        r'(?=\n    def |\nclass |\Z)',
        content, re.DOTALL
    )
    if set_prints_match:
        old_method = set_prints_match.group(1)
        new_method = '''    def set_available_prints(self, names: list[str]) -> None:
        """v7.2.5: Update available prints — populate print_combo dropdown."""
        self._available_prints = names or []
        if hasattr(self, 'print_combo'):
            current = self.print_combo.currentData()
            self.print_combo.blockSignals(True)
            self.print_combo.clear()
            if self._available_prints:
                for name in self._available_prints:
                    self.print_combo.addItem(name, name)
                # Restore previous selection if still valid
                if current:
                    idx = self.print_combo.findData(current)
                    if idx >= 0:
                        self.print_combo.setCurrentIndex(idx)
            else:
                self.print_combo.addItem("(no prints available)", None)
            self.print_combo.blockSignals(False)
            logger.debug(f"Well setup: {len(self._available_prints)} prints available")
'''
        if 'v7.2.5: Update available prints' in content:
            print(f"  {YELLOW}SKIP{RESET}: B2: set_available_prints already enhanced")
            global _skipped
            _skipped += 1
        else:
            content = content.replace(old_method, new_method, 1)
            print(f"  {GREEN}OK{RESET}:   B2: Enhanced set_available_prints with print_combo population")
            global _applied
            _applied += 1
    else:
        print(f"  {RED}MISS{RESET}: B2: set_available_prints method not found")
        global _failed
        _failed += 1

    # ── B3: Add pump-ink info to rosette combo tooltip ──
    old_rosette_refresh = (
        '        # Refresh rosette combo from HardwareConfig rosette library\n'
        '        if hasattr(self, \'rosette_combo\'):\n'
        '            current_ros = self.rosette_combo.currentData()\n'
        '            self.rosette_combo.clear()\n'
        '            self.rosette_combo.addItem("None", None)\n'
        '            for name in config.rosette_library:\n'
        '                self.rosette_combo.addItem(name, name)\n'
        '            if current_ros:\n'
        '                idx = self.rosette_combo.findData(current_ros)\n'
        '                if idx >= 0:\n'
        '                    self.rosette_combo.setCurrentIndex(idx)'
    )
    new_rosette_refresh = (
        '        # v7.2.5: Refresh rosette combo with sub-well info\n'
        '        if hasattr(self, \'rosette_combo\'):\n'
        '            current_ros = self.rosette_combo.currentData()\n'
        '            self.rosette_combo.clear()\n'
        '            self.rosette_combo.addItem("None", None)\n'
        '            for name, rosette in config.rosette_library.items():\n'
        '                n_sub = getattr(rosette, "num_subwells", "?")\n'
        '                display = f"{name} ({n_sub} sub-wells)"\n'
        '                self.rosette_combo.addItem(display, name)\n'
        '            if current_ros:\n'
        '                idx = self.rosette_combo.findData(current_ros)\n'
        '                if idx >= 0:\n'
        '                    self.rosette_combo.setCurrentIndex(idx)\n'
        '            logger.debug(f"Well setup rosette combo: "\n'
        '                         f"{self.rosette_combo.count()-1} rosettes")'
    )
    content = patch_replace(content, old_rosette_refresh, new_rosette_refresh,
                            "B3: Enhance rosette combo with sub-well count display")

    write_file(filepath, content)


# ═══════════════════════════════════════════════════════════════════
# PATCH C: print_setup.py — Ensure collections forwarding
# ═══════════════════════════════════════════════════════════════════

def patch_print_setup_forwarding(root: Path):
    filepath = root / "gui" / "pages" / "print_setup.py"
    print(f"\n{'═' * 60}")
    print(f"PATCH C: {filepath.name} — Prints Forwarding Verification")
    print(f"{'═' * 60}")

    if not filepath.exists():
        print(f"  {RED}ERROR{RESET}: File not found!")
        return

    backup_file(filepath)
    content = read_file(filepath)

    # ── C1: Enhance _on_collections_changed to also log + handle prints_changed ──
    old_collections = (
        '    def _on_collections_changed(self, collection_names: list[str]):\n'
        '        """Print Objects tab changed collections → update Well Setup tab."""\n'
        '        if hasattr(self.tab_wells, \'set_available_prints\'):\n'
        '            self.tab_wells.set_available_prints(collection_names)'
    )
    new_collections = (
        '    def _on_collections_changed(self, collection_names: list[str]):\n'
        '        """v7.2.5: Print Objects tab changed collections → update Well Setup tab."""\n'
        '        if hasattr(self.tab_wells, \'set_available_prints\'):\n'
        '            self.tab_wells.set_available_prints(collection_names)\n'
        '            logger.info(f"Forwarded {len(collection_names)} prints to well setup")'
    )
    content = patch_replace(content, old_collections, new_collections,
                            "C1: Add logging to collections forwarding")

    # ── C2: Ensure both prints_changed AND collections_changed are connected ──
    # Check if prints_changed signal is connected
    if 'prints_changed' not in content or 'tab_objects.prints_changed' not in content:
        # Look for collections_changed connection and add prints_changed alongside
        old_connect = 'self.tab_objects.collections_changed.connect(self._on_collections_changed)'
        new_connect = (
            'self.tab_objects.collections_changed.connect(self._on_collections_changed)\n'
            '        # v7.2.5: Also connect prints_changed (primary signal)\n'
            '        if hasattr(self.tab_objects, \'prints_changed\'):\n'
            '            self.tab_objects.prints_changed.connect(self._on_collections_changed)'
        )
        content = patch_replace(content, old_connect, new_connect,
                                "C2: Connect prints_changed signal alongside collections_changed")

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
    print(f" MEBP v7.2.5 — Session 3 Patch")
    print(f" Prints List + HW Ink/Rosette Sync")
    print(f"{'=' * 60}{RESET}")
    print(f"Project root: {root}")

    patch_print_objects(root)
    patch_well_setup(root)
    patch_print_setup_forwarding(root)

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

#!/usr/bin/env python3
"""
MEBP v7.2.5 — Session 5 Patch
Well Border Colors + Integration Wiring

Issues Covered: #6 (continued — well border colors: green=ready, red=incomplete, gray=empty)

Files Modified:
    gui/widgets/well_plate_view.py   — Add update_well_status for border colors
    gui/pages/print_well_setup.py    — Wire status updates on assignment changes

Prerequisites:
    - Sessions 1-4 patches applied

Usage:
    python patch_s5_colors_and_integration.py [/path/to/MEBP]
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
    backup = filepath.with_suffix(f".bak_v725s5_{ts}")
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


# ═══════════════════════════════════════════════════════════════════
# PATCH A: well_plate_view.py — Well status border colors
# ═══════════════════════════════════════════════════════════════════

WELL_STATUS_METHOD = '''
    def update_well_status(self, status_map: dict[str, str]):
        """
        v7.2.5: Update well border colors based on assignment status.

        Args:
            status_map: {well_name: "ready" | "incomplete" | "empty"}

        Border colors:
            ready      → green (#a6e3a1) — fully configured
            incomplete → red (#f38ba8) — has role but missing details
            empty      → gray (#585b70) — no assignment
        """
        STATUS_COLORS = {
            "ready":      "#a6e3a1",  # Green
            "incomplete": "#f38ba8",  # Red
            "empty":      "#585b70",  # Gray
        }

        for well_name, status in status_map.items():
            item = self._well_items.get(well_name)
            if item is None:
                continue

            color = STATUS_COLORS.get(status, STATUS_COLORS["empty"])

            # Only update border if well is not currently selected
            # (selected wells keep their selection highlight)
            if well_name not in self._selected_wells:
                pen = QPen(QColor(color), WELL_BORDER_WIDTH)
                item.setPen(pen)

            # Store status for re-application after selection changes
            if not hasattr(self, '_well_status'):
                self._well_status = {}
            self._well_status[well_name] = status

    def _restore_well_borders(self):
        """v7.2.5: Restore status-based borders after selection changes."""
        if not hasattr(self, '_well_status'):
            return

        STATUS_COLORS = {
            "ready":      "#a6e3a1",
            "incomplete": "#f38ba8",
            "empty":      "#585b70",
        }

        for well_name, item in self._well_items.items():
            if well_name in self._selected_wells:
                continue  # Keep selection highlight
            status = self._well_status.get(well_name, "empty")
            color = STATUS_COLORS.get(status, STATUS_COLORS["empty"])
            pen = QPen(QColor(color), WELL_BORDER_WIDTH)
            item.setPen(pen)

'''


def patch_well_plate_view(root: Path):
    filepath = root / "gui" / "widgets" / "well_plate_view.py"
    print(f"\n{'═' * 60}")
    print(f"PATCH A: {filepath.name} — Well Status Border Colors")
    print(f"{'═' * 60}")

    if not filepath.exists():
        print(f"  {RED}ERROR{RESET}: File not found!")
        return

    backup_file(filepath)
    content = read_file(filepath)

    # ── A1: Add update_well_status and _restore_well_borders methods ──
    content = insert_before(
        content,
        '\nclass WellRoleLegend',
        WELL_STATUS_METHOD,
        "A1: Add update_well_status and _restore_well_borders"
    )

    # ── A2: Call _restore_well_borders after selection changes ──
    # Find where selection_changed is emitted and add restore call
    if '_restore_well_borders' in content and '_restore_well_borders()' not in content.split('selection_changed.emit')[0]:
        # Find the selection_changed.emit call in mouseReleaseEvent
        old_emit = '            self.selection_changed.emit(list(self._selected_wells))'
        new_emit = (
            '            self._restore_well_borders()  # v7.2.5: restore status colors\n'
            '            self.selection_changed.emit(list(self._selected_wells))'
        )
        content = patch_replace(content, old_emit, new_emit,
                                "A2: Call _restore_well_borders after selection changes")

    write_file(filepath, content)


# ═══════════════════════════════════════════════════════════════════
# PATCH B: print_well_setup.py — Wire status updates
# ═══════════════════════════════════════════════════════════════════

REFRESH_STATUS_METHOD = '''
    def _refresh_well_status_colors(self):
        """v7.2.5: Update well border colors based on assignment status."""
        if not hasattr(self, '_plate_view') or not self._plate_view:
            return

        status_map = {}
        for name, wa in self._model.assignments.items():
            if wa.role == WellRole.EMPTY:
                status_map[name] = "empty"
            elif self._check_well_ready(wa):
                status_map[name] = "ready"
            else:
                status_map[name] = "incomplete"

        if hasattr(self._plate_view, 'update_well_status'):
            self._plate_view.update_well_status(status_map)

'''


def patch_well_setup(root: Path):
    filepath = root / "gui" / "pages" / "print_well_setup.py"
    print(f"\n{'═' * 60}")
    print(f"PATCH B: {filepath.name} — Wire Status Color Updates")
    print(f"{'═' * 60}")

    if not filepath.exists():
        print(f"  {RED}ERROR{RESET}: File not found!")
        return

    backup_file(filepath)
    content = read_file(filepath)

    # ── B1: Add _refresh_well_status_colors method ──
    # Insert before _save_layout or at end of class methods
    if '_refresh_well_status_colors' not in content:
        # Find a good insertion point
        if '    def _save_layout' in content:
            content = insert_before(
                content,
                '    def _save_layout',
                REFRESH_STATUS_METHOD,
                "B1: Add _refresh_well_status_colors method"
            )
        elif '    def validate' in content:
            content = insert_before(
                content,
                '    def validate',
                REFRESH_STATUS_METHOD,
                "B1: Add _refresh_well_status_colors method (before validate)"
            )

    # ── B2: Call _refresh_well_status_colors after role changes ──
    # Find _apply_role and add the call
    if '_refresh_well_status_colors' in content:
        # Wire into setup_changed emission points
        # Find all setup_changed.emit() calls and add status refresh after them
        # But only do it once — add to _apply_role and _apply_ink

        # For _apply_active_role (v7.2.5 new method)
        if '_apply_active_role' in content and '_refresh_well_status_colors' not in \
                content[content.find('_apply_active_role'):content.find('\n    def ', content.find('_apply_active_role') + 20)]:
            old_emit = '        self.setup_changed.emit()\n        # Auto-regenerate plan if available\n        if hasattr(self, \'_on_plan_auto_regen\'):'
            new_emit = '        self._refresh_well_status_colors()\n        self.setup_changed.emit()\n        # Auto-regenerate plan if available\n        if hasattr(self, \'_on_plan_auto_regen\'):'
            content = patch_replace(content, old_emit, new_emit,
                                    "B2: Wire status colors into _apply_active_role")

        # For legacy _apply_role
        if '_apply_role' in content and 'def _apply_role(self)' in content:
            # Find the setup_changed.emit in _apply_role specifically
            apply_role_section = content[content.find('def _apply_role(self)'):
                                         content.find('\n    def ', content.find('def _apply_role(self)') + 20)]
            if '_refresh_well_status_colors' not in apply_role_section:
                # Add after the setup_changed.emit in _apply_role
                old_apply_emit = (
                    '        self._plate_view.update_well_roles(self._model.get_role_map())\n'
                    '        self.setup_changed.emit()'
                )
                new_apply_emit = (
                    '        self._plate_view.update_well_roles(self._model.get_role_map())\n'
                    '        self._refresh_well_status_colors()\n'
                    '        self._refresh_summary_table()\n'
                    '        self.setup_changed.emit()'
                )
                content = patch_replace(content, old_apply_emit, new_apply_emit,
                                        "B2b: Wire status colors into legacy _apply_role")

        # For _apply_ink
        if '_apply_ink' in content:
            apply_ink_section = content[content.find('def _apply_ink'):
                                        content.find('\n    def ', content.find('def _apply_ink') + 15)]
            if '_refresh_well_status_colors' not in apply_ink_section and 'setup_changed.emit()' in apply_ink_section:
                # Find the specific emit in _apply_ink
                ink_emit_start = content.find('def _apply_ink')
                ink_emit_pos = content.find('self.setup_changed.emit()', ink_emit_start)
                if ink_emit_pos > 0:
                    content = (
                        content[:ink_emit_pos] +
                        'self._refresh_well_status_colors()\n        self._refresh_summary_table()\n        ' +
                        content[ink_emit_pos:]
                    )
                    print(f"  {GREEN}OK{RESET}:   B2c: Wire status colors into _apply_ink")
                    global _applied
                    _applied += 1

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
    print(f" MEBP v7.2.5 — Session 5 Patch")
    print(f" Well Border Colors + Integration Wiring")
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

#!/usr/bin/env python3
"""
MEBP v7.2.5 — Session 6 Patch: Layout Fixes

Fixes two gaps identified in the v7.2.5 audit:

G1: P1/P2/P3 column headers in jog_control.py don't show "(µL)" units
G2: Well setup summary table is below plate view instead of beside it
    — restructures to QSplitter(horizontal) with plate LEFT, summary RIGHT

Files Modified:
    gui/pages/jog_control.py         — Fix pump position header labels
    gui/pages/print_well_setup.py    — Restructure to side-by-side layout

Prerequisites:
    - All v7.2.5 Sessions 1-5 patches applied

Usage:
    python patch_s6_layout_fixes.py [/path/to/MEBP]
"""

import os
import re
import sys

# === Windows console encoding fix ===
import sys as _sys
if _sys.platform == 'win32':
    for _stream_name in ('stdout', 'stderr'):
        _stream = getattr(_sys, _stream_name, None)
        if _stream and hasattr(_stream, 'reconfigure'):
            _stream.reconfigure(encoding='utf-8', errors='replace')
# === End encoding fix ===

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
    backup = filepath.with_suffix(f".bak_v725s6_{ts}")
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


def patch_replace_regex(content: str, pattern: str, replacement: str, description: str) -> str:
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
# FIX G1: jog_control.py — Pump position header labels
# ═══════════════════════════════════════════════════════════════════

def fix_jog_headers(root: Path):
    filepath = root / "gui" / "pages" / "jog_control.py"
    print(f"\n{'═' * 60}")
    print(f"FIX G1: {filepath.name} — Pump position header labels")
    print(f"{'═' * 60}")

    if not filepath.exists():
        print(f"  {RED}ERROR{RESET}: File not found!")
        return

    backup_file(filepath)
    content = read_file(filepath)

    # Try exact match first (standard indentation)
    old_headers = (
        '            ("P1:", "lbl_p1"), ("P2:", "lbl_p2"), ("P3:", "lbl_p3"),'
    )
    new_headers = (
        '            ("P1 (µL):", "lbl_p1"), ("P2 (µL):", "lbl_p2"), ("P3 (µL):", "lbl_p3"),'
    )
    content = patch_replace(content, old_headers, new_headers,
                            "G1a: Update P1/P2/P3 headers to show (µL)")

    # If exact match failed, try regex for flexible whitespace
    if 'P1 (µL)' not in content:
        pattern = r'\("P1:", "lbl_p1"\), \("P2:", "lbl_p2"\), \("P3:", "lbl_p3"\),'
        replacement = '("P1 (µL):", "lbl_p1"), ("P2 (µL):", "lbl_p2"), ("P3 (µL):", "lbl_p3"),'
        content = patch_replace_regex(content, pattern, replacement,
                                       "G1b: Update P1/P2/P3 headers (regex fallback)")

    # Also update the S2 patch A10 header if it was already changed to P1 (µL)
    # but we want to handle the case where S2 A10 changed the already-in-loop format
    # E.g., if the tuple is spread across multiple lines
    if 'P1 (µL)' not in content:
        # Try multiline match
        pattern_ml = re.compile(
            r'\("P1:"\s*,\s*"lbl_p1"\)\s*,\s*\("P2:"\s*,\s*"lbl_p2"\)\s*,\s*\("P3:"\s*,\s*"lbl_p3"\)\s*,',
            re.DOTALL
        )
        match = pattern_ml.search(content)
        if match:
            content = content[:match.start()] + \
                      '("P1 (µL):", "lbl_p1"), ("P2 (µL):", "lbl_p2"), ("P3 (µL):", "lbl_p3"),' + \
                      content[match.end():]
            global _applied
            _applied += 1
            print(f"  {GREEN}OK{RESET}:   G1c: Update P1/P2/P3 headers (multiline fallback)")

    write_file(filepath, content)


# ═══════════════════════════════════════════════════════════════════
# FIX G2: print_well_setup.py — Side-by-side layout with QSplitter
# ═══════════════════════════════════════════════════════════════════

def fix_well_setup_layout(root: Path):
    filepath = root / "gui" / "pages" / "print_well_setup.py"
    print(f"\n{'═' * 60}")
    print(f"FIX G2: {filepath.name} — Side-by-side plate + summary layout")
    print(f"{'═' * 60}")

    if not filepath.exists():
        print(f"  {RED}ERROR{RESET}: File not found!")
        return

    backup_file(filepath)
    content = read_file(filepath)

    # ── G2.1: Ensure QSplitter is imported ──
    if 'QSplitter' not in content:
        content = patch_replace(
            content,
            'from PySide6.QtWidgets import (',
            'from PySide6.QtWidgets import (\n    QSplitter,',
            "G2.1: Add QSplitter import"
        )
    else:
        print(f"  {YELLOW}SKIP{RESET}: G2.1 — QSplitter already imported")

    # ── G2.2: Find the plate view creation and wrap in splitter ──
    # We need to restructure: plate view (full width) → plate view (left) + summary (right)
    # The approach: Find where the plate view is added to the scroll layout,
    # and instead add it to a QSplitter alongside a new summary widget.

    # Current layout adds plate view like:
    #   parent_layout.addWidget(self._plate_view, stretch=1)
    # or via the _build_plate_view method.

    # We'll add a _build_top_splitter method that creates the side-by-side layout
    # and call it in place of the plate view + summary table.

    new_splitter_method = '''
    def _build_top_splitter(self, parent_layout: QVBoxLayout) -> None:
        """v7.2.5 G2: Build side-by-side layout with plate view (left) + summary (right)."""
        from PySide6.QtCore import Qt as QtCore_Qt

        splitter = QSplitter(QtCore_Qt.Orientation.Horizontal)
        splitter.setHandleWidth(3)

        # ── Left pane: Plate view ──
        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.setSpacing(4)

        # Move the plate view into the left pane
        if hasattr(self, '_plate_view') and self._plate_view is not None:
            # Remove from any existing parent layout
            old_parent = self._plate_view.parent()
            if old_parent and old_parent.layout():
                old_parent.layout().removeWidget(self._plate_view)
            left_layout.addWidget(self._plate_view, stretch=1)

        left_widget.setLayout(left_layout)
        splitter.addWidget(left_widget)

        # ── Right pane: Assignment summary ──
        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.setSpacing(4)

        summary_label = QLabel("Assignment Summary")
        summary_label.setStyleSheet(
            f"color: {COLORS['blue']}; font-weight: bold; font-size: 12px;")
        right_layout.addWidget(summary_label)

        # Summary table
        if not hasattr(self, '_summary_table'):
            self._summary_table = QTableWidget()
            self._summary_table.setColumnCount(4)
            self._summary_table.setHorizontalHeaderLabels(
                ["Well", "Role", "Assignment", "Status"])
            self._summary_table.horizontalHeader().setStretchLastSection(True)
            self._summary_table.setSelectionBehavior(
                QTableWidget.SelectionBehavior.SelectRows)
            self._summary_table.setEditTriggers(
                QTableWidget.EditTrigger.NoEditTriggers)
            self._summary_table.setAlternatingRowColors(True)
            self._summary_table.cellClicked.connect(self._on_summary_row_clicked)
        right_layout.addWidget(self._summary_table, stretch=1)

        # Summary counts footer
        if not hasattr(self, '_summary_counts_label'):
            self._summary_counts_label = QLabel("")
            self._summary_counts_label.setStyleSheet(
                f"color: {COLORS.get('overlay0', '#6c7086')}; font-size: 10px;")
        right_layout.addWidget(self._summary_counts_label)

        right_widget.setLayout(right_layout)
        splitter.addWidget(right_widget)

        # Set initial sizes: ~70% plate, ~30% summary
        splitter.setSizes([700, 300])

        self._top_splitter = splitter
        parent_layout.addWidget(splitter, stretch=2)

'''

    # Check if _build_top_splitter already exists
    if '_build_top_splitter' in content:
        print(f"  {YELLOW}SKIP{RESET}: G2.2 — _build_top_splitter already exists")
    else:
        # Insert before _build_selection_actions
        content = insert_before(
            content,
            '    def _build_selection_actions',
            new_splitter_method,
            "G2.2: Add _build_top_splitter method"
        )

    # ── G2.3: Wire the splitter into _build_ui ──
    # Find where _build_plate_view is called and add the splitter call after it.
    # Then remove the separate _build_summary_table call.

    # Look for the plate view being added to scroll_layout
    # Common patterns:
    #   self._build_plate_view(scroll_layout)
    #   parent_layout.addWidget(self._plate_view)
    # We need to ADD the splitter call and REMOVE the separate summary table call.

    # Strategy: After the plate view build, insert the splitter call.
    # The splitter will MOVE the plate view into its left pane.
    # Then we skip the old summary table build.

    # First, add the splitter build call after plate view creation
    plate_view_anchor = 'self._build_plate_view(scroll_layout)'
    if plate_view_anchor in content and '_build_top_splitter' in content:
        # Replace: _build_plate_view(scroll_layout)
        # With:    _build_plate_view(scroll_layout) then _build_top_splitter
        old_plate_call = '        self._build_plate_view(scroll_layout)'
        new_plate_call = (
            '        self._build_plate_view(scroll_layout)\n'
            '        # v7.2.5 G2: Restructure to side-by-side layout\n'
            '        self._build_top_splitter(scroll_layout)'
        )
        content = patch_replace(content, old_plate_call, new_plate_call,
                                "G2.3: Wire _build_top_splitter after plate view")

    # ── G2.4: Remove or skip the old _build_summary_table call ──
    # The summary table will now be built inside _build_top_splitter,
    # so the old standalone call should be removed.
    old_summary_call = '        self._build_summary_table(scroll_layout)'
    if old_summary_call in content and '_build_top_splitter' in content:
        new_summary_call = (
            '        # v7.2.5 G2: Summary table now inside top splitter (right pane)\n'
            '        # self._build_summary_table(scroll_layout)  # moved to _build_top_splitter'
        )
        content = patch_replace(content, old_summary_call, new_summary_call,
                                "G2.4: Comment out old summary table call (now in splitter)")

    # ── G2.5: Add QTableWidget import if needed ──
    if 'QTableWidget' not in content:
        content = patch_replace(
            content,
            'from PySide6.QtWidgets import (',
            'from PySide6.QtWidgets import (\n    QTableWidget,',
            "G2.5: Add QTableWidget import"
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
    print(f" MEBP v7.2.5 — Session 6: Layout Fixes")
    print(f" G1: Pump header labels (µL)")
    print(f" G2: Side-by-side plate + summary layout")
    print(f"{'=' * 60}{RESET}")
    print(f"Project root: {root}")

    fix_jog_headers(root)
    fix_well_setup_layout(root)

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
        print("Check the MISS messages above.")
        sys.exit(1)
    else:
        print(f"\n{GREEN}All layout fixes applied successfully!{RESET}")
        print(f"\nFiles modified:")
        print(f"  gui/pages/jog_control.py         — P1/P2/P3 header labels → (µL)")
        print(f"  gui/pages/print_well_setup.py    — QSplitter: plate LEFT + summary RIGHT")


if __name__ == "__main__":
    main()

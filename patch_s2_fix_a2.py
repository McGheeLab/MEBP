#!/usr/bin/env python3
"""
MEBP v7.2.4 — Session 2 Fix: Apply the missed A2 patch (conversion factor + warning labels).

The main patch_s2 missed A2 because the anchor text had a whitespace/dash mismatch.
This fix finds the correct insertion point dynamically.

Usage:
    python patch_s2_fix_a2.py [project_root]
"""

import sys
import re
from pathlib import Path

BOLD = "\033[1m"
GREEN = "\033[92m"
RED = "\033[91m"
YELLOW = "\033[93m"
RESET = "\033[0m"


def find_project_root() -> Path:
    candidates = [Path("."), Path(".."),
                  Path("MEBP-Version-7.0"), Path("MEBP-Version-7.1"),
                  Path("MEBP-Version-7.2"), Path("MEBP-Version-7.2.3")]
    for c in candidates:
        if (c / "SupportClasses").is_dir() and (c / "gui").is_dir():
            return c.resolve()
    print(f"{RED}ERROR{RESET}: Cannot find project root")
    sys.exit(1)


def main():
    if len(sys.argv) > 1:
        root = Path(sys.argv[1]).resolve()
    else:
        root = find_project_root()

    filepath = root / "gui" / "pages" / "jog_control.py"
    print(f"\n{BOLD}Session 2 Fix — A2: Conversion Factor + Warning Labels{RESET}")
    print(f"File: {filepath}")

    if not filepath.exists():
        print(f"{RED}ERROR{RESET}: File not found!")
        return 1

    content = filepath.read_text(encoding="utf-8")

    # Check if already applied
    if '_lbl_conversion_factor' in content:
        print(f"{YELLOW}SKIP{RESET}: A2 already applied (found _lbl_conversion_factor)")
        return 0

    # Find the Speed Multipliers section dynamically using regex
    # Looking for: "# ── Speed Multipliers" followed by speed_label = QLabel("Speed
    pattern = re.compile(
        r'([ \t]*# ──[─ ]*Speed Multipliers[─ ]*\n'
        r'[ \t]*speed_label = QLabel\("Speed \(Xbox Jog\)"\))',
        re.MULTILINE
    )
    match = pattern.search(content)

    if not match:
        # Try alternative: just find the speed_label line
        pattern2 = re.compile(
            r'([ \t]*speed_label = QLabel\("Speed \(Xbox Jog\)"\))',
            re.MULTILINE
        )
        match = pattern2.search(content)

    if not match:
        print(f"{RED}ERROR{RESET}: Could not find 'Speed (Xbox Jog)' label in context panel")
        # Diagnostic: show what's around "Speed" in the file
        for i, line in enumerate(content.split('\n')):
            if 'Speed' in line and 'Xbox' in line:
                print(f"  Line {i+1}: {line.rstrip()}")
        return 1

    anchor_text = match.group(0)
    print(f"  Found anchor at position {match.start()}")
    print(f"  Anchor: {repr(anchor_text[:80])}...")

    # Build the insertion block (goes BEFORE the speed multipliers section)
    insertion = (
        '        # ── v7.2.4: Conversion Factor Display ──────────────\n'
        '        factor_label = QLabel("Unit Conversion")\n'
        '        factor_label.setObjectName("contextSectionLabel")\n'
        '        layout.addWidget(factor_label)\n'
        '\n'
        '        self._lbl_conversion_factor = QLabel(\n'
        '            f"Scale: {self._microsteps_per_micron:.1f} steps/µm")\n'
        '        self._lbl_conversion_factor.setStyleSheet(\n'
        '            f"color: {COLORS[\'subtext0\']}; font-size: 9pt;")\n'
        '        layout.addWidget(self._lbl_conversion_factor)\n'
        '\n'
        '        self._lbl_factor_warning = QLabel(\n'
        '            "⚠ Using default factor — verify in Settings")\n'
        '        self._lbl_factor_warning.setStyleSheet(\n'
        '            f"color: {COLORS[\'yellow\']}; font-size: 8pt;")\n'
        '        self._lbl_factor_warning.setWordWrap(True)\n'
        '        self._lbl_factor_warning.setVisible(not self._conversion_factor_set)\n'
        '        layout.addWidget(self._lbl_factor_warning)\n'
        '\n'
        '        # ── v7.2.4: Last Jog Step Verification ─────────────\n'
        '        self._lbl_last_jog = QLabel("")\n'
        '        self._lbl_last_jog.setStyleSheet(\n'
        '            f"color: {COLORS[\'subtext0\']}; font-size: 8pt;")\n'
        '        self._lbl_last_jog.setWordWrap(True)\n'
        '        layout.addWidget(self._lbl_last_jog)\n'
        '\n'
    )

    # Insert before the speed multipliers section
    new_content = content[:match.start()] + insertion + content[match.start():]

    filepath.write_text(new_content, encoding="utf-8")
    print(f"  {GREEN}OK{RESET}: Inserted conversion factor display + warning + last jog label")
    print(f"  {GREEN}OK{RESET}: A2 fix applied successfully")
    return 0


if __name__ == "__main__":
    sys.exit(main())

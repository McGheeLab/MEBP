#!/usr/bin/env python3
"""
patch_fix_imports_v731.py
Fix: add QCheckBox and QStackedWidget to the PySide6.QtWidgets import block
in gui/pages/print_well_setup.py so the v7.3.1 WellSetupTab class can find them.
"""
import ast, re, sys, shutil
from pathlib import Path
from datetime import datetime

GREEN  = "\033[92m"; RED = "\033[91m"; YELLOW = "\033[93m"
CYAN   = "\033[96m"; RESET = "\033[0m"

def find_root() -> Path:
    for p in [Path.cwd(), Path(__file__).parent]:
        for c in [p, p.parent, p.parent.parent]:
            if (c / "SupportClasses").is_dir() and (c / "gui").is_dir():
                return c.resolve()
    raise RuntimeError("Cannot locate MEBP project root")

def main():
    root   = find_root()
    target = root / "gui" / "pages" / "print_well_setup.py"
    print(f"{CYAN}Target: {target}{RESET}")

    content = target.read_text(encoding="utf-8")

    changed = False

    # ── 1. Add QCheckBox if missing ─────────────────────────────
    if "QCheckBox" not in content:
        # Insert after QDoubleSpinBox inside the QtWidgets import block
        content = content.replace(
            "    QDoubleSpinBox,",
            "    QDoubleSpinBox, QCheckBox,",
            1
        )
        if "QCheckBox" in content:
            print(f"  {GREEN}✓ Added QCheckBox{RESET}")
            changed = True
        else:
            print(f"  {RED}✗ Could not insert QCheckBox — check import block manually{RESET}")

    # ── 2. Add QStackedWidget if missing ─────────────────────────
    if "QStackedWidget" not in content:
        content = content.replace(
            "    QMenu, QSplitter,",
            "    QMenu, QSplitter, QStackedWidget,",
            1
        )
        if "QStackedWidget" in content:
            print(f"  {GREEN}✓ Added QStackedWidget{RESET}")
            changed = True
        else:
            print(f"  {RED}✗ Could not insert QStackedWidget{RESET}")

    if not changed:
        print(f"  {YELLOW}○ SKIP: all imports already present{RESET}")
        return

    # AST verify
    try:
        ast.parse(content)
    except SyntaxError as e:
        print(f"  {RED}✗ AST FAIL — not writing: {e}{RESET}")
        sys.exit(1)

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    shutil.copy2(target, target.with_suffix(f".bak_v731fix_{ts}"))
    target.write_text(content, encoding="utf-8")
    print(f"  {GREEN}✓ Written — imports fixed{RESET}")

if __name__ == "__main__":
    main()

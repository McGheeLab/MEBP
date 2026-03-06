#!/usr/bin/env python3
"""
Hotfix: Add _get_controller() to PrintMonitorPage.

The Session 2 patch failed to inject this helper because the __init__
regex didn't match the actual file. This script uses a more robust
insertion strategy.
"""

import ast, sys, shutil, re
from pathlib import Path
from datetime import datetime

G = "\033[92m"; R = "\033[91m"; Y = "\033[93m"; X = "\033[0m"

def find_root():
    if len(sys.argv) > 1:
        p = Path(sys.argv[1])
        if (p / "gui").is_dir():
            return p
    for c in [Path.cwd(), Path(__file__).resolve().parent.parent]:
        if (c / "gui").is_dir():
            return c
    print(f"{R}Cannot find MEBP root{X}"); sys.exit(1)

def main():
    root = find_root()
    path = root / "gui" / "pages" / "print_monitor.py"
    content = path.read_text(encoding="utf-8")

    if "def _get_controller" in content:
        print(f"  {Y}○ SKIP: _get_controller already exists{X}")
        sys.exit(0)

    # Strategy: insert right before the first "def get_page_title" or
    # right before "def get_context_widget" — these are stable anchors
    inserted = False
    for anchor in ["def get_page_title", "def get_context_widget", "def set_hardware_config"]:
        m = re.search(rf'^(    {anchor}\(self)', content, re.MULTILINE)
        if m:
            helper = '''    def _get_controller(self):
        """v7.2.6: Helper to get controller reference."""
        return getattr(self, '_controller', None) or getattr(self, 'controller', None)

'''
            content = content[:m.start()] + helper + content[m.start():]
            inserted = True
            break

    if not inserted:
        print(f"  {R}✗ Could not find insertion point{X}")
        sys.exit(1)

    try:
        ast.parse(content)
    except SyntaxError as e:
        print(f"  {R}AST FAIL: {e}{X}"); sys.exit(1)

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    shutil.copy2(path, path.with_suffix(f".bak_hotfix_{ts}"))
    path.write_text(content, encoding="utf-8")
    print(f"  {G}✓ Added _get_controller() to print_monitor.py{X}")

if __name__ == "__main__":
    main()

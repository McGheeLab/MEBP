#!/usr/bin/env python3
"""
patch_v726_s1_future_fix.py — Fix XYStage.py + ZPStage.py
S1 injected 'import threading' before 'from __future__ import annotations'.
Python requires __future__ imports to be the very first statement.
This script moves the threading import to after the __future__ line.
"""
import ast, sys, shutil
from pathlib import Path
from datetime import datetime

GREEN = "\033[92m"; YELLOW = "\033[93m"; RED = "\033[91m"; RESET = "\033[0m"
OK = f"{GREEN}  v{RESET}"; SKIP = f"{YELLOW}  o{RESET}"; MISS = f"{RED}  x{RESET}"
applied = 0; failed = 0

def find_root():
    here = Path(__file__).resolve().parent
    for d in [here, here.parent, here.parent.parent]:
        if (d / "SupportClasses").is_dir() and (d / "gui").is_dir():
            return d
    sys.exit(f"{RED}ERROR: Cannot find MEBP project root{RESET}")

def fix_file(path):
    global applied, failed
    content = path.read_text(encoding="utf-8")

    # Check if broken: import threading appears before from __future__
    ti = content.find("import threading")
    fi = content.find("from __future__ import annotations")
    if ti == -1:
        print(f"{SKIP} No 'import threading' in {path.name}")
        return
    if fi == -1:
        print(f"{SKIP} No __future__ import in {path.name}")
        return
    if ti > fi:
        print(f"{SKIP} {path.name} already has correct import order")
        return

    # Remove the misplaced 'import threading\n' before __future__
    content = content.replace("import threading\n", "", 1)

    # Insert 'import threading' on its own line after the __future__ line
    future_line = "from __future__ import annotations"
    content = content.replace(
        future_line,
        future_line + "\nimport threading  # v7.2.6: serial lock (moved after __future__)",
        1
    )

    # Verify
    try:
        ast.parse(content)
    except SyntaxError as e:
        print(f"{MISS} AST FAIL in {path.name}: {e}")
        failed += 1
        return

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    shutil.copy2(path, path.with_suffix(f".bak_v726_{ts}"))
    path.write_text(content, encoding="utf-8")
    applied += 1
    print(f"{OK} Fixed import order in {path.name}")

ROOT = find_root()
print("\n=== Fixing __future__ import order in XYStage.py and ZPStage.py ===")
fix_file(ROOT / "SupportClasses" / "XYStage.py")
fix_file(ROOT / "SupportClasses" / "ZPStage.py")

print(f"\n{'='*50}")
print(f"Results: {GREEN}{applied} fixed{RESET}  {RED}{failed} failed{RESET}")
if failed:
    sys.exit(1)

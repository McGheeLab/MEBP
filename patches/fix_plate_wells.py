#!/usr/bin/env python3
"""Fix: plate.wells → plate._wells in print_monitor.py.

WellPlate stores wells as _wells (private dict). The monitor's
mini_plate.set_plate() accesses plate.wells which doesn't exist.
"""

import ast, sys, shutil
from pathlib import Path
from datetime import datetime

G = "\033[92m"; R = "\033[91m"; X = "\033[0m"

def find_root():
    if len(sys.argv) > 1:
        p = Path(sys.argv[1])
        if (p / "gui").is_dir(): return p
    for c in [Path.cwd(), Path(__file__).resolve().parent.parent]:
        if (c / "gui").is_dir(): return c
    print(f"{R}Cannot find MEBP root{X}"); sys.exit(1)

def main():
    root = find_root()
    path = root / "gui" / "pages" / "print_monitor.py"
    content = path.read_text(encoding="utf-8")

    old = "plate.wells"
    new = "plate._wells"

    count = content.count(old)
    # Don't double-fix: plate._wells._wells
    already_fixed = content.count("plate._wells")

    if count == 0 and already_fixed > 0:
        print(f"  {G}Already fixed ({already_fixed} occurrences of plate._wells){X}")
        return

    if count == 0:
        print(f"  No 'plate.wells' found in file")
        return

    content = content.replace(old, new)
    print(f"  Replaced {count} occurrence(s) of plate.wells → plate._wells")

    try:
        ast.parse(content)
    except SyntaxError as e:
        print(f"  {R}AST FAIL: {e}{X}"); sys.exit(1)

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    shutil.copy2(path, path.with_suffix(f".bak_wells_{ts}"))
    path.write_text(content, encoding="utf-8")
    print(f"  {G}✓ Done. Run: python main.py{X}")

if __name__ == "__main__":
    main()

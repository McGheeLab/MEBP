#!/usr/bin/env python3
"""Fix: WellInfo.x_mm → WellInfo.x and WellInfo.y_mm → WellInfo.y in print_monitor.py."""

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

    replacements = [
        ("info.x_mm", "info.x"),
        ("info.y_mm", "info.y"),
        ("well.x_mm", "well.x"),
        ("well.y_mm", "well.y"),
    ]

    count = 0
    for old, new in replacements:
        n = content.count(old)
        if n > 0:
            content = content.replace(old, new)
            count += n
            print(f"  {G}✓{X} {old} → {new} ({n}x)")

    if count == 0:
        print(f"  No x_mm/y_mm references found — already fixed")
        return

    try:
        ast.parse(content)
    except SyntaxError as e:
        print(f"  {R}AST FAIL: {e}{X}"); sys.exit(1)

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    shutil.copy2(path, path.with_suffix(f".bak_xyfix_{ts}"))
    path.write_text(content, encoding="utf-8")
    print(f"  {G}✓ Fixed {count} attribute(s). Run: python main.py{X}")

if __name__ == "__main__":
    main()

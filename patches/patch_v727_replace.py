#!/usr/bin/env python3
"""
MEBP v7.2.7 — Replace ImagePathPlanner + helper_functions with v2.

Replaces both files with the new versions that support:
  - Mask-based rastering (only within black pixel edges)
  - Proportional pump flow from greyscale
  - Multi-layer with 3 input modes (TIFF stack / sequence / single×N)
  - Corner slowdown + layer alternation + closest-start reorder

This script simply overwrites the two target files.
No app.py changes needed (already patched by previous scripts).
"""

import ast
import shutil
import sys
from pathlib import Path
from datetime import datetime

GREEN = "\033[92m"
RED = "\033[91m"
YELLOW = "\033[93m"
CYAN = "\033[96m"
RESET = "\033[0m"


def find_root() -> Path:
    candidates = [
        Path(__file__).resolve().parent.parent.parent,
        Path(__file__).resolve().parent.parent,
        Path(__file__).resolve().parent,
        Path.cwd(),
    ]
    for p in candidates:
        if (p / "SupportClasses").is_dir() and (p / "gui").is_dir():
            return p
    print(f"{RED}ERROR: Cannot find MEBP project root{RESET}")
    sys.exit(1)


def replace_file(src: Path, dst: Path, label: str) -> bool:
    if not src.exists():
        print(f"  {RED}✗ MISS:{RESET} Source not found: {src}")
        return False

    # AST verify source
    content = src.read_text(encoding="utf-8")
    try:
        ast.parse(content)
    except SyntaxError as e:
        print(f"  {RED}✗ AST FAIL:{RESET} {label}: {e}")
        return False

    # Write
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(src, dst)
    print(f"  {GREEN}✓ OK:{RESET} {label} → {dst}")
    return True


def main():
    print(f"\n{'='*60}")
    print(f" MEBP v7.2.7 — Replace with mask-based v2 files")
    print(f" {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print(f"{'='*60}")

    root = find_root()
    script_dir = Path(__file__).resolve().parent
    print(f"Project root: {root}")
    print(f"Script dir:   {script_dir}")

    ok = 0
    fail = 0

    print(f"\n{CYAN}Replacing files:{RESET}")

    if replace_file(
        script_dir / "ImagePathPlanner.py",
        root / "SupportClasses" / "ImagePathPlanner.py",
        "SupportClasses/ImagePathPlanner.py",
    ):
        ok += 1
    else:
        fail += 1

    if replace_file(
        script_dir / "helper_functions.py",
        root / "gui" / "pages" / "helper_functions.py",
        "gui/pages/helper_functions.py",
    ):
        ok += 1
    else:
        fail += 1

    print(f"\n{'='*60}")
    if fail == 0:
        print(f" {GREEN}✓ All {ok} files replaced successfully{RESET}")
    else:
        print(f" {RED}⚠ {fail} replacement(s) failed{RESET}")
    print(f"{'='*60}\n")

    return 0 if fail == 0 else 1


if __name__ == "__main__":
    sys.exit(main())

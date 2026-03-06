#!/usr/bin/env python3
"""
patch_fix_calibration_format_key.py
Fix: WellPlate has .format not .format_key.
Replace all occurrences of .format_key with .format in calibration.py.
"""
import ast, sys, shutil
from pathlib import Path
from datetime import datetime

GREEN = "\033[92m"; RED = "\033[91m"; YELLOW = "\033[93m"
CYAN  = "\033[96m"; RESET = "\033[0m"

def find_root() -> Path:
    for p in [Path.cwd(), Path(__file__).parent]:
        for c in [p, p.parent, p.parent.parent]:
            if (c / "SupportClasses").is_dir() and (c / "gui").is_dir():
                return c.resolve()
    raise RuntimeError("Cannot locate MEBP project root")

def main():
    root   = find_root()
    target = root / "gui" / "pages" / "calibration.py"
    print(f"{CYAN}Target: {target}{RESET}")

    if not target.exists():
        print(f"  {RED}✗ File not found{RESET}")
        sys.exit(1)

    content = target.read_text(encoding="utf-8")

    if ".format_key" not in content:
        print(f"  {YELLOW}○ SKIP: no .format_key found{RESET}")
        return

    count = content.count(".format_key")
    new_content = content.replace(".format_key", ".format")

    try:
        ast.parse(new_content)
    except SyntaxError as e:
        print(f"  {RED}✗ AST FAIL: {e}{RESET}")
        sys.exit(1)

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    shutil.copy2(target, target.with_suffix(f".bak_v731cal_{ts}"))
    target.write_text(new_content, encoding="utf-8")
    print(f"  {GREEN}✓ Replaced {count} occurrence(s) of .format_key → .format{RESET}")

if __name__ == "__main__":
    main()

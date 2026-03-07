#!/usr/bin/env python3
"""Fix calibration.py position display — indentation-aware replacement."""

import ast, re, sys, shutil
from pathlib import Path
from datetime import datetime

GREEN, RED, YELLOW, CYAN, RESET, BOLD = "\033[92m", "\033[91m", "\033[93m", "\033[96m", "\033[0m", "\033[1m"

def find_root():
    for c in [Path(__file__).resolve().parent.parent.parent, Path.cwd(),
              Path.home() / "Documents" / "GitHub" / "MEBP"]:
        if (c / "SupportClasses").is_dir() and (c / "gui").is_dir():
            return c
    sys.exit(f"{RED}Cannot find MEBP root{RESET}")

def main():
    print(f"\n{BOLD}  MEBP v7.2.7: Calibration Position Fix{RESET}")
    root = find_root()
    path = root / "gui" / "pages" / "calibration.py"
    content = path.read_text(encoding="utf-8")

    marker = "v7.2.7: controller reports µm directly"
    if marker in content:
        print(f"  {YELLOW}○ Already patched{RESET}")
        return 0

    changed = False

    # ── Fix 1: on_status_update XY display ────────────────────────
    # Match the 4-line block and capture its indentation from the first line
    pat1 = re.compile(
        r'^(\s+)zx = xy\[0\] - ctrl\.zero_position\["x"\]\s*\n'
        r'\s+zy = xy\[1\] - ctrl\.zero_position\["y"\]\s*\n'
        r'\s+ux = steps_to_um\(zx, self\._microsteps_per_micron\)\s*\n'
        r'\s+uy = steps_to_um\(zy, self\._microsteps_per_micron\)\s*\n',
        re.MULTILINE
    )
    m1 = pat1.search(content)
    if m1:
        ind = m1.group(1)
        repl = (
            f"{ind}# {marker} — no conversion needed\n"
            f"{ind}ux = xy[0] - ctrl.zero_position[\"x\"]\n"
            f"{ind}uy = xy[1] - ctrl.zero_position[\"y\"]\n"
        )
        content = content[:m1.start()] + repl + content[m1.end():]
        changed = True
        print(f"  {GREEN}✓ Fixed on_status_update XY display{RESET}")
    else:
        print(f"  {RED}✗ Could not find steps_to_um in on_status_update{RESET}")

    # ── Fix 2: _set_zero display ──────────────────────────────────
    # Match: zx_um = steps_to_um(z['x'], self._microsteps_per_micron)
    #        zy_um = steps_to_um(z['y'], self._microsteps_per_micron)
    pat2 = re.compile(
        r'^(\s+)zx_um = steps_to_um\(z\[.x.\], self\._microsteps_per_micron\)\s*\n'
        r'\s+zy_um = steps_to_um\(z\[.y.\], self\._microsteps_per_micron\)',
        re.MULTILINE
    )
    m2 = pat2.search(content)
    if m2:
        ind2 = m2.group(1)
        repl2 = (
            f"{ind2}# v7.2.7: zero positions already in µm\n"
            f"{ind2}zx_um = z['x']\n"
            f"{ind2}zy_um = z['y']"
        )
        content = content[:m2.start()] + repl2 + content[m2.end():]
        changed = True
        print(f"  {GREEN}✓ Fixed _set_zero display{RESET}")
    else:
        print(f"  {YELLOW}○ _set_zero pattern not found (may already be fixed){RESET}")

    if not changed:
        print(f"  {YELLOW}No changes to apply{RESET}")
        return 0

    # AST verify
    try:
        ast.parse(content)
    except SyntaxError as e:
        print(f"  {RED}✗ AST FAIL: line {e.lineno}: {e.msg}{RESET}")
        lines = content.split('\n')
        for i in range(max(0, e.lineno-4), min(len(lines), e.lineno+3)):
            m = ">>>" if i == e.lineno-1 else "   "
            print(f"    {m} {i+1:4d} | {lines[i]}")
        return 1

    # Backup + write
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    shutil.copy2(path, path.with_suffix(f".bak_v727cal_{ts}"))
    path.write_text(content, encoding="utf-8")

    print(f"\n  {GREEN}✓ calibration.py patched and AST verified{RESET}")
    return 0

if __name__ == "__main__":
    sys.exit(main())

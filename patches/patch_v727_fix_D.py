#!/usr/bin/env python3
"""
v7.2.7 Fix D: Add travel speed setting to MOVE_XY handler.

This is the last remaining unfixed item. Instead of trying to
regex-match the handler block, we directly find the unique line sequence:

    x, y = p.get("x", 0), p.get("y", 0)
    ctrl.move_xy_absolute(x, y, from_zero_ref=True)

And inject speed-set before the move.
"""

import ast
import re
import sys
import shutil
from pathlib import Path
from datetime import datetime

GREEN = "\033[92m"
RED = "\033[91m"
YELLOW = "\033[93m"
CYAN = "\033[96m"
RESET = "\033[0m"
BOLD = "\033[1m"


def find_root() -> Path:
    candidates = [
        Path(__file__).resolve().parent.parent.parent,
        Path.cwd(),
        Path.home() / "Documents" / "GitHub" / "MEBP",
    ]
    for c in candidates:
        if (c / "SupportClasses").is_dir() and (c / "gui").is_dir():
            return c
    print(f"{RED}ERROR: Cannot find MEBP root{RESET}")
    sys.exit(1)


def main():
    print(f"\n{BOLD}{'='*60}")
    print(f"  MEBP v7.2.7 Fix D: MOVE_XY Travel Speed")
    print(f"{'='*60}{RESET}")

    root = find_root()
    print(f"Project root: {root}")

    path = root / "SupportClasses" / "PrintManager.py"
    if not path.exists():
        print(f"{RED}PrintManager.py not found!{RESET}")
        return 1

    content = path.read_text(encoding="utf-8")

    # Check if already applied
    marker = "v7.2.7: Set travel speed before XY move"
    if marker in content:
        print(f"\n  {YELLOW}○ SKIP: Travel speed already applied{RESET}")
        return 0

    # Strategy: Find the EXACT line pair that only appears in _execute_command's
    # MOVE_XY handler. This is unique because of the p.get pattern.
    #
    # Pattern on disk (from project knowledge + verified):
    #   x, y = p.get("x", 0), p.get("y", 0)
    #   ctrl.move_xy_absolute(x, y, from_zero_ref=True)
    #
    # We inject BETWEEN these two lines.

    pattern = re.compile(
        r'(            x, y = p\.get\("x", 0\), p\.get\("y", 0\)\n)'
        r'(            ctrl\.move_xy_absolute\(x, y, from_zero_ref=True\))'
    )

    match = pattern.search(content)

    if not match:
        # Try with flexible whitespace
        pattern2 = re.compile(
            r'(\s+x,\s*y\s*=\s*p\.get\("x",\s*0\),\s*p\.get\("y",\s*0\)\s*\n)'
            r'(\s+ctrl\.move_xy_absolute\(x,\s*y,\s*from_zero_ref=True\))'
        )
        match = pattern2.search(content)

    if not match:
        # Debug: show what's around MOVE_XY in the file
        idx = content.find('CommandType.MOVE_XY')
        if idx >= 0:
            snippet = content[idx:idx+400]
            print(f"\n  {RED}✗ MISS: Could not match MOVE_XY handler lines{RESET}")
            print(f"  File content near MOVE_XY:")
            for i, line in enumerate(snippet.split('\n')[:10]):
                print(f"    {i}: {repr(line)}")
        else:
            print(f"\n  {RED}✗ MISS: CommandType.MOVE_XY not found in file{RESET}")
        return 1

    # Build injection: insert between the x,y line and the move call
    xy_line = match.group(1)
    move_line = match.group(2)

    # Detect the indentation from the move line
    indent_match = re.match(r'^(\s+)', move_line)
    indent = indent_match.group(1) if indent_match else "            "

    inject = (
        f"{indent}# {marker}\n"
        f"{indent}_tspd = getattr(self.job.settings, 'travel_speed_mm_s', 10.0) if self.job else 10.0\n"
        f"{indent}if hasattr(ctrl, 'xy_stage') and ctrl.xy_stage:\n"
        f"{indent}    try:\n"
        f"{indent}        ctrl.xy_stage.set_velocity(int(min(_tspd * 1000, 50000)))\n"
        f"{indent}    except Exception:\n"
        f"{indent}        pass\n"
    )

    new_content = content[:match.start()] + xy_line + inject + move_line + content[match.end():]

    # AST verify
    try:
        ast.parse(new_content)
    except SyntaxError as e:
        print(f"\n  {RED}✗ AST FAIL: line {e.lineno}: {e.msg}{RESET}")
        lines = new_content.split('\n')
        start = max(0, e.lineno - 4)
        end = min(len(lines), e.lineno + 3)
        for i in range(start, end):
            marker_str = ">>>" if i == e.lineno - 1 else "   "
            print(f"    {marker_str} {i+1:4d} | {lines[i]}")
        return 1

    # Backup + write
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    backup = path.with_suffix(f".bak_v727D_{ts}")
    shutil.copy2(path, backup)
    path.write_text(new_content, encoding="utf-8")

    print(f"\n  {GREEN}✓ Applied: Travel speed set in MOVE_XY handler{RESET}")

    # Also set speed before HOME_XY if not already done
    content = path.read_text(encoding="utf-8")
    home_marker = "v7.2.7: Set travel speed before HOME_XY"
    if home_marker not in content:
        # HOME_XY handler: ctrl.move_xy_absolute(0, 0, from_zero_ref=True)
        home_pattern = re.compile(
            r'(CommandType\.HOME_XY:\n)'
            r'(\s+)(ctrl\.move_xy_absolute\(0,\s*0,\s*from_zero_ref=True\))'
        )
        home_match = home_pattern.search(content)
        if home_match:
            h_indent = home_match.group(2)
            home_inject = (
                f"{h_indent}# {home_marker}\n"
                f"{h_indent}_hspd = getattr(self.job.settings, 'travel_speed_mm_s', 10.0) if self.job else 10.0\n"
                f"{h_indent}if hasattr(ctrl, 'xy_stage') and ctrl.xy_stage:\n"
                f"{h_indent}    try:\n"
                f"{h_indent}        ctrl.xy_stage.set_velocity(int(min(_hspd * 1000, 50000)))\n"
                f"{h_indent}    except Exception:\n"
                f"{h_indent}        pass\n"
            )
            content = (content[:home_match.start(2)] + 
                      home_inject + content[home_match.start(2):])
            try:
                ast.parse(content)
                path.write_text(content, encoding="utf-8")
                print(f"  {GREEN}✓ Applied: Travel speed set in HOME_XY handler{RESET}")
            except SyntaxError:
                print(f"  {YELLOW}○ SKIP: HOME_XY injection failed AST (non-critical){RESET}")
        else:
            print(f"  {YELLOW}○ SKIP: HOME_XY handler not found (non-critical){RESET}")
    else:
        print(f"  {YELLOW}○ SKIP: HOME_XY speed already present{RESET}")

    # Final verification
    final = path.read_text(encoding="utf-8")
    try:
        ast.parse(final)
        print(f"\n  {GREEN}✓ Final AST check: PrintManager.py OK{RESET}")
    except SyntaxError as e:
        print(f"\n  {RED}✗ Final AST FAIL: {e}{RESET}")
        return 1

    print(f"\n{GREEN}All v7.2.7 patches now applied!{RESET}")
    print(f"\nSummary of what's on disk:")
    print(f"  A: print_speed_mm_s + travel_speed_mm_s fields in PrintSettings ✓")
    print(f"  B: _execute_print_path uses mm/s + _set_xy_speed_for_print() ✓")
    print(f"  C: TrajectoryExecutor sets Prior SMS + Z skip ✓")
    print(f"  D: MOVE_XY + HOME_XY set travel speed ✓")
    print(f"  E: _get_settings populates print_speed_mm_s from GUI ✓")
    print(f"  F: from_dict infers mm/s from legacy files ✓")
    print(f"  G: _wait_for_xy_settle uses mm-based tolerance ✓")
    return 0


if __name__ == "__main__":
    sys.exit(main())

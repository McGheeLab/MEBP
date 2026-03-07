#!/usr/bin/env python3
"""Fix TrajectoryPlanner: add xy_travel_speed + pass to create_travel_move."""

import ast, re, sys, shutil
from pathlib import Path
from datetime import datetime

GREEN, RED, YELLOW, CYAN, RESET, BOLD = (
    "\033[92m", "\033[91m", "\033[93m", "\033[96m", "\033[0m", "\033[1m")

def find_root():
    for c in [Path(__file__).resolve().parent.parent.parent, Path.cwd(),
              Path.home() / "Documents" / "GitHub" / "MEBP"]:
        if (c / "SupportClasses").is_dir() and (c / "gui").is_dir(): return c
    sys.exit("Cannot find MEBP root")

def main():
    print(f"\n{BOLD}  TrajectoryPlanner travel speed fix{RESET}")
    root = find_root()
    path = root / "SupportClasses" / "TrajectoryPlanner.py"
    content = path.read_text(encoding="utf-8")

    # ── Fix 1: Add xy_travel_speed variable after z_speed ────────
    print(f"\n{CYAN}[1] Add xy_travel_speed variable{RESET}")
    marker = "xy_travel_speed"
    if marker in content:
        print(f"  {YELLOW}○ Already present{RESET}")
    else:
        # Find: z_speed = self._settings.get("z_feed_rate_mm_s", 5.0)
        # (our earlier patch raised the default from 2.0 to 5.0)
        z_pat = re.compile(
            r'^(\s+)(z_speed = self\._settings\.get\("z_feed_rate_mm_s",\s*[\d.]+\))\s*$',
            re.MULTILINE
        )
        m = z_pat.search(content)
        if m:
            ind = m.group(1)
            inject = (
                f'\n{ind}# v7.2.7: XY travel speed from workspace settings\n'
                f'{ind}xy_travel_speed = self._settings.get("travel_speed_mm_s",\n'
                f'{ind}                    self._settings.get("xy_travel_speed_mm_s", 50.0))\n'
            )
            content = content[:m.end()] + inject + content[m.end():]
            print(f"  {GREEN}✓ Added xy_travel_speed after z_speed{RESET}")
        else:
            print(f"  {RED}✗ Could not find z_speed line{RESET}")
            return 1

    # ── Fix 2: Pass xy_speed_mm_s to create_travel_move ──────────
    print(f"\n{CYAN}[2] Pass xy_speed to create_travel_move{RESET}")
    if "xy_speed_mm_s=xy_travel_speed" in content:
        print(f"  {YELLOW}○ Already passed{RESET}")
    else:
        # Find the exact call pattern:
        #   z_speed_mm_s=z_speed,
        #   pump_positions=tuple(p_start),
        # Insert xy_speed_mm_s=xy_travel_speed between them
        call_pat = re.compile(
            r'(z_speed_mm_s=z_speed,\n)'
            r'(\s+)(pump_positions=tuple\(p_start\),)'
        )
        m2 = call_pat.search(content)
        if m2:
            replacement = (
                f'{m2.group(1)}'
                f'{m2.group(2)}xy_speed_mm_s=xy_travel_speed,\n'
                f'{m2.group(2)}{m2.group(3)}'
            )
            content = content[:m2.start()] + replacement + content[m2.end():]
            print(f"  {GREEN}✓ Added xy_speed_mm_s parameter{RESET}")
        else:
            print(f"  {RED}✗ Could not find create_travel_move call{RESET}")
            return 1

    # AST + write
    try:
        ast.parse(content)
    except SyntaxError as e:
        print(f"\n  {RED}AST FAIL: line {e.lineno}: {e.msg}{RESET}")
        lines = content.split('\n')
        for i in range(max(0,e.lineno-4), min(len(lines),e.lineno+3)):
            mk = ">>>" if i==e.lineno-1 else "   "
            print(f"    {mk} {i+1:4d} | {lines[i]}")
        return 1

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    shutil.copy2(path, path.with_suffix(f".bak_v727ttf_{ts}"))
    path.write_text(content, encoding="utf-8")
    print(f"\n  {GREEN}✓ TrajectoryPlanner.py written + AST OK{RESET}")
    return 0

if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""Fix PrintManager speed callers — the SMS patch's regex indentation was wrong."""

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

def find_method(content, name):
    pat = re.compile(
        rf'^(    def {re.escape(name)}\(self.*?\n)(.*?)(?=\n    def |\nclass |\Z)',
        re.DOTALL | re.MULTILINE)
    return pat.search(content)

def main():
    print(f"\n{BOLD}  PrintManager speed callers fix{RESET}")
    root = find_root()
    path = root / "SupportClasses" / "PrintManager.py"
    content = path.read_text(encoding="utf-8")
    changed = False

    # ── Fix 1: _set_xy_speed_for_print ───────────────────────────
    print(f"\n{CYAN}[1] _set_xy_speed_for_print{RESET}")
    marker1 = "v7.2.7: use set_speed_mm_s"
    m = find_method(content, "_set_xy_speed_for_print")
    if m and marker1 in m.group(0):
        print(f"  {YELLOW}○ Already has v7.2.7 marker{RESET}")
    elif m:
        new = '''    def _set_xy_speed_for_print(self, speed_mm_s: float = 0):
        """v7.2.7: use set_speed_mm_s — Set Prior XY stage speed before print."""
        settings = self.job.settings if self.job else None
        if speed_mm_s <= 0 and settings:
            speed_mm_s = getattr(settings, 'print_speed_mm_s', 0)
        if speed_mm_s <= 0 and settings:
            speed_mm_s = max(getattr(settings, 'print_feedrate', 200), 1) / 60.0
        if speed_mm_s <= 0:
            speed_mm_s = 5.0
        ctrl = self.controller
        if hasattr(ctrl, 'xy_stage') and ctrl.xy_stage:
            if hasattr(ctrl.xy_stage, 'set_speed_mm_s'):
                ctrl.xy_stage.set_speed_mm_s(speed_mm_s)
            else:
                pct = max(1, min(100, int(speed_mm_s * 1000 / 50000 * 100)))
                ctrl.xy_stage.set_velocity(pct)
            logger.info(f"Print speed set: {speed_mm_s:.1f} mm/s")

'''
        content = content[:m.start()] + new + content[m.end():]
        changed = True
        print(f"  {GREEN}✓ Replaced{RESET}")
    else:
        print(f"  {RED}✗ Not found{RESET}")

    # ── Fix 2: TrajectoryExecutor set_velocity → set_speed_mm_s ──
    print(f"\n{CYAN}[2] TrajectoryExecutor speed caller{RESET}")
    # Find the exact block: _sms_val = ...\n  ctrl.xy_stage.set_velocity(_sms_val)
    traj_pat = re.compile(
        r'^(\s+)(_sms_val = int\(min\(_max_spd \* 1\.5 \* 1000\.0, 50000\)\)\n)'
        r'(\s+)(ctrl\.xy_stage\.set_velocity\(_sms_val\))',
        re.MULTILINE
    )
    m2 = traj_pat.search(content)
    if m2:
        ind = m2.group(1)  # indentation of _sms_val line
        ind2 = m2.group(3)  # indentation of set_velocity line
        replacement = (
            f'{ind}# v7.2.7: use set_speed_mm_s for trajectory\n'
            f'{ind}if hasattr(ctrl.xy_stage, "set_speed_mm_s"):\n'
            f'{ind}    ctrl.xy_stage.set_speed_mm_s(_max_spd * 1.5)\n'
            f'{ind}else:\n'
            f'{ind}    _sms_val = int(min(_max_spd * 1.5 * 1000.0, 50000))\n'
            f'{ind}    ctrl.xy_stage.set_velocity(_sms_val)'
        )
        content = content[:m2.start()] + replacement + content[m2.end():]
        changed = True
        print(f"  {GREEN}✓ Fixed{RESET}")
    else:
        # Check if already patched
        if "set_speed_mm_s(_max_spd" in content:
            print(f"  {YELLOW}○ Already patched{RESET}")
        else:
            print(f"  {RED}✗ Pattern not found{RESET}")

    # ── Fix 3: MOVE_XY travel speed ──────────────────────────────
    print(f"\n{CYAN}[3] MOVE_XY travel speed{RESET}")
    # Find: ctrl.xy_stage.set_velocity(int(min(_tspd * 1000, 50000)))
    move_pat = re.compile(
        r'(\s+)ctrl\.xy_stage\.set_velocity\(int\(min\(_tspd \* 1000, 50000\)\)\)'
    )
    m3 = move_pat.search(content)
    if m3:
        ind = m3.group(1)
        replacement = (
            f'{ind}if hasattr(ctrl.xy_stage, "set_speed_mm_s"):\n'
            f'{ind}    ctrl.xy_stage.set_speed_mm_s(_tspd)\n'
            f'{ind}else:\n'
            f'{ind}    ctrl.xy_stage.set_velocity(max(1, min(100, int(_tspd * 1000 / 50000 * 100))))'
        )
        content = content[:m3.start()] + replacement + content[m3.end():]
        changed = True
        print(f"  {GREEN}✓ Fixed{RESET}")
    else:
        if "set_speed_mm_s(_tspd)" in content:
            print(f"  {YELLOW}○ Already patched{RESET}")
        else:
            print(f"  {RED}✗ Pattern not found{RESET}")

    # ── Fix 4: HOME_XY travel speed ──────────────────────────────
    print(f"\n{CYAN}[4] HOME_XY travel speed{RESET}")
    home_pat = re.compile(
        r'(\s+)ctrl\.xy_stage\.set_velocity\(int\(min\(_hspd \* 1000, 50000\)\)\)'
    )
    m4 = home_pat.search(content)
    if m4:
        ind = m4.group(1)
        replacement = (
            f'{ind}if hasattr(ctrl.xy_stage, "set_speed_mm_s"):\n'
            f'{ind}    ctrl.xy_stage.set_speed_mm_s(_hspd)\n'
            f'{ind}else:\n'
            f'{ind}    ctrl.xy_stage.set_velocity(max(1, min(100, int(_hspd * 1000 / 50000 * 100))))'
        )
        content = content[:m4.start()] + replacement + content[m4.end():]
        changed = True
        print(f"  {GREEN}✓ Fixed{RESET}")
    else:
        if "set_speed_mm_s(_hspd)" in content:
            print(f"  {YELLOW}○ Already patched{RESET}")
        else:
            print(f"  {RED}✗ Pattern not found{RESET}")

    if not changed:
        print(f"\n  {YELLOW}No changes needed{RESET}")
        return 0

    # AST + write
    try:
        ast.parse(content)
    except SyntaxError as e:
        print(f"\n  {RED}AST FAIL: line {e.lineno}: {e.msg}{RESET}")
        lines = content.split('\n')
        for i in range(max(0,e.lineno-5), min(len(lines),e.lineno+3)):
            mk = ">>>" if i==e.lineno-1 else "   "
            print(f"    {mk} {i+1:4d} | {lines[i]}")
        return 1

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    shutil.copy2(path, path.with_suffix(f".bak_v727pmc_{ts}"))
    path.write_text(content, encoding="utf-8")
    print(f"\n  {GREEN}✓ PrintManager.py written + AST OK{RESET}")
    return 0

if __name__ == "__main__":
    sys.exit(main())

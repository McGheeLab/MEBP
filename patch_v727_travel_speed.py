#!/usr/bin/env python3
"""
v7.2.7: Fix travel speed in trajectory planning.

Root cause: 
  1. plan_well_print reads z_speed from workspace (default 2mm/s) — very slow
  2. plan_well_print never passes xy_speed to create_travel_move (defaults 20mm/s)
  3. GUI travel_speed_mm_s and z_feedrate never reach workspace.print_settings
  4. plan_to_trajectory in PrintTrajectoryPlanner also uses workspace defaults

Fix:
  1. TrajectoryPlanner.plan_well_print: read travel speeds from workspace, 
     pass xy_speed to create_travel_move
  2. print_setup._generate_print: push GUI speeds to workspace before planning
  3. Higher defaults: z_speed 5mm/s, xy_travel 50mm/s
"""

import ast, re, sys, shutil
from pathlib import Path
from datetime import datetime

GREEN, RED, YELLOW, CYAN, RESET, BOLD = (
    "\033[92m", "\033[91m", "\033[93m", "\033[96m", "\033[0m", "\033[1m")
ok_count = skip_count = miss_count = 0

def find_root():
    for c in [Path(__file__).resolve().parent.parent.parent, Path.cwd(),
              Path.home() / "Documents" / "GitHub" / "MEBP"]:
        if (c / "SupportClasses").is_dir() and (c / "gui").is_dir(): return c
    sys.exit(f"{RED}Cannot find MEBP root{RESET}")

def report(s, msg):
    global ok_count, skip_count, miss_count
    if s=="OK": ok_count+=1; print(f"  {GREEN}✓ {msg}{RESET}")
    elif s=="SKIP": skip_count+=1; print(f"  {YELLOW}○ {msg}{RESET}")
    else: miss_count+=1; print(f"  {RED}✗ {msg}{RESET}")

def find_method(content, name):
    pat = re.compile(
        rf'^(    def {re.escape(name)}\(self.*?\n)(.*?)(?=\n    def |\nclass |\Z)',
        re.DOTALL | re.MULTILINE)
    return pat.search(content)


def patch_trajectory_planner(root):
    """Fix plan_well_print to pass travel speeds from workspace settings."""
    print(f"\n{CYAN}[1] TrajectoryPlanner.py — travel speed in plan_well_print{RESET}")
    path = root / "SupportClasses" / "TrajectoryPlanner.py"
    if not path.exists():
        report("MISS", "TrajectoryPlanner.py not found")
        return
    content = path.read_text(encoding="utf-8")

    marker = "v7.2.7: travel speed from settings"
    if marker in content:
        report("SKIP", "Already patched")
        return

    # Find where plan_well_print reads z_speed from settings
    # Pattern: z_speed = self._settings.get("z_feed_rate_mm_s", 2.0)
    z_speed_pat = re.compile(
        r'^(\s+)(z_speed = self\._settings\.get\("z_feed_rate_mm_s",\s*)([\d.]+)(\))',
        re.MULTILINE
    )
    m = z_speed_pat.search(content)
    if m:
        ind = m.group(1)
        # Replace with higher default and add xy_travel_speed
        replacement = (
            f'{ind}# {marker}\n'
            f'{ind}z_speed = self._settings.get("z_feed_rate_mm_s", 5.0)\n'
            f'{ind}xy_travel_speed = self._settings.get("travel_speed_mm_s",\n'
            f'{ind}                    self._settings.get("xy_travel_speed_mm_s", 50.0))'
        )
        content = content[:m.start()] + replacement + content[m.end():]
        report("OK", "Added xy_travel_speed + raised z_speed default to 5mm/s")
    else:
        report("MISS", "Could not find z_speed setting read")

    # Now pass xy_travel_speed to create_travel_move
    # Find: create_travel_move(
    #          ...
    #          z_speed_mm_s=z_speed,
    #          pump_positions=...
    # Replace to add xy_speed_mm_s=xy_travel_speed
    travel_call_pat = re.compile(
        r'(z_speed_mm_s=z_speed,\n)'
        r'(\s+pump_positions=)'
    )
    m_call = travel_call_pat.search(content)
    if m_call:
        replacement = (
            f'{m_call.group(1)}'
            f'{m_call.group(2).rstrip()}xy_speed_mm_s=xy_travel_speed,\n'
            f'{m_call.group(2)}pump_positions='
        )
        content = content[:m_call.start()] + replacement + content[m_call.end():]
        report("OK", "Passed xy_travel_speed to create_travel_move")
    else:
        report("MISS", "Could not find create_travel_move call to add xy_speed")

    # AST + write
    try:
        ast.parse(content)
    except SyntaxError as e:
        print(f"  {RED}AST FAIL: line {e.lineno}: {e.msg}{RESET}")
        lines = content.split('\n')
        for i in range(max(0,e.lineno-4), min(len(lines),e.lineno+3)):
            mk = ">>>" if i==e.lineno-1 else "   "
            print(f"    {mk} {i+1:4d} | {lines[i]}")
        report("MISS", "AST failed")
        return

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    shutil.copy2(path, path.with_suffix(f".bak_v727ts_{ts}"))
    path.write_text(content, encoding="utf-8")
    print(f"  {GREEN}TrajectoryPlanner.py written + AST OK{RESET}")


def patch_print_setup_workspace_sync(root):
    """Push GUI speeds to workspace.print_settings before trajectory generation."""
    print(f"\n{CYAN}[2] print_setup.py — sync ALL speeds to workspace{RESET}")
    path = root / "gui" / "pages" / "print_setup.py"
    if not path.exists():
        report("MISS", "print_setup.py not found")
        return
    content = path.read_text(encoding="utf-8")

    marker = "v7.2.7: sync all speeds to workspace"
    if marker in content:
        report("SKIP", "Already patched")
        return

    # Find the call to plan_to_trajectory and inject workspace speed sync before it
    call_pat = re.compile(
        r'^(\s+)(trajectory_result = plan_to_trajectory\()',
        re.MULTILINE
    )
    m = call_pat.search(content)
    if not m:
        # Try alternate — might have been modified by previous patch
        call_pat2 = re.compile(
            r'^(\s+)(from SupportClasses\.PrintTrajectoryPlanner import plan_to_trajectory)',
            re.MULTILINE
        )
        m = call_pat2.search(content)

    if not m:
        report("MISS", "Could not find plan_to_trajectory import/call")
        return

    ind = m.group(1)
    inject = (
        f'{ind}# {marker}\n'
        f'{ind}try:\n'
        f'{ind}    _ws = getattr(self, "_workspace", None)\n'
        f'{ind}    if _ws is None:\n'
        f'{ind}        _ws = getattr(getattr(self, "tab_objects", None), "_workspace", None)\n'
        f'{ind}    if _ws and hasattr(_ws, "print_settings") and isinstance(_ws.print_settings, dict):\n'
        f'{ind}        _gui_spd = getattr(settings, "print_speed_mm_s", 5.0)\n'
        f'{ind}        _gui_travel = getattr(settings, "travel_speed_mm_s", _gui_spd * 2)\n'
        f'{ind}        _gui_z = getattr(settings, "z_feedrate", 60.0)\n'
        f'{ind}        _ws.print_settings["print_speed_mm_s"] = _gui_spd\n'
        f'{ind}        _ws.print_settings["travel_speed_mm_s"] = _gui_travel\n'
        f'{ind}        _ws.print_settings["z_feed_rate_mm_s"] = _gui_z if _gui_z < 20 else _gui_z / 60.0\n'
        f'{ind}        _ws.print_settings["travel_z_mm"] = getattr(settings, "travel_z_height", 5.0)\n'
        f'{ind}        logger.info(f"Workspace synced: print={{_gui_spd:.1f}}, "\n'
        f'{ind}                   f"travel={{_gui_travel:.1f}}, z={{_ws.print_settings[\'z_feed_rate_mm_s\']:.1f}} mm/s")\n'
        f'{ind}except Exception as _e:\n'
        f'{ind}    logger.debug(f"Workspace speed sync: {{_e}}")\n'
        f'\n'
    )
    content = content[:m.start()] + inject + content[m.start():]

    try:
        ast.parse(content)
    except SyntaxError as e:
        print(f"  {RED}AST FAIL: line {e.lineno}: {e.msg}{RESET}")
        lines = content.split('\n')
        for i in range(max(0,e.lineno-4), min(len(lines),e.lineno+3)):
            mk = ">>>" if i==e.lineno-1 else "   "
            print(f"    {mk} {i+1:4d} | {lines[i]}")
        report("MISS", "AST failed")
        return

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    shutil.copy2(path, path.with_suffix(f".bak_v727ws_{ts}"))
    path.write_text(content, encoding="utf-8")
    report("OK", "Added workspace speed sync before plan_to_trajectory")


def patch_default_travel_speed(root):
    """Raise default xy_speed_mm_s in create_travel_move from 20 to 50."""
    print(f"\n{CYAN}[3] TrajectoryPlanner.py — raise default travel speed{RESET}")
    path = root / "SupportClasses" / "TrajectoryPlanner.py"
    if not path.exists():
        report("MISS", "not found")
        return
    content = path.read_text(encoding="utf-8")

    # Find: xy_speed_mm_s: float = 20.0,
    old_default = re.compile(r'xy_speed_mm_s:\s*float\s*=\s*20\.0')
    if old_default.search(content):
        content = old_default.sub('xy_speed_mm_s: float = 50.0', content)
        try:
            ast.parse(content)
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            shutil.copy2(path, path.with_suffix(f".bak_v727td_{ts}"))
            path.write_text(content, encoding="utf-8")
            report("OK", "Raised default XY travel speed from 20→50 mm/s")
        except SyntaxError:
            report("MISS", "AST fail after default change")
    else:
        report("SKIP", "Default already changed or not found")


def main():
    global ok_count, skip_count, miss_count
    print(f"\n{BOLD}{'='*60}")
    print(f"  v7.2.7: Travel Speed Fix (23min overhead)")
    print(f"{'='*60}{RESET}")
    root = find_root()

    patch_trajectory_planner(root)
    patch_print_setup_workspace_sync(root)
    patch_default_travel_speed(root)

    total = ok_count + skip_count + miss_count
    print(f"\n{BOLD}{'='*60}")
    print(f"  Summary: {ok_count} applied, {skip_count} skipped, {miss_count} missed")
    print(f"{'='*60}{RESET}")

    print(f"\n{CYAN}Final AST verification:{RESET}")
    for rel in ["SupportClasses/TrajectoryPlanner.py", "gui/pages/print_setup.py"]:
        fpath = root / rel
        if fpath.exists():
            try:
                ast.parse(fpath.read_text(encoding="utf-8"))
                print(f"  {GREEN}✓ {rel}{RESET}")
            except SyntaxError as e:
                print(f"  {RED}✗ {rel}: line {e.lineno}{RESET}")

    if miss_count == 0:
        print(f"\n{GREEN}✓ All travel speed fixes applied!{RESET}")
    return 0 if miss_count == 0 else 1

if __name__ == "__main__":
    sys.exit(main())

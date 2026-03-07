#!/usr/bin/env python3
"""
v7.2.7: Fix trajectory time estimate — GUI speed never reaches trajectory planner.

Root cause: Object trajectories are time-parameterized at creation time on the
Print Objects tab using workspace.print_settings.print_speed_mm_s (default 5.0).
When _generate_print() runs, it has the correct speed from the GUI but the
trajectory timestamps are already baked at 5.0 mm/s.

Fix approach:
  1. In _generate_print(), re-scale trajectory timestamps to match GUI speed
  2. In _extract_needle_syringe(), also check PrintSettings from parent page
  3. Ensure plan_to_trajectory passes speed to TrajectoryPlanner

This is a simple time-rescale: if objects were generated at 5mm/s but we want
50mm/s, multiply all timestamps by 5/50 = 0.1, giving 10x shorter duration.
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


def patch_generate_print(root):
    """Inject trajectory time rescale in _generate_print before plan_to_trajectory."""
    print(f"\n{CYAN}[1] print_setup.py — rescale trajectory timestamps{RESET}")
    path = root / "gui" / "pages" / "print_setup.py"
    content = path.read_text(encoding="utf-8")

    marker = "v7.2.7: rescale trajectory speed"
    if marker in content:
        report("SKIP", "Already patched")
        return

    # Find the call to plan_to_trajectory
    call_pat = re.compile(
        r'^(\s+)(trajectory_result = plan_to_trajectory\()',
        re.MULTILINE
    )
    m = call_pat.search(content)
    if not m:
        report("MISS", "Could not find plan_to_trajectory call")
        return

    ind = m.group(1)
    inject = (
        f'{ind}# {marker}\n'
        f'{ind}# Object trajectories were time-parameterized at object creation\n'
        f'{ind}# speed (default 5mm/s). Rescale timestamps to match GUI speed.\n'
        f'{ind}_gui_speed = getattr(settings, "print_speed_mm_s", 5.0)\n'
        f'{ind}if _gui_speed > 0 and _gui_speed != 5.0:\n'
        f'{ind}    try:\n'
        f'{ind}        _objects_tab = getattr(self, "tab_objects", None)\n'
        f'{ind}        if _objects_tab:\n'
        f'{ind}            _objs = getattr(_objects_tab, "_objects", [])\n'
        f'{ind}            for _odata in _objs:\n'
        f'{ind}                _obj = _odata if not isinstance(_odata, dict) else None\n'
        f'{ind}                if _obj is None and isinstance(_odata, dict):\n'
        f'{ind}                    _obj = _odata.get("_print_object")\n'
        f'{ind}                if _obj and hasattr(_obj, "trajectory") and _obj.trajectory is not None:\n'
        f'{ind}                    import numpy as _np\n'
        f'{ind}                    _traj = _obj.trajectory\n'
        f'{ind}                    if len(_traj) > 1:\n'
        f'{ind}                        # Compute what speed the trajectory was generated at\n'
        f'{ind}                        _dx = _np.diff(_traj[:, 0])\n'
        f'{ind}                        _dy = _np.diff(_traj[:, 1])\n'
        f'{ind}                        _dt = _np.diff(_traj[:, 6])\n'
        f'{ind}                        _dt_safe = _np.maximum(_dt, 1e-9)\n'
        f'{ind}                        _dists = _np.sqrt(_dx**2 + _dy**2)\n'
        f'{ind}                        _speeds = _dists / _dt_safe\n'
        f'{ind}                        _mask = _dists > 0.001\n'
        f'{ind}                        if _np.any(_mask):\n'
        f'{ind}                            _orig_speed = float(_np.median(_speeds[_mask]))\n'
        f'{ind}                            if _orig_speed > 0.1:\n'
        f'{ind}                                _scale = _orig_speed / _gui_speed\n'
        f'{ind}                                _traj[:, 6] *= _scale\n'
        f'{ind}                                logger.info(f"Rescaled trajectory: "\n'
        f'{ind}                                           f"{{_orig_speed:.1f}} -> {{_gui_speed:.1f}} mm/s "\n'
        f'{ind}                                           f"(scale={{_scale:.3f}})")\n'
        f'{ind}    except Exception as _e:\n'
        f'{ind}        logger.warning(f"Trajectory speed rescale failed: {{_e}}")\n'
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
    shutil.copy2(path, path.with_suffix(f".bak_v727ts_{ts}"))
    path.write_text(content, encoding="utf-8")
    report("OK", "Added trajectory speed rescale before plan_to_trajectory")


def patch_workspace_speed(root):
    """Ensure workspace.print_settings gets GUI speed during _generate_print."""
    print(f"\n{CYAN}[2] print_setup.py — push speed to workspace{RESET}")
    path = root / "gui" / "pages" / "print_setup.py"
    content = path.read_text(encoding="utf-8")

    marker = "v7.2.7: push speed to workspace"
    if marker in content:
        report("SKIP", "Already patched")
        return

    # Find "settings = self._get_settings()" in _generate_print
    settings_pat = re.compile(
        r'^(\s+)(settings = self\._get_settings\(\))\s*$',
        re.MULTILINE
    )
    m = settings_pat.search(content)
    if not m:
        report("MISS", "Could not find settings = self._get_settings()")
        return

    ind = m.group(1)
    inject = (
        f'\n{ind}# {marker}\n'
        f'{ind}# Sync GUI speed to workspace so TrajectoryPlanner uses it\n'
        f'{ind}try:\n'
        f'{ind}    _ws = getattr(self, "_workspace", None)\n'
        f'{ind}    if _ws is None:\n'
        f'{ind}        _ws = getattr(getattr(self, "tab_objects", None), "_workspace", None)\n'
        f'{ind}    if _ws and hasattr(_ws, "print_settings"):\n'
        f'{ind}        if isinstance(_ws.print_settings, dict):\n'
        f'{ind}            _ws.print_settings["print_speed_mm_s"] = settings.print_speed_mm_s\n'
        f'{ind}            _ws.print_settings["travel_speed_mm_s"] = getattr(settings, "travel_speed_mm_s", 10.0)\n'
        f'{ind}            logger.info(f"Workspace speed synced: {{settings.print_speed_mm_s:.1f}} mm/s")\n'
        f'{ind}except Exception as _e:\n'
        f'{ind}    logger.debug(f"Workspace speed sync: {{_e}}")\n'
    )
    content = content[:m.end()] + inject + content[m.end():]

    try:
        ast.parse(content)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        shutil.copy2(path, path.with_suffix(f".bak_v727ws_{ts}"))
        path.write_text(content, encoding="utf-8")
        report("OK", "Added workspace speed sync after _get_settings")
    except SyntaxError as e:
        report("MISS", f"AST fail: {e}")


def patch_extract_needle_fallback(root):
    """Add fallback speed source in print_objects._extract_needle_syringe."""
    print(f"\n{CYAN}[3] print_objects.py — speed fallback to parent settings{RESET}")
    path = root / "gui" / "pages" / "print_objects.py"
    if not path.exists():
        report("MISS", "print_objects.py not found")
        return
    content = path.read_text(encoding="utf-8")

    marker = "v7.2.7: fallback speed from parent"
    if marker in content:
        report("SKIP", "Already patched")
        return

    # Find: speed = 5.0 (the default)
    # and add a fallback that checks parent page's xy_feed_spin
    default_speed_pat = re.compile(
        r'^(\s+)(speed = 5\.0)\s*$',
        re.MULTILINE
    )
    m = default_speed_pat.search(content)
    if not m:
        report("MISS", "Could not find 'speed = 5.0' default")
        return

    ind = m.group(1)
    # Replace with a version that tries to get speed from parent
    new_default = (
        f'{ind}speed = 5.0\n'
        f'{ind}# {marker}\n'
        f'{ind}# Try to get speed from parent PrintSetup page\'s GUI\n'
        f'{ind}try:\n'
        f'{ind}    _parent = self.parent()\n'
        f'{ind}    while _parent is not None:\n'
        f'{ind}        if hasattr(_parent, "xy_feed_spin"):\n'
        f'{ind}            speed = _parent.xy_feed_spin.value()\n'
        f'{ind}            break\n'
        f'{ind}        _parent = _parent.parent() if hasattr(_parent, "parent") else None\n'
        f'{ind}except Exception:\n'
        f'{ind}    pass'
    )
    content = content[:m.start()] + new_default + content[m.end():]

    try:
        ast.parse(content)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        shutil.copy2(path, path.with_suffix(f".bak_v727spd_{ts}"))
        path.write_text(content, encoding="utf-8")
        report("OK", "Added parent page speed fallback")
    except SyntaxError as e:
        report("MISS", f"AST fail: {e}")


def main():
    global ok_count, skip_count, miss_count
    print(f"\n{BOLD}{'='*60}")
    print(f"  v7.2.7: Trajectory Speed Fix (10x time estimate)")
    print(f"{'='*60}{RESET}")
    root = find_root()
    print(f"Project root: {root}")

    patch_generate_print(root)
    patch_workspace_speed(root)
    patch_extract_needle_fallback(root)

    total = ok_count + skip_count + miss_count
    print(f"\n{BOLD}{'='*60}")
    print(f"  Summary: {ok_count} applied, {skip_count} skipped, {miss_count} missed")
    print(f"{'='*60}{RESET}")

    print(f"\n{CYAN}Final AST verification:{RESET}")
    for rel in ["gui/pages/print_setup.py", "gui/pages/print_objects.py"]:
        fpath = root / rel
        if fpath.exists():
            try:
                ast.parse(fpath.read_text(encoding="utf-8"))
                print(f"  {GREEN}✓ {rel}{RESET}")
            except SyntaxError as e:
                print(f"  {RED}✗ {rel}: line {e.lineno}: {e.msg}{RESET}")

    if miss_count == 0:
        print(f"\n{GREEN}✓ All trajectory speed fixes applied!{RESET}")
    return 0 if miss_count == 0 else 1

if __name__ == "__main__":
    sys.exit(main())

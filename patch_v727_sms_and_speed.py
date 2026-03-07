#!/usr/bin/env python3
"""
v7.2.7: Fix SMS percentage bug + speed propagation chain.

Two fixes:
  1. XYStage.set_velocity() sends raw µm/s to SMS which expects 1-100%.
     Add set_speed_mm_s() that converts properly. Fix set_velocity() to
     accept either µm/s (auto-convert to %) or raw percentage.

  2. PrintManager._set_xy_speed_for_print() and TrajectoryExecutor call
     set_velocity(int(µm/s)) — update to use new set_speed_mm_s().
     Also fix _safe_navigate_to in calibration.

  3. print_setup._get_settings() passes GUI speed to workspace for
     trajectory generation.
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


def patch_xystage(root):
    """Fix set_velocity + add set_speed_mm_s to XYStage.py."""
    print(f"\n{CYAN}[1] XYStage.py — SMS percentage conversion{RESET}")
    path = root / "SupportClasses" / "XYStage.py"
    content = path.read_text(encoding="utf-8")

    marker = "v7.2.7: SMS percentage"
    if marker in content:
        report("SKIP", "Already patched")
        return

    # Replace set_velocity method
    m = find_method(content, "set_velocity")
    if not m:
        report("MISS", "set_velocity not found")
        return

    new_set_velocity = '''    def set_velocity(self, velocity: int) -> None:
        """v7.2.7: SMS percentage — Set max stage velocity.

        The Prior SMS command takes a percentage (1-100), not µm/s.
        If the caller passes a value > 100, we assume it's µm/s and convert.
        If <= 100, we assume it's already a percentage.

        For explicit mm/s control, use set_speed_mm_s() instead.
        """
        if velocity > 100:
            # Caller sent µm/s — convert to percentage
            max_speed = getattr(self, '_protocol_max_speed_um_s', 50000)
            pct = max(1, min(100, int(velocity / max_speed * 100)))
            logger.debug(f"set_velocity: {velocity} µm/s → SMS {pct}%")
        else:
            pct = max(1, min(100, int(velocity)))
        self._send_protocol_command(
            "set_max_speed",
            fallback_cmd=f"SMS,{pct}",
            speed=pct,
        )

    def set_speed_mm_s(self, speed_mm_s: float) -> None:
        """v7.2.7: Set stage speed in mm/s — converts to SMS percentage.

        This is the preferred method for all print/calibration code.
        Handles the full conversion: mm/s → µm/s → percentage of max.

        Args:
            speed_mm_s: Desired speed in mm/s (e.g., 1.0, 5.0, 50.0)
        """
        max_speed_um_s = getattr(self, '_protocol_max_speed_um_s', 50000)
        speed_um_s = speed_mm_s * 1000.0
        pct = max(1, min(100, int(speed_um_s / max_speed_um_s * 100)))
        logger.info(f"set_speed_mm_s: {speed_mm_s:.1f} mm/s = {speed_um_s:.0f} µm/s "
                    f"= SMS {pct}% (max={max_speed_um_s} µm/s)")
        self._send_protocol_command(
            "set_max_speed",
            fallback_cmd=f"SMS,{pct}",
            speed=pct,
        )

'''
    content = content[:m.start()] + new_set_velocity + content[m.end():]
    report("OK", "Replaced set_velocity + added set_speed_mm_s")

    # Add _protocol_max_speed_um_s attribute initialization
    # Find _apply_protocol_parameters to inject max_speed extraction
    marker2 = "_protocol_max_speed_um_s"
    if marker2 not in content:
        apply_pat = re.compile(
            r'^(    def _apply_protocol_parameters\(self\).*?\n)'
            r'(\s+""".*?""")',
            re.DOTALL | re.MULTILINE
        )
        m_apply = apply_pat.search(content)
        if m_apply:
            # Find the end of the method's docstring and inject after it
            inject_pos = m_apply.end()
            inject = (
                "\n"
                "        # v7.2.7: Extract max speed for SMS percentage calculation\n"
                "        if self._protocol:\n"
                "            params = self._protocol._config.get('parameters', {})\n"
                "            self._protocol_max_speed_um_s = float(params.get('max_speed', 50000))\n"
                "        else:\n"
                "            self._protocol_max_speed_um_s = 50000.0\n"
            )
            content = content[:inject_pos] + inject + content[inject_pos:]
            report("OK", "Added _protocol_max_speed_um_s extraction")
        else:
            # Fallback: add in __init__ area
            init_pat = re.compile(r'(self\.max_speed\s*=\s*\d+)', re.MULTILINE)
            m_init = init_pat.search(content)
            if m_init:
                content = content[:m_init.end()] + (
                    "\n        self._protocol_max_speed_um_s = 50000.0  "
                    "# v7.2.7: default, overridden by protocol"
                ) + content[m_init.end():]
                report("OK", "Added _protocol_max_speed_um_s in __init__")
            else:
                report("MISS", "Could not find init spot for _protocol_max_speed_um_s")

    # AST + write
    try:
        ast.parse(content)
    except SyntaxError as e:
        print(f"  {RED}AST FAIL: line {e.lineno}: {e.msg}{RESET}")
        lines = content.split('\n')
        for i in range(max(0,e.lineno-4), min(len(lines),e.lineno+3)):
            mk = ">>>" if i==e.lineno-1 else "   "
            print(f"    {mk} {i+1:4d} | {lines[i]}")
        report("MISS", "AST failed for XYStage.py")
        return

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    shutil.copy2(path, path.with_suffix(f".bak_v727sms_{ts}"))
    path.write_text(content, encoding="utf-8")
    print(f"  {GREEN}XYStage.py written + AST OK{RESET}")


def patch_printmanager_callers(root):
    """Update all set_velocity callers in PrintManager to use set_speed_mm_s."""
    print(f"\n{CYAN}[2] PrintManager.py — Update speed callers{RESET}")
    path = root / "SupportClasses" / "PrintManager.py"
    content = path.read_text(encoding="utf-8")

    marker = "v7.2.7: use set_speed_mm_s"
    if marker in content:
        report("SKIP", "Already patched")
        return

    count = 0

    # Replace _set_xy_speed_for_print to use set_speed_mm_s
    m = find_method(content, "_set_xy_speed_for_print")
    if m:
        new_method = '''    def _set_xy_speed_for_print(self, speed_mm_s: float = 0):
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
                # Fallback: old set_velocity with percentage estimate
                pct = max(1, min(100, int(speed_mm_s * 1000 / 50000 * 100)))
                ctrl.xy_stage.set_velocity(pct)
            logger.info(f"Print speed set: {speed_mm_s:.1f} mm/s")

'''
        content = content[:m.start()] + new_method + content[m.end():]
        count += 1
        report("OK", "Replaced _set_xy_speed_for_print")
    else:
        report("MISS", "_set_xy_speed_for_print not found")

    # Fix TrajectoryExecutor speed setup — find the block we injected
    # Replace: ctrl.xy_stage.set_velocity(speed_val)
    # With: ctrl.xy_stage.set_speed_mm_s(_max_spd * 1.5)
    traj_set_vel = re.compile(
        r'ctrl\.xy_stage\.set_velocity\(_sms_val\)'
    )
    if traj_set_vel.search(content):
        content = traj_set_vel.sub(
            "# v7.2.7: use set_speed_mm_s\n"
            "                    if hasattr(ctrl.xy_stage, 'set_speed_mm_s'):\n"
            "                        ctrl.xy_stage.set_speed_mm_s(_max_spd * 1.5)\n"
            "                    else:\n"
            "                        ctrl.xy_stage.set_velocity(_sms_val)",
            content
        )
        count += 1
        report("OK", "Fixed TrajectoryExecutor speed call")
    else:
        # Try the older pattern
        traj_set_vel2 = re.compile(
            r'ctrl\.xy_stage\.set_velocity\(speed_val\)'
        )
        if traj_set_vel2.search(content):
            content = traj_set_vel2.sub(
                "# v7.2.7: use set_speed_mm_s\n"
                "                    if hasattr(ctrl.xy_stage, 'set_speed_mm_s'):\n"
                "                        ctrl.xy_stage.set_speed_mm_s(max_speed_mm_s * 1.5)\n"
                "                    else:\n"
                "                        ctrl.xy_stage.set_velocity(speed_val)",
                content
            )
            count += 1
            report("OK", "Fixed TrajectoryExecutor speed call (alt pattern)")
        else:
            report("MISS", "TrajectoryExecutor set_velocity call not found (may be fine)")

    # Fix MOVE_XY travel speed — replace set_velocity(int(min(...))) with set_speed_mm_s
    travel_pat = re.compile(
        r'ctrl\.xy_stage\.set_velocity\(int\(min\(_tspd \* 1000, 50000\)\)\)'
    )
    if travel_pat.search(content):
        content = travel_pat.sub(
            "ctrl.xy_stage.set_speed_mm_s(_tspd) if hasattr(ctrl.xy_stage, 'set_speed_mm_s') "
            "else ctrl.xy_stage.set_velocity(int(min(_tspd * 1000 / 50000 * 100, 100)))",
            content
        )
        count += 1
        report("OK", "Fixed MOVE_XY travel speed call")
    else:
        report("MISS", "MOVE_XY travel speed pattern not found")

    # Fix HOME_XY travel speed
    home_pat = re.compile(
        r'ctrl\.xy_stage\.set_velocity\(int\(min\(_hspd \* 1000, 50000\)\)\)'
    )
    if home_pat.search(content):
        content = home_pat.sub(
            "ctrl.xy_stage.set_speed_mm_s(_hspd) if hasattr(ctrl.xy_stage, 'set_speed_mm_s') "
            "else ctrl.xy_stage.set_velocity(int(min(_hspd * 1000 / 50000 * 100, 100)))",
            content
        )
        count += 1
        report("OK", "Fixed HOME_XY travel speed call")
    else:
        report("MISS", "HOME_XY travel speed pattern not found")

    if count == 0:
        report("MISS", "No callers updated")
        return

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
    shutil.copy2(path, path.with_suffix(f".bak_v727sms_{ts}"))
    path.write_text(content, encoding="utf-8")
    print(f"  {GREEN}PrintManager.py written + AST OK{RESET}")


def patch_calibration_callers(root):
    """Fix set_velocity calls in calibration's _safe_navigate_to."""
    print(f"\n{CYAN}[3] calibration.py — safe_navigate speed{RESET}")
    path = root / "gui" / "pages" / "calibration.py"
    if not path.exists():
        report("MISS", "calibration.py not found")
        return
    content = path.read_text(encoding="utf-8")

    # Replace: ctrl.xy_stage.set_velocity(int(min(10000, 50000)))
    old_pat = re.compile(
        r'ctrl\.xy_stage\.set_velocity\(int\(min\(10000,\s*50000\)\)\)'
    )
    if old_pat.search(content):
        content = old_pat.sub(
            "ctrl.xy_stage.set_speed_mm_s(10.0) if hasattr(ctrl.xy_stage, 'set_speed_mm_s') "
            "else ctrl.xy_stage.set_velocity(20)  # 20% ≈ 10mm/s",
            content
        )
        try:
            ast.parse(content)
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            shutil.copy2(path, path.with_suffix(f".bak_v727sms_{ts}"))
            path.write_text(content, encoding="utf-8")
            report("OK", "Fixed _safe_navigate_to speed call")
        except SyntaxError as e:
            report("MISS", f"AST fail after calibration fix: {e}")
    else:
        report("SKIP", "calibration speed call already fixed or not present")


def patch_speed_propagation(root):
    """Ensure GUI speed reaches trajectory generation."""
    print(f"\n{CYAN}[4] print_setup.py — speed propagation to trajectory{RESET}")
    path = root / "gui" / "pages" / "print_setup.py"
    if not path.exists():
        report("MISS", "print_setup.py not found")
        return
    content = path.read_text(encoding="utf-8")

    marker = "v7.2.7: propagate GUI speed"
    if marker in content:
        report("SKIP", "Already patched")
        return

    # Find _get_settings and ensure print_speed_mm_s is set
    # (Our earlier patch should have done this, but verify)
    if "s.print_speed_mm_s" not in content:
        pat = re.compile(
            r'^(\s+)(s\.xy_feedrate\s*=\s*self\.xy_feed_spin\.value\(\))',
            re.MULTILINE
        )
        m = pat.search(content)
        if m:
            ind = m.group(1)
            inject = (
                f"\n{ind}# {marker} to trajectory planner\n"
                f"{ind}s.print_speed_mm_s = self.xy_feed_spin.value()\n"
                f"{ind}s.travel_speed_mm_s = self.xy_feed_spin.value() * 2.0\n"
                f"{ind}s.print_feedrate = self.xy_feed_spin.value() * 60.0\n"
            )
            content = content[:m.end()] + inject + content[m.end():]
        else:
            report("MISS", "Could not find xy_feedrate assignment")
            return
    else:
        report("SKIP", "print_speed_mm_s already set in _get_settings")

    # Find _generate_print and ensure speed is passed to trajectory planner
    # Look for generate_object_trajectory or plan_trajectory calls
    gen_pat = re.compile(
        r'(generate_object_trajectory\(\s*\n\s+obj,\s*needle,\s*syringe_map,)'
    )
    m_gen = gen_pat.search(content)
    if m_gen and "print_speed_mm_s=self.xy_feed_spin" not in content:
        # Check if the call already has print_speed_mm_s parameter
        after_call = content[m_gen.end():m_gen.end()+200]
        if "print_speed_mm_s" not in after_call:
            # Need to add it — find the closing paren of the call
            report("MISS", "Need to add speed param to generate_object_trajectory call — complex")
        else:
            report("SKIP", "generate_object_trajectory already has speed param")
    else:
        report("SKIP", "Speed propagation already present or call not found")

    try:
        ast.parse(content)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        shutil.copy2(path, path.with_suffix(f".bak_v727spd_{ts}"))
        path.write_text(content, encoding="utf-8")
        report("OK", "print_setup.py written + AST OK")
    except SyntaxError as e:
        report("MISS", f"AST fail: {e}")


def main():
    global ok_count, skip_count, miss_count
    print(f"\n{BOLD}{'='*60}")
    print(f"  v7.2.7: SMS Percentage Fix + Speed Chain")
    print(f"{'='*60}{RESET}")
    root = find_root()
    print(f"Project root: {root}")

    patch_xystage(root)
    patch_printmanager_callers(root)
    patch_calibration_callers(root)
    patch_speed_propagation(root)

    total = ok_count + skip_count + miss_count
    print(f"\n{BOLD}{'='*60}")
    print(f"  Summary: {ok_count} applied, {skip_count} skipped, {miss_count} missed")
    print(f"{'='*60}{RESET}")

    # Final AST on all files
    print(f"\n{CYAN}Final AST verification:{RESET}")
    for rel in ["SupportClasses/XYStage.py", "SupportClasses/PrintManager.py",
                 "gui/pages/calibration.py", "gui/pages/print_setup.py"]:
        fpath = root / rel
        if fpath.exists():
            try:
                ast.parse(fpath.read_text(encoding="utf-8"))
                print(f"  {GREEN}✓ {rel}{RESET}")
            except SyntaxError as e:
                print(f"  {RED}✗ {rel}: line {e.lineno}: {e.msg}{RESET}")

    if miss_count > 0:
        print(f"\n{YELLOW}⚠ {miss_count} items need attention{RESET}")
    else:
        print(f"\n{GREEN}✓ All SMS + speed fixes applied!{RESET}")
    return 0 if miss_count == 0 else 1

if __name__ == "__main__":
    sys.exit(main())

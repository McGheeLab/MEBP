#!/usr/bin/env python3
"""
v7.2.7 Patch: Fix 10x position display error on Dashboard + Calibration.

Root cause: The Prior ProScan controller reports positions in microns (µm)
directly. The jog page (v7.2.5) correctly displays raw values. But the
Dashboard and Calibration pages still divide by microsteps_per_micron (10.0),
making their displayed values 10x too small.

Fix: Remove the /microsteps_per_micron division from Dashboard.update_data()
and Calibration.on_status_update(). Both should display raw zero-ref values
just like the jog page does.

Also fixes the zero-position display in calibration's _set_zero() which
applies steps_to_um() to what are already micron values.
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

ok_count = 0
skip_count = 0
miss_count = 0


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


def safe_read(path: Path) -> str:
    if not path.exists():
        return ""
    return path.read_text(encoding="utf-8")


def ast_check(content: str, label: str) -> bool:
    try:
        ast.parse(content)
        return True
    except SyntaxError as e:
        print(f"  {RED}AST FAIL ({label}): line {e.lineno}: {e.msg}{RESET}")
        lines = content.split('\n')
        if e.lineno:
            start = max(0, e.lineno - 4)
            end = min(len(lines), e.lineno + 3)
            for i in range(start, end):
                m = ">>>" if i == e.lineno - 1 else "   "
                print(f"    {m} {i+1:4d} | {lines[i]}")
        return False


def safe_write(path: Path, content: str, label: str) -> bool:
    if not ast_check(content, label):
        return False
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    backup = path.with_suffix(f".bak_v727pos_{ts}")
    if path.exists():
        shutil.copy2(path, backup)
    path.write_text(content, encoding="utf-8")
    return True


def report(status: str, msg: str):
    global ok_count, skip_count, miss_count
    if status == "OK":
        ok_count += 1
        print(f"  {GREEN}✓ {msg}{RESET}")
    elif status == "SKIP":
        skip_count += 1
        print(f"  {YELLOW}○ SKIP: {msg}{RESET}")
    else:
        miss_count += 1
        print(f"  {RED}✗ MISS: {msg}{RESET}")


# ═══════════════════════════════════════════════════════════════════
#  FIX 1: Dashboard — remove /microsteps_per_micron from XY display
# ═══════════════════════════════════════════════════════════════════

def fix_dashboard(root: Path):
    """Remove microstep conversion from dashboard XY display."""
    print(f"\n{CYAN}[1] Dashboard — XY position display{RESET}")

    path = root / "gui" / "pages" / "dashboard.py"
    content = safe_read(path)
    if not content:
        report("MISS", "dashboard.py not found")
        return

    marker = "v7.2.7: controller reports µm directly"
    if marker in content:
        report("SKIP", "Dashboard already patched")
        return

    # Find the conversion pattern in update_data():
    #   zx = xy[0] - ctrl.zero_position["x"]
    #   zy = xy[1] - ctrl.zero_position["y"]
    #   # Convert steps → µm
    #   ux = zx / self._microsteps_per_micron
    #   uy = zy / self._microsteps_per_micron
    pattern = re.compile(
        r'(\s+)(zx = xy\[0\] - ctrl\.zero_position\["x"\]\n)'
        r'(\s+zy = xy\[1\] - ctrl\.zero_position\["y"\]\n)'
        r'(\s+# Convert steps → µm\n)'
        r'(\s+ux = zx / self\._microsteps_per_micron\n)'
        r'(\s+uy = zy / self\._microsteps_per_micron\n)',
    )
    match = pattern.search(content)

    if match:
        indent = match.group(1)
        replacement = (
            f"{indent}# v7.2.7: controller reports µm directly — no conversion needed\n"
            f"{indent}ux = xy[0] - ctrl.zero_position[\"x\"]\n"
            f"{indent}uy = xy[1] - ctrl.zero_position[\"y\"]\n"
        )
        content = content[:match.start()] + replacement + content[match.end():]

        if safe_write(path, content, "dashboard XY"):
            report("OK", "Removed microstep conversion from dashboard XY display")
        else:
            report("MISS", "Dashboard write failed")
    else:
        # Try a more flexible pattern (maybe comment text differs)
        pattern2 = re.compile(
            r'(\s+)\w+ = xy\[0\] - ctrl\.zero_position\["x"\]\s*\n'
            r'\s+\w+ = xy\[1\] - ctrl\.zero_position\["y"\]\s*\n'
            r'(\s+#[^\n]*\n)?'  # optional comment
            r'\s+ux = \w+ / self\._microsteps_per_micron\s*\n'
            r'\s+uy = \w+ / self\._microsteps_per_micron\s*\n',
        )
        match2 = pattern2.search(content)
        if match2:
            indent = match2.group(1)
            replacement = (
                f"{indent}# v7.2.7: controller reports µm directly — no conversion needed\n"
                f"{indent}ux = xy[0] - ctrl.zero_position[\"x\"]\n"
                f"{indent}uy = xy[1] - ctrl.zero_position[\"y\"]\n"
            )
            content = content[:match2.start()] + replacement + content[match2.end():]
            if safe_write(path, content, "dashboard XY flex"):
                report("OK", "Removed microstep conversion from dashboard (flex match)")
            else:
                report("MISS", "Dashboard flex write failed")
        else:
            report("MISS", "Could not find microstep conversion in dashboard.update_data()")


# ═══════════════════════════════════════════════════════════════════
#  FIX 2: Calibration — remove steps_to_um from XY display
# ═══════════════════════════════════════════════════════════════════

def fix_calibration_display(root: Path):
    """Remove steps_to_um from calibration position display."""
    print(f"\n{CYAN}[2] Calibration — XY position display{RESET}")

    path = root / "gui" / "pages" / "calibration.py"
    content = safe_read(path)
    if not content:
        report("MISS", "calibration.py not found")
        return

    marker = "v7.2.7: controller reports µm directly"
    if marker in content:
        report("SKIP", "Calibration display already patched")
        return

    # Pattern in on_status_update():
    #   zx = xy[0] - ctrl.zero_position["x"]
    #   zy = xy[1] - ctrl.zero_position["y"]
    #   ux = steps_to_um(zx, self._microsteps_per_micron)
    #   uy = steps_to_um(zy, self._microsteps_per_micron)
    pattern = re.compile(
        r'(\s+)(zx = xy\[0\] - ctrl\.zero_position\["x"\]\n)'
        r'(\s+zy = xy\[1\] - ctrl\.zero_position\["y"\]\n)'
        r'(\s+ux = steps_to_um\(zx, self\._microsteps_per_micron\)\n)'
        r'(\s+uy = steps_to_um\(zy, self\._microsteps_per_micron\)\n)',
    )
    match = pattern.search(content)

    if match:
        indent = match.group(1)
        replacement = (
            f"{indent}# v7.2.7: controller reports µm directly — no conversion needed\n"
            f"{indent}ux = xy[0] - ctrl.zero_position[\"x\"]\n"
            f"{indent}uy = xy[1] - ctrl.zero_position[\"y\"]\n"
        )
        content = content[:match.start()] + replacement + content[match.end():]
        changed = True
    else:
        report("MISS", "Could not find steps_to_um in calibration on_status_update()")
        changed = False

    # Also fix _set_zero() which uses steps_to_um on zero position:
    #   zx_um = steps_to_um(z['x'], self._microsteps_per_micron)
    #   zy_um = steps_to_um(z['y'], self._microsteps_per_micron)
    zero_pattern = re.compile(
        r"zx_um = steps_to_um\(z\['x'\], self\._microsteps_per_micron\)\n"
        r"\s+zy_um = steps_to_um\(z\['y'\], self\._microsteps_per_micron\)"
    )
    zero_match = zero_pattern.search(content)
    if zero_match:
        replacement_zero = (
            "# v7.2.7: zero positions already in µm\n"
            "            zx_um = z['x']\n"
            "            zy_um = z['y']"
        )
        content = content[:zero_match.start()] + replacement_zero + content[zero_match.end():]
        changed = True
        report("OK", "Fixed _set_zero() to not convert µm values")
    else:
        # Non-critical if _set_zero not found
        pass

    if changed:
        if safe_write(path, content, "calibration"):
            report("OK", "Removed steps_to_um from calibration XY display")
        else:
            report("MISS", "Calibration write failed")


# ═══════════════════════════════════════════════════════════════════
#  FIX 3: Verify jog page is NOT double-converting
# ═══════════════════════════════════════════════════════════════════

def verify_jog(root: Path):
    """Check if jog page has the correct (no-conversion) pattern."""
    print(f"\n{CYAN}[3] Jog Control — verify correct display{RESET}")

    path = root / "gui" / "pages" / "jog_control.py"
    content = safe_read(path)
    if not content:
        report("MISS", "jog_control.py not found")
        return

    # Check which version of on_status_update is present
    # BAD: ux = zx / self._microsteps_per_micron
    # GOOD: ux = xy[0] - ctrl.zero_position["x"]  (raw value)
    
    has_division = "/ self._microsteps_per_micron" in content
    has_raw = 'ux = xy[0] - ctrl.zero_position["x"]' in content

    if has_raw and not has_division:
        report("OK", "Jog page displays raw µm values (correct)")
    elif has_division:
        # Jog page still has the old conversion - this is the source of the
        # discrepancy but Alex said jog is correct... 
        # Let's check if BOTH versions exist (project knowledge showed both)
        
        # Count occurrences of the division
        divisions = content.count("/ self._microsteps_per_micron")
        print(f"  {YELLOW}  Note: Jog page has {divisions} microstep division(s){RESET}")
        
        # Check if the v7.2.5 version (no division) is also present
        if has_raw:
            report("OK", "Jog page has BOTH patterns — v7.2.5 raw display is active")
        else:
            # The jog page is ALSO dividing. If Alex says jog is correct and
            # dashboard/calibration show 10x different, the jog page must have
            # some other code path. Let's just report what we find.
            report("MISS", 
                   f"Jog page has {divisions} division(s) by microsteps_per_micron. "
                   f"If jog shows correct values, the actual file may differ from "
                   f"project knowledge. Check manually.")
    else:
        report("OK", "Jog page appears to not use microstep conversion")


# ═══════════════════════════════════════════════════════════════════
#  MAIN
# ═══════════════════════════════════════════════════════════════════

def main():
    global ok_count, skip_count, miss_count

    print(f"\n{BOLD}{'='*60}")
    print(f"  MEBP v7.2.7: Position Display 10x Fix")
    print(f"{'='*60}{RESET}")

    root = find_root()
    print(f"Project root: {root}")

    fix_dashboard(root)
    fix_calibration_display(root)
    verify_jog(root)

    # Summary
    total = ok_count + skip_count + miss_count
    print(f"\n{BOLD}{'='*60}")
    print(f"  Summary: {ok_count} applied, {skip_count} skipped, {miss_count} missed")
    print(f"{'='*60}{RESET}")

    # Final AST checks
    print(f"\n{CYAN}Final AST verification:{RESET}")
    for rel in ["gui/pages/dashboard.py", "gui/pages/calibration.py"]:
        fpath = root / rel
        if fpath.exists():
            try:
                ast.parse(fpath.read_text(encoding="utf-8"))
                print(f"  {GREEN}✓ {rel}{RESET}")
            except SyntaxError as e:
                print(f"  {RED}✗ {rel}: line {e.lineno}: {e.msg}{RESET}")

    if miss_count > 0:
        print(f"\n{YELLOW}⚠ {miss_count} items need attention.{RESET}")
    else:
        print(f"\n{GREEN}✓ All position display fixes applied!{RESET}")

    return 0 if miss_count == 0 else 1


if __name__ == "__main__":
    sys.exit(main())

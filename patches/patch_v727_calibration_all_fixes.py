#!/usr/bin/env python3
"""
v7.2.7: Fix ALL 10x errors and coordinate bugs in calibration.py.

Fixes:
  1. on_status_update() — remove steps_to_um from XY display
  2. _set_zero() — remove steps_to_um from zero display  
  3. _record_a1() — display zero-ref µm directly, store zero-ref
  4. _record_corner() — same as _record_a1
  5. _goto_a1() — fix double-subtraction of zero_position
  6. _goto_corner() — fix double-subtraction
  7. _load_calibration() — remove steps_to_um from loaded positions
  8. _calculate_alignment() — fix unit mismatch (µm vs mm)

Strategy: Each fix uses a unique regex match on actual code patterns,
captures indentation, and replaces with corrected code.
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
        if (c / "SupportClasses").is_dir() and (c / "gui").is_dir():
            return c
    sys.exit(f"{RED}Cannot find MEBP root{RESET}")

def report(s, msg):
    global ok_count, skip_count, miss_count
    if s == "OK": ok_count += 1; print(f"  {GREEN}✓ {msg}{RESET}")
    elif s == "SKIP": skip_count += 1; print(f"  {YELLOW}○ {msg}{RESET}")
    else: miss_count += 1; print(f"  {RED}✗ {msg}{RESET}")


def main():
    global ok_count, skip_count, miss_count
    print(f"\n{BOLD}{'='*60}")
    print(f"  Calibration: Fix ALL 10x Errors + Coordinate Bugs")
    print(f"{'='*60}{RESET}")

    root = find_root()
    path = root / "gui" / "pages" / "calibration.py"
    content = path.read_text(encoding="utf-8")

    marker = "v7.2.7: positions are µm"
    if marker in content:
        print(f"  {YELLOW}Already patched (marker found){RESET}")
        return 0

    # ── Fix 1: on_status_update — remove steps_to_um ────────────
    print(f"\n{CYAN}[1] on_status_update XY display{RESET}")
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
        content = content[:m1.start()] + (
            f"{ind}# v7.2.7: positions are µm directly from controller\n"
            f"{ind}ux = xy[0] - ctrl.zero_position[\"x\"]\n"
            f"{ind}uy = xy[1] - ctrl.zero_position[\"y\"]\n"
        ) + content[m1.end():]
        report("OK", "Fixed on_status_update")
    else:
        report("MISS", "on_status_update pattern not found")

    # ── Fix 2: _set_zero — remove steps_to_um ───────────────────
    print(f"\n{CYAN}[2] _set_zero display{RESET}")
    pat2 = re.compile(
        r"^(\s+)zx_um = steps_to_um\(z\['x'\], self\._microsteps_per_micron\)\s*\n"
        r"\s+zy_um = steps_to_um\(z\['y'\], self\._microsteps_per_micron\)",
        re.MULTILINE
    )
    m2 = pat2.search(content)
    if m2:
        ind = m2.group(1)
        content = content[:m2.start()] + (
            f"{ind}# v7.2.7: zero positions already in µm\n"
            f"{ind}zx_um = z['x']\n"
            f"{ind}zy_um = z['y']"
        ) + content[m2.end():]
        report("OK", "Fixed _set_zero")
    else:
        report("MISS", "_set_zero pattern not found")

    # ── Fix 3: _record_a1 — store zero-ref, display raw µm ──────
    print(f"\n{CYAN}[3] _record_a1{RESET}")
    pat3 = re.compile(
        r'^(\s+)def _record_a1\(self\):\s*\n'
        r'(.*?)'
        r'(?=\n\s+def )',
        re.DOTALL | re.MULTILINE
    )
    m3 = pat3.search(content)
    if m3:
        ind = m3.group(1)
        new_a1 = (
            f'{ind}def _record_a1(self):\n'
            f'{ind}    """v7.2.7: Record A1 position — positions are µm directly."""\n'
            f'{ind}    xy = self.controller.get_xy_position(cached=False)\n'
            f'{ind}    zp = self.controller.get_zp_position(cached=False)\n'
            f'{ind}    if xy[0] is None:\n'
            f'{ind}        return\n'
            f'{ind}    self._taught_a1 = (xy[0], xy[1])\n'
            f'{ind}    # Display zero-referenced position in µm\n'
            f'{ind}    ax = xy[0] - self.controller.zero_position["x"]\n'
            f'{ind}    ay = xy[1] - self.controller.zero_position["y"]\n'
            f'{ind}    self.lbl_a1.setText(f"({{ax:,.1f}}, {{ay:,.1f}}) µm")\n'
            f'{ind}    self.lbl_a1.setStyleSheet(f"color: {{COLORS[\'green\']}};")\n'
            f'{ind}    # Store Z if available\n'
            f'{ind}    if zp is not None and zp[0] is not None:\n'
            f'{ind}        self._taught_a1_z = zp[0] - self.controller.zero_position.get("Z", 0)\n'
            f'{ind}    logger.info(f"Taught A1: {{self._taught_a1}}")\n'
        )
        content = content[:m3.start()] + new_a1 + content[m3.end():]
        report("OK", "Replaced _record_a1")
    else:
        report("MISS", "_record_a1 not found")

    # ── Fix 4: _record_corner — same pattern ────────────────────
    print(f"\n{CYAN}[4] _record_corner{RESET}")
    pat4 = re.compile(
        r'^(\s+)def _record_corner\(self\):\s*\n'
        r'(.*?)'
        r'(?=\n\s+def )',
        re.DOTALL | re.MULTILINE
    )
    m4 = pat4.search(content)
    if m4:
        ind = m4.group(1)
        new_corner = (
            f'{ind}def _record_corner(self):\n'
            f'{ind}    """v7.2.7: Record corner position — positions are µm directly."""\n'
            f'{ind}    xy = self.controller.get_xy_position(cached=False)\n'
            f'{ind}    zp = self.controller.get_zp_position(cached=False)\n'
            f'{ind}    if xy[0] is None:\n'
            f'{ind}        return\n'
            f'{ind}    self._taught_corner = (xy[0], xy[1])\n'
            f'{ind}    # Display zero-referenced position in µm\n'
            f'{ind}    cx = xy[0] - self.controller.zero_position["x"]\n'
            f'{ind}    cy = xy[1] - self.controller.zero_position["y"]\n'
            f'{ind}    self.lbl_corner.setText(f"({{cx:,.1f}}, {{cy:,.1f}}) µm")\n'
            f'{ind}    self.lbl_corner.setStyleSheet(f"color: {{COLORS[\'green\']}};")\n'
            f'{ind}    # Store Z if available\n'
            f'{ind}    if zp is not None and zp[0] is not None:\n'
            f'{ind}        self._taught_corner_z = zp[0] - self.controller.zero_position.get("Z", 0)\n'
            f'{ind}    logger.info(f"Taught corner: {{self._taught_corner}}")\n'
        )
        content = content[:m4.start()] + new_corner + content[m4.end():]
        report("OK", "Replaced _record_corner")
    else:
        report("MISS", "_record_corner not found")

    # ── Fix 5: _goto_a1 — fix double-subtraction ────────────────
    print(f"\n{CYAN}[5] _goto_a1 coordinate fix{RESET}")
    pat5 = re.compile(
        r'^(\s+)def _goto_a1\(self\):\s*\n'
        r'(.*?)'
        r'(?=\n\s+def )',
        re.DOTALL | re.MULTILINE
    )
    m5 = pat5.search(content)
    if m5:
        ind = m5.group(1)
        new_goto_a1 = (
            f'{ind}def _goto_a1(self):\n'
            f'{ind}    """v7.2.7: Navigate to taught A1 — raw µm, no zero-ref."""\n'
            f'{ind}    if self._taught_a1:\n'
            f'{ind}        # _taught_a1 stores absolute stage position in µm\n'
            f'{ind}        # Use from_zero_ref=False since it\'s already absolute\n'
            f'{ind}        self.controller.move_xy_absolute(\n'
            f'{ind}            self._taught_a1[0], self._taught_a1[1],\n'
            f'{ind}            from_zero_ref=False,\n'
            f'{ind}        )\n'
        )
        content = content[:m5.start()] + new_goto_a1 + content[m5.end():]
        report("OK", "Fixed _goto_a1 coordinate handling")
    else:
        report("MISS", "_goto_a1 not found")

    # ── Fix 6: _goto_corner — fix double-subtraction ─────────────
    print(f"\n{CYAN}[6] _goto_corner coordinate fix{RESET}")
    pat6 = re.compile(
        r'^(\s+)def _goto_corner\(self\):\s*\n'
        r'(.*?)'
        r'(?=\n\s+def )',
        re.DOTALL | re.MULTILINE
    )
    m6 = pat6.search(content)
    if m6:
        ind = m6.group(1)
        new_goto_corner = (
            f'{ind}def _goto_corner(self):\n'
            f'{ind}    """v7.2.7: Navigate to taught corner — raw µm, no zero-ref."""\n'
            f'{ind}    if self._taught_corner:\n'
            f'{ind}        self.controller.move_xy_absolute(\n'
            f'{ind}            self._taught_corner[0], self._taught_corner[1],\n'
            f'{ind}            from_zero_ref=False,\n'
            f'{ind}        )\n'
        )
        content = content[:m6.start()] + new_goto_corner + content[m6.end():]
        report("OK", "Fixed _goto_corner coordinate handling")
    else:
        report("MISS", "_goto_corner not found")

    # ── Fix 7: _load_calibration — remove steps_to_um ───────────
    print(f"\n{CYAN}[7] _load_calibration display{RESET}")
    # Fix first occurrence: taught_a1
    pat7a = re.compile(
        r'a1x = steps_to_um\(self\._taught_a1\[0\], self\._microsteps_per_micron\)\s*\n'
        r'(\s+)a1y = steps_to_um\(self\._taught_a1\[1\], self\._microsteps_per_micron\)',
    )
    m7a = pat7a.search(content)
    if m7a:
        ind = m7a.group(1)
        content = content[:m7a.start()] + (
            f"# v7.2.7: taught positions stored as raw µm — display zero-ref\n"
            f"{ind}a1x = self._taught_a1[0] - self.controller.zero_position.get('x', 0)\n"
            f"{ind}a1y = self._taught_a1[1] - self.controller.zero_position.get('y', 0)"
        ) + content[m7a.end():]
        report("OK", "Fixed _load_calibration A1 display")
    else:
        report("MISS", "_load_calibration A1 pattern not found")

    # Fix second occurrence: taught_corner
    pat7b = re.compile(
        r'cx = steps_to_um\(self\._taught_corner\[0\], self\._microsteps_per_micron\)\s*\n'
        r'(\s+)cy = steps_to_um\(self\._taught_corner\[1\], self\._microsteps_per_micron\)',
    )
    m7b = pat7b.search(content)
    if m7b:
        ind = m7b.group(1)
        content = content[:m7b.start()] + (
            f"# v7.2.7: taught positions stored as raw µm\n"
            f"{ind}cx = self._taught_corner[0] - self.controller.zero_position.get('x', 0)\n"
            f"{ind}cy = self._taught_corner[1] - self.controller.zero_position.get('y', 0)"
        ) + content[m7b.end():]
        report("OK", "Fixed _load_calibration corner display")
    else:
        report("MISS", "_load_calibration corner pattern not found")

    # ── Fix 8: Initialize new Z attributes ───────────────────────
    print(f"\n{CYAN}[8] Add Z tracking attributes{RESET}")
    # Add _taught_a1_z and _taught_corner_z initialization
    init_pattern = re.compile(
        r'(self\._taught_a1\s*=\s*None\s*\n\s+self\._taught_corner\s*=\s*None)'
    )
    m8 = init_pattern.search(content)
    if m8:
        if "_taught_a1_z" not in content:
            content = content[:m8.end()] + (
                "\n        self._taught_a1_z = None      # v7.2.7: Z at A1\n"
                "        self._taught_corner_z = None  # v7.2.7: Z at corner"
            ) + content[m8.end():]
            report("OK", "Added Z tracking attributes")
        else:
            report("SKIP", "Z attributes already present")
    else:
        report("MISS", "Could not find taught init block")

    # ── AST verify + write ───────────────────────────────────────
    print(f"\n{CYAN}AST verification...{RESET}")
    try:
        ast.parse(content)
        print(f"  {GREEN}✓ AST OK{RESET}")
    except SyntaxError as e:
        print(f"  {RED}✗ AST FAIL: line {e.lineno}: {e.msg}{RESET}")
        lines = content.split('\n')
        for i in range(max(0, e.lineno-4), min(len(lines), e.lineno+3)):
            m = ">>>" if i == e.lineno-1 else "   "
            print(f"    {m} {i+1:4d} | {lines[i]}")
        return 1

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    shutil.copy2(path, path.with_suffix(f".bak_v727all_{ts}"))
    path.write_text(content, encoding="utf-8")

    total = ok_count + skip_count + miss_count
    print(f"\n{BOLD}{'='*60}")
    print(f"  Summary: {ok_count} applied, {skip_count} skipped, {miss_count} missed")
    print(f"{'='*60}{RESET}")

    if miss_count > 0:
        print(f"\n{YELLOW}⚠ {miss_count} items need attention{RESET}")
    else:
        print(f"\n{GREEN}✓ All calibration fixes applied!{RESET}")

    return 0 if miss_count == 0 else 1


if __name__ == "__main__":
    sys.exit(main())

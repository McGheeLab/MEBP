#!/usr/bin/env python3
"""Fix: reset _scale/_rotation on A1 teach + guard _estimate against stale values."""

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
    print(f"\n{BOLD}  Fix stale _scale in calibration navigation{RESET}")
    root = find_root()
    path = root / "gui" / "pages" / "calibration.py"
    content = path.read_text(encoding="utf-8")
    changed = False

    # ── Fix 1: _record_a1_xyz — reset scale/rotation ────────────
    print(f"\n{CYAN}[1] Reset _scale/_rotation on A1 teach{RESET}")
    m = find_method(content, "_record_a1_xyz")
    if m and "v7.2.7-scalefix" not in m.group(0):
        body = m.group(0)
        # Find the logger.info line at the end and inject before it
        log_pat = re.compile(r'^(\s+)(logger\.info\(f"Taught A1)', re.MULTILINE)
        log_m = log_pat.search(body)
        if log_m:
            ind = log_m.group(1)
            inject = (
                f'{ind}# v7.2.7-scalefix: reset scale/rotation — only valid after corner is taught\n'
                f'{ind}self._scale = 1.0\n'
                f'{ind}self._rotation = 0.0\n'
                f'{ind}self._offset_x = 0\n'
                f'{ind}self._offset_y = 0\n'
            )
            abs_pos = m.start() + log_m.start()
            content = content[:abs_pos] + inject + content[abs_pos:]
            changed = True
            print(f"  {GREEN}✓ Added scale/rotation reset to _record_a1_xyz{RESET}")
        else:
            print(f"  {RED}✗ Could not find logger.info in _record_a1_xyz{RESET}")
    elif m:
        print(f"  {YELLOW}○ Already has scalefix marker{RESET}")
    else:
        print(f"  {RED}✗ _record_a1_xyz not found{RESET}")

    # ── Fix 2: Replace _estimate_well_position_um with guarded version
    print(f"\n{CYAN}[2] Guard _estimate against stale scale{RESET}")
    m2 = find_method(content, "_estimate_well_position_um")
    if m2:
        new_est = '''    def _estimate_well_position_um(self, well_name):
        """v7.2.7-scalefix: estimate well position — ignore scale/rotation until calibrated.

        After A1 is taught but BEFORE corner is taught, scale/rotation
        should be 1.0/0.0. Only apply non-identity transform if both
        A1 and corner have been taught AND scale is reasonable.
        """
        if self._taught_a1 is None or self._plate is None:
            return None
        try:
            wx, wy = self._plate.get_well_position(well_name)
            a1x, a1y = self._plate.get_well_position("A1")
            dx_mm = wx - a1x
            dy_mm = wy - a1y

            # Only apply scale/rotation if both points taught AND scale is sane
            scale = getattr(self, '_scale', 1.0)
            rot = getattr(self, '_rotation', 0.0)
            has_corner = getattr(self, '_taught_corner', None) is not None

            if has_corner and 0.8 < scale < 1.2 and abs(rot) < 10:
                # Apply calibrated transform
                if abs(rot) > 0.001:
                    import math as _m
                    rad = _m.radians(rot)
                    dx_mm, dy_mm = (dx_mm*_m.cos(rad) - dy_mm*_m.sin(rad),
                                    dx_mm*_m.sin(rad) + dy_mm*_m.cos(rad))
                dx_mm *= scale
                dy_mm *= scale
            elif has_corner and (scale < 0.8 or scale > 1.2):
                logger.warning(f"Ignoring stale scale={scale:.2f} (out of range 0.8-1.2)")

            tx = self._taught_a1[0] + dx_mm * 1000.0
            ty = self._taught_a1[1] + dy_mm * 1000.0
            logger.info(f"Est {well_name}: offset ({dx_mm:.2f},{dy_mm:.2f})mm, "
                        f"scale={scale:.4f}, rot={rot:.2f}° "
                        f"-> ({tx:.0f},{ty:.0f}) µm")
            return (tx, ty)
        except Exception as e:
            logger.warning(f"Cannot estimate {well_name}: {e}")
            return None

'''
        content = content[:m2.start()] + new_est + content[m2.end():]
        changed = True
        print(f"  {GREEN}✓ Replaced with guarded version{RESET}")
    else:
        print(f"  {RED}✗ _estimate_well_position_um not found{RESET}")

    # ── Fix 3: Also reset in _on_plate_changed ───────────────────
    print(f"\n{CYAN}[3] Reset scale on plate change{RESET}")
    if "_scale = 1.0" not in (content[content.find("def _on_plate_changed"):content.find("def _on_plate_changed")+500] if "def _on_plate_changed" in content else ""):
        m3 = find_method(content, "_on_plate_changed")
        if m3:
            body = m3.group(0)
            if "self._scale = 1.0" not in body:
                # Find: self._taught_a1 = None
                a1_none = re.search(r'^(\s+)(self\._taught_a1 = None)', body, re.MULTILINE)
                if a1_none:
                    ind = a1_none.group(1)
                    inject = (
                        f'{ind}self._scale = 1.0\n'
                        f'{ind}self._rotation = 0.0\n'
                    )
                    abs_pos = m3.start() + a1_none.end()
                    content = content[:abs_pos] + "\n" + inject + content[abs_pos:]
                    changed = True
                    print(f"  {GREEN}✓ Added scale reset to _on_plate_changed{RESET}")
                else:
                    print(f"  {YELLOW}○ _taught_a1 = None not found in method{RESET}")
            else:
                print(f"  {YELLOW}○ Scale reset already in _on_plate_changed{RESET}")
        else:
            print(f"  {RED}✗ _on_plate_changed not found{RESET}")
    else:
        print(f"  {YELLOW}○ Already has scale reset{RESET}")

    if not changed:
        print(f"\n  {YELLOW}No changes needed{RESET}")
        return 0

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
    shutil.copy2(path, path.with_suffix(f".bak_v727sr_{ts}"))
    path.write_text(content, encoding="utf-8")
    print(f"\n  {GREEN}✓ calibration.py written + AST OK{RESET}")
    return 0

if __name__ == "__main__":
    sys.exit(main())

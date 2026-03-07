#!/usr/bin/env python3
"""Fix _safe_navigate_to: use blocking absolute moves, no polling."""

import ast, re, sys, shutil
from pathlib import Path
from datetime import datetime

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
    print("  Fix _safe_navigate_to — absolute blocking moves")
    root = find_root()
    path = root / "gui" / "pages" / "calibration.py"
    content = path.read_text(encoding="utf-8")

    m = find_method(content, "_safe_navigate_to")
    if not m:
        print("  ✗ not found"); return 1

    new = '''    def _safe_navigate_to(self, target_x_um, target_y_um, target_z_mm=None):
        """v7.2.7-abs: Blocking absolute moves — Z up, XY travel, Z down.

        The Prior G command is blocking (returns R on arrival).
        No polling needed — just send commands in sequence.
        target_x_um/y_um are absolute stage coordinates in µm.
        """
        ctrl = self.controller
        safe_z = getattr(self, '_safe_z', None) or 0.0

        # Step 1: Raise Z to safe height (blocking on real hw)
        if ctrl.is_zp_connected:
            ctrl.move_z_absolute(safe_z, from_zero_ref=True)
            import time; time.sleep(0.3)  # brief settle for simulator

        # Step 2: Set travel speed and send absolute XY move
        if ctrl.is_xy_connected:
            if hasattr(ctrl, 'xy_stage') and ctrl.xy_stage:
                if hasattr(ctrl.xy_stage, 'set_speed_mm_s'):
                    ctrl.xy_stage.set_speed_mm_s(50.0)
                else:
                    ctrl.xy_stage.set_velocity(100)  # max speed for travel

            # Absolute move — G command blocks until stage arrives
            ctrl.move_xy_absolute(target_x_um, target_y_um, from_zero_ref=False)
            import time; time.sleep(0.3)  # brief settle for simulator

        # Step 3: Lower Z to approach height
        if ctrl.is_zp_connected:
            if target_z_mm is not None:
                ctrl.move_z_absolute(target_z_mm, from_zero_ref=True)
            elif getattr(self, '_top_z', None) is not None:
                approach = self._top_z + getattr(self, '_z_buffer_mm', 0.5)
                ctrl.move_z_absolute(approach, from_zero_ref=True)

        logger.info(f"Safe navigate to ({target_x_um:.0f}, {target_y_um:.0f}) µm")

'''
    content = content[:m.start()] + new + content[m.end():]

    try:
        ast.parse(content)
    except SyntaxError as e:
        print(f"  ✗ AST FAIL: line {e.lineno}: {e.msg}"); return 1

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    shutil.copy2(path, path.with_suffix(f".bak_v727abs_{ts}"))
    path.write_text(content, encoding="utf-8")
    print("  ✓ Replaced with blocking absolute move version")
    return 0

if __name__ == "__main__":
    sys.exit(main())

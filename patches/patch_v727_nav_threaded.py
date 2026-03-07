#!/usr/bin/env python3
"""Fix: run _safe_navigate_to in a thread, disable buttons during travel, 
   wait for simulator arrival."""

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
    print("  Threaded navigation + simulator wait")
    root = find_root()
    path = root / "gui" / "pages" / "calibration.py"
    content = path.read_text(encoding="utf-8")

    m = find_method(content, "_safe_navigate_to")
    if not m:
        print("  ✗ not found"); return 1

    new = '''    def _safe_navigate_to(self, target_x_um, target_y_um, target_z_mm=None):
        """v7.2.7-threaded: Run navigation in background thread.

        Disables nav buttons during travel to prevent spam.
        Handles both real hardware (G blocks) and simulator (poll for arrival).
        """
        import threading

        # Disable nav buttons
        for btn_attr in ['btn_go_corner_auto', 'btn_go_third', 'btn_safe_z',
                         'btn_top_z', 'btn_rec_a1']:
            btn = getattr(self, btn_attr, None)
            if btn and hasattr(btn, 'setEnabled'):
                btn.setEnabled(False)

        if hasattr(self, '_gen_status'):
            self._gen_status.setText("Navigating...")

        def _nav_worker():
            import time, math
            ctrl = self.controller
            safe_z = getattr(self, '_safe_z', None) or 0.0

            # Step 1: Raise Z
            if ctrl.is_zp_connected:
                ctrl.move_z_absolute(safe_z, from_zero_ref=True)
                # Wait for Z arrival (simulator is async)
                for _ in range(100):
                    time.sleep(0.05)
                    zp = ctrl.get_zp_position(cached=False)
                    if zp and zp[0] is not None:
                        cur_z = zp[0] - ctrl.zero_position.get("Z", 0)
                        if abs(cur_z - safe_z) < 0.2:
                            break

            # Step 2: Fast XY travel
            if ctrl.is_xy_connected:
                if hasattr(ctrl, 'xy_stage') and ctrl.xy_stage:
                    if hasattr(ctrl.xy_stage, 'set_speed_mm_s'):
                        ctrl.xy_stage.set_speed_mm_s(50.0)
                    else:
                        ctrl.xy_stage.set_velocity(100)

                ctrl.move_xy_absolute(target_x_um, target_y_um, from_zero_ref=False)

                # Wait for XY arrival — needed for simulator
                # Real hardware: G command already blocked, so this exits immediately
                xy = ctrl.get_xy_position(cached=False)
                if xy[0] is not None:
                    dist = math.sqrt((xy[0] - target_x_um)**2 +
                                     (xy[1] - target_y_um)**2)
                    timeout = max(2.0, dist / 5000.0)  # dist in µm / speed in µm/s
                else:
                    timeout = 30.0

                t0 = time.monotonic()
                while time.monotonic() - t0 < timeout:
                    time.sleep(0.1)
                    xy = ctrl.get_xy_position(cached=False)
                    if xy[0] is not None:
                        if (abs(xy[0] - target_x_um) < 100 and
                                abs(xy[1] - target_y_um) < 100):
                            break

            # Step 3: Lower Z
            if ctrl.is_zp_connected:
                if target_z_mm is not None:
                    ctrl.move_z_absolute(target_z_mm, from_zero_ref=True)
                elif getattr(self, '_top_z', None) is not None:
                    approach = self._top_z + getattr(self, '_z_buffer_mm', 0.5)
                    ctrl.move_z_absolute(approach, from_zero_ref=True)
                # Wait for Z
                for _ in range(100):
                    time.sleep(0.05)
                    zp = ctrl.get_zp_position(cached=False)
                    if zp and zp[0] is not None:
                        break

            logger.info(f"Safe navigate to ({target_x_um:.0f}, {target_y_um:.0f}) µm")

            # Re-enable buttons (must be done via QTimer for thread safety)
            try:
                from PySide6.QtCore import QTimer
                QTimer.singleShot(0, self._reenable_nav_buttons)
            except Exception:
                self._reenable_nav_buttons()

        thread = threading.Thread(target=_nav_worker, daemon=True)
        thread.start()

    def _reenable_nav_buttons(self):
        """Re-enable navigation buttons after travel completes."""
        for btn_attr in ['btn_go_corner_auto', 'btn_go_third', 'btn_safe_z',
                         'btn_top_z', 'btn_rec_a1']:
            btn = getattr(self, btn_attr, None)
            if btn and hasattr(btn, 'setEnabled'):
                btn.setEnabled(True)
        if hasattr(self, '_gen_status'):
            self._gen_status.setText("Ready")

'''
    content = content[:m.start()] + new + content[m.end():]

    try:
        ast.parse(content)
    except SyntaxError as e:
        print(f"  ✗ AST FAIL: line {e.lineno}: {e.msg}"); return 1

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    shutil.copy2(path, path.with_suffix(f".bak_v727thr_{ts}"))
    path.write_text(content, encoding="utf-8")
    print("  ✓ Replaced with threaded version + button disable")
    return 0

if __name__ == "__main__":
    sys.exit(main())

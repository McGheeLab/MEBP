#!/usr/bin/env python3
"""
patch_fix_monitor_poll_methods.py  (v7.3.1)
Adds the three missing position-poll methods to PrintMonitorPage:
  _start_position_poll()
  _stop_position_poll()
  _poll_needle_position()
These were wired by patch_fix_monitor_timer_wire.py but never written
because patch_fix_monitor_live_updates.py bailed before saving.
"""
import ast, sys, shutil, re
from pathlib import Path
from datetime import datetime

GREEN = "\033[92m"; RED = "\033[91m"; YELLOW = "\033[93m"; CYAN = "\033[96m"; RESET = "\033[0m"
GUARD = "v7.3.1-pollmethods"

def find_root():
    for p in [Path.cwd(), Path(__file__).parent]:
        for c in [p, p.parent, p.parent.parent]:
            if (c / "SupportClasses").is_dir() and (c / "gui").is_dir():
                return c.resolve()
    raise RuntimeError("Cannot locate MEBP root")

def ast_ok(text, tag):
    try:
        ast.parse(text); return True
    except SyntaxError as e:
        print(f"  {RED}AST FAIL after {tag}: {e}{RESET}")
        lines = text.splitlines()
        ln = e.lineno or 0
        for i in range(max(0, ln - 3), min(len(lines), ln + 3)):
            print(f"    {i+1:4d}: {repr(lines[i])}")
        return False

POLL_METHODS = '''\
    # ── Live position polling (v7.3.1-pollmethods) ───────────────

    def _start_position_poll(self) -> None:
        """Start 250 ms position poll timer."""
        if not hasattr(self, "_pos_poll_timer"):
            from PySide6.QtCore import QTimer
            self._pos_poll_timer = QTimer(self)
            self._pos_poll_timer.setInterval(250)
            self._pos_poll_timer.timeout.connect(self._poll_needle_position)
        self._pos_poll_timer.start()

    def _stop_position_poll(self) -> None:
        """Stop position poll timer."""
        if hasattr(self, "_pos_poll_timer"):
            self._pos_poll_timer.stop()

    def _poll_needle_position(self) -> None:
        """Poll controller XY+Z and push to trajectory view."""
        ctrl = self._controller
        if ctrl is None:
            return
        try:
            xy = ctrl.get_xy_position()
            zp = ctrl.get_zp_position()
            if isinstance(xy, dict):
                x_s = xy.get("x", 0) or 0
                y_s = xy.get("y", 0) or 0
            elif xy and len(xy) >= 2:
                x_s, y_s = xy[0], xy[1]
            else:
                return
            mpm = self._microsteps_per_micron or 1.0
            x_mm = float(x_s) / (mpm * 1000.0)
            y_mm = float(y_s) / (mpm * 1000.0)
            if isinstance(zp, dict):
                z_mm = float(zp.get("Z", 0) or 0)
            elif zp and len(zp) >= 1:
                z_mm = float(zp[0])
            else:
                z_mm = None
            self.update_needle_position(x_mm, y_mm, z_mm)
        except Exception:
            pass  # Never crash the GUI over a position read failure

'''

def main():
    root   = find_root()
    target = root / "gui" / "pages" / "print_monitor.py"
    print(f"{CYAN}Target: {target}{RESET}")
    if not target.exists():
        print(f"  {RED}File not found{RESET}"); sys.exit(1)

    content = target.read_text(encoding="utf-8")

    if GUARD in content:
        print(f"  {YELLOW}SKIP: already applied{RESET}"); return

    if "_start_position_poll" in content:
        print(f"  {YELLOW}SKIP: methods already present{RESET}"); return

    # Insert before _advance_queue (which always exists)
    anchor = "\n    def _advance_queue(self)"
    if anchor not in content:
        # Fallback: before on_print_progress
        anchor = "\n    def on_print_progress(self"
    if anchor not in content:
        print(f"  {RED}MISS: no suitable anchor found{RESET}"); sys.exit(1)

    idx = content.index(anchor)
    content = content[:idx] + "\n" + POLL_METHODS + content[idx:]

    if not ast_ok(content, "poll methods"):
        sys.exit(1)

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    shutil.copy2(target, target.with_suffix(f".bak_{GUARD}_{ts}"))
    target.write_text(content, encoding="utf-8")
    print(f"  {GREEN}Poll methods added to PrintMonitorPage{RESET}")

if __name__ == "__main__":
    main()

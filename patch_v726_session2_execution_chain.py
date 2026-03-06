#!/usr/bin/env python3
"""
MEBP v7.2.6 Session 2 — Fix the Execution Chain.

Patches:
  1. app.py: Fix _on_monitor_start() — load_job() then start()
  2. app.py: Fix _wire_print_manager_to_monitor() — thread-safe signal bridge
  3. print_monitor.py: Add on_status_update() for live position polling
  4. print_monitor.py: Fix on_print_state_changed() queue advance

Follows PATCHING_BEST_PRACTICES.md.
"""

import ast
import re
import sys
import shutil
from pathlib import Path
from datetime import datetime

# ── Terminal colours ─────────────────────────────────────────────
G = "\033[92m"; Y = "\033[93m"; R = "\033[91m"; X = "\033[0m"; B = "\033[1m"
_ok = 0; _skip = 0; _miss = 0

def ok(msg):
    global _ok; _ok += 1; print(f"  {G}✓{X} {msg}")
def skip(msg):
    global _skip; _skip += 1; print(f"  {Y}○{X} SKIP: {msg}")
def miss(msg):
    global _miss; _miss += 1; print(f"  {R}✗{X} MISS: {msg}")

def find_root(start=None):
    if len(sys.argv) > 1:
        p = Path(sys.argv[1])
        if (p / "SupportClasses").is_dir() and (p / "gui").is_dir():
            return p
    for c in [Path.cwd(), Path(__file__).resolve().parent.parent]:
        if (c / "SupportClasses").is_dir() and (c / "gui").is_dir():
            return c
    print(f"{R}ERROR: Cannot find MEBP root.{X}")
    sys.exit(1)

def safe_read(path):
    return path.read_text(encoding="utf-8") if path.exists() else ""

def safe_write(path, content, label):
    try:
        ast.parse(content)
    except SyntaxError as e:
        print(f"  {R}AST FAIL on {path.name} ({label}): {e}{X}")
        return False
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    backup = path.with_suffix(f".bak_v726s2_{ts}")
    if path.exists():
        shutil.copy2(path, backup)
    path.write_text(content, encoding="utf-8")
    print(f"  {G}WROTE{X}: {path.name} ({label})")
    return True

def find_method(content, name, indent=4):
    prefix = " " * indent
    pattern = re.compile(
        rf'^({prefix}def {re.escape(name)}\(self.*?\n)'
        rf'(.*?)'
        rf'(?=\n{prefix}def |\nclass |\Z)',
        re.DOTALL | re.MULTILINE
    )
    return pattern.search(content)


# ═══════════════════════════════════════════════════════════════════
#  PATCH 1: app.py — Fix execution control wiring
# ═══════════════════════════════════════════════════════════════════

def patch_app(root):
    print(f"\n{B}── Patching gui/app.py ──{X}")
    path = root / "gui" / "app.py"
    content = safe_read(path)
    if not content:
        miss("app.py not found")
        return
    changed = False

    # ── 2A. Fix _on_monitor_start() ──────────────────────────────
    marker_2a = "v7.2.6: load_job then start"
    if marker_2a not in content:
        m = find_method(content, "_on_monitor_start")
        if m:
            new_method = '''    def _on_monitor_start(self, job):
        """Monitor requested start. v7.2.6: load_job then start.

        PrintManager.start() takes no arguments — the job must be
        loaded first via load_job(). The old code called start(job)
        which raises TypeError.
        """
        setup_page = self._page_widgets[4]
        if not hasattr(setup_page, "print_manager"):
            logger.error("No print_manager on setup page")
            return
        pm = setup_page.print_manager
        try:
            pm.load_job(job)
            pm.start()
            logger.info(f"Print started: {job.name}")
        except Exception as exc:
            logger.error(f"PrintManager start failed: {exc}", exc_info=True)

'''
            content = content[:m.start()] + new_method + content[m.end():]
            ok("Fixed _on_monitor_start() — load_job() then start()")
            changed = True
        else:
            miss("_on_monitor_start() not found")
    else:
        skip("_on_monitor_start() already fixed")

    # ── 2B. Fix _wire_print_manager_to_monitor() ─────────────────
    marker_2b = "v7.2.6: Thread-safe signal bridge with QueuedConnection"
    if marker_2b not in content:
        m = find_method(content, "_wire_print_manager_to_monitor")
        if m:
            new_method = '''    def _wire_print_manager_to_monitor(self):
        """v7.2.6: Thread-safe signal bridge with QueuedConnection.

        PrintManager callbacks fire from a daemon thread.
        Direct widget calls from threads cause SIGSEGV on macOS/PySide6.
        We bounce through a QObject signal bridge so all GUI updates
        execute on the main thread.
        """
        setup_page = self._page_widgets[4]
        monitor_page = self._page_widgets[5]

        if not hasattr(setup_page, "print_manager"):
            logger.warning("_wire_print_manager_to_monitor: no print_manager")
            return

        pm = setup_page.print_manager

        # Create signal bridge owned by app.py
        try:
            from gui.pages.print_setup import PrintSignalBridge
        except ImportError:
            from PySide6.QtCore import QObject, Signal as _Sig
            class PrintSignalBridge(QObject):
                progress_signal = _Sig(int, int, str)
                state_signal = _Sig(object)

        self._print_signal_bridge = PrintSignalBridge(self)
        bridge = self._print_signal_bridge

        # Connect bridge → monitor with QueuedConnection (thread-safe)
        from PySide6.QtCore import Qt
        if hasattr(monitor_page, "on_print_progress"):
            bridge.progress_signal.connect(
                monitor_page.on_print_progress,
                type=Qt.ConnectionType.QueuedConnection,
            )
        if hasattr(monitor_page, "on_print_state_changed"):
            bridge.state_signal.connect(
                monitor_page.on_print_state_changed,
                type=Qt.ConnectionType.QueuedConnection,
            )

        # Wire PrintManager callbacks → emit bridge signals
        # These lambdas may fire from the print thread — .emit() is thread-safe
        original_progress = pm.on_progress
        original_state = pm.on_state_changed

        def safe_progress(step, total, msg):
            try:
                if original_progress:
                    original_progress(step, total, msg)
            except Exception:
                pass
            try:
                bridge.progress_signal.emit(step, total, msg)
            except Exception:
                pass

        def safe_state(state):
            try:
                if original_state:
                    original_state(state)
            except Exception:
                pass
            try:
                bridge.state_signal.emit(state)
            except Exception:
                pass

        pm.on_progress = safe_progress
        pm.on_state_changed = safe_state
        logger.info("Print manager -> monitor wired via thread-safe signal bridge")

'''
            content = content[:m.start()] + new_method + content[m.end():]
            ok("Fixed _wire_print_manager_to_monitor() — thread-safe bridge")
            changed = True
        else:
            miss("_wire_print_manager_to_monitor() not found")
    else:
        skip("_wire_print_manager_to_monitor() already fixed")

    if changed:
        safe_write(path, content, "Session 2 execution chain fixes")
    else:
        print(f"  {Y}No changes needed{X}")


# ═══════════════════════════════════════════════════════════════════
#  PATCH 2: print_monitor.py — Live position + queue advance
# ═══════════════════════════════════════════════════════════════════

def patch_monitor(root):
    print(f"\n{B}── Patching gui/pages/print_monitor.py ──{X}")
    path = root / "gui" / "pages" / "print_monitor.py"
    content = safe_read(path)
    if not content:
        miss("print_monitor.py not found")
        return
    changed = False

    # ── 2C. Add/replace on_status_update() ───────────────────────
    marker_2c = "v7.2.6: Live position polling during print execution"
    if marker_2c not in content:
        m = find_method(content, "on_status_update")
        if m:
            # Replace existing
            new_method = _get_on_status_update_method()
            content = content[:m.start()] + new_method + content[m.end():]
            ok("Replaced on_status_update() with live position polling")
            changed = True
        else:
            # Insert after on_print_state_changed
            m2 = find_method(content, "on_print_state_changed")
            if m2:
                new_method = _get_on_status_update_method()
                content = content[:m2.end()] + "\n" + new_method + content[m2.end():]
                ok("Added on_status_update() after on_print_state_changed()")
                changed = True
            else:
                # Insert before _connect_signals or any method
                m3 = find_method(content, "_connect_signals")
                if m3:
                    new_method = _get_on_status_update_method()
                    content = content[:m3.start()] + new_method + content[m3.start():]
                    ok("Added on_status_update() before _connect_signals()")
                    changed = True
                else:
                    miss("Cannot find insertion point for on_status_update()")
    else:
        skip("on_status_update() already has live position polling")

    # ── 2D. Fix on_print_state_changed() to advance queue ────────
    marker_2d = "v7.2.6: Queue advance on completion"
    if marker_2d not in content:
        m = find_method(content, "on_print_state_changed")
        if m:
            body = m.group(0)
            # Check if _advance_queue is already called
            if "_advance_queue" in body:
                skip("on_print_state_changed already calls _advance_queue")
            else:
                new_method = _get_on_print_state_changed_method()
                content = content[:m.start()] + new_method + content[m.end():]
                ok("Fixed on_print_state_changed() — queue advance on completion")
                changed = True
        else:
            miss("on_print_state_changed() not found")
    else:
        skip("on_print_state_changed() already patched")

    # ── 2E. Ensure _get_controller() helper exists ───────────────
    marker_2e = "v7.2.6: Helper to get controller reference"
    if marker_2e not in content:
        if "def _get_controller" not in content:
            # Insert after __init__ or at class level
            m = find_method(content, "__init__")
            if m:
                helper = '''    def _get_controller(self):
        """v7.2.6: Helper to get controller reference.

        The controller may be stored as _controller or controller.
        """
        ctrl = getattr(self, '_controller', None)
        if ctrl is None:
            ctrl = getattr(self, 'controller', None)
        return ctrl

'''
                content = content[:m.end()] + "\n" + helper + content[m.end():]
                ok("Added _get_controller() helper")
                changed = True
            else:
                miss("Cannot find __init__ to insert _get_controller()")
        else:
            skip("_get_controller() already exists")
    else:
        skip("_get_controller() marker already present")

    if changed:
        safe_write(path, content, "Session 2 monitor fixes")
    else:
        print(f"  {Y}No changes needed{X}")


def _get_on_status_update_method():
    return '''    def on_status_update(self):
        """v7.2.6: Live position polling during print execution.

        Called by MainWindow timer (~300ms). Updates live visualization
        with current needle position during active printing.
        """
        from SupportClasses.PrintManager import PrintState

        if self._print_state != PrintState.RUNNING:
            return

        controller = self._get_controller()
        if controller is None:
            return

        try:
            # Read cached positions (fast, no serial I/O)
            xy_pos = None
            zp_pos = None
            if getattr(controller, 'is_xy_connected', False):
                xy_pos = controller.get_xy_position(cached=True)
            if getattr(controller, 'is_zp_connected', False):
                zp_pos = controller.get_zp_position(cached=True)
        except Exception:
            return

        # Update trajectory view with current position
        if xy_pos and hasattr(self, 'trajectory_view'):
            try:
                x, y = xy_pos[0], xy_pos[1]
                if x is not None and y is not None:
                    self.trajectory_view.set_current_position(x, y)
            except Exception:
                pass

        # Update plate overview — highlight active well
        if hasattr(self, 'plate_view') and hasattr(self, '_current_well_name'):
            try:
                self.plate_view.set_active_well(self._current_well_name)
            except Exception:
                pass

        # Update position readout labels
        if hasattr(self, '_progress_labels'):
            try:
                if xy_pos and xy_pos[0] is not None:
                    pos_text = f"XY: ({xy_pos[0]:.0f}, {xy_pos[1]:.0f})"
                    if zp_pos:
                        z_val = zp_pos.get('Z', zp_pos.get(0, None))
                        if z_val is not None:
                            pos_text += f"  Z: {z_val:.0f}"
                    lbl = self._progress_labels.get("position")
                    if lbl:
                        lbl.setText(pos_text)
            except Exception:
                pass

'''


def _get_on_print_state_changed_method():
    return '''    def on_print_state_changed(self, state) -> None:
        """Called when print state changes. v7.2.6: Queue advance on completion."""
        from SupportClasses.PrintManager import PrintState

        self._print_state = state

        # Update state label
        state_names = {
            PrintState.IDLE: ("IDLE", "overlay0"),
            PrintState.RUNNING: ("RUNNING", "green"),
            PrintState.PAUSED: ("PAUSED", "yellow"),
            PrintState.COMPLETED: ("COMPLETED", "green"),
            PrintState.ABORTED: ("ABORTED", "red"),
            PrintState.ERROR: ("ERROR", "red"),
        }
        name, color_key = state_names.get(state, (str(state), "overlay0"))

        if hasattr(self, 'state_label'):
            from gui.styles import COLORS
            self.state_label.setText(name)
            self.state_label.setStyleSheet(
                f"color: {COLORS.get(color_key, '#a6adc8')}; "
                f"font-size: 14px; font-weight: bold;")

        # Update button states
        if hasattr(self, 'btn_pause'):
            if state == PrintState.RUNNING:
                self.btn_pause.setText("\\u23f8 Pause")
                self.btn_pause.setEnabled(True)
            elif state == PrintState.PAUSED:
                self.btn_pause.setText("\\u25b6 Resume")
                self.btn_pause.setEnabled(True)
            else:
                self.btn_pause.setText("\\u23f8 Pause")
                self.btn_pause.setEnabled(False)

        if hasattr(self, 'btn_abort'):
            self.btn_abort.setEnabled(
                state in (PrintState.RUNNING, PrintState.PAUSED))

        # Track print timing
        import time
        if state == PrintState.RUNNING and self._print_start_time is None:
            self._print_start_time = time.time()
        elif state in (PrintState.COMPLETED, PrintState.ABORTED, PrintState.ERROR):
            self._print_start_time = None

        # Update progress bar
        if hasattr(self, 'progress_bar'):
            if state == PrintState.COMPLETED:
                self.progress_bar.setValue(100)
            elif state in (PrintState.ABORTED, PrintState.ERROR):
                pass  # Keep current value

        # Queue advance on completion
        if state == PrintState.COMPLETED:
            if hasattr(self, '_advance_queue'):
                self._advance_queue()

        # Refresh job queue UI (enables/disables start button)
        if hasattr(self, '_refresh_job_queue_ui'):
            self._refresh_job_queue_ui()

'''


# ═══════════════════════════════════════════════════════════════════
#  MAIN
# ═══════════════════════════════════════════════════════════════════

def main():
    print(f"\n{B}{'=' * 60}")
    print(f"  MEBP v7.2.6 Session 2 — Execution Chain Fix")
    print(f"{'=' * 60}{X}\n")

    root = find_root()
    print(f"  Project root: {root}\n")

    patch_app(root)
    patch_monitor(root)

    print(f"\n{B}── Summary ──{X}")
    print(f"  {G}OK:   {_ok}{X}")
    print(f"  {Y}SKIP: {_skip}{X}")
    print(f"  {R}MISS: {_miss}{X}")

    if _miss > 0:
        print(f"\n  {R}⚠ Some patches did not apply. Review output above.{X}")
        sys.exit(1)
    else:
        print(f"\n  {G}✓ Session 2 complete. Ready for Session 3.{X}")


if __name__ == "__main__":
    main()

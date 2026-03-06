#!/usr/bin/env python3
"""
patch_fix_print_thread_segfault.py
Fix: segfault when print starts.

Root cause: _wire_print_manager_to_monitor() in app.py sets PrintManager
callbacks that directly call monitor widget methods from the print
execution thread. On macOS PySide6, touching Qt objects from a non-main
thread causes an immediate SIGSEGV.

Fix: replace the direct-callback wiring with a PrintSignalBridge so all
     GUI updates are safely bounced to the main thread via Qt signals.

PrintSignalBridge already exists in print_setup.py — we create one
instance in app.py and reuse it.
"""
import ast, re, sys, shutil
from pathlib import Path
from datetime import datetime

GREEN = "\033[92m"; RED = "\033[91m"; YELLOW = "\033[93m"
CYAN  = "\033[96m"; RESET = "\033[0m"

def find_root() -> Path:
    for p in [Path.cwd(), Path(__file__).parent]:
        for c in [p, p.parent, p.parent.parent]:
            if (c / "SupportClasses").is_dir() and (c / "gui").is_dir():
                return c.resolve()
    raise RuntimeError("Cannot locate MEBP project root")

def find_method(content: str, name: str):
    pat = re.compile(
        r'^(    def ' + re.escape(name) + r'\(self.*?\n)'
        r'(.*?)'
        r'(?=\n    def |\nclass |\Z)',
        re.DOTALL | re.MULTILINE
    )
    return pat.search(content)

# ── Replacement for _wire_print_manager_to_monitor ───────────────
NEW_WIRE_METHOD = '''    def _wire_print_manager_to_monitor(self):
        """
        v7.3.1: Wire PrintManager progress/state to monitor via Qt signals.

        CRITICAL: PrintManager callbacks fire from a daemon thread.
        Direct widget calls from threads cause SIGSEGV on macOS/PySide6.
        We bounce through a QObject signal bridge so all GUI updates
        execute on the main thread.
        """
        setup_page  = self._page_widgets[4]
        monitor_page = self._page_widgets[5]

        if not hasattr(setup_page, "print_manager"):
            logger.warning("_wire_print_manager_to_monitor: no print_manager on setup page")
            return

        pm = setup_page.print_manager

        # ── Create a signal bridge owned by app.py ────────────────
        try:
            from gui.pages.print_setup import PrintSignalBridge
        except ImportError:
            # Fallback: define minimal bridge inline
            from PySide6.QtCore import QObject, Signal as _Signal
            class PrintSignalBridge(QObject):
                progress_signal = _Signal(int, int, str)
                state_signal    = _Signal(object)

        self._print_signal_bridge = PrintSignalBridge(self)

        # Connect bridge signals → monitor (runs on main thread) ──
        if hasattr(monitor_page, "on_print_progress"):
            self._print_signal_bridge.progress_signal.connect(
                monitor_page.on_print_progress,
                type=__import__("PySide6.QtCore", fromlist=["Qt"]).Qt.QueuedConnection,
            )
        if hasattr(monitor_page, "on_print_state_changed"):
            self._print_signal_bridge.state_signal.connect(
                monitor_page.on_print_state_changed,
                type=__import__("PySide6.QtCore", fromlist=["Qt"]).Qt.QueuedConnection,
            )

        # Wire PrintManager callbacks → emit bridge signals ────────
        # These lambdas may be called from the print thread — they only
        # call .emit() which is thread-safe in Qt.
        bridge = self._print_signal_bridge

        original_progress  = pm.on_progress
        original_state     = pm.on_state_changed

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

        pm.on_progress       = safe_progress
        pm.on_state_changed  = safe_state
        logger.info("Print manager → monitor wired via thread-safe signal bridge")

'''

def main():
    root   = find_root()
    target = root / "gui" / "app.py"
    print(f"{CYAN}Target: {target}{RESET}")

    if not target.exists():
        print(f"  {RED}✗ File not found{RESET}")
        sys.exit(1)

    content = target.read_text(encoding="utf-8")

    guard = "v7.3.1: Wire PrintManager progress/state to monitor via Qt signals"
    if guard in content:
        print(f"  {YELLOW}○ SKIP: already applied{RESET}")
        return

    m = find_method(content, "_wire_print_manager_to_monitor")
    if not m:
        print(f"  {RED}✗ MISS: _wire_print_manager_to_monitor not found in app.py{RESET}")
        sys.exit(1)

    new_content = content[:m.start()] + NEW_WIRE_METHOD + content[m.end():]

    try:
        ast.parse(new_content)
    except SyntaxError as e:
        print(f"  {RED}✗ AST FAIL: {e}{RESET}")
        sys.exit(1)

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    shutil.copy2(target, target.with_suffix(f".bak_v731seg_{ts}"))
    target.write_text(new_content, encoding="utf-8")
    print(f"  {GREEN}✓ _wire_print_manager_to_monitor replaced with thread-safe version{RESET}")
    print(f"    PrintManager callbacks now emit Qt signals → no direct widget calls from thread")

if __name__ == "__main__":
    main()

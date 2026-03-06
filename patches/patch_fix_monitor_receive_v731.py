#!/usr/bin/env python3
"""
patch_fix_monitor_receive_v731.py
Fix PrintMonitorPage.receive_job crashing silently because _job_queue,
_current_job, and related attributes were never initialized in __init__.

Strategy: replace receive_job with a defensive version that lazy-inits
all required attributes before using them.
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

NEW_RECEIVE_JOB = '''    def receive_job(self, job) -> None:
        """v7.3.1: Receive a PrintJob — lazy-init queue attrs if missing."""
        # Lazy-init queue attributes in case __init__ patch was not applied
        if not hasattr(self, "_job_queue") or self._job_queue is None:
            self._job_queue = []
        if not hasattr(self, "_current_job"):
            self._current_job = None

        self._job_queue.append(job)
        logger.info(
            f"Job received: {job.name} ({len(self._job_queue)} in queue)")

        if self._current_job is None:
            self._current_job = job

        # Refresh queue UI
        self._refresh_job_queue_ui()

        # Update progress labels if they exist
        if hasattr(self, "_progress_labels") and self._progress_labels:
            try:
                self._progress_labels["job_name"].setText(job.name)
                total = getattr(job, "total_steps", 0)
                self._progress_labels["step_info"].setText(f"0 / {total:,}")
            except Exception:
                pass

        logger.info(f"Monitor ready — job '{job.name}' queued.")

'''

NEW_REFRESH_QUEUE_UI = '''    def _refresh_job_queue_ui(self) -> None:
        """v7.3.1: Update job queue list — safe against missing attrs."""
        if not hasattr(self, "_job_queue"):
            self._job_queue = []
        if not hasattr(self, "_current_job"):
            self._current_job = None

        if hasattr(self, "_queue_list") and self._queue_list is not None:
            try:
                self._queue_list.clear()
                for job in self._job_queue:
                    prefix = "▶ " if job is self._current_job else "  "
                    steps = getattr(job, "total_steps", "?")
                    self._queue_list.addItem(
                        f"{prefix}{job.name}  [{steps} steps]")
            except Exception as exc:
                logger.error(f"_refresh_job_queue_ui error: {exc}")

        if hasattr(self, "_ctx_btn_start") and self._ctx_btn_start is not None:
            try:
                from SupportClasses.PrintManager import PrintState
                has_job = self._current_job is not None
                idle = getattr(self, "_print_state", PrintState.IDLE) in (
                    PrintState.IDLE, PrintState.COMPLETED,
                    PrintState.ABORTED, PrintState.ERROR,
                )
                self._ctx_btn_start.setEnabled(has_job and idle)
            except Exception:
                pass

'''

def main():
    root   = find_root()
    target = root / "gui" / "pages" / "print_monitor.py"
    print(f"{CYAN}Target: {target}{RESET}")

    if not target.exists():
        print(f"  {RED}✗ File not found{RESET}")
        sys.exit(1)

    content = target.read_text(encoding="utf-8")
    changed = False

    # ── 1. Replace receive_job ────────────────────────────────────
    guard1 = "v7.3.1: Receive a PrintJob — lazy-init"
    if guard1 not in content:
        m = find_method(content, "receive_job")
        if m:
            content = content[:m.start()] + NEW_RECEIVE_JOB + content[m.end():]
            print(f"  {GREEN}✓ Replaced receive_job{RESET}")
            changed = True
        else:
            print(f"  {RED}✗ MISS: receive_job not found{RESET}")
            sys.exit(1)
    else:
        print(f"  {YELLOW}○ SKIP: receive_job already fixed{RESET}")

    # ── 2. Replace _refresh_job_queue_ui ─────────────────────────
    guard2 = "v7.3.1: Update job queue list — safe against"
    if guard2 not in content:
        m = find_method(content, "_refresh_job_queue_ui")
        if m:
            content = content[:m.start()] + NEW_REFRESH_QUEUE_UI + content[m.end():]
            print(f"  {GREEN}✓ Replaced _refresh_job_queue_ui{RESET}")
            changed = True
        else:
            print(f"  {YELLOW}○ SKIP: _refresh_job_queue_ui not found (non-fatal){RESET}")
    else:
        print(f"  {YELLOW}○ SKIP: _refresh_job_queue_ui already fixed{RESET}")

    if not changed:
        return

    try:
        ast.parse(content)
    except SyntaxError as e:
        print(f"  {RED}✗ AST FAIL: {e}{RESET}")
        sys.exit(1)

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    shutil.copy2(target, target.with_suffix(f".bak_v731mon_{ts}"))
    target.write_text(content, encoding="utf-8")
    print(f"  {GREEN}✓ print_monitor.py patched{RESET}")

if __name__ == "__main__":
    main()

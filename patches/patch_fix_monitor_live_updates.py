#!/usr/bin/env python3
"""
patch_fix_monitor_live_updates.py  (v7.3.1)
Fixes two issues in gui/pages/print_monitor.py:

  A. _advance_queue() NameError crash
     Orphaned Recording Browser UI code was accidentally left inside
     _advance_queue() after the queue logic.  Remove it so the method
     ends cleanly after  logger.info("Job queue empty").

  B. No live updates during print
     1. Add a 250 ms QTimer that polls controller XY+Z position while
        RUNNING and calls update_needle_position().
     2. Parse well_name from the progress message so the "Well:" label
        updates in real time (PrintManager emits "[step/total] label"
        where label may contain well identifiers).
     3. Start timer on RUNNING, stop on COMPLETED/ABORTED/ERROR.
"""
import ast, re, sys, shutil
from pathlib import Path
from datetime import datetime

GREEN  = "\033[92m"; RED   = "\033[91m"; YELLOW = "\033[93m"
CYAN   = "\033[96m"; RESET = "\033[0m"

GUARD = "v7.3.1-live"

def find_root() -> Path:
    for p in [Path.cwd(), Path(__file__).parent]:
        for c in [p, p.parent, p.parent.parent]:
            if (c / "SupportClasses").is_dir() and (c / "gui").is_dir():
                return c.resolve()
    raise RuntimeError("Cannot locate MEBP project root")

def ast_ok(content: str, tag: str) -> bool:
    try:
        ast.parse(content)
        return True
    except SyntaxError as e:
        print(f"  {RED}✗ AST FAIL after {tag}: {e}{RESET}")
        return False

# ── New / replacement method bodies ──────────────────────────────

# Replacement _advance_queue — no orphaned code
ADVANCE_QUEUE_REPLACEMENT = '''\
    def _advance_queue(self) -> None:
        """
        v7.2.3: After a job completes, advance to next in queue.

        Called from on_print_state_changed when state becomes COMPLETED.
        """
        if self._current_job in self._job_queue:
            self._job_queue.remove(self._current_job)

        if self._job_queue:
            self._current_job = self._job_queue[0]
            self._refresh_job_queue_ui()
            logger.info(f"Queue advanced: next job = {self._current_job.name}")
        else:
            self._current_job = None
            self._refresh_job_queue_ui()
            logger.info("Job queue empty")
'''

# Position-polling helpers to inject before _advance_queue
POSITION_POLL_METHODS = '''\
    # ── Live position polling (v7.3.1-live) ──────────────────────

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
            # XY comes back as (x_steps, y_steps) or dict
            if isinstance(xy, dict):
                x_s = xy.get("x", 0) or 0
                y_s = xy.get("y", 0) or 0
            elif xy and len(xy) >= 2:
                x_s, y_s = xy[0], xy[1]
            else:
                return
            # Convert microsteps → mm  (microsteps_per_micron × 1000 = steps/mm)
            mpm = self._microsteps_per_micron or 1.0
            x_mm = float(x_s) / (mpm * 1000.0)
            y_mm = float(y_s) / (mpm * 1000.0)
            # Z from ZP stage (already in mm)
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

# Replacement on_print_progress with well-name parsing
ON_PRINT_PROGRESS_REPLACEMENT = '''\
    def on_print_progress(self, step: int, total: int, message: str) -> None:
        """
        v7.3.1: Simplified progress callback (compatible with PrintManager).

        Parses well name from message where possible so the Well: label
        updates in real time without requiring PrintManager changes.
        """
        # Try to extract a well name like A1, B3, H12 from the message
        well_name = ""
        if message:
            m = re.search(r"\\b([A-H])([1-9]|1[0-2])\\b", message)
            if m:
                well_name = m.group(0)

        self.on_progress_update(
            current_step=step,
            total_steps=total,
            message=message,
            job_name=self._current_job.name if self._current_job else "",
            well_name=well_name,
        )
'''

def main():
    root   = find_root()
    target = root / "gui" / "pages" / "print_monitor.py"
    print(f"{CYAN}Target: {target}{RESET}")
    if not target.exists():
        print(f"  {RED}✗ File not found{RESET}"); sys.exit(1)

    content = target.read_text(encoding="utf-8")

    if GUARD in content:
        print(f"  {YELLOW}○ SKIP: already applied{RESET}"); return

    changed = False

    # ── A. Replace _advance_queue to remove orphaned UI code ─────
    # The method starts cleanly; the orphaned blob starts after
    # logger.info("Job queue empty") and ends at the first `def ` that
    # follows OR at `return widget` (whichever comes first).
    adv_pattern = re.compile(
        r'( {4}def _advance_queue\(self\).*?'          # method start
        r'logger\.info\("Job queue empty"\))'           # last clean line
        r'.*?'                                           # orphaned code
        r'(?=\n {4}def )',                               # stop before next method
        re.DOTALL,
    )
    m = adv_pattern.search(content)
    if m:
        content = content[:m.start()] + ADVANCE_QUEUE_REPLACEMENT + content[m.end():]
        if not ast_ok(content, "_advance_queue fix"):
            sys.exit(1)
        print(f"  {GREEN}✓ _advance_queue orphaned code removed{RESET}")
        changed = True
    elif "def _advance_queue" not in content:
        print(f"  {RED}✗ MISS: _advance_queue not found{RESET}"); sys.exit(1)
    else:
        # Method already clean?
        adv_body = re.search(
            r'def _advance_queue\(self\)(.*?)(?=\n    def )', content, re.DOTALL)
        if adv_body and 'layout.addWidget' in adv_body.group(1):
            print(f"  {RED}✗ Could not match _advance_queue pattern — manual fix needed{RESET}")
            sys.exit(1)
        else:
            print(f"  {YELLOW}○ SKIP _advance_queue: already clean{RESET}")

    # ── B1. Add re import if missing ─────────────────────────────
    if "import re" not in content:
        content = content.replace(
            "from __future__ import annotations",
            "from __future__ import annotations\nimport re",
            1)
        print(f"  {GREEN}✓ import re added{RESET}")
        changed = True
    else:
        print(f"  {YELLOW}○ SKIP: re already imported{RESET}")

    # ── B2. Replace on_print_progress with well-parsing version ──
    old_opp_pattern = re.compile(
        r'( {4}def on_print_progress\(self.*?)'
        r'(?=\n {4}def )',
        re.DOTALL,
    )
    if "well_name=well_name" not in content:
        m2 = old_opp_pattern.search(content)
        if m2:
            content = content[:m2.start()] + ON_PRINT_PROGRESS_REPLACEMENT + "\n" + content[m2.end():]
            if not ast_ok(content, "on_print_progress replacement"):
                sys.exit(1)
            print(f"  {GREEN}✓ on_print_progress updated (well-name parsing){RESET}")
            changed = True
        else:
            print(f"  {YELLOW}⚠ on_print_progress not found — skipping{RESET}")
    else:
        print(f"  {YELLOW}○ SKIP: on_print_progress already updated{RESET}")

    # ── B3. Inject position-poll helpers before _advance_queue ───
    if "_poll_needle_position" not in content:
        m3 = re.search(r'(\n    def _advance_queue\(self\))', content)
        if m3:
            content = content[:m3.start()] + "\n" + POSITION_POLL_METHODS + content[m3.start():]
            if not ast_ok(content, "poll helpers injection"):
                sys.exit(1)
            print(f"  {GREEN}✓ Position poll helpers added{RESET}")
            changed = True
        else:
            print(f"  {YELLOW}⚠ Could not find _advance_queue anchor for poll injection{RESET}")
    else:
        print(f"  {YELLOW}○ SKIP: poll helpers already present{RESET}")

    # ── B4. Wire timer start/stop in on_print_state_changed ──────
    # Start on RUNNING, stop on terminal states
    start_anchor  = "if state == PrintState.RUNNING and self._print_start_time is None:"
    stop_anchor   = "elif state in (PrintState.COMPLETED, PrintState.ABORTED, PrintState.ERROR):"
    timer_start   = "            self._start_position_poll()  # " + GUARD
    timer_stop    = "            self._stop_position_poll()   # " + GUARD

    if timer_start not in content:
        if start_anchor in content:
            # Insert after the _print_start_time = time.time() line
            old_block = (
                "        if state == PrintState.RUNNING and self._print_start_time is None:\n"
                "            self._print_start_time = time.time()"
            )
            new_block = (
                "        if state == PrintState.RUNNING and self._print_start_time is None:\n"
                "            self._print_start_time = time.time()\n"
                f"            {timer_start}"
            )
            if old_block in content:
                content = content.replace(old_block, new_block, 1)
                if not ast_ok(content, "timer start wire"):
                    sys.exit(1)
                print(f"  {GREEN}✓ Position poll starts on RUNNING{RESET}")
                changed = True
            else:
                print(f"  {YELLOW}⚠ RUNNING block not matched exactly — skipping start wire{RESET}")
        else:
            print(f"  {YELLOW}⚠ RUNNING anchor not found — skipping{RESET}")
    else:
        print(f"  {YELLOW}○ SKIP: start timer already wired{RESET}")

    if timer_stop not in content:
        if stop_anchor in content:
            old_stop = (
                "        elif state in (PrintState.COMPLETED, PrintState.ABORTED, PrintState.ERROR):\n"
                "            self._print_start_time = None"
            )
            new_stop = (
                "        elif state in (PrintState.COMPLETED, PrintState.ABORTED, PrintState.ERROR):\n"
                "            self._print_start_time = None\n"
                f"            {timer_stop}"
            )
            if old_stop in content:
                content = content.replace(old_stop, new_stop, 1)
                if not ast_ok(content, "timer stop wire"):
                    sys.exit(1)
                print(f"  {GREEN}✓ Position poll stops on terminal state{RESET}")
                changed = True
            else:
                print(f"  {YELLOW}⚠ terminal state block not matched exactly — skipping stop wire{RESET}")
        else:
            print(f"  {YELLOW}⚠ terminal state anchor not found — skipping{RESET}")
    else:
        print(f"  {YELLOW}○ SKIP: stop timer already wired{RESET}")

    if not changed:
        print(f"  {YELLOW}○ Nothing to do{RESET}"); return

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    shutil.copy2(target, target.with_suffix(f".bak_{GUARD}_{ts}"))
    target.write_text(content, encoding="utf-8")
    print(f"  {GREEN}✓ print_monitor.py patched{RESET}")

if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
patch_fix_monitor_timer_wire.py
Minimal targeted patch — ONLY wires _start/_stop_position_poll()
into on_print_state_changed().  Run this after
patch_fix_monitor_live_updates.py has applied A+B1-B3.
"""
import ast, sys, shutil
from pathlib import Path
from datetime import datetime

GREEN  = "\033[92m"; RED = "\033[91m"; YELLOW = "\033[93m"
CYAN   = "\033[96m"; RESET = "\033[0m"
GUARD  = "v7.3.1-timerwire"

def find_root() -> Path:
    for p in [Path.cwd(), Path(__file__).parent]:
        for c in [p, p.parent, p.parent.parent]:
            if (c / "SupportClasses").is_dir() and (c / "gui").is_dir():
                return c.resolve()
    raise RuntimeError("Cannot locate MEBP project root")

def ast_ok(text, tag):
    try:
        ast.parse(text)
        return True
    except SyntaxError as e:
        print(f"  {RED}AST FAIL after {tag}: {e}{RESET}")
        # Show the problem lines
        lines = text.splitlines()
        ln = e.lineno or 0
        for i in range(max(0, ln-3), min(len(lines), ln+2)):
            print(f"    {i+1:4d}: {repr(lines[i])}")
        return False

def main():
    root   = find_root()
    target = root / "gui" / "pages" / "print_monitor.py"
    print(f"{CYAN}Target: {target}{RESET}")
    if not target.exists():
        print(f"  {RED}File not found{RESET}"); sys.exit(1)

    content = target.read_text(encoding="utf-8")

    if GUARD in content:
        print(f"  {YELLOW}SKIP: already applied{RESET}"); return

    changed = False

    # ── Wire _start_position_poll after _print_start_time = time.time() ──
    OLD_RUN = (
        "            self._print_start_time = time.time()\n"
    )
    NEW_RUN = (
        "            self._print_start_time = time.time()\n"
        "            self._start_position_poll()  # " + GUARD + "\n"
    )
    if "_start_position_poll()" not in content:
        if OLD_RUN in content:
            content = content.replace(OLD_RUN, NEW_RUN, 1)
            if not ast_ok(content, "start poll"):
                sys.exit(1)
            print(f"  {GREEN}Position poll starts on RUNNING{RESET}")
            changed = True
        else:
            print(f"  {RED}MISS: could not find _print_start_time = time.time(){RESET}")
            sys.exit(1)
    else:
        print(f"  {YELLOW}SKIP: _start_position_poll already present{RESET}")

    # ── Wire _stop_position_poll after _print_start_time = None ──
    # There may be multiple assignments of _print_start_time = None;
    # we want the one inside the elif COMPLETED/ABORTED/ERROR block.
    OLD_TERM = (
        "        elif state in (PrintState.COMPLETED, PrintState.ABORTED, PrintState.ERROR):\n"
        "            self._print_start_time = None\n"
    )
    NEW_TERM = (
        "        elif state in (PrintState.COMPLETED, PrintState.ABORTED, PrintState.ERROR):\n"
        "            self._print_start_time = None\n"
        "            self._stop_position_poll()   # " + GUARD + "\n"
    )
    if "_stop_position_poll()" not in content:
        if OLD_TERM in content:
            content = content.replace(OLD_TERM, NEW_TERM, 1)
            if not ast_ok(content, "stop poll"):
                sys.exit(1)
            print(f"  {GREEN}Position poll stops on terminal state{RESET}")
            changed = True
        else:
            print(f"  {RED}MISS: terminal state block not found — showing relevant lines:{RESET}")
            for i, l in enumerate(content.splitlines()):
                if "_print_start_time = None" in l or "COMPLETED" in l:
                    print(f"  {i+1:4d}: {repr(l)}")
            sys.exit(1)
    else:
        print(f"  {YELLOW}SKIP: _stop_position_poll already present{RESET}")

    if not changed:
        print(f"  {YELLOW}Nothing to do{RESET}"); return

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    shutil.copy2(target, target.with_suffix(f".bak_timerwire_{ts}"))
    target.write_text(content, encoding="utf-8")
    print(f"  {GREEN}print_monitor.py timer wiring done{RESET}")

if __name__ == "__main__":
    main()

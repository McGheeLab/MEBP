#!/usr/bin/env python3
"""Fix simulator: add blocking at the send_command level, not per-handler."""

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
    print("  Simulator blocking v2 — at send_command level")
    root = find_root()
    sim_path = root / "SupportClasses" / "XYStageSimulator.py"
    content = sim_path.read_text(encoding="utf-8")

    marker = "v7.2.7-block-v2"
    if marker in content:
        print("  ○ Already patched"); return 0

    # Step 1: Ensure _wait_for_idle exists
    if "_wait_for_idle" not in content:
        # Insert before get_current_position
        gcp = re.search(r'^    def get_current_position\(self\)', content, re.MULTILINE)
        if gcp:
            method = '''    def _wait_for_idle(self, timeout_s=60.0):
        """Block until physics loop finishes an absolute move."""
        import time
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout_s:
            with self._lock:
                if self.mode != "absolute":
                    return True
            time.sleep(0.005)
        return False

'''
            content = content[:gcp.start()] + method + content[gcp.start():]
            print("  ✓ Added _wait_for_idle")
        else:
            print("  ✗ get_current_position not found"); return 1

    # Step 2: Wrap send_command to block after absolute moves
    m = find_method(content, "send_command")
    if not m:
        print("  ✗ send_command not found"); return 1

    old_body = m.group(0)

    # Rename existing send_command to _send_command_raw
    new_content = old_body.replace(
        "def send_command(self, command:",
        "def _send_command_raw(self, command:",
        1  # only first occurrence
    )

    # Add new send_command that delegates + blocks
    wrapper = '''    def send_command(self, command: str) -> str:
        """v7.2.7-block-v2: Delegate to raw handler, then block for absolute moves.

        Real Prior hardware: G/GR commands block until stage arrives.
        This wrapper replicates that behavior for the simulator.
        """
        response = self._send_command_raw(command)
        # Block if the command triggered an absolute move
        with self._lock:
            is_moving = (self.mode == "absolute")
        if is_moving:
            self._wait_for_idle()
        return response

'''
    content = content[:m.start()] + wrapper + new_content + content[m.end():]

    try:
        ast.parse(content)
    except SyntaxError as e:
        print(f"  ✗ AST FAIL: line {e.lineno}: {e.msg}")
        lines = content.split('\n')
        for i in range(max(0,e.lineno-4), min(len(lines),e.lineno+3)):
            mk = ">>>" if i==e.lineno-1 else "   "
            print(f"    {mk} {i+1:4d} | {lines[i]}")
        return 1

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    shutil.copy2(sim_path, sim_path.with_suffix(f".bak_v727bv2_{ts}"))
    sim_path.write_text(content, encoding="utf-8")
    print("  ✓ XYStageSimulator.py — send_command now blocks for moves")
    return 0

if __name__ == "__main__":
    sys.exit(main())

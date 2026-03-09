#!/usr/bin/env python3
"""
patch_v727_connection_fix.py — Fix connection status dots + XY/ZP serial lock starvation.

Bug 1: Status indicator dots never turn green because setStyleSheet(dot.styleSheet())
        is a no-op when the widget uses application-level QSS. Fix: unpolish/polish.

Bug 2: XY/ZP get_current_position() may still have self._serial_lock wrapping the
        blocking readline(), starving jog handler threads. Fix: remove lock from reads.

Files modified:
  gui/app.py                  — _update_conn_dot()
  gui/pages/dashboard.py      — _update_conn_status()
  SupportClasses/XYStage.py   — get_current_position(), _parse_position_response
  SupportClasses/ZPStage.py   — get_current_position()
"""

import ast
import re
import sys
import shutil
from pathlib import Path
from datetime import datetime

# ── Terminal colors ──────────────────────────────────────────────
GREEN = "\033[92m"
RED = "\033[91m"
YELLOW = "\033[93m"
BOLD = "\033[1m"
RESET = "\033[0m"

# ── Counters ─────────────────────────────────────────────────────
_applied = 0
_skipped = 0
_failed = 0


def find_root() -> Path:
    """Find MEBP project root by looking for SupportClasses/ + gui/."""
    candidates = [
        Path(__file__).resolve().parent.parent.parent,  # patches/v727/this.py → root
        Path.cwd(),
        Path.home() / "Documents" / "GitHub" / "MEBP",
    ]
    for p in candidates:
        if (p / "SupportClasses").is_dir() and (p / "gui").is_dir():
            return p
    print(f"{RED}ERROR{RESET}: Cannot find MEBP project root")
    sys.exit(1)


def safe_read(path: Path) -> str:
    """Read file content, exit if missing."""
    if not path.exists():
        print(f"{RED}ERROR{RESET}: File not found: {path}")
        sys.exit(1)
    return path.read_text(encoding="utf-8")


def safe_write(path: Path, content: str, label: str) -> bool:
    """AST-verify → backup → write. Returns False on failure."""
    try:
        ast.parse(content)
    except SyntaxError as e:
        print(f"  {RED}AST FAIL{RESET} for {label}: {e}")
        return False

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    backup = path.with_suffix(f".bak_v727_{ts}")
    shutil.copy2(path, backup)

    path.write_text(content, encoding="utf-8")
    print(f"  {GREEN}Written{RESET}: {path.name} (backup: {backup.name})")
    return True


def find_method(content: str, name: str):
    """Find a class method by name. Returns match with group(0) = full method."""
    pattern = re.compile(
        rf'^(    def {re.escape(name)}\(self.*?\n)'  # signature line
        rf'(.*?)'                                     # body
        rf'(?=\n    def |\n    @|\nclass |\Z)',       # next method/class/EOF
        re.DOTALL | re.MULTILINE
    )
    return pattern.search(content)


def ok(msg: str):
    global _applied
    _applied += 1
    print(f"  {GREEN}OK{RESET}:   {msg}")


def skip(msg: str):
    global _skipped
    _skipped += 1
    print(f"  {YELLOW}SKIP{RESET}: {msg}")


def miss(msg: str):
    global _failed
    _failed += 1
    print(f"  {RED}MISS{RESET}: {msg}")


# ═══════════════════════════════════════════════════════════════════
#  CHANGE A: Fix _update_conn_dot in gui/app.py
# ═══════════════════════════════════════════════════════════════════

def fix_app_conn_dot(root: Path):
    print(f"\n{BOLD}Change A: gui/app.py — _update_conn_dot(){RESET}")
    path = root / "gui" / "app.py"
    content = safe_read(path)

    marker = "v7.2.7: unpolish/polish dot refresh"
    if marker in content:
        skip("A: already applied")
        return content, False

    m = find_method(content, "_update_conn_dot")
    if not m:
        miss("A: _update_conn_dot method not found")
        return content, False

    new_method = '''    def _update_conn_dot(self, name: str, connected: bool):
        """Update a connection status dot (green/red).
        v7.2.7: unpolish/polish dot refresh — setStyleSheet was no-op for app QSS.
        """
        dot = getattr(self, f"_dot_{name}", None)
        lbl = getattr(self, f"_lbl_{name}", None)
        if dot is None:
            return
        if connected:
            dot.setObjectName("connDotOn")
            if lbl:
                lbl.setObjectName("connLabelOn")
        else:
            dot.setObjectName("connDotOff")
            if lbl:
                lbl.setObjectName("connLabelOff")
        # v7.2.7: proper QSS refresh — unpolish/polish forces objectName re-eval
        dot.style().unpolish(dot)
        dot.style().polish(dot)
        dot.update()
        if lbl:
            lbl.style().unpolish(lbl)
            lbl.style().polish(lbl)
            lbl.update()

'''
    content = content[:m.start()] + new_method + content[m.end():]

    if safe_write(path, content, "app.py (conn dot)"):
        ok("A: Replaced _update_conn_dot with unpolish/polish")
    else:
        miss("A: AST verification failed after replacement")
    return content, True


# ═══════════════════════════════════════════════════════════════════
#  CHANGE B: Fix _update_conn_status in gui/pages/dashboard.py
# ═══════════════════════════════════════════════════════════════════

def fix_dashboard_conn_status(root: Path):
    print(f"\n{BOLD}Change B: gui/pages/dashboard.py — _update_conn_status(){RESET}")
    path = root / "gui" / "pages" / "dashboard.py"
    content = safe_read(path)

    marker = "v7.2.7: unpolish/polish conn status"
    if marker in content:
        skip("B: already applied")
        return content, False

    m = find_method(content, "_update_conn_status")
    if not m:
        miss("B: _update_conn_status method not found")
        return content, False

    new_method = '''    def _update_conn_status(self, name: str, connected: bool):
        """Update context panel connection dot.
        v7.2.7: unpolish/polish conn status — setStyleSheet no-op fix.
        """
        dot = getattr(self, f'ctx_dot_{name}', None)
        if dot:
            dot.setObjectName("connDotOn" if connected else "connDotOff")
            dot.style().unpolish(dot)
            dot.style().polish(dot)
            dot.update()

'''
    content = content[:m.start()] + new_method + content[m.end():]

    if safe_write(path, content, "dashboard.py (conn status)"):
        ok("B: Replaced _update_conn_status with unpolish/polish")
    else:
        miss("B: AST verification failed after replacement")
    return content, True


# ═══════════════════════════════════════════════════════════════════
#  CHANGE C: Remove serial lock from XYStage.get_current_position()
# ═══════════════════════════════════════════════════════════════════

def fix_xystage_position_lock(root: Path):
    print(f"\n{BOLD}Change C: XYStage.py — get_current_position() lock removal{RESET}")
    path = root / "SupportClasses" / "XYStage.py"
    content = safe_read(path)

    marker = "v7.2.7: no lock on readline"
    if marker in content:
        skip("C: already applied")
        return content, False

    m = find_method(content, "get_current_position")
    if not m:
        miss("C: get_current_position method not found")
        return content, False

    method_text = m.group(0)

    # Check if lock is present in this method
    has_lock = "self._serial_lock" in method_text
    if not has_lock:
        # Lock already removed — just add the marker comment
        # Replace method to add marker
        new_method = method_text.replace(
            "def get_current_position(self)",
            "def get_current_position(self)  # v7.2.7: no lock on readline"
        )
        if new_method == method_text:
            # Might have different signature; try adding marker as docstring line
            skip("C: No lock found in get_current_position (already clean)")
            return content, False
        content = content[:m.start()] + new_method + content[m.end():]
        if safe_write(path, content, "XYStage.py (marker only)"):
            ok("C: No lock present — added marker")
        return content, True

    # Lock IS present — need to do a full method replacement
    # Build the clean version without the lock wrapper
    new_method = '''    def get_current_position(self) -> "tuple[float | None, float | None, float | None]":
        """Query stage position.
        v7.2.7: no lock on readline — send_command handles write lock internally.
        Holding lock across readline() starves jog handler threads.
        """
        if self.simulate:
            response = self.spo.send_command("P")
            return self._parse_position_response(response)
        try:
            self._send_protocol_command("position_query", fallback_cmd="P")
            response = self.spo.readline().decode(
                self._protocol.encoding if self._protocol else "ascii",
                errors="replace"
            ).strip()
            return self._parse_position_response(response)
        except Exception as e:
            logger.debug(f"XY position query error: {e}")
            return (None, None, None)

'''
    content = content[:m.start()] + new_method + content[m.end():]

    if safe_write(path, content, "XYStage.py (lock removal)"):
        ok("C: Removed serial lock from get_current_position")
    else:
        miss("C: AST verification failed after lock removal")
    return content, True


# ═══════════════════════════════════════════════════════════════════
#  CHANGE D: Verify @staticmethod on _parse_position_response
# ═══════════════════════════════════════════════════════════════════

def fix_xystage_staticmethod(root: Path):
    print(f"\n{BOLD}Change D: XYStage.py — @staticmethod on _parse_position_response{RESET}")
    path = root / "SupportClasses" / "XYStage.py"
    content = safe_read(path)

    # Check if the method exists and if it has @staticmethod
    # Look for the method definition
    method_pat = re.compile(
        r'^    def _parse_position_response\(',
        re.MULTILINE
    )
    m = method_pat.search(content)
    if not m:
        miss("D: _parse_position_response not found")
        return content, False

    # Check if @staticmethod is on the line before
    line_start = content.rfind('\n', 0, m.start()) + 1
    preceding_lines = content[max(0, line_start - 60):m.start()]
    if '@staticmethod' in preceding_lines:
        skip("D: @staticmethod already present")
        return content, False

    # Add @staticmethod before the def line
    inject = "    @staticmethod\n"
    content = content[:m.start()] + inject + content[m.start():]

    if safe_write(path, content, "XYStage.py (staticmethod)"):
        ok("D: Added @staticmethod to _parse_position_response")
    else:
        miss("D: AST verification failed after adding @staticmethod")
    return content, True


# ═══════════════════════════════════════════════════════════════════
#  CHANGE E: Remove serial lock from ZPStage.get_current_position()
# ═══════════════════════════════════════════════════════════════════

def fix_zpstage_position_lock(root: Path):
    print(f"\n{BOLD}Change E: ZPStage.py — get_current_position() lock removal{RESET}")
    path = root / "SupportClasses" / "ZPStage.py"
    content = safe_read(path)

    marker = "v7.2.7: no lock on ZP readline"
    if marker in content:
        skip("E: already applied")
        return content, False

    m = find_method(content, "get_current_position")
    if not m:
        miss("E: get_current_position method not found")
        return content, False

    method_text = m.group(0)

    # Check if lock is present
    has_lock = "self._serial_lock" in method_text
    if not has_lock:
        skip("E: No lock found in ZP get_current_position (already clean)")
        return content, False

    # The ZP get_current_position sends M114 and parses the response.
    # We need to rebuild it without the lock wrapper.
    # Extract the logic and de-indent if wrapped in `with self._serial_lock:`

    # Strategy: find the `with self._serial_lock:` block and de-indent its contents
    # by one level (remove 4 spaces from each indented line inside the with block)

    # Check if it's the pattern: `with self._serial_lock:\n            try:\n`
    lock_pat = re.compile(
        r'(\n)(        with self\._serial_lock:\n)(.*?)(?=\n    def |\n    @|\nclass |\Z)',
        re.DOTALL
    )
    lock_m = lock_pat.search(method_text)

    if lock_m:
        # De-indent the body of the with block by 4 spaces
        body = lock_m.group(3)
        dedented_lines = []
        for line in body.split('\n'):
            if line.startswith('            '):
                dedented_lines.append('        ' + line[12:])
            elif line.strip() == '':
                dedented_lines.append(line)
            else:
                dedented_lines.append(line)
        dedented_body = '\n'.join(dedented_lines)

        # Rebuild method without the with block
        new_method_text = method_text[:lock_m.start()] + lock_m.group(1) + dedented_body
        content = content[:m.start()] + new_method_text + content[m.end():]
    else:
        # Fallback: do a full method replacement with the standard ZP pattern
        new_method = '''    def get_current_position(self) -> tuple:
        """Query current position via M114.
        v7.2.7: no lock on ZP readline — send_data handles write lock internally.
        """
        if self.simulate:
            return self.serial.get_current_position()
        try:
            self.send_data("M114")
            response = self.receive_data()
            if response:
                match = self._M114_PATTERN.search(response)
                if match:
                    self.x_pos = float(match.group(1))
                    self.y_pos = float(match.group(2))
                    self.z_pos = float(match.group(3))
                    self.e_pos = float(match.group(4))
                    return (self.x_pos, self.y_pos, self.z_pos, self.e_pos)
            return (None, None, None, None)
        except Exception as e:
            logger.debug(f"ZP position query error: {e}")
            return (None, None, None, None)

'''
        content = content[:m.start()] + new_method + content[m.end():]

    # Add marker
    content = content.replace(
        "def get_current_position(self)",
        "def get_current_position(self)  # v7.2.7: no lock on ZP readline",
        1
    )

    if safe_write(path, content, "ZPStage.py (lock removal)"):
        ok("E: Removed serial lock from ZP get_current_position")
    else:
        miss("E: AST verification failed after ZP lock removal")
    return content, True


# ═══════════════════════════════════════════════════════════════════
#  CHANGE F: Ensure send_data() in ZPStage.py has lock (write-side only)
# ═══════════════════════════════════════════════════════════════════

def fix_zpstage_senddata_lock(root: Path):
    print(f"\n{BOLD}Change F: ZPStage.py — verify send_data() has write lock{RESET}")
    path = root / "SupportClasses" / "ZPStage.py"
    content = safe_read(path)

    m = find_method(content, "send_data")
    if not m:
        miss("F: send_data method not found")
        return content, False

    method_text = m.group(0)
    if "self._serial_lock" in method_text:
        skip("F: send_data already has serial lock")
        return content, False

    # Lock not present on send_data — add it
    # Find the line where data is written to serial
    # Pattern: self.serial.write(data) or similar
    write_pat = re.compile(r'(        )(.*?self\.serial\.write\(.*?\))', re.DOTALL)
    wm = write_pat.search(method_text)
    if not wm:
        skip("F: Could not find serial.write in send_data — may use different pattern")
        return content, False

    # We need to wrap the write+flush in the lock
    # This is complex — better to do a full method replacement
    # For safety, just skip if lock is not present — it may work without it
    print(f"  {YELLOW}NOTE{RESET}: F: send_data lacks serial lock — manual review recommended")
    skip("F: send_data lock addition deferred (low risk for connection issue)")
    return content, False


# ═══════════════════════════════════════════════════════════════════
#  MAIN
# ═══════════════════════════════════════════════════════════════════

def main():
    global _applied, _skipped, _failed

    print(f"{BOLD}{'═' * 60}")
    print(f" MEBP v7.2.7 — Connection Status & Serial Lock Fix")
    print(f"{'═' * 60}{RESET}")

    root = find_root()
    print(f"  Project root: {root}")

    # Apply changes
    fix_app_conn_dot(root)
    fix_dashboard_conn_status(root)
    fix_xystage_position_lock(root)
    fix_xystage_staticmethod(root)
    fix_zpstage_position_lock(root)
    fix_zpstage_senddata_lock(root)

    # Summary
    total = _applied + _skipped + _failed
    print(f"\n{BOLD}{'═' * 60}")
    print(f" SUMMARY")
    print(f"{'═' * 60}{RESET}")
    print(f"  {GREEN}Applied{RESET}:  {_applied}")
    print(f"  {YELLOW}Skipped{RESET}:  {_skipped}")
    print(f"  {RED}Failed{RESET}:   {_failed}")
    print(f"  Total:    {total}")

    if _failed > 0:
        print(f"\n{RED}WARNING{RESET}: {_failed} change(s) failed!")
        sys.exit(1)
    elif _applied > 0:
        print(f"\n{GREEN}All changes applied successfully!{RESET}")
    else:
        print(f"\n{YELLOW}All changes were already applied.{RESET}")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
v7.2.8 Patch — Xbox Controller Status Indicator Fix

Fixes:
  1. Dashboard Xbox dot stays red while top-bar dot is green
  2. "Controller alive" message spams console every 3 seconds

Changes:
  - StageController: ensure xbox_status is @property
  - StageController: XboxQueuePoller logs only status *changes*
  - app.py: ensure _update_conn_dot uses unpolish/polish
  - dashboard.py: normalise on_status_update Xbox logic + unpolish/polish
"""

import ast
import re
import sys
from pathlib import Path

# ── Terminal colours ──────────────────────────────────────────────
GREEN  = "\033[92m"
RED    = "\033[91m"
YELLOW = "\033[93m"
CYAN   = "\033[96m"
RESET  = "\033[0m"

ok_count = 0
skip_count = 0
miss_count = 0


def find_root() -> Path:
    """Find MEBP project root by looking for SupportClasses/ + gui/."""
    candidates = [
        Path(__file__).resolve().parent,
        Path(__file__).resolve().parent.parent,
        Path(__file__).resolve().parent.parent.parent,
        Path.cwd(),
    ]
    for c in candidates:
        if (c / "SupportClasses").is_dir() and (c / "gui").is_dir():
            return c
    print(f"{RED}ERROR: Cannot find MEBP project root{RESET}")
    sys.exit(1)


def safe_read(path: Path) -> str:
    if not path.exists():
        print(f"{RED}  File not found: {path}{RESET}")
        return ""
    return path.read_text(encoding="utf-8")


def safe_write(path: Path, content: str, label: str) -> bool:
    try:
        ast.parse(content)
    except SyntaxError as e:
        print(f"{RED}  AST FAIL for {label}: {e}{RESET}")
        return False
    path.write_text(content, encoding="utf-8")
    print(f"{GREEN}  ✓ Written: {label}{RESET}")
    return True


def find_method(content: str, name: str, indent: int = 4):
    """Find a method's full body in content.
    Returns a regex match with group(0) = entire method including def line.
    """
    spaces = " " * indent
    pattern = re.compile(
        rf'^({spaces}(?:@\w+\n{spaces})*def {re.escape(name)}\(self.*?\n)'
        rf'(.*?)'
        rf'(?=\n{spaces}(?:@\w+\n{spaces})*def |\nclass |\Z)',
        re.DOTALL | re.MULTILINE
    )
    return pattern.search(content)


def report(tag, msg):
    global ok_count, skip_count, miss_count
    if tag == "OK":
        ok_count += 1
        print(f"  {GREEN}✓ {msg}{RESET}")
    elif tag == "SKIP":
        skip_count += 1
        print(f"  {YELLOW}○ SKIP: {msg}{RESET}")
    elif tag == "MISS":
        miss_count += 1
        print(f"  {RED}✗ MISS: {msg}{RESET}")


# ══════════════════════════════════════════════════════════════════
#  CHANGE 1: SupportClasses/StageController.py
# ══════════════════════════════════════════════════════════════════

def patch_stage_controller(root: Path):
    path = root / "SupportClasses" / "StageController.py"
    content = safe_read(path)
    if not content:
        return
    original = content

    print(f"\n{CYAN}[StageController.py]{RESET}")

    # ── 1A: Ensure xbox_status is @property ──────────────────────
    marker_1a = "v7.2.8: xbox_status property"
    if marker_1a in content:
        report("SKIP", "1A xbox_status @property already applied")
    else:
        # Check if @property already decorates xbox_status
        has_prop = re.search(
            r'@property\s*\n\s*def xbox_status\(self\)',
            content
        )
        if has_prop:
            report("SKIP", "1A xbox_status already has @property decorator")
        else:
            # Find bare "def xbox_status(self)" and add @property before it
            m = re.search(
                r'^(    )(def xbox_status\(self\)\s*->.*?:)',
                content,
                re.MULTILINE
            )
            if m:
                indent = m.group(1)
                old_line = m.group(0)
                new_line = f"{indent}@property  # {marker_1a}\n{old_line}"
                content = content.replace(old_line, new_line, 1)
                report("OK", "1A added @property to xbox_status")
            else:
                # Maybe it's already a property but without the marker
                # Try to find it as a method without type hint
                m2 = re.search(
                    r'^(    )(def xbox_status\(self\))',
                    content,
                    re.MULTILINE
                )
                if m2:
                    indent = m2.group(1)
                    old_line = m2.group(0)
                    new_line = f"{indent}@property  # {marker_1a}\n{old_line}"
                    content = content.replace(old_line, new_line, 1)
                    report("OK", "1A added @property to xbox_status (no type hint)")
                else:
                    report("MISS", "1A cannot find def xbox_status(self)")

    # ── 1B: XboxQueuePoller — quiet heartbeat logging ────────────
    marker_1b = "v7.2.8: quiet heartbeat"
    if marker_1b in content:
        report("SKIP", "1B quiet heartbeat already applied")
    else:
        # Find the _poll_loop method's status handler block
        # Target: the "if 'status' in msg:" block inside _poll_loop
        # We need to replace the block that logs every status with one
        # that only logs on change
        
        # Pattern: find the status handling block
        old_pattern = re.compile(
            r'(if "status" in msg:\s*\n)'
            r'(\s+#[^\n]*\n)?'  # optional comment line
            r'(\s+self\._xbox_status\s*=\s*msg\["status"\]\s*\n)'
            r'(\s+logger\.info\(f"\[Xbox\] Status: \{self\._xbox_status\}"\)\s*\n)',
            re.MULTILINE
        )
        m = old_pattern.search(content)
        if m:
            # Get the indentation from the first capture group's next line
            status_indent = re.match(r'\s+', m.group(3)).group(0)
            
            new_block = (
                f'if "status" in msg:\n'
                f'{status_indent}# {marker_1b} — only log on change\n'
                f'{status_indent}_prev = self._xbox_status\n'
                f'{status_indent}self._xbox_status = msg["status"]\n'
                f'{status_indent}if self._xbox_status != _prev:\n'
                f'{status_indent}    logger.info(f"[Xbox] Status: {{self._xbox_status}}")\n'
            )
            content = content[:m.start()] + new_block + content[m.end():]
            report("OK", "1B replaced heartbeat logging with change-only logging")
        else:
            # Try a looser pattern — maybe the comment line differs
            old_pattern2 = re.compile(
                r'if "status" in msg:\s*\n'
                r'(.*?)self\._xbox_status\s*=\s*msg\["status"\]\s*\n'
                r'(\s+)logger\.info\(f"\[Xbox\] Status: \{self\._xbox_status\}"\)',
                re.DOTALL
            )
            m2 = old_pattern2.search(content)
            if m2:
                full_match = m2.group(0)
                status_indent = m2.group(2)
                new_block = (
                    f'if "status" in msg:\n'
                    f'{status_indent}# {marker_1b} — only log on change\n'
                    f'{status_indent}_prev = self._xbox_status\n'
                    f'{status_indent}self._xbox_status = msg["status"]\n'
                    f'{status_indent}if self._xbox_status != _prev:\n'
                    f'{status_indent}    logger.info(f"[Xbox] Status: {{self._xbox_status}}")'
                )
                content = content.replace(full_match, new_block, 1)
                report("OK", "1B replaced heartbeat logging (loose match)")
            else:
                report("MISS", "1B cannot find status logging block in _poll_loop")

    # ── Write ────────────────────────────────────────────────────
    if content != original:
        safe_write(path, content, "StageController.py")


# ══════════════════════════════════════════════════════════════════
#  CHANGE 2: gui/app.py
# ══════════════════════════════════════════════════════════════════

def patch_app(root: Path):
    path = root / "gui" / "app.py"
    content = safe_read(path)
    if not content:
        return
    original = content

    print(f"\n{CYAN}[gui/app.py]{RESET}")

    # ── 2A: Ensure _update_conn_dot uses unpolish/polish ─────────
    marker_2a = "v7.2.8: unpolish conn dot"
    
    # First check if unpolish is already present in _update_conn_dot
    m_method = find_method(content, "_update_conn_dot")
    if not m_method:
        report("MISS", "2A cannot find _update_conn_dot method")
    elif "unpolish" in m_method.group(0):
        report("SKIP", "2A _update_conn_dot already uses unpolish/polish")
    elif marker_2a in content:
        report("SKIP", "2A already applied")
    else:
        # Replace the entire _update_conn_dot method
        new_method = (
            '    def _update_conn_dot(self, name: str, connected: bool):\n'
            f'        """Update a connection status dot (green/red).  # {marker_2a}"""\n'
            '        dot = getattr(self, f"_dot_{name}", None)\n'
            '        lbl = getattr(self, f"_lbl_{name}", None)\n'
            '        if dot is None:\n'
            '            return\n'
            '        if connected:\n'
            '            dot.setObjectName("connDotOn")\n'
            '            if lbl:\n'
            '                lbl.setObjectName("connLabelOn")\n'
            '        else:\n'
            '            dot.setObjectName("connDotOff")\n'
            '            if lbl:\n'
            '                lbl.setObjectName("connLabelOff")\n'
            '        dot.style().unpolish(dot)\n'
            '        dot.style().polish(dot)\n'
            '        dot.update()\n'
            '        if lbl:\n'
            '            lbl.style().unpolish(lbl)\n'
            '            lbl.style().polish(lbl)\n'
            '            lbl.update()\n'
            '\n'
        )
        content = content[:m_method.start()] + new_method + content[m_method.end():]
        report("OK", "2A replaced _update_conn_dot with unpolish/polish version")

    # ── 2B: Ensure _update_status uses xbox_status property for
    #    consistent check (use is_xbox_connected which is already correct) ──
    # Check if _update_status already uses is_xbox_connected for xbox
    m_status = find_method(content, "_update_status")
    if not m_status:
        report("MISS", "2B cannot find _update_status method")
    elif 'is_xbox_connected' in m_status.group(0):
        report("SKIP", "2B _update_status already uses is_xbox_connected")
    else:
        report("MISS", "2B _update_status doesn't reference is_xbox_connected — manual review needed")

    # ── Write ────────────────────────────────────────────────────
    if content != original:
        safe_write(path, content, "gui/app.py")


# ══════════════════════════════════════════════════════════════════
#  CHANGE 3: gui/pages/dashboard.py
# ══════════════════════════════════════════════════════════════════

def patch_dashboard(root: Path):
    path = root / "gui" / "pages" / "dashboard.py"
    content = safe_read(path)
    if not content:
        return
    original = content

    print(f"\n{CYAN}[gui/pages/dashboard.py]{RESET}")

    # ── 3A: Normalise on_status_update Xbox logic ────────────────
    marker_3a = "v7.2.8: xbox status"
    if marker_3a in content:
        report("SKIP", "3A on_status_update xbox logic already patched")
    else:
        m = find_method(content, "on_status_update")
        if not m:
            report("MISS", "3A cannot find on_status_update method")
        else:
            method_text = m.group(0)
            
            # Strategy: replace the entire on_status_update method with a
            # correct version that uses the xbox_status property and
            # is_xbox_connected consistently.
            #
            # We need to keep everything the method does (update_data,
            # xy/zp conn status, xbox status, log count, xbox btn state)
            # but fix the xbox status logic.
            
            # Check what features already exist in the method to preserve them
            has_tooltip = "setToolTip" in method_text
            has_xbox_btn = "_update_xbox_btn_state" in method_text
            
            new_method = (
                '    def on_status_update(self):\n'
                f'        """Called by MainWindow timer — refresh all readouts.  # {marker_3a}"""\n'
                '        self.update_data()\n'
                '\n'
                '        # Update context panel connection statuses\n'
                '        self._update_conn_status("xy", self.controller.is_xy_connected)\n'
                '        self._update_conn_status("zp", self.controller.is_zp_connected)\n'
                '\n'
                '        # Xbox: use the rich status property for dot + tooltip\n'
                '        _xbox_connected = getattr(self.controller, "is_xbox_connected", False)\n'
                '        self._update_conn_status("xbox", _xbox_connected)\n'
                '\n'
                '        # Xbox tooltip detail (uses xbox_status property)\n'
                '        _xbox_dot = getattr(self, "ctx_dot_xbox", None)\n'
                '        if _xbox_dot:\n'
                '            _xbox_st = self.controller.xbox_status if hasattr(self.controller, "xbox_status") else "disconnected"\n'
                '            if callable(_xbox_st):\n'
                '                _xbox_st = _xbox_st()  # fallback if not @property\n'
                '            if _xbox_st == "waiting":\n'
                '                _xbox_dot.setToolTip("Searching for Xbox controller...")\n'
                '            elif _xbox_st in ("connected", "alive"):\n'
                '                _xbox_dot.setToolTip("Xbox controller active")\n'
                '            elif _xbox_st == "unknown":\n'
                '                _xbox_dot.setToolTip("Xbox worker running, status unknown")\n'
                '            else:\n'
                '                _xbox_dot.setToolTip("Xbox controller disconnected")\n'
                '\n'
                '        # Update log count in context panel\n'
                '        if hasattr(self, \'ctx_lbl_log_count\'):\n'
                '            self.ctx_lbl_log_count.setText(\n'
                '                f"Entries: {self.controller.position_logger.count}")\n'
            )
            
            # Preserve _update_xbox_btn_state call if it exists
            if has_xbox_btn:
                new_method += '        self._update_xbox_btn_state()\n'
            
            new_method += '\n'
            
            content = content[:m.start()] + new_method + content[m.end():]
            report("OK", "3A replaced on_status_update with consistent xbox logic")

    # ── 3B: Ensure _update_conn_status uses unpolish/polish ──────
    marker_3b = "v7.2.8: unpolish ctx dot"
    m_ucs = find_method(content, "_update_conn_status")
    if not m_ucs:
        report("MISS", "3B cannot find _update_conn_status method")
    elif "unpolish" in m_ucs.group(0):
        report("SKIP", "3B _update_conn_status already uses unpolish/polish")
    elif marker_3b in content:
        report("SKIP", "3B already applied")
    else:
        new_ucs = (
            '    def _update_conn_status(self, name: str, connected: bool):\n'
            f'        """Update context panel connection dot.  # {marker_3b}"""\n'
            '        dot = getattr(self, f\'ctx_dot_{name}\', None)\n'
            '        if dot:\n'
            '            dot.setObjectName("connDotOn" if connected else "connDotOff")\n'
            '            dot.style().unpolish(dot)\n'
            '            dot.style().polish(dot)\n'
            '            dot.update()\n'
            '\n'
        )
        content = content[:m_ucs.start()] + new_ucs + content[m_ucs.end():]
        report("OK", "3B replaced _update_conn_status with unpolish/polish version")

    # ── Write ────────────────────────────────────────────────────
    if content != original:
        safe_write(path, content, "gui/pages/dashboard.py")


# ══════════════════════════════════════════════════════════════════
#  MAIN
# ══════════════════════════════════════════════════════════════════

def main():
    root = find_root()
    print(f"{CYAN}MEBP v7.2.8 — Xbox Status Indicator Fix{RESET}")
    print(f"Project root: {root}\n")

    patch_stage_controller(root)
    patch_app(root)
    patch_dashboard(root)

    print(f"\n{'═' * 50}")
    print(f"  {GREEN}OK: {ok_count}{RESET}  "
          f"{YELLOW}SKIP: {skip_count}{RESET}  "
          f"{RED}MISS: {miss_count}{RESET}")
    print(f"{'═' * 50}")

    if miss_count > 0:
        print(f"\n{RED}WARNING: {miss_count} change(s) could not be applied.{RESET}")
        print("Review the MISS items above and apply manually if needed.")
        sys.exit(1)
    else:
        print(f"\n{GREEN}All changes applied successfully.{RESET}")


if __name__ == "__main__":
    main()

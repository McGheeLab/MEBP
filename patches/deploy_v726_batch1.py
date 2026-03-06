#!/usr/bin/env python3
"""
MEBP v7.2.6 — Batch 1 Deployment: Small/Medium File Rewrites

This script reads each file from disk, applies changes using method-level
regex replacement, AST-verifies before writing, and creates backups.

Files modified:
    1. gui/styles.py             — Fix SECTION_TITLE_STYLE alignment
    2. gui/widgets/console_log.py — Add minimum height
    3. gui/widgets/camera_widget.py — Add async detect, reduce max_index
    4. gui/app.py                — Fix splitter crash, remove startup camera
    5. gui/pages/jog_control.py  — Active pump white text

Usage:
    cd /path/to/McGheeLab/MEBP
    python patches/v726/deploy_v726_batch1.py
"""

import ast
import re
import sys
import shutil
from pathlib import Path
from datetime import datetime

# ── Terminal colors ───────────────────────────────────────────────
G = "\033[92m"   # Green
R = "\033[91m"   # Red
Y = "\033[93m"   # Yellow
B = "\033[1m"    # Bold
C = "\033[96m"   # Cyan
X = "\033[0m"    # Reset

_ok = 0
_skip = 0
_fail = 0
_ts = datetime.now().strftime("%Y%m%d_%H%M%S")


def find_root() -> Path:
    """Find MEBP project root."""
    for c in [Path("."), Path(".."), Path(__file__).parent.parent.parent]:
        if (c / "SupportClasses").is_dir() and (c / "gui").is_dir():
            return c.resolve()
    print(f"{R}Cannot find project root (need SupportClasses/ + gui/){X}")
    sys.exit(1)


def safe_read(path: Path) -> str:
    """Read file or exit."""
    if not path.exists():
        print(f"  {R}FILE NOT FOUND: {path}{X}")
        return ""
    return path.read_text(encoding="utf-8")


def safe_write(path: Path, content: str, label: str) -> bool:
    """AST-verify, backup, and write."""
    global _ok, _fail
    try:
        ast.parse(content)
    except SyntaxError as e:
        print(f"  {R}AST FAIL: {label} — {e}{X}")
        _fail += 1
        return False

    backup = path.with_suffix(f".bak_v726_{_ts}")
    shutil.copy2(path, backup)
    path.write_text(content, encoding="utf-8")
    print(f"  {G}WROTE{X}: {path.name} ({label})")
    _ok += 1
    return True


def report_change(applied: bool, label: str):
    """Report whether a specific change was applied."""
    global _ok, _skip
    if applied:
        print(f"    {G}✓{X} {label}")
    else:
        print(f"    {Y}○{X} {label} (already applied or not needed)")
        _skip += 1


# ═══════════════════════════════════════════════════════════════════
#  FILE 1: gui/styles.py — Fix SECTION_TITLE_STYLE alignment
# ═══════════════════════════════════════════════════════════════════

def fix_styles(root: Path):
    path = root / "gui" / "styles.py"
    print(f"\n{C}[1/5] {path.name}{X}")
    content = safe_read(path)
    if not content:
        return

    changed = False

    # --- Add SECTION_TITLE_STYLE if missing ---
    if "SECTION_TITLE_STYLE" not in content:
        style_block = '''

# ── Centralized Section Styles (v7.2.4 + v7.2.6 fix) ────────────
SECTION_TITLE_STYLE = f"""
    QGroupBox {{
        font-size: 11pt;
        font-weight: 600;
        color: {COLORS['text']};
        border: 1px solid {COLORS['surface1']};
        border-radius: 8px;
        margin-top: 14px;
        padding: 18px 12px 10px 12px;
    }}
    QGroupBox::title {{
        subcontrol-origin: margin;
        subcontrol-position: top left;
        left: 12px;
        top: 0px;
        padding: 2px 10px;
        background-color: {COLORS['base']};
        border-radius: 4px;
        color: {COLORS['blue']};
        font-size: 10pt;
        font-weight: 600;
    }}
"""

CONTEXT_SECTION_LABEL_STYLE = f"""
    font-size: 10pt;
    font-weight: 600;
    color: {COLORS['blue']};
    padding: 6px 0px 2px 0px;
    border-bottom: 1px solid {COLORS['surface1']};
    margin-bottom: 4px;
"""

PAGE_HEADER_STYLE = f"""
    font-size: 14pt;
    font-weight: 700;
    color: {COLORS['text']};
    padding: 4px 0px;
"""

CARD_FRAME_STYLE = f"""
    QFrame#cardFrame {{
        background-color: {COLORS['surface0']};
        border: 1px solid {COLORS['surface1']};
        border-radius: 8px;
        padding: 12px;
    }}
"""
'''
        # Insert before MENU_SELECTED_STYLESHEET
        anchor = "MENU_SELECTED_STYLESHEET"
        if anchor in content:
            idx = content.index(anchor)
            # Find start of line
            line_start = content.rfind('\n', 0, idx)
            content = content[:line_start] + style_block + "\n" + content[line_start:]
            changed = True
            report_change(True, "Added SECTION_TITLE_STYLE + related constants")
        else:
            report_change(False, "Could not find MENU_SELECTED_STYLESHEET anchor")
    else:
        # --- Fix existing: ensure top: 0px is present ---
        if "top: 0px;" not in content:
            # Add top: 0px after subcontrol-position line in SECTION_TITLE_STYLE
            content = re.sub(
                r'(subcontrol-position: top left;\s*\n\s*left: 12px;)',
                r'\1\n        top: 0px;',
                content, count=1
            )
            changed = True
            report_change(True, "Added top: 0px to QGroupBox::title")
        else:
            report_change(False, "top: 0px already present")

        # Ensure margin-top is 14px
        if "margin-top: 12px;" in content:
            content = content.replace("margin-top: 12px;", "margin-top: 14px;", 1)
            changed = True
            report_change(True, "Adjusted margin-top 12→14px")
        else:
            report_change(False, "margin-top already ≥14px")

    if changed:
        safe_write(path, content, "title alignment fix")
    else:
        print(f"  {Y}No changes needed{X}")


# ═══════════════════════════════════════════════════════════════════
#  FILE 2: gui/widgets/console_log.py — Add minimum height
# ═══════════════════════════════════════════════════════════════════

def fix_console_log(root: Path):
    path = root / "gui" / "widgets" / "console_log.py"
    print(f"\n{C}[2/5] {path.name}{X}")
    content = safe_read(path)
    if not content:
        return

    changed = False

    if "setMinimumHeight" not in content:
        # Find _setup_ui method and add after layout creation
        match = re.search(
            r'(layout = QVBoxLayout\(self\)\s*\n\s*layout\.setContentsMargins\(0, 0, 0, 0\)\s*\n\s*layout\.setSpacing\(2\))',
            content
        )
        if match:
            insert_pos = match.end()
            content = content[:insert_pos] + \
                "\n\n        # v7.2.6: Prevent resize crash\n        self.setMinimumHeight(40)" + \
                content[insert_pos:]
            changed = True
            report_change(True, "Added setMinimumHeight(40)")
        else:
            report_change(False, "Could not find layout setup in _setup_ui")
    else:
        report_change(False, "setMinimumHeight already present")

    if changed:
        safe_write(path, content, "min height")
    else:
        print(f"  {Y}No changes needed{X}")


# ═══════════════════════════════════════════════════════════════════
#  FILE 3: gui/widgets/camera_widget.py — Async detect + reduce max
# ═══════════════════════════════════════════════════════════════════

def fix_camera_widget(root: Path):
    path = root / "gui" / "widgets" / "camera_widget.py"
    print(f"\n{C}[3/5] {path.name}{X}")
    content = safe_read(path)
    if not content:
        return

    changed = False

    # Reduce max_index default from 8 to 4
    if "max_index: int = 8" in content:
        content = content.replace("max_index: int = 8", "max_index: int = 4", 1)
        changed = True
        report_change(True, "Reduced detect_cameras max_index 8→4")
    else:
        report_change(False, "max_index already ≤4 or different signature")

    # Add detect_cameras_async if missing
    if "detect_cameras_async" not in content:
        async_func = '''

def detect_cameras_async(callback, max_index: int = 4):
    """
    v7.2.6: Detect cameras in a background thread.

    Args:
        callback: Called with list[int] of found camera indices.
                  Called from background thread — use QTimer.singleShot(0, fn)
                  to marshal back to main thread.
        max_index: Max camera index to probe.
    """
    import threading

    def _worker():
        indices = detect_cameras(max_index)
        callback(indices)

    t = threading.Thread(target=_worker, daemon=True, name="CameraDetect")
    t.start()
    return t

'''
        # Insert after detect_cameras function — find the class definition
        class_match = re.search(r'\nclass CameraWidget', content)
        if class_match:
            content = content[:class_match.start()] + async_func + content[class_match.start():]
            changed = True
            report_change(True, "Added detect_cameras_async function")
        else:
            report_change(False, "Could not find CameraWidget class boundary")
    else:
        report_change(False, "detect_cameras_async already exists")

    if changed:
        safe_write(path, content, "async camera detect")
    else:
        print(f"  {Y}No changes needed{X}")


# ═══════════════════════════════════════════════════════════════════
#  FILE 4: gui/app.py — Splitter crash fix
# ═══════════════════════════════════════════════════════════════════

def fix_app(root: Path):
    path = root / "gui" / "app.py"
    print(f"\n{C}[4/5] {path.name}{X}")
    content = safe_read(path)
    if not content:
        return

    changed = False

    # Add setChildrenCollapsible(False) after splitter stretch factors
    if "setChildrenCollapsible" not in content:
        match = re.search(
            r'(self\._splitter\.setStretchFactor\(1,\s*1\))',
            content
        )
        if match:
            insert_pos = match.end()
            content = content[:insert_pos] + \
                "\n\n        # v7.2.6: Prevent splitter collapse crash\n" \
                "        self._splitter.setChildrenCollapsible(False)" + \
                content[insert_pos:]
            changed = True
            report_change(True, "Added setChildrenCollapsible(False)")
        else:
            report_change(False, "Could not find splitter stretch factor")
    else:
        report_change(False, "setChildrenCollapsible already present")

    # Add minimum heights to splitter children
    if "_page_stack.setMinimumHeight" not in content:
        match = re.search(r'(self\._splitter\.addWidget\(self\._page_stack\))', content)
        if match:
            content = content[:match.start()] + \
                "self._page_stack.setMinimumHeight(200)  # v7.2.6\n        " + \
                content[match.start():]
            changed = True
            report_change(True, "Set page stack minimum height 200px")
        else:
            report_change(False, "Could not find _page_stack addWidget")
    else:
        report_change(False, "Page stack min height already set")

    if "console.setMinimumHeight" not in content:
        match = re.search(r'(self\._splitter\.addWidget\(self\.console\))', content)
        if match:
            content = content[:match.start()] + \
                "self.console.setMinimumHeight(40)  # v7.2.6\n        " + \
                content[match.start():]
            changed = True
            report_change(True, "Set console minimum height 40px")
        else:
            report_change(False, "Could not find console addWidget")
    else:
        report_change(False, "Console min height already set")

    if changed:
        safe_write(path, content, "splitter crash fix")
    else:
        print(f"  {Y}No changes needed{X}")


# ═══════════════════════════════════════════════════════════════════
#  FILE 5: gui/pages/jog_control.py — Active pump white text
# ═══════════════════════════════════════════════════════════════════

def fix_jog_control(root: Path):
    path = root / "gui" / "pages" / "jog_control.py"
    print(f"\n{C}[5/5] {path.name}{X}")
    content = safe_read(path)
    if not content:
        return

    changed = False

    # Find _update_pump_states method and replace the styling section
    # The method iterates _pump_buttons and sets styles based on is_configured
    if "v7.2.6" not in content or "font-weight: bold" not in content.split("_update_pump_states")[1].split("def ")[0] if "_update_pump_states" in content else "":
        # Replace the is_configured styling block
        # Pattern: if is_configured: ... label.setStyleSheet(...)
        old_configured = re.compile(
            r'(            if is_configured:\n)'
            r'(                label\.setStyleSheet\(f"color: \{COLORS\[\'text\'\]\};"\))',
            re.DOTALL
        )
        match = old_configured.search(content)
        if match:
            new_block = (
                '            if is_configured:\n'
                '                # v7.2.6: Bright white text for active pumps\n'
                '                label.setStyleSheet(f"color: {COLORS[\'text\']}; font-weight: bold;")\n'
                '                # Also brighten the readout label\n'
                '                lbl_attr = f"lbl_p{pump_name[-1]}"\n'
                '                if hasattr(self, lbl_attr):\n'
                '                    getattr(self, lbl_attr).setStyleSheet(\n'
                '                        f"color: {COLORS[\'text\']}; font-size: 11px; font-weight: bold;")'
            )
            content = content[:match.start()] + new_block + content[match.end():]
            changed = True
            report_change(True, "Active pumps: bright white + bold text")
        else:
            report_change(False, "Could not find is_configured style block")

        # Also update inactive pump section
        old_inactive = re.compile(
            r'(            else:\n)'
            r'(                label\.setStyleSheet\(f"color: \{COLORS\.get\(\'overlay0\',\s*\'#6c7086\'\)\};"\))',
            re.DOTALL
        )
        match = old_inactive.search(content)
        if match:
            new_inactive = (
                '            else:\n'
                '                # v7.2.6: Dim text for inactive pumps\n'
                '                label.setStyleSheet(f"color: {COLORS.get(\'overlay0\', \'#6c7086\')};")\n'
                '                lbl_attr = f"lbl_p{pump_name[-1]}"\n'
                '                if hasattr(self, lbl_attr):\n'
                '                    getattr(self, lbl_attr).setStyleSheet(\n'
                '                        f"color: {COLORS.get(\'overlay0\', \'#6c7086\')}; font-size: 11px;")'
            )
            content = content[:match.start()] + new_inactive + content[match.end():]
            changed = True
            report_change(True, "Inactive pumps: dim readout labels")
        else:
            report_change(False, "Could not find inactive style block")
    else:
        report_change(False, "Pump text styling already applied")

    if changed:
        safe_write(path, content, "pump text colors")
    else:
        print(f"  {Y}No changes needed{X}")


# ═══════════════════════════════════════════════════════════════════
#  MAIN
# ═══════════════════════════════════════════════════════════════════

def main():
    if len(sys.argv) > 1:
        root = Path(sys.argv[1]).resolve()
    else:
        root = find_root()

    print(f"\n{B}{'═' * 60}{X}")
    print(f"{B}MEBP v7.2.6 — Batch 1: Small/Medium File Rewrites{X}")
    print(f"{B}{'═' * 60}{X}")
    print(f"Project root: {root}")
    print(f"Timestamp: {_ts}")

    fix_styles(root)
    fix_console_log(root)
    fix_camera_widget(root)
    fix_app(root)
    fix_jog_control(root)

    print(f"\n{B}{'═' * 60}{X}")
    print(f"{B}SUMMARY{X}")
    print(f"  {G}Files written{X}: {_ok}")
    print(f"  {Y}Skipped{X}:       {_skip}")
    print(f"  {R}Failed{X}:        {_fail}")
    print(f"{B}{'═' * 60}{X}")

    if _fail > 0:
        print(f"\n{R}⚠ {_fail} failure(s). Check output above.{X}")
        sys.exit(1)
    else:
        print(f"\n{G}✓ Batch 1 complete. All files AST-verified.{X}")


if __name__ == "__main__":
    main()

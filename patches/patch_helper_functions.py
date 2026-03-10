#!/usr/bin/env python3
"""
MEBP v7.2.7 — Helper Functions Page Deployment Script

This script:
1. Copies SupportClasses/ImagePathPlanner.py (new file)
2. Copies gui/pages/helper_functions.py (new file)
3. Patches gui/app.py to add the Helper Functions page

The Helper Functions page is inserted at index 6 (between Print Monitor
and Settings). Settings moves from index 6 to index 7.

All changes are idempotent — safe to re-run.
"""

import ast
import re
import sys
import shutil
from pathlib import Path
from datetime import datetime


# ═══════════════════════════════════════════════════════════════════
#  Utilities
# ═══════════════════════════════════════════════════════════════════

def find_root() -> Path:
    """Find MEBP project root by looking for SupportClasses/ + gui/."""
    candidates = [
        Path(__file__).resolve().parent.parent.parent,  # patches/v727/ → root
        Path(__file__).resolve().parent.parent,          # patches/ → root
        Path(__file__).resolve().parent,                 # root itself
        Path.cwd(),
    ]
    for p in candidates:
        if (p / "SupportClasses").is_dir() and (p / "gui").is_dir():
            return p
    print("ERROR: Cannot find MEBP project root (need SupportClasses/ + gui/)")
    sys.exit(1)


def safe_read(path: Path) -> str:
    """Read file, return empty string if missing."""
    if not path.exists():
        return ""
    return path.read_text(encoding="utf-8")


def safe_write(path: Path, content: str, label: str) -> bool:
    """AST-verify → write. Returns False on AST failure."""
    try:
        ast.parse(content)
    except SyntaxError as e:
        print(f"  ✗ AST FAIL for {label}: {e}")
        print(f"    NOT writing {path}")
        return False
    path.write_text(content, encoding="utf-8")
    print(f"  ✓ Written: {path}")
    return True


def find_method(content: str, name: str):
    """Find method boundaries using regex."""
    pattern = re.compile(
        rf'^(    def {re.escape(name)}\(self.*?\n)'
        rf'(.*?)'
        rf'(?=\n    def |\nclass |\Z)',
        re.DOTALL | re.MULTILINE
    )
    return pattern.search(content)


# Terminal colors
GREEN = "\033[92m"
RED = "\033[91m"
YELLOW = "\033[93m"
CYAN = "\033[96m"
RESET = "\033[0m"

ok_count = 0
skip_count = 0
miss_count = 0


def report(status, msg):
    global ok_count, skip_count, miss_count
    if status == "OK":
        ok_count += 1
        print(f"  {GREEN}✓ OK:{RESET} {msg}")
    elif status == "SKIP":
        skip_count += 1
        print(f"  {YELLOW}○ SKIP:{RESET} {msg}")
    elif status == "MISS":
        miss_count += 1
        print(f"  {RED}✗ MISS:{RESET} {msg}")


# ═══════════════════════════════════════════════════════════════════
#  Step 1: Copy new files
# ═══════════════════════════════════════════════════════════════════

def step1_copy_new_files(root: Path, script_dir: Path):
    """Copy ImagePathPlanner.py and helper_functions.py to project."""
    print(f"\n{CYAN}Step 1: Copy new files{RESET}")

    # ImagePathPlanner.py
    src = script_dir / "ImagePathPlanner.py"
    dst = root / "SupportClasses" / "ImagePathPlanner.py"
    if dst.exists():
        report("SKIP", f"ImagePathPlanner.py already exists at {dst}")
    elif src.exists():
        shutil.copy2(src, dst)
        report("OK", f"Copied ImagePathPlanner.py → {dst}")
    else:
        report("MISS", f"Source not found: {src}")

    # helper_functions.py
    src = script_dir / "helper_functions.py"
    dst = root / "gui" / "pages" / "helper_functions.py"
    if dst.exists():
        report("SKIP", f"helper_functions.py already exists at {dst}")
    elif src.exists():
        shutil.copy2(src, dst)
        report("OK", f"Copied helper_functions.py → {dst}")
    else:
        report("MISS", f"Source not found: {src}")


# ═══════════════════════════════════════════════════════════════════
#  Step 2: Patch gui/app.py
# ═══════════════════════════════════════════════════════════════════

def step2_patch_app(root: Path):
    """Patch gui/app.py to add Helper Functions page."""
    print(f"\n{CYAN}Step 2: Patch gui/app.py{RESET}")

    app_path = root / "gui" / "app.py"
    content = safe_read(app_path)
    if not content:
        report("MISS", "gui/app.py not found or empty")
        return

    # ── 2A: Add import ────────────────────────────────────────────
    marker_2a = "from gui.pages.helper_functions import HelperFunctionsPage"
    if marker_2a in content:
        report("SKIP", "2A: HelperFunctionsPage import already present")
    else:
        # Insert after the last gui.pages import
        anchor = re.search(
            r'(from gui\.pages\.print_monitor import PrintMonitorPage\n)',
            content,
        )
        if anchor:
            inject = f"from gui.pages.helper_functions import HelperFunctionsPage\n"
            content = content[:anchor.end()] + inject + content[anchor.end():]
            report("OK", "2A: Added HelperFunctionsPage import")
        else:
            # Fallback: insert after any gui.pages import
            anchor = re.search(
                r'(from gui\.pages\.\w+ import \w+\n)(?!from gui\.pages\.)',
                content,
            )
            if anchor:
                inject = f"from gui.pages.helper_functions import HelperFunctionsPage\n"
                content = content[:anchor.end()] + inject + content[anchor.end():]
                report("OK", "2A: Added HelperFunctionsPage import (fallback anchor)")
            else:
                report("MISS", "2A: Could not find gui.pages import block")

    # ── 2B: Add menu button ───────────────────────────────────────
    marker_2b = '"btn_helpers"'
    if marker_2b in content:
        report("SKIP", "2B: btn_helpers menu item already present")
    else:
        # Insert after btn_monitor line
        anchor = re.search(
            r'(\("btn_monitor",\s*"[^"]*",\s*"Print Monitor"\),?\s*\n)',
            content,
        )
        if anchor:
            inject = '            ("btn_helpers",  "🧰", "Helper Functions"),\n'
            content = content[:anchor.end()] + inject + content[anchor.end():]
            report("OK", "2B: Added btn_helpers menu item")
        else:
            report("MISS", "2B: Could not find btn_monitor menu item")

    # ── 2C: Add HelperFunctionsPage to pages list ─────────────────
    marker_2c = "HelperFunctionsPage()"
    if marker_2c in content:
        report("SKIP", "2C: HelperFunctionsPage already in pages list")
    else:
        # Insert before SettingsPage line
        anchor = re.search(
            r'(\s*SettingsPage\(self\.controller,\s*self\.settings\),\s*#\s*\d+)',
            content,
        )
        if anchor:
            # Determine the old index from the comment
            old_idx_match = re.search(r'#\s*(\d+)', anchor.group(0))
            old_idx = int(old_idx_match.group(1)) if old_idx_match else 6
            new_idx = old_idx  # HelperFunctions takes the old index
            settings_new_idx = old_idx + 1

            inject = f"            HelperFunctionsPage(),{' ' * 42}# {new_idx}  ← v7.2.7\n"

            # Also update the Settings comment to new index
            old_settings_line = anchor.group(0)
            new_settings_line = re.sub(
                r'#\s*\d+',
                f'# {settings_new_idx}  ← was {old_idx}',
                old_settings_line,
            )
            content = content[:anchor.start()] + inject + new_settings_line + content[anchor.end():]
            report("OK", f"2C: Added HelperFunctionsPage at index {new_idx}, Settings → {settings_new_idx}")
        else:
            report("MISS", "2C: Could not find SettingsPage in pages list")

    # ── 2D: Update btn_map ────────────────────────────────────────
    if '"btn_helpers"' in content and '"btn_settings":  7' in content:
        report("SKIP", "2D: btn_map already updated")
    elif '"btn_helpers"' not in content or '"btn_settings":  6' in content:
        # Find the btn_map dict and update it
        btn_map_pattern = re.compile(
            r'(btn_map\s*=\s*\{[^}]*"btn_settings":\s*)(\d+)(\s*,?\s*\})',
            re.DOTALL,
        )
        match = btn_map_pattern.search(content)
        if match:
            old_val = int(match.group(2))
            new_val = old_val + 1

            # First, update settings index
            content = content[:match.start(2)] + str(new_val) + content[match.end(2):]

            # Now add btn_helpers entry before btn_settings
            btn_settings_in_map = re.search(
                r'("btn_settings":\s*\d+)',
                content,
            )
            if btn_settings_in_map and '"btn_helpers"' not in content:
                inject = f'"btn_helpers":   {old_val},\n            '
                pos = btn_settings_in_map.start()
                content = content[:pos] + inject + content[pos:]
                report("OK", f"2D: Added btn_helpers={old_val}, btn_settings={new_val}")
            else:
                report("OK", f"2D: Updated btn_settings to {new_val}")
        else:
            report("MISS", "2D: Could not find btn_map dict")
    else:
        report("SKIP", "2D: btn_map appears already correct")

    # ── 2E: Update titles list in _navigate_to ────────────────────
    if '"Helper Functions"' in content and '"Helper Functions", "Settings"' in content:
        report("SKIP", "2E: titles list already updated")
    else:
        # Update the titles list
        old_titles = '"Print Monitor",\n                      "Settings"'
        new_titles = '"Print Monitor",\n                      "Helper Functions", "Settings"'
        if old_titles in content:
            content = content.replace(old_titles, new_titles, 1)
            report("OK", "2E: Updated titles list")
        else:
            # Try more flexible match
            titles_pattern = re.compile(
                r'("Print Monitor",\s*\n\s*"Settings")',
            )
            match = titles_pattern.search(content)
            if match and '"Helper Functions"' not in content:
                replacement = match.group(0).replace(
                    '"Settings"',
                    '"Helper Functions", "Settings"',
                )
                content = content[:match.start()] + replacement + content[match.end():]
                report("OK", "2E: Updated titles list (flexible match)")
            elif '"Helper Functions"' in content:
                report("SKIP", "2E: Helper Functions already in titles")
            else:
                report("MISS", "2E: Could not find titles list")

    # ── 2F: Update context_titles ─────────────────────────────────
    if '"Helpers"' in content:
        report("SKIP", "2F: context_titles already updated")
    else:
        old_ctx = '"Recordings",\n                          "Settings"'
        new_ctx = '"Recordings",\n                          "Helpers", "Settings"'
        if old_ctx in content:
            content = content.replace(old_ctx, new_ctx, 1)
            report("OK", "2F: Updated context_titles")
        else:
            ctx_pattern = re.compile(r'("Recordings",\s*\n\s*"Settings")')
            match = ctx_pattern.search(content)
            if match:
                replacement = match.group(0).replace(
                    '"Settings"',
                    '"Helpers", "Settings"',
                )
                content = content[:match.start()] + replacement + content[match.end():]
                report("OK", "2F: Updated context_titles (flexible match)")
            else:
                report("MISS", "2F: Could not find context_titles")

    # ── 2G: Update _update_page_gating settings index ─────────────
    # The objectName check is robust, but update the hardcoded index
    marker_2g = "v7.2.7: page gating"
    if marker_2g in content:
        report("SKIP", "2G: page gating already updated")
    else:
        gating_pattern = re.compile(
            r'(elif i == )(\d+)( or btn\.objectName\(\) == "btn_settings":)',
        )
        match = gating_pattern.search(content)
        if match:
            old_idx = int(match.group(2))
            # Only update if it's still the old value (6)
            if old_idx == 6:
                new_line = f"{match.group(1)}7{match.group(3)}  # v7.2.7: page gating"
                content = content[:match.start()] + new_line + content[match.end():]
                report("OK", "2G: Updated settings index 6→7 in page gating")
            elif old_idx == 7:
                report("SKIP", "2G: Settings index already 7")
            else:
                report("SKIP", f"2G: Unexpected settings index {old_idx}")
        else:
            report("MISS", "2G: Could not find page gating elif")

    # ── 2H: Update page gating comment ────────────────────────────
    old_comment = "4=Print, 5=Monitor, 6=Settings"
    new_comment = "4=Print, 5=Monitor, 6=Helpers, 7=Settings"
    if new_comment in content:
        report("SKIP", "2H: Page gating comment already updated")
    elif old_comment in content:
        content = content.replace(old_comment, new_comment, 1)
        report("OK", "2H: Updated page gating comment")
    else:
        report("SKIP", "2H: Page gating comment not found (non-critical)")

    # ── 2I: Wire print_file_created signal + handler ──────────────
    marker_2i = "def _on_helper_print_created"
    if marker_2i in content:
        report("SKIP", "2I: _on_helper_print_created already present")
    else:
        # Add the handler method before _on_hardware_config_changed
        handler_code = '''
    # ════════════════════════════════════════════════════════════════
    #  v7.2.7: HELPER FUNCTIONS INTEGRATION
    # ════════════════════════════════════════════════════════════════

    def _on_helper_print_created(self, filename: str):
        """Helper Functions page created a print file — notify Print Objects tab."""
        setup_page = self._page_widgets[4]
        if hasattr(setup_page, 'tab_objects'):
            tab = setup_page.tab_objects
            if hasattr(tab, '_load_file_by_name'):
                tab._load_file_by_name(filename)
            elif hasattr(tab, '_emit_prints_changed'):
                tab._emit_prints_changed()
        # Auto-switch to Print Setup page
        self._navigate_to(4)

'''
        anchor = re.search(
            r'(\n    # [═]+\n    #  HARDWARE CONFIG MANAGEMENT\n)',
            content,
        )
        if anchor:
            content = content[:anchor.start()] + handler_code + content[anchor.start():]
            report("OK", "2I: Added _on_helper_print_created handler")
        else:
            # Fallback: insert before _on_hardware_config_changed
            anchor2 = re.search(
                r'(\n    def _on_hardware_config_changed\(self)',
                content,
            )
            if anchor2:
                content = content[:anchor2.start()] + handler_code + content[anchor2.start():]
                report("OK", "2I: Added _on_helper_print_created handler (fallback anchor)")
            else:
                report("MISS", "2I: Could not find insertion point for handler")

    # ── 2J: Wire signal in _create_pages ──────────────────────────
    marker_2j = "v7.2.7: Wire helper functions"
    if marker_2j in content:
        report("SKIP", "2J: Helper functions signal wiring already present")
    else:
        # Find the end of page registration loop (after all pages are added)
        # Insert after the _wire_print_manager_to_monitor call
        anchor = re.search(
            r'(self\._wire_print_manager_to_monitor\(\)\s*\n)',
            content,
        )
        if anchor:
            inject = '''
        # v7.2.7: Wire helper functions signal
        for pg in self._page_widgets:
            if hasattr(pg, 'print_file_created'):
                pg.print_file_created.connect(self._on_helper_print_created)
                break

'''
            content = content[:anchor.end()] + inject + content[anchor.end():]
            report("OK", "2J: Wired print_file_created signal")
        else:
            report("MISS", "2J: Could not find _wire_print_manager_to_monitor call")

    # ── Write ─────────────────────────────────────────────────────
    if not safe_write(app_path, content, "gui/app.py"):
        report("MISS", "Failed to write gui/app.py (AST error)")


# ═══════════════════════════════════════════════════════════════════
#  Main
# ═══════════════════════════════════════════════════════════════════

def main():
    global ok_count, skip_count, miss_count

    print(f"\n{'='*60}")
    print(f" MEBP v7.2.7 — Helper Functions Page Deployment")
    print(f" {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print(f"{'='*60}")

    root = find_root()
    print(f"Project root: {root}")

    # Script directory (where the new files live alongside this script)
    script_dir = Path(__file__).resolve().parent

    step1_copy_new_files(root, script_dir)
    step2_patch_app(root)

    # ── Summary ───────────────────────────────────────────────────
    print(f"\n{'='*60}")
    print(f" Summary: {GREEN}{ok_count} OK{RESET} | "
          f"{YELLOW}{skip_count} SKIP{RESET} | "
          f"{RED}{miss_count} MISS{RESET}")
    if miss_count > 0:
        print(f" {RED}⚠ {miss_count} change(s) failed — review output above{RESET}")
    else:
        print(f" {GREEN}✓ All changes applied successfully{RESET}")
    print(f"{'='*60}\n")

    return 0 if miss_count == 0 else 1


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""
MEBP v7.2.7 Fix — Correct issues from initial patch application.

The original patch didn't account for v7.2.6's PrintResultsPage (index 6).
This fix corrects:
  1. Malformed pages list (line 475 — two entries on one line)
  2. Menu button order (helpers should be after results)
  3. btn_map (missing btn_helpers, btn_settings wrong index)
  4. titles list (missing "Helper Functions")
  5. context_titles (missing "Helpers")
  6. Page gating index (7 → 8 for settings)
"""

import ast
import re
import sys
from pathlib import Path
from datetime import datetime


# ═══════════════════════════════════════════════════════════════════
#  Utilities
# ═══════════════════════════════════════════════════════════════════

def find_root() -> Path:
    candidates = [
        Path(__file__).resolve().parent.parent.parent,
        Path(__file__).resolve().parent.parent,
        Path(__file__).resolve().parent,
        Path.cwd(),
    ]
    for p in candidates:
        if (p / "SupportClasses").is_dir() and (p / "gui").is_dir():
            return p
    print("ERROR: Cannot find MEBP project root")
    sys.exit(1)


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


def main():
    global ok_count, skip_count, miss_count

    print(f"\n{'='*60}")
    print(f" MEBP v7.2.7 Fix — Correct patch application issues")
    print(f" {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print(f"{'='*60}")

    root = find_root()
    print(f"Project root: {root}")

    app_path = root / "gui" / "app.py"
    if not app_path.exists():
        print(f"{RED}ERROR: gui/app.py not found{RESET}")
        return 1

    content = app_path.read_text(encoding="utf-8")

    # ── Fix 1: Malformed pages list ───────────────────────────────
    # Line 475 has two entries jammed on one line.
    # Target the broken pattern and split it properly.
    print(f"\n{CYAN}Fix 1: Pages list formatting{RESET}")

    broken_pattern = re.compile(
        r'(PrintResultsPage\(self\.controller,\s*self\.settings\),\s*)'
        r'(#\s*\d+\s*v7\.2\.6\s*)'
        r'(HelperFunctionsPage\(\),\s*#\s*\d+\s*←\s*v7\.2\.7)',
    )
    match = broken_pattern.search(content)
    if match:
        fixed = (
            f"PrintResultsPage(self.controller, self.settings),             # 6  v7.2.6\n"
            f"            HelperFunctionsPage(),                                        # 7  v7.2.7"
        )
        content = content[:match.start()] + fixed + content[match.end():]
        report("OK", "Fix 1: Split pages list entries onto separate lines")
    elif "HelperFunctionsPage()," in content and "PrintResultsPage(" in content:
        # Check if already properly formatted
        results_line = re.search(r'PrintResultsPage\(.*?\),\s*#.*?\n', content)
        helpers_line = re.search(r'HelperFunctionsPage\(\),\s*#.*?\n', content)
        if results_line and helpers_line and results_line.end() <= helpers_line.start():
            report("SKIP", "Fix 1: Pages list already properly formatted")
        else:
            report("MISS", "Fix 1: Pages list in unexpected format")
    else:
        report("MISS", "Fix 1: Could not find broken pages list pattern")

    # ── Fix 2: Menu button order ──────────────────────────────────
    # btn_helpers should be AFTER btn_results, not before
    print(f"\n{CYAN}Fix 2: Menu button order{RESET}")

    # Check current order
    helpers_pos = content.find('"btn_helpers"')
    results_pos = content.find('"btn_results"')

    if helpers_pos >= 0 and results_pos >= 0:
        # Find the line containing btn_helpers in menu_items
        helpers_line_match = re.search(
            r'(\s*\("btn_helpers",\s*"[^"]*",\s*"Helper Functions"\),?\s*\n)',
            content,
        )
        results_line_match = re.search(
            r'(\s*\("btn_results",\s*"[^"]*",\s*"Print Results"\),?\s*#?\s*v7\.2\.6\s*\n)',
            content,
        )

        if helpers_line_match and results_line_match:
            if helpers_line_match.start() < results_line_match.start():
                # helpers is before results — need to swap
                helpers_line_text = helpers_line_match.group(0)
                # Remove helpers line
                content = content[:helpers_line_match.start()] + content[helpers_line_match.end():]

                # Re-find results line (position shifted)
                results_line_match2 = re.search(
                    r'(\s*\("btn_results",\s*"[^"]*",\s*"Print Results"\),?\s*#?\s*v7\.2\.6\s*\n)',
                    content,
                )
                if results_line_match2:
                    # Insert helpers AFTER results
                    insert_pos = results_line_match2.end()
                    content = content[:insert_pos] + helpers_line_text + content[insert_pos:]
                    report("OK", "Fix 2: Moved btn_helpers after btn_results in menu")
                else:
                    report("MISS", "Fix 2: Lost btn_results after removing btn_helpers")
            else:
                report("SKIP", "Fix 2: btn_helpers already after btn_results")
        else:
            report("MISS", "Fix 2: Could not find menu button lines")
    else:
        report("MISS", "Fix 2: btn_helpers or btn_results not found in content")

    # ── Fix 3: btn_map ────────────────────────────────────────────
    print(f"\n{CYAN}Fix 3: btn_map{RESET}")

    # Replace the entire btn_map block
    btn_map_pattern = re.compile(
        r'(btn_map\s*=\s*\{)(.*?)(\})',
        re.DOTALL,
    )
    match = btn_map_pattern.search(content)
    if match:
        current_map = match.group(2)
        needs_fix = False

        # Check if btn_helpers is missing
        if '"btn_helpers"' not in current_map:
            needs_fix = True
        # Check if btn_settings points to 8
        settings_val = re.search(r'"btn_settings":\s*(\d+)', current_map)
        if settings_val and settings_val.group(1) != '8':
            needs_fix = True

        if needs_fix:
            new_map_body = '''
            "btn_hardware":  0,
            "btn_dashboard": 1,
            "btn_jog":       2,
            "btn_calibrate": 3,
            "btn_print":     4,
            "btn_monitor":   5,
            "btn_results":   6,   # v7.2.6
            "btn_helpers":   7,   # v7.2.7
            "btn_settings":  8,   # v7.2.7
        '''
            new_block = f"btn_map = {{{new_map_body}}}"
            content = content[:match.start()] + new_block + content[match.end():]
            report("OK", "Fix 3: Updated btn_map with btn_helpers=7, btn_settings=8")
        else:
            report("SKIP", "Fix 3: btn_map already correct")
    else:
        report("MISS", "Fix 3: Could not find btn_map block")

    # ── Fix 4: titles list ────────────────────────────────────────
    print(f"\n{CYAN}Fix 4: titles list{RESET}")

    # Find the titles list in _navigate_to
    titles_pattern = re.compile(
        r'(titles\s*=\s*\[)(.*?)(\])',
        re.DOTALL,
    )
    # Need to be careful — there might be multiple `titles =` uses.
    # Search specifically within _navigate_to context
    navigate_method = re.search(
        r'def _navigate_to\(self.*?\n(.*?)(?=\n    def |\nclass |\Z)',
        content,
        re.DOTALL,
    )
    if navigate_method:
        nav_content = navigate_method.group(0)
        nav_start = navigate_method.start()

        titles_match = titles_pattern.search(nav_content)
        if titles_match:
            current_titles = titles_match.group(2)

            if '"Helper Functions"' in current_titles:
                report("SKIP", "Fix 4: titles already contains Helper Functions")
            else:
                # Build corrected titles list
                new_titles_body = (
                    '"Hardware Setup", "Dashboard", "Jog Control",\n'
                    '                      "Calibration", "Print Setup", "Print Monitor",\n'
                    '                      "Print Results", "Helper Functions",\n'
                    '                      "Settings"'
                )
                abs_start = nav_start + titles_match.start(2)
                abs_end = nav_start + titles_match.end(2)
                content = content[:abs_start] + new_titles_body + content[abs_end:]
                report("OK", "Fix 4: Added 'Helper Functions' to titles list")
        else:
            report("MISS", "Fix 4: Could not find titles list in _navigate_to")
    else:
        report("MISS", "Fix 4: Could not find _navigate_to method")

    # ── Fix 5: context_titles ─────────────────────────────────────
    print(f"\n{CYAN}Fix 5: context_titles{RESET}")

    ctx_pattern = re.compile(
        r'(context_titles\s*=\s*\[)(.*?)(\])',
        re.DOTALL,
    )
    # Search within _navigate_to scope (re-find since content changed)
    navigate_method2 = re.search(
        r'def _navigate_to\(self.*?\n(.*?)(?=\n    def |\nclass |\Z)',
        content,
        re.DOTALL,
    )
    if navigate_method2:
        nav_content2 = navigate_method2.group(0)
        nav_start2 = navigate_method2.start()

        ctx_match = ctx_pattern.search(nav_content2)
        if ctx_match:
            current_ctx = ctx_match.group(2)

            if '"Helpers"' in current_ctx:
                report("SKIP", "Fix 5: context_titles already contains Helpers")
            else:
                new_ctx_body = (
                    '"Hardware", "Dashboard", "Jog Settings",\n'
                    '                          "Calibration", "Print Settings", "Recordings",\n'
                    '                          "Results", "Helpers",\n'
                    '                          "Settings"'
                )
                abs_start = nav_start2 + ctx_match.start(2)
                abs_end = nav_start2 + ctx_match.end(2)
                content = content[:abs_start] + new_ctx_body + content[abs_end:]
                report("OK", "Fix 5: Added 'Helpers' to context_titles")
        else:
            report("MISS", "Fix 5: Could not find context_titles in _navigate_to")
    else:
        report("MISS", "Fix 5: Could not find _navigate_to method")

    # ── Fix 6: Page gating index ──────────────────────────────────
    print(f"\n{CYAN}Fix 6: Page gating index{RESET}")

    gating_pattern = re.compile(
        r'(elif i == )(\d+)( or btn\.objectName\(\) == "btn_settings":.*)',
    )
    match = gating_pattern.search(content)
    if match:
        current_idx = int(match.group(2))
        if current_idx == 8:
            report("SKIP", "Fix 6: Page gating index already 8")
        else:
            new_line = f"{match.group(1)}8{match.group(3)}"
            # Remove any old v7.2.6/v7.2.7 comments and add clean one
            new_line = re.sub(r'#\s*v7\.2\.\d+.*', '# v7.2.7: settings at 8', new_line)
            if '#' not in new_line:
                new_line += "  # v7.2.7: settings at 8"
            content = content[:match.start()] + new_line + content[match.end():]
            report("OK", f"Fix 6: Updated page gating index {current_idx} → 8")
    else:
        report("MISS", "Fix 6: Could not find page gating elif")

    # ── Fix 7: Page gating comment ────────────────────────────────
    print(f"\n{CYAN}Fix 7: Page gating comment{RESET}")

    old_comments = [
        "4=Print, 5=Monitor, 6=Results, 7=Settings",
        "4=Print, 5=Monitor, 6=Helpers, 7=Settings",
        "4=Print, 5=Monitor, 6=Settings",
    ]
    new_comment = "4=Print, 5=Monitor, 6=Results, 7=Helpers, 8=Settings"
    if new_comment in content:
        report("SKIP", "Fix 7: Comment already correct")
    else:
        replaced = False
        for old in old_comments:
            if old in content:
                content = content.replace(old, new_comment, 1)
                report("OK", f"Fix 7: Updated page gating comment")
                replaced = True
                break
        if not replaced:
            report("SKIP", "Fix 7: Comment not found (non-critical)")

    # ── Write ─────────────────────────────────────────────────────
    print(f"\n{CYAN}Writing patched file{RESET}")

    try:
        ast.parse(content)
    except SyntaxError as e:
        print(f"  {RED}✗ AST FAIL: {e}{RESET}")
        print(f"  NOT writing {app_path}")
        # Write to a debug file for inspection
        debug_path = app_path.parent / "app_v727_fix_debug.py"
        debug_path.write_text(content, encoding="utf-8")
        print(f"  Debug output written to: {debug_path}")
        return 1

    app_path.write_text(content, encoding="utf-8")
    print(f"  {GREEN}✓ Written: {app_path}{RESET}")

    # ── Summary ───────────────────────────────────────────────────
    print(f"\n{'='*60}")
    print(f" Summary: {GREEN}{ok_count} OK{RESET} | "
          f"{YELLOW}{skip_count} SKIP{RESET} | "
          f"{RED}{miss_count} MISS{RESET}")
    if miss_count > 0:
        print(f" {RED}⚠ {miss_count} change(s) failed — review above{RESET}")
    else:
        print(f" {GREEN}✓ All fixes applied successfully{RESET}")
    print(f"{'='*60}\n")

    return 0 if miss_count == 0 else 1


if __name__ == "__main__":
    sys.exit(main())

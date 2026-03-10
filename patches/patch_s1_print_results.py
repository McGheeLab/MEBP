#!/usr/bin/env python3
"""
v7.2.6 Session 1 — Patch v2 (Fixed): Register Print Results page in app.py.

Fixes from v1:
  - F) Regex now captures trailing colon in gating line
  - J) Matches actual `safe_state` callback, injects after full try/except
  - E) Guard checks titles array specifically, not global content

Adds the Print Results page (📋) at index 6, shifting Settings to index 7.
Idempotent: safe to re-run.
"""

import ast
import re
import sys
from pathlib import Path


# ── Helpers ───────────────────────────────────────────────────────

def find_root() -> Path:
    """Find MEBP project root by looking for SupportClasses/ + gui/."""
    candidates = [
        Path(__file__).resolve().parent.parent.parent,
        Path(__file__).resolve().parent.parent,
        Path(__file__).resolve().parent,
        Path.cwd(),
        Path.home() / "Documents" / "GitHub" / "MEBP",
    ]
    for c in candidates:
        if (c / "SupportClasses").is_dir() and (c / "gui").is_dir():
            return c
    print("ERROR: Could not find MEBP project root")
    sys.exit(1)


def safe_read(path: Path) -> str:
    if not path.exists():
        print(f"  ERROR: File not found: {path}")
        return ""
    return path.read_text(encoding="utf-8")


def safe_write(path: Path, content: str, label: str) -> bool:
    """AST-verify → write. Returns False on AST failure."""
    try:
        ast.parse(content)
    except SyntaxError as e:
        print(f"  ✗ AST FAIL for {label}: {e}")
        lines = content.splitlines()
        err_line = e.lineno or 0
        start = max(0, err_line - 5)
        end = min(len(lines), err_line + 5)
        print(f"\n  Context around line {err_line}:")
        for i in range(start, end):
            marker = " >>>" if i + 1 == err_line else "    "
            print(f"  {marker} {i+1}: {lines[i]}")
        return False
    path.write_text(content, encoding="utf-8")
    print(f"  ✓ Written: {label}")
    return True


APPLIED = 0
SKIPPED = 0
MISSED = 0


def report(status, msg):
    global APPLIED, SKIPPED, MISSED
    if status == "OK":
        APPLIED += 1; print(f"  ✓ {msg}")
    elif status == "SKIP":
        SKIPPED += 1; print(f"  ○ SKIP: {msg}")
    else:
        MISSED += 1; print(f"  ✗ MISS: {msg}")


# ══════════════════════════════════════════════════════════════════
#  PATCHES
# ══════════════════════════════════════════════════════════════════

def patch_app_py(root: Path):
    fpath = root / "gui" / "app.py"
    content = safe_read(fpath)
    if not content:
        return

    print(f"\nPatching {fpath.relative_to(root)}...")

    # ── A) Add import for PrintResultsPage ────────────────────────
    marker_a = "from gui.pages.print_results import PrintResultsPage"
    if marker_a not in content:
        m = re.search(r'(from gui\.pages\.print_monitor import PrintMonitorPage)', content)
        if m:
            content = content[:m.end()] + f"\n{marker_a}  # v7.2.6" + content[m.end():]
            report("OK", "A) Added PrintResultsPage import")
        else:
            m2 = re.search(r'(from gui\.pages\.settings_page import SettingsPage)', content)
            if m2:
                content = content[:m2.start()] + f"{marker_a}  # v7.2.6\n" + content[m2.start():]
                report("OK", "A) Added PrintResultsPage import (before SettingsPage)")
            else:
                report("MISS", "A) Could not find import insertion point")
    else:
        report("SKIP", "A) PrintResultsPage import already present")

    # ── B) Add sidebar menu button ────────────────────────────────
    marker_b = '"btn_results"'
    if marker_b not in content:
        m = re.search(r'(\("btn_monitor",\s*"📈",\s*"Print Monitor"\),?)', content)
        if m:
            inject = '\n            ("btn_results",  "📋", "Print Results"),  # v7.2.6'
            content = content[:m.end()] + inject + content[m.end():]
            report("OK", "B) Added btn_results to sidebar menu")
        else:
            report("MISS", "B) Could not find btn_monitor in menu_items")
    else:
        report("SKIP", "B) btn_results already in sidebar menu")

    # ── C) Add page instantiation ─────────────────────────────────
    marker_c = "PrintResultsPage("
    if marker_c not in content:
        m = re.search(
            r'(PrintMonitorPage\(self\.controller,\s*self\.settings\),\s*#\s*5)',
            content
        )
        if m:
            inject = (
                "\n            PrintResultsPage(self.controller, self.settings),         "
                "# 6  v7.2.6"
            )
            content = content[:m.end()] + inject + content[m.end():]
            content = re.sub(
                r'(SettingsPage\(self\.controller,\s*self\.settings\),\s*)#\s*6\b',
                r'\g<1># 7  v7.2.6',
                content, count=1
            )
            report("OK", "C) Added PrintResultsPage to page list, Settings→#7")
        else:
            report("MISS", "C) Could not find PrintMonitorPage instantiation")
    else:
        report("SKIP", "C) PrintResultsPage already instantiated")

    # ── D) Update btn_map ─────────────────────────────────────────
    marker_d = '"btn_results":'
    if marker_d not in content:
        m = re.search(r'("btn_monitor":\s*5,)', content)
        if m:
            content = (content[:m.end()]
                       + '\n            "btn_results":   6,   # v7.2.6'
                       + content[m.end():])
            report("OK", "D) Added btn_results:6 to btn_map")
        else:
            report("MISS", "D) Could not find btn_monitor in btn_map")

        m2 = re.search(r'"btn_settings":\s*6\b', content)
        if m2:
            content = content[:m2.start()] + '"btn_settings":  7  # v7.2.6' + content[m2.end():]
            report("OK", "D2) Updated btn_settings 6→7 in btn_map")
        elif re.search(r'"btn_settings":\s*7\b', content):
            report("SKIP", "D2) btn_settings already at index 7")
        else:
            report("MISS", "D2) Could not find btn_settings: 6 in btn_map")
    else:
        report("SKIP", "D) btn_results already in btn_map")

    # ── E) Update titles array in _navigate_to ────────────────────
    # FIXED: check specifically in the titles= array, not global content
    # The exact pattern: "Print Monitor",\n                      "Settings"]
    titles_match = re.search(
        r'titles\s*=\s*\[.*?"Print Monitor",\s*\n\s*"Settings"\]',
        content, re.DOTALL
    )
    if titles_match:
        # The array has "Print Monitor" directly followed by "Settings" — needs insertion
        m = re.search(
            r'("Print Monitor",\s*\n\s*"Settings"\])',
            content
        )
        if m:
            replacement = ('"Print Monitor",\n'
                           '                      "Print Results",  # v7.2.6\n'
                           '                      "Settings"]')
            content = content[:m.start()] + replacement + content[m.end():]
            report("OK", "E) Added 'Print Results' to titles list")
        else:
            report("MISS", "E) titles pattern found but replacement match failed")
    else:
        # Check if already updated
        if re.search(r'titles\s*=\s*\[.*?"Print Results"', content, re.DOTALL):
            report("SKIP", "E) 'Print Results' already in titles")
        else:
            report("MISS", "E) Could not locate titles array")

    # ── E2) Update context_titles array ───────────────────────────
    ctx_match = re.search(
        r'context_titles\s*=\s*\[.*?"Recordings",\s*\n\s*"Settings"\]',
        content, re.DOTALL
    )
    if ctx_match:
        m = re.search(r'("Recordings",\s*\n\s*"Settings"\])', content)
        if m:
            replacement = ('"Recordings",\n'
                           '                          "Results",  # v7.2.6\n'
                           '                          "Settings"]')
            content = content[:m.start()] + replacement + content[m.end():]
            report("OK", "E2) Added 'Results' to context_titles")
        else:
            report("MISS", "E2) context_titles pattern found but replacement failed")
    else:
        if re.search(r'context_titles\s*=\s*\[.*?"Results"', content, re.DOTALL):
            report("SKIP", "E2) 'Results' already in context_titles")
        else:
            report("MISS", "E2) Could not locate context_titles array")

    # ── F) Update _update_page_gating: Settings i==6 → i==7 ──────
    marker_f = "v7.2.6: settings"
    if marker_f not in content:
        m = re.search(
            r'elif i == 6 or btn\.objectName\(\) == "btn_settings":',
            content
        )
        if m:
            replacement = 'elif i == 7 or btn.objectName() == "btn_settings":  # v7.2.6: settings at 7'
            content = content[:m.start()] + replacement + content[m.end():]
            report("OK", "F) Updated settings gating 6→7 (with colon)")
        else:
            if re.search(r'elif i == 7 or btn\.objectName', content):
                report("SKIP", "F) Settings gating already at index 7")
            else:
                report("MISS", "F) Could not find settings gating pattern")
    else:
        report("SKIP", "F) Settings gating already updated")

    # ── G) Update page indices comment ────────────────────────────
    marker_g = "6=Results"
    if marker_g not in content:
        m = re.search(
            r'(#\s*Page indices:.*?5=Monitor,)\s*6=Settings',
            content, re.DOTALL
        )
        if m:
            replacement = m.group(1) + " 6=Results, 7=Settings  # v7.2.6"
            content = content[:m.start()] + replacement + content[m.end():]
            report("OK", "G) Updated page indices comment")
        else:
            report("SKIP", "G) Page indices comment not found or already updated")
    else:
        report("SKIP", "G) Page indices comment already updated")

    # ── H) Wire recorder to results page ──────────────────────────
    marker_h = "v7.2.6: wire recorder to results"
    if marker_h not in content:
        m = re.search(
            r"(if hasattr\(monitor, 'set_recorder'\):\s*\n\s*monitor\.set_recorder\(self\.recorder\))",
            content
        )
        if m:
            inject = (
                "\n\n            # v7.2.6: wire recorder to results\n"
                "            results_page = pages[6]\n"
                "            if hasattr(results_page, 'set_recorder'):\n"
                "                results_page.set_recorder(self.recorder)"
            )
            content = content[:m.end()] + inject + content[m.end():]
            report("OK", "H) Wired recorder to results page")
        else:
            report("MISS", "H) Could not find recorder wiring block")
    else:
        report("SKIP", "H) Recorder already wired to results page")

    # ── I) Add _on_print_completed_v726 handler ──────────────────
    marker_i = "def _on_print_completed_v726"
    if marker_i not in content:
        m = re.search(
            r"(def _on_monitor_abort\(self\):"
            r".*?setup_page\.print_manager\.abort\(\))",
            content, re.DOTALL
        )
        if m:
            new_method = (
                "\n\n    def _on_print_completed_v726(self):\n"
                '        """v7.2.6: On print completion, notify results page."""\n'
                "        if len(self._page_widgets) > 6:\n"
                "            results_page = self._page_widgets[6]\n"
                "            if hasattr(results_page, 'load_latest_recording'):\n"
                "                try:\n"
                "                    results_page.load_latest_recording()\n"
                "                    logger.info(\n"
                '                        "v7.2.6: Loaded latest recording into results"\n'
                "                    )\n"
                "                except Exception as e:\n"
                "                    logger.warning(\n"
                '                        "v7.2.6: Failed to load recording: "\n'
                "                        + str(e)\n"
                "                    )\n"
            )
            content = content[:m.end()] + new_method + content[m.end():]
            report("OK", "I) Added _on_print_completed_v726 handler")
        else:
            report("MISS", "I) Could not find _on_monitor_abort for insertion")
    else:
        report("SKIP", "I) _on_print_completed_v726 already present")

    # ── J) Hook completion into safe_state callback ───────────────
    # safe_state has two try/except blocks; inject AFTER the second one.
    call_marker = "self._on_print_completed_v726()"
    if call_marker not in content:
        m = re.search(
            r"(def safe_state\(state\):"
            r".*?bridge\.state_signal\.emit\(state\)\s*\n"
            r"\s*except Exception:\s*\n"
            r"\s*pass)",
            content, re.DOTALL
        )
        if m:
            inject = (
                "\n            # v7.2.6: notify results page on completion\n"
                "            try:\n"
                "                from SupportClasses.PrintManager import PrintState as _PS\n"
                "                if state == _PS.COMPLETED:\n"
                "                    self._on_print_completed_v726()\n"
                "            except Exception:\n"
                "                pass"
            )
            content = content[:m.end()] + inject + content[m.end():]
            report("OK", "J) Hooked completion into safe_state callback")
        else:
            report("MISS", "J) Could not find safe_state with two try/except blocks")
    else:
        report("SKIP", "J) Completion hook already present")

    # ── Write result ──────────────────────────────────────────────
    if not safe_write(fpath, content, "gui/app.py"):
        print("  ✗ FAILED to write gui/app.py")


# ══════════════════════════════════════════════════════════════════
#  MAIN
# ══════════════════════════════════════════════════════════════════

def main():
    global APPLIED, SKIPPED, MISSED

    print("=" * 60)
    print("MEBP v7.2.6 Session 1: Print Results Page (v2 — fixed)")
    print("=" * 60)

    root = find_root()
    print(f"Project root: {root}")

    results_file = root / "gui" / "pages" / "print_results.py"
    if not results_file.exists():
        print(f"\n  ERROR: {results_file} not found!")
        print("  Please copy print_results.py to gui/pages/ first.")
        sys.exit(1)
    else:
        print(f"\n  ✓ print_results.py found")

    patch_app_py(root)

    print("\n" + "=" * 60)
    print(f"Summary: {APPLIED} applied | {SKIPPED} skipped | {MISSED} missed")
    print("=" * 60)

    if MISSED > 0:
        print("\n⚠ Some patches missed — review output above")
        sys.exit(1)
    else:
        print("\n✓ All patches applied successfully")


if __name__ == "__main__":
    main()

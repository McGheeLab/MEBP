#!/usr/bin/env python3
"""
MEBP v7.2.6 — Print Monitor Full Redesign Patch

Strategy: Full file replacement of gui/pages/print_monitor.py
This avoids all anchor-matching issues since we replace the entire file.

Preserves:
- All external API (signals, methods called by app.py)
- Constructor signature
- Context panel (job queue + recording browser)

New features:
- LiveWellPreview: single XY zoomable/pannable real-time preview
- Unified controls (Start/Pause/Abort in main page)
- Compact plate overview
- Cleaner progress display
- Auto-follow mode for needle tracking

Usage:
    python3 patch_print_monitor_redesign.py
"""

import ast
import sys
import shutil
from pathlib import Path
from datetime import datetime


# ── Terminal Colors ───────────────────────────────────────────────
GREEN = "\033[92m"
YELLOW = "\033[93m"
RED = "\033[91m"
BLUE = "\033[94m"
RESET = "\033[0m"
BOLD = "\033[1m"


def find_root() -> Path:
    """Find MEBP project root by looking for SupportClasses/ + gui/."""
    candidates = [
        Path(__file__).resolve().parent.parent.parent,  # patches/v726/ → root
        Path.cwd(),
        Path.home() / "Documents" / "GitHub" / "MEBP",
    ]
    for p in candidates:
        if (p / "SupportClasses").is_dir() and (p / "gui").is_dir():
            return p
    print(f"{RED}ERROR: Could not find MEBP project root{RESET}")
    sys.exit(1)


def ast_verify(content: str, label: str) -> bool:
    """AST-parse content. Returns True on success."""
    try:
        ast.parse(content)
        return True
    except SyntaxError as e:
        print(f"  {RED}AST FAIL{RESET} ({label}): {e}")
        return False


def verify_api(content: str) -> list[str]:
    """Verify all required API methods are present. Returns list of missing."""
    required = [
        "get_page_title", "get_context_widget",
        "set_hardware_config", "set_workspace", "set_recorder",
        "set_microsteps_per_micron", "receive_job",
        "on_print_progress", "on_print_state_changed", "on_progress_update",
        "setup_plate", "update_syringe_state", "update_needle_position",
        "update_upcoming_waypoints", "update_tracking_error", "reset",
    ]
    missing = []
    for method in required:
        if f"def {method}(self" not in content:
            missing.append(method)
    
    # Check signals
    for sig in ["pause_requested", "resume_requested", "abort_requested", "start_requested"]:
        if f"{sig} = Signal" not in content:
            missing.append(f"signal:{sig}")
    
    return missing


# ═══════════════════════════════════════════════════════════════════
# NEW FILE CONTENT
# ═══════════════════════════════════════════════════════════════════

NEW_PRINT_MONITOR = r'''"""
print_monitor.py — Print Monitor Page for MEBP v7.2.6.

Complete redesign: single XY live well preview with zoom/pan,
compact plate overview, unified controls, clean progress display.

Layout:
┌──────────────┬────────────────────────────────────────────────────┐
│ Plate        │  Live Well Preview (XY, zoomable/pannable)         │
│ Overview     │  - Well boundary, completed path, upcoming path    │
│ (compact)    │  - Needle crosshair + tracking error ring          │
│              │  - Grid + mm ruler + zoom toolbar                  │
├──────────────┼─────────────────────────┬──────────────────────────┤
│ Syringe      │  Print Progress          │  Controls + State       │
│ P1/P2/P3     │  Job/Well/Layer/Step/ETA │  Start/Pause/Abort      │
│ + Needle     │  Progress bar            │  Tracking + Controller  │
└──────────────┴─────────────────────────┴──────────────────────────┘

Context Panel: Job Queue + Recording Browser
'''
# We'll read the new file content from disk instead of embedding it inline
# to avoid string escaping issues with the 1400-line file.


def main():
    print(f"\n{BOLD}{BLUE}═══════════════════════════════════════════════════{RESET}")
    print(f"{BOLD}{BLUE}  MEBP v7.2.6 — Print Monitor Redesign Patch{RESET}")
    print(f"{BOLD}{BLUE}═══════════════════════════════════════════════════{RESET}\n")

    root = find_root()
    print(f"  Project root: {root}\n")

    target = root / "gui" / "pages" / "print_monitor.py"
    
    # ── Step 1: Read the new file content ─────────────────────────
    # The new file is located next to this patch script
    new_file = Path(__file__).resolve().parent / "print_monitor_v726.py"
    
    if not new_file.exists():
        print(f"  {RED}ERROR: New file not found: {new_file}{RESET}")
        print(f"  Expected: patches/v726/print_monitor_v726.py")
        sys.exit(1)
    
    new_content = new_file.read_text(encoding="utf-8")
    print(f"  {GREEN}✓{RESET} Read new file: {new_file.name} ({len(new_content)} chars)")
    
    # ── Step 2: Verify new content ────────────────────────────────
    if not ast_verify(new_content, "new print_monitor.py"):
        print(f"\n  {RED}ABORT: New file has syntax errors{RESET}")
        sys.exit(1)
    print(f"  {GREEN}✓{RESET} AST verification passed")
    
    missing = verify_api(new_content)
    if missing:
        print(f"  {RED}ERROR: Missing API: {missing}{RESET}")
        sys.exit(1)
    print(f"  {GREEN}✓{RESET} All required API methods and signals present")
    
    # ── Step 3: Check idempotency ─────────────────────────────────
    if target.exists():
        existing = target.read_text(encoding="utf-8")
        if "v7.2.6" in existing and "LiveWellPreview" in existing:
            print(f"\n  {YELLOW}○ SKIP{RESET}: print_monitor.py already contains v7.2.6 redesign")
            print(f"  {GREEN}All done — no changes needed.{RESET}")
            return
    
    # ── Step 4: Backup existing file ──────────────────────────────
    if target.exists():
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        backup = target.with_suffix(f".bak_v726_{ts}")
        shutil.copy2(target, backup)
        print(f"  {GREEN}✓{RESET} Backup: {backup.name}")
    else:
        print(f"  {YELLOW}⚠{RESET} Target file does not exist (new creation)")
    
    # ── Step 5: Write new file ────────────────────────────────────
    target.write_text(new_content, encoding="utf-8")
    print(f"  {GREEN}✓{RESET} Written: {target}")
    
    # ── Step 6: Final verification ────────────────────────────────
    written = target.read_text(encoding="utf-8")
    if not ast_verify(written, "written file"):
        print(f"\n  {RED}CRITICAL: Written file fails AST! Restoring backup...{RESET}")
        if backup.exists():
            shutil.copy2(backup, target)
            print(f"  Backup restored: {backup.name}")
        sys.exit(1)
    
    print(f"\n{BOLD}{GREEN}═══════════════════════════════════════════════════{RESET}")
    print(f"{BOLD}{GREEN}  SUCCESS — Print Monitor redesigned (v7.2.6){RESET}")
    print(f"{BOLD}{GREEN}═══════════════════════════════════════════════════{RESET}")
    print(f"\n  Changes:")
    print(f"    • gui/pages/print_monitor.py — REPLACED (full redesign)")
    print(f"    • No changes to app.py (API preserved)")
    print(f"\n  New features:")
    print(f"    • LiveWellPreview — XY zoomable/pannable real-time view")
    print(f"    • Unified Start/Pause/Abort controls in main page")
    print(f"    • Compact plate overview (left panel)")
    print(f"    • Auto-follow mode for needle tracking")
    print(f"    • Grid + mm ruler in preview")
    print(f"\n  Verification:")
    print(f"    python3 -c \"import ast; ast.parse(open('{target}').read())\"")
    print(f"    python main.py  # launch and navigate to Monitor page\n")


if __name__ == "__main__":
    main()

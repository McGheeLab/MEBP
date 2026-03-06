#!/usr/bin/env python3
"""
patch_fix_duplicate_generate_btn.py  (v7.3.1)
print_setup.py contains two back-to-back identical blocks that create
self.btn_generate_print and self._gen_status_label.  Remove the second one.
"""
import ast, re, sys, shutil
from pathlib import Path
from datetime import datetime

GREEN = "\033[92m"; RED = "\033[91m"; YELLOW = "\033[93m"; CYAN = "\033[96m"; RESET = "\033[0m"
GUARD = "v7.3.1-dedup-genbtn"

def find_root():
    for p in [Path.cwd(), Path(__file__).parent]:
        for c in [p, p.parent, p.parent.parent]:
            if (c / "SupportClasses").is_dir() and (c / "gui").is_dir():
                return c.resolve()
    raise RuntimeError("Cannot locate MEBP root")

def ast_ok(text, tag):
    try:
        ast.parse(text); return True
    except SyntaxError as e:
        print(f"  {RED}AST FAIL after {tag}: {e}{RESET}"); return False

def main():
    root   = find_root()
    target = root / "gui" / "pages" / "print_setup.py"
    print(f"{CYAN}Target: {target}{RESET}")
    if not target.exists():
        print(f"  {RED}File not found{RESET}"); sys.exit(1)

    content = target.read_text(encoding="utf-8")
    if GUARD in content:
        print(f"  {YELLOW}SKIP: already applied{RESET}"); return

    # The duplicate block: a v7.2.5 comment + QPushButton + setStyleSheet + setToolTip
    # + clicked.connect + addWidget + blank line + QLabel + setStyleSheet + setWordWrap
    # + addWidget  — appears TWICE with identical text
    # Strategy: find the second occurrence of the marker comment and remove that block
    MARKER = "# v7.2.5: Generate Print button — validate + build execution plan"
    idx1 = content.find(MARKER)
    if idx1 == -1:
        # Try alternative marker (the QPushButton line itself)
        MARKER = 'self.btn_generate_print = QPushButton("⚙ Generate Print")'
        idx1 = content.find(MARKER)
        if idx1 == -1:
            print(f"  {RED}MISS: cannot find Generate Print button block{RESET}"); sys.exit(1)

    idx2 = content.find(MARKER, idx1 + len(MARKER))
    if idx2 == -1:
        print(f"  {YELLOW}SKIP: no duplicate found — already clean{RESET}"); return

    # Find end of the second block: look for next non-indented statement or blank+non-indent
    # Safest: remove from idx2 back to the nearest preceding newline, up to
    # the next `layout.addWidget(self._gen_status_label)` after idx2
    END_MARKER = "layout.addWidget(self._gen_status_label)"
    idx_end = content.find(END_MARKER, idx2)
    if idx_end == -1:
        print(f"  {RED}MISS: cannot find end of duplicate block{RESET}"); sys.exit(1)
    # Include the newline after the end marker
    idx_end = content.index("\n", idx_end) + 1

    # Walk back to include the preceding newline before the second block start
    block_start = content.rindex("\n", 0, idx2) + 1  # start of the line containing MARKER

    removed = content[block_start:idx_end]
    content = content[:block_start] + content[idx_end:]

    if not ast_ok(content, "dedup"): sys.exit(1)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    shutil.copy2(target, target.with_suffix(f".bak_{GUARD}_{ts}"))
    target.write_text(content, encoding="utf-8")
    print(f"  {GREEN}Removed duplicate Generate Print button block ({len(removed)} chars){RESET}")

if __name__ == "__main__":
    main()

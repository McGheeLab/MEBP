#!/usr/bin/env python3
"""
patch_v726_xy_bare_lock_fix.py
Remove bare 'with self._serial_lock:' blocks from XYStage.py.
Shows full context of each hit then de-indents the body.
"""
import ast, re, sys, shutil
from pathlib import Path
from datetime import datetime

GREEN = "\033[92m"; RED = "\033[91m"; RESET = "\033[0m"
OK = f"{GREEN}  v{RESET}"; MISS = f"{RED}  x{RESET}"

def find_root():
    here = Path(__file__).resolve().parent
    for d in [here, here.parent, here.parent.parent]:
        if (d / "SupportClasses").is_dir() and (d / "gui").is_dir():
            return d
    sys.exit("ERROR: Cannot find MEBP project root")

ROOT = find_root()
xy_path = ROOT / "SupportClasses" / "XYStage.py"
content = xy_path.read_text(encoding="utf-8")
lines = content.splitlines(keepends=True)

# Show context around every lock occurrence
print("=== Lock occurrences in XYStage.py ===")
for i, line in enumerate(lines):
    if "with self._serial_lock:" in line:
        start = max(0, i-3)
        end = min(len(lines), i+12)
        print(f"\n--- Line {i+1} ---")
        for j in range(start, end):
            print(f"  {j+1:4d}| {lines[j]}", end="")

# Now remove ALL bare 'with self._serial_lock:' blocks by de-indenting their bodies
# Pattern: line with '        with self._serial_lock:\n' followed by indented body
# The body lines start with '            ' (12 spaces) -> de-indent to '        ' (8 spaces)

def remove_all_lock_wraps(text):
    """Remove all 'with self._serial_lock:' blocks, de-indenting the body."""
    result = []
    i = 0
    lines = text.splitlines(keepends=True)
    removed = 0
    while i < len(lines):
        line = lines[i]
        # Match: 8-space indent + 'with self._serial_lock:'
        if re.match(r'^        with self\._serial_lock:\s*$', line):
            removed += 1
            i += 1  # skip the 'with' line
            # De-indent all following lines that are indented >= 12 spaces
            # Stop when we hit a line with <= 8 spaces of indent (or blank then non-indented)
            while i < len(lines):
                body_line = lines[i]
                # blank line - keep and continue looking
                if body_line.strip() == '':
                    result.append(body_line)
                    i += 1
                    # peek: if next non-blank line has <= 8 spaces, we're done
                    j = i
                    while j < len(lines) and lines[j].strip() == '':
                        j += 1
                    if j < len(lines):
                        next_indent = len(lines[j]) - len(lines[j].lstrip())
                        if next_indent <= 8:
                            break
                elif body_line.startswith('            '):
                    # De-indent by 4 spaces (12 -> 8)
                    result.append(body_line[4:])
                    i += 1
                else:
                    # Back to 8-space or less indent — end of with block
                    break
        else:
            result.append(line)
            i += 1
    print(f"\nRemoved {removed} lock wrap(s)")
    return ''.join(result)

print("\n=== Removing lock wraps ===")
new_content = remove_all_lock_wraps(content)

try:
    ast.parse(new_content)
    print(f"{OK} AST OK")
except SyntaxError as e:
    print(f"{MISS} AST FAIL: {e}")
    # Show context around the error
    err_lines = new_content.splitlines()
    el = e.lineno or 0
    for j in range(max(0,el-3), min(len(err_lines), el+3)):
        print(f"  {j+1:4d}| {err_lines[j]}")
    sys.exit(1)

ts = datetime.now().strftime("%Y%m%d_%H%M%S")
shutil.copy2(xy_path, xy_path.with_suffix(f".bak_v726_{ts}"))
xy_path.write_text(new_content, encoding="utf-8")
print(f"{OK} XYStage.py written — all serial lock wraps removed")

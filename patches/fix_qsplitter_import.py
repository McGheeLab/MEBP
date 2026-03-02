#!/usr/bin/env python3
"""fix_qsplitter_v2.py — Force-add QSplitter to the PySide6 import line."""
import os, sys

path = "gui/app.py"
if not os.path.isfile(path):
    print(f"ERROR: {path} not found"); sys.exit(1)

with open(path) as f:
    lines = f.readlines()

# Find the import block and check if QSplitter is already there
for i, line in enumerate(lines):
    if "from PySide6.QtWidgets import" in line:
        # Scan the import block (may span multiple lines)
        import_block = ""
        j = i
        while j < len(lines):
            import_block += lines[j]
            if ")" in lines[j]:
                break
            j += 1
        
        if "QSplitter" in import_block:
            print("QSplitter already in import block")
            sys.exit(0)
        
        # Find the closing ) line and insert QSplitter before it
        for k in range(i, j + 1):
            if ")" in lines[k]:
                # Add QSplitter on the line before the closing paren
                lines.insert(k, "    QSplitter,\n")
                print(f"OK: Inserted QSplitter import at line {k+1}")
                break
        break

with open(path, "w") as f:
    f.writelines(lines)
print("Done — app.py updated")
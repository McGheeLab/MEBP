#!/usr/bin/env python3
"""
Add execution.mode setting to settings.json and Settings.py defaults.

Adds:
  settings.json → "execution": {"mode": "position"}
  Settings.py DEFAULTS → "execution": {"mode": "position"}

Modes:
  "discrete"  — legacy command-based (v7.0)
  "position"  — trajectory with position commands (DEFAULT, simplest)
  "kalman"    — trajectory with Kalman velocity control
  "pid"       — trajectory with PID velocity control
"""

import json, sys, re, shutil
from pathlib import Path
from datetime import datetime

G="\033[92m"; R="\033[91m"; X="\033[0m"; B="\033[1m"

def find_root():
    if len(sys.argv) > 1:
        p = Path(sys.argv[1])
        if (p / "gui").is_dir(): return p
    for c in [Path.cwd(), Path(__file__).resolve().parent.parent]:
        if (c / "gui").is_dir(): return c
    print(f"{R}Cannot find MEBP root{X}"); sys.exit(1)

def main():
    root = find_root()
    print(f"\n{B}=== Add execution.mode setting ==={X}")

    # 1. Update settings.json
    sj_path = root / "settings.json"
    if sj_path.exists():
        data = json.loads(sj_path.read_text())
        if "execution" not in data:
            data["execution"] = {"mode": "position"}
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            shutil.copy2(sj_path, sj_path.with_suffix(f".bak_exec_{ts}"))
            sj_path.write_text(json.dumps(data, indent=2))
            print(f"  {G}✓{X} settings.json: added execution.mode = 'position'")
        else:
            print(f"  Already has execution section: {data['execution']}")
    else:
        print(f"  {R}settings.json not found{X}")

    # 2. Update Settings.py DEFAULTS
    sp_path = root / "SupportClasses" / "Settings.py"
    if sp_path.exists():
        content = sp_path.read_text()
        if '"execution"' not in content:
            # Find the end of DEFAULTS dict — insert before closing }
            # Look for the last entry before the closing brace
            insert_marker = '    "ui":'
            if insert_marker in content:
                # Insert after the ui section
                m = re.search(r'("ui"\s*:\s*\{[^}]*\}),?\s*\n(\})', content)
                if m:
                    insert_pos = m.start(2)
                    insertion = '    "execution": {\n        "mode": "position",\n    },\n'
                    content = content[:insert_pos] + insertion + content[insert_pos:]

                    import ast
                    try:
                        ast.parse(content)
                        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
                        shutil.copy2(sp_path, sp_path.with_suffix(f".bak_exec_{ts}"))
                        sp_path.write_text(content)
                        print(f"  {G}✓{X} Settings.py DEFAULTS: added execution.mode")
                    except SyntaxError as e:
                        print(f"  {R}AST FAIL after insertion: {e}{X}")
                        print(f"  Skipping Settings.py — manually add to DEFAULTS")
                else:
                    print(f"  Could not find insertion point in DEFAULTS")
            else:
                print(f"  'ui' section not found in DEFAULTS — add manually")
        else:
            print(f"  Already has execution in DEFAULTS")
    else:
        print(f"  {R}Settings.py not found{X}")

    # 3. Clean cache
    import os
    for dp, dn, _ in os.walk(root):
        if "__pycache__" in dn:
            shutil.rmtree(Path(dp) / "__pycache__")

    print(f"\n  {G}✓ Done!{X}")
    print(f"  Modes: discrete | position (default) | kalman | pid")
    print(f"  Change in settings.json → execution.mode")

if __name__ == "__main__":
    main()

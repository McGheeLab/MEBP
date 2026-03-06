#!/usr/bin/env python3
"""
Diagnose and fix _get_controller / on_status_update placement in print_monitor.py.

The Session 2 patch likely inserted these methods outside the PrintMonitorPage class
or at incorrect indentation.
"""

import ast, sys, re
from pathlib import Path

G = "\033[92m"; R = "\033[91m"; Y = "\033[93m"; X = "\033[0m"; B = "\033[1m"

def find_root():
    if len(sys.argv) > 1:
        p = Path(sys.argv[1])
        if (p / "gui").is_dir():
            return p
    for c in [Path.cwd(), Path(__file__).resolve().parent.parent]:
        if (c / "gui").is_dir():
            return c
    print(f"{R}Cannot find MEBP root{X}"); sys.exit(1)

def main():
    root = find_root()
    path = root / "gui" / "pages" / "print_monitor.py"
    content = path.read_text(encoding="utf-8")
    lines = content.split("\n")

    print(f"\n{B}=== Diagnosing print_monitor.py ==={X}\n")

    # 1. Find all occurrences of _get_controller and on_status_update
    for target in ["_get_controller", "on_status_update"]:
        print(f"  {B}Searching for '{target}':{X}")
        for i, line in enumerate(lines, 1):
            if target in line and "def " in line:
                indent = len(line) - len(line.lstrip())
                print(f"    Line {i}: indent={indent} spaces: {line.rstrip()}")

    # 2. Find the PrintMonitorPage class and its method indentation
    print(f"\n  {B}PrintMonitorPage class structure:{X}")
    class_start = None
    class_indent = None
    method_indent = None
    for i, line in enumerate(lines, 1):
        if re.match(r'^class PrintMonitorPage', line):
            class_start = i
            class_indent = 0
            print(f"    Class starts at line {i}")
        if class_start and re.match(r'^    def __init__\(self', line):
            method_indent = 4
            print(f"    __init__ at line {i} (indent=4)")
            break

    # 3. Check if _get_controller is at indent=4 (class method) or something else
    gc_lines = [(i, line) for i, line in enumerate(lines, 1)
                if "def _get_controller" in line]

    osu_lines = [(i, line) for i, line in enumerate(lines, 1)
                 if "def on_status_update" in line]

    needs_fix = False

    for lineno, line in gc_lines:
        indent = len(line) - len(line.lstrip())
        if indent != 4:
            print(f"\n  {R}✗ _get_controller at line {lineno} has indent={indent} (should be 4){X}")
            needs_fix = True
        else:
            print(f"\n  {G}✓ _get_controller at line {lineno} indent OK{X}")

    for lineno, line in osu_lines:
        indent = len(line) - len(line.lstrip())
        if indent != 4:
            print(f"  {R}✗ on_status_update at line {lineno} has indent={indent} (should be 4){X}")
            needs_fix = True
        else:
            print(f"  {G}✓ on_status_update at line {lineno} indent OK{X}")

    if not gc_lines:
        print(f"\n  {R}✗ _get_controller NOT FOUND at all{X}")
        needs_fix = True

    if not osu_lines:
        print(f"\n  {R}✗ on_status_update NOT FOUND at all{X}")
        needs_fix = True

    # 4. Show context around line 952 (the error line)
    print(f"\n  {B}Context around line 952 (error site):{X}")
    for i in range(max(0, 948), min(len(lines), 958)):
        marker = " >>>" if i == 951 else "    "
        print(f"  {marker} {i+1:4d}: {lines[i]}")

    # 5. Apply fix if needed
    if not needs_fix:
        # Check if it's a stale .pyc issue or if the methods are after the class
        # Find where PrintMonitorPage class ends
        print(f"\n  {B}Checking if methods are inside the class body...{X}")

        tree = ast.parse(content)
        for node in ast.walk(tree):
            if isinstance(node, ast.ClassDef) and node.name == "PrintMonitorPage":
                method_names = [n.name for n in node.body if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef))]
                if "_get_controller" in method_names:
                    print(f"  {G}✓ _get_controller IS inside PrintMonitorPage class{X}")
                else:
                    print(f"  {R}✗ _get_controller is NOT inside PrintMonitorPage class (defined elsewhere){X}")
                    needs_fix = True
                if "on_status_update" in method_names:
                    print(f"  {G}✓ on_status_update IS inside PrintMonitorPage class{X}")
                else:
                    print(f"  {R}✗ on_status_update is NOT inside PrintMonitorPage class{X}")
                    needs_fix = True
                break

    if needs_fix:
        print(f"\n  {B}Applying fix...{X}")
        _apply_fix(path, content)
    else:
        print(f"\n  {G}Methods look correctly placed. Try:{X}")
        print(f"    1. Delete __pycache__: rm -rf gui/pages/__pycache__")
        print(f"    2. Relaunch: python main.py")

def _apply_fix(path, content):
    """Remove misplaced methods and re-insert at correct location."""
    import shutil
    from datetime import datetime

    # Remove any existing _get_controller and on_status_update definitions
    # that are outside the class or at wrong indent
    lines = content.split("\n")
    new_lines = []
    skip_until_next_def = False
    skip_indent = None

    i = 0
    while i < len(lines):
        line = lines[i]
        # Check if this is a misplaced _get_controller or on_status_update
        if re.match(r'^(def _get_controller|def on_status_update)', line):
            # At module level (indent=0) — remove it and its body
            skip_indent = 0
            skip_until_next_def = True
            print(f"    Removing module-level '{line.strip()}' at line {i+1}")
            i += 1
            continue

        if skip_until_next_def:
            stripped = line.lstrip()
            current_indent = len(line) - len(stripped) if stripped else 999
            if current_indent <= skip_indent and stripped and not stripped.startswith('#'):
                skip_until_next_def = False
                # Don't skip this line — it's the next definition
            else:
                i += 1
                continue

        new_lines.append(line)
        i += 1

    content = "\n".join(new_lines)

    # Now find a good insertion point inside the class
    # Insert before get_page_title
    helper_and_status = '''    def _get_controller(self):
        """v7.2.6: Helper to get controller reference."""
        return getattr(self, '_controller', None) or getattr(self, 'controller', None)

    def on_status_update(self):
        """v7.2.6: Live position polling during print execution."""
        from SupportClasses.PrintManager import PrintState

        if self._print_state != PrintState.RUNNING:
            return

        controller = self._get_controller()
        if controller is None:
            return

        try:
            xy_pos = None
            zp_pos = None
            if getattr(controller, 'is_xy_connected', False):
                xy_pos = controller.get_xy_position(cached=True)
            if getattr(controller, 'is_zp_connected', False):
                zp_pos = controller.get_zp_position(cached=True)
        except Exception:
            return

        if xy_pos and hasattr(self, 'trajectory_view'):
            try:
                x, y = xy_pos[0], xy_pos[1]
                if x is not None and y is not None:
                    self.trajectory_view.set_current_position(x, y)
            except Exception:
                pass

'''

    # Check if on_status_update already exists correctly inside the class
    if "def on_status_update" in content:
        # Verify via AST
        tree = ast.parse(content)
        for node in ast.walk(tree):
            if isinstance(node, ast.ClassDef) and node.name == "PrintMonitorPage":
                names = [n.name for n in node.body if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef))]
                if "on_status_update" in names and "_get_controller" in names:
                    print(f"    {Y}Both methods already inside class after cleanup{X}")
                    # Still write the cleaned content
                    break

    # If _get_controller is missing from class, insert
    tree = ast.parse(content)
    needs_gc = True
    needs_osu = True
    for node in ast.walk(tree):
        if isinstance(node, ast.ClassDef) and node.name == "PrintMonitorPage":
            names = [n.name for n in node.body if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef))]
            if "_get_controller" in names:
                needs_gc = False
            if "on_status_update" in names:
                needs_osu = False
            break

    if needs_gc or needs_osu:
        # Build what we need to insert
        to_insert = ""
        if needs_gc:
            to_insert += '''    def _get_controller(self):
        """v7.2.6: Helper to get controller reference."""
        return getattr(self, '_controller', None) or getattr(self, 'controller', None)

'''
        if needs_osu:
            to_insert += '''    def on_status_update(self):
        """v7.2.6: Live position polling during print execution."""
        from SupportClasses.PrintManager import PrintState

        if self._print_state != PrintState.RUNNING:
            return

        controller = self._get_controller()
        if controller is None:
            return

        try:
            xy_pos = None
            zp_pos = None
            if getattr(controller, 'is_xy_connected', False):
                xy_pos = controller.get_xy_position(cached=True)
            if getattr(controller, 'is_zp_connected', False):
                zp_pos = controller.get_zp_position(cached=True)
        except Exception:
            return

        if xy_pos and hasattr(self, 'trajectory_view'):
            try:
                x, y = xy_pos[0], xy_pos[1]
                if x is not None and y is not None:
                    self.trajectory_view.set_current_position(x, y)
            except Exception:
                pass

'''

        # Insert before get_page_title
        m = re.search(r'^    def get_page_title\(self\)', content, re.MULTILINE)
        if m:
            content = content[:m.start()] + to_insert + content[m.start():]
            print(f"    {G}Inserted methods before get_page_title(){X}")
        else:
            print(f"    {R}Could not find get_page_title insertion point{X}")
            return

    try:
        ast.parse(content)
    except SyntaxError as e:
        print(f"    {R}AST FAIL: {e}{X}")
        return

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    shutil.copy2(path, path.with_suffix(f".bak_diag_{ts}"))
    path.write_text(content, encoding="utf-8")
    print(f"    {G}✓ Fixed print_monitor.py{X}")
    print(f"\n    Now run: rm -rf gui/pages/__pycache__ && python main.py")


if __name__ == "__main__":
    main()

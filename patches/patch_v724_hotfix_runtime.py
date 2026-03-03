#!/usr/bin/env python3
"""
patch_v724_hotfix_runtime.py -- Fix two runtime bugs from smoke testing.

Bug 1: print_objects.py references _file_manager._current_file
        but PrintFileManager uses .current_path and .current
Bug 2: print_well_setup.py calls .get() on get_zp_position() result
        but get_zp_position() returns tuple (z, p1, p2, p3), not dict

Usage:
    python patch_v724_hotfix_runtime.py /path/to/MEBP
"""

import re, sys, os

GREEN = "\033[92m"
RED = "\033[91m"
YELLOW = "\033[93m"
RESET = "\033[0m"

_applied = 0
_skipped = 0
_failed = 0


def read_file(path):
    with open(path, "r") as f:
        return f.read()

def write_file(path, content):
    with open(path, "w") as f:
        f.write(content)

def patch_replace(content, old, new, desc):
    global _applied, _skipped, _failed
    if old not in content:
        if new in content:
            print(f"  SKIP: {desc} -- already applied")
            _skipped += 1
        else:
            print(f"  {RED}MISS{RESET}: {desc} -- old text not found")
            _failed += 1
        return content
    content = content.replace(old, new, 1)
    print(f"  {GREEN}OK{RESET}: {desc}")
    _applied += 1
    return content


def main():
    global _applied, _skipped, _failed

    root = sys.argv[1] if len(sys.argv) > 1 else os.getcwd()
    if not os.path.isfile(os.path.join(root, "gui", "app.py")):
        print(f"{RED}ERROR{RESET}: Not a valid MEBP project: {root}")
        sys.exit(1)

    print("=" * 60)
    print("MEBP v7.2.4 Runtime Hotfix Patch")
    print("=" * 60)

    # ── Bug 1: print_objects.py -- _current_file -> current_path ──
    po_path = os.path.join(root, "gui", "pages", "print_objects.py")
    if os.path.isfile(po_path):
        print(f"\nPatching: {po_path}")
        content = read_file(po_path)

        # Primary fix: the assignment line
        content = patch_replace(
            content,
            "self._current_file = self._file_manager._current_file",
            "self._current_file = str(self._file_manager.current_path) if self._file_manager.current_path else None",
            "Bug 1a: _file_manager._current_file -> current_path (assignment)",
        )

        # Catch any remaining _file_manager._current_file references
        remaining = content.count("self._file_manager._current_file")
        if remaining > 0:
            content = content.replace(
                "self._file_manager._current_file",
                "self._file_manager.current_path",
            )
            print(f"  {GREEN}OK{RESET}: Bug 1b: Fixed {remaining} more _current_file references")
            _applied += 1

        write_file(po_path, content)
        print(f"  {GREEN}DONE{RESET}: print_objects.py patched")
    else:
        print(f"  {YELLOW}WARN{RESET}: print_objects.py not found")

    # ── Bug 2: print_well_setup.py -- tuple.get() -> indexing ──
    pws_path = os.path.join(root, "gui", "pages", "print_well_setup.py")
    if os.path.isfile(pws_path):
        print(f"\nPatching: {pws_path}")
        content = read_file(pws_path)

        # get_zp_position() returns (z, p1, p2, p3) as tuple
        # Code likely does: pos = controller.get_zp_position(); z = pos.get('Z')
        # or: z = controller.get_zp_position().get('Z')

        # Strategy: find all .get('Z') / .get('z') near zp_position context
        # and replace with tuple indexing

        lines = content.split("\n")
        new_lines = []
        fixes = 0
        for i, line in enumerate(lines):
            original = line
            # Pattern: somevar.get('Z'...) or somevar.get('z'...)
            if ".get(" in line and ("zp_pos" in line or "z_pos" in line or "zp_position" in line):
                # Replace .get('Z', default) or .get("Z", default) with [0]
                line = re.sub(
                    r'(\w+)\.get\(\s*["\'](?:Z|z)["\']\s*(?:,\s*[^)]+)?\)',
                    r'(\1[0] if \1 and \1[0] is not None else 0.0)',
                    line,
                )
                for idx, keys in [(1, ['P1','p1']), (2, ['P2','p2']), (3, ['P3','p3'])]:
                    for k in keys:
                        line = re.sub(
                            r'(\w+)\.get\(\s*["\']' + k + r'["\']\s*(?:,\s*[^)]+)?\)',
                            r'(\1[' + str(idx) + r'] if \1 and len(\1) > ' + str(idx) + r' else 0.0)',
                            line,
                        )
            # Also catch: controller.get_zp_position().get('Z')
            if "get_zp_position()" in line and ".get(" in line:
                line = re.sub(
                    r'(self\.\w+\.get_zp_position\([^)]*\))\.get\(\s*["\'](?:Z|z)["\']\s*(?:,\s*[^)]+)?\)',
                    r'(lambda _p: _p[0] if _p and _p[0] is not None else 0.0)(\1)',
                    line,
                )
            if line != original:
                fixes += 1
            new_lines.append(line)

        if fixes > 0:
            content = "\n".join(new_lines)
            print(f"  {GREEN}OK{RESET}: Bug 2: Fixed {fixes} tuple .get() calls -> index access")
            _applied += 1
        else:
            # Try a broader search - maybe the variable name is different
            # Search for the warning message to find the method
            match = re.search(r'def\s+(\w+).*?Could not read Z position', content, re.DOTALL)
            if match:
                method_name = match.group(1)
                print(f"  {YELLOW}INFO{RESET}: Found Z read in method: {method_name}")
                # Extract the method and look for .get( on any variable
                method_match = re.search(
                    r'(def\s+' + method_name + r'\(.*?\n(?:(?:    |\t).*\n)*)',
                    content,
                )
                if method_match:
                    method_body = method_match.group(0)
                    # Find any var.get('Z') or var.get('z')
                    get_matches = re.findall(r'(\w+)\.get\(\s*["\']', method_body)
                    if get_matches:
                        print(f"  {YELLOW}INFO{RESET}: Found .get() calls on: {set(get_matches)}")
                        # Replace in the method body
                        fixed_body = method_body
                        for varname in set(get_matches):
                            fixed_body = re.sub(
                                varname + r'\.get\(\s*["\'](?:Z|z)["\']\s*(?:,\s*[^)]+)?\)',
                                f'({varname}[0] if {varname} and {varname}[0] is not None else 0.0)',
                                fixed_body,
                            )
                        if fixed_body != method_body:
                            content = content.replace(method_body, fixed_body)
                            print(f"  {GREEN}OK{RESET}: Bug 2: Fixed .get() in {method_name}()")
                            _applied += 1
                        else:
                            print(f"  {RED}MISS{RESET}: Bug 2: Could not auto-fix .get() calls")
                            _failed += 1
                    else:
                        print(f"  {RED}MISS{RESET}: Bug 2: No .get() found in {method_name}()")
                        _failed += 1
            else:
                print(f"  {RED}MISS{RESET}: Bug 2: Could not locate Z position read code")
                _failed += 1

        write_file(pws_path, content)
        print(f"  {GREEN}DONE{RESET}: print_well_setup.py patched")
    else:
        print(f"  {YELLOW}WARN{RESET}: print_well_setup.py not found")

    # ── Summary ──
    print("\n" + "=" * 60)
    print("Hotfix Summary")
    print("=" * 60)
    print(f"  Applied:  {_applied}")
    print(f"  Skipped:  {_skipped} (already applied)")
    print(f"  Failed:   {_failed}")

    if _failed > 0:
        print(f"\n{YELLOW}MANUAL FIX INSTRUCTIONS{RESET}:\n")
        print("  Bug 1 (print_objects.py):")
        print("    Find:    self._file_manager._current_file")
        print("    Replace: self._file_manager.current_path\n")
        print("  Bug 2 (print_well_setup.py):")
        print("    get_zp_position() returns TUPLE: (z, p1, p2, p3)")
        print("    Find any .get('Z') on the result and use [0] instead")
        print("    Find any .get('P1') -> [1], .get('P2') -> [2], .get('P3') -> [3]")


if __name__ == "__main__":
    main()
